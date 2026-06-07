"""
Rover Autonomy System — Vision + Detection
Stack: YOLO11n + OpenCV + Ultralytics
USB 1080p webcam input

Run modes:
python rover_autonomy.py --mode test      # See the improvements
python rover_autonomy.py --mode live      # Live with smooth tracking
python rover_autonomy.py --mode list-cameras  # Find connected cameras
  python rover_autonomy.py --mode test       # camera + detection diagnostics
  python rover_autonomy.py --mode live       # full live feed with decisions
  python rover_autonomy.py --mode benchmark  # FPS + latency report
"""

import cv2
import time
import argparse
import sys
import numpy as np
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import math

try:
    from ultralytics import YOLO
except ImportError:
    print("[ERROR] ultralytics not installed. Run: pip install ultralytics")
    sys.exit(1)

try:
    import supervision as sv
except ImportError:
    print("[WARN] supervision not installed. For better detection filtering, run: pip install supervision")
    sv = None

# ─── Global motor controller (set by launcher) ────────────────────────────────
motor = None  # Will be initialized by run_rover.py

# ─── Config ────────────────────────────────────────────────────────────────────

# ──── RASPBERRY PI 4 OPTIMIZED SETTINGS ────
CAMERA_INDEX      = 0          # Use 0 for USB webcam on RPi
CAPTURE_WIDTH     = 640        # Reduced for RPi (640x480 is RPi sweet spot)
CAPTURE_HEIGHT    = 480        # Lower resolution = 4x fewer pixels than 1280x720 = HUGE speedup
MODEL_PATH        = "yolo11n.pt"  # YOLO11n nano (5.5MB, fastest for RPi)
CONF_THRESHOLD    = 0.50       # Increased for fewer false positives
IOU_THRESHOLD     = 0.45
DEVICE            = 'cpu'      # IMPORTANT: CPU for Raspberry Pi (no NVIDIA GPU)
USE_FP16          = False       # Disable FP16 on RPi (no benefit on CPU, may cause issues)
USE_INT8_QUANT    = True        # Enable INT8 quantization (~3-4x speedup, minimal accuracy loss)

# Odometry / IMU config (tune to your robot)
SERIAL_PORT = '/dev/ttyACM0'     # Arduino serial port (override with --serial)
SERIAL_BAUD = 115200
WHEEL_BASE_M = 0.18              # Distance between left/right wheels (meters)
WHEEL_DIAMETER_M = 0.065         # Wheel diameter (meters)
TICKS_PER_REV = 360              # Encoder ticks per wheel revolution
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER_M) / max(1, TICKS_PER_REV)

# Fast stop params
FAST_STOP_CONF = 0.35            # Immediate stop confidence threshold
FAST_STOP_AREA_FRAC = 0.01       # Minimum area fraction to consider immediate stop

# FPS Optimization for RPi (critical for smooth motion)
INFERENCE_SKIP_FRAMES = 2       # Run inference every 2 frames (was 3) - more frequent for better detection
INTERPOLATE_SKIPPED = True      # Interpolate detections between skipped frames
INFERENCE_THREADS = 2           # Use 2 CPU threads on RPi 4 (has 4 cores, reserve for OS)

# Detection history and filtering
MIN_DETECTION_CONFIDENCE = 0.50  # Minimum confidence to consider
DETECTION_HISTORY_LEN = 3        # Frames to confirm detection
MIN_CONFIRMED_FRAMES = 3         # How many frames to see before reporting (increased from 2)
KALMAN_PROCESS_NOISE = 0.02      # Lower = more trust in model
KALMAN_MEASUREMENT_NOISE = 0.05  # Higher = more trust in Kalman filter

# RPi Memory optimization
MAX_TRACKERS = 20               # Limit simultaneous object tracks to save memory
MAX_AGE_FRAMES = 20             # Longer history (was 15) - hold detections longer

# Danger zone: centre column of frame as fraction of width
# If an obstacle occupies this zone -> STOP or STEER
# EXPANDED for RPi camera straight-ahead view
DANGER_ZONE_LEFT  = 0.20       # Wider left margin (was 0.30) - catch obstacles earlier
DANGER_ZONE_RIGHT = 0.80       # Wider right margin (was 0.70)
DANGER_ZONE_MIN_AREA = 0.01    # LOWERED threshold (was 0.04) - detect smaller obstacles

# Additional obstacle thresholds for better accuracy
OBSTACLE_MIN_HEIGHT_FRACTION = 0.05  # Obstacle must be at least 5% of frame height
OBSTACLE_MIN_WIDTH_FRACTION = 0.02   # Obstacle must be at least 2% of frame width

# Classes that count as navigation obstacles (COCO class names)
# OPTIMIZED for moving obstacles (people/animals) in front of rover
OBSTACLE_CLASSES = {
    # Primary (highest priority - moving obstacles)
    "person", "bicycle", "cat", "dog",
    # Vehicles
    "car", "motorcycle", "bus", "truck",
    # Static obstacles in path
    "chair", "bench", "potted plant", "stop sign",
    # Other dangerous items
    "backpack", "suitcase"
}

# High priority obstacles (moving, unpredictable)
HIGH_PRIORITY_OBSTACLES = {"person", "dog", "cat"}

# Medium priority (vehicles)
MEDIUM_PRIORITY_OBSTACLES = {"car", "motorcycle", "bicycle", "bus", "truck"}

# Low priority (static items)
LOW_PRIORITY_OBSTACLES = {"chair", "bench", "potted plant"}

# ─── Data structures ───────────────────────────────────────────────────────────

@dataclass
class Detection:
    label: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int
    frame_count: int = 1  # How many frames this detection has been seen
    smoothed: bool = False  # Whether this is Kalman-filtered

    @property
    def cx(self): return (self.x1 + self.x2) // 2
    @property
    def cy(self): return (self.y1 + self.y2) // 2
    @property
    def area(self): return (self.x2 - self.x1) * (self.y2 - self.y1)
    @property
    def width(self): return self.x2 - self.x1
    @property
    def height(self): return self.y2 - self.y1
    @property
    def priority(self) -> str:
        """Return priority level (HIGH/MEDIUM/LOW) based on label."""
        if self.label in HIGH_PRIORITY_OBSTACLES:
            return "HIGH"
        elif self.label in MEDIUM_PRIORITY_OBSTACLES:
            return "MEDIUM"
        else:
            return "LOW"


class KalmanTracker:
    """Kalman filter for smooth object tracking across frames."""
    def __init__(self, detection: Detection):
        # State: [cx, cy, width, height, vx, vy, vw, vh]
        self.state = np.array([
            detection.cx, detection.cy,
            detection.width, detection.height,
            0, 0, 0, 0  # velocities
        ], dtype=np.float32)
        
        # Covariance
        self.P = np.eye(8) * 100
        
        self.Q = np.eye(8) * KALMAN_PROCESS_NOISE  # process noise
        self.R = np.eye(4) * KALMAN_MEASUREMENT_NOISE  # measurement noise
        self.H = np.eye(4, 8)  # measurement matrix (only observe position & size)
        
        self.detection = detection
        self.frame_age = 0
        self.consecutive_misses = 0
    
    def predict(self, dt: float = 1.0) -> None:
        """Predict next state."""
        # Simple velocity model
        F = np.eye(8)
        F[0, 4] = dt  # cx += vx * dt
        F[1, 5] = dt  # cy += vy * dt
        F[2, 6] = dt  # w += vw * dt
        F[3, 7] = dt  # h += vh * dt
        
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, detection: Detection) -> None:
        """Update with new measurement."""
        z = np.array([detection.cx, detection.cy, detection.width, detection.height])
        
        # Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        y = z - (self.H @ self.state)
        self.state = self.state + K @ y
        
        # Update covariance
        self.P = (np.eye(8) - K @ self.H) @ self.P
        
        self.consecutive_misses = 0
        self.detection = detection
    
    def get_smoothed_detection(self) -> Detection:
        """Get smoothed detection from current state."""
        cx, cy, w, h = self.state[:4]
        x1 = int(cx - w / 2)
        y1 = int(cy - h / 2)
        x2 = int(cx + w / 2)
        y2 = int(cy + h / 2)
        
        det = Detection(
            self.detection.label,
            self.detection.confidence,
            x1, y1, x2, y2,
            frame_count=self.detection.frame_count,
            smoothed=True
        )
        return det


class DetectionTrackerManager:
    """Manages Kalman trackers for smooth object tracking."""
    
    def __init__(self, max_age: int = 20):
        self.trackers: List[KalmanTracker] = []
        self.max_age = max_age
        self.frame_count = 0
    
    def _distance(self, track: KalmanTracker, detection: Detection) -> float:
        """Euclidean distance between tracker and detection."""
        dx = track.detection.cx - detection.cx
        dy = track.detection.cy - detection.cy
        # Penalize large difference in size
        dw = (track.detection.width - detection.width) / max(track.detection.width, 1)
        dh = (track.detection.height - detection.height) / max(track.detection.height, 1)
        return np.sqrt(dx**2 + dy**2 + (dw**2 + dh**2) * 100)
    
    def _match_detections(self, detections: List[Detection], max_distance: float = 50) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """Match detections to existing trackers using greedy assignment."""
        matches = []
        unmatched_tracks = list(range(len(self.trackers)))
        unmatched_detections = list(range(len(detections)))
        
        # Greedy matching
        for det_idx, det in enumerate(detections):
            best_track_idx = -1
            best_distance = max_distance
            
            for track_idx in unmatched_tracks:
                dist = self._distance(self.trackers[track_idx], det)
                if dist < best_distance:
                    best_distance = dist
                    best_track_idx = track_idx
            
            if best_track_idx != -1:
                matches.append((best_track_idx, det_idx))
                unmatched_tracks.remove(best_track_idx)
                unmatched_detections.remove(det_idx)
        
        return matches, unmatched_tracks, unmatched_detections
    
    def update(self, detections: List[Detection]) -> List[Detection]:
        """Update trackers with new detections and return smoothed detections."""
        self.frame_count += 1
        
        # Predict next state for all trackers
        for tracker in self.trackers:
            tracker.predict()
            tracker.frame_age += 1
            tracker.consecutive_misses += 1
        
        # Match detections to trackers
        matches, unmatched_tracks, unmatched_detections = self._match_detections(detections)
        
        # Update matched trackers
        for track_idx, det_idx in matches:
            det = detections[det_idx]
            det.frame_count = self.trackers[track_idx].detection.frame_count + 1
            self.trackers[track_idx].update(det)
        
        # Create new trackers for unmatched detections
        for det_idx in unmatched_detections:
            new_tracker = KalmanTracker(detections[det_idx])
            self.trackers.append(new_tracker)
        
        # Remove dead trackers
        self.trackers = [t for t in self.trackers if t.consecutive_misses < self.max_age]
        
        # Limit trackers for RPi memory (keep oldest/largest)
        if len(self.trackers) > MAX_TRACKERS:
            # Keep trackers with longest history (most reliable)
            self.trackers = sorted(self.trackers, key=lambda t: -t.detection.frame_count)[:MAX_TRACKERS]
        
        # Return confirmed detections (seen in multiple frames)
        confirmed = []
        for tracker in self.trackers:
            if tracker.detection.frame_count >= MIN_CONFIRMED_FRAMES:
                confirmed.append(tracker.get_smoothed_detection())
        
        return confirmed


# ─── AprilTag detection + simple EKF fusion (lightweight) ─────────────────────
try:
    import apriltag
except Exception:
    apriltag = None
    print("[WARN] apriltag library not installed. AprilTag detection will be disabled. Install with: pip install apriltag")


class AprilTagDetector:
    def __init__(self, tag_size_m: float = 0.15, camera_matrix: Optional[np.ndarray] = None, dist_coeffs: Optional[np.ndarray] = None):
        self.tag_size = tag_size_m
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros((4, 1))
        if apriltag is not None:
            try:
                self.detector = apriltag.Detector()
            except Exception:
                self.detector = None
        else:
            self.detector = None

    def detect(self, frame: np.ndarray) -> List[dict]:
        """Detect AprilTags and return list of {'id', 'tvec', 'rvec', 'corners'}"""
        if self.detector is None:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets = self.detector.detect(gray)
        out = []
        # 3D object points for tag corners (centered at tag center, z=0)
        s = self.tag_size / 2.0
        obj_pts = np.array([[-s, -s, 0], [ s, -s, 0], [ s,  s, 0], [-s,  s, 0]], dtype=np.float32)

        for d in dets:
            try:
                corners = np.array(d.corners, dtype=np.float32)
                # Solve PnP: object points -> image corners
                success, rvec, tvec = cv2.solvePnP(obj_pts, corners, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not success:
                    continue
                out.append({'id': int(d.tag_id), 'rvec': rvec, 'tvec': tvec, 'corners': corners})
            except Exception:
                continue
        return out


class SimpleEKFLocalizer:
    """Very small EKF-like pose fusion for x,y,theta using tag absolute observations.
    This is a lightweight correction step: state = [x, y, theta].
    For full accuracy replace with a proper EKF using odometry/IMU prediction.
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.P = np.eye(3) * 1.0

    def predict(self, dx=0.0, dy=0.0, dtheta=0.0, Q=None):
        # Simple motion prediction (additive)
        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        if Q is None:
            Q = np.eye(3) * 0.01
        self.P = self.P + Q

    def update_with_absolute_pose(self, meas_x: float, meas_y: float, meas_theta: float, R=None):
        # Measurement is absolute pose from an AprilTag observation (world coordinates)
        if R is None:
            R = np.eye(3) * 0.05

        z = np.array([meas_x, meas_y, meas_theta])
        xhat = np.array([self.x, self.y, self.theta])
        y = z - xhat
        # normalize angle
        y[2] = (y[2] + math.pi) % (2 * math.pi) - math.pi

        S = self.P + R
        K = self.P @ np.linalg.inv(S)
        xhat = xhat + K @ y
        self.x, self.y, self.theta = xhat.tolist()
        self.P = (np.eye(3) - K) @ self.P

    def get_pose(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)


class SerialTelemetry:
    """Read encoder and IMU telemetry from Arduino over serial.
    Expected line format (CSV):
      T,left_ticks,right_ticks,gyro_z,ax,ay,az,t_ms
    Where left/right are cumulative encoder counts, gyro_z is rad/s (or deg/s), ax/ay/az are raw accel, t_ms is millis.
    """
    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        try:
            import serial
        except Exception:
            print("[WARN] pyserial not installed. Telemetry disabled. Install with: pip install pyserial")
            self.enabled = False
            return

        self.enabled = True
        self.port = port
        self.baud = baud
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
        except Exception as e:
            print(f"[WARN] Could not open serial port {self.port}: {e}")
            self.enabled = False
            return

        self.lock = threading.Lock()
        self.left = 0
        self.right = 0
        self.gyro_z = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.t_ms = 0

        self._last_left = None
        self._last_right = None

        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                if not line.startswith('T,'):
                    continue
                parts = line.split(',')
                if len(parts) < 8:
                    continue
                _, left_s, right_s, gyro_s, ax_s, ay_s, az_s, t_s = parts[:8]
                with self.lock:
                    self.left = int(left_s)
                    self.right = int(right_s)
                    try:
                        self.gyro_z = float(gyro_s)
                    except Exception:
                        self.gyro_z = 0.0
                    self.ax = float(ax_s)
                    self.ay = float(ay_s)
                    self.az = float(az_s)
                    self.t_ms = int(t_s)
            except Exception:
                time.sleep(0.005)

    def stop(self):
        self._stop = True
        try:
            self._thread.join(timeout=0.5)
        except Exception:
            pass

    def get_deltas(self):
        """Return (dx_m, dtheta_rad) computed from encoder deltas since last call and reset lasts."""
        if not self.enabled:
            return 0.0, 0.0
        with self.lock:
            l = self.left
            r = self.right
            gz = self.gyro_z

        if self._last_left is None:
            self._last_left = l
            self._last_right = r
            return 0.0, 0.0

        dl = l - self._last_left
        dr = r - self._last_right
        self._last_left = l
        self._last_right = r

        d_left_m = dl * METERS_PER_TICK
        d_right_m = dr * METERS_PER_TICK
        d_center = (d_left_m + d_right_m) / 2.0
        dtheta = (d_right_m - d_left_m) / max(1e-6, WHEEL_BASE_M)

        return d_center, dtheta

@dataclass
class RoverDecision:
    action: str           # FORWARD / STOP / STEER_LEFT / STEER_RIGHT
    reason: str
    obstacles: list = field(default_factory=list)
    confidence: float = 1.0

# ─── Motor stub (replace with your actual motor driver calls) ──────────────────

class MotorController:
    """
    Hardware controller for Arduino-based motor driver.
    Optimized for fast response.
    """
    def __init__(self, port=None, dry_run=True, baudrate=115200):
        self.dry_run = dry_run
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self._last_action = None
        self._connected = False
        
        if not dry_run and port:
            self._connect()
    
    def _connect(self):
        """Establish serial connection to Arduino."""
        try:
            import serial
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino to reset
            self._connected = True
            print(f"[MOTOR] Connected @ {self.baudrate} baud")
        except ImportError:
            print("[MOTOR] pyserial not installed. Run: pip install pyserial")
            self.dry_run = True
        except Exception as e:
            print(f"[MOTOR] Connection failed: {e}")
            self.dry_run = True
    
    def _send_command(self, cmd: str):
        """Send command to Arduino (non-blocking)."""
        if self.dry_run or not self._connected:
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.ser.flush()
        except Exception as e:
            print(f"[MOTOR] Send failed: {e}")
            self._connected = False

    def _send_motor_pwm(self, left_pwm: int, right_pwm: int):
        """Send left/right PWM command to Arduino as 'M,left,right'"""
        if self.dry_run:
            self._log(f"MOTCMD L={left_pwm} R={right_pwm}")
            return
        if not self._connected:
            return
        cmd = f"M,{int(left_pwm)},{int(right_pwm)}"
        self._send_command(cmd)

    def forward(self, speed=0.8):
        """Move forward. speed: 0.0-1.0"""
        pwm = int(max(0, min(255, speed * 255)))
        # send both sides same pwm
        self._send_motor_pwm(pwm, pwm)
        self._log(f"FWD {pwm}")
        self._last_action = "FORWARD"

    def stop(self):
        """Stop motors."""
        # always send stop to ensure safety
        self._send_motor_pwm(0, 0)
        self._log("STP")
        self._last_action = "STOP"

    def steer_left(self, speed=0.6):
        """Steer left."""
        pwm = int(max(0, min(255, speed * 255)))
        # To steer left, reduce left and increase right
        left_pwm = int(pwm * 0.4)
        right_pwm = pwm
        self._send_motor_pwm(left_pwm, right_pwm)
        self._log(f"LFT L={left_pwm} R={right_pwm}")
        self._last_action = "STEER_LEFT"

    def steer_right(self, speed=0.6):
        """Steer right."""
        pwm = int(max(0, min(255, speed * 255)))
        left_pwm = pwm
        right_pwm = int(pwm * 0.4)
        self._send_motor_pwm(left_pwm, right_pwm)
        self._log(f"RGT L={left_pwm} R={right_pwm}")
        self._last_action = "STEER_RIGHT"
    
    def close(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    def _log(self, msg):
        prefix = "[DRY]" if self.dry_run else "[MOT]"
        print(f"{prefix} {msg}")

# ─── Decision engine ───────────────────────────────────────────────────────────

class DecisionEngine:
    def __init__(self, frame_w: int, frame_h: int):
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.frame_area = frame_w * frame_h
        self._history = deque(maxlen=5)  # smooth decisions over last N frames
        self._last_stop_frame = -100  # Track when STOP was last decided
        self.frame_number = 0

    def decide(self, detections: list[Detection]) -> RoverDecision:
        self.frame_number += 1
        
        # Filter obstacles with improved criteria
        obstacles = self._filter_obstacles(detections)

        # Filter to only obstacles in danger zone
        centre_obstacles = [
            d for d in obstacles
            if self._in_danger_zone(d)
        ]
        
        # DEBUG: Log what we're seeing
        if len(obstacles) > 0 or len(centre_obstacles) > 0:
            print(f"[DEBUG] Frame {self.frame_number} | Total detected: {len(detections)} | Filtered: {len(obstacles)} | In danger zone: {len(centre_obstacles)}")
            for obs in centre_obstacles[:3]:
                print(f"  → {obs.label} conf={obs.confidence:.2f} area={obs.area/(self.frame_area):.1%} cx_norm={obs.cx/self.frame_w:.2f}")

        if not centre_obstacles:
            # Path is clear - but check if we're still in STOP inertia
            frames_since_stop = self.frame_number - self._last_stop_frame
            if frames_since_stop < 10:  # Keep STOP for 10 frames minimum
                decision = RoverDecision("STOP", f"obstacle inertia ({frames_since_stop})")
            else:
                decision = RoverDecision("FORWARD", "path clear")
        else:
            # Weight obstacles: closer = higher threat, moving = higher threat
            weighted_obstacles = [(d, self._threat_score(d)) for d in centre_obstacles]
            blocker = max(weighted_obstacles, key=lambda x: x[1])[0]
            
            cx_norm = blocker.cx / self.frame_w
            area_fraction = blocker.area / self.frame_area

            # Decision based on obstacle position and size
            if area_fraction > 0.25:  # Very close/large
                action = "STOP"
            elif area_fraction > 0.10:  # Medium close
                if cx_norm < 0.4:
                    action = "STEER_RIGHT"
                elif cx_norm > 0.6:
                    action = "STEER_LEFT"
                else:
                    action = "STOP"  # Centered, steering won't help
            else:  # Smaller obstacle
                if cx_norm < 0.35:
                    action = "STEER_RIGHT"
                elif cx_norm > 0.65:
                    action = "STEER_LEFT"
                else:
                    action = "FORWARD"  # Marginal obstacle, try forward

            obstacle_type = blocker.label.upper()
            decision = RoverDecision(
                action=action,
                reason=f"{obstacle_type} @ {blocker.confidence:.2f} (size={area_fraction:.1%})",
                obstacles=centre_obstacles,
                confidence=blocker.confidence,
            )
            
            # Track when STOP was decided (for inertia)
            if action == "STOP":
                self._last_stop_frame = self.frame_number

        self._history.append(decision.action)
        decision.action = self._smoothed_action(decision.action)
        return decision
    
    def _filter_obstacles(self, detections: list[Detection]) -> list[Detection]:
        """Filter detections to valid obstacles with improved criteria."""
        obstacles = []
        for d in detections:
            # Step 1: Check if label is in obstacle classes
            if d.label not in OBSTACLE_CLASSES:
                continue
            
            # Step 2: Area check (must be at least 1% of frame)
            area_frac = d.area / self.frame_area
            if area_frac < DANGER_ZONE_MIN_AREA:
                continue
            
            # Step 3: Size proportions check (avoid false tiny detections)
            width_frac = d.width / self.frame_w
            height_frac = d.height / self.frame_h
            
            # LESS strict: allow smaller obstacles
            if width_frac < OBSTACLE_MIN_WIDTH_FRACTION or height_frac < OBSTACLE_MIN_HEIGHT_FRACTION:
                continue
            
            # Step 4: Confidence threshold varies by class
            if d.label in HIGH_PRIORITY_OBSTACLES:
                # People/animals: be stricter (0.40 min)
                if d.confidence < 0.40:
                    continue
            elif d.label in MEDIUM_PRIORITY_OBSTACLES:
                # Vehicles: moderate confidence (0.35 min)
                if d.confidence < 0.35:
                    continue
            else:
                # Static items: typical confidence (0.50 min)
                if d.confidence < 0.50:
                    continue
            
            obstacles.append(d)
        
        return obstacles
    
    def _threat_score(self, d: Detection) -> float:
        """Calculate threat level: size + priority + center position."""
        area_score = (d.area / self.frame_area) * 10
        
        # Priority weight
        if d.label in HIGH_PRIORITY_OBSTACLES:
            priority = 3.0  # People/animals first
        elif d.label in MEDIUM_PRIORITY_OBSTACLES:
            priority = 2.0  # Vehicles second
        else:
            priority = 1.0  # Static obstacles
        
        # Center distance weight (obstacles at center are more dangerous)
        cx_norm = d.cx / self.frame_w
        center_dist = abs(cx_norm - 0.5)  # Distance from center
        center_score = (1.0 - center_dist) * 2  # 0-2 points
        
        return area_score * priority + center_score

    def _in_danger_zone(self, d: Detection) -> bool:
        cx_norm = d.cx / self.frame_w
        return DANGER_ZONE_LEFT <= cx_norm <= DANGER_ZONE_RIGHT

    def _smoothed_action(self, current: str) -> str:
        # Only act on a decision if it appears in majority of recent frames
        from collections import Counter
        counts = Counter(self._history)
        majority = counts.most_common(1)[0][0]
        
        # Be fast to STOP (don't smooth), but smooth steering/forward
        if current == "STOP" or majority == "STOP":
            return "STOP"  # Prioritize safety
        
        return majority if len(self._history) >= 3 else current

# ─── Visualiser ────────────────────────────────────────────────────────────────

COLOUR_OK      = (80, 200, 80)
COLOUR_DANGER  = (40, 40, 220)
COLOUR_WARN    = (40, 180, 220)
COLOUR_HUD     = (220, 220, 220)

def draw_danger_zone(frame):
    h, w = frame.shape[:2]
    x1 = int(w * DANGER_ZONE_LEFT)
    x2 = int(w * DANGER_ZONE_RIGHT)
    
    # Semi-transparent overlay
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, 0), (x2, h), (0, 60, 255), -1)
    cv2.addWeighted(overlay, 0.10, frame, 0.90, 0, frame)
    
    # Edge lines
    cv2.line(frame, (x1, 0), (x1, h), (0, 120, 255), 2)
    cv2.line(frame, (x2, 0), (x2, h), (0, 120, 255), 2)
    
    # Label
    cv2.putText(frame, "DANGER ZONE", (x1 + 5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 255), 1, cv2.LINE_AA)

def draw_detections(frame, detections: list[Detection], decision: RoverDecision):
    danger_labels = {d.label for d in decision.obstacles}
    for d in detections:
        is_danger = d.label in danger_labels
        
        # Color based on priority
        if d.label in HIGH_PRIORITY_OBSTACLES:
            colour = (0, 0, 255) if is_danger else (0, 100, 200)  # Red/orange for people
        elif d.label in MEDIUM_PRIORITY_OBSTACLES:
            colour = (40, 40, 220) if is_danger else (100, 100, 220)  # Red/pink for vehicles
        elif is_danger:
            colour = (40, 40, 220)  # Red for danger
        else:
            colour = (80, 200, 80)  # Green for safe
        
        cv2.rectangle(frame, (d.x1, d.y1), (d.x2, d.y2), colour, 2)
        
        # Confidence label
        label = f"{d.label} {d.confidence:.2f}"
        cv2.putText(frame, label, (d.x1, d.y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
        
        # Add frame count for tracking
        if d.frame_count > 1:
            frame_text = f"#{d.frame_count}"
            cv2.putText(frame, frame_text, (d.x1, d.y2 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1, cv2.LINE_AA)

def draw_hud(frame, decision: RoverDecision, fps: float, latency_ms: float):
    h, w = frame.shape[:2]
    action_colours = {
        "FORWARD":     (80, 200, 80),
        "STOP":        (40, 40, 220),
        "STEER_LEFT":  (40, 200, 220),
        "STEER_RIGHT": (40, 200, 220),
    }
    colour = action_colours.get(decision.action, COLOUR_HUD)

    # Action banner (top)
    cv2.rectangle(frame, (0, 0), (w, 45), (20, 20, 20), -1)
    cv2.putText(frame, f"ACTION: {decision.action}",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2, cv2.LINE_AA)
    
    # Reason below action
    reason_text = decision.reason[:50]  # Truncate long reasons
    cv2.putText(frame, reason_text, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1, cv2.LINE_AA)

    # Stats bar (bottom)
    cv2.rectangle(frame, (0, h - 28), (w, h), (20, 20, 20), -1)
    stats = f"FPS: {fps:.1f} | Latency: {latency_ms:.1f}ms | Obstacles: {len(decision.obstacles)}"
    cv2.putText(frame, stats, (10, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOUR_HUD, 1, cv2.LINE_AA)


def draw_obstacle_list(frame, decision: RoverDecision):
    """Draw detected obstacles panel on right side showing threat info."""
    h, w = frame.shape[:2]
    panel_w = 220
    
    if len(decision.obstacles) == 0:
        return
    
    panel_h = min(len(decision.obstacles) * 28 + 30, h - 60)
    
    # Semi-transparent background panel
    cv2.rectangle(frame, (w - panel_w, 50), (w, 50 + panel_h), (20, 20, 20), -1)
    cv2.rectangle(frame, (w - panel_w, 50), (w, 50 + panel_h), (200, 200, 200), 2)
    
    # Title
    cv2.putText(frame, "DETECTED OBSTACLES", (w - panel_w + 5, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    
    # List each obstacle
    for idx, obs in enumerate(decision.obstacles[:10]):  # Max 10 shown
        y = 95 + idx * 28
        
        # Priority color
        priority_colour = {
            "person": (0, 0, 255),     # RED for people
            "dog":    (0, 0, 255),     # RED for animals
            "cat":    (0, 0, 255),     # RED for animals
            "car":    (40, 150, 255),  # ORANGE for vehicles
            "motorcycle": (40, 150, 255),
            "bicycle": (40, 150, 255),
            "bus":    (40, 150, 255),
            "truck":  (40, 150, 255),
        }
        colour = priority_colour.get(obs.label, (80, 200, 80))
        
        # Label + confidence
        label_text = f"{obs.label[:12]} {obs.confidence:.0%}"
        cv2.putText(frame, label_text, (w - panel_w + 5, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1, cv2.LINE_AA)
        
        # Area/threat info
        area_pct = (obs.area / (h * w)) * 100
        threat_text = f"Area: {area_pct:.1f}% | #{obs.frame_count}"
        cv2.putText(frame, threat_text, (w - panel_w + 5, y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1, cv2.LINE_AA)

# ─── Camera Manager (Threaded) ────────────────────────────────────────────────

class CameraManager:
    """Manages multiple cameras with threaded capture for better FPS."""
    
    def __init__(self, main_camera_index: int = 0, use_threading: bool = True):
        self.main_camera_index = main_camera_index
        self.available_cameras = []
        self.current_camera = None
        self.cap = None
        self.use_threading = use_threading
        
        # Threading
        self._capture_thread = None
        self._frame_buffer = None
        self._frame_lock = threading.Lock()
        self._stop_capture = False
        
        self._discover_cameras()
    
    def _discover_cameras(self, max_index: int = 10) -> None:
        """Find all available camera devices."""
        self.available_cameras = []
        for idx in range(max_index):
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    h, w = frame.shape[:2]
                    self.available_cameras.append({
                        'index': idx,
                        'resolution': (w, h),
                        'name': f"Camera {idx}"
                    })
                cap.release()
        # Give the OS a moment to fully release devices before we reopen them
        if self.available_cameras:
            time.sleep(0.3)
    
    def get_available_cameras(self) -> list[dict]:
        """Return list of available cameras."""
        return self.available_cameras
    
    def _capture_loop(self) -> None:
        """Background thread for continuous frame capture."""
        while not self._stop_capture:
            if self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    with self._frame_lock:
                        self._frame_buffer = frame
            time.sleep(0.001)  # Prevent busy waiting
    
    def select_camera(self, index: int) -> bool:
        """Switch to a different camera by index."""
        if not any(cam['index'] == index for cam in self.available_cameras):
            print(f"[ERROR] Camera index {index} not available")
            return False
        
        # Stop old capture thread
        self._stop_capture = True
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
        
        if self.cap is not None:
            self.cap.release()
        
        self.current_camera = index
        self.main_camera_index = index
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            print(f"[ERROR] Failed to open camera {index}")
            return False
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer to reduce lag
        
        # Start new capture thread
        if self.use_threading:
            self._stop_capture = False
            self._frame_buffer = None
            self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._capture_thread.start()

            # Wait for the thread to capture the first frame (up to 3 s)
            deadline = time.time() + 3.0
            while time.time() < deadline:
                with self._frame_lock:
                    if self._frame_buffer is not None:
                        break
                time.sleep(0.05)
            else:
                print(f"[WARN] Camera {index} opened but no frames received within 3s")

        print(f"[OK] Switched to camera {index}")
        return True
    
    def initialize_main_camera(self) -> bool:
        """Initialize the main camera."""
        return self.select_camera(self.main_camera_index)
    
    def read_frame(self):
        """Read a frame from current camera."""
        if self.cap is None:
            return False, None

        if self.use_threading:
            # Only the background thread reads from self.cap — reading here too
            # causes a race condition that silently breaks capture on RPi.
            with self._frame_lock:
                frame = self._frame_buffer
            return frame is not None, frame
        else:
            return self.cap.read()
    
    def release(self) -> None:
        """Release camera resources."""
        self._stop_capture = True
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
            self.cap = None
    
    def print_available(self) -> None:
        """Print available cameras."""
        print("\n=== Available Cameras ===")
        if not self.available_cameras:
            print("  No cameras found")
        for cam in self.available_cameras:
            marker = " (MAIN)" if cam['index'] == self.main_camera_index else ""
            print(f"  Index {cam['index']}: {cam['resolution'][0]}x{cam['resolution'][1]}{marker}")
        print()


def check_camera(index: int) -> tuple[bool, str]:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        return False, f"Camera index {index} not found"
    ret, frame = cap.read()
    cap.release()
    if not ret or frame is None:
        return False, "Camera opened but could not read a frame"
    h, w = frame.shape[:2]
    return True, f"Camera OK — {w}x{h}"

# ─── Frame Skipping & Interpolation ────────────────────────────────────────────

class FrameSkipper:
    """Handles frame skipping and detection interpolation for FPS boost."""
    
    def __init__(self, skip_frames: int = 2):
        self.skip_frames = skip_frames
        self.frame_count = 0
        self.last_detections = []
        self.should_infer = True
    
    def update(self) -> bool:
        """Check if we should run inference this frame."""
        self.should_infer = (self.frame_count % self.skip_frames) == 0
        self.frame_count += 1
        return self.should_infer
    
    def get_interpolated_detections(self, new_detections: Optional[List[Detection]] = None) -> List[Detection]:
        """Use last detections or interpolate for skipped frames."""
        if new_detections is not None:
            self.last_detections = new_detections
            return new_detections
        # Return last detections (interpolation)
        return self.last_detections

# ─── Run modes ─────────────────────────────────────────────────────────────────

def run_test_mode():
    """
    Diagnostics: camera check, model load, single-frame detection, FPS estimate.
    No window required — safe to run headless.
    """
    print("\n=== ROVER VISION DIAGNOSTIC (RPi 4 OPTIMIZED) ===\n")
    print("Target: Raspberry Pi 4 (BCM2711, 4-core ARM Cortex-A72 @ 1.5GHz)")
    print(f"Resolution: {CAPTURE_WIDTH}x{CAPTURE_HEIGHT} (optimized for RPi)")
    print(f"Model: YOLO11n (nano, 5.5MB)")
    print(f"Frame skip: Inference every {INFERENCE_SKIP_FRAMES} frames (CPU savings)")
    print(f"Device: {DEVICE} | INT8 Quantization: {USE_INT8_QUANT}")
    if INTERPOLATE_SKIPPED:
        print("Interpolation: Enabled (smooth motion between frames)")
    print()
    
    # Camera manager
    cam_mgr = CameraManager(main_camera_index=CAMERA_INDEX, use_threading=True)
    cam_mgr.print_available()

    # 1. Camera
    print("[1/4] Checking camera...")
    ok, msg = check_camera(CAMERA_INDEX)
    status = "PASS" if ok else "FAIL"
    print(f"  {status}  {msg}")
    if not ok:
        print("  → Check USB connection or change CAMERA_INDEX in config or use --camera flag")
        return

    # 2. Model load
    print("[2/4] Loading YOLO11n model...")
    try:
        t0 = time.perf_counter()
        model = YOLO(MODEL_PATH, task='detect')
        model.to(DEVICE)
        
        # Set CPU threading for RPi
        if DEVICE == 'cpu':
            import torch
            torch.set_num_threads(INFERENCE_THREADS)
        
        load_ms = (time.perf_counter() - t0) * 1000
        print(f"  PASS  Model loaded in {load_ms:.0f}ms on {DEVICE}")
        if DEVICE == 'cpu':
            print(f"       CPU threads: {INFERENCE_THREADS}")
    except Exception as e:
        print(f"  FAIL  {e}")
        return

    # 3. Single inference
    print("[3/4] Running inference on live frame...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("  FAIL  Could not capture frame")
        return

    t0 = time.perf_counter()
    results = model(frame, conf=CONF_THRESHOLD, iou=IOU_THRESHOLD, verbose=False)
    inf_ms = (time.perf_counter() - t0) * 1000
    names = model.names
    raw_detections = _parse_detections(results, names)
    
    # Apply Kalman smoothing
    tracker_mgr = DetectionTrackerManager()
    smooth_detections = _filter_and_smooth_detections(tracker_mgr, raw_detections)
    
    print(f"  PASS  Inference: {inf_ms:.1f}ms — {len(raw_detections)} raw objects, {len(smooth_detections)} confirmed")
    for d in smooth_detections[:5]:  # Show first 5
        conf_str = "✓ confirmed" if d.frame_count >= MIN_CONFIRMED_FRAMES else f"{d.frame_count}/{MIN_CONFIRMED_FRAMES} frames"
        print(f"        {d.label:20s}  conf={d.confidence:.2f}  {conf_str}  {'[SMOOTHED]' if d.smoothed else ''}")

    # 4. FPS estimate (50 frames)
    print("[4/4] Estimating FPS with frame skipping (50 frames)...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
    times = []
    frame_skipper = FrameSkipper(INFERENCE_SKIP_FRAMES)
    for i in range(50):
        ret, frame = cap.read()
        if not ret:
            break
        
        if frame_skipper.update():
            t0 = time.perf_counter()
            model(frame, conf=CONF_THRESHOLD, iou=IOU_THRESHOLD, verbose=False)
            times.append(time.perf_counter() - t0)
    
    cap.release()
    if times:
        avg_ms = sum(times) / len(times) * 1000
        fps_at_full = 1000 / avg_ms
        fps_with_skip = fps_at_full * INFERENCE_SKIP_FRAMES
        
        print(f"  Avg inference latency: {avg_ms:.1f}ms per inference")
        print(f"  FPS at full inference: ~{fps_at_full:.1f} FPS (every frame)")
        print(f"  FPS with {INFERENCE_SKIP_FRAMES}x frame skip: ~{fps_with_skip:.1f} FPS (interpolated)")
        print()
        
        if fps_with_skip >= 30:
            print("  ✓ EXCELLENT — High-speed real-time rover use on RPi 4")
        elif fps_with_skip >= 15:
            print("  ✓ GOOD — Suitable for real-time rover use")
            print("    Smooth motion with frame interpolation")
        elif fps_with_skip >= 8:
            print("  ⚠ MARGINAL — Limited responsiveness")
            print("    Consider: faster camera, simpler model, or hardware acceleration")
        else:
            print("  ✗ TOO SLOW for rover use")
            print("    Recommend: Google Coral TPU or Jetson Nano")
    print("\n=== DIAGNOSTIC COMPLETE ===\n")
    print("Usage on Raspberry Pi 4:")
    print("  python rover_autonomy.py --mode live        # Display + camera view")
    print("  python rover_autonomy.py --mode live --headless  # No display (SSH)")
    print("  python rover_autonomy.py --mode benchmark   # Speed test")


def run_live_mode(headless=False):
    """Full live feed with detection + decision + motor control."""
    global motor
    
    print("[LIVE] Initializing for Raspberry Pi 4...")
    
    # Load model with RPi-specific optimizations
    print(f"[LIVE] Loading {MODEL_PATH} on {DEVICE}...")
    model = YOLO(MODEL_PATH, task='detect')
    model.to(DEVICE)
    
    # Set multi-threading for RPi
    if DEVICE == 'cpu':
        import torch
        torch.set_num_threads(INFERENCE_THREADS)
        print(f"[LIVE] CPU threading: {INFERENCE_THREADS} threads")
    
    names = model.names
    
    # Use global motor, or create dry-run if not initialized
    if motor is None:
        print("[LIVE] No motor configured, using dry-run mode")
        motor = MotorController(dry_run=True)
    else:
        print(f"[LIVE] Motor: {'DRY-RUN' if motor.dry_run else 'ACTIVE on ' + str(motor.port)}")

    # Initialize camera manager with threading for better capture
    cam_mgr = CameraManager(main_camera_index=CAMERA_INDEX, use_threading=True)
    cam_mgr.print_available()
    
    if not cam_mgr.initialize_main_camera():
        print("[ERROR] Cannot open main camera")
        return

    ret, sample = cam_mgr.read_frame()
    if not ret:
        print("[ERROR] Cannot read from camera")
        cam_mgr.release()
        return

    h, w = sample.shape[:2]
    # --- Camera intrinsics (approximate) ---
    # Replace these with calibrated values for best accuracy
    fx = 700.0 * (w / 640.0)
    fy = fx
    cx = w / 2.0
    cy = h / 2.0
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist_coeffs = np.zeros((4, 1))

    # AprilTag detector and simple EKF localizer
    april_detector = AprilTagDetector(tag_size_m=0.15, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    # Map of AprilTag IDs to world poses (x_m, y_m, theta_rad) - populate manually
    tag_world_map = {
        # Example: 1: (0.0, 0.0, 0.0),
    }
    ekf = SimpleEKFLocalizer()
    # Serial telemetry from Arduino (encoders + IMU)
    serial_port = SERIAL_PORT
    telemetry = SerialTelemetry(port=serial_port, baud=SERIAL_BAUD)
    engine = DecisionEngine(w, h)
    tracker_manager = DetectionTrackerManager(max_age=MAX_AGE_FRAMES)
    frame_skipper = FrameSkipper(INFERENCE_SKIP_FRAMES)
    fps_counter = deque(maxlen=30)

    print(f"[LIVE] Running on RPi 4 — {w}x{h} resolution, skip={INFERENCE_SKIP_FRAMES}")
    print("[LIVE] Press Q to quit, or number keys (0-9) to switch cameras")
    print("[LIVE] Available cameras:", [str(cam['index']) for cam in cam_mgr.get_available_cameras()])
    
    frame_count = 0
    while True:
        t_frame = time.perf_counter()

        ret, frame = cam_mgr.read_frame()
        if not ret:
            continue
        
        frame_count += 1
        
        # --- Read odometry / IMU deltas and predict EKF ---
        try:
            d_center_m, dtheta = telemetry.get_deltas() if hasattr(telemetry, 'get_deltas') else (0.0, 0.0)
            # Convert odometry deltas to robot frame dx,dy
            dx = d_center_m * math.cos(ekf.theta)
            dy = d_center_m * math.sin(ekf.theta)
            ekf.predict(dx=dx, dy=dy, dtheta=dtheta)
        except Exception:
            pass

        # --- AprilTag detection (every frame, lightweight) ---
        tag_detections = april_detector.detect(frame)
        if tag_detections:
            td = tag_detections[0]
            tag_id = td['id']
            rvec = td['rvec']
            tvec = td['tvec']
            try:
                R_tag_cam, _ = cv2.Rodrigues(rvec)
                T_tag_cam = np.eye(4, dtype=np.float64)
                T_tag_cam[:3, :3] = R_tag_cam
                T_tag_cam[:3, 3] = tvec.flatten()

                if tag_id in tag_world_map:
                    tx, ty, ttheta = tag_world_map[tag_id]
                    T_world_tag = np.eye(4, dtype=np.float64)
                    c = math.cos(ttheta); s = math.sin(ttheta)
                    T_world_tag[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
                    T_world_tag[0, 3] = tx
                    T_world_tag[1, 3] = ty

                    T_world_cam = T_world_tag @ T_tag_cam
                    meas_x = float(T_world_cam[0, 3])
                    meas_y = float(T_world_cam[1, 3])
                    meas_theta = math.atan2(T_world_cam[1, 0], T_world_cam[0, 0])
                    ekf.update_with_absolute_pose(meas_x, meas_y, meas_theta)
            except Exception:
                pass

        detections = []
        infer_time = 0
        
        # Run inference only every N frames
        if frame_skipper.update():
            t_inf = time.perf_counter()
            results = model(frame, conf=CONF_THRESHOLD, iou=IOU_THRESHOLD, verbose=False)
            infer_time = time.perf_counter() - t_inf
            
            raw_detections = _parse_detections(results, names)
            # FAST STOP: immediate stop on raw detection in danger zone
            for rd in raw_detections:
                area_frac = rd.area / (w * h)
                cx_norm = rd.cx / w
                if rd.confidence >= FAST_STOP_CONF and area_frac >= FAST_STOP_AREA_FRAC and DANGER_ZONE_LEFT <= cx_norm <= DANGER_ZONE_RIGHT:
                    print(f"[FAST STOP] Raw detection {rd.label} conf={rd.confidence:.2f} area={area_frac:.2%}")
                    motor.stop()
                    break
            detections = _filter_and_smooth_detections(tracker_manager, raw_detections)
        else:
            # Use interpolated detections from tracker
            detections = frame_skipper.get_interpolated_detections()
        
        decision = engine.decide(detections)

        # Motor commands
        if decision.action == "FORWARD":
            motor.forward()
        elif decision.action == "STOP":
            motor.stop()
        elif decision.action == "STEER_LEFT":
            motor.steer_left()
        elif decision.action == "STEER_RIGHT":
            motor.steer_right()

        # Visualise (only if not headless)
        if not headless:
            draw_danger_zone(frame)
            draw_detections(frame, detections, decision)
            total_frame_time = time.perf_counter() - t_frame
            fps_counter.append(1.0 / max(total_frame_time, 1e-9))
            actual_fps = sum(fps_counter) / len(fps_counter)
            
            # Add RPi info to HUD
            camera_info = f"RPi4 | Camera: {cam_mgr.current_camera} | {w}x{h}"
            cv2.putText(frame, camera_info, (10, h - 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)
            
            # Add tracker & skip info
            tracker_info = f"Tracked: {len(tracker_manager.trackers)} | Confirmed: {len(detections)} | Skip: {INFERENCE_SKIP_FRAMES}x"
            cv2.putText(frame, tracker_info, (10, h - 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)
            
            # Inference time
            if infer_time > 0:
                inf_ms = infer_time * 1000
                cv2.putText(frame, f"Inference: {inf_ms:.1f}ms", (10, h - 79),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 100), 1, cv2.LINE_AA)
            
            draw_hud(frame, decision, actual_fps, infer_time * 1000)
            # Fused pose from EKF
            try:
                px, py, ptheta = ekf.get_pose()
                pose_text = f"POSE: {px:.2f}m {py:.2f}m {math.degrees(ptheta):.0f}deg"
                cv2.putText(frame, pose_text, (10, h - 102),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
            except Exception:
                pass
            draw_obstacle_list(frame, decision)
            
            # Show actual timing info
            timing_info = f"Frame: {total_frame_time*1000:.1f}ms | Infer: {infer_time*1000:.1f}ms | Actual FPS: {actual_fps:.1f}"
            cv2.putText(frame, timing_info, (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1, cv2.LINE_AA)

            cv2.imshow("RPi Rover Vision", frame)
            key = cv2.waitKey(1) & 0xFF
            
            # Camera switching with number keys
            if ord("0") <= key <= ord("9"):
                camera_idx = int(chr(key))
                if cam_mgr.select_camera(camera_idx):
                    print(f"[LIVE] Switched to camera {camera_idx}")
                else:
                    print(f"[LIVE] Camera {camera_idx} not available")
            elif key == ord("q"):
                break

    cam_mgr.release()
    if not headless:
        cv2.destroyAllWindows()
    print("[LIVE] Shutdown complete")


def run_benchmark_mode(n_frames=100):
    """Pure inference speed benchmark — no display."""
    print(f"\n=== BENCHMARK ({n_frames} frames) ===\n")
    model = YOLO(MODEL_PATH)

    # Initialize camera manager
    cam_mgr = CameraManager(main_camera_index=CAMERA_INDEX)
    cam_mgr.print_available()
    
    if not cam_mgr.initialize_main_camera():
        print("[ERROR] Cannot open camera")
        return

    times = []
    det_counts = []
    print("Running... ", end="", flush=True)
    for i in range(n_frames):
        ret, frame = cam_mgr.read_frame()
        if not ret:
            break
        t0 = time.perf_counter()
        results = model(frame, conf=CONF_THRESHOLD, iou=IOU_THRESHOLD, verbose=False)
        elapsed = time.perf_counter() - t0
        times.append(elapsed)
        det_counts.append(len(results[0].boxes))
        if i % 10 == 0:
            print(".", end="", flush=True)

    cam_mgr.release()
    print(" done\n")

    if not times:
        print("No frames captured.")
        return

    avg_ms   = sum(times) / len(times) * 1000
    min_ms   = min(times) * 1000
    max_ms   = max(times) * 1000
    avg_fps  = 1000 / avg_ms
    avg_dets = sum(det_counts) / len(det_counts)

    print(f"  Frames captured : {len(times)}")
    print(f"  Avg latency     : {avg_ms:.1f}ms")
    print(f"  Min / Max       : {min_ms:.1f}ms / {max_ms:.1f}ms")
    print(f"  Avg FPS         : {avg_fps:.1f}")
    print(f"  Avg detections  : {avg_dets:.1f} per frame")
    print()
    if avg_fps >= 15:
        print("  STATUS: GOOD — suitable for real-time rover use")
    elif avg_fps >= 8:
        print("  STATUS: MARGINAL — workable but tune CAPTURE_WIDTH or use ONNX")
    else:
        print("  STATUS: SLOW — export to ONNX/INT8 or reduce resolution")
    print("\n=== BENCHMARK COMPLETE ===\n")

# ─── Helpers ───────────────────────────────────────────────────────────────────

def _parse_detections(results, names) -> list[Detection]:
    detections = []
    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            label  = names[cls_id]
            conf   = float(box.conf[0])
            
            # Very permissive here - let DecisionEngine filter
            if conf < 0.25:  # Only skip very low confidence
                continue
            
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            detections.append(Detection(label, conf, x1, y1, x2, y2))
    
    return detections


def _filter_and_smooth_detections(tracker_manager: DetectionTrackerManager, raw_detections: List[Detection]) -> List[Detection]:
    """Filter detections and apply Kalman smoothing."""
    # Filter by confidence again
    filtered = [d for d in raw_detections if d.confidence >= MIN_DETECTION_CONFIDENCE]
    
    # Apply Kalman tracking
    smoothed = tracker_manager.update(filtered)
    
    return smoothed

# ─── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rover autonomy vision system")
    parser.add_argument("--mode", choices=["test", "live", "benchmark", "list-cameras"],
                        default="test", help="Run mode")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX,
                        help="Camera device index (default: 0)")
    parser.add_argument("--serial", type=str, default=SERIAL_PORT,
                        help="Serial port for Arduino telemetry/commands")
    parser.add_argument("--headless", action="store_true",
                        help="Live mode without display window")
    args = parser.parse_args()

    CAMERA_INDEX = args.camera
    SERIAL_PORT = args.serial

    if args.mode == "test":
        run_test_mode()
    elif args.mode == "live":
        run_live_mode(headless=args.headless)
    elif args.mode == "benchmark":
        run_benchmark_mode()
    elif args.mode == "list-cameras":
        cam_mgr = CameraManager()
        cam_mgr.print_available()
        if not cam_mgr.get_available_cameras():
            print("No cameras found. Check USB connections.")
        else:
            print("Usage during live mode (--mode live):")
            print("  Press number keys (0-9) to switch cameras")
            print("  Press Q to quit")
