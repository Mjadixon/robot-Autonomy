# Running Full Rover Autonomy with Arduino

Your rover is now fully integrated! Here's how to run it.

## ✅ Prerequisites

1. **Arduino firmware uploaded** (`rover_motor_control.ino`)
2. **Motor controller working** (tested with `hardware_test.py --motor COM11`)
3. **Python packages installed:**
   ```powershell
   pip install pyserial opencv-python ultralytics numpy
   ```
4. **USB cable connected** from Arduino to laptop

---

## 🚀 Quick Start

### **Run Full Autonomy with Motors (LIVE)**
```powershell
python run_rover.py --mode live --port COM11
```

This will:
- Connect to Arduino on COM11
- Load YOLO vision model
- Open camera feed
- Run decision engine
- **Send motor commands to rover**

### **Dry-Run (No Motors Spinning)**
```powershell
python run_rover.py --mode live --dry-run
```

Same as above but motors stay silent (safe testing).

### **Auto-detect Arduino Port**
```powershell
python run_rover.py --mode live
```

Program will search for Arduino automatically.

---

## 📋 All Commands

```powershell
# LIVE: Full vision + decision + motors
python run_rover.py --mode live --port COM11

# DRY-RUN: Test logic without spinning motors
python run_rover.py --mode live --dry-run

# TEST: Diagnostics (camera, model, inference speed)
python run_rover.py --mode test

# BENCHMARK: FPS measurement (50 frames, no display)
python run_rover.py --mode benchmark

# LIST: Show available cameras
python run_rover.py --mode list-cameras

# HELP: Show all options
python run_rover.py --help
```

---

## 🎮 During Live Mode

While the camera feed is running:

| Key | Action |
|-----|--------|
| **0-9** | Switch camera (if multiple connected) |
| **Q** | Quit and stop motors |
| **Ctrl+C** | Emergency stop |

---

## 📊 What You'll See (Live Mode)

```
┌─────────────────────────────────────────────┐
│ HUD (top-left)                              │
│ ACTION: FORWARD (or STOP/STEER_LEFT/RIGHT)  │
│ Reason: Clear path (or obstacle info)       │
│                                             │
│ LIVE CAMERA FEED                            │
│ - Green rectangle = danger zone             │
│ - Red boxes = detected people               │
│ - Orange boxes = detected vehicles          │
│                                             │
│ STATS (bottom): FPS, latency, obstacles     │
│                                             │
│ PANEL (right): List of detected obstacles   │
│ - Priority levels                           │
│ - Confidence scores                         │
└─────────────────────────────────────────────┘
```

---

## 🔧 Troubleshooting

### Motors don't move even though HUD says "FORWARD"
- **Check physical wiring**: IN1-IN4 pins, motor connections
- **Check Arduino**: Upload `rover_motor_control.ino` again
- **Test with:** `python hardware_test.py --motor COM11`
- **Check PWM:** Should see values like `pwm=200` in Arduino Serial Monitor

### "PermissionError: Access is denied" on COM11
```powershell
# Option 1: Close Arduino IDE Serial Monitor
# Option 2: Unplug/replug Arduino USB
# Option 3: Run diagnostic
python fix_port_access.py
```

### Vision is slow (low FPS)
- Reduce `CAPTURE_WIDTH`/`CAPTURE_HEIGHT` in `rover_autonomy.py`
- Increase `INFERENCE_SKIP_FRAMES` (detections every 2-3 frames instead of 1)
- Run `python run_rover.py --mode benchmark` to measure actual FPS

### No camera found
```powershell
python run_rover.py --mode list-cameras
```
Should show at least one camera. If not, check USB cable.

---

## 📈 Performance Tips

### For Laptop Testing (Windows):
```powershell
# Dry-run to test decision logic
python run_rover.py --mode live --dry-run

# Once confident, enable motors
python run_rover.py --mode live --port COM11
```

### For Raspberry Pi Deployment:
```bash
# SSH into RPi
ssh pi@192.168.x.x

# List ports (should be /dev/ttyUSB0)
python3 run_rover.py --mode list-cameras

# Run with motors
python3 run_rover.py --mode live --port /dev/ttyUSB0
```

---

## 🔌 Hardware Setup Reminder

```
Laptop/RPi USB
    ↓
Arduino Uno
    ↓
L298N Motor Driver
    ↓
Left Motor  +  Right Motor
```

**Critical connections:**
- Arduino GND ↔ Motor Driver GND
- Pins 10, 9, 7, 6, 5, 4 ↔ ENA, ENB, IN1, IN2, IN3, IN4
- Motor power: Separate 6-12V supply (NOT from Arduino USB)

---

## 📝 File Reference

| File | Purpose |
|------|---------|
| **run_rover.py** | Main launcher (Arduino + vision) |
| **rover_autonomy.py** | Vision system (YOLO + DecisionEngine) |
| **rover_motor_control.ino** | Arduino firmware |
| **hardware_test.py** | Motor testing utility |
| **test_rover_autonomy.py** | Unit tests (no hardware) |

---

## ✨ What's Running

When you run `python run_rover.py --mode live --port COM11`:

1. **Arduino Connection**: Serial link to motor controller
2. **Camera Capture**: USB camera (threaded for smooth FPS)
3. **YOLO Detection**: Real-time object detection
4. **Kalman Tracking**: Smooth object trajectories
5. **Decision Engine**: Obstacle avoidance logic
6. **Motor Commands**: Sent ~20-30 times per second
7. **Visualization**: HUD + detection boxes (if not headless)

---

## 🚦 Safety Notes

- Rover will move when motors are connected!
- Always test in **dry-run mode first** (`--dry-run`)
- Keep hands away during live testing
- Press **Q** or **Ctrl+C** to stop immediately
- Unplug motors if not in use

---

## Next Steps

1. ✅ Test with `--dry-run` to verify vision logic
2. ✅ Test motors with `hardware_test.py --motor COM11`
3. ✅ Run live mode: `python run_rover.py --mode live --dry-run`
4. ✅ Enable motors: `python run_rover.py --mode live --port COM11`
5. ✅ Deploy to RPi when confident

---

**You're ready to go!** 🤖
