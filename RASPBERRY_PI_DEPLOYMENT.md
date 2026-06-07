# Deploying Rover Autonomy to Raspberry Pi 4

This guide walks through getting your rover code running on a **Raspberry Pi 4** (not on your laptop).

## Overview

The system architecture on RPi 4:

```
Raspberry Pi 4 (4GB+ RAM recommended)
    ├── USB → Arduino (motor control)
    ├── USB → Webcam (vision)
    └── WiFi/Ethernet (for SSH access)
```

The code is already optimized for RPi 4 with:
- Reduced resolution (640×480 instead of 1280×720)
- Lighter YOLO11n model (nano version)
- CPU-only inference (no GPU needed)
- INT8 quantization support (faster inference)
- Efficient frame skipping

---

## Prerequisites

### On Your Laptop
- ✅ Arduino code uploaded to your Arduino
- ✅ `rover_motor_control.ino` working and tested
- ✅ All code files ready to copy

### For Raspberry Pi
- Raspberry Pi 4 (2GB minimum, 4GB+ recommended)
- Micro SD card (32GB+)
- Power supply (5V 3A minimum)
- Cooling case (optional but recommended)
- USB cable to Arduino
- USB webcam (1080p or lower)
- Ethernet cable OR WiFi enabled

---

## Step 1: Prepare Raspberry Pi OS

### 1A. Flash OS to SD Card

**On your laptop:**
1. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Insert micro SD card
3. Open Imager → Choose OS → **Raspberry Pi OS (64-bit)**
4. Select your SD card and write

**Advanced options (recommended):**
- Set hostname: `rover`
- Enable SSH
- Set username: `pi` and password
- Configure WiFi (if not using Ethernet)

### 1B. Boot and SSH

Insert SD card into RPi, power on, wait 30 seconds, then SSH from your laptop:

```bash
# On your laptop (PowerShell or WSL)
ssh pi@192.168.x.x
# or
ssh pi@rover.local
```

If you don't know the IP:
```bash
# On RPi, at the console
hostname -I
```

---

## Step 2: Install Python & Dependencies on RPi

SSH into your RPi and run:

```bash
# Update package lists
sudo apt-get update
sudo apt-get upgrade -y

# Install Python3 + pip
sudo apt-get install -y python3-pip python3-dev

# Install system libraries for OpenCV
sudo apt-get install -y \
    python3-opencv \
    python3-numpy \
    libopenjp2-7 \
    libtiff6 \
    libwebp6 \
    libjasper1 \
    libharfbuzz0b \
    libwebpdemux0

# Install PySerial for Arduino communication
pip3 install pyserial

# Install Ultralytics (YOLO) — this will download the model (~30MB)
# Takes a few minutes on RPi 4
pip3 install ultralytics
```

**Verify installation:**
```bash
python3 -c "import serial; print('✓ pyserial')"
python3 -c "import cv2; print('✓ opencv')"
python3 -c "from ultralytics import YOLO; print('✓ ultralytics')"
```

---

## Step 3: Copy Code to Raspberry Pi

**From your laptop**, copy all rover files to RPi:

```powershell
# PowerShell on Windows
scp rover_autonomy.py pi@192.168.x.x:/home/pi/rover/
scp run_rover.py pi@192.168.x.x:/home/pi/rover/
scp requirements.txt pi@192.168.x.x:/home/pi/rover/
```

Or use SCP from any terminal:
```bash
scp rover_autonomy.py pi@192.168.x.x:/home/pi/rover/
scp run_rover.py pi@192.168.x.x:/home/pi/rover/
scp requirements.txt pi@192.168.x.x:/home/pi/rover/
```

Create the directory first if needed:
```bash
ssh pi@192.168.x.x
mkdir -p /home/pi/rover
```

---

## Step 4: Wire Hardware to Raspberry Pi

### Connect Arduino
Wire your Arduino to RPi **exactly the same as before**, using a **USB cable**:

```
Arduino USB ↔ Raspberry Pi USB port
```

The RPi will assign it to `/dev/ttyUSB0` (or similar).

### Connect Webcam
Plug USB webcam into any RPi USB port.

### Optional: Direct GPIO Motor Control
If you want **direct GPIO control** instead of Arduino (advanced):
- You'll need to modify the code to use `RPi.GPIO` or `gpiozero`
- For now, we recommend keeping the Arduino approach

---

## Step 5: Test Hardware on RPi

SSH into your RPi:

```bash
cd /home/pi/rover

# List serial ports
python3 -c "import serial.tools.list_ports; print(list(serial.tools.list_ports.comports()))"
```

You should see `/dev/ttyUSB0` or `/dev/ttyACM0` for the Arduino.

### Test Arduino Connection
```bash
python3 << 'EOF'
import serial
import time

port = "/dev/ttyUSB0"  # Adjust if different
try:
    ser = serial.Serial(port, 9600, timeout=1)
    time.sleep(2)
    response = ser.readline().decode().strip()
    print(f"✓ Arduino ready: {response}")
    ser.close()
except Exception as e:
    print(f"✗ Failed: {e}")
EOF
```

Expected output: `✓ Arduino ready: Motor Control Ready`

### Test Camera
```bash
python3 << 'EOF'
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    ret, frame = cap.read()
    if ret:
        print(f"✓ Camera OK: {frame.shape}")
    else:
        print("✗ Camera not responding")
    cap.release()
else:
    print("✗ Camera not found")
EOF
```

---

## Step 6: Run Rover Autonomy on RPi

### Option A: Dry-Run (Test Logic, No Motors)
```bash
cd /home/pi/rover
python3 run_rover.py --mode live --dry-run --headless
```

The `--headless` flag disables the GUI (since you're over SSH). On your laptop, you can display the feed with X11 forwarding.

### Option B: Live with Motors
```bash
cd /home/pi/rover
python3 run_rover.py --mode live --port /dev/ttyUSB0 --headless
```

**Substitute** `/dev/ttyUSB0` with your actual Arduino port if different.

### Option C: Benchmark Performance
```bash
python3 run_rover.py --mode benchmark
```

Shows FPS and inference latency (helps identify bottlenecks).

---

## Step 7: Optimize Performance (Optional but Recommended)

The code already has RPi 4 optimizations. To further speed up inference:

### Option 1: Export Model to ONNX (3-4x faster)
```bash
python3 << 'EOF'
from ultralytics import YOLO

print("Exporting YOLO11n to ONNX...")
model = YOLO('yolo11n.pt')
model.export(format='onnx')
print("✓ Exported to yolo11n.onnx")
EOF
```

Then update `rover_autonomy.py`:
```python
MODEL_PATH = "yolo11n.onnx"  # Instead of .pt
```

### Option 2: Use Lite Version
If YOLO11n is still slow (< 5 FPS), try:
```bash
python3 << 'EOF'
from ultralytics import YOLO
model = YOLO('yolo11s.pt')  # Smaller: yolo11n → yolo11s → yolo11m
# Then export to ONNX
model.export(format='onnx')
EOF
```

Update in `rover_autonomy.py`:
```python
MODEL_PATH = "yolo11s.onnx"
```

---

## Step 8: Running as a Background Service (Optional)

To make the rover start automatically on boot or run in the background:

### Create Systemd Service
```bash
sudo nano /etc/systemd/system/rover.service
```

Paste:
```ini
[Unit]
Description=Rover Autonomy System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/rover
ExecStart=/usr/bin/python3 /home/pi/rover/run_rover.py --mode live --headless
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Save (Ctrl+X, Y, Enter), then:
```bash
# Enable the service
sudo systemctl enable rover.service

# Start it
sudo systemctl start rover.service

# View logs
sudo journalctl -u rover.service -f
```

---

## Troubleshooting

### "No module named 'serial'"
```bash
pip3 install pyserial
```

### "No module named 'cv2'"
```bash
pip3 install opencv-python
```

### Arduino not found (`/dev/ttyUSB0` missing)
- Check USB connection
- Try: `ls /dev/tty*` to see available ports
- May be `/dev/ttyACM0` instead of `/dev/ttyUSB0`

### Very slow FPS (< 3 FPS)
1. Check with `--mode benchmark` to see inference time
2. Export to ONNX (see Step 7, Option 1)
3. Reduce resolution further: change `CAPTURE_HEIGHT = 360, CAPTURE_WIDTH = 480`
4. Increase frame skip: change `INFERENCE_SKIP_FRAMES = 3`

### Camera not responding
```bash
# Check if camera is recognized
v4l2-ctl --list-devices
# Try a different camera index (0, 1, 2, ...)
python3 run_rover.py --camera 1 --mode live --headless
```

### RPi gets hot or crashes
- Make sure power supply is **5V 3A minimum**
- Add heatsinks or cooling case
- Reduce `CAPTURE_WIDTH/HEIGHT` further

---

## Expected Performance on RPi 4

With default settings:

| Setting | FPS | Latency | Notes |
|---------|-----|---------|-------|
| YOLO11n (.pt) | 4-8 FPS | 120-250ms | Default, accurate |
| YOLO11n.onnx | 12-15 FPS | 60-90ms | 3x faster, recommended |
| YOLO11s.onnx | 8-12 FPS | 80-120ms | More accurate but slower |

If you need **20+ FPS**, use ONNX + frame skipping or run on a Raspberry Pi 5.

---

## Next Steps

1. SSH into RPi, test with `--mode benchmark`
2. Run `--mode live --dry-run` to verify autonomy logic
3. **Test motors carefully** with low PWM values first
4. Export to ONNX if FPS is < 8
5. Deploy as systemd service for production

Good luck! 🚀
