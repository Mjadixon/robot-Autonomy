# Hardware Setup & Testing Guide

## Overview
You'll test the rover autonomy system on your **laptop first**, then deploy to **Raspberry Pi**.

```
Laptop/PC (Windows)
    ↓
    USB Cable
    ↓
Arduino (Uno/Nano/Mega)
    ↓
Motor Driver (L298N / TB6612FNG)
    ↓
Motors (left + right)
```

---

## Part 1: Arduino Hardware Setup

### Components Needed
- Arduino Uno, Nano, or Mega
- Motor Driver (L298N or TB6612FNG)
- 2x DC motors
- Motor power supply (6-12V, depends on motors)
- Laptop/RPi with USB cable

### Wiring (L298N Motor Driver)

```
Arduino Pin 9  (PWM) → L298N IN1/IN3 (left motor PWM)
Arduino Pin 8  (DIR) → L298N IN2/IN4 (left motor direction)
Arduino Pin 10 (PWM) → L298N IN3/IN4 (right motor PWM)
Arduino Pin 11 (DIR) → L298N IN1/IN2 (right motor direction)

Arduino GND → L298N GND (IMPORTANT!)
L298N OUT1/OUT2 → Left motor
L298N OUT3/OUT4 → Right motor
L298N +12V → Motor power supply (+)
L298N GND → Motor power supply (-)
```

**⚠️ CRITICAL**: Connect Arduino GND to motor driver GND (ground common point)

### Upload Arduino Code
1. Open Arduino IDE
2. Create new sketch
3. Copy contents of `rover_motor_control.ino`
4. Tools → Board → Select your Arduino board
5. Tools → Port → Select COM port (e.g., COM3)
6. Click **Upload**
7. Open Serial Monitor (9600 baud) → should see "Motor Control Ready"

---

## Part 2: Setup Python Environment (Laptop)

### Install Dependencies
```powershell
# Windows PowerShell
cd "c:\Users\micah\OneDrive\Documents\Code (school)\robot Autonomy"

# Install required packages
pip install pyserial opencv-python ultralytics numpy
```

### Verify Installation
```powershell
python -c "import serial; print('✓ pyserial OK')"
python -c "import cv2; print('✓ opencv-python OK')"
python -c "from ultralytics import YOLO; print('✓ ultralytics OK')"
```

---

## Part 3: Testing on Laptop (Step-by-Step)

### Step 1: List Serial Ports
```powershell
python hardware_test.py --list
```
Output should show:
```
📡 Available serial ports:
   [1] COM3 - Arduino Uno (COM3)
```
**Note your port** (e.g., COM3)

### Step 2: Test Arduino Connection
```powershell
python hardware_test.py --test COM3
```
Expected output:
```
🔌 Connecting to COM3 @ 9600 baud...
   Arduino: Motor Control Ready
✅ Connection successful!
```

### Step 3: Interactive Motor Test
```powershell
python hardware_test.py --motor COM3
```
Try these commands:
```
>> f200          # Forward at PWM=200
>> l150          # Steer left at PWM=150
>> r150          # Steer right at PWM=150
>> s             # Stop
>> q             # Quit
```

**Watch your motors!** They should:
- `f200`: Both motors spin forward at ~78% speed
- `l150`: Left motor ~29% speed, right motor ~59% speed (turns left)
- `r150`: Left motor ~59% speed, right motor ~29% speed (turns right)
- `s`: Both stop

### Step 4: Dry-Run Simulation (No Motors)
Test rover autonomy logic without motors spinning:
```powershell
python hardware_test.py --sim COM3
```
Shows decisions without sending to motors (Arduino still connected but won't spin).

### Step 5: Live Hardware Test
Full system test with motors spinning:
```powershell
python hardware_test.py --run COM3
```

---

## Part 4: Full Camera + Vision Testing (Laptop)

### Option A: Dry-Run Mode (Test without camera)
```powershell
python rover_autonomy.py --mode test
```

### Option B: Live with Camera (if connected)
```powershell
python rover_autonomy.py --mode live
```

---

## Part 5: Deploy to Raspberry Pi

### 1. Install OS & Python
```bash
# On RPi, install dependencies
sudo apt-get update
sudo apt-get install python3-pip python3-opencv python3-numpy
pip3 install pyserial ultralytics
```

### 2. Copy Code to RPi
```bash
# From your laptop, SCP files to RPi
# Adjust user/IP for your RPi setup
scp rover_autonomy.py pi@192.168.x.x:/home/pi/rover/
scp rover_motor_control.ino pi@192.168.x.x:/home/pi/rover/
```

### 3. Wire Arduino to RPi
Same wiring as laptop, but:
- Use `/dev/ttyUSB0` instead of `COM3`
- RPi USB ports typically auto-detect

### 4. Test on RPi
```bash
# SSH into RPi
ssh pi@192.168.x.x

# List ports
python3 hardware_test.py --list

# Test motor connection
python3 hardware_test.py --test /dev/ttyUSB0

# Run full autonomy
python3 rover_autonomy.py --mode live
```

---

## Troubleshooting

### "No serial ports found"
- Check USB cable is connected
- Arduino not showing in Device Manager → reinstall CH340/FTDI drivers
- Try different USB port on laptop

### "Connection failed / Arduino not responding"
- Check baud rate (should be 9600 in both Arduino code and Python)
- Check Arduino Tools → Port setting
- Try re-uploading Arduino sketch
- Open Serial Monitor in Arduino IDE to verify Arduino is working

### Motors won't spin
- Check motor power supply voltage (6-12V typically)
- Check motor driver wiring, especially GND connections
- Try manually adjusting `MOTOR_FORWARD` / `MOTOR_BACKWARD` in Arduino code
- Test with `hardware_test.py --motor COM3` and send `f255`

### Vision slow on laptop
- Reduce `CAPTURE_WIDTH` / `CAPTURE_HEIGHT` in rover_autonomy.py
- Increase `INFERENCE_SKIP_FRAMES`
- Disable visualization (`--mode live`)

### RPi performance issues
- Use YOLO11n (already configured)
- Enable INT8 quantization (`USE_INT8_QUANT = True`)
- Run on CPU only (`DEVICE = 'cpu'`)
- Reduce capture resolution

---

## Quick Reference

| Command | Purpose |
|---------|---------|
| `python hardware_test.py --list` | Find Arduino port |
| `python hardware_test.py --test COM3` | Check connection |
| `python hardware_test.py --motor COM3` | Manual motor control |
| `python hardware_test.py --sim COM3` | Dry-run simulation |
| `python hardware_test.py --run COM3` | Full hardware test |
| `python rover_autonomy.py --mode live` | Live camera feed |

---

## Key Files
- **rover_motor_control.ino** → Upload to Arduino
- **rover_autonomy.py** → Main Python code (updated MotorController)
- **hardware_test.py** → Testing utility
- **test_rover_autonomy.py** → Unit tests (no hardware needed)

---

**Next Steps:**
1. ✅ Wire up Arduino + motors
2. ✅ Upload `rover_motor_control.ino`
3. ✅ Run `python hardware_test.py --list`
4. ✅ Test motor control with `--motor COM3`
5. ✅ Deploy to RPi when confident
