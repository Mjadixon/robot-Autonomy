# PWM Debugging Guide

## Problem: PWM stays at 0

The Arduino is receiving commands, but PWM values are always 0. This guide will help you debug the issue.

## Step 1: Re-upload Arduino Code

The Arduino code has been updated with debugging output. Re-upload it:

1. Open Arduino IDE
2. Copy **new** contents of `rover_motor_control.ino`
3. Tools → Port → COM11
4. Sketch → Upload
5. Open Serial Monitor (Tools → Serial Monitor)
   - Set baud to **9600**
   - Should see: `Motor Control Ready (L298N)`

## Step 2: Run Interactive Motor Test

```powershell
python hardware_test.py --motor COM11
```

Now when you send a command, you'll see **TWO** views:

### On Python Side (Your Terminal):
```
>> f200
[TX] 'F:200\n'
[RX] [RX] 'action=F colonIdx=1 pwm=200' 
[RX] '✓ FORWARD PWM=200'
```

### On Arduino Serial Monitor:
```
[RX] 'F:200' → action=F colonIdx=1 pwm=200
✓ FORWARD PWM=200
```

## Expected vs Actual

### ✅ GOOD OUTPUT (PWM being parsed correctly):
```
[RX] 'F:200' → action=F colonIdx=1 pwm=200
✓ FORWARD PWM=200
```

### ❌ BAD OUTPUT #1 (No colon received):
```
[RX] 'F200' → action=F colonIdx=-1 NO_COLON - using 0
✓ FORWARD PWM=0
```
**Fix:** Check that hardware_test.py is sending the colon. Make sure you're using `--motor` flag, not `--run`.

### ❌ BAD OUTPUT #2 (Colon in wrong position):
```
[RX] ':F200' → action=: colonIdx=0 NO_COLON - using 0
✓ UNKNOWN
```
**Fix:** Something is wrong with the Python side. Try a fresh upload of both files.

### ❌ BAD OUTPUT #3 (Extra characters):
```
[RX] 'F:200 ' → action=F colonIdx=1 pwm=200
✓ FORWARD PWM=200
```
(Space after 200 - Arduino's trim() removes it, so this should work)

## Step 3: Diagnose Your Specific Issue

Run this test sequence and **copy the exact output**:

```powershell
python hardware_test.py --motor COM11
```

Then type these commands and share what you see:

```
>> f200
>> l150
>> r100
>> s
>> q
```

Look at **BOTH**:
- What's shown in the Python terminal (`[TX]` and `[RX]`)
- What's shown in Arduino Serial Monitor

## Possible Causes

| Symptom | Cause | Fix |
|---------|-------|-----|
| `colonIdx=-1` always | No colon in command | Check hardware_test.py sending format |
| `pwm=200` in Arduino but motor doesn't spin | Motor driver wiring | Check IN1-IN4 pins and motor connections |
| `[RX]` doesn't appear | Arduino not receiving data | Check USB cable, unplug/replug Arduino |
| Serial Monitor shows nothing | Wrong COM port or baud rate | Verify port in Arduino IDE Tools menu |

## Quick Test

If you want to verify the Arduino without Python, you can use Arduino's Serial Monitor:

1. Open Serial Monitor (9600 baud)
2. Type in the input box: `F:200`
3. Click Send
4. Should see in monitor: `[RX] 'F:200' → ... PWM=200`

This tells you if the Arduino side is working.

---

**Next:** Run the motor test above and share the output. That will tell us exactly what's happening!
