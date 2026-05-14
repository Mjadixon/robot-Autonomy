# Quick Fix: COM Port Access Denied

## 🚨 Problem
```
PermissionError(13, 'Access is denied.', None, 5)
```
Something is holding COM11 open and won't let Python use it.

## ✅ Quick Fixes (Try in Order)

### Fix #1: Close Arduino IDE (Most Common)
1. Look for **Arduino IDE** window
2. Click **Tools** → **Serial Monitor** → **Close** (or just close the tab)
3. Close Arduino IDE completely
4. Wait 2 seconds
5. Try again: `python hardware_test.py --motor COM11`

### Fix #2: Close Any Other Python Windows
1. Look for any Python terminals or Jupyter notebooks
2. Close them completely
3. Wait 2 seconds
4. Try again

### Fix #3: Hardware Reset (Most Reliable)
**This almost always works:**
1. **UNPLUG** the Arduino USB cable
2. Wait **3 seconds** (important!)
3. **PLUG back in**
4. Wait 2 seconds for drivers to load
5. Try: `python hardware_test.py --motor COM11`

### Fix #4: Use the Diagnostic Tool
```powershell
python fix_port_access.py
```
This will:
- Show what processes are using COM11
- Close Arduino IDE automatically
- Walk you through hardware reset
- List all available ports

### Fix #5: Try a Different USB Port
1. Unplug Arduino from current USB port
2. Plug into a **different USB port** on your laptop
3. Run: `python hardware_test.py --list`
4. Find the new COM port (e.g., COM5)
5. Try: `python hardware_test.py --motor COM5`

### Fix #6: Check if Arduino is Actually There
```powershell
python hardware_test.py --list
```
If COM11 doesn't appear in the list, the Arduino isn't connected properly.

## 🔧 If Nothing Works

### Windows Device Manager Check:
1. Open **Device Manager** (right-click Start menu)
2. Look for **Ports (COM & LPT)**
3. You should see something like "Arduino Uno (COM11)"
4. If you see a ⚠️ warning symbol, right-click → **Update driver**

### Full Nuclear Option:
```powershell
# Open PowerShell as Administrator, then:
taskkill /IM arduino.exe /F
taskkill /IM python.exe /F
```

Then:
1. Unplug Arduino
2. Restart your laptop
3. Plug Arduino back in
4. Try again

## ✅ How to Test It Works

Once fixed, this should work:
```powershell
python hardware_test.py --list
```

Output should show:
```
📡 Available serial ports:
   [1] COM11 - Arduino Uno (COM11)
```

Then try:
```powershell
python hardware_test.py --motor COM11
```

You should see the interactive menu without errors.

---

**Start with Fix #1 (close Arduino IDE), then try Fix #3 (unplug/replug) if that doesn't work.**
