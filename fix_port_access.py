#!/usr/bin/env python3
"""
Serial Port Diagnostic Tool
Finds and fixes COM port access issues
"""

import sys
import time
import subprocess


def find_port_user_windows():
    """Find what process is using a COM port on Windows."""
    try:
        # Use tasklist to find Arduino or serial-related processes
        result = subprocess.run(
            ["tasklist", "/v"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        print("\n📋 ACTIVE PROCESSES (looking for Arduino/Serial/Python):\n")
        suspicious = []
        for line in result.stdout.split('\n'):
            if any(x in line.upper() for x in ['ARDUINO', 'PYTHON', 'SERIAL', 'COM']):
                print(f"   {line}")
                suspicious.append(line)
        
        if suspicious:
            print(f"\n⚠️  Found {len(suspicious)} potential process(es) using COM port")
            return True
        return False
    except Exception as e:
        print(f"Could not list processes: {e}")
        return False


def reset_com_port():
    """Reset USB device on Windows."""
    print("\n🔌 HARDWARE RESET:")
    print("   1. Unplug Arduino USB cable")
    print("   2. Wait 3 seconds")
    print("   3. Plug back in")
    print("\nDoing this now...")
    
    time.sleep(1)
    print("   ⏳ Unplugged... waiting...")
    time.sleep(3)
    print("   ✅ Ready to plug back in!")


def list_all_ports():
    """List all available COM ports."""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            print("\n❌ NO SERIAL PORTS FOUND!")
            print("   - Check USB cable is connected")
            print("   - Check Arduino board is powered")
            return []
        
        print("\n📡 AVAILABLE PORTS:\n")
        for i, port in enumerate(ports, 1):
            print(f"   [{i}] {port.device:8s} - {port.description}")
        
        return ports
    except ImportError:
        print("❌ pyserial not installed")
        return []


def kill_arduino_ide():
    """Close Arduino IDE if running."""
    try:
        print("\n🔨 Attempting to close Arduino IDE...")
        subprocess.run(
            ["taskkill", "/IM", "arduino.exe", "/F"],
            timeout=5,
            capture_output=True
        )
        print("   ✅ Arduino IDE closed (if it was running)")
        time.sleep(2)
    except Exception as e:
        print(f"   (Arduino IDE not running or already closed)")


def force_close_port(port):
    """Forcefully close port using Windows mode."""
    try:
        import subprocess
        print(f"\n🔧 Attempting to reset {port}...")
        # Use mode command to reset COM port
        subprocess.run(
            f"mode {port}: BAUD=9600 PARITY=N DATA=8 STOP=1 TO=off XON=off ODSR=off OCTS=off DTRDSR=off",
            shell=True,
            timeout=5,
            capture_output=True
        )
        print(f"   ✅ Port reset attempt complete")
        time.sleep(2)
    except Exception as e:
        print(f"   (Reset not available: {e})")


def main():
    """Run diagnostics."""
    print("""
╔════════════════════════════════════════════════════════════════╗
║         SERIAL PORT ACCESS DENIED DIAGNOSTIC                   ║
║     This tool will help identify and fix COM port issues       ║
╚════════════════════════════════════════════════════════════════╝
    """)
    
    # Step 1: List processes
    print("\n[STEP 1/5] Checking for processes holding COM port...")
    has_process = find_port_user_windows()
    
    if has_process:
        print("\n💡 ACTION: Close any Arduino IDE windows (Serial Monitor)")
        print("           Close any other Python terminals using the port")
        input("\nPress ENTER once closed, then we'll continue...")
    
    # Step 2: Kill Arduino
    print("\n[STEP 2/5] Closing Arduino IDE (if running)...")
    kill_arduino_ide()
    
    # Step 3: Force close port
    print("\n[STEP 3/5] Resetting COM port...")
    force_close_port("COM11")
    
    # Step 4: Hardware reset
    print("\n[STEP 4/5] Hardware reset needed...")
    input("Press ENTER, then:\n   1. UNPLUG Arduino USB\n   2. Wait 3 seconds\n   3. PLUG back in\n\n>> ")
    time.sleep(2)
    
    # Step 5: Verify ports
    print("\n[STEP 5/5] Verifying ports are available...")
    time.sleep(2)
    ports = list_all_ports()
    
    if ports:
        print("\n✅ SUCCESS! Ports are available.")
        print(f"\nYour Arduino should be on one of these ports.")
        print("Try: python hardware_test.py --motor COM11")
    else:
        print("\n⚠️  Still no ports. Troubleshooting:")
        print("   1. Check USB cable (try different port on laptop)")
        print("   2. Check Arduino board is powered")
        print("   3. Check Arduino drivers are installed")
        print("   4. Restart your computer")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nAborted by user.")
        sys.exit(1)
