#!/usr/bin/env python3
"""
Hardware Testing & Setup Script
Test your Arduino + motors on laptop before deploying to RPi
"""

import sys
import time
import argparse


def list_ports():
    """List available serial ports."""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("❌ No serial ports found!")
            return []
        
        print("\n📡 Available serial ports:")
        for i, port in enumerate(ports, 1):
            print(f"   [{i}] {port.device} - {port.description}")
        return [p.device for p in ports]
    except ImportError:
        print("❌ pyserial not installed: pip install pyserial")
        return []


def test_connection(port, baudrate=9600):
    """Test connection to Arduino."""
    try:
        import serial
    except ImportError:
        print("❌ pyserial not installed: pip install pyserial")
        return False
    
    try:
        print(f"\n🔌 Connecting to {port} @ {baudrate} baud...")
        
        # Check if port is already open (common cause of access denied)
        try:
            ser = serial.Serial(port, baudrate, timeout=2)
        except PermissionError:
            print(f"\n❌ Access Denied: {port} is already in use!")
            print("\n🔧 SOLUTIONS:")
            print("   1. Close Arduino IDE Serial Monitor (if open)")
            print("   2. Close any other terminal/script using this port")
            print("   3. Unplug USB and wait 2 seconds, then plug back in")
            print("   4. Try a different USB port on your laptop\n")
            return False
        
        time.sleep(2)  # Wait for Arduino reset
        
        # Read any initial messages
        if ser.in_waiting:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"   Arduino: {msg}")
        
        # Send test command
        ser.write(b"S:0\n")
        ser.flush()
        
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        if response:
            print(f"   Arduino responded: {response}")
            print("✅ Connection successful!\n")
            ser.close()
            return True
        else:
            print("⚠️  Arduino not responding (check baud rate and connections)")
            ser.close()
            return False
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False


def interactive_motor_test(port, baudrate=9600):
    """Interactively test motors."""
    try:
        import serial
    except ImportError:
        print("❌ pyserial not installed: pip install pyserial")
        return
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)
        
        print(f"\n🎮 Motor Test Mode (connected to {port})")
        print("Commands:")
        print("  f<PWM>  → Forward    (e.g., f200)")
        print("  s       → Stop")
        print("  l<PWM>  → Steer Left (e.g., l150)")
        print("  r<PWM>  → Steer Right (e.g., r150)")
        print("  q       → Quit\n")
        
        # Read initial Arduino message
        while ser.in_waiting:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            if msg:
                print(f"[Arduino] {msg}")
        
        while True:
            try:
                cmd = input(">> ").strip().lower()
                if not cmd:
                    continue
                
                if cmd == 'q':
                    ser.write(b"S:0\n")
                    ser.flush()
                    time.sleep(0.5)
                    print("Stopped motors. Exiting.")
                    break
                
                action = cmd[0]
                pwm = 0
                if len(cmd) > 1:
                    try:
                        pwm = int(cmd[1:])
                        pwm = min(255, max(0, pwm))  # Clamp to 0-255
                    except ValueError:
                        print("Invalid PWM value (0-255)")
                        continue
                
                # Send to Arduino
                msg = f"{action.upper()}:{pwm}\n"
                print(f"[TX] {repr(msg)}")  # Show what's being sent
                ser.write(msg.encode())
                ser.flush()
                
                # Read response
                time.sleep(0.1)  # Give Arduino time to respond
                while ser.in_waiting:
                    response = ser.readline().decode('utf-8', errors='ignore').strip()
                    if response:
                        print(f"[RX] {response}")
            
            except KeyboardInterrupt:
                ser.write(b"S:0\n")
                ser.flush()
                time.sleep(0.5)
                print("\nStopped motors.")
                break
        
        ser.close()
    except PermissionError:
        print(f"❌ Access Denied on {port}")
        print("Close Arduino IDE Serial Monitor and try again")
    except Exception as e:
        print(f"❌ Error: {e}")


def test_with_rover_autonomy(port, dry_run=False):
    """Test with actual rover autonomy code."""
    try:
        from rover_autonomy import MotorController, DecisionEngine, Detection
    except ImportError:
        print("❌ rover_autonomy module not found")
        return
    
    print(f"\n🤖 Testing with DecisionEngine")
    print(f"   Mode: {'DRY-RUN' if dry_run else 'HARDWARE'}")
    print(f"   Port: {port}\n")
    
    # Initialize
    try:
        motor = MotorController(port=port, dry_run=dry_run, baudrate=9600)
    except PermissionError:
        print(f"❌ Cannot connect to {port} - already in use")
        print("Close Arduino IDE Serial Monitor and try again")
        return
    except Exception as e:
        print(f"❌ Connection error: {e}")
        return
    
    engine = DecisionEngine(640, 480)
    
    try:
        # Simulate a few decisions
        print("Frame 1-5: Clear path")
        for i in range(5):
            decision = engine.decide([])
            motor_action = decision.action.lower()
            
            if motor_action == "forward":
                motor.forward(0.5)
            elif motor_action == "stop":
                motor.stop()
            elif motor_action == "steer_left":
                motor.steer_left(0.4)
            elif motor_action == "steer_right":
                motor.steer_right(0.4)
            
            time.sleep(0.2)
        
        print("\nFrame 6-10: Obstacle on left (should steer right)")
        det_left = Detection("person", 0.9, 100, 200, 300, 400)
        for i in range(5):
            decision = engine.decide([det_left])
            motor_action = decision.action.lower()
            print(f"  Decision: {decision.action} ({decision.reason})")
            
            if motor_action == "forward":
                motor.forward(0.5)
            elif motor_action == "stop":
                motor.stop()
            elif motor_action == "steer_left":
                motor.steer_left(0.4)
            elif motor_action == "steer_right":
                motor.steer_right(0.4)
            
            time.sleep(0.2)
        
        motor.stop()
        motor.close()
        print("\n✅ Test complete")
    except KeyboardInterrupt:
        motor.stop()
        motor.close()
        print("\nInterrupted by user")


# ──────────────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rover Hardware Testing")
    parser.add_argument("--list", action="store_true", help="List serial ports")
    parser.add_argument("--test", metavar="PORT", help="Test connection (e.g., COM3)")
    parser.add_argument("--motor", metavar="PORT", help="Interactive motor test")
    parser.add_argument("--sim", metavar="PORT", help="Simulate with rover autonomy (dry-run)")
    parser.add_argument("--run", metavar="PORT", help="Run with actual hardware")
    
    args = parser.parse_args()
    
    if args.list:
        list_ports()
    elif args.test:
        test_connection(args.test)
    elif args.motor:
        interactive_motor_test(args.motor)
    elif args.sim:
        test_with_rover_autonomy(args.sim, dry_run=True)
    elif args.run:
        test_with_rover_autonomy(args.run, dry_run=False)
    else:
        print("""
Rover Hardware Testing Script
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

QUICK START (on Windows laptop):

1. List your ports:
   python hardware_test.py --list
   
2. Find your Arduino port (e.g., COM3)

3. Check connection:
   python hardware_test.py --test COM3
   
4. Test motors interactively:
   python hardware_test.py --motor COM3
   
5. Simulate rover logic (dry-run):
   python hardware_test.py --sim COM3
   
6. Run with actual motors:
   python hardware_test.py --run COM3

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        """)
