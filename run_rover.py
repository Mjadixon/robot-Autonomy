#!/usr/bin/env python3
"""
Rover Autonomy Launcher
Connects to Arduino and runs full vision + motor system
"""

import sys
import time
import argparse
from rover_autonomy import MotorController


def main():
    parser = argparse.ArgumentParser(
        description="Rover Autonomy System with Arduino Motor Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
USAGE EXAMPLES:

  Dry-run mode (no motors):
    python run_rover.py --mode live --dry-run
    
  Live with motors on COM11:
    python run_rover.py --mode live --port COM11
    
  Benchmark performance:
    python run_rover.py --mode benchmark
    
  List available cameras:
    python run_rover.py --mode list-cameras
        """
    )
    
    parser.add_argument("--mode", choices=["test", "live", "benchmark", "list-cameras"],
                        default="live", help="Run mode (default: live)")
    parser.add_argument("--port", type=str, default=None,
                        help="Arduino COM port (e.g., COM11). If not specified, will prompt.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Don't send to motors, just log commands")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index (default: 0)")
    parser.add_argument("--headless", action="store_true",
                        help="Live mode without display window")
    
    args = parser.parse_args()
    
    # Initialize motor controller if not in benchmark/list mode
    motor = None
    if args.mode not in ["benchmark", "list-cameras"]:
        motor = init_motor_controller(args.port, args.dry_run)
        if motor is None and not args.dry_run:
            return
    
    # Set global motor in rover_autonomy
    import rover_autonomy
    rover_autonomy.motor = motor
    
    # Run selected mode
    from rover_autonomy import run_test_mode, run_live_mode, run_benchmark_mode, CameraManager
    
    try:
        if args.mode == "test":
            print("\n[LAUNCHER] Starting TEST mode...")
            run_test_mode()
        elif args.mode == "live":
            print("\n[LAUNCHER] Starting LIVE mode with vision...")
            print(f"[LAUNCHER] Motors: {'DRY-RUN' if args.dry_run else 'ACTIVE'}")
            run_live_mode(headless=args.headless)
        elif args.mode == "benchmark":
            print("\n[LAUNCHER] Starting BENCHMARK mode...")
            run_benchmark_mode()
        elif args.mode == "list-cameras":
            print("\n[LAUNCHER] Listing available cameras...")
            cam_mgr = CameraManager()
            cam_mgr.print_available()
    except KeyboardInterrupt:
        print("\n[LAUNCHER] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        if motor:
            motor.stop()
            motor.close()
            print("[LAUNCHER] Motors stopped and port closed")


def init_motor_controller(port, dry_run):
    """Initialize motor controller with port selection."""
    
    if dry_run:
        print("\n[MOTOR] Dry-run mode enabled — motors won't spin")
        motor = MotorController(port=None, dry_run=True)
        return motor
    
    # Try provided port
    if port:
        print(f"\n[MOTOR] Connecting to {port}...")
        motor = MotorController(port=port, dry_run=False)
        if motor._connected:
            return motor
        else:
            print(f"[ERROR] Could not connect to {port}")
            return None
    
    # Auto-detect port
    print("\n[MOTOR] No port specified, searching for Arduino...")
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            print("[ERROR] No serial ports found!")
            print("        Check Arduino USB cable and connections")
            return None
        
        # Filter for Arduino devices
        arduino_ports = [
            p for p in ports 
            if "Arduino" in p.description or "CH340" in p.description
        ]
        
        if not arduino_ports:
            print("[WARN] No Arduino detected, showing all ports:")
            for i, p in enumerate(ports, 1):
                print(f"      [{i}] {p.device:8s} - {p.description}")
            arduino_ports = ports
        
        if len(arduino_ports) == 1:
            port = arduino_ports[0].device
            print(f"[MOTOR] Auto-detected: {port}")
        else:
            print("\n[MOTOR] Multiple ports available:")
            for i, p in enumerate(arduino_ports, 1):
                marker = " ← Likely Arduino" if "Arduino" in p.description else ""
                print(f"      [{i}] {p.device:8s} - {p.description}{marker}")
            
            try:
                choice = input(f"\nSelect port [1-{len(arduino_ports)}]: ").strip()
                idx = int(choice) - 1
                if 0 <= idx < len(arduino_ports):
                    port = arduino_ports[idx].device
                else:
                    print("[ERROR] Invalid selection")
                    return None
            except ValueError:
                print("[ERROR] Invalid input")
                return None
        
        print(f"[MOTOR] Connecting to {port}...")
        motor = MotorController(port=port, dry_run=False)
        
        if motor._connected:
            print(f"[MOTOR] ✅ Connected successfully")
            return motor
        else:
            print(f"[MOTOR] ❌ Connection failed")
            print("        - Close Arduino IDE Serial Monitor")
            print("        - Unplug/replug USB cable")
            print("        - Try: python fix_port_access.py")
            return None
    
    except ImportError:
        print("[ERROR] pyserial not installed: pip install pyserial")
        return None
    except Exception as e:
        print(f"[ERROR] {e}")
        return None


if __name__ == "__main__":
    main()
