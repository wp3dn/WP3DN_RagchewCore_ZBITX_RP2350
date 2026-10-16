#!/usr/bin/env python3
import sys, time
try:
    import serial
except ImportError:
    raise SystemExit("Install pyserial: python3 -m pip install pyserial")
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
with serial.Serial(port, 115200, timeout=1) as s:
    time.sleep(0.2)
    cmd=f"TIME {time.time():.3f}\n"
    s.write(cmd.encode())
    time.sleep(0.1)
    print(cmd.strip())
    print(s.read(s.in_waiting or 1).decode(errors="replace"), end="")
