"""UART link test for the STM32 firmware (uart_comm.c).

Prints everything the board sends, and every 2 s sends "hello <n>" and expects
"echo: hello <n>" back.

Usage:
    pip install pyserial
    python uart_test.py /dev/ttyACM0      (Linux / WSL)
    python uart_test.py COM5              (Windows)
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

with serial.Serial(PORT, 115200, timeout=0.1) as ser:
    print(f"Listening on {PORT} at 115200 8-N-1. Ctrl+C to stop.")
    next_cmd = time.monotonic() + 2.0
    n = 0
    sent, echoed = 0, 0
    try:
        while True:
            line = ser.readline().decode(errors="replace").rstrip("\r\n")
            if line:
                print(line)
                if line == f"echo: hello {n}":
                    echoed += 1
            if time.monotonic() >= next_cmd:
                n += 1
                ser.write(f"hello {n}\n".encode())
                print(f">>> sent: hello {n}")
                sent += 1
                next_cmd += 2.0
    except KeyboardInterrupt:
        print(f"\nCommands sent: {sent}, echoes received: {echoed}")
