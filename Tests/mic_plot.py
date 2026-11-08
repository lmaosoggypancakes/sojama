"""Live mic level plot for the STM32 firmware.

The firmware sends "MIC,<peak1>,<peak2>" every 50 ms, where each peak is the
largest sample deviation in that period (0..8388608 = full scale). This prints
each reading and plots both mics in dBFS over the last 10 seconds.

Usage:
    python mic_plot.py /dev/ttyACM0      (Linux / WSL)
    python mic_plot.py COM5              (Windows)

Needs pyserial and matplotlib.
"""
import math
import sys
from collections import deque

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
FULL_SCALE = 8388608
PERIOD_S = 0.05            # one reading every 50 ms
HISTORY = 200              # 10 s of readings
FLOOR_DB = -100.0


def to_dbfs(peak):
    return 20 * math.log10(peak / FULL_SCALE) if peak > 0 else FLOOR_DB


ser = serial.Serial(PORT, 115200, timeout=0)
times = deque(maxlen=HISTORY)
levels = [deque(maxlen=HISTORY), deque(maxlen=HISTORY)]
pending = b""
count = 0

fig, ax = plt.subplots()
lines = [ax.plot([], [], label=f"Mic {i + 1}")[0] for i in range(2)]
ax.set_xlabel("Time (s)")
ax.set_ylabel("Peak level (dBFS)")
ax.set_ylim(FLOOR_DB, 0)
ax.grid(True)
ax.legend(loc="upper left")
ax.set_title(f"Mic levels from {PORT}")


def update(_frame):
    global pending, count
    pending += ser.read(ser.in_waiting or 1)
    *complete, pending = pending.split(b"\n")
    for raw in complete:
        text = raw.decode(errors="replace").strip()
        parts = text.split(",")
        if len(parts) != 3 or parts[0] != "MIC":
            if text:
                print(text)          # anything else the board sends
            continue
        try:
            peaks = [int(parts[1]), int(parts[2])]
        except ValueError:
            continue
        db = [to_dbfs(p) for p in peaks]
        times.append(count * PERIOD_S)
        for ch in range(2):
            levels[ch].append(db[ch])
        count += 1
        print(f"mic1 {peaks[0]:8d} ({db[0]:6.1f} dBFS)   mic2 {peaks[1]:8d} ({db[1]:6.1f} dBFS)")

    if times:
        for ch in range(2):
            lines[ch].set_data(times, levels[ch])
        ax.set_xlim(max(0.0, times[-1] - HISTORY * PERIOD_S), max(times[-1], HISTORY * PERIOD_S))
    return lines


anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
try:
    plt.show()
finally:
    ser.close()
