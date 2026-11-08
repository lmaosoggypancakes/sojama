"""Live spectrum plot and tone test for the STM32 firmware (stream_mode = 2).

The firmware sends "FFT,<mic>,<v0>,...,<v255>" lines, alternating mic 0 and mic 1.
Bin k is k * 93.75 Hz (512-point FFT at 48 kHz, Hann window); each value is the
bin level in -dBFS, so 6 means -6 dBFS.

Usage:
    python fft_plot.py /dev/ttyACM0                 live plot, prints each mic's loudest frequency
    python fft_plot.py /dev/ttyACM0 --expect 1000   tone test: play a 1 kHz tone into the mics;
                                                    reports PASS/FAIL per mic after 20 spectra each

Needs pyserial and matplotlib.
"""
import argparse
import sys

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

FS = 48000
N_FFT = 512
BINS = N_FFT // 2
BIN_HZ = FS / N_FFT
FREQS = [k * BIN_HZ for k in range(BINS)]
SKIP_BINS = 2            # ignore DC and the lowest bin when finding the loudest frequency
TEST_SPECTRA = 20        # spectra per mic before the tone test decides

parser = argparse.ArgumentParser()
parser.add_argument("port", nargs="?", default="/dev/ttyACM0")
parser.add_argument("--expect", type=float, metavar="HZ",
                    help="tone test: expected tone frequency in Hz")
parser.add_argument("--min-level", type=float, default=-60.0, metavar="DBFS",
                    help="tone test: the tone must be at least this loud (default -60)")
args = parser.parse_args()


def parse(line):
    """Return (mic, [dBFS per bin]) for a valid FFT line, else None."""
    parts = line.split(",")
    if len(parts) != BINS + 2 or parts[0] != "FFT" or parts[1] not in ("0", "1"):
        return None
    try:
        return int(parts[1]), [-int(v) for v in parts[2:]]
    except ValueError:
        return None


def loudest(db):
    k = max(range(SKIP_BINS, BINS), key=lambda i: db[i])
    return k, FREQS[k], db[k]


ser = serial.Serial(args.port, 115200, timeout=0)
pending = b""
counts = [0, 0]
hits = [0, 0]
results = [None, None]

fig, ax = plt.subplots()
lines = [ax.plot(FREQS, [-140] * BINS, label=f"Mic {i + 1}")[0] for i in range(2)]
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Level (dBFS)")
ax.set_xlim(0, FS / 2)
ax.set_ylim(-140, 0)
ax.grid(True)
ax.legend(loc="upper right")
title = f"Spectrum from {args.port}"
if args.expect:
    ax.axvline(args.expect, color="gray", linestyle="--", linewidth=1)
    title += f" (expecting {args.expect:.0f} Hz)"
ax.set_title(title)


def update(_frame):
    global pending
    pending += ser.read(ser.in_waiting or 1)
    *complete, pending = pending.split(b"\n")
    for raw in complete:
        text = raw.decode(errors="replace").strip()
        parsed = parse(text)
        if parsed is None:
            if text and not text.startswith("FFT"):
                print(text)              # anything else the board sends
            continue
        mic, db = parsed
        lines[mic].set_ydata(db)
        counts[mic] += 1
        k, freq, level = loudest(db)
        print(f"mic{mic + 1}: loudest {freq:7.1f} Hz (bin {k:3d}) at {level:4d} dBFS")

        if args.expect and results[mic] is None:
            ok = abs(freq - args.expect) <= BIN_HZ and level >= args.min_level
            hits[mic] += ok
            if counts[mic] == TEST_SPECTRA:
                results[mic] = hits[mic] >= TEST_SPECTRA // 2
                verdict = "PASS" if results[mic] else "FAIL"
                print(f"*** mic{mic + 1} {verdict}: {hits[mic]}/{TEST_SPECTRA} spectra had the "
                      f"loudest peak within {BIN_HZ:.1f} Hz of {args.expect:.0f} Hz "
                      f"at >= {args.min_level:.0f} dBFS")
    return lines


anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
try:
    plt.show()
finally:
    ser.close()
    if args.expect:
        if None in results:
            print(f"Tone test incomplete: spectra received per mic {counts}")
        sys.exit(0 if all(results) else 1)
