"""
Connects to TrikiDumper over BLE, sends a trigger byte,
then collects the raw flash stream between START and DONE markers.

Usage:
    pip install bleak
    python dump_receiver.py                         # auto-scan
    python dump_receiver.py --addr AA:BB:CC:DD:EE:FF
"""

import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

NUS_SERVICE  = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX       = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notifications  (device → host)
NUS_RX       = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write          (host → device)

FLASH_SIZE   = 192 * 1024
OUT_FILE     = "flash_dump.bin"


async def find_dumper():
    print("Scanning for TrikiDumper...")
    dev = await BleakScanner.find_device_by_name("TrikiDumper", timeout=10)
    if dev is None:
        print("Not found. Is the firmware running and advertising?")
        sys.exit(1)
    return dev.address


async def dump(addr: str):
    buf = bytearray()
    capturing = False
    done_event = asyncio.Event()

    def on_notify(_, data: bytearray):
        nonlocal capturing

        if data == b"START\n" or data.startswith(b"START"):
            print("  <- START received, collecting flash...")
            capturing = True
            return

        if data == b"DONE\n" or data.startswith(b"DONE"):
            print(f"  <- DONE received ({len(buf):,} bytes)")
            capturing = False
            done_event.set()
            return

        if capturing:
            buf.extend(data)
            pct = 100 * len(buf) / FLASH_SIZE
            print(f"\r  {len(buf):,} / {FLASH_SIZE:,} bytes  ({pct:.1f}%)", end="", flush=True)

    print(f"Connecting to {addr}...")
    async with BleakClient(addr) as client:
        await client.start_notify(NUS_TX, on_notify)
        print("Connected. Sending trigger...")
        await client.write_gatt_char(NUS_RX, b"\x01", response=False)
        await asyncio.wait_for(done_event.wait(), timeout=120)
        await client.stop_notify(NUS_TX)

    print()

    if len(buf) != FLASH_SIZE:
        print(f"WARNING: expected {FLASH_SIZE} bytes, got {len(buf)}. Saving anyway.")

    with open(OUT_FILE, "wb") as f:
        f.write(buf)
    print(f"Saved -> {OUT_FILE}")
    return buf


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", help="BLE address of TrikiDumper (skip scan)")
    args = ap.parse_args()

    addr = args.addr or await find_dumper()
    flash = await dump(addr)

    # Quick sanity: show first 64 bytes (MBR / SoftDevice header)
    print("\nFirst 64 bytes of dump:")
    print(" ".join(f"{b:02x}" for b in flash[:64]))

    # Look for AES key candidate near bootloader area (last ~32 KB)
    print("\nLast 64 bytes of dump (bootloader tail):")
    print(" ".join(f"{b:02x}" for b in flash[-64:]))


if __name__ == "__main__":
    asyncio.run(main())
