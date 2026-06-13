"""
Triggers DFU mode on the Triki device.
Writes 0x01 to the control characteristic (6e400004) which reboots
the device as TrikiDFU, ready to accept a firmware package.

Usage:
    python dfu_trigger.py                         # auto-scan by name
    python dfu_trigger.py --addr CC:F7:42:8C:D0:F8
"""

import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

NUS_RX       = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
DFU_CTRL     = "6e400004-b5a3-f393-e0a9-e50e24dcca9e"  # confirmed DFU trigger
START_CMD    = bytes.fromhex("201000D007680003")


async def find_triki():
    print("Scanning for Triki...")
    devices = await BleakScanner.discover(timeout=8)
    for d in devices:
        name = d.name or ""
        if "Triki" in name or "triki" in name:
            print(f"Found: {d.name}  {d.address}")
            return d.address
    print("No Triki found. Use --addr to specify manually.")
    sys.exit(1)


async def trigger(addr: str):
    print(f"Connecting to {addr}...")
    async with BleakClient(addr, timeout=10) as client:
        print("Connected.")

        try:
            await client.write_gatt_char(NUS_RX, START_CMD, response=False)
            print("Start command sent.")
        except Exception as e:
            print(f"  (NUS write skipped: {e})")

        await asyncio.sleep(0.3)

        print("Sending DFU trigger (0x01 → 6e400004)...")
        await client.write_gatt_char(DFU_CTRL, b"\x01", response=False)
        print("Done — device should reboot as TrikiDFU in ~2 seconds.")
        await asyncio.sleep(2)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", help="BLE address of Triki (skip scan)")
    args = ap.parse_args()

    addr = args.addr or await find_triki()
    await trigger(addr)


if __name__ == "__main__":
    asyncio.run(main())
