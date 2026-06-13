"""
Triggers Nordic Buttonless DFU on the Triki device.
Sends the normal start command first to keep it alive, then writes
to the Buttonless DFU characteristic → device reboots as TrikiDFU.

Usage:
    python dfu_trigger.py                         # auto-scan by name
    python dfu_trigger.py --addr CC:F7:42:8C:D0:F8
"""

import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

# Nordic UART Service
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

# Nordic Buttonless DFU (no-bonds variant) — most common in nRF5 SDK
BUTTONLESS_DFU_UUID = "8ec90003-f315-4f60-9fb8-838830daea50"

# Normal Triki start command (keeps connection alive)
START_CMD = bytes.fromhex("201000D007680003")


async def find_triki():
    print("Scanning for Triki...")
    devices = await BleakScanner.discover(timeout=8)
    for d in devices:
        name = d.name or ""
        if "Triki" in name or "triki" in name:
            print(f"Found: {d.name}  {d.address}")
            return d.address
    print("No Triki found. Run with --addr to specify manually.")
    sys.exit(1)


async def trigger(addr: str):
    print(f"Connecting to {addr}...")
    async with BleakClient(addr, timeout=10) as client:
        print("Connected.")

        # Send start command over NUS so it doesn't drop the connection
        try:
            await client.write_gatt_char(NUS_RX, START_CMD, response=False)
            print("Start command sent.")
        except Exception as e:
            print(f"  (NUS write failed: {e} — continuing anyway)")

        await asyncio.sleep(0.5)

        # Try buttonless DFU characteristic
        services = client.services
        dfu_char = None
        for service in services:
            for char in service.characteristics:
                if char.uuid.lower() == BUTTONLESS_DFU_UUID:
                    dfu_char = char
                    break

        print("\nAll characteristics on device:")
        for service in services:
            print(f"  Service: {service.uuid}")
            for char in service.characteristics:
                print(f"    Char:  {char.uuid}  props={char.properties}")

        if dfu_char is None:
            print(f"\nButtonless DFU char ({BUTTONLESS_DFU_UUID}) not found.")
            print("Check the list above — look for any char with 'write' in a DFU service.")
            return False

        print(f"\nFound Buttonless DFU char. Triggering...")
        await client.write_gatt_char(dfu_char.uuid, b"\x01", response=True)
        print("DFU trigger sent! Device should reboot as TrikiDFU in ~2 seconds.")
        await asyncio.sleep(3)
        return True


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--addr", help="BLE address of Triki")
    args = ap.parse_args()

    addr = args.addr or await find_triki()
    await trigger(addr)


if __name__ == "__main__":
    asyncio.run(main())
