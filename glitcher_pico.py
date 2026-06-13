"""
Flash this onto the Pico via Thonny or mpremote.
Wiring:
  GP0 -> nRF nRESET
  GP1 -> 47ohm resistor -> nRF VDD
  GND -> nRF GND
  3V3 -> D1 -> D2 -> nRF VDD   (the two 1N4148 diodes)
"""

from machine import Pin
import utime
import sys

RESET = Pin(0, Pin.OPEN_DRAIN, value=1)  # high = released, low = reset
GLITCH = Pin(1, Pin.OUT, value=1)        # low = pulling VDD down via resistor


def do_glitch(offset_us: int, width_us: int) -> None:
    RESET.value(0)
    utime.sleep_us(500)
    RESET.value(1)
    utime.sleep_us(offset_us)
    GLITCH.value(0)
    utime.sleep_us(width_us)
    GLITCH.value(1)


while True:
    try:
        line = input().strip()
        if not line:
            continue
        parts = line.split()
        if parts[0] == "G" and len(parts) == 3:
            do_glitch(int(parts[1]), int(parts[2]))
            print("OK")
        elif parts[0] == "RESET":
            RESET.value(0)
            utime.sleep_ms(100)
            RESET.value(1)
            print("OK")
        else:
            print("ERR unknown command")
    except Exception as e:
        print(f"ERR {e}")
