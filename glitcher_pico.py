"""
Flash onto Pico via Thonny (save as main.py).

Pinout:
  GP0  -> nRF nRESET
  GP1  -> 47ohm resistor -> nRF VDD   (glitch)
  GP2  -> nRF SWCLK
  GP3  -> nRF SWDIO
  GND  -> nRF GND
  3V3  -> D1 -> D2 -> nRF VDD         (power via 2x 1N4148)
"""

from machine import Pin
import utime

RESET = Pin(0, Pin.OPEN_DRAIN, value=1)
GLITCH = Pin(2, Pin.OUT, value=1)
SWCLK = Pin(4, Pin.OUT, value=0)
SWDIO_PIN = 3

NRF52_DPIDR = 0x2BA01477


# ── SWD bitbang ───────────────────────────────────────────────────────────────

def _swdio_out():
    return Pin(SWDIO_PIN, Pin.OUT)

def _swdio_in():
    return Pin(SWDIO_PIN, Pin.IN)

def _write_bits(swdio, val, n):
    for _ in range(n):
        swdio.value(val & 1)
        SWCLK.value(1)
        SWCLK.value(0)
        val >>= 1

def _read_bits(swdio, n):
    val = 0
    for i in range(n):
        SWCLK.value(1)
        val |= swdio.value() << i
        SWCLK.value(0)
    return val

def swd_check():
    swdio = _swdio_out()

    # line reset: 50 clocks with SWDIO high
    _write_bits(swdio, 0xFFFFFFFF, 32)
    _write_bits(swdio, 0x0003FFFF, 18)

    # send IDCODE read request: 0xA5
    _write_bits(swdio, 0xA5, 8)

    # turnaround
    swdio = _swdio_in()
    _read_bits(swdio, 1)

    # read 3-bit ACK
    ack = _read_bits(swdio, 3)

    if ack != 0b001:
        swdio = _swdio_out()
        _write_bits(swdio, 0, 8)
        return False

    # read 32-bit DPIDR + 1 parity
    data = _read_bits(swdio, 32)
    _read_bits(swdio, 1)

    swdio = _swdio_out()
    _write_bits(swdio, 0, 8)

    return data == NRF52_DPIDR


# ── glitch ────────────────────────────────────────────────────────────────────

def do_glitch(offset_us, width_us):
    RESET.value(0)
    utime.sleep_us(500)
    RESET.value(1)
    utime.sleep_us(offset_us)
    GLITCH.value(0)
    utime.sleep_us(width_us)
    GLITCH.value(1)


# ── main loop ─────────────────────────────────────────────────────────────────

while True:
    try:
        line = input().strip()
        if not line:
            continue
        parts = line.split()
        cmd = parts[0]

        if cmd == "G" and len(parts) == 3:
            do_glitch(int(parts[1]), int(parts[2]))
            print("OK")

        elif cmd == "CHECK":
            print("OPEN" if swd_check() else "LOCKED")

        elif cmd == "RESET":
            RESET.value(0)
            utime.sleep_ms(100)
            RESET.value(1)
            print("OK")

        else:
            print("ERR unknown")

    except Exception as e:
        print(f"ERR {e}")
