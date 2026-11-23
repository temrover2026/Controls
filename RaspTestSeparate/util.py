# util.py - small helpers shared by the calibration and recording code:
# folders and free space on the Pico's flash, CSV writing, the LED, countdowns and loop timing.

import os
import time

from machine import Pin

import config

# The Pico H onboard LED (GP25)
led = Pin(config.LED_PIN, Pin.OUT)


# ----------------------------------------------------------------------------- files and space
def ensure_dir(path):
    """Create a folder on the flash drive if it does not exist (MicroPython has no os.makedirs)."""
    try:
        os.mkdir(path)
    except OSError:
        pass                                    # Already there


def exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def free_bytes(path="/"):
    """Free space on the flash filesystem in bytes."""
    st = os.statvfs(path)
    return st[0] * st[3]                        # block size x free blocks


def next_numbered_file(folder, prefix, ext=".csv"):
    """Next unused name like /imu_data/imu_log_007.csv.

    The Pico has no real-time clock that survives power-off, so files are numbered rather than
    named by date (Controls uses the date and time).
    """
    ensure_dir(folder)
    used = [0]
    for name in os.listdir(folder):
        if name.startswith(prefix + "_") and name.endswith(ext):
            try:
                used.append(int(name[len(prefix) + 1:-len(ext)]))
            except ValueError:
                pass
    return "%s/%s_%03d%s" % (folder, prefix, max(used) + 1, ext)


def fmt(value):
    """One number for a CSV cell. 7 significant digits is all a 32-bit float holds."""
    return "{:.7g}".format(value)


def csv_line(values):
    return ",".join(v if isinstance(v, str) else fmt(v) for v in values) + "\n"


def read_csv_row(path):
    """First data row of a header + values CSV, as a {column: text} dict (like csv.DictReader)."""
    with open(path) as f:
        header = f.readline().strip().split(",")
        values = f.readline().strip().split(",")
    if len(values) < len(header):
        raise ValueError("%s must contain a header and one row of values" % path)
    return dict(zip(header, values))


def board_name():
    """This board's name from /board_name.txt (e.g. picoTX or picoRX); "pico" if not set.

    Kept in its own file, not in config.py, so copying new code to a board never changes its name.
    Set it from the laptop:  mpremote exec "open('/board_name.txt','w').write('picoTX')"
    """
    try:
        with open("/board_name.txt") as f:
            return f.read().strip() or "pico"
    except OSError:
        return "pico"


# ----------------------------------------------------------------------------- timing and LED
def countdown(seconds, message):
    """Print a countdown with the LED on, so you know when to start (keep still / start rotating)."""
    led.on()
    for s in range(seconds, 0, -1):
        print("  %s in %d s ..." % (message, s))
        time.sleep(1)
    led.off()
    print("  >>> GO")


class Rate:
    """Runs a loop at a fixed rate:  r = Rate(25); while ...: r.sleep()

    Uses ticks_us arithmetic, which is wrap-around safe on MicroPython.
    """

    def __init__(self, hz):
        self.period_us = int(1000000 / hz)
        self.next = time.ticks_add(time.ticks_us(), self.period_us)

    def sleep(self):
        wait = time.ticks_diff(self.next, time.ticks_us())
        if wait > 0:
            time.sleep_us(wait)
            self.next = time.ticks_add(self.next, self.period_us)
        else:
            # Running late (slow I2C read): restart the schedule instead of trying to catch up
            self.next = time.ticks_add(time.ticks_us(), self.period_us)


def seconds_since(start_us):
    return time.ticks_diff(time.ticks_us(), start_us) / 1000000
