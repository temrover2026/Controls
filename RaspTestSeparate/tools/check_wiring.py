#!/usr/bin/env python3
"""Laptop tool: live IMU wiring monitor for the Pico H (big green / red window).

    python3 tools/check_wiring.py            # runs until the window is closed (or --seconds)

Twice a second the Pico checks the I2C lines and the IMU and reports:
  CONNECTED       IMU answers at 0x68 and gives sensible readings
  NO POWER        SDA/SCL held at 0 V: the wires reach the Pico but the IMU board has no 3.3 V
  NOT CONNECTED   SDA/SCL float: the IMU board is unpowered, or SDA/SCL do not reach GP2/GP3
                  (the bus is only scanned when the lines are pulled up, so noise cannot fake a device)
  WRONG ADDRESS / ERRORS  something answers but not properly
It also counts drop-outs, so wiggling each wire in turn shows which one is loose.
"""

import argparse
import os
import queue
import shutil
import subprocess
import threading
import time

import matplotlib.pyplot as plt

PICO_CODE = """
import time, config
from machine import Pin, I2C
def lines():
    sda_up = Pin(config.I2C_SDA_PIN, Pin.IN, Pin.PULL_UP).value()
    scl_up = Pin(config.I2C_SCL_PIN, Pin.IN, Pin.PULL_UP).value()
    sda_dn = Pin(config.I2C_SDA_PIN, Pin.IN, Pin.PULL_DOWN).value()
    scl_dn = Pin(config.I2C_SCL_PIN, Pin.IN, Pin.PULL_DOWN).value()
    if sda_dn and scl_dn: return 'pulled_up'
    if not sda_up or not scl_up: return 'held_low'
    return 'floating'
end = time.ticks_add(time.ticks_ms(), {ms})
while time.ticks_diff(end, time.ticks_ms()) > 0:
    state = lines()
    who, found = None, []
    if state == 'pulled_up':
        # Only talk to the bus when the IMU's pull-ups are powered: on floating lines a scan takes
        # seconds to time out and "finds" random addresses from noise
        i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA_PIN), scl=Pin(config.I2C_SCL_PIN), freq=100000)
        try:
            who = i2c.readfrom_mem(config.IMU_ADDR, 0, 1)[0]
        except OSError:
            pass
        found = i2c.scan()
    print('S,%s,%s,%s' % (state, who if who is not None else -1, ' '.join(hex(a) for a in found)))
    time.sleep_ms(400)
"""


def classify(state, who, found):
    if who == 0xEA:
        return "CONNECTED", "tab:green", "IMU answers at 0x68 (WHO_AM_I 0xEA)"
    if found:
        return "WRONG DEVICE / ADDRESS", "tab:orange", "found " + " ".join(found) + " but no ICM-20948 at 0x68"
    if state == "held_low":
        return "NO POWER TO IMU", "tab:red", ("SDA/SCL held at 0 V: wires reach GP2/GP3 but the IMU board\n"
                                             "has no 3.3 V. Check IMU 3-6V -> pin 36 and IMU GND.")
    if state == "floating":
        return "NOT CONNECTED", "tab:red", ("SDA/SCL float: nothing on GP2 (pin 4) / GP3 (pin 5).\n"
                                           "Check the SDA and SCL wires and that the IMU is powered.")
    return "NO ANSWER", "tab:red", "lines pulled up but nothing answers at 0x68: SDA/SCL swapped?"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=int, default=600)
    args = parser.parse_args()
    exe = shutil.which("mpremote") or os.path.expanduser("~/.local/bin/mpremote")
    process = subprocess.Popen([exe, "connect", "auto", "exec", PICO_CODE.format(ms=args.seconds * 1000)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    lines = queue.Queue()

    def reader():
        for line in process.stdout:
            lines.put(line.strip())
        lines.put(None)
    threading.Thread(target=reader, daemon=True).start()

    fig = plt.figure(figsize=(10, 5))
    plt.ion()
    fig.show()
    status, colour, detail = "waiting for the Pico ...", "gray", ""
    good = bad = drops = 0
    last_ok = None
    history = []
    running = True
    while running and plt.fignum_exists(fig.number):
        while True:
            try:
                line = lines.get_nowait()
            except queue.Empty:
                break
            if line is None:
                running = False
                break
            if not line.startswith("S,"):
                continue
            _, state, who, found = (line.split(",") + [""])[:4]
            status, colour, detail = classify(state, int(who), found.split())
            ok = status == "CONNECTED"
            good, bad = good + ok, bad + (not ok)
            if last_ok and not ok:
                drops += 1
            last_ok = ok
            history = (history + [ok])[-60:]
        fig.clear()
        fig.patch.set_facecolor(colour if colour != "gray" else "white")
        fig.text(0.5, 0.68, status, ha="center", va="center", fontsize=40, fontweight="bold", color="white"
                 if colour != "gray" else "black")
        fig.text(0.5, 0.42, detail, ha="center", va="center", fontsize=13, color="white" if colour != "gray" else "black")
        fig.text(0.5, 0.18, "checks: %d connected, %d not  |  drop-outs (was connected, then lost): %d\n"
                 "last 30 s: %s" % (good, bad, drops, "".join("|" if h else "." for h in history)),
                 ha="center", va="center", fontsize=11, family="monospace",
                 color="white" if colour != "gray" else "black")
        fig.canvas.draw_idle()
        fig.canvas.start_event_loop(0.25)
    process.terminate()


if __name__ == "__main__":
    main()
