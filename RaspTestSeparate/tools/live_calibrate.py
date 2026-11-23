#!/usr/bin/env python3
"""Laptop tool: run the Pico H gyro + magnetometer calibration and watch it live.

Runs on the laptop, with the Pico plugged in by USB and the RaspTestSeparate code already on it.

    python3 tools/live_calibrate.py                 # gyro (30 s still) then magnetometer (60 s rotating)
    python3 tools/live_calibrate.py --only mag

The Pico runs calibration.record_gyro_bias() and record_mag_calibration() with stream=True, so
every sample also comes over USB. This window shows them as they arrive, with a big countdown
telling you when to keep still and when to rotate, and keeps live_calibration.png up to date.
At the end the calibration files are copied back and plot_pico_results.py makes the final images
(gyro_calibration.png, mag_calibration.png), which are opened on the screen.
Everything is saved in ~/imu_data/pico_<date_time>/.
"""

import argparse
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_pico_results   # noqa: E402

# Runs on the Pico (same start-up as main.py, without the menu)
PICO_CODE = """
import time, config, calibration
from machine import I2C, Pin
from icm20948 import ICM20948
if config.IMU_GROUND_PIN is not None:
    Pin(config.IMU_GROUND_PIN, Pin.OUT, value=0); time.sleep_ms(50)
i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA_PIN), scl=Pin(config.I2C_SCL_PIN), freq=config.I2C_FREQ)
imu = ICM20948(i2c, config.IMU_ADDR, config.MAG_MODE)
print('PHASE,gyro,%d' % config.GYRO_CAL_SECONDS) if {gyro} else None
calibration.record_gyro_bias(imu, stream=True) if {gyro} else None
config.COUNTDOWN_SECONDS = {mag_countdown}
print('PHASE,mag,%d' % config.MAG_CAL_SECONDS) if {mag} else None
calibration.record_mag_calibration(imu, stream=True) if {mag} else None
print('PHASE,done,0')
"""


def reader(process, lines):
    """Background thread: every line the Pico prints goes into the queue."""
    for line in process.stdout:
        lines.put(line.rstrip("\n"))
    lines.put(None)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--only", choices=("gyro", "mag"))
    parser.add_argument("--mag-countdown", type=int, default=10, help="seconds to get ready before rotating")
    parser.add_argument("--folder", default=None, help="default ~/imu_data/<board>_<date_time>")
    args = parser.parse_args()
    if args.folder is None:
        args.folder = os.path.expanduser("~/imu_data/%s_%s" % (plot_pico_results.board_name(),
                                                               datetime.now().strftime("%Y%m%d_%H%M%S")))
    os.makedirs(args.folder, exist_ok=True)
    live_png = os.path.join(args.folder, "live_calibration.png")

    code = PICO_CODE.format(gyro=args.only in (None, "gyro"), mag=args.only in (None, "mag"),
                            mag_countdown=args.mag_countdown)
    exe = shutil.which("mpremote") or os.path.expanduser("~/.local/bin/mpremote")
    process = subprocess.Popen([exe, "connect", "auto", "exec", code], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1,
                               env=dict(os.environ, PYTHONUNBUFFERED="1"))
    lines = queue.Queue()
    threading.Thread(target=reader, args=(process, lines), daemon=True).start()

    # Window layout: gyro rates on top, magnetometer 3 views in the middle, field strength + log below
    fig = plt.figure(figsize=(14, 10))
    grid = fig.add_gridspec(3, 3, height_ratios=[1.1, 1.4, 1])
    ax_gyro = fig.add_subplot(grid[0, :])
    ax_views = [fig.add_subplot(grid[1, k]) for k in range(3)]
    ax_field = fig.add_subplot(grid[2, :2])
    ax_log = fig.add_subplot(grid[2, 2])
    plt.ion()
    fig.show()

    gyro, mag = [], []                  # rows of floats
    log = []                            # Pico text lines (countdown, results)
    phase, phase_seconds = "starting", 0
    last_png, finished = 0.0, False
    while not finished:
        # Take everything the Pico sent since the last frame
        while True:
            try:
                line = lines.get_nowait()
            except queue.Empty:
                break
            if line is None:
                finished = True
                break
            if line.startswith("G,"):
                gyro.append([float(v) for v in line[2:].split(",")])
            elif line.startswith("M,"):
                mag.append([float(v) for v in line[2:].split(",")])
            elif line.startswith("PHASE,"):
                _, phase, seconds = line.split(",")
                phase_seconds = int(seconds)
            elif line.strip():
                log.append(line.strip())
                print(line, flush=True)

        # --- big status banner
        last = log[-1] if log else ""
        rows = gyro if phase == "gyro" else mag
        if "starts in" in last or "Start rotating in" in last:
            banner, colour = "GET READY\n" + last.split(" in ")[-1].replace(" ...", ""), "tab:red"
        elif phase == "gyro" and rows:
            banner, colour = "KEEP STILL  %.0f s" % max(phase_seconds - rows[-1][0], 0), "tab:green"
        elif phase == "mag" and rows:
            banner, colour = "ROTATE IN EVERY DIRECTION  %.0f s" % max(phase_seconds - rows[-1][0], 0), "tab:blue"
        elif phase == "done":
            banner, colour = "DONE", "black"
        else:
            banner, colour = "starting ...", "gray"
        fig.suptitle(banner, fontsize=22, color=colour, fontweight="bold")

        # --- gyro panel
        ax_gyro.clear()
        if gyro:
            g = np.array(gyro)
            for k, axis in enumerate("xyz"):
                ax_gyro.plot(g[:, 0], np.degrees(g[:, 1 + k]), lw=0.8,
                             label="gyro %s  mean %+.3f deg/s" % (axis, np.degrees(g[:, 1 + k].mean())))
            ax_gyro.legend(loc="upper right", fontsize=8)
        ax_gyro.set_xlim(0, max(phase_seconds if phase == "gyro" else (gyro[-1][0] if gyro else 1), 1))
        ax_gyro.set_title("Gyro while still (raw): the means become the bias")
        ax_gyro.set_ylabel("deg/s")
        ax_gyro.grid(True)

        # --- magnetometer panels
        for ax in ax_views + [ax_field]:
            ax.clear()
            ax.grid(True)
        if mag:
            m = np.array(mag)
            uT = m[:, 1:4] * 1e6
            for ax, (a, b, i, j) in zip(ax_views, (("x", "y", 0, 1), ("x", "z", 0, 2), ("y", "z", 1, 2))):
                ax.scatter(uT[:, i], uT[:, j], s=3, c=m[:, 0], cmap="viridis")
                ax.scatter(uT[-1, i], uT[-1, j], s=60, color="red")       # where it is now
                ax.set_aspect("equal", "datalim")
                ax.set_title("raw %s-%s (uT): fill an ellipse" % (a, b), fontsize=9)
            ax_field.plot(m[:, 0], np.linalg.norm(uT, axis=1), color="tab:red", lw=1)
            ax_field.set_xlim(0, max(phase_seconds if phase == "mag" else m[-1, 0], 1))
        ax_field.set_title("Raw field strength |B| (uT)")
        ax_field.set_xlabel("time (s)")

        # --- Pico messages
        ax_log.clear()
        ax_log.axis("off")
        ax_log.text(0, 1, "\n".join(l[:60] for l in log[-14:]), va="top", family="monospace", fontsize=7.5)

        if time.time() - last_png > 2.0 or finished:
            fig.savefig(live_png, dpi=90)
            last_png = time.time()
        fig.canvas.draw_idle()
        fig.canvas.start_event_loop(0.25)

    process.wait()
    fig.savefig(live_png, dpi=90)
    print("live image: " + live_png)

    # Final images from the files the Pico saved
    plot_pico_results.fetch(args.folder)
    images = [p for p in (plot_pico_results.plot_gyro(args.folder), plot_pico_results.plot_mag(args.folder)) if p]
    for path in images:
        print("image: " + path)
    plot_pico_results.open_images(images)
    fig.canvas.start_event_loop(3.0)


if __name__ == "__main__":
    main()
