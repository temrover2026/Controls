#!/usr/bin/env python3
"""Laptop tool: live heading test with the Pico H IMU (like TemRover heading_test_record).

Runs on the laptop with the Pico plugged in by USB, RaspTestSeparate code and calibration on it.

    python3 tools/live_heading.py                   # 60 s at 10 Hz after a 5 s countdown
    python3 tools/live_heading.py --seconds 30 --heading-range 180
    python3 tools/live_heading.py --lines kalman      # heading plot shows only the Kalman heading

The Pico records calibrated data to its flash (logger.record) and streams every row over USB.
This window computes and shows, live:
  - headings 0-360 deg (or -180..180), anticlockwise positive, as in the rover tests:
      mag    = atan2(mag_y, mag_x) of the calibrated field (Controls calc_mag_heading, no tilt correction)
      gyro   = calibrated gyro z integrated (trapezoid), started on the Kalman heading
      kalman = Controls HeadingKalmanFilter (angle + gyro bias), gyro-weighted settings as on the rover
  - calibrated gyro rates, raw magnetometer, calibrated magnetometer
and keeps live_heading.png up to date. At the end it saves heading_test.csv and heading_test.png in
~/imu_data/pico_<date_time>_heading/ and opens the image.

Keep the IMU flat (the heading has no tilt correction) and away from the laptop and steel.
"""

import argparse
import csv
import inspect
import math
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

# The heading Kalman filter is the Controls one (Capstone-3/heading_output/heading_func.py, plain Python)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Capstone-3"))
from heading_output.heading_func import HeadingKalmanFilter   # noqa: E402

# Gyro-weighted settings used on the rover (TemRover hardware.launch.py), except max_dt: the filter skips
# steps longer than max_dt, and at 10 Hz the steps are 0.1 s, so it is 0.25 here. Arguments the Controls
# filter does not have (initial_bias_variance, init_samples) are left out automatically.
KALMAN_SETTINGS = {"angle_noise": 1e-4, "bias_noise": 1e-6, "mag_noise": 1.0, "max_dt": 0.25,
                   "initial_bias_variance": 1e-6, "init_samples": 30}

PICO_CODE = """
import time, config, calibration, logger
from machine import I2C, Pin
from icm20948 import ICM20948
if config.IMU_GROUND_PIN is not None:
    Pin(config.IMU_GROUND_PIN, Pin.OUT, value=0); time.sleep_ms(50)
i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA_PIN), scl=Pin(config.I2C_SCL_PIN), freq=config.I2C_FREQ)
imu = ICM20948(i2c, config.IMU_ADDR, config.MAG_MODE)
cal = calibration.Calibration()
config.COUNTDOWN_SECONDS = {countdown}
logger.record(imu, cal, seconds={seconds}, hz={hz}, stream=True)
"""

# --stream-only: nothing is written on the Pico. This loop runs from the Pico's RAM (mpremote exec),
# reads the IMU, applies the saved calibration (read only) and prints each row over USB.
PICO_STREAM_CODE = """
import time, config, calibration, util
from machine import I2C, Pin
from icm20948 import ICM20948
i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA_PIN), scl=Pin(config.I2C_SCL_PIN), freq=config.I2C_FREQ)
imu = ICM20948(i2c, config.IMU_ADDR, config.MAG_MODE)
cal = calibration.Calibration()
for s in range({countdown}, 0, -1):
    print('  Recording starts in %d s ...' % s); time.sleep(1)
print('  >>> GO')
rate = util.Rate({hz}); start = time.ticks_us()
while True:
    t = util.seconds_since(start)
    if {seconds} and t >= {seconds}:
        break
    try:
        a, g, m = imu.read_si()
    except OSError:
        rate.sleep(); continue
    print('D,' + util.csv_line((t,) + a + cal.apply_gyro(g) + cal.apply_mag(m) + g + m), end='')
    rate.sleep()
print('  stream finished')
"""

COLUMNS = ("t", "gyro_heading", "mag_heading", "kalman_heading", "gyro_x", "gyro_y", "gyro_z",
           "mag_raw_x", "mag_raw_y", "mag_raw_z", "mag_cal_x", "mag_cal_y", "mag_cal_z",
           "accel_x", "accel_y", "accel_z")


def make_filter():
    accepted = inspect.signature(HeadingKalmanFilter.__init__).parameters
    return HeadingKalmanFilter(**{k: v for k, v in KALMAN_SETTINGS.items() if k in accepted})


class Headings:
    """Turns streamed Pico rows into heading rows (all angles in radians)."""

    def __init__(self):
        self.kalman = make_filter()
        self.gyro_heading = None
        self.prev = None            # (t, gyro z) of the previous row
        self.rows = []

    def add(self, v):
        # v: t, accel(3), gyro cal(3), mag cal(3), gyro raw(3), mag raw(3)
        t, accel, gyro, mag_cal, mag_raw = v[0], v[1:4], v[4:7], v[7:10], v[13:16]
        mag_heading = math.atan2(mag_cal[1], mag_cal[0])
        self.kalman.predict(gyro[2], t)
        k = self.kalman.update(mag_heading, t)
        if k is None:               # Filter still averaging its first magnetometer headings
            self.prev = (t, gyro[2])
            return
        if self.gyro_heading is None:
            self.gyro_heading = k   # Start the gyro heading on the Kalman heading so they overlap
        else:
            dt = t - self.prev[0]
            if 0.0 < dt < 0.5:
                self.gyro_heading += 0.5 * (gyro[2] + self.prev[1]) * dt
        self.prev = (t, gyro[2])
        self.rows.append((t, self.gyro_heading, mag_heading, k, *gyro, *mag_raw, *mag_cal, *accel))


def draw(fig, axes, rows, banner, colour, seconds, heading_range, lines=("mag", "gyro", "kalman")):
    head_ax, rate_ax, raw_ax, cal_ax = axes
    for ax in axes:
        ax.clear()
        ax.grid(True)
    low = 0.0 if heading_range == 360 else -180.0
    if rows:
        d = np.array(rows)
        t = d[:, 0]
        for i, name in ((2, "mag"), (1, "gyro"), (3, "kalman")):
            if name not in lines:
                continue
            h = (np.degrees(np.unwrap(d[:, i])) - low) % 360.0 + low
            h[1:][np.abs(np.diff(h)) > 180.0] = np.nan           # Lift the pen where it wraps
            head_ax.plot(t, h, lw=1.6 if name == "kalman" else 1.1, label="%s  %.0f deg" % (name, h[-1] if np.isfinite(h[-1]) else np.nan))
        for k, axis in enumerate("xyz"):
            rate_ax.plot(t, np.degrees(d[:, 4 + k]), lw=1, label="gyro " + axis)
            raw_ax.plot(t, d[:, 7 + k] * 1e6, lw=1, label="raw " + axis)
            cal_ax.plot(t, d[:, 10 + k] * 1e6, lw=1, label="calibrated " + axis)
        turned = {n: math.degrees(np.unwrap(d[:, i])[-1] - d[0, i])
                  for i, n in ((1, "gyro"), (2, "mag"), (3, "kalman")) if n in lines}
        what = "Kalman heading" if tuple(lines) == ("kalman",) else "Headings"
        head_ax.set_title("%s %d to %d deg (anticlockwise +)   turned: %s"
                          % (what, low, low + 360, ", ".join("%s %+.0f deg" % kv for kv in turned.items())), fontsize=10)
        for ax in axes:
            ax.legend(loc="upper left", fontsize=8, ncol=3)
    head_ax.set_ylim(low, low + 360)
    head_ax.set_yticks(range(int(low), int(low) + 361, 45))
    head_ax.set_ylabel("heading (deg)")
    rate_ax.set_ylabel("deg/s")
    rate_ax.set_title("Gyro (calibrated)", fontsize=10)
    raw_ax.set_ylabel("uT")
    raw_ax.set_title("Magnetometer raw", fontsize=10)
    cal_ax.set_ylabel("uT")
    cal_ax.set_title("Magnetometer calibrated (x, y used for heading)", fontsize=10)
    cal_ax.set_xlabel("time (s)")
    for ax in axes:
        ax.set_xlim(0, max(seconds, 1))
    fig.suptitle(banner, fontsize=20, color=colour, fontweight="bold")


def reader(process, lines):
    for line in process.stdout:
        lines.put(line.rstrip("\n"))
    lines.put(None)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=int, default=60)
    parser.add_argument("--hz", type=int, default=10)
    parser.add_argument("--countdown", type=int, default=5)
    parser.add_argument("--heading-range", type=int, choices=(360, 180), default=360)
    parser.add_argument("--stream-only", action="store_true",
                        help="do not write anything on the Pico (no log file); the laptop keeps the data")
    parser.add_argument("--lines", default="mag,gyro,kalman",
                        help="headings to plot, comma separated (e.g. kalman)")
    parser.add_argument("--folder", default=None, help="default ~/imu_data/<board>_<date_time>_heading")
    args = parser.parse_args()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import plot_pico_results
    board = plot_pico_results.board_name()
    if args.folder is None:
        args.folder = os.path.expanduser("~/imu_data/%s_%s_heading" % (board, datetime.now().strftime("%Y%m%d_%H%M%S")))
    lines_shown = tuple(n.strip() for n in args.lines.split(",") if n.strip())
    os.makedirs(args.folder, exist_ok=True)
    live_png = os.path.join(args.folder, "live_heading.png")

    exe = shutil.which("mpremote") or os.path.expanduser("~/.local/bin/mpremote")
    template = PICO_STREAM_CODE if args.stream_only else PICO_CODE
    code = template.format(countdown=args.countdown, seconds=args.seconds, hz=args.hz)
    process = subprocess.Popen([exe, "connect", "auto", "exec", code], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1,
                               env=dict(os.environ, PYTHONUNBUFFERED="1"))
    lines = queue.Queue()
    threading.Thread(target=reader, args=(process, lines), daemon=True).start()

    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True, gridspec_kw={"height_ratios": [2, 1, 1, 1]})
    plt.ion()
    fig.show()
    headings = Headings()
    last_text, last_png, finished = "", 0.0, False
    while not finished:
        if not plt.fignum_exists(fig.number):           # Window closed: stop the Pico and finish
            process.terminate()
            break
        while True:
            try:
                line = lines.get_nowait()
            except queue.Empty:
                break
            if line is None:
                finished = True
                break
            if line.startswith("D,"):
                headings.add([float(x) for x in line[2:].split(",")])
            elif line.strip():
                last_text = line.strip()
                print(line, flush=True)
        rows = headings.rows
        if "starts in" in last_text:
            banner, colour = "GET READY " + last_text.split(" in ")[-1].replace(" ...", ""), "tab:red"
        elif finished:
            banner, colour = "DONE", "black"
        elif rows:
            banner, colour = "TURN THE IMU (FLAT)  %.0f s left" % max(args.seconds - rows[-1][0], 0), "tab:green"
        else:
            banner, colour = "starting ...", "gray"
        draw(fig, axes, rows, banner, colour, args.seconds, args.heading_range, lines_shown)
        if time.time() - last_png > 2.0 and plt.fignum_exists(fig.number):
            fig.savefig(live_png, dpi=90)
            last_png = time.time()
        fig.canvas.draw_idle()
        fig.canvas.start_event_loop(0.3)
    process.wait()
    if not plt.fignum_exists(fig.number):                 # Closed early: still save what came in
        fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True, gridspec_kw={"height_ratios": [2, 1, 1, 1]})

    csv_path = os.path.join(args.folder, "heading_test.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        writer.writerows(headings.rows)
    final_png = os.path.join(args.folder, "heading_test.png")
    draw(fig, axes, headings.rows, "%s IMU heading test: %d samples" % (board, len(headings.rows)), "black",
         args.seconds, args.heading_range, lines_shown)
    fig.savefig(final_png, dpi=110)
    fig.savefig(live_png, dpi=90)
    print("data:  " + csv_path)
    print("image: " + final_png)
    plot_pico_results.open_images([final_png])
    fig.canvas.start_event_loop(3.0)


if __name__ == "__main__":
    main()
