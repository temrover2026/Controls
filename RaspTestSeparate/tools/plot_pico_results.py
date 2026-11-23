#!/usr/bin/env python3
"""Laptop tool: copy the Pico H IMU files over USB and plot them as PNG images.

Runs on the laptop (normal Python with numpy + matplotlib), NOT on the Pico.

    python3 tools/plot_pico_results.py            # copy from the Pico, plot, open the images
    python3 tools/plot_pico_results.py --no-fetch --folder ~/imu_data/pico_20261123_120000

Makes, in ~/imu_data/pico_<date_time>/ (like the Controls/TemRover heading tests):
  gyro_calibration.png  stationary gyro and accelerometer with the fitted bias
  mag_calibration.png   magnetometer before and after calibration (3 views + field strength)
  imu_log_NNN.png       the newest recording: accel, calibrated gyro, calibrated magnetometer,
                        and the headings from the gyro and the magnetometer
"""

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime

import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt     # noqa: E402
import numpy as np                  # noqa: E402

PICO_FILES = ["/imu_calibration/bias_file_gyro.csv", "/imu_calibration/gyr_cal_record.csv",
              "/imu_calibration/mag_calibration.csv", "/imu_calibration/mag_record.csv"]


def open_images(paths):
    """Open images in the desktop image viewer.

    Started with a clean environment: run from VS Code (a snap) the inherited library paths make
    eog/xdg-open crash with "symbol lookup error", so the images silently never appeared.
    """
    uid = os.getuid()
    env = {"HOME": os.path.expanduser("~"), "PATH": "/usr/bin:/bin", "DISPLAY": os.environ.get("DISPLAY", ":0"),
           "XDG_RUNTIME_DIR": "/run/user/%d" % uid, "DBUS_SESSION_BUS_ADDRESS": "unix:path=/run/user/%d/bus" % uid}
    for key in ("WAYLAND_DISPLAY", "XDG_SESSION_TYPE", "USER"):
        if os.environ.get(key):
            env[key] = os.environ[key]
    paths = [p for p in paths if p]
    if paths:
        subprocess.Popen(["xdg-open" if len(paths) == 1 else "eog", *paths], env=env, stdin=subprocess.DEVNULL,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def board_name(port="auto"):
    """Name of the connected Pico (/board_name.txt on the board, e.g. picoTX / picoRX), or "pico"."""
    exe = shutil.which("mpremote") or os.path.expanduser("~/.local/bin/mpremote")
    for attempt in range(4):                            # The first connection after a plug-in can fail
        try:
            out = subprocess.run([exe, "connect", port, "exec",
                                  "import util; print('BOARD=' + util.board_name())"],
                                 capture_output=True, text=True, timeout=30).stdout
        except (OSError, subprocess.TimeoutExpired):
            out = ""
        for line in out.splitlines():
            if line.startswith("BOARD="):
                return line[6:].strip() or "pico"
        time.sleep(1.0)
    return "pico"


def mpremote(*args):
    exe = shutil.which("mpremote") or os.path.expanduser("~/.local/bin/mpremote")
    return subprocess.run([exe, "connect", "auto", *args], capture_output=True, text=True, timeout=120)


def fetch(folder):
    """Copy the calibration files and the newest recording from the Pico into `folder`."""
    os.makedirs(folder, exist_ok=True)
    listing = mpremote("ls", ":/imu_data").stdout.split()
    logs = sorted(name for name in listing if name.startswith("imu_log_") and name.endswith(".csv"))
    files = PICO_FILES + (["/imu_data/" + logs[-1]] if logs else [])
    for remote in files:
        result = mpremote("cp", ":" + remote, os.path.join(folder, os.path.basename(remote)))
        print(("copied  " if result.returncode == 0 else "missing ") + remote)


def load(path):
    """CSV with a header row -> {column: numpy array}, or None if the file is missing."""
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    header, data = rows[0], np.array(rows[1:], dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return {name: data[:, i] for i, name in enumerate(header)}


def plot_gyro(folder):
    rec, bias = load(os.path.join(folder, "gyr_cal_record.csv")), load(os.path.join(folder, "bias_file_gyro.csv"))
    if rec is None:
        return None
    t = rec["time"]
    fig, (ax_raw, ax_cal, ax_acc) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for axis, colour in zip("xyz", ("tab:red", "tab:green", "tab:blue")):
        raw = np.degrees(rec["gyro_" + axis])
        b = math.degrees(bias["gyro_bias_" + axis][0]) if bias else raw.mean()
        ax_raw.plot(t, raw, color=colour, lw=0.8, label="gyro %s  (bias %+.3f deg/s)" % (axis, b))
        ax_raw.axhline(b, color=colour, ls="--", lw=1)
        cal = raw - b
        ax_cal.plot(t, cal, color=colour, lw=0.8, label="gyro %s  (noise std %.3f deg/s)" % (axis, cal.std()))
        ax_acc.plot(t, rec["accel_" + axis], color=colour, lw=0.8, label="accel %s  (mean %+.2f m/s^2)"
                    % (axis, rec["accel_" + axis].mean()))
    g = np.sqrt(sum(rec["accel_" + a] ** 2 for a in "xyz"))
    rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0
    ax_raw.set_title("Raw gyro while still: dashed = bias saved to bias_file_gyro.csv")
    ax_cal.set_title("Calibrated gyro (raw - bias): should sit on 0")
    ax_acc.set_title("Accelerometer (|a| = %.3f m/s^2, gravity 9.807)" % g.mean())
    for ax, unit in ((ax_raw, "deg/s"), (ax_cal, "deg/s"), (ax_acc, "m/s^2")):
        ax.set_ylabel(unit)
        ax.grid(True)
        ax.legend(loc="upper right", fontsize=8)
    ax_acc.set_xlabel("time (s)")
    fig.suptitle(os.path.basename(folder).split("_")[0] + " gyro calibration: %d samples, %.0f Hz, %.0f s" % (len(t), rate, t[-1] - t[0]))
    fig.tight_layout()
    path = os.path.join(folder, "gyro_calibration.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def mag_correct(cal, x, y, z):
    off = np.array([cal["offset_" + a][0] for a in "xyz"])
    m = np.array([[cal["matrix_%d%d" % (r, c)][0] for c in range(3)] for r in range(3)])
    return (m @ (np.vstack([x, y, z]).T - off).T)


def plot_mag(folder):
    rec, cal = load(os.path.join(folder, "mag_record.csv")), load(os.path.join(folder, "mag_calibration.csv"))
    if rec is None:
        return None
    raw = np.vstack([rec["mag_" + a] for a in "xyz"]) * 1e6       # uT
    fig = plt.figure(figsize=(14, 9))
    views = (("x", "y", 0, 1), ("x", "z", 0, 2), ("y", "z", 1, 2))
    corrected = mag_correct(cal, *(rec["mag_" + a] for a in "xyz")) * 1e6 if cal else None
    for k, (a, b, i, j) in enumerate(views):
        ax = fig.add_subplot(2, 3, k + 1)
        ax.scatter(raw[i], raw[j], s=3, color="tab:red", label="raw")
        if corrected is not None:
            ax.scatter(corrected[i], corrected[j], s=3, color="tab:blue", label="calibrated")
        ax.axhline(0, color="k", lw=0.5)
        ax.axvline(0, color="k", lw=0.5)
        ax.set_aspect("equal", "datalim")
        ax.set_xlabel(a + " (uT)")
        ax.set_ylabel(b + " (uT)")
        ax.set_title("%s-%s view" % (a, b))
        ax.grid(True)
        ax.legend(fontsize=8, loc="upper right")
    ax = fig.add_subplot(2, 1, 2)
    t = rec["t_stamp"]
    ax.plot(t, np.linalg.norm(raw, axis=0), color="tab:red", lw=1, label="raw |B|")
    title = "Field strength: calibrated should be flat"
    if corrected is not None:
        length = np.linalg.norm(corrected, axis=0)
        ax.plot(t, length, color="tab:blue", lw=1, label="calibrated |B| (mean %.1f, spread %.1f %%)"
                % (length.mean(), 100 * length.std() / length.mean()))
        off = [cal["offset_" + a][0] * 1e6 for a in "xyz"]
        title += "   |   hard-iron offset %.1f, %.1f, %.1f uT" % tuple(off)
    ax.set_title(title)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("uT")
    ax.grid(True)
    ax.legend(fontsize=8)
    fig.suptitle(os.path.basename(folder).split("_")[0] + " magnetometer calibration: %d samples (red raw, blue calibrated: a circle round 0 in every view)"
                 % raw.shape[1])
    fig.tight_layout()
    path = os.path.join(folder, "mag_calibration.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def kalman_heading(t, gyro_z, mag_heading_rad):
    """Kalman heading (rad) of a recording, computed on the laptop with the Controls filter
    (Capstone-3/heading_output/heading_func.py) and the gyro-weighted rover settings.
    max_dt is 0.25 s so 10 Hz logs (0.1 s steps) are not skipped. Returns NaN until the filter starts."""
    import inspect
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Capstone-3"))
    from heading_output.heading_func import HeadingKalmanFilter
    settings = {"angle_noise": 1e-4, "bias_noise": 1e-6, "mag_noise": 1.0, "max_dt": 0.25,
                "initial_bias_variance": 1e-6, "init_samples": 30}
    accepted = inspect.signature(HeadingKalmanFilter.__init__).parameters
    kf = HeadingKalmanFilter(**{k: v for k, v in settings.items() if k in accepted})
    out = np.full(len(t), np.nan)
    for i in range(len(t)):
        kf.predict(float(gyro_z[i]), float(t[i]))
        k = kf.update(float(mag_heading_rad[i]), float(t[i]))
        if k is not None:
            out[i] = k
    return out


def plot_log(folder, name=None, title=None, lines=("kalman",)):
    """Plot one recording (default: the newest imu_log_NNN.csv in `folder`) as TWO images:

      <name>_kalman.png   the Kalman heading on its own (0-360 deg), plus any extra heading
                          lines asked for in `lines` ("magnetometer", "gyro")
      <name>_sensors.png  the IMU data: calibrated gyro, calibrated magnetometer, accelerometer

    Returns [kalman image, sensors image] (or None if the file has no data)."""
    if name is None:
        logs = sorted(n for n in os.listdir(folder) if n.startswith("imu_log_") and n.endswith(".csv"))
        if not logs:
            return None
        name = logs[-1]
    rec = load(os.path.join(folder, name))
    if rec is None or len(rec["t"]) < 2:
        return None
    t = rec["t"]
    base = os.path.join(folder, name.replace(".csv", ""))
    title = title or "Pico H IMU recording %s: %d rows, %.0f s" % (name, len(t), t[-1] - t[0])

    # --- 1. Kalman heading plot
    mag_rad = np.arctan2(rec["mag_y"], rec["mag_x"])
    mag_heading = np.degrees(mag_rad)
    gyro_heading = mag_heading[0] + np.degrees(np.concatenate([[0], np.cumsum(rec["gyro_z"][1:] * np.diff(t))]))
    kalman = np.degrees(kalman_heading(t, rec["gyro_z"], mag_rad))
    fig, ax = plt.subplots(figsize=(12, 5))
    turned = []
    for label, h in (("magnetometer", mag_heading), ("gyro", gyro_heading), ("kalman", kalman)):
        if label not in lines and label != "kalman":
            continue
        wrapped = h % 360.0
        wrapped[1:][np.abs(np.diff(wrapped)) > 180] = np.nan    # Lift the pen where it wraps
        valid = h[np.isfinite(h)]
        ax.plot(t, wrapped, lw=2.0 if label == "kalman" else 1.0,
                label="%s  (end %.0f deg)" % (label, valid[-1] % 360 if len(valid) else float("nan")))
        if len(valid) > 1:
            turned.append("%s %+.0f" % (label, np.degrees(np.unwrap(np.radians(valid)))[-1] - valid[0]))
    ax.set_ylim(0, 360)
    ax.set_yticks(range(0, 361, 45))
    ax.set_xlim(t[0], t[-1])
    ax.set_ylabel("heading (deg, anticlockwise +)")
    ax.set_xlabel("time (s)")
    ax.grid(True)
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("Kalman heading 0-360 deg (gyro + calibrated magnetometer, Controls filter)   turned: %s deg"
                 % ", ".join(turned), fontsize=10)
    fig.suptitle(title)
    fig.tight_layout()
    kalman_png = base + "_kalman.png"
    fig.savefig(kalman_png, dpi=110)
    plt.close(fig)

    # --- 2. IMU sensor plot
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    gyro_ax, mag_ax, acc_ax = axes
    for axis in "xyz":
        gyro_ax.plot(t, np.degrees(rec["gyro_" + axis]), lw=1, label="gyro " + axis)
        mag_ax.plot(t, rec["mag_" + axis] * 1e6, lw=1, label="calibrated " + axis)
        acc_ax.plot(t, rec["accel_" + axis], lw=1, label="accel " + axis)
    gyro_ax.set_ylabel("deg/s")
    gyro_ax.set_title("Gyro (calibrated: raw - bias)")
    mag_ax.set_ylabel("uT")
    mag_ax.set_title("Magnetometer (calibrated: hard + soft iron)")
    acc_ax.set_ylabel("m/s^2")
    acc_ax.set_title("Accelerometer (as measured)")
    acc_ax.set_xlabel("time (s)")
    for a in axes:
        a.grid(True)
        a.legend(loc="upper left", fontsize=8, ncol=3)
    fig.suptitle(title + " - IMU data")
    fig.tight_layout()
    sensors_png = base + "_sensors.png"
    fig.savefig(sensors_png, dpi=110)
    plt.close(fig)
    return [kalman_png, sensors_png]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--folder", default=None, help="default ~/imu_data/<board>_<date_time>")
    parser.add_argument("--no-fetch", action="store_true", help="plot files already in --folder")
    parser.add_argument("--no-open", action="store_true", help="do not open the images")
    parser.add_argument("--only", choices=("gyro", "mag", "log"), help="make just one of the plots")
    args = parser.parse_args()
    if args.folder is None:
        args.folder = os.path.expanduser("~/imu_data/%s_%s" % (board_name(), datetime.now().strftime("%Y%m%d_%H%M%S")))
    if not args.no_fetch:
        fetch(args.folder)
    makers = {"gyro": plot_gyro, "mag": plot_mag, "log": plot_log}
    images = []
    for name, make in makers.items():
        if args.only in (None, name):
            made = make(args.folder)
            images += made if isinstance(made, list) else ([made] if made else [])
    for path in images:
        print("image: " + path)
    if images and not args.no_open:
        open_images(images)
    if not images:
        sys.exit("No Pico IMU files found to plot in " + args.folder)


if __name__ == "__main__":
    main()
