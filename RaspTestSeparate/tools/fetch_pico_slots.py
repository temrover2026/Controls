#!/usr/bin/env python3
"""Laptop tool: copy the Pico H recordings (slot_new = latest, slot_old = the one before) and plot them.

Plug the Pico into the laptop and run this straight away (within the 10 s fast-blink window if you
can, so the Pico does not rotate the slots and start a new recording; if it already has, the
field run is in slot_old):

    python3 tools/fetch_pico_slots.py

Copies /imu_data/slots.json, slot_new.csv and slot_old.csv to ~/imu_data/pico_<date_time>_slots/, prints
what each slot holds, makes slot_<new|old>_kalman.png (Kalman heading 0-360) and slot_<new|old>_sensors.png (gyro, magnetometer,
accelerometer) for each slot and opens them. Nothing is deleted on the Pico.
"""

import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plot_pico_results   # noqa: E402


PORT = "auto"


def mpremote(*args):
    exe = shutil.which("mpremote") or os.path.expanduser("~/.local/bin/mpremote")
    return subprocess.run([exe, "connect", PORT, *args], capture_output=True, text=True, timeout=300)


def main():
    global PORT
    if "--port" in sys.argv:                            # e.g. --port /dev/ttyACM0 (pico_autofetch.py)
        PORT = sys.argv[sys.argv.index("--port") + 1]
    board = plot_pico_results.board_name(PORT)          # picoTX / picoRX (from /board_name.txt on the Pico)
    folder = os.path.expanduser("~/imu_data/%s_%s_slots" % (board, datetime.now().strftime("%Y%m%d_%H%M%S")))
    print("board: " + board)
    os.makedirs(folder, exist_ok=True)
    # Connecting interrupts whatever main.py is doing (a recording in progress is closed and kept)
    for name in ("slots.json", "slot_new.csv", "slot_old.csv"):
        # Retried: right after a plug-in the first connection can fail while the port settles
        for attempt in range(4):
            result = mpremote("cp", ":/imu_data/" + name, os.path.join(folder, name))
            if result.returncode == 0 or "No such file" in (result.stdout + result.stderr) or "ENOENT" in (
                    result.stdout + result.stderr):
                break
            time.sleep(1.0)
        print(("copied  " if result.returncode == 0 else "missing ") + name)
    try:
        with open(os.path.join(folder, "slots.json")) as f:
            state = json.load(f)
    except (OSError, ValueError):
        state = {}
    images = []
    for slot in ("new", "old"):
        info = state.get(slot, {})
        if not info.get("seq"):
            if not os.path.exists(os.path.join(folder, "slot_%s.csv" % slot)):
                print("slot %s: empty" % slot)
                continue
            info = {"seq": 0, "complete": False, "reason": "unknown (slots.json not copied)"}
        csv_name = "slot_%s.csv" % slot
        rows, seconds = info.get("rows", 0), info.get("seconds", 0)
        path = os.path.join(folder, csv_name)
        if os.path.exists(path) and not rows:
            # After a power cut slots.json has no totals: count them from the file itself
            with open(path) as f:
                lines = [l for l in f.read().splitlines()[1:] if l.strip()]
            rows = len(lines)
            seconds = float(lines[-1].split(",")[0]) if lines else 0.0
        label = "recording #%s, %s, %d rows, %.0f s, stopped by %s%s" % (
            info["seq"] or "?", "complete" if info.get("complete") else "CUT SHORT", rows, seconds,
            info.get("reason", "?"), "  (latest)" if slot == "new" else "  (the one before)")
        print("slot %s: %s" % (slot, label))
        if os.path.exists(os.path.join(folder, csv_name)):
            made = plot_pico_results.plot_log(folder, csv_name, "%s slot %s: %s" % (board, slot, label))
            if made:
                images += made                          # slot_X_kalman.png and slot_X_sensors.png
    for image in images:
        print("image: " + image)
    plot_pico_results.open_images(images)
    print("folder: " + folder)
    return folder, state, images


if __name__ == "__main__":
    main()
