# dualslot.py - "two storage" recording that runs every time the Pico H powers on.
#
# Two slots with fixed names:
#   /imu_data/slot_new.csv   the latest recording
#   /imu_data/slot_old.csv   the recording before it
#
# Every power-on:
#   1. slot_new -> slot_old   the previous "new" recording becomes "old" (it replaces what was in
#                             slot_old: there are only two slots, each with half of the space)
#   2. slot_new is rewritten  live data is recorded into it at LOG_RATE_HZ until it holds its share
#                             (SLOT_FRACTION), then it stops and saves - LED on solid = finished.
#
# So after any recording, slot_new is the one just made and slot_old the one before.
# A power cut part-way just leaves a shorter slot_new (everything up to the last flush is kept).
#
# The 10 s window at power-on (LED blinking fast): if a laptop talks to the Pico (mpremote, a
# serial terminal or Enter) in that time, NOTHING is rotated or deleted and the menu opens instead.
# Collect the data with tools/fetch_pico_slots.py, which connects straight away. If the laptop
# connects later, the rotation has already happened: your field run is then in slot_old (still
# safe) and slot_new holds the few seconds recorded on the desk.
#
# slots.json keeps, for each slot: the recording number (counts up; the Pico has no clock),
# whether it completed, and its rows / seconds / stop reason.

import json
import os
import time

import config
import logger
import util

SLOTS = ("new", "old")
STATE_FILE = config.DATA_DIR + "/slots.json"


def slot_path(name):
    return "%s/slot_%s.csv" % (config.DATA_DIR, name)


def load_state():
    """{"new": {...}, "old": {...}}; a slot that never recorded has seq 0."""
    state = {name: {"seq": 0, "complete": False} for name in SLOTS}
    try:
        with open(STATE_FILE) as f:
            saved = json.load(f)
        for name in SLOTS:
            if name in saved:
                state[name].update(saved[name])
        # Keep counting on from recordings made by the earlier A/B version
        state["_last_seq"] = max([v.get("seq", 0) for v in saved.values() if isinstance(v, dict)] + [0])
    except (OSError, ValueError):
        pass                                            # First run, or the file was damaged
    return state


def save_state(state):
    saved = {name: state[name] for name in SLOTS}
    saved["board"] = util.board_name()                  # Which Pico made these recordings
    with open(STATE_FILE, "w") as f:
        json.dump(saved, f)


def file_size(path):
    try:
        return os.stat(path)[6]
    except OSError:
        return 0


def remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


def slot_budget():
    """Bytes one slot may use: SLOT_FRACTION of (free space + both slot files - the safety margin)."""
    usable = util.free_bytes() + sum(file_size(slot_path(n)) for n in SLOTS) - config.MIN_FREE_BYTES
    return max(int(usable * config.SLOT_FRACTION), 0)


def status():
    """Print what is in each slot (menu 9)."""
    state = load_state()
    state.pop("_last_seq", None)
    print("  board: %s" % util.board_name())
    for name in SLOTS:
        s = state[name]
        if not s["seq"]:
            print("  slot %s: empty" % name)
            continue
        print("  slot %s: recording #%d, %s, %d rows, %.0f s, %d KB, stopped by %s"
              % (name, s["seq"], "complete" if s["complete"] else "cut short", s.get("rows", 0),
                 s.get("seconds", 0), file_size(slot_path(name)) // 1024, s.get("reason", "?")))
    print("  each slot may use %d KB = about %.0f min at %d Hz"
          % (slot_budget() // 1024, slot_budget() / (115 * config.LOG_RATE_HZ) / 60, config.LOG_RATE_HZ))


def rotate(state):
    """Step 1: slot_new -> slot_old. The file is renamed; the previous slot_old is removed first
    because a rename cannot overwrite an existing file. Returns the updated state."""
    remove(slot_path("old"))
    if file_size(slot_path("new")):
        os.rename(slot_path("new"), slot_path("old"))
        state["old"] = state["new"]
    else:
        state["old"] = {"seq": 0, "complete": False}    # No new recording yet (first ever run)
    state["new"] = {"seq": 0, "complete": False}
    return state


def record_slot(imu, cal):
    """Step 1: slot_new -> slot_old. Step 2: record live data into slot_new until it holds its share."""
    util.ensure_dir(config.DATA_DIR)
    # Recordings made by the earlier A/B version are not used any more: free their space
    remove(config.DATA_DIR + "/slot_A.csv")
    remove(config.DATA_DIR + "/slot_B.csv")

    state = load_state()
    seq = max([state[n]["seq"] for n in SLOTS] + [state.pop("_last_seq", 0)]) + 1
    state = rotate(state)
    # Mark slot_new as started-but-not-finished first: if the power is pulled, the state says so
    state["new"] = {"seq": seq, "complete": False, "reason": "power lost"}
    save_state(state)

    budget = slot_budget()                              # After the rotation, so old + free space count
    print("\nRECORDING #%d into slot_new (previous recording now in slot_old: %s)"
          % (seq, "#%d" % state["old"]["seq"] if state["old"]["seq"] else "empty"))
    print("  slot size limit %d KB = about %.0f min at %d Hz"
          % (budget // 1024, budget / (115 * config.LOG_RATE_HZ) / 60, config.LOG_RATE_HZ))

    result = logger.record(imu, cal, seconds=0, path=slot_path("new"), max_bytes=budget, countdown=0)

    complete = result["reason"] in ("size", "flash")    # Ran until its share (or the flash) was full
    state["new"] = {"seq": seq, "complete": complete, "rows": result["rows"],
                    "seconds": result["seconds"], "reason": result["reason"]}
    save_state(state)
    print("  slot_new %s" % ("COMPLETE: data saved" if complete else "stopped early (data kept)"))
    return complete


def wait_for_laptop(seconds):
    """Fast LED blink for `seconds`. True if a laptop interrupted (Ctrl+C from mpremote) or Enter came in."""
    import select
    import sys
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    end = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
    print("Recording starts in %d s (slot_new -> slot_old, then slot_new is rewritten). Connect from the laptop "
          "(or press Enter) now to open the menu instead; nothing is changed." % seconds)
    try:
        while time.ticks_diff(end, time.ticks_ms()) > 0:
            util.led.toggle()
            if poll.poll(100):                          # 100 ms per blink = 5 Hz flashing
                # Only a real key press counts (Enter, or a short command). A serial port opened
                # with echo on (e.g. `cat /dev/ttyACM0`) sends our own long messages straight back;
                # that text is ignored so it cannot stop the recording. mpremote sends Ctrl+C,
                # which arrives as KeyboardInterrupt below.
                got = sys.stdin.read(1)
                while poll.poll(20):
                    got += sys.stdin.read(1)
                    if len(got) > 8:
                        break
                if len(got) <= 4 and ("\r" in got or "\n" in got):
                    util.led.off()
                    return True
    except KeyboardInterrupt:
        util.led.off()
        return True
    util.led.off()
    return False


def finished_light():
    """LED on solid: recording finished and saved."""
    util.led.on()


def error_light(times=20):
    """Three quick flashes, repeated: the IMU could not be started."""
    for _ in range(times):
        for _ in range(3):
            util.led.on()
            time.sleep_ms(100)
            util.led.off()
            time.sleep_ms(100)
        time.sleep_ms(600)
