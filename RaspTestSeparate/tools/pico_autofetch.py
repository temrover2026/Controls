#!/usr/bin/env python3
"""Laptop service: copy the Pico H recordings automatically the moment the Pico is plugged in.

Plugging the Pico into the laptop powers it on, and after a 10 s window it would rotate its slots
(new -> old) and start recording on the desk. This watcher sees the Pico appear on USB (MicroPython
board, USB id 2e8a:0005), connects within about a second - inside that window, so the Pico skips
recording and nothing rotates - then runs fetch_pico_slots.py: both slots are copied to
~/imu_data/pico_<date_time>_slots/, the Kalman and sensor plots are made and opened, and a desktop
notification says where they are.

Runs as a systemd user service (installed by tools/install_autofetch.sh):
    systemctl --user status pico-autofetch      # is it running
    journalctl --user -u pico-autofetch -f      # its log
    touch ~/.cache/pico_autofetch_pause         # pause it (e.g. while testing on the bench)
    rm ~/.cache/pico_autofetch_pause            # resume
"""

import glob
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PAUSE_FILE = os.path.expanduser("~/.cache/pico_autofetch_pause")
VENDOR, PRODUCT = "2e8a", "0005"        # Raspberry Pi RP2040 running MicroPython


def find_pico():
    """(usb device id, /dev/ttyACMn) of a connected MicroPython Pico, or None."""
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(dev + "/idVendor") as f:
                vendor = f.read().strip()
            with open(dev + "/idProduct") as f:
                product = f.read().strip()
        except OSError:
            continue
        if (vendor, product) != (VENDOR, PRODUCT):
            continue
        ttys = glob.glob(dev + "/*:1.0/tty/ttyACM*")
        if ttys:
            with open(dev + "/busnum") as f, open(dev + "/devnum") as g:
                ident = "%s-%s" % (f.read().strip(), g.read().strip())   # Changes on every plug-in
            return ident, "/dev/" + os.path.basename(ttys[0])
    return None


def hold_pico(port, seconds=8.0):
    """Stop the Pico's power-on recording before its 10 s window ends.

    Writes Ctrl+C straight to the serial port, again and again for a moment, as soon as the port can
    be opened. main.py is then interrupted inside its start-up window (dualslot.wait_for_laptop), so
    the slots are NOT rotated and nothing is recorded. Starting mpremote first was too slow: once
    its first connection failed, the window had passed and the Pico rotated new -> old.
    Returns True if the port could be written to."""
    import termios
    end = time.time() + seconds
    while time.time() < end:
        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError:
            time.sleep(0.05)                            # Port not ready yet (permissions settling)
            continue
        try:
            attrs = termios.tcgetattr(fd)
            attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHONL | termios.ISIG)
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            for _ in range(15):                         # 1.5 s of Ctrl+C, covers a slow boot
                try:
                    os.write(fd, b"\x03")
                except OSError:
                    pass
                time.sleep(0.1)
            return True
        finally:
            os.close(fd)
    return False


def notify(title, text):
    subprocess.run(["notify-send", "-a", "Pico IMU", title, text], env=desktop_env(),
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def desktop_env():
    uid = os.getuid()
    env = dict(os.environ)
    env.setdefault("DISPLAY", ":0")
    env.setdefault("WAYLAND_DISPLAY", "wayland-0")
    env.setdefault("XDG_RUNTIME_DIR", "/run/user/%d" % uid)
    env.setdefault("DBUS_SESSION_BUS_ADDRESS", "unix:path=/run/user/%d/bus" % uid)
    env["PATH"] = os.path.expanduser("~/.local/bin") + ":" + env.get("PATH", "/usr/bin:/bin")
    return env


def main():
    seen = None
    print("watching for the Pico (USB %s:%s)" % (VENDOR, PRODUCT), flush=True)
    while True:
        found = find_pico()
        if found and found[0] != seen:
            seen = found[0]
            if os.path.exists(PAUSE_FILE):
                print("Pico plugged in on %s; paused (%s exists), not fetching" % (found[1], PAUSE_FILE), flush=True)
            else:
                held = hold_pico(found[1])              # First: stop the power-on recording
                print("Pico plugged in on %s: %s, fetching" % (found[1], "stopped its start-up recording" if held
                      else "could NOT open the port in time"), flush=True)
                result = subprocess.run([sys.executable, os.path.join(HERE, "fetch_pico_slots.py"), "--port", found[1]],
                                        capture_output=True, text=True, env=desktop_env(), timeout=600)
                print(result.stdout + result.stderr, flush=True)
                lines = [l for l in result.stdout.splitlines() if l.startswith(("slot ", "folder:"))]
                notify("Pico IMU data copied" if result.returncode == 0 else "Pico IMU fetch failed",
                       "\n".join(lines) or result.stderr[-300:])
        elif not found:
            seen = None
        time.sleep(0.3)


if __name__ == "__main__":
    main()
