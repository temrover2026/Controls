# main.py - runs automatically every time the Pico H powers up (MicroPython runs main.py at boot).
#
# Pico H + Pimoroni ICM-20948: read the IMU, calibrate it and store the data on the Pico,
# doing on its own what Controls Capstone-3 does on the LattePanda with ROS 2:
#
#   Controls (ROS 2 on LattePanda)          This folder (MicroPython on Pico H)
#   imu_raw_live         read the sensor    icm20948.py   ICM20948.read_si()
#   cal_gyro_record      gyro bias          calibration.py record_gyro_bias()       menu 2
#   cal_mag_record +     magnetometer       calibration.py record_mag_calibration() menu 3
#     calibrate_mag        ellipsoid fit    magfit.py     fit_calibration()
#   imu_cal_live         apply calibration  calibration.py Calibration.apply_*()
#   ros2 bag / CSVs      store data         logger.py     record()                  menu 4
#
# Use it from the laptop over USB:   mpremote connect auto repl    (Ctrl+] to leave)
# then press Ctrl+D in the REPL to restart main.py and see the menu.

import os
import select
import sys
import time

from machine import I2C, Pin

import calibration
import config
import dualslot
import logger
import util
from icm20948 import ICM20948

MENU = """
========= Pico H IMU (ICM-20948) =========
 1  live view (raw + calibrated)
 2  gyro calibration       (keep still, %d s)
 3  magnetometer calibration (rotate in 3D, %d s)
 4  record calibrated data to flash (%d Hz)
 5  list files and free space
 6  delete recorded logs (keeps calibration)
 7  I2C scan (wiring check)
 8  refit magnetometer from the last recording (no rotating)
 9  slots: show slot_new / slot_old    10  slots: rotate and record now
 q  quit to the Python prompt
==========================================""" % (config.GYRO_CAL_SECONDS, config.MAG_CAL_SECONDS, config.LOG_RATE_HZ)


def make_i2c():
    if config.IMU_GROUND_PIN is not None:
        # See config.py: hold the pin the IMU's GND wire is on at 0 V, then let the IMU start
        Pin(config.IMU_GROUND_PIN, Pin.OUT, value=0)
        time.sleep_ms(50)
    return I2C(config.I2C_ID, sda=Pin(config.I2C_SDA_PIN), scl=Pin(config.I2C_SCL_PIN), freq=config.I2C_FREQ)


def i2c_scan(i2c):
    found = i2c.scan()
    print("  I2C devices: " + (", ".join("0x%02X" % a for a in found) or "none"))
    if config.IMU_ADDR not in found:
        print("  ! IMU not at 0x%02X: check 3V3, GND, SDA->GP%d, SCL->GP%d"
              % (config.IMU_ADDR, config.I2C_SDA_PIN, config.I2C_SCL_PIN))


def list_files():
    for folder in (config.CAL_DIR, config.DATA_DIR):
        print("  " + folder)
        try:
            names = sorted(os.listdir(folder))
        except OSError:
            names = []
        for name in names:
            print("    %-28s %8d bytes" % (name, os.stat(folder + "/" + name)[6]))
        if not names:
            print("    (empty)")
    print("  Free flash: %d KB" % (util.free_bytes() // 1024))


def delete_logs():
    try:
        names = os.listdir(config.DATA_DIR)
    except OSError:
        names = []
    if not names:
        print("  No logs to delete.")
        return
    if input("  Delete %d log file(s) in %s? type yes: " % (len(names), config.DATA_DIR)).strip() != "yes":
        print("  Nothing deleted.")
        return
    for name in names:
        os.remove(config.DATA_DIR + "/" + name)
    print("  Deleted. Free flash: %d KB" % (util.free_bytes() // 1024))


def key_pressed_within(seconds):
    """True if something arrives on the USB serial within `seconds` (used for AUTO_MODE)."""
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    return bool(poll.poll(int(seconds * 1000)))


def main():
    print("\nPico H IMU logger (%s) starting ..." % util.board_name())
    i2c = make_i2c()
    try:
        imu = ICM20948(i2c, config.IMU_ADDR, config.MAG_MODE)
    except (OSError, RuntimeError) as exc:
        print("  ! IMU start failed: %s" % exc)
        i2c_scan(i2c)
        if config.AUTO_MODE == "dual_slot":
            dualslot.error_light()                      # Three quick flashes, repeated
        return
    print("  ICM-20948 ready (magnetometer mode: %s)" % config.MAG_MODE)
    cal = calibration.Calibration()

    # Power-on recording into slot A / B until half the space is used (dualslot.py), unless a
    # laptop connects during the fast-blinking LED window
    if config.AUTO_MODE == "dual_slot":
        if not dualslot.wait_for_laptop(config.AUTO_START_WAIT_S):
            if dualslot.record_slot(imu, cal):
                dualslot.finished_light()               # LED on solid: done, data saved
            return
        print("  Laptop connected: dual-slot recording skipped, nothing deleted.")

    # Unattended start (battery, nothing plugged in): record unless a key is pressed in time
    if config.AUTO_MODE == "record":
        print("  Press Enter within %d s for the menu, otherwise recording starts." % config.AUTO_START_WAIT_S)
        if not key_pressed_within(config.AUTO_START_WAIT_S):
            logger.record(imu, cal)
            return
        sys.stdin.readline()                            # Throw away the key that was pressed

    show_menu = True
    while True:
        if show_menu:
            print(MENU)
        show_menu = True
        try:
            choice = input("choice: ").strip().lower()
        except KeyboardInterrupt:
            choice = "q"
        if not choice or len(choice) > 2:
            # Blank line, or echoed text from a serial port opened with echo on: ignore quietly
            # (printing the menu again would be echoed back again, forever)
            show_menu = False
            continue
        if choice == "1":
            logger.live_view(imu, cal)
        elif choice == "2":
            calibration.record_gyro_bias(imu)
            cal.load()                                  # Use the new calibration straight away
        elif choice == "3":
            calibration.record_mag_calibration(imu)
            cal.load()
        elif choice == "4":
            logger.record(imu, cal)
        elif choice == "5":
            list_files()
        elif choice == "6":
            delete_logs()
        elif choice == "7":
            i2c_scan(i2c)
        elif choice == "8":
            calibration.refit_mag_from_record()
            cal.load()
        elif choice == "9":
            dualslot.status()
        elif choice == "10":
            dualslot.record_slot(imu, cal)
        elif choice == "q":
            print("  Bye. Press Ctrl+D to restart the menu.")
            return
        else:
            print("  Unknown choice")


main()
