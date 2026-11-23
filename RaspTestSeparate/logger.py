# logger.py - live view and recording of calibrated IMU data on the Pico H.
#
# live_view(): prints raw and calibrated values a few times a second (like Controls
#              imu_raw_live / imu_cal_live printing), nothing stored.
# record()   : stores calibrated samples to /imu_data/imu_log_NNN.csv on the Pico's flash.
#
# Recorded columns (units as the Controls ROS topics /imu/gyr_cal and /imu/mag_cal):
#   t              s       time since the recording started
#   accel_x,y,z    m/s^2   accelerometer (not corrected, as in Controls)
#   gyro_x,y,z     rad/s   calibrated gyro  (raw - bias)
#   mag_x,y,z      T       calibrated magnetometer (matrix x (raw - offset))
#   + gyro_raw_x,y,z, mag_raw_x,y,z when config.LOG_INCLUDE_RAW is True
#
# Copy a log to the laptop afterwards:   mpremote cp :/imu_data/imu_log_001.csv .

import time

import config
import util

COLUMNS = ["t", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z", "mag_x", "mag_y", "mag_z"]
RAW_COLUMNS = ["gyro_raw_x", "gyro_raw_y", "gyro_raw_z", "mag_raw_x", "mag_raw_y", "mag_raw_z"]


def live_view(imu, cal):
    """Print raw and calibrated readings until Ctrl+C."""
    print("\nLIVE VIEW (Ctrl+C to stop). gyro in deg/s, field in uT for easy reading.")
    rate = util.Rate(config.LIVE_PRINT_HZ)
    try:
        while True:
            try:
                accel, gyro, mag = imu.read_si()
            except OSError as exc:
                print("  I2C read failed (%s), skipped" % exc)
                rate.sleep()
                continue
            g, m = cal.apply_gyro(gyro), cal.apply_mag(mag)
            print("accel %6.2f %6.2f %6.2f m/s2 | gyro raw %6.2f %6.2f %6.2f cal %6.2f %6.2f %6.2f deg/s"
                  " | mag raw %6.1f %6.1f %6.1f cal %6.1f %6.1f %6.1f uT"
                  % (accel + tuple(v * 57.29578 for v in gyro + g) + tuple(v * 1e6 for v in mag + m)))
            rate.sleep()
    except KeyboardInterrupt:
        print("  live view stopped")


def record(imu, cal, seconds=None, hz=None, stream=False, path=None, max_bytes=None, countdown=None):
    """Record calibrated data to a CSV on flash.

    Stops on Ctrl+C, after `seconds`, when the file reaches `max_bytes`, or when the flash is nearly
    full. `path` defaults to the next /imu_data/imu_log_NNN.csv; dualslot.py passes its slot file.
    Returns a dict: path, rows, seconds, bytes and reason ("time", "size", "flash" or "stopped").

    stream=True also prints every row over USB as
    "D,t,ax,ay,az,gx,gy,gz,mx,my,mz,gx_raw,gy_raw,gz_raw,mx_raw,my_raw,mz_raw"
    (calibrated gyro/mag first, raw after) for the laptop's live heading plot (tools/live_heading.py).
    """
    if seconds is None:
        seconds = config.LOG_SECONDS
    hz = hz or config.LOG_RATE_HZ
    if not cal.complete:
        print("  ! Calibration incomplete: uncorrected values are stored for the missing sensor.")

    path = path or util.next_numbered_file(config.DATA_DIR, "imu_log")
    columns = COLUMNS + (RAW_COLUMNS if config.LOG_INCLUDE_RAW else [])
    row_bytes = 12 * len(columns)                       # Rough size of one line
    free = util.free_bytes() - config.MIN_FREE_BYTES
    minutes = free / (row_bytes * hz) / 60
    print("\nRECORDING to %s at %d Hz. Flash left: %d KB = about %.0f min. Ctrl+C to stop."
          % (path, hz, free // 1024, minutes))
    util.countdown(config.COUNTDOWN_SECONDS if countdown is None else countdown, "Recording starts")

    rows = errors = 0
    reason = "stopped"
    t = 0.0
    start = time.ticks_us()
    last_flush = last_space_check = 0.0
    rate = util.Rate(hz)
    f = open(path, "w")
    header = ",".join(columns) + "\n"
    f.write(header)
    written = len(header)                               # Bytes in the file so far
    try:
        while True:
            t = util.seconds_since(start)
            if seconds and t >= seconds:
                reason = "time"
                break
            if max_bytes and written >= max_bytes:
                reason = "size"
                break
            try:
                accel, gyro, mag = imu.read_si()
            except OSError:
                errors += 1                            # Skip this sample, as imu_raw_live does
                rate.sleep()
                continue
            values = (t,) + accel + cal.apply_gyro(gyro) + cal.apply_mag(mag)
            if stream:
                print("D," + util.csv_line(values + gyro + mag), end="")
            if config.LOG_INCLUDE_RAW:
                values += gyro + mag
            line = util.csv_line(values)
            f.write(line)
            written += len(line)
            rows += 1

            # Flush regularly: after a power cut you lose at most FLUSH_EVERY_S of data
            if t - last_flush >= config.FLUSH_EVERY_S:
                f.flush()
                last_flush = t
                util.led.toggle()                      # Slow blink = recording
            # Checking free space is slow-ish, so do it every 5 s
            if t - last_space_check >= 5.0:
                last_space_check = t
                if util.free_bytes() < config.MIN_FREE_BYTES:
                    print("  ! Flash nearly full: recording stopped")
                    reason = "flash"
                    break
                print("  %5.0f s  %6d rows  last gyro z %+.3f rad/s" % (t, rows, values[6]))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        util.led.off()
    print("  Saved %d rows, %.0f s, %d KB (%d I2C errors skipped, IMU set up again %d times) to %s; stopped by: %s"
          % (rows, t, written // 1024, errors, imu.resets, path, reason))
    return {"path": path, "rows": rows, "seconds": t, "bytes": written, "reason": reason}
