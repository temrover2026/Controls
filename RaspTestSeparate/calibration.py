# calibration.py - create, save, load and apply the IMU calibration on the Pico H.
#
# Pico H version of these Controls (Capstone-3) files:
#   imu_cal/cal_gyro_record.py   -> record_gyro_bias()       keep still, average the gyro = bias
#   imu_cal/cal_mag_record.py  +
#   imu_cal/calibrate_mag.py     -> record_mag_calibration() rotate in 3D, ellipsoid fit (magfit.py)
#   imu_processing/imu_cal_live.py -> Calibration.apply_*()  correct each new sample
#
# The files use the SAME names, columns and units as Controls (~/imu_calibration on the
# LattePanda), so they can be copied between the Pico and the rover either way:
#   bias_file_gyro.csv : gyro_bias_x,y,z (rad/s), accel_average_x,y,z (m/s^2)
#   mag_calibration.csv: offset_x,y,z (T), matrix_00 ... matrix_22
#
# As in Controls, only the gyro and magnetometer are corrected; the accelerometer is left as
# measured.

import gc
import time
from array import array

import config
import magfit
import util


# ============================================================================ apply (imu_cal_live)
class Calibration:
    """Loaded calibration. Missing files leave that sensor uncorrected (with a warning)."""

    def __init__(self):
        self.gyro_bias = None       # (bx, by, bz) rad/s
        self.mag_offset = None      # (ox, oy, oz) T
        self.mag_matrix = None      # 3x3 tuple
        self.load()

    def load(self):
        self.gyro_bias = self.mag_offset = self.mag_matrix = None
        try:
            row = util.read_csv_row(config.GYRO_BIAS_FILE)
            self.gyro_bias = tuple(float(row["gyro_bias_" + a]) for a in "xyz")
        except (OSError, KeyError, ValueError) as exc:
            print("  ! no gyro calibration (%s): run menu option 2" % exc)
        try:
            row = util.read_csv_row(config.MAG_CAL_FILE)
            self.mag_offset = tuple(float(row["offset_" + a]) for a in "xyz")
            self.mag_matrix = tuple(tuple(float(row["matrix_%d%d" % (r, c)]) for c in range(3)) for r in range(3))
        except (OSError, KeyError, ValueError) as exc:
            print("  ! no magnetometer calibration (%s): run menu option 3" % exc)

    @property
    def complete(self):
        return self.gyro_bias is not None and self.mag_matrix is not None

    def apply_gyro(self, gyro):
        """Calibrated gyro = raw - bias (rad/s)."""
        if self.gyro_bias is None:
            return gyro
        b = self.gyro_bias
        return (gyro[0] - b[0], gyro[1] - b[1], gyro[2] - b[2])

    def apply_mag(self, mag):
        """Calibrated field = matrix x (raw - offset) (tesla). Centre first, then the soft-iron matrix."""
        if self.mag_matrix is None:
            return mag
        o, m = self.mag_offset, self.mag_matrix
        x, y, z = mag[0] - o[0], mag[1] - o[1], mag[2] - o[2]
        return (m[0][0] * x + m[0][1] * y + m[0][2] * z,
                m[1][0] * x + m[1][1] * y + m[1][2] * z,
                m[2][0] * x + m[2][1] * y + m[2][2] * z)


# ============================================================================ gyro (cal_gyro_record)
def record_gyro_bias(imu, stream=False):
    """Keep the IMU still for GYRO_CAL_SECONDS; the average gyro reading is its bias.

    Writes gyr_cal_record.csv (every sample) and, if at least GYRO_MIN_SAMPLES were collected,
    bias_file_gyro.csv. Returns the bias tuple or None.
    stream=True also prints every sample as "G,t,gx,gy,gz,ax,ay,az" over USB, for the laptop's live
    plot (tools/live_calibrate.py).
    """
    util.ensure_dir(config.CAL_DIR)
    seconds, hz = config.GYRO_CAL_SECONDS, config.GYRO_CAL_RATE_HZ
    print("\nGYRO CALIBRATION: put the rover/IMU down and do not touch it for %d s." % seconds)
    util.countdown(config.COUNTDOWN_SECONDS, "Recording starts")

    record = None
    if config.GYRO_SAVE_RECORD:
        record = open(config.GYRO_RECORD_FILE, "w")
        record.write("time,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n")

    # Running totals, exactly like Controls (no need to keep the samples in RAM)
    g_tot = [0.0, 0.0, 0.0]
    a_tot = [0.0, 0.0, 0.0]
    count = errors = 0
    start = time.ticks_us()
    rate = util.Rate(hz)
    last_flush = 0.0
    try:
        while True:
            t = util.seconds_since(start)
            if t >= seconds:
                break
            try:
                accel, gyro, _ = imu.read_si()
            except OSError:
                errors += 1                            # Skip a failed I2C read
                rate.sleep()
                continue
            for k in range(3):
                g_tot[k] += gyro[k]
                a_tot[k] += accel[k]
            count += 1
            if stream:
                print("G," + util.csv_line((t,) + gyro + accel), end="")
            if record is not None:
                record.write(util.csv_line((t,) + gyro + accel))
                if t - last_flush >= config.FLUSH_EVERY_S:
                    record.flush()
                    last_flush = t
            if count % hz == 0:
                util.led.toggle()
                print("  %2d s left  (%d samples)" % (seconds - int(t), count))
            rate.sleep()
    except KeyboardInterrupt:
        print("  stopped early")
    finally:
        if record is not None:
            record.close()
        util.led.off()

    if count == 0:
        print("  No IMU data was received.")
        return None
    bias = tuple(g / count for g in g_tot)
    accel_avg = tuple(a / count for a in a_tot)
    print("========== CALIBRATION RESULTS ==========")
    print("Samples collected: %d   (I2C errors skipped: %d)" % (count, errors))
    for axis, value in zip("XYZ", bias):
        print("Gyro bias %s: %.6f rad/s" % (axis, value))
    for axis, value in zip("XYZ", accel_avg):
        print("Accelerometer average %s: %.4f m/s^2" % (axis, value))
    print("==========================================")

    # Same safety rule as Controls: a short run never overwrites a good calibration
    if count < config.GYRO_MIN_SAMPLES:
        print("  ! Only %d samples (minimum %d). bias_file_gyro.csv was NOT updated." % (count, config.GYRO_MIN_SAMPLES))
        return None
    with open(config.GYRO_BIAS_FILE, "w") as f:
        f.write("gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_average_x,accel_average_y,accel_average_z\n")
        f.write(util.csv_line(bias + accel_avg))
    print("  Calibration values saved to " + config.GYRO_BIAS_FILE)
    return bias


# ============================================================================ magnetometer (cal_mag_record + calibrate_mag)
def record_mag_calibration(imu, stream=False):
    """Rotate the IMU slowly through every orientation for MAG_CAL_SECONDS, then fit and save.

    Samples are kept in compact float arrays (4 bytes each: 1500 samples x 3 = 18 KB of the
    Pico's 264 KB RAM) and also written to mag_record.csv, which Controls calibrate_mag.py can
    refit on a laptop:  ros2 run imu_processing calibrate_mag mag_record.csv
    Returns True if a new mag_calibration.csv was saved.
    stream=True also prints every sample as "M,t,mx,my,mz" over USB for the laptop's live plot.
    """
    util.ensure_dir(config.CAL_DIR)
    seconds, hz = config.MAG_CAL_SECONDS, config.MAG_CAL_RATE_HZ
    print("\nMAGNETOMETER CALIBRATION: for %d s, slowly turn the IMU through EVERY orientation:" % seconds)
    print("  spin it flat, then on each side, then upside down - like painting the inside of a ball.")
    print("  Keep it away from laptops, motors and steel while you do it.")
    util.countdown(config.COUNTDOWN_SECONDS, "Start rotating")

    capacity = seconds * hz + hz                       # Room for every sample at this rate
    gc.collect()
    xs, ys, zs = array("f"), array("f"), array("f")
    record = open(config.MAG_RECORD_FILE, "w")
    record.write("t_stamp,mag_x,mag_y,mag_z\n")         # Columns calibrate_mag.py looks for
    errors = 0
    start = time.ticks_us()
    rate = util.Rate(hz)
    try:
        while True:
            t = util.seconds_since(start)
            if t >= seconds or len(xs) >= capacity:
                break
            try:
                _, _, mag = imu.read_si()
            except OSError:
                errors += 1
                rate.sleep()
                continue
            xs.append(mag[0])
            ys.append(mag[1])
            zs.append(mag[2])
            record.write(util.csv_line((t,) + mag))
            if stream:
                print("M," + util.csv_line((t,) + mag), end="")
            if len(xs) % hz == 0:
                util.led.toggle()
                record.flush()
                print("  %2d s left   field %5.1f uT   (%d samples)"
                      % (seconds - int(t), (mag[0] ** 2 + mag[1] ** 2 + mag[2] ** 2) ** 0.5 * 1e6, len(xs)))
            rate.sleep()
    except KeyboardInterrupt:
        print("  stopped early")
    finally:
        record.close()
        util.led.off()

    print("  %d samples recorded (I2C errors skipped: %d). Fitting ..." % (len(xs), errors))
    if imu.mag_overflows:
        print("  ! %d readings were out of range: a magnet was too close." % imu.mag_overflows)
    # A loose wire drops the IMU whenever it is moved: the fit would then come from a few scattered
    # points. More than 10 % failed reads -> keep the existing calibration.
    if errors > 0.1 * (len(xs) + errors):
        print("  ! Calibration NOT saved: %d of %d reads failed while rotating - a wire to the IMU is "
              "loose. Fix the wiring and record again." % (errors, len(xs) + errors))
        return False
    return fit_and_save(xs, ys, zs)


def refit_mag_from_record(path=None):
    """Fit again from a saved mag_record.csv (menu 8): no need to rotate the IMU again.

    Useful when a fit failed (or after changing MAG_MAX_SPREAD); the recording stays on the Pico.
    """
    path = path or config.MAG_RECORD_FILE
    xs, ys, zs = array("f"), array("f"), array("f")
    try:
        with open(path) as f:
            f.readline()                                # Header: t_stamp,mag_x,mag_y,mag_z
            for line in f:
                parts = line.split(",")
                if len(parts) >= 4:
                    xs.append(float(parts[1]))
                    ys.append(float(parts[2]))
                    zs.append(float(parts[3]))
    except OSError as exc:
        print("  ! Cannot read %s (%s): record a calibration first (menu 3)" % (path, exc))
        return False
    print("\nRefitting %d samples from %s ..." % (len(xs), path))
    return fit_and_save(xs, ys, zs)


def fit_and_save(xs, ys, zs):
    """Ellipsoid fit (magfit.py) of the samples; on success write mag_calibration.csv (old one -> .bak)."""
    gc.collect()                                        # Free RAM left over from recording first
    total = len(xs)
    xs, ys, zs = magfit.thin(xs, ys, zs, config.MAG_MIN_STEP_UT * 1e-6)
    gc.collect()
    print("  using %d of %d samples (the rest repeat a spot within %.0f uT, e.g. while held still)"
          % (len(xs), total, config.MAG_MIN_STEP_UT))
    try:
        offsets, matrix, report = magfit.fit_calibration(xs, ys, zs, config.MAG_MAX_SPREAD, config.MAG_MIN_SAMPLES)
    except ValueError as exc:
        print("  ! Calibration NOT saved: %s" % exc)
        return False

    # Keep the previous calibration as .bak before overwriting it
    if util.exists(config.MAG_CAL_FILE):
        with open(config.MAG_CAL_FILE) as old, open(config.MAG_CAL_FILE + ".bak", "w") as bak:
            bak.write(old.read())
    header = ["offset_x", "offset_y", "offset_z"] + ["matrix_%d%d" % (r, c) for r in range(3) for c in range(3)]
    values = list(offsets) + [matrix[r][c] for r in range(3) for c in range(3)]
    with open(config.MAG_CAL_FILE, "w") as f:
        f.write(",".join(header) + "\n")
        f.write(util.csv_line(values))

    print("========== MAGNETOMETER CALIBRATION ==========")
    print("Hard-iron offset : %.1f, %.1f, %.1f uT" % tuple(o * 1e6 for o in offsets))
    print("Ellipsoid axes   : %.1f, %.1f, %.1f uT (equal = no soft iron)" % tuple(report["semi_axes_uT"]))
    print("Calibrated field : %.1f uT, spread %.1f %% (limit %.0f %%)"
          % (report["field_uT"], report["spread"] * 100, config.MAG_MAX_SPREAD * 100))
    print("Saved to " + config.MAG_CAL_FILE)
    print("==============================================")
    return True
