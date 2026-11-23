# config.py - every setting for the Pico H IMU logger in one place.
#
# Board : Raspberry Pi Pico H (RP2040, 264 KB RAM, 2 MB flash, no Wi-Fi), running MicroPython.
# Sensor: Pimoroni ICM-20948 9-DoF breakout (accelerometer + gyro + AK09916 magnetometer).
#
# Change values here rather than inside the other files, then copy this file to the Pico again:
#     mpremote cp config.py :config.py

# ---------------------------------------------------------------------------------------------
# Wiring (Pico H pin numbers are the GPxx labels, not the physical pin numbers)
#
#   Pimoroni breakout   ->  Pico H
#   3-6V (or 3V3)       ->  3V3(OUT)  physical pin 36   (use 3.3 V, as on the LattePanda)
#   GND                 ->  GND       physical pin 38
#   SDA                 ->  GP2       physical pin 4    (I2C1 SDA)
#   SCL                 ->  GP3       physical pin 5    (I2C1 SCL)
#   INT                 ->  not used
# ---------------------------------------------------------------------------------------------
I2C_ID = 1              # Pico hardware I2C block 1 on GP2/GP3 (how this Pico H is wired)
I2C_SDA_PIN = 2         # Other pairs work too, e.g. block 0 on GP4/GP5: menu 7 checks the wiring
I2C_SCL_PIN = 3
I2C_FREQ = 400000      # 400 kHz fast mode: the ICM-20948 supports it and the Pimoroni board has pull-ups
IMU_ADDR = 0x68         # Same address as the Controls version (0x69 if the ADDR trace is cut)

# Work-around for a GND wire on a GPIO pin. On the bench Pico H the IMU's GND wire sits on GP22
# (physical pin 29) instead of GND (physical pin 28 next to it), so the IMU only got ground through
# the I2C lines and stopped answering for ~12 ms after every pause. Setting a pin number here
# drives that pin LOW so it acts as ground (the IMU draws ~3 mA; a GPIO can sink 12 mA).
# Move the wire to pin 28 and set this back to None.
IMU_GROUND_PIN = None

# Onboard LED of the Pico H (GP25). Blinks while recording, solid during countdowns.
LED_PIN = 25

# ---------------------------------------------------------------------------------------------
# Magnetometer reading mode
#   "continuous": the AK09916 measures at 100 Hz by itself and the ICM-20948 copies each result
#                 into its own registers, so one I2C read returns accel + gyro + mag together.
#                 Fast (about 1 ms per sample). Recommended.
#   "single"    : exactly what the Pimoroni Python library (used by Controls) does: trigger one
#                 measurement, wait for it, read it. About 20-30 ms per sample, so at most ~35 Hz.
#                 Use this if "continuous" ever gives stuck or zero magnetometer values.
# Both give identical units and scaling.
# ---------------------------------------------------------------------------------------------
MAG_MODE = "continuous"

# ---------------------------------------------------------------------------------------------
# Where files go on the Pico's flash drive (same file names as Controls ~/imu_calibration)
# ---------------------------------------------------------------------------------------------
CAL_DIR = "/imu_calibration"
DATA_DIR = "/imu_data"
GYRO_BIAS_FILE = CAL_DIR + "/bias_file_gyro.csv"     # read/written in the Controls format
GYRO_RECORD_FILE = CAL_DIR + "/gyr_cal_record.csv"   # every stationary sample (Controls format)
MAG_CAL_FILE = CAL_DIR + "/mag_calibration.csv"      # offsets + 3x3 matrix (Controls format)
MAG_RECORD_FILE = CAL_DIR + "/mag_record.csv"        # samples used for the fit (calibrate_mag.py can read it)

# Stop writing when less than this much flash is left, so the filesystem never fills completely.
MIN_FREE_BYTES = 64 * 1024

# ---------------------------------------------------------------------------------------------
# Gyro calibration (keep the IMU completely still)
# ---------------------------------------------------------------------------------------------
GYRO_CAL_SECONDS = 30       # 30 s at 100 Hz = 3000 samples
GYRO_CAL_RATE_HZ = 100      # Controls publishes raw IMU data at 100 Hz
GYRO_MIN_SAMPLES = 1000     # Same rule as Controls: a shorter run never overwrites the bias file
GYRO_SAVE_RECORD = True     # Also store every sample (about 200 KB for 30 s)

# ---------------------------------------------------------------------------------------------
# Magnetometer calibration (rotate the IMU slowly through every orientation in 3D)
# ---------------------------------------------------------------------------------------------
MAG_CAL_SECONDS = 60
MAG_CAL_RATE_HZ = 25        # 60 s x 25 Hz = 1500 samples; fits in RAM and is plenty for the fit
MAG_MIN_SAMPLES = 30        # Same minimum as Controls calibrate_mag.py
MAG_MAX_SPREAD = 0.15       # Same quality limit as Controls: std/mean of the corrected field length
MAG_MIN_STEP_UT = 3.0       # Fit only uses a sample 3 uT or more from the last one used, so time spent
                            # holding still does not outweigh the rotation (noise is about 1 uT)

# Countdown before any calibration or recording starts, so you can get the rover ready
COUNTDOWN_SECONDS = 5

# ---------------------------------------------------------------------------------------------
# Data recording (menu option 4)
#   A calibrated row is about 115 bytes. At 10 Hz that is ~1.2 KB/s, so the ~1.2 MB free on a
#   Pico H holds roughly 17 minutes (20 Hz ~8 min, 50 Hz ~3.5 min). Raw columns add about 60 %.
# ---------------------------------------------------------------------------------------------
LOG_RATE_HZ = 10
LOG_SECONDS = 0             # 0 = record until Ctrl+C or the flash is full
LOG_INCLUDE_RAW = False     # True also stores raw gyro and raw magnetometer columns
FLUSH_EVERY_S = 1.0         # Data is safe on flash at most this long after it was measured

# ---------------------------------------------------------------------------------------------
# What happens at power-on (main.py)
#   "dual_slot": (dualslot.py) 1. slot_new -> slot_old (the previous recording becomes "old"),
#               2. slot_new is rewritten with live data at LOG_RATE_HZ until it holds SLOT_FRACTION
#               of the space, then it stops and saves (LED solid). During the first AUTO_START_WAIT_S seconds the LED blinks
#               fast: connect from the laptop then to get the menu without changing anything.
#   "menu"     : show the menu and wait for a choice over USB (bench use)
#   "record"   : if nothing is typed within AUTO_START_WAIT_S seconds, record a numbered log until
#               the flash is full
# ---------------------------------------------------------------------------------------------
AUTO_MODE = "dual_slot"
AUTO_START_WAIT_S = 10
SLOT_FRACTION = 0.5         # Each slot may use half of the space for data (two data sets fit)

# Live view print rate (menu option 1)
LIVE_PRINT_HZ = 2
