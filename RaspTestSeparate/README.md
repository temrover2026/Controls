# RaspTestSeparate: IMU logger on a Raspberry Pi Pico H

This folder is a standalone version of the `Capstone-3` IMU pipeline. It runs on a **Raspberry Pi Pico H**
with **MicroPython** and needs no ROS or LattePanda. It reads the Pimoroni ICM-20948, calibrates it and
stores the data on the Pico's own flash.

| Controls `Capstone-3` (ROS 2, LattePanda)  | Here (MicroPython, Pico H)                              |
|--------------------------------------------|---------------------------------------------------------|
| `imu_raw_live`: read the sensor            | `icm20948.py`: `ICM20948.read_si()`                     |
| `cal_gyro_record`: gyro bias               | `calibration.py`: `record_gyro_bias()`, menu 2          |
| `cal_mag_record` + `calibrate_mag`         | `calibration.py`: `record_mag_calibration()` + `magfit.py`, menu 3 |
| `imu_cal_live`: apply the calibration      | `calibration.py`: `Calibration.apply_gyro/apply_mag()`  |
| ROS bag / CSV recording                    | `logger.py`: `record()`, menu 4                         |

**Same as Controls:**
- **Sensor settings:** gyro at 100 Hz, ±250 °/s, low-pass filter 5; accelerometer at 125 Hz, ±16 g, low-pass filter 5; magnetometer at 0.15 µT per count.
- **Units:** acceleration in m/s², rotation rate in rad/s, magnetic field in tesla.
- **I2C address:** 0x68.
- **Calibration files:** `bias_file_gyro.csv` and `mag_calibration.csv` have the same names and columns, so you can copy them between the Pico and the rover in either direction.

## Files

| File             | What it does |
|------------------|--------------|
| `main.py`        | Runs at power-on and shows the menu |
| `config.py`      | **All settings**: pins, rates, durations, file names, start-up mode |
| `icm20948.py`    | ICM-20948 / AK09916 driver, ported from the Pimoroni library to `machine.I2C` |
| `calibration.py` | Gyro bias, magnetometer calibration, and loading/applying the calibration |
| `magfit.py`      | Ellipsoid fit for the magnetometer, without numpy |
| `logger.py`      | Live view and CSV recording |
| `util.py`        | File, LED, countdown and loop-timing helpers |

## Wiring

| Pimoroni ICM-20948 | Pico H                   |
|--------------------|--------------------------|
| 3-6V               | **3V3(OUT)**, pin 36     |
| GND                | GND, pin 38              |
| SDA                | **GP2**, pin 4 (I2C1)    |
| SCL                | **GP3**, pin 5 (I2C1)    |

Use 3.3 V, as on the LattePanda. The Pimoroni board already has I2C pull-up resistors.

## 1. Put MicroPython on the Pico H (once)

1. Download the **Raspberry Pi Pico** `.uf2` file from https://micropython.org/download/RPI_PICO/. The Pico H uses the plain Pico build, not the Pico W one.
2. Hold the **BOOTSEL** button, plug the Pico into USB, then let go. A drive called `RPI-RP2` appears.
3. Copy the `.uf2` file onto that drive. The Pico restarts as a USB serial port (`/dev/ttyACM0`).

## 2. Copy the code onto the Pico

```bash
pip install mpremote
cd ~/workspace/Controls/RaspTestSeparate
mpremote cp config.py icm20948.py magfit.py util.py calibration.py logger.py main.py :
mpremote reset
mpremote repl            # shows the menu; Ctrl+] leaves, Ctrl+D restarts main.py
```

If `mpremote` can't open `/dev/ttyACM0`, add yourself to the serial group with `sudo usermod -aG dialout $USER`, then log out and back in.

## 3. Use it (menu)

```
 1  live view (raw + calibrated)
 2  gyro calibration       (keep still, 30 s)
 3  magnetometer calibration (rotate in 3D, 60 s)
 4  record calibrated data to flash (10 Hz)
 5  list files and free space
 6  delete recorded logs (keeps calibration)
 7  I2C scan (wiring check)
```

Steps:
1. **Option 7:** check that `0x68` is listed.
2. **Option 2:** leave the IMU completely still. If fewer than 1000 samples are collected, the bias file is not changed (the same rule as Controls).
3. **Option 3:** turn the IMU slowly through *every* orientation. Spin it flat, then on each side, then upside down. A flat spin alone is refused, because the fit is 3D.
   - Your previous calibration is kept as `mag_calibration.csv.bak`.
   - The fit is centred on the mean of the samples first. Unlike Controls `calibrate_mag.py`, it still works when the rover's magnetic offset is as large as the Earth's field (about 40 µT was measured on the rover).
4. **Option 1:** check that the calibrated gyro reads about 0 when still, and that the calibrated field length stays roughly constant as you turn it.
5. **Option 4:** record data. Ctrl+C stops it; it also stops by itself when the flash is nearly full.

The LED stays on during a countdown, blinks while recording, and is off when idle.

## 4. Get the data onto the laptop

```bash
mpremote ls :/imu_data
mpremote cp :/imu_data/imu_log_001.csv .
mpremote cp :/imu_calibration/mag_calibration.csv :/imu_calibration/bias_file_gyro.csv .
mpremote rm :/imu_data/imu_log_001.csv              # free the space
```

To use the Pico's calibration on the rover, copy the two calibration files to `~/imu_calibration/` on the LattePanda.

### `imu_log_NNN.csv` columns

| Column           | Unit  | Notes |
|------------------|-------|-------|
| `t`              | s     | Time since the recording started. The Pico has no clock, so files are numbered rather than dated. |
| `accel_x,y,z`    | m/s²  | Not corrected, as in Controls |
| `gyro_x,y,z`     | rad/s | Calibrated: raw − bias |
| `mag_x,y,z`      | T     | Calibrated: matrix × (raw − offset). The length is about the local field. |
| `gyro_raw_*`, `mag_raw_*` | | Only when `LOG_INCLUDE_RAW = True` |

## Storage limits

MicroPython leaves about **1.4 MB** of the Pico H's 2 MB flash for files.

| Setting | Flash used | Recording time |
|---|---|---|
| 10 Hz, calibrated columns only (default) | about 1.2 KB/s | about 17 minutes |
| 20 Hz, calibrated columns only | about 2.3 KB/s | about 8–9 minutes |
| 50 Hz, raw columns too | about 9 KB/s | about 2–3 minutes |

Data reaches flash at least once a second (`FLUSH_EVERY_S`), so a power cut loses at most about 1 s.

For longer runs, either add a microSD card module (SPI), or plug the Pico into the LattePanda and record there.

## Settings worth knowing (`config.py`)

- **`MAG_MODE`**
  - `"continuous"` (default): the magnetometer measures at 100 Hz by itself, and each sample is one I2C read (about 1 ms).
  - `"single"`: does exactly what the Pimoroni library does. It's slower (at most about 35 Hz), but use it if continuous mode ever gives frozen magnetometer values.
- **`AUTO_MODE = "record"`:** at power-on, if nothing is typed within `AUTO_START_WAIT_S` seconds, the Pico starts recording by itself. Use this when it runs from a battery.
- **`LOG_RATE_HZ`, `LOG_SECONDS`, `LOG_INCLUDE_RAW`:** recording rate, length and columns.
- **`GYRO_CAL_SECONDS`, `MAG_CAL_SECONDS`, `COUNTDOWN_SECONDS`:** calibration timing.

## How it was checked

It has not run on a real Pico yet. It was tested on a laptop with a simulated ICM-20948 at register level:
- **Sensor setup:** register values match the Pimoroni library (GYRO_CONFIG_1 = 0x51, ACCEL_CONFIG = 0x57, gyro divider 10).
- **Gyro calibration:** recovers the bias exactly.
- **Magnetometer fit:** in 32-bit floats, matches Controls `calibrate_mag.py` to 0.008° in direction and 0.02 µT in offset.
- **Large offset:** still works with an 86 µT offset, where the numpy fit fails.
- **Flat-only data:** refused.
- **Recorded files:** `mag_record.csv` refits correctly with `calibrate_mag.py`.
