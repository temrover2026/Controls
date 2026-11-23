# icm20948.py - MicroPython driver for the Pimoroni ICM-20948 breakout on a Raspberry Pi Pico H.
#
# This is a port of the Pimoroni `icm20948` Python library that the Controls code uses on the
# LattePanda (smbus2 -> machine.I2C). It sets the sensor up exactly the same way:
#
#   gyro : 100 Hz sample rate, low-pass filter mode 5, +-250 deg/s full scale
#   accel: 125 Hz sample rate, low-pass filter mode 5, +-16 g full scale
#   mag  : AK09916, 0.15 uT per count (fixed by the chip)
#
# and returns the same units as Controls imu_pubplot_func.read_imu_data() publishes:
#
#   accelerometer: m/s^2   (library g  x 9.80665)
#   gyroscope    : rad/s   (library deg/s converted to radians)
#   magnetometer : tesla   (library uT x 1e-6)
#
# Background - how the magnetometer is reached
#   The ICM-20948 package holds two chips: the ICM-20948 itself (accel + gyro, I2C address 0x68)
#   and an AK09916 magnetometer on a private "auxiliary" I2C bus inside the package. The Pico
#   cannot talk to the AK09916 directly; instead the ICM-20948's built-in I2C master ("SLV0")
#   reads or writes the AK09916 for us and puts the result in its EXT_SLV_SENS_DATA registers.
#
# The ICM-20948 registers are split into 4 banks (0-3); register 0x7F selects the bank.

import struct
import time
import math

# --- Chip identity ---------------------------------------------------------------------------
CHIP_ID = 0xEA              # WHO_AM_I of the ICM-20948
AK09916_ADDR = 0x0C         # Magnetometer address on the internal bus
AK09916_CHIP_ID = 0x09      # WIA register of the AK09916

# --- Register bank select (same address in every bank) ----------------------------------------
BANK_SEL = 0x7F

# --- Bank 0 ----------------------------------------------------------------------------------
WHO_AM_I = 0x00
USER_CTRL = 0x03            # bit 5 (0x20) = I2C_MST_EN: run the internal I2C master
PWR_MGMT_1 = 0x06           # 0x80 = reset, 0x01 = wake up with the best clock source
PWR_MGMT_2 = 0x07           # 0x00 = accelerometer and gyro all axes on
INT_PIN_CFG = 0x0F
ACCEL_XOUT_H = 0x2D         # Start of accel (6 bytes), gyro (6), temperature (2) ...
EXT_SLV_SENS_DATA_00 = 0x3B  # ... followed by the data the I2C master read from the AK09916

# --- Bank 2 ----------------------------------------------------------------------------------
GYRO_SMPLRT_DIV = 0x00
GYRO_CONFIG_1 = 0x01
ACCEL_SMPLRT_DIV_1 = 0x10
ACCEL_SMPLRT_DIV_2 = 0x11
ACCEL_CONFIG = 0x14

# --- Bank 3 (internal I2C master) ------------------------------------------------------------
I2C_MST_CTRL = 0x01
I2C_MST_DELAY_CTRL = 0x02
I2C_SLV0_ADDR = 0x03        # Address of the chip to talk to; | 0x80 means "read"
I2C_SLV0_REG = 0x04         # Register inside that chip
I2C_SLV0_CTRL = 0x05        # 0x80 = enable, low 4 bits = number of bytes to read
I2C_SLV0_DO = 0x06          # Byte to write

# --- AK09916 magnetometer registers -----------------------------------------------------------
AK_WIA = 0x01
AK_ST1 = 0x10               # bit 0 = data ready
AK_HXL = 0x11               # X low byte; X, Y, Z are little-endian int16, 6 bytes total
AK_ST2 = 0x18               # bit 3 = magnetic overflow. Reading ST2 releases the next sample.
AK_CNTL2 = 0x31             # Mode: 0x01 single, 0x08 continuous 100 Hz
AK_CNTL3 = 0x32             # 0x01 = soft reset

# --- Unit conversions (identical to Controls) ---------------------------------------------------
GRAVITY = 9.80665           # m/s^2 per g
MAG_UT_PER_COUNT = 0.15     # AK09916 datasheet section 3.3
DEG2RAD = math.pi / 180.0

# Scale tables from the ICM-20948 datasheet, sections 3.1 and 3.2
ACCEL_COUNTS_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYRO_COUNTS_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}


class ICM20948:
    """ICM-20948 on a MicroPython machine.I2C bus.

    imu = ICM20948(i2c)                 # set up exactly like the Pimoroni library
    accel, gyro, mag = imu.read_si()    # (x, y, z) tuples in m/s^2, rad/s, tesla
    """

    def __init__(self, i2c, addr=0x68, mag_mode="continuous",
                 gyro_full_scale=250, accel_full_scale=16):
        self._i2c = i2c
        self._addr = addr
        self._bank = -1                 # Unknown until the first bank() call
        self.mag_mode = mag_mode
        self.mag_overflows = 0          # Counts readings where the field was out of range
        self._buf1 = bytearray(1)       # Reused for single-byte writes (no allocation per write)
        self._gyro_full_scale = gyro_full_scale
        self._accel_full_scale = accel_full_scale
        self.resets = 0                 # Times the chip was found reset mid-run and set up again
        if mag_mode not in ("continuous", "single"):
            raise ValueError("mag_mode must be 'continuous' or 'single'")
        self.setup()

    def setup(self):
        """Configure the chip. Run at start, and again by read_si() if the chip resets itself
        (a power dip puts the ICM-20948 back in its power-on state: asleep, every reading 0)."""
        self._bank = -1
        addr = self._addr
        gyro_full_scale, accel_full_scale = self._gyro_full_scale, self._accel_full_scale

        # 1. Is there an ICM-20948 at this address?
        for attempt in range(10):          # Give a just-powered bus a few tries
            try:
                self.bank(0)
                who = self.read(WHO_AM_I)
                break
            except OSError:
                self._bank = -1
                if attempt == 9:
                    raise
                time.sleep_ms(10)
        if who != CHIP_ID:
            raise RuntimeError("No ICM-20948 at 0x%02X (WHO_AM_I=0x%02X, expected 0xEA)" % (addr, who))

        # 2. Reset the chip, then wake it up with the auto-selected clock and all axes on
        self.write(PWR_MGMT_1, 0x80)
        time.sleep_ms(10)
        self.write(PWR_MGMT_1, 0x01)
        self.write(PWR_MGMT_2, 0x00)

        # 3. Same sample rates, filters and ranges as the Pimoroni library
        self.set_gyro_sample_rate(100)
        self.set_gyro_low_pass(True, 5)
        self.set_gyro_full_scale(gyro_full_scale)
        self.set_accelerometer_sample_rate(125)
        self.set_accelerometer_low_pass(True, 5)
        self.set_accelerometer_full_scale(accel_full_scale)
        # Pimoroni reads the range register on every sample; we keep the divisors instead
        self._counts_per_g = ACCEL_COUNTS_PER_G[accel_full_scale]
        self._counts_per_dps = GYRO_COUNTS_PER_DPS[gyro_full_scale]

        # 4. Interrupt pin config (latched, cleared on any read) - same value as Pimoroni
        self.bank(0)
        self.write(INT_PIN_CFG, 0x30)

        # 5. Internal I2C master: same clock and delay settings as Pimoroni
        self.bank(3)
        self.write(I2C_MST_CTRL, 0x4D)
        self.write(I2C_MST_DELAY_CTRL, 0x01)

        # 6. Is the magnetometer there? Then soft-reset it and wait until the reset bit clears
        if self.mag_read(AK_WIA) != AK09916_CHIP_ID:
            raise RuntimeError("AK09916 magnetometer not found inside the ICM-20948")
        self.mag_write(AK_CNTL3, 0x01)
        start = time.ticks_ms()
        while self.mag_read(AK_CNTL3) == 0x01:
            if time.ticks_diff(time.ticks_ms(), start) > 100:
                raise RuntimeError("AK09916 did not come out of reset")
            time.sleep_ms(1)

        # 7. Continuous mode: let the chips do the magnetometer reading by themselves
        if self.mag_mode == "continuous":
            self._start_mag_continuous()

    # ------------------------------------------------------------------ low-level register access
    # Every transfer is retried a few times: on the bench Pico H the first transfers after the bus
    # starts sometimes fail with OSError EIO (errno 5), then the bus is clean. A transfer that still
    # fails after RETRIES attempts raises OSError, and the callers skip that sample.
    RETRIES = 3

    def _transfer(self, fn, *args):
        for attempt in range(self.RETRIES):
            try:
                return fn(self._addr, *args)
            except OSError:
                if attempt == self.RETRIES - 1:
                    raise
                time.sleep_us(200)

    def write(self, reg, value):
        """Write one byte to an ICM-20948 register in the current bank."""
        self._buf1[0] = value & 0xFF
        self._transfer(self._i2c.writeto_mem, reg, self._buf1)
        time.sleep_us(100)              # Pimoroni waits 0.1 ms after every write

    def read(self, reg):
        """Read one byte from an ICM-20948 register in the current bank."""
        return self._transfer(self._i2c.readfrom_mem, reg, 1)[0]

    def read_bytes(self, reg, length):
        """Read `length` consecutive registers in one I2C transaction."""
        return self._transfer(self._i2c.readfrom_mem, reg, length)

    def bank(self, value):
        """Select register bank 0-3 (only writes when it changes)."""
        if self._bank != value:
            self.write(BANK_SEL, value << 4)
            self._bank = value

    # ------------------------------------------------------------------ magnetometer via SLV0
    def _trigger_mag_io(self):
        """Run the internal I2C master briefly so it performs the SLV0 transfer set up in bank 3."""
        self.bank(0)
        user = self.read(USER_CTRL)
        self.write(USER_CTRL, user | 0x20)
        time.sleep_ms(5)
        self.write(USER_CTRL, user)

    def mag_write(self, reg, value):
        """Write one byte to an AK09916 register (setup only)."""
        self.bank(3)
        self.write(I2C_SLV0_ADDR, AK09916_ADDR)         # Bit 7 clear = write
        self.write(I2C_SLV0_REG, reg)
        self.write(I2C_SLV0_DO, value)
        self.bank(0)
        self._trigger_mag_io()

    def mag_read(self, reg):
        """Read one byte from an AK09916 register."""
        self.bank(3)
        self.write(I2C_SLV0_ADDR, AK09916_ADDR | 0x80)  # Bit 7 set = read
        self.write(I2C_SLV0_REG, reg)
        self.write(I2C_SLV0_DO, 0xFF)
        self.write(I2C_SLV0_CTRL, 0x80 | 1)             # Enable, read 1 byte
        self.bank(0)
        self._trigger_mag_io()
        return self.read(EXT_SLV_SENS_DATA_00)

    def mag_read_bytes(self, reg, length):
        """Read several AK09916 registers (single mode). Same register writes as Pimoroni."""
        self.bank(3)
        self.write(I2C_SLV0_CTRL, 0x80 | 0x08 | length)
        self.write(I2C_SLV0_ADDR, AK09916_ADDR | 0x80)
        self.write(I2C_SLV0_REG, reg)
        self.write(I2C_SLV0_DO, 0xFF)
        self.bank(0)
        self._trigger_mag_io()
        return self.read_bytes(EXT_SLV_SENS_DATA_00, length)

    def _start_mag_continuous(self):
        """AK09916 measures at 100 Hz; SLV0 copies HXL..ST2 (8 bytes) into EXT_SLV_SENS_DATA each cycle.

        Reading through ST2 matters: in continuous mode the AK09916 only releases the next sample
        after ST2 has been read. The 8 bytes are X, Y, Z (6), a dummy byte (0x17) and ST2.
        """
        self.mag_write(AK_CNTL2, 0x08)                  # Continuous measurement mode 4 = 100 Hz
        time.sleep_ms(10)
        self.bank(3)
        self.write(I2C_SLV0_ADDR, AK09916_ADDR | 0x80)
        self.write(I2C_SLV0_REG, AK_HXL)
        self.write(I2C_SLV0_CTRL, 0x80 | 8)             # Enable, read 8 bytes every sample
        self.bank(0)
        self.write(USER_CTRL, self.read(USER_CTRL) | 0x20)   # Leave the I2C master running
        time.sleep_ms(20)                               # Let the first magnetometer result arrive

    def _mag_single_counts(self, timeout_ms=1000):
        """Pimoroni read_magnetometer_data(): trigger one measurement, wait for it, read 6 bytes."""
        self.mag_write(AK_CNTL2, 0x01)
        start = time.ticks_ms()
        while not (self.mag_read(AK_ST1) & 0x01):
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                raise RuntimeError("Timeout waiting for Magnetometer Ready")
        return struct.unpack("<hhh", self.mag_read_bytes(AK_HXL, 6))

    # ------------------------------------------------------------------ range / rate / filter setup
    def set_gyro_sample_rate(self, rate=125):
        """Gyro output rate in Hz: 1.125 kHz / (1 + divider)."""
        self.bank(2)
        self.write(GYRO_SMPLRT_DIV, int((1125.0 / rate) - 1))

    def set_gyro_low_pass(self, enabled=True, mode=5):
        self.bank(2)
        value = self.read(GYRO_CONFIG_1) & 0b10001110
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(GYRO_CONFIG_1, value)

    def set_gyro_full_scale(self, scale=250):
        self.bank(2)
        value = self.read(GYRO_CONFIG_1) & 0b11111001
        value |= {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}[scale] << 1
        self.write(GYRO_CONFIG_1, value)

    def set_accelerometer_sample_rate(self, rate=125):
        """Accel output rate in Hz: 1.125 kHz / (1 + divider), divider is 12 bits over two registers."""
        self.bank(2)
        rate = int((1125.0 / rate) - 1)
        self.write(ACCEL_SMPLRT_DIV_1, (rate >> 8) & 0xFF)
        self.write(ACCEL_SMPLRT_DIV_2, rate & 0xFF)

    def set_accelerometer_low_pass(self, enabled=True, mode=5):
        self.bank(2)
        value = self.read(ACCEL_CONFIG) & 0b10001110
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(ACCEL_CONFIG, value)

    def set_accelerometer_full_scale(self, scale=16):
        self.bank(2)
        value = self.read(ACCEL_CONFIG) & 0b11111001
        value |= {2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11}[scale] << 1
        self.write(ACCEL_CONFIG, value)

    # ------------------------------------------------------------------ reading data
    def read_si(self):
        """One sample of everything: (accel m/s^2, gyro rad/s, mag T), each an (x, y, z) tuple.

        Raises OSError if the I2C transfer fails (loose wire, noise): callers skip that sample,
        the same way the merged TemRover imu_raw_live does.
        """
        self.bank(0)
        if self.mag_mode == "continuous":
            # One 22-byte burst: accel(6) gyro(6) temperature(2) AK09916 X,Y,Z(6) dummy(1) ST2(1)
            data = self.read_bytes(ACCEL_XOUT_H, 22)
            ax, ay, az, gx, gy, gz = struct.unpack_from(">hhhhhh", data, 0)
            mx, my, mz = struct.unpack_from("<hhh", data, 14)
            if data[21] & 0x08:                         # ST2 HOFL: field beyond +-4912 uT
                self.mag_overflows += 1
        else:
            data = self.read_bytes(ACCEL_XOUT_H, 12)
            ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhh", data)
            mx, my, mz = self._mag_single_counts()

        # All three accelerometer axes exactly 0 cannot be a real reading (gravity is always there):
        # the chip has reset itself (power dip) and is back asleep with its settings lost. Set it up
        # again and skip this sample, instead of recording zeros for the rest of the run.
        if ax == 0 and ay == 0 and az == 0:
            self.resets += 1
            try:
                self.setup()
            except RuntimeError as exc:
                raise OSError("IMU reset; set-up failed: %s" % exc)
            raise OSError("IMU had reset; set up again (reset %d)" % self.resets)

        # counts -> g -> m/s^2
        a = self._counts_per_g
        accel = (ax / a * GRAVITY, ay / a * GRAVITY, az / a * GRAVITY)
        # counts -> deg/s -> rad/s
        g = self._counts_per_dps
        gyro = (gx / g * DEG2RAD, gy / g * DEG2RAD, gz / g * DEG2RAD)
        # counts -> uT -> T
        m = MAG_UT_PER_COUNT * 1e-6
        mag = (mx * m, my * m, mz * m)
        return accel, gyro, mag
