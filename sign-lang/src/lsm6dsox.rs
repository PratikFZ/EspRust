/// LSM6DSOX driver (bare-metal, no_std)
///
/// I2C address: 0x6A (SDO/SA0→GND) or 0x6B (SDO/SA0→3.3V)

use crate::filter::{ImuReading, Vec3};
use embedded_hal::i2c::I2c;

// ── Registers ─────────────────────────────────────────────
const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1_XL: u8 = 0x10; // Accel ODR + FS
const REG_CTRL2_G: u8 = 0x11; // Gyro  ODR + FS
const REG_CTRL3_C: u8 = 0x12; // Control: BDU, IF_INC, SW_RESET
const REG_CTRL6_C: u8 = 0x15; // Accel high-performance mode
const REG_CTRL7_G: u8 = 0x16; // Gyro  high-performance mode
const REG_OUTX_L_G: u8 = 0x22; // 12 bytes: gyro(6) + accel(6)

// Sensitivity @ ±2g    : 0.061 mg/LSB
// Sensitivity @ ±500°/s: 17.50 mdps/LSB
const ACCEL_MG_PER_LSB: f32 = 0.061;
const GYRO_MDPS_PER_LSB: f32 = 17.50;

/// Raw signed 16-bit readings from LSM6DSOX
#[derive(Debug, Default, Clone, Copy)]
pub struct Lsm6dsoxRaw {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

pub struct Lsm6dsox<'a, I: I2c> {
    i2c: &'a mut I,
    addr: u8,
}

impl<'a, I: I2c> Lsm6dsox<'a, I> {
    pub fn new(i2c: &'a mut I, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Read WHO_AM_I — should return 0x6C for LSM6DSOX
    pub fn who_am_i(&mut self) -> Result<u8, I::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[REG_WHO_AM_I], &mut buf)?;
        Ok(buf[0])
    }

    /// Full init: software reset, BDU, 208Hz ODR, ±2g, ±500°/s, high-perf
    pub fn init(&mut self, delay: &mut esp_hal::delay::Delay) -> Result<(), I::Error> {
        // Software reset
        self.i2c.write(self.addr, &[REG_CTRL3_C, 0x01])?;
        delay.delay_millis(50);

        // BDU on + IF_INC (auto-increment for burst reads)
        self.i2c.write(self.addr, &[REG_CTRL3_C, 0x44])?;
        delay.delay_millis(10);

        // Accel: 208 Hz ODR, ±2g  (0x50)
        self.i2c.write(self.addr, &[REG_CTRL1_XL, 0x50])?;
        delay.delay_millis(10);

        // Gyro: 208 Hz ODR, ±500°/s  (0x54)
        self.i2c.write(self.addr, &[REG_CTRL2_G, 0x54])?;
        delay.delay_millis(10);

        // High-performance mode for both
        self.i2c.write(self.addr, &[REG_CTRL6_C, 0x00])?;
        self.i2c.write(self.addr, &[REG_CTRL7_G, 0x00])?;
        delay.delay_millis(10);

        // Discard first 50 samples (settling + filter warm-up)
        for _ in 0..50 {
            let mut tmp = [0u8; 12];
            let _ = self.i2c.write_read(self.addr, &[REG_OUTX_L_G], &mut tmp);
            delay.delay_millis(5);
        }
        Ok(())
    }

    /// Read all raw sensor data in one burst
    pub fn read_raw(&mut self) -> Result<Lsm6dsoxRaw, I::Error> {
        let mut buf = [0u8; 12];
        self.i2c
            .write_read(self.addr, &[REG_OUTX_L_G], &mut buf)?;
        // LSM6DSOX is little-endian: low byte first
        Ok(Lsm6dsoxRaw {
            gyro_x: i16::from_le_bytes([buf[0], buf[1]]),
            gyro_y: i16::from_le_bytes([buf[2], buf[3]]),
            gyro_z: i16::from_le_bytes([buf[4], buf[5]]),
            accel_x: i16::from_le_bytes([buf[6], buf[7]]),
            accel_y: i16::from_le_bytes([buf[8], buf[9]]),
            accel_z: i16::from_le_bytes([buf[10], buf[11]]),
        })
    }

    /// Read sensor data in physical units (g + dps)
    pub fn read_imu(&mut self) -> Result<ImuReading, I::Error> {
        let raw = self.read_raw()?;
        Ok(ImuReading {
            accel: Vec3 {
                x: (raw.accel_x as f32) * ACCEL_MG_PER_LSB * 0.001,
                y: (raw.accel_y as f32) * ACCEL_MG_PER_LSB * 0.001,
                z: (raw.accel_z as f32) * ACCEL_MG_PER_LSB * 0.001,
            },
            gyro: Vec3 {
                x: (raw.gyro_x as f32) * GYRO_MDPS_PER_LSB * 0.001,
                y: (raw.gyro_y as f32) * GYRO_MDPS_PER_LSB * 0.001,
                z: (raw.gyro_z as f32) * GYRO_MDPS_PER_LSB * 0.001,
            },
        })
    }
}
