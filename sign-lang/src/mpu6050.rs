/// MPU-6050 driver (bare-metal, no_std)
///
/// I2C address: 0x68 (AD0→GND) or 0x69 (AD0→3.3V)

use crate::filter::{ImuReading, Vec3};
use embedded_hal::i2c::I2c;

// ── Registers ─────────────────────────────────────────────
const REG_WHO_AM_I: u8 = 0x75;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_XOUT_H: u8 = 0x3B; // 14 bytes: accel(6) + temp(2) + gyro(6)

// Sensitivity @ ±2g  : 16384 LSB/g   → 0.061 mg/LSB
// Sensitivity @ ±500°/s: 65.5 LSB/(°/s) → 15.267 mdps/LSB
const ACCEL_MG_PER_LSB: f32 = 0.061;
const GYRO_MDPS_PER_LSB: f32 = 15.267;

/// Raw signed 16-bit readings from MPU-6050
#[derive(Debug, Default, Clone, Copy)]
pub struct Mpu6050Raw {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    pub temp_raw: i16,
}

pub struct Mpu6050<'a, I: I2c> {
    i2c: &'a mut I,
    addr: u8,
}

impl<'a, I: I2c> Mpu6050<'a, I> {
    pub fn new(i2c: &'a mut I, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Read WHO_AM_I — should return 0x68 for genuine MPU-6050
    pub fn who_am_i(&mut self) -> Result<u8, I::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[REG_WHO_AM_I], &mut buf)?;
        Ok(buf[0])
    }

    /// Full init: wake up, ±2g accel, ±500°/s gyro, DLPF 44Hz, 200Hz sample rate
    pub fn init(&mut self) -> Result<(), I::Error> {
        // Wake up — use internal 8 MHz oscillator
        self.i2c.write(self.addr, &[REG_PWR_MGMT_1, 0x00])?;
        // Sample rate divider: 1kHz / (1+4) = 200Hz
        self.i2c.write(self.addr, &[REG_SMPLRT_DIV, 0x04])?;
        // DLPF config: bandwidth ~44Hz (config = 3)
        self.i2c.write(self.addr, &[REG_CONFIG, 0x03])?;
        // Accel ±2g (0x00)
        self.i2c.write(self.addr, &[REG_ACCEL_CONFIG, 0x00])?;
        // Gyro ±500°/s (0x08)
        self.i2c.write(self.addr, &[REG_GYRO_CONFIG, 0x08])?;
        Ok(())
    }

    /// Read all raw sensor data in one burst
    pub fn read_raw(&mut self) -> Result<Mpu6050Raw, I::Error> {
        let mut buf = [0u8; 14];
        self.i2c
            .write_read(self.addr, &[REG_ACCEL_XOUT_H], &mut buf)?;
        Ok(Mpu6050Raw {
            accel_x: i16::from_be_bytes([buf[0], buf[1]]),
            accel_y: i16::from_be_bytes([buf[2], buf[3]]),
            accel_z: i16::from_be_bytes([buf[4], buf[5]]),
            temp_raw: i16::from_be_bytes([buf[6], buf[7]]),
            gyro_x: i16::from_be_bytes([buf[8], buf[9]]),
            gyro_y: i16::from_be_bytes([buf[10], buf[11]]),
            gyro_z: i16::from_be_bytes([buf[12], buf[13]]),
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
