#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    clock::CpuClock,
    i2c::master::{Config as I2cConfig, I2c},
    time::Rate,
    timer::timg::TimerGroup,
};
use log::info;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]

#[esp_hal::main]
fn main() -> ! {
    // generator version: 1.2.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let delay = Delay::new();

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // Initialize I2C for MPU-6050 (SDA: GPIO21, SCL: GPIO22)
    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(100)))
        .expect("Failed to create I2C")
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    // ── MPU-6050 registers ────────────────────────────────
    const MPU_ADDR: u8 = 0x68; // AD0 pin low (use 0x69 if AD0 is HIGH)
    const REG_WHO_AM_I: u8 = 0x75;
    const REG_PWR_MGMT_1: u8 = 0x6B;
    const REG_ACCEL_CONFIG: u8 = 0x1C;
    const REG_GYRO_CONFIG: u8 = 0x1B;
    const REG_ACCEL_XOUT_H: u8 = 0x3B;

    delay.delay_millis(100); // let MPU boot up

    // ── I2C bus scan — find all connected devices ─────────
    info!("Scanning I2C bus...");
    let mut found = false;
    for addr in 0x01..=0x7F {
        let mut dummy = [0u8; 1];
        if i2c.read(addr, &mut dummy).is_ok() {
            info!("  Found device at 0x{:02X}", addr);
            found = true;
        }
    }
    if !found {
        info!("  No I2C devices found! Check wiring:");
        info!("    - SDA → GPIO21,  SCL → GPIO22");
        info!("    - VCC → 3.3V,    GND → GND");
        info!("    - Pull-up resistors (4.7kΩ) on SDA & SCL to 3.3V");
        info!("    - (Many breakout boards have pull-ups built in)");
    }
    info!("");

    // Verify connection — WHO_AM_I should return 0x68
    let mut who_am_i = [0u8; 1];
    match i2c.write_read(MPU_ADDR, &[REG_WHO_AM_I], &mut who_am_i) {
        Ok(_) => info!("MPU-6050 WHO_AM_I: 0x{:02X} (expected 0x68)", who_am_i[0]),
        Err(e) => {
            info!("ERROR: No response from MPU-6050 at 0x{:02X}: {:?}", MPU_ADDR, e);
            info!("If scan found device at 0x69, change MPU_ADDR to 0x69 (AD0 pin is HIGH)");
            loop { delay.delay_millis(1000); }
        }
    }

    // Wake up MPU-6050 (it starts in sleep mode!)
    // Write 0x00 to PWR_MGMT_1 → clears SLEEP bit
    i2c.write(MPU_ADDR, &[REG_PWR_MGMT_1, 0x00]).unwrap();
    delay.delay_millis(100);

    // Accel range: ±2g  (0x00), ±4g (0x08), ±8g (0x10), ±16g (0x18)
    i2c.write(MPU_ADDR, &[REG_ACCEL_CONFIG, 0x00]).unwrap();

    // Gyro range: ±250°/s (0x00), ±500 (0x08), ±1000 (0x10), ±2000 (0x18)
    i2c.write(MPU_ADDR, &[REG_GYRO_CONFIG, 0x00]).unwrap();

    info!("MPU-6050 configured: Accel ±2g, Gyro ±250 deg/s");
    info!("Reading raw data...");
    info!("");

    loop {
        // Read 14 bytes: Accel(6) + Temp(2) + Gyro(6) starting at 0x3B
        let mut buf = [0u8; 14];
        i2c.write_read(MPU_ADDR, &[REG_ACCEL_XOUT_H], &mut buf).unwrap();

        // Combine high + low bytes (big-endian, signed 16-bit)
        let accel_x = i16::from_be_bytes([buf[0], buf[1]]);
        let accel_y = i16::from_be_bytes([buf[2], buf[3]]);
        let accel_z = i16::from_be_bytes([buf[4], buf[5]]);

        let temp_raw = i16::from_be_bytes([buf[6], buf[7]]);

        let gyro_x = i16::from_be_bytes([buf[8], buf[9]]);
        let gyro_y = i16::from_be_bytes([buf[10], buf[11]]);
        let gyro_z = i16::from_be_bytes([buf[12], buf[13]]);

        // ── Scale to real units ───────────────────────────
        // Accel: ±2g range → 16384 LSB/g
        // Multiply by 100 to get centi-g (avoids floats)
        let ax_cg = (accel_x as i32) * 100 / 16384;
        let ay_cg = (accel_y as i32) * 100 / 16384;
        let az_cg = (accel_z as i32) * 100 / 16384;

        // Gyro: ±250°/s range → 131 LSB/(°/s)
        // Multiply by 100 to get centi-deg/s
        let gx_cd = (gyro_x as i32) * 100 / 131;
        let gy_cd = (gyro_y as i32) * 100 / 131;
        let gz_cd = (gyro_z as i32) * 100 / 131;

        // Temp: °C = raw/340 + 36.53 → use fixed point
        let temp_c_x100 = (temp_raw as i32) * 100 / 340 + 3653;

        info!(
            "Accel(cg) X:{:>6} Y:{:>6} Z:{:>6} | Gyro(cd/s) X:{:>6} Y:{:>6} Z:{:>6} | Temp: {}.{}C",
            ax_cg, ay_cg, az_cg,
            gx_cd, gy_cd, gz_cd,
            temp_c_x100 / 100, temp_c_x100 % 100,
        );

        delay.delay_millis(200);
    }
}
