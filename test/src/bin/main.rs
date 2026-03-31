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
    i2c::master::{Config as I2cConfig, I2c},
    timer::timg::TimerGroup,
    clock::CpuClock,
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
    const LSM_ADDR: u8 = 0x6A;
    const START_ADDR: u8 = 0x22;

    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    delay.delay_millis(500);

    loop {
        // ── Read 6 gyro bytes (Gx_L, Gx_H, Gy_L, Gy_H, Gz_L, Gz_H) ──
        let mut gbuf = [0u8; 12];
        i2c.write_read(LSM_ADDR, &[START_ADDR], &mut gbuf).unwrap();

        info!("{:?}", gbuf);

        delay.delay_millis(100);
    }
}
