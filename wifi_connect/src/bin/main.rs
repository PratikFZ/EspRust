#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use log::info;
use wifi_connect::connecter::connect;
// use wifi_connect::types::panic;

extern crate alloc;
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0
    esp_println::logger::init_logger_from_env(); // Initialize the logger from environment variables
    // esp_println::logger::init_logger(log::LevelFilter::Info);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    wifi_connect::allocator::init_memory_heap();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");
    Timer::after(Duration::from_millis(1000)).await;

    let stack = connect(spawner, peripherals.WIFI).await;

    loop {
        if let Some(ip) = stack.config_v4() {
            info!("Got IP address: {}", ip.address);
        } else {
            info!("No IP address assigned yet.");
        }
        Timer::after(Duration::from_secs(1000)).await;
    }
}