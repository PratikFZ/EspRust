//! WiFi driver functionality for ESP32.
//!
//! This module provides async tasks for WiFi scanning and network operations.

use core::fmt::Error;

use embassy_time::{Duration, Timer};
use embassy_executor::Spawner;
use esp_hal::peripherals::WIFI;
use esp_println::println;
use esp_radio::wifi::WifiController;
use crate::types::{RADIO_INIT, WIFI_CONTROLLER};

/// Interval between WiFi scans in seconds
const SCAN_INTERVAL_SECS: u64 = 10;

/// Embassy task that continuously scans for WiFi networks.
///
/// This task runs indefinitely, performing WiFi scans at regular intervals
/// and printing discovered networks with their signal strength.
///
/// # Arguments
///
/// * `wifi_controller` - Mutable reference to the WiFi controller with static lifetime
///
/// # Panics
///
/// Panics if WiFi mode cannot be set to Station mode.
#[embassy_executor::task]
pub async fn wifi_scan_task(wifi_controller: &'static mut WifiController<'static>) {
    // Set WiFi mode once
    wifi_controller
        .set_mode(esp_radio::wifi::WifiMode::Sta)
        .unwrap();

    loop {
        println!("Starting Wi-Fi scan...");

        let scan_config = esp_radio::wifi::ScanConfig::default();
        match wifi_controller.scan_with_config_async(scan_config).await {
            Ok(scan_results) => {
                println!("Found {} networks:", scan_results.len());

                for (i, ap) in scan_results.iter().enumerate() {
                    println!(
                        "  {}: SSID: {}, Channel: {}, RSSI: {}",
                        i + 1,
                        ap.ssid.as_str(),
                        ap.channel,
                        ap.signal_strength
                    );
                }
            }
            Err(e) => {
                println!("WiFi scan failed: {}", e);
            }
        }
        println!("Waiting before next scan...");
        Timer::after(Duration::from_secs(SCAN_INTERVAL_SECS)).await;
    }
}

/// Initializes the WiFi subsystem and spawns a background scanning task.
///
/// This function sets up the radio and WiFi controller, then spawns
/// an async task that continuously scans for available WiFi networks.
///
/// # Arguments
///
/// * `spawner` - Embassy task spawner for creating the background scan task
/// * `device` - WiFi peripheral device with static lifetime
///
/// # Returns
///
/// Returns `Ok(())` on successful initialization, or an `Error` if any step fails.
///
/// # Errors
///
/// This function will return an error if:
/// - Radio initialization fails
/// - WiFi controller creation fails
/// - Setting WiFi mode fails
/// - Starting the WiFi controller fails
/// - Spawning the scan task fails
pub async fn wifi_scanner(
    spawner: Spawner, 
    device: WIFI<'static>,
) -> Result<(), Error> {
    let radio_init = esp_radio::init()
        .map_err(|e| {
            println!("Failed to initialize radio controller: {}", e);
            Error
        })?;
    let radio_init = RADIO_INIT.init(radio_init);
    
    println!("Radio initialized!");
    
    println!("Creating WiFi controller...");
    let (wifi_controller, _interfaces) = esp_radio::wifi::new(
        radio_init,
        device,
        Default::default(),
    ).map_err(|e| {
        println!("Failed to create WiFi controller: {}", e);
        Error
    })?;
    println!("WiFi controller created!");
    
    let wifi_controller = WIFI_CONTROLLER.init(wifi_controller);

    wifi_controller
        .set_mode(esp_radio::wifi::WifiMode::Sta).map_err(|e| {
            println!("Failed to set Wi-Fi mode: {}", e);
            Error
        })?;

    Timer::after(Duration::from_millis(500)).await;
    
    println!("Starting WiFi controller...");
    wifi_controller.start_async().await.map_err(|e|{
        println!("Failed to start Wi-Fi controller: {}", e);
        Error
    })?;
    println!("WiFi controller started!");
    
    // Give WiFi some time to initialize
    Timer::after(Duration::from_millis(500)).await;

    spawner.spawn(wifi_scan_task(wifi_controller)).map_err(|e| {
        println!("Failed to spawn WiFi scan task: {}", e);
        Error
    })?;

    Ok(())

}