use embassy_net::Runner;
use esp_radio::wifi::{WifiDevice, WifiController, ModeConfig, ClientConfig};
use crate::types::{PASSWORD, SSID, TIMER};
use log::info;
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

#[embassy_executor::task]
pub async fn wifi_task( mut controller: WifiController<'static> ){

    let wait_timer= TIMER as u64;
    loop {
        if !controller.is_started().unwrap_or(false) {
            info!("Starting wifi...");

            let config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.try_into().unwrap())
                    .with_password(PASSWORD.try_into().unwrap()),
            );
            controller.set_config(&config).unwrap();
            controller.start_async().await.unwrap()
        }

        match controller.connect_async().await {
            Ok(_) => info!("Connected to WiFi!"),
            Err(e) => {
                info!("Failed to connect to WiFi: {:?}. Retrying...", e);
                Timer::after(Duration::from_secs(wait_timer)).await;
                continue;
            }
        }

        loop {
            if controller.is_connected().unwrap_or(false) {
                info!("WiFi is still connected.");
                break;
            } 
            Timer::after(Duration::from_secs(wait_timer)).await;
        }
        Timer::after(Duration::from_secs(wait_timer)).await;
    }
}