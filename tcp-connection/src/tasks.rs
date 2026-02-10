
use esp_println::println;
use esp_radio::wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice};
use embassy_time::{Duration, Timer};
use embassy_net::Runner;
use crate::types::{PASSWORD, SSID};

#[embassy_executor::task]
pub async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    println!("Starting network task...");
    runner.run().await;
}

#[embassy_executor::task]
pub async fn connection_task(mut controller: WifiController<'static>) {
    println!("Starting WiFi connection task...");
    
    loop {
        if !matches!(controller.is_started(), Ok(true)) {
            println!("Configuring WiFi with SSID: {}", SSID);
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.try_into().unwrap())
                    .with_password(PASSWORD.try_into().unwrap()),
            );
            controller.set_config(&client_config).unwrap();
            println!("Starting WiFi...");
            controller.start_async().await.unwrap();
            println!("WiFi started!");
        }
        
        println!("Attempting to connect to WiFi...");
        match controller.connect_async().await {
            Ok(_) => {
                println!("WiFi connected successfully!");
            }
            Err(e) => {
                println!("Failed to connect to WiFi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await;
                continue;
            }
        }
        
        // Stay connected - just loop checking connection
        loop {
            if !matches!(controller.is_connected(), Ok(true)) {
                println!("WiFi disconnected, reconnecting...");
                Timer::after(Duration::from_millis(1000)).await;
                break;
            }
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}