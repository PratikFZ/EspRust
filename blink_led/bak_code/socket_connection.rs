#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Ipv4Address, Runner, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice};
use panic_rtt_target as _;
use esp_println::println;

extern crate alloc;

// WiFi credentials - CHANGE THESE TO YOUR NETWORK!
const SSID: &str = "2601_Hall";
const PASSWORD: &str = "Juspay@2601";

// Target server
const SERVER_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 200);
const SERVER_PORT: u16 = 8080;

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

esp_bootloader_esp_idf::esp_app_desc!();

// WiFi connection task
#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) {
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

// Network stack runner task
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    println!("Starting network task...");
    runner.run().await;
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");
    println!("Embassy initialized!");
    
    Timer::after(Duration::from_millis(100)).await;
    
    // Initialize radio
    println!("Initializing radio...");
    let radio_ctrl = mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize radio")
    );
    println!("Radio initialized!");
    
    Timer::after(Duration::from_millis(100)).await;
    
    // Create WiFi controller and interface
    println!("Creating WiFi controller...");
    let (controller, interfaces) = esp_radio::wifi::new(
        radio_ctrl,
        peripherals.WIFI,
        Default::default(),
    ).expect("Failed to create WiFi controller");
    println!("WiFi controller created!");
    
    let wifi_interface = interfaces.sta;
    
    // Initialize network stack with DHCP
    println!("Initializing network stack...");
    let net_config = embassy_net::Config::dhcpv4(Default::default());
    
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    
    let stack: &'static mut _ = mk_static!(embassy_net::Stack, stack);
    
    println!("Network stack initialized!");
    
    // Spawn tasks
    spawner.spawn(connection_task(controller)).ok();
    spawner.spawn(net_task(runner)).ok();
    
    // Wait for link to be up
    println!("Waiting for WiFi link...");
    loop {
        if stack.is_link_up() {
            println!("WiFi link is up!");
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    
    // Wait for IP address via DHCP
    println!("Waiting for IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP address: {}", config.address);
            println!("Gateway: {:?}", config.gateway);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    
    // Now make HTTP request
    println!("Starting HTTP request loop...");
    
    // TCP buffers
    static mut RX_BUFFER: [u8; 4096] = [0; 4096];
    static mut TX_BUFFER: [u8; 4096] = [0; 4096];
    
    loop {
        println!("\n--- Making HTTP request to {}:{}/health ---", SERVER_IP, SERVER_PORT);
        
        let mut socket = TcpSocket::new(
            *stack,
            unsafe { &mut *core::ptr::addr_of_mut!(RX_BUFFER) },
            unsafe { &mut *core::ptr::addr_of_mut!(TX_BUFFER) },
        );
        
        socket.set_timeout(Some(Duration::from_secs(10)));
        
        println!("Connecting to server...");
        match socket.connect((SERVER_IP, SERVER_PORT)).await {
            Ok(_) => println!("Connected to server!"),
            Err(e) => {
                println!("Failed to connect: {:?}", e);
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        }
        
        // Send HTTP GET request
        let request = "GET /health HTTP/1.1\r\nHost: 192.168.1.200:8080\r\nConnection: close\r\n\r\n";
        println!("Sending request...");
        
        match socket.write(request.as_bytes()).await {
            Ok(_) => println!("Request sent!"),
            Err(e) => {
                println!("Failed to send request: {:?}", e);
                socket.close();
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        }
        
        // Read response
        println!("Reading response...");
        let mut buf = [0u8; 1024];
        let mut total_read = 0;
        
        loop {
            match socket.read(&mut buf[total_read..]).await {
                Ok(0) => {
                    println!("Connection closed by server");
                    break;
                }
                Ok(n) => {
                    total_read += n;
                    println!("Read {} bytes (total: {})", n, total_read);
                    if total_read >= buf.len() - 1 {
                        break;
                    }
                }
                Err(e) => {
                    println!("Read error: {:?}", e);
                    break;
                }
            }
        }
        
        // Print response
        if total_read > 0 {
            if let Ok(response) = core::str::from_utf8(&buf[..total_read]) {
                println!("\n=== HTTP Response ===");
                println!("{}", response);
                println!("=== End Response ===\n");
            } else {
                println!("Response (raw bytes): {:?}", &buf[..total_read]);
            }
        }
        
        socket.close();
        
        // Wait before next request
        println!("Waiting 10 seconds before next request...");
        Timer::after(Duration::from_secs(10)).await;
    }
}