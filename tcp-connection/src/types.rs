use embassy_net::Ipv4Address;

// WiFi credentials - CHANGE THESE TO YOUR NETWORK!
pub const SSID: &str = "2601_Hall";
pub const PASSWORD: &str = "Juspay@2601";

// Target server
pub const SERVER_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 100);
pub const SERVER_PORT: u16 = 8080;