// WiFi credentials - CHANGE THESE TO YOUR NETWORK!
pub const SSID: &str = "Rishi's S23 ultra";
pub const PASSWORD: &str = "????????";
pub static TIMER: u8 = 10;

#[panic_handler]
pub fn panic(info: &core::panic::PanicInfo) -> ! {
    log::error!("Panic: {}", info);
    loop {}
}