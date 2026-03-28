#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    clock::CpuClock,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let delay = Delay::new();

    let mut adc_config = AdcConfig::new();
    let mut flex_pin = adc_config.enable_pin(peripherals.GPIO35 , Attenuation::_11dB);

    let mut adc = Adc::new(peripherals.ADC1, adc_config);
 
    loop {

        let value: u16 = nb::block!(adc.read_oneshot(&mut flex_pin)).unwrap();
        esp_println::println!("raw readings : {}", value);

        delay.delay_millis(100);
    }
}