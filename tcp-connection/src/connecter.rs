
use embassy_time::{Duration, Timer};
use embassy_executor::Spawner;
use esp_hal::peripherals::WIFI;
use embassy_net::{Stack, StackResources};
use esp_println::println;
use crate::tasks::{net_task, connection_task};

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

pub async fn connect(
    spawner: Spawner,
    device: WIFI<'static>,
) -> &'static mut Stack<'static>
{
    let radio_ctrl = mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize radio")
    );

    Timer::after(Duration::from_millis(1000)).await;

    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_ctrl, device, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = interfaces.sta;

    Timer::after(Duration::from_millis(1000)).await;

    let rng = esp_hal::rng::Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    let stack: &'static mut _ = mk_static!(embassy_net::Stack, stack);

    Timer::after(Duration::from_millis(1000)).await;

    spawner.spawn(connection_task(wifi_controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    
    loop {
        if stack.is_link_up() {
            println!("WiFi link is up!");
            break;
        }
        Timer::after(Duration::from_millis(1000)).await;
    }

    stack
}