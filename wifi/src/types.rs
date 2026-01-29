//! Global static storage for WiFi components.
//!
//! This module provides static cells for WiFi controller and radio initialization,
//! ensuring they have the 'static lifetime required by Embassy async tasks.

use esp_radio::wifi::WifiController;
use static_cell::StaticCell;

/// Static storage for WiFi controller.
///
/// This static cell ensures the WiFi controller has a 'static lifetime,
/// which is required for spawning async tasks with Embassy.
pub static WIFI_CONTROLLER: StaticCell<WifiController<'static>> = StaticCell::new();

/// Static storage for radio initialization controller.
///
/// This static cell stores the radio controller that manages WiFi/BLE hardware.
pub static RADIO_INIT: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
