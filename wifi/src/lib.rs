//! ESP32 WiFi scanning library
//!
//! This library provides an async WiFi scanning implementation for ESP32 microcontrollers
//! using the Embassy async runtime and esp-hal ecosystem.
//!
//! ## Features
//!
//! - Async WiFi network scanning
//! - Embassy executor integration
//! - Optimized heap memory allocation for WiFi operations
//! - Clean module organization for embedded Rust projects
//!
//! ## Example
//!
//! ```no_run
//! use wifi::{allocator, driver, types};
//! use embassy_executor::Spawner;
//!
//! #[esp_rtos::main]
//! async fn main(spawner: Spawner) -> ! {
//!     // Initialize heap
//!     allocator::init_heap();
//!     
//!     // Initialize WiFi and spawn scan task
//!     // ... (see bin/main.rs for complete example)
//! }
//! ```

#![no_std]
#![warn(missing_docs)]

/// Memory allocation configuration
pub mod allocator;

/// WiFi driver and scanning tasks
pub mod scanner;

/// Global static storage for WiFi components
pub mod types;