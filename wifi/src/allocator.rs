//! Memory allocation configuration for ESP32 WiFi operations.
//!
//! This module handles heap memory setup required for WiFi functionality.
//! ESP32 WiFi operations require significant heap memory for buffers and internal state.

/// Reclaimed RAM heap size (from bootloader sections)
const RECLAIMED_HEAP_SIZE: usize = 98768; // 72 KB

/// Main heap size for WiFi operations
const MAIN_HEAP_SIZE: usize = 128 * 1024; // 128 KB

/// Initialize heap allocators for WiFi operations.
///
/// This function sets up two heap allocators:
/// - Reclaimed RAM: Memory reclaimed from bootloader sections
/// - Main heap: Additional memory for WiFi buffers and operations
///
/// # Panics
///
/// Panics if heap allocation fails or insufficient memory is available.
pub fn init_heap() {
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: RECLAIMED_HEAP_SIZE);
    esp_alloc::heap_allocator!(size: MAIN_HEAP_SIZE);
}
