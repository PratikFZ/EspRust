# 🦀 ESP32 Rust Examples

[![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)](https://www.rust-lang.org/)
[![ESP32](https://img.shields.io/badge/ESP32-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)

A comprehensive collection of example projects for ESP32 development using Rust. This repository demonstrates various capabilities of the ESP32 microcontroller, from basic LED blinking to WiFi connectivity and TCP networking.

---

## 📋 Table of Contents

- [Prerequisites](#-prerequisites)
- [Quick Start](#-quick-start)
- [Examples](#-examples)
- [Setup Guide](#-setup-guide)
- [Building and Flashing](#-building-and-flashing)
- [Troubleshooting](#-troubleshooting)
- [Resources](#-resources)
- [Contributing](#-contributing)

---

## 🔧 Prerequisites

Before getting started, ensure you have the following installed:

- **Rust** (latest stable version)
- **cargo** (comes with Rust)
- **espup** (ESP Rust toolchain installer)
- **espflash** (for flashing firmware to ESP32)
- An **ESP32 development board**
- USB cable for connecting your ESP32

---

## ⚡ Quick Start

```bash
# 1. Install ESP Rust toolchain
cargo install espup --locked
espup install

# 2. Set up environment variables (run this in every new terminal)
. ~/export-esp.sh

# 3. Clone this repository
git clone https://github.com/PratikFZ/EspRust.git
cd EspRust

# 4. Navigate to an example and flash it
cd blink_led
cargo build && espflash flash target/xtensa-esp32-none-elf/debug/blink_led --monitor
```

---

## 📚 Examples

This repository contains the following example projects:

### 1. 💡 Blink LED
**Directory:** `blink_led/`

A classic "Hello World" for embedded systems. This example demonstrates basic GPIO control by blinking an LED at regular intervals.

**Features:**
- GPIO initialization and configuration
- Basic delay/timing operations
- LED on/off control

**Hardware Required:**
- ESP32 board with built-in LED (or external LED connected to a GPIO pin)

### 2. 📡 WiFi
**Directory:** `wifi/`

Basic WiFi functionality example showing how to initialize and use WiFi capabilities on ESP32.

**Features:**
- WiFi initialization
- Basic WiFi operations
- Radio management

**Hardware Required:**
- ESP32 board with WiFi capability

### 3. 🌐 WiFi Connect
**Directory:** `wifi_connect/`

Advanced WiFi example demonstrating how to connect to a WiFi access point and establish network connectivity.

**Features:**
- WiFi station mode configuration
- Connect to access point with SSID and password
- Network status monitoring
- IP address acquisition

**Hardware Required:**
- ESP32 board with WiFi capability
- Access to a WiFi network

**Configuration:**
Update the WiFi credentials in the code before flashing:
```rust
const SSID: &str = "Your_WiFi_SSID";
const PASSWORD: &str = "Your_WiFi_Password";
```

### 4. 🔌 TCP Connection
**Directory:** `tcp-connection/`

Demonstrates TCP client/server functionality over WiFi, allowing network communication with other devices.

**Features:**
- WiFi connectivity
- TCP socket creation
- Client/server communication
- Data transmission over network

**Hardware Required:**
- ESP32 board with WiFi capability
- WiFi network access
- Another device to communicate with (PC, phone, or another ESP32)

**Use Cases:**
- IoT data transmission
- Remote control applications
- Sensor data reporting
- Device-to-device communication

---

## 🛠️ Setup Guide

### 1. Install the Toolchain

The `espup` utility simplifies the installation and management of necessary toolchains for Espressif SoCs.

```bash
# Install espup
cargo install espup --locked

# Install ESP Rust toolchains
espup install
```

### 2. Set Up Environment Variables

After installation, configure the environment variables to use the toolchain.

#### Standard Shell (Bash/Zsh)

```bash
. ~/export-esp.sh
```

> ⚠️ **Important:** This command must be executed every time you open a new terminal session.

**Tip:** Add this to your `~/.bashrc` or `~/.zshrc` to automate:
```bash
echo '. ~/export-esp.sh' >> ~/.bashrc  # for Bash
echo '. ~/export-esp.sh' >> ~/.zshrc   # for Zsh
```

#### 🐟 Fish Shell

Add the following to your Fish configuration (`~/.config/fish/config.fish`):

```fish
fish_add_path ~/.rustup/toolchains/esp/xtensa-esp-elf/esp-15.2.0_20250920/xtensa-esp-elf/bin
set -x LIBCLANG_PATH ~/.rustup/toolchains/esp/xtensa-esp32-elf-clang/esp-20.1.1_20250829/esp-clang/lib
```

---

## 🔥 Building and Flashing

### Build a Project

Navigate to any example directory and build:

```bash
cd blink_led
cargo build
```

### Flash to ESP32

Flash the compiled binary to your ESP32 and open the serial monitor:

```bash
cargo build && espflash flash target/xtensa-esp32-none-elf/debug/blink_led --monitor
```

**Command breakdown:**
- `cargo build` - Compiles your Rust project
- `espflash flash` - Flashes the binary to your ESP32
- `target/xtensa-esp32-none-elf/debug/blink_led` - Path to the compiled binary
- `--monitor` - Opens a serial monitor (UART) to view output

### Flash Only (without rebuilding)

```bash
espflash flash target/xtensa-esp32-none-elf/debug/blink_led --monitor
```

### Monitor Serial Output

If you want to monitor output without flashing:

```bash
espflash monitor
```

---

## 🐛 Troubleshooting

### Common Issues

#### Port Permission Denied (Linux/macOS)

```bash
# Add your user to the dialout group (Linux)
sudo usermod -a -G dialout $USER

# On macOS, you may need to allow access to the serial port
```

#### Environment Variables Not Set

```bash
# Make sure to source the export file
. ~/export-esp.sh

# Verify with:
echo $LIBCLANG_PATH
```

#### Build Errors

```bash
# Clean and rebuild
cargo clean
cargo build
```

#### Flash Errors

- Ensure ESP32 is properly connected via USB
- Try pressing the BOOT button on ESP32 while flashing
- Check if the correct port is selected
- Verify USB cable supports data transfer (not just charging)

#### WiFi Examples Not Working

- Double-check SSID and password in the code
- Ensure WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
- Verify the ESP32 is within WiFi range

---

## 📖 Resources

### Official Documentation
- [ESP Rust Book](https://esp-rs.github.io/book/) - Comprehensive guide for Rust on ESP
- [esp-rs GitHub Organization](https://github.com/esp-rs) - Official ESP Rust repositories
- [Espressif Documentation](https://docs.espressif.com/) - ESP32 technical documentation

### Tools & Utilities
- [esp-generate](https://github.com/esp-rs/esp-generate) - Project template generator
- [espflash](https://github.com/esp-rs/espflash) - Flash and monitor tool
- [espup](https://github.com/esp-rs/espup) - Toolchain installer

### Community
- [ESP32 Rust Matrix Chat](https://matrix.to/#/#esp-rs:matrix.org)
- [Rust Embedded Working Group](https://github.com/rust-embedded)
- [ESP32 Forum](https://esp32.com/)

---

## 🚀 Creating Your Own Project

To create a new ESP32 Rust project from scratch:

```bash
# Install esp-generate
cargo install esp-generate --locked

# Generate a new project with TUI
esp-generate

# Or use CLI with specific options
esp-generate --chip esp32 -o alloc -o wifi my-project

# Available options:
# -o alloc     : Enable heap allocation
# -o wifi      : WiFi support
# -o ble       : Bluetooth Low Energy support
# -o embassy   : Use Embassy async framework
```

---

## 🤝 Contributing

Contributions are welcome! If you have examples, improvements, or bug fixes:

1. Fork the repository
2. Create a new branch (`git checkout -b feature/amazing-example`)
3. Commit your changes (`git commit -m 'Add amazing example'`)
4. Push to the branch (`git push origin feature/amazing-example`)
5. Open a Pull Request

### Contribution Guidelines

- Follow Rust naming conventions and style guidelines
- Add comprehensive comments to your code
- Update the README if adding new examples
- Test your code on actual ESP32 hardware
- Include any necessary hardware setup information

---

## 📜 License

This project is open source and available for educational purposes.

---

## 🙏 Acknowledgments

- [Espressif Systems](https://www.espressif.com/) for the amazing ESP32 platform
- [esp-rs team](https://github.com/esp-rs) for making Rust on ESP possible
- The Rust embedded community

---

## 💬 Questions or Issues?

If you encounter any problems or have questions:

1. Check the [Troubleshooting](#-troubleshooting) section
2. Review the [official documentation](https://esp-rs.github.io/book/)
3. Open an issue in this repository
4. Ask in the [ESP32 Rust Matrix Chat](https://matrix.to/#/#esp-rs:matrix.org)

---

**Happy Coding! 🎉**

*Made with ❤️ using Rust and ESP32*
