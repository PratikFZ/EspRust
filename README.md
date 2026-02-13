# 🦀 Developing Rust Applications for ESP32

A comprehensive guide to setting up and developing Rust applications for Espressif SoCs (ESP32).

---

## 📦 1. Install the Toolchain

The `espup` utility simplifies the installation and management of necessary toolchains for Espressif SoCs.

### Installation Steps

**Install espup:**
```bash
cargo install espup --locked
```

**Install the required toolchains:**
```bash
espup install
```

---

## ⚙️ 2. Set Up Environment Variables

After installation, configure the environment variables to use the toolchain.

### Standard Shell Setup

Run this command in your terminal:
```bash
. ~/export-esp.sh
```

> ⚠️ **Important:** This command must be executed every time you open a new terminal session.

For alternative environment setup methods, refer to the [official documentation](https://esp-rs.github.io/book/installation/riscv-and-xtensa.html#3-set-up-the-environment-variables).

### 🐟 Fish Shell Specific Setup

If you're using Fish shell, add the following to your configuration:

```fish
fish_add_path ~/.rustup/toolchains/esp/xtensa-esp-elf/esp-15.2.0_20250920/xtensa-esp-elf/bin

set -x LIBCLANG_PATH ~/.rustup/toolchains/esp/xtensa-esp32-elf-clang/esp-20.1.1_20250829/esp-clang/lib
```

---

## 🚀 3. Generate a New Project

Use the `esp-generate` tool to create a new project from a template.

### Install esp-generate

```bash
cargo install esp-generate --locked
```

> 💡 **Tip:** You can also use pre-compiled release binaries or `cargo-binstall`.

### Project Generation Options

You have two options for generating a project:

#### Option A: Terminal User Interface (TUI)

```bash
esp-generate
```

This launches an interactive interface where you can select:
- Target chip
- Project name
- Additional options

#### Option B: Command Line Interface (CLI)

```bash
esp-generate --chip esp32 -o alloc -o wifi {project_name}
```

**Parameters:**
- `--chip`: Specify the target chip (e.g., `esp32`)
- `-o` or `--option`: Add features like `alloc`, `wifi`, etc.
- `--headless`: Skip the TUI for automation
- Replace `your-project` with your desired project name

> 📚 For a complete list of available options, consult the "Available Options" section in the [main README](https://github.com/esp-rs/esp-generate).

---

## 🔥 4. Build and Flash

Build your project and flash it to your ESP32 device:

```bash
cargo build && espflash flash target/xtensa-esp32-none-elf/debug/{project_name} --monitor
```

**What this does:**
- `cargo build` - Compiles your Rust project
- `espflash flash` - Flashes the binary to your ESP32
- `--monitor` - Opens a serial monitor {UART} to view output

---

## 📚 Additional Resources

- [ESP Rust Book](https://esp-rs.github.io/book/)
- [esp-rs GitHub Organization](https://github.com/esp-rs)
- [Espressif Documentation](https://docs.espressif.com/)

---

## 🤝 Contributing

Feel free to submit issues and enhancement requests!

---

**Happy coding! 🎉**
