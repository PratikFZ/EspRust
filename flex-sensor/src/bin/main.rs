#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    clock::CpuClock,
};

esp_bootloader_esp_idf::esp_app_desc!();

// ── Tuning constants ───────────────────────────────────────────────
/// 64× oversampling: sum 64 samples, shift right 3 → 15-bit effective ADC.
/// Noise std drops from ~15 counts → ~1.9 counts; SNR improves 8×.
const OVERSAMPLE: u32 = 64;

/// Calibration ticks per phase  (80 × 50 ms ≈ 4 s)
const CALIB_TICKS: usize = 80;

/// Kalman process noise × 256.  Controls how fast the filter tracks movement.
/// Higher = reacts faster but noisier. Start at 4, increase if too sluggish.
const KF_Q256: i32 = 4;

/// Kalman measurement noise × 256.
/// After 64× oversampling noise std ≈ 2 counts → variance ≈ 4 → ×256 = 1024.
/// Higher = more smoothing but slower. Lower = faster but noisier.
const KF_R256: i32 = 512;

/// Hysteresis band (%) — prevents label flickering at boundaries.
const HYST: u32 = 3;

#[esp_hal::main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let delay = Delay::new();

    let mut adc_config = AdcConfig::new();
    let mut flex_pin = adc_config.enable_pin(peripherals.GPIO35, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // Inline macro so we avoid spelling out the Adc generic type in a fn sig.
    // Sums OVERSAMPLE raw readings and shifts right 3 → 15-bit value (0..32767).
    macro_rules! read15 {
        () => {{
            let mut s: u32 = 0;
            for _ in 0..OVERSAMPLE {
                s += nb::block!(adc.read_oneshot(&mut flex_pin)).unwrap() as u32;
            }
            (s >> 3) as i32
        }};
    }

    // ── Calibration: IQR mean over 4 s per phase ──────────────────
    let mut buf = [0i32; CALIB_TICKS];

    esp_println::println!("================================");
    esp_println::println!("  CALIBRATION  (15-bit / IQR)");
    esp_println::println!("================================");
    esp_println::println!("[1/2] Keep sensor STRAIGHT for 4 s...");

    for i in 0..CALIB_TICKS {
        buf[i] = read15!();
        if (i + 1) % 20 == 0 {
            esp_println::println!("  {} s left  sample={}", (CALIB_TICKS - i - 1) / 20, buf[i]);
        }
        delay.delay_millis(50);
    }
    let straight_val = iqr_mean(&mut buf);
    esp_println::println!("  STRAIGHT baseline = {}", straight_val);

    esp_println::println!("--------------------------------");
    esp_println::println!("[2/2] FULLY BEND sensor for 4 s...");

    for i in 0..CALIB_TICKS {
        buf[i] = read15!();
        if (i + 1) % 20 == 0 {
            esp_println::println!("  {} s left  sample={}", (CALIB_TICKS - i - 1) / 20, buf[i]);
        }
        delay.delay_millis(50);
    }
    let bent_val = iqr_mean(&mut buf);
    esp_println::println!("  BENT baseline     = {}", bent_val);

    let spread = (bent_val - straight_val).unsigned_abs();
    esp_println::println!("--------------------------------");
    esp_println::println!("Spread = {}  (15-bit units)", spread);
    if spread < 80 {
        esp_println::println!("WARNING: spread very small!");
        esp_println::println!("  → Try a 47kΩ pull-down resistor between GPIO35 and GND.");
    } else {
        esp_println::println!("Calibration OK.");
    }
    esp_println::println!("================================");
    esp_println::println!("  LIVE  (Kalman + 64x oversample)");
    esp_println::println!("================================");

    // ── Kalman filter — seed from bent_val (last calibration position) ──
    let mut kf_x: i32 = bent_val;      // state estimate (15-bit ADC units)
    let mut kf_p256: i32 = KF_R256;    // error covariance × 256

    let mut last_pct: u32 = 200; // sentinel → force first print

    loop {
        let meas = read15!();

        // Kalman predict step
        kf_p256 += KF_Q256;

        // Kalman update step
        // K = P / (P + R)  →  K256 ∈ [0, 256]
        let k256 = kf_p256 * 256 / (kf_p256 + KF_R256);
        let innov = meas - kf_x;          // innovation (signed, small)
        kf_x += k256 * innov / 256;       // state update
        kf_p256 = (256 - k256) * kf_p256 / 256 + KF_Q256; // covariance update

        let pct = map_pct(kf_x, straight_val, bent_val);

        // Only reprint when % changes by more than HYST
        let delta = if pct > last_pct { pct - last_pct } else { last_pct - pct };
        if delta > HYST || last_pct > 100 {
            last_pct = pct;

            let bars = (pct / 10).min(10) as usize;
            let bar_str = [
                "          ", "█         ", "██        ", "███       ", "████      ",
                "█████     ", "██████    ", "███████   ", "████████  ", "█████████ ",
                "██████████",
            ][bars];

            let label = match pct {
                0..=12  => "STRAIGHT",
                13..=35 => "SLIGHT  ",
                36..=65 => "MODERATE",
                66..=88 => "STRONG  ",
                _       => "FULL    ",
            };

            esp_println::println!(
                "[{}] {:>3}%  {}  (kf={} raw={})",
                bar_str, pct, label, kf_x, meas
            );
        }

        delay.delay_millis(50);
    }
}

/// Interquartile-range mean: sort the buffer, average the middle 50%.
/// Discards the noisiest 25% at each extreme → robust baseline.
fn iqr_mean(buf: &mut [i32]) -> i32 {
    // Insertion sort (fast enough for 80 elements)
    for i in 1..buf.len() {
        let key = buf[i];
        let mut j = i;
        while j > 0 && buf[j - 1] > key {
            buf[j] = buf[j - 1];
            j -= 1;
        }
        buf[j] = key;
    }
    let q1 = buf.len() / 4;
    let q3 = 3 * buf.len() / 4;
    let mut sum: i32 = 0;
    for i in q1..q3 {
        sum += buf[i];
    }
    sum / (q3 - q1) as i32
}

/// Maps a Kalman-filtered 15-bit value to 0–100 % bend intensity.
fn map_pct(value: i32, straight: i32, bent: i32) -> u32 {
    if bent == straight {
        return 0;
    }
    let (lo, hi) = if straight < bent { (straight, bent) } else { (bent, straight) };
    let clamped = value.clamp(lo, hi);
    let pct = (clamped - lo) * 100 / (hi - lo);
    let pct = pct as u32;
    if bent < straight { 100 - pct } else { pct }
}