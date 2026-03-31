#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::CpuClock,
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    time::Rate,
    timer::timg::TimerGroup,
};
use log::info;

use sign_lang::filter::{
    madgwick_update, CalData, DualGestureState, GestureState, Quaternion, Vec3,
};
use sign_lang::flex::{FlexHand, oversample_count};
use sign_lang::lsm6dsox::Lsm6dsox;
use sign_lang::mpu6050::Mpu6050;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// ── I2C addresses ─────────────────────────────────────────
const MPU_ADDR: u8 = 0x68; // MPU-6050  (AD0→GND)
const LSM_ADDR: u8 = 0x6B; // LSM6DSOX  (SDO/SA0→3.3V, most boards)

// ── Timing ────────────────────────────────────────────────
const SAMPLE_HZ: f32 = 200.0;
const DT: f32 = 1.0 / SAMPLE_HZ;
const SAMPLE_MS: u64 = 5;
const MADGWICK_BETA: f32 = 0.04;
const CALIBRATION_SAMPLES: usize = 500;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let mut delay = Delay::new();

    info!("═══════════════════════════════════════════");
    info!("  Sign Language Glove — Dual IMU + Madgwick");
    info!("═══════════════════════════════════════════");

    // ── I2C bus ───────────────────────────────────────────
    let mut i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("I2C init failed")
    .with_sda(peripherals.GPIO21)
    .with_scl(peripherals.GPIO22);

    // ── I2C bus scan ──────────────────────────────────────
    info!("Scanning I2C bus...");
    let mut lsm_addr: u8 = LSM_ADDR;
    for addr in 0x01..=0x7Fu8 {
        let mut d = [0u8; 1];
        if i2c.read(addr, &mut d).is_ok() {
            info!("  Found device at 0x{:02X}", addr);
            if addr == 0x6A || addr == 0x6B {
                lsm_addr = addr;
            }
        }
    }
    info!("");

    delay.delay_millis(100);

    // ── Init MPU-6050 ─────────────────────────────────────
    let mpu_ok;
    {
        let mut mpu = Mpu6050::new(&mut i2c, MPU_ADDR);
        match mpu.who_am_i() {
            Ok(id) => {
                info!("MPU-6050  WHO_AM_I: 0x{:02X} (expect 0x68)", id);
                mpu_ok = true;
            }
            Err(e) => {
                info!("MPU-6050  not found: {:?}", e);
                mpu_ok = false;
            }
        }
        if mpu_ok {
            let _ = mpu.init();
        }
    }
    delay.delay_millis(50);

    // ── Init LSM6DSOX ─────────────────────────────────────
    let lsm_ok;
    {
        let mut lsm = Lsm6dsox::new(&mut i2c, lsm_addr);
        match lsm.who_am_i() {
            Ok(id) => {
                info!("LSM6DSOX  WHO_AM_I: 0x{:02X} (expect 0x6C)", id);
                lsm_ok = true;
            }
            Err(e) => {
                info!("LSM6DSOX  not found at 0x{:02X}: {:?}", lsm_addr, e);
                lsm_ok = false;
            }
        }
        if lsm_ok {
            let _ = lsm.init(&mut delay);
        }
    }

    if !mpu_ok && !lsm_ok {
        info!("No IMU found! Check wiring. Halting.");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    // ── Calibrate (keep hand flat & still) ────────────────
    info!("════════════════════════════════════════════");
    info!("  CALIBRATING — KEEP HAND FLAT & STILL");
    info!("════════════════════════════════════════════");

    delay.delay_millis(500);

    let mpu_cal = if mpu_ok {
        let mut cal_data = CalData::default();
        let (mut sgx, mut sgy, mut sgz) = (0.0f32, 0.0f32, 0.0f32);
        let (mut sax, mut say, mut saz) = (0.0f32, 0.0f32, 0.0f32);
        let mut count = 0usize;
        for _ in 0..CALIBRATION_SAMPLES {
            let mut mpu = Mpu6050::new(&mut i2c, MPU_ADDR);
            if let Ok(r) = mpu.read_imu() {
                sgx += r.gyro.x; sgy += r.gyro.y; sgz += r.gyro.z;
                sax += r.accel.x; say += r.accel.y; saz += r.accel.z;
                count += 1;
            }
            delay.delay_millis(10);
        }
        if count > 0 {
            let n = count as f32;
            cal_data = CalData {
                gyro_bias: Vec3 { x: sgx / n, y: sgy / n, z: sgz / n },
                accel_bias: Vec3 { x: sax / n, y: say / n, z: (saz / n) - 1.0 },
            };
            info!("  MPU gyro bias:  X={:.3} Y={:.3} Z={:.3}", cal_data.gyro_bias.x, cal_data.gyro_bias.y, cal_data.gyro_bias.z);
            info!("  MPU accel bias: X={:.4} Y={:.4} Z={:.4}", cal_data.accel_bias.x, cal_data.accel_bias.y, cal_data.accel_bias.z);
        }
        cal_data
    } else {
        CalData::default()
    };

    let lsm_cal = if lsm_ok {
        let mut cal_data = CalData::default();
        let (mut sgx, mut sgy, mut sgz) = (0.0f32, 0.0f32, 0.0f32);
        let (mut sax, mut say, mut saz) = (0.0f32, 0.0f32, 0.0f32);
        let mut count = 0usize;
        for _ in 0..CALIBRATION_SAMPLES {
            let mut lsm = Lsm6dsox::new(&mut i2c, lsm_addr);
            if let Ok(r) = lsm.read_imu() {
                sgx += r.gyro.x; sgy += r.gyro.y; sgz += r.gyro.z;
                sax += r.accel.x; say += r.accel.y; saz += r.accel.z;
                count += 1;
            }
            delay.delay_millis(10);
        }
        if count > 0 {
            let n = count as f32;
            cal_data = CalData {
                gyro_bias: Vec3 { x: sgx / n, y: sgy / n, z: sgz / n },
                accel_bias: Vec3 { x: sax / n, y: say / n, z: (saz / n) - 1.0 },
            };
            info!("  LSM gyro bias:  X={:.3} Y={:.3} Z={:.3}", cal_data.gyro_bias.x, cal_data.gyro_bias.y, cal_data.gyro_bias.z);
            info!("  LSM accel bias: X={:.4} Y={:.4} Z={:.4}", cal_data.accel_bias.x, cal_data.accel_bias.y, cal_data.accel_bias.z);
        }
        cal_data
    } else {
        CalData::default()
    };

    // ── Init Madgwick filters (one per sensor) ────────────
    let mut q_mpu = Quaternion::identity();
    let mut q_lsm = Quaternion::identity();
    let mut gesture_mpu = GestureState::new();
    let mut gesture_lsm = GestureState::new();
    let mut gesture_combined = DualGestureState::new();

    // ── Flex sensors (5 fingers on ADC1) ──────────────────
    // Wiring per finger:
    //   3.3V ──[ Flex Sensor ]──┬──[ 10kΩ ]── GND
    //                           └── GPIO pin
    //
    // GPIO36=Thumb, GPIO39=Index, GPIO34=Middle, GPIO35=Ring, GPIO32=Pinky
    let mut adc1_config = AdcConfig::new();
    let mut thumb_pin  = adc1_config.enable_pin(peripherals.GPIO36, Attenuation::_11dB);
    let mut index_pin  = adc1_config.enable_pin(peripherals.GPIO39, Attenuation::_11dB);
    let mut middle_pin = adc1_config.enable_pin(peripherals.GPIO34, Attenuation::_11dB);
    let mut ring_pin   = adc1_config.enable_pin(peripherals.GPIO35, Attenuation::_11dB);
    let mut pinky_pin  = adc1_config.enable_pin(peripherals.GPIO32, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let mut flex_hand = FlexHand::new();
    let n_oversample = oversample_count();

    // ── Flex calibration: STRAIGHT phase ──────────────────
    info!("════════════════════════════════════════════");
    info!("  FLEX CAL: Keep fingers STRAIGHT for 3s...");
    info!("════════════════════════════════════════════");
    delay.delay_millis(1000); // settle time
    for _ in 0..300 {
        // Oversample each finger
        let mut samples = [0u16; 32];
        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut thumb_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[0].calibrate_straight(avg as f32 / n_oversample as f32);

        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut index_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[1].calibrate_straight(avg as f32 / n_oversample as f32);

        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut middle_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[2].calibrate_straight(avg as f32 / n_oversample as f32);

        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut ring_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[3].calibrate_straight(avg as f32 / n_oversample as f32);

        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut pinky_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[4].calibrate_straight(avg as f32 / n_oversample as f32);

        delay.delay_millis(10);
    }
    for i in 0..5 {
        let (mn, mx) = flex_hand.fingers[i].cal_range();
        info!("  {} straight baseline: {:.0} (range {:.0}-{:.0})", FlexHand::finger_name(i), flex_hand.fingers[i].raw_ema(), mn, mx);
    }

    // ── Flex calibration: BENT phase ──────────────────────
    info!("════════════════════════════════════════════");
    info!("  FLEX CAL: Now BEND all fingers for 3s...");
    info!("  (starting in 2 seconds...)");
    info!("════════════════════════════════════════════");
    delay.delay_millis(2000); // give user time to bend
    for _ in 0..300 {
        let mut samples = [0u16; 32];
        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut thumb_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[0].calibrate_bent(avg as f32 / n_oversample as f32);

        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut index_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[1].calibrate_bent(avg as f32 / n_oversample as f32);

        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut middle_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[2].calibrate_bent(avg as f32 / n_oversample as f32);

        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut ring_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[3].calibrate_bent(avg as f32 / n_oversample as f32);

        for s in samples.iter_mut().take(n_oversample as usize) {
            *s = nb::block!(adc1.read_oneshot(&mut pinky_pin)).unwrap_or(0);
        }
        let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
        flex_hand.fingers[4].calibrate_bent(avg as f32 / n_oversample as f32);

        delay.delay_millis(10);
    }
    // Finalize: detect direction, pad range 20%, lock calibration
    for i in 0..5 {
        flex_hand.fingers[i].finalize_calibration();
    }
    for i in 0..5 {
        let (mn, mx) = flex_hand.fingers[i].cal_range();
        info!("  {} final range: {:.0} — {:.0}  (span: {:.0}, padded)", FlexHand::finger_name(i), mn, mx, mx - mn);
    }
    info!("Flex calibration locked! Range padded for noise immunity.");
    info!("");

    // Warm up both filters (~1 sec, high beta for fast convergence)
    info!("Warming up Madgwick filters...");
    for _ in 0..200 {
        if mpu_ok {
            let mut mpu = Mpu6050::new(&mut i2c, MPU_ADDR);
            if let Ok(raw) = mpu.read_imu() {
                let imu = mpu_cal.apply(&raw);
                madgwick_update(&mut q_mpu, &imu, 0.5, DT);
            }
        }
        if lsm_ok {
            let mut lsm = Lsm6dsox::new(&mut i2c, lsm_addr);
            if let Ok(raw) = lsm.read_imu() {
                let imu = lsm_cal.apply(&raw);
                madgwick_update(&mut q_lsm, &imu, 0.5, DT);
            }
        }
        delay.delay_millis(SAMPLE_MS as u32);
    }

    info!("═══════════════════════════════════════════");
    info!("  READY — Move your hand!");
    info!("═══════════════════════════════════════════");

    let mut tick: u32 = 0;

    loop {
        // ── Read & process MPU-6050 ───────────────────────
        let mpu_imu = if mpu_ok {
            let mut mpu = Mpu6050::new(&mut i2c, MPU_ADDR);
            match mpu.read_imu() {
                Ok(raw) => {
                    let imu = mpu_cal.apply(&raw);
                    madgwick_update(&mut q_mpu, &imu, MADGWICK_BETA, DT);
                    let euler = q_mpu.to_euler_deg();
                    gesture_mpu.update(&imu, &euler, DT);
                    Some(imu)
                }
                Err(_) => None,
            }
        } else {
            None
        };

        // ── Read & process LSM6DSOX ───────────────────────
        let lsm_imu = if lsm_ok {
            let mut lsm = Lsm6dsox::new(&mut i2c, lsm_addr);
            match lsm.read_imu() {
                Ok(raw) => {
                    let imu = lsm_cal.apply(&raw);
                    madgwick_update(&mut q_lsm, &imu, MADGWICK_BETA, DT);
                    let euler = q_lsm.to_euler_deg();
                    gesture_lsm.update(&imu, &euler, DT);
                    Some(imu)
                }
                Err(_) => None,
            }
        } else {
            None
        };

        // ── Read flex sensors (oversampled) ─────────────────
        {
            let mut samples = [0u16; 32];
            // Thumb
            for s in samples.iter_mut().take(n_oversample as usize) {
                *s = nb::block!(adc1.read_oneshot(&mut thumb_pin)).unwrap_or(0);
            }
            let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
            flex_hand.update_finger(0, avg as f32 / n_oversample as f32);
            // Index
            for s in samples.iter_mut().take(n_oversample as usize) {
                *s = nb::block!(adc1.read_oneshot(&mut index_pin)).unwrap_or(0);
            }
            let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
            flex_hand.update_finger(1, avg as f32 / n_oversample as f32);
            // Middle
            for s in samples.iter_mut().take(n_oversample as usize) {
                *s = nb::block!(adc1.read_oneshot(&mut middle_pin)).unwrap_or(0);
            }
            let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
            flex_hand.update_finger(2, avg as f32 / n_oversample as f32);
            // Ring
            for s in samples.iter_mut().take(n_oversample as usize) {
                *s = nb::block!(adc1.read_oneshot(&mut ring_pin)).unwrap_or(0);
            }
            let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
            flex_hand.update_finger(3, avg as f32 / n_oversample as f32);
            // Pinky
            for s in samples.iter_mut().take(n_oversample as usize) {
                *s = nb::block!(adc1.read_oneshot(&mut pinky_pin)).unwrap_or(0);
            }
            let avg: u32 = samples.iter().take(n_oversample as usize).map(|&v| v as u32).sum();
            flex_hand.update_finger(4, avg as f32 / n_oversample as f32);
        }
        let flex_bends = flex_hand.bend_all();

        // ── Combined dual-sensor gesture ──────────────────
        if let (Some(palm), Some(finger)) = (&lsm_imu, &mpu_imu) {
            let palm_euler = q_lsm.to_euler_deg();
            let finger_euler = q_mpu.to_euler_deg();
            gesture_combined.update(palm, &palm_euler, finger, &finger_euler, Some(&flex_bends));
        }

        // ── Print at ~5 Hz (every 40th sample) ───────────
        if tick % 40 == 0 {
            info!("═════════════════════════════════════════════");

            if let Some(imu) = mpu_imu {
                let e = q_mpu.to_euler_deg();
                info!("MPU-6050 (index finger)");
                info!(
                    "  Roll:{:>7.1}°  Pitch:{:>7.1}°  Yaw:{:>7.1}°",
                    e.roll, e.pitch, e.yaw
                );
                info!(
                    "  Gyro {:>6.1} {:>6.1} {:>6.1} dps | Accel {:>5.2} {:>5.2} {:>5.2} g",
                    imu.gyro.x, imu.gyro.y, imu.gyro.z,
                    imu.accel.x, imu.accel.y, imu.accel.z
                );
                info!("  >>> {}", gesture_mpu.current_gesture);
            }

            if let Some(imu) = lsm_imu {
                let e = q_lsm.to_euler_deg();
                info!("LSM6DSOX (palm)");
                info!(
                    "  Roll:{:>7.1}°  Pitch:{:>7.1}°  Yaw:{:>7.1}°",
                    e.roll, e.pitch, e.yaw
                );
                info!(
                    "  Gyro {:>6.1} {:>6.1} {:>6.1} dps | Accel {:>5.2} {:>5.2} {:>5.2} g",
                    imu.gyro.x, imu.gyro.y, imu.gyro.z,
                    imu.accel.x, imu.accel.y, imu.accel.z
                );
                info!("  >>> {}", gesture_lsm.current_gesture);
            }

            info!("FLEX SENSORS");
            info!(
                "  T:{:>3.0}%  I:{:>3.0}%  M:{:>3.0}%  R:{:>3.0}%  P:{:>3.0}%",
                flex_bends[0],
                flex_bends[1],
                flex_bends[2],
                flex_bends[3],
                flex_bends[4],
            );
            info!("  Hand pose: {}", flex_hand.classify_hand_pose());
            info!("──── COMBINED ──────────────────────────────");
            info!("  >>> {}", gesture_combined.current_gesture);
        }

        tick = tick.wrapping_add(1);
        Timer::after(Duration::from_millis(SAMPLE_MS)).await;
    }
}
