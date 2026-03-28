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
use embedded_hal::i2c::I2c as I2cTrait;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    time::Rate,
    timer::timg::TimerGroup,
};
use log::info;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// ═══════════════════════════════════════════════════════════════════
//  LSM6DSO Register Map
// ═══════════════════════════════════════════════════════════════════
const IMU_ADDR: u8 = 0x6B;
const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1_XL: u8 = 0x10;
const REG_CTRL2_G: u8 = 0x11;
const REG_CTRL3_C: u8 = 0x12;
const REG_CTRL6_C: u8 = 0x15;  // Accel high-performance mode
const REG_CTRL7_G: u8 = 0x16;  // Gyro high-performance mode
const REG_OUTX_L_G: u8 = 0x22;

// Sensitivity: ±2g accel, ±500dps gyro (better range for hand gestures)
const ACCEL_MG_PER_LSB: f32 = 0.061;  // mg/LSB at ±2g
const GYRO_MDPS_PER_LSB: f32 = 17.50; // mdps/LSB at ±500dps

const DEG_TO_RAD: f32 = 0.017453293; // π/180
const RAD_TO_DEG: f32 = 57.295780;   // 180/π

// Sample rate: 208 Hz ODR, we read every ~5ms = 200 Hz effective
const SAMPLE_HZ: f32 = 200.0;
const DT: f32 = 1.0 / SAMPLE_HZ;
const SAMPLE_MS: u64 = 5;

// Madgwick filter gain (beta)
// Lower = smoother but slower response, higher = noisier but faster
// 0.033 is typical for wrist-worn IMU. Increase to 0.05-0.1 for faster tracking.
const MADGWICK_BETA: f32 = 0.04;

// Calibration
const CALIBRATION_SAMPLES: usize = 500;

// Gesture detection thresholds
const GESTURE_GYRO_THRESHOLD: f32 = 80.0;  // dps — fast rotation
const GESTURE_ACCEL_THRESHOLD: f32 = 1.8;  // g — sharp acceleration (punch/flick)
const GESTURE_SNAP_THRESHOLD: f32 = 2.5;   // g — very sharp snap
const WAVE_REVERSAL_COUNT: u8 = 3;         // reversals needed to detect a wave

// ═══════════════════════════════════════════════════════════════════
//  Data types
// ═══════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, Copy)]
struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

impl Vec3 {
    fn magnitude(&self) -> f32 { sqrt_approx(self.x * self.x + self.y * self.y + self.z * self.z) }
}

/// Raw sensor data in physical units
#[derive(Debug, Clone, Copy)]
struct ImuReading {
    gyro: Vec3,   // degrees/sec
    accel: Vec3,  // g
}

/// Unit quaternion representing 3D orientation (no gimbal lock!)
#[derive(Clone, Copy)]
struct Quaternion {
    w: f32,
    x: f32,
    y: f32,
    z: f32,
}

impl Quaternion {
    fn identity() -> Self { Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 } }

    fn normalize(&mut self) {
        let n = sqrt_approx(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if n > 0.0 {
            let inv = 1.0 / n;
            self.w *= inv;
            self.x *= inv;
            self.y *= inv;
            self.z *= inv;
        }
    }

    /// Extract Euler angles (aerospace convention: ZYX)
    fn to_euler_deg(&self) -> EulerAngles {
        // Roll (X)
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = atan2_approx(sinr_cosp, cosr_cosp);

        // Pitch (Y) — clamp to avoid NaN near ±90°
        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp >= 1.0 {
            90.0
        } else if sinp <= -1.0 {
            -90.0
        } else {
            asin_approx(sinp) * RAD_TO_DEG
        };

        // Yaw (Z)
        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = atan2_approx(siny_cosp, cosy_cosp);

        EulerAngles { roll, pitch, yaw }
    }
}

#[derive(Clone, Copy)]
struct EulerAngles {
    roll: f32,   // tilt left(−) / right(+)
    pitch: f32,  // tilt forward(−) / backward(+)
    yaw: f32,    // twist left(−) / right(+)
}

/// Calibration offsets
#[derive(Clone, Copy)]
struct CalData {
    gyro_bias: Vec3,
    accel_bias: Vec3, // Z bias already has 1g removed
}

/// State for dynamic gesture detection
struct GestureState {
    // Wave detection: track roll reversals
    prev_gyro_x_sign: bool,    // true = positive
    reversal_count: u8,
    reversal_timer: u16,       // frames since first reversal

    // Punch/flick detection
    #[allow(dead_code)]
    peak_accel: f32,
    accel_cooldown: u8,

    // Twist detection
    twist_accumulator: f32,
    twist_cooldown: u8,

    // Current detected gesture (with hold timer)
    current_gesture: &'static str,
    gesture_hold: u8,          // frames to keep displaying gesture
}

impl GestureState {
    fn new() -> Self {
        Self {
            prev_gyro_x_sign: false,
            reversal_count: 0,
            reversal_timer: 0,
            peak_accel: 0.0,
            accel_cooldown: 0,
            twist_accumulator: 0.0,
            twist_cooldown: 0,
            current_gesture: "IDLE",
            gesture_hold: 0,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Math utilities (no libm/std needed)
// ═══════════════════════════════════════════════════════════════════

fn sqrt_approx(x: f32) -> f32 {
    if x <= 0.0 { return 0.0; }
    // Fast inverse sqrt (Quake III style) then invert
    let half = 0.5 * x;
    let i = f32::from_bits(0x5f3759df_u32.wrapping_sub(x.to_bits() >> 1));
    let i = i * (1.5 - half * i * i);
    let i = i * (1.5 - half * i * i); // 2nd Newton iteration for accuracy
    x * i
}

fn abs_f32(x: f32) -> f32 { if x < 0.0 { -x } else { x } }

fn atan2_approx(y: f32, x: f32) -> f32 {
    // Returns degrees
    if x == 0.0 && y == 0.0 { return 0.0; }
    let ay = abs_f32(y);
    let ax = abs_f32(x);
    let (a, b) = if ax > ay { (ay, ax) } else { (ax, ay) };
    let r = a / b;
    let rad = r * (0.9998660 + r * r * (-0.3302995 + r * r * 0.1801410));
    let mut deg = rad * RAD_TO_DEG;
    if ax < ay { deg = 90.0 - deg; }
    if x < 0.0 { deg = 180.0 - deg; }
    if y < 0.0 { deg = -deg; }
    deg
}

/// Approximate asin for range [-1, 1], returns radians
fn asin_approx(x: f32) -> f32 {
    let ax = abs_f32(x);
    if ax >= 1.0 {
        return if x > 0.0 { 1.5707963 } else { -1.5707963 };
    }
    // For small |x|, use Taylor series: asin(x) ≈ x + x³/6 + 3x⁵/40
    // For larger |x|, use identity: asin(x) = π/2 − 2·asin(√((1−x)/2))
    let result = if ax < 0.5 {
        let x2 = x * x;
        x * (1.0 + x2 * (0.16666667 + x2 * (0.075 + x2 * 0.04464286)))
    } else {
        let z = (1.0 - ax) * 0.5;
        let s = sqrt_approx(z);
        1.5707963 - 2.0 * (s + s * (z * (0.16666667 + z * (0.075 + z * 0.04464286))))
    };
    if x < 0.0 { -abs_f32(result) } else { abs_f32(result) }
}

// ═══════════════════════════════════════════════════════════════════
//  Madgwick AHRS Filter (IMU mode — no magnetometer)
// ═══════════════════════════════════════════════════════════════════
//
//  The Madgwick filter uses gradient descent to find the quaternion
//  that best aligns the measured gravity with the expected gravity
//  direction, while integrating gyroscope data for fast response.
//
//  Reference: S. Madgwick, "An efficient orientation filter for
//  inertial and inertial/magnetic sensor arrays" (2010)
//

fn madgwick_update(q: &mut Quaternion, imu: &ImuReading, beta: f32, dt: f32) {
    // Convert gyro to rad/s
    let gx = imu.gyro.x * DEG_TO_RAD;
    let gy = imu.gyro.y * DEG_TO_RAD;
    let gz = imu.gyro.z * DEG_TO_RAD;

    let mut ax = imu.accel.x;
    let mut ay = imu.accel.y;
    let mut az = imu.accel.z;

    // Normalize accelerometer (we only care about direction of gravity)
    let a_norm = sqrt_approx(ax * ax + ay * ay + az * az);
    if a_norm < 0.01 {
        // Free-fall or bad data — use gyro only
        let qw = q.w; let qx = q.x; let qy = q.y; let qz = q.z;
        q.w += (-qx * gx - qy * gy - qz * gz) * 0.5 * dt;
        q.x += (qw * gx + qy * gz - qz * gy) * 0.5 * dt;
        q.y += (qw * gy - qx * gz + qz * gx) * 0.5 * dt;
        q.z += (qw * gz + qx * gy - qy * gx) * 0.5 * dt;
        q.normalize();
        return;
    }
    let inv_a = 1.0 / a_norm;
    ax *= inv_a;
    ay *= inv_a;
    az *= inv_a;

    let qw = q.w; let qx = q.x; let qy = q.y; let qz = q.z;

    // Auxiliary variables to avoid repeated arithmetic
    let _2qw = 2.0 * qw;
    let _2qx = 2.0 * qx;
    let _2qy = 2.0 * qy;
    let _2qz = 2.0 * qz;
    let _4qw = 4.0 * qw;
    let _4qx = 4.0 * qx;
    let _4qy = 4.0 * qy;
    let _8qx = 8.0 * qx;
    let _8qy = 8.0 * qy;
    let qw2 = qw * qw;
    let qx2 = qx * qx;
    let qy2 = qy * qy;
    let qz2 = qz * qz;

    // Gradient descent corrective step
    // Objective function: f = R(q)⁻¹ * [0,0,1]ᵀ − [ax,ay,az]ᵀ
    let s0 = _4qw * qy2 + _2qy * ax + _4qw * qx2 - _2qx * ay;
    let s1 = _4qx * qz2 - _2qz * ax + 4.0 * qw2 * qx - _2qw * ay - _4qx + _8qx * qx2 + _8qx * qy2 + _4qx * az;
    let s2 = 4.0 * qw2 * qy + _2qw * ax + _4qy * qz2 - _2qz * ay - _4qy + _8qy * qx2 + _8qy * qy2 + _4qy * az;
    let s3 = 4.0 * qx2 * qz - _2qx * ax + 4.0 * qy2 * qz - _2qy * ay;

    // Normalize step
    let s_norm = sqrt_approx(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if s_norm < 1e-10 {
        // Already aligned, just integrate gyro
        q.w += (-qx * gx - qy * gy - qz * gz) * 0.5 * dt;
        q.x += (qw * gx + qy * gz - qz * gy) * 0.5 * dt;
        q.y += (qw * gy - qx * gz + qz * gx) * 0.5 * dt;
        q.z += (qw * gz + qx * gy - qy * gx) * 0.5 * dt;
        q.normalize();
        return;
    }
    let inv_s = 1.0 / s_norm;
    let s0 = s0 * inv_s;
    let s1 = s1 * inv_s;
    let s2 = s2 * inv_s;
    let s3 = s3 * inv_s;

    // Rate of change = gyro integration − beta * gradient
    let qd_w = (-qx * gx - qy * gy - qz * gz) * 0.5 - beta * s0;
    let qd_x = (qw * gx + qy * gz - qz * gy) * 0.5 - beta * s1;
    let qd_y = (qw * gy - qx * gz + qz * gx) * 0.5 - beta * s2;
    let qd_z = (qw * gz + qx * gy - qy * gx) * 0.5 - beta * s3;

    // Integrate
    q.w += qd_w * dt;
    q.x += qd_x * dt;
    q.y += qd_y * dt;
    q.z += qd_z * dt;
    q.normalize();
}

// ═══════════════════════════════════════════════════════════════════
//  Dynamic Gesture Detection
// ═══════════════════════════════════════════════════════════════════

fn update_gesture(gs: &mut GestureState, imu: &ImuReading, euler: &EulerAngles) {
    // Tick down cooldowns
    if gs.accel_cooldown > 0 { gs.accel_cooldown -= 1; }
    if gs.twist_cooldown > 0 { gs.twist_cooldown -= 1; }
    if gs.gesture_hold > 0 {
        gs.gesture_hold -= 1;
        if gs.gesture_hold > 0 { return; } // keep showing current gesture
    }

    let accel_mag = imu.accel.magnitude();
    let gyro_mag = imu.gyro.magnitude();

    // ── 1. SNAP / PUNCH detection (sharp linear acceleration) ──
    if accel_mag > GESTURE_SNAP_THRESHOLD && gs.accel_cooldown == 0 {
        gs.current_gesture = "SNAP!";
        gs.gesture_hold = 40; // hold for ~200ms
        gs.accel_cooldown = 60;
        return;
    }
    if accel_mag > GESTURE_ACCEL_THRESHOLD && gs.accel_cooldown == 0 {
        // Determine direction from accel axis
        if abs_f32(imu.accel.x) > abs_f32(imu.accel.y) && abs_f32(imu.accel.x) > abs_f32(imu.accel.z) {
            gs.current_gesture = if imu.accel.x > 0.0 { "PUSH FORWARD" } else { "PULL BACK" };
        } else if abs_f32(imu.accel.y) > abs_f32(imu.accel.z) {
            gs.current_gesture = if imu.accel.y > 0.0 { "FLICK LEFT" } else { "FLICK RIGHT" };
        } else {
            gs.current_gesture = if imu.accel.z > 1.0 { "FLICK UP" } else { "FLICK DOWN" };
        }
        gs.gesture_hold = 30;
        gs.accel_cooldown = 40;
        return;
    }

    // ── 2. WAVE detection (rapid roll reversals) ──
    let gx_positive = imu.gyro.x > 15.0;
    let gx_negative = imu.gyro.x < -15.0;
    if (gx_positive && !gs.prev_gyro_x_sign) || (gx_negative && gs.prev_gyro_x_sign) {
        if gx_positive || gx_negative {
            gs.reversal_count += 1;
            gs.prev_gyro_x_sign = gx_positive;
        }
    }
    // Reset wave if too slow
    if gs.reversal_timer > 0 {
        gs.reversal_timer += 1;
        if gs.reversal_timer > 150 { // ~750ms window
            gs.reversal_count = 0;
            gs.reversal_timer = 0;
        }
    }
    if gs.reversal_count == 1 && gs.reversal_timer == 0 {
        gs.reversal_timer = 1; // start counting
    }
    if gs.reversal_count >= WAVE_REVERSAL_COUNT {
        gs.current_gesture = "WAVE!";
        gs.gesture_hold = 60;
        gs.reversal_count = 0;
        gs.reversal_timer = 0;
        return;
    }

    // ── 3. TWIST detection (yaw rotation) ──
    if abs_f32(imu.gyro.z) > GESTURE_GYRO_THRESHOLD && gs.twist_cooldown == 0 {
        gs.twist_accumulator += imu.gyro.z * DT;
        if abs_f32(gs.twist_accumulator) > 30.0 {
            gs.current_gesture = if gs.twist_accumulator > 0.0 { "TWIST CW" } else { "TWIST CCW" };
            gs.gesture_hold = 40;
            gs.twist_cooldown = 60;
            gs.twist_accumulator = 0.0;
            return;
        }
    } else {
        gs.twist_accumulator *= 0.9; // decay if not twisting
    }

    // ── 4. STATIC pose detection from Euler angles ──
    if gyro_mag < 10.0 {
        // Hand is relatively still — classify static pose
        gs.current_gesture = classify_static_pose(euler);
    } else {
        gs.current_gesture = "MOVING";
    }
}

fn classify_static_pose(e: &EulerAngles) -> &'static str {
    let r = e.roll;
    let p = e.pitch;

    if p < -60.0            { "POINT DOWN" }
    else if p > 60.0        { "POINT UP" }
    else if r < -60.0       { "TILT LEFT" }
    else if r > 60.0        { "TILT RIGHT" }
    else if p < -25.0       { "SLIGHT DOWN" }
    else if p > 25.0        { "SLIGHT UP" }
    else if r < -25.0       { "SLIGHT LEFT" }
    else if r > 25.0        { "SLIGHT RIGHT" }
    else                    { "FLAT" }
}

// ═══════════════════════════════════════════════════════════════════
//  IMU driver
// ═══════════════════════════════════════════════════════════════════

fn init_imu<I: I2cTrait>(i2c: &mut I, delay: &mut Delay) -> Result<(), I::Error> {
    let mut buf = [0u8; 1];
    i2c.write_read(IMU_ADDR, &[REG_WHO_AM_I], &mut buf)?;
    info!("WHO_AM_I: 0x{:02X} (expected 0x6C for LSM6DSO)", buf[0]);

    // Software reset
    i2c.write(IMU_ADDR, &[REG_CTRL3_C, 0x01])?;
    delay.delay_millis(50);

    // BDU on + IF_INC (auto-increment for burst reads)
    i2c.write(IMU_ADDR, &[REG_CTRL3_C, 0x44])?;
    delay.delay_millis(10);

    // Accel: 208 Hz ODR, ±2g  (0x50 = 208Hz | ±2g)
    i2c.write(IMU_ADDR, &[REG_CTRL1_XL, 0x50])?;
    delay.delay_millis(10);

    // Gyro: 208 Hz ODR, ±500dps  (0x54 = 208Hz | ±500dps)
    i2c.write(IMU_ADDR, &[REG_CTRL2_G, 0x54])?;
    delay.delay_millis(10);

    // High-performance mode for accel & gyro (default, but make sure)
    i2c.write(IMU_ADDR, &[REG_CTRL6_C, 0x00])?; // accel high-perf
    i2c.write(IMU_ADDR, &[REG_CTRL7_G, 0x00])?; // gyro high-perf
    delay.delay_millis(10);

    // Discard first 50 samples (sensor settling + filter warm-up)
    for _ in 0..50 {
        let mut tmp = [0u8; 12];
        let _ = i2c.write_read(IMU_ADDR, &[REG_OUTX_L_G], &mut tmp);
        delay.delay_millis(5);
    }

    info!("LSM6DSO: 208Hz, +/-2g, +/-500dps, high-perf mode");
    Ok(())
}

fn read_raw<I: I2cTrait>(i2c: &mut I) -> Result<ImuReading, I::Error> {
    let mut buf = [0u8; 12];
    i2c.write_read(IMU_ADDR, &[REG_OUTX_L_G], &mut buf)?;

    let gx = (buf[0] as i16) | ((buf[1] as i16) << 8);
    let gy = (buf[2] as i16) | ((buf[3] as i16) << 8);
    let gz = (buf[4] as i16) | ((buf[5] as i16) << 8);
    let ax = (buf[6] as i16) | ((buf[7] as i16) << 8);
    let ay = (buf[8] as i16) | ((buf[9] as i16) << 8);
    let az = (buf[10] as i16) | ((buf[11] as i16) << 8);

    Ok(ImuReading {
        gyro: Vec3 {
            x: (gx as f32) * GYRO_MDPS_PER_LSB * 0.001,
            y: (gy as f32) * GYRO_MDPS_PER_LSB * 0.001,
            z: (gz as f32) * GYRO_MDPS_PER_LSB * 0.001,
        },
        accel: Vec3 {
            x: (ax as f32) * ACCEL_MG_PER_LSB * 0.001,
            y: (ay as f32) * ACCEL_MG_PER_LSB * 0.001,
            z: (az as f32) * ACCEL_MG_PER_LSB * 0.001,
        },
    })
}

fn calibrate<I: I2cTrait>(i2c: &mut I, delay: &mut Delay) -> Result<CalData, I::Error> {
    info!("========================================");
    info!("  CALIBRATING — KEEP HAND FLAT & STILL");
    info!("  (500 samples, ~5 seconds)");
    info!("========================================");

    let (mut sgx, mut sgy, mut sgz) = (0.0_f32, 0.0_f32, 0.0_f32);
    let (mut sax, mut say, mut saz) = (0.0_f32, 0.0_f32, 0.0_f32);

    for i in 0..CALIBRATION_SAMPLES {
        let r = read_raw(i2c)?;
        sgx += r.gyro.x;  sgy += r.gyro.y;  sgz += r.gyro.z;
        sax += r.accel.x;  say += r.accel.y;  saz += r.accel.z;
        if i % 100 == 0 { info!("  calibrating {}/{}...", i, CALIBRATION_SAMPLES); }
        delay.delay_millis(10);
    }

    let n = CALIBRATION_SAMPLES as f32;
    let cal = CalData {
        gyro_bias: Vec3 { x: sgx / n, y: sgy / n, z: sgz / n },
        accel_bias: Vec3 { x: sax / n, y: say / n, z: (saz / n) - 1.0 },
    };

    info!("Calibration done!");
    info!("  Gyro bias:  X={:.3} Y={:.3} Z={:.3} dps", cal.gyro_bias.x, cal.gyro_bias.y, cal.gyro_bias.z);
    info!("  Accel bias: X={:.4} Y={:.4} Z={:.4} g", cal.accel_bias.x, cal.accel_bias.y, cal.accel_bias.z);
    Ok(cal)
}

fn apply_cal(raw: &ImuReading, cal: &CalData) -> ImuReading {
    ImuReading {
        gyro: Vec3 {
            x: raw.gyro.x - cal.gyro_bias.x,
            y: raw.gyro.y - cal.gyro_bias.y,
            z: raw.gyro.z - cal.gyro_bias.z,
        },
        accel: Vec3 {
            x: raw.accel.x - cal.accel_bias.x,
            y: raw.accel.y - cal.accel_bias.y,
            z: raw.accel.z - cal.accel_bias.z,
        },
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    let mut i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .expect("Failed to create I2C")
    .with_sda(peripherals.GPIO21)
    .with_scl(peripherals.GPIO22);

    let mut delay = Delay::new();

    // ── I2C scan ──
    info!("Scanning I2C...");
    for addr in 0x08..0x78 {
        let mut b = [0u8; 1];
        if i2c.read(addr, &mut b).is_ok() {
            info!("  Device at 0x{:02X}", addr);
        }
    }

    // ── Init ──
    if let Err(e) = init_imu(&mut i2c, &mut delay) {
        log::error!("IMU init failed: {:?}", e);
        loop { Timer::after(Duration::from_secs(1)).await; }
    }

    // ── Calibrate ──
    delay.delay_millis(500);
    let cal = match calibrate(&mut i2c, &mut delay) {
        Ok(c) => c,
        Err(e) => {
            log::error!("Calibration failed: {:?}", e);
            loop { Timer::after(Duration::from_secs(1)).await; }
        }
    };

    // ── Init Madgwick filter with identity quaternion ──
    let mut q = Quaternion::identity();
    let mut gesture = GestureState::new();

    // Warm up the filter for ~1 second so it converges from identity
    info!("Warming up Madgwick filter...");
    for _ in 0..200 {
        if let Ok(raw) = read_raw(&mut i2c) {
            let imu = apply_cal(&raw, &cal);
            // Use higher beta during warm-up for faster convergence
            madgwick_update(&mut q, &imu, 0.5, DT);
        }
        delay.delay_millis(SAMPLE_MS as u32);
    }

    info!("=============================================");
    info!("  GESTURE GLOVE READY — Madgwick AHRS active");
    info!("  Move your hand!");
    info!("=============================================");

    let _ = spawner;
    let mut tick: u32 = 0;

    loop {
        match read_raw(&mut i2c) {
            Ok(raw) => {
                let imu = apply_cal(&raw, &cal);

                // ── Madgwick filter update ──
                madgwick_update(&mut q, &imu, MADGWICK_BETA, DT);

                // ── Extract Euler angles ──
                let euler = q.to_euler_deg();

                // ── Gesture detection ──
                update_gesture(&mut gesture, &imu, &euler);

                // Print at ~5 Hz (every 40th sample at 200 Hz)
                if tick % 40 == 0 {
                    info!("─────────────────────────────────────────");
                    info!(
                        "Roll:{:>7.1}°  Pitch:{:>7.1}°  Yaw:{:>7.1}°",
                        euler.roll, euler.pitch, euler.yaw
                    );
                    info!(
                        "Quat: w={:.3} x={:.3} y={:.3} z={:.3}",
                        q.w, q.x, q.y, q.z
                    );
                    info!(
                        "Gyro  {:>6.1} {:>6.1} {:>6.1} dps  |  Accel {:>5.2} {:>5.2} {:>5.2} g",
                        imu.gyro.x, imu.gyro.y, imu.gyro.z,
                        imu.accel.x, imu.accel.y, imu.accel.z
                    );
                    info!(">>> {}", gesture.current_gesture);
                }
            }
            Err(e) => {
                log::error!("Read error: {:?}", e);
            }
        }

        tick = tick.wrapping_add(1);
        Timer::after(Duration::from_millis(SAMPLE_MS)).await;
    }
}
