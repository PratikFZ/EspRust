/// Madgwick AHRS filter, math utilities, and gesture detection.
///
/// Ported from gyro-scope project — no_std, no libm, no floats in hardware.

// ═══════════════════════════════════════════════════════════════════
//  Constants
// ═══════════════════════════════════════════════════════════════════

pub const DEG_TO_RAD: f32 = 0.017453293; // π/180
pub const RAD_TO_DEG: f32 = 57.295780; // 180/π

// Gesture detection thresholds
const GESTURE_GYRO_THRESHOLD: f32 = 80.0; // dps — fast rotation
const GESTURE_ACCEL_THRESHOLD: f32 = 1.8; // g — sharp acceleration
const GESTURE_SNAP_THRESHOLD: f32 = 2.5; // g — very sharp snap
const WAVE_REVERSAL_COUNT: u8 = 3;
const CIRCLE_MIN_SAMPLES: u16 = 40; // ~200ms at 200Hz
const CIRCLE_MAX_SAMPLES: u16 = 400; // ~2s
const NOD_THRESHOLD: f32 = 40.0; // dps pitch for nod
const SHAKE_THRESHOLD: f32 = 40.0; // dps yaw for shake

// ═══════════════════════════════════════════════════════════════════
//  Data types
// ═══════════════════════════════════════════════════════════════════

#[derive(Debug, Clone, Copy, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub fn magnitude(&self) -> f32 {
        sqrt_approx(self.x * self.x + self.y * self.y + self.z * self.z)
    }
}

/// Sensor data in physical units
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuReading {
    pub gyro: Vec3,  // degrees/sec
    pub accel: Vec3, // g
}

/// Unit quaternion (3-D orientation — no gimbal lock)
#[derive(Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn normalize(&mut self) {
        let n = sqrt_approx(
            self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z,
        );
        if n > 0.0 {
            let inv = 1.0 / n;
            self.w *= inv;
            self.x *= inv;
            self.y *= inv;
            self.z *= inv;
        }
    }

    /// Extract Euler angles (aerospace ZYX convention)
    pub fn to_euler_deg(&self) -> EulerAngles {
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
pub struct EulerAngles {
    pub roll: f32,  // tilt left(−) / right(+)
    pub pitch: f32, // tilt forward(−) / backward(+)
    pub yaw: f32,   // twist left(−) / right(+)
}

/// Calibration offsets (gyro bias + accel bias with 1g removed from Z)
#[derive(Clone, Copy, Default)]
pub struct CalData {
    pub gyro_bias: Vec3,
    pub accel_bias: Vec3,
}

impl CalData {
    /// Apply calibration to a raw reading
    pub fn apply(&self, raw: &ImuReading) -> ImuReading {
        ImuReading {
            gyro: Vec3 {
                x: raw.gyro.x - self.gyro_bias.x,
                y: raw.gyro.y - self.gyro_bias.y,
                z: raw.gyro.z - self.gyro_bias.z,
            },
            accel: Vec3 {
                x: raw.accel.x - self.accel_bias.x,
                y: raw.accel.y - self.accel_bias.y,
                z: raw.accel.z - self.accel_bias.z,
            },
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Math utilities (no libm / no std)
// ═══════════════════════════════════════════════════════════════════

pub fn sqrt_approx(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let half = 0.5 * x;
    let i = f32::from_bits(0x5f3759df_u32.wrapping_sub(x.to_bits() >> 1));
    let i = i * (1.5 - half * i * i);
    let i = i * (1.5 - half * i * i);
    x * i
}

pub fn abs_f32(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

pub fn atan2_approx(y: f32, x: f32) -> f32 {
    if x == 0.0 && y == 0.0 {
        return 0.0;
    }
    let ay = abs_f32(y);
    let ax = abs_f32(x);
    let (a, b) = if ax > ay { (ay, ax) } else { (ax, ay) };
    let r = a / b;
    let rad = r * (0.9998660 + r * r * (-0.3302995 + r * r * 0.1801410));
    let mut deg = rad * RAD_TO_DEG;
    if ax < ay {
        deg = 90.0 - deg;
    }
    if x < 0.0 {
        deg = 180.0 - deg;
    }
    if y < 0.0 {
        deg = -deg;
    }
    deg
}

pub fn asin_approx(x: f32) -> f32 {
    let ax = abs_f32(x);
    if ax >= 1.0 {
        return if x > 0.0 { 1.5707963 } else { -1.5707963 };
    }
    let result = if ax < 0.5 {
        let x2 = x * x;
        x * (1.0 + x2 * (0.16666667 + x2 * (0.075 + x2 * 0.04464286)))
    } else {
        let z = (1.0 - ax) * 0.5;
        let s = sqrt_approx(z);
        1.5707963 - 2.0 * (s + s * (z * (0.16666667 + z * (0.075 + z * 0.04464286))))
    };
    if x < 0.0 {
        -abs_f32(result)
    } else {
        abs_f32(result)
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Madgwick AHRS Filter (IMU mode — no magnetometer)
// ═══════════════════════════════════════════════════════════════════

pub fn madgwick_update(q: &mut Quaternion, imu: &ImuReading, beta: f32, dt: f32) {
    let gx = imu.gyro.x * DEG_TO_RAD;
    let gy = imu.gyro.y * DEG_TO_RAD;
    let gz = imu.gyro.z * DEG_TO_RAD;

    let mut ax = imu.accel.x;
    let mut ay = imu.accel.y;
    let mut az = imu.accel.z;

    let a_norm = sqrt_approx(ax * ax + ay * ay + az * az);
    if a_norm < 0.01 {
        // Free-fall / bad data — gyro only
        let (qw, qx, qy, qz) = (q.w, q.x, q.y, q.z);
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

    let (qw, qx, qy, qz) = (q.w, q.x, q.y, q.z);

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

    let s0 = _4qw * qy2 + _2qy * ax + _4qw * qx2 - _2qx * ay;
    let s1 = _4qx * qz2 - _2qz * ax + 4.0 * qw2 * qx - _2qw * ay - _4qx + _8qx * qx2
        + _8qx * qy2
        + _4qx * az;
    let s2 = 4.0 * qw2 * qy + _2qw * ax + _4qy * qz2 - _2qz * ay - _4qy + _8qy * qx2
        + _8qy * qy2
        + _4qy * az;
    let s3 = 4.0 * qx2 * qz - _2qx * ax + 4.0 * qy2 * qz - _2qy * ay;

    let s_norm = sqrt_approx(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if s_norm < 1e-10 {
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

    let qd_w = (-qx * gx - qy * gy - qz * gz) * 0.5 - beta * s0;
    let qd_x = (qw * gx + qy * gz - qz * gy) * 0.5 - beta * s1;
    let qd_y = (qw * gy - qx * gz + qz * gx) * 0.5 - beta * s2;
    let qd_z = (qw * gz + qx * gy - qy * gx) * 0.5 - beta * s3;

    q.w += qd_w * dt;
    q.x += qd_x * dt;
    q.y += qd_y * dt;
    q.z += qd_z * dt;
    q.normalize();
}

// ═══════════════════════════════════════════════════════════════════
//  Per-sensor Gesture Detection
// ═══════════════════════════════════════════════════════════════════

pub struct GestureState {
    // Wave: rapid roll reversals
    prev_gyro_x_sign: bool,
    reversal_count: u8,
    reversal_timer: u16,

    // Nod: pitch reversals (yes gesture)
    prev_gyro_y_sign: bool,
    nod_count: u8,
    nod_timer: u16,

    // Shake: yaw reversals (no gesture)
    prev_gyro_z_sign: bool,
    shake_count: u8,
    shake_timer: u16,

    // Punch / flick cooldown
    accel_cooldown: u8,

    // Twist accumulator
    twist_accumulator: f32,
    twist_cooldown: u8,

    // Circle detection: sustained rotation
    circle_samples: u16,
    circle_axis_sum: Vec3, // accumulated gyro during circle

    // Chop detection: sharp downward
    chop_cooldown: u8,

    pub current_gesture: &'static str,
    gesture_hold: u8,
}

impl GestureState {
    pub fn new() -> Self {
        Self {
            prev_gyro_x_sign: false,
            reversal_count: 0,
            reversal_timer: 0,
            prev_gyro_y_sign: false,
            nod_count: 0,
            nod_timer: 0,
            prev_gyro_z_sign: false,
            shake_count: 0,
            shake_timer: 0,
            accel_cooldown: 0,
            twist_accumulator: 0.0,
            twist_cooldown: 0,
            circle_samples: 0,
            circle_axis_sum: Vec3::default(),
            chop_cooldown: 0,
            current_gesture: "IDLE",
            gesture_hold: 0,
        }
    }

    pub fn update(&mut self, imu: &ImuReading, euler: &EulerAngles, dt: f32) {
        // Tick down cooldowns
        if self.accel_cooldown > 0 { self.accel_cooldown -= 1; }
        if self.twist_cooldown > 0 { self.twist_cooldown -= 1; }
        if self.chop_cooldown > 0 { self.chop_cooldown -= 1; }
        if self.gesture_hold > 0 {
            self.gesture_hold -= 1;
            if self.gesture_hold > 0 { return; }
        }

        let accel_mag = imu.accel.magnitude();
        let gyro_mag = imu.gyro.magnitude();

        // ── 1. SNAP (very sharp acceleration) ────────────
        if accel_mag > GESTURE_SNAP_THRESHOLD && self.accel_cooldown == 0 {
            self.current_gesture = "SNAP!";
            self.gesture_hold = 40;
            self.accel_cooldown = 60;
            return;
        }

        // ── 2. CHOP (sharp downward motion) ──────────────
        if imu.accel.z < -GESTURE_ACCEL_THRESHOLD && self.chop_cooldown == 0 && self.accel_cooldown == 0 {
            self.current_gesture = "CHOP!";
            self.gesture_hold = 35;
            self.chop_cooldown = 50;
            self.accel_cooldown = 40;
            return;
        }

        // ── 3. Directional FLICK / PUSH / PULL ──────────
        if accel_mag > GESTURE_ACCEL_THRESHOLD && self.accel_cooldown == 0 {
            if abs_f32(imu.accel.x) > abs_f32(imu.accel.y)
                && abs_f32(imu.accel.x) > abs_f32(imu.accel.z)
            {
                self.current_gesture = if imu.accel.x > 0.0 { "PUSH FORWARD" } else { "PULL BACK" };
            } else if abs_f32(imu.accel.y) > abs_f32(imu.accel.z) {
                self.current_gesture = if imu.accel.y > 0.0 { "FLICK LEFT" } else { "FLICK RIGHT" };
            } else {
                self.current_gesture = if imu.accel.z > 1.0 { "FLICK UP" } else { "FLICK DOWN" };
            }
            self.gesture_hold = 30;
            self.accel_cooldown = 40;
            return;
        }

        // ── 4. WAVE (rapid roll/X reversals) ────────────
        if self.detect_reversals(
            imu.gyro.x, 15.0,
            &mut self.prev_gyro_x_sign.clone(),
            &mut self.reversal_count.clone(),
            &mut self.reversal_timer.clone(),
        ) {
            // we do it inline to avoid borrow issues
        }
        {
            let gx_pos = imu.gyro.x > 15.0;
            let gx_neg = imu.gyro.x < -15.0;
            if (gx_pos && !self.prev_gyro_x_sign) || (gx_neg && self.prev_gyro_x_sign) {
                if gx_pos || gx_neg {
                    self.reversal_count += 1;
                    self.prev_gyro_x_sign = gx_pos;
                }
            }
            if self.reversal_timer > 0 {
                self.reversal_timer += 1;
                if self.reversal_timer > 150 { self.reversal_count = 0; self.reversal_timer = 0; }
            }
            if self.reversal_count == 1 && self.reversal_timer == 0 { self.reversal_timer = 1; }
            if self.reversal_count >= WAVE_REVERSAL_COUNT {
                self.current_gesture = "WAVE!";
                self.gesture_hold = 60;
                self.reversal_count = 0;
                self.reversal_timer = 0;
                return;
            }
        }

        // ── 5. NOD / YES (pitch/Y reversals) ────────────
        {
            let gy_pos = imu.gyro.y > NOD_THRESHOLD;
            let gy_neg = imu.gyro.y < -NOD_THRESHOLD;
            if (gy_pos && !self.prev_gyro_y_sign) || (gy_neg && self.prev_gyro_y_sign) {
                if gy_pos || gy_neg {
                    self.nod_count += 1;
                    self.prev_gyro_y_sign = gy_pos;
                }
            }
            if self.nod_timer > 0 {
                self.nod_timer += 1;
                if self.nod_timer > 200 { self.nod_count = 0; self.nod_timer = 0; }
            }
            if self.nod_count == 1 && self.nod_timer == 0 { self.nod_timer = 1; }
            if self.nod_count >= 2 {
                self.current_gesture = "NOD (YES)";
                self.gesture_hold = 60;
                self.nod_count = 0;
                self.nod_timer = 0;
                return;
            }
        }

        // ── 6. SHAKE / NO (yaw/Z reversals) ─────────────
        {
            let gz_pos = imu.gyro.z > SHAKE_THRESHOLD;
            let gz_neg = imu.gyro.z < -SHAKE_THRESHOLD;
            if (gz_pos && !self.prev_gyro_z_sign) || (gz_neg && self.prev_gyro_z_sign) {
                if gz_pos || gz_neg {
                    self.shake_count += 1;
                    self.prev_gyro_z_sign = gz_pos;
                }
            }
            if self.shake_timer > 0 {
                self.shake_timer += 1;
                if self.shake_timer > 200 { self.shake_count = 0; self.shake_timer = 0; }
            }
            if self.shake_count == 1 && self.shake_timer == 0 { self.shake_timer = 1; }
            if self.shake_count >= 3 {
                self.current_gesture = "SHAKE (NO)";
                self.gesture_hold = 60;
                self.shake_count = 0;
                self.shake_timer = 0;
                return;
            }
        }

        // ── 7. TWIST CW / CCW (sustained yaw) ──────────
        if abs_f32(imu.gyro.z) > GESTURE_GYRO_THRESHOLD && self.twist_cooldown == 0 {
            self.twist_accumulator += imu.gyro.z * dt;
            if abs_f32(self.twist_accumulator) > 30.0 {
                self.current_gesture = if self.twist_accumulator > 0.0 { "TWIST CW" } else { "TWIST CCW" };
                self.gesture_hold = 40;
                self.twist_cooldown = 60;
                self.twist_accumulator = 0.0;
                return;
            }
        } else {
            self.twist_accumulator *= 0.9;
        }

        // ── 8. CIRCLE (sustained rotation on one axis) ──
        if gyro_mag > 50.0 {
            self.circle_samples += 1;
            self.circle_axis_sum.x += imu.gyro.x * dt;
            self.circle_axis_sum.y += imu.gyro.y * dt;
            self.circle_axis_sum.z += imu.gyro.z * dt;

            if self.circle_samples >= CIRCLE_MIN_SAMPLES {
                // Check if accumulated rotation is large enough (~180°+)
                let rot_mag = self.circle_axis_sum.magnitude();
                if rot_mag > 120.0 {
                    self.current_gesture = "CIRCLE!";
                    self.gesture_hold = 60;
                    self.circle_samples = 0;
                    self.circle_axis_sum = Vec3::default();
                    return;
                }
            }
            if self.circle_samples > CIRCLE_MAX_SAMPLES {
                self.circle_samples = 0;
                self.circle_axis_sum = Vec3::default();
            }
        } else {
            // Decaying — if stopped rotating, reset
            if self.circle_samples > 0 {
                self.circle_samples = self.circle_samples.saturating_sub(2);
                if self.circle_samples == 0 {
                    self.circle_axis_sum = Vec3::default();
                }
            }
        }

        // ── 9. STATIC pose classification ───────────────
        if gyro_mag < 10.0 {
            self.current_gesture = classify_static_pose(euler);
        } else {
            self.current_gesture = "MOVING";
        }
    }

    // Helper — not actually used (inline version above), but keeps API clean
    fn detect_reversals(
        &self, _val: f32, _thresh: f32,
        _prev_sign: &mut bool, _count: &mut u8, _timer: &mut u16,
    ) -> bool {
        false
    }
}

fn classify_static_pose(e: &EulerAngles) -> &'static str {
    let r = e.roll;
    let p = e.pitch;

    if p < -60.0       { "POINT DOWN" }
    else if p > 60.0   { "POINT UP" }
    else if r < -60.0  { "TILT LEFT" }
    else if r > 60.0   { "TILT RIGHT" }
    else if p < -25.0  { "SLIGHT DOWN" }
    else if p > 25.0   { "SLIGHT UP" }
    else if r < -25.0  { "SLIGHT LEFT" }
    else if r > 25.0   { "SLIGHT RIGHT" }
    else               { "FLAT" }
}

// ═══════════════════════════════════════════════════════════════════
//  Combined Dual-Sensor Gesture (palm + finger)
// ═══════════════════════════════════════════════════════════════════
//
//  Uses BOTH IMUs to detect gestures that need context from two
//  body points — e.g. "finger pointing while palm is flat" = POINT
//

pub struct DualGestureState {
    pub current_gesture: &'static str,
    gesture_hold: u8,

    // Beckon: finger curling repeatedly (finger pitch oscillation while palm steady)
    finger_pitch_prev_sign: bool,
    beckon_count: u8,
    beckon_timer: u16,
}

impl DualGestureState {
    pub fn new() -> Self {
        Self {
            current_gesture: "IDLE",
            gesture_hold: 0,
            finger_pitch_prev_sign: false,
            beckon_count: 0,
            beckon_timer: 0,
        }
    }

    /// Call every sample with data from BOTH sensors + flex bend data
    ///
    /// `flex_bends`: array of 5 bend percentages [thumb, index, middle, ring, pinky]
    ///               pass `None` if flex sensors not yet available
    pub fn update(
        &mut self,
        palm_imu: &ImuReading,
        palm_euler: &EulerAngles,
        finger_imu: &ImuReading,
        finger_euler: &EulerAngles,
        flex_bends: Option<&[f32; 5]>,
    ) {
        if self.gesture_hold > 0 {
            self.gesture_hold -= 1;
            if self.gesture_hold > 0 { return; }
        }

        let palm_gyro_mag = palm_imu.gyro.magnitude();
        let finger_gyro_mag = finger_imu.gyro.magnitude();
        let palm_still = palm_gyro_mag < 15.0;
        let finger_still = finger_gyro_mag < 15.0;

        // ── Flex-based hand pose (highest priority when static) ──
        if let Some(b) = flex_bends {
            let all_bent = b.iter().all(|&v| v > 65.0);
            let all_straight = b.iter().all(|&v| v < 25.0);
            let index_straight = b[1] < 30.0; // INDEX = 1
            let others_bent = b[0] > 50.0 && b[2] > 50.0 && b[3] > 50.0 && b[4] > 50.0;

            // Flex + IMU combined gestures
            if palm_still && finger_still {
                // FIST + palm face down
                if all_bent && palm_euler.roll > 40.0 {
                    self.current_gesture = "FIST (face down)";
                    return;
                }
                if all_bent {
                    self.current_gesture = "FIST";
                    return;
                }

                // POINTING + finger aimed up/forward
                if index_straight && others_bent {
                    let fp = finger_euler.pitch;
                    if fp > 40.0 {
                        self.current_gesture = "POINT UP";
                        return;
                    } else if fp < -40.0 {
                        self.current_gesture = "POINT DOWN";
                        return;
                    } else {
                        self.current_gesture = "POINTING";
                        return;
                    }
                }

                // PEACE / V sign
                if b[0] > 50.0 && b[1] < 30.0 && b[2] < 30.0 && b[3] > 50.0 && b[4] > 50.0 {
                    self.current_gesture = "PEACE / V";
                    return;
                }

                // THUMBS UP: thumb straight, rest bent, palm rolled
                if b[0] < 30.0 && others_bent && abs_f32(palm_euler.roll) > 50.0 {
                    if finger_euler.pitch > 20.0 {
                        self.current_gesture = "THUMBS UP";
                    } else {
                        self.current_gesture = "THUMBS DOWN";
                    }
                    return;
                }

                // HANG LOOSE (shaka): thumb + pinky out
                if b[0] < 30.0 && b[1] > 50.0 && b[2] > 50.0 && b[3] > 50.0 && b[4] < 30.0 {
                    self.current_gesture = "HANG LOOSE";
                    return;
                }

                // ROCK ON: index + pinky out
                if b[0] > 50.0 && b[1] < 30.0 && b[2] > 50.0 && b[3] > 50.0 && b[4] < 30.0 {
                    self.current_gesture = "ROCK ON";
                    return;
                }

                // THREE: index + middle + ring
                if b[0] > 50.0 && b[1] < 30.0 && b[2] < 30.0 && b[3] < 30.0 && b[4] > 50.0 {
                    self.current_gesture = "THREE";
                    return;
                }

                // FOUR: all except thumb
                if b[0] > 50.0 && b[1] < 30.0 && b[2] < 30.0 && b[3] < 30.0 && b[4] < 30.0 {
                    self.current_gesture = "FOUR";
                    return;
                }

                // OPEN HAND
                if all_straight {
                    if abs_f32(palm_euler.roll) < 20.0 && abs_f32(palm_euler.pitch) < 20.0 {
                        self.current_gesture = "OPEN HAND";
                    } else if palm_euler.roll < -50.0 {
                        self.current_gesture = "PALM UP (open)";
                    } else if palm_euler.roll > 50.0 {
                        self.current_gesture = "PALM DOWN (open)";
                    } else {
                        self.current_gesture = "HAND FLAT";
                    }
                    return;
                }

                // OK sign: thumb + index make circle (both partially bent), rest straight
                if b[0] > 30.0 && b[0] < 70.0 && b[1] > 30.0 && b[1] < 70.0
                    && b[2] < 30.0 && b[3] < 30.0 && b[4] < 30.0
                {
                    self.current_gesture = "OK";
                    return;
                }
            }

            // Dynamic + flex: WAVE with open hand
            if all_straight && palm_gyro_mag > 30.0 {
                // Waving with open hand (more natural wave detection)
                self.current_gesture = "WAVE (open)";
                return;
            }

            // FIST PUMP: fist + upward accel
            if all_bent && palm_imu.accel.z > 1.5 {
                self.current_gesture = "FIST PUMP!";
                self.gesture_hold = 30;
                return;
            }

            // BECKONING: index finger cycling bend while palm still
            // (detected by flex change, not just gyro)
        }

        // ── Fallback: IMU-only gestures (no flex or flex inconclusive) ──

        // BECKON (finger curling while palm steady)
        if palm_still && abs_f32(finger_imu.gyro.y) > 25.0 {
            let fy_pos = finger_imu.gyro.y > 25.0;
            let fy_neg = finger_imu.gyro.y < -25.0;
            if (fy_pos && !self.finger_pitch_prev_sign) || (fy_neg && self.finger_pitch_prev_sign) {
                if fy_pos || fy_neg {
                    self.beckon_count += 1;
                    self.finger_pitch_prev_sign = fy_pos;
                }
            }
            if self.beckon_timer > 0 { self.beckon_timer += 1; }
            if self.beckon_count == 1 && self.beckon_timer == 0 { self.beckon_timer = 1; }
            if self.beckon_timer > 200 { self.beckon_count = 0; self.beckon_timer = 0; }
            if self.beckon_count >= 3 {
                self.current_gesture = "BECKON";
                self.gesture_hold = 60;
                self.beckon_count = 0;
                self.beckon_timer = 0;
                return;
            }
        }

        // POINT (IMU only: finger pitched, palm flat)
        if palm_still && finger_still {
            let fp = finger_euler.pitch;
            let pp = palm_euler.pitch;

            if fp > 45.0 && abs_f32(pp) < 25.0 {
                self.current_gesture = "FINGER POINT UP";
                return;
            }
            if fp < -45.0 && abs_f32(pp) < 25.0 {
                self.current_gesture = "FINGER POINT DOWN";
                return;
            }

            if abs_f32(palm_euler.roll) > 60.0 && fp > 30.0 {
                self.current_gesture = "THUMBS UP";
                return;
            }
            if abs_f32(palm_euler.roll) > 60.0 && fp < -30.0 {
                self.current_gesture = "THUMBS DOWN";
                return;
            }

            if abs_f32(fp) < 20.0 && abs_f32(pp) < 20.0
                && abs_f32(palm_euler.roll) < 20.0
                && abs_f32(finger_euler.roll) < 20.0
            {
                self.current_gesture = "OPEN HAND";
                return;
            }

            if abs_f32(pp) < 15.0 && palm_euler.roll < -50.0 {
                self.current_gesture = "PALM UP";
                return;
            }
            if abs_f32(pp) < 15.0 && palm_euler.roll > 50.0 {
                self.current_gesture = "PALM DOWN";
                return;
            }
        }

        // WAG FINGER
        if palm_still && abs_f32(finger_imu.gyro.z) > 50.0 {
            self.current_gesture = "WAG FINGER";
            return;
        }

        // FIST PUMP (IMU only)
        if palm_imu.accel.z > 1.5 && finger_imu.accel.z > 1.5 {
            self.current_gesture = "FIST PUMP!";
            self.gesture_hold = 30;
            return;
        }

        // Both moving together
        if !palm_still && !finger_still {
            let gyro_diff = abs_f32(palm_imu.gyro.x - finger_imu.gyro.x)
                + abs_f32(palm_imu.gyro.y - finger_imu.gyro.y)
                + abs_f32(palm_imu.gyro.z - finger_imu.gyro.z);
            if gyro_diff < 60.0 {
                self.current_gesture = "HAND MOVING";
            } else {
                self.current_gesture = "FINGERS MOVING";
            }
            return;
        }

        self.current_gesture = "IDLE";
    }
}
