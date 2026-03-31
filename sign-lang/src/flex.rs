/// Flex sensor module with advanced calibration for small-range ADC readings.
///
/// Techniques used:
///   1. **Oversampling + averaging** — read N samples, average to reduce noise
///   2. **EMA (Exponential Moving Average)** — smooth jitter between reads
///   3. **Auto min/max calibration** — learns each finger's actual range over time
///   4. **Dead-zone / hysteresis** — prevents flicker near thresholds
///   5. **Adaptive range expansion** — slowly expands range as sensor warms up
///
/// Even if the raw ADC only spans 50 counts (e.g. 1650–1700),
/// this maps that tiny range to a clean 0–100% bend output.

// ═══════════════════════════════════════════════════════════════════
//  Constants
// ═══════════════════════════════════════════════════════════════════

/// Number of ADC samples to average per read (oversampling)
const OVERSAMPLE_COUNT: u32 = 32;

/// EMA smoothing factor: higher = more responsive, lower = smoother
const EMA_ALPHA: f32 = 0.25;

/// During auto-cal, how quickly min/max expand toward new extremes
/// Very slow so noise doesn't wreck the calibrated range
const CAL_EXPAND_RATE: f32 = 0.001;

/// Minimum span (in ADC counts) to consider calibration valid
const MIN_VALID_SPAN: f32 = 3.0;

/// Hysteresis band in percent — tiny so small bends register
const HYSTERESIS_PCT: f32 = 0.5;

// ═══════════════════════════════════════════════════════════════════
//  Per-finger sensor
// ═══════════════════════════════════════════════════════════════════

#[derive(Clone, Copy)]
pub struct FlexSensor {
    /// Smoothed ADC value (EMA-filtered)
    ema_value: f32,

    /// Calibration bounds
    cal_min: f32,
    cal_max: f32,

    /// Current output bend percentage (with hysteresis)
    bend_pct: f32,

    /// Whether initial calibration has been seeded
    seeded: bool,

    /// Whether bending gives lower ADC readings
    inverted: bool,

    /// When true, auto-cal only expands gently (never drifts/shrinks)
    cal_locked: bool,
}

impl FlexSensor {
    pub fn new() -> Self {
        Self {
            ema_value: 0.0,
            cal_min: 4095.0,
            cal_max: 0.0,
            bend_pct: 0.0,
            seeded: false,
            inverted: false,
            cal_locked: false,
        }
    }

    /// Feed a new raw oversampled ADC value. Returns smoothed bend %.
    pub fn update(&mut self, raw_avg: f32) -> f32 {
        // ── 1. EMA smoothing ──────────────────────────────
        if !self.seeded {
            self.ema_value = raw_avg;
            self.cal_min = raw_avg - 5.0; // small initial window
            self.cal_max = raw_avg + 5.0;
            self.seeded = true;
        } else {
            self.ema_value = EMA_ALPHA * raw_avg + (1.0 - EMA_ALPHA) * self.ema_value;
        }

        // ── 2. Auto-calibration: gently expand only (no shrink/drift) ──
        if !self.cal_locked {
            if self.ema_value < self.cal_min {
                self.cal_min += (self.ema_value - self.cal_min) * CAL_EXPAND_RATE;
            }
            if self.ema_value > self.cal_max {
                self.cal_max += (self.ema_value - self.cal_max) * CAL_EXPAND_RATE;
            }
            if self.cal_max - self.cal_min < MIN_VALID_SPAN {
                let mid = (self.cal_max + self.cal_min) * 0.5;
                self.cal_min = mid - MIN_VALID_SPAN * 0.5;
                self.cal_max = mid + MIN_VALID_SPAN * 0.5;
            }
        }

        // ── 3. Map to 0–100% ─────────────────────────────
        let span = self.cal_max - self.cal_min;
        let normalized = (self.ema_value - self.cal_min) / span;
        let clamped = if normalized < 0.0 {
            0.0
        } else if normalized > 1.0 {
            1.0
        } else {
            normalized
        };

        // Determine direction: if "rest" position was the first reading
        // and bending increases OR decreases the value, auto-detect
        let new_pct = if self.inverted {
            (1.0 - clamped) * 100.0
        } else {
            clamped * 100.0
        };

        // ── 4. Hysteresis ─────────────────────────────────
        let diff = new_pct - self.bend_pct;
        if diff > HYSTERESIS_PCT || diff < -HYSTERESIS_PCT {
            self.bend_pct = new_pct;
        }

        self.bend_pct
    }

    /// Call during calibration phase with sensor STRAIGHT.
    /// Fast 0.85/0.15 blend with direct seeding on first call.
    pub fn calibrate_straight(&mut self, raw_avg: f32) {
        if !self.seeded {
            self.ema_value = raw_avg;
            self.cal_min = raw_avg;
            self.seeded = true;
        } else {
            self.cal_min = self.cal_min * 0.85 + raw_avg * 0.15;
        }
    }

    /// Call during calibration phase with sensor BENT.
    /// Fast 0.85/0.15 blend with direct seeding on first call.
    pub fn calibrate_bent(&mut self, raw_avg: f32) {
        if self.cal_max < 1.0 {
            // First call — seed directly
            self.cal_max = raw_avg;
        } else {
            self.cal_max = self.cal_max * 0.85 + raw_avg * 0.15;
        }
    }

    /// Call after both calibration phases complete.
    /// Detects direction, adds 20% padding on each side for noise
    /// immunity, and locks the range so auto-cal only expands gently.
    pub fn finalize_calibration(&mut self) {
        // Detect direction: does bending give lower readings?
        if self.cal_max < self.cal_min {
            self.inverted = true;
            let tmp = self.cal_min;
            self.cal_min = self.cal_max;
            self.cal_max = tmp;
        } else {
            self.inverted = false;
        }

        // Pad the range by 20% on each side so noise stays inside bounds
        let span = self.cal_max - self.cal_min;
        if span > 0.5 {
            let pad = span * 0.20;
            self.cal_min -= pad;
            self.cal_max += pad;
        }

        self.cal_locked = true;
        self.bend_pct = 0.0;
    }

    /// Force-set calibration from known values
    pub fn set_calibration(&mut self, straight_val: f32, bent_val: f32) {
        if bent_val > straight_val {
            self.cal_min = straight_val;
            self.cal_max = bent_val;
            self.inverted = false;
        } else {
            self.cal_min = bent_val;
            self.cal_max = straight_val;
            self.inverted = true;
        }
        self.seeded = true;
    }

    pub fn bend(&self) -> f32 {
        self.bend_pct
    }

    pub fn raw_ema(&self) -> f32 {
        self.ema_value
    }

    pub fn cal_range(&self) -> (f32, f32) {
        (self.cal_min, self.cal_max)
    }
}

// ═══════════════════════════════════════════════════════════════════
//  5-finger hand
// ═══════════════════════════════════════════════════════════════════

/// Finger indices
pub const THUMB: usize = 0;
pub const INDEX: usize = 1;
pub const MIDDLE: usize = 2;
pub const RING: usize = 3;
pub const PINKY: usize = 4;

const FINGER_NAMES: [&str; 5] = ["Thumb", "Index", "Middle", "Ring", "Pinky"];

pub struct FlexHand {
    pub fingers: [FlexSensor; 5],
}

impl FlexHand {
    pub fn new() -> Self {
        Self {
            fingers: [FlexSensor::new(); 5],
        }
    }

    /// Update finger `idx` with a new oversampled raw ADC value
    pub fn update_finger(&mut self, idx: usize, raw_avg: f32) -> f32 {
        self.fingers[idx].update(raw_avg)
    }

    /// Get bend percentages for all fingers
    pub fn bend_all(&self) -> [f32; 5] {
        [
            self.fingers[0].bend(),
            self.fingers[1].bend(),
            self.fingers[2].bend(),
            self.fingers[3].bend(),
            self.fingers[4].bend(),
        ]
    }

    /// Classify hand pose from flex data alone
    pub fn classify_hand_pose(&self) -> &'static str {
        let b = self.bend_all();
        let all_straight = b.iter().all(|&v| v < 25.0);
        let all_bent = b.iter().all(|&v| v > 65.0);
        let index_straight = b[INDEX] < 30.0;
        let others_bent = b[THUMB] > 50.0 && b[MIDDLE] > 50.0 && b[RING] > 50.0 && b[PINKY] > 50.0;

        if all_bent {
            "FIST"
        } else if all_straight {
            "OPEN HAND"
        } else if index_straight && others_bent {
            "POINTING"
        } else if b[THUMB] < 30.0 && b[INDEX] < 30.0 && b[MIDDLE] > 50.0 && b[RING] > 50.0 && b[PINKY] > 50.0 {
            "PEACE / V"
        } else if b[THUMB] < 30.0 && b[INDEX] > 50.0 && b[MIDDLE] > 50.0 && b[RING] > 50.0 && b[PINKY] < 30.0 {
            "HANG LOOSE"
        } else if b[THUMB] < 30.0 && others_bent {
            "THUMBS UP"
        } else if b[INDEX] < 30.0 && b[MIDDLE] < 30.0 && b[RING] < 30.0 && b[PINKY] > 50.0 && b[THUMB] > 50.0 {
            "THREE"
        } else if b[INDEX] < 30.0 && b[MIDDLE] < 30.0 && b[RING] > 50.0 && b[PINKY] > 50.0 && b[THUMB] > 50.0 {
            "TWO"
        } else if b[INDEX] < 30.0 && b[MIDDLE] > 50.0 && b[RING] > 50.0 && b[PINKY] < 30.0 && b[THUMB] > 50.0 {
            "ROCK ON"
        } else if b.iter().all(|&v| v > 30.0 && v < 70.0) {
            "RELAXED"
        } else {
            "---"
        }
    }

    pub fn finger_name(idx: usize) -> &'static str {
        FINGER_NAMES[idx]
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Oversampling helper — call from main with the ADC
// ═══════════════════════════════════════════════════════════════════

/// Read `OVERSAMPLE_COUNT` samples and return the average as f32.
/// Call this from main.rs with your ADC + pin.
pub fn oversample_average(samples: &[u16]) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }
    let sum: u32 = samples.iter().map(|&s| s as u32).sum();
    sum as f32 / samples.len() as f32
}

/// How many oversamples to take
pub fn oversample_count() -> u32 {
    OVERSAMPLE_COUNT
}