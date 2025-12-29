//! Pointing device features for trackballs and other pointing devices.
//!
//! This module provides runtime state and configuration for advanced pointing device features:
//! - Scroll mode (toggle/momentary)
//! - CPI switching (software-based scaling)
//! - Drag lock
//! - Sniper mode (low CPI for precision)
//! - Angle snapping
//! - Gesture detection

/// Runtime state for pointing device features.
/// This state is shared between the keyboard (which handles keycodes) and
/// the pointing device processor (which applies the features).
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PointingState {
    /// Toggle state for scroll mode
    pub scroll_mode: bool,
    /// Momentary state for scroll mode (held key)
    pub scroll_momentary: bool,
    /// Sniper mode active (momentary low CPI)
    pub sniper_mode: bool,
    /// Toggle state for angle snapping
    pub angle_snap: bool,
    /// Momentary state for angle snapping
    pub angle_snap_momentary: bool,
    /// Which button is drag-locked (0-7), or None if no drag lock
    pub drag_lock_button: Option<u8>,
    /// Current CPI level index (0-3 for 4 preset levels)
    pub cpi_level: u8,
}

impl PointingState {
    /// Create a new PointingState with default values
    pub const fn new() -> Self {
        Self {
            scroll_mode: false,
            scroll_momentary: false,
            sniper_mode: false,
            angle_snap: false,
            angle_snap_momentary: false,
            drag_lock_button: None,
            cpi_level: 0,
        }
    }

    /// Returns true if effective scroll mode is active (toggle XOR momentary)
    #[inline]
    pub fn is_scroll_active(&self) -> bool {
        self.scroll_mode ^ self.scroll_momentary
    }

    /// Returns true if effective angle snap is active (toggle XOR momentary)
    #[inline]
    pub fn is_angle_snap_active(&self) -> bool {
        self.angle_snap ^ self.angle_snap_momentary
    }
}

/// Configuration for pointing device features.
/// These values are typically loaded from keyboard.toml.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PointingConfig {
    /// CPI preset levels (4 levels)
    pub cpi_levels: [u16; 4],
    /// Default CPI level index
    pub default_cpi_index: u8,
    /// Divisor for scroll mode (higher = slower scroll)
    pub scroll_divisor: i16,
    /// Divisor for sniper mode (higher = slower/more precise)
    pub sniper_divisor: i16,
    /// Threshold ratio for angle snapping (e.g., 3 means need 3x movement to break snap)
    pub angle_snap_ratio: i16,
    /// Whether gesture detection is enabled
    pub gesture_enabled: bool,
    /// Minimum accumulated movement to trigger a gesture
    pub gesture_threshold: i16,
    /// Timeout in milliseconds for gesture detection
    pub gesture_timeout_ms: u16,
    /// Acceleration curve type
    pub acceleration_curve: AccelerationCurve,
}

impl Default for PointingConfig {
    fn default() -> Self {
        Self {
            cpi_levels: [400, 800, 1200, 1600],
            default_cpi_index: 1, // Start at 800 CPI
            scroll_divisor: 4,
            sniper_divisor: 4,
            angle_snap_ratio: 3,
            gesture_enabled: false,
            gesture_threshold: 50,
            gesture_timeout_ms: 200,
            acceleration_curve: AccelerationCurve::Linear,
        }
    }
}

impl PointingConfig {
    /// Create a new PointingConfig with default values
    pub const fn new() -> Self {
        Self {
            cpi_levels: [400, 800, 1200, 1600],
            default_cpi_index: 1,
            scroll_divisor: 4,
            sniper_divisor: 4,
            angle_snap_ratio: 3,
            gesture_enabled: false,
            gesture_threshold: 50,
            gesture_timeout_ms: 200,
            acceleration_curve: AccelerationCurve::Linear,
        }
    }
}

/// Acceleration curve types for pointer movement
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelerationCurve {
    /// Linear acceleration (constant multiplier)
    #[default]
    Linear,
    /// Ease-in: slow start, fast end
    EaseIn,
    /// Ease-out: fast start, slow end
    EaseOut,
    /// S-curve: slow-fast-slow
    SCurve,
}

/// Gesture types that can be detected
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Gesture {
    /// Swipe left
    Left,
    /// Swipe right
    Right,
    /// Swipe up
    Up,
    /// Swipe down
    Down,
}

/// Gesture detector for tracking movement and detecting swipe gestures
#[derive(Clone, Copy, Debug, Default)]
pub struct GestureDetector {
    /// Accumulated X movement
    acc_x: i32,
    /// Accumulated Y movement
    acc_y: i32,
    /// Whether we're currently tracking a gesture
    tracking: bool,
    /// Start time of current gesture (in ticks)
    start_ticks: u64,
}

impl GestureDetector {
    /// Create a new gesture detector
    pub const fn new() -> Self {
        Self {
            acc_x: 0,
            acc_y: 0,
            tracking: false,
            start_ticks: 0,
        }
    }

    /// Reset the gesture detector
    pub fn reset(&mut self) {
        self.acc_x = 0;
        self.acc_y = 0;
        self.tracking = false;
        self.start_ticks = 0;
    }

    /// Accumulate movement and check for gesture.
    /// Returns Some(Gesture) if a gesture is detected.
    ///
    /// # Arguments
    /// * `dx` - X movement delta
    /// * `dy` - Y movement delta
    /// * `current_ticks` - Current time in ticks (milliseconds)
    /// * `config` - Pointing configuration
    pub fn accumulate(
        &mut self,
        dx: i16,
        dy: i16,
        current_ticks: u64,
        config: &PointingConfig,
    ) -> Option<Gesture> {
        if !config.gesture_enabled {
            return None;
        }

        // Start tracking if not already
        if !self.tracking {
            self.tracking = true;
            self.start_ticks = current_ticks;
            self.acc_x = 0;
            self.acc_y = 0;
        }

        // Accumulate movement
        self.acc_x += dx as i32;
        self.acc_y += dy as i32;

        // Check timeout
        let elapsed = current_ticks.saturating_sub(self.start_ticks);
        if elapsed > config.gesture_timeout_ms as u64 {
            // Time to evaluate the gesture
            let gesture = self.evaluate_gesture(config);
            self.reset();
            return gesture;
        }

        None
    }

    /// Called when no movement detected - evaluate accumulated movement
    pub fn on_idle(&mut self, current_ticks: u64, config: &PointingConfig) -> Option<Gesture> {
        if !self.tracking || !config.gesture_enabled {
            return None;
        }

        let elapsed = current_ticks.saturating_sub(self.start_ticks);
        if elapsed > config.gesture_timeout_ms as u64 {
            let gesture = self.evaluate_gesture(config);
            self.reset();
            return gesture;
        }

        None
    }

    /// Evaluate accumulated movement to determine gesture
    fn evaluate_gesture(&self, config: &PointingConfig) -> Option<Gesture> {
        let threshold = config.gesture_threshold as i32;
        let abs_x = self.acc_x.abs();
        let abs_y = self.acc_y.abs();

        // Require dominant axis to be 1.5x the other
        let ratio_threshold = 3; // 3/2 = 1.5x

        if abs_x > threshold && abs_x * 2 > abs_y * ratio_threshold {
            // Horizontal gesture
            if self.acc_x > 0 {
                Some(Gesture::Right)
            } else {
                Some(Gesture::Left)
            }
        } else if abs_y > threshold && abs_y * 2 > abs_x * ratio_threshold {
            // Vertical gesture
            if self.acc_y > 0 {
                Some(Gesture::Down)
            } else {
                Some(Gesture::Up)
            }
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pointing_state_default() {
        let state = PointingState::new();
        assert!(!state.scroll_mode);
        assert!(!state.sniper_mode);
        assert!(state.drag_lock_button.is_none());
        assert_eq!(state.cpi_level, 0);
    }

    #[test]
    fn test_scroll_mode_xor() {
        let mut state = PointingState::new();
        assert!(!state.is_scroll_active());

        state.scroll_mode = true;
        assert!(state.is_scroll_active());

        state.scroll_momentary = true;
        assert!(!state.is_scroll_active()); // XOR: both true = false

        state.scroll_mode = false;
        assert!(state.is_scroll_active()); // Only momentary
    }

    #[test]
    fn test_pointing_config_default() {
        let config = PointingConfig::default();
        assert_eq!(config.cpi_levels, [400, 800, 1200, 1600]);
        assert_eq!(config.default_cpi_index, 1);
        assert_eq!(config.scroll_divisor, 4);
    }

    #[test]
    fn test_gesture_detector() {
        let mut detector = GestureDetector::new();
        let config = PointingConfig {
            gesture_enabled: true,
            gesture_threshold: 50,
            gesture_timeout_ms: 200,
            ..Default::default()
        };

        // Accumulate rightward movement
        for i in 0..10 {
            detector.accumulate(10, 0, i * 10, &config);
        }

        // Check gesture on timeout
        let gesture = detector.on_idle(250, &config);
        assert_eq!(gesture, Some(Gesture::Right));
    }
}
