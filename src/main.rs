#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::f32::consts::{PI, TAU};
use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};

// This rig uses GM6020 voltage commands for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);
const DT_S: f32 = 0.001;

// ---------------------------------------------------------------------------------------
// Rig configuration. Confirm every value here on the rig before enabling output.
// ---------------------------------------------------------------------------------------

/// GM6020 DIP-switch IDs. Feedback arrives on 0x204 + ID; the 0x1FF voltage frame
/// carries ID n in bytes 2(n-1) and 2(n-1)+1.
const YAW_MOTOR_ID: u16 = 1;
const PITCH_MOTOR_ID: u16 = 2;
const FEEDBACK_BASE_ID: u16 = 0x204;

/// Mechanical zero, raw count 0..=8191, read with output disabled while the rig is held
/// at the agreed front / level pose. `None` disables recentering for that axis: Prepare
/// then holds the pose it starts from instead of moving to a guessed reference.
const YAW_ZERO_COUNT: Option<u16> = None;
const PITCH_ZERO_COUNT: Option<u16> = None;

/// Pitch range in motor-shaft rad relative to the reference pose. Pitch is driven through
/// a linkage, so these are motor angles, not joint angles. Conservative placeholders:
/// replace with the measured range of the rig.
const PITCH_MIN_RAD: f32 = -0.35;
const PITCH_MAX_RAD: f32 = 0.35;

/// Flip these if stick left/right or up/down moves the rig the wrong way.
const YAW_STICK_SIGN: f32 = 1.0;
const PITCH_STICK_SIGN: f32 = 1.0;

/// Bench output limit in voltage counts (protocol +/-25000).
const OUTPUT_LIMIT: f32 = 10000.0;

// ---------------------------------------------------------------------------------------
// Behaviour tuning.
// ---------------------------------------------------------------------------------------

/// Normalised stick deadzone (0.05 of full travel is 33 counts).
const STICK_DEADZONE: f32 = 0.05;
/// Target angle rate at full stick, motor-shaft rad/s.
const MAX_TARGET_RATE_RAD_S: f32 = 1.0;
/// Feedback older than this many ticks counts as lost. GM6020 reports at 1 kHz.
const FEEDBACK_TIMEOUT_TICKS: u16 = 20;

/// Recentering limits and completion: both axes within the angle and speed bands for
/// `SETTLE_TICKS` in a row. Not settled after `PREPARE_TIMEOUT_TICKS` returns to Waiting.
const PREPARE_MAX_RPM: f32 = 30.0;
const PREPARE_MAX_OUTPUT: f32 = 5000.0;
const SETTLE_ANGLE_RAD: f32 = 0.03;
const SETTLE_SPEED_RPM: f32 = 5.0;
const SETTLE_TICKS: u16 = 300;
const PREPARE_TIMEOUT_TICKS: u16 = 5000;

/// Suspected stall: |target speed| >= 10 rpm, |output| >= 4000 counts and |speed| <= 3 rpm,
/// all held for 500 ticks (0.5 s).
const STALL_TARGET_RPM: f32 = 10.0;
const STALL_OUTPUT: f32 = 4000.0;
const STALL_SPEED_RPM: f32 = 3.0;
const STALL_TICKS: u16 = 500;

/// Sustained oscillation: the speed swings past +/-15 rpm in both directions, each full
/// swing (+ to - and back to +) is timed as one period, and every period stays above
/// 8 Hz for a total of 500 ticks (0.5 s, at least four periods).
const OSC_BAND_RPM: f32 = 15.0;
const OSC_FREQ_HZ: f32 = 8.0;
const OSC_SUSTAIN_TICKS: u16 = 500;
/// The longest period still faster than `OSC_FREQ_HZ`, in ticks.
const OSC_MAX_PERIOD_TICKS: u16 = (1000.0 / OSC_FREQ_HZ) as u16;

/// Print every command frame's data every 200 ms, to check the encoding.
const LOG_TX_FRAMES: bool = false;

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// Wrap an angle in rad to (-pi, pi].
fn wrap_pi(mut x: f32) -> f32 {
    if !x.is_finite() {
        return 0.0;
    }
    while x > PI {
        x -= TAU;
    }
    while x <= -PI {
        x += TAU;
    }
    x
}

/// Wrap an angle in rad to [0, 2pi).
fn wrap_tau(x: f32) -> f32 {
    let x = wrap_pi(x);
    if x < 0.0 { x + TAU } else { x }
}

/// Normalise a stick channel to -1..=1 and remove the deadzone, rescaling the rest so the
/// output starts from zero at the deadzone edge.
fn stick(raw: i16) -> f32 {
    let x = (raw as f32 / 660.0).clamp(-1.0, 1.0);
    if x.abs() <= STICK_DEADZONE {
        0.0
    } else {
        (x.abs() - STICK_DEADZONE) / (1.0 - STICK_DEADZONE) * x.signum()
    }
}

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Raw single-turn mechanical angle, 0..=8191 for one turn.
    pub angle_raw: u16,
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    pub temperature_c: u8,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    /// GM6020 feedback: bytes 0-1 angle (0..=8191), 2-3 speed in rpm (signed),
    /// 4-5 current (signed, raw), 6 temperature in degC; all big-endian.
    pub fn read(&mut self, data: &[u8; 8]) {
        self.angle_raw = u16::from_be_bytes([data[0], data[1]]) & 0x1FFF;
        self.angle_rad = self.angle_raw as f32 * (TAU / 8192.0);
        self.speed_rpm = i16::from_be_bytes([data[2], data[3]]) as f32;
        self.current_raw = i16::from_be_bytes([data[4], data[5]]);
        self.temperature_c = data[6];
    }
}

/// PID gains, plus the integral and history kept between ticks.
#[derive(Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Limit of the integral contribution, in output units.
    pub max_i: f32,
    /// Symmetric output limit; 0 makes the output 0.
    pub maxout: f32,
    /// Integral contribution, already multiplied by `ki`.
    pub i_out: f32,
    /// Error from the previous call; `None` until the first call after a reset, so the
    /// first call has no derivative kick.
    pub last_error: Option<f32>,
    pub p_out: f32,
    pub d_out: f32,
    pub output: f32,
}

impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32, max_i: f32, maxout: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            max_i,
            maxout,
            i_out: 0.0,
            last_error: None,
            p_out: 0.0,
            d_out: 0.0,
            output: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.i_out = 0.0;
        self.last_error = None;
        self.p_out = 0.0;
        self.d_out = 0.0;
        self.output = 0.0;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        let error = target - measured;
        if !error.is_finite() || dt_s.is_nan() || dt_s <= 0.0 {
            self.reset();
            return 0.0;
        }
        self.p_out = self.kp * error;
        self.i_out = (self.i_out + self.ki * error * dt_s).clamp(-self.max_i, self.max_i);
        self.d_out = match self.last_error {
            Some(last) => self.kd * (error - last) / dt_s,
            None => 0.0,
        };
        self.last_error = Some(error);
        self.output = (self.p_out + self.i_out + self.d_out).clamp(-self.maxout, self.maxout);
        self.output
    }
}

/// Run the PID checks from the README once at startup and print the results.
fn pid_self_test() {
    let mut pid = Pid::new(2.0, 0.0, 0.0, 0.0, 100.0);
    let a = pid.calculate(10.0, 7.0, DT_S);
    pid.maxout = 5.0;
    let b = pid.calculate(10.0, 7.0, DT_S);
    pid.maxout = 0.0;
    let c = pid.calculate(10.0, 7.0, DT_S);
    rtt_target::rprintln!("PID self-test: {} (6) {} (5) {} (0)", a, b, c);
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultReason {
    Stall,
    Oscillation,
}

/// The fault that latched `Error`. Kept after recovery as the last fault seen.
#[derive(Clone, Copy, Debug)]
pub struct Fault {
    pub axis: &'static str,
    pub reason: FaultReason,
}

/// Per-axis fixed settings.
#[derive(Clone, Copy)]
pub struct AxisConfig {
    pub name: &'static str,
    pub motor_id: u16,
    pub zero_count: Option<u16>,
    /// Yaw may take the shortest way round; pitch must stay inside `limits_rad`.
    pub shortest_path: bool,
    /// Allowed target range in rad relative to the reference, if limited.
    pub limits_rad: Option<(f32, f32)>,
    /// Index into `Dr16::channels`.
    pub stick_channel: usize,
    pub stick_sign: f32,
}

/// Stall and oscillation detection for one axis.
#[derive(Default)]
pub struct Detector {
    /// Ticks the stall condition has held.
    pub stall_ticks: u16,
    /// Which side of the +/-band the speed was last on: 1, -1, or 0 before the first.
    osc_side: i8,
    /// Whether a period start has been seen, so the next one closes a full period.
    osc_timing: bool,
    /// Ticks since the current period started.
    pub osc_period_ticks: u16,
    /// Frequency of the last complete period, Hz.
    pub osc_freq_hz: f32,
    /// Total ticks of consecutive periods faster than `OSC_FREQ_HZ`.
    pub osc_fast_ticks: u16,
}

impl Detector {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Run once per tick in Prepare and Running.
    pub fn update(&mut self, target_rpm: f32, speed_rpm: f32, command: f32) -> Option<FaultReason> {
        let stalled = target_rpm.abs() >= STALL_TARGET_RPM
            && command.abs() >= STALL_OUTPUT
            && speed_rpm.abs() <= STALL_SPEED_RPM;
        self.stall_ticks = if stalled { self.stall_ticks.saturating_add(1) } else { 0 };
        if self.stall_ticks >= STALL_TICKS {
            return Some(FaultReason::Stall);
        }

        self.osc_period_ticks = self.osc_period_ticks.saturating_add(1);
        let side = if speed_rpm > OSC_BAND_RPM {
            1
        } else if speed_rpm < -OSC_BAND_RPM {
            -1
        } else {
            0
        };
        if side != 0 && side != self.osc_side {
            // Crossing from below -band to above +band closes one full period.
            if side == 1 && self.osc_side == -1 {
                if self.osc_timing {
                    let period = self.osc_period_ticks;
                    self.osc_freq_hz = 1000.0 / period as f32;
                    self.osc_fast_ticks = if self.osc_freq_hz > OSC_FREQ_HZ {
                        self.osc_fast_ticks.saturating_add(period)
                    } else {
                        0
                    };
                }
                self.osc_timing = true;
                self.osc_period_ticks = 0;
            }
            self.osc_side = side;
        }
        // No new period in time: whatever comes next is slower than the threshold.
        if self.osc_period_ticks > OSC_MAX_PERIOD_TICKS {
            self.osc_fast_ticks = 0;
            self.osc_freq_hz = 0.0;
        }
        if self.osc_fast_ticks >= OSC_SUSTAIN_TICKS {
            return Some(FaultReason::Oscillation);
        }
        None
    }
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
pub struct Axis {
    pub config: AxisConfig,
    pub motor: Motor,
    /// Ticks since the last feedback frame from this motor.
    pub feedback_age_ticks: u16,
    /// Reference pose in motor-shaft rad: the zero count, or the angle when Prepare began.
    pub reference_rad: f32,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,
    pub detector: Detector,
}

impl Axis {
    pub fn new(config: AxisConfig, position_pid: Pid, speed_pid: Pid) -> Self {
        Self {
            config,
            motor: Motor::default(),
            feedback_age_ticks: u16::MAX,
            reference_rad: 0.0,
            target_angle_rad: 0.0,
            target_rpm: 0.0,
            position_pid,
            speed_pid,
            detector: Detector::default(),
        }
    }

    pub fn feedback_fresh(&self) -> bool {
        self.feedback_age_ticks < FEEDBACK_TIMEOUT_TICKS
    }

    /// Target minus measured angle in rad, without a jump where the count wraps
    /// from 8191 to 0. Yaw takes the shortest way; pitch measures both against the
    /// reference so its path never leaves the allowed range.
    pub fn position_error(&self) -> f32 {
        if self.config.shortest_path {
            wrap_pi(self.target_angle_rad - self.motor.angle_rad)
        } else {
            wrap_pi(self.target_angle_rad - self.reference_rad)
                - wrap_pi(self.motor.angle_rad - self.reference_rad)
        }
    }

    /// Clear the target speed, PID history and detection counters.
    pub fn reset_control(&mut self) {
        self.target_rpm = 0.0;
        self.position_pid.reset();
        self.speed_pid.reset();
        self.detector.reset();
    }

    /// Move the target angle by `rate_rad_s` for one tick, keeping it in range. A target
    /// already outside the range may move back towards it but not further out.
    pub fn move_target(&mut self, rate_rad_s: f32) {
        let target = self.target_angle_rad + rate_rad_s * DT_S;
        self.target_angle_rad = match self.config.limits_rad {
            Some((min, max)) => {
                let old = wrap_pi(self.target_angle_rad - self.reference_rad);
                let new = wrap_pi(target - self.reference_rad).clamp(old.min(min), old.max(max));
                wrap_tau(self.reference_rad + new)
            }
            None => wrap_tau(target),
        };
    }

    /// Run the cascade and write the voltage command, with extra limits on the target
    /// speed and output on top of the PID limits.
    pub fn run_cascade(&mut self, max_rpm: f32, max_output: f32) {
        let measured = self.motor.angle_rad;
        let error = self.position_error();
        self.target_rpm = self
            .position_pid
            .calculate(measured + error, measured, DT_S)
            .clamp(-max_rpm, max_rpm);
        self.motor.command_voltage_raw = self
            .speed_pid
            .calculate(self.target_rpm, self.motor.speed_rpm, DT_S)
            .clamp(-max_output, max_output);
    }
}

/// State shared by every control tick.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    /// Switch positions from the previous tick, `None` while the remote is lost, so a
    /// switch held through a reconnect is not taken as a new gesture.
    pub last_right_switch: Option<u8>,
    pub last_left_switch: Option<u8>,
    /// Ticks spent in Prepare, and ticks both axes have been settled in a row.
    pub prepare_ticks: u16,
    pub settle_ticks: u16,
    /// The fault that latched the current or last Error.
    pub fault: Option<Fault>,
    tx_log_ticks: u16,
}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            yaw: Axis::new(
                AxisConfig {
                    name: "yaw",
                    motor_id: YAW_MOTOR_ID,
                    zero_count: YAW_ZERO_COUNT,
                    shortest_path: true,
                    limits_rad: None,
                    stick_channel: 2,
                    stick_sign: YAW_STICK_SIGN,
                },
                // rad -> rpm
                Pid::new(60.0, 0.0, 0.0, 0.0, 120.0),
                // rpm -> voltage counts
                Pid::new(40.0, 0.0, 0.0, 3000.0, 8000.0),
            ),
            pitch: Axis::new(
                AxisConfig {
                    name: "pitch",
                    motor_id: PITCH_MOTOR_ID,
                    zero_count: PITCH_ZERO_COUNT,
                    shortest_path: false,
                    limits_rad: Some((PITCH_MIN_RAD, PITCH_MAX_RAD)),
                    stick_channel: 3,
                    stick_sign: PITCH_STICK_SIGN,
                },
                Pid::new(60.0, 0.0, 0.0, 0.0, 120.0),
                Pid::new(40.0, 0.0, 0.0, 3000.0, 8000.0),
            ),
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            last_right_switch: None,
            last_left_switch: None,
            prepare_ticks: 0,
            settle_ticks: 0,
            fault: None,
            tx_log_ticks: 0,
        }
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }
        self.update();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            if self.state != State::Error {
                if self.state != State::Waiting {
                    rtt_target::rprintln!("{:?} -> Waiting: remote lost", self.state);
                }
                self.state = State::Waiting;
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        self.yaw.feedback_age_ticks = self.yaw.feedback_age_ticks.saturating_add(1);
        self.pitch.feedback_age_ticks = self.pitch.feedback_age_ticks.saturating_add(1);
        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let frame = envelope.frame;
                    let id = match frame.id() {
                        embedded_can::Id::Standard(id) => id.as_raw(),
                        embedded_can::Id::Extended(_) => continue,
                    };
                    let Ok(data) = <&[u8; 8]>::try_from(frame.data()) else {
                        continue;
                    };
                    for axis in [&mut self.yaw, &mut self.pitch] {
                        if id == FEEDBACK_BASE_ID + axis.config.motor_id {
                            axis.motor.read(data);
                            axis.feedback_age_ticks = 0;
                        }
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// True when both left-stick channels are inside the deadzone.
    fn sticks_centered(&self) -> bool {
        self.remote
            .is_some_and(|r| stick(r.channels[2]) == 0.0 && stick(r.channels[3]) == 0.0)
    }

    fn set_state(&mut self, state: State, why: &str) {
        rtt_target::rprintln!("{:?} -> {:?}: {}", self.state, state, why);
        self.state = state;
    }

    fn enter_prepare(&mut self) {
        for axis in [&mut self.yaw, &mut self.pitch] {
            axis.reference_rad = match axis.config.zero_count {
                Some(count) => (count & 0x1FFF) as f32 * (TAU / 8192.0),
                None => {
                    rtt_target::rprintln!(
                        "{}: no zero count configured, recentering disabled, holding {:.3} rad",
                        axis.config.name,
                        axis.motor.angle_rad
                    );
                    axis.motor.angle_rad
                }
            };
            axis.target_angle_rad = axis.reference_rad;
            axis.reset_control();
        }
        self.prepare_ticks = 0;
        self.settle_ticks = 0;
        self.set_state(State::Prepare, "enable");
    }

    /// Check both axes for faults; on one, latch Error and clear outputs this tick.
    fn check_faults(&mut self) -> bool {
        let mut fault = None;
        for axis in [&mut self.yaw, &mut self.pitch] {
            let reason = axis.detector.update(
                axis.target_rpm,
                axis.motor.speed_rpm,
                axis.motor.command_voltage_raw,
            );
            if let Some(reason) = reason {
                rtt_target::rprintln!(
                    "FAULT {} {:?}: target_rpm={:.1} rpm={:.1} out={:.0} stall_ticks={} osc={:.1}Hz for {} ticks",
                    axis.config.name,
                    reason,
                    axis.target_rpm,
                    axis.motor.speed_rpm,
                    axis.motor.command_voltage_raw,
                    axis.detector.stall_ticks,
                    axis.detector.osc_freq_hz,
                    axis.detector.osc_fast_ticks,
                );
                fault = Some(Fault {
                    axis: axis.config.name,
                    reason,
                });
                break;
            }
        }
        if fault.is_none() {
            return false;
        }
        self.fault = fault;
        self.disable_all();
        self.set_state(State::Error, "fault");
        true
    }

    /// Choose the state from the remote, feedback and fault conditions.
    pub fn update_mode(&mut self) {
        let right = self.remote.map(|r| r.right_switch);
        let left = self.remote.map(|r| r.left_switch);
        // Enable: right switch from down (2) to middle (3), seen on consecutive ticks.
        let enable_request = self.last_right_switch == Some(2) && right == Some(3);
        // Recovery: right switch at middle, sticks centered, left switch from down (2) to up (1).
        let recover_request = right == Some(3)
            && self.last_left_switch == Some(2)
            && left == Some(1)
            && self.sticks_centered();
        self.last_right_switch = right;
        self.last_left_switch = left;
        let feedback_ok = self.yaw.feedback_fresh() && self.pitch.feedback_fresh();

        match self.state {
            State::Waiting => {
                if enable_request {
                    if feedback_ok {
                        self.enter_prepare();
                    } else {
                        rtt_target::rprintln!("enable ignored: motor feedback missing");
                    }
                }
            }
            State::Prepare => {
                if right != Some(3) {
                    self.set_state(State::Waiting, "disable");
                } else if !feedback_ok {
                    self.set_state(State::Waiting, "motor feedback lost");
                } else if !self.check_faults() {
                    self.prepare_ticks = self.prepare_ticks.saturating_add(1);
                    let settled = [&self.yaw, &self.pitch].iter().all(|axis| {
                        axis.position_error().abs() <= SETTLE_ANGLE_RAD
                            && axis.motor.speed_rpm.abs() <= SETTLE_SPEED_RPM
                    });
                    self.settle_ticks = if settled { self.settle_ticks.saturating_add(1) } else { 0 };
                    if self.settle_ticks >= SETTLE_TICKS {
                        self.set_state(State::Running, "recentered");
                    } else if self.prepare_ticks >= PREPARE_TIMEOUT_TICKS {
                        self.disable_all();
                        self.set_state(State::Waiting, "recenter timeout");
                    }
                }
            }
            State::Running => {
                if right != Some(3) {
                    self.set_state(State::Waiting, "disable");
                } else if !feedback_ok {
                    self.set_state(State::Waiting, "motor feedback lost");
                } else {
                    self.check_faults();
                }
            }
            State::Error => {
                if recover_request {
                    if feedback_ok {
                        for axis in [&mut self.yaw, &mut self.pitch] {
                            axis.target_angle_rad = axis.motor.angle_rad;
                            axis.reset_control();
                            rtt_target::rprintln!(
                                "recover {}: angle={:.3} new target={:.3}, PID and detectors reset",
                                axis.config.name,
                                axis.motor.angle_rad,
                                axis.target_angle_rad
                            );
                        }
                        self.set_state(State::Running, "manual recovery");
                    } else {
                        rtt_target::rprintln!("recovery ignored: motor feedback missing");
                    }
                }
            }
        }
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    pub fn update(&mut self) {
        match self.state {
            State::Waiting | State::Error => {
                self.disable_all();
                self.yaw.target_rpm = 0.0;
                self.pitch.target_rpm = 0.0;
            }
            State::Prepare => {
                // Targets stay at the reference set on entry; the sticks are ignored.
                self.yaw.run_cascade(PREPARE_MAX_RPM, PREPARE_MAX_OUTPUT);
                self.pitch.run_cascade(PREPARE_MAX_RPM, PREPARE_MAX_OUTPUT);
            }
            State::Running => {
                let Some(remote) = self.remote else {
                    self.disable_all();
                    return;
                };
                for axis in [&mut self.yaw, &mut self.pitch] {
                    let input = stick(remote.channels[axis.config.stick_channel]);
                    axis.move_target(input * axis.config.stick_sign * MAX_TARGET_RATE_RAD_S);
                    axis.run_cascade(f32::INFINITY, OUTPUT_LIMIT);
                }
            }
        }
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];
        for axis in [&self.yaw, &self.pitch] {
            // NaN clamps to NaN, which casts to 0.
            let counts = axis.motor.command_voltage_raw.clamp(-OUTPUT_LIMIT, OUTPUT_LIMIT) as i16;
            let slot = usize::from(axis.config.motor_id - 1) * 2;
            data[slot..slot + 2].copy_from_slice(&counts.to_be_bytes());
        }
        if LOG_TX_FRAMES {
            self.tx_log_ticks += 1;
            if self.tx_log_ticks >= 200 {
                self.tx_log_ticks = 0;
                rtt_target::rprintln!("tx {:#05x} {:02x?}", MOTOR_COMMAND_ID, data);
            }
        }
        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    pid_self_test();
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

fn log_axis(name: &str, axis: &Axis) {
    rtt_target::rprintln!(
        "  {}: angle={:.3} target={:.3} rpm={:.1} target_rpm={:.1} out={:.0} age={} stall={} osc={:.1}Hz/{}",
        name,
        axis.motor.angle_rad,
        axis.target_angle_rad,
        axis.motor.speed_rpm,
        axis.target_rpm,
        axis.motor.command_voltage_raw,
        axis.feedback_age_ticks,
        axis.detector.stall_ticks,
        axis.detector.osc_freq_hz,
        axis.detector.osc_fast_ticks,
    );
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    let mut log_ticks = 0u16;
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        watchdog.pet();

        log_ticks += 1;
        if log_ticks >= 200 {
            log_ticks = 0;
            rtt_target::rprintln!(
                "state={:?} settle={} prepare={} fault={:?} tx_failed={}",
                control.state,
                control.settle_ticks,
                control.prepare_ticks,
                control.fault,
                TX_FAILED.load(Ordering::Relaxed),
            );
            log_axis("yaw", &control.yaw);
            log_axis("pitch", &control.pitch);
        }
    }
}
