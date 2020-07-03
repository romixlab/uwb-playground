#[derive(Default, Debug, Copy, Clone)]
pub struct Rpm(pub i32);

#[derive(Default, Debug, Copy, Clone)]
pub struct Tachometer(pub i32);

#[derive(Default)]
pub struct MCData {
    /// 0 - no answer, +1 on answer from MC up to threshold, -1 on nack
    pub tokens: u8,
    pub rpm: Rpm,
    pub tacho: Tachometer,
    pub tacho_shift: i32,
    pub power_in: f32,
}

impl MCData {
    pub fn stop(&mut self) {
        self.rpm = Rpm(0);
    }
}

#[derive(Default)]
pub struct MecanumWheels {
    pub top_left: MCData,
    pub top_right: MCData,
    pub bottom_left: MCData,
    pub bottom_right: MCData
}

impl MecanumWheels {
    #[cfg(feature = "master")]
    pub fn all_stop(&mut self) {
        self.top_left.rpm = Rpm(0);
        self.top_right.rpm = Rpm(0);
        self.bottom_left.rpm = Rpm(0);
        self.bottom_right.rpm = Rpm(0);
    }
}

#[derive(Default, Debug)]
pub struct RpmArray {
    pub top_left: Rpm,
    pub top_right: Rpm,
    pub bottom_left: Rpm,
    pub bottom_right: Rpm
}

impl RpmArray {
    #[cfg(feature = "master")]
    pub fn is_all_zero(&self) -> bool {
        self.top_left.0 == 0 &&
            self.top_right.0 == 0 &&
            self.bottom_left.0 == 0 &&
            self.bottom_right.0 == 0
    }
}

#[derive(Debug)]
pub struct MilliMeters(pub i32);
#[derive(Debug)]
pub struct Degrees(pub i32);

#[derive(Debug)]
pub struct XYThetaMove {
    pub dx: MilliMeters,
    pub dy: MilliMeters,
    pub dtheta: Degrees
}

#[derive(Debug)]
pub enum MotorControlEvent {
    #[cfg(feature = "master")]
    /// Received from Ctrl input, no override.
    /// Save values into resources.mecanum_wheels, they will be sent to slaves by radio_chrono task.
    SetRpmArray(RpmArray),
    #[cfg(feature = "master")]
    /// Received from dev uwb node, override ctrl input
    /// Displace and keep heading while rotating (normal car).
    /// Calculate rpms fromm displacement and do the same as SetRpmArray.
    MovePreserveHeading(XYThetaMove),
    #[cfg(feature = "master")]
    /// Received from dev uwb node, override ctrl input.
    /// Displace and ingore heading change caused by rotation (drifting car).
    /// Calculate rpms fromm displacement and do the same as SetRpmArray.
    MoveIgnoreHeading(XYThetaMove),
    /// Commit SET_RPM frame to vesc send bip buffer. Pend vesc_serial task to
    /// actually start sending it.
    SetRpm(Rpm),
    /// Commit GET_VALUES_SELECTIVE to vesc send bip buffer. Pend vesc_serial task to
    /// actually start sending it.
    RequestTelemetry,
    /// Check if too much time passed since last command and turn off the motors.
    TimingCheck,
    /// Quick hack
    #[cfg(feature = "master")]
    ResetTacho // TODO: forward messages untouched and implement real reset
}

impl MotorControlEvent {
    /// Checks if the event is related to RPM change
    pub fn is_move_event(&self) -> bool {
        use MotorControlEvent::*;
        match self {
            #[cfg(feature = "master")]
            SetRpmArray(_) | MovePreserveHeading(_) | MoveIgnoreHeading(_) => { true },
            SetRpm(_) => { true },
            _ => { false }
        }
    }
}