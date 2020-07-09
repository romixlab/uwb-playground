use core::fmt::Formatter;
/// * busywait!(ms, rtic_cx, 100);
/// * busywait!(us, rtix_cx, 100);
/// * busywait!(ms_alt, clocks, 100);
macro_rules! busywait {
    (ms, $cx:ident, $amount:expr) => {
        cortex_m::asm::delay($cx.resources.clocks.sysclk().0 / 1_000 * $amount);
    };
    (ms_alt, $clocks:ident, $amount:expr) => {
        cortex_m::asm::delay($clocks.sysclk().0 / 1_000 * $amount);
    };
    (us, $cx:ident, $amount:expr) => {
        cortex_m::asm::delay($cx.resources.clocks.sysclk().0 / 1_000_000 * $amount);
    };
}

/// * ms2cycles!(rtic_cx, 100);
macro_rules! ms2cycles {
    ($cx:ident, $amount:expr) => {
        ($cx.resources.clocks.sysclk().0 / 1_000 * $amount).cycles()
    };
}

macro_rules! ms2cycles_raw {
    ($cx:ident, $amount:expr) => {
        $cx.resources.clocks.sysclk().0 / 1_000 * $amount
    };
}

macro_rules! cycles2ms {
    ($cx:ident, $amount:expr) => {
        (($amount as u64) * 1_000) / $cx.resources.clocks.sysclk().0 as u64
    };
}

#[allow(unused_macros)]
macro_rules! cycles2us {
    ($cx:ident, $amount:expr) => {
        (($amount as u64) * 1_000_000) / $cx.resources.clocks.sysclk().0 as u64
    };
}

#[allow(unused_macros)]
macro_rules! cycles2us_raw {
    ($sysclk:expr, $amount:expr) => {
        (($amount as u64) * 1_000_000) / $sysclk as u64
    };
}

#[derive(PartialEq)]
pub enum TraceEvent {
    GTSStart,
    GTSStartedReceivingAnswers,
    GTSAnswerReceived,
    GTSEnded,

    GTSAnswerSent,

    RequestedSlotStarted,
    MessageReceived,
    MessageSent,
    RequestedSlotEnded
}

impl core::fmt::Display for TraceEvent {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            TraceEvent::GTSStart => { write!(f, "G_S") },
            TraceEvent::GTSStartedReceivingAnswers => { write!(f, "G_W") },
            TraceEvent::GTSAnswerReceived => { write!(f, "G_AR") },
            TraceEvent::GTSEnded => { write!(f, "G_E") },
            TraceEvent::RequestedSlotStarted => { write!(f, "R_S") },
            TraceEvent::MessageReceived => { write!(f, "MR") },
            TraceEvent::MessageSent => { write!(f, "MS") },
            TraceEvent::RequestedSlotEnded => { write!(f, "R_E") },
            TraceEvent::GTSAnswerSent => { write!(f, "G_AS") }
        }
    }
}

pub trait Tracer {
    fn event(&mut self, e: TraceEvent);
}