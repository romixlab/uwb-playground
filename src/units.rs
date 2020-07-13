use core::fmt;
use core::fmt::Formatter;
use serde::{Serialize, Deserialize};
use core::ops::Sub;

#[derive(Eq, PartialEq, PartialOrd, Clone, Copy,)]
pub struct Seconds(pub u32);

impl fmt::Display for Seconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result { write!(f, "{}s", self.0) }
}

impl fmt::Debug for Seconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result { write!(f, "{}", self) }
}

#[derive(Eq, PartialEq, PartialOrd, Clone, Copy,)]
pub struct MilliSeconds(pub u32);

impl fmt::Display for MilliSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}ms", self.0)
    }
}

impl fmt::Debug for MilliSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Into<MilliSeconds> for Seconds {
    fn into(self) -> MilliSeconds { MilliSeconds(self.0 * 1_000) }
}

impl core::ops::Sub<MicroSeconds> for MilliSeconds {
    type Output = MicroSeconds;

    fn sub(self, rhs: MicroSeconds) -> Self::Output {
        MicroSeconds(self.0 * 1000 - rhs.0)
    }
}

#[derive(Eq, PartialEq, PartialOrd, Clone, Copy,)]
pub struct MicroSeconds(pub u32);

impl fmt::Display for MicroSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}us", self.0)
    }
}

impl fmt::Debug for MicroSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result { write!(f, "{}", self) }
}

impl Into<MicroSeconds> for MilliSeconds {
    fn into(self) -> MicroSeconds { MicroSeconds(self.0 * 1_000) }
}

impl Into<MicroSeconds> for Seconds {
    fn into(self) -> MicroSeconds { MicroSeconds(self.0 * 1_000_000) }
}

impl core::ops::Sub for MicroSeconds {
    type Output = MicroSeconds;

    fn sub(self, rhs: Self) -> Self::Output {
        MicroSeconds(self.0 - rhs.0)
    }
}

#[derive(Eq, PartialEq, PartialOrd, Clone, Copy)]
pub struct NanoSeconds(pub u64);

impl fmt::Display for NanoSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}ns", self.0)
    }
}

impl fmt::Debug for NanoSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result { write!(f, "{}", self) }
}

impl Into<NanoSeconds> for MicroSeconds {
    fn into(self) -> NanoSeconds { NanoSeconds(self.0 as u64 * 1_000) }
}

impl Into<NanoSeconds> for MilliSeconds {
    fn into(self) -> NanoSeconds { NanoSeconds(self.0 as u64 * 1_000_000) }
}

impl Into<NanoSeconds> for Seconds {
    fn into(self) -> NanoSeconds { NanoSeconds(self.0 as u64 * 1_000_000_000) }
}

pub trait U32UnitsExt {
    fn us(self) -> MicroSeconds;
    fn ms(self) -> MilliSeconds;
    fn s(self) -> Seconds;
}

impl U32UnitsExt for u32 {
    fn us(self) -> MicroSeconds { MicroSeconds(self) }

    fn ms(self) -> MilliSeconds { MilliSeconds(self) }

    fn s(self) -> Seconds { Seconds(self) }
}

pub const fn us(amount: u32) -> MicroSeconds { MicroSeconds(amount) }
pub const fn ms(amount: u32) -> MilliSeconds { MilliSeconds(amount) }