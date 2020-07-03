use core::fmt;
use core::fmt::Formatter;

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct Seconds(pub u32);

impl fmt::Display for Seconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}s", self.0)
    }
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MilliSeconds(pub u32);

impl fmt::Display for MilliSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}ms", self.0)
    }
}

impl Into<MilliSeconds> for Seconds {
    fn into(self) -> MilliSeconds { MilliSeconds(self.0 * 1_000) }
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MicroSeconds(pub u32);

impl fmt::Display for MicroSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}us", self.0)
    }
}

impl Into<MicroSeconds> for MilliSeconds {
    fn into(self) -> MicroSeconds { MicroSeconds(self.0 * 1_000) }
}
impl Into<MicroSeconds> for Seconds {
    fn into(self) -> MicroSeconds { MicroSeconds(self.0 * 1_000_000) }
}

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct NanoSeconds(pub u64);

impl fmt::Display for NanoSeconds {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{}ns", self.0)
    }
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