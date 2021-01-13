
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

pub struct DumpDelay{

}

impl DumpDelay{
    pub fn new() -> Self {
        DumpDelay {

        }
    }

    pub fn delay(&mut self, delay: u32) {
        let cp: u32 = 625;//[ns] (1 / 160);
        cortex_m::asm::delay(((delay/cp) * 100_000) as u32);
    }

}

impl DelayUs<u32> for DumpDelay{
    fn delay_us(&mut self, us: u32) {
        self.delay(us);
    }
}

impl DelayMs<u32> for DumpDelay{
    fn delay_ms(&mut self, ms: u32) { self.delay_us((ms * 1_000) as u32);}
}