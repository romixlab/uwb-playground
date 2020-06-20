use crate::board::hal;
use hal::stm32::Interrupt;

#[cfg(feature = "pozyx-board")]
use hal::gpio::{PushPull, Output, Alternate, AF5};
#[cfg(feature = "pozyx-board")]
use hal::gpio::{gpioa::{PA4, PA5, PA6, PA7}};

#[cfg(feature = "pozyx-board")]
pub type Dw1000Clk = PA5<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Miso = PA6<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Mosi = PA7<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Cs = PA4<Output<PushPull>>;

#[cfg(feature = "dragonfly-board")]
use hal::gpio::{PushPull, Input, Output, PullDown, Alternate, Floating, AF5};
#[cfg(feature = "dragonfly-board")]
use hal::gpio::{gpioa::{PA8}, gpiob::{PB3, PB4, PB5}};

#[cfg(feature = "dragonfly-board")]
pub type Dw1000Clk = PB3<Alternate<AF5, Input<Floating>>>;
#[cfg(feature = "dragonfly-board")]
pub type Dw1000Miso = PB4<Alternate<AF5, Input<Floating>>>;
#[cfg(feature = "dragonfly-board")]
pub type Dw1000Mosi = PB5<Alternate<AF5, Input<Floating>>>;
#[cfg(feature = "dragonfly-board")]
pub type Dw1000Cs = PA8<Output<PushPull>>;

/// Blink LED with specified period (alive indicator).
pub const BLINK_PERIOD_MS: u32 = 500;

/// Ignore IRQ and check state anyway with specified period.
/// To ensure that endless lockup won't happen.
#[cfg(feature = "slave")]
pub const DW1000_CHECK_PERIOD_MS: u32 = 1000;

/// Which EXTI line is used for DW1000 interrupt.
/// Ensure that radio_irq task is coherent with this!
pub const DW1000_IRQ_EXTI: Interrupt = Interrupt::EXTI0;

/// Period for synchronous data exchange (guaranteed time slots (GTS) with slaves).
#[cfg(feature = "master")]
pub const GTS_PERIOD_MS: u32 = 100;

/// Allocate at least this amount of GTS and signal a failure (to all slaves and to uplink)
/// if one or more is missing for >= THRESH
#[cfg(feature = "master")]
pub const REQUIRED_SLAVE_COUNT: u8 = 3;

#[cfg(feature = "tr")]
pub const SLAVE_ID: usize = 0;

#[cfg(feature = "bl")]
pub const SLAVE_ID: usize = 1;

#[cfg(feature = "br")]
pub const SLAVE_ID: usize = 2;