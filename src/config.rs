use crate::board::hal;
use crate::units::{
    MilliSeconds,
    ms
};
use typenum::consts;
use bbqueue::{BBBuffer, Consumer, Producer};
#[cfg(feature = "pozyx-board")]
use hal::gpio::{PushPull, Output, Alternate, Input, PullDown};

#[cfg(feature = "pozyx-board")]
use hal::gpio::{AF5, gpioa::{PA4, PA5, PA6, PA7}};

#[cfg(feature = "pozyx-board")]
pub type Dw1000Clk = PA5<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Miso = PA6<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Mosi = PA7<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Cs = PA4<Output<PushPull>>;

#[cfg(feature = "gcharger-board")]
use hal::gpio::{gpioa::*, gpiob::*, gpioc::*, gpiod::*};
#[cfg(feature = "gcharger-board")]
use hal::gpio::{Output, Input, PushPull, PullDown, DefaultMode};

#[cfg(feature = "gcharger-board")]
pub type Dw1000Clk = PC10<DefaultMode>;
#[cfg(feature = "gcharger-board")]
pub type Dw1000Miso = PC11<DefaultMode>;
#[cfg(feature = "gcharger-board")]
pub type Dw1000Mosi = PC12<DefaultMode>;
#[cfg(feature = "gcharger-board")]
pub type Dw1000Cs = PD2<Output<PushPull>>;

/// Blink LED with specified period (alive indicator).
pub const BLINK_PERIOD_MS: u32 = 500;

/// Ignore IRQ and check state anyway with specified period.
/// To ensure that endless lockup won't happen.
#[cfg(feature = "slave")]
pub const DW1000_CHECK_PERIOD: MilliSeconds = ms(1000);

use hal::stm32::Interrupt;
/// Which EXTI line is used for DW1000 interrupt.
/// Ensure that radio_irq task is coherent with this!
pub const DW1000_IRQ_EXTI: Interrupt = Interrupt::EXTI0;

/// Period for synchronous data exchange (guaranteed time slots (GTS) with slaves).
pub const GTS_PERIOD: MilliSeconds = ms(50);

use dw1000::mac::{PanId, ShortAddress};
pub const PAN_ID: PanId = PanId(0x777);

/// Allocate at least this amount of GTS and signal a failure (to all slaves and to uplink)
/// if one or more is missing for >= THRESH
pub const REQUIRED_SLAVE_COUNT: u8 = 3;

/// Maximum number of nodes in a PAN
pub type TotalNodeCount = consts::U5;

pub const DEFAULT_UWB_CHANNEL: UwbChannel = UwbChannel::Channel3;

#[cfg(feature = "master")]
pub const UWB_ADDR: ShortAddress = ShortAddress(0x999);

#[cfg(any(feature = "master", feature = "devnode", feature = "tr"))]
pub const TR_UWB_ADDR: ShortAddress = ShortAddress(0xaa_00); // 43520

#[cfg(any(feature = "master", feature = "devnode", feature = "bl"))]
pub const BL_UWB_ADDR: ShortAddress = ShortAddress(0xaa_01); // 43521

#[cfg(any(feature = "master", feature = "devnode", feature = "br"))]
pub const BR_UWB_ADDR: ShortAddress = ShortAddress(0xaa_02); // 43522

#[cfg(feature = "devnode")]
pub const DEV_UWB_ADDR: ShortAddress = ShortAddress(0x777);

#[cfg(feature = "tr")]
pub const SLAVE_ID: usize = 0;
#[cfg(feature = "tr")]
pub const UWB_ADDR: ShortAddress = TR_UWB_ADDR;

#[cfg(feature = "bl")]
pub const SLAVE_ID: usize = 1;
#[cfg(feature = "bl")]
pub const UWB_ADDR: ShortAddress = BL_UWB_ADDR;

#[cfg(feature = "br")]
pub const SLAVE_ID: usize = 2;
#[cfg(feature = "br")]
pub const UWB_ADDR: ShortAddress = BR_UWB_ADDR;

#[cfg(feature = "devnode")]
pub const SLAVE_ID: usize = REQUIRED_SLAVE_COUNT as usize - 1;
#[cfg(feature = "devnode")]
pub const UWB_ADDR: ShortAddress = DEV_UWB_ADDR;

#[cfg(feature = "anchor")]
pub const UWB_ADDR: ShortAddress = ShortAddress(0x666);

pub mod motor_control {
    /// How often timing checks are performed.
    pub const TIMING_CHECK_INTERVAL_MS: u32 = 100;
    /// Turn off all motors if no move commands received for this amount of time.
    pub const STOP_TIMEOUT_MS: u32 = 500;
}

#[cfg(feature = "pozyx-board")]
use hal::gpio::{gpioa::{PA0, PA1}, gpiob::{PB5}};
use dw1000::configs::UwbChannel;

#[cfg(feature = "pozyx-board")]
pub type LedBlinkyPin = PB5<Output<PushPull>>;
#[cfg(feature = "gcharger-board")]
pub type LedBlinkyPin = PB15<Output<PushPull>>;

#[cfg(feature = "pozyx-board")]
pub type RadioIrqPin = PA0<Input<PullDown>>;
//type RadioIrqPin = PC4<Input<PullDown>>;
#[cfg(feature = "gcharger-board")]
pub type RadioIrqPin = PC15<Input<PullDown>>;

#[cfg(feature = "pozyx-board")]
pub type RadioTracePin = PA1<Output<PushPull>>;
#[cfg(feature = "gcharger-board")]
pub type RadioTracePin = PA10<Output<PushPull>>;

pub const CHANNEL_EVENT_IRQ: Interrupt = Interrupt::EXTI4;

#[cfg(feature = "master")]
pub const DEVICE_NAME: &str = "Master";
#[cfg(feature = "tr")]
pub const DEVICE_NAME: &str = "TopRight";
#[cfg(feature = "bl")]
pub const DEVICE_NAME: &str = "BottomLeft";
#[cfg(feature = "br")]
pub const DEVICE_NAME: &str = "BottomRight";
#[cfg(feature = "anchor")]
pub const DEVICE_NAME: &str = "Anchor";