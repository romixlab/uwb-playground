use crate::board::hal;
use crate::units::{MilliSeconds, ms, U32UnitsExt};
use typenum::consts;
use bbqueue::{BBBuffer, Consumer, Producer};
use hal::gpio::{PushPull, Output, Input, PullDown, Floating, DefaultMode, OpenDrain};
use hal::stm32::I2C1;
use heapless::binary_heap::{BinaryHeap, Min};
use heapless::consts::*;

#[cfg(feature = "pozyx-board")]
use hal::gpio::{AF5, gpioa::{PA4, PA5, PA6, PA7}};

#[cfg(feature = "gcharger-board")]
pub type Can0Tx = PB9<Input<Floating>>;
#[cfg(feature = "gcharger-board")]
pub type Can0Rx = PB8<Input<Floating>>;
#[cfg(any(feature = "gcharger-board", feature = "gcarrier-board"))]
pub type Can0 = hal::can::ClassicalCanInstance;

pub type CanSendHeap = vhrdcan::FrameHeap<U256>; // uwb -> can
pub type CanReceiveHeap = vhrdcan::FrameHeap<U128>; // from can
pub type CanLocalProcessingHeap = vhrdcan::FrameHeap<U32>; // receiveHeap -> Routing table -> this
pub type ForwardHeapSize = consts::U512;
pub const CAN0_SEND_IRQ: Interrupt = Interrupt::FDCAN1_INTR1_IT;
pub type RxRoutingTableSize = consts::U32;

pub type ImxSerialTx = PB3<Input<Floating>>;
pub type ImxSerialRx = PA15<Input<Floating>>;
pub type ImxSerial = hal::serial::Serial<hal::stm32::USART2, hal::serial::config::FullConfig>;
pub type Usart2DmaRxBufferSize = consts::U129; // Must be power of 2 + 1 for bip buffer to function properly in this case
pub type Usart2DmaRxBuffer = BBBuffer<Usart2DmaRxBufferSize>;
pub type Usart2DmaRxBufferC = Consumer<'static, Usart2DmaRxBufferSize>;
pub type Usart2DmaRxContext = crate::tasks::dma::DmaRxContext<Usart2DmaRxBufferSize>;
// pub type Usart2DmaTxBuffer = BBBuffer<Usart2DmaTxBufferSize>;
// pub type Usart2DmaTxBufferP = Producer<'static, Usart2DmaTxBufferSize>;
// pub type Usart2DmaTxContext = crate::tasks::dma::DmaTxContext<Usart2DmaTxBufferSize>;
// Pend after any write to [tx buffer](Usart2DmaTxBufferP) to start the DMA.
// pub const USART2_TX_DMA: Interrupt = Interrupt::DMA1_STREAM6;


pub type I2cPortType = hal::i2c::I2c<I2C1,
    hal::gpio::gpiob::PB7<hal::gpio::Output<OpenDrain>>, //SDA
    hal::gpio::gpioa::PA15<hal::gpio::Output<OpenDrain>>, //SCL
>;

#[cfg(feature = "pozyx-board")]
pub type Dw1000Clk = PA5<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Miso = PA6<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Mosi = PA7<Alternate<AF5>>;
#[cfg(feature = "pozyx-board")]
pub type Dw1000Cs = PA4<Output<PushPull>>;

#[cfg(any(feature = "gcharger-board", feature = "gcarrier-board"))]
use hal::gpio::{gpioa::*, gpiob::*, gpioc::*, gpiod::*, gpiof::*};

#[cfg(feature = "gcharger-board")]
pub type Dw1000Clk = PC10<DefaultMode>;
#[cfg(feature = "gcharger-board")]
pub type Dw1000Miso = PC11<DefaultMode>;
#[cfg(feature = "gcharger-board")]
pub type Dw1000Mosi = PC12<DefaultMode>;
#[cfg(feature = "gcharger-board")]
pub type Dw1000Cs = PD2<Output<PushPull>>;

// UWB-A
#[cfg(all(feature = "gcarrier-board", feature = "uwb-a"))]
pub type Dw1000Clk = PA5<DefaultMode>;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-a"))]
pub type Dw1000Miso = PA6<DefaultMode>;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-a"))]
pub type Dw1000Mosi = PA7<DefaultMode>;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-a"))]
pub type Dw1000Cs = PA1<Output<PushPull>>;

// UWB-B
#[cfg(all(feature = "gcarrier-board", feature = "uwb-b"))]
pub type Dw1000Clk = PF1<DefaultMode>;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-b"))]
pub type Dw1000Miso = PB14<DefaultMode>;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-b"))]
pub type Dw1000Mosi = PB15<DefaultMode>;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-b"))]
pub type Dw1000Cs = PC7<Output<PushPull>>;

/// Blink LED with specified period (alive indicator).
pub const BLINK_PERIOD_MS: u32 = 500;

/// Ignore IRQ and check state anyway with specified period.
/// To ensure that endless lockup won't happen.
#[cfg(any(feature = "slave", feature = "anchor"))]
pub const DW1000_CHECK_PERIOD: MilliSeconds = ms(1000);

use hal::stm32::Interrupt;
/// Which EXTI line is used for DW1000 interrupt.
/// Ensure that radio_irq task is coherent with this!
#[cfg(feature = "gcharger-board")]
pub const DW1000_IRQ_EXTI: Interrupt = Interrupt::EXTI15_10;
#[cfg(feature = "gcarrier-board")]
pub const DW1000_IRQ_EXTI: Interrupt = Interrupt::EXTI9_5;

/// Period for synchronous data exchange (guaranteed time slots (GTS) with slaves).
pub const GTS_PERIOD: MilliSeconds = ms(20);

use dw1000::mac::{PanId, ShortAddress};
pub const PAN_ID: PanId = PanId(0x777);

/// Allocate at least this amount of GTS and signal a failure (to all slaves and to uplink)
/// if one or more is missing for >= THRESH
pub const REQUIRED_SLAVE_COUNT: u8 = 4; // 2 for BR

/// Maximum number of nodes in a PAN
// pub type TotalNodeCount = consts::U5;

#[cfg(not(feature = "dev-uwb-channel"))]
pub const DEFAULT_UWB_CHANNEL: UwbChannel = UwbChannel::Channel5;

#[cfg(feature = "dev-uwb-channel")]
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

// #[cfg(feature = "tr")]
// pub const SLAVE_ID: usize = 0;
#[cfg(feature = "tr")]
pub const UWB_ADDR: ShortAddress = TR_UWB_ADDR;

// #[cfg(feature = "bl")]
// pub const SLAVE_ID: usize = 1;
#[cfg(feature = "bl")]
pub const UWB_ADDR: ShortAddress = BL_UWB_ADDR;

// #[cfg(feature = "br")]
// pub const SLAVE_ID: usize = 2;
#[cfg(feature = "br")]
pub const UWB_ADDR: ShortAddress = BR_UWB_ADDR;

#[cfg(feature = "devnode")]
pub const SLAVE_ID: usize = REQUIRED_SLAVE_COUNT as usize - 1;
#[cfg(feature = "devnode")]
pub const UWB_ADDR: ShortAddress = DEV_UWB_ADDR;

#[cfg(feature = "anchor")]
pub const UWB_ADDR: ShortAddress = ShortAddress(0x666);

#[cfg(feature = "pozyx-board")]
use hal::gpio::{gpioa::{PA0, PA1}, gpiob::{PB5}};
use dw1000::configs::UwbChannel;

#[cfg(feature = "pozyx-board")]
pub type LedBlinkyPin = PB5<Output<PushPull>>;
#[cfg(feature = "gcharger-board")]
pub type LedBlinkyPin = PB15<Output<PushPull>>;
#[cfg(feature = "gcarrier-board")]
pub type LedBlinkyPin = PB1<Output<PushPull>>;

#[cfg(feature = "pozyx-board")]
pub type RadioIrqPin = PA0<Input<PullDown>>;
//type RadioIrqPin = PC4<Input<PullDown>>;
#[cfg(feature = "gcharger-board")]
pub type RadioIrqPin = PC15<Input<PullDown>>;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-a"))]
pub type RadioIrqPin = PA8<Input<PullDown>>;
#[cfg(all(feature = "gcarrier-board", feature = "uwb-b"))]
pub type RadioIrqPin = PC8<Input<PullDown>>;

#[cfg(feature = "pozyx-board")]
pub type RadioTracePin = PA1<Output<PushPull>>;
#[cfg(feature = "gcharger-board")]
pub type RadioTracePin = PA2<Output<PushPull>>;
#[cfg(feature = "gcarrier-board")]
pub type RadioTracePin = PC3<Output<PushPull>>;

// pub const CHANNEL_EVENT_IRQ: Interrupt = Interrupt::EXTI4;

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
