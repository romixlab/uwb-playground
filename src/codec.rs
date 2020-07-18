use bincrc_codec::{
    BinCrc,
    BinCrcError
};
use crate::hal::stm32::Interrupt;
use crate::config;

pub type Usart1Coder = BinCrcBBCoder<config::Usart1DmaTxBufferSize>;
pub type Usart1Decoder = BinCrcBBDecoder<config::Usart1DmaRxBufferSize, config::Usart1MaxFrameSize>;

pub type Usart2Coder = BinCrcBBCoder<config::Usart2DmaTxBufferSize>;
pub type Usart2Decoder = BinCrcBBDecoder<config::Usart2DmaRxBufferSize, config::Usart2MaxFrameSize>;

/// B bip buffer length
/// F maximum frame length
pub struct BinCrcBBDecoder<B, F>
where
    B: generic_array::ArrayLength<u8>,
    F: generic_array::ArrayLength<u8>,
{
    consumer: bbqueue::Consumer<'static, B>,
    bincrc: BinCrc<F>
}

impl<B, F> BinCrcBBDecoder<B, F>
where
    B: generic_array::ArrayLength<u8>,
    F: generic_array::ArrayLength<u8>,
{
    pub fn new(consumer: bbqueue::Consumer<'static, B>) -> Self {
        BinCrcBBDecoder {
            consumer,
            bincrc: BinCrc::new()
        }
    }

    pub fn consume_all<D: FnMut(&[u8])>(&mut self, mut on_frame: &mut D) {
        match self.consumer.read() {
            Ok(rgr) => {
                for byte in rgr.iter() {
                    rtt_target::rprintln!(=>5, "b: {}", *byte);
                    self.bincrc.eat_byte(*byte, on_frame);
                }
                let len = rgr.len();
                rgr.release(len);
                rtt_target::rprintln!(=>5, "release: {}", len);
            },
            Err(_) => {}
        }
    }
}

pub struct BinCrcBBCoder<B>
where
    B: generic_array::ArrayLength<u8>,
{
    producer: bbqueue::Producer<'static, B>,
    irq: Interrupt,
}

impl<B> BinCrcBBCoder<B>
where
    B: generic_array::ArrayLength<u8>,
{
    pub fn new(producer: bbqueue::Producer<'static, B>, irq: Interrupt) -> Self {
        BinCrcBBCoder {
            producer, irq
        }
    }

    pub fn commit_frame(&mut self, frame: &[u8]) -> Result<(), BinCrcError> {
        let bytes_required = BinCrc::<B>::size_hint(frame.len())?;
        let mut wgr = self.producer.grant_exact(bytes_required).map_err(|_| BinCrcError::NotEnoughSpace)?;
        BinCrc::<B>::commit_frame(frame, &mut wgr);
        rtic::pend(self.irq);
        Ok(())
    }
}