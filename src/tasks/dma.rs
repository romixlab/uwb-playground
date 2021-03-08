use crate::hal;
use rtt_target::rprintln;
use crate::color;
use typenum::marker_traits::Unsigned;
use crate::config;
use bbqueue::{BBBuffer, Consumer, GrantR, GrantW, consts::*, Producer};
use generic_array::ArrayLength;
use core::ops::Deref;
use hal::stm32::usart1::RegisterBlock as UsartRegisterBlock;
use hal::stm32::dma1::RegisterBlock as DmaRegisterBlock;

// pub enum Usart {
//     Usart1,
//     Usart2,
//     Usart3
// }

pub struct DmaRxContext<N>
where
    N: generic_array::ArrayLength<u8>,
{
    producer: Producer<'static, N>,
    first_half_wgr: Option<GrantW<'static, N>>,
    second_half_wgr: Option<GrantW<'static, N>>,
    usart: *mut UsartRegisterBlock,
    dma: *mut DmaRegisterBlock,
    last_committed: usize,
    array: *const u8,
    // stream: usize,
}

unsafe impl<N: generic_array::ArrayLength<u8>> Send for DmaRxContext<N> {}

impl<N: ArrayLength<u8>> DmaRxContext<N>
where
    N: generic_array::ArrayLength<u8>,
{
    pub fn new(
        mut producer: Producer<'static, N>,
        usart: *mut UsartRegisterBlock,
        dma: *mut DmaRegisterBlock,
        // stream: usize
    ) -> (Self, u32)
    {
        let wgr = producer.grant_exact(N::USIZE / 2).unwrap();
        let mem_addr = wgr.as_ptr() as *const _ as u32;
        let array = wgr.as_ptr();
        (
            DmaRxContext {
                producer,
                first_half_wgr: Some(wgr), second_half_wgr: None,
                usart, dma,
                last_committed: 0,
                array
                // stream
            },
            mem_addr
        )
    }

    pub fn buf_size(&self) -> usize { N::USIZE - 1 }

    fn handle_rx<F: FnMut(&'static [u8])>(&mut self, mut notify_consumer: F) {
        let dma = unsafe { &(*self.dma) };
        let dma_idx = self.buf_size() - dma.cndtr1.read().bits() as usize;
        // rprintln!(=>5, "idx:{} lc: {}\n", dma_idx, self.last_committed);
        if dma_idx > self.last_committed {
            notify_consumer(unsafe {
                &*core::ptr::slice_from_raw_parts(
                    self.array.offset(self.last_committed as isize),
                    dma_idx - self.last_committed)
            });
        } else if self.last_committed > dma_idx {
            notify_consumer(unsafe {
                &*core::ptr::slice_from_raw_parts(
                    self.array.offset(self.last_committed as isize),
                    self.buf_size() - self.last_committed)
            });
            if dma_idx > 0 {
                notify_consumer(unsafe {
                    &*core::ptr::slice_from_raw_parts(
                        self.array,
                        dma_idx)
                });
            }
        }
        self.last_committed = dma_idx;
    }

    pub fn handle_dma_rx_irq<F: FnMut(&'static [u8])>(&mut self, mut notify_consumer: F) {
        let dma = unsafe { &(*self.dma) };
        dma.ifcr.write(|w| w.tcif1().set_bit().htif1().set_bit());
        // rprintln!(=>5, "dma");
        self.handle_rx(notify_consumer);
    }

    pub fn handle_usart_irq<F: FnMut(&'static [u8])>(&mut self, mut notify_consumer: F) {
        let usart = unsafe { &mut(*self.usart) };
        usart.icr.write(|w| w.idlecf().set_bit());
        // rprintln!(=>5, "idle");
        self.handle_rx(notify_consumer);
    }
}

// pub struct DmaTxContext<N: ArrayLength<u8>> {
//     consumer: Consumer<'static, N>,
//     rgr: Option<GrantR<'static, N>>,
//     rgr_len: usize,
// }
//
// impl<N: ArrayLength<u8>> DmaTxContext<N> {
//     pub fn new(consumer: Consumer<'static, N>) -> Self {
//         DmaTxContext {
//             consumer,
//             rgr: None, rgr_len: 0
//         }
//     }
//
//     pub fn handle_dma_tx_irq(&mut self) {
//         unsafe {
//             let dma1 = &(*hal::stm32::DMA1::ptr());
//             let stream6 = &dma1.st[6];
//             let hisr = dma1.hisr.read();
//             if hisr.tcif6().is_complete() {
//                 dma1.hifcr.write(|w| w.ctcif6().clear());
//             }
//             let stream_is_active = stream6.cr.read().en().is_enabled();
//
//             if stream_is_active {
//                 //rprintln!(=>5, "{}DMA:pend\n{}", color::CYAN, color::DEFAULT);
//                 return; // transfer still in progress, someone tried to send smth and pended this irq
//             }
//             if self.rgr.is_some() {
//                 // transfer is now complete, release the buffer
//                 //rprintln!(=>5, "{}DMA:tr.finished\n{}", color::GREEN, color::DEFAULT);
//                 let rgr = self.rgr.take().unwrap();
//                 rgr.release(self.rgr_len);
//                 self.rgr = None;
//             }
//             match self.consumer.read() { // send more
//                 Ok(rgr) => {
//                     let len = rgr.len();
//                     stream6.ndtr.write(|w| w.bits(len as u32)); // transfer size
//                     let mem_addr = rgr.as_ptr() as *const _ as u32;
//                     stream6.m0ar.write(|w| w.bits(mem_addr));
//                     self.rgr = Some(rgr);
//                     self.rgr_len = len;
//                     //rprintln!(=>5, "{}DMA:send {}\n{}", color::GREEN, len, color::DEFAULT);
//
//                     stream6.cr.modify(|_, w| w.en().enabled());
//                 },
//                 Err(_) => {} // nothing to send
//             }
//         }
//     }
// }