use crate::hal;
use rtt_target::rprintln;
use crate::color;
use typenum::marker_traits::Unsigned;
use crate::config;

#[cfg(feature = "br")]
pub fn lidar_dma_irq(mut cx: crate::lidar_dma_irq::Context) {
    let (half_complete, complete) = unsafe {
        let dma1 = &(*hal::stm32::DMA1::ptr());
        let hisr = dma1.hisr.read();
        dma1.hifcr.write(|w| w.ctcif5().clear().chtif5().clear());
        ( hisr.htif5().is_half(), hisr.tcif5().is_complete() )
    };
    let lidar_dma = &mut cx.resources.lidar_dma;
    let dma_buf_size = crate::tasks::ctrl_link::LidarDmaBufferSize::USIZE;
    if half_complete {
        if lidar_dma.first_half.is_some() {
            let wgr = lidar_dma.first_half.take().unwrap();
            wgr.commit(dma_buf_size / 2);
            rtic::pend(config::CTRL_IRQ_EXTI);
            lidar_dma.first_half = None;
        }
        match lidar_dma.producer.grant_exact(dma_buf_size / 2) {
            Ok(wgr) => {
                lidar_dma.second_half = Some(wgr);
                rprintln!(=>5, "DMA:F->S\n");
            },
            Err(_) => {
                rprintln!(=>5, "{}DMA:ErrorA\n{}", color::RED, color::DEFAULT);
            }
        }
    } else if complete {
        if lidar_dma.second_half.is_some() {
            let wgr = lidar_dma.second_half.take().unwrap();
            wgr.commit(dma_buf_size / 2);
            rtic::pend(config::CTRL_IRQ_EXTI);
            lidar_dma.second_half = None;
        }
        match lidar_dma.producer.grant_exact(dma_buf_size / 2) {
            Ok(wgr) => {
                lidar_dma.first_half = Some(wgr);
                rprintln!(=>5, "DMA:S->F\n");
            },
            Err(_) => {
                rprintln!(=>5, "{}DMA:ErrorB\n{}", color::RED, color::DEFAULT);
            }
        }
    }
}

pub fn dma_ctrl_serial_tx(
    cx: crate::dma_ctrl_serial_tx::Context,
    cx_rgr: &mut Option<bbqueue::GrantR<'static, config::CtrlBBBufferSize>>,
    rgr_len: &mut usize
) {
    unsafe {
        let dma1 = &(*hal::stm32::DMA1::ptr());
        let stream6 = &dma1.st[6];
        let hisr = dma1.hisr.read();
        if hisr.tcif6().is_complete() {
            dma1.hifcr.write(|w| w.ctcif6().clear());
        }
        let stream_is_active = stream6.cr.read().en().is_enabled();

        if stream_is_active {
            rprintln!(=>5, "{}DMA:pend\n{}", color::CYAN, color::DEFAULT);
            return; // transfer still in progress, someone tried to send smth and pended this irq
        }
        if cx_rgr.is_some() {
            // transfer is now complete, release the buffer
            rprintln!(=>5, "{}DMA:tr.finished\n{}", color::GREEN, color::DEFAULT);
            let rgr = cx_rgr.take().unwrap();
            rgr.release(*rgr_len);
            *cx_rgr = None;
        }
        match cx.resources.ctrl_bbbuffer_c.read() { // send more
            Ok(rgr) => {
                let len = rgr.len();
                stream6.ndtr.write(|w| w.bits(len as u32)); // transfer size
                let mem_addr = rgr.as_ptr() as *const _ as u32;
                stream6.m0ar.write(|w| w.bits(mem_addr));
                *cx_rgr = Some(rgr);
                *rgr_len = len;
                rprintln!(=>5, "{}DMA:send {}\n{}", color::GREEN, len, color::DEFAULT);

                stream6.cr.modify(|_, w| w.en().enabled());
            },
            Err(_) => {} // nothing to send
        }
    }
}