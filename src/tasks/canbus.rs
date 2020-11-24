use crate::{
    config
};
use rtt_target::{rprint, rprintln};

#[derive(Default)]
pub struct DirectionStatistics {
    pub frames_processed: u32,
    pub frames_dropped: u32,
    pub bytes_processed: u32
}

#[derive(Default)]
pub struct Statistics {
    pub rx: DirectionStatistics,
    pub tx: DirectionStatistics
}

pub fn can0_irq0(mut cx: crate::can0_irq0::Context) {
    let can: &mut config::Can0 = cx.resources.can0;
    use crate::hal::can::ClassicalCan;
    let ll_statistics: &mut Statistics = cx.resources.can0_ll_statistics;

    rprintln!("can_irq0");
    rprintln!("reason: {:?}", can.interrupt_reason());
    // rprintln!("{:?}", can.protocol_status());
    // rprintln!("rec:{} tec:{}", can.receive_error_counter(), can.transmit_error_counter());
    // rprintln!("rx_pin: {:?}", can.rx_pin_state());
    // rprintln!("queue: {}", cx.resources.can0_send_heap.heap.len());
    if can.free_slots() != 0 {
        if let Some(frame) = cx.resources.can0_send_heap.heap.pop() {
            match can.send(&frame) {
                Ok(_) => {
                    ll_statistics.tx.frames_processed += 1;
                    ll_statistics.tx.bytes_processed += frame.len() as u32;
                },
                Err(_) => {
                    ll_statistics.tx.frames_dropped += 1;
                }
            }

            //rprintln!("send: {:?}{:?}{}", frame.id(), r, cx.resources.can0_send_heap.heap.len());
        }
    }

    let receive_heap: &mut config::CanReceiveHeap = cx.resources.can0_receive_heap;
    can.get_all(|id, data| {
        // rprintln!("rx: {:?}, {}", id, data.len());
        // for b in data {
        //     rprint!("{:02x} ", *b);
        // }
        // rprintln!("");
        match receive_heap.pool.new_frame(id, data) {
            Ok(frame) => {
                let len = frame.len() as u32;
                match receive_heap.heap.push(frame) {
                    Ok(_) => {
                        ll_statistics.rx.frames_processed += 1;
                        ll_statistics.rx.bytes_processed += len;
                    },
                    Err(_) => {
                        ll_statistics.rx.frames_dropped += 1;
                    }
                }
            },
            Err(_) => {
                ll_statistics.rx.frames_dropped += 1;
            }
        }
    });

    unsafe { can.regs_mut().ir.write(|w| w.bits(0x00ff_ffff)) };
}