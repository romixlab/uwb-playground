use crate::{
    config
};
use rtt_target::{rprint, rprintln};

pub fn can0_irq0(mut cx: crate::can0_irq0::Context) {
    let can: &mut config::Can0 = cx.resources.can0;
    use crate::hal::can::ClassicalCan;
    rprintln!("can_irq0");
    rprintln!("reason: {:?}", can.interrupt_reason());
    // rprintln!("{:?}", can.protocol_status());
    // rprintln!("rec:{} tec:{}", can.receive_error_counter(), can.transmit_error_counter());
    // rprintln!("rx_pin: {:?}", can.rx_pin_state());
    // rprintln!("queue: {}", cx.resources.can0_send_heap.heap.len());
    if can.free_slots() != 0 {
        if let Some(frame) = cx.resources.can0_send_heap.heap.pop() {
            let r = can.send(&frame);
            rprintln!("send: {:?}{:?}{}", frame.id(), r, cx.resources.can0_send_heap.heap.len());
        }
    }

    can.get_all(|id, data| {
        rprintln!("rx: {:?}, {}", id, data.len());
        for b in data {
            rprint!("{:02x} ", *b);
        }
        rprintln!("");
    });

    unsafe { can.regs_mut().ir.write(|w| w.bits(0x00ff_ffff)) };
}