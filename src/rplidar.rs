use rtt_target::{rprint, rprintln};
use crate::{color, config, newconfig};
use crate::rplidar::LidarState::Disconnected;
//use heapless::spsc::{Queue, Producer, Consumer};
use rtic::cyccnt::U32Ext;
use rtic::Mutex;

const TIMEOUT_TICKS: u8 = 60;
const BUFFER_SIZE: usize = 135;
pub const FRAME_SIZE: usize = 84;

// type QueueSize = typenum::consts::U64;
// pub struct Frame(pub [u8; FRAME_SIZE]);
// pub type LidarQueue = Queue<Frame, QueueSize>;
// pub type LidarQueueP = Producer<'static, Frame, QueueSize>;
// pub type LidarQueueC = Consumer<'static, Frame, QueueSize>;

use bbqueue::{BBBuffer, consts::*};
use bbqueue::framed::{FrameProducer, FrameConsumer, FrameGrantW};
use embedded_hal::serial::Write;
use crate::tasks::canbus::{ForwardHeap, ForwardEntry, Destination};
use vhrdcan::FrameId;

pub type LidarBBufferSize = U2048;
pub type LidarBBuffer = BBBuffer<LidarBBufferSize>;
pub type LidarBBufferC = FrameConsumer<'static, LidarBBufferSize>;
pub type LidarBBufferP = FrameProducer<'static, LidarBBufferSize>;

#[derive(PartialEq, Copy, Clone)]
pub enum LidarState {
    Disconnected,
    WaitingGetHealthAnswer,
    WaitingScanResponseHeader,
    ReceivingScans,
    Timeout
}

pub enum FramerState {
    /// initial and after response received
    Unlocked,
    /// waiting 5a after a5 received
    WaitingStart5A,
    /// waiting for single response length
    WaitingLen,
    /// waiting 5_ nibble after a_ received
    WaitingStart5_,
    /// receiving single response or scan response, depending on lidar state
    ResponseReceiving(u8),
}

pub struct RpLidar {
    state: LidarState,
    framer: FramerState,
    buffer: [u8; BUFFER_SIZE],
    buffer_idx: usize,
    scan_response_len: u8,
    timeout_ticks: u8,
    frame_p: FrameProducer<'static, LidarBBufferSize>,
    frame_wgr: Option<FrameGrantW<'static, LidarBBufferSize>>,
    //pub frame_queue: LidarQueue,
}

impl RpLidar {
    pub fn new(frame_producer: FrameProducer<'static, LidarBBufferSize>) -> Self {
        RpLidar {
            state: LidarState::Disconnected,
            framer: FramerState::Unlocked,
            buffer: [0u8; BUFFER_SIZE],
            buffer_idx: 0,
            scan_response_len: 0,
            timeout_ticks: TIMEOUT_TICKS,
            frame_p: frame_producer,
            frame_wgr: None,
            //frame_queue: heapless::spsc::Queue(heapless::i::Queue::new())
        }
    }

    pub fn state(&self) -> LidarState {
        self.state
    }

    pub fn eat_byte<T>(&mut self, b: u8, mut tx: T)
        where
            T: FnMut(&[u8])
    {
        use FramerState::*;
        use LidarState::*;

        self.framer = match self.framer {
            Unlocked => {
                self.buffer_idx = 0;
                match self.state {
                    Disconnected => {
                        //rprint!(=> 5, "{}E1{}", color::RED, color::DEFAULT);
                        Unlocked
                    },
                    WaitingGetHealthAnswer | WaitingScanResponseHeader => {
                        //rprint!(=> 5, "{}E2{}", color::RED, color::DEFAULT);

                        if b == 0xa5 {
                            self.timeout_ticks = TIMEOUT_TICKS;
                            WaitingStart5A
                        } else {
                            Unlocked
                        }
                    },
                    ReceivingScans => {
                        if b >> 4 == 0xa {
                            self.buffer[0] = b;
                            self.timeout_ticks = TIMEOUT_TICKS;
                            //rprint!(=> 5, "A2: {:02x}", b);

                            WaitingStart5_
                        } else {
                            rprint!(=> 5, "{}U{}", color::RED, color::DEFAULT);
                            Unlocked
                        }
                    },
                    Timeout => { Unlocked }
                }
            },
            WaitingStart5A => {
                if b == 0x5a {
                    self.timeout_ticks = TIMEOUT_TICKS;
                    WaitingLen
                } else {
                    //rprint!(=> 5, "{}E5{}", color::RED, color::DEFAULT);

                    Unlocked
                }
            },
            WaitingLen => {
                self.timeout_ticks = TIMEOUT_TICKS;
                if b > 30 { // scan response
                    self.scan_response_len = b;
                    ResponseReceiving(b + 4) // 4 byte to ignore
                } else { // regular answer
                    ResponseReceiving(4) // 4 byte to ignore
                }
            },
            WaitingStart5_ => {
                if b >> 4 == 0x5 {
                    self.buffer[1] |= b;
                    self.buffer_idx = 2;
                    self.timeout_ticks = TIMEOUT_TICKS;
                    if self.scan_response_len > 30 {
                        let r = self.frame_p.grant(self.scan_response_len as usize);
                        match r {
                            Ok(mut wgr) => {
                                wgr[0] = self.buffer[0];
                                wgr[1] = self.buffer[1];
                                self.frame_wgr = Some(wgr);
                                //rprintln!(=> 5, "WG:{}\n", b);
                            },
                            Err(_) => {
                                rprintln!(=> 5, "{}WGR:drop{}\n", color::YELLOW, color::DEFAULT);
                            }
                        }
                    }
                    // 2 bytes for angle & sync bit + payload len received previously in payload descriptor response
                    ResponseReceiving(self.scan_response_len - 2)
                } else {
                    rprint!(=> 5, "{}V{}", color::RED, color::DEFAULT);
                    Unlocked
                }
            },
            ResponseReceiving(bytes_left) => {
                self.buffer[self.buffer_idx] = b;
                match &mut self.frame_wgr {
                    Some(wgr) => {
                        wgr[self.buffer_idx] = b;
                    }
                    None => {}
                }
                self.buffer_idx += 1;
                if self.buffer_idx == BUFFER_SIZE {
                    rprint!(=> 5, "{}EO{}", color::RED, color::DEFAULT);

                    Unlocked
                } else {
                    self.timeout_ticks = TIMEOUT_TICKS;
                    let bytes_left = bytes_left - 1;
                    if bytes_left == 0 {
                        if self.frame_wgr.is_some() {
                            let wgr = self.frame_wgr.take().unwrap();
                            wgr.commit(self.scan_response_len as usize);
                            self.frame_wgr = None;
                        }
                        self.advance_state(tx);
                        Unlocked
                    } else {
                        ResponseReceiving(bytes_left)
                    }
                }
            },
        };
    }

    fn advance_state<T>(&mut self, mut tx: T)
        where
            T: FnMut(&[u8])
    {
        use LidarState::*;
        self.state = match self.state {
            ReceivingScans => {
                // if self.scan_response_len != 0 {
                //     on_scan(&self.buffer[..self.scan_response_len as usize]);
                // }
                // self.scan_response_len = 0;
                // if self.frame_wgr.is_some() {
                //     let wgr = self.frame_wgr.take().unwrap();
                //     wgr.commit(self.scan_response_len as usize);
                //     self.frame_wgr = None;
                // }
                ReceivingScans
            },
            Disconnected => {
                tx(&[0xa5, 0x52]);
                // get health status
                rprintln!(=> 5, "req get_health");
                WaitingGetHealthAnswer
            },
            WaitingGetHealthAnswer => {
                rprint!(=> 5, "{}get_health ans:[", color::GREEN);
                for i in 0..3 {
                    rprint!(=> 5, "{:02x} ", self.buffer[i]);
                }
                rprintln!(=> 5, "]{}", color::DEFAULT);
                tx(&[0xa5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22]); // scan start
                self.timeout_ticks = 70;
                WaitingScanResponseHeader
            },
            WaitingScanResponseHeader => {
                rprint!(=> 5, "{}scan_resp ans:[", color::GREEN,);
                for i in 0..4 {
                    rprint!(=> 5, "{:02x} ", self.buffer[i]);
                }
                rprintln!(=> 5, "]");
                rprintln!(=> 5, "scan_len: {}{}", self.scan_response_len, color::DEFAULT);
                ReceivingScans
            },
            Timeout => {
                rprintln!(=> 5, "timeout");
                //tx(&[0xa5, 0x40]); // reset
                Disconnected
            }
        }
    }

    pub fn periodic_check<T>(&mut self, tx: T)
    where
        T: FnMut(&[u8])
    {
        if self.timeout_ticks == 0 {
            self.state = LidarState::Timeout;
            self.advance_state(tx);
            self.timeout_ticks = TIMEOUT_TICKS;
            return;
        }
        self.timeout_ticks -= 1;
        if self.state == Disconnected {
            self.advance_state(tx);
        }
    }
}

pub enum TaskEvent {
    PeriodicCheck,
    DataReceived(&'static [u8])
}

pub fn push_can_frame(channels: &mut crate::channels::Channels, id: FrameId, data: &[u8]) {
    let forward_heap: &mut ForwardHeap = &mut channels.can0_forward_heap;
    let forward_pool: &mut vhrdcan::FramePool = &mut channels.can0_forward_pool;
    let frame = forward_pool.new_frame(id, data).unwrap();
    let forward_entry = ForwardEntry {
        to: Destination::Broadcast,
        frame
    };
    let r = forward_heap.push(forward_entry);
    if r.is_err() {
        rprintln!(=>5, "{}CAN dropped{}", color::YELLOW, color::DEFAULT);
    }
}

#[cfg(feature = "br")]
pub fn lidar_task(mut cx: crate::lidar::Context, e: TaskEvent, uavcan_transfer_id: &mut u8, frame_count: &mut usize) {
    let rplidar: &mut RpLidar = cx.resources.lidar;
    let serial: &mut config::ImxSerial = cx.resources.imx_serial;
    match e {
        TaskEvent::PeriodicCheck => {
            rplidar.periodic_check(|tx| {
                for b in tx {
                    nb::block!(serial.write(*b));
                }
            });
            cx.schedule.lidar(cx.scheduled + ms2cycles!(cx.resources.clocks, 100), TaskEvent::PeriodicCheck).ok();
            // rprintln!(=>5, "CHECK");
        }
        TaskEvent::DataReceived(bytes) => {
            for b in bytes {
                rplidar.eat_byte(*b, |tx| {
                    for b in tx {
                        nb::block!(serial.write(*b));
                    }
                });
            }
        }
    }

    let frame_c: &mut LidarBBufferC = cx.resources.lidar_frame_c;
    match frame_c.read() {
        Some(mut rgr) => {
            rgr.auto_release(true);

            rprintln!(=>5, "Frame: {}", *frame_count);
            *frame_count += 1;
            if rgr.len() != FRAME_SIZE {
                return;
            }
            let can_heap_free_slots: usize = cx.resources.channels.lock(|channels| channels.can0_forward_heap.capacity() - channels.can0_forward_heap.len());
            if can_heap_free_slots < 13 {
                rprintln!(=>5, "{}Early dropped{}", color::CYAN, color::DEFAULT);
                return;
            }

            let mut can_frame = [0u8; 8];
            let can_id = FrameId::new_extended(newconfig::LIDAR_UAVCAN_ID).unwrap();
            for i in 0..12 {
                can_frame[0..7].copy_from_slice(&rgr[i*7..i*7+7]);
                can_frame[7] = uavcan_tailbyte(i == 0, false, i+1, *uavcan_transfer_id);
                cx.resources.channels.lock(|channels| push_can_frame(channels, can_id, &can_frame));
            }
            let mut crc16 = crc16::State::<crc16::CCITT_FALSE>::new();
            crc16.update(&rgr);
            let crc16 = crc16.get().to_be_bytes();
            can_frame[0..2].copy_from_slice(&crc16);
            can_frame[2] = uavcan_tailbyte(false, true, 13, *uavcan_transfer_id);
            cx.resources.channels.lock(|channels| push_can_frame(channels, can_id, &can_frame[0..=2]));

            *uavcan_transfer_id += 1;
            if *uavcan_transfer_id == 32 {
                *uavcan_transfer_id = 0;
            }
        },
        None => {}
    }
}

pub fn uavcan_tailbyte(start_of_transfer: bool, end_of_transfer: bool, counter: usize, transfer_id: u8) -> u8 {
    ((start_of_transfer as u8) << 7) |
        ((end_of_transfer as u8) << 6) |
        (((counter % 2) as u8) << 5) |
        (transfer_id & 0b0001_1111)
}