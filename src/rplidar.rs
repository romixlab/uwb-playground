use rtt_target::{rprint, rprintln};
use crate::rplidar::LidarState::Disconnected;
use heapless::spsc::{Queue, Producer, Consumer};

const TIMEOUT_TICKS: u8 = 30;
const BUFFER_SIZE: usize = 135;
pub const FRAME_SIZE: usize = 84;
type QueueSize = typenum::consts::U64;

pub struct Frame(pub [u8; FRAME_SIZE]);

pub type LidarQueue = Queue<Frame, QueueSize>;
pub type LidarQueueP = Producer<'static, Frame, QueueSize>;
pub type LidarQueueC = Consumer<'static, Frame, QueueSize>;

#[derive(PartialEq)]
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
    pub frame_queue: LidarQueue,
}

impl RpLidar {
    pub fn new() -> Self {
        RpLidar {
            state: LidarState::Disconnected,
            framer: FramerState::Unlocked,
            buffer: [0u8; BUFFER_SIZE],
            buffer_idx: 0,
            scan_response_len: 0,
            timeout_ticks: TIMEOUT_TICKS,
            frame_queue: heapless::spsc::Queue(heapless::i::Queue::new())
        }
    }

    pub fn eat_byte<S, T>(&mut self, b: u8, on_scan: S, mut tx: T)
        where
            S: FnMut(&[u8]),
            T: FnMut(&[u8])
    {
        use FramerState::*;
        use LidarState::*;

        self.framer = match self.framer {
            Unlocked => {
                self.buffer_idx = 0;
                match self.state {
                    Disconnected => {
                        rprint!(=> 5, "E1");
                        Unlocked
                    },
                    WaitingGetHealthAnswer | WaitingScanResponseHeader => {
                        rprint!(=> 5, "E2");

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
                            rprint!(=> 5, "E3: {:02x}", b);

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
                    rprint!(=> 5, "E5");

                    Unlocked
                }
            },
            WaitingLen => {
                self.timeout_ticks = TIMEOUT_TICKS;
                if b > 30 { // scan response
                    self.scan_response_len = b;
                    ResponseReceiving(4) // 4 byte to ignore
                } else { // regular answer
                    ResponseReceiving(b + 4) // 4 byte to ignore
                }
            },
            WaitingStart5_ => {
                if b >> 4 == 0x5 {
                    self.buffer[1] |= b;
                    self.buffer_idx = 2;
                    self.timeout_ticks = TIMEOUT_TICKS;
                    // 2 bytes for angle & sync bit + payload len received previously in payload descriptor response
                    ResponseReceiving(self.scan_response_len - 2)
                } else {
                    rprint!(=> 5, "E6");

                    Unlocked
                }
            },
            ResponseReceiving(bytes_left) => {
                self.buffer[self.buffer_idx] = b;
                self.buffer_idx += 1;
                if self.buffer_idx == BUFFER_SIZE {
                    rprint!(=> 5, "EO");

                    Unlocked
                } else {
                    self.timeout_ticks = TIMEOUT_TICKS;
                    let bytes_left = bytes_left - 1;
                    if bytes_left == 0 {
                        self.advance_state(on_scan, tx);
                        Unlocked
                    } else {
                        ResponseReceiving(bytes_left)
                    }
                }
            },
        };
    }

    fn advance_state<S, T>(&mut self, mut on_scan: S, mut tx: T)
        where
            S: FnMut(&[u8]),
            T: FnMut(&[u8])
    {
        use LidarState::*;
        self.state = match self.state {
            Disconnected => {
                tx(&[0xa5, 0x52]);
                // get health status
                rprintln!(=> 5, "req get_health");
                WaitingGetHealthAnswer
            },
            WaitingGetHealthAnswer => {
                rprint!(=> 5, "get_health ans:[");
                for i in 0..3 {
                    rprint!(=> 5, "{:02x} ", self.buffer[i]);
                }
                rprintln!(=> 5, "]");
                tx(&[0xa5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22]); // scan start
                self.timeout_ticks = 70;
                WaitingScanResponseHeader
            },
            WaitingScanResponseHeader => {
                rprint!(=> 5, "scan_resp ans:[");
                for i in 0..4 {
                    rprint!(=> 5, "{:02x} ", self.buffer[i]);
                }
                rprintln!(=> 5, "]");
                rprintln!(=> 5, "scan_len: {}", self.scan_response_len);
                ReceivingScans
            },
            ReceivingScans => {
                if self.scan_response_len != 0 {
                    on_scan(&self.buffer[..self.scan_response_len as usize]);
                }
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
            self.advance_state(|_| {}, tx);
            self.timeout_ticks = TIMEOUT_TICKS;
            return;
        }
        self.timeout_ticks -= 1;
        if self.state == Disconnected {
            self.advance_state(|_| {}, tx);
        }
    }
}