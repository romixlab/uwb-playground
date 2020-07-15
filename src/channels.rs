use crate::radio::channelization::{
    Arbiter,
    LogicalDestination,
    ChannelId,
    Multiplex
};
use crate::radio::Error;
use crate::motion::MoveCommand;
use rtt_target::{
    rprint,
    rprintln
};
use crate::rplidar;
use cfg_if::cfg_if;
cfg_if! {
    if #[cfg(feature = "slave")] {
        use crate::motion::Tachometer;
    }
}

/// **Reliability**
/// * `R` - (Reliable) ACKed, with retransmission (as in TCP).
/// * `N` - NACKed, with retransmission (as in CoAP).
/// * `I` - Ignore lost bytes/frames (as in UDP).
///
/// **Queueing**
/// * `0` - Unbuffered.
/// * `P` - Packetized queue with fixed or dynamic frames.
/// * `B` - Byte buffer or Stream.
///
/// **Ordering**
/// * `L` - LIFO (need newest data).
/// * `F` - FIFO (need sequential data).
///
/// **Timing**
/// * `A` - Asynchronous, sent some time in the future.
/// * `T` - Timed / synchronous, sent in a predetermined window.
///
/// **Overflow behaviour**
/// * `D` - Drop on overflow.
/// * `G` - Stop on overflow (gave up).
/// * `P` - Panic on overflow.
///
/// **Security**
/// * `E` - Encrypted.
/// * `H` - Hackable.
///
/// **Duplicates**
/// * `S` - Sequence numbered, repeated data dropped (copies may arrive as in UDP or redundant systems).
/// * `U` - Unique, channel does not create copies.
///
/// **Priority**
/// * `1` and up, default is 1. Equal priorities are round-robin scheduled. Lowest priority sync is higher than any other async.
///
/// **Channels**
/// * `@ID` - data is grabbed from specific queue and enqueued to the same id on the other node.
pub struct Channels {
    ///// I/0/_/T/G/H/_/10 @66
    //pub stop_command: Option<Stop>,
    //#[queue(I/0/L/T/D/H/S/9 @70)]
    pub move_command: Option<MoveCommand>,
    ///// I/0/L/T/D/H/S/9 @71
    //pub odometry: Option<OdometryData>
    ///// I/0/_/T/D/H/S/8 @72
    //pub heartbeat: Option<Heartbeat>

    ///// I/P/L/A/D/H/U/10 @170
    #[cfg(feature = "br")]
    pub lidar_queue_c: rplidar::LidarQueueC,
    #[cfg(feature = "master")]
    pub lidar_queue_p: rplidar::LidarQueueP,

    ///// R/P/F/A/D/H/S/9 @171
    // pub reqrep

}

impl Channels {
    #[cfg(not(any(feature = "br", feature = "master")))]
    pub fn new() -> Self {
        Channels {
            move_command: None,
        }
    }
}

impl Arbiter for Channels {
    type Error = Error;

    fn source_sync<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M)
    {
        cfg_if! {
            if #[cfg(feature = "slave")] {
                let tacho = Tachometer(1234);
                let _ = mux.mux(&tacho, LogicalDestination::Implicit, ChannelId::new(1));
            }
        }
    }

    fn source_async<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M) {
        // let mut saw =[0u8; 84*4];
        // for i in 0..saw.len() {
        //     saw[i] = i as u8;
        // }
        // mux.mux(&&saw[..], LogicalDestination::Implicit, ChannelId::new(11));

        // let angle: u16 = ((scan[3] & 0b0111_1111) as u16) << 8 | scan[2] as u16;
        // let angle_dec = angle / 64;
        // let angle_frac = angle % 64;
        // rprintln!(=> 5, "{}.{}\n", angle_dec, angle_frac);
        // for i in 0..84 {
        //     rprint!(=>5, "{:02x} ", scan[i]);
        // }
        // rprintln!(=>5, "]\n");

        cfg_if! {
            if #[cfg(feature = "br")] {
                let mut scans_written = 0;
                while self.lidar_queue_c.ready() {
                    if scans_written > 5 {
                        break;
                    }
                    match self.lidar_queue_c.dequeue() {
                        Some(frame) => {
                            mux.mux(&&frame.0[..], LogicalDestination::Implicit, ChannelId::new(11));
                            scans_written += 1;

                        },
                        None => { }
                    }
                }
            }
        }
    }

    fn sink_sync(&mut self, channel: ChannelId, chunk: &[u8]) {
        rprint!(=>1, "arb: CH{}:[", channel);
        for b in chunk {
            rprint!(=>1, "{:02x} ", b);
        }
        rprintln!(=>1, "]\n");
    }

    fn sink_async(&mut self, channel: ChannelId, chunk: &[u8]) {
        rprint!(=>1, "async: CH{}:{}[", channel, chunk.len());
        for b in &chunk[..10] {
            rprint!(=>1, "{:02x} ", b);
        }
        rprintln!(=>1, "]\n");
        cfg_if! {
            if #[cfg(feature = "master")] {
                if channel == ChannelId::new(11) && chunk[0] == 0xfd && chunk.len() == rplidar::FRAME_SIZE + 1 {
                    let mut frame = [0u8; rplidar::FRAME_SIZE];
                    frame.copy_from_slice(&chunk[1..]);
                    let frame = rplidar::Frame(frame);
                    self.lidar_queue_p.enqueue(frame).ok();
                }
            }
        }
    }
}