use crate::radio::channelization::{
    Arbiter,
    LogicalDestination,
    ChannelId,
    Multiplex
};
use crate::motion::MoveCommand;
use rtt_target::{
    rprint,
    rprintln
};

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

    //#[cfg(any(feature = "master", feature = "br"))]
    ///// I/P/L/A/D/H/U/10 @170
    //pub lidar_data: crate::rplidar::LidarQueue,

    ///// R/P/F/A/D/H/S/9 @171
    // pub reqrep

}

impl Channels {
    pub fn new() -> Self {
        Channels {
            move_command: None,
        }
    }
}

impl Arbiter for Channels {
    fn source_sync<M: Multiplex>(&mut self, mux: &mut M)
    {

    }

    fn source_async<M: Multiplex>(&mut self, mux: &mut M) {
        unimplemented!()
    }

    fn sink_async(&mut self, channel: ChannelId, chunk: &[u8]) {
        unimplemented!()
    }

    fn sink_sync(&mut self, channel: ChannelId, chunk: &[u8]) {
        rprint!(=>1, "arb: CH{}:[", channel);
        for b in chunk {
            rprint!(=>1, "{:02x} ", b);
        }
        rprintln!(=>1, "]\n");
    }
}