use crate::radio::channelization::{
    Arbiter,
    LogicalDestination,
    ChannelId,
    Multiplex,
};
use crate::radio::serdes::{
    Deserialize,
    Buf,
};
use crate::radio::Error;
use crate::motion;
use rtt_target::{
    rprint,
    rprintln
};
use crate::rplidar;
use crate::config;
use cfg_if::cfg_if;
use crate::motion::{TelemetryItem, Tachometer};
use crate::units::Watts;
use dw1000::mac::Address;
use crate::color;

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
    #[cfg(feature = "master")]
    pub motion_c: motion::ChannelC, // P in ctrl_link task (receive from ros)
    #[cfg(feature = "slave")]
    pub motion_p: motion::ChannelP, // C in motion task (send to mc)

    #[cfg(feature = "master")]
    pub motion_telemetry_p: motion::TelemetryChannelP, // C in ctrl_link task (send to ros)
    #[cfg(feature = "master")]
    pub local_motion_telemetry_c: motion::TelemetryLocalChannelC, // P in motion task (receive from mc)
    #[cfg(feature = "master")]
    pub telemetry_staging_area_tacho: TelemetryStagingAreaTacho,
    #[cfg(feature = "master")]
    pub telemetry_staging_area_power: TelemetryStagingAreaPower,

    #[cfg(feature = "slave")]
    pub motion_telemetry_c: motion::TelemetryChannelC, // P in ctrl_link task (receive from mc)
    ///// I/0/L/T/D/H/S/9 @71
    //pub odometry: Option<OdometryData>
    ///// I/0/_/T/D/H/S/8 @72
    //pub heartbeat: Option<Heartbeat>

    ///// I/P/L/A/D/H/U/10 @170
    #[cfg(feature = "br")]
    pub lidar_bbuffer_c: rplidar::LidarBBufferC,
    //pub lidar_queue_c: rplidar::LidarQueueC,
    #[cfg(feature = "master")]
    pub lidar_bbuffer_p: rplidar::LidarBBufferP,

    ///// R/P/F/A/D/H/S/9 @171
    // pub reqrep
}

#[derive(Default)]
pub struct TelemetryStagingAreaTacho {
    top_left: Option<Tachometer>,
    top_right: Option<Tachometer>,
    bottom_left: Option<Tachometer>,
    bottom_right: Option<Tachometer>,
}

#[derive(Default)]
pub struct TelemetryStagingAreaPower {
    top_left: Option<Watts>,
    top_right: Option<Watts>,
    bottom_left: Option<Watts>,
    bottom_right: Option<Watts>,
}

impl TelemetryStagingAreaPower {
    pub fn is_all_ready(&self) -> bool {
        self.top_left.is_some() &&
            self.top_right.is_some() &&
            self.bottom_left.is_some() &&
            self.bottom_right.is_some()
    }

    pub fn remove_all(&mut self) {
        *self = Self::default()
    }
}

impl TelemetryStagingAreaTacho {
    pub fn is_all_ready(&self) -> bool {
        self.top_left.is_some() &&
            self.top_right.is_some() &&
            self.bottom_left.is_some() &&
            self.bottom_right.is_some()
    }

    pub fn remove_all(&mut self) {
        *self = Self::default()
    }
}

impl Channels {
    #[cfg(feature = "master")]
    pub fn grab_local_telemetry(&mut self) {
        match self.local_motion_telemetry_c.dequeue() {
            Some(telem_item) => {
                rprint!(=>5, "{}local telem grabbed{}\n", color::GREEN, color::DEFAULT);
                Self::stage(
                    telem_item,
                    &mut self.telemetry_staging_area_tacho.top_left,
                    &mut self.telemetry_staging_area_power.top_left
                );
            },
            None => {}
        }
    }
    #[cfg(feature = "master")]
    fn stage(telem_item: TelemetryItem, tacho_stage: &mut Option<Tachometer>, power_stage: &mut Option<Watts>) {
        match telem_item {
            TelemetryItem::Tachometer(tacho) => {
                *tacho_stage = Some(tacho);
            },
            TelemetryItem::Power(power) => {
                *power_stage = Some(power);
            },
            _ => {}
        }
    }
}

impl Arbiter for Channels {
    type Error = Error;

    fn source_sync<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M)
    {
        cfg_if! {
            if #[cfg(feature = "master")] {
                match self.motion_c.dequeue() {
                    Some(rpm_array) => {
                        let _ = mux.mux(&rpm_array.top_right, LogicalDestination::Unicast(config::TR_UWB_ADDR), ChannelId::new(2));
                        let _ = mux.mux(&rpm_array.bottom_left, LogicalDestination::Unicast(config::BL_UWB_ADDR), ChannelId::new(2));
                        let _ = mux.mux(&rpm_array.bottom_right, LogicalDestination::Unicast(config::BR_UWB_ADDR), ChannelId::new(2));
                    },
                    None => {}
                }
            } else if #[cfg(feature = "slave")] {
                loop {
                    match self.motion_telemetry_c.dequeue() {
                        Some(telem_item) => {
                            rprintln!(=> 5, "{}source ok: {:?}{}\n", color::GREEN, telem_item, color::DEFAULT);
                            let _ = mux.mux(&telem_item, LogicalDestination::Implicit, ChannelId::new(3));
                        },
                        None => {
                            break;
                        }
                    }
                }
            }
        }
    }

    fn source_async<M: Multiplex<Error = Self::Error>>(&mut self, mux: &mut M) {
        cfg_if! {
            if #[cfg(feature = "br")] {
                let mut scans_written = 0;
                loop {
                    if scans_written > 11 {
                        break;
                    }
                    let rgr = self.lidar_bbuffer_c.read();
                    match rgr {
                        Some(rgr) => {
                            mux.mux(&&rgr[..], LogicalDestination::Implicit, ChannelId::new(11));
                            scans_written += 1;
                            rgr.release();
                        },
                        None => { break; }
                    }
                }
            }
        }
    }

    fn sink_sync(&mut self, source: Address,  channel: ChannelId, chunk: &[u8]) {
        // rprint!(=>1, "arb: CH{}:[", channel);
        // for b in chunk {
        //     rprint!(=>1, "{:02x} ", b);
        // }
        // rprintln!(=>1, "]\n");
        let mut buf = Buf::new(chunk);
        cfg_if! {
            if #[cfg(feature = "master")] {
                match motion::TelemetryItem::des(&mut buf) {
                    Ok(telem_item) => {
                    //     let r = self.motion_telemetry_p.enqueue(telem_item);
                    //     rprintln!(=>1, "telem enq: {:?}", r);\
                        if let Address::Short(pan_id, short_addr) = source {
                            if pan_id == config::PAN_ID {
                                if short_addr == config::TR_UWB_ADDR {
                                    rprintln!(=>5, "{}Stage TR {:?}{}\n", color::CYAN, telem_item, color::DEFAULT);
                                    Channels::stage(
                                        telem_item,
                                        &mut self.telemetry_staging_area_tacho.top_right,
                                        &mut self.telemetry_staging_area_power.top_right
                                    );
                                } else if short_addr == config::BL_UWB_ADDR {
                                    rprintln!(=>5, "{}Stage BL {:?}{}\n", color::CYAN, telem_item, color::DEFAULT);
                                    Channels::stage(
                                        telem_item,
                                        &mut self.telemetry_staging_area_tacho.bottom_left,
                                        &mut self.telemetry_staging_area_power.bottom_left
                                    );
                                } else if short_addr == config::BR_UWB_ADDR {
                                    rprintln!(=>5, "{}Stage BR {:?}{}\n", color::CYAN, telem_item, color::DEFAULT);
                                    Channels::stage(
                                        telem_item,
                                        &mut self.telemetry_staging_area_tacho.bottom_right,
                                        &mut self.telemetry_staging_area_power.bottom_right
                                    );
                                }
                            }
                            use crate::motion::{ TelemetryArray, PowerArray, TachoArray };
                            if self.telemetry_staging_area_tacho.is_all_ready() {
                                let tacho_array = TachoArray {
                                    top_left: self.telemetry_staging_area_tacho.top_left.unwrap(),
                                    top_right: self.telemetry_staging_area_tacho.top_right.unwrap(),
                                    bottom_left: self.telemetry_staging_area_tacho.bottom_left.unwrap(),
                                    bottom_right: self.telemetry_staging_area_tacho.bottom_right.unwrap()
                                };
                                let item = TelemetryArray::Tachometer(tacho_array);
                                let r = self.motion_telemetry_p.enqueue(item);
                                rprintln!(=>5, "{}tacho arr enq: {}{}", color::GREEN, r.is_ok(), color::DEFAULT);
                                self.telemetry_staging_area_tacho.remove_all();
                                rtic::pend(config::CHANNEL_EVENT_IRQ);
                            }
                            if self.telemetry_staging_area_power.is_all_ready() {
                                let power_array = PowerArray {
                                    top_left: self.telemetry_staging_area_power.top_left.unwrap(),
                                    top_right: self.telemetry_staging_area_power.top_right.unwrap(),
                                    bottom_left: self.telemetry_staging_area_power.bottom_left.unwrap(),
                                    bottom_right: self.telemetry_staging_area_power.bottom_right.unwrap()
                                };
                                let item = TelemetryArray::Power(power_array);
                                let r = self.motion_telemetry_p.enqueue(item);
                                rprintln!(=>5, "{}power arr enq: {}{}", color::GREEN, r.is_ok(), color::DEFAULT);
                                self.telemetry_staging_area_power.remove_all();
                                rtic::pend(config::CHANNEL_EVENT_IRQ);
                            }
                        }
                    },
                    Err(_) => {}
                }
            } else if #[cfg(feature = "slave")] {
                match motion::Rpm::des(&mut buf) {
                    Ok(rpm) => {
                        let r = self.motion_p.enqueue(rpm);
                        rprintln!(=>5, "{}Rpm enq:{} {:?}{}", color::GREEN, r.is_ok(), rpm, color::DEFAULT);
                        rtic::pend(config::CHANNEL_EVENT_IRQ);
                    },
                    Err(_) => {}
                }
            }
        }
    }

    fn sink_async(&mut self, source: Address, channel: ChannelId, chunk: &[u8]) {
        //rprintln!(=>1, "async: CH{}:[{}]\n", channel, chunk.len());
        // for b in &chunk[..10] {
        //     rprint!(=>1, "{:02x} ", b);
        // }
        //rprintln!(=>1, "]\n");
        cfg_if! {
            if #[cfg(feature = "master")] {
                if channel == ChannelId::new(11) && chunk[0] == 0xfd && chunk.len() == rplidar::FRAME_SIZE + 1 {
                    // let mut frame = [0u8; rplidar::FRAME_SIZE];
                    // frame.copy_from_slice(&chunk[1..]);
                    // let frame = rplidar::Frame(frame);
                    // let r = self.lidar_queue_p.enqueue(frame);
                    // rtic::pend(config::CHANNEL_EVENT_IRQ);
                    // rprintln!(=>1, "l.frame.enq: {}\n", r.is_ok());

                    let r = self.lidar_bbuffer_p.grant(rplidar::FRAME_SIZE);
                    match r {
                        Ok(mut wgr) => {
                            wgr.copy_from_slice(&chunk[1..]);
                            wgr.commit(rplidar::FRAME_SIZE);
                            rprintln!(=>15, "{}sink.frame.enq OK\n{}", color::GREEN, color::DEFAULT);
                            rtic::pend(config::CHANNEL_EVENT_IRQ);
                        },
                        Err(_) => {
                            rprintln!(=>15, "{}sink.frame.DROP\n{}", color::YELLOW, color::DEFAULT);
                        }
                    }
                }
            }
        }
    }
}