use crate::config;
use crate::dump_delay;
use vl53l1;

use stm32g4xx_hal::prelude::_embedded_hal_blocking_i2c_Write;
use stm32g4xx_hal::prelude::_embedded_hal_blocking_delay_DelayUs;
use vl53l1::{RangingMeasurementData, RangeStatus};


struct DevMulti{
    device: vl53l1::Device,
    i2c_multi_chan: u8,
}

pub struct Vl53l1Multi{
    i2c: config::I2cPortType,
    delay: dump_delay::DumpDelay,
    devs: [DevMulti; 4],
    tcaaddr: u8
}

impl Vl53l1Multi{
    pub fn new(i2c: config::I2cPortType, delay: dump_delay::DumpDelay, i2c_multi_chan_map: [u8; 4], tcaaddr: u8) -> Vl53l1Multi{
        let mut devs: [DevMulti; 4] = [ DevMulti { device: Default::default(), i2c_multi_chan: 0 },
                                        DevMulti { device: Default::default(), i2c_multi_chan: 0 },
                                        DevMulti { device: Default::default(), i2c_multi_chan: 0 },
                                        DevMulti { device: Default::default(), i2c_multi_chan: 0 }];
        for i in 0..4{
            devs[i].i2c_multi_chan = i2c_multi_chan_map[i];
        };

        Vl53l1Multi{
            i2c,
            delay,
            devs,
            tcaaddr
        }
    }
    pub fn init_devices(&mut self){
        for i in 0..4{
            self.i2c.write(self.tcaaddr, &[1 <<  self.devs[i].i2c_multi_chan]);
            self.delay.delay_us(100u32);

            vl53l1::software_reset(&mut self.devs[i].device, &mut self.i2c, &mut self.delay);

            vl53l1::data_init(&mut self.devs[i].device, &mut self.i2c);

            vl53l1::static_init(&mut self.devs[i].device);

            vl53l1::set_distance_mode(&mut self.devs[i].device, vl53l1::DistanceMode::Short);

            let roi = vl53l1::UserRoi {
                bot_right_x: 15,
                bot_right_y: 0,
                top_left_x: 0,
                top_left_y: 15,
            };
            /*let roi = vl53l1::UserRoi {
                bot_right_x: 7,
                bot_right_y: 5,
                top_left_x: 5,
                top_left_y: 7,
            };*/
            vl53l1::set_user_roi(&mut self.devs[i].device, roi);

            vl53l1::set_measurement_timing_budget_micro_seconds(&mut self.devs[i].device, 20_000);
            vl53l1::set_inter_measurement_period_milli_seconds(&mut self.devs[i].device, 20);

            vl53l1::start_measurement(&mut self.devs[i].device, &mut self.i2c);
        }
    }

    pub fn read_all(&mut self) -> [u16; 4]{
        let mut mes:[u16; 4] = [0; 4];
        for i in 0..4 {
            self.i2c.write(self.tcaaddr, &[1 << self.devs[i].i2c_multi_chan]);
            self.delay.delay_us(100u32);
            vl53l1::wait_measurement_data_ready(&mut self.devs[i].device, &mut self.i2c, &mut self.delay);
        }
        for i in 0..4{
            self.i2c.write(self.tcaaddr, &[1 << self.devs[i].i2c_multi_chan]);
            self.delay.delay_us(100u32);
           // vl53l1::wait_measurement_data_ready(&mut self.devs[i].device, &mut self.i2c, &mut self.delay);
            let rmd =
                match vl53l1::get_ranging_measurement_data(&mut self.devs[i].device, &mut self.i2c)
                {
                    Ok(x) => {
                        match x.range_status{
                            RangeStatus::RANGE_VALID => {x.range_milli_meter}
                            RangeStatus::SIGMA_FAIL => {0}
                            RangeStatus::SIGNAL_FAIL => {0}
                            RangeStatus::RANGE_VALID_MIN_RANGE_CLIPPED => {0}
                            RangeStatus::OUTOFBOUNDS_FAIL => {0}
                            RangeStatus::HARDWARE_FAIL => {0}
                            RangeStatus::RANGE_VALID_NO_WRAP_CHECK_FAIL => {0}
                            RangeStatus::WRAP_TARGET_FAIL => {0}
                            RangeStatus::PROCESSING_FAIL => {0}
                            RangeStatus::XTALK_SIGNAL_FAIL => {0}
                            RangeStatus::SYNCRONISATION_INT => {0}
                            RangeStatus::RANGE_VALID_MERGED_PULSE => {0}
                            RangeStatus::TARGET_PRESENT_LACK_OF_SIGNAL => {0}
                            RangeStatus::MIN_RANGE_FAIL => {0}
                            RangeStatus::RANGE_INVALID => {0}
                            RangeStatus::NONE => {0}
                            _ => {0}
                        }
                        // x.range_milli_meter
                    }
                    Err(_) => {0}
                };
            mes[i] = rmd as u16;
            //vl53l1::clear_interrupt_and_start_measurement(&mut self.devs[i].device, &mut self.i2c, &mut self.delay);
        }
        for i in 0..4{
            self.i2c.write(self.tcaaddr, &[1 << self.devs[i].i2c_multi_chan]);
            self.delay.delay_us(100u32);
            vl53l1::clear_interrupt_and_start_measurement(&mut self.devs[i].device, &mut self.i2c, &mut self.delay);
        }

        mes
    }
}