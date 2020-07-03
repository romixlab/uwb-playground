use rtt_target::{rprint, rprintln};

use rtic::cyccnt::U32Ext;

#[derive(Debug)]
pub enum LiftControlCommand {
    Continue,
    LeftUp,
    LeftDown,
    RightUp,
    RightDown,
    AllUp,
    AllDown,
    //AllUpLong,
    //AllDownLong,
    //Stop
}

pub fn lift_control(
    cx: crate::lift_control::Context,
    command: LiftControlCommand,
    tokens: &mut u16,
    current_symbol: &mut u8
) {
    use embedded_hal::serial::Write;

    const RESEND_TIME_MS: u32 = 10;
    const ABORT_TIME_MS: u32 = 250;
    const ALL_UP_LONG_TIME: u32 = 25_000;
    const ALL_DOWN_LONG_TIME: u32 = 25_000;

    const ABORT_TOKENS: u16 = (ABORT_TIME_MS / RESEND_TIME_MS) as u16;
    const ALL_UP_LONG_TOKENS: u16 = (ALL_UP_LONG_TIME / RESEND_TIME_MS) as u16;
    const ALL_DOWN_LONG_TOKENS: u16 = (ALL_DOWN_LONG_TIME / RESEND_TIME_MS) as u16;

    rprintln!(=> 6, "{:?} {}", command, *tokens);
    use LiftControlCommand::*;

    let prev_tokens = *tokens;
    match command {
        Continue => {
            if *tokens != 0 {
                cx.resources.lift_serial.write(*current_symbol).ok();
                cx.schedule.lift_control(cx.scheduled + ms2cycles!(cx, 10), Continue).ok();
                *tokens -= 1;
            }
        },
        LeftUp => {      *current_symbol = 'a' as u8; *tokens = ABORT_TOKENS; },
        LeftDown => {    *current_symbol = 'q' as u8; *tokens = ABORT_TOKENS; },
        RightUp => {     *current_symbol = 'd' as u8; *tokens = ABORT_TOKENS; },
        RightDown => {   *current_symbol = 'e' as u8; *tokens = ABORT_TOKENS; },
        AllUp => {       *current_symbol = 's' as u8; *tokens = ABORT_TOKENS; },
        AllDown => {     *current_symbol = 'w' as u8; *tokens = ABORT_TOKENS; },
        AllUpLong => {   *current_symbol = 's' as u8; *tokens = ALL_UP_LONG_TOKENS; },
        AllDownLong => { *current_symbol = 'w' as u8; *tokens = ALL_DOWN_LONG_TOKENS; },
        Stop => {
            *tokens = 0;
        },
    }
    if prev_tokens == 0 {
        cx.resources.lift_serial.write(*current_symbol).ok();
        cx.schedule.lift_control(cx.scheduled + ms2cycles!(cx, 10), Continue).ok();
    }
}
