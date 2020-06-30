/// * busywait!(ms, rtic_cx, 100);
/// * busywait!(us, rtix_cx, 100);
/// * busywait!(ms_alt, clocks, 100);
macro_rules! busywait {
    (ms, $cx:ident, $amount:expr) => {
        cortex_m::asm::delay($cx.resources.clocks.sysclk().0 / 1_000 * $amount);
    };
    (ms_alt, $clocks:ident, $amount:expr) => {
        cortex_m::asm::delay($clocks.sysclk().0 / 1_000 * $amount);
    };
    (us, $cx:ident, $amount:expr) => {
        cortex_m::asm::delay($cx.resources.clocks.sysclk().0 / 1_000_000 * $amount);
    };
}

/// * ms2cycles!(rtic_cx, 100);
macro_rules! ms2cycles {
    ($cx:ident, $amount:expr) => {
        ($cx.resources.clocks.sysclk().0 / 1_000 * $amount).cycles()
    };
}

macro_rules! ms2cycles_raw {
    ($cx:ident, $amount:expr) => {
        $cx.resources.clocks.sysclk().0 / 1_000 * $amount
    };
}

macro_rules! cycles2ms {
    ($cx:ident, $amount:expr) => {
        (($amount as u64) * 1_000) / $cx.resources.clocks.sysclk().0 as u64
    };
}

macro_rules! cycles2us {
    ($cx:ident, $amount:expr) => {
        (($amount as u64) * 1_000_000) / $cx.resources.clocks.sysclk().0 as u64
    };
}