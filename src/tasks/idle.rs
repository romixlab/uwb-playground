use core::num::Wrapping;
use rtic::Mutex;

pub fn idle(mut cx: crate::idle::Context) -> ! {
    loop {
        cx.resources.idle_counter.lock(|counter| *counter += Wrapping(1u32));
        cortex_m::asm::delay(20_000_000);

        cfg_if::cfg_if! {
                if #[cfg(feature = "master")] {
                    rprintln!(=> 8, "\x1b[2J\x1b[0m");
                    let (tr_gts_answers, bl_gts_answers, br_gts_answers) = cx.resources.stat.lock(|s|
                        (
                            s.tr_gts_answers,
                            s.bl_gts_answers,
                            s.br_gts_answers,
                        )
                    );
                    rprintln!(=> 8, "TR: {}\n", tr_gts_answers);
                    rprintln!(=> 8, "BL: {}\n", bl_gts_answers);
                    rprintln!(=> 8, "BR: {}\n", br_gts_answers);

                    let (tacho_tl, tacho_tr, tacho_bl, tacho_br) = cx.resources.mecanum_wheels.lock(|wheels| {
                        (
                            wheels.top_left.tacho.0 - wheels.top_left.tacho_shift,
                            wheels.top_right.tacho.0 - wheels.top_right.tacho_shift,
                            wheels.bottom_left.tacho.0 - wheels.bottom_left.tacho_shift,
                            wheels.bottom_right.tacho.0 - wheels.bottom_right.tacho_shift
                        )
                    });
                    rprintln!(=> 8, "TL: {}\tTR: {}", tacho_tl, tacho_tr);
                    rprintln!(=> 8, "BL: {}\tBR: {}", tacho_bl, tacho_br)
                }
            }


        //atomic::compiler_fence(Ordering::SeqCst);
    }
}
