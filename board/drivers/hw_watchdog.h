#pragma once

// ******************** hardware watchdog (IWDG) ********************
//
// LionDriver: independent hardware watchdog for the safety MCU (TSR-502, GAP-07).
//
// The IWDG runs from the LSI oscillator, independent of the system clock and of the
// interrupt system, so it also catches a hang with interrupts disabled. Once started it
// cannot be stopped except by a reset. A reset floats all GPIOs, which de-energises the
// harness relay and restores the stock camera path (TSR-506), and restarts in SILENT.
//
// It is started the first time a safety mode that drives the intercept relay is set,
// because only then can a hang leave the relay energised (GAP-43). It is fed from the 8 Hz
// tick, and only when the main loop has run since the previous tick, so a stall of the
// tick interrupt or of the main loop both end in a reset.
//
// Timeout = 4 * 2^PR * (RLR + 1) / f_LSI. With PR = 0 (/4) and RLR = 1399 the nominal
// timeout is 175 ms at f_LSI = 32 kHz. It must exceed the 125 ms feed period (8 Hz tick)
// with margin for LSI tolerance, which must be confirmed from the STM32H725 datasheet.
//
// Known deviations from LionDriver TSR-501 (v0.3), recorded for review:
//  - TSR-501 requires a reset within 0.1 s of a stall (SG-07 forwarding budget). 175 ms
//    meets the SG-01 budget (0.5 s) only. Reaching 0.1 s needs a feed source faster than
//    the 8 Hz tick, i.e. the >= 50 Hz periodic safety task of TSR-401/TSR-519.
//  - TSR-501 requires the watchdog to start before the application runs. Here it starts
//    when the relay is first driven: in SILENT the relay is never energised, so a hang
//    there is already safe, and an idle device keeps its power-save and stop modes.

#define HW_WATCHDOG_KEY_START 0xCCCCU
#define HW_WATCHDOG_KEY_UNLOCK 0x5555U
#define HW_WATCHDOG_KEY_FEED 0xAAAAU
#define HW_WATCHDOG_PRESCALER 0U
#define HW_WATCHDOG_RELOAD 1399U
// bounded wait for the prescaler/reload update (takes up to 5 LSI cycles, about 160 us)
#define HW_WATCHDOG_UPDATE_WAIT 200000U

static bool hw_watchdog_running = false;
static volatile bool hw_watchdog_main_loop_ran = false;

static void hw_watchdog_start(void) {
  if (!hw_watchdog_running) {
    IND_WDG->KR = HW_WATCHDOG_KEY_START;
    IND_WDG->KR = HW_WATCHDOG_KEY_UNLOCK;
    IND_WDG->PR = HW_WATCHDOG_PRESCALER;
    IND_WDG->RLR = HW_WATCHDOG_RELOAD;
    for (uint32_t i = 0U; (i < HW_WATCHDOG_UPDATE_WAIT) && ((IND_WDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U); i++) {}
    IND_WDG->KR = HW_WATCHDOG_KEY_FEED;
    hw_watchdog_running = true;
  }
}

// called on every main-loop iteration
static void hw_watchdog_main_loop_alive(void) {
  hw_watchdog_main_loop_ran = true;
}

// called from the 8 Hz tick
static void hw_watchdog_tick(void) {
  if (hw_watchdog_running && hw_watchdog_main_loop_ran) {
    hw_watchdog_main_loop_ran = false;
    IND_WDG->KR = HW_WATCHDOG_KEY_FEED;
  }
}
