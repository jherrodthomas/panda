#pragma once

// declarations are in board/drivers/drivers.h, included before this header

// ******************** hardware watchdog (IWDG) ********************
//
// LionDriver: independent hardware watchdog for the safety MCU (TSR-502, GAP-07).
//
// The IWDG runs from the LSI oscillator, independent of the system clock and of the
// interrupt system, so it also catches a hang with interrupts disabled. Once started it
// cannot be stopped except by a reset. A reset floats all GPIOs, which de-energises the
// harness relay and restores the stock camera path (TSR-506), and restarts in SILENT.
//
// It is started by set_intercept_relay() the first time the relay is driven, by any path
// (a relay-driving safety mode or the 0xc5 debug command), because only then can a hang
// leave the relay energised (GAP-43). It is fed from the 100 Hz safety task, and only
// when the main loop has run since the previous feed, so a stall of the safety task
// interrupt or of the main loop both end in a reset.
//
// Timeout = 4 * 2^PR * (RLR + 1) / f_LSI. With PR = 0 (/4) and RLR = 719 the nominal
// timeout is 90 ms at f_LSI = 32 kHz, within the 0.1 s of LionDriver TSR-501 (SG-07
// forwarding budget). It must exceed the 10 ms feed period (100 Hz safety task) with
// margin for LSI tolerance, which must be confirmed from the STM32H725 datasheet.
//
// Known deviation from LionDriver TSR-501, recorded for review:
//  - The watchdog runs in normal (not window) mode; HWSR-501b asks for window mode
//    (deviation D-3, accepted as interim on 2026-10-12).

#define HW_WATCHDOG_KEY_START 0xCCCCU
#define HW_WATCHDOG_KEY_UNLOCK 0x5555U
#define HW_WATCHDOG_KEY_FEED 0xAAAAU
#define HW_WATCHDOG_PRESCALER 0U
#define HW_WATCHDOG_RELOAD 719U
// bounded wait for the prescaler/reload update (takes up to 5 LSI cycles, about 160 us)
#define HW_WATCHDOG_UPDATE_WAIT 200000U

static bool hw_watchdog_running = false;
static volatile bool hw_watchdog_main_loop_ran = false;

void hw_watchdog_start(void) {
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
void hw_watchdog_main_loop_alive(void) {
  hw_watchdog_main_loop_ran = true;
}

// called from the 100 Hz safety task
void hw_watchdog_tick(void) {
  if (hw_watchdog_running && hw_watchdog_main_loop_ran) {
    hw_watchdog_main_loop_ran = false;
    IND_WDG->KR = HW_WATCHDOG_KEY_FEED;
  }
}
