#pragma once

// Independent watchdog IWDG1 (LionDriver TSR-501, SWSR-501, HWSR-501).
//
// IWDG1 is clocked by the LSI oscillator, independent of the core clock tree. Once started it
// cannot be stopped by software or by the SoC; only a reset stops it. It is started when the
// firmware first enters a mode that drives the intercept relay (see set_safety_mode), and fed
// from the 8 Hz tick only when the main loop has made progress since the previous tick, so a
// hang in an interrupt handler, in the main loop or with interrupts disabled ends in a reset.
//
// Timeout: LSI (32 kHz nominal) / 8 = 4 kHz, reload 1200 -> 300 ms nominal. This must exceed the
// 125 ms tick period with margin. The 0.1 s target of TSR-501 needs a faster periodic safety task
// (SWSR-401a); until then the timeout stays at 300 ms.

#define IWDG_KEY_START   0xCCCCU
#define IWDG_KEY_ACCESS  0x5555U
#define IWDG_KEY_RELOAD  0xAAAAU
#define IWDG_PRESCALER_DIV8 1U
#define IWDG_RELOAD_300MS 1200U

// reported in the health flags after a reset caused by IWDG1; kept out of board/health.h, whose hash is the
// health packet version that the host checks
#define HEALTH_FLAG_WATCHDOG_RESET ((uint16_t)1U << 9)

static bool iwdg_running = false;

static void iwdg_start(void) {
  if (!iwdg_running) {
    IND_WDG->KR = IWDG_KEY_START;   // starts the counter (and the LSI)
    IND_WDG->KR = IWDG_KEY_ACCESS;  // unlock PR and RLR
    IND_WDG->PR = IWDG_PRESCALER_DIV8;
    IND_WDG->RLR = IWDG_RELOAD_300MS;
    while ((IND_WDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U) {
      // wait for the update to reach the LSI domain; if this never completes the watchdog resets the MCU
    }
    IND_WDG->KR = IWDG_KEY_RELOAD;
    iwdg_running = true;
  }
}

static void iwdg_feed(void) {
  if (iwdg_running) {
    IND_WDG->KR = IWDG_KEY_RELOAD;
  }
}
