#pragma once

// ******************** fault injection (LionDriver HIL bench) ********************
//
// Hooks that make the firmware hang on request, so the HIL bench can show that the
// hardware watchdog resets the MCU and releases the relay (LionDriver WP-V-05, TSR-501).
// Compiled only into debug builds made with FAULT_INJECTION=1 (see SConscript); a release
// image never contains them. Requested over the 0xa9 control command.

#ifndef ALLOW_DEBUG
#error "fault injection is only allowed in debug builds"
#endif

#define FAULT_INJECTION_NONE 0U
#define FAULT_INJECTION_HANG_MAIN_LOOP 1U
#define FAULT_INJECTION_HANG_TICK 2U
#define FAULT_INJECTION_HANG_IRQ_DISABLED 3U
#define FAULT_INJECTION_HANG_SAFETY_TASK 4U
#define FAULT_INJECTION_EARLY_FEED 5U

static volatile uint32_t fault_injection_mode = FAULT_INJECTION_NONE;

static void fault_injection_request(uint32_t mode) {
  fault_injection_mode = mode;
}

// called on every main-loop iteration
static void fault_injection_main_loop(void) {
  if (fault_injection_mode == FAULT_INJECTION_HANG_IRQ_DISABLED) {
    disable_interrupts();
  }
  if ((fault_injection_mode == FAULT_INJECTION_HANG_MAIN_LOOP) || (fault_injection_mode == FAULT_INJECTION_HANG_IRQ_DISABLED)) {
    while (true) {}
  }
}

// called from the 8 Hz tick
static void fault_injection_tick(void) {
  if (fault_injection_mode == FAULT_INJECTION_HANG_TICK) {
    while (true) {}
  }
}

// called from the 100 Hz safety task, after its watchdog feed
static void fault_injection_safety_task(void) {
  if (fault_injection_mode == FAULT_INJECTION_HANG_SAFETY_TASK) {
    while (true) {}
  }
  if (fault_injection_mode == FAULT_INJECTION_EARLY_FEED) {
    // two feeds back to back: the second falls in the closed part of the window
    // (HWSR-501b), as a safety task running much too fast would
    IND_WDG->KR = HW_WATCHDOG_KEY_FEED;
    IND_WDG->KR = HW_WATCHDOG_KEY_FEED;
  }
}
