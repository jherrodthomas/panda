#pragma once

// declarations are in board/drivers/drivers.h, included before this header

// ******************** EPS fault siren ********************
//
// LionDriver TSR-516(b) / SWSR-516 for the EPS fault (TSR-110): the panda sounds its own siren, without the SoC, when
// the car's EPS has reported a fault for longer than EPS_FAULT_SIREN_PERSIST_TICKS while authority is granted. The
// envelope already refuses lateral actuation from the first fault frame (safety_eps_fault); the siren is for faults that
// persist. Short temporary codes during a message drop-out are common and a siren on each would teach the driver to
// ignore it (decision 2026-10-12: persistent faults only, 1 s).
//
// Once triggered, the siren sounds for at least EPS_FAULT_SIREN_HOLD_TICKS (as long as the 3 s siren on heartbeat
// loss), and is re-triggered while the fault persists. Runs in the 100 Hz safety task; the 8 Hz tick drives the
// speaker, so the siren starts within 1 s + 10 ms + 125 ms of the fault onset.

#define EPS_FAULT_SIREN_PERSIST_TICKS 100U  // 1 s at SAFETY_TASK_FREQ (100 Hz)
#define EPS_FAULT_SIREN_HOLD_TICKS 300U     // 3 s

static uint32_t eps_fault_persist_ticks = 0U;
static uint32_t eps_fault_siren_ticks = 0U;

// called on every safety mode change: a new mode starts without a pending siren
void eps_fault_siren_reset(void) {
  eps_fault_persist_ticks = 0U;
  eps_fault_siren_ticks = 0U;
}

// called from the 100 Hz safety task; returns true while the siren shall sound
bool eps_fault_siren_tick(bool eps_fault, bool engaged) {
  if (eps_fault && engaged) {
    if (eps_fault_persist_ticks < EPS_FAULT_SIREN_PERSIST_TICKS) {
      eps_fault_persist_ticks += 1U;
    }
  } else {
    eps_fault_persist_ticks = 0U;
  }

  if (eps_fault_persist_ticks >= EPS_FAULT_SIREN_PERSIST_TICKS) {
    eps_fault_siren_ticks = EPS_FAULT_SIREN_HOLD_TICKS;
  } else if (eps_fault_siren_ticks > 0U) {
    eps_fault_siren_ticks -= 1U;
  } else {
    // siren off
  }
  return eps_fault_siren_ticks > 0U;
}
