#pragma once

// declarations are in board/drivers/drivers.h, included before this header

// ******************** EPS fault siren ********************
//
// LionDriver TSR-516(b) / SWSR-516 for the EPS fault (TSR-110): the panda sounds its own siren, without the SoC, when
// the car's EPS has reported a fault for longer than EPS_FAULT_SIREN_PERSIST_TICKS while authority is granted. The
// envelope already refuses lateral actuation from the first fault frame (safety_eps_fault); the siren is for faults that
// persist. Short temporary codes during a message drop-out are common and a siren on each would teach the driver to
// ignore it (decision 2026-10-12: persistent faults only). The persistence is 0.7 s (LionDriver decision DQ-048,
// 2026-10-14; WP-S-04 OI-8): with 1 s the siren came up to 1.26 s after the fault onset, past the SG-02 1 s warning
// budget (WP-S-04 §3.7); with 0.7 s it comes at most 0.7 s + 10 ms + 2 x 125 ms = 0.96 s after the onset.
//
// A fault counts once it has been seen while authority was granted, and keeps counting after authority is revoked
// (LionDriver decision DQ-367): since SWSR-408 counts only accepted command frames, a host that keeps requesting torque
// through the fault loses authority 50 ms after the envelope starts refusing its frames, and the siren must still sound.
//
// Once triggered, the siren sounds for at least EPS_FAULT_SIREN_HOLD_TICKS (as long as the 3 s siren on heartbeat
// loss), and is re-triggered while the fault persists. Runs in the 100 Hz safety task; the 8 Hz tick drives the
// speaker, sounding only on every other tick, so the siren starts within 0.7 s + 10 ms + 250 ms of the fault onset.

#define EPS_FAULT_SIREN_PERSIST_TICKS 70U   // 0.7 s at SAFETY_TASK_FREQ (100 Hz), DQ-048
#define EPS_FAULT_SIREN_HOLD_TICKS 300U     // 3 s

static uint32_t eps_fault_persist_ticks = 0U;
static uint32_t eps_fault_siren_ticks = 0U;
static bool eps_fault_seen_engaged = false;

// called on every safety mode change: a new mode starts without a pending siren
void eps_fault_siren_reset(void) {
  eps_fault_persist_ticks = 0U;
  eps_fault_siren_ticks = 0U;
  eps_fault_seen_engaged = false;
}

// called from the 100 Hz safety task; returns true while the siren shall sound
bool eps_fault_siren_tick(bool eps_fault, bool engaged) {
  if (!eps_fault) {
    eps_fault_seen_engaged = false;
  } else if (engaged) {
    eps_fault_seen_engaged = true;
  } else {
    // fault without authority: keeps the state it had
  }

  if (eps_fault && eps_fault_seen_engaged) {
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
