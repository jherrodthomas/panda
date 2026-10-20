#pragma once

// declarations are in board/drivers/drivers.h, included before this header

// ******************** heartbeat-loss siren ********************
//
// LionDriver TSR-516(b) / SWSR-516 for heartbeat loss (TSR-407, SG-02 and SG-06 warning within 1 s): when the 0.3 s
// heartbeat monitor revokes authority that was granted, the panda sounds its own siren at once, without the SoC.
// Before this, the only siren on heartbeat loss came from the 1 Hz tick when it entered SILENT, 4-5 s after the last
// heartbeat (WP-S-04 T-07). The 1 Hz path is unchanged and still sounds when it fires.
//
// LionDriver TSR-516(b) for missing SoC command frames (TSR-408, DQ-404 A): the same siren sounds when the TX-arrival
// check in the safety mode revokes granted authority. A stalled SoC stops both its command frames and its heartbeat;
// the arrival check (50 ms for STEERING_LKA) revokes first, inside safety_tick(), so authority is already gone when the
// heartbeat monitor fires and the heartbeat trigger alone stayed silent (WP-S-04 T-34). Whichever revokes first sounds
// the siren, once. A disengagement by the driver (brake, cruise off) and an RX fault do not set the arrival flag.
//
// The siren sounds only when authority was granted at the moment of revocation: a heartbeat loss while disengaged
// takes nothing from the driver and needs no warning. It sounds for HEARTBEAT_SIREN_HOLD_TICKS, as long as the
// existing siren on heartbeat loss. Runs in the 100 Hz safety task; the 8 Hz tick drives the speaker on every other
// tick, so the siren starts within 0.3 s + 10 ms + 2 x 125 ms of the last heartbeat.

#define HEARTBEAT_SIREN_HOLD_TICKS 300U  // 3 s at SAFETY_TASK_FREQ (100 Hz)

static uint32_t heartbeat_siren_ticks = 0U;

// called on every safety mode change: a new mode starts without a pending siren
void heartbeat_siren_reset(void) {
  heartbeat_siren_ticks = 0U;
}

// called from the 100 Hz safety task after safety_tick() and before the heartbeat revocation; start_siren is true
// in the task in which the arrival check revoked granted authority (safety_tx_arrival_revoked) or the engaged
// mismatch asks for the siren (engaged_mismatch.h, TSR-516(c)); returns true while the siren shall sound
bool heartbeat_siren_tick(bool heartbeat_revoke, bool authority_granted, bool start_siren) {
  if ((heartbeat_revoke && authority_granted) || start_siren) {
    heartbeat_siren_ticks = HEARTBEAT_SIREN_HOLD_TICKS;
  } else if (heartbeat_siren_ticks > 0U) {
    heartbeat_siren_ticks -= 1U;
  } else {
    // siren off
  }
  return heartbeat_siren_ticks > 0U;
}
