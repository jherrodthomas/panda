#pragma once

// declarations are in board/drivers/drivers.h, included before this header

// ******************** heartbeat monitor ********************
//
// LionDriver TSR-407 / SWSR-407: in a car safety mode, authority is revoked when the SoC heartbeat (0xf3)
// has been missing for more than 0.3 s. The host sends the heartbeat at 10 Hz, so this tolerates two
// missed heartbeats. The check runs in the 100 Hz safety task, so detection is at most 0.3 s plus one
// task period. Re-engagement then needs a new PCM rising edge.
//
// The existing 1 Hz heartbeat counter in the tick still enters SILENT after the longer timeout
// (TSR-510); this monitor only adds the fast revocation of authority.

#define HEARTBEAT_AUTHORITY_TIMEOUT_TICKS 30U  // 0.3 s at SAFETY_TASK_FREQ (100 Hz)

static uint32_t heartbeat_age_ticks = 0U;

// called on every heartbeat received from the SoC
void heartbeat_monitor_reset(void) {
  heartbeat_age_ticks = 0U;
}

// called from the 100 Hz safety task; returns true while authority must be revoked
bool heartbeat_monitor_tick(void) {
  if (heartbeat_age_ticks < HEARTBEAT_AUTHORITY_TIMEOUT_TICKS) {
    heartbeat_age_ticks += 1U;
  }
  return heartbeat_age_ticks >= HEARTBEAT_AUTHORITY_TIMEOUT_TICKS;
}
