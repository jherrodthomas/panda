#pragma once

// declarations are in board/drivers/drivers.h, included before this header

// ******************** engaged mismatch (reference build) ********************
//
// LionDriver TSR-307(a): while authority is granted, a SoC heartbeat that reports "not engaged" revokes authority
// within ENGAGED_MISMATCH_REVOKE_TICKS (0.3 s), in the 100 Hz safety task. Before, only the 1 Hz check in the tick
// handler revoked, after 3 mismatched ticks (2-3 s), with no siren. That case occurs when selfdrived stops while
// pandad keeps sending the heartbeat (WP-S-04 §5, sim result engaged-mismatch.md): card keeps sending STEERING_LKA
// without a torque request, so the TX-arrival check (TSR-408) does not revoke.
//
// LionDriver TSR-516(b)/(c): the siren sounds when PCM cruise is still active ENGAGED_MISMATCH_SIREN_TICKS (0.5 s)
// after the heartbeat first reported "not engaged" while authority was granted, that is when openpilot left
// engagement and the cruise cancel it sends had no effect. An ordinary disengagement stays silent: a brake press or
// a PCM cruise drop is seen by the envelope itself and revokes before the heartbeat changes (TSR-301, TSR-302), and
// every disengagement openpilot makes on its own sends a cruise cancel, so the PCM drops cruise and ends the episode
// before 0.5 s (cancel lag to be confirmed from reference logs, IA OI-1). The episode starts only while authority is
// granted, and ends when the heartbeat reports "engaged" again or PCM cruise drops.

#define ENGAGED_MISMATCH_REVOKE_TICKS 30U  // 0.3 s at SAFETY_TASK_FREQ (100 Hz), TSR-307(a)
#define ENGAGED_MISMATCH_SIREN_TICKS 50U   // 0.5 s, TSR-516(c)

static bool engaged_mismatch_active = false;
static uint32_t engaged_mismatch_ticks = 0U;

// called on every safety mode change: a new mode starts without a pending mismatch
void engaged_mismatch_reset(void) {
  engaged_mismatch_active = false;
  engaged_mismatch_ticks = 0U;
}

// called from the 100 Hz safety task; returns true in the task in which authority shall be revoked, and sets
// *siren in the task in which the siren shall start
bool engaged_mismatch_tick(bool authority_granted, bool heartbeat_engaged_flag, bool cruise_engaged, bool *siren) {
  *siren = false;
  if (heartbeat_engaged_flag || (!authority_granted && !engaged_mismatch_active)) {
    engaged_mismatch_reset();
  } else if (!engaged_mismatch_active) {
    // authority granted and the heartbeat reports "not engaged": the episode starts
    engaged_mismatch_active = true;
    engaged_mismatch_ticks = 1U;
  } else if (!cruise_engaged && !authority_granted) {
    // PCM cruise dropped after the revocation: the cancel took effect, the episode ends without a siren
    engaged_mismatch_reset();
  } else if (engaged_mismatch_ticks <= ENGAGED_MISMATCH_SIREN_TICKS) {
    engaged_mismatch_ticks += 1U;
  } else {
    // past the siren time: hold, so the siren starts once, until the heartbeat or the cruise state ends the episode
  }

  // the episode outlives the revocation only while PCM cruise stays active, so reaching the siren time means the
  // cancel had no effect (TSR-516(c))
  if (engaged_mismatch_active && (engaged_mismatch_ticks == ENGAGED_MISMATCH_SIREN_TICKS)) {
    *siren = true;
  }
  return engaged_mismatch_active && authority_granted && (engaged_mismatch_ticks >= ENGAGED_MISMATCH_REVOKE_TICKS);
}
