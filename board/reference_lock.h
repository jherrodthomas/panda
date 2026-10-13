#pragma once

// declarations (ControlPacket_t, current_safety_mode, SAFETY_* modes, print, puth) are included before
// this header

bool is_car_safety_mode(uint16_t mode);

// Health flags that identify the build (TSR-514, TSR-619): the host checks HEALTH_FLAG_REFERENCE_BUILD before it
// configures a car safety mode, so firmware without the reference lock cannot be used for engagement.
static inline uint16_t reference_build_health_flags(void) {
#ifdef LIONDRIVER_REFERENCE
  return HEALTH_FLAG_REFERENCE_BUILD;
#else
  return 0U;
#endif
}

// LionDriver reference build (TSR-512/513, SWSR-512/512a/513): the 2020 Corolla LE TSS2 reference
// configuration. The host may only request SILENT, NOOUTPUT, ELM327 and TOYOTA with the reference
// parameter; once TOYOTA is active only NOOUTPUT or SILENT. The alternative experience stays at its
// reset value (0). Requests that change actuation, relay, CAN or comms state are refused in a car mode.
// Internal safety mode changes (heartbeat loss, faults) do not pass through here and are not affected.
//
// Diagnostic mode (LD-DES-001 DR-05, DFA DFI-08): ELM327 is used for fingerprinting before the car mode is
// set. Once TOYOTA has been accepted with the ignition on, ELM327 is refused until the ignition goes off, so a
// faulty host cannot downgrade to NOOUTPUT and send diagnostic traffic to the EPS or ADAS ECUs while driving.
// The panda has no vehicle speed outside a car mode, so the "stationary" alternative of DR-05 is not used.
#ifdef LIONDRIVER_REFERENCE
#define LIONDRIVER_REFERENCE_TOYOTA_PARAM 73U  // EPS factor 73, no flags
// refused requests since reset, reported in the health packet (saturating)
static uint16_t reference_requests_rejected = 0U;
// ignition state from the 1 Hz tick, and whether TOYOTA was accepted since the ignition was last seen off
static bool reference_ignition = false;
static bool reference_car_mode_this_ignition = false;
#endif

// Called at 1 Hz from main() with the ignition state (line or CAN). No effect outside the reference build.
static inline void reference_lock_ignition_update(bool ignition) {
#ifdef LIONDRIVER_REFERENCE
  reference_ignition = ignition;
  if (!ignition) {
    reference_car_mode_this_ignition = false;
  }
#else
  (void)ignition;
#endif
}

#ifdef LIONDRIVER_REFERENCE

static bool reference_request_allowed(const ControlPacket_t *req) {
  bool allowed = true;
  bool car_mode = is_car_safety_mode(current_safety_mode);
  switch (req->request) {
    case 0xdcU:
      if (current_safety_mode == SAFETY_TOYOTA) {
        allowed = (req->param1 == SAFETY_NOOUTPUT) || (req->param1 == SAFETY_SILENT);
      } else {
        bool elm327_allowed = !(reference_ignition && reference_car_mode_this_ignition);
        allowed = (req->param1 == SAFETY_SILENT) || (req->param1 == SAFETY_NOOUTPUT) ||
                  ((req->param1 == SAFETY_ELM327) && elm327_allowed) ||
                  ((req->param1 == SAFETY_TOYOTA) && (req->param2 == LIONDRIVER_REFERENCE_TOYOTA_PARAM));
        // set whatever the cached ignition says (it lags by up to 1 s); the 1 Hz tick clears it while the ignition is off
        if (allowed && (req->param1 == SAFETY_TOYOTA)) {
          reference_car_mode_this_ignition = true;
        }
      }
      break;
    case 0xdfU:
      allowed = false;
      break;
    case 0xd1U:
      // softloader entry (param 1, SWSR-511a); bootloader entry (param 0) is already debug-only
      allowed = !(car_mode && (req->param1 == 1U));
      break;
    case 0xc5U:
    case 0xd8U:
    case 0xdbU:
    case 0xdeU:
    case 0xe5U:
    case 0xe6U:
    case 0xe7U:
    case 0xe8U:
    case 0xf1U:
    case 0xf9U:
    case 0xfcU:
      allowed = !car_mode;
      break;
    default:
      break;
  }
  if (!allowed) {
    if (reference_requests_rejected < 0xFFFFU) {
      reference_requests_rejected += 1U;
    }
    print("Request rejected (LionDriver reference build): ");
    puth(req->request);
    print("\n");
  }
  return allowed;
}
#endif
