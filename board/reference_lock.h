#pragma once

// declarations (ControlPacket_t, current_safety_mode, SAFETY_* modes, print, puth) are included before
// this header

bool is_car_safety_mode(uint16_t mode);

// LionDriver reference build (TSR-512/513, SWSR-512/512a/513): the 2020 Corolla LE TSS2 reference
// configuration. The host may only request SILENT, NOOUTPUT, ELM327 and TOYOTA with the reference
// parameter; once TOYOTA is active only NOOUTPUT or SILENT. The alternative experience stays at its
// reset value (0). Requests that change actuation, relay, CAN or comms state are refused in a car mode.
// Internal safety mode changes (heartbeat loss, faults) do not pass through here and are not affected.
#ifdef LIONDRIVER_REFERENCE
#define LIONDRIVER_REFERENCE_TOYOTA_PARAM 73U  // EPS factor 73, no flags

static bool reference_request_allowed(const ControlPacket_t *req) {
  bool allowed = true;
  bool car_mode = is_car_safety_mode(current_safety_mode);
  switch (req->request) {
    case 0xdcU:
      if (current_safety_mode == SAFETY_TOYOTA) {
        allowed = (req->param1 == SAFETY_NOOUTPUT) || (req->param1 == SAFETY_SILENT);
      } else {
        allowed = (req->param1 == SAFETY_SILENT) || (req->param1 == SAFETY_NOOUTPUT) || (req->param1 == SAFETY_ELM327) ||
                  ((req->param1 == SAFETY_TOYOTA) && (req->param2 == LIONDRIVER_REFERENCE_TOYOTA_PARAM));
      }
      break;
    case 0xdfU:
      allowed = false;
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
    // reported on the debug console only; reporting to the host needs a health packet field (open item)
    print("Request rejected (LionDriver reference build): ");
    puth(req->request);
    print("\n");
  }
  return allowed;
}
#endif
