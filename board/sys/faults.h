#pragma once

#include "board/sys/sys.h"

uint8_t fault_status = FAULT_STATUS_NONE;
uint32_t faults = 0U;

static void fault_hook_none(uint32_t fault) {
  (void)fault;
}

// Called with each newly recorded fault, so the firmware can react at once instead of at its next
// periodic check (LionDriver safety supervisor, board/main.c). A no-op in builds that don't set it.
fault_hook_t fault_hook = fault_hook_none;

void fault_occurred(uint32_t fault) {
  if ((faults & fault) == 0U) {
    if ((PERMANENT_FAULTS & fault) != 0U) {
      print("Permanent fault occurred: 0x"); puth(fault); print("\n");
      fault_status = FAULT_STATUS_PERMANENT;
    } else {
      print("Temporary fault occurred: 0x"); puth(fault); print("\n");
      fault_status = FAULT_STATUS_TEMPORARY;
    }
  }
  faults |= fault;
  fault_hook(fault);
}

void fault_recovered(uint32_t fault) {
  if ((PERMANENT_FAULTS & fault) == 0U) {
    faults &= ~fault;
  } else {
    print("Cannot recover from a permanent fault!\n");
  }
}
