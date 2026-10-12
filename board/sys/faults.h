#pragma once

#include "board/sys/sys.h"

uint8_t fault_status = FAULT_STATUS_NONE;
uint32_t faults = 0U;

// LionDriver WP-W-03 §5.4.3 R-8: the fault words are read-modify-written from several interrupts; update them
// atomically so that a preempting interrupt cannot lose another fault bit
void fault_occurred(uint32_t fault) {
  ENTER_CRITICAL();
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
  EXIT_CRITICAL();
}

void fault_recovered(uint32_t fault) {
  ENTER_CRITICAL();
  if ((PERMANENT_FAULTS & fault) == 0U) {
    faults &= ~fault;
  } else {
    print("Cannot recover from a permanent fault!\n");
  }
  EXIT_CRITICAL();
}
