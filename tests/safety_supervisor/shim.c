// Host build of the LionDriver safety supervisor for unit tests.
#include <stdint.h>
#include <stdbool.h>

#include "board/sys/sys.h"
#include "board/sys/safety_supervisor.h"

uint32_t shim_escalated_faults(void) { return SUPERVISOR_ESCALATED_FAULTS; }
uint32_t shim_crc32(const uint8_t *data, uint32_t len) { return ~crc32_update(0xFFFFFFFFU, data, len); }
