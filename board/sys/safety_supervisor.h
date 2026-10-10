#pragma once

// LionDriver safety supervisor: technical safety concept LD-TSC-001, gap G1, layer 1.
//
// Pure logic with no register access, so it can be unit tested on the host
// (tests/safety_supervisor). The STM32H7 glue (independent watchdog, RAM ECC)
// is in board/stm32h7/llsafety.h; the wiring is in board/main.c.
//
// What it provides:
//   * fault escalation (TSR-MCU-04): faults that used to be only recorded latch a
//     no-output state that lasts until power cycle
//   * program-flow monitoring (TSR-MCU-06): the watchdog is only refreshed when the
//     main loop and the 1 Hz housekeeping have both run
//   * redundant storage with an inverted copy (TSR-MCU-08)
//   * an incremental CRC-32 over the signed application image (TSR-MCU-09)
//   * a March C- RAM test (TSR-MCU-10)

// ******************** redundant storage ********************

typedef struct {
  uint32_t value;
  uint32_t inverse;
} redundant_u32_t;

void redundant_set(redundant_u32_t *r, uint32_t value) {
  r->value = value;
  r->inverse = ~value;
}

bool redundant_valid(const redundant_u32_t *r) {
  return r->value == (~r->inverse);
}

// ******************** fault escalation and program flow ********************

// Faults that mean the MCU or its firmware can no longer be trusted to check commands.
// FAULT_RELAY_MALFUNCTION is handled by the safety mode itself; USB, sound, tach, siren,
// EXTI and clock-source faults do not affect the safety path and stay record-only.
#define SUPERVISOR_ESCALATED_FAULTS (FAULT_UNUSED_INTERRUPT_HANDLED | \
                                     FAULT_INTERRUPT_RATE_CAN_1 | FAULT_INTERRUPT_RATE_CAN_2 | FAULT_INTERRUPT_RATE_CAN_3 | \
                                     FAULT_INTERRUPT_RATE_INTERRUPTS | FAULT_INTERRUPT_RATE_TICK | \
                                     FAULT_INTERRUPT_RATE_SPI | FAULT_INTERRUPT_RATE_SPI_DMA | \
                                     FAULT_REGISTER_DIVERGENT | FAULT_HEARTBEAT_LOOP_WATCHDOG | \
                                     FAULT_ECC_UNCORRECTABLE | FAULT_SAFETY_STATE_CORRUPT | \
                                     FAULT_CODE_CRC_MISMATCH | FAULT_STARTUP_RAM_TEST)

// Program-flow checkpoints
#define SUPERVISOR_CP_MAIN_LOOP 1U
#define SUPERVISOR_CP_HOUSEKEEPING 2U

// Housekeeping runs every 8th tick, so 8 ticks is the most that may pass without it
#define SUPERVISOR_MAX_TICKS_WITHOUT_HOUSEKEEPING 8U

typedef struct {
  redundant_u32_t latched_faults;  // escalated faults seen since power-up; never cleared
  uint32_t checkpoints;            // checkpoints passed since the last watchdog decision
  uint32_t ticks_since_housekeeping;
} supervisor_state_t;

void supervisor_init(supervisor_state_t *s) {
  redundant_set(&s->latched_faults, 0U);
  s->checkpoints = 0U;
  s->ticks_since_housekeeping = 0U;
}

// Latch any escalated fault in fault_bits. A corrupted latch counts as a fault itself.
void supervisor_report_faults(supervisor_state_t *s, uint32_t fault_bits) {
  uint32_t latched = s->latched_faults.value;
  if (!redundant_valid(&s->latched_faults)) {
    latched = FAULT_SAFETY_STATE_CORRUPT;
  }
  latched |= (fault_bits & SUPERVISOR_ESCALATED_FAULTS);
  redundant_set(&s->latched_faults, latched);
}

// True only when the latch is intact and empty. A zeroed (never initialised) state is invalid
// because its inverse is not all ones, so it also blocks output.
bool supervisor_output_allowed(const supervisor_state_t *s) {
  return redundant_valid(&s->latched_faults) && (s->latched_faults.value == 0U);
}

void supervisor_checkpoint(supervisor_state_t *s, uint32_t checkpoint) {
  s->checkpoints |= checkpoint;
}

void supervisor_housekeeping_done(supervisor_state_t *s) {
  s->ticks_since_housekeeping = 0U;
  s->checkpoints |= SUPERVISOR_CP_HOUSEKEEPING;
}

// Called once per 8 Hz tick, after housekeeping. Returns true when the watchdog may be refreshed:
// the main loop has run since the last tick and housekeeping has not been skipped.
// Refreshing does not depend on the fault latch: a latched state is already safe, and a reset
// would clear the latch.
bool supervisor_tick(supervisor_state_t *s) {
  if (s->ticks_since_housekeeping < 0xFFFFFFFFU) {
    s->ticks_since_housekeeping += 1U;
  }
  bool main_loop_ran = (s->checkpoints & SUPERVISOR_CP_MAIN_LOOP) != 0U;
  bool housekeeping_on_time = s->ticks_since_housekeeping <= SUPERVISOR_MAX_TICKS_WITHOUT_HOUSEKEEPING;
  s->checkpoints = 0U;
  return main_loop_ran && housekeeping_on_time;
}

// ******************** CRC-32 (IEEE 802.3, reflected) ********************

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len) {
  static const uint32_t crc32_table[256] = {
    0x00000000U, 0x77073096U, 0xEE0E612CU, 0x990951BAU, 0x076DC419U, 0x706AF48FU,
    0xE963A535U, 0x9E6495A3U, 0x0EDB8832U, 0x79DCB8A4U, 0xE0D5E91EU, 0x97D2D988U,
    0x09B64C2BU, 0x7EB17CBDU, 0xE7B82D07U, 0x90BF1D91U, 0x1DB71064U, 0x6AB020F2U,
    0xF3B97148U, 0x84BE41DEU, 0x1ADAD47DU, 0x6DDDE4EBU, 0xF4D4B551U, 0x83D385C7U,
    0x136C9856U, 0x646BA8C0U, 0xFD62F97AU, 0x8A65C9ECU, 0x14015C4FU, 0x63066CD9U,
    0xFA0F3D63U, 0x8D080DF5U, 0x3B6E20C8U, 0x4C69105EU, 0xD56041E4U, 0xA2677172U,
    0x3C03E4D1U, 0x4B04D447U, 0xD20D85FDU, 0xA50AB56BU, 0x35B5A8FAU, 0x42B2986CU,
    0xDBBBC9D6U, 0xACBCF940U, 0x32D86CE3U, 0x45DF5C75U, 0xDCD60DCFU, 0xABD13D59U,
    0x26D930ACU, 0x51DE003AU, 0xC8D75180U, 0xBFD06116U, 0x21B4F4B5U, 0x56B3C423U,
    0xCFBA9599U, 0xB8BDA50FU, 0x2802B89EU, 0x5F058808U, 0xC60CD9B2U, 0xB10BE924U,
    0x2F6F7C87U, 0x58684C11U, 0xC1611DABU, 0xB6662D3DU, 0x76DC4190U, 0x01DB7106U,
    0x98D220BCU, 0xEFD5102AU, 0x71B18589U, 0x06B6B51FU, 0x9FBFE4A5U, 0xE8B8D433U,
    0x7807C9A2U, 0x0F00F934U, 0x9609A88EU, 0xE10E9818U, 0x7F6A0DBBU, 0x086D3D2DU,
    0x91646C97U, 0xE6635C01U, 0x6B6B51F4U, 0x1C6C6162U, 0x856530D8U, 0xF262004EU,
    0x6C0695EDU, 0x1B01A57BU, 0x8208F4C1U, 0xF50FC457U, 0x65B0D9C6U, 0x12B7E950U,
    0x8BBEB8EAU, 0xFCB9887CU, 0x62DD1DDFU, 0x15DA2D49U, 0x8CD37CF3U, 0xFBD44C65U,
    0x4DB26158U, 0x3AB551CEU, 0xA3BC0074U, 0xD4BB30E2U, 0x4ADFA541U, 0x3DD895D7U,
    0xA4D1C46DU, 0xD3D6F4FBU, 0x4369E96AU, 0x346ED9FCU, 0xAD678846U, 0xDA60B8D0U,
    0x44042D73U, 0x33031DE5U, 0xAA0A4C5FU, 0xDD0D7CC9U, 0x5005713CU, 0x270241AAU,
    0xBE0B1010U, 0xC90C2086U, 0x5768B525U, 0x206F85B3U, 0xB966D409U, 0xCE61E49FU,
    0x5EDEF90EU, 0x29D9C998U, 0xB0D09822U, 0xC7D7A8B4U, 0x59B33D17U, 0x2EB40D81U,
    0xB7BD5C3BU, 0xC0BA6CADU, 0xEDB88320U, 0x9ABFB3B6U, 0x03B6E20CU, 0x74B1D29AU,
    0xEAD54739U, 0x9DD277AFU, 0x04DB2615U, 0x73DC1683U, 0xE3630B12U, 0x94643B84U,
    0x0D6D6A3EU, 0x7A6A5AA8U, 0xE40ECF0BU, 0x9309FF9DU, 0x0A00AE27U, 0x7D079EB1U,
    0xF00F9344U, 0x8708A3D2U, 0x1E01F268U, 0x6906C2FEU, 0xF762575DU, 0x806567CBU,
    0x196C3671U, 0x6E6B06E7U, 0xFED41B76U, 0x89D32BE0U, 0x10DA7A5AU, 0x67DD4ACCU,
    0xF9B9DF6FU, 0x8EBEEFF9U, 0x17B7BE43U, 0x60B08ED5U, 0xD6D6A3E8U, 0xA1D1937EU,
    0x38D8C2C4U, 0x4FDFF252U, 0xD1BB67F1U, 0xA6BC5767U, 0x3FB506DDU, 0x48B2364BU,
    0xD80D2BDAU, 0xAF0A1B4CU, 0x36034AF6U, 0x41047A60U, 0xDF60EFC3U, 0xA867DF55U,
    0x316E8EEFU, 0x4669BE79U, 0xCB61B38CU, 0xBC66831AU, 0x256FD2A0U, 0x5268E236U,
    0xCC0C7795U, 0xBB0B4703U, 0x220216B9U, 0x5505262FU, 0xC5BA3BBEU, 0xB2BD0B28U,
    0x2BB45A92U, 0x5CB36A04U, 0xC2D7FFA7U, 0xB5D0CF31U, 0x2CD99E8BU, 0x5BDEAE1DU,
    0x9B64C2B0U, 0xEC63F226U, 0x756AA39CU, 0x026D930AU, 0x9C0906A9U, 0xEB0E363FU,
    0x72076785U, 0x05005713U, 0x95BF4A82U, 0xE2B87A14U, 0x7BB12BAEU, 0x0CB61B38U,
    0x92D28E9BU, 0xE5D5BE0DU, 0x7CDCEFB7U, 0x0BDBDF21U, 0x86D3D2D4U, 0xF1D4E242U,
    0x68DDB3F8U, 0x1FDA836EU, 0x81BE16CDU, 0xF6B9265BU, 0x6FB077E1U, 0x18B74777U,
    0x88085AE6U, 0xFF0F6A70U, 0x66063BCAU, 0x11010B5CU, 0x8F659EFFU, 0xF862AE69U,
    0x616BFFD3U, 0x166CCF45U, 0xA00AE278U, 0xD70DD2EEU, 0x4E048354U, 0x3903B3C2U,
    0xA7672661U, 0xD06016F7U, 0x4969474DU, 0x3E6E77DBU, 0xAED16A4AU, 0xD9D65ADCU,
    0x40DF0B66U, 0x37D83BF0U, 0xA9BCAE53U, 0xDEBB9EC5U, 0x47B2CF7FU, 0x30B5FFE9U,
    0xBDBDF21CU, 0xCABAC28AU, 0x53B39330U, 0x24B4A3A6U, 0xBAD03605U, 0xCDD70693U,
    0x54DE5729U, 0x23D967BFU, 0xB3667A2EU, 0xC4614AB8U, 0x5D681B02U, 0x2A6F2B94U,
    0xB40BBE37U, 0xC30C8EA1U, 0x5A05DF1BU, 0x2D02EF8DU
  };
  uint32_t c = crc;
  for (uint32_t i = 0U; i < len; i++) {
    c = crc32_table[(c ^ data[i]) & 0xFFU] ^ (c >> 8U);
  }
  return c;
}

// ******************** application image integrity ********************

#define CODE_CRC_BUSY 0U
#define CODE_CRC_OK 1U
#define CODE_CRC_MISMATCH 2U

typedef struct {
  const uint8_t *start;
  uint32_t len;
  uint32_t offset;
  uint32_t crc;
  bool reference_set;
  redundant_u32_t reference;
} code_crc_state_t;

void code_crc_init(code_crc_state_t *st, const uint8_t *start, uint32_t len) {
  st->start = start;
  st->len = len;
  st->offset = 0U;
  st->crc = 0xFFFFFFFFU;
  st->reference_set = false;
  redundant_set(&st->reference, 0U);
}

// Process up to chunk bytes. The first complete pass sets the reference: the bootstub has just
// verified the image's signature, so the image is trusted at that point. Later passes compare.
uint8_t code_crc_step(code_crc_state_t *st, uint32_t chunk) {
  uint8_t result = CODE_CRC_BUSY;
  uint32_t n = st->len - st->offset;
  if (n > chunk) {
    n = chunk;
  }
  st->crc = crc32_update(st->crc, &st->start[st->offset], n);
  st->offset += n;

  if (st->offset >= st->len) {
    uint32_t final_crc = ~st->crc;
    if (!st->reference_set) {
      redundant_set(&st->reference, final_crc);
      st->reference_set = true;
      result = CODE_CRC_OK;
    } else if (redundant_valid(&st->reference) && (st->reference.value == final_crc)) {
      result = CODE_CRC_OK;
    } else {
      result = CODE_CRC_MISMATCH;
    }
    st->offset = 0U;
    st->crc = 0xFFFFFFFFU;
  }
  return result;
}

// ******************** RAM test ********************

// March C- over words: up(w0); up(r0,w1); up(r1,w0); down(r0,w1); down(r1,w0); up(r0).
// Detects stuck-at, transition and most coupling faults between words. Destroys the contents
// and leaves the memory zeroed. Returns true when the memory passed.
bool march_c_minus(volatile uint32_t *mem, uint32_t words) {
  const uint32_t zero = 0U;
  const uint32_t ones = 0xFFFFFFFFU;
  bool ok = true;

  for (uint32_t i = 0U; i < words; i++) {
    mem[i] = zero;
  }
  for (uint32_t i = 0U; i < words; i++) {
    if (mem[i] != zero) {
      ok = false;
    }
    mem[i] = ones;
  }
  for (uint32_t i = 0U; i < words; i++) {
    if (mem[i] != ones) {
      ok = false;
    }
    mem[i] = zero;
  }
  for (uint32_t i = words; i > 0U; i--) {
    if (mem[i - 1U] != zero) {
      ok = false;
    }
    mem[i - 1U] = ones;
  }
  for (uint32_t i = words; i > 0U; i--) {
    if (mem[i - 1U] != ones) {
      ok = false;
    }
    mem[i - 1U] = zero;
  }
  for (uint32_t i = 0U; i < words; i++) {
    if (mem[i] != zero) {
      ok = false;
    }
  }
  return ok;
}
