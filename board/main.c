// ********************* Includes *********************
#include "board/config.h"

#include "board/drivers/led.h"
#include "board/drivers/pwm.h"
#include "board/drivers/usb.h"
#include "board/drivers/simple_watchdog.h"
#include "board/drivers/bootkick.h"

#include "board/early_init.h"
#include "board/provision.h"

#include "opendbc/safety/safety.h"

#include "board/health.h"

#include "board/drivers/can_common.h"

#include "board/drivers/fdcan.h"

#include "board/sys/power_saving.h"

#include "board/sys/safety_supervisor.h"
#include "board/stm32h7/llsafety.h"

#include "board/obj/gitversion.h"

#include "board/can_comms.h"
#include "board/main_comms.h"

// ************************* LionDriver safety supervisor *************************
// Technical safety concept LD-TSC-001, gap G1, layer 1. See board/sys/safety_supervisor.h.

#define RAM_TEST_PASS 0x50415353U
#define RAM_TEST_FAIL 0x4641494CU
// read the whole application image (about 80 KB) about once per second at the 8 Hz tick
#define CODE_CRC_CHUNK 10240U

// defined by the linker script
extern uint32_t startup_ram_test_result;
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _sdata[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _ebss[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _sitcmram[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _eitcmram[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _saxisram[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _eaxisram[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _ssram12[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _esram12[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _ssram4[];
// cppcheck-suppress misra-c2012-8.11 ; linker symbols, the size is not known to C
extern uint32_t _esram4[];

static supervisor_state_t supervisor;
static code_crc_state_t code_crc;
static redundant_u32_t safety_mode_shadow;  // TSR-MCU-08: inverted copy of the active safety mode

static void zero_words(volatile uint32_t *start, const volatile uint32_t *end) {
  uint32_t words = (((uint32_t)end) - ((uint32_t)start)) / 4U;
  for (uint32_t i = 0U; i < words; i++) {
    start[i] = 0U;
  }
}

// TSR-MCU-04: block actuation the moment an escalated fault is recorded. The TX hooks refuse every
// actuating frame without controls, and set_safety_mode refuses car modes once the latch is set.
// The relay is released at the next tick (<= 125 ms), in safety_supervisor_tick.
static void supervisor_fault_hook(uint32_t fault) {
  supervisor_report_faults(&supervisor, fault);
  if (!supervisor_output_allowed(&supervisor)) {
    controls_allowed = false;
  }
}

static uint32_t pack_safety_mode(uint16_t mode, uint16_t param) {
  return (((uint32_t)mode) << 16U) | ((uint32_t)param);
}

static void set_safety_hooks_checked(uint16_t mode, uint16_t param, int *err) {
  ENTER_CRITICAL()
  *err = set_safety_hooks(mode, param);
  redundant_set(&safety_mode_shadow, pack_safety_mode(current_safety_mode, current_safety_param));
  EXIT_CRITICAL()
}

// ****************************** safety mode ******************************

bool is_car_safety_mode(uint16_t mode) {
  return (mode != SAFETY_SILENT) &&
         (mode != SAFETY_NOOUTPUT) &&
         (mode != SAFETY_ALLOUTPUT) &&
         (mode != SAFETY_ELM327);
}

// this is the only way to leave silent mode
void set_safety_mode(uint16_t mode, uint16_t param) {
  uint16_t mode_copy = mode;
  uint16_t param_copy = param;

  // TSR-MCU-04: once a fault has latched, no car safety mode until power cycle
  supervisor_report_faults(&supervisor, faults);
  if (is_car_safety_mode(mode_copy) && !supervisor_output_allowed(&supervisor)) {
    print("Safety supervisor fault latched. Using NOOUTPUT\n");
    mode_copy = SAFETY_NOOUTPUT;
    param_copy = 0U;
  }

  int err;
  set_safety_hooks_checked(mode_copy, param_copy, &err);
  if (err == -1) {
    print("Error: safety set mode failed. Falling back to SILENT\n");
    mode_copy = SAFETY_SILENT;
    set_safety_hooks_checked(mode_copy, 0U, &err);
    // TERMINAL ERROR: we can't continue if SILENT safety mode isn't succesfully set
    assert_fatal(err == 0, "Error: Failed setting SILENT mode. Hanging\n");
  }
  safety_tx_blocked = 0;
  safety_rx_invalid = 0;

  switch (mode_copy) {
    case SAFETY_SILENT:
      set_intercept_relay(false, false);
      current_board->set_can_mode(CAN_MODE_NORMAL);
      can_silent = true;
      break;
    case SAFETY_NOOUTPUT:
      set_intercept_relay(false, false);
      current_board->set_can_mode(CAN_MODE_NORMAL);
      can_silent = false;
      break;
    case SAFETY_ELM327:
      set_intercept_relay(false, false);
      heartbeat_counter = 0U;
      heartbeat_lost = false;

      // Clear any pending messages in the can core (i.e. sending while comma power is unplugged)
      // TODO: rewrite using hardware queues rather than fifo to cancel specific messages
      can_clear_send(CANIF_FROM_CAN_NUM(1), 1);
      if (param == 0U) {
        current_board->set_can_mode(CAN_MODE_OBD_CAN2);
      } else {
        current_board->set_can_mode(CAN_MODE_NORMAL);
      }
      can_silent = false;
      break;
    default:
      // TSR-MCU-05: the relay closes only in car modes, so the watchdog runs from the first one on
      iwdg_start();
      set_intercept_relay(true, false);
      heartbeat_counter = 0U;
      heartbeat_lost = false;
      current_board->set_can_mode(CAN_MODE_NORMAL);
      can_silent = false;
      break;
  }
  can_init_all();
}

// ***************************** main code *****************************

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
// cppcheck-suppress misra-c2012-8.4
void __initialize_hardware_early(void) {
  // TSR-MCU-10: test the RAM that will hold .data and .bss before the C runtime initializes it.
  // Runs on the reset stack only; the result goes to SRAM4, which the runtime does not touch.
  uint32_t words = (((uint32_t)_ebss) - ((uint32_t)_sdata)) / 4U;
  startup_ram_test_result = march_c_minus(_sdata, words) ? RAM_TEST_PASS : RAM_TEST_FAIL;
  // TSR-MCU-07: after power-up the RAM holds random data with random ECC bits, so reading a word
  // that was never written reports an ECC error. Zero the sections the C runtime leaves alone.
  // The bootloader magic and fault cookie live inside SRAM4 and must survive. They are accessed
  // through volatile pointers: the compiler cannot see that the zeroed region overlaps them and
  // would otherwise drop the restore.
  volatile uint32_t *boot_mode_ptr = &enter_bootloader_mode;
  volatile uint32_t *cookie_ptr = &fault_cookie;
  uint32_t boot_mode = *boot_mode_ptr;
  uint32_t cookie = *cookie_ptr;
  zero_words(_sitcmram, _eitcmram);
  zero_words(_saxisram, _eaxisram);
  zero_words(_ssram12, _esram12);
  zero_words(_ssram4, _esram4);
  *boot_mode_ptr = boot_mode;
  *cookie_ptr = cookie;
  early_initialization();
}

static void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}

// go into SILENT when heartbeat isn't received for this amount of seconds.
#define HEARTBEAT_IGNITION_CNT_ON 5U
#define HEARTBEAT_IGNITION_CNT_OFF 2U

// Called from the 8 Hz tick, after the 1 Hz housekeeping
static void safety_supervisor_tick(void) {
  // TSR-MCU-08: the active safety mode must match its inverted-copy shadow
  ENTER_CRITICAL()
  bool mode_intact = redundant_valid(&safety_mode_shadow) &&
                     (safety_mode_shadow.value == pack_safety_mode(current_safety_mode, current_safety_param));
  EXIT_CRITICAL()
  if (!mode_intact) {
    fault_occurred(FAULT_SAFETY_STATE_CORRUPT);
  }

  // TSR-MCU-09: application image integrity
  if (code_crc_step(&code_crc, CODE_CRC_CHUNK) == CODE_CRC_MISMATCH) {
    fault_occurred(FAULT_CODE_CRC_MISMATCH);
  }

  // TSR-MCU-04: an escalated fault ends controls and opens the relay until power cycle
  supervisor_report_faults(&supervisor, faults);
  if (!supervisor_output_allowed(&supervisor)) {
    controls_allowed = false;
    if (is_car_safety_mode(current_safety_mode)) {
      print("Safety supervisor fault latched. Entering NOOUTPUT\n");
      set_safety_mode(SAFETY_NOOUTPUT, 0U);
    }
  }

  // TSR-MCU-05/06: refresh the watchdog only if the main loop and housekeeping ran
  if (supervisor_tick(&supervisor)) {
    iwdg_refresh();
  }
}

// called at 8Hz
static void tick_handler(void) {
  if (TICK_TIMER->SR != 0U) {
    static uint32_t siren_countdown = 0; // siren plays while countdown > 0
    static uint8_t prev_harness_status = HARNESS_STATUS_NC;
    static uint8_t loop_counter = 0U;
    static bool relay_malfunction_prev = false;

    // siren
    current_board->set_siren((loop_counter & 1U) && (siren_enabled || (siren_countdown > 0U)));

    // tick drivers at 8Hz
    fan_tick();
    harness_tick();
    simple_watchdog_kick();
    sound_tick();

    if (relay_malfunction_prev != relay_malfunction) {
      if (relay_malfunction) {
        fault_occurred(FAULT_RELAY_MALFUNCTION);
      } else {
        fault_recovered(FAULT_RELAY_MALFUNCTION);
      }
    }
    relay_malfunction_prev = relay_malfunction;

    // re-init everything that uses harness status
    if (harness.status != prev_harness_status) {
      prev_harness_status = harness.status;
      can_set_orientation(harness.status == HARNESS_STATUS_FLIPPED);

      // re-init everything that uses harness status
      can_init_all();
      set_safety_mode(current_safety_mode, current_safety_param);
      set_power_save_state(power_save_enabled);
    }

    // decimated to 1Hz
    if (loop_counter == 0U) {
      static uint32_t controls_allowed_countdown = 0;

      // set green LED to be controls allowed
      led_set(LED_GREEN, controls_allowed);

      // turn off the blue LED, turned on by CAN
      // unless we are in power saving mode
      led_set(LED_BLUE, (uptime_cnt & 1U) && power_save_enabled);

      const bool recent_heartbeat = heartbeat_counter == 0U;

      // tick drivers at 1Hz
      bool started = harness_check_ignition() || ignition_can;
      bootkick_tick(started, recent_heartbeat);

      // increase heartbeat counter and cap it at the uint32 limit
      if (heartbeat_counter < UINT32_MAX) {
        heartbeat_counter += 1U;
      }

      // disabling heartbeat not allowed while in safety mode
      if (is_car_safety_mode(current_safety_mode)) {
        heartbeat_disabled = false;
      }

      if (siren_countdown > 0U) {
        siren_countdown -= 1U;
      }

      if (controls_allowed || heartbeat_engaged) {
        controls_allowed_countdown = 5U;
      } else if (controls_allowed_countdown > 0U) {
        controls_allowed_countdown -= 1U;
      } else {

      }

      // exit controls allowed if unused by openpilot for a few seconds
      if (controls_allowed && !heartbeat_engaged) {
        heartbeat_engaged_mismatches += 1U;
        if (heartbeat_engaged_mismatches >= 3U) {
          controls_allowed = false;
        }
      } else {
        heartbeat_engaged_mismatches = 0U;
      }

      if (!heartbeat_disabled) {
        // if the heartbeat has been gone for a while, go to SILENT safety mode and enter power save
        if (heartbeat_counter >= (started ? HEARTBEAT_IGNITION_CNT_ON : HEARTBEAT_IGNITION_CNT_OFF)) {
          print("device hasn't sent a heartbeat for 0x");
          puth(heartbeat_counter);
          print(" seconds. Safety is set to SILENT mode.\n");

          if (controls_allowed_countdown > 0U) {
            siren_countdown = 3U;
            controls_allowed_countdown = 0U;
          }

          // set flag to indicate the heartbeat was lost
          if (is_car_safety_mode(current_safety_mode)) {
            heartbeat_lost = true;
          }

          // clear heartbeat engaged state
          heartbeat_engaged = false;

          if (current_safety_mode != SAFETY_SILENT) {
            set_safety_mode(SAFETY_SILENT, 0U);
          }

          if (!power_save_enabled) {
            set_power_save_state(true);
          }

          // Also disable IR when the heartbeat goes missing
          current_board->set_ir_power(0U);

          // Run fan when device is up but not talking to us.
          // The bootloader enables the SOM GPIO on boot.
          fan_set_power(current_board->read_som_gpio() ? 30U : 0U);
        }
      }

      // check registers
      check_registers();

      // set ignition_can to false after 2s of no CAN seen
      if (ignition_can_cnt > 2U) {
        ignition_can = false;
      }

      // on to the next one
      uptime_cnt += 1U;
      safety_mode_cnt += 1U;
      ignition_can_cnt += 1U;

      // synchronous safety check
      safety_tick();
      supervisor_housekeeping_done(&supervisor);
    }

    safety_supervisor_tick();

    loop_counter++;
    loop_counter %= 8U;
  }
  TICK_TIMER->SR = 0;
}

int main(void) {
  supervisor_init(&supervisor);
  fault_hook = supervisor_fault_hook;

  // Init interrupt table
  init_interrupts(true);

  // shouldn't have interrupts here, but just in case
  disable_interrupts();

  // init early devices
  clock_init();
  peripherals_init();
  detect_board_type();
  led_init();
  // red+green leds enabled until succesful USB/SPI init, as a debug indicator
  led_set(LED_RED, true);
  led_set(LED_GREEN, true);
  adc_init(ADC1);
  dts_init();

  // print hello
  print("\n\n\n************************ MAIN START ************************\n");

  // check for non-supported board types
  assert_fatal(hw_type != HW_TYPE_UNKNOWN, "Unsupported board type");

  print("Config:\n");
  print("  Board type: 0x"); puth(hw_type); print("\n");
  print("  MCU UID: "); hexdump((const uint8_t *)UID_BASE, 12);

  // init board
  current_board->init();
  current_board->set_can_mode(CAN_MODE_NORMAL);
  harness_init();

  // panda has an FPU, let's use it!
  enable_fpu();

  microsecond_timer_init();

  current_board->set_siren(false);
  if (current_board->has_fan) {
    fan_init();
  }

  // LionDriver safety supervisor (TSC LD-TSC-001, G1 layer 1)
  if (startup_ram_test_result != RAM_TEST_PASS) {
    print("Startup RAM test failed\n");
    fault_occurred(FAULT_STARTUP_RAM_TEST);
  }
  if (iwdg_caused_reset()) {
    print("Last reset was caused by the independent watchdog\n");
  }
  if (!bor_enabled()) {
    print("WARNING: brown-out reset is disabled in the option bytes\n");
  }
  // the signed length is stored in the image's first word (see board/crypto/sign.py)
  uint32_t app_len = (uint32_t)_app_start[0];
  if ((app_len < 8U) || (app_len > 0xE0000U)) {
    print("Application image length invalid\n");
    fault_occurred(FAULT_CODE_CRC_MISMATCH);
    app_len = 8U;
  }
  code_crc_init(&code_crc, (const uint8_t *)_app_start, app_len);
  ramecc_init();
  supervisor_report_faults(&supervisor, faults);

  // init to SILENT and can silent
  set_safety_mode(SAFETY_SILENT, 0U);

  // enable CAN TXs
  enable_can_transceivers(true);

  // init watchdog for heartbeat loop, fed at 8Hz
  simple_watchdog_init(FAULT_HEARTBEAT_LOOP_WATCHDOG, (3U * 1000000U / 8U));

  // 8Hz timer
  REGISTER_INTERRUPT(TICK_TIMER_IRQ, tick_handler, 10U, FAULT_INTERRUPT_RATE_TICK)
  tick_timer_init();

  // enable USB (right before interrupts or enum can fail!)
  usb_init();

  if (current_board->has_spi) {
    gpio_spi_init();
    spi_init();
  }

  led_set(LED_RED, false);
  led_set(LED_GREEN, false);
  led_set(LED_BLUE, false);

  if (fault_cookie == NMI_COOKIE) {
    reset_flags |= HEALTH_FLAG_NMI_RESET;
  }
  if (fault_cookie == HARDFAULT_COOKIE) {
    reset_flags |= HEALTH_FLAG_HARDFAULT_RESET;
  }
  fault_cookie = 0U;

  print("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // LED should keep on blinking all the time
  while (true) {
    #ifdef ALLOW_DEBUG
    if (stop_mode_requested) {
      enter_stop_mode();
    }
    #endif
    if (!power_save_enabled) {
      // useful for debugging, fade breaks = panda is overloaded
      for (uint32_t fade = 0U; fade < MAX_LED_FADE; fade += 1U) {
        supervisor_checkpoint(&supervisor, SUPERVISOR_CP_MAIN_LOOP);
        led_set(LED_RED, true);
        delay(fade >> 4);
        led_set(LED_RED, false);
        delay((MAX_LED_FADE - fade) >> 4);
      }

      for (uint32_t fade = MAX_LED_FADE; fade > 0U; fade -= 1U) {
        supervisor_checkpoint(&supervisor, SUPERVISOR_CP_MAIN_LOOP);
        led_set(LED_RED, true);
        delay(fade >> 4);
        led_set(LED_RED, false);
        delay((MAX_LED_FADE - fade) >> 4);
      }
    } else {
      supervisor_checkpoint(&supervisor, SUPERVISOR_CP_MAIN_LOOP);
      if ((hw_type == HW_TYPE_CUATRO) && !current_board->read_som_gpio()) {
        assert_fatal(current_safety_mode == SAFETY_SILENT, "Error: Entering low power mode while not in SAFETY_SILENT. Hanging\n");
        enter_stop_mode(); // deep sleep, wakes on CAN or SBU activity
        assert_fatal(false, "Error: enter_stop_mode returned after system reset. Hanging\n");
      }
      // cppcheck-suppress misra-c2012-17.3 ; CMSIS __WFI macro expands to inline asm
      __WFI();
      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    }
  }

  return 0;
}
