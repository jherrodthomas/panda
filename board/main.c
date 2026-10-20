// ********************* Includes *********************
#include "board/config.h"

#include "board/drivers/led.h"
#include "board/drivers/pwm.h"
#include "board/drivers/usb.h"
#include "board/drivers/simple_watchdog.h"
#include "board/drivers/hw_watchdog.h"
#include "board/drivers/heartbeat_monitor.h"
#include "board/drivers/eps_fault_siren.h"
#include "board/drivers/heartbeat_siren.h"
#include "board/drivers/irq_priorities.h"
#ifdef LIONDRIVER_FAULT_INJECTION
#include "board/drivers/fault_injection.h"
#endif
#include "board/drivers/bootkick.h"

#include "board/early_init.h"
#include "board/provision.h"

#include "opendbc/safety/safety.h"

#include "board/health.h"

#include "board/drivers/can_common.h"

#include "board/drivers/fdcan.h"

#include "board/sys/power_saving.h"

#include "board/obj/gitversion.h"

#include "board/can_comms.h"
#include "board/reference_lock.h"
#include "board/main_comms.h"

// ****************************** safety mode ******************************

// this is the only way to leave silent mode
void set_safety_mode(uint16_t mode, uint16_t param) {
  uint16_t mode_copy = mode;
  // LionDriver WP-W-03 §5.4.3 R-1: switch the safety configuration atomically, so that no interrupt (in
  // particular a higher-priority safety task) sees rx_checks and rx_checks_len from different modes.
  // set_safety_hooks only resets state and copies the configuration; it does not wait on hardware.
  ENTER_CRITICAL();
  reference_lock_relay_malfunction_update(relay_malfunction);  // DFA DM-25: before set_safety_hooks clears it
  int err = set_safety_hooks(mode_copy, param);
  eps_fault_siren_reset();  // LionDriver TSR-516(b): no siren carried over from the previous mode
  heartbeat_siren_reset();
  EXIT_CRITICAL();
  if (err == -1) {
    print("Error: safety set mode failed. Falling back to SILENT\n");
    mode_copy = SAFETY_SILENT;
    ENTER_CRITICAL();
    err = set_safety_hooks(mode_copy, 0U);
    EXIT_CRITICAL();
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
      set_intercept_relay(true, false);
      heartbeat_counter = 0U;
      heartbeat_lost = false;
      current_board->set_can_mode(CAN_MODE_NORMAL);
      can_silent = false;
      break;
  }
  can_init_all();
}

bool is_car_safety_mode(uint16_t mode) {
  return (mode != SAFETY_SILENT) &&
         (mode != SAFETY_NOOUTPUT) &&
         (mode != SAFETY_ALLOUTPUT) &&
         (mode != SAFETY_ELM327);
}

// ***************************** main code *****************************

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
// cppcheck-suppress misra-c2012-8.4
void __initialize_hardware_early(void) {
  early_initialization();
}

static void __attribute__ ((noinline)) enable_fpu(void) {
  // enable the FPU
  SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
}

// go into SILENT when heartbeat isn't received for this amount of seconds.
#define HEARTBEAT_IGNITION_CNT_ON 5U
#define HEARTBEAT_IGNITION_CNT_OFF 2U

// marks main-loop progress for the hardware watchdog; called often enough that an injected
// hang starts within microseconds of the request, not at the end of an LED fade cycle
static void main_loop_progress(void) {
  hw_watchdog_main_loop_alive();
  #ifdef LIONDRIVER_FAULT_INJECTION
  fault_injection_main_loop();
  #endif
}

// LionDriver TSR-516(b): set by the safety task, read by the tick that drives the speaker
static bool eps_fault_siren_active = false;
static bool heartbeat_siren_active = false;

// called at SAFETY_TASK_FREQ (100 Hz): periodic safety checks (LionDriver SWSR-401a) and
// hardware watchdog feed
static void safety_task_handler(void) {
  if (SAFETY_TASK_TIMER->SR != 0U) {
    // RX timeout and validity checks of the safety mode
    safety_tick();
    // LionDriver TSR-407: no SoC heartbeat for > 0.3 s in a car mode revokes all authority
    const bool heartbeat_revoke = heartbeat_monitor_tick() && is_car_safety_mode(current_safety_mode);
    // LionDriver TSR-516(b): siren at once when this (evaluated before the revocation) or the TX-arrival check in
    // safety_tick() (TSR-408, DQ-404 A) revokes granted authority, so a SoC stall is never silent
    heartbeat_siren_active = heartbeat_siren_tick(heartbeat_revoke, controls_allowed, safety_tx_arrival_revoked);
    if (heartbeat_revoke) {
      controls_allowed = false;
    }
    // LionDriver TSR-516(b): siren when the EPS fault (TSR-110) persists while authority is granted
    eps_fault_siren_active = eps_fault_siren_tick(safety_eps_fault, controls_allowed);
    hw_watchdog_tick();
    #ifdef LIONDRIVER_FAULT_INJECTION
    fault_injection_safety_task();
    #endif
  }
  SAFETY_TASK_TIMER->SR = 0;
}

// called at 8Hz
static void tick_handler(void) {
  if (TICK_TIMER->SR != 0U) {
    static uint32_t siren_countdown = 0; // siren plays while countdown > 0
    static uint8_t prev_harness_status = HARNESS_STATUS_NC;
    static uint8_t loop_counter = 0U;
    static bool relay_malfunction_prev = false;

    // siren
    current_board->set_siren((loop_counter & 1U) && (siren_enabled || (siren_countdown > 0U) || eps_fault_siren_active || heartbeat_siren_active));

    // tick drivers at 8Hz
    fan_tick();
    harness_tick();
    simple_watchdog_kick();
    #ifdef LIONDRIVER_FAULT_INJECTION
    fault_injection_tick();
    #endif
    sound_tick();

    if (relay_malfunction_prev != relay_malfunction) {
      if (relay_malfunction) {
        fault_occurred(FAULT_RELAY_MALFUNCTION);
      } else {
        fault_recovered(FAULT_RELAY_MALFUNCTION);
      }
    }
    relay_malfunction_prev = relay_malfunction;
    reference_lock_relay_malfunction_update(relay_malfunction);

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
      reference_lock_ignition_update(started);

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
    }

    loop_counter++;
    loop_counter %= 8U;
  }
  TICK_TIMER->SR = 0;
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  // shouldn't have interrupts here, but just in case
  disable_interrupts();

  // LionDriver SWSR-519: explicit priority for every interrupt, before any of them is enabled
  interrupt_priorities_init();

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

  // init to SILENT and can silent
  set_safety_mode(SAFETY_SILENT, 0U);

  // enable CAN TXs
  enable_can_transceivers(true);

  // init watchdog for heartbeat loop, fed at 8Hz
  simple_watchdog_init(FAULT_HEARTBEAT_LOOP_WATCHDOG, (3U * 1000000U / 8U));

  // 8Hz timer
  REGISTER_INTERRUPT(TICK_TIMER_IRQ, tick_handler, 10U, FAULT_INTERRUPT_RATE_TICK)
  tick_timer_init();

  // 100 Hz safety task; shares the tick's rate fault (fault bits mirror the cereal FaultType enum)
  REGISTER_INTERRUPT(SAFETY_TASK_TIMER_IRQ, safety_task_handler, (SAFETY_TASK_FREQ + 10U), FAULT_INTERRUPT_RATE_TICK)
  safety_task_timer_init();

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

  // LionDriver: report a reset by the independent hardware watchdog (TSR-501), then clear the reset cause
  // flags, which otherwise persist, so that the next reset reports its own cause only
  if ((RCC->RSR & RCC_RSR_IWDG1RSTF) != 0U) {
    reset_flags |= HEALTH_FLAG_WATCHDOG_RESET;
  }
  RCC->RSR |= RCC_RSR_RMVF;

  print("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // LED should keep on blinking all the time
  while (true) {
    main_loop_progress();
    #ifdef ALLOW_DEBUG
    if (stop_mode_requested) {
      enter_stop_mode();
    }
    #endif
    if (!power_save_enabled) {
      // useful for debugging, fade breaks = panda is overloaded
      for (uint32_t fade = 0U; fade < MAX_LED_FADE; fade += 1U) {
        main_loop_progress();
        led_set(LED_RED, true);
        delay(fade >> 4);
        led_set(LED_RED, false);
        delay((MAX_LED_FADE - fade) >> 4);
      }

      for (uint32_t fade = MAX_LED_FADE; fade > 0U; fade -= 1U) {
        main_loop_progress();
        led_set(LED_RED, true);
        delay(fade >> 4);
        led_set(LED_RED, false);
        delay((MAX_LED_FADE - fade) >> 4);
      }
    } else {
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
