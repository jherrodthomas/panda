#pragma once

// STM32H7 glue for the LionDriver safety supervisor (board/sys/safety_supervisor.h).
// Technical safety concept LD-TSC-001, gap G1, layer 1.

// ******************** independent watchdog (TSR-MCU-05) ********************
// IWDG1 runs from the LSI oscillator, independent of the HSE crystal and the PLL.
// Once started it cannot be stopped by software; any reset stops it.

#define IWDG_KEY_START 0xCCCCU
#define IWDG_KEY_REFRESH 0xAAAAU
#define IWDG_KEY_UNLOCK 0x5555U
#define IWDG_PRESCALER_DIV32 3U  // LSI ~32 kHz / 32 = ~1 kHz
#define IWDG_RELOAD_250MS 250U   // ~250 ms; LSI tolerance makes this roughly 220 to 270 ms

static bool iwdg_running = false;

void iwdg_start(void) {
  if (!iwdg_running) {
    IND_WDG->KR = IWDG_KEY_START;
    IND_WDG->KR = IWDG_KEY_UNLOCK;
    IND_WDG->PR = IWDG_PRESCALER_DIV32;
    IND_WDG->RLR = IWDG_RELOAD_250MS;
    // the new prescaler and reload take effect after a few LSI cycles
    for (uint32_t i = 0U; (i < 1000000U) && (IND_WDG->SR != 0U); i++) {}
    IND_WDG->KR = IWDG_KEY_REFRESH;
    iwdg_running = true;
  }
}

void iwdg_refresh(void) {
  if (iwdg_running) {
    IND_WDG->KR = IWDG_KEY_REFRESH;
  }
}

// True if the last reset came from the independent watchdog. Clears the reset flags.
bool iwdg_caused_reset(void) {
  bool ret = (RCC->RSR & RCC_RSR_IWDG1RSTF) != 0U;
  RCC->RSR |= RCC_RSR_RMVF;
  return ret;
}

// ******************** RAM ECC (TSR-MCU-07) ********************
// The STM32H7 corrects single-bit errors in its RAMs. Double-bit errors are reported through the
// RAMECC monitors; enable their interrupt and escalate.

#define RAMECC_DOUBLE_ERRORS (RAMECC_SR_DEDF | RAMECC_SR_DEBWDF)

static void ecc_check_monitor(RAMECC_MonitorTypeDef *monitor) {
  uint32_t sr = monitor->SR;
  if ((sr & RAMECC_DOUBLE_ERRORS) != 0U) {
    fault_occurred(FAULT_ECC_UNCORRECTABLE);
  }
  // flags are cleared by writing 0
  monitor->SR = 0U;
}

static void ecc_irq_handler(void) {
  ecc_check_monitor(RAMECC1_Monitor1);
  ecc_check_monitor(RAMECC1_Monitor2);
  ecc_check_monitor(RAMECC1_Monitor3);
  ecc_check_monitor(RAMECC1_Monitor4);
  ecc_check_monitor(RAMECC1_Monitor5);
  ecc_check_monitor(RAMECC1_Monitor6);
  ecc_check_monitor(RAMECC2_Monitor1);
  ecc_check_monitor(RAMECC2_Monitor2);
  ecc_check_monitor(RAMECC2_Monitor3);
  ecc_check_monitor(RAMECC3_Monitor1);
  ecc_check_monitor(RAMECC3_Monitor2);
}

void ramecc_init(void) {
  const uint32_t ier = RAMECC_IER_GIE | RAMECC_IER_GECCDEIE | RAMECC_IER_GECCDEBWIE;
  // Discard errors flagged before this point: at a cold boot the bootloader magic words in SRAM4
  // are read before anything writes them, which flags an ECC error on random power-up contents.
  RAMECC_MonitorTypeDef *const monitors[] = {
    RAMECC1_Monitor1, RAMECC1_Monitor2, RAMECC1_Monitor3, RAMECC1_Monitor4, RAMECC1_Monitor5, RAMECC1_Monitor6,
    RAMECC2_Monitor1, RAMECC2_Monitor2, RAMECC2_Monitor3, RAMECC3_Monitor1, RAMECC3_Monitor2,
  };
  for (uint32_t i = 0U; i < (sizeof(monitors) / sizeof(monitors[0])); i++) {
    monitors[i]->SR = 0U;
  }
  REGISTER_INTERRUPT(ECC_IRQn, ecc_irq_handler, 10U, FAULT_ECC_UNCORRECTABLE)
  register_set(&(RAMECC1->IER), ier, 0xFU);
  register_set(&(RAMECC2->IER), ier, 0xFU);
  register_set(&(RAMECC3->IER), ier, 0xFU);
  NVIC_EnableIRQ(ECC_IRQn);
}

// ******************** brown-out reset (TSR-MCU-11) ********************
// The brown-out level is an option byte set at provisioning. Firmware only reports it.

bool bor_enabled(void) {
  return (FLASH->OPTSR_CUR & FLASH_OPTSR_BOR_LEV) != 0U;
}
