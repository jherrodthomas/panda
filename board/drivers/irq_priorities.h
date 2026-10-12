#pragma once

// ******************** interrupt priorities ********************
//
// LionDriver TSR-519 / SWSR-519: an explicit NVIC priority for every interrupt the panda firmware enables
// (LionDriver WP-W-03 §5.4). The priorities are grouped into classes so that the scheme can be changed in one
// place.
//
// All classes are at the same level for now. That keeps the equal-priority, no-preemption behaviour on which
// the shared-data argument of WP-W-03 §6.3 rests. WP-W-03 §5.4.2 proposes distinct levels (safety task 2,
// CAN 4, host 6, housekeeping 8, peripherals 10); they are to be set here only after the timing measurements
// of §5.4.5 and with the measures of §5.4.4 in place. Level 8 of 16 (4 priority bits) leaves room above for a
// later high-priority supervisor; NMI and HardFault keep their fixed priorities.

#define IRQ_PRIO_SAFETY_TASK 8U   // TIM4 100 Hz safety task
#define IRQ_PRIO_CAN 8U           // FDCAN RX (forward and RX hooks) and TX completion
#define IRQ_PRIO_HOST 8U          // SPI and USB host commands, TX hook
#define IRQ_PRIO_HOUSEKEEPING 8U  // 8 Hz tick, interrupt-rate timer
#define IRQ_PRIO_PERIPHERAL 8U    // sound DMA, fan tachometer, stop-mode wake-up lines

void interrupt_priorities_init(void) {
  NVIC_SetPriority(SAFETY_TASK_TIMER_IRQ, IRQ_PRIO_SAFETY_TASK);

  NVIC_SetPriority(FDCAN1_IT0_IRQn, IRQ_PRIO_CAN);
  NVIC_SetPriority(FDCAN1_IT1_IRQn, IRQ_PRIO_CAN);
  NVIC_SetPriority(FDCAN2_IT0_IRQn, IRQ_PRIO_CAN);
  NVIC_SetPriority(FDCAN2_IT1_IRQn, IRQ_PRIO_CAN);
  NVIC_SetPriority(FDCAN3_IT0_IRQn, IRQ_PRIO_CAN);
  NVIC_SetPriority(FDCAN3_IT1_IRQn, IRQ_PRIO_CAN);

  NVIC_SetPriority(DMA2_Stream2_IRQn, IRQ_PRIO_HOST);
  NVIC_SetPriority(DMA2_Stream3_IRQn, IRQ_PRIO_HOST);
  NVIC_SetPriority(SPI4_IRQn, IRQ_PRIO_HOST);
  NVIC_SetPriority(OTG_HS_IRQn, IRQ_PRIO_HOST);

  NVIC_SetPriority(TICK_TIMER_IRQ, IRQ_PRIO_HOUSEKEEPING);
  NVIC_SetPriority(INTERRUPT_TIMER_IRQ, IRQ_PRIO_HOUSEKEEPING);

  NVIC_SetPriority(BDMA_Channel0_IRQn, IRQ_PRIO_PERIPHERAL);
  NVIC_SetPriority(DMA1_Stream0_IRQn, IRQ_PRIO_PERIPHERAL);
  NVIC_SetPriority(EXTI2_IRQn, IRQ_PRIO_PERIPHERAL);
  NVIC_SetPriority(EXTI1_IRQn, IRQ_PRIO_PERIPHERAL);
  NVIC_SetPriority(EXTI4_IRQn, IRQ_PRIO_PERIPHERAL);
  NVIC_SetPriority(EXTI9_5_IRQn, IRQ_PRIO_PERIPHERAL);
  NVIC_SetPriority(EXTI15_10_IRQn, IRQ_PRIO_PERIPHERAL);
}
