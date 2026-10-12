#pragma once

static void timer_init(TIM_TypeDef *TIM, int psc) {
  register_set(&(TIM->PSC), (psc-1), 0xFFFFU);
  register_set(&(TIM->DIER), TIM_DIER_UIE, 0x5F5FU);
  register_set(&(TIM->CR1), TIM_CR1_CEN, 0x3FU);
  TIM->SR = 0;
}

void microsecond_timer_init(void) {
  MICROSECOND_TIMER->PSC = (APB1_TIMER_FREQ - 1U);
  MICROSECOND_TIMER->CR1 = TIM_CR1_CEN;
  MICROSECOND_TIMER->EGR = TIM_EGR_UG;
}

uint32_t microsecond_timer_get(void) {
  return MICROSECOND_TIMER->CNT;
}

void interrupt_timer_init(void) {
  enable_interrupt_timer();
  REGISTER_INTERRUPT(INTERRUPT_TIMER_IRQ, interrupt_timer_handler, 2U, FAULT_INTERRUPT_RATE_INTERRUPTS)
  register_set(&(INTERRUPT_TIMER->PSC), ((uint16_t)(15.25*APB1_TIMER_FREQ)-1U), 0xFFFFU);
  register_set(&(INTERRUPT_TIMER->DIER), TIM_DIER_UIE, 0x5F5FU);
  register_set(&(INTERRUPT_TIMER->CR1), TIM_CR1_CEN, 0x3FU);
  INTERRUPT_TIMER->SR = 0;
  NVIC_EnableIRQ(INTERRUPT_TIMER_IRQ);
}

void safety_task_timer_init(void) {
  // APB1 timer clock / APB1_TIMER_FREQ = 1 MHz count; period 1e6 / SAFETY_TASK_FREQ = 10 ms
  register_set(&(SAFETY_TASK_TIMER->PSC), (APB1_TIMER_FREQ - 1U), 0xFFFFU);
  register_set(&(SAFETY_TASK_TIMER->ARR), ((1000000U / SAFETY_TASK_FREQ) - 1U), 0xFFFFU);
  register_set(&(SAFETY_TASK_TIMER->DIER), TIM_DIER_UIE, 0x5F5FU);
  register_set(&(SAFETY_TASK_TIMER->CR1), TIM_CR1_CEN, 0x3FU);
  SAFETY_TASK_TIMER->SR = 0;
  NVIC_EnableIRQ(SAFETY_TASK_TIMER_IRQ);
}

void tick_timer_init(void) {
  timer_init(TICK_TIMER, (uint16_t)((15.25*APB2_TIMER_FREQ)/8U));
  NVIC_EnableIRQ(TICK_TIMER_IRQ);
}
