#include <stdlib.h>
#include <stdint.h>

static volatile uint64_t time_ns;

// ChibiOS specific begin
#include <ch.h>
#include <hal.h>
#include "timestamp.h"

// settings
#include <timestamp_stm32_settings.h>

static inline uint32_t timer_read(void)
{
    return TIMER_REG->CNT;
}


CH_FAST_IRQ_HANDLER(TIMER_IRQ_NAME)
{
    TIMER_REG->SR &= ~STM32_TIM_SR_UIF; // clear interrupt flag
    time_ns += (((uint64_t)COUNTER_MAX+1) * TICK_PERIOD_NS);
    nvicClearPending(NVIC_NB); // sometimes the interrupt runs twice, not sure why. This solves it
}

void timestamp_stm32_init(void)
{
    time_ns = 0;
    RCC_EN();
    RCC_RESET();
    nvicEnableVector(NVIC_NB, INTERRUPT_PRIO);
    TIMER_REG->ARR = COUNTER_MAX;
    TIMER_REG->PSC = PRESCALER;
    TIMER_REG->DIER = STM32_TIM_DIER_UIE; // enable update interrupt
    TIMER_REG->CR1 |= STM32_TIM_CR1_CEN; // enable timer
}
// ChibiOS specific end


timestamp_t timestamp_get()
{
    nvicDisableVector(NVIC_NB);
    uint64_t c = time_ns;
    uint32_t tim = timer_read();
    bool o = TIMER_REG->SR & STM32_TIM_SR_UIF;
    uint32_t tim2 = timer_read();
    nvicEnableVector(NVIC_NB, INTERRUPT_PRIO);
    if (!o) {
        return c + (uint64_t)tim * TICK_PERIOD_NS;
    } else {
        return c + ((uint64_t)COUNTER_MAX * TICK_PERIOD_NS) + (uint64_t)tim2 * TICK_PERIOD_NS;
    }
}
