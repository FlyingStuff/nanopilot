#ifndef TIMESTAMP_STM32_SETTINGS_H
#define TIMESTAMP_STM32_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

// settings
#define TIMESTAMP_TIMER TIM7
#define TIMER_REG       STM32_TIM7
#define TIMER_IRQ_NAME  STM32_TIM7_HANDLER
#define RCC_EN()        rccEnableTIM7(FALSE)
#define RCC_RESET()     rccResetTIM7()
#define NVIC_NB         STM32_TIM7_NUMBER

#define COUNTER_MAX     0xffff

// CK_CNT = CK_INT / (PSC[15:0] + 1)
#if STM32_PPRE1 == STM32_PPRE1_DIV1
#define PRESCALER       (STM32_PCLK1/2000000 - 1)
#else
#define PRESCALER       (2*STM32_PCLK1/2000000 - 1)
#endif
#define INTERRUPT_PRIO  2

#if (PRESCALER < 0)
// todo check for rounding
#error bad prescaler
#endif

#define TICK_PERIOD_NS 500

#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_STM32_SETTINGS_H */
