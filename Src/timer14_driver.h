/*
 * timer14_driver.h
 *
 *  Created on: 27 Jun 2018
 *      Author: Maor
 */

#ifndef TIMER14_DRIVER_H_
#define TIMER14_DRIVER_H_

#include "stm32f0xx.h"
#include <stdint.h>


#define _TIMER14_START() (TIM14->CR1) |= TIM_CR1_CEN;
#define _TIMER14_STOP()  (TIM14->CR1) &= ~TIM_CR1_CEN;

#define _TIMER14_SET_COMPARE(val) (TIM14->CCR1) = val;
#define _TIMER14_SET_COUNTER(val) (TIM14->CNT) = val;
#define _TIMER14_CLEAR_IRQ() (TIM14->SR) &= ~TIM_SR_CC1IF;



#endif /* TIMER14_DRIVER_H_ */
