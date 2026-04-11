#ifndef __PIN_DEFS_H
#define __PIN_DEFS_H

#include "stm32f4xx_hal.h"

#define PARK_SENSE_PORT GPIOA
#define PARK_SENSE_PIN  GPIO_PIN_8

#define FAST_INH_PORT   GPIOA
#define FAST_INH_PIN    GPIO_PIN_5

#define SLOW_INH_PORT   GPIOA
#define SLOW_INH_PIN    GPIO_PIN_6

#define SLOW_PWM_PORT   GPIOA
#define SLOW_PWM_PIN    GPIO_PIN_2

#define FAST_PWM_PORT   GPIOA
#define FAST_PWM_PIN    GPIO_PIN_3

#endif /* __PIN_DEFS_H */
