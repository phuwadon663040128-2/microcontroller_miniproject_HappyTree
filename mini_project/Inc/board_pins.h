#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#ifndef STM32F411xE
#define STM32F411xE
#endif

#include "stm32f4xx.h"

/* LED outputs */
#define LED1_GPIO GPIOA
#define LED2_GPIO GPIOA
#define LED3_GPIO GPIOA
#define LED4_GPIO GPIOB
#define LED1_PIN 5u
#define LED2_PIN 6u
#define LED3_PIN 7u
#define LED4_PIN 6u

/* 7-seg BCD: A,B,C,D pins */
#define SEG_A_GPIO GPIOC
#define SEG_A_PIN  7u   /* PC7 */
#define SEG_B_GPIO GPIOA
#define SEG_B_PIN  8u   /* PA8 */
#define SEG_C_GPIO GPIOB
#define SEG_C_PIN  10u  /* PB10 */
#define SEG_D_GPIO GPIOA
#define SEG_D_PIN  9u   /* PA9  */

/* Buttons (active-low, pull-ups) */
#define BTN1_GPIO GPIOA
#define BTN1_PIN  10u
#define BTN2_GPIO GPIOB
#define BTN2_PIN  3u
#define BTN3_GPIO GPIOB
#define BTN3_PIN  5u
#define BTN4_GPIO GPIOB
#define BTN4_PIN  4u

/* ADC channels */
#define CH_TEMP 0u // PA0
#define CH_LDR  1u // PA1
#define CH_SOIL 8u // PB0
#define CH_POT  4u // PA4

#define ADC_IDX_TEMP 0u
#define ADC_IDX_LDR  1u
#define ADC_IDX_SOIL 2u
#define ADC_IDX_POT  3u

#endif /* BOARD_PINS_H */
