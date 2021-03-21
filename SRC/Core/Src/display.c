/*
 * display.c
 *
 *  Created on: Mar 14, 2021
 *      Author: vadym.mishchuk
 */
//		 A
//	  _______
//   |       |
// F |       | B
//	 |  G    |
//	 ---------
//	 |       |
// E |       | C
//	 |       |
//	 ---------    ----
//		D        | DP |
//                ----

#include <stdint.h>
#include <stdbool.h>

#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_tim.h"

#define DIGITS 3

#define SEG_A  0x80
#define SEG_B  0x40
#define SEG_C  0x20
#define SEG_D  0x10
#define SEG_E  0x08
#define SEG_F  0x04
#define SEG_G  0x02
#define SEG_DP 0x01

#define DIGIT_0  (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F)
#define DIGIT_1  (SEG_B | SEG_C)
#define DIGIT_2  (SEG_A | SEG_B | SEG_G | SEG_E | SEG_D)
#define DIGIT_3  (SEG_A | SEG_B | SEG_G | SEG_C | SEG_D)
#define DIGIT_4  (SEG_F | SEG_B | SEG_G | SEG_C)
#define DIGIT_5  (SEG_A | SEG_F | SEG_G | SEG_C | SEG_D)
#define DIGIT_6  (SEG_A | SEG_F | SEG_G | SEG_C | SEG_D | SEG_E)
#define DIGIT_7  (SEG_A | SEG_B | SEG_C)
#define DIGIT_8  (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G)
#define DIGIT_9  (SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G)

#define SYMBOL_E  (SEG_A | SEG_D | SEG_E | SEG_F | SEG_G)
#define SYMBOL_r  (SEG_E | SEG_G)

static const uint8_t digitBitmaps[] = {
		DIGIT_0,
		DIGIT_1,
		DIGIT_2,
		DIGIT_3,
		DIGIT_4,
		DIGIT_5,
		DIGIT_6,
		DIGIT_7,
		DIGIT_8,
		DIGIT_9};

static const uint32_t digitEnablePins[DIGITS] = {
		LL_GPIO_PIN_6,
		LL_GPIO_PIN_9,
		LL_GPIO_PIN_10
};

static uint8_t digits[DIGITS];

static uint8_t currentDigitIdx = 0;

static bool isBlinking = false;

void TIM14_IRQHandler(void)
{
	static bool displayOn;
	static uint8_t ticks;

	LL_GPIO_ResetOutputPin(GPIOA, digitEnablePins[currentDigitIdx]);

	//blink once in a second
	ticks = (ticks + 1) % 250;

	//duty cycle is 50% - 0.5 sec on and 0.5 sec off
	if (ticks == 0) {
		displayOn = true;
	} else if (ticks == 125) {
		displayOn = false;
	}

	if (!isBlinking || displayOn)
	{
		currentDigitIdx = (currentDigitIdx + 1) % DIGITS;
		LL_SPI_TransmitData8(SPI1, digits[currentDigitIdx]);
		LL_GPIO_SetOutputPin(GPIOA, digitEnablePins[currentDigitIdx]);
	}

	LL_TIM_ClearFlag_UPDATE(TIM14);
}

void DISPLAY_Init(void)
{
	LL_SPI_Enable(SPI1);
	LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV8);

	LL_TIM_SetClockSource(TIM14, LL_TIM_CLOCKSOURCE_INTERNAL);
	//timer will be counting on 10 kHz frequency
	LL_TIM_SetPrescaler(TIM14, 1600 - 1);
	//timer IRQ will be generated each 4 ms (250 kHz frequency)
	LL_TIM_SetAutoReload(TIM14, 40 - 1);
	LL_TIM_SetCounterMode(TIM14, LL_TIM_COUNTERMODE_UP);
	LL_TIM_EnableUpdateEvent(TIM14);
	LL_TIM_EnableIT_UPDATE(TIM14);
	LL_TIM_EnableCounter(TIM14);

	NVIC_EnableIRQ(TIM14_IRQn);

}

void DISPLAY_SetNumber(uint16_t number)
{
	if (number > 999)
	{
		number = 999;
	}

	for (uint8_t Idx = 0; Idx < DIGITS; Idx++)
	{
		uint8_t CurrentDigit = number % 10;
		digits[Idx] = digitBitmaps[CurrentDigit];
		number = number / 10;
	}
}

void DISPLAY_SetError(void)
{
	digits[2] = SYMBOL_E;
	digits[1] = SYMBOL_r;
	digits[0] = SYMBOL_r;
}

void DISPLAY_StartBlinking(void)
{
	isBlinking = true;
}

void DISPLAY_StopBlinking(void)
{
	isBlinking = false;
}
