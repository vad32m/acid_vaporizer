/*
 * load_control.c
 *
 *  Created on: Apr 4, 2021
 *      Author: vadym.mishchuk
 */

#include "load_control.h"

#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_gpio.h"

#include <stddef.h>

static void (*cycleEndedCallback)(void) = NULL;

void TIM3_IRQHandler(void)
{
	if (LL_TIM_IsActiveFlag_UPDATE(TIM3))
	{
		if (cycleEndedCallback)
		{
			cycleEndedCallback();
		}

		LL_TIM_ClearFlag_UPDATE(TIM3);
	}
}

void LOAD_CONTROL_init(void)
{
	//16Mhz / 64000 = 250 Hz
	LL_TIM_SetPrescaler(TIM3, 64000);
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);

	//counter gets reset each second
	LL_TIM_SetAutoReload(TIM3, 250 - 1);

	LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetCompareCH4(TIM3, 0);

	LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);

	LL_TIM_EnableIT_UPDATE(TIM3);
	NVIC_EnableIRQ(TIM3_IRQn);

	LL_TIM_EnableCounter(TIM3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
}

void LOAD_CONTROL_setLoad(uint8_t loadPercent)
{
	if (loadPercent > 100)
	{
		loadPercent = 100;
	}

	uint16_t compareVal = loadPercent * 5;
	compareVal = compareVal / 2;
	LL_TIM_OC_SetCompareCH4(TIM3, compareVal);
}

void LOAD_CONTROL_setCycleEndedCallback(void (*callback)(void))
{
	cycleEndedCallback = callback;
}
