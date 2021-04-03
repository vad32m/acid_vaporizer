/*
 * sensorc.c
 *
 *  Created on: Mar 28, 2021
 *      Author: vadym.mishchuk
 */

#include <sensors.h>
#include <stm32f0xx_ll_adc.h>

#define SAMPLE_COUNT 32

typedef struct
{
	uint32_t samples[SAMPLE_COUNT];
	uint8_t sampleIdx;
} ADC_ChannelData;

ADC_ChannelData channels[5];

uint8_t channelIdx;


void ADC1_IRQHandler(void)
{
	if (LL_ADC_IsActiveFlag_EOC(ADC1))
	{
		channels[channelIdx].samples[channels[channelIdx].sampleIdx] = LL_ADC_REG_ReadConversionData32(ADC1);
		channels[channelIdx].sampleIdx++;
		channels[channelIdx].sampleIdx %= SAMPLE_COUNT;
		channelIdx++;
		LL_ADC_ClearFlag_EOC(ADC1);
	}

	if (LL_ADC_IsActiveFlag_EOS(ADC1))
	{
		channelIdx = 0;
		LL_ADC_ClearFlag_EOS(ADC1);
	}
}

void SENSORS_init(void)
{

	LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV4);
	LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

	LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
	//at thus point 4 MHz clock is fed into ADC

	LL_ADC_Enable(ADC1);
	LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);
	//samples are converted at approximately 16kHz frequency

	//TODO: calibrate
	//LL_ADC_StartCalibration(ADC1);

	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);

	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_1);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_2);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_3);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_VREFINT);

	LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_VREFINT);

	LL_ADC_EnableIT_EOC(ADC1);
	LL_ADC_EnableIT_EOS(ADC1);

	NVIC_EnableIRQ(ADC1_IRQn);

	LL_ADC_REG_StartConversion(ADC1);
}


SENSORS_Readings SENSORS_getReadings(void)
{
	SENSORS_Readings readings;
	uint32_t coldJuction = 0;
	uint32_t thermoCouple = 0;
	uint32_t potentiometer = 0;
	uint32_t vdd = 0;


	for (uint8_t i = 0; i < SAMPLE_COUNT; i++)
	{
		vdd = VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR);
		vdd = vdd / channels[4].samples[i];

		potentiometer += (vdd * channels[0].samples[i])/4095;

		thermoCouple += (vdd * channels[1].samples[i])/4095;

		coldJuction += (vdd * channels[2].samples[i])/4095;

		coldJuction += (vdd * channels[3].samples[i])/4095;
	}

	potentiometer /= SAMPLE_COUNT;
	thermoCouple /= SAMPLE_COUNT;
	coldJuction /= SAMPLE_COUNT;
	coldJuction /= 2;

	readings.coldJunctionTemp = coldJuction;
	readings.potentiometerAngle = potentiometer;
	readings.thermocoupleTemp = thermoCouple;

	return readings;
}
