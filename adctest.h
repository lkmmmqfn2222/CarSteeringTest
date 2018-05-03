#ifndef _ADC_TEST_H
#define _ADC_TEST_H

#include "stm32f10x.h"


u16		Get_ADC(u8 ch);
void	Adc_Init(void);
u16		Get_ADC_Average(u8 ch, u8 times);

typedef struct 
{
	float adc_ch0;
	float adc_ch1;
	float adc_ch2;
	float adc_ch3;
}ADCValueStruct;

void ADCTestProgram(ADCValueStruct ADCValueS1);

#endif // !_ADC_TEST_H
