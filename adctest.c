/************************************************************************/
/* by Cairne 2014 04 30                                                 */
/************************************************************************/
#include "stm32f10x_adc.h"
#include "sys.h"
#include "adctest.h"

u16 GetADCValue(u8 ch)
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_1Cycles5);//setting for the channel you used
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//Enables or disables the selected ADC software start conversion
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//
	return ADC_GetConversionValue(ADC1);
}

u16 GetEverageValue(u8 ch, u8 times)
{
	u32 TempValue = 0;
	u8 t;
	for (t = 0; t < times;t++)
	{
		TempValue += GetADCValue(ch);
		delay_ms(5);
	}
	return TempValue / times;
}

ADCValueStruct ADC1ValueInit(ADCValueStruct ADC1Value)
{
	ADC1Value.adc_ch0 = (float)(GetADCValue(0)*(3.3 / 4096));
	ADC1Value.adc_ch1 = (float)(GetADCValue(1)*(3.3 / 4096));
	ADC1Value.adc_ch2 = (float)(GetADCValue(2)*(3.3 / 4096));
	ADC1Value.adc_ch3 = (float)(GetADCValue(3)*(3.3 / 4096));
	return ADC1Value;
}

void ADCValuePrint(ADCValueStruct *ADC1ValuePrint)
{
	u8 i;
	float adcvalue[] =
	{
		ADC1ValuePrint->adc_ch0,
		ADC1ValuePrint->adc_ch1,
		ADC1ValuePrint->adc_ch2,
		ADC1ValuePrint->adc_ch3
	};
	for (i = 0; i < sizeof(ADC1ValuePrint); i++)
	{
		myPrintf("The ADC %d value is %f \r\n", i, ADC1ValuePrint[i]);
		delay_ms(20);
	}
}

void ADCTestProgram(ADCValueStruct ADCValueS1)
{
	u16 adc1;
	ADCValueStruct ADCTempValue;
	ADCTempValue = ADC1ValueInit(ADCValueS1);
	ADCValuePrint(&ADCTempValue);
}
