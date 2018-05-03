/************************************************************************/
/*  1:MyPrintf function realization										*/
/*	2:RS485 function realization										*/
/*	3:ADC,DAC Triggered by tim2											*/
/*	4:Utilization of DMA												*/
/*	By Cairne 2018-04-28												*/	
/************************************************************************/
#include "Peripheral.h"
#include "sys.h"
#include "adctest.h"
#include "rs485test.h"

void main(void)
{
	delay_init();
	Perpheral_Init(115200,115200);
	ADCValueStruct ADCValueSt1;
	while (1)
	{
		ADCTestProgram(ADCValueSt1);
		RS485TestProgram();
		delay_ms(500);
	}
}