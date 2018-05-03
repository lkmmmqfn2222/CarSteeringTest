/************************************************************************/
/* By Cairne 2018 04 28                                                 */
/************************************************************************/

#include "Peripheral.h"	  
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#define TEST1 1
#define TEST2 0

#if TEST1

/*-------------------------------  RCC_APB_INIT  ------------------------*/

void RCC_APB_Init(void)
{
	//USART1 RCC DAC RCC--PA4 PA5
	USART_DeInit(USART1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1, ENABLE);
	//ADC RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	//USART3 RCC  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

}

/*-------------------------------  USART PART  --------------------------*/

void MyUART1Init(u32 bound)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

void MyUART3Init(u32 bound)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);; 
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

/*----------------------------- ADC PART  ---------------------------------*/
void MyADC1Init()
{
	ADC_InitTypeDef ADC_InitStructure;

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//chanel1 and chanel2 are independent
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//single converse model
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);
	//ADC CALIBRATION
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));//wait for the calibration status reset
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
}
/*----------------------------- MyDACInit --------------------------------*/
void MyDACInit()
{
	DAC_InitTypeDef DAC_InitStructure;
	//DAC_CHANNEL1
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1023;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	//DAC_CNANNEL2
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1023;
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);
	//ENABLE DAC
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Cmd(DAC_Channel_2, ENABLE);
	//INITIAL THE VALUE OF DAC
	DAC_SetDualChannelData(DAC_Align_12b_R, 0x100, 0x100);
}
/*-----------------------------  TIMER Init ------------------------------*/
void TimerInit()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructInit;
	TIM_TimeBaseStructInit.TIM_Prescaler = TIM4_PRESCALER_VALUE;
	TIM_TimeBaseStructInit.TIM_CounterMode =TIM_CounterMode_Up ;
	TIM_TimeBaseStructInit.TIM_Period = TIM4_PERIOD_TIMING;
	TIM_TimeBaseStructInit.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructInit);
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
	//TIM ENABLE
	TIM_Cmd(TIM2, ENABLE);
}

/*-----------------------------  NVIC PART -------------------------------*/

void NVIC_Configration_Peri(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//UASRT1_IRQInit
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//UART3_IRQInit
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //使能串口2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);


}
/*-------------------------------  GPIO_INIT     ------------------------*/

void PeriphGPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//UART GPIO INIT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//ADC GPIO INIT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART3 GPIO INIT  (RS485)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//PB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//DACOut GPIO Init
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*----------------------------  IRQHandler  -----------------------------*/
u8 USART_RX_BUF[USART_REC_LEN] = {0};
u16 USART_RX_STA = 0;

void USART1_IRQHandler(void)
{
	u8 Res;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Res = USART_ReceiveData(USART1);

		if ((USART_RX_STA & 0x8000) == 0)
		{
			if (USART_RX_STA & 0x4000)
			{
				if (Res != 0x0a)
					USART_RX_STA = 0;
				else
					USART_RX_STA |= 0x8000;
			}
			else
			{
				if (Res == 0x0d)
					USART_RX_STA |= 0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = Res;
					USART_RX_STA++;
					if (USART_RX_STA > (RS485_REC_LEN - 1))
						USART_RX_STA = 0;
				}
			}
		}
	}
}


u8 RS485_RX_BUF[RS485_REC_LEN] = {0};
u16 RS485_RX_STA=0;
u8	RS485_RX_CNT=0;

void USART3_IRQHandler(void)
{
	u8 res;

	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
	{

		res = USART_ReceiveData(USART3); 	//读取接收到的数据
		if (RS485_RX_CNT < 64)
		{
			RS485_RX_BUF[RS485_RX_CNT] = res;		//记录接收到的值
			RS485_RX_CNT++;						//接收数据增加1 
		}
	}
}

void Perpheral_Init(u32 uart1bound,u32 uart3bound)
{
	RCC_APB_Init();	
	NVIC_Configration_Peri();
	PeriphGPIO_Init();
	MyUART1Init(uart1bound);
	MyUART3Init(uart3bound);
	TimerInit();
	MyDACInit();
	MyADC1Init();
}
#endif

