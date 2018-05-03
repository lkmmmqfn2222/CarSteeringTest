/************************************************************************/
/* By Cairne 2018 04 28                                                 */
/************************************************************************/

#ifndef _PERIPHERAL_H
#define _PERIPHERAL_H

#include "stm32f10x.h"

#define TIM4_PERIOD_TIMING	(10-1)
#define TIM4_COUNTER_CLOCK   1000000
#define TIM4_PRESCALER_VALUE (SystemCoreClock/TIM4_COUNTER_CLOCK)	

#define  USART_REC_LEN 200
#define  RS485_REC_LEN 64
extern u8 USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;

extern u8 RS485_RX_BUF[RS485_REC_LEN];
extern u16 RS485_RX_STA;
extern u8 RS485_RX_CNT;//not use now

void Perpheral_Init(u32 uart1bound, u32 uart3bound);


#endif