/************************************************************************/
/* By Cairne 2018 04 28                                                 */
/************************************************************************/

#include "stm32f10x_usart.h"
#include "rs485test.h"
#include "Peripheral.h"


void RS485SendData(u8 *buf, u8 len)
{
	u8 t;
	RS485_TX_EN = 1;//send model
	for (t = 0; t < len;t++)
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC));
		USART_SendData(USART3, buf[t]);
	}
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC));//end of sending
	RS485_RX_CNT = 0;//waiting for receiving
	RS485_TX_EN = 0;//receiving model
}

void RS485RevData(u8 *buf, u8 *len)
{
	u8 rxlen = RS485_RX_CNT;
	u8 i;
	*len = 0;
	delay_us(10);
	if (rxlen==RS485_RX_CNT&&rxlen)
	{
		for (i = 0; i < rxlen;i++)
		{
			buf[i] = RS485_RX_BUF[i];
		}
		*len = RS485_RX_CNT;
		RS485_RX_CNT = 0;
	}
}

void RS485TestProgram(void)
{
	RS485_TX_EN = 0;
	u8 bgn = 0;
	u8 TLength = 0;
	if (!bgn)
	{
		delay_ms(1000);
		char *TestText = "RS485 Test,Please Key In Any Word End With Enter\r\n";
		delay_ms(10);
		TLength = strlen(TestText);
		RS485SendData(TestText, TLength);
		bgn += 1;
	}
	else
	{
		RS485SendData(RS485_RX_BUF, RS485_RX_CNT);
		delay_ms(1000);
	}
}