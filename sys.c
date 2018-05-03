/*----------------------------------------------------------------------*/
/* By Cairne 2018-04-28                                                 */
/*----------------------------------------------------------------------*/

#include "sys.h"
#include "misc.h"
#include <stdarg.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include "stm32f10x_usart.h"

/*----------------------	delay function	----------------------------*/

static u8  fac_us = 0;//us Plus faction of delay us
static u16 fac_ms = 0;//ms Plus faction of delay ms


void delay_init()
{

#ifdef OS_CRITICAL_METHOD 	//if OS_CRITICAL_METHOD id defined 
	u32 reload;				//it means that ucosII will be used
#endif
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//  HCLK/8
	fac_us = SystemCoreClock / 8000000;	//1/8 of the system clock  

#ifdef OS_CRITICAL_METHOD 	//���OS_CRITICAL_METHOD������,˵��ʹ��ucosII��.
	reload = SystemCoreClock / 8000000;		//ÿ���ӵļ������� ��λΪK	   
	reload *= 1000000 / OS_TICKS_PER_SEC;//����OS_TICKS_PER_SEC�趨���ʱ��
	//reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��1.86s����	
	fac_ms = 1000 / OS_TICKS_PER_SEC;//����ucos������ʱ�����ٵ�λ	   
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;   	//����SYSTICK�ж�
	SysTick->LOAD = reload; 	//ÿ1/OS_TICKS_PER_SEC���ж�һ��	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   	//����SYSTICK    
#else
	fac_ms = (u16)fac_us * 1000;// if not use ucos, every ms comprise of systick   
#endif
}


void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD = nus*fac_us; //load time to delay	  		 
	SysTick->VAL = 0x00;        //clear the counter
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;          //begin
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp&(1 << 16)));//waiting
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //disable the counter
	SysTick->VAL = 0X00;       //clear the counter	 
}

void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD = (u32)nms*fac_ms;//load time (SysTick->LOADΪ24bit)
	SysTick->VAL = 0x00;           //clear the counter
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;          //begin  
	do
	{
		temp = SysTick->CTRL;
	} while (temp & 0x01 && !(temp&(1 << 16)));//waiting   
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //disable the counter
	SysTick->VAL = 0X00;       //clear the counter  	    
}

/*--------------------------  MY PRINTF -----------------------------*/

void myPrintf(char* fomat, ...)
{
	char buf[1000] = "";
	char* p;
	u16 i = 0;
	int num;
	float fnum;
	char* str;
	char tem[100] = "";
	va_list va;
	va_start(va, fomat);
	p = fomat;
	while (*p)
	{
		if (*p == '%')
		{
			switch (*(++p))
			{
			case 'd':
				num = va_arg(va, int);
				itoa(num, tem, 10);
				strcpy(buf + i, tem);
				i += strlen(tem) - 1;
				break;
			case 's':
				str = va_arg(va, char*);
				strcpy(buf + i, str);
				i += strlen(str) - 1;
				break;
			case 'f':
				fnum = va_arg(va, double);
				gcvt(fnum, 8, tem);
				strcpy(buf + i, tem);
				i += strlen(tem) - 1;
				break;
			}
		}
		else
		{
			buf[i] = *p;
		}
		i++;
		p++;
	}
	p = buf;
	while (*p)
	{
		while ((USART1->SR & 0X40) == 0);//wait for transmission complete
		USART1->DR = (u8)*p;
		p++;
	}
	va_end(va);
}
