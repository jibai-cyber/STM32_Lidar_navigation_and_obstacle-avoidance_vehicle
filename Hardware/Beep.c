#include "stm32f10x.h"
#include "Delay.h"
#include "LED.h"
uint8_t count = 0;
void Beep_Init(void)
{
/*------------------Beep init--------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
/*------------------interrupt---------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//选用PB0作为中断触发，默认高电平
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line =EXTI_Line0;
	EXTI_InitStructure.EXTI_LineCmd =ENABLE;
	EXTI_InitStructure.EXTI_Mode =EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger =EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel =EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_Init(&NVIC_InitStructure);
}

/*void Beep_On(void)
{
	uint16_t time = 40;
	while(time--)
	{
		GPIO_WriteBit(GPIOB,GPIO_Pin_8,Bit_RESET);
		Delay_ms(10);
		GPIO_WriteBit(GPIOB,GPIO_Pin_8,Bit_SET);
		Delay_ms(10);
	}
}*/
void EXTI0_IRQHandler(void)
{
	count++;
	if(count == 100){count = 0;LEDgreen(1);}
	EXTI_ClearITPendingBit(EXTI_Line0);
}
