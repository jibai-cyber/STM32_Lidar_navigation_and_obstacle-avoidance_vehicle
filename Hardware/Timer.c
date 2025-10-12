#include "stm32f10x.h"
uint8_t count = 0;
void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);              //RCC开启
	
	TIM_InternalClockConfig(TIM2);                                   //时钟选择-内部时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;               //时基单元配置
	TIM_TimeBaseInitStructure.TIM_ClockDivision =TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                       //更新中断开启
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                  //NVIC通道开启
	NVIC_InitTypeDef NVIC_Initstructure;
	NVIC_Initstructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Initstructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_Initstructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_Initstructure);
	
	TIM_Cmd(TIM2, ENABLE);																					 //开启时钟
}
/*void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}
}*/
