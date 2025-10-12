#include "stm32f10x.h"

void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);              //RCC开启
	
	TIM_InternalClockConfig(TIM2);                                   //时钟选择-内部时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;               //时基单元配置
	TIM_TimeBaseInitStructure.TIM_ClockDivision =TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;   								 //ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;								 //PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;                	 //选择输出比较模式为PWM1
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;           //设置有效电平为1
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;       //输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;           //CCR
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);             //OC1复用GPIOA0
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_Cmd(TIM2, ENABLE);                                     			 //定时器使能
}
