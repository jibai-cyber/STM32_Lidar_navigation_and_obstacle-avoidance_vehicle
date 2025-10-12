#include "stm32f10x.h"                 
#include "Motor.h"
#include "Delay.h"
#include "Control.h"
extern uint8_t fflag;
extern uint8_t Judge;
/*
2进1出为反转
*/
void Stop(void)//小车停转
{
	TIM_SetCompare2(TIM2,0);
	TIM_SetCompare3(TIM2,0);
	Backward();
	Delay_ms(10);
	TIM_SetCompare1(TIM2,0);
	TIM_SetCompare4(TIM2,0);
}
void Backward(void)//小车两个电机反转
{
	TIM_SetCompare1(TIM2,18000);  //PA0
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);//PA1
	TIM_SetCompare4(TIM2,18000);  //PA3
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);//PA2
}
void Forward(void)//小车两个电机正转
{
	TIM_SetCompare2(TIM2,7200);  //PA1
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);//PA0
	TIM_SetCompare3(TIM2,7200);  //PA2
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);//PA3
}

void Forward_TB(uint16_t Vl,uint16_t Vr)
{
    TIM_SetCompare3(TIM2,Vl);  //PA2
    GPIO_SetBits(GPIOD,GPIO_Pin_11);    //BIN1
    GPIO_ResetBits(GPIOD,GPIO_Pin_10);  //BIN2
    TIM_SetCompare2(TIM2,Vr);  //PA1
    GPIO_SetBits(GPIOD,GPIO_Pin_9);     //AIN1
    GPIO_ResetBits(GPIOD,GPIO_Pin_8);   //AIN2
}

void Backward_TB(void)
{
    TIM_SetCompare2(TIM2,10000);  //PA1
    GPIO_SetBits(GPIOD,GPIO_Pin_8);     //AIN2
    GPIO_ResetBits(GPIOD,GPIO_Pin_9);   //AIN1
    TIM_SetCompare3(TIM2,10000);  //PA2
    GPIO_SetBits(GPIOD,GPIO_Pin_10);    //BIN2
    GPIO_ResetBits(GPIOD,GPIO_Pin_11);  //BIN1
}

void Stop_TB(void)
{
    GPIO_ResetBits(GPIOD,GPIO_Pin_8);
    GPIO_ResetBits(GPIOD,GPIO_Pin_9);
    GPIO_ResetBits(GPIOD,GPIO_Pin_10);
    GPIO_ResetBits(GPIOD,GPIO_Pin_11);
}

void Turn_left(uint16_t Vl,uint16_t Vr)
{
    TIM_SetCompare3(TIM2,Vl);  //PA2
    GPIO_SetBits(GPIOD,GPIO_Pin_10);    //BIN2
    GPIO_ResetBits(GPIOD,GPIO_Pin_11);  //BIN1
    TIM_SetCompare2(TIM2,Vr);  //PA1
    GPIO_SetBits(GPIOD,GPIO_Pin_9);     //AIN1
    GPIO_ResetBits(GPIOD,GPIO_Pin_8);   //AIN2
}

void Turn_right(uint16_t Vl,uint16_t Vr)
{
    TIM_SetCompare3(TIM2,Vl);  //PA2
    GPIO_SetBits(GPIOD,GPIO_Pin_11);    //BIN1
    GPIO_ResetBits(GPIOD,GPIO_Pin_10);  //BIN2
    TIM_SetCompare2(TIM2,Vr);  //PA1
    GPIO_SetBits(GPIOD,GPIO_Pin_8);     //AIN2
    GPIO_ResetBits(GPIOD,GPIO_Pin_9);   //AIN1
}

void TIM2_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);    

	TIM_TimeBaseStructure.TIM_Period = arr;//; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值 2400
	TIM_TimeBaseStructure.TIM_Prescaler =psc;//设置用来作为TIMx时钟频率除数的预分频值 3
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的相应模式配置

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse =0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM2, &TIM_OCInitStructure); //根据TIM_OCInitStruct中指定的参数初始化TIM2_CH1
	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM2, &TIM_OCInitStructure); 

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //CH1预装载使能	 	
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);   
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);   	
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);   

	TIM_ARRPreloadConfig(TIM2, ENABLE); //使能TIM2在ARR上的预装载寄存器
		
  TIM_Cmd(TIM2, ENABLE);  //开启TIM2时钟

}
