#include "stm32f10x.h"                 
#include "Motor.h"
#include "Delay.h"
#include "Control.h"
extern uint8_t fflag;
extern uint8_t Judge;
/*
2��1��Ϊ��ת
*/
void Stop(void)//С��ͣת
{
	TIM_SetCompare2(TIM2,0);
	TIM_SetCompare3(TIM2,0);
	Backward();
	Delay_ms(10);
	TIM_SetCompare1(TIM2,0);
	TIM_SetCompare4(TIM2,0);
}
void Backward(void)//С�����������ת
{
	TIM_SetCompare1(TIM2,18000);  //PA0
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);//PA1
	TIM_SetCompare4(TIM2,18000);  //PA3
	GPIO_ResetBits(GPIOA,GPIO_Pin_2);//PA2
}
void Forward(void)//С�����������ת
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
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�����������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);    

	TIM_TimeBaseStructure.TIM_Period = arr;//; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ 2400
	TIM_TimeBaseStructure.TIM_Prescaler =psc;//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 3
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx����Ӧģʽ����

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse =0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM2, &TIM_OCInitStructure); //����TIM_OCInitStruct��ָ���Ĳ�����ʼ��TIM2_CH1
	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM2, &TIM_OCInitStructure); 

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	 	
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);   
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);   	
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);   

	TIM_ARRPreloadConfig(TIM2, ENABLE); //ʹ��TIM2��ARR�ϵ�Ԥװ�ؼĴ���
		
  TIM_Cmd(TIM2, ENABLE);  //����TIM2ʱ��

}
