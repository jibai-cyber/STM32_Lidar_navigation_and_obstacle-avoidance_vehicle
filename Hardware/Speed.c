#include "stm32f10x.h"
#include "Speed.h"
int16_t Speed_left=0,Speed_right=0;
void TIM5_Init()    //��ʱ50ms
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //�ж����ȼ��Ľṹ�嶨��

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //TIM2��ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = 7199*5; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	7199*5
	TIM_TimeBaseStructure.TIM_Prescaler =99; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 999
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx����Ӧ��ģʽ����
 
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  //ʹ��TIM3�ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM3�жϷ�����������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ��IRQ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM5, ENABLE);  //����TIM2
}
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��PB�˿�ʱ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_Period = 65535; //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
  TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
  TIM_ICInit(TIM4, &TIM_ICInitStructure);//���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx
	
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	
 
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
 
  TIM_SetCounter(TIM4,0);//����TIMx �������Ĵ���ֵ
  TIM_Cmd(TIM4, ENABLE); //ʹ�ܶ�ʱ��
}
void Encoder_Init_TIM1(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//ʹ�ܸ߼���ʱ��1��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);//ʹ��PE�˿�ʱ��           //ע�⣡������������ȫӳ�䣡 GPIOE GPIO_Pin_7 ��12������ռ��
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_9;	//�˿�����   PA8->CH1 PA9->CH2          ӳ���PA8->PE9 PA9->PE11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA
  
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);  //������ӳ�䣡������������������
    
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
  TIM_TimeBaseStructure.TIM_Period = 65535; //�趨�������Զ���װֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM�������¼���  
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//�ظ�������Ϊ0
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
  TIM_ICStructInit(&TIM_ICInitStructure); //��TIM_ICInitStruct �е�ÿһ��������ȱʡֵ����
  TIM_ICInitStructure.TIM_ICFilter = 10;  //�����˲�������
  TIM_ICInit(TIM1, &TIM_ICInitStructure);//���� TIM_ICInitStruct �Ĳ�����ʼ������	TIMx
	
  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
 
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
 
  TIM_SetCounter(TIM1,0);//����TIMx �������Ĵ���ֵ
  TIM_Cmd(TIM1, ENABLE); //ʹ�ܶ�ʱ��
}
void Serial_SendByte(int16_t Byte)  //����ĳ����з���λ
{
	USART_SendData(USART2, (Byte&0xFF00)>>8);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, Byte&0x00FF);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
}
void TIM5_IRQHandler(void)   //TIM5�жϷ�����
{
	if(TIM5->SR&0X0001)//��ʱ��ÿ500ms��ʱ�ж�һ��
	{   
        TIM5->SR&=~(1<<0);//�����ʱ�����ļ�ʱ��־��ִ����TIM3�жϷ�������Ĳ�����ʱ�������¼�ʱ
        Speed_right = -Read_Encoder(1)/1.3f;
        Speed_left = -Read_Encoder(4)/1.3f;   //ת����ԭ������������ֵ���㣬ע�����Ҫ��С���㣬�����0
//        Serial_SendByte(Speed_left);
//        USART_SendData(USART2, 0xAA);
//        while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
//        Serial_SendByte(Speed_right);
//        USART_SendData(USART2, 0xBB);
//        while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
    }
}

int16_t Read_Encoder(u8 TIMX)   //����ĳ��з���λ��ע�����ݷ�Χ
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
		 case 1:  Encoder_TIM=TIM1 -> CNT; TIM1 -> CNT = 0;break;		
	   case 4:  Encoder_TIM=TIM4 -> CNT; TIM4 -> CNT = 0;break;	
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
