#include "stm32f10x.h"
#include "Speed.h"
int16_t Speed_left=0,Speed_right=0;
void TIM5_Init()    //定时50ms
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //中断优先级的结构体定义

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //TIM2的时钟使能

	TIM_TimeBaseStructure.TIM_Period = 7199*5; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	7199*5
	TIM_TimeBaseStructure.TIM_Prescaler =99; //设置用来作为TIMx时钟频率除数的预分频值 999
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的相应的模式配置
 
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  //使能TIM3中断
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //TIM3中断服务函数的配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;  //主优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能IRQ中断通道
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化NVIC寄存器

	TIM_Cmd(TIM5, ENABLE);  //开启TIM2
}
void Encoder_Init_TIM4(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
  TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  TIM_ICInit(TIM4, &TIM_ICInitStructure);//根据 TIM_ICInitStruct 的参数初始化外设	TIMx
	
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	
 
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
 
  TIM_SetCounter(TIM4,0);//设置TIMx 计数器寄存器值
  TIM_Cmd(TIM4, ENABLE); //使能定时器
}
void Encoder_Init_TIM1(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//使能高级定时器1的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);//使能PE端口时钟           //注意！！！！这里是全映射！ GPIOE GPIO_Pin_7 到12都不可占用
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_9;	//端口配置   PA8->CH1 PA9->CH2          映射后PA8->PE9 PA9->PE11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA
  
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);  //开启重映射！！！！！！！！！！
    
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上向下计数  
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复计数器为0
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
  TIM_ICStructInit(&TIM_ICInitStructure); //把TIM_ICInitStruct 中的每一个参数按缺省值填入
  TIM_ICInitStructure.TIM_ICFilter = 10;  //设置滤波器长度
  TIM_ICInit(TIM1, &TIM_ICInitStructure);//根据 TIM_ICInitStruct 的参数初始化外设	TIMx
	
  TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
 
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清除TIM的更新标志位
 
  TIM_SetCounter(TIM1,0);//设置TIMx 计数器寄存器值
  TIM_Cmd(TIM1, ENABLE); //使能定时器
}
void Serial_SendByte(int16_t Byte)  //这里改成了有符号位
{
	USART_SendData(USART2, (Byte&0xFF00)>>8);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, Byte&0x00FF);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
}
void TIM5_IRQHandler(void)   //TIM5中断服务函数
{
	if(TIM5->SR&0X0001)//定时器每500ms定时中断一次
	{   
        TIM5->SR&=~(1<<0);//清除定时器二的计时标志，执行完TIM3中断服务函数里的操作后定时器会重新计时
        Speed_right = -Read_Encoder(1)/1.3f;
        Speed_left = -Read_Encoder(4)/1.3f;   //转换成原来编码器的数值运算，注意除法要加小数点，否则得0
//        Serial_SendByte(Speed_left);
//        USART_SendData(USART2, 0xAA);
//        while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
//        Serial_SendByte(Speed_right);
//        USART_SendData(USART2, 0xBB);
//        while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
    }
}

int16_t Read_Encoder(u8 TIMX)   //这里改成有符号位，注意数据范围
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
