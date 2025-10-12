#include "stm32f10x.h"
#include "Data_analyse.h"
#include "Usart.h"

void uart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_Initstructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 			//PB10复用TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//上拉或者浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			//PB11复用RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	USART_Initstructure.USART_BaudRate = 460800;																			//波特率设置
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //不开硬件流
	USART_Initstructure.USART_Mode = USART_Mode_Rx;							//选择仅接收模式
	USART_Initstructure.USART_Parity = USART_Parity_No; 														//没有校验位
	USART_Initstructure.USART_StopBits = USART_StopBits_1;													//1位停止位
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;											//8位数据位
	USART_Init(USART3,&USART_Initstructure); 
    
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
    
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断


    USART_Cmd(USART3, ENABLE);																//打开UART
    /******************************************************************************/
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 			//PB10复用TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//上拉或者浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			//PB11复用RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	
	USART_Initstructure.USART_BaudRate = 9600;																			//波特率设置
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //不开硬件流
	USART_Initstructure.USART_Mode = USART_Mode_Rx;							//选择仅接收模式
	USART_Initstructure.USART_Parity = USART_Parity_No; 														//没有校验位
	USART_Initstructure.USART_StopBits = USART_StopBits_1;													//1位停止位
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;											//8位数据位
	USART_Init(UART4,&USART_Initstructure); 
    
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
    
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启串口接受中断


    USART_Cmd(UART4, ENABLE);																//打开UART
    /******************************************************************************/

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);        //这里的引脚可能被编码器占用！！！！！！！
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 			//PB10复用TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//上拉或者浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			//PB11复用RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	USART_Initstructure.USART_BaudRate = 115200;																			//波特率设置
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //不开硬件流
	USART_Initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;							//选择仅接收模式
	USART_Initstructure.USART_Parity = USART_Parity_No; 														//没有校验位
	USART_Initstructure.USART_StopBits = USART_StopBits_1;													//1位停止位
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;											//8位数据位
	USART_Init(USART1,&USART_Initstructure); 
    
    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断


    USART_Cmd(USART1, ENABLE);																//打开UART
    /******************************************************************************/
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 			//PD5复用TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//上拉或者浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;			//PD6复用RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
    GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);  //开启重定义功能!!!!!!
	
	USART_Initstructure.USART_BaudRate = 9600;																			//波特率设置
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //不开硬件流
	USART_Initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;							//选择仅接收模式
	USART_Initstructure.USART_Parity = USART_Parity_No; 														//没有校验位
	USART_Initstructure.USART_StopBits = USART_StopBits_1;													//1位停止位
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;											//8位数据位
	USART_Init(USART2,&USART_Initstructure); 
    
    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断


    USART_Cmd(USART2, ENABLE);																//打开UART
    
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
	{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
        UART1_Data_Prase_Prepare_Lite(Res);
   } 
}

void USART3_IRQHandler(void)                	//串口3中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
		Res =USART_ReceiveData(USART3);	//读取接收到的数据
        NCLink_Data_Prase_Prepare_Lite(Res);
   } 
}

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		Res =USART_ReceiveData(USART2);	//读取接收到的数据
//        if(Res>='0' && Res<='5' && read_color_flag==0) color_num = Res; //read_color_flag = 1时不接收数据更新
//        else if(Res=='k') read_color_flag = 1;
//        else
//        {
//            color_num = '6';
//            read_color_flag = 255;    //错误数据
//        }
   } 
}

void UARTx_SendByte(USART_TypeDef* USARTx,uint8_t Byte)
{
	USART_SendData(USARTx, Byte);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE) == RESET);
}

void Serial_SendString(USART_TypeDef* USARTx,char Strings[],FlagStatus LineBreak) //LineBreak即是否添加换行
{
		char *point = Strings;
		if(LineBreak == SET)
		{
			while(*point != '\0')
			{
				UARTx_SendByte(USARTx, *point);
				point++;
			}
            UARTx_SendByte(USARTx, '\r');
            UARTx_SendByte(USARTx, '\n');
		}else
		{
			while(*point != '\0')
			{
				UARTx_SendByte(USARTx, *point);
				point++;
			}
		}
}
