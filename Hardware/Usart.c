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
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 			//PB10����TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//�������߸�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			//PB11����RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	USART_Initstructure.USART_BaudRate = 460800;																			//����������
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //����Ӳ����
	USART_Initstructure.USART_Mode = USART_Mode_Rx;							//ѡ�������ģʽ
	USART_Initstructure.USART_Parity = USART_Parity_No; 														//û��У��λ
	USART_Initstructure.USART_StopBits = USART_StopBits_1;													//1λֹͣλ
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;											//8λ����λ
	USART_Init(USART3,&USART_Initstructure); 
    
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�


    USART_Cmd(USART3, ENABLE);																//��UART
    /******************************************************************************/
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 			//PB10����TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//�������߸�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			//PB11����RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	
	USART_Initstructure.USART_BaudRate = 9600;																			//����������
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //����Ӳ����
	USART_Initstructure.USART_Mode = USART_Mode_Rx;							//ѡ�������ģʽ
	USART_Initstructure.USART_Parity = USART_Parity_No; 														//û��У��λ
	USART_Initstructure.USART_StopBits = USART_StopBits_1;													//1λֹͣλ
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;											//8λ����λ
	USART_Init(UART4,&USART_Initstructure); 
    
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�������ڽ����ж�


    USART_Cmd(UART4, ENABLE);																//��UART
    /******************************************************************************/

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);        //��������ſ��ܱ�������ռ�ã�������������
	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 			//PB10����TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//�������߸�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			//PB11����RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	USART_Initstructure.USART_BaudRate = 115200;																			//����������
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //����Ӳ����
	USART_Initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;							//ѡ�������ģʽ
	USART_Initstructure.USART_Parity = USART_Parity_No; 														//û��У��λ
	USART_Initstructure.USART_StopBits = USART_StopBits_1;													//1λֹͣλ
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;											//8λ����λ
	USART_Init(USART1,&USART_Initstructure); 
    
    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�


    USART_Cmd(USART1, ENABLE);																//��UART
    /******************************************************************************/
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 			//PD5����TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//�������߸�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;			//PD6����RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
    GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);  //�����ض��幦��!!!!!!
	
	USART_Initstructure.USART_BaudRate = 9600;																			//����������
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //����Ӳ����
	USART_Initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;							//ѡ�������ģʽ
	USART_Initstructure.USART_Parity = USART_Parity_No; 														//û��У��λ
	USART_Initstructure.USART_StopBits = USART_StopBits_1;													//1λֹͣλ
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;											//8λ����λ
	USART_Init(USART2,&USART_Initstructure); 
    
    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�


    USART_Cmd(USART2, ENABLE);																//��UART
    
}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
	{
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
        UART1_Data_Prase_Prepare_Lite(Res);
   } 
}

void USART3_IRQHandler(void)                	//����3�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
		Res =USART_ReceiveData(USART3);	//��ȡ���յ�������
        NCLink_Data_Prase_Prepare_Lite(Res);
   } 
}

void USART2_IRQHandler(void)                	//����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		Res =USART_ReceiveData(USART2);	//��ȡ���յ�������
//        if(Res>='0' && Res<='5' && read_color_flag==0) color_num = Res; //read_color_flag = 1ʱ���������ݸ���
//        else if(Res=='k') read_color_flag = 1;
//        else
//        {
//            color_num = '6';
//            read_color_flag = 255;    //��������
//        }
   } 
}

void UARTx_SendByte(USART_TypeDef* USARTx,uint8_t Byte)
{
	USART_SendData(USARTx, Byte);
	while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE) == RESET);
}

void Serial_SendString(USART_TypeDef* USARTx,char Strings[],FlagStatus LineBreak) //LineBreak���Ƿ���ӻ���
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
