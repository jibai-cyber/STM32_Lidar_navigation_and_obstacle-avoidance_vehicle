#include "stm32f10x.h"
#include "lcd1602.h"
#include "Delay.h"
#include "LED.h"
/*
PG5 = RS
PG6 = RW
PG4 = E
PG3 = V0
D0 = PB2
D1 = PB0
D2 = PC4
D3 = PA6
D4 = PA4
D5 = PA0
D6 = PC2
D7 = PC0

*/ 
/*******************************************************************************
* �� �� ��       : lcd1602_write_cmd    GPIO_ResetBits(GPIOB,GPIO_Pin_5);
* ��������		 : LCD1602д����        GPIO_SetBits(GPIOB,GPIO_Pin_5);	
* ��    ��       : cmd��ָ��            Bit_RESET��д��͵�ƽ Bit_SET��д��ߵ�ƽ
* ��    ��    	 : ��
*******************************************************************************/
void lcd1602_write_cmd(uint8_t cmd)
{
	GPIO_ResetBits(GPIOG,GPIO_Pin_5);//ѡ������
	GPIO_ResetBits(GPIOG,GPIO_Pin_6);//ѡ��д
	GPIO_ResetBits(GPIOG,GPIO_Pin_4);//E�Ž��� 
	GPIO_WriteBit(GPIOB,GPIO_Pin_2,cmd&0x01);GPIO_WriteBit(GPIOB,GPIO_Pin_0,(cmd&0x02)>>1);
	GPIO_WriteBit(GPIOC,GPIO_Pin_4,(cmd&0x04)>>2);GPIO_WriteBit(GPIOA,GPIO_Pin_6,(cmd&0x08)>>3);
	GPIO_WriteBit(GPIOA,GPIO_Pin_4,(cmd&0x10)>>4);GPIO_WriteBit(GPIOA,GPIO_Pin_0,(cmd&0x20)>>5);
	GPIO_WriteBit(GPIOC,GPIO_Pin_2,(cmd&0x40)>>6);GPIO_WriteBit(GPIOC,GPIO_Pin_0,(cmd&0x80)>>7);
	Delay_ms(5);
	GPIO_SetBits(GPIOG,GPIO_Pin_4);//ʹ�ܽ�E��������д��
	Delay_ms(5);
	GPIO_ResetBits(GPIOG,GPIO_Pin_4);//ʹ�ܽ�E���������д��	
}
/*******************************************************************************
* �� �� ��       : lcd1602_write_data
* ��������		 : LCD1602д����
* ��    ��       : dat������
* ��    ��    	 : ��
*******************************************************************************/
void lcd1602_write_data(uint8_t dat) 
{
	GPIO_SetBits(GPIOG,GPIO_Pin_5);//ѡ������
	GPIO_ResetBits(GPIOG,GPIO_Pin_6);//ѡ��д
	GPIO_ResetBits(GPIOG,GPIO_Pin_4);
	GPIO_WriteBit(GPIOB,GPIO_Pin_2,dat&0x01);GPIO_WriteBit(GPIOB,GPIO_Pin_0,(dat&0x02)>>1);
	GPIO_WriteBit(GPIOC,GPIO_Pin_4,(dat&0x04)>>2);GPIO_WriteBit(GPIOA,GPIO_Pin_6,(dat&0x08)>>3);
	GPIO_WriteBit(GPIOA,GPIO_Pin_4,(dat&0x10)>>4);GPIO_WriteBit(GPIOA,GPIO_Pin_0,(dat&0x20)>>5);
	GPIO_WriteBit(GPIOC,GPIO_Pin_2,(dat&0x40)>>6);GPIO_WriteBit(GPIOC,GPIO_Pin_0,(dat&0x80)>>7);
	Delay_ms(5);
	GPIO_SetBits(GPIOG,GPIO_Pin_4);
	Delay_ms(5);
	GPIO_SetBits(GPIOG,GPIO_Pin_4);	
}
/*******************************************************************************
* �� �� ��       : lcd1602_init
* ��������		 : LCD1602��ʼ��
* ��    ��       : ��
* ��    ��    	 : ��
*******************************************************************************/
void lcd1602_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_Init(GPIOG,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOG,GPIO_Pin_3,0);
	
	lcd1602_write_cmd(0x38);
	lcd1602_write_cmd(0x0C);
	lcd1602_write_cmd(0x06);
	lcd1602_write_cmd(0x01);
}
/*******************************************************************************
* �� �� ��       : lcd1602_clear
* ��������		 : LCD1602����
* ��    ��       : ��
* ��    ��    	 : ��
*******************************************************************************/
void lcd1602_clear(void)
{
	lcd1602_write_cmd(0x01);	
}

/*******************************************************************************
* �� �� ��       : lcd1602_show_string
* ��������		 : LCD1602��ʾ�ַ�
* ��    ��       : x,y����ʾ���꣬x=0~15��y=0~1;
				   str����ʾ�ַ���
* ��    ��    	 : ��
*******************************************************************************/
void lcd1602_show_string(uint8_t x,uint8_t y,uint8_t *str)
{
	uint8_t i=0;

	if(y>1||x>15)return;//���в���������ǿ���˳�

	if(y<1)	//��1����ʾ
	{	
		while(*str!='\0')//�ַ�������'\0'��β��ֻҪǰ�������ݾ���ʾ
		{
			if(i<16-x)//����ַ����ȳ�����һ����ʾ��Χ�����ڵڶ��м�����ʾ
			{
				lcd1602_write_cmd(0x80+i+x);//��һ����ʾ��ַ����	
			}
			else
			{
				lcd1602_write_cmd(0x40+0x80+i+x-16);//�ڶ�����ʾ��ַ����	
			}
			lcd1602_write_data(*str);//��ʾ����
			str++;//ָ�����
			i++;	
		}	
	}
	else	//��2����ʾ
	{
		while(*str!='\0')
		{
			if(i<16-x) //����ַ����ȳ����ڶ�����ʾ��Χ�����ڵ�һ�м�����ʾ
			{
				lcd1602_write_cmd(0x80+0x40+i+x);	
			}
			else
			{
				lcd1602_write_cmd(0x80+i+x-16);	
			}
			lcd1602_write_data(*str);
			str++;
			i++;	
		}	
	}				
}

