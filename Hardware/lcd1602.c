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
* 函 数 名       : lcd1602_write_cmd    GPIO_ResetBits(GPIOB,GPIO_Pin_5);
* 函数功能		 : LCD1602写命令        GPIO_SetBits(GPIOB,GPIO_Pin_5);	
* 输    入       : cmd：指令            Bit_RESET：写入低电平 Bit_SET：写入高电平
* 输    出    	 : 无
*******************************************************************************/
void lcd1602_write_cmd(uint8_t cmd)
{
	GPIO_ResetBits(GPIOG,GPIO_Pin_5);//选择命令
	GPIO_ResetBits(GPIOG,GPIO_Pin_6);//选择写
	GPIO_ResetBits(GPIOG,GPIO_Pin_4);//E脚降低 
	GPIO_WriteBit(GPIOB,GPIO_Pin_2,cmd&0x01);GPIO_WriteBit(GPIOB,GPIO_Pin_0,(cmd&0x02)>>1);
	GPIO_WriteBit(GPIOC,GPIO_Pin_4,(cmd&0x04)>>2);GPIO_WriteBit(GPIOA,GPIO_Pin_6,(cmd&0x08)>>3);
	GPIO_WriteBit(GPIOA,GPIO_Pin_4,(cmd&0x10)>>4);GPIO_WriteBit(GPIOA,GPIO_Pin_0,(cmd&0x20)>>5);
	GPIO_WriteBit(GPIOC,GPIO_Pin_2,(cmd&0x40)>>6);GPIO_WriteBit(GPIOC,GPIO_Pin_0,(cmd&0x80)>>7);
	Delay_ms(5);
	GPIO_SetBits(GPIOG,GPIO_Pin_4);//使能脚E先上升沿写入
	Delay_ms(5);
	GPIO_ResetBits(GPIOG,GPIO_Pin_4);//使能脚E后负跳变完成写入	
}
/*******************************************************************************
* 函 数 名       : lcd1602_write_data
* 函数功能		 : LCD1602写数据
* 输    入       : dat：数据
* 输    出    	 : 无
*******************************************************************************/
void lcd1602_write_data(uint8_t dat) 
{
	GPIO_SetBits(GPIOG,GPIO_Pin_5);//选择数据
	GPIO_ResetBits(GPIOG,GPIO_Pin_6);//选择写
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
* 函 数 名       : lcd1602_init
* 函数功能		 : LCD1602初始化
* 输    入       : 无
* 输    出    	 : 无
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
* 函 数 名       : lcd1602_clear
* 函数功能		 : LCD1602清屏
* 输    入       : 无
* 输    出    	 : 无
*******************************************************************************/
void lcd1602_clear(void)
{
	lcd1602_write_cmd(0x01);	
}

/*******************************************************************************
* 函 数 名       : lcd1602_show_string
* 函数功能		 : LCD1602显示字符
* 输    入       : x,y：显示坐标，x=0~15，y=0~1;
				   str：显示字符串
* 输    出    	 : 无
*******************************************************************************/
void lcd1602_show_string(uint8_t x,uint8_t y,uint8_t *str)
{
	uint8_t i=0;

	if(y>1||x>15)return;//行列参数不对则强制退出

	if(y<1)	//第1行显示
	{	
		while(*str!='\0')//字符串是以'\0'结尾，只要前面有内容就显示
		{
			if(i<16-x)//如果字符长度超过第一行显示范围，则在第二行继续显示
			{
				lcd1602_write_cmd(0x80+i+x);//第一行显示地址设置	
			}
			else
			{
				lcd1602_write_cmd(0x40+0x80+i+x-16);//第二行显示地址设置	
			}
			lcd1602_write_data(*str);//显示内容
			str++;//指针递增
			i++;	
		}	
	}
	else	//第2行显示
	{
		while(*str!='\0')
		{
			if(i<16-x) //如果字符长度超过第二行显示范围，则在第一行继续显示
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

