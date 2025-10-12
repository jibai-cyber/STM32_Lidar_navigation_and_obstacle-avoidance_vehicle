#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 精英STM32开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化

void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);	 //使能PB,PE端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
 GPIO_ResetBits(GPIOB,GPIO_Pin_5);						 //PB.5 输出高，属于库函数写法。也可以直接用PBout(5)=1，属于位操作。

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
 GPIO_ResetBits(GPIOE,GPIO_Pin_5); 						 //PE.5 输出高 |RCC_APB2Periph_GPIOE
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //LED0-->PB.5 端口配置
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
     GPIO_Init(GPIOG, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
    GPIO_ResetBits(GPIOG,GPIO_Pin_9);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	    		 //LED1-->PE.5 端口配置, 推挽输出
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOG,GPIO_Pin_11);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	    		 //LED1-->PE.5 端口配置, 推挽输出
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
}

uint8_t Bling_mode(u8 num,u8 times) //目前只使用与times<=5的情况
{
    static u8 bling_timer_cnt,working_flag;  //bling_timer_cnt为固定时间间隔
    switch(num)
    {
    	case 0:
        {
            GPIO_ResetBits(GPIOG,GPIO_Pin_9);
            GPIO_ResetBits(GPIOG,GPIO_Pin_11);
            GPIO_ResetBits(GPIOG,GPIO_Pin_13);
            return 1;
        }
    	case 1:
        {
            if(working_flag==0)
            {
                bling_timer_cnt = times*20-1; //20个bling_timer_cnt为一个周期，即亮一次灭一次
                working_flag = 1;
                if(bling_timer_cnt/100!=0) return 2;    //错误
            }
            else if(working_flag==1)
            {
                if(bling_timer_cnt/10%2==1) //个位数或十位为奇数的数，所有目前只使用与times<=5的情况
                {
                    GPIO_SetBits(GPIOG,GPIO_Pin_9);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_11);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
                }
                else    //十位为偶数的数
                {
                    GPIO_ResetBits(GPIOG,GPIO_Pin_9);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_11);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
                    if(bling_timer_cnt==0)
                    {
                        working_flag = 0;
                        return 1;
                    }
                }
                
                bling_timer_cnt--;
                
            }
            
        }
    		break;
        
        case 2:
        {
            if(working_flag==0)
            {
                bling_timer_cnt = times*20-1; //20个bling_timer_cnt为一个周期，即亮一次灭一次
                working_flag = 1;
                if(bling_timer_cnt/100!=0) return 2;    //错误
            }
            else if(working_flag==1)
            {
                if(bling_timer_cnt/10%2==1) //个位数或十位为奇数的数，所有目前只使用与times<=5的情况
                {
                    GPIO_SetBits(GPIOG,GPIO_Pin_9);
                    GPIO_SetBits(GPIOG,GPIO_Pin_11);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
                }
                else    //十位为偶数的数
                {
                    GPIO_ResetBits(GPIOG,GPIO_Pin_9);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_11);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
                    if(bling_timer_cnt==0)
                    {
                        working_flag = 0;
                        return 1;
                    }
                }
                
                bling_timer_cnt--;
                
            }
            
        }
    		break;
        
        case 3:
        {
            if(working_flag==0)
            {
                bling_timer_cnt = times*20-1; //20个bling_timer_cnt为一个周期，即亮一次灭一次
                working_flag = 1;
                if(bling_timer_cnt/100!=0) return 2;    //错误
            }
            else if(working_flag==1)
            {
                if(bling_timer_cnt/10%2==1) //个位数或十位为奇数的数，所有目前只使用与times<=5的情况
                {
                    GPIO_SetBits(GPIOG,GPIO_Pin_9);
                    GPIO_SetBits(GPIOG,GPIO_Pin_11);
                    GPIO_SetBits(GPIOG,GPIO_Pin_13);
                }
                else    //十位为偶数的数
                {
                    GPIO_ResetBits(GPIOG,GPIO_Pin_9);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_11);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
                    if(bling_timer_cnt==0)
                    {
                        working_flag = 0;
                        return 1;
                    }
                }
                
                bling_timer_cnt--;
                
            }
            
        }
    		break;

    	default:
    		break;
    }
    
    return 0;
}
 

