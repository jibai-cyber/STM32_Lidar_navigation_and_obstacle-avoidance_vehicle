#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ��ӢSTM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	   

//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��

void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
 GPIO_ResetBits(GPIOB,GPIO_Pin_5);						 //PB.5 ����ߣ����ڿ⺯��д����Ҳ����ֱ����PBout(5)=1������λ������

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 �˿�����, �������
 GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
 GPIO_ResetBits(GPIOE,GPIO_Pin_5); 						 //PE.5 ����� |RCC_APB2Periph_GPIOE
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //LED0-->PB.5 �˿�����
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
     GPIO_Init(GPIOG, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
    GPIO_ResetBits(GPIOG,GPIO_Pin_9);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	    		 //LED1-->PE.5 �˿�����, �������
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOG,GPIO_Pin_11);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	    		 //LED1-->PE.5 �˿�����, �������
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
}

uint8_t Bling_mode(u8 num,u8 times) //Ŀǰֻʹ����times<=5�����
{
    static u8 bling_timer_cnt,working_flag;  //bling_timer_cntΪ�̶�ʱ����
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
                bling_timer_cnt = times*20-1; //20��bling_timer_cntΪһ�����ڣ�����һ����һ��
                working_flag = 1;
                if(bling_timer_cnt/100!=0) return 2;    //����
            }
            else if(working_flag==1)
            {
                if(bling_timer_cnt/10%2==1) //��λ����ʮλΪ��������������Ŀǰֻʹ����times<=5�����
                {
                    GPIO_SetBits(GPIOG,GPIO_Pin_9);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_11);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
                }
                else    //ʮλΪż������
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
                bling_timer_cnt = times*20-1; //20��bling_timer_cntΪһ�����ڣ�����һ����һ��
                working_flag = 1;
                if(bling_timer_cnt/100!=0) return 2;    //����
            }
            else if(working_flag==1)
            {
                if(bling_timer_cnt/10%2==1) //��λ����ʮλΪ��������������Ŀǰֻʹ����times<=5�����
                {
                    GPIO_SetBits(GPIOG,GPIO_Pin_9);
                    GPIO_SetBits(GPIOG,GPIO_Pin_11);
                    GPIO_ResetBits(GPIOG,GPIO_Pin_13);
                }
                else    //ʮλΪż������
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
                bling_timer_cnt = times*20-1; //20��bling_timer_cntΪһ�����ڣ�����һ����һ��
                working_flag = 1;
                if(bling_timer_cnt/100!=0) return 2;    //����
            }
            else if(working_flag==1)
            {
                if(bling_timer_cnt/10%2==1) //��λ����ʮλΪ��������������Ŀǰֻʹ����times<=5�����
                {
                    GPIO_SetBits(GPIOG,GPIO_Pin_9);
                    GPIO_SetBits(GPIOG,GPIO_Pin_11);
                    GPIO_SetBits(GPIOG,GPIO_Pin_13);
                }
                else    //ʮλΪż������
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
 

