#ifndef __KEY_H
#define __KEY_H	 


#define KEY0  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//��ȡ����0
#define KEY1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//��ȡ����1
#define KEY14  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_14)//��ȡ����PA14
#define KEY15  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)//��ȡ����PA15

#define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//��ȡ����3(WK_UP) 

 

#define KEY0_PRES 	1	//KEY0����
#define KEY1_PRES	  2	//KEY1����
#define KEY14_PRES 	  14	//KEY14����
#define KEY15_PRES	  15	//KEY15����
#define WKUP_PRES   3	//KEY_UP����(��WK_UP/KEY_UP)


void KEY_Init(void);//IO��ʼ��
u8 KEY_Scan(u8);  	//����ɨ�躯��					    
#endif
