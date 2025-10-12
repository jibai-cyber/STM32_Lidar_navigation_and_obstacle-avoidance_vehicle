#ifndef __KEY_H
#define __KEY_H	 


#define KEY0  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//读取按键1
#define KEY14  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_14)//读取按键PA14
#define KEY15  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)//读取按键PA15

#define WK_UP   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取按键3(WK_UP) 

 

#define KEY0_PRES 	1	//KEY0按下
#define KEY1_PRES	  2	//KEY1按下
#define KEY14_PRES 	  14	//KEY14按下
#define KEY15_PRES	  15	//KEY15按下
#define WKUP_PRES   3	//KEY_UP按下(即WK_UP/KEY_UP)


void KEY_Init(void);//IO初始化
u8 KEY_Scan(u8);  	//按键扫描函数					    
#endif
