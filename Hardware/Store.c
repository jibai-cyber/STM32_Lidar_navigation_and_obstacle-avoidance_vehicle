#include "stm32f10x.h"                  // Device header
#include "MyFLASH.h"
#include "Store.h"
#include "Data_analyse.h"

#define STORE_START_ADDRESS		0x0807F000      //从这里到0x0807F7FF为一个扇区，魔术棒target中的编译范围也要改，Debug里面的烧录方式也要选部分擦除，只擦除程序部分占用的扇区
#define STORE_COUNT				512     //最大能存储的点的个数：512/4

uint16_t Store_Data[STORE_COUNT];
Store_point store_point[20];
uint8_t buildings_color[6];
void Store_Init(void)
{
	if (MyFLASH_ReadHalfWord(STORE_START_ADDRESS) != 0xA5A5)
	{
		MyFLASH_ErasePage(STORE_START_ADDRESS);
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS, 0xA5A5);
		for (uint16_t i = 1; i < STORE_COUNT; i ++)
		{
			MyFLASH_ProgramHalfWord(STORE_START_ADDRESS + i * 2, 0x0000);
		}
	}
	
	for (uint16_t i = 0; i < STORE_COUNT; i ++)
	{
		Store_Data[i] = MyFLASH_ReadHalfWord(STORE_START_ADDRESS + i * 2);
	}
    
    for(uint8_t i=0; i<10; i++) Get_a_point(i);     //读出10个点
    
    Read_colors();
}

void Store_Save(void)
{
	MyFLASH_ErasePage(STORE_START_ADDRESS);
	for (uint16_t i = 0; i < STORE_COUNT; i ++)
	{
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS + i * 2, Store_Data[i]);
	}
}

void Store_Clear(void)
{
	for (uint16_t i = 1; i < STORE_COUNT; i ++)
	{
		Store_Data[i] = 0x0000;
	}
	Store_Save();
}

void Store_a_point(uint8_t point_num)
{
    Float2Byte(store_point[point_num].x,store_point[point_num].x_,0);
    Store_Data[1+point_num*4] = COMB_2BYTE(store_point[point_num].x_[0],store_point[point_num].x_[1]);
    Store_Data[2+point_num*4] = COMB_2BYTE(store_point[point_num].x_[2],store_point[point_num].x_[3]);
    
    Float2Byte(store_point[point_num].y,store_point[point_num].y_,0);
    Store_Data[3+point_num*4] = COMB_2BYTE(store_point[point_num].y_[0],store_point[point_num].y_[1]);
    Store_Data[4+point_num*4] = COMB_2BYTE(store_point[point_num].y_[2],store_point[point_num].y_[3]);
}

void Get_a_point(uint8_t point_num)
{
    store_point[point_num].x_[0] = (Store_Data[1+point_num*4]>>8)&0xFF; //高8位
    store_point[point_num].x_[1] = Store_Data[1+point_num*4]&0xFF;  //低8位
    store_point[point_num].x_[2] = (Store_Data[2+point_num*4]>>8)&0xFF; //高8位
    store_point[point_num].x_[3] = Store_Data[2+point_num*4]&0xFF;  //低8位
    Byte2Float(store_point[point_num].x_,0,&store_point[point_num].x);
    
    store_point[point_num].y_[0] = (Store_Data[3+point_num*4]>>8)&0xFF; //高8位
    store_point[point_num].y_[1] = Store_Data[3+point_num*4]&0xFF;  //低8位
    store_point[point_num].y_[2] = (Store_Data[4+point_num*4]>>8)&0xFF; //高8位
    store_point[point_num].y_[3] = Store_Data[4+point_num*4]&0xFF;  //低8位
    Byte2Float(store_point[point_num].y_,0,&store_point[point_num].y);
}

void Store_colors(void) //调三个元素来存储颜色数据
{
    Store_Data[260] = COMB_2BYTE(buildings_color[0],buildings_color[1]);
    Store_Data[261] = COMB_2BYTE(buildings_color[2],buildings_color[3]);
    Store_Data[262] = COMB_2BYTE(buildings_color[4],buildings_color[5]);
    Store_Save();
}

void Read_colors(void)
{
    buildings_color[0] = (Store_Data[260]>>8)&0xFF;
    buildings_color[1] = Store_Data[260]&0xFF;
    buildings_color[2] = (Store_Data[261]>>8)&0xFF;
    buildings_color[3] = Store_Data[261]&0xFF;
    buildings_color[4] = (Store_Data[262]>>8)&0xFF;
    buildings_color[5] = Store_Data[262]&0xFF;
}











