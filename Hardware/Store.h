#ifndef __STORE_H
#define __STORE_H

typedef struct
{
    float x;
    float y;
    uint8_t x_[4];
    uint8_t y_[4];
}Store_point;

extern uint16_t Store_Data[];
extern Store_point store_point[20];
extern uint8_t buildings_color[];

void Store_Init(void);
void Store_Save(void);
void Store_Clear(void);
void Store_a_point(uint8_t point_num);
void Get_a_point(uint8_t point_num);
void Store_colors(void);
void Read_colors(void);

#endif
