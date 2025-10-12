#ifndef _SPEED_H
#define _SPEED_H
void Encoder_Init_TIM1(void);
void Encoder_Init_TIM4(void);
void TIM5_Init(void);
int16_t Read_Encoder(u8 TIMX);
#endif
