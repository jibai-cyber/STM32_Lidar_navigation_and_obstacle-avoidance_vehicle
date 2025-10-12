#ifndef MOTOR_H
#define MOTOR_H
void Stop(void);
void Forward(void);
void Backward(void);
void Forward_TB(uint16_t Vl,uint16_t Vr);
void Backward_TB(void);
void Stop_TB(void);
void TIM2_PWM_Init(u16 arr,u16 psc);
void Turn_left(uint16_t Vl,uint16_t Vr);
void Turn_right(uint16_t Vl,uint16_t Vr);
#endif
