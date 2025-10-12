#ifndef _CONTROL_H
#define _CONTROL_H

#define SDK_NUM_TOTAL 7   //���������
#define SDK_POINT_TOTAL 10  //�ܹ���¼��ĸ���
#define BODY_LENTH 16
/*��ͬĦ�������������ͬ���������pwmֵ*/
#define FRICTION_COF FRICTION_COF_1 //����ֵ
#define FRICTION_COF_1 1.0f //�׵�
#define FRICTION_COF_2 1.43f //�����õ�

enum YAW_CTRL_MODE
{
	NEAR=0,							//�ֶ�ƫ������ģʽ
  AZIMUTH=1,						//����ƫ���Ƕȿ���ģʽ
	CLOCKWISE=2,					//���ƫ���Ƕ�˳ʱ�����ģʽ	
	ANTI_CLOCKWISE=3,			//���ƫ���Ƕ���ʱ�����ģʽ	
	CLOCKWISE_TURN=4,			//���ٶȿ���˳ʱ��ģʽ
	ANTI_CLOCKWISE_TURN=5,//���ٶȿ�����ʱ��ģʽ
    NULL=6,
};

typedef struct 
{
  uint8_t Start_Flag;
  uint8_t Execute_Flag;
  uint8_t End_flag;
}Duty_Status;

typedef struct
{
	float yaw_control_output;				  	//ƫ����̬���������������������δʹ��
  float yaw_outer_control_output;	  	//ƫ����̬����������
	uint16_t yaw_ctrl_cnt;							//ƫ�����Ƽ�����
	uint8_t yaw_ctrl_mode;							//ƫ������ģʽ
	uint8_t yaw_ctrl_start;							//ƫ�����ƿ�ʼ��־λ
	uint8_t yaw_ctrl_end;								//ƫ�����ƽ�����־λ
    
}Controller_Output;

typedef struct
{
    float Expect;//����
    float FeedBack;//����ֵ
    float Err;//ƫ��
    float Last_Err;//�ϴ�ƫ��
    float Err_Max;//ƫ���޷�ֵ
    float Integrate;//����ֵ
    float Integrate_Max;//�����޷�ֵ
    float Kp;//���Ʋ���Kp
    float Ki;//���Ʋ���Ki
    float Kd;//���Ʋ���Kd
    float Control_OutPut;//�����������
    float Control_OutPut_Limit;//����޷�
}PID_Controler;

typedef struct
{
    float re_x;
    float re_y;
    float ab_x;
    float ab_y;
    u8 start_flag;
}RE_pos;

void TIM3_Init(void);
void move_with_xy_target_relatively(float x,float y,Duty_Status* Status,u8 number,float bia);
void POS_conctrol(float x,float y,float Relative_angle,float bia);
float PID_Control_Yaw(PID_Controler *Controler,u8 mode);
void Angle_Control_Target(Controller_Output *_car_output);
void Move_around(float x,float y,float re_dis,float re_ang_dkr,float bia);
void move_xy_barrier(float x,float y,float ba_dis,float ba_ang);
void Turn_with_relative_angle(float angle);
void From_relative_xy_to_angle(float x,float y,float* re_ang_dkr);
void move_xy_barrier1(float x,float y,float ba_dis,float ba_ang);
void wheel_speed_control(int16_t target_left_spd, int16_t target_right_spd);
void SDK_Mode_Task(uint8_t *sdk_mode);
void SDK_Num_Reset(void);
#endif
