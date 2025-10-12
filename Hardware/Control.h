#ifndef _CONTROL_H
#define _CONTROL_H

#define SDK_NUM_TOTAL 7   //总任务个数
#define SDK_POINT_TOTAL 10  //总共能录点的个数
#define BODY_LENTH 16
/*不同摩擦因数会决定不同的死区电机pwm值*/
#define FRICTION_COF FRICTION_COF_1 //最终值
#define FRICTION_COF_1 1.0f //白地
#define FRICTION_COF_2 1.43f //竞赛用地

enum YAW_CTRL_MODE
{
	NEAR=0,							//手动偏航控制模式
  AZIMUTH=1,						//绝对偏航角度控制模式
	CLOCKWISE=2,					//相对偏航角度顺时针控制模式	
	ANTI_CLOCKWISE=3,			//相对偏航角度逆时针控制模式	
	CLOCKWISE_TURN=4,			//角速度控制顺时针模式
	ANTI_CLOCKWISE_TURN=5,//角速度控制逆时针模式
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
	float yaw_control_output;				  	//偏航姿态控制器最终输出，变量暂未使用
  float yaw_outer_control_output;	  	//偏航姿态控制器输入
	uint16_t yaw_ctrl_cnt;							//偏航控制计数器
	uint8_t yaw_ctrl_mode;							//偏航控制模式
	uint8_t yaw_ctrl_start;							//偏航控制开始标志位
	uint8_t yaw_ctrl_end;								//偏航控制结束标志位
    
}Controller_Output;

typedef struct
{
    float Expect;//期望
    float FeedBack;//反馈值
    float Err;//偏差
    float Last_Err;//上次偏差
    float Err_Max;//偏差限幅值
    float Integrate;//积分值
    float Integrate_Max;//积分限幅值
    float Kp;//控制参数Kp
    float Ki;//控制参数Ki
    float Kd;//控制参数Kd
    float Control_OutPut;//控制器总输出
    float Control_OutPut_Limit;//输出限幅
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
