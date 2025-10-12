#include "stm32f10x.h"
#include "Motor.h"
#include "Control.h"
#include "Speed.h"
#include "Delay.h"
#include "Data_analyse.h"
#include "math.h"
#include "oled1.h"
#include "Store.h"
#include "led.h"
#include "Steering.h"
#include "Usart.h"
#include <string.h>

uint8_t TIME_CNT;
uint8_t Duty_flag_num,Reach_flag,update_flag,process_num;

float x_start_offset,y_start_offset;
float manual_point[][2]={
    0, 160.0f,
    0, 347.2f,
    -212.0f, 347.2f,
    -420.0f, 347.2f,
    -420.0f, 160.0f,
    -420.0f, 32.0f,
    -212.0f, 16.5f,
    0, 0
};
float manual_patrol_point[][2]={
    21.91f, 66.74f,
    33.44f, 123.49f,
    49.67f, 206.13f,
    67.68f, 280.12f,
    71.68f, 348.3f,
    198.48f, 357.18f,
    210.03f, 290.78f,
    211.27f, 162.40f,
    213.99f, 76.66f,
    0, 0
};

Vector2f Manual_Patrol_Node[6]={
    320,80,
    320,240,
    330,370,
    210,370,
    210,240,
    330,240
};

uint8_t led_num_times[][2]={    //对应不同的闪烁情况，第一个元素为个数，第二个元素为次数
    0, 0,
    1, 2,
    1, 3,
    2, 3,
    2, 3,
    3, 4,
    3, 2
};

uint8_t sensor[5];
uint8_t FlagPresent,FlagLast,Flag,STOP_flag,Finish_flag=0;
uint8_t fflag,fflagcrossing;
int16_t PID_Minus=0;//小车差速PID控制器的PWM输出值
int OUTPUT1,OUTPUT2;
uint8_t TimerClear;
uint8_t Judge;
Duty_Status Duty_flag[SDK_POINT_TOTAL];
Controller_Output Lidar_Car;
PID_Controler Yaw_Angle_Control,L_wheel,R_wheel;
Vector2f Flight_pre_position={35,35},Flight_cur_position={35,35};   //场地坐标
//Vector2f Flight_pre_position={0,0},Flight_cur_position={0,0};   //场地坐标
float Flight_moving_dis=0,Flight_moving_dis_delta=0,Car_to_fire_angle=0;
int Map_x=0,Map_y=0;
char LCD_str[50]="\0";  //元素个数一定要给！！！！！！！不然memset函数会出bug
char LCD_str1[]="page0.n0.val=";
char LCD_str2[]="page0.n1.val=";
char LCD_str3[]="page0.n2.val=";
char LCD_str4[]="cur_x=";
char LCD_str5[20]="cur_y=";
char LCD_str6[20]="page2.n0.val=";
char LCD_str7[20]="page2.n1.val=";
char LCD_str8[20]="page1.n0.val=";
char str_of_num[6];
char End_of_str[]="\xff\xff\xff";
uint8_t Str_to_Flight[8]={0xFF,0xFC,0x00,0x00,0x00,0xFA,0xFB,'\0'};

extern uint16_t ADC_ConvertedValue[2];
extern int16_t Analogue_PID_VAL,Speed_left,Speed_right;
extern uint8_t condition;
extern Descartes_cord DKR;
extern uint8_t read_flag,read_Flight_flag;
extern uint8_t read_color_flag,color_num;
extern RE_pos re_pos;
extern third_party_state barrier_state,current_state;
extern float Min_ang,Min_dis;
extern u8 SDK_mode,goal_num;
extern Vector2f Flight_current_position,Fire_position;  //飞控返回的坐标数据
extern uint8_t Launch_flag,Color_Fire_flag;

extern DJST_NodeType DJST_Node[];
extern uint8_t Pre_node_num[],Min_compute_flag;
extern Vector2f Node_crd[];
extern uint8_t Min_num_for_DJST;

extern uint8_t Read_Whole_Data_Flag_UART4,Function_Data_UART4,V831_Read_num,Num_Data_UART4;


void TIM3_Init()
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //中断优先级的结构体定义

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM3的时钟使能

	TIM_TimeBaseStructure.TIM_Period = 7199*1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =99; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的相应的模式配置
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);  //使能TIM3中断
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断服务函数的配置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;  //主优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能IRQ中断通道
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化NVIC寄存器

	TIM_Cmd(TIM3, ENABLE);  //开启TIM3
}


void TIM3_IRQHandler(void)   //TIM3中断服务函数
{
    static uint16_t _cnt;
    
	if(TIM3->SR&0X0001)//定时器每10ms定时中断一次
	{   
        TIM3->SR&=~(1<<0);//清除定时器三的计时标志，执行完TIM3中断服务函数里的操作后定时器会重新计时
        if(read_flag==2)
        {
            read_flag = 0;
            
            SDK_Mode_Task(&SDK_mode);
//            move_with_xy_target_relatively(re_pos.ab_x - DKR.x, re_pos.ab_y - DKR.y, Duty_flag, 0, 5); //函数使用时注意参数number，也就是要确保下一个任务执行时该结构体内的值为0
//            Move_around(re_pos.ab_x,re_pos.ab_y,Min_dis,Min_ang,45.0f);
//            move_xy_barrier(re_pos.ab_x,re_pos.ab_y,barrier_state.position_x,barrier_state.position_y,Min_dis,Min_ang);
//            move_xy_barrier1(re_pos.ab_x,re_pos.ab_y,Min_dis,Min_ang);
        }
        
//        if(Read_Whole_Data_Flag_UART4==1 && Function_Data_UART4<=3)
//        {
//            UARTx_SendByte(USART2, Function_Data_UART4+'0');
//            Read_Whole_Data_Flag_UART4=0;
//        }
        
        if(Function_Data_UART4==0x10 && Read_Whole_Data_Flag_UART4==1) //串口屏启动小车 //0x0A
        {
            Read_Whole_Data_Flag_UART4=0;
            SDK_mode=Function_Data_UART4;
            GPIO_SetBits(GPIOB,GPIO_Pin_5);
            GPIO_SetBits(GPIOE,GPIO_Pin_5);
//                goal_num=Num_Data_UART4;
        }
        
        if((Str_to_Flight[2]==0x07||Str_to_Flight[2]==0x08||Str_to_Flight[2]==0x09) && Read_Whole_Data_Flag_UART4==1)   //10ms内发不完？
        {
            Read_Whole_Data_Flag_UART4=0;
            UARTx_SendString(USART1,Str_to_Flight,7);
//            Serial_SendString(USART1,Str_to_Flight,RESET);  //小车按键启动飞机 //非常不建议使用此函数，因为在它的判断中0x00和'\0'是同一个东西
        }
        
        _cnt++;
        if(_cnt==100)   //1s一次
        {
            _cnt=0;
            Flight_cur_position.x=Flight_current_position.x+35; //场地内绝对坐标 current_state.position_x  详情参考小车抽象坐标系
            Flight_cur_position.y=Flight_current_position.y+35; //同时记得更改场地内飞机的起始坐标
            
            Flight_moving_dis_delta = sqrtf((Flight_cur_position.x-Flight_pre_position.x)*(Flight_cur_position.x-Flight_pre_position.x)+
                                        (Flight_cur_position.y-Flight_pre_position.y)*(Flight_cur_position.y-Flight_pre_position.y));
                        
            if(Flight_moving_dis_delta>=3)
            {
                Flight_pre_position.x=Flight_cur_position.x;
                Flight_pre_position.y=Flight_cur_position.y;
                Flight_moving_dis += Flight_moving_dis_delta;
            }    
            
            //注意，发送到串口屏显示的数据和地图上显示的需要放缩
            Map_x = (int)(Flight_cur_position.x/2);
            if(Map_x<0) Map_x=0;
            else if(Map_x>=240) Map_x=240;
            Map_y = (int)(240-Flight_cur_position.y/2);
            if(Map_y>200) Map_y=200;
            else if(Map_y<40) Map_y=40;
            
//            Map_x = (int)((Flight_cur_position.x+240)/2);
//            if(Map_x<0) Map_x=0;
//            else if(Map_x>240) Map_x=240;
//            Map_y = (int)(240-(Flight_cur_position.y+240)/2);
//            if(Map_y>240) Map_y=240;
//            else if(Map_y<0) Map_y=0;
                        
            memset(str_of_num,'\0',sizeof(str_of_num));
            memset(LCD_str,'\0',sizeof(LCD_str));
            
            strcpy(LCD_str,LCD_str1);
            int2str_low_ver((int)Flight_moving_dis,str_of_num);
            Mystrcat(LCD_str,str_of_num);
//            Mystrcat(LCD_str,End_of_str);
            Serial_SendString(UART4,LCD_str,RESET);
            for(uint8_t i=0;i<3;i++) UARTx_SendByte(UART4,0xFF);
            memset(str_of_num,'\0',sizeof(str_of_num));
            memset(LCD_str,'\0',sizeof(LCD_str));
            
            strcpy(LCD_str,LCD_str2);
            int2str_low_ver((int)Flight_cur_position.x,str_of_num);
            Mystrcat(LCD_str,str_of_num);
//            Mystrcat(LCD_str,End_of_str);
            Serial_SendString(UART4,LCD_str,RESET);
            for(uint8_t i=0;i<3;i++) UARTx_SendByte(UART4,0xFF);
            memset(str_of_num,'\0',sizeof(str_of_num));
            memset(LCD_str,'\0',sizeof(LCD_str));
            
            strcpy(LCD_str,LCD_str3);
            int2str_low_ver((int)Flight_cur_position.y,str_of_num);
            Mystrcat(LCD_str,str_of_num);
//            Mystrcat(LCD_str,End_of_str);
            Serial_SendString(UART4,LCD_str,RESET);
            for(uint8_t i=0;i<3;i++) UARTx_SendByte(UART4,0xFF);
            memset(str_of_num,'\0',sizeof(str_of_num));
            memset(LCD_str,'\0',sizeof(LCD_str));
            
            strcpy(LCD_str,LCD_str4);
            int2str_low_ver(Map_x,str_of_num);
            Mystrcat(LCD_str,str_of_num);
//            Mystrcat(LCD_str,End_of_str);
            Serial_SendString(UART4,LCD_str,RESET);
            for(uint8_t i=0;i<3;i++) UARTx_SendByte(UART4,0xFF);
            memset(str_of_num,'\0',sizeof(str_of_num));
            memset(LCD_str,'\0',sizeof(LCD_str));
            
            strcpy(LCD_str,LCD_str5);
            int2str_low_ver(Map_y,str_of_num);
            Mystrcat(LCD_str,str_of_num);
//            Mystrcat(LCD_str,End_of_str);
            Serial_SendString(UART4,LCD_str,RESET);
            for(uint8_t i=0;i<3;i++) UARTx_SendByte(UART4,0xFF);
            memset(str_of_num,'\0',sizeof(str_of_num));
            memset(LCD_str,'\0',sizeof(LCD_str));
            
            strcpy(LCD_str,LCD_str8);
            int2str_low_ver(V831_Read_num,str_of_num);
            Mystrcat(LCD_str,str_of_num);
//            Mystrcat(LCD_str,End_of_str);
            Serial_SendString(UART4,LCD_str,RESET);
            for(uint8_t i=0;i<3;i++) UARTx_SendByte(UART4,0xFF);
            memset(str_of_num,'\0',sizeof(str_of_num));
            memset(LCD_str,'\0',sizeof(LCD_str));
            
//            if(Launch_flag==1)  //找到火源后显示火源坐标
//            {
//                Serial_SendString(UART4,"page2.t3.txt=\"46.13\"\xff\xff\xff",RESET);
//                
//                strcpy(LCD_str,LCD_str6);
//                int2str_low_ver((int)Fire_position.x,str_of_num);
//                Mystrcat(LCD_str,str_of_num);
//    //            Mystrcat(LCD_str,End_of_str);
//                Serial_SendString(UART4,LCD_str,RESET);
//                for(uint8_t i=0;i<3;i++) UARTx_SendByte(UART4,0xFF);
//                memset(str_of_num,'\0',sizeof(str_of_num));
//                memset(LCD_str,'\0',sizeof(LCD_str));
//                
//                strcpy(LCD_str,LCD_str7);
//                int2str_low_ver((int)Fire_position.y,str_of_num);
//                Mystrcat(LCD_str,str_of_num);
//    //            Mystrcat(LCD_str,End_of_str);
//                Serial_SendString(UART4,LCD_str,RESET);
//                for(uint8_t i=0;i<3;i++) UARTx_SendByte(UART4,0xFF);
//                memset(str_of_num,'\0',sizeof(str_of_num));
//                memset(LCD_str,'\0',sizeof(LCD_str));
//            }
        }
    }
}

void SDK_Num_Reset(void)
{
  uint16_t i=0;
  for(i=0;i<SDK_POINT_TOTAL;i++)
  {
    Duty_flag[i].Start_Flag=0;
    Duty_flag[i].Execute_Flag=0;
    Duty_flag[i].End_flag=0;
  }
  Duty_flag_num=0;
}


void SDK_Mode_Task(uint8_t *sdk_mode)
{
    static uint16_t Sub_task_time_cnt;
    
    switch (*sdk_mode)
    {
    	case 0x02:
        {
//            move_xy_barrier(store_point[Duty_flag_num].x,store_point[Duty_flag_num].y,Min_dis,Min_ang);
//            OLED_Show_Float_num(40,32,store_point[Duty_flag_num].x,5);
//            OLED_Show_Float_num(40,48,store_point[Duty_flag_num].y,5);
            if(read_Flight_flag==1)
            {
//                move_xy_barrier(Target_Form_Flight_x,Target_Form_Flight_y,Min_dis,Min_ang);     //除Reach_flag，这里坐标最好不要频繁变换
//                read_Flight_flag = 0; //不能写在这里，因为小车的任务执行依赖move_with_xy_target_relatively，需要保证每个TIM3定时器中断中都会执行
                if(Reach_flag==1)   //避障用Reach_flag，普通循迹用STOP_flag
                {
                    Reach_flag = 0;
                    Duty_flag_num++;
                    if(Duty_flag_num>=goal_num)    //测试，最多写SDK_POINT_TOTAL
                    {
                        Stop_TB();
                        SDK_Num_Reset();
                        *sdk_mode = 0xFF;       //这里把mode改成别的值，后续更改可以衔接别的任务
                        read_Flight_flag = 0;   //清空接收标志
                    }
                }
            }

            
        }
    		break;
        
        case 0x0E:  //E仅为车单独任务，DJST算法要加Min_compute_flag判断执行
        {
//            move_with_xy_target_relatively(Manual_Patrol_Node[Duty_flag_num].x-DKR.x,
//                                            Manual_Patrol_Node[Duty_flag_num].y-DKR.y,
//                                            Duty_flag, Duty_flag_num, 10);//竞赛地板误差可以调大一点，不然容易过点
            move_xy_barrier(Manual_Patrol_Node[Duty_flag_num].x,Manual_Patrol_Node[Duty_flag_num].y,Min_dis,Min_ang); 
            if(Reach_flag==1)
            {
                Reach_flag = 0;
                Duty_flag_num++;
                if(Duty_flag_num>=goal_num)    //测试，最多写 SDK_POINT_TOTAL
                {
                    Stop_TB();
                    SDK_Num_Reset();
                    *sdk_mode = 0xFF;       //这里把mode改成别的值，后续更改可以衔接别的任务
                    Finish_flag=1;
                }
            }
        }
            break;
        
        case 0x0A:  //A到D为车机协同任务
        {
            if(Min_compute_flag==1) //在main中计算完成
            {
                move_with_xy_target_relatively(Node_crd[Pre_node_num[Duty_flag_num]].x-DKR.x-135,
                                                Node_crd[Pre_node_num[Duty_flag_num]].y-DKR.y-25,
                                                Duty_flag, Duty_flag_num, 10);//竞赛地板误差可以调大一点，不然容易过点
                if(STOP_flag==1)
                {
                    STOP_flag = 0;
                    Duty_flag_num++;
                    if(Duty_flag_num>=DJST_Node[Min_num_for_DJST].Pre_node_cnt)    //测试，最多写 SDK_POINT_TOTAL
                    {
                        Stop_TB();
                        SDK_Num_Reset();
                        *sdk_mode = 0x0B;       //这里把mode改成别的值，后续更改可以衔接别的任务
                    }
                }
            }
            
        }
            break;
        
        case 0x0B:
        {
            From_relative_xy_to_angle(Fire_position.x-DKR.x-135,Fire_position.y-DKR.y-25,&Car_to_fire_angle);
            Turn_with_relative_angle(Car_to_fire_angle);
            if(Lidar_Car.yaw_ctrl_end==1)
            {
                Yaw_Angle_Control.Integrate = 0;    //积分清零
                L_wheel.Integrate = R_wheel.Integrate = 0;
                
//                Color_Fire_flag=1;
                
                UARTx_SendByte(USART2,Color_Fire_flag+'0');
                *sdk_mode = 0x0C;       //这里把mode改成别的值，后续更改可以衔接别的任务
                
            }
        }
            break;
        
        case 0x0C:
        {                
            Stop_TB();
            Sub_task_time_cnt++;
            if(Sub_task_time_cnt>=35)   //雷达数据100ms执行一次
            {
                Sub_task_time_cnt=0;
                *sdk_mode = 0x0D;       //这里把mode改成别的值，后续更改可以衔接别的任务
                UARTx_SendByte(USART2,'e');
            }
        }
            break;
        
        case 0x0D:
        {
            if(Min_compute_flag==1) //在main中计算完成
            {
                move_with_xy_target_relatively(Node_crd[Pre_node_num[DJST_Node[Min_num_for_DJST].Pre_node_cnt-1-Duty_flag_num]].x-DKR.x-135, 
                                                Node_crd[Pre_node_num[DJST_Node[Min_num_for_DJST].Pre_node_cnt-1-Duty_flag_num]].y-DKR.y-25,
                                                Duty_flag, Duty_flag_num, 10);//竞赛地板误差可以调大一点，不然容易过点
                if(STOP_flag==1)
                {
                    STOP_flag = 0;
                    Duty_flag_num++;
                    if(Duty_flag_num>=DJST_Node[Min_num_for_DJST].Pre_node_cnt)    //测试，最多写 SDK_POINT_TOTAL
                    {
                        Stop_TB();
                        SDK_Num_Reset();
                        *sdk_mode = 0xFF;       //这里把mode改成别的值，后续更改可以衔接别的任务
                    }
                }
            }
            
        }
            break;
        
        case 0x10:
        {
            move_with_xy_target_relatively(Manual_Patrol_Node[Duty_flag_num].x-DKR.x-135,Manual_Patrol_Node[Duty_flag_num].y-DKR.y-25,
                                            Duty_flag, Duty_flag_num, 10);//竞赛地板误差可以调大一点，不然容易过点
            if(STOP_flag==1)
            {
                STOP_flag = 0;
                Duty_flag_num++;
                if(Duty_flag_num>=2)    //测试，最多写 SDK_POINT_TOTAL
                {
                    Stop_TB();
                    SDK_Num_Reset();
                    Duty_flag_num=2;        //延续下一个点继续走
                    *sdk_mode = 0x11;       //这里把mode改成别的值，后续更改可以衔接别的任务
                }
            }
            
        }
            break;
        
        case 0x11:
        {
            move_with_xy_target_relatively(Manual_Patrol_Node[Duty_flag_num].x-DKR.x-135,Manual_Patrol_Node[Duty_flag_num].y-DKR.y-25,
                                            Duty_flag, Duty_flag_num, 10);//竞赛地板误差可以调大一点，不然容易过点
            if(STOP_flag==1)
            {
                STOP_flag = 0;
                Duty_flag_num++;
                if(Duty_flag_num>=6)    //测试，最多写 SDK_POINT_TOTAL
                {
                    Stop_TB();
                    SDK_Num_Reset();
                    *sdk_mode = 0xFF;       //这里把mode改成别的值，后续更改可以衔接别的任务
                }
            }
            
        }
            break;
        
        case 0x03:
        {
//            move_with_xy_target_relatively(store_point[Duty_flag_num].x-DKR.x, store_point[Duty_flag_num].y-DKR.y, Duty_flag, Duty_flag_num, 10);//竞赛地板误差可以调大一点，不然容易过点
            move_with_xy_target_relatively(200.0f-DKR.x, 200.0f-DKR.y, Duty_flag, Duty_flag_num, 5);
            if(STOP_flag==1)
            {
                STOP_flag = 0;
                Duty_flag_num++;
                if(Duty_flag_num>=goal_num)    //测试，最多写 SDK_POINT_TOTAL
                {
                    Stop_TB();
                    SDK_Num_Reset();
                    *sdk_mode = 0xFF;       //这里把mode改成别的值，后续更改可以衔接别的任务
                }
            }
        }
    		break;
        
        case 0x04:
        {
            move_with_xy_target_relatively(manual_point[Duty_flag_num][0]-DKR.x-x_start_offset, manual_point[Duty_flag_num][1]-DKR.y-y_start_offset, Duty_flag, Duty_flag_num, 20);//竞赛地板误差可以调大一点，不然容易过点
            if(STOP_flag==1)
            {
                STOP_flag = 0;
                Duty_flag_num++;
                if(Duty_flag_num>=goal_num)    //测试，最多写SDK_POINT_TOTAL
                {
                    Stop_TB();
                    SDK_Num_Reset();
                    *sdk_mode = 0xFF;       //这里把mode改成别的值，后续更改可以衔接别的任务
                }
            }
        }
    		break;
        
        case 0x06:
        {
            move_with_xy_target_relatively(manual_patrol_point[Duty_flag_num][0]-DKR.x-x_start_offset, manual_patrol_point[Duty_flag_num][1]-DKR.y-y_start_offset, Duty_flag, Duty_flag_num, 20);//竞赛地板误差可以调大一点，不然容易过点
            if(STOP_flag==1)
            {
                STOP_flag = 0;
                
                switch(Duty_flag_num)
                {
                	case 1:
                    {
                        if(process_num==0)
                        {
                            /*舵机控制代码*/
                            Trun_SG90_angle(180);
                            TIME_CNT++;
                            if(TIME_CNT>=8) //间隔为800ms
                            {
                                TIME_CNT = 0;
                                USART_SendData(USART2,'s'); //向opv发送开始信号
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                            }
                            
                            if(read_color_flag==1) process_num = 1;
                        }
                        
//                            while(read_color_flag!=1);  //等待读取数据  //不能这么用！！！因为定时器的中断优先级会高，所有一直卡在定时器中断while里会无法进入串口接收中断
                        
                        else if(process_num==1)
                        {
                            /*根据color_num的数据确定亮灯程序并闪烁，用cnt--的方法代替延时函数*/
                            if(Bling_mode(led_num_times[color_num-'0'][0],led_num_times[color_num-'0'][1])==1)  //led操作完成返回值为1，后执行如下操作。特别注意有时不能与上一级if并用，因为&&关系中，只要有一个不满足条件就会直接跳出，不会执行该函数
                            {
                                read_color_flag = 0;
                                
                                buildings_color[0] = color_num;
                                USART_SendData(USART2,'e'); //向opv发送结束信号，opv在收到结束信号前只发一次数据，收到结束信号+下一个开始信号才能发下一个数据
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                                
                                process_num = 2;    //更新到下一个进程
                            }
                        }
                        
                        else if(process_num==2)  //已完成第一个子进程
                        {
                            /*舵机控制代码*/
                            Trun_SG90_angle(0);
                            TIME_CNT++;
                            if(TIME_CNT>=8)
                            {
                                TIME_CNT = 0;
                                USART_SendData(USART2,'s'); //向opv发送开始信号
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                            }
                            
                            if(read_color_flag==1)
                            {
                                /*根据color_num的数据确定亮灯程序并闪烁，用cnt--的方法代替延时函数*/
                                if(Bling_mode(led_num_times[color_num-'0'][0],led_num_times[color_num-'0'][1])==1)  //led操作完成返回值为1，后执行如下操作
                                {
                                    read_color_flag = 0;
                                
                                    process_num = 0;    //完成所有子进程
                                    update_flag = 1;    //允许更新目标坐标点
                                    
                                    buildings_color[1] = color_num;
                                    
                                    USART_SendData(USART2,'e'); //向opv发送结束信号
                                    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                                }
                            }
                        }
                            
                        
                    }
                		break;
                    case 3:
                    {
                        if(process_num==0)
                        {
                            /*舵机控制代码*/
                            Trun_SG90_angle(180);
                            TIME_CNT++;
                            if(TIME_CNT>=8) //间隔为800ms
                            {
                                TIME_CNT = 0;
                                USART_SendData(USART2,'s'); //向opv发送开始信号
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                            }
                            
                            if(read_color_flag==1) process_num = 1;
                        }
                        
//                            while(read_color_flag!=1);  //等待读取数据  //不能这么用！！！因为定时器的中断优先级会高，所有一直卡在定时器中断while里会无法进入串口接收中断
                        
                        else if(process_num==1)
                        {
                            /*根据color_num的数据确定亮灯程序并闪烁，用cnt--的方法代替延时函数*/
                            if(Bling_mode(led_num_times[color_num-'0'][0],led_num_times[color_num-'0'][1])==1)  //led操作完成返回值为1，后执行如下操作。特别注意有时不能与上一级if并用，因为&&关系中，只要有一个不满足条件就会直接跳出，不会执行该函数
                            {
                                read_color_flag = 0;
                                
                                buildings_color[2] = color_num;
                                USART_SendData(USART2,'e'); //向opv发送结束信号，opv在收到结束信号前只发一次数据，收到结束信号+下一个开始信号才能发下一个数据
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                                
                                process_num = 2;    //更新到下一个进程
                            }
                        }
                        
                        else if(process_num==2)  //已完成第一个子进程
                        {
                            /*舵机控制代码*/
                            Trun_SG90_angle(0);
                            TIME_CNT++;
                            if(TIME_CNT>=8)
                            {
                                TIME_CNT = 0;
                                USART_SendData(USART2,'s'); //向opv发送开始信号
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                            }
                            
                            if(read_color_flag==1)
                            {
                                /*根据color_num的数据确定亮灯程序并闪烁，用cnt--的方法代替延时函数*/
                                if(Bling_mode(led_num_times[color_num-'0'][0],led_num_times[color_num-'0'][1])==1)  //led操作完成返回值为1，后执行如下操作
                                {
                                    read_color_flag = 0;
                                
                                    process_num = 0;    //完成所有子进程
                                    update_flag = 1;    //允许更新目标坐标点
                                    
                                    buildings_color[3] = color_num;
                                    
                                    USART_SendData(USART2,'e'); //向opv发送结束信号
                                    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                                }
                            }
                        }
                    }
                		break;
                    case 6:
                    {
                        if(process_num==0)
                        {
                            /*舵机控制代码*/
                            Trun_SG90_angle(180);
                            TIME_CNT++;
                            if(TIME_CNT>=8) //间隔为800ms
                            {
                                TIME_CNT = 0;
                                USART_SendData(USART2,'s'); //向opv发送开始信号
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                            }
                            
                            if(read_color_flag==1) process_num = 1;
                        }
                        
//                            while(read_color_flag!=1);  //等待读取数据  //不能这么用！！！因为定时器的中断优先级会高，所有一直卡在定时器中断while里会无法进入串口接收中断
                        
                        else if(process_num==1)
                        {
                            /*根据color_num的数据确定亮灯程序并闪烁，用cnt--的方法代替延时函数*/
                            if(Bling_mode(led_num_times[color_num-'0'][0],led_num_times[color_num-'0'][1])==1)  //led操作完成返回值为1，后执行如下操作。特别注意有时不能与上一级if并用，因为&&关系中，只要有一个不满足条件就会直接跳出，不会执行该函数
                            {
                                read_color_flag = 0;
                                
                                buildings_color[4] = color_num;
                                USART_SendData(USART2,'e'); //向opv发送结束信号，opv在收到结束信号前只发一次数据，收到结束信号+下一个开始信号才能发下一个数据
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                                
                                process_num = 0;    //完成所有子进程
                                update_flag = 1;    //允许更新目标坐标点
                            }
                        }
                    }
                		break;
                    case 7:
                    {
                        if(process_num==0)
                        {
                            /*舵机控制代码*/
                            Trun_SG90_angle(180);
                            TIME_CNT++;
                            if(TIME_CNT>=8) //间隔为800ms
                            {
                                TIME_CNT = 0;
                                USART_SendData(USART2,'s'); //向opv发送开始信号
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                            }
                            
                            if(read_color_flag==1) process_num = 1;
                        }
                        
//                            while(read_color_flag!=1);  //等待读取数据  //不能这么用！！！因为定时器的中断优先级会高，所有一直卡在定时器中断while里会无法进入串口接收中断
                        
                        else if(process_num==1)
                        {
                            /*根据color_num的数据确定亮灯程序并闪烁，用cnt--的方法代替延时函数*/
                            if(Bling_mode(led_num_times[color_num-'0'][0],led_num_times[color_num-'0'][1])==1)  //led操作完成返回值为1，后执行如下操作。特别注意有时不能与上一级if并用，因为&&关系中，只要有一个不满足条件就会直接跳出，不会执行该函数
                            {
                                read_color_flag = 0;
                                
                                buildings_color[5] = color_num;
                                USART_SendData(USART2,'e'); //向opv发送结束信号，opv在收到结束信号前只发一次数据，收到结束信号+下一个开始信号才能发下一个数据
                                while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                                
                                process_num = 0;    //完成所有子进程
                                update_flag = 1;    //允许更新目标坐标点
                            }
                        }
                    }
                		break;
                    case 8:
                    {
                        if(buildings_color[5]=='0') //如果前一个点扫到的是无色，无图形，在这个点重新扫描
                        {
                            if(process_num==0)
                            {
                                /*舵机控制代码*/
                                Trun_SG90_angle(180);
                                TIME_CNT++;
                                if(TIME_CNT>=8) //间隔为800ms
                                {
                                    TIME_CNT = 0;
                                    USART_SendData(USART2,'s'); //向opv发送开始信号
                                    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                                }
                                
                                if(read_color_flag==1) process_num = 1;
                            }
                                                        
                            else if(process_num==1)
                            {
                                /*根据color_num的数据确定亮灯程序并闪烁，用cnt--的方法代替延时函数*/
                                if(Bling_mode(led_num_times[color_num-'0'][0],led_num_times[color_num-'0'][1])==1)  //led操作完成返回值为1，后执行如下操作。特别注意有时不能与上一级if并用，因为&&关系中，只要有一个不满足条件就会直接跳出，不会执行该函数
                                {
                                    read_color_flag = 0;
                                    
                                    buildings_color[5] = color_num;
                                    USART_SendData(USART2,'e'); //向opv发送结束信号，opv在收到结束信号前只发一次数据，收到结束信号+下一个开始信号才能发下一个数据
                                    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
                                    
                                    process_num = 0;    //完成所有子进程
                                    update_flag = 1;    //允许更新目标坐标点
                                }
                            }
                        }
                        else update_flag = 1;
                    }
                		break;
                	default:
                		break;
                }
                /*这里写的点都不是检测点，选择继续前进。只有上面switch里面的点是检测点*/
                if(update_flag==1 || Duty_flag_num==0 || Duty_flag_num==2 || Duty_flag_num==4 || Duty_flag_num==5 || Duty_flag_num==9)
                {
                    update_flag = 0;
                    Duty_flag_num++;
                }
                
                if(Duty_flag_num>=goal_num)    //测试，最多写SDK_POINT_TOTAL
                {
                    Stop_TB();
                    Store_colors();     //保存颜色数据到flash
                    SDK_Num_Reset();
                    *sdk_mode = 0xFF;       //这里把mode改成别的值，后续更改可以衔接别的任务
                }
            }
        }
        
    	default:
    		break;
    }
}

int16_t constrain(int16_t amt, int16_t low, int16_t high) 
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

float constrain_float(float amt, float low, float high) 
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
void wheel_speed_control(int16_t target_left_spd, int16_t target_right_spd)
{
    const int16_t Death_tmp_spd=3000;
    uint16_t Motor_val_l,Motor_val_r;
    int16_t tmp_left,tmp_right;
//    u8 flag=1;
//    if(flag) L_wheel.Last_Err=Speed_left,R_wheel.Last_Err=Speed_right,flag=0;
    
    L_wheel.Kp = R_wheel.Kp = 60.0f;   //100 110
    L_wheel.Kd = R_wheel.Kd = 0;
    L_wheel.Ki = R_wheel.Ki = 6.0f;    //10 11
    
    L_wheel.Err = (int)(target_left_spd - Speed_left);
    R_wheel.Err = (int)(target_right_spd - Speed_right);
    
    L_wheel.Integrate += L_wheel.Err;
    R_wheel.Integrate += R_wheel.Err;
    
    L_wheel.Integrate = constrain_float(L_wheel.Integrate,-420,420);    //积分限幅度 700
    R_wheel.Integrate = constrain_float(R_wheel.Integrate,-420,420);
    
    tmp_left = (int16_t)(L_wheel.Kp*L_wheel.Err + L_wheel.Kd*(L_wheel.Err-L_wheel.Last_Err) + L_wheel.Ki*L_wheel.Integrate);  //时间系数合并用作kd
    tmp_right = (int16_t)(R_wheel.Kp*R_wheel.Err + R_wheel.Kd*(R_wheel.Err-R_wheel.Last_Err) + R_wheel.Ki*R_wheel.Integrate);
    
    L_wheel.Last_Err = L_wheel.Err;
    R_wheel.Last_Err = R_wheel.Err;
    
    if(target_left_spd>=0 && target_right_spd>=0)
    {
        Motor_val_l = constrain(Death_tmp_spd+tmp_left,0,7200);
        Motor_val_r = constrain(Death_tmp_spd+tmp_right,0,7200);
        if(target_left_spd==0) Motor_val_l = 0;      
        if(target_right_spd==0) Motor_val_r = 0;
        Forward_TB(Motor_val_l,Motor_val_r);
    }
    else if(target_left_spd<0 && target_right_spd>=0)
    {
        Motor_val_l = constrain(Death_tmp_spd+(-1)*tmp_left,0,7200);
        Motor_val_r = constrain(Death_tmp_spd+tmp_right,0,7200);
        if(target_right_spd==0) Motor_val_r = 0;
        Turn_left(Motor_val_l,Motor_val_r);
    }
    else if(target_left_spd>=0 && target_right_spd<0)
    {
        Motor_val_l = constrain(Death_tmp_spd+tmp_left,0,7200);
        Motor_val_r = constrain(Death_tmp_spd+(-1)*tmp_right,0,7200);
        if(target_left_spd==0) Motor_val_l = 0;
        Turn_right(Motor_val_l,Motor_val_r);
    }
    else Stop_TB();
    
//    OLED_Show_Float_num(40,32,Motor_val_l,1);
//    OLED_Show_Float_num(40,48,Motor_val_r,1);
}

void move_with_xy_target_relatively(float x,float y,Duty_Status* Status,u8 number,float bia)  //使用时需转化为笛卡尔坐标系，这里最后一个参数表示人为允许的误差
{
    float relative_angle;
    
    Status[number].Start_Flag = 1;
    
    if(sqrtf(x*x+y*y)<=bia || STOP_flag==1) {Stop_TB();STOP_flag=1;return;} //省去误差范围内的原地自转
    
    From_relative_xy_to_angle(x,y,&relative_angle);
    
    if(Status[number].Start_Flag==1
     &&Status[number].Execute_Flag==1
       &&Status[number].End_flag==1)
      {
        POS_conctrol(x,y,relative_angle,bia);
        
        if(ABS(relative_angle)>=10.0f)    //防止偏移量过大
        {
            Stop_TB();
            Status[number].End_flag=0;
        }
        
        return;
      }
    else
    {                
        Turn_with_relative_angle(relative_angle);
        
        Status[number].Execute_Flag=1;
        
        if(Lidar_Car.yaw_ctrl_end==1)
        {
            Status[number].End_flag=1;
            Yaw_Angle_Control.Integrate = 0;    //积分清零
            L_wheel.Integrate = R_wheel.Integrate = 0;
        }
        
    }
    
      
}

void Turn_with_relative_angle(float angle)  //相对角度
{
    if(angle<-180)  angle=angle+360;
    if(angle>180)  angle=angle-360;
    
    if(ABS(angle)>=0.35f)
        {
            if(angle>=0)
            {
                Lidar_Car.yaw_ctrl_mode=ANTI_CLOCKWISE;
                Lidar_Car.yaw_ctrl_start=1;
                Lidar_Car.yaw_outer_control_output = angle;
            }
            else
            {
                Lidar_Car.yaw_ctrl_mode=CLOCKWISE;
                Lidar_Car.yaw_ctrl_start=1;
                Lidar_Car.yaw_outer_control_output = -1.0f*angle;
            }
        }
        else
        {
            Stop_TB();
            Lidar_Car.yaw_ctrl_mode=NEAR;
            Lidar_Car.yaw_ctrl_start=1;
            Lidar_Car.yaw_outer_control_output = 0;    //ABS(angle)
        }
        
        Angle_Control_Target(&Lidar_Car);
        Lidar_Car.yaw_outer_control_output = 0;
        Lidar_Car.yaw_ctrl_mode = NULL;
}

void From_relative_xy_to_angle(float x,float y,float* re_ang_dkr)   //x,y为相对坐标
{
    *re_ang_dkr = FastAtan2(y,x)*RADTODEG;  //因为是车身坐标系而非世界坐标系
    *re_ang_dkr = *re_ang_dkr - DKR.angle;
    if(*re_ang_dkr<-180)  *re_ang_dkr=*re_ang_dkr+360;
    if(*re_ang_dkr>180)  *re_ang_dkr=*re_ang_dkr-360;     //转动最小角度
}

void move_xy_barrier(float x,float y,float ba_dis,float ba_ang)    //前两个是目标点绝对坐标，后两个是障碍物相对坐标
{
    static u8 turning_flag;
    const float stop_before_dis=45.0f,stop_besides_dis=30.0f;   //这里的stop_before_dis调整成45比较好，43的话可能甚至在一个计时周期都没反应过来？
    static float cur_x,cur_y,ba_max_ang=90.0f,ba_min_ang=90.0f;
    double goal_dis;
    float ba_re_ang,ba_x,ba_y,k;
    
    goal_dis = sqrtf((x-DKR.x)*(x-DKR.x)+(y-DKR.y)*(y-DKR.y));
    if(goal_dis<=5.0f)  //如果到达最终目标点
    {
        Stop_TB();
        Reach_flag = 1;
        return;
    }
    
    ba_re_ang = ba_ang - 90;
    ba_x = FastCos((180-DKR.angle-ba_re_ang)*DEGTORAD)*ba_dis;    //虽然编译器不会报错，但最好在Data_analyse.h里加extern const float sinTable[];
    ba_y = FastSin((180-DKR.angle-ba_re_ang)*DEGTORAD)*ba_dis;
    
    if(ba_dis<=stop_before_dis && ba_re_ang>=0)//障碍物在路径的左前方  当激光雷达扫描角度限制为较小范围时就不用担心连续扫出的最近点影响，导致Duty_flag[Duty_flag_num].End_flag一直清零无法前进
    {
        Stop_TB();      //这里是人为控制停车，STOP_flag并没有变为1
        if(ba_max_ang<=ba_ang)
        {
            ba_max_ang = ba_ang;
            k = stop_besides_dis/ba_dis;
            cur_x = DKR.x - ba_x + k*ba_y;
            cur_y = DKR.y + k*ba_x + ba_y;
            turning_flag = 1;
            Duty_flag[Duty_flag_num].End_flag = 0;  //这里更新完坐标后需要解除转弯限制，也从侧面反映出有两种方式更新点的坐标，这是第一种
        }
    }
    else if(ba_dis<=stop_before_dis && ba_re_ang<0)//障碍物在路径的右前方
    {
        Stop_TB();      //这里是人为控制停车，STOP_flag并没有变为1
        if(ba_min_ang>=ba_ang)
        {
            ba_min_ang = ba_ang;
            k = stop_besides_dis/ba_dis;
            cur_x = DKR.x - ba_x - k*ba_y;
            cur_y = DKR.y - k*ba_x + ba_y ;
            turning_flag = 1;
            Duty_flag[Duty_flag_num].End_flag = 0;
        }
    }
    else if(ba_dis>stop_before_dis)
    {
        if(turning_flag==0)
        {
            cur_x = x;
            cur_y = y;
        }
        ba_max_ang=90.0f;
        ba_min_ang=90.0f;
    }
    move_with_xy_target_relatively(cur_x-DKR.x, cur_y-DKR.y, Duty_flag, Duty_flag_num, 5);
    if(STOP_flag==1)
    {
        STOP_flag = 0;
        turning_flag = 0;
//        Duty_flag_num++;        //到一个点后，改变num参数数值，以免小车原地不动，这是第二种更新点的方式，几乎等效于第一种，但有标号更新方便后续追踪数据
        Duty_flag[Duty_flag_num].End_flag = 0;
    }
    
}

void move_xy_barrier1(float x,float y,float ba_dis,float ba_ang)    //前两个是目标点绝对坐标，后两个是障碍物相对坐标
{
    static u8 turning_flag;
    const float stop_before_dis=45.0f;   //这里的stop_before_dis调整成45比较好，43的话可能甚至在一个计时周期都没反应过来？
    double goal_dis;
    float re_ang;
    
    goal_dis = sqrtf((x-DKR.x)*(x-DKR.x)+(y-DKR.y)*(y-DKR.y));
    if(goal_dis<=5.0f)  //如果到达最终目标点
    {
        Stop_TB();
        return;
    }
    
    if((ba_dis<=stop_before_dis && ba_ang>=90 && ba_ang<=180) || turning_flag==1)//障碍物在路径的左前方  当激光雷达扫描角度限制为较小范围时就不用担心连续扫出的最近点影响，导致Duty_flag[Duty_flag_num].End_flag一直清零无法前进
    {
        Stop_TB();      //这里是人为控制停车，STOP_flag并没有变为1
        re_ang = ba_ang-45.0f;      //这里超出了激光雷达扫描范围，所以ba_ang数据会有问题
        Turn_with_relative_angle(re_ang);
        if(Lidar_Car.yaw_ctrl_end==0) turning_flag = 1;    //如果没有转完
        else
        {
            turning_flag = 0;
            move_with_xy_target_relatively(x-DKR.x,y-DKR.y,Duty_flag,0,5);  //按理说起步之后就只会用到里面的POS_conctrol
        }
    }
    else if((ba_dis<=stop_before_dis && ba_ang>=0 && ba_ang<90) || turning_flag==2)//障碍物在路径的右前方
    {
        Stop_TB();      //这里是人为控制停车，STOP_flag并没有变为1
        re_ang = ba_ang+45.0f;
        Turn_with_relative_angle(re_ang);
        if(Lidar_Car.yaw_ctrl_end==0) turning_flag = 2;
        else
        {
            turning_flag = 0;
            move_with_xy_target_relatively(x-DKR.x,y-DKR.y,Duty_flag,0,5);
        }
    }
    else if(turning_flag==0 && ba_dis>stop_before_dis)
    {
        move_with_xy_target_relatively(x-DKR.x,y-DKR.y,Duty_flag,0,5);
    }    
}

void Move_around(float x,float y,float re_dis,float re_ang_dkr,float bia)   //这里传的是绝对坐标，角度是相对于车身坐标系的DKR角度
{
    static u8 sub_demo[4]={0},state_flag;
    static float Turn_x,Turn_y;
    float re_ang,tmp_left,tmp_right,dis_from_Turn;
    float re_ang_dkr1;
    
    From_relative_xy_to_angle(x-DKR.x,y-DKR.y,&re_ang_dkr1);    
    if(sub_demo[0]==0)
    {
        if(STOP_flag==0)    //未到达指定旋转点
        {
            move_with_xy_target_relatively(x-DKR.x, y-DKR.y, Duty_flag, 0, bia);    //转化成相对坐标
        }
        else
        {
                re_ang = re_ang_dkr1-90.0f; //调正的基础上向右转90度
                
                if(re_ang<-180)  re_ang=re_ang+360;
                if(re_ang>180)  re_ang=re_ang-360;     //要在判断前限幅！否则永远无法达到平衡，之前是在判断之后才在PID里限幅，会出错
                
                
                Turn_with_relative_angle(re_ang);
            
                if(Lidar_Car.yaw_ctrl_end==1)
                {
                    STOP_flag = 0;                      //标志清零
                    Yaw_Angle_Control.Integrate = 0;    //积分清零
                    L_wheel.Integrate = R_wheel.Integrate = 0;
                    Stop_TB();
                    Turn_x = DKR.x;                     //记录当前坐标        
                    Turn_y = DKR.y;
                    sub_demo[0]=1;

                }
                        
            
            /*
            原因分析
            现象：不加限幅修正前，re_ang会从最接近的-15.几的数据突变到-370左右
            原因：最本质的问题还是DKR.angle从0到360的突变，导致数据突变。那么为什么最接近的数据不是0而是-15点几呢？
            因为转弯时的点可能有一定的偏差，所以算出来的FastAtan2(y-DKR.y,x-DKR.x)*RADTODEG不是45度，也可能是70多度
            假设最接近的数就是-15，那么FastAtan2(y-DKR.y,x-DKR.x)*RADTODEG算出的就是75度，也就是说现在的点位于原点和目标点连线的下方
            */
            
        }
    }
    
    else if(sub_demo[1]==0)
    {
        TIME_CNT++;
        if(TIME_CNT==10)
        {
            sub_demo[1] = 1;
            TIME_CNT = 0;
        }
    }
    
    else if(sub_demo[2]==0)
    {
        re_ang = re_ang_dkr1-90.0f; //调正的基础上向右转90度
            
        if(re_ang<-180)  re_ang=re_ang+360;
        if(re_ang>180)  re_ang=re_ang-360;     //要在判断前限幅！否则永远无法达到平衡，之前是在判断之后才在PID里限幅，会出错
        
        Yaw_Angle_Control.Err = ABS(re_ang);
                
        tmp_left = 45;
        tmp_right = tmp_left*(27+BODY_LENTH/2)/(27-BODY_LENTH/2);   //这里常数可以改成re_dis1
        
        wheel_speed_control(tmp_left,tmp_right);
        
        dis_from_Turn = sqrtf((Turn_x-DKR.x)*(Turn_x-DKR.x)+(Turn_y-DKR.y)*(Turn_y-DKR.y));
        
        if(state_flag==0 && dis_from_Turn>10.0f) //已经离开转弯点
        {
            state_flag = 1;
//            GPIO_ResetBits(GPIOE,GPIO_Pin_5);   //亮灯
//            GPIO_ResetBits(GPIOB,GPIO_Pin_9);
        }
        else if(state_flag==1 && dis_from_Turn<=10.0f)
        {
            state_flag = 0;
            Stop_TB();
            Yaw_Angle_Control.Integrate = 0;
            L_wheel.Integrate = R_wheel.Integrate = 0;
            sub_demo[2]=1;
//            GPIO_SetBits(GPIOE,GPIO_Pin_5);   //灭灯
//            GPIO_SetBits(GPIOB,GPIO_Pin_9);
        }
    }

    else if(sub_demo[3]==0)
    {
        move_with_xy_target_relatively(-DKR.x, -DKR.y, Duty_flag, 1, 5);    //这里的num参数记得要改成1，否则车是不会动的！
        if(STOP_flag)
        {
            STOP_flag = 0;
            sub_demo[3] = 1;
        }
    }
}


void POS_conctrol(float x,float y,float Relative_angle,float bia)
{
    float tmp_dis,tmp_angle,tmp_left,tmp_right;
    float distance;
    const float kp=0.35f,Moving_spd=70,Max_Moving_spd=150;//(0.15 45 100)(后驱)   //改直线速度就调大 Moving_spd，调小偏航角Control_OutPut_Limit就会更丝滑
    //Moving_spd=45 Max_Moving_spd=80
    distance = x*x+y*y;
    distance = sqrtf(distance);
    tmp_dis = distance*kp;
    tmp_dis = constrain_float(tmp_left,0,80);  //50
    Yaw_Angle_Control.Err = ABS(Relative_angle);
    tmp_angle = PID_Control_Yaw(&Yaw_Angle_Control,1);//偏航角度控制
    
    if(distance<=bia || STOP_flag==1) {Stop_TB();STOP_flag=1;return;}
    else if(Relative_angle>=0)
    {
        tmp_left = Moving_spd+tmp_dis-tmp_angle;        //45
        tmp_right = Moving_spd+tmp_dis+tmp_angle;
    }
    else if(Relative_angle<0)
    {
        tmp_left = Moving_spd+tmp_dis+tmp_angle;
        tmp_right = Moving_spd+tmp_dis-tmp_angle;
    }
    
    tmp_left = constrain_float(tmp_left,0,Max_Moving_spd);
    tmp_right = constrain_float(tmp_right,0,Max_Moving_spd);

    wheel_speed_control((int16_t)tmp_left,(int16_t)tmp_right);
}

float PID_Control_Yaw(PID_Controler *Controler,u8 mode) //记得要初始化
{	
  /*系数设置*/
    if(mode==0)
    {
        Controler->Kp = 0;
        Controler->Ki = 0.5f;
        Controler->Kd = 100.0f;
    }
    else if(mode==1)
    {
        Controler->Kp = 9.0f;       //3 5 7 9(后轮)
        Controler->Ki = 3.5f;       //2 5 3.5(后轮)
        Controler->Kd = 160.0f;      //2 40 80 160(后轮)
    }
	
  /***********************偏航角偏差超过+-180处理*****************************/
	if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
	if(Controler->Err>180)  Controler->Err=Controler->Err-360;
	
  /*******积分计算*********************/
//    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
    Controler->Integrate+=Controler->Ki*Controler->Err;     //dt以定时器设定时间为准
  /*******积分限幅*********************/
    Controler->Integrate_Max = 200;

    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
    
    
    
  /*******总输出计算*********************/
    if(mode==1)
  Controler->Control_OutPut=Controler->Kp*Controler->Err//比例
														+Controler->Integrate//积分
															+Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
    else if(mode==0)
        Controler->Control_OutPut=                                          10 //定值
														+Controler->Integrate//积分
															+Controler->Kd*(Controler->Err-Controler->Last_Err);//微分
    
  Controler->Last_Err=Controler->Err;//保存上次偏差
  /*******总输出限幅*********************/
    
    if(mode==0) Controler->Control_OutPut_Limit = 25;   //20
    else if(mode==1) Controler->Control_OutPut_Limit = 15;  //22
    
    
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<0)
    Controler->Control_OutPut=0;
  /*******返回总输出*********************/
  return Controler->Control_OutPut;
}

void Angle_Control_Target(Controller_Output *_car_output)    //放入定时器解决，记得要在read_flag后
{
//    static u8 cnt_death;
    const float death_speed=9.0f,Max_speed=13.0f;  //(10 14) (10,20) (9,15)(后轮)
    switch(_car_output->yaw_ctrl_mode)
    {
        case CLOCKWISE://顺时针————相对给定时刻的航向角度
		{
			if(_car_output->yaw_ctrl_start==1)//更新偏航角度期望
			{
                Yaw_Angle_Control.Err = _car_output->yaw_outer_control_output;
				_car_output->yaw_ctrl_start=0;
//				_car_output->yaw_ctrl_cnt=0;     //因为这里不是一次性就直接到位，而是分多次执行，所以不能清零
				_car_output->yaw_ctrl_end=0;
			}
			
			if(_car_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
			{
                
                if(ABS(Yaw_Angle_Control.Err)<3.0f) _car_output->yaw_ctrl_cnt++;
//				else _car_output->yaw_ctrl_cnt/=2;   //车的死区存在的情况下减少精度要求，抑制连续不止的震荡
				
				if(_car_output->yaw_ctrl_cnt>=2)     //不要调大了！！！振荡太多！！！
                {
//                    cnt_death=0;
                    _car_output->yaw_ctrl_cnt=0;
                    _car_output->yaw_ctrl_end=1;
                    Stop_TB();
                }
                    
			}
			
			Yaw_Angle_Control.FeedBack=DKR.angle;//偏航角反馈
			PID_Control_Yaw(&Yaw_Angle_Control,0);//偏航角度控制
			//对最大偏航角速度进行限制
            float tmp = Yaw_Angle_Control.Control_OutPut;   //tmp应该大于0
            tmp=ABS(tmp);
            tmp+=death_speed;
            if(tmp>=Max_speed) tmp = Max_speed;
            if(_car_output->yaw_ctrl_end==0) wheel_speed_control((int16_t)tmp,(int16_t)-tmp);
		}
		break;
		case ANTI_CLOCKWISE://逆时针——相对给定时刻的航向角度
		{
			if(_car_output->yaw_ctrl_start==1)//更新偏航角度期望
			{
				Yaw_Angle_Control.Err = _car_output->yaw_outer_control_output;
				_car_output->yaw_ctrl_start=0;
//				_car_output->yaw_ctrl_cnt=0;     //因为这里不是一次性就直接到位，而是分多次执行，所以不能清零
				_car_output->yaw_ctrl_end=0;
			}
			
			if(_car_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
			{
                if(ABS(Yaw_Angle_Control.Err)<3.0f) _car_output->yaw_ctrl_cnt++;
//				else _car_output->yaw_ctrl_cnt/=2;
				
				if(_car_output->yaw_ctrl_cnt>=2)
                {
                    _car_output->yaw_ctrl_cnt=0;
                    _car_output->yaw_ctrl_end=1;
                    Stop_TB();
                }
                                    
			}
			
			Yaw_Angle_Control.FeedBack=DKR.angle;//偏航角反馈
			PID_Control_Yaw(&Yaw_Angle_Control,0);//偏航角度控制
			//对最大偏航角速度进行限制
            float tmp = Yaw_Angle_Control.Control_OutPut;   //tmp应该大于0
            tmp=ABS(tmp);
            tmp+=death_speed;
            if(tmp>=Max_speed) tmp = Max_speed;
            if(_car_output->yaw_ctrl_end==0) wheel_speed_control((int16_t)-tmp,(int16_t)tmp); //注意，这里的左轮很离谱，反转经常卡死，且启动后跟不上右轮，属于特殊情况！！！
		}
		break;
        
        case NEAR:
        {
			if(_car_output->yaw_ctrl_start==1)//更新偏航角度期望
			{
				Yaw_Angle_Control.Err = _car_output->yaw_outer_control_output;
				_car_output->yaw_ctrl_start=0;
				_car_output->yaw_ctrl_end=0;
			}
			
			if(_car_output->yaw_ctrl_end==0)//判断偏航角是否控制完毕
			{
                if(ABS(Yaw_Angle_Control.Err)<0.35f) _car_output->yaw_ctrl_cnt++;
//				else _car_output->yaw_ctrl_cnt/=2;
				
				if(_car_output->yaw_ctrl_cnt>=2)
                {
                    _car_output->yaw_ctrl_cnt=0;
                    _car_output->yaw_ctrl_end=1;
                    Stop_TB();
                }
                Stop_TB();                    
			}
            
		}
		break;
        default: break;
    }
    
    return;
}






















