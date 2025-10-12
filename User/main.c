#include "stm32f10x.h"
#include "Delay.h"
#include "Motor.h"
#include "Control.h"
#include "Speed.h"
#include "ADC.h"
#include "Data_analyse.h"
#include "oled1.h"
#include "led.h"
#include "key.h"
#include "Store.h"
#include "Steering.h"
#include "Usart.h"
#include <math.h>
#include <string.h>

/*
设定CNT基础值为0x7FFF
看着轮子，顺时针，减；逆时针，加
按照上面规则来看
前进时，右轮为减，左轮为加
如果想整体减速，右轮的后一次减前一次为正数；左轮的后一次减前一次为负数
*/
extern uint8_t fflag;
uint16_t ADC_ConvertedValue[2];
int16_t Analogue_PID_VAL;
uint8_t condition,keynum,opv_x,opv_y,read_color_flag,color_num;
extern uint8_t read_flag,read_flag_opv;
extern uint8_t TimerClear;
extern uint8_t Judge;
extern uint8_t Launch_flag;
extern third_party_state current_state,barrier_state;
extern Descartes_cord DKR;
extern Vector2f Fire_position;
extern float Min_dis,Min_ang;
extern float x_start_offset,y_start_offset;
extern uint8_t Function_Data_UART4;
void TIM_Init(void);
void gpio_init(void);
void Map_Create(void);
void DJST_Algo(uint8_t s_label);
void DJST_Algo2(uint8_t s_label,uint8_t e_label);

u8 SDK_mode=0,point_num=0,goal_num=1;   //goal_num为要跑的点的个数
RE_pos re_pos;
//DJST变量
DJST_NodeType DJST_Node[NODE_NUM];
uint8_t Min_compute_flag=0;
float Min_dis_for_DJST=INF;
float Fire_to_Node_dis;
uint8_t Min_num_for_DJST=0;     //距离火源最小距离的点标号
uint8_t Pre_node_num[NODE_NUM];

const Vector2f Node_crd[NODE_NUM]={ //节点坐标
    135,25,
    100,100,      //1
    50,100,
    35,150,
    50,230,
    120,230,
    170,230,      //6

    170,155,
    170,100,
    255,100,
    340,100,      //10

    410,40,
    340,155,
    340,230,
    410,230,
    270,230,     //15

    340,305,
    410,360,
    270,360,
    200,305,
    200,360,    //20

    115,360,
    50,360,
    50,305,
    340,360,    
    340,40     //25
};

const uint8_t Incident_Matrix[NODE_NUM][NODE_NUM]={
    1,	1,	1,	0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,
    1,	1,	1,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,
    0,	0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,
    0,	0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    1,	1,	0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    1,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    1,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	1,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	1,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0,	1,	0,	0,	0,	1,	0,
    0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	0,	0,	0,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	0,	0,	0,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	0,	0,
    0,	0,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	0,
    0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	0,	0,	0,	0,	0,	1,	0,
    1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1
};

float Node_Map[NODE_NUM][NODE_NUM];

extern uint8_t Function_Data_UART4,Read_Whole_Data_Flag_UART4,Num_Data_UART4;
extern Vector2f Manual_Patrol_Node[];
int main(void)
{
    SDK_Num_Reset();
    OLED_Init();
    Store_Init();
    gpio_init();
	uart();
    LED_Init();
    KEY_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
//    Map_Create();
//    DJST_Algo(0);    //第0个点作为原点
    
    if(USART_HMI==1) TIM_Init();
        
//    Launch_flag=1;
//    Fire_position.x=105;
//    Fire_position.y=270;
    
    OLED_ShowString(0,0,"mode:",16);
    OLED_Refresh_Gram();
    
    
//    Forward_TB(7200,7200);  //应该低于原来车转速
    //注意！！！小车电机顺序已在Motor.c中修改
    
    
//    while(read_flag!=1 || Launch_flag!=1);    //一直循环到能读取数据
    while(read_flag!=1)    //没有车机协同任务使用   
    {
        OLED_ShowNum(40,0,Function_Data_UART4,2,16,1);
        OLED_Refresh_Gram();
    }
    
//        OLED_ShowString(0,0," x :",16);
        OLED_ShowString(0,16," x :",16);
        OLED_ShowString(0,32," y :",16);
        OLED_ShowString(0,48,"yaw:",16);
        OLED_Refresh_Gram();    
    
//    if(Min_compute_flag==0)
//    {
//        for(uint16_t i=0;i<NODE_NUM;i++)
//        {
//            //不要用sqrt，进口32芯片可跑，但国产芯片算力不足，会在莫名其妙的地方卡住
//            Fire_to_Node_dis=sqrtf((Fire_position.x-Node_crd[i].x)*(Fire_position.x-Node_crd[i].x)+(Fire_position.y-Node_crd[i].y)*(Fire_position.y-Node_crd[i].y));
//            if(Min_dis_for_DJST>Fire_to_Node_dis)
//                {
//                    Min_num_for_DJST=i;
//                    Min_dis_for_DJST=Fire_to_Node_dis;
//                }
//        }
//        DJST_Algo2(0,Min_num_for_DJST);
//        Min_compute_flag=1;
//    }
    
//    OLED_ShowString(0,0,"mode:",16);
//    OLED_ShowNum(40,16,SDK_mode,2,16,1);
//    OLED_Refresh_Gram();
    
#if USART_HMI==0
    while(keynum!=WKUP_PRES)
    {
        keynum = KEY_Scan(0);
        if(keynum==KEY0_PRES)
        {
            SDK_mode++;
            SDK_mode%=SDK_NUM_TOTAL;
            OLED_ShowNum(40,16,SDK_mode,2,16,1);
            OLED_Refresh_Gram();
        }
    }
    OLED_Clear();
    
    keynum = 0;
    
    if(SDK_mode==0x02 || SDK_mode==0x03 || SDK_mode==0x04)
    {
        OLED_ShowString(0,16,"goal:",16);
        OLED_ShowNum(40,16,goal_num,2,16,1);
        OLED_Refresh_Gram();
        
        while(keynum!=WKUP_PRES)
        {
            keynum = KEY_Scan(0);
            if(keynum==KEY0_PRES)
            {
                goal_num++;
                if(goal_num>=SDK_POINT_TOTAL+1) goal_num=1;
                OLED_ShowNum(40,16,goal_num,2,16,1);
                OLED_Refresh_Gram();
            }
        }
        OLED_Clear();
        
        x_start_offset = DKR.x; //记录初始偏移量
        y_start_offset = DKR.y;
    }
    else if(SDK_mode==0x06)
    {
        goal_num = 10;  //写死了10个点
        x_start_offset = DKR.x; //记录初始偏移量
        y_start_offset = DKR.y;
    }
    
    if(SDK_mode!=0x00 && SDK_mode!=0x01 && SDK_mode!=0x05)
    {
        TIM_Init();  //录点模式下不开启定时器
        OLED_ShowString(0,0," x :",16);
        OLED_ShowString(0,16," y :",16);
        OLED_ShowString(0,32,"bia:",16);
        OLED_ShowString(0,48,"yaw:",16);
        OLED_Refresh_Gram();
    }
    else if(SDK_mode==0x00)
    {
        OLED_ShowString(0,0," x :",16);
        OLED_ShowString(0,16," y :",16);
        OLED_ShowString(0,32," p :",16);
        OLED_ShowString(0,48,"state:",16);
        OLED_Refresh_Gram();
    }        
    else if(SDK_mode==0x01)
    {
        OLED_ShowString(0,0," x :",16);
        OLED_ShowString(0,16," y :",16);
        OLED_ShowString(0,32," p :",16);
        OLED_Refresh_Gram();
    }
    else if(SDK_mode==0x05)
    {
        OLED_ShowString(0,0,"col:",16);
        OLED_ShowString(0,16,"num:",16);
        OLED_Refresh_Gram();
    }

    if(SDK_mode!=0x00 && SDK_mode!=0x01 && SDK_mode!=0x05) Delay_s(3);
#endif
    
	while(1)
	{
        if(read_flag==1 && SDK_mode!=0x01 && SDK_mode!=0x05)  //模式1和模式5为读取点，不需要更新显示
        {            
            if(re_pos.start_flag)
            {
                re_pos.start_flag = 0;
                re_pos.ab_x = DKR.x + re_pos.re_x;
                re_pos.ab_y = DKR.y + re_pos.re_y;
            }
            
//            OLED_Show_Float_num(40,0,DKR.x,5);
            OLED_Show_Float_num(40,16,DKR.x,2);
            OLED_Show_Float_num(40,32,DKR.y,2);
            OLED_Show_Float_num(40,48,DKR.angle,2);

            OLED_Refresh_Gram();
            
            read_flag = 2;  //防止定时器打断
        }
        
#if USART_HMI==0
        if(SDK_mode==0x00)
        {
            keynum = KEY_Scan(0);
            if(keynum==KEY0_PRES)
            {
                if(point_num>=SDK_POINT_TOTAL)    //OLED显示内容有限
                {
                    point_num = 0;
                    OLED_ShowString(32,32,"            ",16);
//                    OLED_Refresh_Gram();
                }
                else
                {
                    OLED_ShowNum(32+point_num*8,32,point_num,1,16,1);
//                    OLED_Refresh_Gram();
                    store_point[point_num].x = DKR.x;
                    store_point[point_num].y = DKR.y;
                    Store_a_point(point_num);
                    point_num++;
                }
            }
            else if(keynum==WKUP_PRES)
            {
                Store_Save();
                OLED_ShowString(56,48,"finish",16);
                OLED_Refresh_Gram();
            }
        }
        else if(SDK_mode==0x01)
        {
            keynum = KEY_Scan(0);
            if(keynum==KEY0_PRES)
            {
                point_num++;
                point_num%=SDK_POINT_TOTAL;
            }
            else if(keynum==WKUP_PRES)
            {
                if(point_num==0) point_num = 9;
                else point_num--;
            }
//            Get_a_point(point_num);       //已经在Store_Init函数里面完成了
            OLED_Show_Float_num(40,0,store_point[point_num].x,5);
            OLED_Show_Float_num(40,16,store_point[point_num].y,5);
            OLED_ShowNum(40,32,point_num,1,16,1);
            OLED_Refresh_Gram();
        }
        else if(SDK_mode==0x05)
        {
            keynum = KEY_Scan(0);
            if(keynum==KEY0_PRES)
            {
                point_num++;
                point_num%=6;  //目前有六栋建筑
            }
            else if(keynum==WKUP_PRES)
            {
                if(point_num==0) point_num = 5;
                else point_num--;
            }
            
            switch(buildings_color[point_num])
            {
            	case '0':
                    OLED_ShowString(40,0,"none",16);
            		break;
                case '1':
                    OLED_ShowString(40,0,"R-cir",16);
            		break;
                case '2':
                    OLED_ShowString(40,0,"B-cir",16);
            		break;
                case '3':
                    OLED_ShowString(40,0,"B-trg",16);
            		break;
                case '4':
                    OLED_ShowString(40,0,"G-trg",16);
            		break;
                case '5':
                    OLED_ShowString(40,0,"G-sqr",16);
            		break;
            	case '6':
                    OLED_ShowString(40,0,"R-sqr",16);
            		break;
            	default:
            		break;
            }
            OLED_ShowNum(40,16,point_num,1,16,1);
            OLED_Refresh_Gram();
        }
#endif
    }
}

void TIM_Init()
{
    Encoder_Init_TIM4();
    Encoder_Init_TIM1();
    TIM8_PWM_Init();
    TIM2_PWM_Init(48000,3);
    TIM5_Init();
    TIM3_Init();
}

void gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOB,ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
   
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//mini板LED初始化
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
}

void Map_Create()
{
    for(uint16_t i=0;i<NODE_NUM;i++)
        for(uint16_t j=i;j<NODE_NUM;j++)
            if(Incident_Matrix[i][j]==1) Node_Map[i][j]=Node_Map[j][i]=sqrtf((Node_crd[i].x-Node_crd[j].x)*(Node_crd[i].x-Node_crd[j].x)+(Node_crd[i].y-Node_crd[j].y)*(Node_crd[i].y-Node_crd[j].y));
            else Node_Map[i][j]=Node_Map[j][i]=INF;
}

void DJST_Algo(uint8_t s_label)
{
    static uint8_t iFvisit[NODE_NUM];
    static float dis_to_origin[NODE_NUM];
    uint8_t perpetual_label; //永久节点标号
    float min_dis;
    
    memset(iFvisit,0,sizeof(iFvisit));
//    memset(dis_to_origin,0x3f,sizeof(dis_to_origin)); //错误写法
    for(uint16_t i=0;i<NODE_NUM;i++) dis_to_origin[i] = INF;
    dis_to_origin[s_label]=0;
    
    while(1)
    {
        perpetual_label = NODE_NUM;
        min_dis = INF;
        
        for(uint16_t i=0;i<NODE_NUM;i++)
            if(iFvisit[i]==0 && min_dis>dis_to_origin[i])   //选取离永久标号最近的临时标号，将其更新为永久标号
            {
                perpetual_label = i;
                min_dis = dis_to_origin[i];
            }
            
        if(perpetual_label==NODE_NUM) break;   //没有标号更新
        else iFvisit[perpetual_label] = 1;
        
        for(uint16_t i=0;i<NODE_NUM;i++)
            if(dis_to_origin[i]>dis_to_origin[perpetual_label]+Node_Map[perpetual_label][i])
            {
                dis_to_origin[i] = dis_to_origin[perpetual_label]+Node_Map[perpetual_label][i]; //更新了距离的点相当于临时标号
                DJST_Node[i].Min_dis_to_origin = dis_to_origin[i];
                DJST_Node[i].Pre_node_label = perpetual_label;
            }
    }
}

void DJST_Algo2(uint8_t s_label,uint8_t e_label)
{
    uint8_t cur_label,a[NODE_NUM],cnt=0;
    
    memset(Pre_node_num,0,sizeof(Pre_node_num));
    cur_label = e_label;
        
    while(cur_label!=s_label)
    {
        a[cnt++] = cur_label;
        cur_label = DJST_Node[cur_label].Pre_node_label;
    }
    a[cnt++] = cur_label;   //把原点也加上
    DJST_Node[e_label].Pre_node_cnt = cnt;
    
    for(uint8_t i=0;i<cnt;i++)
        Pre_node_num[i] = a[cnt-1-i];
}

//    Get_a_point(0);
//    test_x = store_point[0].x;
//    test_y = store_point[0].y;
//    OLED_Show_Float_num(40,0,test_x,5);
//    OLED_Show_Float_num(40,16,test_y,5);
//    
//    Get_a_point(1);
//    test_x = store_point[1].x;
//    test_y = store_point[1].y;
//    OLED_Show_Float_num(40,32,test_x,5);
//    OLED_Show_Float_num(40,48,test_y,5);
//    OLED_Refresh_Gram();
    
//    store_point[0].x = 534.567f;
//    store_point[0].y = 234.567f;
//    store_point[1].x = 183.456f;
//    store_point[1].y = 2304.567f;
//    Store_a_point(0);
//    Store_a_point(1);
//    Store_Save();
    
//    while(read_flag!=1);    //一直循环到能读取数据
//    
//    while(keynum!=KEY0_PRES)
//    {
//        keynum = KEY_Scan(0);
//        OLED_Show_Float_num(40,0,DKR.x,5);
//        OLED_Show_Float_num(40,16,DKR.y,5);
//        OLED_Show_Float_num(40,32,barrier_state.position_x,5);
//        OLED_Show_Float_num(40,48,barrier_state.position_y,5);
//        OLED_Refresh_Gram();
//    }

//    if(keynum==KEY0_PRES)
//    {
//        re_pos.re_x = barrier_state.position_x;
//        re_pos.re_y = barrier_state.position_y;
//        re_pos.start_flag = 1;
//    }
//    
////    re_pos.re_x = barrier_state.position_x;
////    re_pos.re_y = barrier_state.position_y;
////    re_pos.start_flag = 1;
//    
//    if(re_pos.start_flag)
//    {
//        re_pos.start_flag = 0;
//        re_pos.ab_x = DKR.x + re_pos.re_x;
//        re_pos.ab_y = DKR.y + re_pos.re_y;
//    }
//    
//    OLED_Show_Float_num(40,32,re_pos.ab_x,5);
//    OLED_Show_Float_num(40,48,re_pos.ab_y,5);
    
    
//    re_pos.re_x = 150.0f;
//    re_pos.re_y = 150.0f;
//    re_pos.start_flag = 1;

//C语言中似乎没能很好的找到二维指针转二维数组的方法（包括申请动态内存）
//void Map_Create(const Vector2f* Crd,const uint8_t** INM,float** Map)
//{
//    for(uint16_t i=0;i<NODE_NUM;i++)
//        for(uint16_t j=i;j<NODE_NUM;j++)
//            if(INM[i][j]==1) Map[i][j]=Map[j][i]=sqrtf((Crd[i].x-Crd[j].x)*(Crd[i].x-Crd[j].x)+(Crd[i].y-Crd[j].y)*(Crd[i].y-Crd[j].y));
//            else Map[i][j]=Map[j][i]=INF;
//}

//void DJST_Algo(uint8_t s_label,DJST_NodeType** NODE)
//{
//    static uint8_t iFvisit[NODE_NUM];
//    static float dis_to_origin[NODE_NUM];
//    uint8_t perpetual_label; //永久节点标号
//    float min_dis;
//    
//    memset(iFvisit,0,sizeof(iFvisit));
//    memset(dis_to_origin,INF,sizeof(dis_to_origin));
//    dis_to_origin[s_label]=0;
//    
//    while(1)
//    {
//        perpetual_label = NODE_NUM;
//        min_dis = INF;
//        
//        for(uint16_t i=0;i<NODE_NUM;i++)
//            if(iFvisit[i]==0 && min_dis>dis_to_origin[i])   //选取离永久标号最近的临时标号，将其更新为永久标号
//            {
//                perpetual_label = i;
//                min_dis = dis_to_origin[i];
//            }
//            
//        if(perpetual_label==NODE_NUM) break;   //没有标号更新
//        else iFvisit[perpetual_label] = 1;
//        
//        for(uint16_t i=0;i<NODE_NUM;i++)
//            if(dis_to_origin[i]>dis_to_origin[perpetual_label]+Node_Map[perpetual_label][i])
//            {
//                dis_to_origin[i] = dis_to_origin[perpetual_label]+Node_Map[perpetual_label][i]; //更新了距离的点相当于临时标号
//                NODE[i]->Min_dis_to_origin = dis_to_origin[i];
//                NODE[i]->Pre_node_label = perpetual_label;    //暂时将前一个点储存在数组元素0上
//                NODE[i]->Pre_node_cnt++;
//            }
//    }
//}

