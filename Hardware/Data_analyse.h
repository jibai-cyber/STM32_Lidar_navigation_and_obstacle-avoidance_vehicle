#ifndef _DATA_ANALYSE_H
#define _DATA_ANALYSE_H

#define PI 3.1415926
#define PI_2 (1.5707963267948966192313216916398f)
#define PI_3 (1.0471975511965977461542144610932f)
#define PI_4 (0.78539816339744830961566084581988f)
#define PI_6 (0.52359877559829887307710723054658f)
#define TWO_MINUS_ROOT3 (0.26794919243112270647255365849413f)
#define SQRT3_MINUS_1 (0.73205080756887729352744634150587f)
#define SQRT3 (1.7320508075688772935274463415059f)
#define EPS_FLOAT (+3.452669830012e-4f)
//Coefficients used for atan/atan2
#define ATANP_COEF0 (-1.44008344874f)
#define ATANP_COEF1 (-7.20026848898e-1f)
#define ATANQ_COEF0 (+4.32025038919f)
#define ATANQ_COEF1 (+4.75222584599f)
//Coefficients used for asin/acos
#define ASINP_COEF1 (-2.7516555290596f)
#define ASINP_COEF2 (+2.9058762374859f)
#define ASINP_COEF3 (-5.9450144193246e-1f)
#define ASINQ_COEF0 (-1.6509933202424e+1f)
#define ASINQ_COEF1 (+2.4864728969164e+1f)
#define ASINQ_COEF2 (-1.0333867072113e+1f)
#define RADTODEG 57.29577951f
#define DEGTORAD 0.01745329252f
#define ABS(X)  (((X)>0)?(X):-(X))
#define COMB_2BYTE(x,y) ((((uint16_t)(x))<<8)|(((uint16_t)(y))&0xFF))   //将两个u8转化成u16

#define USART_HMI 1 //是否使用串口屏
#define INF 1000    //无穷大
#define NODE_NUM 26 //节点个数

#define FAST_SIN_TABLE_SIZE 512
extern const float sinTable[];

typedef struct
{
	float position_x;
	float position_y;
	float position_z;
	float velocity_x;
	float velocity_y;
	float velocity_z;
	float q[4];
    float yaw;
//	float quality;
//	uint8_t update_flag;
//	uint8_t byte[8];
//	float rpy[3];
//	
//	uint8_t rec_update_flag;
//	uint8_t rec_head_update_flag;
//	uint8_t valid;
//	uint8_t fault,last_fault;
//	slam_sensor_mode slam_sensor;
}third_party_state;

typedef struct
{
    float x;
    float y;
    float angle;
}Descartes_cord;

typedef struct
{
  float x;
  float y;
}Vector2f;

typedef struct
{
    float Min_dis_to_origin;    //到原点的最小距离
    uint8_t Pre_node_label;   //前一个点的标号
    uint8_t Pre_node_cnt;   //前面节点的个数，类似于goal_num
}DJST_NodeType;

void UART1_Data_Prase_Prepare_Lite(uint8_t data);
void UART1_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num);
void NCLink_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num);
void NCLink_Data_Prase_Prepare_Lite(uint8_t data);
void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue);
void Float2Byte(float FloatValue, unsigned char *Byte, unsigned char Subscript);
float FastAtan2(float y, float x);
float FastCos(float x);
float FastSin(float x);
float FastAsin(float x);
void Mystrcat(char *str1,char *str2);
void int2str(int num,char* str);
char* int2str_simple(int num);
void int2str_low_ver(int num,char* str);
void SDK_Data_Receive_Prepare_3(uint8_t data);
void SDK_Data_Receive_Prepare_1(uint8_t data);
#endif

