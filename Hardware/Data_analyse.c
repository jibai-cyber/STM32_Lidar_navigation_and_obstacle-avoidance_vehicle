#include "stm32f10x.h"
#include "Data_analyse.h"
#include "Usart.h"
#include "oled1.h"
#include <math.h>

static uint8_t NCLink_Head[2]={0xFF,0xFC};//数据帧头
static uint8_t NCLink_End[2] ={0xA1,0xA2};//数据帧尾
static uint8_t nclink_buf[100];//待接收数据缓冲区
static uint8_t UART1_nclink_buf[100];//待接收数据缓冲区

float Min_ang,Min_dis;
Vector2f Flight_current_position={0,0},Fire_position={0,0};  //飞控返回的坐标数据

third_party_state current_state,barrier_state;
Descartes_cord DKR;
uint8_t read_flag,read_flag_opv,read_Flight_flag,Launch_flag,Color_Fire_flag;

const float sinTable[FAST_SIN_TABLE_SIZE + 1] = {
  0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f,
  0.07356456f, 0.08579731f, 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f,
  0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f, 0.19509032f, 0.20711138f,
  0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
  0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f,
  0.35989504f, 0.37131719f, 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f,
  0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f, 0.47139674f, 0.48218377f,
  0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
  0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f,
  0.61523159f, 0.62485949f, 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f,
  0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f, 0.70710678f, 0.71573083f,
  0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
  0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f,
  0.81758481f, 0.82458930f, 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f,
  0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f, 0.88192126f, 0.88763962f,
  0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
  0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f,
  0.94952818f, 0.95330604f, 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f,
  0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f, 0.98078528f, 0.98310549f,
  0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
  0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f,
  0.99969882f, 0.99992470f, 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f,
  0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f, 0.99518473f, 0.99390697f,
  0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
  0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f,
  0.96377607f, 0.96043052f, 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f,
  0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f, 0.92387953f, 0.91911385f,
  0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
  0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f,
  0.84485357f, 0.83822471f, 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f,
  0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f, 0.77301045f, 0.76516727f,
  0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
  0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f,
  0.65317284f, 0.64383154f, 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f,
  0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f, 0.55557023f, 0.54532499f,
  0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
  0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f,
  0.40524131f, 0.39399204f, 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f,
  0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f, 0.29028468f, 0.27851969f,
  0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
  0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f,
  0.12241068f, 0.11022221f, 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f,
  0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f, 0.00000000f, -0.01227154f,
  -0.02454123f, -0.03680722f, -0.04906767f, -0.06132074f, -0.07356456f,
  -0.08579731f, -0.09801714f, -0.11022221f, -0.12241068f, -0.13458071f,
  -0.14673047f, -0.15885814f, -0.17096189f, -0.18303989f, -0.19509032f, 
  -0.20711138f, -0.21910124f, -0.23105811f, -0.24298018f, -0.25486566f, 
  -0.26671276f, -0.27851969f, -0.29028468f, -0.30200595f, -0.31368174f, 
  -0.32531029f, -0.33688985f, -0.34841868f, -0.35989504f, -0.37131719f, 
  -0.38268343f, -0.39399204f, -0.40524131f, -0.41642956f, -0.42755509f, 
  -0.43861624f, -0.44961133f, -0.46053871f, -0.47139674f, -0.48218377f, 
  -0.49289819f, -0.50353838f, -0.51410274f, -0.52458968f, -0.53499762f, 
  -0.54532499f, -0.55557023f, -0.56573181f, -0.57580819f, -0.58579786f, 
  -0.59569930f, -0.60551104f, -0.61523159f, -0.62485949f, -0.63439328f, 
  -0.64383154f, -0.65317284f, -0.66241578f, -0.67155895f, -0.68060100f, 
  -0.68954054f, -0.69837625f, -0.70710678f, -0.71573083f, -0.72424708f, 
  -0.73265427f, -0.74095113f, -0.74913639f, -0.75720885f, -0.76516727f, 
  -0.77301045f, -0.78073723f, -0.78834643f, -0.79583690f, -0.80320753f, 
  -0.81045720f, -0.81758481f, -0.82458930f, -0.83146961f, -0.83822471f, 
  -0.84485357f, -0.85135519f, -0.85772861f, -0.86397286f, -0.87008699f, 
  -0.87607009f, -0.88192126f, -0.88763962f, -0.89322430f, -0.89867447f, 
  -0.90398929f, -0.90916798f, -0.91420976f, -0.91911385f, -0.92387953f, 
  -0.92850608f, -0.93299280f, -0.93733901f, -0.94154407f, -0.94560733f, 
  -0.94952818f, -0.95330604f, -0.95694034f, -0.96043052f, -0.96377607f, 
  -0.96697647f, -0.97003125f, -0.97293995f, -0.97570213f, -0.97831737f, 
  -0.98078528f, -0.98310549f, -0.98527764f, -0.98730142f, -0.98917651f, 
  -0.99090264f, -0.99247953f, -0.99390697f, -0.99518473f, -0.99631261f, 
  -0.99729046f, -0.99811811f, -0.99879546f, -0.99932238f, -0.99969882f, 
  -0.99992470f, -1.00000000f, -0.99992470f, -0.99969882f, -0.99932238f, 
  -0.99879546f, -0.99811811f, -0.99729046f, -0.99631261f, -0.99518473f, 
  -0.99390697f, -0.99247953f, -0.99090264f, -0.98917651f, -0.98730142f, 
  -0.98527764f, -0.98310549f, -0.98078528f, -0.97831737f, -0.97570213f, 
  -0.97293995f, -0.97003125f, -0.96697647f, -0.96377607f, -0.96043052f, 
  -0.95694034f, -0.95330604f, -0.94952818f, -0.94560733f, -0.94154407f, 
  -0.93733901f, -0.93299280f, -0.92850608f, -0.92387953f, -0.91911385f, 
  -0.91420976f, -0.90916798f, -0.90398929f, -0.89867447f, -0.89322430f, 
  -0.88763962f, -0.88192126f, -0.87607009f, -0.87008699f, -0.86397286f, 
  -0.85772861f, -0.85135519f, -0.84485357f, -0.83822471f, -0.83146961f, 
  -0.82458930f, -0.81758481f, -0.81045720f, -0.80320753f, -0.79583690f, 
  -0.78834643f, -0.78073723f, -0.77301045f, -0.76516727f, -0.75720885f, 
  -0.74913639f, -0.74095113f, -0.73265427f, -0.72424708f, -0.71573083f, 
  -0.70710678f, -0.69837625f, -0.68954054f, -0.68060100f, -0.67155895f, 
  -0.66241578f, -0.65317284f, -0.64383154f, -0.63439328f, -0.62485949f, 
  -0.61523159f, -0.60551104f, -0.59569930f, -0.58579786f, -0.57580819f, 
  -0.56573181f, -0.55557023f, -0.54532499f, -0.53499762f, -0.52458968f, 
  -0.51410274f, -0.50353838f, -0.49289819f, -0.48218377f, -0.47139674f, 
  -0.46053871f, -0.44961133f, -0.43861624f, -0.42755509f, -0.41642956f, 
  -0.40524131f, -0.39399204f, -0.38268343f, -0.37131719f, -0.35989504f, 
  -0.34841868f, -0.33688985f, -0.32531029f, -0.31368174f, -0.30200595f, 
  -0.29028468f, -0.27851969f, -0.26671276f, -0.25486566f, -0.24298018f, 
  -0.23105811f, -0.21910124f, -0.20711138f, -0.19509032f, -0.18303989f, 
  -0.17096189f, -0.15885814f, -0.14673047f, -0.13458071f, -0.12241068f, 
  -0.11022221f, -0.09801714f, -0.08579731f, -0.07356456f, -0.06132074f, 
  -0.04906767f, -0.03680722f, -0.02454123f, -0.01227154f, -0.00000000f
};
union
{
	unsigned char floatByte[4];
	float floatValue;
}FloatUnion;

float FastAsin(float x)
{
  float y, g;
  float num, den, result;
  long i;
  float sign = 1.0;
  
  y = x;
  if (y < (float)0.0){
    y = -y;
    sign = -sign;
  }
  
  if (y > (float)0.5){
    i = 1;
    if (y > (float)1.0){
      result = 0.0;
      return result;
    }    
    g = (1.0f - y) * 0.5f;
    y = -2.0f * sqrtf(g);
  }
  else{
    i = 0;
    if (y < (float)EPS_FLOAT){
      result = y;
      if (sign < (float)0.0){
        result = -result;
      }
      return result;
    }
    g = y * y;
  }
  num = ((ASINP_COEF3 * g + ASINP_COEF2) * g + ASINP_COEF1) * g;
  den = ((g + ASINQ_COEF2) * g + ASINQ_COEF1) * g + ASINQ_COEF0;
  result = num / den;
  result = result * y + y;
  if (i == 1){
    result = result + (float)PI_2;
  }
  if (sign < (float)0.0){
    result = -result;
  }
  return result;
}

float FastSin(float x)  //形参要转为弧度制！！！！！！！！
{
  float sinVal, fract, in; // Temporary variables for input, output
  unsigned short index; // Index variable
  float a, b; // Two nearest output values
  int n;
  float findex;
  
  // input x is in radians
  // Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi
  in = x * 0.159154943092f;
  
  // Calculation of floor value of input
  n = (int) in;
  
  // Make negative values towards -infinity
  if(x < 0.0f){
    n--;
  }
  
  // Map input value to [0 1]
  in = in - (float) n;
  
  // Calculation of index of the table
  findex = (float) FAST_SIN_TABLE_SIZE * in;
  index = ((unsigned short)findex) & 0x1ff;
  
  // fractional value calculation
  fract = findex - (float) index;
  
  // Read two nearest values of input value from the sin table
  a = sinTable[index];
  b = sinTable[index+1];
  
  // Linear interpolation process
  sinVal = (1.0f-fract)*a + fract*b;
  
  // Return the output value
  return (sinVal);
}

float FastCos(float x)
{
  float cosVal, fract, in; // Temporary variables for input, output
  unsigned short index; // Index variable
  float a, b; // Two nearest output values
  int n;
  float findex;
  
  // input x is in radians
  // Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi, add 0.25 (pi/2) to read sine table
  in = x * 0.159154943092f + 0.25f;
  
  // Calculation of floor value of input
  n = (int) in;
  
  // Make negative values towards -infinity
  if(in < 0.0f){
    n--;
  }
  
  // Map input value to [0 1]
  in = in - (float) n;
  
  // Calculation of index of the table
  findex = (float) FAST_SIN_TABLE_SIZE * in;
  index = ((unsigned short)findex) & 0x1ff;
  
  // fractional value calculation
  fract = findex - (float) index;
  
  // Read two nearest values of input value from the cos table
  a = sinTable[index];
  b = sinTable[index+1];
  
  // Linear interpolation process
  cosVal = (1.0f-fract)*a + fract*b;
  
  // Return the output value
  return (cosVal);
}


void NCLink_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num)//树莓派雷达数据解析进程
{
  uint8_t sum = 0;
  float min_dis=0,min_ang=0;
  for(uint8_t i=0;i<(num-3);i++)  sum ^= *(data_buf+i);
  if(!(sum==*(data_buf+num-3)))    																					return;//判断sum	
	if(!(*(data_buf)==NCLink_Head[1]&&*(data_buf+1)==NCLink_Head[0]))         return;//判断帧头
	if(!(*(data_buf+num-2)==NCLink_End[0]&&*(data_buf+num-1)==NCLink_End[1])) return;//帧尾校验  
    
    if(*(data_buf+2)==0X0D)
	{
		Byte2Float(data_buf,4,&current_state.position_x);
		Byte2Float(data_buf,8,&current_state.position_y);
		Byte2Float(data_buf,12,&barrier_state.position_x);
		Byte2Float(data_buf,16,&current_state.velocity_x);
		Byte2Float(data_buf,20,&current_state.velocity_y);
		Byte2Float(data_buf,24,&barrier_state.position_y);
		current_state.position_x*=100.0f;
		current_state.position_y*=100.0f;
		current_state.velocity_x*=100.0f;
		current_state.velocity_y*=100.0f;

        min_dis = barrier_state.position_x*100;
        min_ang = barrier_state.position_y;
        
        if(min_ang>=0 && min_ang<=90)
        {
            min_ang+=270.0f;
        }
        else if(min_ang>=90 && min_ang<=180)
        {
            min_ang-=90.0f;
        }
		else if(min_ang<=0 && min_ang>=-90)
        {
            min_ang+=270.0f;
        }
        else if(min_ang<=-90 && min_ang>=-180)
        {
            min_ang*=-1.0f;
            min_ang-=90.0f;
            min_ang = 180.0f - min_ang;
        }
        
        
        Min_ang = min_ang;
        Min_dis = min_dis;
        
        
        
        
        barrier_state.position_x = FastCos(min_ang*DEGTORAD)*min_dis;
        barrier_state.position_y = FastSin(min_ang*DEGTORAD)*min_dis;
        
		Byte2Float(data_buf,28,&current_state.q[0]);		
		Byte2Float(data_buf,32,&current_state.q[1]);
		Byte2Float(data_buf,36,&current_state.q[2]);
		Byte2Float(data_buf,40,&current_state.q[3]);
		float _q[4];
		_q[0]=current_state.q[3]*(1.0f);
		_q[1]=current_state.q[0]*(1.0f);
		_q[2]=current_state.q[2]*(-1.0f);
		_q[3]=current_state.q[1]*(1.0f);	
		
		current_state.q[0]=_q[0];
		current_state.q[1]=_q[1];
		current_state.q[2]=_q[2];
		current_state.q[3]=_q[3];
		
        //q0和q1恒为0，说明只传偏航数据
        current_state.yaw = FastAtan2(2*current_state.q[1]*current_state.q[2]+2*current_state.q[0]*current_state.q[3], -2*current_state.q[2]*current_state.q[2]-2*current_state.q[3]*current_state.q[3]+1)*RADTODEG;
//        current_state.pitch=FastAsin(-2*current_state.q[1]*current_state.q[3]+2*current_state.q[0]*current_state.q[2])*RADTODEG;
//        current_state.roll=FastAtan2(2*current_state.q[2]*current_state.q[3]+2*current_state.q[0]*current_state.q[1],-2*current_state.q[1]*current_state.q[1]-2*current_state.q[2]*current_state.q[2]+1)*RADTODEG;
        
        /*转化为笛卡尔坐标系*/
        DKR.x = current_state.position_y;
        DKR.y = current_state.position_x*-1.0f;
        if(current_state.yaw>=-90.0f && current_state.yaw<=180.0f) DKR.angle = current_state.yaw + 90.0f;
        else DKR.angle = 360.0f + (current_state.yaw + 90.0f);
        
        read_flag = 1;
	}	
}

void UART1_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num)//飞控数据解析进程
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-3);i++)  sum ^= *(data_buf+i);
  if(!(sum==*(data_buf+num-3)))    																					return;//判断sum	
	if(!(*(data_buf)==NCLink_Head[0]&&*(data_buf+1)==NCLink_Head[1]))         return;//判断帧头
	if(!(*(data_buf+num-2)==NCLink_End[0]&&*(data_buf+num-1)==NCLink_End[1])) return;//帧尾校验  
    
    if(*(data_buf+2)==0xE0)
	{        
        //这里只是飞机传来的机体坐标，需要后续转化为场地坐标
        Byte2Float(data_buf,4,&Flight_current_position.x);
		Byte2Float(data_buf,8,&Flight_current_position.y);
        Byte2Float(data_buf,12,&Fire_position.x);
		Byte2Float(data_buf,16,&Fire_position.y);
        Launch_flag=data_buf[20];
        Color_Fire_flag=data_buf[21];
        read_Flight_flag = 1;
	}	
}

void UART1_Data_Prase_Prepare_Lite(uint8_t data)  //解析opv数据
{
    static uint8_t data_len = 0,data_cnt = 0;
      static uint8_t state = 0;
      if(state==0&&data==NCLink_Head[0])//判断帧头1
      {
        state=1;
        UART1_nclink_buf[0]=data;
      }
      else if(state==1&&data==NCLink_Head[1])//判断帧头2
      {
        state=2;
        UART1_nclink_buf[1]=data;
      }
      else if(state==2&&data<0XF1)//功能字节
      {
        state=3;
        UART1_nclink_buf[2]=data;
      }
      else if(state==3&&data<100)//有效数据长度
      {
        state = 4;
        UART1_nclink_buf[3]=data;
        data_len = data;
        data_cnt = 0;
      }
      else if(state==4&&data_len>0)//数据接收
      {
        data_len--;
        UART1_nclink_buf[4+data_cnt++]=data;
        if(data_len==0)  state = 5;
      }
      else if(state==5)//异或校验
      {
        state = 6;
        UART1_nclink_buf[4+data_cnt++]=data;
      }
        else if(state==6&&data==NCLink_End[0])//帧尾0
        {
                state = 7;
                UART1_nclink_buf[4+data_cnt++]=data;
        }
        else if(state==7&&data==NCLink_End[1])//帧尾1
        {
                state = 0;
                UART1_nclink_buf[4+data_cnt]=data;
              UART1_Data_Prase_Process_Lite(UART1_nclink_buf,data_cnt+5);//数据解析
        }
      else 
        {
            state = 0;
        }
}

uint8_t Function_Data_UART4=0,Num_Data_UART4=0;
uint8_t Height_Data,Read_Whole_Data_Flag_UART4=0; //Read_Whole_Data_Flag
extern uint8_t Str_to_Flight[];
extern u8 SDK_mode;
void SDK_Data_Receive_Prepare_3(uint8_t data)
{
    static uint8_t state = 0;
  if(state==0&&data==0xFF)//帧头1
  {
    state=1;
  }
  else if(state==1&&data==0xFC)//帧头2
  {
    state=2;
  }
  else if(state==2&&data<0XFF)//功能字节
  {
    state=3;
    Function_Data_UART4=data;
//    if(USART_HMI==1) SDK_mode=data;
  }
  else if(state==3&&data<0XFF)//具体存储点的序列号，根据功能字节的不同可以改编为其他的功能
  {
    state=4;
    Num_Data_UART4=data;
  }
  else if(state==4&&data<=0XFF)//具体存储点的序列号，根据功能字节的不同可以改编为其他的功能
  {
    state=5;
    Height_Data=data;
  }
  else if(state==5&&data==0xFA)//帧尾
  {
    state = 6;
  }
  else if(state==6&&data==0xFB)
  {
    state = 0;
      Read_Whole_Data_Flag_UART4=1;
      if(Function_Data_UART4==0x07||Function_Data_UART4==0x08||Function_Data_UART4==0x09)
      {
          Str_to_Flight[2]=Function_Data_UART4;
//          Serial_SendString(USART1,Str_to_Flight,RESET);//根本发不出正确字符，且每次只能发一个
//          for(uint8_t i=0;i<7;i++) UARTx_SendByte(USART1,Str_to_Flight[i]);
      }
  }
  else state = 0;
}

extern Vector2f Manual_Patrol_Node[];
void SDK_Data_Receive_Prepare_1(uint8_t data)
{
    static uint8_t state = 0;
    static uint16_t High_8=0,Low_8=0;
  if(state==0&&data==0xFF)//帧头1
  {
    state=1;
  }
  else if(state==1&&data==0xFC)//帧头2
  {
    state=2;
  }
  else if(state==2&&data<0XFF)//功能字节
  {
    state=3;
    Function_Data_UART4=data;
  }
  else if(state==3&&data<0XFF)//坐标序号或goal_num
  {
    state=4;
    Num_Data_UART4=data;
  }
  else if(state==4&&data<0XFF)//X坐标
  {
    state=5;
    Low_8=data;
  }
  else if(state==5&&data<0XFF)//X坐标
  {
    state=6;
    High_8=data;
//    High_8=(High_8 << 8) | Low_8;
    if(Function_Data_UART4==0x04)
    {
        Manual_Patrol_Node[Num_Data_UART4].x=COMB_2BYTE(High_8,Low_8);
    }
    
  }
  else if(state==6&&data<0XFF)//Y坐标
  {
    state=7;
    Low_8=data;
  }
  else if(state==7&&data<0XFF)//Y坐标
  {
    state=8;
    High_8=data;
//    High_8=(High_8 << 8) | Low_8;
    if(Function_Data_UART4==0x04)
    {
        Manual_Patrol_Node[Num_Data_UART4].y=COMB_2BYTE(High_8,Low_8);
    }
  }
  else if(state==8&&data==0xFA)//帧尾
  {
    state = 9;
  }
  else if(state==9&&data==0xFB)
  {
    state = 0;
    Read_Whole_Data_Flag_UART4 = 1;
  }
  else state = 0;
}

void NCLink_Data_Prase_Prepare_Lite(uint8_t data)//地面站数据解析
{
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==NCLink_Head[1])//判断帧头1
  {
    state=1;
    nclink_buf[0]=data;
  }
  else if(state==1&&data==NCLink_Head[0])//判断帧头2
  {
    state=2;
    nclink_buf[1]=data;
  }
  else if(state==2&&data<0XF1)//功能字节
  {
    state=3;
    nclink_buf[2]=data;
  }
  else if(state==3&&data<100)//有效数据长度
  {
    state = 4;
    nclink_buf[3]=data;
    data_len = data;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//数据接收
  {
    data_len--;
    nclink_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//异或校验
  {
    state = 6;
    nclink_buf[4+data_cnt++]=data;
  }
	else if(state==6&&data==NCLink_End[0])//帧尾0
	{
			state = 7;
			nclink_buf[4+data_cnt++]=data;
	}
	else if(state==7&&data==NCLink_End[1])//帧尾1
	{
			state = 0;
			nclink_buf[4+data_cnt]=data;
		  NCLink_Data_Prase_Process_Lite(nclink_buf,data_cnt+5);//数据解析
	}
  else 
	{
		state = 0;
	}
}

float FastAtan2(float y, float x)
{
  float f, g;
  float num, den;
  float result;
  int n;
  
  static const float a[4] = {0, (float)PI_6, (float)PI_2, (float)PI_3};
  
  if (x == (float)0.0){
    if (y == (float)0.0){
      result = 0.0;
      return result;
    }
    
    result = (float)PI_2;
    if (y > (float)0.0){
      return result;
    }
    if (y < (float)0.0){
      result = -result;
      return result;
    }
  }
  n = 0;
  num = y;
  den = x;
  
  if (num < (float)0.0){
    num = -num;
  }
  if (den < (float)0.0){
    den = -den;
  }
  if (num > den){
    f = den;
    den = num;
    num = f;
    n = 2;
  }
  f = num / den;
  
  if (f > (float)TWO_MINUS_ROOT3){
    num = f * (float)SQRT3_MINUS_1 - 1.0f + f;
    den = (float)SQRT3 + f;
    f = num / den;
    n = n + 1;
  }
  
  g = f;
  if (g < (float)0.0){
    g = -g;
  }
  
  if (g < (float)EPS_FLOAT){
    result = f;
  }
  else{
    g = f * f;
    num = (ATANP_COEF1 * g + ATANP_COEF0) * g;
    den = (g + ATANQ_COEF1) * g + ATANQ_COEF0;
    result = num / den;
    result = result * f + f;
  }
  if (n > 1){
    result = -result;
  }
  result = result + a[n];
  
  if (x < (float)0.0){
    result = PI - result;
  }
  if (y < (float)0.0){
    result = -result;
  }
  return result;
}

void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue)
{
	FloatUnion.floatByte[0]=Byte[Subscript];
	FloatUnion.floatByte[1]=Byte[Subscript + 1];
	FloatUnion.floatByte[2]=Byte[Subscript + 2];
	FloatUnion.floatByte[3]=Byte[Subscript + 3];
	*FloatValue=FloatUnion.floatValue;
}

void Float2Byte(float FloatValue, unsigned char *Byte, unsigned char Subscript)
{
    FloatUnion.floatValue = FloatValue;
    Byte[Subscript]     = FloatUnion.floatByte[0];
    Byte[Subscript + 1] = FloatUnion.floatByte[1];
    Byte[Subscript + 2] = FloatUnion.floatByte[2];
    Byte[Subscript + 3] = FloatUnion.floatByte[3];
}
 
void Mystrcat(char *str1,char *str2)
{
    char *t;
    while(*str1!='\0')
    {
        str1++;
    }
    t=str1;
    while(*str2!='\0')
    {
        *t++=*str2++;
    }
    *t='\0';
}

void int2str(int num,char* str) //串口显示后前面莫名会多出一堆'0'，但是C语言编译器测试却没问题
{
    int8_t _cnt;   //_cnt表示有几位数
    int _num,__num;
    
    if(num<0)
    {
        *str++ = '-';
        num*=-1;
    }    
    
    else if(num==0)
    {
        *str='0';
        return;
    }
    
    _num = num;
    while(_num!=0)
    {
        _num/=10;
        _cnt++;
    }
    _cnt-=1;
    __num=num;
    
    while(_cnt>=0)
    {
        for(uint8_t i=0;i<_cnt;i++) __num/=10;
        *str++ = __num%10+'0';
        __num=num;
        _cnt--;
    }
}

char* int2str_simple(int num) //神奇，没有任何显示
{
    int8_t _cnt;   //_cnt表示有几位数
    int _num,__num;
    char* p;
    
    if(num<0)
    {
        *p++ = '-';
        num*=-1;
    }    
    
    else if(num==0)
    {
        *p++='0';
        *p='\0';
        return p;
    }
    
    _num = num;
    while(_num!=0)
    {
        _num/=10;
        _cnt++;
    }
    _cnt-=1;
    __num=num;
    
    while(_cnt>=0)
    {
        for(uint8_t i=0;i<_cnt;i++) __num/=10;
        *p++ = __num%10+'0';
        __num=num;
        _cnt--;
    }
    
    *p='\0';
    
    return p;
}

void int2str_low_ver(int num,char* str) //最低级的方法
{
    int8_t _cnt;   //_cnt表示有几位数
    int _num;
    
    if(num<0)
    {
        str[0] = '-';
        num*=-1;
        
        _num = num;
        while(_num!=0)
        {
            _num/=10;
            _cnt++;
        }
        _cnt-=1;
        _num=num;
        
        switch(_cnt)
        {
        	case 0:
                str[1]=_num+'0';
                for(uint8_t i=2;i<6;i++) str[i]='\0';
        		break;
        	case 1:
                str[1]=_num/10+'0';
                str[2]=_num%10+'0';
                for(uint8_t i=3;i<6;i++) str[i]='\0';
        		break;
            case 2:
                str[1]=num/100+'0';
                str[2]=num/10%10+'0';
                str[3]=num%10+'0';
                for(uint8_t i=4;i<6;i++) str[i]='\0';
                break;
            case 3:
                str[1]=num/1000+'0';
                str[2]=num/100%10+'0';
                str[3]=num/10%10+'0';
                str[4]=num%10+'0';
                str[5]='\0';
                break;
        	default:
                return;
        }
    }    
    
    else if(num==0)
    {
        str[0] = '0';
        for(uint8_t i=1;i<5;i++) str[i]='\0';
        return;
    }
    else if(num>0)
    {
        _num = num;
        while(_num!=0)
        {
            _num/=10;
            _cnt++;
        }
        _cnt-=1;
        _num=num;
        
        switch(_cnt)
        {
        	case 0:
                str[0]=_num+'0';
                for(uint8_t i=1;i<6;i++) str[i]='\0';
        		break;
        	case 1:
                str[0]=_num/10+'0';
                str[1]=_num%10+'0';
                for(uint8_t i=2;i<6;i++) str[i]='\0';
        		break;
            case 2:
                str[0]=num/100+'0';
                str[1]=num/10%10+'0';
                str[2]=num%10+'0';
                for(uint8_t i=3;i<6;i++) str[i]='\0';
                break;
            case 3:
                str[0]=num/1000+'0';
                str[1]=num/100%10+'0';
                str[2]=num/10%10+'0';
                str[3]=num%10+'0';
                for(uint8_t i=4;i<6;i++) str[i]='\0';
                break;
        	default:
                return;
        }
    }
    
    
}
