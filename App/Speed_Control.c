#include "include.h"
#include "math.h"
#include "project.h"
#include "picture.h"
#include "S3010_Control.h"

#include "Speed_Control.h"




#define QEP_time 5000   //编码器定时5ms

//时钟变量
unsigned long runtime = 0;


#define LEFT 0
#define RIGHT 1
#define DIFF 2


int stop_of_outroad = 0;
extern int speedtest;
/**************************全局变量**************************/
int16 double_ObjectSpeed=200;	        /*无意义*/

/****************
小车平均速度闭环变量
*****************/
int16 MotorSpeed =70;               //平均速度设定
int16 NowSpeed;                /*当前速度*/      
int16 SpeedErr;
int16 His_SpeedErr=0;
int16 His2_SpeedErr=0;

//typedef struct
//{
//  int MOROR_id;
//  int nowSpeed;
//  int ObjectSpeed;
//  int PI_SpeedErr;
//  int His_PI_SpeedErr;
//  int His2_PI_SpeedErr;
//  int Motor_Out;
//}MOTOR_CTR;
//typedef struct
//{
//  
//    int Adj_KP_Speed;
//    int Adj_KI_Speed;
//    int Adj_KD_Speed;
//    int Adj_BangBang_Pos;
//    int Adj_BangBang_Neg;
//    int Adj_KP_DifSpd;
//    int Adj_KD_DifSpd;
//    int Adj_KI_DifSpd;
//                  /*当前速度*/
//}MOTOR_PID;
MOTOR_CTR Speed_L={LEFT,0,0,0,0,0,0};
MOTOR_CTR Speed_R={RIGHT,0,0,0,0,0,0};
MOTOR_CTR Speed_DIFF={DIFF,0,0,0,0,0,0};
/*********************电极相关参数****************/
//float speed_Kp=167;//50                  80
//float speed_Ki=25;//30 one  75 five     50
//float speed_Kd=25;
//MOTOR_PID Speed_all={167L,25L,8L,50,-50,33L,10,200L}; //D=18
MOTOR_PID Speed_all={167L,25L,8L,50,-50,40L,10L,30L}; //D=18
unsigned short DIFF_TEMP=650;

int16 serve_value=0;





/**************************内部变量**************************/
  
long int Pwm_Delta=0;   //
long int double_Speed_out;//
long int P_part;
long int I_part;//



const unsigned short DiffTbl[90]={
  4,5,5,6,6,7,8,8,9,9,
10,10,11,12,12,13,13,14,15,16,
18,19,20,21,22,23,27,35,42,49,
56,63,70,77,83,86,89,92,95,98,
101,103,106,109,112,115,118,120,123,126,
128,131,135,139,143,146,150,153,157,161,
164,167,170,173,176,179,182,185,188,192,
196,200,204,208,212,215,219,224,229,234,
239,244,250,254,259,259,259,259,259,259};


signed short Abs(signed short dat)         
{
  return (dat>0)? dat : (-dat);
}

void Speed_get_init(void)
{

  ftm_quad_init(FTM1);                                    //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
  ftm_quad_init(FTM2);
  pit_init_us(PIT1, QEP_time);                                     //初始化PIT0，定时时间为： QEP_time
  set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);                   //设置PIT0的中断服务函数为 PIT0_IRQHandler
  enable_irq (PIT1_IRQn);                                           //使能PIT0中断
}

void PIT1_IRQHandler(void)
{   
    runtime++;
    speed_decision();                           //速度决策
    PIT_Flag_Clear(PIT1);                       //清中断标志位     
}
void speed_decision(void)
{
    Speed_Read();
    SetMotorSpeed();
    Diff_Speed((Control-S_D5_centervalue));//参数为舵机转角量
    SpeedControl();
    
}
void SetMotorSpeed(void)
{
    int stop_flag=0;
    int stop_c_flag=0;
    int speedset_flag=0;
    if(mode1==1)
    {
      speedset_flag=0;
      if(effctline<4 && effctline-lasteffctline>0) {MotorSpeed = 110;speedset_flag=1;}
      if(effctline<8 && effctline>=4 && effctline-lasteffctline>0) {MotorSpeed = 108;speedset_flag=1;}
      if(effctline<13 && effctline>=8 && effctline-lasteffctline>0) {MotorSpeed = 106;speedset_flag=1;}
        if(effctline<18 && effctline>=13 && effctline-lasteffctline>0) {MotorSpeed = 104;speedset_flag=1;}
        if(effctline>=18 && deltax*Control_error>0) {MotorSpeed = 97;speedset_flag=1;}
        //if(effctline>30) MotorSpeed = 70;
      if(obstacle_flag) {MotorSpeed = 90;speedset_flag=1;}
        //MotorSpeed = 80;
        if(ring_flag)
        {
               MotorSpeed = 95;
               speedset_flag=1;
        }
 //     if(speedset_flag==0)
 //       MotorSpeed = 60;
//            speedtest=1;
//      else  speedtest=0;
    }
     if(mode2==1)
    {
      speedset_flag=0;
      if(effctline<4 && effctline-lasteffctline>0) {MotorSpeed = 106;speedset_flag=1;}
      if(effctline<8 && effctline>=4 && effctline-lasteffctline>0) {MotorSpeed = 104;speedset_flag=1;}
      if(effctline<13 && effctline>=8 && effctline-lasteffctline>0) {MotorSpeed = 102;speedset_flag=1;}
        if(effctline<18 && effctline>=13 && effctline-lasteffctline>0) {MotorSpeed = 100;speedset_flag=1;}
        if(effctline>=18 && deltax*Control_error>0) {MotorSpeed = 93;speedset_flag=1;}
        //if(effctline>30) MotorSpeed = 70;
      if(obstacle_flag) {MotorSpeed = 85;speedset_flag=1;}
        //MotorSpeed = 80;
        if(ring_flag)
        {
               MotorSpeed = 87;
               speedset_flag=1;
        }
    }         
    if(mode3==1)
    {
     speedset_flag=0;
      if(effctline<4 && effctline-lasteffctline>0) {MotorSpeed = 103;speedset_flag=1;}
      if(effctline<8 && effctline>=4 && effctline-lasteffctline>0) {MotorSpeed = 102;speedset_flag=1;}
      if(effctline<13 && effctline>=8 && effctline-lasteffctline>0) {MotorSpeed = 100;speedset_flag=1;}
        if(effctline<18 && effctline>=13 && effctline-lasteffctline>0) {MotorSpeed = 99;speedset_flag=1;}
        if(effctline>=18 && deltax*Control_error>0) {MotorSpeed = 90;speedset_flag=1;}
        //if(effctline>30) MotorSpeed = 70;
      if(obstacle_flag) {MotorSpeed = 83;speedset_flag=1;}
        //MotorSpeed = 80;
        if(ring_flag)
        {
               MotorSpeed = 85;
               speedset_flag=1;
        }
 //     if(speedset_flag==0)
 //       MotorSpeed = 60;
//            speedtest=1;
//      else  speedtest=0;
    }         
    if(mode4==1)
    {
//        if(effctline<4 && effctline-lasteffctline>0) MotorSpeed = 90;
//        if(effctline<8 && effctline>=4 && effctline-lasteffctline>0) MotorSpeed = 89;
//        if(effctline<13 && effctline>=8 && effctline-lasteffctline>0) MotorSpeed = 88;
//        if(effctline<18 && effctline>=13 && effctline-lasteffctline>0) MotorSpeed = 87;
//        if(effctline>=18 && deltax*Control_error>0) MotorSpeed = 83;
//        if(effctline>30) MotorSpeed = 65;
//        if(obstacle_flag) MotorSpeed = 75;
//        MotorSpeed = 80;
//        if(ring_flag)
//        {
//               MotorSpeed = 79;
//        }
            speedset_flag=0;
      if(effctline<4 && effctline-lasteffctline>0) {MotorSpeed = 90;speedset_flag=1;}
      if(effctline<8 && effctline>=4 && effctline-lasteffctline>0) {MotorSpeed = 89;speedset_flag=1;}
      if(effctline<13 && effctline>=8 && effctline-lasteffctline>0) {MotorSpeed = 88;speedset_flag=1;}
        if(effctline<18 && effctline>=13 && effctline-lasteffctline>0) {MotorSpeed = 87;speedset_flag=1;}
        if(effctline>=18 && deltax*Control_error>0) {MotorSpeed = 83;speedset_flag=1;}
        if(effctline>30) {MotorSpeed = 65;speedset_flag=1;}
      if(obstacle_flag) {MotorSpeed = 75;speedset_flag=1;}
        //MotorSpeed = 80;
        if(ring_flag)
        {
               MotorSpeed = 79;
               speedset_flag=1;
        }
      if(speedset_flag==0)
        MotorSpeed = 88;
    }            
       
    /**********安全模式*************/
//    if(picture[W*(H-1)+centerline[H/GER-1]]==black&&picture[W*(H-2)+centerline[H/GER-2]]==black&&picture[W*(H-3)+centerline[H/GER-3]]==black&&picture[W*(H-4)+centerline[H/GER-4]]==black&&picture[W*(H-5)+centerline[H/GER-5]]==black)
//    {stop_flag=1;}
    if( runtime>500&&image[H-1][centerline[H-1]]==0 && image[H-2][centerline[H-2]]==0
       &&image[H-3][centerline[H-3]]==0&& image[H-4][centerline[H-4]]==0
         &&image[H-5][W/2]==0)
    {
        stop_flag=1;
    }
    if(stop_flag==1) stop_c_flag =1;
    if(stop_c_flag) stop_count1++;
    else{stop_count1=0;}
    if(stop_count1>25) stop_of_outroad=1;
    if(stop_count1>40000) stop_count1=0;
     
}
/*====================================
函数名:Speed_Read()
作用  ：读取速度，5ms调用一次
备注  ：无
=====================================*/
void Speed_Read(void)
{
  int16 cnt_spd_R;
  int16 cnt_spd_L;
  /********************************
  获取正交解码数值
  *********************************/
  cnt_spd_L =    ftm_quad_get(FTM1);
  ftm_quad_clean(FTM1);
  
  cnt_spd_R =   -1* ftm_quad_get(FTM2);          
  ftm_quad_clean(FTM2);
  
  
  Speed_L.nowSpeed=cnt_spd_L;
  Speed_R.nowSpeed=cnt_spd_R;
  NowSpeed=(Speed_L.nowSpeed+ Speed_R.nowSpeed)/2; 
}

/****************************************************************************************************
 * DifferSpeed_PID
 * 差速PID计算
****************************************************************************************************/
void DifferSpeed_PID(void)
{
  signed short DiffLimit = 0;
  //未开电机或者停车情况下不进行差速PID运算
 // if((Speed_L.nowSpeed+Speed_R.nowSpeed)==0||(Speed_L.ObjectSpeed+Speed_R.ObjectSpeed)==0) return;
  Speed_DIFF.nowSpeed=(long signed)(Speed_L.nowSpeed-Speed_R.nowSpeed);                
  Speed_DIFF.ObjectSpeed=(long signed)(Speed_L.ObjectSpeed-Speed_R.ObjectSpeed);  
  
  Speed_DIFF.PI_SpeedErr=Speed_DIFF.ObjectSpeed-Speed_DIFF.nowSpeed;
  Speed_DIFF.Motor_Out=(Speed_all.Adj_KP_DifSpd+Speed_all.Adj_KI_DifSpd+Speed_all.Adj_KD_DifSpd)*((signed long)Speed_DIFF.PI_SpeedErr);
  Speed_DIFF.Motor_Out-=(Speed_all.Adj_KP_DifSpd+2*Speed_all.Adj_KD_DifSpd)*((signed long) Speed_DIFF.His_PI_SpeedErr);
  Speed_DIFF.Motor_Out+=Speed_all.Adj_KD_DifSpd*((signed long)Speed_DIFF.His2_PI_SpeedErr);
  //Speed_DIFF.Motor_Out = (int)(Speed_DIFF.Motor_Out/160.0);
  Speed_DIFF.Motor_Out = (int)(Speed_DIFF.Motor_Out);
  if(Speed_DIFF.Motor_Out>0)
    DiffLimit = MAX_MotorOut - double_Speed_out - Speed_DIFF.Motor_Out;             //差速余量
  else
    DiffLimit = MAX_MotorOut - double_Speed_out + Speed_DIFF.Motor_Out;             //差速余量
  
  if(DiffLimit>0) DiffLimit =0;
  
  double_Speed_out = double_Speed_out + DiffLimit;                                     //差速优先

  Speed_DIFF.His2_PI_SpeedErr=Speed_DIFF.His_PI_SpeedErr;
  Speed_DIFF.His_PI_SpeedErr= Speed_DIFF.PI_SpeedErr;
  
  Speed_L.Motor_Out = (double_Speed_out + Speed_DIFF.Motor_Out);
  Speed_R.Motor_Out = (double_Speed_out - Speed_DIFF.Motor_Out);
  

//孙总差速bang-bang          //4.23屏蔽
//  if(Speed_L.ObjectSpeed>Speed_R.ObjectSpeed+10) 
//   if(Speed_L.nowSpeed<Speed_L.ObjectSpeed-30)  Speed_L.Motor_Out=MAX_MotorOut;
//  
//  if(Speed_R.ObjectSpeed>Speed_L.ObjectSpeed+10) 
//    if(Speed_R.nowSpeed<Speed_R.ObjectSpeed-30)  Speed_R.Motor_Out=MAX_MotorOut;
// 
//  if(Speed_R.ObjectSpeed<Speed_L.ObjectSpeed+10&&Speed_R.ObjectSpeed>Speed_L.ObjectSpeed-10) 
//  {
//   if(Speed_R.nowSpeed<Speed_R.ObjectSpeed-30)  Speed_R.Motor_Out=MAX_MotorOut;
//   if(Speed_L.nowSpeed<Speed_L.ObjectSpeed-30)  Speed_L.Motor_Out=MAX_MotorOut;
//  }
  
//  SetMotorPWM(Speed_L.Motor_Out,Speed_R.Motor_Out);
}


/************************************************************************************************ 
* Speed_PID
* 增量式速度PID控制
************************************************************************************************/
static void Speed_PID(void)
{ 

  if(SpeedErr<100&&(I_part<4300||SpeedErr<0))  //SpeedErr<203 ：限制积分的区间 把误差过的误差滤去I_part<35000 饱和限幅为3500: SpeedErr<0 减速
   {
//     if(SpeedErr<0)
//       I_part+=(Speed_all.Adj_KI_Speed+20)*(SpeedErr);     
//     else
     I_part+=(Speed_all.Adj_KI_Speed)*(SpeedErr);    

/*     if(SpeedErr<-20)
     {
       I_part=0;
       Pwm_Delta+=((Speed_all.Adj_KI_Speed)*(SpeedErr)-I_part);
     }
     else
       Pwm_Delta+=((Speed_all.Adj_KI_Speed)*(SpeedErr));
     */
     if(SpeedErr<-20)
     {
       Pwm_Delta=Pwm_Delta/2;
       I_part=I_part/2;
     }
     else
        Pwm_Delta+=((Speed_all.Adj_KI_Speed)*(SpeedErr));
   }
   Pwm_Delta+=(Speed_all.Adj_KD_Speed*(SpeedErr-2*His_SpeedErr+His2_SpeedErr));
   P_part=Speed_all.Adj_KP_Speed * SpeedErr;
//   if(P_part<0)
//  Pwm_Delta+=Speed_all.Adj_KP_Speed*SpeedErr+Speed_all.Adj_KD_Speed*(SpeedErr-His_SpeedErr);
//  I_OUT+=Speed_all.Adj_KI_Speed*SpeedErr;
//  Pwm_Delta+=I_OUT;
   if(Pwm_Delta>=7500)
     Pwm_Delta = 7500;
   if(Pwm_Delta < 0) Pwm_Delta =(long int)( Pwm_Delta * 1.2); 
   double_Speed_out = (long int)((Pwm_Delta+P_part));
  //SetMotorPWM(double_Speed_out,double_Speed_out); (signed short)
}

/************************************************************************************************ 
* SpeedControl
* 速度控制函数,负责将计数器的计数脉冲数转化为速度,并计算速度误差,进行速度控制
  (!!!如果想要实现闭环控制,应将其设置为定时器的回调函数!!!)
* 入口参数  pulse:测速计数器计的脉冲数
            pulse_2:第二测速计数器的脉冲数
************************************************************************************************/
void SpeedControl (void)
{ 
                                             /*计算当前速度*/
    static int lpwm_out_last=0;
    static int rpwm_out_last=0;
    float a_temp=1.0;
    /*如果系统当前处在闭环模式,则进行电机闭环控制*/    
    SpeedErr=MotorSpeed-NowSpeed;              /*计算偏差*/    
    
   /*速度控制采用bang-bang加PID控制*/
//    
   if(SpeedErr>=Speed_all.Adj_BangBang_Pos&&runtime<=400)  
    {  /*全力加速*/
      double_Speed_out = MAX_MotorOut;
    }
    else if(SpeedErr<=Speed_all.Adj_BangBang_Neg&&runtime<=400)
    {  /*全力减速*/
      double_Speed_out = MIN_MotorOut;
    }
    else
    {  
        /*PID控制*/
        Speed_PID();     
    }
    if(double_Speed_out>5000)
     double_Speed_out=5000;
    if(double_Speed_out<0)
     double_Speed_out=0; //封0
    DifferSpeed_PID();                          /*差速PID*/

    His2_SpeedErr = His_SpeedErr;               /*记录偏差历史值*/
    His_SpeedErr = SpeedErr;
    Speed_L.Motor_Out=a_temp*Speed_L.Motor_Out+(1-a_temp)*lpwm_out_last; //a_temp != 1 时，具有一阶惯性的作用
    lpwm_out_last=Speed_L.Motor_Out;
    Speed_R.Motor_Out=a_temp*Speed_R.Motor_Out+(1-a_temp)*rpwm_out_last;
    rpwm_out_last=Speed_R.Motor_Out;
//    Speed_L.Motor_Out = (3000 );
//    Speed_R.Motor_Out = (3000);
    SetMotorPWM(Speed_L.Motor_Out,Speed_R.Motor_Out);
    
    
}
    
  

/****************************************************************************************************
 * Diff_Speed
 * 计算差速量
****************************************************************************************************/
int SpdDiff = 0;
void Diff_Speed(signed long serve_value)
{ 
    unsigned short ServoAbs; //加速运算
    signed short ProgServo;

    static signed long HisServeOut[4];

    //舵机历史值的保存
    HisServeOut[3]=HisServeOut[2];
    HisServeOut[2]=HisServeOut[1];
    HisServeOut[1]=HisServeOut[0];
    HisServeOut[0]=serve_value;

    ProgServo = (4 * HisServeOut[0] + 2 * HisServeOut[1] + 1 * HisServeOut[2] + 1 * HisServeOut[3])/8;

    if(ProgServo > 449) ProgServo = 449;
    if(ProgServo < -449) ProgServo = -449;
    ServoAbs = Abs(ProgServo);
    SpdDiff=(signed long)(DiffTbl[ServoAbs/5]*NowSpeed)/DIFF_TEMP;
    //SpdDiff=0;
    //SpdDiff = (INT16S)(((FP32)((objRate-1)*1000.0*ObjectSpeed/(objRate+1)))/Kp_DiffSpd);
    //SpdDiff = (INT16S)(((INT32S)((objRate-1000)*1000L*NowSpeed/(objRate+1000)))/Kp_DiffSpd);
    //ProgServo>0 右转弯 :ProgServo<0 左转弯
    if(ProgServo>0)
    {//加速优先                     
        Speed_L.ObjectSpeed = double_ObjectSpeed -5*SpdDiff/5;//6,1
        Speed_R.ObjectSpeed = double_ObjectSpeed + 5*SpdDiff/5;
    }
    else
    {
        Speed_L.ObjectSpeed = double_ObjectSpeed + 5*SpdDiff/5;
        Speed_R.ObjectSpeed = double_ObjectSpeed - 5*SpdDiff/5;
    } 
}


void SetMotorPWM(int Speed_L_set,int Speed_R_set)
{
    int left_forward;
    int left_backward;
    int right_forward;
    int right_backward;
    
    if(Speed_L_set>=0)
    {
       
       if(Speed_L_set>5000)
        Speed_L_set = 5000;
       left_forward = Speed_L_set;
       left_backward= 0;
    }
    else
    {
       left_forward = 0;
       Speed_L_set= -Speed_L_set ;
       if(Speed_L_set>5000)
        Speed_L_set = 5000;
       left_backward = Speed_L_set;
    }
    if(Speed_R_set>=0)
    {
       if(Speed_R_set>5000)
        Speed_R_set = 5000;
        right_forward = Speed_R_set;
        right_backward= 0;
    }
    else
    {
      
        Speed_R_set= -Speed_R_set;
        if(Speed_R_set>5000)
        Speed_R_set = 5000;
        right_forward = 0;
        right_backward= Speed_R_set;
    }
    
    if(runtime>400)
    {
        if(stop||stop_of_outroad)
        //if(stop)
        {
            ftm_pwm_duty(FTM3, FTM_CH5,0);//左轮正传
            ftm_pwm_duty(FTM3, FTM_CH6,0);//左轮反转
            ftm_pwm_duty(FTM3, FTM_CH7,0);//右轮正转
            ftm_pwm_duty(FTM3, FTM_CH4,0);//右轮反转
        }
        else
        {
            ftm_pwm_duty(FTM3, FTM_CH5,left_forward);//左轮正传
            ftm_pwm_duty(FTM3, FTM_CH6,left_backward);//左轮反转
            ftm_pwm_duty(FTM3, FTM_CH7,right_forward);//右轮正转
            ftm_pwm_duty(FTM3, FTM_CH4,right_backward);//右轮反转
        }
    }    
}






