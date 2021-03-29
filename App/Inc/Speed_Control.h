#ifndef _SPEED_CONTROL_H_
#define _SPEED_CONTROL_H_


/****************************************************/
#define MAX_MotorOut     5000
#define MIN_MotorOut     3300
/***************************函数外部声明**********************************/
//系统运行时间
extern unsigned long runtime;

typedef struct
{
  int MOROR_id;
  int nowSpeed;
  int ObjectSpeed;
  int PI_SpeedErr;
  int His_PI_SpeedErr;
  int His2_PI_SpeedErr;
  int Motor_Out;
}MOTOR_CTR;

typedef struct
{
  
    int Adj_KP_Speed;
    int Adj_KI_Speed;
    int Adj_KD_Speed;
    int Adj_BangBang_Pos;
    int Adj_BangBang_Neg;
    int Adj_KP_DifSpd;
    int Adj_KD_DifSpd;
    int Adj_KI_DifSpd;
                  /*当前速度*/
}MOTOR_PID;

extern int16 double_ObjectSpeed;      //无意义变量
extern int16 NowSpeed;                //当前小车平均速度
extern int16 MotorSpeed;              //小车设定平均速度

extern int16 SpeedErr;
extern int16 His_SpeedErr;
extern int16 His2_SpeedErr;

extern long int double_Speed_out;
extern long int P_part;
extern long int I_part;
extern long int Pwm_Delta;



extern MOTOR_CTR Speed_R;
extern MOTOR_CTR Speed_L;
extern MOTOR_CTR Speed_DIFF;
extern MOTOR_PID Speed_all;


void Speed_get_init(void);
void PIT1_IRQHandler(void);
extern void Speed_Read(void);
extern void SetMotorSpeed(void);
extern void speed_decision(void);
extern void SpeedControl (void);
extern void Diff_Speed(signed long serve_value);
extern void DifferSpeed_PID(void);
extern void SetMotorPWM(int Speed_L_set,int Speed_R_set);

#endif    /**_SPEED_CONTROL_H_*/
