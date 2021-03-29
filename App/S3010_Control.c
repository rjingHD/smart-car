#include "include.h"
#include "project.h"
#include "picture.h"
#include "Speed_Control.h"
#include "S3010_Control.h"
#define QEP_time 5000
//舵机相关的宏定义
#define S3010_FTM   FTM0
#define S3010_CH    FTM_CH5                             //舵机初始化PTA12 
#define S3010_HZ    (100)

#define ServeKp_numL 80
#define ServeKp_numR 80
#define row_num      9

#define Control_left_limit  4340//4350
#define Control_right_limit 3400 //3350 3400-3390


/*****************舵机相关***********/
float row_percent[row_num] = {0.454,0.32,0.151,0.05,0.005,0.005,0.005,0.005,0.005};
int deltax = 0;

int deltax_error;
int pre_deltax = 0; //远处路况检测
int S3010_row = 0;
int Control = S_D5_centervalue;
int Control_error =0;
int radius = 0;    //弯道半径
float curvature = 0; //曲率


/****************************
速度设定
*****************************/
int mode1= 0;
int mode2= 0;
int mode3= 0;
int mode4= 0;
/*****************************
环形弯转向设定
*****************************/
int ring_direction1 = 0;
int ring_direction2 = 0;






void Key_Init(void)
{
#define KEY1 gpio_get(PTE27)
#define KEY2 gpio_get(PTE26)
#define KEY3 gpio_get(PTE25)
#define KEY4 gpio_get(PTE24)

    if(KEY1==0&&KEY2==0)
    {
       DELAY_MS(10);
       if(KEY1==0&&KEY2==0)
       {
          mode1 = 1;
       }
    }
    if(KEY1==0&&KEY2==1)
    {
       DELAY_MS(10);
       if(KEY1==0&&KEY2==1)
       {
          mode2 = 1;
       }
    }
    if(KEY1==1&&KEY2==0)
    {
       DELAY_MS(10);
       if(KEY1==1&&KEY2==0)
       {
          mode3 = 1;
       }
    }
    if(KEY1==1 && KEY2==1)
    {
       DELAY_MS(10);
       if(KEY1==1&&KEY2==1)
       {
         mode4 = 1;
       }
    }
    if(KEY3==0)
    {
       DELAY_MS(10);
       if(KEY3==0)
        ring_direction1 = 0;
    }
    if(KEY3==1)
    {
       DELAY_MS(10);
       if(KEY3==1)
         ring_direction1 = 1;
    }
    if(KEY4== 0)
    {
       DELAY_MS(10);
       if(KEY4==0)
         ring_direction2 = 0;
    }
    if(KEY4== 1)
    {
       DELAY_MS(10);
       if(KEY4==1)
        ring_direction2 = 1; 
    }
}

             
void S3010_control()
{
   // float ServeKpL[ServeKp_numL] = {0,1.91,2.68,2.54,5.49,5.52,5.64,5.85,6.15,7.53,9,9.56,9.20,11.93,12.75,12.66,11.65,11.73,11.90,11.16,11.50,11.93,15.43,15,14.63,14.31,14.04,13.81,13.61,13.45,13.32,13.21,13.11,13.03,12.95,12.87,12.79,12.70,12.60,12.49,12.38,12.26,12.13,12.01,11.88,11.76,11.63,11.51,11.39,11.27,11.16,11.05,10.94,10.83,10.73,10.63,10.54,10.44,10.35,10.27,10.19,10.11,10.04,9.980,9.920,9.870,9.830,9.790,9.760,8.740,8.720,8.720,8.720,8.730,8.750,8.780,8.820,8.870,8.930};
   //float ServeKpR[ServeKp_numR] = {0,1.91,2.68,2.54,5.49,5.52,5.64,5.85,6.15,7.53,9,9.56,9.20,11.93,12.75,12.66,11.65,11.73,11.90,11.16,11.50,11.93,15.43,15,14.63,14.31,14.04,13.81,13.61,13.45,13.32,13.21,13.11,13.03,12.95,12.87,12.79,12.70,12.60,12.49,12.38,12.26,12.13,12.01,11.88,11.76,11.63,11.51,11.39,11.27,11.16,11.05,10.94,10.83,10.73,10.63,10.54,10.44,10.35,10.27,10.19,10.11,10.04,9.980,9.920,9.870,9.830,9.790,9.760,8.740,8.720,8.720,8.720,8.730,8.750,8.780,8.820,8.870,8.930};
      
  
    float ServeKpR[ServeKp_numL] = {1.38 ,2.36 ,2.77 ,2.72 ,2.72 ,3.26 ,3.85 ,4.48 ,4.65 ,5.37 ,5.63 ,5.93 ,5.78 ,6.49 ,6.87 ,6.84 ,6.36 ,6.42 ,6.53 ,6.18 ,6.37 ,6.61 ,8.38 ,8.18 ,8.02 ,7.88 ,7.77 ,7.68 ,7.60 ,7.54 ,7.49 ,7.46 ,7.43 ,7.41 ,7.39 ,7.38 ,7.40 ,7.41 ,7.42 ,7.43 ,7.43 ,7.41 ,7.88 ,7.86 ,7.83 ,7.80 ,7.78 ,7.75 ,7.73 ,7.71 ,8.13 ,8.07 ,8.01 ,7.96 ,7.90 ,7.85 ,7.81 ,7.76 ,7.71 ,7.66 ,7.62 ,7.58 ,7.55 ,7.14 ,6.74 ,6.72 ,6.71 ,6.69 ,6.69 ,6.19 ,5.94 ,5.96 ,5.98 ,6.01 ,6.10 ,6.20 ,7.38 ,7.50 ,7.63 ,7.75 };
  // float ServeKpR[ServeKp_numR] = {0.69 ,2.13 ,2.72 ,2.63 ,2.61 ,3.39 ,4.24 ,5.16 ,5.40 ,6.45 ,7.31 ,7.75 ,7.49 ,9.21 ,9.81 ,9.75 ,9.01 ,9.08 ,9.21 ,8.67 ,8.94 ,9.27 ,11.91 ,11.59 ,11.33 ,11.10 ,10.90 ,10.74 ,10.60 ,10.49 ,10.41 ,10.34 ,10.27 ,10.22 ,10.17 ,10.12 ,10.09 ,10.06 ,10.01 ,9.96 ,9.91 ,9.83 ,10.01 ,9.93 ,9.85 ,9.78 ,9.70 ,9.63 ,9.56 ,9.49 ,9.65 ,9.56 ,9.48 ,9.39 ,9.31 ,9.24 ,9.17 ,9.10 ,9.03 ,8.97 ,8.91 ,8.84 ,8.79 ,8.56 ,8.33 ,8.30 ,8.27 ,8.24 ,8.23 ,7.47 ,7.33 ,7.34 ,7.35 ,7.37 ,7.42 ,7.49 ,8.10 ,8.19 ,8.28 ,8.50 };
//   float ServeKpR[ServeKp_numR] = {0.13,2.04,2.81,2.67,2.62,3.65,4.77,5.98,6.28,7.66,7.13,7.69,7.33,10.15,10.97,10.88,9.87,9.95,10.12,9.38,9.72,10.15,13.57,13.14,12.77,12.45,12.18,11.95,11.75,11.59,11.46,11.35,11.25,11.17,11.09,11.01,10.93,10.84,10.74,10.63,10.52,10.4,12.27,12.15,12.02,11.9,11.77,11.65,11.53,11.41,11.3,11.19,11.08,10.97,10.87,10.77,10.68,10.58,10.49,10.41,10.33,10.25,10.18,8.64,8.58,8.53,8.49,8.45,8.42,7.4,7.38,7.38,7.38,7.39,7.41,7.44,7.48,7.53,7.59,7.63};
                          //         0    1      2     3     4     5     6     7     8     9    10     11    12    13   14     15    16    17    18   19     20   21     22     23    24     25    26    27    28    29    30   31    32     33    34    35   36    37    38    39    40    41     42     43     44     45     46     47     48     49     50     51     52     53     54    55      56     57     58     59     60     61     62     63    64   65    66     67    68    69   70    71    72    73    74    75     76   77    78    79  
    float ServeKpL[ServeKp_numR] = {1.38 ,2.36 ,2.77 ,2.72 ,2.72 ,3.26 ,3.85 ,4.48 ,4.65 ,5.37 ,5.13 ,5.43 ,5.28 ,5.72 ,6.12 ,6.10 ,5.63 ,5.71 ,5.82 ,5.49 ,5.69 ,5.94 ,7.72 ,7.54 ,7.39 ,7.27 ,7.16 ,7.08 ,7.02 ,6.97 ,6.94 ,6.92 ,6.90 ,6.89 ,6.89 ,6.88 ,6.91 ,6.94 ,6.96 ,6.98 ,7.00 ,6.99 ,7.48 ,7.46 ,7.45 ,7.41 ,7.38 ,7.34 ,7.31 ,7.28 ,7.69 ,7.61 ,7.53 ,7.45 ,7.38 ,7.30 ,7.24 ,7.17 ,7.05 ,7.04 ,7.04 ,7.03 ,7.03 ,6.66 ,6.30 ,6.31 ,6.33 ,6.36 ,6.39 ,5.80 ,5.54 ,5.55 ,5.57 ,5.59 ,5.61 ,5.65 ,6.77 ,6.84 ,6.91 ,6.95 };

   //float ServeKpL[ServeKp_numR] = {1.38 ,2.36 ,2.77 ,2.72 ,2.72 ,3.26 ,3.85 ,4.48 ,4.65 ,5.37 ,5.63 ,5.93 ,5.78 ,6.49 ,6.87 ,6.84 ,6.36 ,6.42 ,6.53 ,6.18 ,6.37 ,6.61 ,8.38 ,8.18 ,8.02 ,7.88 ,7.77 ,7.68 ,7.60 ,7.54 ,7.49 ,7.46 ,7.43 ,7.41 ,7.39 ,7.38 ,7.40 ,7.41 ,7.42 ,7.43 ,7.43 ,7.41 ,7.88 ,7.86 ,7.83 ,7.80 ,7.78 ,7.75 ,7.73 ,7.71 ,8.13 ,8.07 ,8.01 ,7.96 ,7.90 ,7.85 ,7.81 ,7.76 ,7.71 ,7.66 ,7.62 ,7.58 ,7.55 ,7.14 ,6.74 ,6.72 ,6.71 ,6.69 ,6.69 ,6.19 ,5.94 ,5.96 ,5.98 ,6.01 ,6.10 ,6.20 ,7.38 ,7.50 ,7.63 ,7.75 };
   float Serve_Kd_in = 50;
   float Serve_Kd_out = 45;
    if(abs(deltax)<=20)
    {  
         ServeKpR[abs(deltax)]+=0.5;
            ServeKpL[abs(deltax)]+=0.5;  
    }
    if(mode1 == 1)
    { 
        Serve_Kd_in = 27;//8.7 50  45
        Serve_Kd_out = 20;
        if(abs(deltax)<40)
        {
            Serve_Kd_in  =  25;
            Serve_Kd_out =  17;
        }
        if(abs(deltax)>18 && abs(deltax)<65) 
        {
            ServeKpR[abs(deltax)]+=1.8;
         //   ServeKpL[abs(deltax)]+=1.8;
        }
        if(abs(deltax)>65)
        {
            ServeKpR[abs(deltax)]+=1.5;
          //  ServeKpL[abs(deltax)]+=0.5;
        }
        if(ring_flag)
        {
           Serve_Kd_in  =  30;
           Serve_Kd_out =  30;
        }
        if(obstacle_flag)
        {
          ServeKpR[abs(deltax)] = 6;
          ServeKpR[abs(deltax)] = 6;
          Serve_Kd_in  =  17;
          Serve_Kd_out =  17;
        }
     }
     if(mode2 == 1)
     { 
        Serve_Kd_in = 27;//8.7 50  45
        Serve_Kd_out = 20;
        if(abs(deltax)<40)
        {
            Serve_Kd_in  =  25;
            Serve_Kd_out =  17;
        }
        if(abs(deltax)>18 && abs(deltax)<65) 
        {
            ServeKpR[abs(deltax)]+=1.8;
         //   ServeKpL[abs(deltax)]+=1.8;
        }
        if(abs(deltax)>65)
        {
            ServeKpR[abs(deltax)]+=1.5;
          //  ServeKpL[abs(deltax)]+=0.5;
        }
        if(ring_flag)
        {
           Serve_Kd_in  =  30;
           Serve_Kd_out =  30;
        }
        if(obstacle_flag)
        {
          ServeKpR[abs(deltax)] = 6;
          ServeKpR[abs(deltax)] = 6;
          Serve_Kd_in  =  17;
          Serve_Kd_out =  17;
        }
     }
     if(mode3 == 1)
     { 
        Serve_Kd_in = 27;//8.7 50  45
        Serve_Kd_out = 20;
        if(abs(deltax)<40)
        {
            Serve_Kd_in  =  25;
            Serve_Kd_out =  17;
        }
        if(abs(deltax)>18 && abs(deltax)<65) 
        {
            ServeKpR[abs(deltax)]+=1.8;
         //   ServeKpL[abs(deltax)]+=1.8;
        }
        if(abs(deltax)>65)
        {
            ServeKpR[abs(deltax)]+=1.5;
          //  ServeKpL[abs(deltax)]+=0.5;
        }
        if(ring_flag)
        {
           Serve_Kd_in  =  30;
           Serve_Kd_out =  30;
        }
        if(obstacle_flag)
        {
          ServeKpR[abs(deltax)] = 8;
          ServeKpR[abs(deltax)] = 8;
          Serve_Kd_in  =  20;
          Serve_Kd_out =  20;
        }
     }
     if(mode4==1)
     {
        Serve_Kd_in = 22;//8.7 50  45
        Serve_Kd_out = 18;
        if(abs(deltax)<40)
        {
            Serve_Kd_in  =  18;
            Serve_Kd_out =  14;
        }
        if(abs(deltax)>18 && abs(deltax)<65) 
        {
            ServeKpR[abs(deltax)]+=1.8;
         //   ServeKpL[abs(deltax)]+=1.8;
        }
        if(abs(deltax)>65)
        {
            ServeKpR[abs(deltax)]+=1.5;
          //  ServeKpL[abs(deltax)]+=0.5;
        }
        if(ring_flag)
        {
           Serve_Kd_in  =  30;
           Serve_Kd_out =  25;
        }
        if(obstacle_flag)
        {
          ServeKpR[abs(deltax)] = 8;
          ServeKpR[abs(deltax)] = 8;
          Serve_Kd_in  =  20;
          Serve_Kd_out =  20;
        }
     }
    int i = 0;
    int j = 0;
    int deltax_last = 0; //上一次的偏差
    
    float center_error= 0; //浮点型中线加权量
    deltax_last  = deltax;
    if(effctline>=0)
    {
        if(effctline>H-10)
        {
            for(i=effctline;i>=0;i--)
               centerline[i] = centerline[H-1];
        }
        else if(effctline<=35&&abs(centerline[effctline]-W/2)<10&&ring_flag==0)
        {
            for(i=effctline;i>=0;i--)
               centerline[i] = W/2;
        }
        else if(centerline[H-1]<centerline[effctline])
        {
            for(i=effctline;i>=0;i--)
               centerline[i] = W-1;
        }
        else
        {
           if(centerline[H-1]>centerline[effctline])
           {
              for(i=effctline;i>=0;i--)
               centerline[i] = 1;
           }
        }
    }
    for(i= 15;i< 67; i++)
    {
        if(i==23 || i==25 || i==27 || i==35 || i==37 || i==39 || i==60 || i==63 || i==66 )
        {
            center_error += (W/2-centerline[i])*row_percent[j];
            j++;
        }

    }
    deltax = (int)(center_error);
    //路径优化
//    if((zero_point_get(1,H-2,centerline)==3||zero_point_get(1,H-2,centerline)==4)&&effctline<=5&&abs(W/2-centerline[5])<=30 \
//       &&!ring_flag&&!obstacle_flag&&!stopcar())
//    {
//        printf("s弯\n");
//        deltax= (int)(W/2-(get_left_max(effctline+5)+get_right_min(effctline+5))/2);
//    }
    Control_error=deltax-deltax_last;
    if (deltax>= W/2) deltax = W/2;
    if (deltax<= -W/2) deltax = -W/2;
    if(deltax> 0)
    {
        if(deltax*(deltax-deltax_last)>0) 
        { 
            Control = (int)(ServeKpL[abs(deltax)]*deltax + Serve_Kd_in*(deltax-deltax_last))+S_D5_centervalue;
        }
        else if(deltax*(deltax-deltax_last)<= 0) 
        {
            Control = (int)(ServeKpL[abs(deltax)]*deltax + Serve_Kd_out*(deltax-deltax_last))+S_D5_centervalue;
        }
    }
    else if(deltax<= 0)
    {
        if(deltax*(deltax-deltax_last)>0)
        {
            Control = (int)(ServeKpR[abs(deltax)]*deltax + Serve_Kd_in*(deltax-deltax_last))+S_D5_centervalue;
        }
        else if(deltax*(deltax-deltax_last)<= 0)
        {
            Control = (int)(ServeKpR[abs(deltax)]*deltax + Serve_Kd_in*(deltax-deltax_last))+S_D5_centervalue;
        }   
    }
    if(deltax>= ServeKp_numR-1) Control = Control_left_limit;
    if(deltax<= -ServeKp_numR+1) Control = Control_right_limit;
    if(Control>Control_left_limit)  Control=Control_left_limit;
    if(Control<Control_right_limit)   Control=Control_right_limit;
    if(stop==0&&stopcar() && stop_line>=50) 
    {
      Control = S_D5_centervalue;
      DELAY_MS(50);
    }
    if(stop&&stopcar()&& stop_line>=50)
    {
       Control = S_D5_centervalue;
       DELAY_MS(70);
    }
    ftm_pwm_duty(S3010_FTM, S3010_CH,Control);
}
