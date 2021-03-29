
#include "common.h"
#include "include.h"
#include "math.h"
#include "project.h"
#include "picture.h"
#include "S3010_Control.h"
#include "Speed_Control.h"

#define S3010_FTM   FTM0
#define S3010_CH    FTM_CH5                            //舵机初始化PTA12 
#define S3010_HZ    (100)


extern int deltax;
int speedtest=0;
void  main(void)//CNST
{
    ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,S_D5_centervalue);//舵机初始化  FTM1_CH0_PIN   PTA12
    ftm_pwm_init(FTM3, FTM_CH4,10*1000,0);
    ftm_pwm_init(FTM3, FTM_CH5,10*1000,0);
    ftm_pwm_init(FTM3, FTM_CH6,10*1000,0);
    ftm_pwm_init(FTM3, FTM_CH7,10*1000,0);
    
    gpio_init(PTB22,GPO,LED_ON);
    
    gpio_init(PTE27,GPI,0);
    port_init(PTE27,PF|ALT1|PULLUP);
    gpio_init(PTE26,GPI,0);
    port_init(PTE26,PF|ALT1|PULLUP);
    gpio_init(PTE25,GPI,0);
    port_init(PTE25,PF|ALT1|PULLUP);
    gpio_init(PTE24,GPI,0);
    port_init(PTE24,PF|ALT1|PULLUP);
     //配置中断优先级
    NVIC_SetPriority(PORTA_IRQn,1);                         
    NVIC_SetPriority(DMA0_IRQn,0);
    NVIC_SetPriority(PIT1_IRQn,2); 
    //FTM初始化
    
    //这里设定  imgbuff 为采集缓冲区
    camera_init(imgbuff); 
    Key_Init();

    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置DMA0的中断服务函数
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置PORTA的中断服务函为 DMA0_IRQHandler
    Speed_get_init(); 
   
   // OLED_Init();
    while(1)
    {
        
        camera_get_img();//获取图像		                          
        deal_picture();
        S3010_control();
        if(ring_flag)
        {
           gpio_set(PTB22,LED_ON);
        }
        else
        {
           gpio_set(PTB22,LED_OFF);
        }
//                if(speedtest)
//        {
//           gpio_set(PTB22,LED_ON);
//        }
//        else
//        {
//           gpio_set(PTB22,LED_OFF);
//        }
//        int8 Speed_LR[3];
//        Speed_LR[0] = MotorSpeed;
//        Speed_LR[1] = Speed_L.nowSpeed;
//        Speed_LR[2] = Speed_R.nowSpeed;
//        vcan_sendware(Speed_LR, sizeof(Speed_LR));
    }
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}

