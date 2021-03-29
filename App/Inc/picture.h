#include "include.h"


#ifndef PICTURE_H
#define PICTURE_H

#define H 70
#define W 160
typedef struct KB
{
    float x;
    float y;
}line_tampeta;

extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 image[H][W];
extern uint8 send_buff[H*W/8];
extern uint8 border[H][2];     //最左、右边界存储数组
extern uint8 centerline[H];    //中线数组
extern uint8 width_array[H];   //直线赛道宽度数组
extern uint8 temp[3];          //temp[0]->白色块的起始位置 temp[1]->白色块的终止位置 temp[3]->白色块是否与上一行连通（ 1->为连通 0->不连通）
extern uint8 effctline;        //有效行
extern uint8 lasteffctline;
extern uint8 ring_flag;        //环形弯标志
extern uint8 cross_flag;
extern uint8 obstacle_flag;
extern uint8 ring_count;



//停车的相关变量
extern int stop_count1;
extern int start_flag1;
extern int start_flag2;
extern int stop ;
extern int stop_line ;
//extern line_tampeta leftK1;
//extern line_tampeta leftK2;
//extern line_tampeta rightK1,
//extern line_tampeta rightK2;
typedef struct point
{
    uint8 x;
    uint8 y;
}Point;

//图像处理有关函数

void deal_picture(void);
//提取边界
void get_border(uint8 image[][W]);
//环形弯检测
void ring_road(void);
//十字弯判断
int cross_road(void);
//障碍物检测
void obstacle(void);
//障碍物处理
void obstacle_deal(void);
//十字弯检测
int cross_road(void);
//边界处理，比如十字。弯道的边界需要处理
void second_deal_border(void);
//显示边界
void show_border(uint8 image[][W]);
//计算得到中线
void getcenter(void);
//中线过滤函数
void centerline_filter(int end_line);
//有效行过滤函数
void effctline_filter(void);
//确定白色块的起始位置
uint8* white_block(uint8 row,uint8 col,uint8 image[][W],int deriction);
//环形弯补线确定白色区域块是否属于连通区域，目标区域赋值为1，背景赋值为0
void graph_region_label(uint8 image[][W],uint8 last_center);
//停车检测
int stopcar(void);
//补线函数 flag->0 补左边界flag->1补右边界 flag->2补中线

line_tampeta linear_fittint(int start,int end,int mode);
int get_left_max(int end);
int get_right_min(int end);
int get_left(int row,int center);
int get_right(int row,int center);
int get_left_row_max(int start,int end);
int get_right_row_min(int start,int end);
int lookfor_peak(float k,int start,int end,int mode);
void mend_line(uint8 X1,uint8 Y1,uint8 X2,uint8 Y2,uint8 flag);
// 推测点
uint8 get_point(uint8 X1, uint8 Y1,uint8 X2,uint8 Y2,uint8 X3);
/******************************************************************
*函数名称：zero_point_get
*函数功能：穿越加权线的零点个数
*备注：
********************************************************************/
int zero_point_get(int start,int end,uint8 *a);
/******************************************************************
*函数名称：dispersion_degree
*函数功能：求一系列点的离散程度
*备注：
********************************************************************/
int dispersion_degree(int begin,int end,int mode);
/*****************************************************************
线性拟合
start—>起始行
end->终止行
mode-> 0 走边界拟合 1->右边界拟合 2->中线拟合
******************************************************************/

//压缩图像 eight pixels is a character
void  img_reduce(uint8 *aim,uint8 *get,uint32 length);
void image_copy(void);
#endif



