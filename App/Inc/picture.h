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
extern uint8 border[H][2];     //�����ұ߽�洢����
extern uint8 centerline[H];    //��������
extern uint8 width_array[H];   //ֱ�������������
extern uint8 temp[3];          //temp[0]->��ɫ�����ʼλ�� temp[1]->��ɫ�����ֹλ�� temp[3]->��ɫ���Ƿ�����һ����ͨ�� 1->Ϊ��ͨ 0->����ͨ��
extern uint8 effctline;        //��Ч��
extern uint8 lasteffctline;
extern uint8 ring_flag;        //�������־
extern uint8 cross_flag;
extern uint8 obstacle_flag;
extern uint8 ring_count;



//ͣ������ر���
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

//ͼ�����йغ���

void deal_picture(void);
//��ȡ�߽�
void get_border(uint8 image[][W]);
//��������
void ring_road(void);
//ʮ�����ж�
int cross_road(void);
//�ϰ�����
void obstacle(void);
//�ϰ��ﴦ��
void obstacle_deal(void);
//ʮ������
int cross_road(void);
//�߽紦������ʮ�֡�����ı߽���Ҫ����
void second_deal_border(void);
//��ʾ�߽�
void show_border(uint8 image[][W]);
//����õ�����
void getcenter(void);
//���߹��˺���
void centerline_filter(int end_line);
//��Ч�й��˺���
void effctline_filter(void);
//ȷ����ɫ�����ʼλ��
uint8* white_block(uint8 row,uint8 col,uint8 image[][W],int deriction);
//�����䲹��ȷ����ɫ������Ƿ�������ͨ����Ŀ������ֵΪ1��������ֵΪ0
void graph_region_label(uint8 image[][W],uint8 last_center);
//ͣ�����
int stopcar(void);
//���ߺ��� flag->0 ����߽�flag->1���ұ߽� flag->2������

line_tampeta linear_fittint(int start,int end,int mode);
int get_left_max(int end);
int get_right_min(int end);
int get_left(int row,int center);
int get_right(int row,int center);
int get_left_row_max(int start,int end);
int get_right_row_min(int start,int end);
int lookfor_peak(float k,int start,int end,int mode);
void mend_line(uint8 X1,uint8 Y1,uint8 X2,uint8 Y2,uint8 flag);
// �Ʋ��
uint8 get_point(uint8 X1, uint8 Y1,uint8 X2,uint8 Y2,uint8 X3);
/******************************************************************
*�������ƣ�zero_point_get
*�������ܣ���Խ��Ȩ�ߵ�������
*��ע��
********************************************************************/
int zero_point_get(int start,int end,uint8 *a);
/******************************************************************
*�������ƣ�dispersion_degree
*�������ܣ���һϵ�е����ɢ�̶�
*��ע��
********************************************************************/
int dispersion_degree(int begin,int end,int mode);
/*****************************************************************
�������
start��>��ʼ��
end->��ֹ��
mode-> 0 �߽߱���� 1->�ұ߽���� 2->�������
******************************************************************/

//ѹ��ͼ�� eight pixels is a character
void  img_reduce(uint8 *aim,uint8 *get,uint32 length);
void image_copy(void);
#endif



