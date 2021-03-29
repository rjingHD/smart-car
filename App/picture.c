#include "include.h"
#include "picture.h"
#include "project.h"
#include "S3010_Control.h"
#include "Speed_Control.h"

#define H 70
#define W 160
#define Middle W/2

uint8 imgbuff[CAMERA_SIZE];
uint8 image[H][W];
uint8 image2[H][W];
uint8 send_buff[H*W/8] = {0};
uint8 border[H][2];     //最左、右边界存储数组
uint8 left_flag[H];     //赛道左边界标记
uint8 right_flag[H];    //赛道右边界标记
uint8 centerline[H];    //中线数组

uint8 width_array[H]={ 27,28,29,30,30,30,30,31,32,33,
                  35,36,38,39,41,42,44,45,46,48,
                  50,52,53,55,57,57,59,61,63,64,
                  66,68,69,71,72,73,75,77,79,80,
                  81,83,85,86,88,89,90,92,93,95,
                  97,99,99,101,103,104,106,108,108,110,
                  111,113,114,115,117,119,120,122,122,122,
};   //直线赛道宽度数组


uint8 temp[3];          //temp[0]->白色块的起始位置 temp[1]->白色块的终止位置 temp[3]->白色块是否与上一行连通（ 1->为连通 0->不连通）
uint8 effctline=0;        //有效行
uint8 lasteffctline; 
uint8 ring_flag = 0;        //环形弯标志
uint8 lastring_flag = 0;

uint8 ring_first_break_point = H-1;
uint8 ring_count= 0;
uint8 ring_flag_count = 0;
int ring_left_begin,ring_left_over;
int ring_right_begin,ring_right_over;
/*********************
十字标志
*********************/
uint8 cross_flag= 0; 
/*******************
*******************/
uint8 obstacle_flag =0;

static uint8 last_middle = Middle;
int leftdispersion;
int rightdispersion;

/***********************************************/
//停车的相关变量

int stop_count1 =0;
int start_flag1 = 0;
int start_flag2 = 0;
int stop = 0;
int stop_line = 0;




void deal_picture(void)
{
    lastring_flag = ring_flag;
    obstacle_flag =0;
    image_copy();         //图像解压
    get_border(image);    //边界提取
    if(stopcar()==0)
    {
        obstacle();           //障碍物检测
    }
    ring_road();          //环形弯检测
    if(start_flag1==0)
    {
       if(stopcar())
         start_flag1=1;
    }
    if(start_flag1&&!stopcar())
    {
       start_flag2 = 1;
    }
    if(stopcar())
    {
      obstacle_flag = 0;
      ring_flag = 0;
    }
    if(obstacle_flag)
      ring_flag = 0;
    if(cross_road())
      ring_flag = 0;
    if(ring_flag)
    {
        ring_count++;
        graph_region_label(image,last_middle);
        //getcenter();
        if(ring_count>160)
        {
           ring_count = 0;
           ring_flag = 0;
        }
    }
    if(ring_flag==0&&obstacle_flag==0)
    {
       second_deal_border(); 
    }
    
    if(lastring_flag==1&&ring_flag==0)
    {
       ring_flag_count++;
      // printf("%d\n",ring_flag_count);
    }
   
#if 0
    show_border(image2);
    img_reduce((uint8 *)image2,send_buff,H*W);
    vcan_sendimg(send_buff,H*W/8);
#endif
}


//提取边界
void get_border(uint8 image[][W])
{
    int i,j,ii,jj;
    int i_begin, i_stop;
    int temp;
    int max_width;
    uint8 left,right;
    uint8 temp_left1,temp_left2;
    uint8 left_lost;
    uint8 temp_right1,temp_right2;
    uint8 right_lost;
    int8 street_stautus;
    centerline[H-1] = Middle;
    lasteffctline = effctline;
    for(i=H-1; i>-1; i--) 
    {
        left_flag[i] = 0;
        right_flag[i] = 0;
        
        if(i==H-1)
        {
          if(image[i][last_middle])
          {
               for(j=last_middle; j<W-1&&image[i][j]!=0; j++){}
               if(j>W-3)
               {
                  right = W-2;
                  right_flag[i] = 1;
               }
               else{right = j-1;}
               for(j=last_middle; j>-1&&image[i][j]!=0; j-- ){}
               if(j<2)
               {
                  left = 1;
                  left_flag[i] = 1;
               }
               else{left = j+1;}
          }
          else if(image[i][W/2])
          {
              for(j=last_middle; j<W-1&&image[i][j]!=0; j++){}
              if(j>W-3)
              {
                  right = W-2;
                  right_flag[i] = 1;
              }
              else{right = j-1;}
              for(j=last_middle; j>-1&&image[i][j]!=0; j-- ){}
              if(j<2)
              {
                  left = 1;
                  left_flag[i] = 1;
              }
              else{left = j+1;}
          }
          else if(image[i][3*W/4])
          {
               for(j=3*W/4; j<W-1&&image[i][j]!=0; j++){}
                if(j>W-3)
                {
                    right = W-2;
                    right_flag[i] = 1;
                }
                else{right = j-1;}
                for(j=3*W/4; j>0&&image[i][j]!=0; j-- ){}
                if(j<2)
                {
                    left = 1;
                    left_flag[i] = 1;
                }
                else{left = j+1;}
          }
          else
          {
                for(j= W/4; j<W-1&&image[i][j]!=0; j++){}
                if(j>W-3)
                {
                    right = W-2;
                    right_flag[i] = 1;
                }
                else{right = j-1;}
                for(j=W/4; j>0&&image[i][j]!=0; j-- ){}
                if(j<2)
                {
                    left = 1;
                    left_flag[i] = 1;
                }
               else{left = j+1;}
          }
        }
        else
        {
            if(street_stautus== 0)
            {
               //右边界查询
                if(image[i][right]!= 0)//white
                {
                    temp = min(right+6,W-1);
                    for(j= right; j< temp && image[i][j]!= 0; j++){}
                    if(j==temp)
                    { 
                          /****************************************
                          2017年8月1日
                          丢线判断修改
                          ****************************************/
                        if(i>H/2-10)
                        {
                            right_lost = 0;
                            ii = i;
                            temp_right1 = border[i+1][1];
                            for(ii= i;ii> max(ii-10,0); ii--)
                            {
                               temp_right2 = get_right(ii,temp);
                               if(temp_right2 - temp_right1>5|| temp_right2>W-10)
                               {
                                  right_lost++;
                               }
                               else
                               {
                                  break;
                               }
                               temp_right1 = temp_right2;
                            }
                            if(right_lost>=5)
                            {
                                right = right;
                                right_flag[i]= 1;
                            }
                            else
                            {
                                right = get_right(i,right);
                            }
                        }
                        else
                        {
                            right = right;
                            right_flag[i]= 1;
                        }
                    }
                    else
                    {
                        right = get_right(i,right);
                    }
                }
                else
                {
                    temp = max(right-6,left);
                    for(j=right; j>temp &&image[i][j]==0; j--){}
                    if(j==temp)
                    {
                        right = right;
                        right_flag[i]= 1;
                    }
                    else {right = j+1;}
                }
                //左边界查询
                if(image[i][left]!= 0)
                {   
                    temp = max(left-6,0) ;
                    for(j= left; j> temp && image[i][j]!= 0; j--){}
                    /******************
                    原来的丢线判断
                    *******************/
//                    if(j== temp)
//                    {
//                        left = left;
//                        left_flag[i]= 1;
//                    }
//                    else{ left= j+1;} 
                    /******************************************/
                    
                     if(j==temp)
                    {
                      /****************************************
                      2017年8月1日
                      丢线判断修改
                      ****************************************/
                        if(i>H/2-10)
                        {
                            left_lost = 0;
                            ii = i;
                            temp_left1 = border[i+1][0];
                            for(ii= i;ii> max(ii-10,0); ii--)
                            {
                               temp_left2 = get_left(ii,temp);
                               if(temp_left1 - temp_left2>5 ||temp_left2<10)
                               {
                                  left_lost++;
                               }
                               else
                               {
                                  break;
                               }
                               temp_left1 = temp_left2;
                            }
                            if(left_lost>=5)
                            {
                                left = left;
                                left_flag[i]= 1;
                            }
                            else
                            {
                               left = get_left(i,left);
                            }
                        }
                        else
                        {
                            left = left;
                            left_flag[i]= 1;
                        }
                    }
                    else
                    {
                        left = get_left(i,left);
                    }
                    /*************************************/
                }
                else
                {
                    temp = min(left+6,right);
                    for(j= left; j< temp && image[i][j]== 0; j++){}
                    if(j==temp)
                    {
                        left = left;
                        left_flag[i]= 1;
                    }
                    else{left= j-1;}
                }
            }
            //上一次左边界丢点
            else if(street_stautus== -1)
            {
                //检验左边界是否丢点
                //左边界查询
                if(image[i][left]!= 0)
                {
                    //可以修改
                    left = left;
                    left_flag[i] = 1;
                }
                else
                {
                    for(j= left; j< right && image[i][j]== 0; j++){}
                    if(j== right)
                    {
                        left = right;
                    }
                    else {left = j-1;}
                }
                //右边界查询
                if(image[i][right]!= 0)
                {
                    if(i>35)
                    {   
//                            i_begin = i;
//                            for(ii= i_begin;ii>-1&&image[ii][right];ii--){}
//                            i_stop= max(ii+1,0);
//                            max_width = 0;
//                            for(ii= i_begin; ii>i_stop;ii--)
//                            {  
//                               for(jj=right;jj<W-1&&image[ii][jj];jj++){}
//                               if(max_width<jj-right-1)
//                               {
//                                  max_width = jj-right-1;
//                               }
//                            }
//                            if(max_width>25&&abs(right-Middle)<20&&left<20&&i_begin<H-15)
//                            {
//                                 right = right;
//                                 right_flag[i] = 1; 
//                            }
//                            else
//                            {
                                  temp = min(right+6,W-1);
                                  for(j= right; j< temp && image[i][j]!= 0; j++){}
                                  if(j==temp)
                                  {
                                        right = right;
                                        right_flag[i] = 1;
                                        
                                  }
                                  else{ right = j-1;}
                                  
//                           }
                    }
                    else
                    {
                       right = right;
                       right_flag[i] = 1;
                    }
                }
                else
                {
                    for(j= right; j> left && image[i][j]== 0; j--){}
                    if(j==left){ right= left;}
                    else{right= j+1;}
                }
            }//上一次右边丢点
            else if(street_stautus== 1)
            {
                //检验右边界是否还是继续丢线
                //右边界查询
                if(image[i][right]!= 0)
                {
                    right = right;
                    right_flag[i] = 1;
                }
                else
                {
                    for(j= right; j> left && image[i][j]== 0; j--){}
                    if(j== left){right = left;}
                    else{ right = j+1;}
                }
                //左边界查询
                if(image[i][left] != 0)
                {
                     if(i>35)
                     {
//                          i_begin = i;
//                          for(ii= i_begin;ii>-1&&image[ii][left];ii--){}
//                          i_stop= max(ii+1,0);
//                          max_width = 0;
//                          for(ii= i_begin; ii>i_stop;ii--)
//                          {  
//                               for(jj=left;jj>0&&image[ii][jj];jj--){}
//                               if(max_width<left-jj-1)
//                               {
//                                  max_width =left-jj-1 ;
//                               }
//                               //max_width>25&&abs(right-Middle)<20&&left<20&&i_begin<H-15
//                          }
//                          if(max_width>25&&abs(left-Middle)<20&&right>W-20&&i_begin<H-15)
//                          {
//                                left = left;
//                                left_flag[i] = 1; 
//                          }
//                          else
//                          {
                                temp = max(left-6, 0);
                                for(j= left; j> temp && image[i][j] != 0; j--){}
                                if(j== temp)
                                {
                                   left = left;
                                   left_flag[i] = 1;
                                }
                                else{left= j+1;}
//                         }
                          
                          
                     }
                     else
                     {
                          left = left;
                          left_flag[i] = 1;
                     }
                }
                else
                {
                    for(j= left; j< right && image[i][j]== 0; j++){}
                    if(j== right){ left = right;}
                    else {left = j-1;}
                }
            }
            //上一次左右两边都丢点
            else
            {
                //右边界查询
                //右边界是否还是继续丢点
                if(image[i][right]!= 0)
                {
                    right = right;
                    right_flag[i]= 1;
                }
                //否
                else
                {
                      for(j= right; j>left&&image[i][j]==0; j--){}
                      if(j==left){right = left;}
                      else{right= j+1;}
                }
                //左边界查询
                //走边界是否还是继续丢点
                if(image[i][left] != 0)
                {
                    left = left;
                    left_flag[i] = 1;
                }
                //否
                else
                {
                    for(j= left; j<right && image[i][j]== 0; j++){}
                    if(j== right) {left = right;}
                    else{left = j-1;}
                }
            }
        }
        border[i][0] = left;
        border[i][1] = right;
        if(left_flag[i]== 0 && right_flag[i]== 0)     //左右两边都未丢点
        {
          street_stautus = 0;
        }
        else if(left_flag[i]== 1 && right_flag[i]== 1)//左右两边都丢点
        {
            street_stautus = 2;
        }
        else if(left_flag[i]== 0 && right_flag[i]== 1)//右边丢点
        {
            street_stautus = 1;
        }                                              //左边丢点
        else
        {
            street_stautus = -1;
        }
        centerline[i] = (border[i][0] + border[i][1])/2;
        if(abs(right - left)<20 )break;
        
    }
    
    effctline = max(i+1,0) ;
    effctline = min(effctline,H-1);
   // getcenter();
    last_middle = centerline[H-2];
}
int cross_road(void)
{
    int i,j;
    int i_begin,i_stop;
    int max;
    int count;
    max = 0;
    for(i=H-1; i>=effctline;i--)
    {
        count = 0;
        for(j= i-1; i>effctline&&centerline[j]==centerline[i]&&abs(centerline[i]-W/2)<20;j--){count++;}
        if(count>max)
        {
            max = count;
            i_begin = i;
            i_stop = j;
        }
        count =0;
    } 
    if( max>=20 && effctline<10 && abs(border[i_begin][1]- border[i_begin][0])>150&&i_begin>H-10 &&(abs(i_stop-effctline)>10))
    {
        for(i=i_begin;i>effctline;i--)
          if(image[i][W/2]==0)
            break;
        if(i-effctline<20||i<25)
          return 1;
    }
    return 0;
    
}
/************************************************
环形弯判断
************************************************/
void ring_road(void)
{

      int i,j;
      int i_begin,i_stop;
      int left,right,center;
      int left_begin,left_over,left_count;
      int right_begin,right_over,right_count;
      int count;
      int left1, right1;
      int white_black_white_count=0;
      int black_left, black_right;
      int white_left, white_right;
      int black_flag  = 0;
      int left_KB_begin,left_KB_stop;
      int right_KB_begin,right_KB_stop;
      
      int rowtestline= H/2;
      //环形弯近端判断
      count = 0;
      for(i=H-1;i>rowtestline-5;i--)
      {
         if(left_flag[i]== 0 && right_flag[i]== 0)
         {
             count++;
         }
      }
      if(count>=15)
      {
         left_KB_stop = get_left_row_max(H-1,rowtestline-1);
         left_KB_begin = min(H-1,left_KB_stop+10);
         right_KB_stop = get_right_row_min(H-1,rowtestline-1);
         right_KB_begin = min(H-1,right_KB_stop+10);
         
         if(left_KB_begin-left_KB_stop>=10 && right_KB_begin-right_KB_stop>=10)
         {
               count = 0;
               for(i=left_KB_begin;i>=left_KB_stop;i--)
               {
                  if(border[i][0]<2)
                    count++;
               }
               if(count>3)
                 return;
               count = 0;
               for(i=right_KB_begin;i>=right_KB_stop;i--)
               {
                  if(border[i][1]>W-3)
                    count++;
               }
               if(count>3)
                 return;
               
               i_begin = max(left_KB_begin,right_KB_begin); 
               i_begin = min(H-1,i_begin);
               i_begin = max(i_begin,0);
               
               for(i= i_begin-1;i>-1;i--)
               {
                      left = border[left_KB_begin][0] + (border[left_KB_stop][0]-border[left_KB_begin][0])*(i-left_KB_begin)/(left_KB_stop-left_KB_begin);
                      right = border[right_KB_begin][1] + (border[right_KB_stop][1]-border[right_KB_begin][1])*(i-right_KB_begin)/(right_KB_stop-right_KB_begin);
                      center = (left+right)/2;
                      
                      if(center<2||center>W-2||left<2||left>W-2||right<2||right>W-2)
                      {
                         break;
                      }
                      if(image[i][center]==0)
                      {  
                         i_stop = i+1; //2017.7.17  19:22
                         black_flag = 1;
                         break;
                      }
               }
               if(black_flag&&abs(center-W/2)<20&&i_stop>10&&i_stop<45)
               {  
                   count = 0;
                   for(i=i_stop;i<min(i_stop+20,H-1);i++)
                   {
                       left = border[left_KB_begin][0] + (border[left_KB_stop][0]-border[left_KB_begin][0])*(i-left_KB_begin)/(left_KB_stop-left_KB_begin);
                       right = border[right_KB_begin][1] + (border[right_KB_stop][1]-border[right_KB_begin][1])*(i-right_KB_begin)/(right_KB_stop-right_KB_begin);
                       center = (left+right)/2;
                       left1 = get_left(i,center);
                       right1 = get_right(i,center);
                       if(right1>=130 && left1<=30)
                       {
                          count++;
                       }
                   }
                   if(count>5)
                   {
                       for(i=i_stop-1;i>-1;i--)
                       {
                          left = border[left_KB_begin][0] + (border[left_KB_stop][0]-border[left_KB_begin][0])*(i-left_KB_begin)/(left_KB_stop-left_KB_begin);
                          right = border[right_KB_begin][1] + (border[right_KB_stop][1]-border[right_KB_begin][1])*(i-right_KB_begin)/(right_KB_stop-right_KB_begin);
                          center = (left+right)/2;
                          if(center<2||center>W-2||image[i][center]) break;
                          if(image[i][center]==0)
                          { 
                                for(j=center;j<W-1&&image[i][j]==0;j++){}
                                 black_right= j-1;
                                for(j=center;j>0&&image[i][j]==0;j--){}
                                 black_left = j+1;
                                for(j=black_right+1;j<W-1&&image[i][j];j++){}
                                 white_right= j-1;
                                for(j=black_left-1;j>0&&image[i][j];j--){}
                                 white_left = j+1;
                                if(black_right-black_left>9&&(black_left-white_left>10)&&(white_right-black_right>10))
                                {
                                   white_black_white_count++;
                                }
                          }
                       }
                       if(white_black_white_count>=5) ring_flag = 1;
                   }
               }
         }
      }
       black_flag = 0;
       white_black_white_count = 0;
     //环形弯远端判断 
     //左边连续相同重新扫或右边连续相同的重新扫描边界
      left_count =0;
      for(i=H-1; i>=effctline+1;i--)
      {
            count = 0;
            for(j= i-1; i>effctline&&border[j][0]==border[i][0];j--){count++;}
            if(count>left_count)
            {
                left_count = count;
                left_begin = i;
                left_over = max(j+1,0);
            }
      }
        //右边界连续相同重新扫描
      right_count= 0;
      for(i=H-1;i>=effctline+1;i--)
      {
            count = 0;
            for(j= i-1; i>effctline&&border[j][1]==border[i][1];j--){count++;}
            if(count>right_count)
            {
                right_count = count;
                right_begin = i;
                right_over = max(j+1,0);
            }
      }
      //环形弯远端判断
      if(ring_flag==0)
      {
            if( effctline<20 && left_count>5 && right_count>5 && border[left_begin][0]>2&&border[right_begin][1]<W-3 && abs(left_over-effctline)<10&&abs(right_over-effctline)<10&&right_begin<=H-15&&left_begin<=H-15&&abs(left_count-right_count)<15)
            {
               left_KB_stop = get_left_row_max(H-1,left_begin);
               if(left_KB_stop>=left_begin+12)
                 return;
               if(left_KB_stop+10>=H-1)
               {  
                 return;
               }
               left_KB_begin = min(left_KB_stop+10,H-1);
               right_KB_stop = get_right_row_min(H-1,right_begin);
               if(right_KB_stop>=right_begin+12)
                 return;
               if(right_KB_stop+10>=H-1)
               {
                  return;
               }
               right_KB_begin = min(right_KB_stop+10,H-1);//
               
               count = 0;
               for(i=left_KB_begin;i>=left_KB_stop;i--)
               {
                  if(border[i][0]<2)
                    count++;
               }
               if(count>3)
                 return;
               count = 0;
               for(i=right_KB_begin;i>=right_KB_stop;i--)
               {
                  if(border[i][1]>W-3)
                    count++;
               }
               if(count>3)
                 return;
               if(border[left_KB_stop][0]-border[left_KB_begin][0]<0)
                 return;
               if(border[right_KB_stop][1]-border[right_KB_begin][1]>0)
                 return;
          // left_KB = linear_fittint(left_KB_begin,left_KB_stop,0);
          // right_KB = linear_fittint(right_KB_begin,right_KB_stop,1);
               i_begin = max(left_begin,right_begin); 
               i_begin = min(H-1,i_begin);
               i_begin = max(i_begin,0);
               for(i= i_begin;i>-1;i--)
               {
                      left = border[left_KB_begin][0] + (border[left_KB_stop][0]-border[left_KB_begin][0])*(i-left_KB_begin)/(left_KB_stop-left_KB_begin);
                      right = border[right_KB_begin][1] + (border[right_KB_stop][1]-border[right_KB_begin][1])*(i-right_KB_begin)/(right_KB_stop-right_KB_begin);
                      center = (left+right)/2;
                      if(center<2||center>W-2||left<2||left>W-2||right<2||right>W-2)
                      {
                         break;
                      }
                      if(image[i][center]==0)
                      {  
                         i_stop = i+1; //2017.7.17  19:22
                         black_flag = 1;
                         break;
                      }
               }
           }
           if(black_flag)
           {  
               for(i=i_stop-1;i>-1;i--)
               {
                  left = border[left_KB_begin][0] + (border[left_KB_stop][0]-border[left_KB_begin][0])*(i-left_KB_begin)/(left_KB_stop-left_KB_begin);
                  right = border[right_KB_begin][1] + (border[right_KB_stop][1]-border[right_KB_begin][1])*(i-right_KB_begin)/(right_KB_stop-right_KB_begin);
                  center = (left+right)/2;
                  if(center<2||center>W-2||image[i][center]) break;
                  if(image[i][center]==0)
                  { 
                        for(j=center;j<W-1&&image[i][j]==0;j++){}
                         black_right= j-1;
                        for(j=center;j>0&&image[i][j]==0;j--){}
                         black_left = j+1;
                        for(j=black_right+1;j<W-1&&image[i][j];j++){}
                         white_right= j-1;
                        for(j=black_left-1;j>0&&image[i][j];j--){}
                         white_left = j+1;
                        if(black_right-black_left>9&&(black_left-white_left>10)&&(white_right-black_right>10))
                        {
                           white_black_white_count++;
                        }
                  }
               }
               if(white_black_white_count>=5) ring_flag = 1;
           }
       }
}

/**********************************************
环形弯处理
**********************************************/
void deal_ring_border(void)
{
    int i,j;
    int left_begin,left_over;
    int right_begin,right_over;
    int count,count_leftmax,count_rightmax;
    uint8 bordertemp[H][2];
    
    
    for(i= H-1; i>= effctline; i--)
    {  
        bordertemp[i][0] = border[i][0];
        bordertemp[i][1] = border[i][1];
    }
//2017.7.21
    //弯道增加补给量
    if(H-effctline>10)
    {
        count_leftmax = 0;
        int flag  = 0;
        for(i= effctline; i<= (effctline+15); i++)
        {
            
            count = 0;
            for(j= i+1; i<H && abs(bordertemp[j][0]-bordertemp[i][0])<2&&bordertemp[i][0]<=W/2-30;j++)
            {
              count++;
              if(get_left(i,bordertemp[i][0])<3)
                flag =1;
              
            }
            if(count>count_leftmax&&flag)
            {
                left_begin = i;
                left_over = min(left_begin+count,H-1);
                count_leftmax = count;
            }
        }
        count_rightmax = 0;
        flag = 0;
        for(i= effctline; i<= (effctline+15); i++)
        {
            count = 0;
            for(j= i+1; i<H && abs(bordertemp[j][1]- bordertemp[i][1])<2&&bordertemp[i][1]>=W/2+30;j++)
            {
              count++;
              if(get_right(i,bordertemp[i][1])>W-3)
                flag =1;
            }
            if(count>count_rightmax&&flag)
            {
                right_begin =i;
                right_over = min(right_begin+count,H-1);
                count_rightmax = count;   
            }
        }
        if(count_leftmax>count_rightmax&&count_leftmax>5&&count_rightmax<15)
        {
            count_rightmax = 0;
            if(left_over<H-20)
            {
                for(i=left_over;i>=left_begin;i--)
                {
                    if(get_right(i,bordertemp[i][1])>W-10)
                      count_rightmax++;
                }
            }
            if(count_rightmax>=count_leftmax/2||count_rightmax>=10)
              return;
            for(i=left_over;i>=left_begin;i--)
            {
               if(get_right(i,bordertemp[i][1])-border[i][1]>10)
                 return;
            }
            for(i=left_over;i>=left_begin;i--)
            {
                if(get_left(i,centerline[i])<2)
                  break;
                centerline[i] = (get_left(i,centerline[i])+get_right(i,centerline[i]))/2;
            }
            effctline = left_begin;
            for(;i>=left_begin;i--)
            {
 
               if(get_left(i,centerline[i])<=5)
                 centerline[i] = max(get_right(i,centerline[i])-(width_array[i]/2+15),1);
               else
               {
                  effctline = min(i+1,H-1);
                  break;
               }
              if(centerline[i]<=1)
              { 
                    effctline = min(i+1,H-1);
                    break;
              }
            }      
        }
        if(count_rightmax>count_leftmax&&count_rightmax>5&&count_leftmax<15)
        {   
            count_leftmax = 0;
            if(right_over<H-20)
            {
                for(i=right_over;i>=right_begin;i--)
                {
                    if(get_left(i,bordertemp[i][0])<10)
                      count_leftmax++;
                }
            }
            if(count_leftmax>=count_rightmax/2||count_leftmax>=10)
              return;
            for(i=right_over;i>=right_begin;i--)
            {
               if(border[i][0]-get_left(i,bordertemp[i][0])>10)
                 return;
            }
            effctline = right_begin;
            for(i=right_over;i>=right_begin;i--)
            {
                if(get_right(i,centerline[i])>W-3)
                  break;
                 centerline[i] = (get_left(i,centerline[i])+get_right(i,centerline[i]))/2; 
            }
            for(;i>=right_begin;i--)
            {
                if(get_right(i,centerline[i])>=W-6)
                {
                    centerline[i] = min(get_left(i,centerline[i])+(width_array[i]/2+15),W-2); 
                }
                else
                {
                    effctline = min(i+1,H-1);  
                    break;
                }
                if(centerline[i]>=W-1)
                { 
                    effctline = min(i+1,H-1);
                    break;
                }
            }
        }
    }
}



/*****************************************************
第二次图像处理
*****************************************************/
void  second_deal_border(void)
{
    int i,ii,j;
    int i_begin, i_stop;
    int left_begin,left_over;
    int right_begin,right_over;
    int offest;
    uint8 X[2],Y[2];
    int count,count_leftmax,count_rightmax;
    uint8 bordertemp[H][2];
    int break_flag = 0;
    int break_point_stop;
    int break_point_begin;
    if(H - effctline>10)
    {
        count_leftmax =0;
        for(i= H-1; i>=effctline+1; i--)
        {
           left_begin =i;
           count =0;
           for(ii= left_begin-1; ii>effctline-1; ii--)
           {
              if(border[ii][0]!=border[left_begin][0]) break;
              count++;
           }
           if(count>count_leftmax)
           {
              count_leftmax = count;
              i_begin = left_begin;
              i_stop = i_begin - count;
           }
        }
        if(count_leftmax>10)
        {
            left_begin = i_begin;
            left_over = i_stop;
        }
        //右边沿检测
        count_rightmax = 0;
        for(i = H-1;i>effctline+1;i--)
        {
             right_begin = i;
             count = 0;
             for(ii =right_begin-1; ii>effctline-1;ii--)
             {
                
                if(border[ii][1] != border[right_begin][1]) break;
                count++;
             }
             if(count>count_rightmax)
             {
               count_rightmax = count;
               i_begin = right_begin;
               i_stop = i_begin - count;
             }
         }
         if(count_rightmax>10)
         {
            
              right_begin = i_begin;
              right_over = i_stop;
         } 
         if((count_leftmax)>10&&(count_leftmax>count_rightmax)&&(abs(left_over-effctline)<5)&&(border[left_over][0]<3))
         { 
                  effctline= left_over+1;
                  i = left_over+1;
                  for(j= border[i][0]+1;j<border[i][1]&&image[i][j];j++){}
                  border[i][1] =j-1;
                  i = left_over+2;
                  break_flag = 0;
                  for(;i < left_begin; i++)
                  {
                      for(j= border[i][0]+1;j<border[i][1]&&image[i][j];j++){}
                      border[i][1] =j-1;
                      if(border[i][1]-border[i-1][1]>10)
                     {
                        break_flag =1;
                        break_point_stop = i-1;
                        break;
                     }
                  }
                  if(break_flag)
                  { 
                     break_point_begin = get_right_row_min(left_begin,break_point_stop+1);
                     mend_line(break_point_begin,border[break_point_begin][1],break_point_stop,border[break_point_stop][1],1);
                     getcenter();
                     return;
                  }
        }
        if((count_rightmax)>10&&(count_rightmax>count_leftmax)&&(abs(right_over-effctline)<5)&&(border[right_over][1]>=W-3))
        {
              effctline = right_over + 1;
              i = right_over+1;
              for(j= border[i][1]-1;j>border[i][0]&&image[i][j];j--){}
              border[i][0]=j+1;
              i = right_over+2;
              break_flag =0;
              for(; i < right_begin; i++)
              {
                for(j= border[i][1]-1;j>border[i][0]&&image[i][j];j--){}
                border[i][0] =j+1;
                if(border[i-1][0]-border[i][0]>10)
                {
                    break_flag =1;
                    break_point_stop = i-1;
                    break;
                }
              }
              if(break_flag)
              {
                 break_point_begin = get_left_row_max(right_begin,break_point_stop+1);
                 mend_line(break_point_begin,border[break_point_begin][0],break_point_stop,border[break_point_stop][0],0);
                 getcenter();
                 return;
              }
              
        }
    }
    
    for(i= H-1; i>= effctline; i--)
    {  
        bordertemp[i][0] = border[i][0];
        bordertemp[i][1] = border[i][1];
    }
    if(H-effctline>10)
    {
        
        for(i= H-1; i> effctline+1; i--)
        {
            offest = 7;
            left_begin = i;
            count = 0;
            for(ii= left_begin-1; ii> effctline-1; ii-- )
            {
                if(border[ii][0] != border[left_begin][0]) break;
                count++;
            }
            if(count>offest)
            {
                //判断是否需要补线
                left_over = max(left_begin - count,effctline);
                if((abs(border[left_over][0]-border[max(left_over-3,effctline)][0])>=7)&&centerline[left_over]>=40&&centerline[left_over]<=120)
                {
                   if(left_begin<H-5)
                    {
                        //X[0] = min(left_begin+6,H-1);
                        X[0] = min(left_begin+20,H-1);
                        X[0] = get_left_row_max(X[0],left_begin);
                        X[1] = max(effctline,left_over-9);
                        Y[0] = border[X[0]][0];       Y[1] = border[X[1]][0];
                        mend_line(X[0], Y[0], X[1], Y[1], 0);
                    }
                    else
                    {
                        X[0] = max(left_over-5,effctline);          X[1] = max(left_over-10,effctline);
                        Y[0] = border[X[0]][0];      Y[1] = border[X[1]][0];
                        if(X[1]!=X[0])
                        {
                            for(i= H-1; i>=X[0];i--)
                            {
                                border[i][0] = max((i-X[0])*(Y[1]-Y[0])/(X[1]-X[0]) + border[X[0]][0],1);
                            }
                        }
                    }
                }
              
            }
        }
        for(i= H-1; i> effctline-1; i--)
        {
            offest =7;
            right_begin = i;
            count = 0;
            for(ii = right_begin-1; ii> effctline-1; ii--)
            {
                if(border[ii][1] != border[right_begin][1]) break;
                count++;
            }
            if(count>offest)
            {
                right_over = max(right_begin - count,effctline);
                if((abs(border[right_over][1] - border[max(right_over-3,effctline)][1])>=7)&&centerline[right_over]>=40&&centerline[right_over]<=120)
                {   
                    if(i<H-5)
                    {
                        //X[0] = min(right_begin+6,H-1);
                        X[0] = min(right_begin+20,H-1);
                        X[0] = get_right_row_min(X[0],right_begin);
                        X[1] = max(effctline,right_over-9);
                        Y[0] = border[X[0]][1];   
                        Y[1] = border[X[1]][1];
                        mend_line(X[0],Y[0],X[1],Y[1],1);
                    }
                    else
                    {
                        X[0] = max(right_over-5,effctline);          X[1] = max(right_over-10,effctline);
                        
                        Y[0] = border[X[0]][1];      Y[1] = border[X[1]][1];
                        if(X[0]!=X[1])
                        for(i= H-1; i>=X[0];i--)
                        {
                            border[i][1] = min((i-X[0])*(Y[1]-Y[0])/(X[1]-X[0]) + border[X[0]][1],W-2);
                        }
                    }
                }
            }
        }
    }
    getcenter();
    // 出十字补线
    if(H-effctline>10)
    {  
        int temp;
        for(i= effctline;i<effctline+10 ;i++)
        {
            i_begin = i;
            count =0 ;
            for(j=i_begin+1;i<H&&centerline[j]==centerline[i_begin]&&i_begin<(H+effctline)/2;j++){count++;}
            if(count>=7)
            {
                
                i_stop = min(i_begin+count,H-1);
                if(centerline[i_begin]>100)
                {
                    temp = max(i_begin-5,effctline);
                    X[0] = min(i_stop+8,H-1); X[1] = temp;
                    Y[0] = centerline[X[0]]; Y[1] = centerline[X[1]];
                    if(border[X[1]][0]-border[i_begin][0]<5)
                    {
                        Y[1]= border[i_begin][1];
                    }
                    mend_line(X[0],Y[0],X[1],Y[1],2);
                    
                }
                else if(centerline[i_begin]<60)
                {
                    temp = max(i_begin-5,effctline);
                    X[0] = min(i_stop+8,H-1); X[1] = temp;
                    Y[0] = centerline[X[0]]; Y[1] = centerline[X[1]];
                     if(border[i_begin][1]-border[X[1]][1]<5)
                    {
                        Y[1]= border[i_begin][0];
                    }
                    mend_line(X[0],Y[0],X[1],Y[1],2);
                }
             
            }
            
        }
    }
//2017.7.21
       //弯道增加补给量
    if(H-effctline>15)
    {
        count_leftmax = 0;
        int flag  = 0;
        for(i= effctline; i<= (effctline+15); i++)
        {
            count = 0;
            for(j= i+1; i<H && abs(bordertemp[j][0]-bordertemp[i][0])<2&&bordertemp[i][0]<=W/2-30;j++)
            {
              count++;
              if(get_left(i,bordertemp[i][0])<3)
                flag =1;
            }
            if(count>count_leftmax&&flag)
            {
                left_begin = i;
                left_over = min(left_begin+count,H-1);
                count_leftmax = count;
            }
        }
        count_rightmax = 0;
        flag = 0;
        for(i= effctline; i<= (effctline+15); i++)
        {
            count = 0;
            for(j= i+1; i<H && abs(bordertemp[j][1]- bordertemp[i][1])<2&&bordertemp[i][1]>=W/2+30;j++)
            {
              count++;
              if(get_right(i,bordertemp[i][1])>W-3)
                flag =1;
            }
            if(count>count_rightmax&&flag)
            {
                right_begin =i;
                right_over = min(right_begin+count,H-1);
                count_rightmax = count;   
            }
        }
        if(count_leftmax>count_rightmax&&count_leftmax>5&&count_rightmax<15)
        {
            count_rightmax = 0;
            if(left_over<H-20)
            {
                for(i=left_over;i>=left_begin;i--)
                {
                    if(get_right(i,bordertemp[i][1])>W-10)
                      count_rightmax++;
                }
            }
            if(count_rightmax>=count_leftmax/2||count_rightmax>=10)
              return;
            for(i=left_over;i>=left_begin;i--)
            {
               if(get_right(i,bordertemp[i][1])-border[i][1]>10)
                 return;
            }
            for(i=left_over;i>=left_begin;i--)
            {
                if(get_left(i,centerline[i])<2)
                  break;
                centerline[i] = (get_left(i,centerline[i])+get_right(i,centerline[i]))/2;
            }
            effctline = left_begin;
            for(;i>=left_begin;i--)
            {
 
               if(get_left(i,centerline[i])<=5)
                 //2017.8.12 修改
                 //centerline[i] = max(get_right(i,centerline[i])-(width_array[i]/2+15),1);
                 centerline[i] = max(get_right(i,centerline[i])-(width_array[i]/2+15),1);
               else
               {
                  effctline = min(i+1,H-1);
                  break;
               }
              if(centerline[i]<=1)
              { 
                    effctline = min(i+1,H-1);
                    break;
              }
            }      
        }
        if(count_rightmax>count_leftmax&&count_rightmax>5&&count_leftmax<15)
        {   
            count_leftmax = 0;
            if(right_over<H-20)
            {
                for(i=right_over;i>=right_begin;i--)
                {
                    if(get_left(i,bordertemp[i][0])<10)
                      count_leftmax++;
                }
            }
            if(count_leftmax>=count_rightmax/2||count_leftmax>=10)
              return;
            for(i=right_over;i>=right_begin;i--)
            {
               if(border[i][0]-get_left(i,bordertemp[i][0])>10)
                 return;
            }
            effctline = right_begin;
            for(i=right_over;i>=right_begin;i--)
            {
                if(get_right(i,centerline[i])>W-3)
                  break;
                 centerline[i] = (get_left(i,centerline[i])+get_right(i,centerline[i]))/2; 
            }
            for(;i>=right_begin;i--)
            {
                if(get_right(i,centerline[i])>=W-6)
                {
                    //2017.8.12 
                    //author:王鹏
                    //内容：边界偏移量减小
                    //centerline[i] = min(get_left(i,centerline[i])+(width_array[i]/2+15),W-2);
                    centerline[i] = min(get_left(i,centerline[i])+(width_array[i]/2+10),W-2);
                  
                }
                else
                {
                    effctline = min(i+1,H-1);  
                    break;
                }
                if(centerline[i]>=W-1)
                { 
                    effctline = min(i+1,H-1);
                    break;
                }
            }
        }
    }
}

/********************************
障碍物检测
************************************/
void obstacle(void)
{
#define   block_deal_near    52     //8
#define   block_deal_mid     34     //6
#define   block_deal_far     16      //3
  
#define   block_predeal_line_er_far 22
#define   block_predeal_line_er_far1 11
   int i;
   int i_begin,i_stop;
   int left1,right1;
   int left2,right2;
   int flag = 0;//障碍物起始标志
   int flag1 = 0;
   int direction = 0;
   if(effctline<10)
   {
      for(i= H-1; i>effctline+1;i--)
      {
          left1 = get_left(i,centerline[i]);
          right1 = get_right(i,centerline[i]);
          left2 = get_left(i-1,centerline[i-1]);
          right2 = get_right(i-1,centerline[i-1]);
          
          if(image[i][centerline[i]]==0)
             goto a;
          if((right1-left1>width_array[i]*3/2)||(right1-left1)>150)
            goto a;
          if(left1<border[i][0]-5&&right1>border[i][0]+5)
            goto a;
          if((right2-left2)<=(right1-left1)-10)
          {
             flag = 1;
             i_begin = i-1;
             if(centerline[i]-left2>right2-centerline[i])
               direction = 0;//障碍物在右侧，车从赛道左侧穿过
             else
               direction = 1;//障碍物在左侧，车从赛道右侧穿过
             break;
          }
      }
      if(flag)
      {
          if(direction==0)
          {
               for(i=i_begin;i>effctline+2;i--)
                {   
                    if(get_right(i-2,centerline[i-1]) > get_right(i,centerline[i])+8)
                    {
                      flag1 = 1;
                      i_stop = i;
                      break;
                     
                    }
                }
            
          }
          else
          {
               for(i=i_begin;i>effctline+2;i--)
               {     
                  if(get_left(i-2,centerline[i-1])<get_left(i,centerline[i])-8)
                  {
                     flag1 =1;
                     i_stop = i;
                     flag1 = 1;
                     break;
                  }
              }
          }
      }
      if(flag1&&(i_begin-i_stop)>=6&&(i_begin-i_stop)<36)
      {
          obstacle_flag = 1;  
      }
      if(obstacle_flag)
      {
           for(i=i_begin;i>=i_stop;i--)
           {
              centerline[i] = (get_left(i,centerline[i])+ get_right(i,centerline[i]))/2;
           }
           if(i_begin<=block_deal_mid)
           {
              if(direction==1)
              {
                  for(i=i_begin+block_predeal_line_er_far;i>=i_begin+block_predeal_line_er_far1;i--)
                  {
                      centerline[i]+=width_array[i]*2/32;
                      if(centerline[i]<centerline[i+1])  centerline[i] = centerline[i+1];
                  }
                  for(i=i_begin+block_predeal_line_er_far1;i>=i_begin;i--)
                  {
                      centerline[i]+=width_array[i]*3/32;
                      if(centerline[i]<centerline[i+1])  centerline[i] = centerline[i+1];
                  }
                  for(i=i_begin;i>=effctline+1;i--)
                  {
                      centerline[i]+=width_array[i]*3/16;
                  }
              }
              if(direction==0)
              {
                  for(i=i_begin+block_predeal_line_er_far;i>=i_begin+block_predeal_line_er_far1;i--)
                  {
                      centerline[i]-=width_array[i]*2/32;
                      if(centerline[i]>centerline[i+1])  centerline[i] = centerline[i+1];
                  }
                  for(i=i_begin+block_predeal_line_er_far1;i>=i_begin;i--)
                  {
                      centerline[i]-=width_array[i]*3/32;
                      if(centerline[i]>centerline[i+1])  centerline[i] = centerline[i+1];
                  }
                  for(i=i_begin;i>=effctline+1;i--)
                  {
                      centerline[i]-=width_array[i]*3/16;
                  }
              }
              
          }
          else
          {
              if(direction==1)
              {
                  for(i=H-1;i>=effctline+1;i--)
                    centerline[i]+=width_array[i]*3/16;
              }
              if(direction==0)
              {
                  for(i=H-1;i>=effctline+1;i--)
                    centerline[i]-=width_array[i]*3/16;
              }
          }
          centerline_filter(i_begin);
      }
        
   }
  a: return;
}

/***********************************
停车检测函数
************************************/
int stopcar(void)
{
     //停车标志位检测
    int i,j;
    int count1= 0;
    int count2= 0;
    int black_flag=0;
    int flag = 1;
    stop_line = 0;
    if(effctline<10)
    {
           count1= 0;
           for(i=H-1;i>effctline+1;i--)
           {
                count1=0;
                for(j=border[i][0]+2;j<border[i][1]-2;j++)
                {
                   if(image[i][j]!= black_flag)
                     continue;
                   else
                   {
                      if(black_flag) black_flag =0;
                      else black_flag = 255;
                      count1++;
                   }    
                   
                }
                if(count1>5)
                {
                  if(flag)
                    stop_line = i;
                  count2++;
                }
           }
           if(count2>5)
           {
                if(runtime>2000&&start_flag2)
                {
                    stop = 1;
                }
               // if(stop_line>=50)
                  return 1;
             
           } 
    }
    return 0;
     
}

/***********************************
确定白色块的起始位置
************************************/
uint8* white_block(uint8 row,uint8 col,uint8 image[][W],int deriction)
{
    
    int j;
    int count =0;
    temp[0] = col;//确定白色块的起始位置
    uint8 flag=0;
    if(deriction== 0)
    {
        for(j= col; j<W-1; j++)
        {
            if(image[row][j]==0)//black_region
            {
                break;
            }
            else
            {  
                if(image[row+1][j]==1)
                {
                    count++;//确定白色块是否属于连通区域
                }
            }
        }
        temp[1] = j-1;//确定白色块的终止位置
        if(count>5)flag = 1;
        for(j=temp[0]; j<=temp[1]; j++) image[row][j]=flag;//写标记
        temp[2]= flag;
    }
    else if(deriction==1)
    {
        for(j= col; j>0;j--)
        {
            if(image[row][j]==0)
            {
              break;
            }
            else
            {
                if(image[row+1][j]==1)
                {
                    count++;
                }
            }
        }
        temp[1] = j+1;
        if(count>5) flag=1;
        for(j=temp[0]; j>=temp[1]; j--) image[row][j]=flag;
        temp[2] = flag;
    }
    return temp;
}
/*******************************************
确定白色区域块是否属于连通区域
*******************************************/

void graph_region_label(uint8 image[][W],uint8 last_center)//目标区域赋值为1，背景赋值为0
{
   int i,j;
   int right_begin,right_over;
   int left_begin,left_over;
   int breakpoint= H-1;
   uint8 *temp;
   //先扫描最后一行，标定最后一行的目标区域
   uint8 flag = 0;//终止行的标志位
   uint8 flag1 = 0;//分叉点的标志位
   uint8 X[2],Y[2];
   
   
   i= H-1;
   for(j= last_center; j< W-1; j++)
   {
      if(image[i][j]!=0) image[i][j]=1;
      else
      { image[i][j]=0;
        break;
      }
   }
   border[i][1] = j-1;
   for(;j<W;j++) image[i][j]=0;

   for(j= last_center; j>0;j--)
   {
      if(image[i][j]!=0) image[i][j]=1;
      else
      {
          image[i][j]=0;
          break;
      }
   }
   border[i][0] = j+1;
   for(; j>-1; j--) image[i][j]=0;
   
   //确定倒数第二行到第0行的目标区域
    //环形弯左拐
   if((ring_flag_count==0 && ring_direction1==1)||(ring_flag_count==1&&ring_direction2==1)||(ring_flag_count==2 && ring_direction2==1))
   {
       for(i= H-2; i> -1; i--)
       {
         //初始化变量
          border[i][0] = 1;
          flag = 0;
          for(j=1;j<border[i+1][1];j++)
          {
              if(image[i][j]==0)
              {
                continue;
              }
              else
              {
                  temp = white_block(i,j,image,0);
                  if(flag==0&&temp[2])
                  {
                      border[i][0]=temp[0];
                      border[i][1]=temp[1];
                      flag=1;
                  }
                  /********************************
                  2017.8.6 之前的间断点判断
                  *********************************/
                  else if(flag1==0&& temp[2])
                  {
                      breakpoint = i;
                      flag1 = 1;
                  }
                  /********************************/
                  j = temp[1]+1;
                  if(border[i][1]-border[i][0]<20)
                  {
                      effctline = i+1;
                      goto left_ring;
                  }
              }
          }
          if(flag==0)
          {
            effctline= i+1;
            break;
          }
       }
left_ring:
       if(ring_count>25)
       {
            if(effctline<=15)
            {
                leftdispersion = dispersion_degree(H-1,15,0);
                rightdispersion = dispersion_degree(H-1,15,1);
                if(leftdispersion<=10&&rightdispersion<=10)
                {
                   ring_flag = 0;
                   ring_count = 0;
                }
            }
       }
       if(flag1 && (abs(effctline-breakpoint)<2))
       {
          flag1 = 0;
       }
       if(flag1)
       {
            effctline = get_right_row_min(breakpoint,effctline); 
       }
       ring_left_begin = get_left_row_max(H-1,effctline);
       ring_left_over = max(ring_left_begin-15,effctline);
       leftdispersion = dispersion_degree(ring_left_begin,ring_left_over,0);
       getcenter();
       if(flag1)
       {
            right_begin = get_right_row_min(H-1,breakpoint+5);
            right_over = breakpoint;
            X[0] = right_begin; Y[0] = border[X[0]][1];
            X[1] = right_over;  Y[1] = border[X[1]][1];
            mend_line(X[0],Y[0],X[1],Y[1],1); 
            getcenter();
       }
       else
       {
          deal_ring_border();
       }
       
    }
    if((ring_flag_count==0&&ring_direction1==0) || (ring_flag_count==1&& ring_direction2==0) || (ring_flag_count==2&&ring_direction2==0)||ring_flag_count>=3)
    {//环形弯右拐
       for(i=H-2; i>-1; i--)
       {
         //初始化变量
          border[i][1] = W-2;
          flag = 0;
          for(j=W-2;j>border[i+1][0];j--)
          {
              if(image[i][j]==0)
              {
                 continue;
              }
              else
              {
                  temp = white_block(i,j,image,1);
                  if(flag==0&&temp[2])
                  {
                      border[i][1]=temp[0];
                      border[i][0]=temp[1];
                      flag=1;
                  }
                  /********************************
                  2017.8.6 之前的间断点判断
                  *********************************/
                  else if(flag1==0&& temp[2])
                  {
                      breakpoint = i;
                      flag1 = 1;
                  }
                  /****************************/
                  j = temp[1]-1;
              }
              if(border[i][1]-border[i][0]<20)
              {
                  effctline = i+1;
                  goto right_ring;
              }
          }
          if(flag==0)
          {
            effctline= i+1;
            break;
          }
       }
right_ring:
       if(ring_count>25)
       {
            if(effctline<=15)
            {
                leftdispersion = dispersion_degree(H-1,15,0);
                rightdispersion = dispersion_degree(H-1,15,1);
                if(leftdispersion<=10&&rightdispersion<=10)
                {
                   ring_flag = 0;
                   ring_count = 0;
                }
            }
       }
       if(flag1)
       {
            effctline = get_left_row_max(breakpoint,effctline);
       }
       ring_right_begin = get_right_row_min(H-1,effctline);
       ring_right_over =  max(ring_right_begin-15,effctline);
       rightdispersion = dispersion_degree(ring_right_begin,ring_right_over,1);
       getcenter();
       if(flag1)
       {
            left_begin = get_left_row_max(H-1,breakpoint+5);
            left_over = breakpoint;
            X[0] = left_begin; Y[0] = border[X[0]][0];
            X[1] = left_over;  Y[1] = border[X[1]][0];
            mend_line(X[0],Y[0],X[1],Y[1],0); 
            getcenter();
       }
       else
       {
           deal_ring_border();
       }
    }
}
/******************************************************************
*函数名称：zero_point_get
*函数功能：穿越加权线的零点个数
*备注：
********************************************************************/
int zero_point_get(int start,int end,uint8 *a)
{
    int count=0;
    int zero_point_count=0;
    int center=W/2;
    for(count=start;count<=end;count++)
      center+=a[count];
    center=(int)(center/(end-start+1));
    for(count=start;count<end;count++)
    {
        if(((a[count]-center)*(a[count+1]-center)<=0)&&(a[count+1]!=center))
          zero_point_count++;
    }
    return zero_point_count;
}
/******************************************************************
*函数名称：dispersion_degree
*函数功能：求一系列点的离散程度
*备注：
********************************************************************/
int dispersion_degree(int begin,int end,int mode)
{
    int i=0;
    int disp=0;
    if(begin==end)
      return 0;
    if(mode==0)
    {
//        for(i=end;i<H;i++)
//          ave+=border[i][0];
//        ave=ave/(H-end);
//        for(i=end;i<H;i++)
//          disp += (border[i][0]-ave)*(border[i][0]-ave);
      
          for(i=end;i<=begin;i++)
          {
             disp += (border[i][0]-border[begin][0]-(border[begin][0]-border[end][0])*(i-begin)/(begin-end))*(border[i][0]-border[begin][0]-(border[begin][0]-border[end][0])*(i-begin)/(begin-end));
          }
    }
    else if(mode==1)
    {
//        for(i=end;i<H;i++)
//          ave+=border[i][1];
//          ave=ave/(H-end);
//        for(i=end;i<H;i++)
//          disp += (border[i][1]-ave)*(border[i][1]-ave);
           for(i=end;i<=begin;i++)
          {
             disp += (border[i][1]-border[begin][1]-(border[begin][1]-border[end][1])*(i-begin)/(begin-end))*(border[i][1]-border[begin][1]-(border[begin][1]-border[end][1])*(i-begin)/(begin-end));
          }
    }
    else
    {
//        for(i=end;i<H;i++)
//          ave+=centerline[i];
//          ave=ave/(H-end);
//        for(i=end;i<H;i++)
//          disp += (centerline[i]-ave)*(centerline[i]-ave);
          for(i=end;i<=begin;i++)
          {
             disp += (centerline[i]-centerline[begin]-(centerline[begin]-centerline[end])*(i-begin)/(begin-end))*(centerline[i]-centerline[begin]-(centerline[begin]-centerline[end])*(i-begin)/(begin-end));
          }
    }
    return (int)(disp/(begin-end+1));
}
int get_left(int row,int center)
{
     int i;
     for(i= center;i>0;i--)
     {
        if(image[row][i]==0)
          break;
     }
     i = max(i+1,1);
     return i;
}
int get_right(int row ,int center)
{
     int i;
     for(i= center; i<W-1;i++)
     {
        if(image[row][i]==0)
          break;
     }
     if(i>=W-1)
       i = W-2;
     else
       i = i-1;
     return i;
}
int get_left_max(int end)
{
    int left_max=0;
    int count=H-1;
    for(;count>=end;count--)
    {
        if(border[count][0]>left_max)
          left_max=border[count][0];
    }
    
    return left_max;
}

int get_right_min(int end)
{
    int right_min=W-1;
    int count=H-1;
    for(;count>=end;count--)
    {
        if(border[count][1]<right_min)
          right_min=border[count][1];
    }
    
    return right_min;
}
int get_left_row_max(int start,int end)
{
    int left_max=0;
    int row;
    int count=start;
    for(;count>=end;count--)
    {
        if(border[count][0]>left_max)
        {
          left_max=border[count][0];
          row = count;
        }
    }
    return row;
    
}
int get_right_row_min(int start,int end)
{
    int right_min=W-1;
    int count=start;
    int row;
    for(;count>=end;count--)
    {
        if(border[count][1]<right_min)
        {
          right_min=border[count][1];
          row = count;
        }
    }
    return row;
}
line_tampeta linear_fittint(int start,int end,int mode)
{
    int i;
    int num;
    float a;
    float k,b;
    float sum_X = 0.0;
    float sum_Y = 0.0;
    float sum_XY = 0.0;
    float sum_XX = 0.0;
    line_tampeta kb;
    num = abs(start-end) +1;
    if(mode == 0)
    {
        for( i= min(start,end); i<= max(start,end);i++)
        {
            sum_X += i;
            sum_Y += border[i][0];
            sum_XX += i*i;
            sum_XY += i*border[i][0];
        }
    }
    else if(mode ==1)
    {
        for( i= min(start,end); i<= max(start,end);i++)
        {
            sum_X += i;
            sum_Y += border[i][1];
            sum_XX += i*i;
            sum_XY += i*border[i][1];
        }
    }
    else
    {
        for( i= min(start,end); i<=max(start,end);i++)
        {
            sum_X += i;
            sum_Y += centerline[i];
            sum_XX += i*i;
            sum_XY += i*centerline[i];
        }
    }
    a = sum_X/num;
    k = (sum_XY -a*sum_Y)/(sum_XX-a*sum_X);
    b = (sum_Y -sum_X*k)/num;
    kb.x = k;
    kb.y = b;
//    line_tampeta[0] = k;
//    line_tampeta[1] = b;
    return kb;

}
//int lookfor_peak(float k,int start,int end,int mode);
//{
//   int i,j;
//   
//}
//补线
void mend_line(uint8 X1,uint8 Y1,uint8 X2,uint8 Y2,uint8 flag)
{
    int i;
    float k  ;
    int y;
    if(X1==X2)
      return;
    k = (float)(Y2-Y1)/(X2-X1);
    if(flag==0)
    {
        for(i= X1; i>= X2; i--)
        {
            y = (int)(Y1 + k*(i - X1));
            if(y<1) y = 1;
            border[i][0] = y;
        }
    }
    else if(flag==1)
    {
        for(i= X1; i>= X2; i--)
        {
            y = (int)(Y1 + k*(i - X1));
            if(y>W-2) y = W-2;
            border[i][1] = y;
        }
    }
    else if(flag==2)
    {
        for(i= X1; i>= X2; i--)
        {
            y = (int)(Y1 + k*(i - X1));
            if(y>W-2) y = W-2;
            centerline[i] = y;
        }
    }
}
//计算得到中线
void getcenter(void)
{
    int i;
    
    for(i=H-1; i>=effctline;i--) 
      centerline[i] = (border[i][0]+border[i][1])/2;
}
/**************************************************
中线滤波
**************************************************/
void centerline_filter(int end_line)
{ 
#define centerline_error_limit   (8)
#define center_deal_point     (3)
  int count = 0,deal_count = 0;
  int error = 0;
  for(count = H-1;count > end_line;count--)
  {
    if(abs(centerline[count] - centerline[count - 1]) > centerline_error_limit)
    {
      
      error = centerline[count] - centerline[count - 1];
      //近处中线偏移
      if(count > H - center_deal_point)
      {
        for(deal_count = count;deal_count < H;deal_count++)
          centerline[deal_count] -= error/3;
      }
      else
      {
        for(deal_count = count;deal_count < count+center_deal_point;deal_count++)
          centerline[deal_count] -= error/3;
      }
      //远处中线偏移
      if(count-1 < effctline + center_deal_point)
      {
        for(deal_count = count-1;deal_count >= effctline;deal_count--)
          centerline[deal_count] += error/3;
      }
      else
      {
        for(deal_count = count-1;deal_count > count-1-center_deal_point;deal_count--)
          centerline[deal_count] += error/3;
      }
    }
  }
}
//有效行过滤函数
void effctline_filter(void)
{
   int count = H- 1;
   for(count=H-1; count>=effctline+1;count--)
   {
      if(centerline[count]<5 && count <= 45)
      {
            effctline = count;
            break;
      }
      if(centerline[count]>W-6 && count <= 45)
      {
           effctline = count;
           break;
      }
      if(image[count-1][centerline[count]]== 0 )
      {
         effctline = count;
         break;
      }
   }
}
//显示边界
void show_border(uint8 image[][W])
{
    int i,j;

    for(i=0;i<H;i++)
    {
        for(j=0;j<W;j++)
        {       if(i>= effctline)
               {  
                  if(j==border[i][0]||j==border[i][1]||j== centerline[i]) image[i][j]=0;//边界赋值为0即为black
                  //else image[i][j]=1;
               }
               //else image[i][j]=1;
        }
                
    }
}
//压缩图像 eight pixels is a character
void  img_reduce(uint8 *aim,uint8 *get,uint32 length)
{
  while(length>0)
  {
    *get = 0;
    *get += ((*aim++)&0x01) << 7;
    *get += ((*aim++)&0x01) << 6;
    *get += ((*aim++)&0x01) << 5;
    *get += ((*aim++)&0x01) << 4;
    *get += ((*aim++)&0x01) << 3;
    *get += ((*aim++)&0x01) << 2;
    *get += ((*aim++)&0x01) << 1;
    *get += ((*aim++)&0x01) << 0;
   // *get =~ *get;
    get++;
    length -= 8;
  }
  
}
void image_copy(void)
{
  int copy_count = 0;
  for(copy_count = 0;copy_count < H;copy_count++)
  {
    img_extract((uint8* )&image[copy_count][0],&imgbuff[((28 + 3*copy_count)*W)/8],W/8);
    img_extract((uint8* )&image2[copy_count][0],&imgbuff[((28 + 3*copy_count)*W)/8],W/8);
  }
}
