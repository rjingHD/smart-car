#include "include.h"


#ifndef PROJECT_H
#define PROJECT_H
extern int mode1;
extern int mode2;
extern int mode3;
extern int mode4;
extern int ring_direction1;
extern int ring_direction2;


#define max(a,b)((a)>(b)?(a):(b))
#define min(a,b)((a)>(b)?(b):(a))

void PORTA_IRQHandler();
void DMA0_IRQHandler();
#endif