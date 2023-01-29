#ifndef __DELAY_H
#define __DELAY_H

#include "ALL_Includes.h"

#define BIT(x)	(1 << (x))

void delay_s(u16 nms);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void Delay(u32 count);
void Delay_Init(u8 SYSCLK);

extern u32 fac_s;
extern u16 fac_ms;//全局变量
extern u8 fac_us;//全局变量


#endif

