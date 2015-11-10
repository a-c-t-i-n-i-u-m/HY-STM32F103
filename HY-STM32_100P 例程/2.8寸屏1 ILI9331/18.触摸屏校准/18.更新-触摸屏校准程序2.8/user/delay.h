#ifndef __DELAY_H
#define __DELAY_H 			   
//使用SysTick的普通计数模式对延迟进行管理
//包括delay_us,delay_ms  
//正点原子@SCUT
//2008/12/14
//V1.2
//修正了中断中调用出现死循环的错误
//防止延时不准确,采用do while结构!
	 
static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数
//初始化延迟函数

#endif
