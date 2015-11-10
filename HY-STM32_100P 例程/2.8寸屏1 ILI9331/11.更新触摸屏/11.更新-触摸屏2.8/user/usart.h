#ifndef __USART_H
#define __USART_H
#include "stdio.h"
//注意要包含库:STM32F10xR.LIB,提供USART_SendData()和USART_GetFlagStatus()这两个函数
//正点原子 2008/12/08
//@scut	  
#ifdef __GNUC__								 
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
//重定义C语言库函数printf到串口1
PUTCHAR_PROTOTYPE
{										 
	USART_SendData(USART1,(u8)ch);//写一个字符到串口1 
  	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//循环发送,直到发送完毕   
  	return ch;
}  

//初始化IO 串口1
void uart_init()
{  	  
	//先使能时钟(切记)
   	RCC->APB2ENR|=1<<9;	  //TX
	RCC->APB2ENR|=1<<10;  //RX   
	GPIOA->CRH=0X444444B4;//IO状态设置
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	//波特率设置
 	USART1->BRR=0X1D4C;//9600波特率(72M时钟)
	USART1->CR1|=0X200C;//1位停止,无校验位.	
}
#endif