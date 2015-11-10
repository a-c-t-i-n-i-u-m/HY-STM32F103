#ifndef __USART_H
#define __USART_H
#include "stdio.h"
//ע��Ҫ������:STM32F10xR.LIB,�ṩUSART_SendData()��USART_GetFlagStatus()����������
//����ԭ�� 2008/12/08
//@scut	  
#ifdef __GNUC__								 
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 
//�ض���C���Կ⺯��printf������1
PUTCHAR_PROTOTYPE
{										 
	USART_SendData(USART1,(u8)ch);//дһ���ַ�������1 
  	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//ѭ������,ֱ���������   
  	return ch;
}  

//��ʼ��IO ����1
void uart_init()
{  	  
	//��ʹ��ʱ��(�м�)
   	RCC->APB2ENR|=1<<9;	  //TX
	RCC->APB2ENR|=1<<10;  //RX   
	GPIOA->CRH=0X444444B4;//IO״̬����
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	//����������
 	USART1->BRR=0X1D4C;//9600������(72Mʱ��)
	USART1->CR1|=0X200C;//1λֹͣ,��У��λ.	
}
#endif