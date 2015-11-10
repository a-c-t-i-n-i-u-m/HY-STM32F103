#ifndef __SYS_H
#define __SYS_H	    		 
//ϵͳʱ�ӳ�ʼ��
//ʹ���ⲿ8M����,PLL��72MƵ��		    
//����ԭ��@SCUT
//2008/12/14 
//V1.2
//����������ִ���������踴λ!�����������𴮿ڲ�����.

#define uint unsigned int
#define uchar unsigned char	   

//������ʱ�ӼĴ�����λ
void MYRCC_DeInit(void)
{						   
	RCC->APB1RSTR = 0x00000000;//��λ����			 
	RCC->APB2RSTR = 0x00000000; 
	  
  	RCC->AHBENR = 0x00000014;  //flashʱ��,����ʱ��ʹ��.DMAʱ�ӹر�	  
  	RCC->APB2ENR = 0x00000000; //����ʱ�ӹر�.			   
  	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //ʹ���ڲ�����ʱ��HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //��λSW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //��λHSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //��λHSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //��λPLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //�ر������ж�
}

//�ⲿ8M,��õ�72M��ϵͳʱ��	
void Stm32_Clock_Init(void)
{
	unsigned char temp=0;   
	MYRCC_DeInit();
	RCC->CR|=0x00010000;  //�ⲿ����ʱ��ʹ��HSEON
	while(!(RCC->CR>>17));//�ȴ��ⲿʱ�Ӿ���
	RCC->CFGR=0X1C0400;   //APB1/2=DIV2;AHB=DIV1;PLL=9*CLK;
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2����ʱ����

	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//�ȴ�PLL����
	RCC->CFGR|=0x00000002;//PLL��Ϊϵͳʱ��	 
	while(temp!=0x02)     //�ȴ�PLL��Ϊϵͳʱ�����óɹ�
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}
#endif
