#ifndef __SYS_H
#define __SYS_H	    		 
//系统时钟初始化
//使用外部8M晶振,PLL到72M频率		    
//正点原子@SCUT
//2008/12/14 
//V1.2
//不能在这里执行所有外设复位!否则至少引起串口不工作.

#define uint unsigned int
#define uchar unsigned char	   

//把所有时钟寄存器复位
void MYRCC_DeInit(void)
{						   
	RCC->APB1RSTR = 0x00000000;//复位结束			 
	RCC->APB2RSTR = 0x00000000; 
	  
  	RCC->AHBENR = 0x00000014;  //flash时钟,闪存时钟使能.DMA时钟关闭	  
  	RCC->APB2ENR = 0x00000000; //外设时钟关闭.			   
  	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //使能内部高速时钟HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //复位HSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //复位HSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //关闭所有中断
}

//外部8M,则得到72M的系统时钟	
void Stm32_Clock_Init(void)
{
	unsigned char temp=0;   
	MYRCC_DeInit();
	RCC->CR|=0x00010000;  //外部高速时钟使能HSEON
	while(!(RCC->CR>>17));//等待外部时钟就绪
	RCC->CFGR=0X1C0400;   //APB1/2=DIV2;AHB=DIV1;PLL=9*CLK;
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2个延时周期

	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//等待PLL锁定
	RCC->CFGR|=0x00000002;//PLL作为系统时钟	 
	while(temp!=0x02)     //等待PLL作为系统时钟设置成功
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}
#endif
