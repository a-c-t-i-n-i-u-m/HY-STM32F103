
#define __TOUCH_H  
//��������ʼ��
//ʹ���ⲿ8M����,PLL��72MƵ��	


#define DOUT GPIOA->IDR&1<<6 //PA6��������
			 
#define PEN  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)   //PB6  	
 
#define TDIN  (1<<7)  // PA7
#define TCLK  (1<<5)  // PA5
#define TCS   (1<<7)  // PB7   			    
#define TDIN_SET(x) GPIOA->ODR=(GPIOA->ODR&~TDIN)|(x ? TDIN:0)
#define TCLK_SET(x) GPIOA->ODR=(GPIOA->ODR&~TCLK)|(x ? TCLK:0)													    
#define TCS_SET(x)  GPIOB->ODR=(GPIOB->ODR&~TCS)|(x ? TCS:0)  
