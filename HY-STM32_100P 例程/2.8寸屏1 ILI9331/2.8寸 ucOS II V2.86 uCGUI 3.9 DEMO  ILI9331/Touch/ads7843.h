#ifndef __ADS7843_H
#define __ADS7843_H

// A/D 通道选择命令字和工作寄存器
// A/D 通道选择命令字和工作寄存器
#define	CHX 	0x90 	//通道X+的选择控制字	
#define	CHY 	0xd0	//通道Y+的选择控制字 

#define	uint32          unsigned long		 
#define	uint8           unsigned char

#define TP_DCLK_H() GPIO_SetBits(GPIOD, GPIO_Pin_10)

#define TP_DCLK_L() GPIO_ResetBits(GPIOD, GPIO_Pin_10)

#define TP_CS_H()  GPIO_SetBits(GPIOB, GPIO_Pin_10)

#define TP_CS_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_10)

#define TP_DIN_H() GPIO_SetBits(GPIOD, GPIO_Pin_9)

#define TP_DIN_L() GPIO_ResetBits(GPIOD, GPIO_Pin_9)

#define TP_DOUT		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15)//( IOPIN1 & MASK_DOUT )	//数据输入
//#define TP_BUSY		PD9in  //GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)//( IOPIN1 & MASK_DOUT )	//数据输入

void TP_Init(void);
u16 Read_XY(u8 xy);
uint8 TP_GetAdXY(u16 *x, u16 *y);
uint8 TP_GetAdXY2(u16 *x, u16 *y, uint32 delay);
uint8 TP_GetLCDXY(u16 *x, u16 *y);
#endif

