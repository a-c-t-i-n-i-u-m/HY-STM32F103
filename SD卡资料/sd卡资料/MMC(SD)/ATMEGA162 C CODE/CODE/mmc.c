/******************************************
* 程序名称: MMC(SD)卡读写程序
* 程序功能: MMC(SD)卡接口操作
* 目标硬件: AVR MCU "ATMEGA162" 16.0000MHz
* 创建日期: 2007-12
* 原创作者: XuGuoHong 
*           kk20y@yahoo.com.cn
* 修改记录: 无
******************************************/

/* INCLUDE参数  */
#include <iom162v.h>

/* 全局变量 */
extern unsigned char sector[512];

/************************************
*         MMC卡片选-1选中/0不选中
************************************/
void MMCCS(unsigned char cs)
{
   if(cs==0)
       PORTB|=(1<<PB4);           /* SS=1 */
   else
       PORTB&=~(1<<PB4);          /* SS=0 */
}

/*******************************
*        MMC命令发送
*******************************/
unsigned char MMCWrCmd(unsigned char *cmd)
{
    unsigned char i=0,k=0;
	unsigned char temp=0XFF;
    MMCCS(0);		   			/* 片选无效 */
	spi_send(0XFF);				/* 发送8个时钟 */
	MMCCS(1);		   			/* 片选有效 */
	asm("nop");
	for(i=0; i<6; i++)
	{
	    spi_send(*(cmd++));		/* 发送命令 */   
	}
	while(temp==0XFF)
	{
	    temp = spi_send(0XFF);  /* 等待回复 */
		if(k++>200)             /* 超时返回 */
		{
		    return temp;
		}
	}
    return temp;
}


/*******************************
*        MMC初始化
*******************************/
unsigned char MMCInit(void)
{
    unsigned int timeout;
	unsigned char i=0,temp=0;
    unsigned char cmd[]={0x40,0x00,0x00,0x00,0x00,0x95};  
	DelayMs(500);
	/* 发送一定数量的时钟脉冲 */
    for(i=0;i<0x10;i++) 
    {
        spi_send(0XFF);
    }
	/* 发送CMD0 */
	if(MMCWrCmd(cmd)!=0X01)
	   return 0;
	/* 发送CMD1 */
	cmd[0]=0X41;
	cmd[5]=0XFF;
	while(MMCWrCmd(cmd)!=0X00)
	{
	    if(timeout++>0XFFFE)     /* 等待初始化完成 */
		    return 0;            /* 容量大的MMC卡需要用比较长时间 */
	}
	SPIHiSPD();	   				 /*  提高MCU SPI速度 */
    return 1;
}

/*******************************
*        读取MMC-CID寄存器
*******************************/
unsigned char MMCCID(void)
{
     unsigned char i;
     unsigned char cmd[]={0x40,0x00,0x00,0x00,0x00,0xff};  
	 cmd[0]=0X40+10;
	 if(MMCWrCmd(cmd)!=0X00)     /* 发送CMD9 */
	   return 0;                 /* 读取失败 */
	 for(i=0;i<16;i++)
	    //uart1_send(spi_send(0XFF));
		spi_send(0XFF);
	 return 1;
}

/*******************************
*        读取ONE BLOCK数据
*         address-扇区地址
*******************************/
unsigned char MMCRdBolck1(unsigned long address)
{
     unsigned int i;
	 unsigned char temp;
     unsigned char cmd[]={0x40,0x00,0x00,0x00,0x00,0xff};  
     cmd[0]=0X40+17;			 /* READ SINGLE BLOCK */
	 address=address<<9;         /* address*512,取512的整数倍 */
	 cmd[1]=(address>>24);
	 cmd[2]=(address>>16);
	 cmd[3]=(address>>8);
	 cmd[4]=(address>>0);
	 if(MMCWrCmd(cmd)!=0X00)     /* 发送CMD17 */
	     return 0;               /* 读取失败 */
     while(spi_send(0XFF)!=0XFE)
	 {
	     asm("nop");			 /* 等待数据接受开始，受到0XFE表示开始 */
	 }
	 for(i=0;i<512;i++)          /* 读取数据 */
	 {
	     sector[i]=spi_send(0XFF);
	 }
	 spi_send(0XFF);			 /* 取走CRC字节 */
	 spi_send(0XFF);
	 return 1; 
}
	
/*******************************
*        写ONE BLOCK数据
*        address-扇区地址
*******************************/
unsigned char MMCWrBlock1(unsigned long address,unsigned char *buffer)
{
     unsigned int i;
	 unsigned char temp;
     unsigned char cmd[]={0x40,0x00,0x00,0x00,0x00,0xff};  
	 cmd[0]=0x40+24;              /* WRITE SINGLE BLOCK */
	 address=address<<9;          /* address*512,取512的整数倍 */
	 cmd[1]=(address>>24);
	 cmd[2]=(address>>16);
	 cmd[3]=(address>>8);
	 cmd[4]=(address>>0);
	 if(MMCWrCmd(cmd)!=0X00)     /* 发送CMD24 */
	     return 0;               /* 写入失败 */
	 spi_send(0XFF);             /* 发送填冲字节 */
	 spi_send(0XFE);             /* 发送数据开始标志0XFE */
	 for(i=0;i<512;i++)          /* 写入数据 */
	 {
	     spi_send(sector[i]);
	 }
	 spi_send(0XFF);			 /* 写入CRC字节 */
	 spi_send(0XFF);
	 temp=spi_send(0XFF);		 /* 读取XXX0 0101字节 */
	 temp=temp&0X1F;
	 if(temp!=0X05)
	     return 0; 				 /* 写入失败 */
	 while(spi_send(0XFF)==0X00)
	 {
	     asm("nop");			 /* BUSY等待 */
	 }
	 return 1;
}
	

