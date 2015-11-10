/******************************************
* 程序名称: MMC(SD)卡读写程序
* 程序功能: 主函数
* 目标硬件: AVR MCU "ATMEGA162" 16.0000MHz
* 创建日期: 2007-12
* 原创作者: XuGuoHong 
*           kk20y@yahoo.com.cn
* 修改记录: 无
******************************************/

/* INCLUDE参数  */
#include <iom162v.h>

/* 全局变量 */
unsigned char sector[512];

/*******************************
*          主 函 数
*******************************/
void main(void)
{
	unsigned int i;
    SPL = 0XFF;    				/* 堆栈初始化 */
    SPH = 0X04;
    uart1_init();
    spi_init();
	DelayMs(100);
	MMCInit();
    //asm("sei");             /* 系统总中断开 */

	
	// 测试1:将数据写入第255个扇区
	for(i=0; i<512; i++)
	   sector[i]=0X88;
	MMCWrBlock1(255);
	
	// 测试2:将第1个扇区的数据读出
	MMCRdBolck1(0);
    for(i=0; i<512; i++)
	    uart1_send(sector[i]);
		
	while(1)
	{
	   asm("nop");
	}
}

/******************************************
* 名称:  DelayMs
* 描述:  软件延时函数,单位ms
******************************************/
void DelayMs(unsigned int time)
{
    unsigned int temp;
	unsigned int count;
	count = 2663;
	while(count--)
	{
	   for (temp=0; temp<time; temp++);
	   {
           asm("nop");	
	   }
	}
}