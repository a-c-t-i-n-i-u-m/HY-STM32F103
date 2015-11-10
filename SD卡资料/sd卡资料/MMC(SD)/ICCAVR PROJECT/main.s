	.module main.c
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\main.c
	.dbfunc e main _main fV
;              i -> R20,R21
	.even
_main::
	.dbline -1
	.dbline 21
; /******************************************
; * 程序名称: MMC(SD)卡读写程序
; * 程序功能: 主函数
; * 目标硬件: AVR MCU "ATMEGA162" 16.0000MHz
; * 创建日期: 2007-12
; * 原创作者: XuGuoHong 
; *           kk20y@yahoo.com.cn
; * 修改记录: 无
; ******************************************/
; 
; /* INCLUDE参数  */
; #include <iom162v.h>
; 
; /* 全局变量 */
; unsigned char sector[512];
; 
; /*******************************
; *          主 函 数
; *******************************/
; void main(void)
; {
	.dbline 23
; 	unsigned int i;
;     SPL = 0XFF;    				/* 堆栈初始化 */
	ldi R24,255
	out 0x3d,R24
	.dbline 24
;     SPH = 0X04;
	ldi R24,4
	out 0x3e,R24
	.dbline 25
;     uart1_init();
	xcall _uart1_init
	.dbline 26
;     spi_init();
	xcall _spi_init
	.dbline 27
; 	DelayMs(100);
	ldi R16,100
	ldi R17,0
	xcall _DelayMs
	.dbline 28
; 	MMCInit();
	xcall _MMCInit
	.dbline 33
;     //asm("sei");             /* 系统总中断开 */
; 
; 	
; 	// 测试1:将数据写入第255个扇区
; 	for(i=0; i<512; i++)
	clr R20
	clr R21
	xjmp L5
L2:
	.dbline 34
	ldi R24,<_sector
	ldi R25,>_sector
	movw R30,R20
	add R30,R24
	adc R31,R25
	ldi R24,136
	std z+0,R24
L3:
	.dbline 33
	subi R20,255  ; offset = 1
	sbci R21,255
L5:
	.dbline 33
	cpi R20,0
	ldi R30,2
	cpc R21,R30
	brlo L2
	.dbline 35
; 	   sector[i]=0X88;
; 	MMCWrBlock1(255);
	ldi R16,255
	ldi R17,0
	xcall _MMCWrBlock1
	.dbline 38
; 	
; 	// 测试2:将第1个扇区的数据读出
; 	MMCRdBolck1(0);
	clr R16
	clr R17
	xcall _MMCRdBolck1
	.dbline 39
;     for(i=0; i<512; i++)
	clr R20
	clr R21
	xjmp L9
L6:
	.dbline 40
	ldi R24,<_sector
	ldi R25,>_sector
	movw R30,R20
	add R30,R24
	adc R31,R25
	ldd R16,z+0
	clr R17
	xcall _uart1_send
L7:
	.dbline 39
	subi R20,255  ; offset = 1
	sbci R21,255
L9:
	.dbline 39
	cpi R20,0
	ldi R30,2
	cpc R21,R30
	brlo L6
	xjmp L11
L10:
	.dbline 43
	.dbline 44
	nop
	.dbline 45
L11:
	.dbline 42
	xjmp L10
X0:
	.dbline -2
L1:
	.dbline 0 ; func end
	ret
	.dbsym r i 20 i
	.dbend
	.dbfunc e DelayMs _DelayMs fV
;          count -> R20,R21
;           temp -> R22,R23
;           time -> R16,R17
	.even
_DelayMs::
	xcall push_gset2
	.dbline -1
	.dbline 53
; 	    uart1_send(sector[i]);
; 		
; 	while(1)
; 	{
; 	   asm("nop");
; 	}
; }
; 
; /******************************************
; * 名称:  DelayMs
; * 描述:  软件延时函数,单位ms
; ******************************************/
; void DelayMs(unsigned int time)
; {
	.dbline 56
;     unsigned int temp;
; 	unsigned int count;
; 	count = 2663;
	ldi R20,2663
	ldi R21,10
	xjmp L15
L14:
	.dbline 58
; 	while(count--)
; 	{
	.dbline 59
	clr R22
	clr R23
	xjmp L20
L17:
	.dbline 59
L18:
	.dbline 59
	subi R22,255  ; offset = 1
	sbci R23,255
L20:
	.dbline 59
	cp R22,R16
	cpc R23,R17
	brlo L17
	.dbline 60
	.dbline 61
	nop
	.dbline 62
	.dbline 63
L15:
	.dbline 57
	movw R2,R20
	subi R20,1
	sbci R21,0
	tst R2
	brne L14
	tst R3
	brne L14
X1:
	.dbline -2
L13:
	xcall pop_gset2
	.dbline 0 ; func end
	ret
	.dbsym r count 20 i
	.dbsym r temp 22 i
	.dbsym r time 16 i
	.dbend
	.area bss(ram, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\main.c
_sector::
	.blkb 512
	.dbsym e sector _sector A[512:512]c
