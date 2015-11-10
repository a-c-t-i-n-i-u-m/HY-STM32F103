	.module spi.c
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\spi.c
	.dbfunc e spi_init _spi_init fV
	.even
_spi_init::
	.dbline -1
	.dbline 8
; /* INCLUDE参数  */
; #include <iom162v.h>
; 
; /************************************
; *          SPI初始化
; ************************************/
; void spi_init(void)
; {
	.dbline 9
;     DDRB|=(1<<PB4);                               /* SS端设置为输出 */
	sbi 0x17,4
	.dbline 10
;     DDRB|=(1<<PB5);                               /* MOSI端口设置为输出 */
	sbi 0x17,5
	.dbline 11
;     DDRB&=~(1<<PB6);                              /* MISO端口设置为输入 */
	cbi 0x17,6
	.dbline 12
;     DDRB|=(1<<PB7);                               /* SCK端口设置为输出 */
	sbi 0x17,7
	.dbline 13
;     SPCR|=(1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0); /* SPI ENABLE,MASTER MODE,Fsoc/16 */
	in R24,0xd
	ori R24,83
	out 0xd,R24
	.dbline 14
; 	PORTB|=(1<<PB4);                              /* SS=1 */
	sbi 0x18,4
	.dbline -2
L1:
	.dbline 0 ; func end
	ret
	.dbend
	.dbfunc e SPIHiSPD _SPIHiSPD fV
	.even
_SPIHiSPD::
	.dbline -1
	.dbline 21
; }
; 
; /************************************
; *        SPI转入高速模式
; ************************************/
; void SPIHiSPD(void)
; {
	.dbline 22
;     SPCR|=(0<<SPR1)|(0<<SPR0);                    /* SPI ENABLE,MASTER MODE,Fsoc/16 */
	in R2,0xd
	out 0xd,R2
	.dbline 23
; 	SPSR|=(1<<SPI2X); 
	sbi 0xe,0
	.dbline -2
L2:
	.dbline 0 ; func end
	ret
	.dbend
	.dbfunc e spi_send _spi_send fc
;           temp -> R20
;           data -> R16
	.even
_spi_send::
	xcall push_gset1
	.dbline -1
	.dbline 30
; }
; 
; /************************************
; *           SPI发送数据
; ************************************/
; unsigned char spi_send(unsigned char data)
; {
	.dbline 32
;     unsigned char temp;
; 	SPDR = data;
	out 0xf,R16
	xjmp L5
L4:
	.dbline 34
	.dbline 35
	nop
	.dbline 36
L5:
	.dbline 33
;     while(!(SPSR & (1<<SPIF)))
	sbis 0xe,7
	rjmp L4
	.dbline 37
; 	{
; 	    asm("nop");
; 	}
; 	temp = SPDR;
	in R20,0xf
	.dbline 38
; 	return temp;
	mov R16,R20
	.dbline -2
L3:
	xcall pop_gset1
	.dbline 0 ; func end
	ret
	.dbsym r temp 20 c
	.dbsym r data 16 c
	.dbend
