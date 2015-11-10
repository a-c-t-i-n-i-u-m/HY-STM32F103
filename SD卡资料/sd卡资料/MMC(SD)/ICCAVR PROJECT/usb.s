	.module usb.c
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\usb.c
	.dbfunc e uart1_init _uart1_init fV
	.even
_uart1_init::
	.dbline -1
	.dbline 18
; #include<iom162v.h>
; 
; // 函 数 原 形 说 明
; void uart1_init(void);
; void uart1_send(unsigned char);
; unsigned char uart1_rev(void);
; 
; //串口通信初始化设置
; //1. 波特率设定
; //2. 帧结构设定 
; //3. 接收和发送允许控制
; 
; //******************************************
; //*名称: uart1_init
; //*功能: 串口1初始化
; //******************************************
; void uart1_init(void)
; {
	.dbline 21
;  // Fosc频率为16.0000Mhz
;   // 波特率设定为19200bps +/-0.2%
;     UBRR1H=0x00;                                 
	clr R2
	out 0x3c,R2
	.dbline 22
;     UBRR1L=0x33;   
	ldi R24,51
	out 0x0,R24
	.dbline 24
;  // 8位数据+无奇偶校验+1位STOP                                                             
;     UCSR1C=(1<<URSEL1)|(1<<UCSZ11)|(1<<UCSZ10);
	ldi R24,134
	out 0x3c,R24
	.dbline 26
;  // 允许收发,接收完成后中断
;     UCSR1B|=(1<<RXCIE1)|(1<<TXEN1)|(1<<RXEN1);
	in R24,0x1
	ori R24,152
	out 0x1,R24
	.dbline -2
L1:
	.dbline 0 ; func end
	ret
	.dbend
	.dbfunc e uart1_send _uart1_send fV
;           data -> R16
	.even
_uart1_send::
	.dbline -1
	.dbline 34
; }
; 
; //******************************************
; //*名称: uart1_send()
; //*功能: 串口1发送数据
; //******************************************
; void uart1_send(unsigned char data)
; {
L3:
	.dbline 37
L4:
	.dbline 36
;  // 检测是否可以发送,UDRE=1寄存器为空
;     while ( !( UCSR1A & (1<<UDRE1)) )
	sbis 0x2,5
	rjmp L3
	.dbline 38
;            ;
;     UDR1=data;
	out 0x3,R16
	.dbline -2
L2:
	.dbline 0 ; func end
	ret
	.dbsym r data 16 c
	.dbend
	.dbfunc e uart1_rev _uart1_rev fc
	.even
_uart1_rev::
	.dbline -1
	.dbline 46
; }
; 
; //******************************************
; //*名称: uart1_rev()
; //*功能: 串口0接收数据
; //******************************************
; unsigned char uart1_rev(void)
; {
L7:
	.dbline 49
L8:
	.dbline 48
;  // 检测是否接收完成
;     while ( !(UCSR1A & (1<<RXC1)) )
	sbis 0x2,7
	rjmp L7
	.dbline 51
;           ;
;  // 返回接收数据
;     return UDR1;                      
	in R16,0x3
	.dbline -2
L6:
	.dbline 0 ; func end
	ret
	.dbend
	.area vector(rom, abs)
	.org 80
	jmp _uart1_intrev
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\usb.c
	.dbfunc e uart1_intrev _uart1_intrev fV
	.even
_uart1_intrev::
	xcall push_lset
	.dbline -1
	.dbline 60
; }
; 
; //******************************************
; //*名称: uart1_intrev()
; //*功能: 串口1中断方式接收数据
; //******************************************
; #pragma interrupt_handler uart1_intrev:iv_USART1_RXC
; void uart1_intrev(void)
; {
	.dbline 61
;     asm("cli");
	cli
	.dbline 62
; 	uart1_send(UDR1);
	in R16,0x3
	xcall _uart1_send
	.dbline 63
; 	asm("sei");
	sei
	.dbline -2
L10:
	xcall pop_lset
	.dbline 0 ; func end
	reti
	.dbend
