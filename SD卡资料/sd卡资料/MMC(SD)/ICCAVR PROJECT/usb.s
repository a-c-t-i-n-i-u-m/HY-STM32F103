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
; // �� �� ԭ �� ˵ ��
; void uart1_init(void);
; void uart1_send(unsigned char);
; unsigned char uart1_rev(void);
; 
; //����ͨ�ų�ʼ������
; //1. �������趨
; //2. ֡�ṹ�趨 
; //3. ���պͷ����������
; 
; //******************************************
; //*����: uart1_init
; //*����: ����1��ʼ��
; //******************************************
; void uart1_init(void)
; {
	.dbline 21
;  // FoscƵ��Ϊ16.0000Mhz
;   // �������趨Ϊ19200bps +/-0.2%
;     UBRR1H=0x00;                                 
	clr R2
	out 0x3c,R2
	.dbline 22
;     UBRR1L=0x33;   
	ldi R24,51
	out 0x0,R24
	.dbline 24
;  // 8λ����+����żУ��+1λSTOP                                                             
;     UCSR1C=(1<<URSEL1)|(1<<UCSZ11)|(1<<UCSZ10);
	ldi R24,134
	out 0x3c,R24
	.dbline 26
;  // �����շ�,������ɺ��ж�
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
; //*����: uart1_send()
; //*����: ����1��������
; //******************************************
; void uart1_send(unsigned char data)
; {
L3:
	.dbline 37
L4:
	.dbline 36
;  // ����Ƿ���Է���,UDRE=1�Ĵ���Ϊ��
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
; //*����: uart1_rev()
; //*����: ����0��������
; //******************************************
; unsigned char uart1_rev(void)
; {
L7:
	.dbline 49
L8:
	.dbline 48
;  // ����Ƿ�������
;     while ( !(UCSR1A & (1<<RXC1)) )
	sbis 0x2,7
	rjmp L7
	.dbline 51
;           ;
;  // ���ؽ�������
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
; //*����: uart1_intrev()
; //*����: ����1�жϷ�ʽ��������
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
