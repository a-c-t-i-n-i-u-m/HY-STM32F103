	.module mmc.c
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\mmc.c
	.dbfunc e MMCCS _MMCCS fV
;             cs -> R16
	.even
_MMCCS::
	.dbline -1
	.dbline 21
; /******************************************
; * ��������: MMC(SD)����д����
; * ������: MMC(SD)���ӿڲ���
; * Ŀ��Ӳ��: AVR MCU "ATMEGA162" 16.0000MHz
; * ��������: 2007-12
; * ԭ������: XuGuoHong 
; *           kk20y@yahoo.com.cn
; * �޸ļ�¼: ��
; ******************************************/
; 
; /* INCLUDE����  */
; #include <iom162v.h>
; 
; /* ȫ�ֱ��� */
; extern unsigned char sector[512];
; 
; /************************************
; *         MMC��Ƭѡ-1ѡ��/0��ѡ��
; ************************************/
; void MMCCS(unsigned char cs)
; {
	.dbline 22
;    if(cs==0)
	tst R16
	brne L2
	.dbline 23
;        PORTB|=(1<<PB4);           /* SS=1 */
	sbi 0x18,4
	xjmp L3
L2:
	.dbline 25
	cbi 0x18,4
L3:
	.dbline -2
L1:
	.dbline 0 ; func end
	ret
	.dbsym r cs 16 c
	.dbend
	.dbfunc e MMCWrCmd _MMCWrCmd fc
;              k -> R20
;              i -> R22
;           temp -> R10
;            cmd -> R12,R13
	.even
_MMCWrCmd::
	xcall push_gset4
	movw R12,R16
	.dbline -1
	.dbline 32
;    else
;        PORTB&=~(1<<PB4);          /* SS=0 */
; }
; 
; /*******************************
; *        MMC�����
; *******************************/
; unsigned char MMCWrCmd(unsigned char *cmd)
; {
	.dbline 33
;     unsigned char i=0,k=0;
	clr R22
	.dbline 33
	clr R20
	.dbline 34
; 	unsigned char temp=0XFF;
	ldi R24,255
	mov R10,R24
	.dbline 35
;     MMCCS(0);		   			/* Ƭѡ��Ч */
	clr R16
	xcall _MMCCS
	.dbline 36
; 	spi_send(0XFF);				/* ����8��ʱ�� */
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	.dbline 37
; 	MMCCS(1);		   			/* Ƭѡ��Ч */
	ldi R16,1
	xcall _MMCCS
	.dbline 38
; 	asm("nop");
	nop
	.dbline 39
; 	for(i=0; i<6; i++)
	xjmp L8
L5:
	.dbline 40
	.dbline 41
	movw R30,R12
	ld R16,Z+
	movw R12,R30
	clr R17
	xcall _spi_send
	.dbline 42
L6:
	.dbline 39
	inc R22
L8:
	.dbline 39
	cpi R22,6
	brlo L5
	xjmp L10
L9:
	.dbline 44
; 	{
; 	    spi_send(*(cmd++));		/* �������� */   
; 	}
; 	while(temp==0XFF)
; 	{
	.dbline 45
; 	    temp = spi_send(0XFF);  /* �ȴ��ظ� */
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	mov R10,R16
	.dbline 46
; 		if(k++>200)             /* ��ʱ���� */
	mov R2,R20
	clr R3
	subi R20,255    ; addi 1
	ldi R24,200
	cp R24,R2
	brsh L12
	.dbline 47
; 		{
	.dbline 48
; 		    return temp;
	xjmp L4
L12:
	.dbline 50
L10:
	.dbline 43
	mov R24,R10
	cpi R24,255
	breq L9
	.dbline 51
; 		}
; 	}
;     return temp;
	mov R16,R24
	.dbline -2
L4:
	xcall pop_gset4
	.dbline 0 ; func end
	ret
	.dbsym r k 20 c
	.dbsym r i 22 c
	.dbsym r temp 10 c
	.dbsym r cmd 12 pc
	.dbend
	.area lit(rom, con, rel)
L15:
	.byte 64,0
	.byte 0,0
	.byte 0,149
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\mmc.c
	.dbfunc e MMCInit _MMCInit fc
;           temp -> R22
;        timeout -> R20,R21
;            cmd -> y+0
;              i -> R20
	.even
_MMCInit::
	xcall push_gset2
	sbiw R28,6
	.dbline -1
	.dbline 59
; }
; 
; 
; /*******************************
; *        MMC��ʼ��
; *******************************/
; unsigned char MMCInit(void)
; {
	.dbline 61
;     unsigned int timeout;
; 	unsigned char i=0,temp=0;
	clr R20
	.dbline 61
	clr R22
	.dbline 62
;     unsigned char cmd[]={0x40,0x00,0x00,0x00,0x00,0x95};  
	ldi R24,<L15
	ldi R25,>L15
	movw R30,R28
	ldi R16,6
	ldi R17,0
	st -y,R31
	st -y,R30
	st -y,R25
	st -y,R24
	xcall asgncblk
	.dbline 63
; 	DelayMs(500);
	ldi R16,500
	ldi R17,1
	xcall _DelayMs
	.dbline 65
; 	/* ����һ��������ʱ������ */
;     for(i=0;i<0x10;i++) 
	xjmp L19
L16:
	.dbline 66
	.dbline 67
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	.dbline 68
L17:
	.dbline 65
	inc R20
L19:
	.dbline 65
	cpi R20,16
	brlo L16
	.dbline 70
;     {
;         spi_send(0XFF);
;     }
; 	/* ����CMD0 */
; 	if(MMCWrCmd(cmd)!=0X01)
	movw R16,R28
	xcall _MMCWrCmd
	cpi R16,1
	breq L20
	.dbline 71
; 	   return 0;
	clr R16
	xjmp L14
L20:
	.dbline 73
; 	/* ����CMD1 */
; 	cmd[0]=0X41;
	ldi R24,65
	std y+0,R24
	.dbline 74
; 	cmd[5]=0XFF;
	ldi R24,255
	std y+5,R24
	xjmp L24
L23:
	.dbline 76
; 	while(MMCWrCmd(cmd)!=0X00)
; 	{
	.dbline 77
; 	    if(timeout++>0XFFFE)     /* �ȴ���ʼ����� */
	movw R2,R20
	subi R20,255  ; offset = 1
	sbci R21,255
	ldi R24,65534
	ldi R25,255
	cp R24,R2
	cpc R25,R3
	brsh L26
	.dbline 78
; 		    return 0;            /* �������MMC����Ҫ�ñȽϳ�ʱ�� */
	clr R16
	xjmp L14
L26:
	.dbline 79
L24:
	.dbline 75
	movw R16,R28
	xcall _MMCWrCmd
	tst R16
	brne L23
	.dbline 80
; 	}
; 	SPIHiSPD();	   				 /*  ���MCU SPI�ٶ� */
	xcall _SPIHiSPD
	.dbline 81
;     return 1;
	ldi R16,1
	.dbline -2
L14:
	adiw R28,6
	xcall pop_gset2
	.dbline 0 ; func end
	ret
	.dbsym r temp 22 c
	.dbsym r timeout 20 i
	.dbsym l cmd 0 A[6:6]c
	.dbsym r i 20 c
	.dbend
	.area lit(rom, con, rel)
L29:
	.byte 64,0
	.byte 0,0
	.byte 0,255
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\mmc.c
	.dbfunc e MMCCID _MMCCID fc
;            cmd -> y+0
;              i -> R20
	.even
_MMCCID::
	xcall push_gset1
	sbiw R28,6
	.dbline -1
	.dbline 88
; }
; 
; /*******************************
; *        ��ȡMMC-CID�Ĵ���
; *******************************/
; unsigned char MMCCID(void)
; {
	.dbline 90
;      unsigned char i;
;      unsigned char cmd[]={0x40,0x00,0x00,0x00,0x00,0xff};  
	ldi R24,<L29
	ldi R25,>L29
	movw R30,R28
	ldi R16,6
	ldi R17,0
	st -y,R31
	st -y,R30
	st -y,R25
	st -y,R24
	xcall asgncblk
	.dbline 91
; 	 cmd[0]=0X40+10;
	ldi R24,74
	std y+0,R24
	.dbline 92
; 	 if(MMCWrCmd(cmd)!=0X00)     /* ����CMD9 */
	movw R16,R28
	xcall _MMCWrCmd
	tst R16
	breq L30
	.dbline 93
; 	   return 0;                 /* ��ȡʧ�� */
	clr R16
	xjmp L28
L30:
	.dbline 94
; 	 for(i=0;i<16;i++)
	clr R20
	xjmp L35
L32:
	.dbline 96
	ldi R16,255
	ldi R17,0
	xcall _spi_send
L33:
	.dbline 94
	inc R20
L35:
	.dbline 94
	cpi R20,16
	brlo L32
	.dbline 97
; 	    //uart1_send(spi_send(0XFF));
; 		spi_send(0XFF);
; 	 return 1;
	ldi R16,1
	.dbline -2
L28:
	adiw R28,6
	xcall pop_gset1
	.dbline 0 ; func end
	ret
	.dbsym l cmd 0 A[6:6]c
	.dbsym r i 20 c
	.dbend
	.area lit(rom, con, rel)
L37:
	.byte 64,0
	.byte 0,0
	.byte 0,255
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\mmc.c
	.dbfunc e MMCRdBolck1 _MMCRdBolck1 fc
;           temp -> <dead>
;            cmd -> y+0
;              i -> R20,R21
;        address -> y+8
	.even
_MMCRdBolck1::
	xcall push_arg4
	xcall push_gset1
	sbiw R28,6
	.dbline -1
	.dbline 105
; }
; 
; /*******************************
; *        ��ȡONE BLOCK����
; *         address-������ַ
; *******************************/
; unsigned char MMCRdBolck1(unsigned long address)
; {
	.dbline 108
;      unsigned int i;
; 	 unsigned char temp;
;      unsigned char cmd[]={0x40,0x00,0x00,0x00,0x00,0xff};  
	ldi R24,<L37
	ldi R25,>L37
	movw R30,R28
	ldi R16,6
	ldi R17,0
	st -y,R31
	st -y,R30
	st -y,R25
	st -y,R24
	xcall asgncblk
	.dbline 109
;      cmd[0]=0X40+17;			 /* READ SINGLE BLOCK */
	ldi R24,81
	std y+0,R24
	.dbline 110
; 	 address=address<<9;         /* address*512,ȡ512�������� */
	ldi R24,9
	ldi R25,0
	movw R30,R28
	ldd R2,z+8
	ldd R3,z+9
	ldd R4,z+10
	ldd R5,z+11
	st -y,R24
	movw R16,R2
	movw R18,R4
	xcall lsl32
	movw R30,R28
	std z+8,R16
	std z+9,R17
	std z+10,R18
	std z+11,R19
	.dbline 111
; 	 cmd[1]=(address>>24);
	ldi R24,24
	ldi R25,0
	movw R30,R28
	ldd R2,z+8
	ldd R3,z+9
	ldd R4,z+10
	ldd R5,z+11
	st -y,R24
	movw R16,R2
	movw R18,R4
	xcall lsr32
	std y+1,R16
	.dbline 112
; 	 cmd[2]=(address>>16);
	movw R30,R28
	ldd R2,z+8
	ldd R3,z+9
	ldd R4,z+10
	ldd R5,z+11
	movw R2,R4
	clr R4
	clr R5
	std y+2,R2
	.dbline 113
; 	 cmd[3]=(address>>8);
	ldi R24,8
	ldi R25,0
	movw R30,R28
	ldd R2,z+8
	ldd R3,z+9
	ldd R4,z+10
	ldd R5,z+11
	st -y,R24
	movw R16,R2
	movw R18,R4
	xcall lsr32
	std y+3,R16
	.dbline 114
; 	 cmd[4]=(address>>0);
	movw R30,R28
	ldd R2,z+8
	ldd R3,z+9
	ldd R4,z+10
	ldd R5,z+11
	std y+4,R2
	.dbline 115
; 	 if(MMCWrCmd(cmd)!=0X00)     /* ����CMD17 */
	movw R16,R28
	xcall _MMCWrCmd
	tst R16
	breq L45
	.dbline 116
; 	     return 0;               /* ��ȡʧ�� */
	clr R16
	xjmp L36
L44:
	.dbline 118
	.dbline 119
	nop
	.dbline 120
L45:
	.dbline 117
;      while(spi_send(0XFF)!=0XFE)
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	cpi R16,254
	ldi R30,0
	cpc R17,R30
	brne L44
	.dbline 121
; 	 {
; 	     asm("nop");			 /* �ȴ����ݽ��ܿ�ʼ���ܵ�0XFE��ʾ��ʼ */
; 	 }
; 	 for(i=0;i<512;i++)          /* ��ȡ���� */
	clr R20
	clr R21
	xjmp L50
L47:
	.dbline 122
	.dbline 123
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	ldi R24,<_sector
	ldi R25,>_sector
	movw R30,R20
	add R30,R24
	adc R31,R25
	std z+0,R16
	.dbline 124
L48:
	.dbline 121
	subi R20,255  ; offset = 1
	sbci R21,255
L50:
	.dbline 121
	cpi R20,0
	ldi R30,2
	cpc R21,R30
	brlo L47
	.dbline 125
; 	 {
; 	     sector[i]=spi_send(0XFF);
; 	 }
; 	 spi_send(0XFF);			 /* ȡ��CRC�ֽ� */
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	.dbline 126
; 	 spi_send(0XFF);
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	.dbline 127
; 	 return 1; 
	ldi R16,1
	.dbline -2
L36:
	adiw R28,6
	xcall pop_gset1
	adiw R28,4
	.dbline 0 ; func end
	ret
	.dbsym l temp 1 c
	.dbsym l cmd 0 A[6:6]c
	.dbsym r i 20 i
	.dbsym l address 8 l
	.dbend
	.area lit(rom, con, rel)
L52:
	.byte 64,0
	.byte 0,0
	.byte 0,255
	.area text(rom, con, rel)
	.dbfile E:\SOUCER~1\M162MMC\mmc.c
	.dbfunc e MMCWrBlock1 _MMCWrBlock1 fc
;           temp -> R22
;            cmd -> y+0
;              i -> R20,R21
;         buffer -> y+14
;        address -> y+10
	.even
_MMCWrBlock1::
	xcall push_arg4
	xcall push_gset2
	sbiw R28,6
	.dbline -1
	.dbline 135
; }
; 	
; /*******************************
; *        дONE BLOCK����
; *        address-������ַ
; *******************************/
; unsigned char MMCWrBlock1(unsigned long address,unsigned char *buffer)
; {
	.dbline 138
;      unsigned int i;
; 	 unsigned char temp;
;      unsigned char cmd[]={0x40,0x00,0x00,0x00,0x00,0xff};  
	ldi R24,<L52
	ldi R25,>L52
	movw R30,R28
	ldi R16,6
	ldi R17,0
	st -y,R31
	st -y,R30
	st -y,R25
	st -y,R24
	xcall asgncblk
	.dbline 139
; 	 cmd[0]=0x40+24;              /* WRITE SINGLE BLOCK */
	ldi R24,88
	std y+0,R24
	.dbline 140
; 	 address=address<<9;          /* address*512,ȡ512�������� */
	ldi R24,9
	ldi R25,0
	movw R30,R28
	ldd R2,z+10
	ldd R3,z+11
	ldd R4,z+12
	ldd R5,z+13
	st -y,R24
	movw R16,R2
	movw R18,R4
	xcall lsl32
	movw R30,R28
	std z+10,R16
	std z+11,R17
	std z+12,R18
	std z+13,R19
	.dbline 141
; 	 cmd[1]=(address>>24);
	ldi R24,24
	ldi R25,0
	movw R30,R28
	ldd R2,z+10
	ldd R3,z+11
	ldd R4,z+12
	ldd R5,z+13
	st -y,R24
	movw R16,R2
	movw R18,R4
	xcall lsr32
	std y+1,R16
	.dbline 142
; 	 cmd[2]=(address>>16);
	movw R30,R28
	ldd R2,z+10
	ldd R3,z+11
	ldd R4,z+12
	ldd R5,z+13
	movw R2,R4
	clr R4
	clr R5
	std y+2,R2
	.dbline 143
; 	 cmd[3]=(address>>8);
	ldi R24,8
	ldi R25,0
	movw R30,R28
	ldd R2,z+10
	ldd R3,z+11
	ldd R4,z+12
	ldd R5,z+13
	st -y,R24
	movw R16,R2
	movw R18,R4
	xcall lsr32
	std y+3,R16
	.dbline 144
; 	 cmd[4]=(address>>0);
	movw R30,R28
	ldd R2,z+10
	ldd R3,z+11
	ldd R4,z+12
	ldd R5,z+13
	std y+4,R2
	.dbline 145
; 	 if(MMCWrCmd(cmd)!=0X00)     /* ����CMD24 */
	movw R16,R28
	xcall _MMCWrCmd
	tst R16
	breq L57
	.dbline 146
; 	     return 0;               /* д��ʧ�� */
	clr R16
	xjmp L51
L57:
	.dbline 147
; 	 spi_send(0XFF);             /* ��������ֽ� */
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	.dbline 148
; 	 spi_send(0XFE);             /* �������ݿ�ʼ��־0XFE */
	ldi R16,254
	ldi R17,0
	xcall _spi_send
	.dbline 149
; 	 for(i=0;i<512;i++)          /* д������ */
	clr R20
	clr R21
	xjmp L62
L59:
	.dbline 150
	.dbline 151
	ldi R24,<_sector
	ldi R25,>_sector
	movw R30,R20
	add R30,R24
	adc R31,R25
	ldd R16,z+0
	clr R17
	xcall _spi_send
	.dbline 152
L60:
	.dbline 149
	subi R20,255  ; offset = 1
	sbci R21,255
L62:
	.dbline 149
	cpi R20,0
	ldi R30,2
	cpc R21,R30
	brlo L59
	.dbline 153
; 	 {
; 	     spi_send(sector[i]);
; 	 }
; 	 spi_send(0XFF);			 /* д��CRC�ֽ� */
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	.dbline 154
; 	 spi_send(0XFF);
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	.dbline 155
; 	 temp=spi_send(0XFF);		 /* ��ȡXXX0 0101�ֽ� */
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	mov R22,R16
	.dbline 156
; 	 temp=temp&0X1F;
	andi R22,31
	.dbline 157
; 	 if(temp!=0X05)
	cpi R22,5
	breq L66
	.dbline 158
; 	     return 0; 				 /* д��ʧ�� */
	clr R16
	xjmp L51
L65:
	.dbline 160
	.dbline 161
	nop
	.dbline 162
L66:
	.dbline 159
; 	 while(spi_send(0XFF)==0X00)
	ldi R16,255
	ldi R17,0
	xcall _spi_send
	cpi R16,0
	cpc R16,R17
	breq L65
X0:
	.dbline 163
; 	 {
; 	     asm("nop");			 /* BUSY�ȴ� */
; 	 }
; 	 return 1;
	ldi R16,1
	.dbline -2
L51:
	adiw R28,6
	xcall pop_gset2
	adiw R28,4
	.dbline 0 ; func end
	ret
	.dbsym r temp 22 c
	.dbsym l cmd 0 A[6:6]c
	.dbsym r i 20 i
	.dbsym l buffer 14 pc
	.dbsym l address 10 l
	.dbend
