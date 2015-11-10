CC = iccavr
CFLAGS =  -IE:\SOUCER~1\M162MMC -e -DATMEGA -DATMega162  -l -g -Mavr_enhanced 
ASFLAGS = $(CFLAGS)  -Wa-g
LFLAGS =  -O -LC:\icc\lib\ -g -ucrtatmega.o -bfunc_lit:0x70.0x4000 -dram_end:0x4ff -bdata:0x100.0x4ff -dhwstk_size:16 -beeprom:1.512 -fihx_coff -S2
FILES = main.o mmc.o usb.o spi.o 

MMC:	$(FILES)
	$(CC) -o MMC $(LFLAGS) @MMC.lk   -lcatmega
main.o: E:/SOUCER~1/M162MMC/iom162v.h
main.o:	E:\SOUCER~1\M162MMC\main.c
	$(CC) -c $(CFLAGS) E:\SOUCER~1\M162MMC\main.c
mmc.o: E:/SOUCER~1/M162MMC/iom162v.h
mmc.o:	E:\SOUCER~1\M162MMC\mmc.c
	$(CC) -c $(CFLAGS) E:\SOUCER~1\M162MMC\mmc.c
usb.o: E:/SOUCER~1/M162MMC/iom162v.h
usb.o:	E:\SOUCER~1\M162MMC\usb.c
	$(CC) -c $(CFLAGS) E:\SOUCER~1\M162MMC\usb.c
spi.o: E:/SOUCER~1/M162MMC/iom162v.h
spi.o:	E:\SOUCER~1\M162MMC\spi.c
	$(CC) -c $(CFLAGS) E:\SOUCER~1\M162MMC\spi.c
