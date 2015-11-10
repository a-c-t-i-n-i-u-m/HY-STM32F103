NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   0070   0018 =     24. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0070  __lit_start
       0088  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   0088   04D8 =   1240. bytes (rel,con,rom)

       Addr  Global Symbol
      -----  --------------------------------
       0044  __text_start
       0044  __start
       0063  _exit
       0064  _main
       0093  _DelayMs
       00A9  _MMCCS
       00AF  _MMCWrCmd
       00D8  _MMCInit
       0108  _MMCCID
       0122  _MMCRdBolck1
       016F  _MMCWrBlock1
       01C9  _uart1_init
       01D3  _uart1_send
       01D7  _uart1_rev
       01DB  _uart1_intrev
       01E2  _spi_init
       01EB  _SPIHiSPD
       01EF  _spi_send
       01F9  push_arg4
       01FB  push_arg2
       01FC  pop_gset2
       01FE  pop_gset4
       0200  push_gset5
       0202  push_gset4
       0204  push_gset3
       0206  push_gset2
       0208  push_gset1
       020B  pop_gset1
       020C  pop
       021F  push_lset
       0234  pop_lset
       024B  lsl32
       0257  lsr32
       0263  asgncblk
       027C  <created procedures>
       02B0  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0100   0200 =    512. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0100  __bss_start
       0100  _sector
       0300  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0000   0054 =     84. bytes (abs,ovr,rom)

Files Linked      [ module(s) ]

C:\icc\lib\crtatmega.o	[ crtatmega.s ]
main.o	[ main.c ]
mmc.o	[ mmc.c ]
usb.o	[ usb.c ]
spi.o	[ spi.c ]
<library>	[ asave.s, gpop2.s, gpop4.s, gpush.s, lpush.s, lsl32.s, lsr32.s, pmstruct.s ]

User Global Definitions

ram_end = 0x4ff
hwstk_size = 0x10

User Base Address Definitions

func_lit = 0x70
data = 0x100
eeprom:1.512

