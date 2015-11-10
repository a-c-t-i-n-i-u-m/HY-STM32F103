#ifndef __iom162v_h
#define __iom162v_h

/* ATmega162 header file for
 * ImageCraft ICCAVR compiler
 */

/* i/o register addresses
 * >= 0x60 are memory mapped only
 */

/* not strictly sorted by address no more
 */

/* interrupt vector number definitions added
 */

/* created 2002/05/17
 */

/* UART1 */
#define UBRR1H  (*(volatile unsigned char *)0x5C)
#define  URSEL1   7
#define UBRR1L  (*(volatile unsigned char *)0x20)
#define UBRR1   (*(volatile unsigned char *)0x20)
#define UCSR1C  (*(volatile unsigned char *)0x5C)
#define  URSEL1   7
#define  UMSEL1   6
#define  UPM11    5
#define  UPM10    4
#define  USBS1    3
#define  UCSZ11   2
#define  UCSZ10   1
#define  UCPOL1   0
#define UCSR1B  (*(volatile unsigned char *)0x21)
#define  RXCIE1   7
#define  TXCIE1   6
#define  UDRIE1   5
#define  RXEN1    4
#define  TXEN1    3
#define  UCSZ12   2
#define  RXB81    1
#define  TXB81    0
#define UCSR1A  (*(volatile unsigned char *)0x22)
#define  RXC1     7
#define  TXC1     6
#define  UDRE1    5
#define  FE1      4
#define  OVR1     3    /* Alternative Definition */
#define  DOR1     3
#define  OR1      3    /* Alternative Definition */
#define  UPE1     2
#define  U2X1     1
#define  MPCM1    0
#define UDR1    (*(volatile unsigned char *)0x23)

/* UART0 */
#define UBRR0H  (*(volatile unsigned char *)0x40)
#define  URSEL0   7
#define UBRR0L  (*(volatile unsigned char *)0x29)
#define UBRR0   (*(volatile unsigned char *)0x29)
#define UCSR0C  (*(volatile unsigned char *)0x40)
#define  URSEL0   7
#define  UMSEL0   6
#define  UPM01    5
#define  UPM00    4
#define  USBS0    3
#define  UCSZ01   2
#define  UCSZ00   1
#define  UCPOL0   0
#define UCSR0B  (*(volatile unsigned char *)0x2A)
#define  RXCIE0   7
#define  TXCIE0   6
#define  UDRIE0   5
#define  RXEN0    4
#define  TXEN0    3
#define  UCSZ02   2
#define  RXB80    1
#define  TXB80    0
#define UCSR0A  (*(volatile unsigned char *)0x2B)
#define  RXC0     7
#define  TXC0     6
#define  UDRE0    5
#define  FE0      4
#define  OVR0     3    /* Alternative Definition */
#define  DOR0     3
#define  OR0      3    /* Alternative Definition */
#define  UPE0     2
#define  U2X0     1
#define  MPCM0    0
#define UDR0    (*(volatile unsigned char *)0x2C)

/* Analog Comparator */
#define ACSR	(*(volatile unsigned char *)0x28)
#define  ACD      7
#define  AINBG    6
#define  ACBG     6
#define  ACO      5
#define  ACI      4
#define  ACIE     3
#define  ACIC     2
#define  ACIS1    1
#define  ACIS0    0

/* SPI */
#define SPCR    (*(volatile unsigned char *)0x2D)
#define  SPIE     7
#define  SPE      6
#define  DORD     5
#define  MSTR     4
#define  CPOL     3
#define  CPHA     2
#define  SPR1     1
#define  SPR0     0
#define SPSR	(*(volatile unsigned char *)0x2E)
#define  SPIF     7
#define  WCOL     6
#define  SPI2X    0
#define SPDR	(*(volatile unsigned char *)0x2F)

/* Port E */
#define PINE    (*(volatile unsigned char *)0x25)
#define DDRE    (*(volatile unsigned char *)0x26)
#define PORTE   (*(volatile unsigned char *)0x27)

/* Port D */
#define PIND    (*(volatile unsigned char *)0x30)
#define DDRD    (*(volatile unsigned char *)0x31)
#define PORTD   (*(volatile unsigned char *)0x32)

/* Port C */
#define PINC    (*(volatile unsigned char *)0x33)
#define DDRC    (*(volatile unsigned char *)0x34)
#define PORTC   (*(volatile unsigned char *)0x35)

/* Port B */
#define PINB    (*(volatile unsigned char *)0x36)
#define DDRB    (*(volatile unsigned char *)0x37)
#define PORTB   (*(volatile unsigned char *)0x38)

/* Port A */
#define PINA    (*(volatile unsigned char *)0x39)
#define DDRA    (*(volatile unsigned char *)0x3A)
#define PORTA   (*(volatile unsigned char *)0x3B)

/* EEPROM */
#define EECR    (*(volatile unsigned char *)0x3C)
#define  EERIE    3
#define  EEMWE    2
#define  EEWE     1
#define  EERE     0
#define EEDR    (*(volatile unsigned char *)0x3D)
#define EEAR    (*(volatile unsigned int *)0x3E)
#define EEARL   (*(volatile unsigned char *)0x3E)
#define EEARH   (*(volatile unsigned char *)0x3F)

/* Watch Dog Timer Control Register */
#define WDTCR   (*(volatile unsigned char *)0x41)
#define  WDTOE    4
#define  WDCE     4
#define  WDE      3
#define  WDP2     2
#define  WDP1     1
#define  WDP0     0

/*Timer/Counter3 */
#define ICR3    (*(volatile unsigned int *)0x80)
#define ICR3L   (*(volatile unsigned char *)0x80)
#define ICR3H   (*(volatile unsigned char *)0x81)
#define OCR3B   (*(volatile unsigned int *)0x84)
#define OCR3BL  (*(volatile unsigned char *)0x84)
#define OCR3BH  (*(volatile unsigned char *)0x85)
#define OCR3A   (*(volatile unsigned int *)0x86)
#define OCR3AL  (*(volatile unsigned char *)0x86)
#define OCR3AH  (*(volatile unsigned char *)0x87)
#define TCNT3   (*(volatile unsigned int *)0x88)
#define TCNT3L  (*(volatile unsigned char *)0x88)
#define TCNT3H  (*(volatile unsigned char *)0x89)
#define TCCR3B  (*(volatile unsigned char *)0x8A)
#define  ICNC3    7
#define  ICES3    6
#define  WGM33    4
#define  WGM32    3
#define  CS32     2
#define  CS31     1
#define  CS30     0
#define TCCR3A  (*(volatile unsigned char *)0x8B)
#define  COM3A1   7
#define  COM3A0   6
#define  COM3B1   5
#define  COM3B0   4
#define  FOC3A    3
#define  FOC3B    2
#define  WGM31    1
#define  WGM30    0

/* Timer/Counter2 */
#define ASSR    (*(volatile unsigned char *)0x46)
#define  AS2      3
#define  TCN2UB   2
#define  OCR2UB   1
#define  TCR2UB   0
#define OCR2    (*(volatile unsigned char *)0x42)
#define TCNT2   (*(volatile unsigned char *)0x43)
#define TCCR2   (*(volatile unsigned char *)0x47)
#define  FOC2     7
#define  WGM20    6
#define  COM21    5
#define  COM20    4
#define  WGM21    3
#define  CS22     2
#define  CS21     1
#define  CS20     0

/*Timer/Counter1 */
#define ICR1    (*(volatile unsigned int *)0x44)
#define ICR1L   (*(volatile unsigned char *)0x44)
#define ICR1H   (*(volatile unsigned char *)0x45)
#define OCR1B   (*(volatile unsigned int *)0x48)
#define OCR1BL  (*(volatile unsigned char *)0x48)
#define OCR1BH  (*(volatile unsigned char *)0x49)
#define OCR1A   (*(volatile unsigned int *)0x4A)
#define OCR1AL  (*(volatile unsigned char *)0x4A)
#define OCR1AH  (*(volatile unsigned char *)0x4B)
#define TCNT1   (*(volatile unsigned int *)0x4C)
#define TCNT1L  (*(volatile unsigned char *)0x4C)
#define TCNT1H  (*(volatile unsigned char *)0x4D)
#define TCCR1B  (*(volatile unsigned char *)0x4E)
#define  ICNC1    7
#define  ICES1    6
#define  WGM13    4
#define  WGM12    3
#define  CS12     2
#define  CS11     1
#define  CS10     0
#define TCCR1A  (*(volatile unsigned char *)0x4F)
#define  COM1A1   7
#define  COM1A0   6
#define  COM1B1   5
#define  COM1B0   4
#define  FOC1A    3
#define  FOC1B    2
#define  WGM11    1
#define  WGM10    0

/* Timer/Counter0 */
#define OCR0    (*(volatile unsigned char *)0x51)
#define TCNT0   (*(volatile unsigned char *)0x52)
#define TCCR0   (*(volatile unsigned char *)0x53)
#define  FOC0     7
#define  WGM00    6
#define  COM01    5
#define  COM00    4
#define  WGM01    3
#define  CS02     2
#define  CS01     1
#define  CS00     0

/* Timer/Counter Interrupts */
#define TIFR    (*(volatile unsigned char *)0x58)
#define  TOV1     7
#define  OCF1A    6
#define  OCF1B    5
#define  OCF2     4
#define  ICF1     3
#define  TOV2     2
#define  TOV0     1
#define  OCF0     0
#define TIMSK   (*(volatile unsigned char *)0x59)
#define  TOIE1    7
#define  OCIE1A   6
#define  OCIE1B   5
#define  OCIE2    4
#define  TICIE1   3
#define  TOIE2    2
#define  TOIE0    1
#define  OCIE0    0
#define ETIFR   (*(volatile unsigned char *)0x7C) /* m/m */
#define  ICF3     5
#define  OCF3A    4
#define  OCF3B    3
#define  TOV3     2
#define ETIMSK  (*(volatile unsigned char *)0x7D) /* m/m */
#define  TICIE3   5
#define  OCIE3A   4
#define  OCIE3B   3
#define  TOIE3    2

/* Special Function I/O Register */
#define SFIOR   (*(volatile unsigned char *)0x50)
#define  TSM      7
#define  XMBK     6
#define  XMM2     5
#define  XMM1     4
#define  XMM0     3
#define  PUD      2
#define  PSR2     1
#define  PSR310   0

/* MCU Control/Status Register */
#define MCUSR   (*(volatile unsigned char *)0x54)
#define MCUCSR  (*(volatile unsigned char *)0x54)
#define  JTD      7
#define  SM2      5
#define  JTRF     4
#define  WDRF     3
#define  BORF     2
#define  EXTRF    1
#define  PORF     0
#define MCUCR   (*(volatile unsigned char *)0x55)
#define  SRE      7
#define  SRW10    6
#define  SE       5
#define  SM1      4
#define  ISC11    3
#define  ISC10    2
#define  ISC01    1
#define  ISC00    0
#define EMCUCR  (*(volatile unsigned char *)0x56)
#define  SM0      7
#define  SRL2     6
#define  SRL1     5
#define  SRL0     4
#define  SRW01    3
#define  SRW00    2
#define  SRW11    1
#define  ISC2     0

/* SPM Conrol Register */
#define SPMCR   (*(volatile unsigned char *)0x57)
#define  SPMIE    7
#define  RWWSB    6
#define  RWWSRE   4
#define  BLBSET   3
#define  PGWRT    2
#define  PGERS    1
#define  SPMEN    0

/* JTAG Debug Register */
#define OCDR    (*(volatile unsigned char *)0x24)

/* OSC Calibration Register */
#define OSCCAL  (*(volatile unsigned char *)0x24)

/* Clock Prescaler */
#define CLKPR   (*(volatile unsigned char *)0x61) /* m/m */
#define  CLKPCE   7
#define  CLKPS3   3
#define  CLKPS2   2
#define  CLKPS1   1
#define  CLKPS0   0

/* Pin Change Interrupts */
#define PCMSK1  (*(volatile unsigned char *)0x6C) /* m/m */
#define  PCINT15  7
#define  PCINT14  6
#define  PCINT13  5
#define  PCINT12  4
#define  PCINT11  3
#define  PCINT10  2
#define  PCINT9   1
#define  PCINT8   0
#define PCMSK0  (*(volatile unsigned char *)0x6B) /* m/m */
#define  PCINT7   7
#define  PCINT6   6
#define  PCINT5   5
#define  PCINT4   4
#define  PCINT3   3
#define  PCINT2   2
#define  PCINT1   1
#define  PCINT0   0

/* General Interrupts */
#define GIFR    (*(volatile unsigned char *)0x5A)
#define  INTF1    7
#define  INTF0    6
#define  INTF2    5
#define  PCIF1    4
#define  PCIF0    3
#define GICR    (*(volatile unsigned char *)0x5B)
#define  INT1     7
#define  INT0     6
#define  INT2     5
#define  PCIE1    4
#define  PCIE0    3
#define  IVSEL    1
#define  IVCE     0

/* Stack Pointer */
#define SP      (*(volatile unsigned int *)0x5D)
#define SPL     (*(volatile unsigned char *)0x5D)
#define SPH     (*(volatile unsigned char *)0x5E)

/* Status Register */
#define SREG    (*(volatile unsigned char *)0x5F)


/* Port A bits */
#define  PORTA7   7
#define  PORTA6   6
#define  PORTA5   5
#define  PORTA4   4
#define  PORTA3   3
#define  PORTA2   2
#define  PORTA1   1
#define  PORTA0   0
#define  PA7      7
#define  PA6      6
#define  PA5      5
#define  PA4      4
#define  PA3      3
#define  PA2      2
#define  PA1      1
#define  PA0      0

#define  DDA7     7
#define  DDA6     6
#define  DDA5     5
#define  DDA4     4
#define  DDA3     3
#define  DDA2     2
#define  DDA1     1
#define  DDA0     0

#define  PINA7    7
#define  PINA6    6
#define  PINA5    5
#define  PINA4    4
#define  PINA3    3
#define  PINA2    2
#define  PINA1    1
#define  PINA0    0

/* Port B bits */
#define  PORTB7   7
#define  PORTB6   6
#define  PORTB5   5
#define  PORTB4   4
#define  PORTB3   3
#define  PORTB2   2
#define  PORTB1   1
#define  PORTB0   0
#define  PB7      7
#define  PB6      6
#define  PB5      5
#define  PB4      4
#define  PB3      3
#define  PB2      2
#define  PB1      1
#define  PB0      0

#define  DDB7     7
#define  DDB6     6
#define  DDB5     5
#define  DDB4     4
#define  DDB3     3
#define  DDB2     2
#define  DDB1     1
#define  DDB0     0

#define  PINB7    7
#define  PINB6    6
#define  PINB5    5
#define  PINB4    4
#define  PINB3    3
#define  PINB2    2
#define  PINB1    1
#define  PINB0    0

/* Port C bits */
#define  PORTC7   7
#define  PORTC6   6
#define  PORTC5   5
#define  PORTC4   4
#define  PORTC3   3
#define  PORTC2   2
#define  PORTC1   1
#define  PORTC0   0
#define  PC7      7
#define  PC6      6
#define  PC5      5
#define  PC4      4
#define  PC3      3
#define  PC2      2
#define  PC1      1
#define  PC0      0

#define  DDC7     7
#define  DDC6     6
#define  DDC5     5
#define  DDC4     4
#define  DDC3     3
#define  DDC2     2
#define  DDC1     1
#define  DDC0     0

#define  PINC7    7
#define  PINC6    6
#define  PINC5    5
#define  PINC4    4
#define  PINC3    3
#define  PINC2    2
#define  PINC1    1
#define  PINC0    0

/* Port D bits */
#define  PORTD7   7
#define  PORTD6   6
#define  PORTD5   5
#define  PORTD4   4
#define  PORTD3   3
#define  PORTD2   2
#define  PORTD1   1
#define  PORTD0   0
#define  PD7      7
#define  PD6      6
#define  PD5      5
#define  PD4      4
#define  PD3      3
#define  PD2      2
#define  PD1      1
#define  PD0      0

#define  DDD7     7
#define  DDD6     6
#define  DDD5     5
#define  DDD4     4
#define  DDD3     3
#define  DDD2     2
#define  DDD1     1
#define  DDD0     0

#define  PIND7    7
#define  PIND6    6
#define  PIND5    5
#define  PIND4    4
#define  PIND3    3
#define  PIND2    2
#define  PIND1    1
#define  PIND0    0

/* Port E bits */
#define  PORTE2   2
#define  PORTE1   1
#define  PORTE0   0
#define  PE2      2
#define  PE1      1
#define  PE0      0

#define  DDE2     2
#define  DDE1     1
#define  DDE0     0

#define  PINE2    2
#define  PINE1    1
#define  PINE0    0


/* Lock and Fuse Bits with LPM/SPM instructions */

#define  BLB12    5
#define  BLB11    4
#define  BLB02    3
#define  BLB01    2
#define  LB2      1
#define  LB1      0

#define  BOOTRST  6
#define  SPIEN    5
#define  BODLEVEL 4
#define  BODEN    3
#define  CKSEL2   2
#define  CKSEL1   1
#define  CKSEL0   0


/* Interrupt Vector Numbers */

#define iv_RESET        1
#define iv_INT0         2
#define iv_INT1         3
#define iv_INT2         4
#define iv_PCINT0       5
#define iv_PCINT1       6
#define iv_TIMER3_CAPT  7
#define iv_TIMER3_COMPA 8
#define iv_TIMER3_COMPB 9
#define iv_TIMER3_OVF   10
#define iv_TIMER2_COMP  11
#define iv_TIMER2_OVF   12
#define iv_TIMER1_CAPT  13
#define iv_TIMER1_COMPA 14
#define iv_TIMER1_COMPB 15
#define iv_TIMER1_OVF   16
#define iv_TIMER0_COMP  17
#define iv_TIMER0_OVF   18
#define iv_SPI_STC      19
#define iv_USART0_RX    20
#define iv_USART0_RXC   20
#define iv_USART1_RX    21
#define iv_USART1_RXC   21
#define iv_USART0_DRE   22
#define iv_USART0_UDRE  22
#define iv_USART1_DRE   23
#define iv_USART1_UDRE  23
#define iv_USART0_TX    24
#define iv_USART0_TXC   24
#define iv_USART1_TX    25
#define iv_USART1_TXC   25
#define iv_EE_RDY       26
#define iv_EE_READY     26
#define iv_ANA_COMP     27
#define iv_ANALOG_COMP  27
#define iv_SPM_RDY      28
#define iv_SPM_READY    28

/* */

#endif
