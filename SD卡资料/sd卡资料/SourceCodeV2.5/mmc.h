/*#######################################################################################
Connect ARM to MMC/SD 

Copyright (C) 2004 Ulrich Radig

ARM modifications by Ingo Busker 2005
#######################################################################################*/

#ifndef _MMC_H_
 #define _MMC_H_

#include "typedefs.h"
#include "main.h"
  
#define SPI_Mode			1		//1 = Hardware SPI | 0 = Software SPI
//#define SPI_Mode			0

#define MMC_Write			PORTB	//Port an der die MMC/SD-Karte angeschlossen ist also des SPI 
#define MMC_Read			PINB
#define MMC_Direction_REG	DDRB	

#if defined (__AVR_ATmega128__)
	#define SPI_DI				3		//Port Pin an dem Data Output der MMC/SD-Karte angeschlossen ist 
	#define SPI_DO				2		//Port Pin an dem Data Input der MMC/SD-Karte angeschlossen ist
	#define SPI_Clock			1		//Port Pin an dem die Clock der MMC/SD-Karte angeschlossen ist (clk)
	#define MMC_Chip_Select		4		//Port Pin an dem Chip Select der MMC/SD-Karte angeschlossen ist
	#define SPI_SS				0		//Nicht Benutz muﬂ aber definiert werden
#endif

#if defined (__AVR_ATmega32__)
	#define SPI_DI				6		//Port Pin an dem Data Output der MMC/SD-Karte angeschlossen ist 
	#define SPI_DO				5		//Port Pin an dem Data Input der MMC/SD-Karte angeschlossen ist
	#define SPI_Clock			7		//Port Pin an dem die Clock der MMC/SD-Karte angeschlossen ist (clk)
	#define MMC_Chip_Select		3		//Port Pin an dem Chip Select der MMC/SD-Karte angeschlossen ist
	#define SPI_SS				4		//Nicht Benutz muﬂ aber definiert werden
#endif

//Prototypes
inline U08 mmc_read_byte(void);

inline void mmc_write_byte(U08);

extern void mmc_read_block(U08 *,U08 *,U16);

extern U08 mmc_init(void);

extern U08 mmc_read_sector (U32,U08 *);

extern U08 mmc_write_sector (U32,U08 *);

extern U08 mmc_write_command (U08 *);

extern U08 mmc_read_csd (U08 *);

extern U08 mmc_read_cid (U08 *);

//set MMC_Chip_Select to high (MMC/SD-Karte Inaktiv)
#define MMC_Disable() MMC_Write|= (1<<MMC_Chip_Select);

//set MMC_Chip_Select to low (MMC/SD-Karte Aktiv)
#define MMC_Enable() MMC_Write&=~(1<<MMC_Chip_Select);

#define nop()  __asm__ __volatile__ ("nop" ::)

#endif //_MMC_H_


