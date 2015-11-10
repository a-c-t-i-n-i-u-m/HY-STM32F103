#ifndef _main_h_
	#define _main_h_

#include <stdio.h>
#include <string.h>	
#include <avr/io.h>	
#include <avr/eeprom.h>	
	
#include "mmc.h"
#include "fat.h"

extern void IOInit (void);
extern int uart_putchar (char c);
extern int main (void);

#endif//_main_h_
