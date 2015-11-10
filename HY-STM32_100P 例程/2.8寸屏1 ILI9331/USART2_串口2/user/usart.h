/**
  ******************************************************************************
  * @file 	usart.h 
  * @author www.arm79.com
  * @date   2010-06-24
  * @brief  This file contains the needed headers of usart.
  ******************************************************************************
  */

#ifndef __USART_H
#define __USART_H

#include "platform_config.h"
#include "stm32f10x_usart.h"
#include "misc.h"

void USART1_Config(void);
void GPIO_Configuration(void);
void Delay(vu32 Time);

#endif /* __USART_H */

