/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2006; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   STM3210B-EVAL Evaluation Board
*
* Filename      : includes.h
* Version       : V1.10
* Programmer(s) : BAN
*********************************************************************************************************
*/

#ifndef  __INCLUDES_H__
#define  __INCLUDES_H__

#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>



#include  <stm32f10x_conf.h>
//#include  <stm32f10x_lib.h>
#include  <stm32f10x.h>

//#include  "config.h"

#include  "app_cfg.h"
#include  "os_cfg.h"



//#include  <ucos_ii.h>


//#include  <cpu.h>
#include  "..\uCOS-II\uC-CPU\cpu.h"
#include  "..\uCOS-II\uC-CPU\cpu_def.h"

#include  "..\uCOS-II\Ports\os_cpu.h"
#include  "..\uCOS-II\Source\ucos_ii.h"

#include  "..\BSP\bsp.h"


#include  "..\Touch\ads7843.h"
#include  "..\Touch\calibrate.h"

#include  "..\uCGUIConfig\GUIConf.h"
#include  "..\uCGUIConfig\GUITouchConf.h"
#include  "..\uCGUIConfig\LCDConf.h"

#include "..\GUIinc\GUI.h"
#include "..\GUIinc\lcd.h"
#include "..\GUIinc\GUI_X.h"

#include "..\GUIinc\GUIType.h"
#include "..\uCGUIDemo\GUIDEMO.h"

#endif

