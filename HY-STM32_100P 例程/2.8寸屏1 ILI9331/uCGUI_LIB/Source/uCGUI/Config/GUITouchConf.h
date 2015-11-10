/*
*********************************************************************************************************
*                                                uC/GUI
*                        Universal graphic software for embedded applications
*
*                       (c) Copyright 2002, Micrium Inc., Weston, FL
*                       (c) Copyright 2002, SEGGER Microcontroller Systeme GmbH
*
*              �C/GUI is protected by international copyright laws. Knowledge of the
*              source code may not be used to write a similar product. This file may
*              only be used in accordance with a license and should not be redistributed
*              in any way. We appreciate your understanding and fairness.
*
----------------------------------------------------------------------
File        : GUITouch.Conf.h
Purpose     : Configures touch screen module
----------------------------------------------------------------------
*/


#ifndef __GUITOUCH_CONF_H
#define __GUITOUCH_CONF_H

#define GUI_TOUCH_SWAP_XY       0
#define GUI_TOUCH_MIRROR_X      1
#define GUI_TOUCH_MIRROR_Y      1

/*#define GUI_TOUCH_AD_LEFT       183
#define GUI_TOUCH_AD_RIGHT      3726
#define GUI_TOUCH_AD_TOP        270
#define GUI_TOUCH_AD_BOTTOM     3781
*/
#define GUI_TOUCH_AD_LEFT       1
#define GUI_TOUCH_AD_RIGHT      4095	
#define GUI_TOUCH_AD_TOP        4095	
#define GUI_TOUCH_AD_BOTTOM     1  

#endif /* GUITOUCH_CONF_H */
