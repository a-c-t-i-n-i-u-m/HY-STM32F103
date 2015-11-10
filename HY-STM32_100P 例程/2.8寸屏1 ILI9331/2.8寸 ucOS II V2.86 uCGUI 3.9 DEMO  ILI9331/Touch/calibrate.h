/*
 *
 *   Copyright (c) 2001, Carlos E. Vidales. All rights reserved.
 *
 *   This sample program was written and put in the public domain 
 *    by Carlos E. Vidales.  The program is provided "as is" 
 *    without warranty of any kind, either expressed or implied.
 *   If you choose to use the program within your own products
 *    you do so at your own risk, and assume the responsibility
 *    for servicing, repairing or correcting the program should
 *    it prove defective in any manner.
 *   You may copy and distribute the program's source code in any 
 *    medium, provided that you also include in each copy an
 *    appropriate copyright notice and disclaimer of warranty.
 *   You may also modify this program and distribute copies of
 *    it provided that you include prominent notices stating 
 *    that you changed the file(s) and the date of any change,
 *    and that you do not charge any royalties or licenses for 
 *    its use.
 * 
 *
 *   File Name:  calibrate.h
 *
 *
 *   Definition of constants and structures, and declaration of functions 
 *    in Calibrate.c
 *
 */

#ifndef _CALIBRATE_H_

#define _CALIBRATE_H_

/****************************************************/
/*                                                  */
/* Included files                                   */
/*                                                  */
/****************************************************/

#include <math.h>


/****************************************************/
/*                                                  */
/* Definitions                                      */
/*                                                  */
/****************************************************/

#ifndef		_CALIBRATE_C_
	#define		EXTERN         extern
#else
	#define		EXTERN
#endif



#ifndef		OK
	#define		OK		        0
	#define		NOT_OK		   -1
#endif



#define			INT32				long




/****************************************************/
/*                                                  */
/* Structures                                       */
/*                                                  */
/****************************************************/

typedef struct
{ 
   INT32    x;
   INT32    y;
}POINT;


#define  MATRIX_Type  INT32

typedef struct
{
							/* This arrangement of values facilitates 
							 *  calculations within getDisplayPoint() 
							 */
   MATRIX_Type    An;     /* A = An/Divider */
   MATRIX_Type    Bn;     /* B = Bn/Divider */
   MATRIX_Type    Cn;     /* C = Cn/Divider */
   MATRIX_Type    Dn;     /* D = Dn/Divider */
   MATRIX_Type    En;     /* E = En/Divider */
   MATRIX_Type    Fn;     /* F = Fn/Divider */
   MATRIX_Type    Divider ;
}MATRIX;




/****************************************************/
/*                                                  */
/* Function declarations                            */
/*                                                  */
/****************************************************/


EXTERN int setCalibrationMatrix( POINT * display,
                                 POINT * screen,
                                 MATRIX * matrix) ;


EXTERN int getDisplayPoint( POINT * display,
                            POINT * screen,
                            MATRIX * matrix ) ;


#endif  /* _CALIBRATE_H_ */
