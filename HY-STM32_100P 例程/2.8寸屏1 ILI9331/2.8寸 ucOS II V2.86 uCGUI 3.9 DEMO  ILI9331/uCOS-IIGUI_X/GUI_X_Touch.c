/*
*********************************************************************************************************
*                                                uC/GUI
*                        Universal graphic software for embedded applications
*
*                       (c) Copyright 2002, Micrium Inc., Weston, FL
*                       (c) Copyright 2002, SEGGER Microcontroller Systeme GmbH
*
*              礐/GUI is protected by international copyright laws. Knowledge of the
*              source code may not be used to write a similar product. This file may
*              only be used in accordance with a license and should not be redistributed
*              in any way. We appreciate your understanding and fairness.
*
----------------------------------------------------------------------
File        : GUI_TOUCH_X.C
Purpose     : Config / System dependent externals for GUI
---------------------------END-OF-HEADER------------------------------
*/

#include "..\APP\includes.h"
#include "..\usart\SPI_Flash.h"
//#include "..\GUIinc\GUI.h"
//#include "..\GUIinc\GUI_X.h"
//#include "..\TFT\ili932x.h"
//#include "bsp.h"
#define NotSelect_Flash()    GPIO_SetBits(GPIOA, GPIO_Pin_4)


extern unsigned char AT45_buffer[];

unsigned int Xs_1=0,Xs_2=0,Xs_3=0,Xs_4=0,Ys_1=0,Ys_2=0,Ys_3=0,Ys_4=0; //记录触坐标值
unsigned char dw=0,A_TP=0,b=0,TP_B=0; //TP_B存放 校准参数  0和1为XY轴  

unsigned short int X,Y;

void GUI_TOUCH_X_ActivateX(void) {
}

void GUI_TOUCH_X_ActivateY(void) {
}


int  GUI_TOUCH_X_MeasureX(void) 
{
	float Y1,hh;
    u16 xx;
	unsigned char t=0,t1,count=0;
	unsigned short int databuffer[10]={5,7,9,3,2,6,4,0,3,1};//数据组
	unsigned short temp=0,X=0;

	if(TP_B==0)
	{
	while(count<10)//循环读数10次
	{	   	  
		databuffer[count]=TPReadX();
		count++; 
	}  
	if(count==10)//一定要读到10次数据,否则丢弃
	{  
	    do//将数据X升序排列
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[t]>databuffer[t+1])//升序排列
				{
					temp=databuffer[t+1];
					databuffer[t+1]=databuffer[t];
					databuffer[t]=temp;
					t1=1; 
				}  
			}
		}while(t1); 
		if(A_TP==1||A_TP==2)
		{	    		 	 		  
		Y1=(databuffer[3]+databuffer[4]+databuffer[5])/3;
			xx=Xs_3-Xs_1;	//计算出X1,X3距离
			hh=(float)xx/280;	//计算出X比例因子
			xx=(float)((hh*320)-xx)/2;  //X方向偏移量
			X=320-(Y1-(Xs_1-xx))/hh;	//计算坐标
			X=X*12.79;
		}
		else
		{
		Y1=(databuffer[3]+databuffer[4]+databuffer[5])/3;
			xx=Xs_1-Xs_3;	//计算出X1,X3距离
			hh=(float)xx/280;	//计算出X比例因子
			xx=(float)((hh*320)-xx)/2;  //X方向偏移量
			X=(Y1-(Xs_3-xx))/hh;	//计算坐标
			X=X*12.79;

		}	  
	}
	}
	else
	{
	while(count<10)//循环读数10次
	{	   	  
		databuffer[count]=TPReadY();
		count++; 
	}  
	if(count==10)//一定要读到10次数据,否则丢弃
	{  
	    do//将数据X升序排列
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[t]>databuffer[t+1])//升序排列
				{
					temp=databuffer[t+1];
					databuffer[t+1]=databuffer[t];
					databuffer[t]=temp;
					t1=1; 
				}  
			}
		}while(t1); 	    		 	 		  
		if(A_TP==1||A_TP==3)
		{	    		 	 		  
		Y1=(databuffer[3]+databuffer[4]+databuffer[5])/3;
				xx=Ys_2-Ys_1;	//计算出Y1,Y2距离
				hh=(float)xx/280;		//计算出X比例
				xx=(float)((hh*320)-xx)/2;	//Y方向偏移量
				X=320-(Y1-(Ys_1-xx))/hh;		  //计算坐标
				X=X*12.79;

		}
		else
		{
		Y1=(databuffer[3]+databuffer[4]+databuffer[5])/3;
				xx=Ys_1-Ys_2;	//计算出Y1,Y2距离
				hh=(float)xx/280;		//计算出X比例
				xx=(float)((hh*320)-xx)/2;	//Y方向偏移量
				X=(Y1-(Ys_2-xx))/hh;		  //计算坐标
				X=X*12.79;

		}	  
//		if(X<=3730&&Y<=3730) //个人的屏根据初始参数修改.
//		{
//			if(X>=330)X-=330;
//			else X=0;
//			if(Y>=420)Y-=420;
//			else Y=0;  
//			drawbigpoint(240-X/14,320-Y/10);	 
//		}  
	}
	}





 	
	return(X);  
}

int  GUI_TOUCH_X_MeasureY(void) {
	float X1,hh;
    u16 xx;
  	unsigned char t=0,t1,count=0;
	unsigned short int databuffer[10]={5,7,9,3,2,6,4,0,3,1};//数据组
	unsigned short temp=0,Y=0;	


	if(TP_B==0)
 	{
    while(count<10)	//循环读数10次
	{	   	  
		databuffer[count]=TPReadY();
		count++;  
	}  
	if(count==10)//一定要读到10次数据,否则丢弃
	{  
	    do//将数据X升序排列
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[t]>databuffer[t+1])//升序排列
				{
					temp=databuffer[t+1];
					databuffer[t+1]=databuffer[t];
					databuffer[t]=temp;
					t1=1; 
				}  
			}
		}while(t1);
		if(A_TP==1||A_TP==3)
		{
		X1=(databuffer[3]+databuffer[4]+databuffer[5])/3;
			xx=Ys_2-Ys_1;	//计算出Y1,Y2距离
			hh=(float)xx/200;		//计算出X比例
			xx=(float)((hh*240)-xx)/2;	//Y方向偏移量
			Y=240-(X1-(Ys_1-xx))/hh;		  //计算坐标
			Y=Y*17.06;
		}
		else
		{
		X1=(databuffer[3]+databuffer[4]+databuffer[5])/3;
			xx=Ys_1-Ys_2;	//计算出Y1,Y2距离
			hh=(float)xx/200;		//计算出X比例
			xx=(float)((hh*240)-xx)/2;	//Y方向偏移量
			Y=(X1-(Ys_2-xx))/hh;		  //计算坐标
			Y=Y*17.06;  
		}

	}
	}
	else
	{
    while(count<10)	//循环读数10次
	{	   	  
		databuffer[count]=TPReadX();
		count++;  
	}  
	if(count==10)//一定要读到10次数据,否则丢弃
	{  
	    do//将数据X升序排列
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[t]>databuffer[t+1])//升序排列
				{
					temp=databuffer[t+1];
					databuffer[t+1]=databuffer[t];
					databuffer[t]=temp;
					t1=1; 
				}  
			}
		}while(t1); 	    		 	 		  
		if(A_TP==1||A_TP==2)
		{
		X1=(databuffer[3]+databuffer[4]+databuffer[5])/3;
			xx=Xs_3-Xs_1;	//计算出X1,X3距离
			hh=(float)xx/200;	//计算出X比例因子
			xx=(float)((hh*240)-xx)/2;  //X方向偏移量
			Y=240-(X1-(Xs_1-xx))/hh;	//计算坐标
			Y=Y*17.06;  
		}
		else
		{
		X1=(databuffer[3]+databuffer[4]+databuffer[5])/3;
			xx=Xs_1-Xs_3;	//计算出X1,X3距离
			hh=(float)xx/200;	//计算出X比例因子
			xx=(float)((hh*240)-xx)/2;  //X方向偏移量
			Y=(X1-(Xs_3-xx))/hh;	//计算坐标
			Y=Y*17.06;
		}


	}
	}

	return(Y); 
}		 


//----------------------------------------------------
void M25P80_buf_ToRam_TP(void) //读取保存的值
{

    M25P80_buf_ToRam(1,2,17); //读取数据  
	TP_B=AT45_buffer[17]; // 识别字

	Xs_1=(AT45_buffer[18]<<8)+AT45_buffer[19];
	Ys_1=(AT45_buffer[20]<<8)+AT45_buffer[21];

	Xs_2=(AT45_buffer[22]<<8)+AT45_buffer[23];
	Ys_2=(AT45_buffer[24]<<8)+AT45_buffer[25];

	Xs_3=(AT45_buffer[26]<<8)+AT45_buffer[27];
	Ys_3=(AT45_buffer[28]<<8)+AT45_buffer[29];

	Xs_4=(AT45_buffer[30]<<8)+AT45_buffer[31];
	Ys_4=(AT45_buffer[32]<<8)+AT45_buffer[33];
	
	dw=1;

			   		if((Xs_1<Xs_3)&&(Ys_1<Ys_2))	//左下角为起始坐标时进入
					{	 A_TP=1;
						 if((Ys_1*5<Ys_2)&&(Xs_1*5<Xs_3))	//判断触摸方向
						 {
							 TP_B=0;
						 }
						 else
						 {
							 TP_B=1;
						 }
						 
					}
			   		
					if((Xs_2<Xs_4)&&(Ys_2<Ys_1))	//左上角为起始坐标时进入
					{
						 A_TP=2;
						 if((Ys_2*5<Ys_1)&&(Xs_2*5<Xs_4))	//判断触摸方向
						 {
							 TP_B=0;
						 }
						 else
						 {
							 TP_B=1;
						 }
					}

			   		if((Xs_3<Xs_1)&&(Ys_3<Ys_4))	//右下角为起始坐标时进入
					{	 A_TP=3;
						 if((Ys_3<Ys_4)&&(Xs_3<Xs_1))	//判断触摸方向
						 {
							 TP_B=0;
						 }
						 else
						 {
							 TP_B=1;
						 }
					}

			   		if((Xs_4<Xs_2)&&(Ys_4<Ys_3))	//右上角为起始坐标时进入
					{	 A_TP=4;
						 if((Ys_4<Ys_3)&&(Xs_4<Xs_2))	//判断触摸方向
						 {
							 TP_B=0;
						 }
						 else
						 {
							 TP_B=1;
						 }
					}
//	heyaodz_TP();


}
/*	   
int  GUI_TOUCH_X_MeasureX(void) 
{
	unsigned char t=0,t1,count=0;
	unsigned short int databuffer[10]={5,7,9,3,2,6,4,0,3,1};//数据组
	unsigned short temp=0,X=0;
 	
	while(count<10)//循环读数10次
	{	   	  
		databuffer[count]=TPReadY();
		count++; 
	}  
	if(count==10)//一定要读到10次数据,否则丢弃
	{  
	    do//将数据X升序排列
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[t]>databuffer[t+1])//升序排列
				{
					temp=databuffer[t+1];
					databuffer[t+1]=databuffer[t];
					databuffer[t]=temp;
					t1=1; 
				}  
			}
		}while(t1); 	    		 	 		  
		Y=(databuffer[3]+databuffer[4]+databuffer[5])/3;	  
//		if(X<=3730&&Y<=3730) //个人的屏根据初始参数修改.
//		{
//			if(X>=330)X-=330;
//			else X=0;
//			if(Y>=420)Y-=420;
//			else Y=0;  
//			drawbigpoint(240-X/14,320-Y/10);	 
//		}  
	}
	return(Y);  
}

int  GUI_TOUCH_X_MeasureY(void) {
  	unsigned char t=0,t1,count=0;
	unsigned short int databuffer[10]={5,7,9,3,2,6,4,0,3,1};//数据组
	unsigned short temp=0,Y=0;	
 
    while(count<10)	//循环读数10次
	{	   	  
		databuffer[count]=TPReadX();
		count++;  
	}  
	if(count==10)//一定要读到10次数据,否则丢弃
	{  
	    do//将数据X升序排列
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[t]>databuffer[t+1])//升序排列
				{
					temp=databuffer[t+1];
					databuffer[t+1]=databuffer[t];
					databuffer[t]=temp;
					t1=1; 
				}  
			}
		}while(t1); 	    		 	 		  
		X=(databuffer[3]+databuffer[4]+databuffer[5])/3;	  
//		if(X<=3730&&Y<=3730) //个人的屏根据初始参数修改.
//		{
//			if(X>=330)X-=330;
//			else X=0;
//			if(Y>=420)Y-=420;
//			else Y=0;  
//			drawbigpoint(240-X/14,320-Y/10);	 
//		}   
//    M25P80_buf_ToRam(1,2,17); //读取数据  
	}
	return(X); 
}		 
  */
