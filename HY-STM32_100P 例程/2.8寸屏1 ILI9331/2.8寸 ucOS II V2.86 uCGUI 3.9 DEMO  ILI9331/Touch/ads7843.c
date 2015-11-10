#include "..\APP\includes.h"


// A/D 通道选择命令字和工作寄存器
#define	CHX 	0x90 	//通道X+的选择控制字	
#define	CHY 	0xd0	//通道Y+的选择控制字 

//#define	CHX 	0xd0 	//通道X+的选择控制字	
//#define	CHY 	0x90	//通道Y+的选择控制字 

MATRIX  Matrix;//为了避免重复校准，可以保存该校准值

//====================================================================================
static void Delayus( int k)
{
 int j;
    
    for(j=k;j > 0;j--);    
}

//====================================================================================
void TP_Init(void)
{
  //PB10, PD9, PD10   CS,SI,CLK
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9|GPIO_Pin_10 ;  
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  //PENIRQ, SO	
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  //start_touch();
}

void start_touch()   
{ 	  
	TP_DCLK_L();   
	TP_CS_H(); 	 
	TP_DIN_H();  
	TP_DCLK_H();   
	TP_CS_L(); 	 
}

//====================================================================================
static void WR_CMD (u8 cmd) 
{
 //u8 buf;
 	
}


//====================================================================================
static u16 RD_AD(void) 
{
  u16 buf=0;
 u8 i;
    
    TP_DIN_L(); //TP_DIN(0);
    TP_DCLK_H(); //TP_DCLK(1);
    for(i=0; i<12; i++) 
    {	buf <<= 1;
        
        TP_DCLK_H(); //TP_DCLK(0);         
        Delayus(5);
        TP_DCLK_L(); //TP_DCLK(1);
        
        if(TP_DOUT)	buf |= 1;
        
        Delayus(5);
        
    }
    TP_CS_H(); //TP_CS(1);
    //buf>>=4;//只有12位有效
    //buf&=0x0fff;
    return(buf); 
}





#define ReadLoop 13 //必须>2
#define LOSS_DATA 5 //前后丢掉数据个数
u16 Read_XY(u8 xy) 
{ 
 u16 i, j;
 u16 buf[ReadLoop];
 uint32 sum;
 u16 val;
    
    for(i=0; i<ReadLoop; i++)
    {
       WR_CMD(xy);
       //while(TP_BUSY);
       Delayus(5);
       buf[i]=RD_AD();
       
       //sum += buf[i];
    }
    
    //排序
    for(i=0; i<ReadLoop-1; i++)
    {
       for(j=i+1; j<ReadLoop; j++)
       {
          if(buf[i]>buf[j])
          {
             val=buf[i];
             buf[i]=buf[j];
             buf[j]=val;
          }
       }
    }
    
    sum=0;
    for(i=LOSS_DATA; i<ReadLoop-1-LOSS_DATA; i++)
       sum += buf[i];
    val=sum/(ReadLoop-2*LOSS_DATA);
    
    return (val);    
}

//====================================================================================
uint8 TP_GetAdXY(u16 *x, u16 *y) 
{
 //u16 adx,ady;
    
    *y=Read_XY(CHX);
    *x=Read_XY(CHY);
    //*x=adx;
    //*y=ady;
    if(*x<100 || *y<100)
       return(0);
    else
       return(1);
}

/*
功能：读取触摸屏读坐标，该坐标未做转换，不能直接使用

返回：0=无效坐标
      1=有效坐标

说明：本函数连续采样2次，2次采样结果+-5范围内才算有效
*/
uint8 TP_GetAdXY2(u16 *x, u16 *y, uint32 delay) 
{u16 x1,y1;
 u16 x2,y2;
 u8 flag;
 
    flag=TP_GetAdXY(&x1, &y1);
    
    if(flag==0)
       return(0);
    
//    if(delay>=OS_TIME)
//       os_dly_wait(delay/OS_TIME);//300ms后再采样1次
    
    //do{
       flag=TP_GetAdXY(&x2, &y2);
    //}while(flag);
    
    if(flag==0)
       return(0);
    
    if( ( (x2<=x1 && x1<x2+50) || (x1<=x2 && x2<x1+50) )//前后两次采样在+-5内
     && ( (y2<=y1 && y1<y2+50) || (y1<=y2 && y2<y1+50) ) )
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return(1);
    }
    else
       return(0);
}

///*
//功能：读取触摸屏在LCD的坐标，该坐标已经转换为可用的视图坐标
//返回：0=不在显示区域
//      1=在显示区域
//*/
//uint8 TP_GetLCDXY(u16 *x, u16 *y) 
//{POINT   displayPoint;
// POINT   TouchSample;
// uint16 xt,yt;	
// uint8 flag;
// 
//    flag=TP_GetAdXY2(&xt, &yt, 0);
//    if(flag)
//    {
//       TouchSample.x=xt/4; TouchSample.y=yt/4;
//       getDisplayPoint( &displayPoint, &TouchSample, &Matrix );
//       
//       if(IsDisplayArea(&displayPoint)) //判断是否在显示区域
//       {
//          flag=1;
//          *x=displayPoint.x;
//          *y=displayPoint.y;
//       }
//       else
//          flag=0;
//    }
//    
//    return(flag);
//}
