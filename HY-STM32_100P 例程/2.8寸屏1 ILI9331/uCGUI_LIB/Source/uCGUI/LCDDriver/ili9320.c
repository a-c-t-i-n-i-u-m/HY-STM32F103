#include "stm32f10x_lib.h"
#include "ili9320.h"
#include "ili9320_font.h"

void Lcd_Configuration(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	/*������Ӧʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);  
	/*����Lcd��������Ϊ�������*/
	/*16λ���ݵ�8λ*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*16λ���ݸ�8λ*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/*���ƽ�*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/*�������*/
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	Lcd_Light_ON;
}

/****************************************************************************
* ��    �ƣ�u16 CheckController(void)
* ��    �ܣ����ؿ���������
* ��ڲ�������
* ���ڲ������������ͺ�
* ˵    �������ú󷵻ؼ����ͺŵĿ������ͺ�
* ���÷�����code=CheckController();
****************************************************************************/

u16 CheckController(void)
{
  u16 tmp=0,tmp1=0,tmp2=0; 
	GPIO_InitTypeDef GPIO_InitStructure;

	ili9320_Reset();
	ili9320_WriteRegister(0x0000,0x0001);ili9320_Delay(50000);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//16λ���ݸ�8λ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  	GPIO_ResetBits(GPIOC,GPIO_Pin_8);
  
  	GPIO_SetBits(GPIOC,GPIO_Pin_9);
  
  	GPIO_ResetBits(GPIOC,GPIO_Pin_11);

  	tmp1 = GPIO_ReadInputData(GPIOB);
	tmp2 = GPIO_ReadInputData(GPIOC);

	tmp = (tmp1>>8) | (tmp2<<8);
  
  	GPIO_SetBits(GPIOC,GPIO_Pin_11);
  
  	GPIO_SetBits(GPIOC,GPIO_Pin_8);

	//16λ���ݵ�8λ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//16λ���ݸ�8λ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  	return tmp;
} 

/****************************************************************************
* ��    �ƣ�void ili9320_Initializtion()
* ��    �ܣ���ʼ�� ILI9320 ������
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷�����ili9320_Initializtion();
****************************************************************************/
void ili9320_Initializtion()
{
  /*****************************
  **    Ӳ������˵��          **
  ** STM32         ili9320    **
  ** PE0~15 <----> DB0~15     **
  ** PD15   <----> nRD        **
  ** PD14   <----> RS         **
  ** PD13   <----> nWR        **
  ** PD12   <----> nCS        **
  ** PD11   <----> nReset     **
  ** PC0    <----> BK_LED     **
  ******************************/
 	u16 i,id;

  	ili9320_WriteData(0xffff);
  	Set_Rst;
	Set_nWr;
	Set_Cs;
	Set_Rs;
	Set_nRd;
	Set_Rst;
  ili9320_Reset();                                        // ��λ ili9320_Reset
  for(i=50000;i>0;i--);
  for(i=50000;i>0;i--);
  for(i=50000;i>0;i--);
  ili9320_WriteRegister(0x0000,0x0001);ili9320_Delay(10000);
  for(i=50000;i>0;i--);
  for(i=50000;i>0;i--);
  id = CheckController();
	if(id==0x9325)
	{
  		ili9320_WriteRegister(0x00e7,0x0010);      
        ili9320_WriteRegister(0x0000,0x0001);  			//start internal osc
        ili9320_WriteRegister(0x0001,0x0100);     
        ili9320_WriteRegister(0x0002,0x0700); 				//power on sequence                     
        ili9320_WriteRegister(0x0003,(1<<12)|(1<<5)|(1<<4) ); 	//65K 
        ili9320_WriteRegister(0x0004,0x0000);                                   
        ili9320_WriteRegister(0x0008,0x0207);	           
        ili9320_WriteRegister(0x0009,0x0000);         
        ili9320_WriteRegister(0x000a,0x0000); 				//display setting         
        ili9320_WriteRegister(0x000c,0x0001);				//display setting          
        ili9320_WriteRegister(0x000d,0x0000); 				//0f3c          
        ili9320_WriteRegister(0x000f,0x0000);
        ili9320_WriteRegister(0x0010,0x0000);   
        ili9320_WriteRegister(0x0011,0x0007);
        ili9320_WriteRegister(0x0012,0x0000);                                                                 
        ili9320_WriteRegister(0x0013,0x0000);                 
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        ili9320_WriteRegister(0x0010,0x1590);   
        ili9320_WriteRegister(0x0011,0x0227);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        ili9320_WriteRegister(0x0012,0x009c);                 
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        ili9320_WriteRegister(0x0013,0x1900);   
        ili9320_WriteRegister(0x0029,0x0023);
        ili9320_WriteRegister(0x002b,0x000e);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        ili9320_WriteRegister(0x0020,0x0000);                                                            
        ili9320_WriteRegister(0x0021,0x0000);                 
      
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        ili9320_WriteRegister(0x0030,0x0007); 
        ili9320_WriteRegister(0x0031,0x0707);   
        ili9320_WriteRegister(0x0032,0x0006);
        ili9320_WriteRegister(0x0035,0x0704);
        ili9320_WriteRegister(0x0036,0x1f04); 
        ili9320_WriteRegister(0x0037,0x0004);
        ili9320_WriteRegister(0x0038,0x0000);        
        ili9320_WriteRegister(0x0039,0x0706);     
        ili9320_WriteRegister(0x003c,0x0701);
        ili9320_WriteRegister(0x003d,0x000f);
        for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
        ili9320_WriteRegister(0x0050,0x0000);        
        ili9320_WriteRegister(0x0051,0x00ef);   
        ili9320_WriteRegister(0x0052,0x0000);     
        ili9320_WriteRegister(0x0053,0x013f);
        ili9320_WriteRegister(0x0060,0xa700);        
        ili9320_WriteRegister(0x0061,0x0001); 
        ili9320_WriteRegister(0x006a,0x0000);
        ili9320_WriteRegister(0x0080,0x0000);
        ili9320_WriteRegister(0x0081,0x0000);
        ili9320_WriteRegister(0x0082,0x0000);
        ili9320_WriteRegister(0x0083,0x0000);
        ili9320_WriteRegister(0x0084,0x0000);
        ili9320_WriteRegister(0x0085,0x0000);
      
        ili9320_WriteRegister(0x0090,0x0010);     
        ili9320_WriteRegister(0x0092,0x0000);  
        ili9320_WriteRegister(0x0093,0x0003);
        ili9320_WriteRegister(0x0095,0x0110);
        ili9320_WriteRegister(0x0097,0x0000);        
        ili9320_WriteRegister(0x0098,0x0000);  
         //display on sequence     
        ili9320_WriteRegister(0x0007,0x0133);
    
        ili9320_WriteRegister(0x0020,0x0000);                                                            
        ili9320_WriteRegister(0x0021,0x0000);	
	}
	else if(id==0x9320)
	{
		ili9320_WriteRegister(0x00,0x0000);
		ili9320_WriteRegister(0x01,0x0100);	//Driver Output Contral.
		ili9320_WriteRegister(0x02,0x0700);	//LCD Driver Waveform Contral.
		ili9320_WriteRegister(0x03,0x1030);	//Entry Mode Set.
	
		ili9320_WriteRegister(0x04,0x0000);	//Scalling Contral.
		ili9320_WriteRegister(0x08,0x0202);	//Display Contral 2.(0x0207)
		ili9320_WriteRegister(0x09,0x0000);	//Display Contral 3.(0x0000)
		ili9320_WriteRegister(0x0a,0x0000);	//Frame Cycle Contal.(0x0000)
		ili9320_WriteRegister(0x0c,(1<<0));	//Extern Display Interface Contral 1.(0x0000)
		ili9320_WriteRegister(0x0d,0x0000);	//Frame Maker Position.
		ili9320_WriteRegister(0x0f,0x0000);	//Extern Display Interface Contral 2.
	
		for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
		ili9320_WriteRegister(0x07,0x0101);	//Display Contral.
		for(i=50000;i>0;i--);
		for(i=50000;i>0;i--);
	
		ili9320_WriteRegister(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	//Power Control 1.(0x16b0)
		ili9320_WriteRegister(0x11,0x0007);								//Power Control 2.(0x0001)
		ili9320_WriteRegister(0x12,(1<<8)|(1<<4)|(0<<0));					//Power Control 3.(0x0138)
		ili9320_WriteRegister(0x13,0x0b00);								//Power Control 4.
		ili9320_WriteRegister(0x29,0x0000);								//Power Control 7.
	
		ili9320_WriteRegister(0x2b,(1<<14)|(1<<4));
		
		ili9320_WriteRegister(0x50,0);		//Set X Start.
		ili9320_WriteRegister(0x51,239);	//Set X End.
		ili9320_WriteRegister(0x52,0);		//Set Y Start.
		ili9320_WriteRegister(0x53,319);	//Set Y End.
	
		ili9320_WriteRegister(0x60,0x2700);	//Driver Output Control.
		ili9320_WriteRegister(0x61,0x0001);	//Driver Output Control.
		ili9320_WriteRegister(0x6a,0x0000);	//Vertical Srcoll Control.
	
		ili9320_WriteRegister(0x80,0x0000);	//Display Position? Partial Display 1.
		ili9320_WriteRegister(0x81,0x0000);	//RAM Address Start? Partial Display 1.
		ili9320_WriteRegister(0x82,0x0000);	//RAM Address End-Partial Display 1.
		ili9320_WriteRegister(0x83,0x0000);	//Displsy Position? Partial Display 2.
		ili9320_WriteRegister(0x84,0x0000);	//RAM Address Start? Partial Display 2.
		ili9320_WriteRegister(0x85,0x0000);	//RAM Address End? Partial Display 2.
	
		ili9320_WriteRegister(0x90,(0<<7)|(16<<0));	//Frame Cycle Contral.(0x0013)
		ili9320_WriteRegister(0x92,0x0000);	//Panel Interface Contral 2.(0x0000)
		ili9320_WriteRegister(0x93,0x0001);	//Panel Interface Contral 3.
		ili9320_WriteRegister(0x95,0x0110);	//Frame Cycle Contral.(0x0110)
		ili9320_WriteRegister(0x97,(0<<8));	//
		ili9320_WriteRegister(0x98,0x0000);	//Frame Cycle Contral.

	
		ili9320_WriteRegister(0x07,0x0173);	//(0x0173)
	}
  for(i=50000;i>0;i--);
  ili9320_Clear(0xf800);
}

/****************************************************************************
* ��    �ƣ�void ili9320_SetCursor(u16 x,u16 y)
* ��    �ܣ�������Ļ����
* ��ڲ�����x      ������
*           y      ������
* ���ڲ�������
* ˵    ����
* ���÷�����ili9320_SetCursor(10,10);
****************************************************************************/
void ili9320_SetCursor(u16 x,u16 y)
{
  ili9320_WriteRegister(0x0020,x); // ��
  ili9320_WriteRegister(0x0021,y); // ��
}

/****************************************************************************
* ��    �ƣ�void ili9320_SetWindows(u16 StartX,u16 StartY,u16 EndX,u16 EndY)
* ��    �ܣ����ô�������
* ��ڲ�����StartX     ����ʼ����
*           StartY     ����ʼ����
*           EndX       �н�������
*           EndY       �н�������
* ���ڲ�������
* ˵    ����
* ���÷�����ili9320_SetWindows(0,0,100,100)��
****************************************************************************/
void ili9320_SetWindows(u16 StartX,u16 StartY,u16 EndX,u16 EndY)
{
  ili9320_SetCursor(StartX,StartY);
  ili9320_WriteRegister(0x0050, StartX);
  ili9320_WriteRegister(0x0052, StartY);
  ili9320_WriteRegister(0x0051, EndX);
  ili9320_WriteRegister(0x0053, EndY);
}

/****************************************************************************
* ��    �ƣ�void ili9320_Clear(u16 dat)
* ��    �ܣ�����Ļ����ָ������ɫ��������������� 0xffff
* ��ڲ�����dat      ���ֵ
* ���ڲ�������
* ˵    ����
* ���÷�����ili9320_Clear(0xffff);
****************************************************************************/
void ili9320_Clear(u16 dat)
{
  u32  i;

  	ili9320_SetCursor(0x00, 0x0000);
	ili9320_WriteRegister(0x0050,0x00);//ˮƽ GRAM��ʼλ��
	ili9320_WriteRegister(0x0051,239);//ˮƽGRAM��ֹλ��
	ili9320_WriteRegister(0x0052,0x00);//��ֱGRAM��ʼλ��
	ili9320_WriteRegister(0x0053,319);//��ֱGRAM��ֹλ��  
  
  Clr_Cs; 
  ili9320_WriteIndex(0x0022); 
  Set_Rs;   
  for(i=0;i<76800;i++)
  {
    ili9320_WriteData(dat);
	Clr_nWr;
	Set_nWr;
  }

  Set_Cs;
}

/****************************************************************************
* ��    �ƣ�u16 ili9320_GetPoint(u16 x,u16 y)
* ��    �ܣ���ȡָ���������ɫֵ
* ��ڲ�����x      ������
*           y      ������
* ���ڲ�������ǰ������ɫֵ
* ˵    ����
* ���÷�����i=ili9320_GetPoint(10,10);
****************************************************************************/
u16 ili9320_GetPoint(u16 x,u16 y)
{
  ili9320_SetCursor(x,y);
  return (ili9320_BGR2RGB(ili9320_ReadRegister(0x0022)));
}

/****************************************************************************
* ��    �ƣ�void ili9320_SetPoint(u16 x,u16 y,u16 point)
* ��    �ܣ���ָ�����껭��
* ��ڲ�����x      ������
*           y      ������
*           point  �����ɫ
* ���ڲ�������
* ˵    ����
* ���÷�����ili9320_SetPoint(10,10,0x0fe0);
****************************************************************************/
void ili9320_SetPoint(u16 x,u16 y,u16 point)
{
  if ( (x>240)||(y>320) ) return;
  ili9320_SetCursor(x,y);

  Clr_Cs;
  ili9320_WriteIndex(0x0022);
  Set_Rs;
  ili9320_WriteData(point);
  Clr_nWr;Set_nWr;
  Set_Cs;
}

/****************************************************************************
* ��    �ƣ�void ili9320_DrawPicture(u16 StartX,u16 StartY,u16 EndX,u16 EndY,u16 *pic)
* ��    �ܣ���ָ�����귶Χ��ʾһ��ͼƬ
* ��ڲ�����StartX     ����ʼ����
*           StartY     ����ʼ����
*           EndX       �н�������
*           EndY       �н�������
            pic        ͼƬͷָ��
* ���ڲ�������
* ˵    ����ͼƬȡģ��ʽΪˮƽɨ�裬16λ��ɫģʽ
* ���÷�����ili9320_DrawPicture(0,0,100,100,(u16*)demo);
****************************************************************************/
void ili9320_DrawPicture(u16 StartX,u16 StartY,u16 EndX,u16 EndY,u16 *pic)
{
  u16  i;
  ili9320_SetWindows(StartX,StartY,EndX,EndY);
  ili9320_SetCursor(StartX,StartY);
  
  Clr_Cs;

  ili9320_WriteIndex(0x0022);
  Set_Rs;
  for (i=0;i<(EndX*EndY);i++)
  {
      ili9320_WriteData(*pic++);
	  Clr_nWr;Set_nWr;
  }
      
  Set_Cs;
}

/****************************************************************************
* ��    �ƣ�void ili9320_PutChar(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
* ��    �ܣ���ָ��������ʾһ��8x16�����ascii�ַ�
* ��ڲ�����x          ������
*           y          ������
*           charColor  �ַ�����ɫ
*           bkColor    �ַ�������ɫ
* ���ڲ�������
* ˵    ������ʾ��Χ�޶�Ϊ����ʾ��ascii��
* ���÷�����ili9320_PutChar(10,10,'a',0x0000,0xffff);
****************************************************************************/
void ili9320_PutChar(u16 x,u16 y,u8 c,u16 charColor,u16 bkColor)
{
  u16 i=0;
  u16 j=0;
  
  u8 tmp_char=0;

  for (i=0;i<16;i++)
  {
    tmp_char=ascii_8x16[((c-0x20)*16)+i];
    for (j=0;j<8;j++)
    {
      if ( (tmp_char >> 7-j) & 0x01 == 0x01)
        {
          ili9320_SetPoint(x+j,y+i,charColor); // �ַ���ɫ
        }
        else
        {
          ili9320_SetPoint(x+j,y+i,bkColor); // ������ɫ
        }
    }
  }
}

/****************************************************************************
* ��    �ƣ�void ili9320_Test()
* ��    �ܣ�����Һ����
* ��ڲ�������
* ���ڲ�������
* ˵    ������ʾ����������Һ�����Ƿ���������
* ���÷�����ili9320_Test();
****************************************************************************/
void ili9320_Test()
{
  u8  R_data,G_data,B_data,i,j;

	ili9320_SetCursor(0x00, 0x0000);
	ili9320_WriteRegister(0x0050,0x00);//ˮƽ GRAM��ʼλ��
	ili9320_WriteRegister(0x0051,239);//ˮƽGRAM��ֹλ��
	ili9320_WriteRegister(0x0052,0);//��ֱGRAM��ʼλ��
	ili9320_WriteRegister(0x0053,319);//��ֱGRAM��ֹλ��  
	Clr_Cs; 
	ili9320_WriteIndex(0x0022);
	Set_Rs;
    R_data=0;G_data=0;B_data=0;     
    for(j=0;j<50;j++)//��ɫ��ǿ��
    {
        for(i=0;i<240;i++)
            {R_data=i/8;ili9320_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;Set_nWr;}
    }
    R_data=0x1f;G_data=0x3f;B_data=0x1f;
    for(j=0;j<50;j++)
    {
        for(i=0;i<240;i++)
            {
            G_data=0x3f-(i/4);
            B_data=0x1f-(i/8);
            ili9320_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;
			}
    }
//----------------------------------
    R_data=0;G_data=0;B_data=0;
    for(j=0;j<50;j++)//��ɫ��ǿ��
    {
        for(i=0;i<240;i++)
            {G_data=i/4;
			ili9320_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;}
    }

    R_data=0x1f;G_data=0x3f;B_data=0x1f;
    for(j=0;j<50;j++)
    {
        for(i=0;i<240;i++)
            {
            R_data=0x1f-(i/8);
            B_data=0x1f-(i/8);
            ili9320_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;
		}
    }
//----------------------------------
 
    R_data=0;G_data=0;B_data=0;
    for(j=0;j<60;j++)//��ɫ��ǿ��
    {
        for(i=0;i<240;i++)
            {B_data=i/8;ili9320_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;}
    } 

    B_data=0; 
    R_data=0x1f;G_data=0x3f;B_data=0x1f;

    for(j=0;j<60;j++)
    {
        for(i=0;i<240;i++)
            {
            G_data=0x3f-(i/4);
            R_data=0x1f-(i/8);
            ili9320_WriteData(R_data<<11|G_data<<5|B_data);
			Clr_nWr;
			Set_nWr;
		}
    }	  
	Set_Cs;
}

/****************************************************************************
* ��    �ƣ�u16 ili9320_BGR2RGB(u16 c)
* ��    �ܣ�RRRRRGGGGGGBBBBB ��Ϊ BBBBBGGGGGGRRRRR ��ʽ
* ��ڲ�����c      BRG ��ɫֵ
* ���ڲ�����RGB ��ɫֵ
* ˵    �����ڲ���������
* ���÷�����
****************************************************************************/
u16 ili9320_BGR2RGB(u16 c)
{
  u16  r, g, b;

  b = (c>>0)  & 0x1f;
  g = (c>>5)  & 0x3f;
  r = (c>>11) & 0x1f;
  
  return( (b<<11) + (g<<5) + (r<<0) );
}

/****************************************************************************
* ��    �ƣ�void ili9320_WriteIndex(u16 idx)
* ��    �ܣ�д ili9320 �������Ĵ�����ַ
* ��ڲ�����idx   �Ĵ�����ַ
* ���ڲ�������
* ˵    ��������ǰ����ѡ�п��������ڲ�����
* ���÷�����ili9320_WriteIndex(0x0000);
****************************************************************************/
void ili9320_WriteIndex(u16 idx)
{
    Clr_Rs;
	Set_nRd;
	ili9320_WriteData(idx);
	Clr_nWr;
	Set_nWr;
	Set_Rs;
}

/****************************************************************************
* ��    �ƣ�void ili9320_WriteData(u16 dat)
* ��    �ܣ�д ili9320 �Ĵ�������
* ��ڲ�����dat     �Ĵ�������
* ���ڲ�������
* ˵    �����������ָ����ַд�����ݣ�����ǰ����д�Ĵ�����ַ���ڲ�����
* ���÷�����ili9320_WriteData(0x1030)
****************************************************************************/
void ili9320_WriteData(u16 data)
{
  	u16 temp;
	temp = GPIO_ReadOutputData(GPIOB);
	GPIO_Write(GPIOB, (data<<8)|(temp&0x00ff));
	temp = GPIO_ReadOutputData(GPIOC);
	GPIO_Write(GPIOC, (data>>8)|(temp&0xff00));
}

/****************************************************************************
* ��    �ƣ�u16 ili9320_ReadData(void)
* ��    �ܣ���ȡ����������
* ��ڲ�������
* ���ڲ��������ض�ȡ��������
* ˵    �����ڲ�����
* ���÷�����i=ili9320_ReadData();
****************************************************************************/
u16 ili9320_ReadData(void)
{
	u16 tmp=0,tmp1=0,tmp2=0; 
	GPIO_InitTypeDef GPIO_InitStructure;

	ili9320_WriteData(0xffff);
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*16λ���ݸ�8λ*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  	GPIO_ResetBits(GPIOC,GPIO_Pin_8);
  
  	GPIO_SetBits(GPIOC,GPIO_Pin_9);
  
  	GPIO_ResetBits(GPIOC,GPIO_Pin_11);

  	tmp1 = GPIO_ReadInputData(GPIOB);
	tmp2 = GPIO_ReadInputData(GPIOC);

	tmp = (tmp1>>8) | (tmp2<<8);
  
  	GPIO_SetBits(GPIOC,GPIO_Pin_11);
  
  	GPIO_SetBits(GPIOC,GPIO_Pin_8);

	/*16λ���ݵ�8λ*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*16λ���ݸ�8λ*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  	return tmp;
}

/****************************************************************************
* ��    �ƣ�u16 ili9320_ReadRegister(u16 index)
* ��    �ܣ���ȡָ����ַ�Ĵ�����ֵ
* ��ڲ�����index    �Ĵ�����ַ
* ���ڲ������Ĵ���ֵ
* ˵    �����ڲ�����
* ���÷�����i=ili9320_ReadRegister(0x0022);
****************************************************************************/
u16 ili9320_ReadRegister(u16 index)
{
	u16 tmp;
  
  	Clr_Cs;
	Clr_Rs;
	Set_nRd;
	ili9320_WriteData(index);
	Clr_nWr;
	Set_nWr;
	Clr_nRd;     
	tmp = ili9320_ReadData();       
	Set_nRd;
	Set_Cs;
	return tmp;
}

/****************************************************************************
* ��    �ƣ�void ili9320_WriteRegister(u16 index,u16 dat)
* ��    �ܣ�дָ����ַ�Ĵ�����ֵ
* ��ڲ�����index    �Ĵ�����ַ
*         ��dat      �Ĵ���ֵ
* ���ڲ�������
* ˵    �����ڲ�����
* ���÷�����ili9320_WriteRegister(0x0000,0x0001);
****************************************************************************/
void ili9320_WriteRegister(u16 index,u16 dat)
{
 /************************************************************************
  **                                                                    **
  ** nCS       ----\__________________________________________/-------  **
  ** RS        ------\____________/-----------------------------------  **
  ** nRD       -------------------------------------------------------  **
  ** nWR       --------\_______/--------\_____/-----------------------  **
  ** DB[0:15]  ---------[index]----------[data]-----------------------  **
  **                                                                    **
  ************************************************************************/
  
    Clr_Cs;
	Clr_Rs;
	Set_nRd;
	ili9320_WriteData(index);
	Clr_nWr;
	Set_nWr;
	Set_Rs;       
	ili9320_WriteData(dat);       
	Clr_nWr;
	Set_nWr;
	Set_Cs;
}

/****************************************************************************
* ��    �ƣ�void ili9320_Reset()
* ��    �ܣ���λ ili9320 ������
* ��ڲ�������
* ���ڲ�������
* ˵    ������λ���������ڲ�����
* ���÷�����ili9320_Reset()
****************************************************************************/
void ili9320_Reset()
{
  /***************************************
   **                                   **
   **  -------\______________/-------   **
   **         | <---Tres---> |          **
   **                                   **
   **  Tres: Min.1ms                    **
   ***************************************/
    
  	Set_Rst;;
    ili9320_Delay(50000);
  	Clr_Rst;
    ili9320_Delay(50000);
  	Set_Rst;
    ili9320_Delay(50000);
}

/****************************************************************************
* ��    �ƣ�void ili9320_BackLight(u8 status)
* ��    �ܣ�������Һ������
* ��ڲ�����status     1:���⿪  0:�����
* ���ڲ�������
* ˵    ����
* ���÷�����ili9320_BackLight(1);
****************************************************************************/
void ili9320_BackLight(u8 status)
{
  if ( status >= 1 )
  {
    Lcd_Light_ON;
  }
  else
  {
    Lcd_Light_OFF;
  }
}

/****************************************************************************
* ��    �ƣ�void ili9320_Delay(vu32 nCount)
* ��    �ܣ���ʱ
* ��ڲ�����nCount   ��ʱֵ
* ���ڲ�������
* ˵    ����
* ���÷�����ili9320_Delay(10000);
****************************************************************************/
void ili9320_Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}
