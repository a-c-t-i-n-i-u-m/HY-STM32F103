/*******************************************

				  2.4寸 QVGA显示驱动程序







**********************************************/


#include "fsmc_sram.h"
#include "tftlcd.h"
#include "font.h"   

#define Bank1_LCD_D    ((uint32_t)0x60020000)    //disp Data ADDR
#define Bank1_LCD_C    ((uint32_t)0x60000000)	 //disp Reg ADDR

unsigned long color1=0;
//void MUC_Init();
void LCD_Init(void);	 //初始化
void LCD_WR_REG(unsigned int index);
void LCD_WR_CMD(unsigned int index,unsigned int val);
void LCD_WR_Data(unsigned int val);
void LCD_WR_Data_8(unsigned int val);
void LCD_test(void);	
void LCD_clear(unsigned int p);
void ini(void);
void TFT_DrawPoint(u8 x,u16 y);	   //画点

void TFT_DrawLine(u8 x1, u16 y1, u8 x2, u16 y2);//画线

void lcd_wr_zf(unsigned int a, unsigned int b, unsigned int a1,unsigned int b1, unsigned int d,unsigned int e, unsigned char g, unsigned char *f); 
void lcd_wr_pixel(unsigned int a, unsigned int b, unsigned int e) ;
unsigned char *num_pub(unsigned int a);

unsigned int color[]={0xf800,0x07e0,0x001f,0xffe0,0x0000,0xffff,0x07ff,0xf81f};
extern unsigned char zm9[];
extern unsigned char zm8[];
extern unsigned char zm7[];
extern unsigned char zm6[];
extern unsigned char zm5[];
extern unsigned char zm4[];
extern unsigned char zm3[];
extern unsigned char zm2[];
extern unsigned char zm1[];
extern unsigned char zm0[];
extern unsigned char a1[];
extern unsigned char a2[];
extern unsigned char zf2[];
extern unsigned char zf3[];	

u16 POINT_COLOR=RED;     //默认红色  
	unsigned int LCD_IDP;

unsigned int LCD_RD_data(void);




extern void lcd_rst(void);
extern void Delay(__IO uint32_t nCount);


//写寄存器地址函数
void LCD_WR_REG(unsigned int index)
{
	*(__IO uint16_t *) (Bank1_LCD_C)= index;

}



//写寄存器数据函数
//输入：dbw 数据位数，1为16位，0为8位。
void LCD_WR_CMD(unsigned int index,unsigned int val)
{	
	*(__IO uint16_t *) (Bank1_LCD_C)= index;	
	*(__IO uint16_t *) (Bank1_LCD_D)= val;
}

unsigned int LCD_RD_data(void){
	unsigned int a=0;
	a=(*(__IO uint16_t *) (Bank1_LCD_D)); 	//Dummy
//	a= *(__IO uint16_t *) (Bank1_LCD_D);  	//H
//	a=a<<8;
	a=*(__IO uint16_t *) (Bank1_LCD_D); //L

	return(a);	
}



//写16位数据函数
void    LCD_WR_Data(unsigned int val)
{   
	*(__IO uint16_t *) (Bank1_LCD_D)= val; 	
}

void LCD_WR_Data_8(unsigned int val)
{
	*(__IO uint16_t *) (Bank1_LCD_D)= val;
}
/****************************************************************************
* 名    称：u16 ili9320_ReadRegister(u16 index)
* 功    能：读取指定地址寄存器的值
* 入口参数：index    寄存器地址
* 出口参数：寄存器值
* 说    明：内部函数
* 调用方法：i=ili9320_ReadRegister(0x0022);
****************************************************************************/
u16 ili9320_ReadRegister(u16 index)
{
  u16 tmp;
  tmp= *(volatile unsigned int *)(0x60000000);
  
  return tmp;
}

/****************************************************************************
* 名    称：void ili9320_WriteIndex(u16 idx)
* 功    能：写 ili9320 控制器寄存器地址
* 入口参数：idx   寄存器地址
* 出口参数：无
* 说    明：调用前需先选中控制器，内部函数
* 调用方法：ili9320_WriteIndex(0x0000);
****************************************************************************/

void ili9320_WriteIndex(u16 idx)
{
  LCD_WR_REG(idx);
}



//初始化函数
void LCD_Init(void)
{
	Delay(200);
	LCD_WR_CMD(0x0000,0x0001);    Delay(200);  //打开晶振
	Delay(200);
	LCD_IDP=ili9320_ReadRegister(0X0000);

	LCD_IDP=0X9325;

	lcd_rst();	 	//  TFT 复位操作
	Delay(200);
	if((LCD_IDP==0X9325)||(LCD_IDP==0X9328))
	{
	    Delay(200);
//############# void Power_Set(void) ################//
		LCD_WR_CMD(0x00E7, 0x1014);
LCD_WR_CMD (0x0001, 0x0100); // set SS and SM bit
LCD_WR_CMD (0x0002, 0x0200); // set 1 line inversion
LCD_WR_CMD (0x0003, 0x1030); // set GRAM write direction and BGR=1.
LCD_WR_CMD (0x0008, 0x0202); // set the back porch and front porch
LCD_WR_CMD (0x0009, 0x0000); // set non-display area refresh cycle ISC[3:0]
LCD_WR_CMD (0x000A, 0x0000); // FMARK function
LCD_WR_CMD (0x000C, 0x0000); // RGB interface setting
LCD_WR_CMD (0x000D, 0x0000); // Frame marker Position
LCD_WR_CMD (0x000F, 0x0000); // RGB interface polarity
//*************Power On sequence ****************//
LCD_WR_CMD (0x0010, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB
LCD_WR_CMD (0x0011, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]
LCD_WR_CMD (0x0012, 0x0000); // VREG1OUT voltage
LCD_WR_CMD (0x0013, 0x0000); // VDV[4:0] for VCOM amplitude
	    Delay(200);
LCD_WR_CMD (0x0010, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB
LCD_WR_CMD (0x0011, 0x0227); // DC1[2:0], DC0[2:0], VC[2:0]
	    Delay(200);
LCD_WR_CMD (0x0012, 0x000C); // Internal reference voltage= Vci;
	    Delay(200);
LCD_WR_CMD (0x0013, 0x0800); // Set VDV[4:0] for VCOM amplitude
LCD_WR_CMD (0x0029, 0x0011); // Set VCM[5:0] for VCOMH
LCD_WR_CMD (0x002B, 0x000B); // Set Frame Rate
	    Delay(200);
LCD_WR_CMD (0x0020, 0x0000); // GRAM horizontal Address
LCD_WR_CMD (0x0021, 0x0000); // GRAM Vertical Address
// ----------- Adjust the Gamma Curve ----------//
LCD_WR_CMD (0x0030, 0x0000);
LCD_WR_CMD (0x0031, 0x0106);
LCD_WR_CMD (0x0032, 0x0000);
LCD_WR_CMD (0x0035, 0x0204);
LCD_WR_CMD (0x0036, 0x160A);
LCD_WR_CMD (0x0037, 0x0707);
LCD_WR_CMD (0x0038, 0x0106);
LCD_WR_CMD (0x0039, 0x0707);
LCD_WR_CMD (0x003C, 0x0402);
LCD_WR_CMD (0x003D, 0x0C0F);
//------------------ Set GRAM area ---------------//
LCD_WR_CMD (0x0050, 0x0000); // Horizontal GRAM Start Address
LCD_WR_CMD (0x0051, 0x00EF); // Horizontal GRAM End Address
LCD_WR_CMD (0x0052, 0x0000); // Vertical GRAM Start Address
LCD_WR_CMD (0x0053, 0x013F); // Vertical GRAM Start Address
LCD_WR_CMD (0x0060, 0x2700); // Gate Scan Line
LCD_WR_CMD (0x0061, 0x0001); // NDL,VLE, REV
LCD_WR_CMD (0x006A, 0x0000); // set scrolling line
//-------------- Partial Display Control ---------//
LCD_WR_CMD (0x0080, 0x0000);
LCD_WR_CMD (0x0081, 0x0000);
LCD_WR_CMD (0x0082, 0x0000);
LCD_WR_CMD (0x0083, 0x0000);
LCD_WR_CMD (0x0084, 0x0000);
LCD_WR_CMD (0x0085, 0x0000);
//-------------- Panel Control -------------------//
LCD_WR_CMD (0x0090, 0x0010);
LCD_WR_CMD (0x0092, 0x0600);
LCD_WR_CMD (0x0007, 0x0133); // 262K color and display ON
 /*
 Delay(950);                     //根据不同晶振速度可以调整延时，保障稳定显示
 LCD_WR_CMD(0x0001,0x0100); 
 LCD_WR_CMD(0x0002,0x0700); 
 LCD_WR_CMD(0x0003,0x1030); 
 LCD_WR_CMD(0x0004,0x0000); 
 LCD_WR_CMD(0x0008,0x0207);  
 LCD_WR_CMD(0x0009,0x0000);
 LCD_WR_CMD(0x000A,0x0000); 
 LCD_WR_CMD(0x000C,0x0000); 
 LCD_WR_CMD(0x000D,0x0000);
 LCD_WR_CMD(0x000F,0x0000);
//power on sequence VGHVGL
 LCD_WR_CMD(0x0010,0x0000);   
 LCD_WR_CMD(0x0011,0x0007);  
 LCD_WR_CMD(0x0012,0x0000);  
 LCD_WR_CMD(0x0013,0x0000); 
//vgh 
 LCD_WR_CMD(0x0010,0x1290);   
 LCD_WR_CMD(0x0011,0x0227);
 Delay(100);
 //vregiout 
 LCD_WR_CMD(0x0012,0x001d); //0x001b
 Delay(100); 
 //vom amplitude
 LCD_WR_CMD(0x0013,0x1500);
 Delay(100); 
 //vom H
 LCD_WR_CMD(0x0029,0x0018); 
 LCD_WR_CMD(0x002B,0x000D); 

//gamma
 LCD_WR_CMD(0x0030,0x0004);
 LCD_WR_CMD(0x0031,0x0307);
 LCD_WR_CMD(0x0032,0x0002);// 0006
 LCD_WR_CMD(0x0035,0x0206);
 LCD_WR_CMD(0x0036,0x0408);
 LCD_WR_CMD(0x0037,0x0507); 
 LCD_WR_CMD(0x0038,0x0204);//0200
 LCD_WR_CMD(0x0039,0x0707); 
 LCD_WR_CMD(0x003C,0x0405);// 0504
 LCD_WR_CMD(0x003D,0x0F02); 
 //ram
 LCD_WR_CMD(0x0050,0x0000); 
 LCD_WR_CMD(0x0051,0x00EF);
 LCD_WR_CMD(0x0052,0x0000); 
 LCD_WR_CMD(0x0053,0x013F);  
 LCD_WR_CMD(0x0060,0xA700); 
 LCD_WR_CMD(0x0061,0x0001); 
 LCD_WR_CMD(0x006A,0x0000); 
 //
 LCD_WR_CMD(0x0080,0x0000); 
 LCD_WR_CMD(0x0081,0x0000); 
 LCD_WR_CMD(0x0082,0x0000); 
 LCD_WR_CMD(0x0083,0x0000); 
 LCD_WR_CMD(0x0084,0x0000); 
 LCD_WR_CMD(0x0085,0x0000); 
 //
 LCD_WR_CMD(0x0090,0x0010); 
 LCD_WR_CMD(0x0092,0x0600); 
 LCD_WR_CMD(0x0093,0x0003); 
 LCD_WR_CMD(0x0095,0x0110); 
 LCD_WR_CMD(0x0097,0x0000); 
 LCD_WR_CMD(0x0098,0x0000);
 LCD_WR_CMD(0x0007,0x0133);

*/	
	
//	Write_Cmd_Data(0x0022);//		
 	
 
  
	//ini();

    LCD_WR_CMD(32, 0);
    LCD_WR_CMD(33, 0x013F);
	*(__IO uint16_t *) (Bank1_LCD_C)= 34;
	for(color1=0;color1<76800;color1++)
	{
	  LCD_WR_Data(0xffff);
	}
	color1=0;
	//while(1);					
			   
	}
	else if(LCD_IDP==0x9320||LCD_IDP==0x9300)
	{
	    Delay(200);
		LCD_WR_CMD(0x00,0x0000);
		LCD_WR_CMD(0x01,0x0100);	//Driver Output Contral.
		LCD_WR_CMD(0x02,0x0700);	//LCD Driver Waveform Contral.
//		LCD_WR_REG(0x03,0x1030);	//Entry Mode Set.
		LCD_WR_CMD(0x03,0x1018);	//Entry Mode Set.
	
		LCD_WR_CMD(0x04,0x0000);	//Scalling Contral.
		LCD_WR_CMD(0x08,0x0202);	//Display Contral 2.(0x0207)
		LCD_WR_CMD(0x09,0x0000);	//Display Contral 3.(0x0000)
		LCD_WR_CMD(0x0a,0x0000);	//Frame Cycle Contal.(0x0000)
		LCD_WR_CMD(0x0c,(1<<0));	//Extern Display Interface Contral 1.(0x0000)
		LCD_WR_CMD(0x0d,0x0000);	//Frame Maker Position.
		LCD_WR_CMD(0x0f,0x0000);	//Extern Display Interface Contral 2.
	
	    Delay(200);
		LCD_WR_CMD(0x07,0x0101);	//Display Contral.
	    Delay(200);
	
		LCD_WR_CMD(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	//Power Control 1.(0x16b0)
		LCD_WR_CMD(0x11,0x0007);								//Power Control 2.(0x0001)
		LCD_WR_CMD(0x12,(1<<8)|(1<<4)|(0<<0));					//Power Control 3.(0x0138)
		LCD_WR_CMD(0x13,0x0b00);								//Power Control 4.
		LCD_WR_CMD(0x29,0x0000);								//Power Control 7.
	
		LCD_WR_CMD(0x2b,(1<<14)|(1<<4));
		
		LCD_WR_CMD(0x50,0);		//Set X Start.
		LCD_WR_CMD(0x51,239);	//Set X End.
		LCD_WR_CMD(0x52,0);		//Set Y Start.
		LCD_WR_CMD(0x53,319);	//Set Y End.
	
		LCD_WR_CMD(0x60,0x2700);	//Driver Output Control.
		LCD_WR_CMD(0x61,0x0001);	//Driver Output Control.
		LCD_WR_CMD(0x6a,0x0000);	//Vertical Srcoll Control.
	
		LCD_WR_CMD(0x80,0x0000);	//Display Position? Partial Display 1.
		LCD_WR_CMD(0x81,0x0000);	//RAM Address Start? Partial Display 1.
		LCD_WR_CMD(0x82,0x0000);	//RAM Address End-Partial Display 1.
		LCD_WR_CMD(0x83,0x0000);	//Displsy Position? Partial Display 2.
		LCD_WR_CMD(0x84,0x0000);	//RAM Address Start? Partial Display 2.
		LCD_WR_CMD(0x85,0x0000);	//RAM Address End? Partial Display 2.
	
		LCD_WR_CMD(0x90,(0<<7)|(16<<0));	//Frame Cycle Contral.(0x0013)
		LCD_WR_CMD(0x92,0x0000);	//Panel Interface Contral 2.(0x0000)
		LCD_WR_CMD(0x93,0x0001);	//Panel Interface Contral 3.
		LCD_WR_CMD(0x95,0x0110);	//Frame Cycle Contral.(0x0110)
		LCD_WR_CMD(0x97,(0<<8));	//
		LCD_WR_CMD(0x98,0x0000);	//Frame Cycle Contral.

	
		LCD_WR_CMD(0x07,0x0173);	//(0x0173)
	}
	else if(LCD_IDP==0x9331)
	{
	    Delay(200);
		LCD_WR_CMD(0x00E7, 0x1014);
		LCD_WR_CMD(0x0001, 0x0100); // set SS and SM bit   0x0100
		LCD_WR_CMD(0x0002, 0x0200); // set 1 line inversion
		LCD_WR_CMD(0x0003, 0x1030); // set GRAM write direction and BGR=1.     0x1030
		LCD_WR_CMD(0x0008, 0x0202); // set the back porch and front porch
		LCD_WR_CMD(0x0009, 0x0000); // set non-display area refresh cycle ISC[3:0]
		LCD_WR_CMD(0x000A, 0x0000); // FMARK function
		LCD_WR_CMD(0x000C, 0x0000); // RGB interface setting
		LCD_WR_CMD(0x000D, 0x0000); // Frame marker Position
		LCD_WR_CMD(0x000F, 0x0000); // RGB interface polarity
		//*************Power On sequence ****************//
		LCD_WR_CMD(0x0010, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB
		LCD_WR_CMD(0x0011, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]
		LCD_WR_CMD(0x0012, 0x0000); // VREG1OUT voltage
		LCD_WR_CMD(0x0013, 0x0000); // VDV[4:0] for VCOM amplitude
	    Delay(200);
		LCD_WR_CMD(0x0010, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB
		LCD_WR_CMD(0x0011, 0x0227); // DC1[2:0], DC0[2:0], VC[2:0]
	    Delay(200);
		LCD_WR_CMD(0x0012, 0x000C); // Internal reference voltage= Vci;
	    Delay(200);
		LCD_WR_CMD(0x0013, 0x0800); // Set VDV[4:0] for VCOM amplitude
		LCD_WR_CMD(0x0029, 0x0011); // Set VCM[5:0] for VCOMH
		LCD_WR_CMD(0x002B, 0x000B); // Set Frame Rate
	    Delay(200);
		LCD_WR_CMD(0x0020, 0x0000); // GRAM horizontal Address
		LCD_WR_CMD(0x0021, 0x0000); // GRAM Vertical Address
		// ----------- Adjust the Gamma Curve ----------//
		LCD_WR_CMD(0x0030, 0x0000);
		LCD_WR_CMD(0x0031, 0x0106);
		LCD_WR_CMD(0x0032, 0x0000);
		LCD_WR_CMD(0x0035, 0x0204);
		LCD_WR_CMD(0x0036, 0x160A);
		LCD_WR_CMD(0x0037, 0x0707);
		LCD_WR_CMD(0x0038, 0x0106);
		LCD_WR_CMD(0x0039, 0x0707);
		LCD_WR_CMD(0x003C, 0x0402);
		LCD_WR_CMD(0x003D, 0x0C0F);
		//------------------ Set GRAM area ---------------//
		LCD_WR_CMD(0x0050, 0x0000); // Horizontal GRAM Start Address
		LCD_WR_CMD(0x0051, 0x00EF); // Horizontal GRAM End Address
		LCD_WR_CMD(0x0052, 0x0000); // Vertical GRAM Start Address
		LCD_WR_CMD(0x0053, 0x013F); // Vertical GRAM Start Address
		LCD_WR_CMD(0x0060, 0x2700); // Gate Scan Line
		LCD_WR_CMD(0x0061, 0x0001); // NDL,VLE, REV
		LCD_WR_CMD(0x006A, 0x0000); // set scrolling line
		//-------------- Partial Display Control ---------//
		LCD_WR_CMD(0x0080, 0x0000);
		LCD_WR_CMD(0x0081, 0x0000);
		LCD_WR_CMD(0x0082, 0x0000);
		LCD_WR_CMD(0x0083, 0x0000);
		LCD_WR_CMD(0x0084, 0x0000);
		LCD_WR_CMD(0x0085, 0x0000);
		//-------------- Panel Control -------------------//
		LCD_WR_CMD(0x0090, 0x0010);
		LCD_WR_CMD(0x0092, 0x0600);
		LCD_WR_CMD(0x0007,0x0021);		
	    Delay(200);
		LCD_WR_CMD(0x0007,0x0061);
	    Delay(200);
		LCD_WR_CMD(0x0007,0x0133);  // 262K color and display ON
	    Delay(200);
	}
	else if(LCD_IDP==0x9919)
	{
		//*********POWER ON &RESET DISPLAY OFF
	    Delay(200);
		
		 LCD_WR_CMD(0x28,0x0006);
		
		 LCD_WR_CMD(0x00,0x0001);
		
		 LCD_WR_CMD(0x10,0x0000);
		
		 LCD_WR_CMD(0x01,0x72ef);

		 LCD_WR_CMD(0x02,0x0600);

		 LCD_WR_CMD(0x03,0x6a38);
		
		 LCD_WR_CMD(0x11,0x6874);//70
		
		 
		     //  RAM WRITE DATA MASK
		 LCD_WR_CMD(0x0f,0x0000); 
		    //  RAM WRITE DATA MASK
		 LCD_WR_CMD(0x0b,0x5308); 
		
		 LCD_WR_CMD(0x0c,0x0003);
		
		 LCD_WR_CMD(0x0d,0x000a);
		
		 LCD_WR_CMD(0x0e,0x2e00);  //0030
		
		 LCD_WR_CMD(0x1e,0x00be);
		
		 LCD_WR_CMD(0x25,0x8000);
		
		 LCD_WR_CMD(0x26,0x7800);
		
		 LCD_WR_CMD(0x27,0x0078);
		
		 LCD_WR_CMD(0x4e,0x0000);
		
		 LCD_WR_CMD(0x4f,0x0000);
		
		 LCD_WR_CMD(0x12,0x08d9);
		
		 // -----------------Adjust the Gamma Curve----//
		 LCD_WR_CMD(0x30,0x0000);	 //0007
		
		 LCD_WR_CMD(0x31,0x0104);	   //0203
		
		 LCD_WR_CMD(0x32,0x0100);		//0001

		 LCD_WR_CMD(0x33,0x0305);	  //0007

		 LCD_WR_CMD(0x34,0x0505);	  //0007
		
		 LCD_WR_CMD(0x35,0x0305);		 //0407
		
		 LCD_WR_CMD(0x36,0x0707);		 //0407
		
		 LCD_WR_CMD(0x37,0x0300);		  //0607
		
		 LCD_WR_CMD(0x3a,0x1200);		 //0106
		
		 LCD_WR_CMD(0x3b,0x0800);		 

		 LCD_WR_CMD(0x07,0x0033);
	}
	else if(LCD_IDP==0x1505)
	{
// second release on 3/5  ,luminance is acceptable,water wave appear during camera preview
	    Delay(200);
		LCD_WR_CMD(0x0007,0x0000);
	    Delay(200);
        LCD_WR_CMD(0x0012,0x011C);//0x011A   why need to set several times?
        LCD_WR_CMD(0x00A4,0x0001);//NVM
    //
        LCD_WR_CMD(0x0008,0x000F);
        LCD_WR_CMD(0x000A,0x0008);
        LCD_WR_CMD(0x000D,0x0008);
       
  //GAMMA CONTROL/
        LCD_WR_CMD(0x0030,0x0707);
        LCD_WR_CMD(0x0031,0x0007); //0x0707
        LCD_WR_CMD(0x0032,0x0603); 
        LCD_WR_CMD(0x0033,0x0700); 
        LCD_WR_CMD(0x0034,0x0202); 
        LCD_WR_CMD(0x0035,0x0002); //?0x0606
        LCD_WR_CMD(0x0036,0x1F0F);
        LCD_WR_CMD(0x0037,0x0707); //0x0f0f  0x0105
        LCD_WR_CMD(0x0038,0x0000); 
        LCD_WR_CMD(0x0039,0x0000); 
        LCD_WR_CMD(0x003A,0x0707); 
        LCD_WR_CMD(0x003B,0x0000); //0x0303
        LCD_WR_CMD(0x003C,0x0007); //?0x0707
        LCD_WR_CMD(0x003D,0x0000); //0x1313//0x1f08
	    Delay(200);
        LCD_WR_CMD(0x0007,0x0001);
        LCD_WR_CMD(0x0017,0x0001);   //Power supply startup enable
	    Delay(200);
  
  //power control//
        LCD_WR_CMD(0x0010,0x17A0); 
        LCD_WR_CMD(0x0011,0x0217); //reference voltage VC[2:0]   Vciout = 1.00*Vcivl
        LCD_WR_CMD(0x0012,0x011E);//0x011c  //Vreg1out = Vcilvl*1.80   is it the same as Vgama1out ?
        LCD_WR_CMD(0x0013,0x0F00); //VDV[4:0]-->VCOM Amplitude VcomL = VcomH - Vcom Ampl
        LCD_WR_CMD(0x002A,0x0000);  
        LCD_WR_CMD(0x0029,0x000A); //0x0001F  Vcomh = VCM1[4:0]*Vreg1out    gate source voltage??
        LCD_WR_CMD(0x0012,0x013E); // 0x013C  power supply on
           //Coordinates Control//
        LCD_WR_CMD(0x0050,0x0000);//0x0e00
        LCD_WR_CMD(0x0051,0x00EF); 
        LCD_WR_CMD(0x0052,0x0000); 
        LCD_WR_CMD(0x0053,0x013F); 
    //Pannel Image Control//
        LCD_WR_CMD(0x0060,0x2700); 
        LCD_WR_CMD(0x0061,0x0001); 
        LCD_WR_CMD(0x006A,0x0000); 
        LCD_WR_CMD(0x0080,0x0000); 
    //Partial Image Control//
        LCD_WR_CMD(0x0081,0x0000); 
        LCD_WR_CMD(0x0082,0x0000); 
        LCD_WR_CMD(0x0083,0x0000); 
        LCD_WR_CMD(0x0084,0x0000); 
        LCD_WR_CMD(0x0085,0x0000); 
  //Panel Interface Control//
        LCD_WR_CMD(0x0090,0x0013); //0x0010 frenqucy
        LCD_WR_CMD(0x0092,0x0300); 
        LCD_WR_CMD(0x0093,0x0005); 
        LCD_WR_CMD(0x0095,0x0000); 
        LCD_WR_CMD(0x0097,0x0000); 
        LCD_WR_CMD(0x0098,0x0000); 
  
        LCD_WR_CMD(0x0001,0x0100); 
        LCD_WR_CMD(0x0002,0x0700); 
        LCD_WR_CMD(0x0003,0x1030); 
        LCD_WR_CMD(0x0004,0x0000); 
        LCD_WR_CMD(0x000C,0x0000); 
        LCD_WR_CMD(0x000F,0x0000); 
        LCD_WR_CMD(0x0020,0x0000); 
        LCD_WR_CMD(0x0021,0x0000); 
        LCD_WR_CMD(0x0007,0x0021); 
	    Delay(200);
        LCD_WR_CMD(0x0007,0x0061); 
	    Delay(200);
        LCD_WR_CMD(0x0007,0x0173); 
	    Delay(200);
	}							 
	else if(LCD_IDP==0x8989)
	{
	LCD_WR_CMD(0x0000,0x0001);    Delay(200);  //打开晶振
    LCD_WR_CMD(0x0003,0xA8A4);    Delay(200);   //0xA8A4
    LCD_WR_CMD(0x000C,0x0000);    Delay(200);   
    LCD_WR_CMD(0x000D,0x080C);    Delay(200);   
    LCD_WR_CMD(0x000E,0x2B00);    Delay(200);   
    LCD_WR_CMD(0x001E,0x00B0);    Delay(200);  
    LCD_WR_CMD(0x0001,0x2B3F);    Delay(200);   //驱动输出控制320*240  0x6B3F
    LCD_WR_CMD(0x0002,0x0600);    Delay(200);
    LCD_WR_CMD(0x0010,0x0000);    Delay(200);
    LCD_WR_CMD(0x0011,0x6070);    Delay(200);        //0x4030           //定义数据格式  16位色 
    LCD_WR_CMD(0x0005,0x0000);    Delay(200);
    LCD_WR_CMD(0x0006,0x0000);    Delay(200);
    LCD_WR_CMD(0x0016,0xEF1C);    Delay(200);
    LCD_WR_CMD(0x0017,0x0003);    Delay(200);
    LCD_WR_CMD(0x0007,0x0233);    Delay(200);        //0x0233       
    LCD_WR_CMD(0x000B,0x0000);    Delay(200);
    LCD_WR_CMD(0x000F,0x0000);    Delay(200);        //扫描开始地址
    LCD_WR_CMD(0x0041,0x0000);    Delay(200);
    LCD_WR_CMD(0x0042,0x0000);    Delay(200);
    LCD_WR_CMD(0x0048,0x0000);    Delay(200);
    LCD_WR_CMD(0x0049,0x013F);    Delay(200);
    LCD_WR_CMD(0x004A,0x0000);    Delay(200);
    LCD_WR_CMD(0x004B,0x0000);    Delay(200);
    LCD_WR_CMD(0x0044,0xEF00);   Delay(200);
    LCD_WR_CMD(0x0045,0x0000);    Delay(200);
    LCD_WR_CMD(0x0046,0x013F);    Delay(200);
    LCD_WR_CMD(0x0030,0x0707);    Delay(200);
    LCD_WR_CMD(0x0031,0x0204);    Delay(200);
    LCD_WR_CMD(0x0032,0x0204);    Delay(200);
    LCD_WR_CMD(0x0033,0x0502);    Delay(200);
    LCD_WR_CMD(0x0034,0x0507);    Delay(200);
    LCD_WR_CMD(0x0035,0x0204);    Delay(200);
    LCD_WR_CMD(0x0036,0x0204);    Delay(200);
    LCD_WR_CMD(0x0037,0x0502);   Delay(200);
    LCD_WR_CMD(0x003A,0x0302);    Delay(200);
    LCD_WR_CMD(0x003B,0x0302);    Delay(200);
    LCD_WR_CMD(0x0023,0x0000);    Delay(200);
    LCD_WR_CMD(0x0024,0x0000);    Delay(200);
    LCD_WR_CMD(0x0025,0x8000);    Delay(200);
    LCD_WR_CMD(0x004f,0);        //行首址0
    LCD_WR_CMD(0x004e,0);        //列首址0
	}
	    Delay(200);
	color1=0;	
	//while(1);					

}



//+++++++++++++++++++++++LCD写字符子程序
void lcd_wr_pixel(unsigned int a, unsigned int b, unsigned int e)    
// X， Y，颜色
{
//	LCD_WR_CMD(0x02,a);
//	LCD_WR_CMD(0x03,b);  
//	LCD_WR_CMD(0x04,239);
//	LCD_WR_CMD(0x05,319);		
//	LCD_WR_REG(0x0E); 

	LCD_WR_CMD(0x20, a);
    LCD_WR_CMD(0x21, b);
	LCD_WR_Data(e);
}

void TFT_CLEAR(u8 x,u16 y,u8 len,u16 wid)	//清屏
{                    
	u32 n,temp;
    
	if(LCD_IDP==0x9325||LCD_IDP==0x9328||LCD_IDP==0x9320)    //条件编译
    {
	LCD_WR_CMD(0x0050, x); // Horizontal GRAM Start Address
	LCD_WR_CMD(0x0051, x+len-1); // Horizontal GRAM End Address
	LCD_WR_CMD(0x0052, y); // Vertical GRAM Start Address
	LCD_WR_CMD(0x0053, y+wid-1); // Vertical GRAM Start Address
	LCD_WR_REG(34);

	temp=(u32)len*wid;    
	for(n=0;n<temp;n++)LCD_WR_Data(0xffff);//显示白色 
	}

    else if(LCD_IDP==0x8989)			//条件编译		  */
	{
	   for(temp=y;temp<wid;temp++)
	    {
		   for(n=x;n<len;n++)
		   {
	        LCD_WR_CMD(0x0044,n+1);
           	LCD_WR_CMD(0x0045,temp);
           	LCD_WR_CMD(0x0046,temp+1);
           	LCD_WR_CMD(0x004e,n);
           	LCD_WR_CMD(0x004f,temp);
        	LCD_WR_REG(34);	 
	        LCD_WR_Data(0xffff);   
		   }
		}
	}
}

   
//画点
//x:0~239
//y:0~319
//POINT_COLOR:此点的颜色
void TFT_DrawPoint(u8 x,u16 y)
{

    if (LCD_IDP==0x9325||LCD_IDP==0x9328||LCD_IDP==0x9320)    //条件编译
	{
    LCD_WR_CMD(0x0050, x); // Horizontal GRAM Start Address
	LCD_WR_CMD(0x0051, 0x00EF); // Horizontal GRAM End Address
	LCD_WR_CMD(0x0052, y); // Vertical GRAM Start Address
	LCD_WR_CMD(0x0053, 0x013F); // Vertical GRAM Start Address
	LCD_WR_CMD(32, x); // Horizontal GRAM Start Address		
	LCD_WR_CMD(33, y); // Vertical GRAM Start Address
	LCD_WR_REG(34);
	LCD_WR_Data(POINT_COLOR); 
	}
    
	
	else if(LCD_IDP==0x8989)			//条件编译
	{
	LCD_WR_CMD(0x0044,x+1);
	LCD_WR_CMD(0x0045,y);
	LCD_WR_CMD(0x0046,y+1);
	LCD_WR_CMD(0x004e,x);
	LCD_WR_CMD(0x004f,y);
	LCD_WR_REG(34);		
	LCD_WR_Data(POINT_COLOR); 
	}
} 
//取绝对值函数
//位宽:32bit
u32 abs(s32 res)
{
	if(res<0)return -res;
	else return res;
}  
 
//画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
void TFT_DrawLine(u8 x1, u16 y1, u8 x2, u16 y2)
{
    u16 x, y, t;
	if((x1==x2)&&(y1==y2))TFT_DrawPoint(x1, y1);
	else if(abs(y2-y1)>abs(x2-x1))//斜率大于1 
	{
		if(y1>y2) 
		{
			t=y1;
			y1=y2;
			y2=t; 
			t=x1;
			x1=x2;
			x2=t; 
		}
		for(y=y1;y<y2;y++)//以y轴为基准 
		{
			x=(u32)(y-y1)*(x2-x1)/(y2-y1)+x1;
			TFT_DrawPoint(x, y);  
		}
	}
	else     //斜率小于等于1 
	{
		if(x1>x2)
		{
			t=y1;
			y1=y2;
			y2=t;
			t=x1;
			x1=x2;
			x2=t;
		}   
		for(x=x1;x<x2;x++)//以x轴为基准 
		{
			y =(u32)(x-x1)*(y2-y1)/(x2-x1)+y1;
			TFT_DrawPoint(x,y); 
		}
	} 
}
//6*12大小
//在指定位置显示一个字符
//x:0~234
//y:0~308
//num:要显示的字符:" "--->"~"
void TFT_ShowChar(u8 x,u16 y,u8 num)
{       
#define MAX_CHAR_POSX 232
#define MAX_CHAR_POSY 304 
    u8 temp;
    u8 pos,t;      
    if(x>MAX_CHAR_POSX||y>MAX_CHAR_POSY)return;
    //设定一个字符所占的大小 
    //开辟空间
	  
    if (LCD_IDP==0x9325||LCD_IDP==0x9328||LCD_IDP==0x9320)    //条件编译
	{
	LCD_WR_CMD(0x0050, x); // Horizontal GRAM Start Address
	LCD_WR_CMD(0x0051, x+5); // Horizontal GRAM End Address
	LCD_WR_CMD(0x0052, y); // Vertical GRAM Start Address
	LCD_WR_CMD(0x0053, y+11); // Vertical GRAM Start Address	 
	LCD_WR_CMD(32, x);
    LCD_WR_CMD(33, y);	// 
	LCD_WR_REG(34);
   	}
   
   
	else if(LCD_IDP==0x8989)			//条件编译
   {
   		LCD_WR_CMD(0x0044,(x+5<<8)+x);
	LCD_WR_CMD(0x0045,y);
	LCD_WR_CMD(0x0046,y+11);
	LCD_WR_CMD(0x004e,x);
	LCD_WR_CMD(0x004f,y);
    LCD_WR_REG(34);	     	  	 

   
 //  LCD_SetPos(x,x+5,y,y+11);//320x240
   }
	
	num=num-' ';//得到偏移后的值
	for(pos=0;pos<12;pos++)
	{
	    temp=asc2_1206[num][pos];
	    for(t=0;t<6;t++)
	    {                 
	        if(temp&0x01)LCD_WR_Data(POINT_COLOR);
	        else LCD_WR_Data(0xffff);//白色    
	        temp>>=1; 
	    }
	}
}  	 
//m^n函数
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}
//显示一个数字
//x,y:起点坐标
//num:数值(0~65536); 
void TFT_ShowNum(u8 x,u16 y,u32 num)
{      
	u32 res;   	   
	u8 t=0,t1=0;   
	res=num;
	if(!num)TFT_ShowChar(x,y,'0');//显示0
	while(res)  //得到数字长度
	{
		res/=10;
		t++;
	}
	t1=t;
	while(t)	//显示数字
	{
		res=mypow(10,t-1); 	 
	    TFT_ShowChar(x+(t1-t)*6,y,(num/res)%10+'0');
		t--;
	}				     
} 
//显示字符串
//x,y:起点坐标  
//*p:字符串起始地址
void TFT_ShowString(u8 x,u16 y,const u8 *p)
{         
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=12;}
        if(y>MAX_CHAR_POSY){y=x=0;TFT_CLEAR(0,0,240,320);}
        TFT_ShowChar(x,y,*p);
        x+=6;
        p++;
    }  
}
 /*
//显示一副图片
//x,y:图片起始位置
//lenth:图片的宽度(0~240)  
//wide: 图片高度(0~320)
// *p:图片首地址        
//不带范围判断	 
void TFT_ShowBmp(u8 x,u16 y,u8 lenth,u16 wide,const u8 *p)
{      
    u32 size,temp; 
    //开辟窗口
	
//	LCD_WR_CMD(0x0050, x); // Horizontal GRAM Start Address
//	LCD_WR_CMD(0x0051, (u16)x+lenth-1); // Horizontal GRAM End Address
//	LCD_WR_CMD(0x0052, y); // Vertical GRAM Start Address
//	LCD_WR_CMD(0x0053, y+wide-1); // Vertical GRAM Start Address  
	LCD_WR_CMD(0x0044,(lenth<<8)+x);
	LCD_WR_CMD(0x0045,y);
	LCD_WR_CMD(0x0046,wide);
	LCD_WR_CMD(0x004e,x);
	LCD_WR_CMD(0x004f,y);	//  
	
	LCD_WR_REG(34);


	//LCD_WR_CMD(0,0x2,x);//起始坐标
	//LCD_WR_CMD(1,0x3,y); 
	//LCD_WR_CMD(0,0x04,(u16)x+lenth-1);//结束列数(0~239)	
	//LCD_WR_CMD(1,0x05,y+wide-1);      //结束行数(0~319)   
	//LCD_WR_REG(0x0E);       
	temp=(u32)lenth*wide*2;
	for(size=0;size<temp;size++)LCD_WR_Data_8(p[size]); 
}         
				   */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
