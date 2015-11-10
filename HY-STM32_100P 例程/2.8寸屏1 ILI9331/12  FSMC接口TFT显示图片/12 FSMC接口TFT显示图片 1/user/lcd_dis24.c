/*******************************************

				  2.4寸 QVGA显示驱动程序







**********************************************/


#include "fsmc_sram.h"

#define Bank1_LCD_D    ((uint32_t)0x60020000)    //disp Data ADDR
#define Bank1_LCD_C    ((uint32_t)0x60000000)	 //disp Reg ADDR

unsigned long color1=0;
//void MUC_Init();
void LCD_Init(void);
void LCD_WR_REG(unsigned int index);
void LCD_WR_CMD(unsigned int index,unsigned int val);

void LCD_WR_Data(unsigned int val);
void LCD_WR_Data_8(unsigned int val);
void LCD_test(void);
void LCD_clear(unsigned int p);
void ini(void);

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
	//a=(*(__IO uint16_t *) (Bank1_LCD_D)); 	//Dummy
	//a= *(__IO uint16_t *) (Bank1_LCD_D);  	//H
	//a=a<<8;
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


//初始化函数
void LCD_Init(void)
{
	lcd_rst();	 


	    Delay(200);
//############# void Power_Set(void) ################//
 LCD_WR_CMD(0x00E7, 0x1014);
//LCD_WR_CMD(0x0001, 0x0100); // set SS and SM bit s720->s1(左上角为坐标原点)
    //LCD_WR_CMD(0x0060, 0xA700);	// R01中的SS和R60中的GS一起决定坐标原点

  LCD_WR_CMD(0x0001, 0x0000); // set SS and SM bit
  LCD_WR_CMD(0x0002, 0x0200); // set 1 line inversion
  LCD_WR_CMD(0x0003,0X1018);//65K    
  //LCD_WriteReg(0x0003, 0x1030); // set GRAM write direction and BGR=1.
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
	Delay(0xAFFf);	
  LCD_WR_CMD(0x0010, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB
  LCD_WR_CMD(0x0011, 0x0227); // DC1[2:0], DC0[2:0], VC[2:0]
	Delay(0xAFFf);	
  LCD_WR_CMD(0x0012, 0x000C); // Internal reference voltage= Vci;
	Delay(0xAFFf);	
  LCD_WR_CMD(0x0013, 0x0800); // Set VDV[4:0] for VCOM amplitude
  LCD_WR_CMD(0x0029, 0x0011); // Set VCM[5:0] for VCOMH
  LCD_WR_CMD(0x002B, 0x000B); // Set Frame Rate
	Delay(0xAFFf);	
  LCD_WR_CMD(0x0020, 0x0000); // GRAM horizontal Address
  LCD_WR_CMD(0x0021, 0x013f); // GRAM Vertical Address
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
  LCD_WR_CMD(0x0060, 0xA700); // Gate Scan Line
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
  LCD_WR_CMD(0x0007, 0x0133); // 262K color and display ON

	color1=0;

//	ini();
}

void ini(void){
  LCD_WR_CMD(1,  0x0000); /* set SS and SM bit */




}
//+++++++++++++++++++++++LCD写字符子程序
void lcd_wr_zf(unsigned int a, unsigned int b, unsigned int a1, unsigned int b1, unsigned int d,unsigned int e,unsigned char g,unsigned char *f)    
// X， Y，结束X，y,尺寸，颜色，方向，数据
{   
	

	unsigned int temp=0,num,z,R_dis_mem=0,a2=0,b2=0,b3=0;
	unsigned char temp5;

	if(g==0) LCD_WR_CMD(0x0003,0x1030);		 //左上起
	else if(g==1) LCD_WR_CMD(0x0003,0x1018);   //左下起
	else if(g==2) LCD_WR_CMD(0x0003,0x1010);   //右下起
	else if(g==3) LCD_WR_CMD(0x0003,0x1028);   //右上起
	//LCD_WR_CMD(0x01,0x07);  
	Delay(200); 
	for(temp=0;temp<d;temp++)
	{
	   b2=(temp*8);
	   b3=b2/(a1-a+1);	    //计算纵坐标偏移
	   a2=b2%(a1-a+1);	    //计算横坐标偏移
	   temp5=*f;
	   z=temp5;
	   for(num=0; num<8; num++){		    
		  if((temp5&0x80)>0){			  	
		  	if(g==0){
				LCD_WR_CMD(80,a+a2+num);
	 			LCD_WR_CMD(82,b+b3);  

				LCD_WR_CMD(81,a+a2+num);
	 			LCD_WR_CMD(83,b+b3);  
				LCD_WR_CMD(32, a+a2+num);
    			LCD_WR_CMD(33, b+b3);
	 			//LCD_WR_CMD(0,0x04,239);
	  			//LCD_WR_CMD(1,0x05,319);
			}
			else if(g==1){ 

                

				LCD_WR_CMD(80,b+b3);
	  			LCD_WR_CMD(82,a);
				LCD_WR_CMD(81,b+b3);	
				LCD_WR_CMD(83,319-(a+a2+num));
				//LCD_WR_CMD(83,a);
				LCD_WR_CMD(32,b+b3);
	  			LCD_WR_CMD(33,319-(a+a2+num));		 			
			}
			else if(g==2){ 
				LCD_WR_CMD(80,a);
	 			LCD_WR_CMD(82,b);  
	 			LCD_WR_CMD(81,239-(a+a2+num));
	  			LCD_WR_CMD(83,319-(b+b3));
				LCD_WR_CMD(32,239-(a+a2+num));
	  			LCD_WR_CMD(33,319-(b+b3));
			
			}
			else if(g==3){ 
				LCD_WR_CMD(80,239-(b+b3));
	  			LCD_WR_CMD(82,(a+a2+num));
				LCD_WR_CMD(81,239-(b+b3));	
				LCD_WR_CMD(83,319);	
				LCD_WR_CMD(32,239-(b+b3));
	  			LCD_WR_CMD(33,(a+a2+num));			
			}
	   		LCD_WR_REG(34); 
			LCD_WR_Data(e); 
		  }
		  else{
		   
	
			if(g==0){
				LCD_WR_CMD(80,a+a2+num);
				LCD_WR_CMD(82,b+b3);
				LCD_WR_CMD(81,a+a2+num);
	 			LCD_WR_CMD(83,b+b3);  
				LCD_WR_CMD(32, a+a2+num);
    			LCD_WR_CMD(33, b+b3);
				LCD_WR_REG(34);				
		  		R_dis_mem=LCD_RD_data();		  		
				LCD_WR_CMD(32,a+a2+num);
				LCD_WR_CMD(33,b+b3);
			}
			else if(g==1){


               

				LCD_WR_CMD(80,b+b3);
	  			LCD_WR_CMD(82,a);
				LCD_WR_CMD(81,b+b3);	
				LCD_WR_CMD(83,319-(a+a2+num));
				
				LCD_WR_CMD(32,b+b3);
	  			LCD_WR_CMD(33,319-(a+a2+num));
				LCD_WR_REG(34);				
		  		R_dis_mem=LCD_RD_data();		  		
				LCD_WR_CMD(32,b+b3);
	  			LCD_WR_CMD(33,319-(a+a2+num));
				//LCD_WR_CMD(0,0x04,b+b3);	
				//LCD_WR_CMD(1,0x05,319-(a+a2+num));	
			}
			else if(g==2){
				LCD_WR_CMD(80,a);
	 			LCD_WR_CMD(82,b);  
	 			LCD_WR_CMD(81,239-(a+a2+num));
	  			LCD_WR_CMD(83,319-(b+b3));
				LCD_WR_CMD(32,239-(a+a2+num));
	  			LCD_WR_CMD(33,319-(b+b3));
				LCD_WR_REG(34);				
		  		R_dis_mem=LCD_RD_data();		  		
				LCD_WR_CMD(32,239-(a+a2+num));
	  			LCD_WR_CMD(33,319-(b+b3));
	 			//LCD_WR_CMD(0,0x04,239-(a+a2+num));
	  			//LCD_WR_CMD(1,0x05,319-(b+b3));
			}
			else if(g==3){
				LCD_WR_CMD(80,239-(b+b3));
	  			LCD_WR_CMD(82,(a+a2+num));
				LCD_WR_CMD(81,239-(b+b3));	
				LCD_WR_CMD(83,319);		 
				LCD_WR_CMD(32,239-(b+b3));
	  			LCD_WR_CMD(33,(a+a2+num));
				LCD_WR_REG(34);				
		  		R_dis_mem=LCD_RD_data();		  		
				LCD_WR_CMD(80,239-(b+b3));
	  			LCD_WR_CMD(82,(a+a2+num));
				//LCD_WR_CMD(0,0x04,239-(b+b3));	
				//LCD_WR_CMD(1,0x05,319);		 
			}

			LCD_WR_REG(34); 					
			LCD_WR_Data(R_dis_mem);	 			
		  }
		  temp5=z;
		  temp5=temp5<<1; 
		  z=temp5;
		  	
	   }
	   f++;
	}
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

//演示程序
void LCD_test(void)
{
	unsigned int  temp=0;
	unsigned char *p;
	unsigned long n;

	//LCD_WR_CMD(0x20, 0xa4);
    //LCD_WR_CMD(0x21,0x07);

	/*LCD_WR_CMD(0,0x00,0xa4);
	LCD_WR_CMD(0,0x01,0x07);
	LCD_WR_CMD(0,0x02,0);        //0-239
	LCD_WR_CMD(1,0x03,0);        //0-319
	
    LCD_WR_CMD(0,0x04,239);        //0-239
	LCD_WR_CMD(1,0x05,319);        //0-319
	 */
	

	//LCD_WR_REG(0x0E);
		n=0;
		//LCD_WR_CMD(32, 0);
    	//LCD_WR_CMD(33, 0x013F);
		//*(__IO uint16_t *) (Bank1_LCD_C)= 34;
		LCD_WR_CMD(0x0003,0x1018);   //左下起
		LCD_WR_CMD(0x0050, 0); // Horizontal GRAM Start Address
		LCD_WR_CMD(0x0051, 239); // Horizontal GRAM End Address
		LCD_WR_CMD(0x0052, 0); // Vertical GRAM Start Address
		LCD_WR_CMD(0x0053, 319); // Vertical GRAM Start Address	 
		LCD_WR_CMD(32, 0);
    	LCD_WR_CMD(33, 0);
		//*(__IO uint16_t *) (Bank1_LCD_C)= 34;
		LCD_WR_REG(34);
		while(n<153600)
		{
		     //while(1){

			temp=(uint16_t)( a2[n]<<8)+a2[n+1];
			  //temp++;	
			  //LCD_WR_CMD(0x20, 0xa4);			
			LCD_WR_Data(temp);
			n=n+2;
			// }
			 //LCD_WR_Data(color1);
	 	}	
	Delay(0xafffff);

    lcd_wr_zf(0,0,239,319,870,color1,1,&zf3[0]);  
	p=num_pub((color1/10000));
	lcd_wr_zf(0,30,23,61,96,color1,0,p);  
	
	p=num_pub((color1%10000)/1000);
	lcd_wr_zf(24,30,47,61,96,color1,0,p);
	
	p=num_pub(((color1%10000)%1000)/100);
	lcd_wr_zf(48,30,71,61,96,color1,0,p);

	p=num_pub((((color1%10000)%1000)%100)/10);
	lcd_wr_zf(72,30,95,61,96,color1,0,p);

	p=num_pub((color1%10));
	lcd_wr_zf(96,30,119,61,96,color1,0,p);

	color1++; 
	if(color1==65536) color1=0;  
	Delay(0xaffff);			
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned char *num_pub(unsigned  int a){
	unsigned char *b;
	switch(a){
		case 0x01: b=&zm1[0]; break;
 		case 0x02: b=&zm2[0]; break;
		case 0x03: b=&zm3[0]; break;
		case 0x04: b=&zm4[0]; break;
		case 0x05: b=&zm5[0]; break;
		case 0x06: b=&zm6[0]; break;
		case 0x07: b=&zm7[0]; break;
		case 0x08: b=&zm8[0]; break;
		case 0x09: b=&zm9[0]; break;
		case 0x00: b=&zm0[0]; break;
		default: b=&zm0[0];break;
	}
	return(b);
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
