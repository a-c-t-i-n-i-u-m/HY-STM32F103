#include "stm32f10x.h"	   
#include "sys.h"
#include "delay.h"
#include "tftlcd.h"	
//#include "usart.h" 	    
#include "touch.h"   
#include "platform_config.h"	
#include "stm32f10x_exti.h"	
#include "fsmc_sram.h"
#include "misc.h"	
#define NotSelect_Flash()    GPIO_SetBits(GPIOA, GPIO_Pin_4)

EXTI_InitTypeDef EXTI_InitStructure;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void FillColor(u8 x,u16 y, u8 x1, u16 y1, u16 z);
void Delay(__IO uint32_t nCount);
void delay_init(u8 SYSCLK);
void Touch_Start(void);
void start_spi(void);
 void numm(void);
 u8 read_once(void); //��ȡһ��X,Yֵ
void drawbigpointxx(u16 x,u16 y);//����+����

 void Read_Ads7846(void);
 void delay_ms(u16 nms);
 void delay_us(u32 Nus);
 void WriteByteADS(u8 num);
 void heyaodz_TP(void);	//���ƽ���
 void heyaodz_JF(void);	//��ʾ�ַ���
void heyaodz_TPad(void);	//������У׼

void M25P16_RamTo_buf_TP(void); //����У׼����  
void M25P80_buf_ToRam_TP(void); //��ȡ�����ֵ
void heyaodz_TPad0(void);	//����

extern void SPI_Flash_Init(void);
extern u8 SPI_Flash_ReadByte(void);
extern u8 SPI_Flash_SendByte(u8 byte);
extern void FlashPageEarse(u16 page);
extern void FlashPageRead(u16 page,u8 *Data);
extern void FlashPageWrite(u16 page,u8 *Data);
extern void FlashWaitBusy(void);
extern void AT45_RomTo_buf(unsigned char buffer,unsigned int page);
extern u8 M25P80_buf_ToRam(unsigned char buffer,unsigned int start_address,unsigned int length);
extern u8 M25P16_RamTo_buf(unsigned char buffer,unsigned int start_address,unsigned int length);	  
extern void AT45_buf_ToRom(unsigned char buffer,unsigned int page);
extern void AT45_page_earse(unsigned int page);

extern void LCD_Init(void);
extern void LCD_test(void);
void drawbigpoint(u8 x,u16 y);	//��һ�����
extern unsigned char AT45_buffer[];

unsigned char _it0=0,_it1=0,_it2=0,_it3=0,num=0;
unsigned char dw=0,a=0,b=0,TP_B=0;
unsigned int Xs_1=0,Xs_2=0,Xs_3=0,Xs_4=0,Ys_1=0,Ys_2=0,Ys_3=0,Ys_4=0; //��¼������ֵ
unsigned int X=0,Y=0;	   //X,Y����
extern u16 POINT_COLOR;  
float X2=0,Y2=0;
		 		 
int main()
{//	

	RCC_Configuration();   

  /* Enable the FSMC Clock */
    
  	delay_init(72);
  	
    GPIO_Configuration();	  
	SPI1_Init();
//	SPI1_Init25();
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);	  
	FSMC_LCD_Init();	//FSMC��ʼ��								 
    LCD_Init();	 //LCD��ʼ��
	//delay_ms(800);   	
//	while(1);
	NVIC_Configuration();

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);		 //����IRQ

  /* Configure Key Button EXTI Line to generate an interrupt on falling edge */  
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	//while(1) a=read_once();
	/* NVIC configuration */
    
	M25P80_buf_ToRam_TP();	 //��ȡ����Ĵ�����У׼ֵ
	heyaodz_TPad0();

	//touch_init();               //��������ʼ��  
	while (1)                   //�ȴ��жϵ���
  	{    						   
		if(_it1==1){
			delay_ms(50);
			a=PEN;
			while(a==0){
			    //delay_us(10);
				Read_Ads7846();
				a=PEN;
			}
			_it1=0;	
		
		}
		//delay_ms(10);		  
  	}   	 
}

void heyaodz_TPad0(void)	//����
{
	POINT_COLOR=RED;
	TFT_CLEAR(0,0,240,320);     //����   	
	heyaodz_TP();	//���ƽ���
}

///////////////////////

void M25P80_buf_ToRam_TP(void) //��ȡ�����ֵ
{
	TCS_SET(1); 
    M25P80_buf_ToRam(1,2,17); //��ȡ����  
	TP_B=AT45_buffer[17]; // ʶ����

	Xs_1=(AT45_buffer[18]<<8)+AT45_buffer[19];
	Ys_1=(AT45_buffer[20]<<8)+AT45_buffer[21];

	Xs_2=(AT45_buffer[22]<<8)+AT45_buffer[23];
	Ys_2=(AT45_buffer[24]<<8)+AT45_buffer[25];

	Xs_3=(AT45_buffer[26]<<8)+AT45_buffer[27];
	Ys_3=(AT45_buffer[28]<<8)+AT45_buffer[29];

	Xs_4=(AT45_buffer[30]<<8)+AT45_buffer[31];
	Ys_4=(AT45_buffer[32]<<8)+AT45_buffer[33];
	
	dw=1;

}

 
void heyaodz_JF(void)	//��ʾ�ַ���
{
 	POINT_COLOR=BLUE;
	TFT_ShowString(80,70,"STM32 TOUCH TEST");
	TFT_ShowString(80,80,"STM32-HY-heyaodz");
	TFT_ShowString(80,90,"2011/5/31"); 
	TFT_ShowString(80,110,"Init...."); 
	TFT_ShowString(80,130,"Please Wait..."); 	
}
void heyaodz_TP(void)	//���ƽ���
{

	FillColor(1,1,20,20,RED); 
	FillColor(20,1,40,20,BLUE); 
	FillColor(40,1,60,20,GREEN); 
	FillColor(60,1,80,20,GRED); 
	FillColor(80,1,100,20,BRED); 
	FillColor(100,1,120,20,0); 	 
	TFT_ShowString(210,2,"CLR");	


}

void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
   
  /* Enable Key Button GPIO Port, GPIO_LED and AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}


/////////////////////////

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 

// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_6|GPIO_Pin_3;		 
//  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_2|GPIO_Pin_3;		 
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 //COL4
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_SetBits(GPIOC, GPIO_Pin_5|GPIO_Pin_2|GPIO_Pin_3);
  GPIO_SetBits(GPIOE, GPIO_Pin_6);	     

    
  GPIO_SetBits(GPIOB, GPIO_Pin_5);	     //
  GPIO_SetBits(GPIOD, GPIO_Pin_6|GPIO_Pin_3 );	    
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 	 //LCD-RST
  GPIO_Init(GPIOE, &GPIO_InitStructure);  	
    
 
 

  //PB10, PD9, PD10   CS,SI,CLK
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //�������
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7 ;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;           //SPI1 CS1 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           //SPI1 CS4 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;           //SPI1 NSS 
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  	
  //PENIRQ, SO	
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //GPIO_Init(GPIOB, &GPIO_InitStructure);
  


  
  	
  
/*-- GPIO Configuration ------------------------------------------------------*/  
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
     PE.14(D11), PE.15(D12) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 


  
  /* NE1 configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /* RS */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

  
  GPIO_SetBits(GPIOD, GPIO_Pin_7);			//CS=1 
  GPIO_SetBits(GPIOD, GPIO_Pin_14| GPIO_Pin_15 |GPIO_Pin_0 | GPIO_Pin_1);  	 
  GPIO_SetBits(GPIOE, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10);   
  GPIO_ResetBits(GPIOE, GPIO_Pin_0);
  GPIO_ResetBits(GPIOE, GPIO_Pin_1);		//RESET=0
  GPIO_SetBits(GPIOD, GPIO_Pin_4);		    //RD=1
  GPIO_SetBits(GPIOD, GPIO_Pin_5);			//WR=1

  GPIO_SetBits(GPIOB, GPIO_Pin_6);			//PEn
  //GPIO_SetBits(GPIOA, GPIO_Pin_6);			//PEn
  GPIO_SetBits(GPIOD, GPIO_Pin_13);			//LIGHT
  //GPIO_SetBits(GPIOB, GPIO_Pin_7);			//SPI CS3
  GPIO_SetBits(GPIOC, GPIO_Pin_4);			//SPI CS1
  GPIO_SetBits(GPIOB, GPIO_Pin_12);			//SPI CS4
  GPIO_SetBits(GPIOA, GPIO_Pin_4);			//SPI NSS
}



//////////////////////////////////////
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* Enable the EXTI9_5 Interrupt */
  

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn  ;

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

// ++++++++++++++++TFT ��λ����
void lcd_rst(void){
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    Delay(0x1FFFFf);					   
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );		 	 
	Delay(0x1fFFFf);	
}

void start_spi(void)   
{ 	  
	//TCLK_SET(0);   
	//TCS_SET(1); 	 
	//TDIN_SET(1);  
	//sTCLK_SET(1);
	//SPI_Flash_SendByte(1);  
	TCS_SET(0);   
}
//SPIд����
//��7846д��1byte����   
void WriteByteADS(u8 num)    
{  
	u8 count=0;  
	TCLK_SET(0);   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80)TDIN_SET(1);  
		else TDIN_SET(0);   
		num<<=1;    
		TCLK_SET(0);//��������Ч  	 
		TCLK_SET(1);      
	} 			    
} 
//SPI������ 
//��7846��ȡadcֵ	   
u16 ReadWordADS(void)	  
{ 	 
	u8 count=0; 	  
	u16 Num=0; 	 
	for(count=0;count<12;count++)  
	{ 				  
		Num<<=1; 	 
		TCLK_SET(1);//�½�����Ч 	   
		TCLK_SET(0);    
		if(DOUT)Num++; 		 
	} 		  
	return(Num);   
}
#define CMD_RDX 0X90  //0B10010000���ò�ַ�ʽ��X����
#define CMD_RDY	0XD0  //0B11010000���ò�ַ�ʽ��Y���� 

 
//��ȡһ��X,Yֵ
//������X,Y����ֵ���붼����100
//�ɹ�����1,���ɹ�����0
//����������100~4000֮��.
u8 read_once(void)
{	unsigned int a,b;
	TCS_SET(0); 
	delay_us(5);	   	 
	SPI_SendByte(CMD_RDY); 
	delay_us(5);	
	a=SPI_ReadByte(0);
	a=a<<8;
	a|=SPI_ReadByte(0);
	delay_us(5);	
	TCS_SET(1); 	 
	a>>=3; 
	Y=a;
	delay_us(15);	
	TCS_SET(0); 
	delay_us(5);	 
    SPI_SendByte(CMD_RDX);
	delay_us(5);	
	b=SPI_ReadByte(0);
	b=b<<8;
	b|=SPI_ReadByte(0);
	delay_us(5);	
	b>>=3; 
	X=b;   

	TCS_SET(1); 
	if(X>100&&Y>100&&X<4000&&Y<4000)return 1;//�����ɹ�(��Χ����)
	else return 0;			                 //����ʧ��
}

//��һ��"+"
void drawbigpointxx(u16 x,u16 y)
{
POINT_COLOR=RED;
TFT_DrawLine(x-11,y,x+12,y+1);
TFT_DrawLine(x-11,y+1,x+12,y+2);
TFT_DrawLine(x,y-11,x+1,y+12);
TFT_DrawLine(x+1,y-11,x+2,y+12);

}

//��һ�����
//2*2�ĵ�
//��������"��ť"RST
void drawbigpoint(u8 x,u16 y)
{
	if((x<1||x>238)||(y<1||y>318)){y=0;x=0;}
	if(x>215&&y<20)
	{
		TFT_CLEAR(0,20,240,320);//���� 

//	    heyaodz_TP();	//���ƽ���
	}
	else if(y==0||x==0){
		TFT_DrawPoint(x,y);//���ĵ� 
		TFT_DrawPoint(x+1,y);
		TFT_DrawPoint(x,y+1);
		TFT_DrawPoint(x+1,y+1);	
	}		
	else if(x<120&&y<20)								
	{


	    if(x<=20) POINT_COLOR=RED;
		else if(x>20,x<=40) POINT_COLOR=BLUE;
		else if(x>40,x<=60) POINT_COLOR=GREEN;
		else if(x>60,x<=80) POINT_COLOR=GRED;
		else if(x>80,x<=100) POINT_COLOR=BRED;	
		else if(x>100,x<=120) POINT_COLOR=BLACK;
	}
	else if(y>20){
		TFT_DrawPoint(x,y);//���ĵ� 
		TFT_DrawPoint(x+1,y);
		TFT_DrawPoint(x,y+1);
		TFT_DrawPoint(x+1,y+1);	
	}		  	
}

void FillColor(u8 x,u16 y, u8 x1, u16 y1, u16 z)
{	u16 a,b;
	POINT_COLOR=z; 
	for(a=0; a<(y1-y); a++)
	{
	  for(b=0; b<(x1-x); b++)
	  {
	   TFT_DrawPoint(x+b,y+a);
	  }
	}	  	
}
 
void Read_Ads7846(void)
{	float X1,Y1,hh;
    u16 x1,x2,y1,y2,xx;
	u8 t,t1,count=0;
	u16 databuffer[2][10];//������
	u16 temp=0;	 
    do					  //ѭ������10��
	{
		//t=PEN;		   
		if(read_once())//�����ɹ�
		{	  
			databuffer[0][count]=X;
			databuffer[1][count]=Y;
			count++;  
		}
		t=PEN;
	}while(!t&&count<10); 
	//}while(count<10); 
	if(count==10)//һ��Ҫ����10������,������
	{  
	    do//������X��������
		{	
			t1=0;		  
			for(t=0;t<count-1;t++)
			{
				if(databuffer[0][t]>databuffer[0][t+1])//��������
				{
					temp=databuffer[0][t+1];
					databuffer[0][t+1]=databuffer[0][t];
					databuffer[0][t]=temp;
					t1=1; 
				}  
			}
		}while(t1); 	  
		do//������Y��������
		{	
			t1=0;		 
			for(t=0;t<count-1;t++)
			{
				if(databuffer[1][t]>databuffer[1][t+1])//��������
				{
					temp=databuffer[1][t+1];
					databuffer[1][t+1]=databuffer[1][t];
					databuffer[1][t]=temp;
					t1=1;	 
				}  
			}
		}while(t1);
		x1=databuffer[0][3]; x2=databuffer[0][4]; //x3=databuffer[0][8];		
		y1=databuffer[1][3]; y2=databuffer[1][4]; //y3=databuffer[1][8];	   
		if(((x1>x2)&&(x1>x2+30))||((x2>x1)&&(x2>x1+30))||((y1>y2)&&(y1>y2+30))||((y2>y1)&&(y2>y1+30)));	 		  
//		if(0);
		else
		{
        
		X1=(databuffer[0][3]+databuffer[0][4])/2;
		Y1=(databuffer[1][3]+databuffer[1][4])/2;	
		
			if(X1<=4096&&Y1<=4096) //���˵������ݳ�ʼ�����޸�.		  ����
			{	X=X1;
			    Y=Y1;


              if(dw==1)
			  { 
			   		if((Xs_1<Xs_3)&&(Ys_1<Ys_2))	//���½�Ϊ��ʼ����ʱ����
					{
						 if((Ys_1*5<Ys_2)&&(Xs_1*5<Xs_3))	//�жϴ�������
						 {
						   xx=Ys_2-Ys_1;	//�����Y1,Y2����
						   hh=(float)xx/200;		//�����X����
						   xx=(float)((hh*240)-xx)/2;	//Y����ƫ����
						   X=(Y1-(Ys_1-xx))/hh;		  //��������
						   xx=Xs_3-Xs_1;	//�����X1,X3����
						   hh=(float)xx/280;	//�����X��������
						   xx=(float)((hh*320)-xx)/2;  //X����ƫ����
						   Y=(X1-(Xs_1-xx))/hh;	//��������
						   drawbigpoint(X,Y);
						 }
						 else
						 {
						   xx=Ys_2-Ys_1;	//�����Y1,Y2����
						   hh=(float)xx/280;		//�����X����
						   xx=(float)((hh*320)-xx)/2;	//Y����ƫ����
						   Y=(Y1-(Ys_1-xx))/hh;		  //��������
						   xx=Xs_3-Xs_1;	//�����X1,X3����
						   hh=(float)xx/200;	//�����X��������
						   xx=(float)((hh*240)-xx)/2;  //X����ƫ����
						   X=(X1-(Xs_1-xx))/hh;	//��������
						   drawbigpoint(X,Y);
						 }
						 
					}
			   		
					if((Xs_2<Xs_4)&&(Ys_2<Ys_1))	//���Ͻ�Ϊ��ʼ����ʱ����
					{
						 if((Ys_2*5<Ys_1)&&(Xs_2*5<Xs_4))	//�жϴ�������
						 {
						   xx=Ys_1-Ys_2;	//�����Y1,Y2����
						   hh=(float)xx/200;		//�����X����
						   xx=(float)((hh*240)-xx)/2;	//Y����ƫ����
						   X=240-(Y1-(Ys_2-xx))/hh;		  //��������
						   xx=Xs_4-Xs_2;	//�����X2,X4����
						   hh=(float)xx/280;	//�����X��������
						   xx=(float)((hh*320)-xx)/2;  //X����ƫ����
						   Y=(X1-(Xs_2-xx))/hh;	//��������
						   drawbigpoint(X,Y);
						 }
						 else
						 {
						   xx=Ys_1-Ys_2;	//�����Y1,Y2����
						   hh=(float)xx/280;		//�����X����
						   xx=(float)((hh*320)-xx)/2;	//Y����ƫ����
						   Y=320-(Y1-(Ys_2-xx))/hh;		  //��������
						   xx=Xs_4-Xs_2;	//�����X2,X4����
						   hh=(float)xx/200;	//�����X��������
						   xx=(float)((hh*240)-xx)/2;  //X����ƫ����
						   X=(X1-(Xs_2-xx))/hh;	//��������
						   drawbigpoint(X,Y);
						 }
					}

			   		if((Xs_3<Xs_1)&&(Ys_3<Ys_4))	//���½�Ϊ��ʼ����ʱ����
					{
						 if((Ys_3<Ys_4)&&(Xs_3<Xs_1))	//�жϴ�������
						 {
						   xx=Ys_4-Ys_3;	//�����Y3,Y4����
						   hh=(float)xx/200;		//�����X����
						   xx=(float)((hh*240)-xx)/2;	//Y����ƫ����
						   X=(Y1-(Ys_3-xx))/hh;		  //��������
						   xx=Xs_1-Xs_3;	//�����X1,X3����
						   hh=(float)xx/280;	//�����X��������
						   xx=(float)((hh*320)-xx)/2;  //X����ƫ����
						   Y=320-(X1-(Xs_3-xx))/hh;	//��������
						   drawbigpoint(X,Y);
						 }
						 else
						 {
						   xx=Ys_4-Ys_3;	//�����Y3,Y4����
						   hh=(float)xx/280;		//�����X����
						   xx=(float)((hh*320)-xx)/2;	//Y����ƫ����
						   Y=(Y1-(Ys_3-xx))/hh;		  //��������
						   xx=Xs_1-Xs_3;	//�����X1,X3����
						   hh=(float)xx/200;	//�����X��������
						   xx=(float)((hh*240)-xx)/2;  //X����ƫ����
						   X=240-(X1-(Xs_3-xx))/hh;	//��������
						   drawbigpoint(X,Y);
						 }
					}

			   		if((Xs_4<Xs_2)&&(Ys_4<Ys_3))	//���Ͻ�Ϊ��ʼ����ʱ����
					{
						 if((Ys_4<Ys_3)&&(Xs_4<Xs_2))	//�жϴ�������
						 {
						   xx=Ys_3-Ys_4;	//�����Y3,Y4����
						   hh=(float)xx/200;		//�����X����
						   xx=(float)((hh*240)-xx)/2;	//Y����ƫ����
						   X=240-(Y1-(Ys_4-xx))/hh;		  //��������
						   xx=Xs_2-Xs_4;	//�����X2,X4����
						   hh=(float)xx/280;	//�����X��������
						   xx=(float)((hh*320)-xx)/2;  //X����ƫ����
						   Y=320-(X1-(Xs_4-xx))/hh;	//��������
						   drawbigpoint(X,Y);
						 }
						 else
						 {
						   xx=Ys_3-Ys_4;	//�����Y3,Y4����
						   hh=(float)xx/280;		//�����X����
						   xx=(float)((hh*320)-xx)/2;	//Y����ƫ����
						   Y=320-(Y1-(Ys_4-xx))/hh;		  //��������
						   xx=Xs_2-Xs_4;	//�����X2,X4����
						   hh=(float)xx/200;	//�����X��������
						   xx=(float)((hh*240)-xx)/2;  //X����ƫ����
						   X=240-(X1-(Xs_4-xx))/hh;	//��������
						   drawbigpoint(X,Y);
						 }
					}


			  }  // 				*/
			   }  // 

		}   
	}
}	 

 void delay_init(u8 SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//ѡ���ڲ�ʱ�� HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}								    
//��ʱNms
//ע��Nms�ķ�Χ
//Nms<=0xffffff*8/SYSCLK
//��72M������,Nms<=1864 
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
}   
//��ʱus								   
void delay_us(u32 Nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=Nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	    
}
