#include "stm32f10x.h"
#include "SPI_Flash.h"
#include "stm32f10x_spi.h"

extern void SPI1_Init(void);
//void SPI_Flash_Init(void);
//u8 SPI_Flash_ReadByte(void);
//u8 SPI_Flash_SendByte(u8 byte);
//void FlashPageEarse(u16 page);
//void FlashPageRead(u16 page,u8 *Data);
//void FlashPageWrite(u16 page,u8 *Data);
//void FlashWaitBusy(void);
void AT45_RomTo_buf(unsigned char buffer,unsigned int page);
u8 M25P80_buf_ToRam(unsigned char buffer,unsigned int start_address,unsigned int length);
u8 M25P16_RamTo_buf(unsigned char buffer,unsigned int start_address,unsigned int length);	  
void AT45_buf_ToRom(unsigned char buffer,unsigned int page);
void AT45_page_earse(unsigned int page);
u16 SPI_ReadByte(unsigned char a);
u16 SPI_SendByte(u8 byte);

unsigned char AT45_buffer[40];
//unsigned char AT45_buffer1[10];
/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern void SPI1_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
   
  /* Enable SPI2 GPIOB clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 ,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /* Configure SPI2 pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PB.12 as Output push-pull, used as Flash Chip select */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Deselect the FLASH: Chip Select high */
  NotSelect_Flash();

  /* SPI2 configuration */ 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  
  /* Enable SPI2  */
  SPI_Cmd(SPI1, ENABLE);   
}
/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
 /*
u16 SPI_Flash_ReadByte(void)	   //flash操作基本函数，读一个字节
{
  return (SPI_Flash_SendByte(Dummy_Byte));
}	   

		*/
/*******************************************************************************
* Function Name  : SPI_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
u16 SPI_ReadByte(u8 a)
{
  return (SPI_SendByte(a));
}

/*******************************************************************************
* Function Name  : SPI_ReadByte
* Description    : Sends a byte through the SPI interface and return the byte 
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
u16 SPI_SendByte(u8 byte)
{
  /* Loop while DR register in not emplty */
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  //NotSelect_Flash();  while(1);
  /* Send byte through the SPI2 peripheral */
  SPI_I2S_SendData(SPI1, byte);

  /* Wait to receive a byte */
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}


//将DF_buffer[]数组中指定长度的数据写入
unsigned char M25P16_RamTo_buf(unsigned char buffer,unsigned int start_address,unsigned int length)
{
  unsigned int i;
  FlashPageEarse(0X02); //擦除指定的页,页范围0-4095

  FlashWaitBusy();//Flash忙检测
  Select_Flash(); //CS置"0"
  SPI_ReadByte(0X06);	//写使能
  NotSelect_Flash(); //CS置"1"

  FlashWaitBusy();//Flash忙检测
  Select_Flash(); //CS置"0"
  SPI_ReadByte(0X02);	//页写
  SPI_ReadByte((start_address & 0xFF0000) >> 16);
  SPI_ReadByte((start_address & 0xFF00) >> 8);
  SPI_ReadByte(start_address & 0xFF);
  for (i=0;i<length;i++)SPI_ReadByte(AT45_buffer[i]);

  NotSelect_Flash();
}

void FlashPageEarse(u16 page) //擦除指定的页,页范围0-4095
{	

  FlashWaitBusy();//Flash忙检测
  Select_Flash(); //CS置"0"
  SPI_ReadByte(0X06);	//写使能
  NotSelect_Flash(); //CS置"1"

  FlashWaitBusy();//Flash忙检测
  Select_Flash(); //CS置"0"
  SPI_ReadByte(SE);  //块擦除
  SPI_ReadByte((page & 0xFF0000) >> 16);
  SPI_ReadByte((page & 0xFF00) >> 8);
  SPI_ReadByte(page & 0xFF);
  NotSelect_Flash(); //CS置"1"

}

void FlashWaitBusy(void)//Flash忙检测
{

  u8 FLASH_Status = 0;

  Select_Flash();	

  SPI_ReadByte(RDSR);//读状态

  do
  {

    FLASH_Status = SPI_ReadByte(Dummy_Byte);

  }
  while ((FLASH_Status & WIP_Flag) == SET); 
  NotSelect_Flash();	   

}

//读取数据，保存在DF_buffer[]数组中
unsigned char M25P80_buf_ToRam(unsigned char buffer,unsigned int start_address,unsigned int length)
{
	unsigned int i;
		FlashWaitBusy(); //Flash忙检测
		Select_Flash();//CS置"0" 		

        SPI_ReadByte(0X03);//	读数据

        SPI_ReadByte((start_address & 0xFF0000) >> 16);
        SPI_ReadByte((start_address & 0xFF00) >> 8);
        SPI_ReadByte(start_address & 0xFF);
  		for (i=0;i<length;i++)
		{
			AT45_buffer[i+17] = SPI_ReadByte(0);
		}

		NotSelect_Flash();//CS置"1"
}
