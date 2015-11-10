/**************************************************************************************
//------------------ MMC/SD-Card Reading and Writing implementation -------------------
//FileName     : mmc.c
//Function     : Connect AVR to MMC/SD 
//Created by   : ZhengYanbo
//Created date : 15/08/2005
//Version      : V1.2
//Last Modified: 19/08/2005
//Filesystem   : Read or Write MMC without any filesystem

//CopyRight (c) 2005 ZhengYanbo
//Email: Datazyb_007@163.com
****************************************************************************************/

#ifndef _MMC_H_
#define _MMC_H_

//If use MMC-Card then set to 1
#define	USE_MMC 		1       //EXPERIMENTAL NOT READY!!!! 
#define HW_SPI_Mode     1       //1:hardware SPI 0:software SPI

#define MMC_Write		    PORTA	//SPI port register
#define MMC_Read		    PINA    //Data PIN
#define MMC_Direction_REG	DDRA    //Direction register

#define SPI_DI			6	//-->MMC_DO_PIN
#define SPI_DO			1	//-->MMC_DI_PIN
#define SPI_Clock		4	//-->MMC_CLK_PIN
#define MMC_Chip_Select	0	//-->MMC_CS_PIN 
#define SPI_BUSY        7   //busy led

#define MMC_DO_PIN      MMC_Read.SPI_DI
#define MMC_DI_PIN      MMC_Write.SPI_DO
#define MMC_CLK_PIN     MMC_Write.SPI_Clock
#define MMC_CS_PIN      MMC_Write.MMC_Chip_Select

#define MMC_BUSY_LED    MMC_Write.SPI_BUSY //busy led for spi port

//--------------------------------------------------------------
// Hardwre SPI port define. Here for ATmega162
//--------------------------------------------------------------
#define HW_SPI_DO       5
#define HW_SPI_DI  		6
#define HW_SPI_SCK 		7
#define HW_SPI_SS  		4

#define HW_SPI_Direction_REG  DDRB
#define HW_SPI_PORT_REG       PORTB

#define SPI_SS_PIN      HW_SPI_PORT_REG.HW_SPI_SS

//------------------------------------------------------------
// Error define
//-------------------------------------------------------------
#define INIT_CMD0_ERROR     0x01
#define INIT_CMD1_ERROR		0x02
#define WRITE_BLOCK_ERROR	0x03
#define READ_BLOCK_ERROR   	0x04 
//-------------------------------------------------------------
// data type
//-------------------------------------------------------------   
// this structure holds info on the MMC card currently inserted 
typedef struct MMC_VOLUME_INFO
{ //MMC/SD Card info
  word   size_MB;
  byte   sector_multiply;
  word   sector_count;
  byte   name[6];
} VOLUME_INFO_TYPE; 


typedef struct STORE 
{ 
  byte   data[256]; 
} BUFFER_TYPE; //256 bytes, 128 words

BUFFER_TYPE sectorBuffer; //512 bytes for sector buffer

//--------------------------------------------------------------
	word	readPos=0;
	byte	sectorPos=0;
	byte    LBA_Opened=0; //Set to 1 when a sector is opened.
    byte    Init_Flag;    //Set it to 1 when Init is processing.
//---------------------------------------------------------------
// Prototypes
//---------------------------------------------------------------
void MMC_Port_Init(void);

unsigned char Read_Byte_MMC(void);
//unsigned char Read_Byte_MMC_Long(void);

void Write_Byte_MMC(unsigned char value);
//void Write_Byte_MMC_Long(unsigned char value);

unsigned char MMC_Read_Block(unsigned char *CMD,unsigned char *Buffer,unsigned int Bytes);
unsigned char MMC_Init(void);
unsigned char MMC_write_sector(unsigned long addr,unsigned char *Buffer);
//unsigned char MMC_read_sector(unsigned long addr,unsigned char *Buffer);
unsigned char Write_Command_MMC(unsigned char *CMD);
unsigned char Read_CSD_MMC(unsigned char *Buffer);
unsigned char Read_CID_MMC(unsigned char *Buffer);
void MMC_get_volume_info(void);
unsigned char MMC_Start_Read_Sector(unsigned long sector);
void MMC_get_data(unsigned int Bytes,unsigned char *buffer);
void MMC_get_data_LBA(unsigned long lba, unsigned int Bytes,unsigned char *buffer);
void MMC_GotoSectorOffset(unsigned long LBA,unsigned int offset);
void MMC_LBA_Close();

	
//set MMC_Chip_Select to high (MMC/SD-Card Invalid)
#define MMC_Disable() MMC_Write.MMC_Chip_Select=1;
//set MMC_Chip_Select to low (MMC/SD-Card Active)
#define MMC_Enable() MMC_Write.MMC_Chip_Select=0;
 
#define nop() #asm("nop"); //asm nop defined in CVAVR

#endif //_MMC_H_


