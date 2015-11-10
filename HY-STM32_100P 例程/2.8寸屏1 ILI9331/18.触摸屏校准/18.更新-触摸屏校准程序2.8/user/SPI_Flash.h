#ifndef _SPI_FLASH_H_
#define _SPI_FLASH_H_  1
//#include "stm32f10x_lib.h"

#define FLASH_CHREAD 0x0B
#define FLASH_CLREAD 0x03
#define FLASH_PREAD	0xD2

#define FLASH_BUFWRITE1 0x84
#define FLASH_IDREAD 0x9F
#define FLASH_STATUS 0xD7
#define PAGE_ERASE 0x81
#define PAGE_READ 0xD2
#define MM_PAGE_TO_B1_XFER 0x53				// 将主存储器的指定页数据加载到第一缓冲区
#define BUFFER_2_WRITE 0x87					// 写入第二缓冲区
#define B2_TO_MM_PAGE_PROG_WITH_ERASE 0x86	// 将第二缓冲区的数据写入主存储器（擦除模式）

#define BUFFER_1_WRITE 0x84 // 写入第一缓冲区 
#define BUFFER_2_WRITE 0x87 // 写入第二缓冲区 
#define BUFFER_1_READ 0xD4 // 读取第一缓冲区 
#define BUFFER_2_READ 0xD6 // 读取第二缓冲区 
#define B1_TO_MM_PAGE_PROG_WITH_ERASE 0x83 // 将第一缓冲区的数据写入主存储器（擦除模式） 
#define B2_TO_MM_PAGE_PROG_WITH_ERASE 0x86 // 将第二缓冲区的数据写入主存储器（擦除模式） 
#define MM_PAGE_TO_B1_XFER 0x53 // 将主存储器的指定页数据加载到第一缓冲区 
#define MM_PAGE_TO_B2_XFER 0x55 // 将主存储器的指定页数据加载到第二缓冲区 
#define PAGE_ERASE 0x81 // 页删除（每页512/528字节） 
#define SECTOR_ERASE 0x7C // 扇区擦除（每扇区128K字节） 
#define READ_STATE_REGISTER 0xD7 // 读取状态寄存器 

/* Private typedef -----------------------------------------------------------*/
#define SPI_FLASH_PageSize    0x100

/* Private define ------------------------------------------------------------*/
#define WRITE      0x02  /* Write to Memory instruction */
#define WRSR       0x01  /* Write Status Register instruction */
#define WREN       0x06  /* Write enable instruction */

#define READ       0x03  /* Read from Memory instruction */
#define RDSR       0x05  /* Read Status Register instruction  */
#define RDID       0x9F  /* Read identification */
#define SE         0xD8  /* Sector Erase instruction */
#define BE         0xC7  /* Bulk Erase instruction */

#define WIP_Flag   0x01  /* Write In Progress (WIP) flag */

#define Dummy_Byte 0xA5

/* Select SPI FLASH: ChipSelect pin low  */
#define Select_Flash()     GPIO_ResetBits(GPIOA, GPIO_Pin_4)
/* Deselect SPI FLASH: ChipSelect pin high */
#define NotSelect_Flash()    GPIO_SetBits(GPIOA, GPIO_Pin_4)



void SPI_Flash_Init(void);	         //SPI初始化
u16 SPI_Flash_ReadByte(u8 a);		//flash操作基本函数，读一个字节
u16 SPI_Flash_SendByte(u8 byte);		//	FLASH操作基本函数，发送一个字节
void FlashPageEarse(u16 page);	//擦除指定的页,页范围0-4095

void FlashPageRead(u16 page,u8 *Data);		//读取整页，页范围0-4095
void FlashPageWrite(u16 page,u8 *Data);		//写一整页，页范围0-4095


void FlashWaitBusy(void);			    //Flash忙检测
void FlashReadID(u8 *ProdustID);		//读取flashID四个字节
	





#endif


