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
#define MM_PAGE_TO_B1_XFER 0x53				// �����洢����ָ��ҳ���ݼ��ص���һ������
#define BUFFER_2_WRITE 0x87					// д��ڶ�������
#define B2_TO_MM_PAGE_PROG_WITH_ERASE 0x86	// ���ڶ�������������д�����洢��������ģʽ��

#define BUFFER_1_WRITE 0x84 // д���һ������ 
#define BUFFER_2_WRITE 0x87 // д��ڶ������� 
#define BUFFER_1_READ 0xD4 // ��ȡ��һ������ 
#define BUFFER_2_READ 0xD6 // ��ȡ�ڶ������� 
#define B1_TO_MM_PAGE_PROG_WITH_ERASE 0x83 // ����һ������������д�����洢��������ģʽ�� 
#define B2_TO_MM_PAGE_PROG_WITH_ERASE 0x86 // ���ڶ�������������д�����洢��������ģʽ�� 
#define MM_PAGE_TO_B1_XFER 0x53 // �����洢����ָ��ҳ���ݼ��ص���һ������ 
#define MM_PAGE_TO_B2_XFER 0x55 // �����洢����ָ��ҳ���ݼ��ص��ڶ������� 
#define PAGE_ERASE 0x81 // ҳɾ����ÿҳ512/528�ֽڣ� 
#define SECTOR_ERASE 0x7C // ����������ÿ����128K�ֽڣ� 
#define READ_STATE_REGISTER 0xD7 // ��ȡ״̬�Ĵ��� 

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



void SPI_Flash_Init(void);	         //SPI��ʼ��
u16 SPI_Flash_ReadByte(u8 a);		//flash����������������һ���ֽ�
u16 SPI_Flash_SendByte(u8 byte);		//	FLASH������������������һ���ֽ�
void FlashPageEarse(u16 page);	//����ָ����ҳ,ҳ��Χ0-4095

void FlashPageRead(u16 page,u8 *Data);		//��ȡ��ҳ��ҳ��Χ0-4095
void FlashPageWrite(u16 page,u8 *Data);		//дһ��ҳ��ҳ��Χ0-4095


void FlashWaitBusy(void);			    //Flashæ���
void FlashReadID(u8 *ProdustID);		//��ȡflashID�ĸ��ֽ�
	
void AT45_RomTo_buf(unsigned char buffer,unsigned int page);
u8 M25P80_buf_ToRam(unsigned char buffer,unsigned int start_address,unsigned int length);
u8 M25P16_RamTo_buf(unsigned char buffer,unsigned int start_address,unsigned int length);	  
void AT45_buf_ToRom(unsigned char buffer,unsigned int page);
void AT45_page_earse(unsigned int page);
u16 SPI_ReadByte(unsigned char a);
u16 SPI_SendByte(u8 byte);




#endif


