/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "sdcard.h"
//#include "dosfs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "integer.h"
#include "ff.h"
#include "diskio.h"


/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/

////////////////////////////////////////////////////////////////////
// Redirect fputc to the serial port, so printf() outputs to USART1.
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
///////////////////////////////////////////////////////////////////

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SD_CardInfo SDCardInfo;
SD_Error Status = SD_OK;
ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
u8 USART_Scanf(u32 value);
void Serial_Init(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);

/* Private functions ---------------------------------------------------------*/
    FATFS fs;            // Work area (file system object) for logical drive
    FIL fsrc, fdst;      // file objects
    BYTE buffer[51]; // file copy buffer
    FRESULT res;         // FatFs function common result code
    UINT br, bw;         // File R/W count
    
    
static
FRESULT scan_files (char* path)
{
  DWORD acc_size;				/* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO finfo;
	DIR dirs;
	FRESULT res;
	BYTE i;


	if ((res = f_opendir(&dirs, path)) == FR_OK) {
		i = strlen(path);
		while (((res = f_readdir(&dirs, &finfo)) == FR_OK) && finfo.fname[0]) {
			if (finfo.fattrib & AM_DIR) {
				acc_dirs++;
				*(path+i) = '/'; strcpy(path+i+1, &finfo.fname[0]);
				res = scan_files(path);
				*(path+i) = '\0';
				if (res != FR_OK) break;
			} else {
				acc_files++;
				acc_size += finfo.fsize;
			}
		}
	}

	return res;
}
 
void OutPutFile(void)
{
  FRESULT res;
  FILINFO finfo;
  DIR dirs;
  int i;
  char *fn;
  char path[50]={""};  
  char name[]={"WVO.TXT"};
    
  printf("\n file system starting! \n");
  
  disk_initialize(0);
    
  f_mount(0, &fs);
/* 
  res = f_opendir(&dir, path);
  if (res == FR_OK) 
  {
    i = strlen(path);
    for (;;) 
    {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0) break;
      fn = fno.fname;
      if (fno.fattrib & AM_DIR) 
      {
        sprintf(&path[i], "/%s", fn);
        res = scan_files(path);
        if (res != FR_OK) break;
        path[i] = 0;
      } 
      else 
      {
        printf("%s/%s\n\n", path, fn);
      }
    }
  }
*/  
 if (f_opendir(&dirs, path) == FR_OK) 
  {
    while (f_readdir(&dirs, &finfo) == FR_OK)  
    {
      if (finfo.fattrib & AM_ARC) 
      {
        if(!finfo.fname[0])	
          break;         
        printf("\n file name is:\n   %s\n",finfo.fname);
        res = f_open(&fsrc, finfo.fname, FA_OPEN_EXISTING | FA_READ);
        res = f_read(&fsrc, &buffer, 50, &br);
        printf("\n file contex is:\n   %s\n",buffer);
        f_close(&fsrc);			                      
      }
    } 
    
  }
  
  while(1);
}
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DEBUG
  debug();
#endif
  
  /*
  static uint8_t sector[SECTOR_SIZE];
  static uint8_t transferbuffer[SECTOR_SIZE];
  uint32_t pstart, psize, i;
  uint8_t pactive, ptype;
  VOLINFO vi;
  DIRINFO di;
  FILEINFO fi;
  DIRENT de;
  uint32_t cache;
  */
  
  
  // Clock Config: HSE 72 MHz
  RCC_Configuration();
  
  // Interrupt Config
  NVIC_Configuration();
  
  // USART Config : 115200,8,n,1
  Serial_Init();
  
  /////////////////////////////////////////////////////////////////////
  //////// SDCARD Initialisation //////////////////////////////////////
  /////////////////Section adapted from ST example/////////////////////
  
  /*-------------------------- SD Init ----------------------------- */
  Status = SD_Init();

  if (Status == SD_OK)
  {
    /*----------------- Read CSD/CID MSD registers ------------------*/
    Status = SD_GetCardInfo(&SDCardInfo);
  }
  
  if (Status == SD_OK)
  {
    /*----------------- Select Card --------------------------------*/
    Status = SD_SelectDeselect((u32) (SDCardInfo.RCA << 16));
  }
  
  if (Status == SD_OK)
  {
    //Status = SD_EnableWideBusOperation(SDIO_BusWide_4b);
  }
  
  /* Set Device Transfer Mode to DMA */
  if (Status == SD_OK)
  {  
//    Status = SD_SetDeviceMode(SD_DMA_MODE);//oet
 //   Status = SD_SetDeviceMode(SD_POLLING_MODE);
    Status = SD_SetDeviceMode(SD_INTERRUPT_MODE);
  }
  
  
  OutPutFile();
  /////////////////////////////////////////////////////////////////////
  //////// DOSFS Volume Acess /////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////
/*
  if (DFS_ReadSector(0,sector,0,1))
  {
    printf("Cannot read media!\n");
    while(1);
    return -1;
  }

  if ((sector[0x1FE] == 0x55) & (sector[0x1FF] == 0xAA))
  {
    printf("The end of the last read sector contains a FAT or MBR end delimiter\n");
    if ((sector[0x36] == 'F') & (sector[0x37] == 'A') & (sector[0x38] == 'T'))
    {
      printf("Well, this is not a MBR, but a FAT12/16 header!\n");
      pstart = 0;
      pactive = 0x80;
      ptype = 0x06;
      psize = 0xFFFFFFFF;
    }
    else if ((sector[0x52] == 'F') & (sector[0x53] == 'A') & (sector[0x54] == 'T'))
    {
      printf("Well, this is not a MBR, but a FAT32 header!\n");
      pstart = 0;
      pactive = 0x80;
      ptype = 0x06;
      psize = 0xFFFFFFFF;
    }
    else
    {
      pstart = DFS_GetPtnStart(0, sector, 0, &pactive, &ptype, &psize);
      if (pstart == 0xffffffff)
      {
        printf("Cannot find first partition\n");
        return -1;
      }
    }
  }
  
  printf("Partition 0 start sector 0x%-08.8lX active %-02.2hX type %-02.2hX size %-08.8lX\n", pstart, pactive, ptype, psize);
  
  if (DFS_GetVolInfo(0, sector, pstart, &vi))
  {
    printf("Error getting volume information\n");
    return -1;
  }
  
  printf("Volume label '%-11.11s'\n", vi.label);
  printf("%d sector/s per cluster, %d reserved sector/s, volume total %d sectors.\n", vi.secperclus, vi.reservedsecs, vi.numsecs);
  printf("%d sectors per FAT, first FAT at sector #%d, root dir at #%d.\n",vi.secperfat,vi.fat1,vi.rootdir);
  printf("(For FAT32, the root dir is a CLUSTER number, FAT12/16 it is a SECTOR number)\n");
  printf("%d root dir entries, data area commences at sector #%d.\n",vi.rootentries,vi.dataarea);
  printf("%d clusters (%d bytes) in data area, filesystem IDd as ", vi.numclusters, vi.numclusters * vi.secperclus * SECTOR_SIZE);
  if (vi.filesystem == FAT12)
    printf("FAT12.\n");
  else if (vi.filesystem == FAT16)
    printf("FAT16.\n");
  else if (vi.filesystem == FAT32)
    printf("FAT32.\n");
  else
    printf("[unknown]\n");
  
  /////////////////////////////////////////////////////////////////////
  //////// DOSFS File list Acess //////////////////////////////////////
  /////////////////////////////////////////////////////////////////////
  
  // Directory enumeration test
  di.scratch = sector;
  if (DFS_OpenDir(&vi, "", &di)) {
    printf("Error opening root directory\n");
    return -1;
  }
  ////This section is to list a directory "/MYDIR1"
  if (DFS_OpenDir(&vi, "MYDIR1", &di))
    {
    printf("error opening subdirectory\n");
    return -1;
    }
  //
  while (!DFS_GetNext(&vi, &di, &de)) {
    if (de.name[0])
      printf("file: '%-11.11s'\n", de.name);
  }

//------------------------------------------------------------
// File read test

  printf("\n=====Read test\n");
	if (DFS_OpenFile(&vi, "WVO.TXT", DFS_READ, sector, &fi)) {
		printf("error opening file\n");
		return -1;
	}

	DFS_ReadFile(&fi, sector, transferbuffer, &i, 512);
        printf("read complete %d bytes (expected %d) pointer %d\n", i, fi.filelen, fi.pointer);

        printf("%s",transferbuffer);    
             
//------------------------------------------------------------
// File write test
  printf("\n=====Write test\n");

	if (DFS_OpenFile(&vi, "WRTEST.TXT", DFS_WRITE, sector, &fi))
        {
	  printf("error opening file\n");
	  return -1;
        }
	DFS_WriteFile(&fi, sector, transferbuffer, &cache, SECTOR_SIZE/2);


*/
        
  printf("\nLet's loop for infinity, and beyond!\n");
  while (1)
  {}
}


/*************************************************************************
 * Function Name: Serial_Init
 * Description: Init USARTs
 *************************************************************************/
void Serial_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);

  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

/* USART1 configuration ------------------------------------------------------*/
  // USART1 configured as follow:
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure the USART1 */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable the USART Transmoit interrupt: this interrupt is generated when the 
     USART1 transmit data register is empty */  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

  /* Enable the USART Receive interrupt: this interrupt is generated when the 
     USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
  
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

/*******************************************************************************
* Function Name  : NVIC_Config
* Description    : Configures SDIO IRQ channel.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : PUTCHAR_PROTOTYPE
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (u8) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

/*******************************************************************************
* Function Name  : USART_Scanf
* Description    : Gets numeric values from the hyperterminal.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 USART_Scanf(u32 value)
{
  u32 index = 0;
  u32 tmp[2] = {0, 0};

  while (index < 2)
  {
    /* Loop until RXNE = 1 */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
    {}
    tmp[index++] = (USART_ReceiveData(USART1));
    if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
    {
      printf("\n\rPlease enter valid number between 0 and 9");
      index--;
    }
  }
  /* Calculate the Corresponding value */
  index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
  /* Checks */
  if (index > value)
  {
    printf("\n\rPlease enter valid number between 0 and %d", value);
    return 0xFF;
  }
  return index;
}


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
