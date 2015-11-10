/**
编译器版本：MDK3.5  基于STM32 STD3.0库
测试DEMO表述： 对SD卡读写进行了测试，并通过串口1将测试信息输出。 
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "sdcard.h"
#include "stdio.h"
#include "stm32f10x_usart.h"

/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup SDIO_Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
uint8_t TxBuffer1[] = "USART Interrupt Example: This is USART1 DEMO";
uint8_t RxBuffer1[RxBufferSize1],rec_f;
__IO uint8_t TxCounter1 = 0x00;
__IO uint8_t RxCounter1 = 0x00; 
uint8_t NbrOfDataToTransfer1 = TxBufferSize1;
uint8_t NbrOfDataToRead1 = RxBufferSize1;



/* Private define ------------------------------------------------------------*/
#define BlockSize            512 /* Block Size in Bytes */
//#define BlockSize            128 /* Block Size in Bytes */
#define BufferWordsSize      (BlockSize >> 2)

#define NumberOfBlocks       2  /* For Multi Blocks operation (Read/Write) */
#define MultiBufferWordsSize ((BlockSize * NumberOfBlocks) >> 2)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SD_CardInfo SDCardInfo;
uint32_t Buffer_Block_Tx[BufferWordsSize], Buffer_Block_Rx[BufferWordsSize];
uint32_t Buffer_MultiBlock_Tx[MultiBufferWordsSize], Buffer_MultiBlock_Rx[MultiBufferWordsSize];
volatile TestStatus EraseStatus = FAILED, TransferStatus1 = FAILED, TransferStatus2 = FAILED;
SD_Error Status = SD_OK;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void Fill_Buffer(uint32_t *pBuffer, uint16_t BufferLenght, uint32_t Offset);
TestStatus Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint16_t BufferLength);
TestStatus eBuffercmp(uint32_t* pBuffer, uint16_t BufferLength);


void USART1_Config(void);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len);
int GetKey (void) ;
int SendChar (int ch) ;
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
  /* Clock Config */
  RCC_Configuration();

  /* Interrupt Config */
  NVIC_Configuration();
  Status = SD_Init();
  /*-------------------------- SD Init ----------------------------- */
  //while(Status != SD_OK)  Status = SD_Init();
  
  USART1_Config();  

  GPIO_ResetBits(GPIOB, GPIO_Pin_5); 
  //Delay(0xfffff);	   GPIO_SetBits(GPIOB, GPIO_Pin_5);     	while(1);
  if (Status == SD_OK)
  {	//GPIO_SetBits(GPIOB, GPIO_Pin_5);	  	
    /*----------------- Read CSD/CID MSD registers ------------------*/
	 //USART_OUT(USART1,&TxBuffer1[0],TxBufferSize1);
      printf("\nSD_Init finished..... ");
	 Status = SD_GetCardInfo(&SDCardInfo);	   
  }
  printf("\r\nSD_GetCardInfo..... %d",Status);
  printf("\r\n\t\t info..%d",SDCardInfo.CardType); 
  if (Status == SD_OK)
  {	 printf("\nSD_GetCardInfo finished..... ");
    /*----------------- Select Card --------------------------------*/
    Status = SD_SelectDeselect((uint32_t) (SDCardInfo.RCA << 16));  
  }

  if (Status == SD_OK)
  {	printf("\r\nSD_SelectDeselect finished.....\n ");	   
    //Status = SD_EnableWideBusOperation(SDIO_BusWide_4b);  
  }
  
  /*------------------- Block Erase -------------------------------*/
  if (Status == SD_OK)
  {	 //printf("\n\nSD_EnableWideBusOperation finished.....\n ");
    /* Erase NumberOfBlocks Blocks of WRITE_BL_LEN(512 Bytes) */
    Status = SD_Erase(0x00, (BlockSize * NumberOfBlocks));   
  }	 
  printf("\r\nSD_SetDeviceMode..... %d",Status);
  /* Set Device Transfer Mode to DMA */
  printf("\r\nSD_SetDeviceMode..... %d",Status);
  if (Status == SD_OK)
  {  printf("\r\nSD_Erase finished.....\n "); 
    Status = SD_SetDeviceMode(SD_DMA_MODE);	 
	//Status = SD_SetDeviceMode(SD_INTERRUPT_MODE);	
	//Status = SD_SetDeviceMode(SD_POLLING_MODE);  
  }
  printf("\r\nSD_ReadMultiBlocks..... %d",Status);
 //Fill_Buffer(Buffer_Block_Tx, BufferWordsSize, 0x22222222);
  if (Status == SD_OK)
  {	 
     printf("\r\nSD_SetDeviceMode finished.....\n ");    
	  //Status = SD_ReadBlock(0x00, Buffer_Block_Rx, BlockSize);  
	  //Status = SD_WriteBlock(0x00, Buffer_Block_Tx, BlockSize);   
      Status = SD_ReadMultiBlocks(0x00, Buffer_MultiBlock_Rx, BlockSize, NumberOfBlocks);	
	 
  }
  printf("\r\neBuffercmp..... %d",Status);
  if (Status == SD_OK)
  {	GPIO_SetBits(GPIOB, GPIO_Pin_5);     	
    printf("\r\nSD_ReadMultiBlocks finished.....\n "); 
    EraseStatus = eBuffercmp(Buffer_MultiBlock_Rx, MultiBufferWordsSize);
     printf("\r\n EraseStatus = eBuffercmp.....%d\n ",EraseStatus); 
  }
  
  /*------------------- Block Read/Write --------------------------*/
  /* Fill the buffer to send */
  Fill_Buffer(Buffer_Block_Tx, BufferWordsSize, 0x12345678);


  if (Status == SD_OK)
  {	printf("\r\nSD_ReadMultiBlocks finished.....\n ");  
    /* Write block of 512 bytes on address 0 */
    Status = SD_WriteBlock(0x00, Buffer_Block_Tx, BlockSize);  
//	GPIO_SetBits(GPIOB, GPIO_Pin_5);  while(1);
  }

  if (Status == SD_OK)
  {	 printf("\r\nSD SD_WriteBlock finished.....\n ");
    /* Read block of 512 bytes from address 0 */
    Status = SD_ReadBlock(0x00, Buffer_Block_Rx, BlockSize);  
  }

  if (Status == SD_OK)
  {	printf("\r\nSD SD_ReadBlock finished.....\n ");
    /* Check the corectness of written dada */
    TransferStatus1 = Buffercmp(Buffer_Block_Tx, Buffer_Block_Rx, BufferWordsSize);
  }

  /*--------------- Multiple Block Read/Write ---------------------*/
  /* Fill the buffer to send */
  Fill_Buffer(Buffer_MultiBlock_Tx, MultiBufferWordsSize, 0x0);

  if (Status == SD_OK)
  {
    /* Write multiple block of many bytes on address 0 */
    Status = SD_WriteMultiBlocks(0x00, Buffer_MultiBlock_Tx, BlockSize, NumberOfBlocks); 
  }

  if (Status == SD_OK)
  {
    /* Read block of many bytes from address 0 */
    Status = SD_ReadMultiBlocks(0x00, Buffer_MultiBlock_Rx, BlockSize, NumberOfBlocks);	 
  }

  if (Status == SD_OK)
  {	 
    /* Check the corectness of written dada */
    TransferStatus2 = Buffercmp(Buffer_MultiBlock_Tx, Buffer_MultiBlock_Rx, MultiBufferWordsSize); 	

	printf("\r\nSD SDIO-1bit模式 测试TF卡读写成功！ \n ");
  }

  /* Infinite loop */
  while (1)
  {//GPIO_SetBits(GPIOB, GPIO_Pin_5);              
  }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval : None
  */


void USART1_Config(void){
	/* USART1 configuration ------------------------------------------------------*/
  /* USART and USART2 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);
 
  
  /* Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);
}

void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len){ 
	uint16_t i;
	for(i=0; i<Len; i++){
		USART_SendData(USARTx, Data[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}

/* Implementation of putchar (also used by printf function to output data)    */
int SendChar (int ch)  {                /* Write character to Serial Port     */

  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
  return (ch);
}


int GetKey (void)  {                    /* Read character from Serial Port    */

  while (!(USART1->SR & USART_FLAG_RXNE));
  return (USART_ReceiveData(USART1));
}



/*******************************************************************************
* Function Name  : fputc
* Description    : Retargets the C library printf function to the USART.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (u8) ch);

  /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {
  }

  return ch;
}




void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

void RCC_Configuration(void)
{
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
}

/**
  * @brief  Configures SDIO IRQ channel.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Compares two buffers.
  * @param pBuffer1, pBuffer2: buffers to be compared.
  *   : - BufferLength: buffer's length
  * @retval : PASSED: pBuffer1 identical to pBuffer2
  *   FAILED: pBuffer1 differs from pBuffer2
  */
TestStatus Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

/**
  *   Function name : Fill_Buffer
  * @brief  Fills buffer with user predefined data.
  * @param pBuffer: pointer on the Buffer to fill
  * @param BufferLenght: size of the buffer to fill
  * @param Offset: first value to fill on the Buffer
  * @retval : None
  */
void Fill_Buffer(uint32_t *pBuffer, uint16_t BufferLenght, uint32_t Offset)
{
  uint16_t index = 0;

  /* Put in global buffer same values */
  for (index = 0; index < BufferLenght; index++ )
  {
    pBuffer[index] = index + Offset;
  }
}

/**
  *   Function name : eBuffercmp
  * @brief  Checks if a buffer has all its values are equal to zero.
  * @param pBuffer: buffer to be compared.
  * @param BufferLength: buffer's length
  * @retval : PASSED: pBuffer values are zero
  *   FAILED: At least one value from pBuffer buffer is diffrent 
  *   from zero.
  */
TestStatus eBuffercmp(uint32_t* pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer != 0x00)
    {
      return FAILED;
    }

    pBuffer++;
  }

  return PASSED;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
