/******************** (C) COPYRIGHT 2009 奋斗嵌入式开发工作室 ********************
* File Name          : main.c  
* Author             : Sun68 
* Version            : V1.0
* Date               : 11/11/2009
* Description        : 演示将一段字符串写入AT45DB161D的1页中，然后读出并通过USART1传送出去。
                       字符串：SPI AT45DB161D Example: This is SPI DEMO, 终端上出现这一行字，说明AT45DDB161的读写正常 
                       
    定义：	
	USART1
	TXD1----- PA9-US1-TX
	RXD1----- PA10-US1-RX	  速率：115200,n,8,1 

	SPI2 
	NSS---PB12-SPI2-NSS
    MISO--PB14-SPI2-MISO
	MOSI--PB15-SPI2-MOSI
	SCK---PB13-SPI2-SCK

	
*********************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"
#include "stm32f10x_usart.h"
#include "misc.h"


extern void SPI_Flash_Init(void);
extern u8 SPI_Flash_ReadByte(void);
extern u8 SPI_Flash_SendByte(u8 byte);
extern void FlashPageEarse(u16 page);
extern void FlashPageRead(u16 page,u8 *Data);
extern void FlashPageWrite(u16 page,u8 *Data);
extern void FlashWaitBusy(void);
extern void AT45_RomTo_buf(unsigned char buffer,unsigned int page);
extern u8 AT45_buf_ToRam(unsigned char buffer,unsigned int start_address,unsigned int length);
extern u8 AT45_RamTo_buf(unsigned char buffer,unsigned int start_address,unsigned int length);	  
extern void AT45_buf_ToRom(unsigned char buffer,unsigned int page);
extern void AT45_page_earse(unsigned int page);

extern unsigned char AT45_buffer[];
/** @addtogroup StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Interrupt
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
uint8_t TxBuffer1[] = "SPI AT45DB161D Example: This is SPI DEMO, 终端上出现这一行字，说明AT45DDB161的读写正常";
uint8_t RxBuffer1[RxBufferSize1],rec_f;
__IO uint8_t TxCounter1 = 0x00;
__IO uint8_t RxCounter1 = 0x00; 
uint8_t NbrOfDataToTransfer1 = TxBufferSize1;
uint8_t NbrOfDataToRead1 = RxBufferSize1;
__IO TestStatus TransferStatus1 = FAILED; 

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

void Delay(__IO uint32_t nCount);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len);

TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStruct;
USART_ClockInitTypeDef USART_ClockInitStruct;
int16_t USART_FLAG;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval : None
  */
int main(void)
{

   uint8_t a=0;
   uint16_t i=0;
  /* System Clocks Configuration */
  RCC_Configuration();
       
  /* NVIC configuration */
  NVIC_Configuration();

  /* Configure the GPIO ports */
  GPIO_Configuration();
  
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
  SPI_Flash_Init();
  
  GPIO_SetBits(GPIOB, GPIO_Pin_5);
  
  for(i=0; i<TxBufferSize1;i++) AT45_buffer[i]=TxBuffer1[i];

  AT45_RamTo_buf(1,0,TxBufferSize1);  
  AT45_buf_ToRom(1,1);  //1区to一页  	  

  
  AT45_RomTo_buf(1,1);	//1页to1区  
  AT45_buf_ToRam(1,0,TxBufferSize1);   
  //GPIO_SetBits(GPIOB, GPIO_Pin_5); while(1);
  //GPIO_ResetBits(GPIOB, GPIO_Pin_5);
  a=0;
  for(i=0; i<TxBufferSize1;i++){
  	if(AT45_buffer[i]==TxBuffer1[i]) a=1;
	else {a=0; i=TxBufferSize1;}
  }
  
  if(a==1) USART_OUT(USART1,&AT45_buffer[0],TxBufferSize1);	  
   while (1)
  {	
  }
}


void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,uint16_t Len){ 
	uint16_t i;
	for(i=0; i<Len; i++){
		USART_SendData(USARTx, Data[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
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
  
  /* Enable USART1, GPIOA, GPIOx and AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOx
                         | RCC_APB2Periph_AFIO, ENABLE);
  /* Enable USART2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval : None
  */
void GPIO_Configuration(void)
{

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);
  	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				     //LED1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_3;		 //LED2, LED3
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		 //LCD 背光控制
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOE, GPIO_Pin_0);	   	 	


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         //USART1 TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);		    //A端口 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         //USART1 RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //复用开漏输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);		         //A端口 

  //GPIO_PinRemapConfig(AFIO_MAPR_USART1_REMAP, ENABLE);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval : None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  
}

/**
  * @brief  Compares two buffers.
  * @param pBuffer1, pBuffer2: buffers to be compared.
  * @param BufferLength: buffer's length
  * @retval : PASSED: pBuffer1 identical to pBuffer2
  *   FAILED: pBuffer1 differs from pBuffer2
  */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
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
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
