/**
  ******************************************************************************
  * @file 	main.c 
  * @brief  This file contains the headers of the interrupt handlers.
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "platform_config.h"
#include "stm32f10x_usart.h"
#include "misc.h"

void USART2_Config(void);
void GPIO_Configuration(void);
void Delay(vu32 Time);


unsigned char TxBuf[10]  = " \r\n";
unsigned char TxBuf1[50] = " ******************************************\r\n";
unsigned char TxBuf2[50] = " *     感谢您使用硕耀HY-STM32开发板！^_^  *\r\n";
unsigned char TxBuf3[50] = "\r\n Please input any word :\r\n ";
unsigned char TxBuf4[50]  = " *                                        *\r\n";


int main(void)
{
	int i, RX_status = 0;

	SystemInit();
	GPIO_Configuration();
	USART2_Config();

	/* ========USART打印欢迎信息============ */
	for( i = 0; TxBuf[i] != '\0'; i++) {
		USART_SendData(USART2 , TxBuf[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		}
	for( i = 0; TxBuf1[i] != '\0'; i++) {
		USART_SendData(USART2 , TxBuf1[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		}
	for( i = 0; TxBuf4[i] != '\0'; i++) {
		USART_SendData(USART2 , TxBuf4[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		}
	for( i = 0; TxBuf2[i] != '\0'; i++) {
		USART_SendData(USART2 , TxBuf2[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		}
	for( i = 0; TxBuf4[i] != '\0'; i++) {
		USART_SendData(USART2 , TxBuf4[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		}
	for( i = 0; TxBuf1[i] != '\0'; i++) {
		USART_SendData(USART2 , TxBuf1[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		}
	for( i = 0; TxBuf3[i] != '\0'; i++) {
		USART_SendData(USART2 , TxBuf3[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		}

	while (1)
	{
		GPIO_SetBits(GPIOE, GPIO_Pin_1);
		RX_status = USART_GetFlagStatus(USART2, USART_FLAG_RXNE);
		if(RX_status == SET) {
			USART_SendData(USART2 , USART_ReceiveData(USART2));
			while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
			GPIO_ResetBits(GPIOE, GPIO_Pin_1);
			Delay(0xFFFFF);	
        }
	}
}


void USART2_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}


void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);
    	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	 		//LED4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);					 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);		   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOA, &GPIO_InitStructure);		       

}


void Delay(vu32 Time)
{
	for(; Time != 0; Time--);
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
