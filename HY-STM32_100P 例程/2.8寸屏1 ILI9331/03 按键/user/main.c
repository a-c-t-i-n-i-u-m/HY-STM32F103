/******************** (C) COPYRIGHT 2010 HYÇ¶ÈëÊ½¿ª·¢¹¤×÷ÊÒ ********************

* Description        : 
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

GPIO_InitTypeDef GPIO_InitStructure;


void RCC_Configuration(void);

void Delay(__IO uint32_t nCount);

int main(void)
{
  //uint16_t a;
  /* System Clocks Configuration **********************************************/
  
  RCC_Configuration();   
  
  //
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
  
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);

  /*Configure PE2,3,4,5 as output for S1,2,3,4*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//S1-4		
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;		//D1  D2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);					 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_13;		 //D3, D4
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  while (1)
  {
  
  if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5))        //S1
   {
    GPIO_SetBits(GPIOC, GPIO_Pin_6);// D1ÁÁ                   
    Delay(0x10FFFF);
    GPIO_ResetBits(GPIOC, GPIO_Pin_6);	//D1Ãð 
   }

   if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))       //S2
   {
    GPIO_SetBits(GPIOC, GPIO_Pin_7);// D2ÁÁ                   
    Delay(0x10FFFF);
    GPIO_ResetBits(GPIOC, GPIO_Pin_7);	//D2Ãð 
   }

   if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))       //S3
   {
    GPIO_SetBits(GPIOD, GPIO_Pin_6);// D3ÁÁ                   
    Delay(0x10FFFF);
    GPIO_ResetBits(GPIOD, GPIO_Pin_6);	//D3Ãð 
   }

   if(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))       //S4
   {
    GPIO_SetBits(GPIOD, GPIO_Pin_13);// D4ÁÁ                   
    Delay(0x10FFFF);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);	//D4Ãð 
   }

  }
}

void RCC_Configuration(void)
{   
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
}


void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
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



/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
