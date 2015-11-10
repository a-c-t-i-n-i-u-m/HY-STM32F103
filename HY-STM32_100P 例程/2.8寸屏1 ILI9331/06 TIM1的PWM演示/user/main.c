/*
|               演示通过示波器观察TIM1的1通道的PWM波形      |
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "misc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
TIM_TimeBaseInitTypeDef  TIM1_TimeBaseStructure;
TIM_OCInitTypeDef  TIM1_OCInitStructure;
TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
u16 capture = 0;
u16 CCR1_Val = 0x7FFF ;
u16 CCR2_Val = 0x3FFF ;
u16 CCR3_Val = 0x1FFF ;  
ErrorStatus HSEStartUpStatus;
 
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
    
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DEBUG
  debug();
#endif

  /* System Clocks Configuration */
  RCC_Configuration();

  /* GPIO Configuration */
  GPIO_Configuration();

  /* NVIC configuration */
  NVIC_Configuration();

  /* -----------------------------------------------------------------------
  TIM1 Configuration to:

  1/ Generate 3 complementary PWM signals with 3 different duty cycles:
  TIM1CLK = 72 MHz, Prescaler = 0x0, TIM1 counter clock = 72 MHz
  TIM1 frequency = TIM1CLK/(TIM1_Period + 1) = 1.098 KHz

  TIM1 Channel1 duty cycle = TIM1->CCR1 / TIM1_Period = 50% 
  TIM1 Channel1N duty cycle = (TIM1_Period - TIM1_CCR1) / (TIM1_Period + 1) = 50%

  TIM1 Channel2 duty cycle = TIM1_CCR2 / TIM1_Period = 25%
  TIM1 Channel2N duty cycle = (TIM1_Period - TIM1_CCR1) / (TIM1_Period + 1) = 75% 

  TIM1 Channel3 duty cycle = TIM1_CCR3 / TIM1_Period = 12.5% 
  TIM1 Channel3N duty cycle = (TIM1_Period - TIM1_CCR3) / (TIM1_Period + 1) = 87.5% 

  2/ Insert a dead time equal to 1.62 us
  3/ Configure the break feature, active at High level, and using the automatic 
     output enable feature
  4/ Use the Locking parametres level1. 
  ----------------------------------------------------------------------- */

  /* TIM1 Peripheral Configuration */ 
  TIM_DeInit(TIM1);

  /* Time Base configuration */
  TIM1_TimeBaseStructure.TIM_Prescaler = 0x0;
  TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM1_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM1_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM1_TimeBaseStructure.TIM_RepetitionCounter = 0x0;

  TIM_TimeBaseInit(TIM1,&TIM1_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                  
  TIM1_OCInitStructure.TIM_Pulse = CCR2_Val; 
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
  TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;         
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;          
  
  TIM_OC1Init(TIM1,&TIM1_OCInitStructure); 

  TIM1_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OC2Init(TIM1,&TIM1_OCInitStructure);

  TIM1_OCInitStructure.TIM_Pulse = CCR3_Val;
  TIM_OC3Init(TIM1,&TIM1_OCInitStructure);

  /* Automatic Output enable, Break, dead time and lock configuration*/
  TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIM1_BDTRInitStructure.TIM_DeadTime = 0x75;
  TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

  TIM_BDTRConfig(TIM1,&TIM1_BDTRInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1,ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1,ENABLE);
  
  while(1)
  {
  }    
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
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

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

  /* TIM1, GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOB, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure the TIM1 Pins.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA Configuration: Channel 1 Output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configure the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{  
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while(1)
  {
  }	
}
#endif

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
