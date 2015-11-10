#if !defined( _CONFIG_H )
#define _CONFIG_H

//-----------------------------------------------------------------------------------------------------
//������    ADDRESS=0x4200 0000 + (0x0001 100C*0x20) + (bitx*4)	;bitx:��xλ
// �ѡ�λ����ַ��λ��š�ת��������ַ��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
//�Ѹõ�ַת����һ��ָ��
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))

#define BIT_ADDR(addr, bitnum)   MEM_ADDR( BITBAND(addr, bitnum)  )

#define GPIOA_ODR    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR    (GPIOE_BASE+12) //0x4001180C

#define GPIOA_IDR    (GPIOA_BASE+8) //0x4001080C
#define GPIOB_IDR    (GPIOB_BASE+8) //0x40010C0C
#define GPIOC_IDR    (GPIOC_BASE+8) //0x4001100C
#define GPIOD_IDR    (GPIOD_BASE+8) //0x4001140C
#define GPIOE_IDR    (GPIOE_BASE+8) //0x4001180C

//#define PC13 *((volatile unsigned long  *)(0x422201B4))//((volatile unsigned long *) (0x422201B4))
#define PC13  BIT_ADDR(GPIOC_ODR, 13) //���
#define PC12  BIT_ADDR(GPIOC_ODR, 12) //���
#define PC11  BIT_ADDR(GPIOC_ODR, 11) //���
#define PC10  BIT_ADDR(GPIOC_ODR, 10) //���
#define PC9   BIT_ADDR(GPIOC_ODR, 9) //���
#define PC8   BIT_ADDR(GPIOC_ODR, 8) //���

#define PB10in  BIT_ADDR(GPIOB_IDR, 10) //����
#define PB11in  BIT_ADDR(GPIOB_IDR, 11) //����

#define PD2in   BIT_ADDR(GPIOD_IDR, 2)  //����

#define PD6in   BIT_ADDR(GPIOE_IDR, 15)  //����
#define PD9in   BIT_ADDR(GPIOD_IDR, 9)  //����

//-----------------------------------------------------------------------------------------------------

//#define  myGPIO_SetBits(PORT, Bit)     SetBit32(PORT->ODR, Bit)
//#define  myGPIO_ResetBits(PORT, Bit)   ClrBit32(PORT->ODR, Bit)

//#define  myGPIO_SetBits(PORT, Bit)     GPIO_SetBits(PORT, BIT16(Bit))
//#define  myGPIO_ResetBits(PORT, Bit)   GPIO_ResetBits(PORT, BIT16(Bit))

#define  myGPIO_SetBits(PORT, Bit)     ( PORT->BSRR=BIT16(Bit) )
#define  myGPIO_ResetBits(PORT, Bit)   ( PORT->BRR=BIT16(Bit) )

//485����-------------------------------------------------------------
/******************* ������PC����ͨѶΪ485 ****************/
//#define PC_RS485
#define PC_RS485CAN
/******************* �������������ͨѶΪ485 ****************/
#define RS485

//Ӳ������
#define  RCC_APB2Periph_PC_RS485DIR  RCC_APB2Periph_GPIOB
#define  PC_RS485DIR_PORT    GPIOB
#define  PC_RS485DIR_PIN     0

//#ifdef PC_RS485
#if defined(PC_RS485)
  #define PC_RS485Send_Enable()     myGPIO_SetBits(PC_RS485DIR_PORT, PC_RS485DIR_PIN)
  #define PC_RS485Receive_Enable()  myGPIO_ResetBits(PC_RS485DIR_PORT, PC_RS485DIR_PIN)

#elif defined(PC_RS485CAN)
  #define PC_RS485Send_Enable()     //(USART1->CR1 &= ~USART_FLAG_RXNE)    //�رմ���1�����ж�
  #define PC_RS485Receive_Enable()  //(USART1->CR1 |= USART_FLAG_RXNE)//�򿪴���1�����ж�
  //���ʹ��CANоƬ,ͨ���رս����ж��л����շ���״̬��������ɽ����ж�һֱ�ر��޷���������
#else
  #define PC_RS485Send_Enable()     
  #define PC_RS485Receive_Enable()  

#endif
//--------------------------------------------------------------------

//LED����-------------------------------------------------------------
#define  RCC_APB2Periph_LED  RCC_APB2Periph_GPIOC
#define LED_PORT    GPIOC  //GPIOC
#define LED_PIN    8  
  
  #define LED_Green_ON()   myGPIO_ResetBits(LED_PORT, LED_PIN)//SetBit32(LEDPORT->ODR, 13)
  #define LED_Green_OFF()  myGPIO_SetBits(LED_PORT, LED_PIN)//ClrBit32(LEDPORT->ODR, 13)


#endif
