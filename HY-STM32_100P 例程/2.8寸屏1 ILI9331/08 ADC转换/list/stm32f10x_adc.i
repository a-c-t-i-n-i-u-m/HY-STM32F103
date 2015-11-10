#line 1 "FWlib\\SRC\\stm32f10x_adc.c"


















  

 
#line 1 ".\\FWlib\\inc\\stm32f10x_adc.h"



















  

 



 
#line 1 ".\\user\\stm32f10x.h"





















 



 



 
 


  


 
  


 















 

#line 67 ".\\user\\stm32f10x.h"




 




 





 

 

 

 






 



 



 




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      
  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      

  TIM4_IRQn                   = 30,      

  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      

  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      

  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      

  USART3_IRQn                 = 39,      

  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42,      

  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_IRQn                   = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_5_IRQn        = 59       

} IRQn_Type;



 

#line 1 ".\\user\\core_cm3.h"



















 




















































 

 
 
 
 
 
 
 
 


#line 1 "d:\\Keil\\ARM\\RV31\\INC\\stdint.h"
 
 





 









#line 25 "d:\\Keil\\ARM\\RV31\\INC\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "d:\\Keil\\ARM\\RV31\\INC\\stdint.h"

     







     










     











#line 260 "d:\\Keil\\ARM\\RV31\\INC\\stdint.h"



 


#line 86 ".\\user\\core_cm3.h"

















 









 


 





 






 
typedef struct
{
  volatile uint32_t ISER[8];                       
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                       
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                       
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                       
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                       
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                       
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                          
}  NVIC_Type;


 
typedef struct
{
  volatile const  uint32_t CPUID;                         
  volatile uint32_t ICSR;                          
  volatile uint32_t VTOR;                          
  volatile uint32_t AIRCR;                         
  volatile uint32_t SCR;                           
  volatile uint32_t CCR;                           
  volatile uint8_t  SHP[12];                       
  volatile uint32_t SHCSR;                         
  volatile uint32_t CFSR;                          
  volatile uint32_t HFSR;                          
  volatile uint32_t DFSR;                          
  volatile uint32_t MMFAR;                         
  volatile uint32_t BFAR;                          
  volatile uint32_t AFSR;                          
  volatile const  uint32_t PFR[2];                        
  volatile const  uint32_t DFR;                           
  volatile const  uint32_t ADR;                           
  volatile const  uint32_t MMFR[4];                       
  volatile const  uint32_t ISAR[5];                       
} SCB_Type;


 
typedef struct
{
  volatile uint32_t CTRL;                          
  volatile uint32_t LOAD;                          
  volatile uint32_t VAL;                           
  volatile const  uint32_t CALIB;                         
} SysTick_Type;


 
typedef struct
{
  volatile  union  
  {
    volatile  uint8_t    u8;                        
    volatile  uint16_t   u16;                       
    volatile  uint32_t   u32;                       
  }  PORT [32];                                
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                           
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                           
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                           
       uint32_t RESERVED3[29];
  volatile uint32_t IWR;                           
  volatile uint32_t IRR;                           
  volatile uint32_t IMCR;                          
       uint32_t RESERVED4[43];
  volatile uint32_t LAR;                           
  volatile uint32_t LSR;                           
       uint32_t RESERVED5[6];
  volatile const  uint32_t PID4;                          
  volatile const  uint32_t PID5;
  volatile const  uint32_t PID6;
  volatile const  uint32_t PID7;
  volatile const  uint32_t PID0;
  volatile const  uint32_t PID1;
  volatile const  uint32_t PID2;
  volatile const  uint32_t PID3;
  volatile const  uint32_t CID0;
  volatile const  uint32_t CID1;
  volatile const  uint32_t CID2;
  volatile const  uint32_t CID3;
} ITM_Type;


 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                          



       uint32_t RESERVED1;

} InterruptType_Type;


 
#line 251 ".\\user\\core_cm3.h"


 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;


 
#line 270 ".\\user\\core_cm3.h"

#line 277 ".\\user\\core_cm3.h"










 






#line 304 ".\\user\\core_cm3.h"


 


 




#line 329 ".\\user\\core_cm3.h"


   
   
   
   









 
extern uint32_t __get_PSP(void);









 
extern void __set_PSP(uint32_t topOfProcStack);









 
extern uint32_t __get_MSP(void);









 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 495 ".\\user\\core_cm3.h"









 









 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}








 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0x1ff);
}









 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}








 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}








 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}








 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}








 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}





#line 1024 ".\\user\\core_cm3.h"



 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t priority_grouping)
{
  uint32_t reg_value=0;
  
  reg_value  = ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                                             
  reg_value &= ~((0xFFFFU << 16) | (0x0F << 8));                                                       
  reg_value  = ((reg_value | (0x5FA << 16) | (priority_grouping << 8)));                          
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR = reg_value;
}









 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));                              
}









 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));                              
}










 
static __inline IRQn_Type NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((IRQn_Type) (((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F))));          
}









 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));                              
}









 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));                              
}










 
static __inline IRQn_Type NVIC_GetActive(IRQn_Type IRQn)
{
  return((IRQn_Type)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F))));                         
}













 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, int32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }   
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }          
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }               
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }               
}



 



 














 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > ((1<<24) -1))  return (1);                                                 

  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  =  (ticks & ((1<<24) -1)) - 1;                                          
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);                                
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   =  (0x00);                                                                  
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL = (1 << 2) | (1<<0) | (1<<1);     
  return (0);                                                                                
}







 








 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16) | (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (0x700)) | (1<<2));       
}


 











 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if(ch == '\n') ITM_SendChar('\r');
  
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1 << 24))  &&
      (((ITM_Type *) (0xE0000000))->TCR & 1)                  &&
      (((ITM_Type *) (0xE0000000))->TER & (1UL << 0))  ) 
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}



 
#line 197 ".\\user\\stm32f10x.h"
#line 1 ".\\user\\system_stm32f10x.h"


















 



 





 



 




 

extern const uint32_t SystemFrequency;                    
extern const uint32_t SystemFrequency_SysClk;             
extern const uint32_t SystemFrequency_AHBClk;             
extern const uint32_t SystemFrequency_APB1Clk;            
extern const uint32_t SystemFrequency_APB2Clk;            



 



 



 



 



 



 
  
extern void SystemInit(void);


 



 
#line 198 ".\\user\\stm32f10x.h"
#line 199 ".\\user\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;



 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;	  
  volatile uint32_t EMR;	  
  volatile uint32_t RTSR;	  
  volatile uint32_t FTSR;	  
  volatile uint32_t SWIER;	  
  volatile uint32_t PR;		  
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;   
  volatile uint32_t CRH;   
  volatile uint32_t IDR;   
  volatile uint32_t ODR;   
  volatile uint32_t BSRR;  
  volatile uint32_t BRR;   
  volatile uint32_t LCKR;  
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 









 




#line 911 ".\\user\\stm32f10x.h"

#line 928 ".\\user\\stm32f10x.h"



#line 947 ".\\user\\stm32f10x.h"














 
  


   

#line 1029 ".\\user\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1090 ".\\user\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1266 ".\\user\\stm32f10x.h"

 




 








 










 
#line 1302 ".\\user\\stm32f10x.h"






 











 










 














 
#line 1362 ".\\user\\stm32f10x.h"








 






 
#line 1395 ".\\user\\stm32f10x.h"

 
#line 1412 ".\\user\\stm32f10x.h"

 
#line 1434 ".\\user\\stm32f10x.h"

 
#line 1443 ".\\user\\stm32f10x.h"

 
#line 1460 ".\\user\\stm32f10x.h"

 
#line 1482 ".\\user\\stm32f10x.h"

 








 








   
#line 1511 ".\\user\\stm32f10x.h"

 
 
 
 
 

 




































































 




































































 
#line 1673 ".\\user\\stm32f10x.h"

 
#line 1691 ".\\user\\stm32f10x.h"

 
#line 1709 ".\\user\\stm32f10x.h"

#line 1726 ".\\user\\stm32f10x.h"

 
#line 1744 ".\\user\\stm32f10x.h"

 
#line 1763 ".\\user\\stm32f10x.h"

 

 






 
#line 1790 ".\\user\\stm32f10x.h"






 








 









 








 








 









 










 




#line 1865 ".\\user\\stm32f10x.h"






 





 





 
#line 1891 ".\\user\\stm32f10x.h"

 
#line 1900 ".\\user\\stm32f10x.h"

   
#line 1909 ".\\user\\stm32f10x.h"

 
#line 1918 ".\\user\\stm32f10x.h"

 





 
#line 1933 ".\\user\\stm32f10x.h"

 
#line 1942 ".\\user\\stm32f10x.h"

   
#line 1951 ".\\user\\stm32f10x.h"

 
#line 1960 ".\\user\\stm32f10x.h"

 





 
#line 1975 ".\\user\\stm32f10x.h"

 
#line 1984 ".\\user\\stm32f10x.h"

   
#line 1993 ".\\user\\stm32f10x.h"

 
#line 2002 ".\\user\\stm32f10x.h"

 





 
#line 2017 ".\\user\\stm32f10x.h"

 
#line 2026 ".\\user\\stm32f10x.h"

   
#line 2035 ".\\user\\stm32f10x.h"

 
#line 2044 ".\\user\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2108 ".\\user\\stm32f10x.h"

 
#line 2143 ".\\user\\stm32f10x.h"

 
#line 2178 ".\\user\\stm32f10x.h"

 
#line 2213 ".\\user\\stm32f10x.h"

 
#line 2248 ".\\user\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 2315 ".\\user\\stm32f10x.h"

 



 









 
#line 2339 ".\\user\\stm32f10x.h"




 




 
#line 2355 ".\\user\\stm32f10x.h"

 





 
#line 2377 ".\\user\\stm32f10x.h"

 
 





 
#line 2392 ".\\user\\stm32f10x.h"
 
#line 2399 ".\\user\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 2447 ".\\user\\stm32f10x.h"

 
#line 2468 ".\\user\\stm32f10x.h"

 
#line 2489 ".\\user\\stm32f10x.h"

 
#line 2510 ".\\user\\stm32f10x.h"

 
#line 2531 ".\\user\\stm32f10x.h"

 
#line 2552 ".\\user\\stm32f10x.h"

 
 
 
 
 

 
#line 2588 ".\\user\\stm32f10x.h"

 
#line 2618 ".\\user\\stm32f10x.h"

 
#line 2628 ".\\user\\stm32f10x.h"















 
#line 2652 ".\\user\\stm32f10x.h"















 
#line 2676 ".\\user\\stm32f10x.h"















 
#line 2700 ".\\user\\stm32f10x.h"















 
#line 2724 ".\\user\\stm32f10x.h"















 
#line 2748 ".\\user\\stm32f10x.h"















 
#line 2772 ".\\user\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 2873 ".\\user\\stm32f10x.h"

#line 2882 ".\\user\\stm32f10x.h"















  
 
#line 2905 ".\\user\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3040 ".\\user\\stm32f10x.h"

#line 3047 ".\\user\\stm32f10x.h"

#line 3054 ".\\user\\stm32f10x.h"

#line 3061 ".\\user\\stm32f10x.h"







 
#line 3075 ".\\user\\stm32f10x.h"

#line 3082 ".\\user\\stm32f10x.h"

#line 3089 ".\\user\\stm32f10x.h"

#line 3096 ".\\user\\stm32f10x.h"

#line 3103 ".\\user\\stm32f10x.h"

#line 3110 ".\\user\\stm32f10x.h"

 
#line 3118 ".\\user\\stm32f10x.h"

#line 3125 ".\\user\\stm32f10x.h"

#line 3132 ".\\user\\stm32f10x.h"

#line 3139 ".\\user\\stm32f10x.h"

#line 3146 ".\\user\\stm32f10x.h"

#line 3153 ".\\user\\stm32f10x.h"

 
#line 3161 ".\\user\\stm32f10x.h"

#line 3168 ".\\user\\stm32f10x.h"

#line 3175 ".\\user\\stm32f10x.h"

#line 3182 ".\\user\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 
 
 
 
 

 
















 









#line 3332 ".\\user\\stm32f10x.h"

 

























 
#line 3375 ".\\user\\stm32f10x.h"

 
#line 3389 ".\\user\\stm32f10x.h"

 
#line 3399 ".\\user\\stm32f10x.h"

 




























 





















 




























 





















 
#line 3517 ".\\user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 3552 ".\\user\\stm32f10x.h"





#line 3563 ".\\user\\stm32f10x.h"

 
#line 3571 ".\\user\\stm32f10x.h"

#line 3578 ".\\user\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 3600 ".\\user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 3662 ".\\user\\stm32f10x.h"



 
#line 3674 ".\\user\\stm32f10x.h"







 


 
 
 
 
 

 











#line 3711 ".\\user\\stm32f10x.h"

 











#line 3733 ".\\user\\stm32f10x.h"

 











#line 3755 ".\\user\\stm32f10x.h"

 











#line 3777 ".\\user\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 4174 ".\\user\\stm32f10x.h"

 
#line 4183 ".\\user\\stm32f10x.h"

 
#line 4192 ".\\user\\stm32f10x.h"

 
#line 4203 ".\\user\\stm32f10x.h"

#line 4213 ".\\user\\stm32f10x.h"

#line 4223 ".\\user\\stm32f10x.h"

#line 4233 ".\\user\\stm32f10x.h"

 
#line 4244 ".\\user\\stm32f10x.h"

#line 4254 ".\\user\\stm32f10x.h"

#line 4264 ".\\user\\stm32f10x.h"

#line 4274 ".\\user\\stm32f10x.h"

 
#line 4285 ".\\user\\stm32f10x.h"

#line 4295 ".\\user\\stm32f10x.h"

#line 4305 ".\\user\\stm32f10x.h"

#line 4315 ".\\user\\stm32f10x.h"

 
#line 4326 ".\\user\\stm32f10x.h"

#line 4336 ".\\user\\stm32f10x.h"

#line 4346 ".\\user\\stm32f10x.h"

#line 4356 ".\\user\\stm32f10x.h"

 
#line 4367 ".\\user\\stm32f10x.h"

#line 4377 ".\\user\\stm32f10x.h"

#line 4387 ".\\user\\stm32f10x.h"

#line 4397 ".\\user\\stm32f10x.h"

 
#line 4408 ".\\user\\stm32f10x.h"

#line 4418 ".\\user\\stm32f10x.h"

#line 4428 ".\\user\\stm32f10x.h"

#line 4438 ".\\user\\stm32f10x.h"

 
#line 4449 ".\\user\\stm32f10x.h"

#line 4459 ".\\user\\stm32f10x.h"

#line 4469 ".\\user\\stm32f10x.h"

#line 4479 ".\\user\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 4527 ".\\user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 4597 ".\\user\\stm32f10x.h"

 
#line 4612 ".\\user\\stm32f10x.h"

 
#line 4638 ".\\user\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 4859 ".\\user\\stm32f10x.h"

 
#line 4871 ".\\user\\stm32f10x.h"

 






 
#line 4888 ".\\user\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5032 ".\\user\\stm32f10x.h"



 


#line 5044 ".\\user\\stm32f10x.h"



 


#line 5056 ".\\user\\stm32f10x.h"



 


#line 5068 ".\\user\\stm32f10x.h"



 


#line 5080 ".\\user\\stm32f10x.h"



 


#line 5092 ".\\user\\stm32f10x.h"



 


#line 5104 ".\\user\\stm32f10x.h"



 


#line 5116 ".\\user\\stm32f10x.h"



 

 


#line 5130 ".\\user\\stm32f10x.h"



 


#line 5142 ".\\user\\stm32f10x.h"



 


#line 5154 ".\\user\\stm32f10x.h"



 


#line 5166 ".\\user\\stm32f10x.h"



 


#line 5178 ".\\user\\stm32f10x.h"



 


#line 5190 ".\\user\\stm32f10x.h"



 


#line 5202 ".\\user\\stm32f10x.h"



 


#line 5214 ".\\user\\stm32f10x.h"



 


#line 5226 ".\\user\\stm32f10x.h"



 


#line 5238 ".\\user\\stm32f10x.h"



 


#line 5250 ".\\user\\stm32f10x.h"



 


#line 5262 ".\\user\\stm32f10x.h"



 


#line 5274 ".\\user\\stm32f10x.h"



 


#line 5286 ".\\user\\stm32f10x.h"



 


#line 5298 ".\\user\\stm32f10x.h"



 


#line 5310 ".\\user\\stm32f10x.h"



 
 
 
 
 

 
 
#line 5330 ".\\user\\stm32f10x.h"

 
#line 5341 ".\\user\\stm32f10x.h"

 
#line 5359 ".\\user\\stm32f10x.h"











 





 





 
#line 5397 ".\\user\\stm32f10x.h"

 












 
#line 5418 ".\\user\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 5558 ".\\user\\stm32f10x.h"

 
#line 5575 ".\\user\\stm32f10x.h"

 
#line 5592 ".\\user\\stm32f10x.h"

 
#line 5609 ".\\user\\stm32f10x.h"

 
#line 5643 ".\\user\\stm32f10x.h"

 
#line 5677 ".\\user\\stm32f10x.h"

 
#line 5711 ".\\user\\stm32f10x.h"

 
#line 5745 ".\\user\\stm32f10x.h"

 
#line 5779 ".\\user\\stm32f10x.h"

 
#line 5813 ".\\user\\stm32f10x.h"

 
#line 5847 ".\\user\\stm32f10x.h"

 
#line 5881 ".\\user\\stm32f10x.h"

 
#line 5915 ".\\user\\stm32f10x.h"

 
#line 5949 ".\\user\\stm32f10x.h"

 
#line 5983 ".\\user\\stm32f10x.h"

 
#line 6017 ".\\user\\stm32f10x.h"

 
#line 6051 ".\\user\\stm32f10x.h"

 
#line 6085 ".\\user\\stm32f10x.h"

 
#line 6119 ".\\user\\stm32f10x.h"

 
#line 6153 ".\\user\\stm32f10x.h"

 
#line 6187 ".\\user\\stm32f10x.h"

 
#line 6221 ".\\user\\stm32f10x.h"

 
#line 6255 ".\\user\\stm32f10x.h"

 
#line 6289 ".\\user\\stm32f10x.h"

 
#line 6323 ".\\user\\stm32f10x.h"

 
#line 6357 ".\\user\\stm32f10x.h"

 
#line 6391 ".\\user\\stm32f10x.h"

 
#line 6425 ".\\user\\stm32f10x.h"

 
#line 6459 ".\\user\\stm32f10x.h"

 
#line 6493 ".\\user\\stm32f10x.h"

 
#line 6527 ".\\user\\stm32f10x.h"

 
#line 6561 ".\\user\\stm32f10x.h"

 
 
 
 
 

 









#line 6588 ".\\user\\stm32f10x.h"

 
#line 6596 ".\\user\\stm32f10x.h"

 
#line 6606 ".\\user\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 6667 ".\\user\\stm32f10x.h"

 
#line 6676 ".\\user\\stm32f10x.h"







 



#line 6697 ".\\user\\stm32f10x.h"



 



 


 
#line 6722 ".\\user\\stm32f10x.h"

 
#line 6732 ".\\user\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 6758 ".\\user\\stm32f10x.h"

 


 



 
#line 6781 ".\\user\\stm32f10x.h"

 
#line 6790 ".\\user\\stm32f10x.h"







 
#line 6809 ".\\user\\stm32f10x.h"

 
#line 6820 ".\\user\\stm32f10x.h"



 
 
 
 
 

 


#line 6849 ".\\user\\stm32f10x.h"

 









#line 6873 ".\\user\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 6913 ".\\user\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 





 

 

  

#line 1 ".\\FWlib\\inc\\stm32f10x_conf.h"


















 

 



 
 
 
 
 
 
 
 
 
 
 
 
#line 1 ".\\FWlib\\inc\\stm32f10x_gpio.h"



















  

 



 
#line 1 ".\\user\\stm32f10x.h"





















 



 



 
 
#line 6999 ".\\user\\stm32f10x.h"



 

  

 

 
#line 28 ".\\FWlib\\inc\\stm32f10x_gpio.h"



 



 



 

#line 48 ".\\FWlib\\inc\\stm32f10x_gpio.h"
                                     


 

typedef enum
{ 
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;





 

typedef enum
{ GPIO_Mode_AIN = 0x0,		       
  GPIO_Mode_IN_FLOATING = 0x04,	   
  GPIO_Mode_IPD = 0x28,	           
  GPIO_Mode_IPU = 0x48,			   
  GPIO_Mode_Out_OD = 0x14,		   
  GPIO_Mode_Out_PP = 0x10,		   
  GPIO_Mode_AF_OD = 0x1C,		   
  GPIO_Mode_AF_PP = 0x18		   
}GPIOMode_TypeDef;








 

typedef struct
{
  uint16_t GPIO_Pin;
  GPIOSpeed_TypeDef GPIO_Speed;
  GPIOMode_TypeDef GPIO_Mode;
}GPIO_InitTypeDef;



 

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;





 



 



 

#line 133 ".\\FWlib\\inc\\stm32f10x_gpio.h"



#line 152 ".\\FWlib\\inc\\stm32f10x_gpio.h"



 



 

#line 186 ".\\FWlib\\inc\\stm32f10x_gpio.h"

#line 200 ".\\FWlib\\inc\\stm32f10x_gpio.h"
                              


  



 

#line 221 ".\\FWlib\\inc\\stm32f10x_gpio.h"

#line 229 ".\\FWlib\\inc\\stm32f10x_gpio.h"



 



 

#line 254 ".\\FWlib\\inc\\stm32f10x_gpio.h"

#line 271 ".\\FWlib\\inc\\stm32f10x_gpio.h"



 



 



 



 



 

void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);




 



 



 

 
#line 38 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
 
 
#line 1 ".\\FWlib\\inc\\stm32f10x_rcc.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_rcc.h"



 



 



 

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK1_Frequency;
  uint32_t PCLK2_Frequency;
  uint32_t ADCCLK_Frequency;
}RCC_ClocksTypeDef;



 



 



 









  



 

#line 82 ".\\FWlib\\inc\\stm32f10x_rcc.h"


  



 

#line 113 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 127 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 149 ".\\FWlib\\inc\\stm32f10x_rcc.h"


  



 

#line 165 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 183 ".\\FWlib\\inc\\stm32f10x_rcc.h"




 



 







 



 

#line 211 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 








 



 

#line 238 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 254 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 278 ".\\FWlib\\inc\\stm32f10x_rcc.h"




  



 

#line 309 ".\\FWlib\\inc\\stm32f10x_rcc.h"




 



 

#line 327 ".\\FWlib\\inc\\stm32f10x_rcc.h"


 



 

#line 352 ".\\FWlib\\inc\\stm32f10x_rcc.h"




 



 



 



 



 

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
void RCC_USBCLKConfig(uint32_t RCC_USBCLKSource);
void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);
void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);




 



 



  

 
#line 42 ".\\FWlib\\inc\\stm32f10x_conf.h"
 
 
 
 
 
 
    

 
 

 
 

 
#line 73 ".\\FWlib\\inc\\stm32f10x_conf.h"



 
#line 6974 ".\\user\\stm32f10x.h"




 

















 





 

  

 

 
#line 28 ".\\FWlib\\inc\\stm32f10x_adc.h"
#line 29 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 



 



 

typedef struct
{
  uint32_t ADC_Mode;
  FunctionalState ADC_ScanConvMode; 
  FunctionalState ADC_ContinuousConvMode;
  uint32_t ADC_ExternalTrigConv;
  uint32_t ADC_DataAlign;
  uint8_t ADC_NbrOfChannel;
}ADC_InitTypeDef;


 



 










 

#line 83 ".\\FWlib\\inc\\stm32f10x_adc.h"

#line 94 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 



 

#line 112 ".\\FWlib\\inc\\stm32f10x_adc.h"



 






 

#line 130 ".\\FWlib\\inc\\stm32f10x_adc.h"

#line 145 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 







 



 

#line 183 ".\\FWlib\\inc\\stm32f10x_adc.h"

#line 193 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 217 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 



 









 







 







#line 266 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 282 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 

#line 297 ".\\FWlib\\inc\\stm32f10x_adc.h"

#line 305 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 











 



 

#line 338 ".\\FWlib\\inc\\stm32f10x_adc.h"


 



 





 



 





 



 





 



 





  




 




 



 





 



 





 



 



 



 



 

void ADC_DeInit(ADC_TypeDef* ADCx);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
void ADC_ResetCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_StartCalibration(ADC_TypeDef* ADCx);
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx);
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx);
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number);
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
uint32_t ADC_GetDualModeConversionValue(void);
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv);
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx);
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length);
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel);
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog);
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel);
void ADC_TempSensorVrefintCmd(FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG);
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);





 



 



 

 
#line 23 "FWlib\\SRC\\stm32f10x_adc.c"
#line 24 "FWlib\\SRC\\stm32f10x_adc.c"



 




 



 



 



 

 


 



 



 



 


 


 


 



 



 


 


 


 



 



 


 



 


 



 



 


 




 


 


 



 



 


 




 



 



 



 



 



 



 



 






 
void ADC_DeInit(ADC_TypeDef* ADCx)
{
   
  ((void)0);
  switch (*(uint32_t*)&ADCx)
  {
    case ((((uint32_t)0x40000000) + 0x10000) + 0x2400):
       
      RCC_APB2PeriphResetCmd(((uint32_t)0x00000200), ENABLE);
       
      RCC_APB2PeriphResetCmd(((uint32_t)0x00000200), DISABLE);
      break;
    
    case ((((uint32_t)0x40000000) + 0x10000) + 0x2800):
       
      RCC_APB2PeriphResetCmd(((uint32_t)0x00000400), ENABLE);
       
      RCC_APB2PeriphResetCmd(((uint32_t)0x00000400), DISABLE);
      break;
      
    case ((((uint32_t)0x40000000) + 0x10000) + 0x3C00):
       
      RCC_APB2PeriphResetCmd(((uint32_t)0x00008000), ENABLE);
       
      RCC_APB2PeriphResetCmd(((uint32_t)0x00008000), DISABLE);
      break; 
    default:
      break;
  }
}









 
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
{
  uint32_t tmpreg1 = 0;
  uint8_t tmpreg2 = 0;
   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);   
  ((void)0); 
  ((void)0);
   
   
  tmpreg1 = ADCx->CR1;
   
  tmpreg1 &= ((uint32_t)0xFFF0FEFF);
   
   
   
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_Mode | ((uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8));
   
  ADCx->CR1 = tmpreg1;
   
   
  tmpreg1 = ADCx->CR2;
   
  tmpreg1 &= ((uint32_t)0xFFF1F7FD);
   
   
   
   
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ExternalTrigConv |
            ((uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1));
   
  ADCx->CR2 = tmpreg1;
   
   
  tmpreg1 = ADCx->SQR1;
   
  tmpreg1 &= ((uint32_t)0xFF0FFFFF);
   
   
  tmpreg2 |= (ADC_InitStruct->ADC_NbrOfChannel - 1);
  tmpreg1 |= ((uint32_t)tmpreg2 << 20);
   
  ADCx->SQR1 = tmpreg1;
}






 
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
   
   
  ADC_InitStruct->ADC_Mode = ((uint32_t)0x00000000);
   
  ADC_InitStruct->ADC_ScanConvMode = DISABLE;
   
  ADC_InitStruct->ADC_ContinuousConvMode = DISABLE;
   
  ADC_InitStruct->ADC_ExternalTrigConv = ((uint32_t)0x00000000);
   
  ADC_InitStruct->ADC_DataAlign = ((uint32_t)0x00000000);
   
  ADC_InitStruct->ADC_NbrOfChannel = 1;
}







 
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
     
    ADCx->CR2 |= ((uint32_t)0x00000001);
  }
  else
  {
     
    ADCx->CR2 &= ((uint32_t)0xFFFFFFFE);
  }
}








 
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
     
    ADCx->CR2 |= ((uint32_t)0x00000100);
  }
  else
  {
     
    ADCx->CR2 &= ((uint32_t)0xFFFFFEFF);
  }
}













 
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState)
{
  uint8_t itmask = 0;
   
  ((void)0);
  ((void)0);
  ((void)0);
   
  itmask = (uint8_t)ADC_IT;
  if (NewState != DISABLE)
  {
     
    ADCx->CR1 |= itmask;
  }
  else
  {
     
    ADCx->CR1 &= (~(uint32_t)itmask);
  }
}





 
void ADC_ResetCalibration(ADC_TypeDef* ADCx)
{
   
  ((void)0);
     
  ADCx->CR2 |= ((uint32_t)0x00000008);
}





 
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;
   
  ((void)0);
   
  if ((ADCx->CR2 & ((uint32_t)0x00000008)) != (uint32_t)RESET)
  {
     
    bitstatus = SET;
  }
  else
  {
     
    bitstatus = RESET;
  }
   
  return  bitstatus;
}





 
void ADC_StartCalibration(ADC_TypeDef* ADCx)
{
   
  ((void)0);
     
  ADCx->CR2 |= ((uint32_t)0x00000004);
}





 
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;
   
  ((void)0);
   
  if ((ADCx->CR2 & ((uint32_t)0x00000004)) != (uint32_t)RESET)
  {
     
    bitstatus = SET;
  }
  else
  {
     
    bitstatus = RESET;
  }
   
  return  bitstatus;
}







 
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
    
 
    ADCx->CR2 |= ((uint32_t)0x00500000);
  }
  else
  {
    
 
    ADCx->CR2 &= ((uint32_t)0xFFAFFFFF);
  }
}





 
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;
   
  ((void)0);
   
  if ((ADCx->CR2 & ((uint32_t)0x00400000)) != (uint32_t)RESET)
  {
     
    bitstatus = SET;
  }
  else
  {
     
    bitstatus = RESET;
  }
   
  return  bitstatus;
}








 
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, uint8_t Number)
{
  uint32_t tmpreg1 = 0;
  uint32_t tmpreg2 = 0;
   
  ((void)0);
  ((void)0);
   
  tmpreg1 = ADCx->CR1;
   
  tmpreg1 &= ((uint32_t)0xFFFF1FFF);
   
  tmpreg2 = Number - 1;
  tmpreg1 |= tmpreg2 << 13;
   
  ADCx->CR1 = tmpreg1;
}









 
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
     
    ADCx->CR1 |= ((uint32_t)0x00000800);
  }
  else
  {
     
    ADCx->CR1 &= ((uint32_t)0xFFFFF7FF);
  }
}







































 
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
  uint32_t tmpreg1 = 0, tmpreg2 = 0;
   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
   
  if (ADC_Channel > ((uint8_t)0x09))
  {
     
    tmpreg1 = ADCx->SMPR1;
     
    tmpreg2 = ((uint32_t)0x00000007) << (3 * (ADC_Channel - 10));
     
    tmpreg1 &= ~tmpreg2;
     
    tmpreg2 = (uint32_t)ADC_SampleTime << (3 * (ADC_Channel - 10));
     
    tmpreg1 |= tmpreg2;
     
    ADCx->SMPR1 = tmpreg1;
  }
  else  
  {
     
    tmpreg1 = ADCx->SMPR2;
     
    tmpreg2 = ((uint32_t)0x00000007) << (3 * ADC_Channel);
     
    tmpreg1 &= ~tmpreg2;
     
    tmpreg2 = (uint32_t)ADC_SampleTime << (3 * ADC_Channel);
     
    tmpreg1 |= tmpreg2;
     
    ADCx->SMPR2 = tmpreg1;
  }
   
  if (Rank < 7)
  {
     
    tmpreg1 = ADCx->SQR3;
     
    tmpreg2 = ((uint32_t)0x0000001F) << (5 * (Rank - 1));
     
    tmpreg1 &= ~tmpreg2;
     
    tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 1));
     
    tmpreg1 |= tmpreg2;
     
    ADCx->SQR3 = tmpreg1;
  }
   
  else if (Rank < 13)
  {
     
    tmpreg1 = ADCx->SQR2;
     
    tmpreg2 = ((uint32_t)0x0000001F) << (5 * (Rank - 7));
     
    tmpreg1 &= ~tmpreg2;
     
    tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 7));
     
    tmpreg1 |= tmpreg2;
     
    ADCx->SQR2 = tmpreg1;
  }
   
  else
  {
     
    tmpreg1 = ADCx->SQR1;
     
    tmpreg2 = ((uint32_t)0x0000001F) << (5 * (Rank - 13));
     
    tmpreg1 &= ~tmpreg2;
     
    tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 13));
     
    tmpreg1 |= tmpreg2;
     
    ADCx->SQR1 = tmpreg1;
  }
}








 
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
     
    ADCx->CR2 |= ((uint32_t)0x00100000);
  }
  else
  {
     
    ADCx->CR2 &= ((uint32_t)0xFFEFFFFF);
  }
}





 
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx)
{
   
  ((void)0);
   
  return (uint16_t) ADCx->DR;
}




 
uint32_t ADC_GetDualModeConversionValue(void)
{
   
  return (*(volatile uint32_t *) ((uint32_t)0x4001244C));
}









 
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
     
    ADCx->CR1 |= ((uint32_t)0x00000400);
  }
  else
  {
     
    ADCx->CR1 &= ((uint32_t)0xFFFFFBFF);
  }
}









 
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
     
    ADCx->CR1 |= ((uint32_t)0x00001000);
  }
  else
  {
     
    ADCx->CR1 &= ((uint32_t)0xFFFFEFFF);
  }
}




































 
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, uint32_t ADC_ExternalTrigInjecConv)
{
  uint32_t tmpreg = 0;
   
  ((void)0);
  ((void)0);
   
  tmpreg = ADCx->CR2;
   
  tmpreg &= ((uint32_t)0xFFFF8FFF);
   
  tmpreg |= ADC_ExternalTrigInjecConv;
   
  ADCx->CR2 = tmpreg;
}









 
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
     
    ADCx->CR2 |= ((uint32_t)0x00008000);
  }
  else
  {
     
    ADCx->CR2 &= ((uint32_t)0xFFFF7FFF);
  }
}









 
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
   
  ((void)0);
  ((void)0);
  if (NewState != DISABLE)
  {
    
 
    ADCx->CR2 |= ((uint32_t)0x00208000);
  }
  else
  {
    
 
    ADCx->CR2 &= ((uint32_t)0xFFDF7FFF);
  }
}





 
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;
   
  ((void)0);
   
  if ((ADCx->CR2 & ((uint32_t)0x00200000)) != (uint32_t)RESET)
  {
     
    bitstatus = SET;
  }
  else
  {
     
    bitstatus = RESET;
  }
   
  return  bitstatus;
}







































 
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
  uint32_t tmpreg1 = 0, tmpreg2 = 0, tmpreg3 = 0;
   
  ((void)0);
  ((void)0);
  ((void)0);
  ((void)0);
   
  if (ADC_Channel > ((uint8_t)0x09))
  {
     
    tmpreg1 = ADCx->SMPR1;
     
    tmpreg2 = ((uint32_t)0x00000007) << (3*(ADC_Channel - 10));
     
    tmpreg1 &= ~tmpreg2;
     
    tmpreg2 = (uint32_t)ADC_SampleTime << (3*(ADC_Channel - 10));
     
    tmpreg1 |= tmpreg2;
     
    ADCx->SMPR1 = tmpreg1;
  }
  else  
  {
     
    tmpreg1 = ADCx->SMPR2;
     
    tmpreg2 = ((uint32_t)0x00000007) << (3 * ADC_Channel);
     
    tmpreg1 &= ~tmpreg2;
     
    tmpreg2 = (uint32_t)ADC_SampleTime << (3 * ADC_Channel);
     
    tmpreg1 |= tmpreg2;
     
    ADCx->SMPR2 = tmpreg1;
  }
   
   
  tmpreg1 = ADCx->JSQR;
   
  tmpreg3 =  (tmpreg1 & ((uint32_t)0x00300000))>> 20;
   
  tmpreg2 = ((uint32_t)0x0000001F) << (5 * (uint8_t)((Rank + 3) - (tmpreg3 + 1)));
   
  tmpreg1 &= ~tmpreg2;
   
  tmpreg2 = (uint32_t)ADC_Channel << (5 * (uint8_t)((Rank + 3) - (tmpreg3 + 1)));
   
  tmpreg1 |= tmpreg2;
   
  ADCx->JSQR = tmpreg1;
}







 
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, uint8_t Length)
{
  uint32_t tmpreg1 = 0;
  uint32_t tmpreg2 = 0;
   
  ((void)0);
  ((void)0);
  
   
  tmpreg1 = ADCx->JSQR;
   
  tmpreg1 &= ((uint32_t)0xFFCFFFFF);
   
  tmpreg2 = Length - 1; 
  tmpreg1 |= tmpreg2 << 20;
   
  ADCx->JSQR = tmpreg1;
}














 
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset)
{
   
  ((void)0);
  ((void)0);
  ((void)0);  
   
  *((volatile uint32_t *)((*(uint32_t*)&ADCx) + ADC_InjectedChannel)) = (uint32_t)Offset;
}











 
uint16_t ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, uint8_t ADC_InjectedChannel)
{
   
  ((void)0);
  ((void)0);
   
  return (uint16_t) (*(volatile uint32_t*) (((*(uint32_t*)&ADCx) + ADC_InjectedChannel + ((uint8_t)0x28))));
}






















 
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, uint32_t ADC_AnalogWatchdog)
{
  uint32_t tmpreg = 0;
   
  ((void)0);
  ((void)0);
   
  tmpreg = ADCx->CR1;
   
  tmpreg &= ((uint32_t)0xFF3FFDFF);
   
  tmpreg |= ADC_AnalogWatchdog;
   
  ADCx->CR1 = tmpreg;
}









 
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, uint16_t HighThreshold,
                                        uint16_t LowThreshold)
{
   
  ((void)0);
  ((void)0);
  ((void)0);
   
  ADCx->HTR = HighThreshold;
   
  ADCx->LTR = LowThreshold;
}


























 
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel)
{
  uint32_t tmpreg = 0;
   
  ((void)0);
  ((void)0);
   
  tmpreg = ADCx->CR1;
   
  tmpreg &= ((uint32_t)0xFFFFFFE0);
   
  tmpreg |= ADC_Channel;
   
  ADCx->CR1 = tmpreg;
}






 
void ADC_TempSensorVrefintCmd(FunctionalState NewState)
{
   
  ((void)0);
  if (NewState != DISABLE)
  {
     
    ((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400))->CR2 |= ((uint32_t)0x00800000);
  }
  else
  {
     
    ((ADC_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2400))->CR2 &= ((uint32_t)0xFF7FFFFF);
  }
}












 
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint8_t ADC_FLAG)
{
  FlagStatus bitstatus = RESET;
   
  ((void)0);
  ((void)0);
   
  if ((ADCx->SR & ADC_FLAG) != (uint8_t)RESET)
  {
     
    bitstatus = SET;
  }
  else
  {
     
    bitstatus = RESET;
  }
   
  return  bitstatus;
}












 
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint8_t ADC_FLAG)
{
   
  ((void)0);
  ((void)0);
   
  ADCx->SR = ~(uint32_t)ADC_FLAG;
}










 
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t itmask = 0, enablestatus = 0;
   
  ((void)0);
  ((void)0);
   
  itmask = ADC_IT >> 8;
   
  enablestatus = (ADCx->CR1 & (uint8_t)ADC_IT) ;
   
  if (((ADCx->SR & itmask) != (uint32_t)RESET) && enablestatus)
  {
     
    bitstatus = SET;
  }
  else
  {
     
    bitstatus = RESET;
  }
   
  return  bitstatus;
}










 
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT)
{
  uint8_t itmask = 0;
   
  ((void)0);
  ((void)0);
   
  itmask = (uint8_t)(ADC_IT >> 8);
   
  ADCx->SR = ~(uint32_t)itmask;
}



 



 



 

 
