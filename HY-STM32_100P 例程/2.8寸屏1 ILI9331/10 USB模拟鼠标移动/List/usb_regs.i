#line 1 "usb_library\\src\\usb_regs.c"













 

 
#line 1 ".\\usb_library\\inc\\usb_lib.h"













 

 



 

#line 1 ".\\usb_library\\inc\\usb_regs.h"













 
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

 


 









 


 

 



 



 



 



 



 



 



 





 

 

  

#line 1 ".\\usb_library\\inc\\stm32f10x_conf.h"













 

 



 
#line 1 ".\\user\\stm32f10x.h"





















 



 



 
 
#line 6999 ".\\user\\stm32f10x.h"



 

  

 

 
#line 22 ".\\usb_library\\inc\\stm32f10x_conf.h"

 
 


 
 

 
 




 


 


 









 


 



 
 

 








 




 


 


 


 


 


 




 


 


 





 





 



 


 
#line 139 ".\\usb_library\\inc\\stm32f10x_conf.h"



 
#line 6974 ".\\user\\stm32f10x.h"




 

















 





 

  

 

 
#line 16 ".\\usb_library\\inc\\usb_regs.h"
 



 
 
typedef enum _EP_DBUF_DIR
{
   
  EP_DBUF_ERR,
  EP_DBUF_OUT,
  EP_DBUF_IN
}EP_DBUF_DIR;

 
enum EP_BUF_NUM
{
  EP_NOBUF,
  EP_BUF0,
  EP_BUF1
};

 



 
 
 

 

 

 

 

 

 
 
 


 
#line 70 ".\\usb_library\\inc\\usb_regs.h"
 
 
 
#line 81 ".\\usb_library\\inc\\usb_regs.h"





#line 94 ".\\usb_library\\inc\\usb_regs.h"

 
 
 
#line 106 ".\\usb_library\\inc\\usb_regs.h"








 
 
 





 
 
 


 
 
 
 
#line 141 ".\\usb_library\\inc\\usb_regs.h"

 


 
#line 152 ".\\usb_library\\inc\\usb_regs.h"


 


 
#line 165 ".\\usb_library\\inc\\usb_regs.h"

 
#line 174 ".\\usb_library\\inc\\usb_regs.h"
 
 


 


 


 


 


 


 


 


 


 



 









 









 









 
#line 248 ".\\usb_library\\inc\\usb_regs.h"








 
#line 269 ".\\usb_library\\inc\\usb_regs.h"







 










 










 











 











 









 









 











 











 











 









 














 









 










 
#line 431 ".\\usb_library\\inc\\usb_regs.h"

#line 438 ".\\usb_library\\inc\\usb_regs.h"




















 











 










 











 











 












 
#line 527 ".\\usb_library\\inc\\usb_regs.h"

#line 536 ".\\usb_library\\inc\\usb_regs.h"












 




 
extern volatile u16 wIstr;   

 
void SetCNTR(u16  );
void SetISTR(u16  );
void SetDADDR(u16  );
void SetBTABLE(u16  );
void SetBTABLE(u16  );
u16 GetCNTR(void);
u16 GetISTR(void);
u16 GetFNR(void);
u16 GetDADDR(void);
u16 GetBTABLE(void);
void SetENDPOINT(u8  , u16  );
u16 GetENDPOINT(u8  );
void SetEPType(u8  , u16  );
u16 GetEPType(u8  );
void SetEPTxStatus(u8  , u16  );
void SetEPRxStatus(u8  , u16  );
void SetDouBleBuffEPStall(u8  , u8 bDir);
u16 GetEPTxStatus(u8  );
u16 GetEPRxStatus(u8  );
void SetEPTxValid(u8  );
void SetEPRxValid(u8  );
u16 GetTxStallStatus(u8  );
u16 GetRxStallStatus(u8  );
void SetEP_KIND(u8  );
void ClearEP_KIND(u8  );
void Set_Status_Out(u8  );
void Clear_Status_Out(u8  );
void SetEPDoubleBuff(u8  );
void ClearEPDoubleBuff(u8  );
void ClearEP_CTR_RX(u8  );
void ClearEP_CTR_TX(u8  );
void ToggleDTOG_RX(u8  );
void ToggleDTOG_TX(u8  );
void ClearDTOG_RX(u8  );
void ClearDTOG_TX(u8  );
void SetEPAddress(u8  , u8  );
u8 GetEPAddress(u8  );
void SetEPTxAddr(u8  , u16  );
void SetEPRxAddr(u8  , u16  );
u16 GetEPTxAddr(u8  );
u16 GetEPRxAddr(u8  );
void SetEPCountRxReg(u32 *  , u16  );
void SetEPTxCount(u8  , u16  );
void SetEPRxCount(u8  , u16  );
u16 GetEPTxCount(u8  );
u16 GetEPRxCount(u8  );
void SetEPDblBuf0Addr(u8  , u16  );
void SetEPDblBuf1Addr(u8  , u16  );
void SetEPDblBuffAddr(u8  , u16  , u16  );
u16 GetEPDblBuf0Addr(u8  );
u16 GetEPDblBuf1Addr(u8  );
void SetEPDblBuffCount(u8  , u8  , u16  );
void SetEPDblBuf0Count(u8  , u8  , u16  );
void SetEPDblBuf1Count(u8  , u8  , u16  );
u16 GetEPDblBuf0Count(u8  );
u16 GetEPDblBuf1Count(u8  );
EP_DBUF_DIR GetEPDblBufDir(u8  );
void FreeUserBuffer(u8 bEpNum , u8 bDir);
u16 ToWord(u8, u8);
u16 ByteSwap(u16);



 
#line 23 ".\\usb_library\\inc\\usb_lib.h"
#line 1 ".\\usb_library\\inc\\usb_def.h"













 

 



 
 
typedef enum _RECIPIENT_TYPE
{
  DEVICE_RECIPIENT,      
  INTERFACE_RECIPIENT,   
  ENDPOINT_RECIPIENT,    
  OTHER_RECIPIENT
} RECIPIENT_TYPE;


typedef enum _STANDARD_REQUESTS
{
  GET_STATUS = 0,
  CLEAR_FEATURE,
  RESERVED1,
  SET_FEATURE,
  RESERVED2,
  SET_ADDRESS,
  GET_DESCRIPTOR,
  SET_DESCRIPTOR,
  GET_CONFIGURATION,
  SET_CONFIGURATION,
  GET_INTERFACE,
  SET_INTERFACE,
  TOTAL_sREQUEST,   
  SYNCH_FRAME = 12
} STANDARD_REQUESTS;

 
typedef enum _DESCRIPTOR_TYPE
{
  DEVICE_DESCRIPTOR = 1,
  CONFIG_DESCRIPTOR,
  STRING_DESCRIPTOR,
  INTERFACE_DESCRIPTOR,
  ENDPOINT_DESCRIPTOR
} DESCRIPTOR_TYPE;

 
typedef enum _FEATURE_SELECTOR
{
  ENDPOINT_STALL,
  DEVICE_REMOTE_WAKEUP
} FEATURE_SELECTOR;

 
 







 
 



 
#line 24 ".\\usb_library\\inc\\usb_lib.h"
#line 1 ".\\usb_library\\inc\\usb_core.h"













 

 



 
 
typedef enum _CONTROL_STATE
{
  WAIT_SETUP,        
  SETTING_UP,        
  IN_DATA,           
  OUT_DATA,          
  LAST_IN_DATA,      
  LAST_OUT_DATA,     
  WAIT_STATUS_IN,    
  WAIT_STATUS_OUT,   
  STALLED,           
  PAUSE              
} CONTROL_STATE;     

typedef struct OneDescriptor
{
  u8 *Descriptor;
  u16 Descriptor_Size;
}
ONE_DESCRIPTOR, *PONE_DESCRIPTOR;


 
typedef enum _RESULT
{
  USB_SUCCESS = 0,     
  USB_ERROR,
  USB_UNSUPPORT,
  USB_NOT_READY       
 
} RESULT;


 
typedef struct _ENDPOINT_INFO
{
  




















 
  u16  Usb_wLength;
  u16  Usb_wOffset;
  u16  PacketSize;
  u8   *(*CopyData)(u16 Length);
}
ENDPOINT_INFO;

 

typedef struct _DEVICE
{
  u8 Total_Endpoint;      
  u8 Total_Configuration; 
}
DEVICE;

typedef union
{
  u16 w;
  struct BW
  {
    u8 bb1;
    u8 bb0;
  }
  bw;
} u16_u8;

typedef struct _DEVICE_INFO
{
  u8 USBbmRequestType;        
  u8 USBbRequest;             
  u16_u8 USBwValues;          
  u16_u8 USBwIndexs;          
  u16_u8 USBwLengths;         

  u8 ControlState;            
  u8 Current_Feature;
  u8 Current_Configuration;    
  u8 Current_Interface;        
  u8 Current_AlternateSetting;
 

  ENDPOINT_INFO Ctrl_Info;
}
DEVICE_INFO;

typedef struct _DEVICE_PROP
{
  void (*Init)(void);         
  void (*Reset)(void);        

   
  void (*Process_Status_IN)(void);
  void (*Process_Status_OUT)(void);

   
  













 
  RESULT (*Class_Data_Setup)(u8 RequestNo);

   
  






 
  RESULT (*Class_NoData_Setup)(u8 RequestNo);

  





 

  RESULT  (*Class_Get_Interface_Setting)(u8 Interface, u8 AlternateSetting);

  u8* (*GetDeviceDescriptor)(u16 Length);
  u8* (*GetConfigDescriptor)(u16 Length);
  u8* (*GetStringDescriptor)(u16 Length);

  u8* RxEP_buffer;
  u8 MaxPacketSize;

}
DEVICE_PROP;

typedef struct _USER_STANDARD_REQUESTS
{
  void (*User_GetConfiguration)(void);        
  void (*User_SetConfiguration)(void);        
  void (*User_GetInterface)(void);            
  void (*User_SetInterface)(void);            
  void (*User_GetStatus)(void);               
  void (*User_ClearFeature)(void);            
  void (*User_SetEndPointFeature)(void);      
  void (*User_SetDeviceFeature)(void);        
  void (*User_SetDeviceAddress)(void);        
}
USER_STANDARD_REQUESTS;

 





#line 213 ".\\usb_library\\inc\\usb_core.h"

 
 
u8 Setup0_Process(void);
u8 Post0_Process(void);
u8 Out0_Process(void);
u8 In0_Process(void);

RESULT Standard_SetEndPointFeature(void);
RESULT Standard_SetDeviceFeature(void);

u8 *Standard_GetConfiguration(u16 Length);
RESULT Standard_SetConfiguration(void);
u8 *Standard_GetInterface(u16 Length);
RESULT Standard_SetInterface(void);
u8 *Standard_GetDescriptorData(u16 Length, PONE_DESCRIPTOR pDesc);

u8 *Standard_GetStatus(u16 Length);
RESULT Standard_ClearFeature(void);
void SetDeviceAddress(u8);
void NOP_Process(void);

extern DEVICE_PROP Device_Property;
extern  USER_STANDARD_REQUESTS User_Standard_Requests;
extern  DEVICE  Device_Table;
extern DEVICE_INFO Device_Info;

 
extern u16 SaveRState;
extern u16 SaveTState;



 
#line 25 ".\\usb_library\\inc\\usb_lib.h"
#line 1 ".\\usb_library\\inc\\usb_init.h"













 

 



 
 
 
 
 
void USB_Init(void);

 
 
extern u8	EPindex;
 
 
 
 
extern DEVICE_INFO*	pInformation;
 
 
extern DEVICE_PROP*	pProperty;
 
 
 
 
extern USER_STANDARD_REQUESTS *pUser_Standard_Requests;

extern u16	SaveState ;
extern u16 wInterrupt_Mask;



 
#line 26 ".\\usb_library\\inc\\usb_lib.h"
#line 1 ".\\usb_library\\inc\\usb_mem.h"













 

 



 
 
 
 
 
void UserToPMABufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes);
void PMAToUserBufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes);

 



 
#line 27 ".\\usb_library\\inc\\usb_lib.h"
#line 1 ".\\usb_library\\inc\\usb_int.h"














 

 



 
 
 
 
 
void CTR_LP(void);
void CTR_HP(void);

 



 
#line 28 ".\\usb_library\\inc\\usb_lib.h"

 
 
 
 
 



 
#line 18 "usb_library\\src\\usb_regs.c"

 
 
 
 
 
 
 







 
void SetCNTR(u16 wRegValue)
{
  (*((volatile unsigned *)((0x40005C00L) + 0x40)) = (u16)wRegValue);
}







 
u16 GetCNTR(void)
{
  return(((u16) *((volatile unsigned *)((0x40005C00L) + 0x40))));
}







 
void SetISTR(u16 wRegValue)
{
  (*((volatile unsigned *)((0x40005C00L) + 0x44)) = (u16)wRegValue);
}







 
u16 GetISTR(void)
{
  return(((u16) *((volatile unsigned *)((0x40005C00L) + 0x44))));
}







 
u16 GetFNR(void)
{
  return(((u16) *((volatile unsigned *)((0x40005C00L) + 0x48))));
}







 
void SetDADDR(u16 wRegValue)
{
  (*((volatile unsigned *)((0x40005C00L) + 0x4C)) = (u16)wRegValue);
}







 
u16 GetDADDR(void)
{
  return(((u16) *((volatile unsigned *)((0x40005C00L) + 0x4C))));
}







 
void SetBTABLE(u16 wRegValue)
{
  (*((volatile unsigned *)((0x40005C00L) + 0x50)) = (u16)(wRegValue & 0xFFF8));
}







 
u16 GetBTABLE(void)
{
  return(((u16) *((volatile unsigned *)((0x40005C00L) + 0x50))));
}








 
void SetENDPOINT(u8 bEpNum, u16 wRegValue)
{
  (*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)wRegValue);
}







 
u16 GetENDPOINT(u8 bEpNum)
{
  return(((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))));
}








 
void SetEPType(u8 bEpNum, u16 wType)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)((((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (~(0x0600) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F)))) | wType)));
}







 
u16 GetEPType(u8 bEpNum)
{
  return((((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (0x0600)));
}








 
void SetEPTxStatus(u8 bEpNum, u16 wState)
{
  { register u16 _wRegVal; _wRegVal = ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x0030)|((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))); if(((0x0010) & wState)!= 0) _wRegVal ^= (0x0010); if(((0x0020) & wState)!= 0) _wRegVal ^= (0x0020); (*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)_wRegVal); };
}








 
void SetEPRxStatus(u8 bEpNum, u16 wState)
{
  { register u16 _wRegVal; _wRegVal = ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x3000)|((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))); if(((0x1000) & wState)!= 0) _wRegVal ^= (0x1000); if(((0x2000) & wState)!= 0) _wRegVal ^= (0x2000); (*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)_wRegVal); };
}








 
void SetDouBleBuffEPStall(u8 bEpNum, u8 bDir)
{
  u16 Endpoint_DTOG_Status;
  Endpoint_DTOG_Status = GetENDPOINT(bEpNum);
  if (bDir == EP_DBUF_OUT)
  {  
    (*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)Endpoint_DTOG_Status & ~(0x1000));
  }
  else if (bDir == EP_DBUF_IN)
  {  
    (*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)Endpoint_DTOG_Status & ~(0x0010));
  }
}







 
u16 GetEPTxStatus(u8 bEpNum)
{
  return(((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (0x0030)));
}







 
u16 GetEPRxStatus(u8 bEpNum)
{
  return(((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (0x3000)));
}







 
void SetEPTxValid(u8 bEpNum)
{
  { register u16 _wRegVal; _wRegVal = ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x0030)|((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))); if(((0x0010) & (0x0030))!= 0) _wRegVal ^= (0x0010); if(((0x0020) & (0x0030))!= 0) _wRegVal ^= (0x0020); (*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)_wRegVal); };
}







 
void SetEPRxValid(u8 bEpNum)
{
  { register u16 _wRegVal; _wRegVal = ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x3000)|((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))); if(((0x1000) & (0x3000))!= 0) _wRegVal ^= (0x1000); if(((0x2000) & (0x3000))!= 0) _wRegVal ^= (0x2000); (*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)_wRegVal); };
}







 
void SetEP_KIND(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) | (0x0100)) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}







 
void ClearEP_KIND(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (~(0x0100) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))))));
}






 
void Clear_Status_Out(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (~(0x0100) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))))));
}






 
void Set_Status_Out(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) | (0x0100)) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}






 
void SetEPDoubleBuff(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) | (0x0100)) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}






 
void ClearEPDoubleBuff(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (~(0x0100) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))))));
}






 
u16 GetTxStallStatus(u8 bEpNum)
{
  return((((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (0x0030)) == (0x0010)));
}






 
u16 GetRxStallStatus(u8 bEpNum)
{
  return((((u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (0x3000)) == (0x1000)));
}






 
void ClearEP_CTR_RX(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & 0x7FFF & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}






 
void ClearEP_CTR_TX(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & 0xFF7F & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}






 
void ToggleDTOG_RX(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(0x4000) | ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}






 
void ToggleDTOG_TX(u8 bEpNum)
{
  ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(0x0040) | ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}






 
void ClearDTOG_RX(u8 bEpNum)
{
  if((((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (0x4000)) != 0) ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(0x4000) | ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}






 
void ClearDTOG_TX(u8 bEpNum)
{
  if((((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (0x0040)) != 0) ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(0x0040) | ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
}







 
void SetEPAddress(u8 bEpNum, u8 bAddr)
{
  (*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F)) | bAddr);
}






 
u8 GetEPAddress(u8 bEpNum)
{
  return(((u8)(((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & (0x000F))));
}







 
void SetEPTxAddr(u8 bEpNum, u16 wAddr)
{
  (*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8 )*2 + (0x40006000L))) = ((wAddr >> 1) << 1));
}







 
void SetEPRxAddr(u8 bEpNum, u16 wAddr)
{
  (*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+4)*2 + (0x40006000L))) = ((wAddr >> 1) << 1));
}






 
u16 GetEPTxAddr(u8 bEpNum)
{
  return(((u16)*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8 )*2 + (0x40006000L)))));
}






 
u16 GetEPRxAddr(u8 bEpNum)
{
  return(((u16)*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+4)*2 + (0x40006000L)))));
}







 
void SetEPTxCount(u8 bEpNum, u16 wCount)
{
  (*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+2)*2 + (0x40006000L))) = wCount);
}







 
void SetEPCountRxReg(u32 *pdwReg, u16 wCount)
{
  { u16 wNBlocks; if(wCount > 62){{ wNBlocks = wCount >> 5; if((wCount & 0x1f) == 0) wNBlocks--; *pdwReg = (u32)((wNBlocks << 10) | 0x8000); };} else {{ wNBlocks = wCount >> 1; if((wCount & 0x1) != 0) wNBlocks++; *pdwReg = (u32)(wNBlocks << 10); };} };
}







 
void SetEPRxCount(u8 bEpNum, u16 wCount)
{
  { u32 *pdwReg = ((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+6)*2 + (0x40006000L))); { u16 wNBlocks; if(wCount > 62){{ wNBlocks = wCount >> 5; if((wCount & 0x1f) == 0) wNBlocks--; *pdwReg = (u32)((wNBlocks << 10) | 0x8000); };} else {{ wNBlocks = wCount >> 1; if((wCount & 0x1) != 0) wNBlocks++; *pdwReg = (u32)(wNBlocks << 10); };} }; };
}






 
u16 GetEPTxCount(u8 bEpNum)
{
  return(((u16)(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+2)*2 + (0x40006000L)))) & 0x3ff));
}






 
u16 GetEPRxCount(u8 bEpNum)
{
  return(((u16)(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+6)*2 + (0x40006000L)))) & 0x3ff));
}








 
void SetEPDblBuffAddr(u8 bEpNum, u16 wBuf0Addr, u16 wBuf1Addr)
{
  { {(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8 )*2 + (0x40006000L))) = ((wBuf0Addr >> 1) << 1));}; {(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+4)*2 + (0x40006000L))) = ((wBuf1Addr >> 1) << 1));}; };
}







 
void SetEPDblBuf0Addr(u8 bEpNum, u16 wBuf0Addr)
{
  {(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8 )*2 + (0x40006000L))) = ((wBuf0Addr >> 1) << 1));};
}







 
void SetEPDblBuf1Addr(u8 bEpNum, u16 wBuf1Addr)
{
  {(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+4)*2 + (0x40006000L))) = ((wBuf1Addr >> 1) << 1));};
}






 
u16 GetEPDblBuf0Addr(u8 bEpNum)
{
  return((((u16)*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8 )*2 + (0x40006000L))))));
}






 
u16 GetEPDblBuf1Addr(u8 bEpNum)
{
  return((((u16)*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+4)*2 + (0x40006000L))))));
}







 
void SetEPDblBuffCount(u8 bEpNum, u8 bDir, u16 wCount)
{
  { { if(bDir == EP_DBUF_OUT) {{ u32 *pdwReg = ((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+2)*2 + (0x40006000L))); { u16 wNBlocks; if(wCount > 62){{ wNBlocks = wCount >> 5; if((wCount & 0x1f) == 0) wNBlocks--; *pdwReg = (u32)((wNBlocks << 10) | 0x8000); };} else {{ wNBlocks = wCount >> 1; if((wCount & 0x1) != 0) wNBlocks++; *pdwReg = (u32)(wNBlocks << 10); };} }; };} else if(bDir == EP_DBUF_IN) *((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+2)*2 + (0x40006000L))) = (u32)wCount; }; { if(bDir == EP_DBUF_OUT) {{ u32 *pdwReg = ((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+6)*2 + (0x40006000L))); { u16 wNBlocks; if(wCount > 62){{ wNBlocks = wCount >> 5; if((wCount & 0x1f) == 0) wNBlocks--; *pdwReg = (u32)((wNBlocks << 10) | 0x8000); };} else {{ wNBlocks = wCount >> 1; if((wCount & 0x1) != 0) wNBlocks++; *pdwReg = (u32)(wNBlocks << 10); };} }; };} else if(bDir == EP_DBUF_IN) *((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+6)*2 + (0x40006000L))) = (u32)wCount; }; };
}







 
void SetEPDblBuf0Count(u8 bEpNum, u8 bDir, u16 wCount)
{
  { if(bDir == EP_DBUF_OUT) {{ u32 *pdwReg = ((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+2)*2 + (0x40006000L))); { u16 wNBlocks; if(wCount > 62){{ wNBlocks = wCount >> 5; if((wCount & 0x1f) == 0) wNBlocks--; *pdwReg = (u32)((wNBlocks << 10) | 0x8000); };} else {{ wNBlocks = wCount >> 1; if((wCount & 0x1) != 0) wNBlocks++; *pdwReg = (u32)(wNBlocks << 10); };} }; };} else if(bDir == EP_DBUF_IN) *((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+2)*2 + (0x40006000L))) = (u32)wCount; };
}







 
void SetEPDblBuf1Count(u8 bEpNum, u8 bDir, u16 wCount)
{
  { if(bDir == EP_DBUF_OUT) {{ u32 *pdwReg = ((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+6)*2 + (0x40006000L))); { u16 wNBlocks; if(wCount > 62){{ wNBlocks = wCount >> 5; if((wCount & 0x1f) == 0) wNBlocks--; *pdwReg = (u32)((wNBlocks << 10) | 0x8000); };} else {{ wNBlocks = wCount >> 1; if((wCount & 0x1) != 0) wNBlocks++; *pdwReg = (u32)(wNBlocks << 10); };} }; };} else if(bDir == EP_DBUF_IN) *((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+6)*2 + (0x40006000L))) = (u32)wCount; };
}







 
u16 GetEPDblBuf0Count(u8 bEpNum)
{
  return((((u16)(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+2)*2 + (0x40006000L)))) & 0x3ff)));
}







 
u16 GetEPDblBuf1Count(u8 bEpNum)
{
  return((((u16)(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+6)*2 + (0x40006000L)))) & 0x3ff)));
}







 
EP_DBUF_DIR GetEPDblBufDir(u8 bEpNum)
{
  if ((u16)(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+6)*2 + (0x40006000L))) & 0xFC00) != 0)
    return(EP_DBUF_OUT);
  else if (((u16)(*((u32 *)((((u16) *((volatile unsigned *)((0x40005C00L) + 0x50)))+bEpNum*8+2)*2 + (0x40006000L)))) & 0x03FF) != 0)
    return(EP_DBUF_IN);
  else
    return(EP_DBUF_ERR);
}







 
void FreeUserBuffer(u8 bEpNum, u8 bDir)
{
  if (bDir == EP_DBUF_OUT)
  {  
    ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(0x0040) | ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
  }
  else if (bDir == EP_DBUF_IN)
  {  
    ((*(((volatile unsigned *)((0x40005C00L))) + bEpNum)= (u16)(0x4000) | ((u16)(*(((volatile unsigned *)((0x40005C00L))) + bEpNum))) & ((0x8000)|(0x0800)|(0x0600)|(0x0100)|(0x0080)|(0x000F))));
  }
}







 
u16 ToWord(u8 bh, u8 bl)
{
  u16 wRet;
  wRet = (u16)bl | ((u16)bh << 8);
  return(wRet);
}






 
u16 ByteSwap(u16 wSwW)
{
  u8 bTemp;
  u16 wRet;
  bTemp = (u8)(wSwW & 0xff);
  wRet =  (wSwW >> 8) | ((u16)bTemp << 8);
  return(wRet);
}

 
