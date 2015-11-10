#line 1 "user\\sdcard.c"














 

 
#line 1 "user\\sdcard.h"



















  



 



  

 



 
#line 1 "user\\stm32f10x.h"





















 



 



 
 


  


 
  


 















 

#line 67 "user\\stm32f10x.h"




 




 





 

 

 

 






 



 



 




 
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



 

#line 1 "user\\core_cm3.h"



















 




















































 

 
 
 
 
 
 
 
 


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



 


#line 86 "user\\core_cm3.h"

















 









 


 





 






 
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


 
#line 251 "user\\core_cm3.h"


 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;


 
#line 270 "user\\core_cm3.h"

#line 277 "user\\core_cm3.h"










 






#line 304 "user\\core_cm3.h"


 


 




#line 329 "user\\core_cm3.h"


   
   
   
   









 
extern uint32_t __get_PSP(void);









 
extern void __set_PSP(uint32_t topOfProcStack);









 
extern uint32_t __get_MSP(void);









 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 495 "user\\core_cm3.h"









 









 
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





#line 1024 "user\\core_cm3.h"



 










 
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



 
#line 197 "user\\stm32f10x.h"
#line 1 "user\\system_stm32f10x.h"


















 



 





 



 




 

extern const uint32_t SystemFrequency;                    
extern const uint32_t SystemFrequency_SysClk;             
extern const uint32_t SystemFrequency_AHBClk;             
extern const uint32_t SystemFrequency_APB1Clk;            
extern const uint32_t SystemFrequency_APB2Clk;            



 



 



 



 



 



 
  
extern void SystemInit(void);


 



 
#line 198 "user\\stm32f10x.h"
#line 199 "user\\stm32f10x.h"



   

 
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



 
  


 









 




#line 911 "user\\stm32f10x.h"

#line 928 "user\\stm32f10x.h"



#line 947 "user\\stm32f10x.h"














 
  


   

#line 1029 "user\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1090 "user\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1266 "user\\stm32f10x.h"

 




 








 










 
#line 1302 "user\\stm32f10x.h"






 











 










 














 
#line 1362 "user\\stm32f10x.h"








 






 
#line 1395 "user\\stm32f10x.h"

 
#line 1412 "user\\stm32f10x.h"

 
#line 1434 "user\\stm32f10x.h"

 
#line 1443 "user\\stm32f10x.h"

 
#line 1460 "user\\stm32f10x.h"

 
#line 1482 "user\\stm32f10x.h"

 








 








   
#line 1511 "user\\stm32f10x.h"

 
 
 
 
 

 




































































 




































































 
#line 1673 "user\\stm32f10x.h"

 
#line 1691 "user\\stm32f10x.h"

 
#line 1709 "user\\stm32f10x.h"

#line 1726 "user\\stm32f10x.h"

 
#line 1744 "user\\stm32f10x.h"

 
#line 1763 "user\\stm32f10x.h"

 

 






 
#line 1790 "user\\stm32f10x.h"






 








 









 








 








 









 










 




#line 1865 "user\\stm32f10x.h"






 





 





 
#line 1891 "user\\stm32f10x.h"

 
#line 1900 "user\\stm32f10x.h"

   
#line 1909 "user\\stm32f10x.h"

 
#line 1918 "user\\stm32f10x.h"

 





 
#line 1933 "user\\stm32f10x.h"

 
#line 1942 "user\\stm32f10x.h"

   
#line 1951 "user\\stm32f10x.h"

 
#line 1960 "user\\stm32f10x.h"

 





 
#line 1975 "user\\stm32f10x.h"

 
#line 1984 "user\\stm32f10x.h"

   
#line 1993 "user\\stm32f10x.h"

 
#line 2002 "user\\stm32f10x.h"

 





 
#line 2017 "user\\stm32f10x.h"

 
#line 2026 "user\\stm32f10x.h"

   
#line 2035 "user\\stm32f10x.h"

 
#line 2044 "user\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2108 "user\\stm32f10x.h"

 
#line 2143 "user\\stm32f10x.h"

 
#line 2178 "user\\stm32f10x.h"

 
#line 2213 "user\\stm32f10x.h"

 
#line 2248 "user\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 2315 "user\\stm32f10x.h"

 



 









 
#line 2339 "user\\stm32f10x.h"




 




 
#line 2355 "user\\stm32f10x.h"

 





 
#line 2377 "user\\stm32f10x.h"

 
 





 
#line 2392 "user\\stm32f10x.h"
 
#line 2399 "user\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 2447 "user\\stm32f10x.h"

 
#line 2468 "user\\stm32f10x.h"

 
#line 2489 "user\\stm32f10x.h"

 
#line 2510 "user\\stm32f10x.h"

 
#line 2531 "user\\stm32f10x.h"

 
#line 2552 "user\\stm32f10x.h"

 
 
 
 
 

 
#line 2588 "user\\stm32f10x.h"

 
#line 2618 "user\\stm32f10x.h"

 
#line 2628 "user\\stm32f10x.h"















 
#line 2652 "user\\stm32f10x.h"















 
#line 2676 "user\\stm32f10x.h"















 
#line 2700 "user\\stm32f10x.h"















 
#line 2724 "user\\stm32f10x.h"















 
#line 2748 "user\\stm32f10x.h"















 
#line 2772 "user\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 2873 "user\\stm32f10x.h"

#line 2882 "user\\stm32f10x.h"















  
 
#line 2905 "user\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3040 "user\\stm32f10x.h"

#line 3047 "user\\stm32f10x.h"

#line 3054 "user\\stm32f10x.h"

#line 3061 "user\\stm32f10x.h"







 
#line 3075 "user\\stm32f10x.h"

#line 3082 "user\\stm32f10x.h"

#line 3089 "user\\stm32f10x.h"

#line 3096 "user\\stm32f10x.h"

#line 3103 "user\\stm32f10x.h"

#line 3110 "user\\stm32f10x.h"

 
#line 3118 "user\\stm32f10x.h"

#line 3125 "user\\stm32f10x.h"

#line 3132 "user\\stm32f10x.h"

#line 3139 "user\\stm32f10x.h"

#line 3146 "user\\stm32f10x.h"

#line 3153 "user\\stm32f10x.h"

 
#line 3161 "user\\stm32f10x.h"

#line 3168 "user\\stm32f10x.h"

#line 3175 "user\\stm32f10x.h"

#line 3182 "user\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 
 
 
 
 

 
















 









#line 3332 "user\\stm32f10x.h"

 

























 
#line 3375 "user\\stm32f10x.h"

 
#line 3389 "user\\stm32f10x.h"

 
#line 3399 "user\\stm32f10x.h"

 




























 





















 




























 





















 
#line 3517 "user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 3552 "user\\stm32f10x.h"





#line 3563 "user\\stm32f10x.h"

 
#line 3571 "user\\stm32f10x.h"

#line 3578 "user\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 3600 "user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 3662 "user\\stm32f10x.h"



 
#line 3674 "user\\stm32f10x.h"







 


 
 
 
 
 

 











#line 3711 "user\\stm32f10x.h"

 











#line 3733 "user\\stm32f10x.h"

 











#line 3755 "user\\stm32f10x.h"

 











#line 3777 "user\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 4174 "user\\stm32f10x.h"

 
#line 4183 "user\\stm32f10x.h"

 
#line 4192 "user\\stm32f10x.h"

 
#line 4203 "user\\stm32f10x.h"

#line 4213 "user\\stm32f10x.h"

#line 4223 "user\\stm32f10x.h"

#line 4233 "user\\stm32f10x.h"

 
#line 4244 "user\\stm32f10x.h"

#line 4254 "user\\stm32f10x.h"

#line 4264 "user\\stm32f10x.h"

#line 4274 "user\\stm32f10x.h"

 
#line 4285 "user\\stm32f10x.h"

#line 4295 "user\\stm32f10x.h"

#line 4305 "user\\stm32f10x.h"

#line 4315 "user\\stm32f10x.h"

 
#line 4326 "user\\stm32f10x.h"

#line 4336 "user\\stm32f10x.h"

#line 4346 "user\\stm32f10x.h"

#line 4356 "user\\stm32f10x.h"

 
#line 4367 "user\\stm32f10x.h"

#line 4377 "user\\stm32f10x.h"

#line 4387 "user\\stm32f10x.h"

#line 4397 "user\\stm32f10x.h"

 
#line 4408 "user\\stm32f10x.h"

#line 4418 "user\\stm32f10x.h"

#line 4428 "user\\stm32f10x.h"

#line 4438 "user\\stm32f10x.h"

 
#line 4449 "user\\stm32f10x.h"

#line 4459 "user\\stm32f10x.h"

#line 4469 "user\\stm32f10x.h"

#line 4479 "user\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 4527 "user\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 4597 "user\\stm32f10x.h"

 
#line 4612 "user\\stm32f10x.h"

 
#line 4638 "user\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 4859 "user\\stm32f10x.h"

 
#line 4871 "user\\stm32f10x.h"

 






 
#line 4888 "user\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5032 "user\\stm32f10x.h"



 


#line 5044 "user\\stm32f10x.h"



 


#line 5056 "user\\stm32f10x.h"



 


#line 5068 "user\\stm32f10x.h"



 


#line 5080 "user\\stm32f10x.h"



 


#line 5092 "user\\stm32f10x.h"



 


#line 5104 "user\\stm32f10x.h"



 


#line 5116 "user\\stm32f10x.h"



 

 


#line 5130 "user\\stm32f10x.h"



 


#line 5142 "user\\stm32f10x.h"



 


#line 5154 "user\\stm32f10x.h"



 


#line 5166 "user\\stm32f10x.h"



 


#line 5178 "user\\stm32f10x.h"



 


#line 5190 "user\\stm32f10x.h"



 


#line 5202 "user\\stm32f10x.h"



 


#line 5214 "user\\stm32f10x.h"



 


#line 5226 "user\\stm32f10x.h"



 


#line 5238 "user\\stm32f10x.h"



 


#line 5250 "user\\stm32f10x.h"



 


#line 5262 "user\\stm32f10x.h"



 


#line 5274 "user\\stm32f10x.h"



 


#line 5286 "user\\stm32f10x.h"



 


#line 5298 "user\\stm32f10x.h"



 


#line 5310 "user\\stm32f10x.h"



 
 
 
 
 

 
 
#line 5330 "user\\stm32f10x.h"

 
#line 5341 "user\\stm32f10x.h"

 
#line 5359 "user\\stm32f10x.h"











 





 





 
#line 5397 "user\\stm32f10x.h"

 












 
#line 5418 "user\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 5558 "user\\stm32f10x.h"

 
#line 5575 "user\\stm32f10x.h"

 
#line 5592 "user\\stm32f10x.h"

 
#line 5609 "user\\stm32f10x.h"

 
#line 5643 "user\\stm32f10x.h"

 
#line 5677 "user\\stm32f10x.h"

 
#line 5711 "user\\stm32f10x.h"

 
#line 5745 "user\\stm32f10x.h"

 
#line 5779 "user\\stm32f10x.h"

 
#line 5813 "user\\stm32f10x.h"

 
#line 5847 "user\\stm32f10x.h"

 
#line 5881 "user\\stm32f10x.h"

 
#line 5915 "user\\stm32f10x.h"

 
#line 5949 "user\\stm32f10x.h"

 
#line 5983 "user\\stm32f10x.h"

 
#line 6017 "user\\stm32f10x.h"

 
#line 6051 "user\\stm32f10x.h"

 
#line 6085 "user\\stm32f10x.h"

 
#line 6119 "user\\stm32f10x.h"

 
#line 6153 "user\\stm32f10x.h"

 
#line 6187 "user\\stm32f10x.h"

 
#line 6221 "user\\stm32f10x.h"

 
#line 6255 "user\\stm32f10x.h"

 
#line 6289 "user\\stm32f10x.h"

 
#line 6323 "user\\stm32f10x.h"

 
#line 6357 "user\\stm32f10x.h"

 
#line 6391 "user\\stm32f10x.h"

 
#line 6425 "user\\stm32f10x.h"

 
#line 6459 "user\\stm32f10x.h"

 
#line 6493 "user\\stm32f10x.h"

 
#line 6527 "user\\stm32f10x.h"

 
#line 6561 "user\\stm32f10x.h"

 
 
 
 
 

 









#line 6588 "user\\stm32f10x.h"

 
#line 6596 "user\\stm32f10x.h"

 
#line 6606 "user\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 6667 "user\\stm32f10x.h"

 
#line 6676 "user\\stm32f10x.h"







 



#line 6697 "user\\stm32f10x.h"



 



 


 
#line 6722 "user\\stm32f10x.h"

 
#line 6732 "user\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 6758 "user\\stm32f10x.h"

 


 



 
#line 6781 "user\\stm32f10x.h"

 
#line 6790 "user\\stm32f10x.h"







 
#line 6809 "user\\stm32f10x.h"

 
#line 6820 "user\\stm32f10x.h"



 
 
 
 
 

 


#line 6849 "user\\stm32f10x.h"

 









#line 6873 "user\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 6913 "user\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 





 

 

  

#line 1 ".\\usb_library\\inc\\stm32f10x_conf.h"













 

 



 
#line 1 ".\\user\\stm32f10x.h"





















 



 



 
 
#line 6999 ".\\user\\stm32f10x.h"



 

  

 

 
#line 22 ".\\usb_library\\inc\\stm32f10x_conf.h"


 
 
 
 
 
 
 
 
#line 1 ".\\FWlib\\inc\\stm32f10x_dma.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_dma.h"



 



 



 



 

typedef struct
{
  uint32_t DMA_PeripheralBaseAddr;
  uint32_t DMA_MemoryBaseAddr;
  uint32_t DMA_DIR;
  uint32_t DMA_BufferSize;
  uint32_t DMA_PeripheralInc;
  uint32_t DMA_MemoryInc;
  uint32_t DMA_PeripheralDataSize;
  uint32_t DMA_MemoryDataSize;
  uint32_t DMA_Mode;
  uint32_t DMA_Priority;
  uint32_t DMA_M2M;
}DMA_InitTypeDef;



 



 

#line 80 ".\\FWlib\\inc\\stm32f10x_dma.h"



 







 



 







 



 







 



 

#line 127 ".\\FWlib\\inc\\stm32f10x_dma.h"


 



 

#line 141 ".\\FWlib\\inc\\stm32f10x_dma.h"


 



 






 



 

#line 168 ".\\FWlib\\inc\\stm32f10x_dma.h"


 



 







 



 








 

#line 225 ".\\FWlib\\inc\\stm32f10x_dma.h"



 

#line 250 ".\\FWlib\\inc\\stm32f10x_dma.h"



#line 277 ".\\FWlib\\inc\\stm32f10x_dma.h"



 



 



 

#line 318 ".\\FWlib\\inc\\stm32f10x_dma.h"



 

#line 343 ".\\FWlib\\inc\\stm32f10x_dma.h"



#line 370 ".\\FWlib\\inc\\stm32f10x_dma.h"


 



 





 



 



 



 



 

void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx);
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct);
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState);
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, uint32_t DMA_IT, FunctionalState NewState);
uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
FlagStatus DMA_GetFlagStatus(uint32_t DMA_FLAG);
void DMA_ClearFlag(uint32_t DMA_FLAG);
ITStatus DMA_GetITStatus(uint32_t DMA_IT);
void DMA_ClearITPendingBit(uint32_t DMA_IT);




 



 



 

 
#line 33 ".\\usb_library\\inc\\stm32f10x_conf.h"
 
 
 
#line 1 ".\\FWlib\\inc\\stm32f10x_gpio.h"



















  

 



 
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




 



 



 

 
#line 37 ".\\usb_library\\inc\\stm32f10x_conf.h"
 
 
 
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




 



 



  

 
#line 41 ".\\usb_library\\inc\\stm32f10x_conf.h"
 
#line 1 ".\\FWlib\\inc\\stm32f10x_sdio.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\stm32f10x_sdio.h"



 



 



 

typedef struct
{
  uint8_t SDIO_ClockDiv;
  uint32_t SDIO_ClockEdge;
  uint32_t SDIO_ClockBypass;
  uint32_t SDIO_ClockPowerSave;
  uint32_t SDIO_BusWide;
  uint32_t SDIO_HardwareFlowControl;
} SDIO_InitTypeDef;

typedef struct
{
  uint32_t SDIO_Argument;
  uint32_t SDIO_CmdIndex;
  uint32_t SDIO_Response;
  uint32_t SDIO_Wait;
  uint32_t SDIO_CPSM;
} SDIO_CmdInitTypeDef;

typedef struct
{
  uint32_t SDIO_DataTimeOut;
  uint32_t SDIO_DataLength;
  uint32_t SDIO_DataBlockSize;
  uint32_t SDIO_TransferDir;
  uint32_t SDIO_TransferMode;
  uint32_t SDIO_DPSM;
} SDIO_DataInitTypeDef;



  



 



 







 



 







  



 







 



 









 



 







 



 






  




 

#line 181 ".\\FWlib\\inc\\stm32f10x_sdio.h"


  



 




 



 

#line 204 ".\\FWlib\\inc\\stm32f10x_sdio.h"


 



 








 



 






  



 

#line 242 ".\\FWlib\\inc\\stm32f10x_sdio.h"


 



 




 



 

#line 289 ".\\FWlib\\inc\\stm32f10x_sdio.h"


 



 







 



 







 



 






 



 

#line 380 ".\\FWlib\\inc\\stm32f10x_sdio.h"



#line 407 ".\\FWlib\\inc\\stm32f10x_sdio.h"





 



 







 



 



 



 



 

void SDIO_DeInit(void);
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct);
void SDIO_ClockCmd(FunctionalState NewState);
void SDIO_SetPowerState(uint32_t SDIO_PowerState);
uint32_t SDIO_GetPowerState(void);
void SDIO_ITConfig(uint32_t SDIO_IT, FunctionalState NewState);
void SDIO_DMACmd(FunctionalState NewState);
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct);
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct);
uint8_t SDIO_GetCommandResponse(void);
uint32_t SDIO_GetResponse(uint32_t SDIO_RESP);
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct);
uint32_t SDIO_GetDataCounter(void);
uint32_t SDIO_ReadData(void);
void SDIO_WriteData(uint32_t Data);
uint32_t SDIO_GetFIFOCount(void);
void SDIO_StartSDIOReadWait(FunctionalState NewState);
void SDIO_StopSDIOReadWait(FunctionalState NewState);
void SDIO_SetSDIOReadWaitMode(uint32_t SDIO_ReadWaitMode);
void SDIO_SetSDIOOperation(FunctionalState NewState);
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState);
void SDIO_CommandCompletionCmd(FunctionalState NewState);
void SDIO_CEATAITCmd(FunctionalState NewState);
void SDIO_SendCEATACmd(FunctionalState NewState);
FlagStatus SDIO_GetFlagStatus(uint32_t SDIO_FLAG);
void SDIO_ClearFlag(uint32_t SDIO_FLAG);
ITStatus SDIO_GetITStatus(uint32_t SDIO_IT);
void SDIO_ClearITPendingBit(uint32_t SDIO_IT);




 



 



 

 
#line 43 ".\\usb_library\\inc\\stm32f10x_conf.h"
 
 
 
 
#line 1 ".\\FWlib\\inc\\misc.h"



















  

 



 
#line 28 ".\\FWlib\\inc\\misc.h"



 



 



 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;
  uint8_t NVIC_IRQChannelPreemptionPriority;
  uint8_t NVIC_IRQChannelSubPriority;
  FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;



 



 



 







 



 

#line 83 ".\\FWlib\\inc\\misc.h"


 



 

#line 101 ".\\FWlib\\inc\\misc.h"















 



 







 



 



 



 



 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);





 



 



 

 
#line 48 ".\\usb_library\\inc\\stm32f10x_conf.h"
 
 


 
 

 
 




 


 


 









 


 



 
 

 








 




 


 


 


 


 


 




 


 


 





 





 



 


 
#line 164 ".\\usb_library\\inc\\stm32f10x_conf.h"



 
#line 6974 "user\\stm32f10x.h"




 

















 





 

  

 

 
#line 36 "user\\sdcard.h"

 
typedef enum
{
   
  SD_CMD_CRC_FAIL                    = (1),  
  SD_DATA_CRC_FAIL                   = (2),  
  SD_CMD_RSP_TIMEOUT                 = (3),  
  SD_DATA_TIMEOUT                    = (4),  
  SD_TX_UNDERRUN                     = (5),  
  SD_RX_OVERRUN                      = (6),  
  SD_START_BIT_ERR                   = (7),  
  SD_CMD_OUT_OF_RANGE                = (8),  
  SD_ADDR_MISALIGNED                 = (9),  
  SD_BLOCK_LEN_ERR                   = (10),  
  SD_ERASE_SEQ_ERR                   = (11),  
  SD_BAD_ERASE_PARAM                 = (12),  
  SD_WRITE_PROT_VIOLATION            = (13),  
  SD_LOCK_UNLOCK_FAILED              = (14),  
  SD_COM_CRC_FAILED                  = (15),  
  SD_ILLEGAL_CMD                     = (16),  
  SD_CARD_ECC_FAILED                 = (17),  
  SD_CC_ERROR                        = (18),  
  SD_GENERAL_UNKNOWN_ERROR           = (19),  
  SD_STREAM_READ_UNDERRUN            = (20),  
  SD_STREAM_WRITE_OVERRUN            = (21),  
  SD_CID_CSD_OVERWRITE               = (22),  
  SD_WP_ERASE_SKIP                   = (23),  
  SD_CARD_ECC_DISABLED               = (24),  
  SD_ERASE_RESET                     = (25),  
  SD_AKE_SEQ_ERROR                   = (26),  
  SD_INVALID_VOLTRANGE               = (27),
  SD_ADDR_OUT_OF_RANGE               = (28),
  SD_SWITCH_ERROR                    = (29),
  SD_SDIO_DISABLED                   = (30),
  SD_SDIO_FUNCTION_BUSY              = (31),
  SD_SDIO_FUNCTION_FAILED            = (32),
  SD_SDIO_UNKNOWN_FUNCTION           = (33),

   
  SD_INTERNAL_ERROR, 
  SD_NOT_CONFIGURED,
  SD_REQUEST_PENDING, 
  SD_REQUEST_NOT_APPLICABLE, 
  SD_INVALID_PARAMETER,  
  SD_UNSUPPORTED_FEATURE,  
  SD_UNSUPPORTED_HW,  
  SD_ERROR,  
  SD_OK,  
} SD_Error;

 
#line 123 "user\\sdcard.h"




#line 134 "user\\sdcard.h"



 
#line 146 "user\\sdcard.h"


 
#line 160 "user\\sdcard.h"

typedef enum
{
  SD_NO_TRANSFER  = 0,
  SD_TRANSFER_IN_PROGRESS
} SDTransferState;

typedef struct
{
  uint16_t TransferredBytes;
  SD_Error TransferError;
  uint8_t  padding;
} SDLastTransferInfo;

typedef struct       
{
  volatile uint8_t  CSDStruct;             
  volatile uint8_t  SysSpecVersion;        
  volatile uint8_t  Reserved1;             
  volatile uint8_t  TAAC;                  
  volatile uint8_t  NSAC;                  
  volatile uint8_t  MaxBusClkFrec;         
  volatile uint16_t CardComdClasses;       
  volatile uint8_t  RdBlockLen;            
  volatile uint8_t  PartBlockRead;         
  volatile uint8_t  WrBlockMisalign;       
  volatile uint8_t  RdBlockMisalign;       
  volatile uint8_t  DSRImpl;               
  volatile uint8_t  Reserved2;             
  volatile uint32_t DeviceSize;            
  volatile uint8_t  MaxRdCurrentVDDMin;    
  volatile uint8_t  MaxRdCurrentVDDMax;    
  volatile uint8_t  MaxWrCurrentVDDMin;    
  volatile uint8_t  MaxWrCurrentVDDMax;    
  volatile uint8_t  DeviceSizeMul;         
  volatile uint8_t  EraseGrSize;           
  volatile uint8_t  EraseGrMul;            
  volatile uint8_t  WrProtectGrSize;       
  volatile uint8_t  WrProtectGrEnable;     
  volatile uint8_t  ManDeflECC;            
  volatile uint8_t  WrSpeedFact;           
  volatile uint8_t  MaxWrBlockLen;         
  volatile uint8_t  WriteBlockPaPartial;   
  volatile uint8_t  Reserved3;             
  volatile uint8_t  ContentProtectAppli;   
  volatile uint8_t  FileFormatGrouop;      
  volatile uint8_t  CopyFlag;              
  volatile uint8_t  PermWrProtect;         
  volatile uint8_t  TempWrProtect;         
  volatile uint8_t  FileFormat;            
  volatile uint8_t  ECC;                   
  volatile uint8_t  CSD_CRC;               
  volatile uint8_t  Reserved4;             
} SD_CSD;

typedef struct       
{
  volatile uint8_t  ManufacturerID;        
  volatile uint16_t OEM_AppliID;           
  volatile uint32_t ProdName1;             
  volatile uint8_t  ProdName2;             
  volatile uint8_t  ProdRev;               
  volatile uint32_t ProdSN;                
  volatile uint8_t  Reserved1;             
  volatile uint16_t ManufactDate;          
  volatile uint8_t  CID_CRC;               
  volatile uint8_t  Reserved2;             
} SD_CID;

typedef struct
{
  SD_CSD SD_csd;
  SD_CID SD_cid;
  uint32_t CardCapacity;  
  uint32_t CardBlockSize;  
  uint16_t RCA;
  uint8_t CardType;
} SD_CardInfo;

 




 
#line 253 "user\\sdcard.h"

 
 
SD_Error SD_Init(void);
SD_Error SD_PowerON(void);
SD_Error SD_PowerOFF(void);
SD_Error SD_InitializeCards(void);
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo);
SD_Error SD_EnableWideBusOperation(uint32_t WideMode);
SD_Error SD_SetDeviceMode(uint32_t Mode);
SD_Error SD_SelectDeselect(uint32_t addr);
SD_Error SD_ReadBlock(uint32_t addr, uint32_t *readbuff, uint16_t BlockSize);
SD_Error SD_ReadMultiBlocks(uint32_t addr, uint32_t *readbuff, uint16_t BlockSize, uint32_t NumberOfBlocks);
SD_Error SD_WriteBlock(uint32_t addr, uint32_t *writebuff, uint16_t BlockSize);
SD_Error SD_WriteMultiBlocks(uint32_t addr, uint32_t *writebuff, uint16_t BlockSize, uint32_t NumberOfBlocks);
SDTransferState SD_GetTransferState(void);
SD_Error SD_StopTransfer(void);
SD_Error SD_Erase(uint32_t startaddr, uint32_t endaddr);
SD_Error SD_SendStatus(uint32_t *pcardstatus);
SD_Error SD_SendSDStatus(uint32_t *psdstatus);
SD_Error SD_ProcessIRQSrc(void);





 



 
  
 
#line 19 "user\\sdcard.c"

 
 





 
#line 48 "user\\sdcard.c"

 












#line 73 "user\\sdcard.c"




 





 





 
 
static u32 CardType =  ((uint32_t)0x0);
static u32 CSD_Tab[4], CID_Tab[4], RCA = 0;
static u32 DeviceMode = ((uint32_t)0x00000000);
static u32 TotalNumberOfBytes = 0, StopCondition = 0;
u32 *SrcBuffer, *DestBuffer;
volatile SD_Error TransferError = SD_OK;
vu32 TransferEnd = 0;
vu32 NumberOfBytes = 0;
SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;

 
static SD_Error CmdError(void);
static SD_Error CmdResp1Error(u8 cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(u8 cmd, u16 *prca);
static SD_Error SDEnWideBus(FunctionalState NewState);
static SD_Error IsCardProgramming(u8 *pstatus);
static SD_Error FindSCR(u16 rca, u32 *pscr);
static u8 convert_from_bytes_to_power_of_two(u16 NumberOfBytes);
static void GPIO_Configuration(void);
static void DMA_TxConfiguration(u32 *BufferSRC, u32 BufferSize);
static void DMA_RxConfiguration(u32 *BufferDST, u32 BufferSize);

 








 
SD_Error SD_Init(void)
{
  SD_Error errorstatus = SD_OK;

  {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(((uint32_t)0x00000008), ENABLE);

  GPIO_InitStructure.GPIO_Pin =  ((uint16_t)0x0020);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00)), &GPIO_InitStructure);
  
  GPIO_ResetBits(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00)), ((uint16_t)0x0020)) ;
  }
  
   
  GPIO_Configuration();

   
  RCC_AHBPeriphClockCmd(((uint32_t)0x00000400), ENABLE);

   
  RCC_AHBPeriphClockCmd(((uint32_t)0x00000002), ENABLE);

  SDIO_DeInit();

  errorstatus = SD_PowerON();

  if (errorstatus != SD_OK)
  {
     
    return(errorstatus);
  }

  errorstatus = SD_InitializeCards();

  if (errorstatus != SD_OK)
  {
     
    return(errorstatus);
  }

   
     
  SDIO_InitStructure.SDIO_ClockDiv = ((u8)0x1); 
  SDIO_InitStructure.SDIO_ClockEdge = ((uint32_t)0x00000000);
  SDIO_InitStructure.SDIO_ClockBypass = ((uint32_t)0x00000000);
  SDIO_InitStructure.SDIO_ClockPowerSave = ((uint32_t)0x00000000);
  SDIO_InitStructure.SDIO_BusWide = ((uint32_t)0x00000000);
  SDIO_InitStructure.SDIO_HardwareFlowControl = ((uint32_t)0x00000000);
  SDIO_Init(&SDIO_InitStructure);

  return(errorstatus);
}








 
SD_Error SD_PowerON(void)
{
  SD_Error errorstatus = SD_OK;
  u32 response = 0, count = 0;
  bool validvoltage = FALSE;
  u32 SDType = ((u32)0x00000000);

   
   
  SDIO_InitStructure.SDIO_ClockDiv = ((u8)0xB2);  
  SDIO_InitStructure.SDIO_ClockEdge = ((uint32_t)0x00000000);
  SDIO_InitStructure.SDIO_ClockBypass = ((uint32_t)0x00000000);
  SDIO_InitStructure.SDIO_ClockPowerSave = ((uint32_t)0x00000000);
  SDIO_InitStructure.SDIO_BusWide = ((uint32_t)0x00000000);
  SDIO_InitStructure.SDIO_HardwareFlowControl = ((uint32_t)0x00000000);
  SDIO_Init(&SDIO_InitStructure);

   
  SDIO_SetPowerState(((uint32_t)0x00000003));

   
  SDIO_ClockCmd(ENABLE);

   
   
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)0);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdError();

  if (errorstatus != SD_OK)
  {
     
    return(errorstatus);
  }

   
   
  

 
   
  SDIO_CmdInitStructure.SDIO_Argument = ((u32)0x000001AA);
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((u32)0x00000008);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp7Error();

  if (errorstatus == SD_OK)
  {
    CardType = ((uint32_t)0x1);  
    SDType = ((u32)0x40000000);
  }
  else
  {
     
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)55);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(((uint8_t)55));
  }
   
  SDIO_CmdInitStructure.SDIO_Argument = 0x00;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)55);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(((uint8_t)55));

   
  
 
  if (errorstatus == SD_OK)
  {
     
     
    while ((!validvoltage) && (count < ((u32)0x0000FFFF)))
    {

       
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)55);
      SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
      SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
      SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(((uint8_t)55));

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      SDIO_CmdInitStructure.SDIO_Argument = ((u32)0x80100000) | SDType;
      SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)41);
      SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
      SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
      SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp3Error();
      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      response = SDIO_GetResponse(((uint32_t)0x00000000));
      validvoltage = (bool) (((response >> 31) == 1) ? 1 : 0);
      count++;
    }
    if (count >= ((u32)0x0000FFFF))
    {
      errorstatus = SD_INVALID_VOLTRANGE;
      return(errorstatus);
    }

    if (response &= ((u32)0x40000000))
    {
      CardType = ((uint32_t)0x2);
    }

  } 

  return(errorstatus);
}







 
SD_Error SD_PowerOFF(void)
{
  SD_Error errorstatus = SD_OK;

   
  SDIO_SetPowerState(((uint32_t)0x00000000));

  return(errorstatus);
}








 
SD_Error SD_InitializeCards(void)
{
  SD_Error errorstatus = SD_OK;
  u16 rca = 0x01;

  if (SDIO_GetPowerState() == ((uint32_t)0x00000000))
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }

  if (((uint32_t)0x4) != CardType)
  {
     
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)2);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x000000C0);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CID_Tab[0] = SDIO_GetResponse(((uint32_t)0x00000000));
    CID_Tab[1] = SDIO_GetResponse(((uint32_t)0x00000004));
    CID_Tab[2] = SDIO_GetResponse(((uint32_t)0x00000008));
    CID_Tab[3] = SDIO_GetResponse(((uint32_t)0x0000000C));
  }
  if ((((uint32_t)0x0) == CardType) ||  (((uint32_t)0x1) == CardType) ||  (((uint32_t)0x6) == CardType)
      ||  (((uint32_t)0x2) == CardType))
  {
     
     
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)3);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp6Error(((uint8_t)3), &rca);

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }

  if (((uint32_t)0x4) != CardType)
  {
    RCA = rca;

     
    SDIO_CmdInitStructure.SDIO_Argument = (u32)(rca << 16);
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)9);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x000000C0);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CSD_Tab[0] = SDIO_GetResponse(((uint32_t)0x00000000));
    CSD_Tab[1] = SDIO_GetResponse(((uint32_t)0x00000004));
    CSD_Tab[2] = SDIO_GetResponse(((uint32_t)0x00000008));
    CSD_Tab[3] = SDIO_GetResponse(((uint32_t)0x0000000C));
  }

  errorstatus = SD_OK;  

  return(errorstatus);
}








 
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
  SD_Error errorstatus = SD_OK;
  u8 tmp = 0;

  cardinfo->CardType = (u8)CardType;
  cardinfo->RCA = (u16)RCA;

   
  tmp = (u8)((CSD_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
  cardinfo->SD_csd.Reserved1 = tmp & 0x03;

   
  tmp = (u8)((CSD_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.TAAC = tmp;

   
  tmp = (u8)((CSD_Tab[0] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.NSAC = tmp;

   
  tmp = (u8)(CSD_Tab[0] & 0x000000FF);
  cardinfo->SD_csd.MaxBusClkFrec = tmp;

   
  tmp = (u8)((CSD_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CardComdClasses = tmp << 4;

   
  tmp = (u8)((CSD_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
  cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

   
  tmp = (u8)((CSD_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.Reserved2 = 0;  

  if ((CardType == ((uint32_t)0x0)) || (CardType == ((uint32_t)0x1)))
  {
    cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

     
    tmp = (u8)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

     
    tmp = (u8)((CSD_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

    cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
    cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

     
    tmp = (u8)((CSD_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
    cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
     
    tmp = (u8)((CSD_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;
    
    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
    cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
    cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
    cardinfo->CardCapacity *= cardinfo->CardBlockSize;
  }
  else if (CardType == ((uint32_t)0x2))
  {
     
    tmp = (u8)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

     
    tmp = (u8)((CSD_Tab[2] & 0xFF000000) >> 24);

    cardinfo->SD_csd.DeviceSize |= (tmp << 8);

     
    tmp = (u8)((CSD_Tab[2] & 0x00FF0000) >> 16);

    cardinfo->SD_csd.DeviceSize |= (tmp);

     
    tmp = (u8)((CSD_Tab[2] & 0x0000FF00) >> 8);
    
    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;
    cardinfo->CardBlockSize = 512;    
  }


  cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;

   
  tmp = (u8)(CSD_Tab[2] & 0x000000FF);
  cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

   
  tmp = (u8)((CSD_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
  cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
  cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;

   
  tmp = (u8)((CSD_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.Reserved3 = 0;
  cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

   
  tmp = (u8)((CSD_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
  cardinfo->SD_csd.ECC = (tmp & 0x03);

   
  tmp = (u8)(CSD_Tab[3] & 0x000000FF);
  cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_csd.Reserved4 = 1;


   
  tmp = (u8)((CID_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ManufacturerID = tmp;

   
  tmp = (u8)((CID_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.OEM_AppliID = tmp << 8;

   
  tmp = (u8)((CID_Tab[0] & 0x000000FF00) >> 8);
  cardinfo->SD_cid.OEM_AppliID |= tmp;

   
  tmp = (u8)(CID_Tab[0] & 0x000000FF);
  cardinfo->SD_cid.ProdName1 = tmp << 24;

   
  tmp = (u8)((CID_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdName1 |= tmp << 16;

   
  tmp = (u8)((CID_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdName1 |= tmp << 8;

   
  tmp = (u8)((CID_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdName1 |= tmp;

   
  tmp = (u8)(CID_Tab[1] & 0x000000FF);
  cardinfo->SD_cid.ProdName2 = tmp;

   
  tmp = (u8)((CID_Tab[2] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdRev = tmp;

   
  tmp = (u8)((CID_Tab[2] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdSN = tmp << 24;

   
  tmp = (u8)((CID_Tab[2] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdSN |= tmp << 16;

   
  tmp = (u8)(CID_Tab[2] & 0x000000FF);
  cardinfo->SD_cid.ProdSN |= tmp << 8;

   
  tmp = (u8)((CID_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdSN |= tmp;

   
  tmp = (u8)((CID_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
  cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

   
  tmp = (u8)((CID_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ManufactDate |= tmp;

   
  tmp = (u8)(CID_Tab[3] & 0x000000FF);
  cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_cid.Reserved2 = 1;
  
  return(errorstatus);
}












 
SD_Error SD_EnableWideBusOperation(u32 WideMode)
{
  SD_Error errorstatus = SD_OK;

   
  if (((uint32_t)0x3) == CardType)
  {
    errorstatus = SD_UNSUPPORTED_FEATURE;
    return(errorstatus);
  }
  else if ((((uint32_t)0x0) == CardType) || (((uint32_t)0x1) == CardType) || (((uint32_t)0x2) == CardType))
  {
    if (((uint32_t)0x00001000) == WideMode)
    {
      errorstatus = SD_UNSUPPORTED_FEATURE;
      return(errorstatus);
    }
    else if (((uint32_t)0x00000800) == WideMode)
    {
      errorstatus = SDEnWideBus(ENABLE);

      if (SD_OK == errorstatus)
      {
         
        SDIO_InitStructure.SDIO_ClockDiv = ((u8)0x1); 
        SDIO_InitStructure.SDIO_ClockEdge = ((uint32_t)0x00000000);
        SDIO_InitStructure.SDIO_ClockBypass = ((uint32_t)0x00000000);
        SDIO_InitStructure.SDIO_ClockPowerSave = ((uint32_t)0x00000000);
        SDIO_InitStructure.SDIO_BusWide = ((uint32_t)0x00000800);
        SDIO_InitStructure.SDIO_HardwareFlowControl = ((uint32_t)0x00000000);
        SDIO_Init(&SDIO_InitStructure);
      }
    }
    else
    {
      errorstatus = SDEnWideBus(DISABLE);

      if (SD_OK == errorstatus)
      {
         
        SDIO_InitStructure.SDIO_ClockDiv = ((u8)0x1); 
        SDIO_InitStructure.SDIO_ClockEdge = ((uint32_t)0x00000000);
        SDIO_InitStructure.SDIO_ClockBypass = ((uint32_t)0x00000000);
        SDIO_InitStructure.SDIO_ClockPowerSave = ((uint32_t)0x00000000);
        SDIO_InitStructure.SDIO_BusWide = ((uint32_t)0x00000000);
        SDIO_InitStructure.SDIO_HardwareFlowControl = ((uint32_t)0x00000000);
        SDIO_Init(&SDIO_InitStructure);
      }
    }
  }

  return(errorstatus);
}












 
SD_Error SD_SetDeviceMode(u32 Mode)
{
  SD_Error errorstatus = SD_OK;

  if ((Mode == ((uint32_t)0x00000000)) || (Mode == ((uint32_t)0x00000001)) || (Mode == ((uint32_t)0x00000002)))
  {
    DeviceMode = Mode;
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
  }
  return(errorstatus);

}







 
SD_Error SD_SelectDeselect(u32 addr)
{
  SD_Error errorstatus = SD_OK;

   
  SDIO_CmdInitStructure.SDIO_Argument =  addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)7);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)7));

  return(errorstatus);
}










 
SD_Error SD_ReadBlock(u32 addr, u32 *readbuff, u16 BlockSize)
{
  SD_Error errorstatus = SD_OK;
  u32 count = 0, *tempbuff = readbuff;
  u8 power = 0;

  if (0 == readbuff)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;

   
  SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000000);
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE);

  if (SDIO_GetResponse(((uint32_t)0x00000000)) & ((u32)0x02000000))
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }
  
  if (CardType == ((uint32_t)0x2))
  {
    BlockSize = 512;
    addr /= 512;
  }
  if ((BlockSize > 0) && (BlockSize <= 2048) && ((BlockSize & (BlockSize - 1)) == 0))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);

     
    SDIO_CmdInitStructure.SDIO_Argument = (u32) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)16);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)16));

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
  SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
  SDIO_DataInitStructure.SDIO_DataBlockSize = (u32) power << 4;
  SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000002);
  SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000001);
  SDIO_DataConfig(&SDIO_DataInitStructure);

  TotalNumberOfBytes = BlockSize;
  StopCondition = 0;
  DestBuffer = readbuff;

   
  SDIO_CmdInitStructure.SDIO_Argument = (u32)addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)17);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)17));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
   
  if (DeviceMode == ((uint32_t)0x00000002))
  {
     
    while (!(((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA &(((uint32_t)0x00000020) | ((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000400) | ((uint32_t)0x00000200))))
    {
      if (SDIO_GetFlagStatus(((uint32_t)0x00008000)) != RESET)
      {
        for (count = 0; count < 8; count++)
        {
          *(tempbuff + count) = SDIO_ReadData();
        }
        tempbuff += 8;
      }
    }

    if (SDIO_GetFlagStatus(((uint32_t)0x00000008)) != RESET)
    {
      SDIO_ClearFlag(((uint32_t)0x00000008));
      errorstatus = SD_DATA_TIMEOUT;
      return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(((uint32_t)0x00000002)) != RESET)
    {
      SDIO_ClearFlag(((uint32_t)0x00000002));
      errorstatus = SD_DATA_CRC_FAIL;
      return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(((uint32_t)0x00000020)) != RESET)
    {
      SDIO_ClearFlag(((uint32_t)0x00000020));
      errorstatus = SD_RX_OVERRUN;
      return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(((uint32_t)0x00000200)) != RESET)
    {
      SDIO_ClearFlag(((uint32_t)0x00000200));
      errorstatus = SD_START_BIT_ERR;
      return(errorstatus);
    }
    while (SDIO_GetFlagStatus(((uint32_t)0x00200000)) != RESET)
    {
      *tempbuff = SDIO_ReadData();
      tempbuff++;
    }

     
    SDIO_ClearFlag(((u32)0x000005FF));
  }
  else if (DeviceMode == ((uint32_t)0x00000001))
  {
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) | ((uint32_t)0x00000020) | ((uint32_t)0x00008000) | ((uint32_t)0x00000200), ENABLE);
    while ((TransferEnd == 0) && (TransferError == SD_OK))
    {}
    if (TransferError != SD_OK)
    {
      return(TransferError);
    }
  }
  else if (DeviceMode == ((uint32_t)0x00000000))
  {
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) | ((uint32_t)0x00000020) | ((uint32_t)0x00000200), ENABLE);
    SDIO_DMACmd(ENABLE);
    DMA_RxConfiguration(readbuff, BlockSize);
    while (DMA_GetFlagStatus(((uint32_t)0x10002000)) == RESET)
    {}
  }
  return(errorstatus);
}











 
SD_Error SD_ReadMultiBlocks(u32 addr, u32 *readbuff, u16 BlockSize, u32 NumberOfBlocks)
{
  SD_Error errorstatus = SD_OK;
  u32 count = 0, *tempbuff = readbuff;
  u8 power = 0;

  if (0 == readbuff)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;

   
  SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000000);
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE);

  if (SDIO_GetResponse(((uint32_t)0x00000000)) & ((u32)0x02000000))
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  if (CardType == ((uint32_t)0x2))
  {
    BlockSize = 512;
    addr /= 512;
  }
  
  if ((BlockSize > 0) && (BlockSize <= 2048) && (0 == (BlockSize & (BlockSize - 1))))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);

     
    SDIO_CmdInitStructure.SDIO_Argument = (u32) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)16);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)16));

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  if (NumberOfBlocks > 1)
  {
     
    if (NumberOfBlocks * BlockSize > ((u32)0x01FFFFFF))
    {
      errorstatus = SD_INVALID_PARAMETER;
      return(errorstatus);
    }

    TotalNumberOfBytes = NumberOfBlocks * BlockSize;
    StopCondition = 1;
    DestBuffer = readbuff;

    SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
    SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
    SDIO_DataInitStructure.SDIO_DataBlockSize = (u32) power << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000002);
    SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
    SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000001);
    SDIO_DataConfig(&SDIO_DataInitStructure);

     
    SDIO_CmdInitStructure.SDIO_Argument = (u32)addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)18);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)18));

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }

    if (DeviceMode == ((uint32_t)0x00000002))
    {
       
      while (!(((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA &(((uint32_t)0x00000020) | ((uint32_t)0x00000002) | ((uint32_t)0x00000100) | ((uint32_t)0x00000008) | ((uint32_t)0x00000200))))
      {
        if (SDIO_GetFlagStatus(((uint32_t)0x00008000)) != RESET)
        {
          for (count = 0; count < ((u32)0x00000008); count++)
          {
            *(tempbuff + count) = SDIO_ReadData();
          }
          tempbuff += ((u32)0x00000008);
        }
      }

      if (SDIO_GetFlagStatus(((uint32_t)0x00000008)) != RESET)
      {
        SDIO_ClearFlag(((uint32_t)0x00000008));
        errorstatus = SD_DATA_TIMEOUT;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(((uint32_t)0x00000002)) != RESET)
      {
        SDIO_ClearFlag(((uint32_t)0x00000002));
        errorstatus = SD_DATA_CRC_FAIL;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(((uint32_t)0x00000020)) != RESET)
      {
        SDIO_ClearFlag(((uint32_t)0x00000020));
        errorstatus = SD_RX_OVERRUN;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(((uint32_t)0x00000200)) != RESET)
      {
        SDIO_ClearFlag(((uint32_t)0x00000200));
        errorstatus = SD_START_BIT_ERR;
        return(errorstatus);
      }
      while (SDIO_GetFlagStatus(((uint32_t)0x00200000)) != RESET)
      {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
      }

      if (SDIO_GetFlagStatus(((uint32_t)0x00000100)) != RESET)
      {
         
        if ((((uint32_t)0x0) == CardType) || (((uint32_t)0x2) == CardType) || (((uint32_t)0x1) == CardType))
        {
           
          SDIO_CmdInitStructure.SDIO_Argument = 0x0;
          SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)12);
          SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
          SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
          SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
          SDIO_SendCommand(&SDIO_CmdInitStructure);

          errorstatus = CmdResp1Error(((uint8_t)12));

          if (errorstatus != SD_OK)
          {
            return(errorstatus);
          }
        }
      }
       
      SDIO_ClearFlag(((u32)0x000005FF));
    }
    else if (DeviceMode == ((uint32_t)0x00000001))
    {
      SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) | ((uint32_t)0x00000020) | ((uint32_t)0x00008000) | ((uint32_t)0x00000200), ENABLE);
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
    else if (DeviceMode == ((uint32_t)0x00000000))
    {
      SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) | ((uint32_t)0x00000020) | ((uint32_t)0x00000200), ENABLE);
      SDIO_DMACmd(ENABLE);
      DMA_RxConfiguration(readbuff, (NumberOfBlocks * BlockSize));
      while (DMA_GetFlagStatus(((uint32_t)0x10002000)) == RESET)
      {}
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
  }
  return(errorstatus);
}











 
SD_Error SD_WriteBlock(u32 addr, u32 *writebuff, u16 BlockSize)
{
  SD_Error errorstatus = SD_OK;
  u8  power = 0, cardstate = 0;
  u32 timeout = 0, bytestransferred = 0;
  u32 cardstatus = 0, count = 0, restwords = 0;
  u32 *tempbuff = writebuff;

  if (writebuff == 0)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;

  SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000000);
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE);

  if (SDIO_GetResponse(((uint32_t)0x00000000)) & ((u32)0x02000000))
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  if (CardType == ((uint32_t)0x2))
  {
    BlockSize = 512;
    addr /= 512;
  }
  
   
  if ((BlockSize > 0) && (BlockSize <= 2048) && ((BlockSize & (BlockSize - 1)) == 0))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);

    SDIO_CmdInitStructure.SDIO_Argument = (u32) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)16);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)16));

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

   
  SDIO_CmdInitStructure.SDIO_Argument = (u32) (RCA << 16);
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)13);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)13));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  cardstatus = SDIO_GetResponse(((uint32_t)0x00000000));

  timeout = ((u32)0x000FFFFF);

  while (((cardstatus & 0x00000100) == 0) && (timeout > 0))
  {
    timeout--;
    SDIO_CmdInitStructure.SDIO_Argument = (u32) (RCA << 16);
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)13);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)13));

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
    cardstatus = SDIO_GetResponse(((uint32_t)0x00000000));
  }

  if (timeout == 0)
  {
    return(SD_ERROR);
  }

   
  SDIO_CmdInitStructure.SDIO_Argument = addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)24);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)24));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  TotalNumberOfBytes = BlockSize;
  StopCondition = 0;
  SrcBuffer = writebuff;

  SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
  SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
  SDIO_DataInitStructure.SDIO_DataBlockSize = (u32) power << 4;
  SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000001);
  SDIO_DataConfig(&SDIO_DataInitStructure);

   
  if (DeviceMode == ((uint32_t)0x00000002))
  {
    while (!(((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA & (((uint32_t)0x00000400) | ((uint32_t)0x00000010) | ((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000200))))
    {
      if (SDIO_GetFlagStatus(((uint32_t)0x00004000)) != RESET)
      {
        if ((TotalNumberOfBytes - bytestransferred) < 32)
        {
          restwords = ((TotalNumberOfBytes - bytestransferred) % 4 == 0) ? ((TotalNumberOfBytes - bytestransferred) / 4) : (( TotalNumberOfBytes -  bytestransferred) / 4 + 1);

          for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
          {
            SDIO_WriteData(*tempbuff);
          }
        }
        else
        {
          for (count = 0; count < 8; count++)
          {
            SDIO_WriteData(*(tempbuff + count));
          }
          tempbuff += 8;
          bytestransferred += 32;
        }
      }
    }
    if (SDIO_GetFlagStatus(((uint32_t)0x00000008)) != RESET)
    {
      SDIO_ClearFlag(((uint32_t)0x00000008));
      errorstatus = SD_DATA_TIMEOUT;
      return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(((uint32_t)0x00000002)) != RESET)
    {
      SDIO_ClearFlag(((uint32_t)0x00000002));
      errorstatus = SD_DATA_CRC_FAIL;
      return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(((uint32_t)0x00000010)) != RESET)
    {
      SDIO_ClearFlag(((uint32_t)0x00000010));
      errorstatus = SD_TX_UNDERRUN;
      return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(((uint32_t)0x00000200)) != RESET)
    {
      SDIO_ClearFlag(((uint32_t)0x00000200));
      errorstatus = SD_START_BIT_ERR;
      return(errorstatus);
    }
  }
  else if (DeviceMode == ((uint32_t)0x00000001))
  {
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) | ((uint32_t)0x00004000) | ((uint32_t)0x00000010) | ((uint32_t)0x00000200), ENABLE);
    while ((TransferEnd == 0) && (TransferError == SD_OK))
    {}
    if (TransferError != SD_OK)
    {
      return(TransferError);
    }
  }
  else if (DeviceMode == ((uint32_t)0x00000000))
  {
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) | ((uint32_t)0x00000010) | ((uint32_t)0x00000200), ENABLE);
    DMA_TxConfiguration(writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);
    while (DMA_GetFlagStatus(((uint32_t)0x10002000)) == RESET)
    {}
    while ((TransferEnd == 0) && (TransferError == SD_OK))
    {}
    if (TransferError != SD_OK)
    {
      return(TransferError);
    }
  }

   
  SDIO_ClearFlag(((u32)0x000005FF));

   
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((cardstate == ((u32)0x00000007)) || (cardstate == ((u32)0x00000006))))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}












 
SD_Error SD_WriteMultiBlocks(u32 addr, u32 *writebuff, u16 BlockSize, u32 NumberOfBlocks)
{
  SD_Error errorstatus = SD_OK;
  u8  power = 0, cardstate = 0;
  u32 bytestransferred = 0;
  u32 count = 0, restwords = 0;
  u32 *tempbuff = writebuff;

  if (writebuff == 0)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;

  SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000000);
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE);

  if (SDIO_GetResponse(((uint32_t)0x00000000)) & ((u32)0x02000000))
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  if (CardType == ((uint32_t)0x2))
  {
    BlockSize = 512;
    addr /= 512;
  }
  
   
  if ((BlockSize > 0) && (BlockSize <= 2048) && ((BlockSize & (BlockSize - 1)) == 0))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);

    SDIO_CmdInitStructure.SDIO_Argument = (u32) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)16);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)16));

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

   
  SDIO_CmdInitStructure.SDIO_Argument = (u32) (RCA << 16);
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)13);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)13));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  if (NumberOfBlocks > 1)
  {
     
    if (NumberOfBlocks * BlockSize > ((u32)0x01FFFFFF))
    {
      errorstatus = SD_INVALID_PARAMETER;
      return(errorstatus);
    }

    if ((((uint32_t)0x0) == CardType) || (((uint32_t)0x1) == CardType) || (((uint32_t)0x2) == CardType))
    {
       
      SDIO_CmdInitStructure.SDIO_Argument = (u32) (RCA << 16);
      SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)55);
      SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
      SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
      SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(((uint8_t)55));

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
       
      SDIO_CmdInitStructure.SDIO_Argument = (u32)NumberOfBlocks;
      SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)23);
      SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
      SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
      SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(((uint8_t)23));

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
    }

     
    SDIO_CmdInitStructure.SDIO_Argument = (u32)addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)25);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)25));

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    TotalNumberOfBytes = NumberOfBlocks * BlockSize;
    StopCondition = 1;
    SrcBuffer = writebuff;

    SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
    SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
    SDIO_DataInitStructure.SDIO_DataBlockSize = (u32) power << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000000);
    SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
    SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000001);
    SDIO_DataConfig(&SDIO_DataInitStructure);

    if (DeviceMode == ((uint32_t)0x00000002))
    {
      while (!(((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA & (((uint32_t)0x00000010) | ((uint32_t)0x00000002) | ((uint32_t)0x00000100) | ((uint32_t)0x00000008) | ((uint32_t)0x00000200))))
      {
        if (SDIO_GetFlagStatus(((uint32_t)0x00004000)) != RESET)
        {
          if (!((TotalNumberOfBytes - bytestransferred) < ((u32)0x00000020)))
          {
            for (count = 0; count < ((u32)0x00000008); count++)
            {
              SDIO_WriteData(*(tempbuff + count));
            }
            tempbuff += ((u32)0x00000008);
            bytestransferred += ((u32)0x00000020);
          }
          else
          {
            restwords = ((TotalNumberOfBytes - bytestransferred) % 4 == 0) ? ((TotalNumberOfBytes - bytestransferred) / 4) :
                        ((TotalNumberOfBytes - bytestransferred) / 4 + 1);

            for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
            {
              SDIO_WriteData(*tempbuff);
            }
          }
        }
      }

      if (SDIO_GetFlagStatus(((uint32_t)0x00000008)) != RESET)
      {
        SDIO_ClearFlag(((uint32_t)0x00000008));
        errorstatus = SD_DATA_TIMEOUT;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(((uint32_t)0x00000002)) != RESET)
      {
        SDIO_ClearFlag(((uint32_t)0x00000002));
        errorstatus = SD_DATA_CRC_FAIL;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(((uint32_t)0x00000010)) != RESET)
      {
        SDIO_ClearFlag(((uint32_t)0x00000010));
        errorstatus = SD_TX_UNDERRUN;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(((uint32_t)0x00000200)) != RESET)
      {
        SDIO_ClearFlag(((uint32_t)0x00000200));
        errorstatus = SD_START_BIT_ERR;
        return(errorstatus);
      }

      if (SDIO_GetFlagStatus(((uint32_t)0x00000100)) != RESET)
      {
       if ((((uint32_t)0x0) == CardType) || (((uint32_t)0x1) == CardType) || (((uint32_t)0x2) == CardType))
        {
           
          SDIO_CmdInitStructure.SDIO_Argument = 0x0;
          SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)12);
          SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
          SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
          SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
          SDIO_SendCommand(&SDIO_CmdInitStructure);


          errorstatus = CmdResp1Error(((uint8_t)12));

          if (errorstatus != SD_OK)
          {
            return(errorstatus);
          }
        }
      }
    }
    else if (DeviceMode == ((uint32_t)0x00000001))
    {
      SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) | ((uint32_t)0x00004000) | ((uint32_t)0x00000010) | ((uint32_t)0x00000200), ENABLE);
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
    else if (DeviceMode == ((uint32_t)0x00000000))
    {
      SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) | ((uint32_t)0x00000010) | ((uint32_t)0x00000200), ENABLE);
      SDIO_DMACmd(ENABLE);
      DMA_TxConfiguration(writebuff, (NumberOfBlocks * BlockSize));
      while (DMA_GetFlagStatus(((uint32_t)0x10002000)) == RESET)
      {}
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
  }
   
  SDIO_ClearFlag(((u32)0x000005FF));

   
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((cardstate == ((u32)0x00000007)) || (cardstate == ((u32)0x00000006))))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}










 
SDTransferState SD_GetTransferState(void)
{
  if (((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA & (((uint32_t)0x00001000) | ((uint32_t)0x00002000)))
  {
    return(SD_TRANSFER_IN_PROGRESS);
  }
  else
  {
    return(SD_NO_TRANSFER);
  }
}







 
SD_Error SD_StopTransfer(void)
{
  SD_Error errorstatus = SD_OK;

   
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)12);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)12));

  return(errorstatus);
}








 
SD_Error SD_Erase(u32 startaddr, u32 endaddr)
{
  SD_Error errorstatus = SD_OK;
  u32 delay = 0;
  vu32 maxdelay = 0;
  u8 cardstate = 0;

   
  if (((CSD_Tab[1] >> 20) & ((u32)0x00000020)) == 0)
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }

  maxdelay = 72000 / ((((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->CLKCR & 0xFF) + 2);

  if (SDIO_GetResponse(((uint32_t)0x00000000)) & ((u32)0x02000000))
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  if (CardType == ((uint32_t)0x2))
  {
    startaddr /= 512;
    endaddr /= 512;
  }
  
   
  if ((((uint32_t)0x0) == CardType) || (((uint32_t)0x1) == CardType) || (((uint32_t)0x2) == CardType))
  {
     
    SDIO_CmdInitStructure.SDIO_Argument = startaddr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)32);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)32));
    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }

     
    SDIO_CmdInitStructure.SDIO_Argument = endaddr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)33);
    SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
    SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
    SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(((uint8_t)33));
    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }

   
  SDIO_CmdInitStructure.SDIO_Argument = 0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)38);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)38));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  for (delay = 0; delay < maxdelay; delay++)
  {}

   
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((((u32)0x00000007) == cardstate) || (((u32)0x00000006) == cardstate)))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}








 
SD_Error SD_SendStatus(u32 *pcardstatus)
{
  SD_Error errorstatus = SD_OK;

  if (pcardstatus == 0)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  SDIO_CmdInitStructure.SDIO_Argument = (u32) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)13);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);


  errorstatus = CmdResp1Error(((uint8_t)13));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  *pcardstatus = SDIO_GetResponse(((uint32_t)0x00000000));

  return(errorstatus);
}








 
SD_Error SD_SendSDStatus(u32 *psdstatus)
{
  SD_Error errorstatus = SD_OK;
  u32 count = 0;

  if (SDIO_GetResponse(((uint32_t)0x00000000)) & ((u32)0x02000000))
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

   
  SDIO_CmdInitStructure.SDIO_Argument = 64;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)16);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)16));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

   
  SDIO_CmdInitStructure.SDIO_Argument = (u32) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)55);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(((uint8_t)55));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
  SDIO_DataInitStructure.SDIO_DataLength = 64;
  SDIO_DataInitStructure.SDIO_DataBlockSize = ((uint32_t)0x00000060);
  SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000002);
  SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000001);
  SDIO_DataConfig(&SDIO_DataInitStructure);


   
  SDIO_CmdInitStructure.SDIO_Argument = 0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)13);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(((uint8_t)13));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  while (!(((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA &(((uint32_t)0x00000020) | ((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000400) | ((uint32_t)0x00000200))))
  {
    if (SDIO_GetFlagStatus(((uint32_t)0x00008000)) != RESET)
    {
      for (count = 0; count < 8; count++)
      {
        *(psdstatus + count) = SDIO_ReadData();
      }
      psdstatus += 8;
    }
  }

  if (SDIO_GetFlagStatus(((uint32_t)0x00000008)) != RESET)
  {
    SDIO_ClearFlag(((uint32_t)0x00000008));
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(((uint32_t)0x00000002)) != RESET)
  {
    SDIO_ClearFlag(((uint32_t)0x00000002));
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(((uint32_t)0x00000020)) != RESET)
  {
    SDIO_ClearFlag(((uint32_t)0x00000020));
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(((uint32_t)0x00000200)) != RESET)
  {
    SDIO_ClearFlag(((uint32_t)0x00000200));
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

  while (SDIO_GetFlagStatus(((uint32_t)0x00200000)) != RESET)
  {
    *psdstatus = SDIO_ReadData();
    psdstatus++;
  }

   
  SDIO_ClearFlag(((u32)0x000005FF));
  psdstatus -= 16;
  for (count = 0; count < 16; count++)
  {
    psdstatus[count] = ((psdstatus[count] & ((u32)0x000000FF)) << 24) |((psdstatus[count] & ((u32)0x0000FF00)) << 8) |
                       ((psdstatus[count] & ((u32)0x00FF0000)) >> 8) |((psdstatus[count] & ((u32)0xFF000000)) >> 24);
  }
  return(errorstatus);
}







 
SD_Error SD_ProcessIRQSrc(void)
{
  u32 count = 0, restwords = 0;

  if (DeviceMode == ((uint32_t)0x00000001))
  {
    if (SDIO_GetITStatus(((uint32_t)0x00008000)) != RESET)
    {
      for (count = 0; count < ((u32)0x00000008); count++)
      {
        *(DestBuffer + count) = SDIO_ReadData();
      }
      DestBuffer += ((u32)0x00000008);
      NumberOfBytes += ((u32)0x00000020);
    }
    else if (SDIO_GetITStatus(((uint32_t)0x00004000)) != RESET)
    {
      if ((TotalNumberOfBytes - NumberOfBytes) < ((u32)0x00000020))
      {
        restwords = ((TotalNumberOfBytes - NumberOfBytes) %  4 == 0) ?
                    ((TotalNumberOfBytes - NumberOfBytes) / 4) :
                    ((TotalNumberOfBytes - NumberOfBytes) / 4 + 1);

        for (count = 0; count < restwords;  count++, SrcBuffer++, NumberOfBytes += 4)
        {
          SDIO_WriteData(*SrcBuffer);
        }
      }
      else
      {
        for (count = 0; count < ((u32)0x00000008); count++)
        {
          SDIO_WriteData(*(SrcBuffer + count));
        }

        SrcBuffer += ((u32)0x00000008);
        NumberOfBytes += ((u32)0x00000020);
      }
    }
  }

  if (SDIO_GetITStatus(((uint32_t)0x00000100)) != RESET)
  {
    if (DeviceMode != ((uint32_t)0x00000000))
    {
      while ((SDIO_GetFlagStatus(((uint32_t)0x00200000)) != RESET)  &&  (NumberOfBytes < TotalNumberOfBytes))
      {
        *DestBuffer = SDIO_ReadData();
        DestBuffer++;
        NumberOfBytes += 4;
      }
    }

    if (StopCondition == 1)
    {
      TransferError = SD_StopTransfer();
    }
    else
    {
      TransferError = SD_OK;
    }
    SDIO_ClearITPendingBit(((uint32_t)0x00000100));
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) |
                  ((uint32_t)0x00004000) | ((uint32_t)0x00008000) | ((uint32_t)0x00000010) |
                  ((uint32_t)0x00000020) | ((uint32_t)0x00000200), DISABLE);
    TransferEnd = 1;
    NumberOfBytes = 0;
    return(TransferError);
  }

  if (SDIO_GetITStatus(((uint32_t)0x00000002)) != RESET)
  {
    SDIO_ClearITPendingBit(((uint32_t)0x00000002));
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) |
                  ((uint32_t)0x00004000) | ((uint32_t)0x00008000) | ((uint32_t)0x00000010) |
                  ((uint32_t)0x00000020) | ((uint32_t)0x00000200), DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_DATA_CRC_FAIL;
    return(SD_DATA_CRC_FAIL);
  }

  if (SDIO_GetITStatus(((uint32_t)0x00000008)) != RESET)
  {
    SDIO_ClearITPendingBit(((uint32_t)0x00000008));
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) |
                  ((uint32_t)0x00004000) | ((uint32_t)0x00008000) | ((uint32_t)0x00000010) |
                  ((uint32_t)0x00000020) | ((uint32_t)0x00000200), DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_DATA_TIMEOUT;
    return(SD_DATA_TIMEOUT);
  }

  if (SDIO_GetITStatus(((uint32_t)0x00000020)) != RESET)
  {
    SDIO_ClearITPendingBit(((uint32_t)0x00000020));
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) |
                  ((uint32_t)0x00004000) | ((uint32_t)0x00008000) | ((uint32_t)0x00000010) |
                  ((uint32_t)0x00000020) | ((uint32_t)0x00000200), DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_RX_OVERRUN;
    return(SD_RX_OVERRUN);
  }

  if (SDIO_GetITStatus(((uint32_t)0x00000010)) != RESET)
  {
    SDIO_ClearITPendingBit(((uint32_t)0x00000010));
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) |
                  ((uint32_t)0x00004000) | ((uint32_t)0x00008000) | ((uint32_t)0x00000010) |
                  ((uint32_t)0x00000020) | ((uint32_t)0x00000200), DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_TX_UNDERRUN;
    return(SD_TX_UNDERRUN);
  }

  if (SDIO_GetITStatus(((uint32_t)0x00000200)) != RESET)
  {
    SDIO_ClearITPendingBit(((uint32_t)0x00000200));
    SDIO_ITConfig(((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000100) |
                  ((uint32_t)0x00004000) | ((uint32_t)0x00008000) | ((uint32_t)0x00000010) |
                  ((uint32_t)0x00000020) | ((uint32_t)0x00000200), DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_START_BIT_ERR;
    return(SD_START_BIT_ERR);
  }

  return(SD_OK);
}







 
static SD_Error CmdError(void)
{
  SD_Error errorstatus = SD_OK;
  u32 timeout;

  timeout = ((u32)0x00002710);  

  while ((timeout > 0) && (SDIO_GetFlagStatus(((uint32_t)0x00000080)) == RESET))
  {
    timeout--;
  }

  if (timeout == 0)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    return(errorstatus);
  }

   
  SDIO_ClearFlag(((u32)0x000005FF));

  return(errorstatus);
}








 
static SD_Error CmdResp7Error(void)
{
  SD_Error errorstatus = SD_OK;
  u32 status;
  u32 timeout = ((u32)0x00002710);

  status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;

  while (!(status & (((uint32_t)0x00000001) | ((uint32_t)0x00000040) | ((uint32_t)0x00000004))) && (timeout > 0))
  {
    timeout--;
    status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;
  }

  if ((timeout == 0) || (status & ((uint32_t)0x00000004)))
  {
     
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(((uint32_t)0x00000004));
    return(errorstatus);
  }

  if (status & ((uint32_t)0x00000040))
  {
     
    errorstatus = SD_OK;
    SDIO_ClearFlag(((uint32_t)0x00000040));
    return(errorstatus);
  }
  return(errorstatus);
}








 
static SD_Error CmdResp1Error(u8 cmd)
{
  SD_Error errorstatus = SD_OK;
  u32 status;
  u32 response_r1;

  status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;

  while (!(status & (((uint32_t)0x00000001) | ((uint32_t)0x00000040) | ((uint32_t)0x00000004))))
  {
    status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;
  }

  if (status & ((uint32_t)0x00000004))
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(((uint32_t)0x00000004));
    return(errorstatus);
  }
  else if (status & ((uint32_t)0x00000001))
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(((uint32_t)0x00000001));
    return(errorstatus);
  }

   
  if (SDIO_GetCommandResponse() != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

   
  SDIO_ClearFlag(((u32)0x000005FF));

   
  response_r1 = SDIO_GetResponse(((uint32_t)0x00000000));

  if ((response_r1 & ((u32)0xFDFFE008)) == ((u32)0x00000000))
  {
    return(errorstatus);
  }

  if (response_r1 & ((u32)0x80000000))
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (response_r1 & ((u32)0x40000000))
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (response_r1 & ((u32)0x20000000))
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (response_r1 & ((u32)0x10000000))
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (response_r1 & ((u32)0x08000000))
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (response_r1 & ((u32)0x04000000))
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (response_r1 & ((u32)0x01000000))
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (response_r1 & ((u32)0x00800000))
  {
    return(SD_COM_CRC_FAILED);
  }

  if (response_r1 & ((u32)0x00400000))
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & ((u32)0x00200000))
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (response_r1 & ((u32)0x00100000))
  {
    return(SD_CC_ERROR);
  }

  if (response_r1 & ((u32)0x00080000))
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & ((u32)0x00040000))
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (response_r1 & ((u32)0x00020000))
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (response_r1 & ((u32)0x00010000))
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (response_r1 & ((u32)0x00008000))
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (response_r1 & ((u32)0x00004000))
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (response_r1 & ((u32)0x00002000))
  {
    return(SD_ERASE_RESET);
  }

  if (response_r1 & ((u32)0x00000008))
  {
    return(SD_AKE_SEQ_ERROR);
  }
  return(errorstatus);
}








 
static SD_Error CmdResp3Error(void)
{
  SD_Error errorstatus = SD_OK;
  u32 status;

  status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;

  while (!(status & (((uint32_t)0x00000001) | ((uint32_t)0x00000040) | ((uint32_t)0x00000004))))
  {
    status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;
  }

  if (status & ((uint32_t)0x00000004))
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(((uint32_t)0x00000004));
    return(errorstatus);
  }
   
  SDIO_ClearFlag(((u32)0x000005FF));
  return(errorstatus);
}








 
static SD_Error CmdResp2Error(void)
{
  SD_Error errorstatus = SD_OK;
  u32 status;

  status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;

  while (!(status & (((uint32_t)0x00000001) | ((uint32_t)0x00000004) | ((uint32_t)0x00000040))))
  {
    status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;
  }

  if (status & ((uint32_t)0x00000004))
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(((uint32_t)0x00000004));
    return(errorstatus);
  }
  else if (status & ((uint32_t)0x00000001))
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(((uint32_t)0x00000001));
    return(errorstatus);
  }

   
  SDIO_ClearFlag(((u32)0x000005FF));

  return(errorstatus);
}










 
static SD_Error CmdResp6Error(u8 cmd, u16 *prca)
{
  SD_Error errorstatus = SD_OK;
  u32 status;
  u32 response_r1;

  status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;

  while (!(status & (((uint32_t)0x00000001) | ((uint32_t)0x00000004) | ((uint32_t)0x00000040))))
  {
    status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;
  }

  if (status & ((uint32_t)0x00000004))
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(((uint32_t)0x00000004));
    return(errorstatus);
  }
  else if (status & ((uint32_t)0x00000001))
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(((uint32_t)0x00000001));
    return(errorstatus);
  }

   
  if (SDIO_GetCommandResponse() != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

   
  SDIO_ClearFlag(((u32)0x000005FF));

   
  response_r1 = SDIO_GetResponse(((uint32_t)0x00000000));

  if (((u32)0x00000000) == (response_r1 & (((u32)0x00002000) | ((u32)0x00004000) | ((u32)0x00008000))))
  {
    *prca = (u16) (response_r1 >> 16);
    return(errorstatus);
  }

  if (response_r1 & ((u32)0x00002000))
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & ((u32)0x00004000))
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & ((u32)0x00008000))
  {
    return(SD_COM_CRC_FAILED);
  }

  return(errorstatus);
}








 
static SD_Error SDEnWideBus(FunctionalState NewState)
{
  SD_Error errorstatus = SD_OK;

  u32 scr[2] = {0, 0};

  if (SDIO_GetResponse(((uint32_t)0x00000000)) & ((u32)0x02000000))
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

   
  errorstatus = FindSCR(RCA, scr);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

   
  if (NewState == ENABLE)
  {
     
    if ((scr[1] & ((u32)0x00040000)) != ((u32)0x00000000))
    {
       
      SDIO_CmdInitStructure.SDIO_Argument = (u32) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)55);
      SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
      SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
      SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(((uint8_t)55));

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

       
      SDIO_CmdInitStructure.SDIO_Argument = 0x2;
      SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)6);
      SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
      SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
      SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(((uint8_t)6));

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }    
  else
  {
     
    if ((scr[1] & ((u32)0x00010000)) != ((u32)0x00000000))
    {
       
      SDIO_CmdInitStructure.SDIO_Argument = (u32) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)55);
      SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
      SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
      SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(((uint8_t)55));

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

       
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)6);
      SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
      SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
      SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(((uint8_t)6));

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }
}








 
static SD_Error IsCardProgramming(u8 *pstatus)
{
  SD_Error errorstatus = SD_OK;
  vu32 respR1 = 0, status = 0;

  SDIO_CmdInitStructure.SDIO_Argument = (u32) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)13);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;
  while (!(status & (((uint32_t)0x00000001) | ((uint32_t)0x00000040) | ((uint32_t)0x00000004))))
  {
    status = ((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA;
  }

  if (status & ((uint32_t)0x00000004))
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(((uint32_t)0x00000004));
    return(errorstatus);
  }
  else if (status & ((uint32_t)0x00000001))
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(((uint32_t)0x00000001));
    return(errorstatus);
  }

  status = (u32)SDIO_GetCommandResponse();

   
  if (status != ((uint8_t)13))
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

   
  SDIO_ClearFlag(((u32)0x000005FF));


   
  respR1 = SDIO_GetResponse(((uint32_t)0x00000000));

   
  *pstatus = (u8) ((respR1 >> 9) & 0x0000000F);

  if ((respR1 & ((u32)0xFDFFE008)) == ((u32)0x00000000))
  {
    return(errorstatus);
  }

  if (respR1 & ((u32)0x80000000))
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (respR1 & ((u32)0x40000000))
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (respR1 & ((u32)0x20000000))
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (respR1 & ((u32)0x10000000))
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (respR1 & ((u32)0x08000000))
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (respR1 & ((u32)0x04000000))
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (respR1 & ((u32)0x01000000))
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (respR1 & ((u32)0x00800000))
  {
    return(SD_COM_CRC_FAILED);
  }

  if (respR1 & ((u32)0x00400000))
  {
    return(SD_ILLEGAL_CMD);
  }

  if (respR1 & ((u32)0x00200000))
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (respR1 & ((u32)0x00100000))
  {
    return(SD_CC_ERROR);
  }

  if (respR1 & ((u32)0x00080000))
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (respR1 & ((u32)0x00040000))
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (respR1 & ((u32)0x00020000))
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (respR1 & ((u32)0x00010000))
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (respR1 & ((u32)0x00008000))
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (respR1 & ((u32)0x00004000))
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (respR1 & ((u32)0x00002000))
  {
    return(SD_ERASE_RESET);
  }

  if (respR1 & ((u32)0x00000008))
  {
    return(SD_AKE_SEQ_ERROR);
  }

  return(errorstatus);
}








 
static SD_Error FindSCR(u16 rca, u32 *pscr)
{
  u32 index = 0;
  SD_Error errorstatus = SD_OK;
  u32 tempscr[2] = {0, 0};

   
   
  SDIO_CmdInitStructure.SDIO_Argument = (u32)8;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)16);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)16));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

   
  SDIO_CmdInitStructure.SDIO_Argument = (u32) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)55);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)55));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
  SDIO_DataInitStructure.SDIO_DataTimeOut = ((u32)0x000FFFFF);
  SDIO_DataInitStructure.SDIO_DataLength = 8;
  SDIO_DataInitStructure.SDIO_DataBlockSize = ((uint32_t)0x00000030);
  SDIO_DataInitStructure.SDIO_TransferDir = ((uint32_t)0x00000002);
  SDIO_DataInitStructure.SDIO_TransferMode = ((uint32_t)0x00000000);
  SDIO_DataInitStructure.SDIO_DPSM = ((uint32_t)0x00000001);
  SDIO_DataConfig(&SDIO_DataInitStructure);


   
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = ((uint8_t)51);
  SDIO_CmdInitStructure.SDIO_Response = ((uint32_t)0x00000040);
  SDIO_CmdInitStructure.SDIO_Wait = ((uint32_t)0x00000000);
  SDIO_CmdInitStructure.SDIO_CPSM = ((uint32_t)0x00000400);
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(((uint8_t)51));

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  while (!(((SDIO_TypeDef *) (((uint32_t)0x40000000) + 0x18000))->STA & (((uint32_t)0x00000020) | ((uint32_t)0x00000002) | ((uint32_t)0x00000008) | ((uint32_t)0x00000400) | ((uint32_t)0x00000200))))
  {
    if (SDIO_GetFlagStatus(((uint32_t)0x00200000)) != RESET)
    {
      *(tempscr + index) = SDIO_ReadData();
      index++;
    }
  }

  if (SDIO_GetFlagStatus(((uint32_t)0x00000008)) != RESET)
  {
    SDIO_ClearFlag(((uint32_t)0x00000008));
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(((uint32_t)0x00000002)) != RESET)
  {
    SDIO_ClearFlag(((uint32_t)0x00000002));
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(((uint32_t)0x00000020)) != RESET)
  {
    SDIO_ClearFlag(((uint32_t)0x00000020));
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(((uint32_t)0x00000200)) != RESET)
  {
    SDIO_ClearFlag(((uint32_t)0x00000200));
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

   
  SDIO_ClearFlag(((u32)0x000005FF));

  *(pscr + 1) = ((tempscr[0] & ((u32)0x000000FF)) << 24) | ((tempscr[0] & ((u32)0x0000FF00)) << 8) | ((tempscr[0] & ((u32)0x00FF0000)) >> 8) | ((tempscr[0] & ((u32)0xFF000000)) >> 24);

  *(pscr) = ((tempscr[1] & ((u32)0x000000FF)) << 24) | ((tempscr[1] & ((u32)0x0000FF00)) << 8) | ((tempscr[1] & ((u32)0x00FF0000)) >> 8) | ((tempscr[1] & ((u32)0xFF000000)) >> 24);

  return(errorstatus);
}








 
static u8 convert_from_bytes_to_power_of_two(u16 NumberOfBytes)
{
  u8 count = 0;

  while (NumberOfBytes != 1)
  {
    NumberOfBytes >>= 1;
    count++;
  }
  return(count);
}







 
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

   
  RCC_APB2PeriphClockCmd(((uint32_t)0x00000010) | ((uint32_t)0x00000020), ENABLE);

   
  GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0100) | ((uint16_t)0x0200) | ((uint16_t)0x0400) | ((uint16_t)0x0800) | ((uint16_t)0x1000);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1000)), &GPIO_InitStructure);

   
  GPIO_InitStructure.GPIO_Pin = ((uint16_t)0x0004);
  GPIO_Init(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1400)), &GPIO_InitStructure);
}








 
static void DMA_TxConfiguration(u32 *BufferSRC, u32 BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(((uint32_t)0x10002000) | ((uint32_t)0x10008000) | ((uint32_t)0x10004000) | ((uint32_t)0x10001000));

   
  DMA_Cmd(((DMA_Channel_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0444)), DISABLE);

   
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)((u32)0x40018080);
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)BufferSRC;
  DMA_InitStructure.DMA_DIR = ((uint32_t)0x00000010);
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = ((uint32_t)0x00000000);
  DMA_InitStructure.DMA_MemoryInc = ((uint32_t)0x00000080);
  DMA_InitStructure.DMA_PeripheralDataSize = ((uint32_t)0x00000200);
  DMA_InitStructure.DMA_MemoryDataSize = ((uint32_t)0x00000800);
  DMA_InitStructure.DMA_Mode = ((uint32_t)0x00000000);
  DMA_InitStructure.DMA_Priority = ((uint32_t)0x00002000);
  DMA_InitStructure.DMA_M2M = ((uint32_t)0x00000000);
  DMA_Init(((DMA_Channel_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0444)), &DMA_InitStructure);

   
  DMA_Cmd(((DMA_Channel_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0444)), ENABLE);
}








 
static void DMA_RxConfiguration(u32 *BufferDST, u32 BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(((uint32_t)0x10002000) | ((uint32_t)0x10008000) | ((uint32_t)0x10004000) | ((uint32_t)0x10001000));

   
  DMA_Cmd(((DMA_Channel_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0444)), DISABLE);

   
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)((u32)0x40018080);
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)BufferDST;
  DMA_InitStructure.DMA_DIR = ((uint32_t)0x00000000);
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = ((uint32_t)0x00000000);
  DMA_InitStructure.DMA_MemoryInc = ((uint32_t)0x00000080);
  DMA_InitStructure.DMA_PeripheralDataSize = ((uint32_t)0x00000200);
  DMA_InitStructure.DMA_MemoryDataSize = ((uint32_t)0x00000800);
  DMA_InitStructure.DMA_Mode = ((uint32_t)0x00000000);
  DMA_InitStructure.DMA_Priority = ((uint32_t)0x00002000);
  DMA_InitStructure.DMA_M2M = ((uint32_t)0x00000000);
  DMA_Init(((DMA_Channel_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0444)), &DMA_InitStructure);

   
  DMA_Cmd(((DMA_Channel_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x0444)), ENABLE);
}

 

