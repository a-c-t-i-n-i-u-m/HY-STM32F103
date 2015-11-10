/*
***************
*/

/*
*********************************************************************************************************
编译器版本：MDK3.5  基于STM32 STD3.0库
硬件平台：HY-STM32开发板
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

//#include <includes.h>
#include "includes.h"
//#include "stm32f10x.h"                                

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  OS_STK App_TaskStartStk[APP_TASK_START_STK_SIZE];
//static  OS_STK App_TaskLCDStk[APP_TASK_LCD_STK_SIZE];
//static  OS_STK App_TaskKbdStk[APP_TASK_KBD_STK_SIZE];
//static  OS_STK App_TaskJoystickStk[APP_TASK_Joystick_STK_SIZE];

//static  OS_EVENT* InfoSem;                      
//static  OS_EVENT* Disp_Box;                  
//static  char* dp;
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static  void App_TaskCreate(void);
static  void App_DispScr_SignOn(void);

static  void App_TaskStart(void* p_arg);
//static  void App_TaskLCD(void* p_arg);
//static  void App_TaskKbd(void* p_arg);
//static  void App_TaskJoystick(void* p_arg);

#define LED_LED1_ON()   GPIO_SetBits(GPIOC, GPIO_Pin_6 );  	   //LED1 
#define LED_LED1_OFF()  GPIO_ResetBits(GPIOC, GPIO_Pin_6 ); 	   //LED1


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Argument : none.
*
* Return   : none.
*********************************************************************************************************
*/

int main(void)
{
   CPU_INT08U os_err;

   BSP_IntDisAll();                                            /* Disable all ints until we are ready to accept them.  */
  
   
   OSInit();                                                   /* Initialize "uC/OS-II, The Real-Time Kernel".         */

   BSP_Init();                                                 /* Initialize BSP functions.  */
   
   
   App_DispScr_SignOn();


   printf("OS Init OK！\r\n");
   os_err = OSTaskCreate((void (*) (void *)) App_TaskStart,
               /* Create the start task.                               */
                          (void *) 0,
               (OS_STK *) &App_TaskStartStk[APP_TASK_START_STK_SIZE - 1],
               (INT8U) APP_TASK_START_PRIO);
   printf("Creat App_TaskStart！\r\n");
#if (OS_TASK_NAME_SIZE >= 11)
   OSTaskNameSet(APP_TASK_START_PRIO, (CPU_INT08U *) "Start Task", &os_err);
#endif


   OSTimeSet(0);
   OSStart();                                                  /* Start multitasking (i.e. give control to uC/OS-II).  */

   return (0);
}

/*
*********************************************************************************************************
*                                          App_TaskStart()
*
* Description : The startup task.  The uC/OS-II ticker should only be initialize once multitasking starts.
*
* Argument : p_arg       Argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return   : none.
*
* Caller   : This is a task.
*
* Note     : none.
*********************************************************************************************************
*/

static  void App_TaskStart(void* p_arg)
{
   
   (void) p_arg;

   OS_CPU_SysTickInit();                                       /* Initialize the SysTick.       */

#if (OS_TASK_STAT_EN > 0)
   OSStatInit();                                               /* Determine CPU capacity.                              */
#endif

   //App_TaskCreate();
   //App_DispScr_SignOn();
   while (DEF_TRUE)
   {
      LED_LED1_ON();
      OSTimeDlyHMSM(0, 0, 0, 100);
      
      LED_LED1_OFF();
      OSTimeDlyHMSM(0, 0, 0, 100);
   }
}

/*
*********************************************************************************************************
*                                            App_TaskCreate()
*
* Description : Create the application tasks.
*
* Argument : none.
*
* Return   : none.
*
* Caller   : App_TaskStart().
*
* Note     : none.
*********************************************************************************************************
*/

static  void App_TaskCreate(void)
{
   CPU_INT08U os_err;

//   os_err = OSTaskCreate((void (*) (void *)) App_TaskLCD, (void *) 0,
//               (OS_STK *) &App_TaskLCDStk[APP_TASK_LCD_STK_SIZE - 1],
//               (INT8U) APP_TASK_LCD_PRIO);
//   printf("Creat App_TaskLCD！\r\n");
//#if (OS_TASK_NAME_SIZE >= 9)
//   OSTaskNameSet(APP_TASK_LCD_PRIO, "LCD", &os_err);
//#endif

//   os_err = OSTaskCreate((void (*) (void *)) App_TaskKbd, (void *) 0,
//               (OS_STK *) &App_TaskKbdStk[APP_TASK_KBD_STK_SIZE - 1],
//               (INT8U) APP_TASK_KBD_PRIO);
//   printf("Creat App_TaskKbd！\r\n");
//#if (OS_TASK_NAME_SIZE >= 9)
//   OSTaskNameSet(APP_TASK_KBD_PRIO, "KeyBoard", &os_err);
//#endif

//   os_err = OSTaskCreate((void (*) (void *))
//               App_TaskJoystick, (void *)
//               0,
//               (OS_STK *) &App_TaskJoystickStk[APP_TASK_Joystick_STK_SIZE - 1],
//               (INT8U)
//               APP_TASK_Joystick_PRIO);
//   printf("Creat App_TaskJoystick！\r\n");
//#if (OS_TASK_NAME_SIZE >= 9)
//   OSTaskNameSet(APP_TASK_Joystick_PRIO, "Joystick", &os_err);
//#endif
}




/*
*********************************************************************************************************
*                                          App_DispScr_SignOn()
*
* Description : Display uC/OS-II system information on the USART.
*
* Argument: none.
*
* Return   : none.
*
* Caller   : App_TaskKbd().
*
* Note    : none.
*********************************************************************************************************
*/

static  void App_DispScr_SignOn(void)
{
   printf("\r\n  Micrium uC/OS-II  \r\n");
   printf("  ST STM32 (Cortex-M3)\r\n\r\n");

   printf("  uC/OS-II:  V%ld.%ld%ld\r\n", OSVersion() / 100,
      (OSVersion() % 100) / 10, (OSVersion() % 10));
   printf("  TickRate: %ld  \r\n", OS_TICKS_PER_SEC);
   printf("  CPU Usage: %ld%    \r\n", OSCPUUsage);
   printf("  CPU Speed:%ld MHz  \r\n", BSP_CPU_ClkFreq() / 1000000L);
   printf("  #Ticks: %ld  \r\n", OSTime);
   printf("  #CtxSw: %ld  \r\n\r\n", OSCtxSwCtr);
}

























/*
*********************************************************************************************************
*********************************************************************************************************
*                                          uC/OS-II APP HOOKS
*********************************************************************************************************
*********************************************************************************************************
*/

#if (OS_APP_HOOKS_EN > 0)
/*
*********************************************************************************************************
*                                      TASK CREATION HOOK (APPLICATION)
*
* Description : This function is called when a task is created.
*
* Argument : ptcb   is a pointer to the task control block of the task being created.
*
* Note     : (1) Interrupts are disabled during this call.
*********************************************************************************************************
*/

void App_TaskCreateHook(OS_TCB* ptcb)
{
}

/*
*********************************************************************************************************
*                                    TASK DELETION HOOK (APPLICATION)
*
* Description : This function is called when a task is deleted.
*
* Argument : ptcb   is a pointer to the task control block of the task being deleted.
*
* Note     : (1) Interrupts are disabled during this call.
*********************************************************************************************************
*/

void App_TaskDelHook(OS_TCB* ptcb)
{
   (void) ptcb;
}

/*
*********************************************************************************************************
*                                      IDLE TASK HOOK (APPLICATION)
*
* Description : This function is called by OSTaskIdleHook(), which is called by the idle task.  This hook
*               has been added to allow you to do such things as STOP the CPU to conserve power.
*
* Argument : none.
*
* Note     : (1) Interrupts are enabled during this call.
*********************************************************************************************************
*/

#if OS_VERSION >= 251
void App_TaskIdleHook(void)
{
}
#endif

/*
*********************************************************************************************************
*                                        STATISTIC TASK HOOK (APPLICATION)
*
* Description : This function is called by OSTaskStatHook(), which is called every second by uC/OS-II's
*               statistics task.  This allows your application to add functionality to the statistics task.
*
* Argument : none.
*********************************************************************************************************
*/

void App_TaskStatHook(void)
{
}

/*
*********************************************************************************************************
*                                        TASK SWITCH HOOK (APPLICATION)
*
* Description : This function is called when a task switch is performed.  This allows you to perform other
*               operations during a context switch.
*
* Argument : none.
*
* Note     : 1 Interrupts are disabled during this call.
*
*            2  It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
*                   will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the
*                  task being switched out (i.e. the preempted task).
*********************************************************************************************************
*/

#if OS_TASK_SW_HOOK_EN > 0
void App_TaskSwHook(void)
{
}
#endif

/*
*********************************************************************************************************
*                                     OS_TCBInit() HOOK (APPLICATION)
*
* Description : This function is called by OSTCBInitHook(), which is called by OS_TCBInit() after setting
*               up most of the TCB.
*
* Argument : ptcb    is a pointer to the TCB of the task being created.
*
* Note     : (1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/

#if OS_VERSION >= 204
void App_TCBInitHook(OS_TCB* ptcb)
{
   (void) ptcb;
}
#endif

#endif
