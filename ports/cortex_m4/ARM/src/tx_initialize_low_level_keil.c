/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** ThreadX Component                                                     */
/**                                                                       */
/**   Initialize                                                          */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

#define TX_SOURCE_CODE


/* Include necessary system files.  */

#include "tx_api.h"
//#include "tx_initialize.h"
//#include "tx_thread.h"
//#include "tx_timer.h"
// #include "ARMCM4_FP.h"
//#include    "system_ARMCM4.h"
#include    <XMC4500.h>
#include    "system_XMC4500.h"


extern  uint32_t    _tx_thread_system_stack_ptr;
extern  uint32_t    _tx_initialize_unused_memory;
//extern uint32_t __RAM_segment_used_end__

extern  uint32_t    __Vectors;
//extern  uint32_t    __initial_sp;

extern  void    _tx_timer_interrupt(void);

//;    IMPORT     __tx_SVCallHandler
//;    IMPORT     __tx_PendSVHandler

//;    EXPORT     __tx_NMIHandler                     ; NMI
//;    EXPORT     __tx_BadHandler                     ; HardFault
//;    EXPORT     __tx_SVCallHandler                  ; SVCall
//;    EXPORT     __tx_DBGHandler                     ; Monitor
//;    EXPORT     __tx_PendSVHandler                  ; PendSV
//;    EXPORT     __tx_SysTickHandler                 ; SysTick
//;    EXPORT     __tx_IntHandler                     ; Int 0
//;



/**************************************************************************/
/*                                                                                     */
/*  FUNCTION                                               RELEASE                     */
/*                                                                                     */
/*    _tx_initialize_low_level                          Cortex-M4/GNU                  */
/*                                                           6.0                       */
/*  AUTHOR                                                                             */
/*    Ysh                                                                              */
/*                                                                                     */
/*  DESCRIPTION                                                                        */
/*     本函数处理所有底层的处理器初始化工作                                            */
/*     包括 设置中断向量, 配置系统定时器中断源, 保存系统堆栈指针以供后续 ISR 处理,     */ 
/*     以及 为 tx_application_define 查找第一块可用的 RAM 存储器地址                   */
/*                                                                                     */
/*  INPUT                                                                               */
/*    None                                                                              */
/*                                                                                      */
/*  OUTPUT                                                                              */
/*    None                                                                              */
/*                                                                                      */
/*  CALLS                                                                               */
/*    None                                                                              */
/*                                                                                      */
/*  CALLED BY                                                                           */
/*    _tx_initialize_kernel_enter           ThreadX entry function                      */
/*                                                                                      */
/*  RELEASE HISTORY                                                                     */
/*                                                                                      */
/*    DATE              NAME                      DESCRIPTION                           */
/*                                                                                      */
/*  05-19-2020     William E. Lamie         Initial Version 6.0                         */
/*                                                                                      */
/****************************************************************************************/
VOID   _tx_initialize_low_level(VOID)
{
    /* 在ThreadX初始化期间禁用中断。Disable interrupts during ThreadX initialization.  */
    __disable_irq();                            // CPSID   i

    /* Set base of available memory to end of non-initialised RAM area.  */
//  _tx_initialize_unused_memory = __RAM_segment_used_end__;
//  _tx_initialize_unused_memory = __initial_sp;
    _tx_initialize_unused_memory = __Vectors;

    /* 设置向量表偏移寄存器。 Setup Vector Table Offset Register.  */
    SCB->VTOR = (uint32_t)(&__Vectors);

    /* 用向量值设置 系统堆栈指针。Set system stack pointer from vector value.  */
    _tx_thread_system_stack_ptr = __Vectors;

    /* 启用系统定时器 Enable the cycle count register.  */
//  DWT->CTRL |= 1;

    /* Configure SysTick for 100Hz clock, or 16384 cycles if no reference.  */
    SysTick_Config(SystemCoreClock/1000);

    /* 配置中断优先级 Configure handler priorities.  */
    SCB->SHP[0] = 0x00;                    // Rsrv, UsgF, BusF, MemM  Setup System Handlers 4-7 Priority Registers
    SCB->SHP[1] = 0x00;
    SCB->SHP[2] = 0x00;
    SCB->SHP[3] = 0x00;
//  PPB->SHPR2 = 0xFF000000;                    // SVCl, Rsrv, Rsrv, Rsrv  SVCall 的优先级 
    SCB->SHP[4] = 0x00;
    SCB->SHP[5] = 0x00;
    SCB->SHP[6] = 0x00;
    SCB->SHP[7] = 0xFF;
    //  setup System Handlers 8-11 Priority Registers   Note: SVC must be lowest priority, which is 0xFF
//  PPB->SHPR3 = 0x40FF0000;                    // SysT, PnSV, Rsrv, DbgM SysTick与PendSV的优先级
    SCB->SHP[8] = 0x00;
    SCB->SHP[9] = 0x00;
    SCB->SHP[10] = 0xFF;
    SCB->SHP[11] = 0x40;
    //; Setup System Handlers 12-15 Priority Registers   Note: PnSV must be lowest priority, which is 0xFF
}



// 定时器中断服务程序
// __tx_SysTickHandler
void SysTick_Handler(void)
{
#ifdef  TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    _tx_execution_isr_enter();                  // Call the ISR enter function
#endif

    _tx_timer_interrupt();                      // 定时器中断处理

#ifdef  TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    _tx_execution_isr_exit();                   // Call the ISR exit function
#endif
}


/*
; // Generic interrupt handler template 
__tx_IntHandler     PROC
    EXPORT  __tx_IntHandler
; VOID InterruptHandler (VOID)
; {
    PUSH    {r0, lr}
    IF :DEF:TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    BL      _tx_execution_isr_enter             ; Call the ISR enter function
    ENDIF

;    // Do interrupt handler work here
;    // BL <your C Function>...

    IF :DEF:TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    BL      _tx_execution_isr_exit              ; Call the ISR exit function
    ENDIF
    POP     {r0, lr}
    BX      LR
    ENDP
; }
*/

/*
;// Define shells for each of the unused vectors. 
__tx_BadHandler     PROC
    EXPORT  __tx_BadHandler
    B       __tx_BadHandler
    ENDP

; // NMI, DBG handlers 
__tx_NMIHandler     PROC
    EXPORT  __tx_NMIHandler
    B       __tx_NMIHandler
    ENDP

; // added to catch the hardfault 
__tx_HardfaultHandler   PROC
    EXPORT  __tx_HardfaultHandler
    B       __tx_HardfaultHandler
    ENDP

; // added to catch the SVC
__tx_SVCallHandler  PROC
    EXPORT  __tx_SVCallHandler
    B       __tx_SVCallHandler
    ENDP

;    .thumb_func
__tx_DBGHandler     PROC
    EXPORT  __tx_DBGHandler
    B       __tx_DBGHandler
    ENDP
*/
