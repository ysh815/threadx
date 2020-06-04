;/**************************************************************************/
;/*                                                                        */
;/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
;/*                                                                        */
;/*       This software is licensed under the Microsoft Software License   */
;/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
;/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
;/*       and in the root directory of this software.                      */
;/*                                                                        */
;/**************************************************************************/
;
;
;/**************************************************************************/
;/**************************************************************************/
;/**                                                                       */
;/** ThreadX Component                                                     */
;/**                                                                       */
;/**   Initialize                                                          */
;/**                                                                       */
;/**************************************************************************/
;/**************************************************************************/
;
;#define TX_SOURCE_CODE
;
;
;/* Include necessary system files.  */
;
;#include "tx_api.h"
;#include "tx_initialize.h"
;#include "tx_thread.h"
;#include "tx_timer.h"
;
;
    IMPORT     _tx_thread_system_stack_ptr
    IMPORT     _tx_initialize_unused_memory
;   IMPORT     __RAM_segment_used_end__

    IMPORT     _tx_timer_interrupt
    IMPORT     __main
    IMPORT     __Vectors
    IMPORT     __initial_sp                     ; __RAM_segment_used_end__

;    IMPORT     __tx_SVCallHandler
;    IMPORT     __tx_PendSVHandler

;    EXPORT     __tx_NMIHandler                     ; NMI
;    EXPORT     __tx_BadHandler                     ; HardFault
;    EXPORT     __tx_SVCallHandler                  ; SVCall
;    EXPORT     __tx_DBGHandler                     ; Monitor
;    EXPORT     __tx_PendSVHandler                  ; PendSV
;    EXPORT     __tx_SysTickHandler                 ; SysTick
;    EXPORT     __tx_IntHandler                     ; Int 0
;
;

        IMPORT        SYSTICK_CYCLES
;SYSTEM_CLOCK      EQU   6000000
;SYSTICK_CYCLES    EQU   ((SYSTEM_CLOCK / 100) -1)

;__tx_NMIHandler         EQU    NMI_Handler
;__tx_HardfaultHandler   EQU    HardFault_Handler



        PRESERVE8
        THUMB

        AREA     |.text|, CODE, READONLY, ALIGN=2

;/**************************************************************************/
;/*                                                                                     */
;/*  FUNCTION                                               RELEASE                     */
;/*                                                                                     */
;/*    _tx_initialize_low_level                          Cortex-M4/GNU                  */
;/*                                                           6.0                       */
;/*  AUTHOR                                                                             */
;/*    Ysh                                                                              */
;/*                                                                                     */
;/*  DESCRIPTION                                                                        */
;/*     本函数处理所有底层的处理器初始化工作                                            */
;/*     包括 设置中断向量, 配置系统定时器中断源, 保存系统堆栈指针以供后续 ISR 处理,     */ 
;/*     以及 为 tx_application_define 查找第一块可用的 RAM 存储器地址                   */
;/*                                                                        */
;/*  INPUT                                                                 */
;/*    None                                                                */
;/*                                                                        */
;/*  OUTPUT                                                                */
;/*    None                                                                */
;/*                                                                        */
;/*  CALLS                                                                 */
;/*    None                                                                */
;/*                                                                        */
;/*  CALLED BY                                                             */
;/*    _tx_initialize_kernel_enter           ThreadX entry function        */
;/*                                                                        */
;/*  RELEASE HISTORY                                                       */
;/*                                                                        */
;/*    DATE              NAME                      DESCRIPTION             */
;/*                                                                        */
;/*  05-19-2020     William E. Lamie         Initial Version 6.0           */
;/*                                                                        */
;/**************************************************************************/
;VOID   _tx_initialize_low_level(VOID)
;{
;    .thumb_func
_tx_initialize_low_level    PROC
    EXPORT  _tx_initialize_low_level

;
;    /* 在ThreadX初始化期间禁用中断。Disable interrupts during ThreadX initialization.  */
;
    CPSID   i
;
;    /* Set base of available memory to end of non-initialised RAM area.  */
;
    LDR     r0, =_tx_initialize_unused_memory       ; Build address of unused memory pointer
;   LDR     r1, =__RAM_segment_used_end__           ; Build first free address
    LDR     r1, =__initial_sp                       ; Build first free address
    ADD     r1, r1, #4                              ;
    STR     r1, [r0]                                ; Setup first unused memory pointer
;
;    /* 设置向量表偏移寄存器。 Setup Vector Table Offset Register.  */
;
    MOV     r0, #0xE000E000                         ; Build address of NVIC registers
    LDR     r1, =__Vectors                          ; Pickup address of vector table
    STR     r1, [r0, #0xD08]                        ; Set vector table address
;
;    /* 用向量值设置 系统堆栈指针。Set system stack pointer from vector value.  */
;
    LDR     r0, =_tx_thread_system_stack_ptr        ; Build address of system stack pointer
    LDR     r1, =__Vectors                           ; Pickup address of vector table
    LDR     r1, [r1]                                ; Pickup reset stack pointer
    STR     r1, [r0]                                ; Save system stack pointer
;
;    /* 启用系统定时器 Enable the cycle count register.  */
;
    LDR     r0, =0xE0001000                         ; Build address of DWT register
    LDR     r1, [r0]                                ; Pickup the current value
    ORR     r1, r1, #1                              ; Set the CYCCNTENA bit
    STR     r1, [r0]                                ; Enable the cycle count register
;
;    /* Configure SysTick for 100Hz clock, or 16384 cycles if no reference.  */
;
    MOV     r0, #0xE000E000                         ; Build address of NVIC registers
    LDR     r1, =SYSTICK_CYCLES
    STR     r1, [r0, #0x14]                         ; Setup SysTick Reload Value
    MOV     r1, #0x7                                ; Build SysTick Control Enable Value
    STR     r1, [r0, #0x10]                         ; Setup SysTick Control
;
;    /* 配置中断优先级 Configure handler priorities.  */
;
    LDR     r1, =0x00000000                         ; Rsrv, UsgF, BusF, MemM
    STR     r1, [r0, #0xD18]                        ; Setup System Handlers 4-7 Priority Registers

    LDR     r1, =0xFF000000                         ; SVCl, Rsrv, Rsrv, Rsrv  SVCall 的优先级
    STR     r1, [r0, #0xD1C]                        ; Setup System Handlers 8-11 Priority Registers
                                                    ; Note: SVC must be lowest priority, which is 0xFF

    LDR     r1, =0x40FF0000                         ; SysT, PnSV, Rsrv, DbgM SysTick与PendSV的优先级
    STR     r1, [r0, #0xD20]                        ; Setup System Handlers 12-15 Priority Registers
                                                    ; Note: PnSV must be lowest priority, which is 0xFF
;
;    /* 函数退出 Return to caller.  */
;
    BX      lr

    ENDP
;}
;


; /* System Tick timer interrupt handler */
;__tx_SysTickHandler     PROC
;    EXPORT  __tx_SysTickHandler

SysTick_Handler         PROC
    EXPORT  SysTick_Handler
; VOID TimerInterruptHandler (VOID)
; {
;
    PUSH    {r0, lr}
    IF :DEF:TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    BL      _tx_execution_isr_enter             ; Call the ISR enter function
    ENDIF

    BL      _tx_timer_interrupt                 ; 定时器中断处理

    IF :DEF:TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    BL      _tx_execution_isr_exit              ; Call the ISR exit function
    ENDIF
    POP     {r0, lr}
    BX      LR

    ENDP
; }


; /* Generic interrupt handler template */
__tx_IntHandler     PROC
    EXPORT  __tx_IntHandler
; VOID InterruptHandler (VOID)
; {
    PUSH    {r0, lr}
    IF :DEF:TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    BL      _tx_execution_isr_enter             ; Call the ISR enter function
    ENDIF

;    /* Do interrupt handler work here */
;    /* BL <your C Function>.... */

    IF :DEF:TX_ENABLE_EXECUTION_CHANGE_NOTIFY
    BL      _tx_execution_isr_exit              ; Call the ISR exit function
    ENDIF
    POP     {r0, lr}
    BX      LR
    ENDP
; }



;/* Define shells for each of the unused vectors.  */
__tx_BadHandler     PROC
    EXPORT  __tx_BadHandler
    B       __tx_BadHandler
    ENDP

; /* NMI, DBG handlers */
__tx_NMIHandler     PROC
    EXPORT  __tx_NMIHandler
    B       __tx_NMIHandler
    ENDP

; /* added to catch the hardfault */
__tx_HardfaultHandler   PROC
    EXPORT  __tx_HardfaultHandler
    B       __tx_HardfaultHandler
    ENDP

; /* added to catch the SVC */
__tx_SVCallHandler  PROC
    EXPORT  __tx_SVCallHandler
    B       __tx_SVCallHandler
    ENDP

;    .thumb_func
__tx_DBGHandler     PROC
    EXPORT  __tx_DBGHandler
    B       __tx_DBGHandler
    ENDP

    ALIGN
    END
