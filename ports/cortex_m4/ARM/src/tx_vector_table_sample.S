

    EXPORT reset_handler

    EXPORT __tx_NMIHandler
    EXPORT __tx_BadHandler
    EXPORT __tx_SVCallHandler
    EXPORT __tx_DBGHandler
    EXPORT __tx_PendSVHandler 
    EXPORT __tx_SysTickHandler 
    EXPORT __tx_BadHandler


  .syntax unified
  .section .vectors, "ax"
  .code 16
  .align 0
  EXPORT _vectors

_vectors:
  .word __stack_end__ 
  .word reset_handler 
  .word __tx_NMIHandler 
  .word __tx_HardfaultHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word 0 // Reserved
  .word 0 // Reserved
  .word 0 // Reserved
  .word 0 // Reserved
  .word __tx_SVCallHandler //_SVC_Handler - used by Threadx scheduler //
  .word __tx_DBGHandler
  .word 0 // Reserved
  .word __tx_PendSVHandler 
  .word __tx_SysTickHandler   // Used by Threadx timer functionality
  .word __tx_BadHandler       // Populate with user Interrupt handler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler
  .word __tx_BadHandler



    .section .init, "ax"
    .thumb_func
reset_handler:

// low level hardware config, such as PLL setup goes here

  b _start



