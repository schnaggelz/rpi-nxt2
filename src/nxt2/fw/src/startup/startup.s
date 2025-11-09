/***********************************************************************************
 *
 * File   : startup.s
 * Purpose: Init code for AT91SAM7S256
 *
 ***********************************************************************************/

.text           /* All assembler code that follows will go into .text section. */
.arm            /* Compile for 32-bit ARM instruction set. */
.align          /* Align section on 32-bit boundary. */

/* Standard definitions of mode bits and interrupt (I&F) flags in PSRs
   (PSR = program status registers).                                               */

.set  ARM_MODE_USR, 0x10 /* Normal user mode                                       */
.set  ARM_MODE_FIQ, 0x11 /* FIQ processing fast interrupts mode                    */
.set  ARM_MODE_IRQ, 0x12 /* IRQ processing standard interrupts mode                */
.set  ARM_MODE_SVC, 0x13 /* Supervisor processing software interrupts mode         */
.set  ARM_MODE_ABT, 0x17 /* Abort processing memory faults mode                    */
.set  ARM_MODE_UND, 0x1B /* Processing undefined instructions mode                 */
.set  ARM_MODE_SYS, 0x1F /* System running priviledged operating system tasks mode */
.set  I_BIT, 0x80        /* When I bit is set, IRQ is disabled (PSRs)              */
.set  F_BIT, 0x40        /* When F bit is set, FIQ is disabled (PSRs)              */

/* Vector table */
.global _vector
.extern _irq_handler
.extern _fiq_handler

.section .vectors, "ax"

_vector:
        ldr     pc, v0    /* Reset vector */
        ldr     pc, v1    /* Undefined instruction */
        ldr     pc, v2    /* Software interrupt */
        ldr     pc, v3    /* Prefetch abort */
        ldr     pc, v4    /* Data abort */
        ldr     pc, v5    /* Reserved */
        ldr     pc, v6    /* IRQ */
        ldr     pc, v7    /* FIQ */

v0:     .word   _init_reset_handler
v1:     .word   _und_handler
v2:     .word   _swi_handler
v3:     .word   _pab_handler
v4:     .word   _dab_handler
v5:     .word   _rsv_handler
v6:     .word   _irq_handler
v7:     .word   _fiq_handler

/**********************************************************************
* _init_reset_handler
*
* Execution starts here.
* After a reset, the mode is ARM, supervisor, interrupts disabled.
*/
.global _init_reset_handler
.global _und_handler
.global _swi_handler
.global _pab_handler
.global _dab_handler
.global _rsv_handler
.extern _crt0
.arm
.section .text, "ax"

_init_reset_handler:
        /*
         * Setup a stack for each mode
         */
        msr   CPSR_c, #ARM_MODE_UND | I_BIT | F_BIT     /* Undefined Instruction Mode */
        ldr   sp, =__stack_und_end__

        msr   CPSR_c, #ARM_MODE_ABT | I_BIT | F_BIT     /* Abort Mode */
        ldr   sp, =__stack_abt_end__

        msr   CPSR_c, #ARM_MODE_FIQ   | I_BIT | F_BIT   /* FIQ Mode */
        ldr   sp, =__stack_fiq_end__

        msr   CPSR_c, #ARM_MODE_IRQ   | I_BIT | F_BIT   /* IRQ Mode */
        ldr   sp, =__stack_irq_end__

        msr   CPSR_c, #ARM_MODE_SVC   | I_BIT | F_BIT   /* Supervisor Mode */
        ldr   sp, =__stack_svc_end__

        /*
         * Now enter crt0 function,
         * which does low-level and segment initialization.
         * and then calls main().
         */
        ldr   r0, =_crt0
        mov   lr, pc
        bx    r0
end:    b     end

/* Dummy handlers. */
_und_handler:   b    _und_handler
_swi_handler:   b    _swi_handler
_pab_handler:   b    _pab_handler
_dab_handler:   b    _dab_handler
_rsv_handler:   b    _rsv_handler
_fiq_handler:   b    _fiq_handler

.end
