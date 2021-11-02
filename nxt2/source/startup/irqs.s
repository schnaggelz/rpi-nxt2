/****************************************************************************
 *
 * File   : irqs.s
 * Purpose: Interrupt control code for AT91SAM7S256 FLASH
 *
 ****************************************************************************/

.text        /* all assembler code that follows will go into .text section  */
.arm         /* compile for 32-bit ARM instruction set                      */
.align       /* align section on 32-bit boundary                            */

/* Standard definitions of mode bits and interrupt (I&F) flags in PSRs
   (PSR = program status registers). */

.set  ARM_MODE_IRQ, 0x12  /* IRQ processing standard interrupts mode        */
.set  ARM_MODE_SVC, 0x13  /* Supervisor processing software interrupts mode */
.set  I_BIT, 0x80    /* I bit set = IRQ disabled (program status registers) */
.set  F_BIT, 0x40    /* F bit set = FIQ disabled (program status registers) */

/* Interrupt control functions. */
.global _irqs_disable
.global _irqs_enable
.global _fiqs_disable
.global _fiqs_enable

/* Function _irqs_disable. Disables all interrupts. */
_irqs_disable:
        mrs     r0, cpsr
        orr     r0, r0, #I_BIT
        msr     cpsr_c, r0
        bx      lr

/* Function _irqs_enable. Enables all interrupts. */
_irqs_enable:
        mrs     r0, cpsr
        bic     r0, r0, #I_BIT
        msr     cpsr_c, r0
        bx      lr

/* Function _fiqs_disable. Disables all fast interrupts. */
_fiqs_disable:
        mrs     r0, cpsr
        orr     r0, r0, #F_BIT
        msr     cpsr_c, r0
        bx      lr

/* Function _fiqs_enable. Enables all fast interrupts. */
_fiq_enable:
        mrs     r0, cpsr
        bic     r0, r0, #F_BIT
        msr     cpsr_c, r0
        bx      lr
