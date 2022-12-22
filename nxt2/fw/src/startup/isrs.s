/****************************************************************************
 *
 * File   : isrs.s
 * Purpose: Interrupt service code for AT91SAM7S256 FLASH
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

/* Addresses and offsets of AIC and PIO */
.set  AT91C_BASE_AIC, 0xFFFFF000  /* (AIC) base address                     */
.set  AIC_IVR,   256    /* IRQ vector register offset from base above       */
.set  AIC_FVR,   260    /* FIQ vector register offset from base above       */
.set  AIC_EOICR, 304    /* End of interrupt command register                */

/* Interrupt control functions */
.global _irq_handler
.global _fiq_handler

/* ======================================================================== */
/* Function:             _irq_handler                                       */
/*                                                                          */
/* This IRQ handler supports nested interrupts (an IRQ interrupt can itself */
/* be interrupted).                                                         */
/*                                                                          */
/* This handler re-enables interrupts and switches to "Supervisor" mode to  */
/* prevent any corruption to the link and IP registers.                     */
/*                                                                          */
/* The Interrupt Vector Register (AIC_IVR) is read to determine the address */
/* of the required interrupt service routine. The ISR routine can be a      */
/* standard C function since this handler minds all the save/restore        */
/* protocols.                                                               */
/*                                                                          */
/*                                                                          */
/* Programmers:                                                             */
/*--------------------------------------------------------------------------*/
/*         ATMEL Microcontroller Software Support  -  ROUSSET  -            */
/*--------------------------------------------------------------------------*/
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS  */
/* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        */
/* WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND      */
/* NON-INFRINGEMENT ARE DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR   */
/* ANY DIRECT, INDIRECT,    INCIDENTAL, SPECIAL, EXEMPLARY, OR              */
/* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT    LIMITED TO, PROCUREMENT     */
/* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,    OR PROFITS; OR    */
/* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    LIABILITY, */
/* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING    NEGLIGENCE  */
/* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,        */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                       */
/* File source          : Cstartup.s79                                      */
/* Object               : Generic CStartup to AT91SAM7S256                  */
/* 1.0 09/May/06 JPP    : Creation                                          */
/*                                                                          */ 
/*                                                                          */
/* Note: taken from Atmel web site (www.at91.com)                           */
/*         Keil example project:  AT91SAM7S-Interrupt_SAM7S                 */
/* ======================================================================== */
_irq_handler:
        /* Adjust and save LR_irq on the IRQ stack. */
        sub     lr, lr, #4
        stmfd   sp!, {lr}

        /* Save r0 and  SPSR on the IRQ stack (for nested interrupts). */
        mrs     r14, SPSR
        stmfd   sp!, {r0, r14}

        /* Write in the IVR to support protect mode. Has no effect in normal 
           mode. De-assert the NIRQ and clear the source in protect mode. */
        ldr     r14, =AT91C_BASE_AIC
        ldr     r0, [r14, #AIC_IVR]
        str     r14, [r14, #AIC_IVR]

        /* Switch in supervisor mode with interrupts disabled. */
        msr     CPSR_c, #ARM_MODE_SVC | I_BIT | F_BIT

        /* Save scratch/used registers and LR in user stack. */
        stmfd   sp!, {r1-r3, r12, r14}

        /* Enable interrupts. */
        msr     CPSR_c, #ARM_MODE_SVC & ~I_BIT

        /* Branch to the routine pointed by the AIC_IVR. */
        mov     r14, pc
        bx      r0

        /* Disable interrupts. */
        msr     CPSR_c, #ARM_MODE_SVC | I_BIT

        /* Restore scratch/used registers and LR from user stack. */
        ldmia   sp!, {r1-r3, r12, r14}

        /* Switch back in IRQ mode with interrupts disabled. */
        msr     CPSR_c, #ARM_MODE_IRQ | I_BIT | F_BIT

        /* Mark the end of interrupt on the AIC. */
        ldr     r14, =AT91C_BASE_AIC
        str     r14, [r14, #AIC_EOICR]

        /* Restore SPSR_irq and r0 from IRQ stack. */
        ldmia   sp!, {r0, r14}
        msr     SPSR_cxsf, r14

        /* Restore adjusted LR_irq from IRQ stack directly in the PC. */
        ldmia   sp!, {pc}^

.end
