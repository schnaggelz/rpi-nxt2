/***********************************************************************************
 *
 * File   : crt.s
 * Purpose: C init code for AT91SAM7S256
 *
 ***********************************************************************************/

.text           /* All assembler code that follows will go into .text section. */
.arm            /* Compile for 32-bit ARM instruction set. */
.align          /* Align section on 32-bit boundary. */

/* Standard definitions of mode bits and interrupt (I&F) flags in PSRs
   (PSR = program status registers).                                               */

.set  I_BIT, 0x80        /* When I bit is set, IRQ is disabled (PSRs)              */
.set  F_BIT, 0x40        /* When F bit is set, FIQ is disabled (PSRs)              */

.global _crt0
.global _exit
.extern _low_level_init
.extern main

/* Macros to do memory initialization. */
.macro mem_copy, src_start, src_end, dest_start
        ldr     r0, =\src_start
        ldr     r1, =\src_end
        ldr     r2, =\dest_start
        bl      mem_copy_func
.endm

.macro mem_init, dest_start, dest_end, val
        ldr     r0, =\dest_start
        ldr     r1, =\dest_end
        ldr     r2, =\val
        bl      mem_init_func
.endm

_crt0:
        /*
         * Call _low_level_init to initialize hardware.
         */
        ldr     r0,=_low_level_init
        mov     lr, pc
        bx      r0

mem_setup:
        /*
         * Initialize the RAM (data, stack and bss).
         */
        mem_copy    __data_load_start__, __data_load_end__, __data_start__
        mem_init    __bss_start__, __bss_end__, 0

call_main:
        /*
         *  Prepare and call main()
         */
        mrs     r0, cpsr
        bic     r0, r0, #(I_BIT | F_BIT)  /* Enable FIQ and IRQ interrupt */
        msr     cpsr, r0
        mov     r0, #0 /* No arguments are passed to main */
        mov     r1, #0
        ldr     r2, =main
        mov     lr, pc
        bx      r2
        /* If we get here then main returned -- bad! */
main_returned:
        b       main_returned


/* Function mem_copy_func, r0 = source start, r1 = end of source, r2 = destination. */
mem_copy_func:
        /* Bail out if source and dest addresses are the same. */
        cmp     r0, r2
        bxeq    lr
        /* Test if all addressed are 16-byte aligned, if so use ldm/stm copy. */
        mov     r3, r0
        orr     r3, r3, r1
        orr     r3, r3, r2
        ands    r3, r3, #15
        beq     mcf_16_aligned
mcf_4_aligned:
l1:     cmp     r0, r1
        ldrlo   r3, [r0], #4
        strlo   r3, [r2], #4
        blo     l1
        bx      lr
mcf_16_aligned:
l2:     cmp     r0, r1
        ldmloia r0!, {r3-r6}
        stmloia r2!, {r3-r6}
        blo     l2
        bx      lr

/* Function mem_init_func: r0 = start address, r1 = end address, r2 is value to write. */
mem_init_func:
        /* Test if start and end addresses are multiples of 16, if so use stm store. */
        orr     r3, r0, r1
        ands    r3, r2, #15
        beq     mif_16_aligned
mif_4_aligned:
l3:     cmp     r0, r1
        strlo   r2, [r0], #4
        blo     l3
        bx      lr
mif_16_aligned:
        mov     r3, r2
        mov     r4, r2
        mov     r5, r2
l4:     cmp     r0, r1
        stmloia r0!, {r2-r5}
        blo     l4
        bx      lr

.end
