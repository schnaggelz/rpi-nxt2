/***********************************************************************************
 *
 * File   : nxt_bios.lds
 * Purpose: Linker file for AT91SAM7S64 FLASH
 *
 *  This script is for a system that runs with the memory mapped so that RAM is
 *  visible at both 0 and 2M.
 *
 *  Notes on alignment:
 *  1) Sections should be 4-byte aligned otherwise ARM fetches will be misaligned.
 *  2) The stack should be 8-byte aligned for the APCS. That's because STRD and
 *     LDRD assume that they are on 8-byte boundaries.
 *  3) Align 16 is a good idea for any copied or initialised sections because
 *     this allows the faster LDM/STM code to run instead.
 *
 ***********************************************************************************/

ENTRY(_vector)

/*
 * Stack sizes
 */

FIQ_STACK_SIZE = 0x0;
IRQ_STACK_SIZE = 0x0100;
ABT_STACK_SIZE = 0x0;
UND_STACK_SIZE = 0x0;
SVC_STACK_SIZE = 0x0400;

/*
 * Memory definitions
 */

MEMORY
{
  ROM (rx) : ORIGIN = 1M, LENGTH = 256K
  RAM (wx) : ORIGIN = 2M, LENGTH = 64K
}

ROM_BASE = 1M;
ROM_SIZE = 256k;
RAM_BASE = 2M;
RAM_SIZE = 64k;

/*
 * Section definitions
 */

SECTIONS
{
  /* The main program area is placed in ROM. */
  .text :
  {
    *(.vectors);
    . = ALIGN(4);
    *(.init);
    . = ALIGN(4);
    *(.text);
    . = ALIGN(4);
    *(.rodata);
    . = ALIGN(4);
    *(.rodata*);
    . = ALIGN(4);
    *(.glue_7t);
    . = ALIGN(4);
    *(.glue_7);
    . = ALIGN(4);
  } > ROM

  __text_start__  = ADDR(.text);
  __text_end__    = ADDR(.text) + SIZEOF(.text);

  .init_array :
  {
    KEEP (*(.init_array*))
    . = ALIGN(4);
  } > ROM

  __init_array_start__  = ADDR(.init_array);
  __init_array_end__    = ADDR(.init_array) + SIZEOF(.init_array);

  /* Initialized data has two addresses: VMA (run-time address) and LMA (load address).
   * LMA is where the data is stored in ROM and the init code copies it into RAM. */
  .data :
  {
    * (*.data *.data.*)
    . = ALIGN(16);
  } > RAM
  AT> ROM

  __data_start__  = ADDR(.data);
  __data_end__    = ADDR(.data) + SIZEOF(.data);

  __data_load_start__ = LOADADDR(.data);
  __data_load_end__   = __data_load_start__ + SIZEOF(.data) ;

  . = ALIGN(4);
  _edata = . ;
  PROVIDE (edata = .);

  /* Zero initialized data is placed in RAM. */
  .bss :
  {
    * (*.bss *.bss.*)
    . = ALIGN(16);
  } > RAM

  __bss_start__ = ADDR(.bss);
  __bss_end__   = (ADDR(.bss) + SIZEOF(.bss));

  /* The stack is placed in RAM. It does not have to be initialized, but we do this to
   * monitor stack usage. It needs to be 8-byte aligned, but we align it to 16 bytes.
   * Notes about stack usage:
   * 1) Stack grows down.
   * 2) When an interrupt service routine is entered, it starts using the IRQ stack.
   *    If the interrupts are nesting then we have to switch to the system (or some
   *    other non-IRQ) stack. For this reason, the IRQ stack only needs to be big
   *    enough to hold a few registers for nested mode and the real stack usage is
   *    on the system stack. */
  .stack :
  {
    PROVIDE (__stack_fiq_start__ = .);
    . += FIQ_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_fiq_end__ = .);

    PROVIDE (__stack_irq_start__ = .);
    . += IRQ_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_irq_end__ = .);

    PROVIDE (__stack_abt_start__ = .);
    . += ABT_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_abt_end__ = .);

    PROVIDE (__stack_und_start__ = .);
    . += UND_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_und_end__ = .);

    PROVIDE (__stack_svc_start__ = .);
    . += SVC_STACK_SIZE;
    . = ALIGN(4);
    PROVIDE (__stack_svc_end__ = .);
  } > RAM

  __stack_start__ = ADDR(.stack);
  __stack_end__   = (ADDR(.stack) + SIZEOF(.stack));

  __free_ram_start__ = __bss_end__ ;
  __free_ram_end__ = (RAM_BASE + RAM_SIZE);

  __free_rom_start__ = __text_end__ ;
  __free_rom_end__ = (ROM_BASE + ROM_SIZE);

  REPORTME_FREE_RAM_SIZE = __free_ram_end__ - __free_ram_start__;
  REPORTME_FREE_ROM_SIZE = __free_rom_end__ - __free_rom_start__;
}
