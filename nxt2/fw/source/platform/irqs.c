/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment. Inspired by the Lejos project.
*
* License notes see LICENSE.txt
*******************************************************************************/

#include "platform/irqs.h"

#define IRQ_MASK 0x00000080
#define FIQ_MASK 0x00000040
#define INT_MASK (IRQ_MASK | FIQ_MASK)

static inline unsigned __get_cpsr(void)
{
	unsigned long retval;
	asm volatile (" mrs %0, cpsr" : "=r" (retval) : /* no inputs */  );
	return retval;
}

static inline void __set_cpsr(unsigned val)
{
	asm volatile (" msr cpsr, %0" : /* no outputs */ : "r" (val)  );
}

unsigned irqs_get_and_disable(void)
{
	unsigned _cpsr;
	_cpsr = __get_cpsr();
	__set_cpsr(_cpsr | IRQ_MASK);
	return _cpsr;
}

unsigned irqs_get_and_restore(unsigned val)
{
	unsigned _cpsr;

	_cpsr = __get_cpsr();
	__set_cpsr((_cpsr & ~IRQ_MASK) | (val & IRQ_MASK));
	return _cpsr;
}

unsigned irqs_get_and_enable(void)
{
	unsigned _cpsr;

	_cpsr = __get_cpsr();
	__set_cpsr(_cpsr & ~IRQ_MASK);
	return _cpsr;
}

void irqs_enable(void)
{
    unsigned _cpsr;

	_cpsr = __get_cpsr();
	__set_cpsr(_cpsr & ~IRQ_MASK);
}

unsigned fiqs_get_and_disable(void)
{
	unsigned _cpsr;

	_cpsr = __get_cpsr();
	__set_cpsr(_cpsr | FIQ_MASK);
	return _cpsr;
}

unsigned fiqs_get_and_restore(unsigned val)
{
	unsigned _cpsr;

	_cpsr = __get_cpsr();
	__set_cpsr((_cpsr & ~FIQ_MASK) | (val & FIQ_MASK));
	return _cpsr;
}

unsigned fiqs_get_and_enable(void)
{
	unsigned _cpsr;

	_cpsr = __get_cpsr();
	__set_cpsr(_cpsr & ~FIQ_MASK);
	return _cpsr;
}

void fiqs_enable(void)
{
    unsigned _cpsr;

	_cpsr = __get_cpsr();
	__set_cpsr(_cpsr & ~FIQ_MASK);
}
