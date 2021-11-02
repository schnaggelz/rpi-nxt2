#ifndef __IRQS_H__
#define __IRQS_H__

#define enable_all_interrupts()  _irqs_enable()
#define disable_all_interrupts() _irqs_disable()
#define enter_critical_section() _irqs_enable()
#define leave_critical_section() _irqs_disable()

/* ASM implementations. */
extern int _irqs_disable(void);
extern int _fiqs_disable(void);
extern void _irqs_enable(void);
extern void _fiqs_enable(void);

/* C implementations. */
unsigned irqs_get_and_disable(void);
unsigned fiqs_get_and_disable(void);
unsigned irqs_get_and_restore(unsigned val);
unsigned fiqs_get_and_restore(unsigned val);
unsigned irqs_get_and_enable(void);
unsigned fiqs_get_and_enable(void);
void irqs_enable(void);
void fiqs_enable(void);

#endif /*__IRQS_H__*/
