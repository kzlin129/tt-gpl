#ifndef ASM_ARM_KMEMCHECK
#define ASM_ARM_KMEMCHECK
bool
kmemcheck_fault(struct pt_regs *regs, unsigned long address, unsigned long error_code,pte_t *pte);
#endif
