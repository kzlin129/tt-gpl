#ifndef __PLAT_BCM476X__
#define __PLAT_BCM476X__

void bcm476x_map_io   (void);
void bcm476x_init_irq (void);

void bcm476x_set_pci_emu_devid (int, int);

extern struct sys_timer bcm476x_timer;

#endif
