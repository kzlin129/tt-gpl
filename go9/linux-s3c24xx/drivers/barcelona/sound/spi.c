#include <linux/spinlock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/hardware.h>
#include <barcelona/gopins.h>
#include "spi.h"

static spinlock_t spi_lock = SPIN_LOCK_UNLOCKED;

static void spi_send(uint16_t data)
{
	int i;

	spin_lock(&spi_lock);
	/* Set CSB pin to low. */
	IO_Deactivate(L3MODE);

	for (i = 0; i < 16; i++) {
		/* Set SCLK pin to low. */
		IO_Deactivate(L3CLOCK);

		/* Bit 15 high? */
		if (data & 0x8000)
			IO_Activate(L3DATA);
		else
			IO_Deactivate(L3DATA);
		IO_Activate(L3CLOCK);
		data <<= 1;
	}
	/* Set CSB pin to high. */
	IO_Activate(L3MODE);
	spin_unlock(&spi_lock);
}

void spi_write_register(uint16_t reg, uint16_t data)
{
	spi_send((reg << 9) | data);
}
