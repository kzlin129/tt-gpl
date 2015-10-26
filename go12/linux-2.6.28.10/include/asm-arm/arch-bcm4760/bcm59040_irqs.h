/*****************************************************************************
* Copyright 2006 - 2010 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/

/**
 * @file bcm59040_irqs.h
 * @brief Virtual IRQs for BCM59040 PMU
 *
 * This file is released under the GPLv2
 *
 */

#ifndef _BCM59040_IRQS_H_
#define _BCM59040_IRQS_H_

/*
 * Included in this interrupt are:
 *
 *     - PONKEY Rising (after debounce time)
 *     - PONKEY Falling (after debounce time)
 *     - PONKEY Hold (PONKEY held low for preset time
 *     - PONKEY System Standby (PONKEY held low for preset time
 *     - PONKEY Restart (PONKEY held low for preset time
 *     - PONKEY Restart On. Triggers after restart has completed
 *       HOSTON is entered again.
 */
#define BCM59040_IRQ_PONKEY				IRQ_BCM59XXX_FIRST

/*
 * Included in this interrupt is Hibernate mode interrupt.
 */
#define BCM59040_IRQ_INVALID_HIB_MODE	IRQ_BCM59XXX_FIRST+1

/*
 * Included in this interrupt is Chip too warm interrupt.
 */
#define BCM59040_IRQ_PMU_TOO_WARM		IRQ_BCM59XXX_FIRST+2

/*
 * Included in this interrupt are:
 *
 *     - Wall Charger Insertion
 *     - Wall Charger Removal
 *     - Wall Charger Over-voltage
 *     - End of Charge
 *     - USB Charger Insertion
 *     - USB Charger Removal
 *     - USB Charger Over-voltage
 *     - Charger Detect Done
 *
 *     - Charger not ok (shorted to ground)
 *     - Charger watchdog alarm
 *     - VBUS voltage collapse
 *     - Charging error disappearance
 *     - Charger watchdog alarm expiration
 *     - Ideal diode overcurrent
 */
#define BCM59040_IRQ_CHARGER			IRQ_BCM59XXX_FIRST+3

/*
 * Included in this interrupt are:
 *
 *      - LDO1 overcurrent interrupt
 *      - LDO2 overcurrent interrupt
 *      - LDO3 overcurrent interrupt
 *      - LDO4 overcurrent interrupt
 *      - LDO5 overcurrent interrupt
 *      - LDO6 overcurrent interrupt
 *      - IOSR overcurrent interrupt
 *      - CSR overcurrent interrupt
 *      - IOSR overvoltage interrupt
 *      - CSR overvoltage interrupt
 */
#define BCM59040_IRQ_REGULATORS			IRQ_BCM59XXX_FIRST+4

/*
 *
 * Included in this interrupt are:
 *
 *     - temperature fault
 *     - temperature too low
 *     - temperature too high
 *     - battery removed
 *     - battery over voltage
 *     - battery inserted
 *     - battery low
 *     - battery very low
 *     - backup battery low interrupt
 *     - FGC comparator interrupt
 */
#define BCM59040_IRQ_BATT				IRQ_BCM59XXX_FIRST+5

/*
 * Included in this interrupt are:
 *
 *      - RTC needs adjustment as power-up from totally no power
 *      - RTC periodic 1-second interrupt
 *      - RTC periodic 60-second interrupt
 */
#define BCM59040_IRQ_RTC				IRQ_BCM59XXX_FIRST+6

/*
 * Included in this interrupt are:
 *
 *      - RTC alarm 1 interrupt
 */
#define BCM59040_IRQ_RTC_ALARM				IRQ_BCM59XXX_FIRST+7

/*
 * Interrupts included in this interrupt are:
 *
 *     - VBUS valid comparator falling edge
 *     - Device A session valid falling edge
 *     - Device B session ended falling edge
 *     - ID insertion (IDOUT high to low transition)
 *     - VBUS valid comparator rising edge
 *     - Device A session valid rising edge
 *     - Device B session ended rising edge
 *     - ID removal (IDOUT low to high transition)
 *     - resume VBUS
 *     - IDIN value change
 */
#define BCM59040_IRQ_USB				IRQ_BCM59XXX_FIRST+8

/* Interrupts included in this interrupt are:
 *
 *     - ADC conversion completed
 *     - ADC data end
 *     - ADC continuous conversion failed
 *     - ADC asynchronous conversion off
 *     - ADC asynchronous conversion failed
 */
#define BCM59040_IRQ_ADC				IRQ_BCM59XXX_FIRST+9

/*
 * These may look like duplications but they both need to
 * be defined for the whole thing to work correctly.
 */

#define BCM59040_NUM_IRQS	10

#endif

