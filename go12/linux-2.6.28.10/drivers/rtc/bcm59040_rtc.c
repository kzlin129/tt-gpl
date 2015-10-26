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




/*
 *****************************************************************************
 *
 *  bcm59040_rtc.c
 *
 *  PURPOSE:
 *
 *     This implements the real time clock interface for the BCM59040 chips.
 *
 *  NOTES:
 *
 ****************************************************************************
 */


/* ---- Include Files ---------------------------------------------------- */
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/sysctl.h>
#include <linux/rtc.h>
#include <linux/io.h>

#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/broadcom/bcm_major.h>
#include <linux/broadcom/rtc.h>
#include <linux/broadcom/pmu_bcm59040.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/pmu_device.h>
#include <linux/broadcom/pmu_bcm59040.h>

#include <asm/arch/bcm59040_irqs.h>

#include "rtc.h"

#include <asm/arch/rtcHw.h>

/*
 * Structure and macro definitions.
 */

#define BCM59040_RTC_DRIVER			"bcm59040_rtc"
#define RTC_BCM59040_MODULE_DESC     "Broadcom BCM59040 RTC Driver"
#define RTC_BCM59040_MODULE_VERSION  "2.0"

#define BCM59040_DEVNAME				BCM59040_RTC_DRIVER
#define PFX BCM59040_DEVNAME			": "

/* Debug logging */
#ifdef DEBUG
#undef DEBUG
#endif
//#define DEBUG 1

#define DBG_ERROR	0x01
#define DBG_INFO	0x02
#define DBG_TRACE	0x04
#define DBG_TRACE2	0x08
#define DBG_DATA	0x10
#define DBG_DATA2	0x20

#ifdef DEBUG
//#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO | DBG_TRACE | DBG_DATA)
#define DBG_DEFAULT_LEVEL	(DBG_ERROR | DBG_INFO)

static int	gLevel = DBG_DEFAULT_LEVEL;

#	define PMU_DEBUG(level,x) {if (level & gLevel) printk x;}
#else
#	define PMU_DEBUG(level,x)
#endif

#define RTC_SET_PMU 0x1239
#define RTC_GET_PMU 0x123A

#define LEAPS_THRU_END_OF(y) ((y)/4 - (y)/100 + (y)/400)
#define LEAP_YEAR(year) ((!(year % 4) && (year % 100)) || !(year % 400))

typedef struct bcm59040_rtc_config_struct
{
	struct pmu_client		*pclient;
	struct work_struct		work;
	struct workqueue_struct		*workqueue;
	struct rtc_device		*rtc;
	int				freq4760;
} bcm59040_rtc_config_t;

/*
 * Local variables.
 */

static int32_t safe_rtc_offset=0;

static int32_t safe_rtc=0;

/*
 * Bits for RTC_SECURE register
 */
#define BCM59040_SAFE_RTC_BIT_RTCVALID		0x01
#define BCM59040_SAFE_RTC_BIT_OFFSETSIGN	0x02

/*
 * Local function declarations.
 */

static DEFINE_SPINLOCK(bcm59040_476x_rtc_lock);
static void bcm59040_476x_rtc_setaie(struct device *dev, int to);

static int bcm59040_476x_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm);

static int bcm59040_476x_rtc_setpie(struct device *dev, int enabled)
{
	int				rc;
	struct pmu_client		*pclient=to_pmu_client(dev);

	spin_lock_irq(&bcm59040_476x_rtc_lock);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "From:%s enabled:%x\n",__FUNCTION__,enabled));
	if(enabled) {
		// clear interrupts that might be present
		pmu_bus_set_bit(pclient, BCM59040_REG_VINT1, BCM59040_VINT1_RTC_BIT);
		rc = pmu_bus_clear_bit(pclient, BCM59040_REG_VINT1M, BCM59040_VINT1_RTC_BIT);
		if(rc < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error clearing interrupt mask.\n"));
			spin_unlock_irq(&bcm59040_476x_rtc_lock);
			return -EINVAL;
		}
	} else {
		if (!signal_pending(current)) {
			rc = pmu_bus_set_bit(pclient, BCM59040_REG_VINT1M, BCM59040_VINT1_RTC_BIT);
			if(rc < 0) {
				PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error setting interrupt mask.\n"));
				spin_unlock_irq(&bcm59040_476x_rtc_lock);
				return -EINVAL;
			}
		}
	}

	spin_unlock_irq(&bcm59040_476x_rtc_lock);

	return 0;
}

static int bcm59040_476x_rtc_setfreq(struct device *dev, int freq)
{
	bcm59040_rtc_config_t	*rtc_config;
	struct pmu_client		*pclient = to_pmu_client(dev);

	rtc_config = pmu_get_drvdata(pclient);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "setFREQ:%d\n",freq));
	rtc_config->freq4760 = 1;			//support only 1s

	return 0;
}

static int bcm59040_476x_rtc_getfreq(struct pmu_client *pclient, int *freq)
{
	bcm59040_rtc_config_t		*rtc_config;

	rtc_config = pmu_get_drvdata(pclient);

	*freq = rtc_config->freq4760;
	return 0;
}

static int bcm59040_resetrtc(struct device *dev)
{
	u8						rtcReg[BCM59040_NUM_RTC_REG], data;
	int						rc;
	struct pmu_client		*pclient=to_pmu_client(dev);
	unsigned long			rtc_timeval, timeval;
	struct rtc_time			rtc_tm;

	if((rc = pmu_bus_seqread(pclient,
							 BCM59040_REG_RTCSC,
							 &rtcReg,
							 BCM59040_NUM_RTC_REG)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSC registers.\n"));
		return -EINVAL;
	}

	rtc_tm.tm_sec = rtcReg[0];
	rtc_tm.tm_min = rtcReg[1];
	rtc_tm.tm_hour = rtcReg[2];
	rtc_tm.tm_mday = rtcReg[4];
	rtc_tm.tm_mon = rtcReg[5] - 1;
	rtc_tm.tm_year = rtcReg[6] + 100;

	rtc_tm_to_time(&rtc_tm, &rtc_timeval);
	rtc_timeval += safe_rtc_offset;
	rtc_time_to_tm(rtc_timeval, &rtc_tm);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "adjusted time %u-%02u-%02u %02u:%02u:%02u\n",
						  rtc_tm.tm_year+1900,
						  rtc_tm.tm_mon+1,
						  rtc_tm.tm_mday,
						  rtc_tm.tm_hour,
						  rtc_tm.tm_min,
						  rtc_tm.tm_sec));

	rtcReg[0] = rtc_tm.tm_sec;
	rtcReg[1] = rtc_tm.tm_min;
	rtcReg[2] = rtc_tm.tm_hour;
	rtcReg[3] = rtc_tm.tm_wday;
	rtcReg[4] = rtc_tm.tm_mday;
	rtcReg[5] = rtc_tm.tm_mon + 1;
	rtcReg[6] = rtc_tm.tm_year - 100;

	if((rc = pmu_bus_seqwrite(pclient,
							BCM59040_REG_RTCSC,
							&rtcReg,
							BCM59040_NUM_RTC_REG)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error Writing BCM59040_REG_RTCSC registers.\n"));
		return -EINVAL;
	}

	/*
		* Set sign bit but clear valid bit. This is used to indicate that we've set
		* an actual but invalid time into the PMU RTC. When the GPS time is set into
		* the PMU RTC registers we'll detect this condition and preserve that time
		* first and then calculate the offset of the previous RTC time against the
		* GPS provided "safe" time value and store it into the PMU registers we
		* use for storing the offset. We'll also then set the sign and safe bits
		* correctly.
		*/

	/*
		* Force Offset to zero to simplify remainder of coding.
		* Also set bit combination indicating Hardware RTC is set but
		* not with a GPS Safe value.
		*/

	timeval = 0;
	if((rc = pmu_bus_seqwrite(pclient,
								BCM59040_REG_FGGNRL1,
								&timeval,
								3)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGGNRL1 registers.\n"));
		return -EINVAL;
	}

	data = BCM59040_SAFE_RTC_BIT_OFFSETSIGN;

	if((rc = pmu_bus_seqwrite(pclient,
								BCM59040_REG_RTCSECURE,
								&data,
								1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_RTCSECURE registers.\n"));
		return -EINVAL;
	}

	safe_rtc_offset = 0;
	safe_rtc = 0;

	return 0;
}

/* Time read/write */

static int bcm59040_getrtc(struct device *dev, struct rtc_time *rtc_tm)
{
	u8						rtcReg[BCM59040_NUM_RTC_REG];
	int						rc;
	struct pmu_client		*pclient=to_pmu_client(dev);

	if((rc = pmu_bus_seqread(pclient,
							 BCM59040_REG_RTCSC,
							 &rtcReg,
							 BCM59040_NUM_RTC_REG)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSC registers.\n"));
		return -EINVAL;
	}

	rtc_tm->tm_sec = rtcReg[0];
	rtc_tm->tm_min = rtcReg[1];
	rtc_tm->tm_hour = rtcReg[2];
	rtc_tm->tm_mday = rtcReg[4];
	rtc_tm->tm_mon = rtcReg[5] - 1;
	rtc_tm->tm_year = rtcReg[6] + 100;

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "read time %u-%02u-%02u %02u:%02u:%02u\n",
						  rtc_tm->tm_year+1900,
						  rtc_tm->tm_mon+1,
						  rtc_tm->tm_mday,
						  rtc_tm->tm_hour,
						  rtc_tm->tm_min,
						  rtc_tm->tm_sec));

	return 0;
}

static int bcm59040_setrtc(struct device *dev, struct rtc_time *time)
{
	u8						rtcReg[BCM59040_NUM_RTC_REG],
							rtcOldReg[BCM59040_NUM_RTC_REG], secure_rtc;
	int						rc, timeval_sign;
	struct rtc_time 		rtc_tm;
	unsigned long			rtc_timeval, timeval;
	struct pmu_client		*pclient=to_pmu_client(dev);

	rtcReg[0] = time->tm_sec;
	rtcReg[1] = time->tm_min;
	rtcReg[2] = time->tm_hour;
	rtcReg[3] = time->tm_wday;
	rtcReg[4] = time->tm_mday;
	rtcReg[5] = time->tm_mon + 1;
	rtcReg[6] = time->tm_year - 100;

	/*
	 * Get current values in RTC_SECURE register and determine if
	 * a time has been previously set but is not "safe". This condition
	 * occurs when the Linux RTC set function is called but we don't
	 * have a valid "safe" time from the GPS yet. We save it into the
	 * PMU RTC registers but we don't indicate it is safe.
	 */

	if((rc = pmu_bus_seqread(pclient,
								BCM59040_REG_RTCSECURE,
								&secure_rtc,
								1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSECURE registers.\n"));
		return -EINVAL;
	}

	if(secure_rtc == BCM59040_SAFE_RTC_BIT_OFFSETSIGN) {
		if((rc = pmu_bus_seqread(pclient,
								 BCM59040_REG_RTCSC,
								 &rtcOldReg,
								 BCM59040_NUM_RTC_REG)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSC registers.\n"));
			return -EINVAL;
		}

		rtc_tm.tm_sec = rtcOldReg[0];
		rtc_tm.tm_min = rtcOldReg[1];
		rtc_tm.tm_hour = rtcOldReg[2];
		rtc_tm.tm_mday = rtcOldReg[4];
		rtc_tm.tm_mon = rtcOldReg[5] - 1;
		rtc_tm.tm_year = rtcOldReg[6] + 100;

		PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "read time %u-%02u-%02u %02u:%02u:%02u\n",
							rtc_tm.tm_year+1900,
							rtc_tm.tm_mon+1,
							rtc_tm.tm_mday,
							rtc_tm.tm_hour,
							rtc_tm.tm_min,
							rtc_tm.tm_sec));

		if((rc = pmu_bus_seqwrite(pclient,
								BCM59040_REG_RTCSC,
								&rtcReg,
								BCM59040_NUM_RTC_REG)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error Writing BCM59040_REG_RTCSC registers.\n"));
			return -EINVAL;
		}

		rtc_tm_to_time(&rtc_tm, &rtc_timeval);
		rtc_tm_to_time(time, &timeval);

		PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "rtc_timeval=%lu, timeval=%lu.\n",
							 rtc_timeval, timeval));

		if(timeval < rtc_timeval) {
			timeval = rtc_timeval-timeval;
			timeval_sign = 0;
			safe_rtc_offset = timeval;
		} else {
			timeval = timeval-rtc_timeval;
			timeval_sign = 1;
			safe_rtc_offset = -timeval;
		}

		PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "timeval=%lu, timeval_sign=%d.\n",
							 timeval, timeval_sign));

		if(timeval > 0x00FFFFFF)
			timeval = 0x00FFFFFF;

		if((rc = pmu_bus_seqwrite(pclient,
								  BCM59040_REG_FGGNRL1,
								  &timeval,
								  3)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGGNRL1 registers.\n"));
			return -EINVAL;
		}

		/*
		 * Set sign bit correctly in RTC_SECURE for this offset value.
		 */

		secure_rtc &= ~BCM59040_SAFE_RTC_BIT_OFFSETSIGN;
		if(timeval_sign)
			secure_rtc |= BCM59040_SAFE_RTC_BIT_OFFSETSIGN;
		secure_rtc |= BCM59040_SAFE_RTC_BIT_RTCVALID;

		if((rc = pmu_bus_seqwrite(pclient,
								  BCM59040_REG_RTCSECURE,
								  &secure_rtc,
								  1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_RTCSECURE registers.\n"));
			return -EINVAL;
		}
	} else {
		if((rc = pmu_bus_seqwrite(pclient,
								BCM59040_REG_RTCSC,
								&rtcReg,
								BCM59040_NUM_RTC_REG)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error Writing BCM59040_REG_RTCSC registers.\n"));
			return -EINVAL;
		}

		/*
			* No offset set yet so just set valid in SECURE_RTC register.
			*/

		safe_rtc_offset = 0;
		secure_rtc = BCM59040_SAFE_RTC_BIT_RTCVALID;

		if((rc = pmu_bus_seqwrite(pclient,
									BCM59040_REG_RTCSECURE,
									&secure_rtc,
									1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_RTCSECURE registers.\n"));
			return -EINVAL;
		}
	}

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Written time %u-%02u-%02u %02u:%02u:%02u\n",
						time->tm_year+1900,
						time->tm_mon+1,
						time->tm_mday,
						time->tm_hour,
						time->tm_min,
						time->tm_sec));

	safe_rtc = 1;

	return 0;
}

/* Time read/write */
static int bcm59040_476x_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	u8						rtcReg[BCM59040_NUM_RTC_REG];
	int						rc;
	struct pmu_client		*pclient=to_pmu_client(dev);
	unsigned long			rtc_timeval;

	if((rc = pmu_bus_seqread(pclient,
							 BCM59040_REG_RTCSC,
							 &rtcReg,
							 BCM59040_NUM_RTC_REG)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSC registers.\n"));
		return -EINVAL;
	}

	rtc_tm->tm_sec = rtcReg[0];
	rtc_tm->tm_min = rtcReg[1];
	rtc_tm->tm_hour = rtcReg[2];
	rtc_tm->tm_mday = rtcReg[4];
	rtc_tm->tm_mon = rtcReg[5] - 1;
	rtc_tm->tm_year = rtcReg[6] + 100;

	rtc_tm_to_time(rtc_tm, &rtc_timeval);
	rtc_timeval += safe_rtc_offset;
	rtc_time_to_tm(rtc_timeval, rtc_tm);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "read time %u-%02u-%02u %02u:%02u:%02u\n",
						  rtc_tm->tm_year+1900,
						  rtc_tm->tm_mon+1,
						  rtc_tm->tm_mday,
						  rtc_tm->tm_hour,
						  rtc_tm->tm_min,
						  rtc_tm->tm_sec));

	return 0;
}

static int bcm59040_476x_rtc_settime(struct device *dev, struct rtc_time *time)
{
	u8						rtcReg[BCM59040_NUM_RTC_REG], data;
	struct rtc_time 		rtc_tm;
	int						rc, timeval_sign;
	struct pmu_client		*pclient=to_pmu_client(dev);
	unsigned long			rtc_timeval, timeval;

	rtcReg[0] = time->tm_sec;
	rtcReg[1] = time->tm_min;
	rtcReg[2] = time->tm_hour;
	rtcReg[3] = time->tm_wday;
	rtcReg[4] = time->tm_mday;
	rtcReg[5] = time->tm_mon + 1;
	rtcReg[6] = time->tm_year - 100;

	if(safe_rtc)
	{

		if((rc = pmu_bus_seqread(pclient,
								BCM59040_REG_RTCSC,
								&rtcReg,
								BCM59040_NUM_RTC_REG)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSC registers.\n"));
			return -EINVAL;
		}

		rtc_tm.tm_sec = rtcReg[0];
		rtc_tm.tm_min = rtcReg[1];
		rtc_tm.tm_hour = rtcReg[2];
		rtc_tm.tm_mday = rtcReg[4];
		rtc_tm.tm_mon = rtcReg[5] - 1;
		rtc_tm.tm_year = rtcReg[6] + 100;

		rtc_tm_to_time(&rtc_tm, &rtc_timeval);
		rtc_tm_to_time(time, &timeval);

		PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "rtc_timeval=%lu, timeval=%lu.\n",
							 rtc_timeval, timeval));

		if(timeval < rtc_timeval) {
			timeval = rtc_timeval-timeval;
			timeval_sign = 1;
			safe_rtc_offset = -timeval;
		} else {
			timeval = timeval-rtc_timeval;
			timeval_sign = 0;
			safe_rtc_offset = timeval;
		}

		PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "timeval=%lu, timeval_sign=%d.\n",
							 timeval, timeval_sign));

		PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "timeval=%lu, timeval_sign=%d.\n",
							 timeval, timeval_sign));

		if(timeval > 0x00FFFFFF) {
			safe_rtc_offset = 0;
			return -EINVAL;
		}

		if((rc = pmu_bus_seqwrite(pclient,
								  BCM59040_REG_FGGNRL1,
								  &timeval,
								  3)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGGNRL1 registers.\n"));
			return -EINVAL;
		}

		/*
		 * Set sign bit correctly in RTC_SECURE for this offset value.
		 */

		if((rc = pmu_bus_seqread(pclient,
								 BCM59040_REG_RTCSECURE,
								 &data,
								 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSECURE registers.\n"));
			return -EINVAL;
		}

		data &= ~BCM59040_SAFE_RTC_BIT_OFFSETSIGN;
		if(timeval_sign)
			data |= BCM59040_SAFE_RTC_BIT_OFFSETSIGN;

		if((rc = pmu_bus_seqwrite(pclient,
								  BCM59040_REG_RTCSECURE,
								  &data,
								  1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_RTCSECURE registers.\n"));
			return -EINVAL;
		}
	}
	else
	{
		if((rc = pmu_bus_seqwrite(pclient,
								BCM59040_REG_RTCSC,
								&rtcReg,
								BCM59040_NUM_RTC_REG)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_RTCSC registers.\n"));
			return -EINVAL;
		}

		/*
		 * Set sign bit but clear valid bit. This is used to indicate that we've set
		 * an actual but invalid time into the PMU RTC. When the GPS time is set into
		 * the PMU RTC registers we'll detect this condition and preserve that time
		 * first and then calculate the offset of the previous RTC time against the
		 * GPS provided "safe" time value and store it into the PMU registers we
		 * use for storing the offset. We'll also then set the sign and safe bits
		 * correctly.
		 */

		if((rc = pmu_bus_seqread(pclient,
								 BCM59040_REG_RTCSECURE,
								 &data,
								 1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSECURE registers.\n"));
			return -EINVAL;
		}

		/*
		 * Force Offset to zero to simplify remainder of coding.
		 * Also set bit combination indicating Hardware RTC is set but
 		 * not with a GPS Safe value.
		 */

		timeval = 0;
		if((rc = pmu_bus_seqwrite(pclient,
								  BCM59040_REG_FGGNRL1,
								  &timeval,
								  3)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGGNRL1 registers.\n"));
			return -EINVAL;
		}

		data = BCM59040_SAFE_RTC_BIT_OFFSETSIGN;

		if((rc = pmu_bus_seqwrite(pclient,
								  BCM59040_REG_RTCSECURE,
								  &data,
								  1)) < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_RTCSECURE registers.\n"));
			return -EINVAL;
		}
	}

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Written time %u-%02u-%02u %02u:%02u:%02u\n",
						  time->tm_year+1900,
						  time->tm_mon+1,
						  time->tm_mday,
						  time->tm_hour,
						  time->tm_min,
						  time->tm_sec));

	return 0;
}

static int bcm59040_476x_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time		*alm_tm = &alrm->time;
	u8			rtcReg[BCM59040_NUM_RTC_REG], intReg;
	int			rc;
	struct pmu_client	*pclient=to_pmu_client(dev);
	unsigned long		alm_timeval;

	if((rc = pmu_bus_seqread(pclient,
				BCM59040_REG_RTCSC_A1,
				&rtcReg,
				BCM59040_NUM_RTC_REG)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSC_A1 registers.\n"));
		return -EINVAL;
	}
	if((rc = pmu_bus_seqread(pclient,
				BCM59040_REG_VINT1M,
				&intReg,
				1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading interrupt mask registers.\n"));
		return -EINVAL;
	}

	alm_tm->tm_sec = rtcReg[0];
	alm_tm->tm_min = rtcReg[1];
	alm_tm->tm_hour = rtcReg[2];
	alm_tm->tm_mday = rtcReg[4];
	alm_tm->tm_mon = rtcReg[5] - 1;
	alm_tm->tm_year = rtcReg[6] + 100;

	alrm->enabled = !(intReg & (1 << BCM59040_VINT1_RTC_ALARM_BIT));

	rtc_tm_to_time(alm_tm, &alm_timeval);
	alm_timeval += safe_rtc_offset;
	rtc_time_to_tm(alm_timeval, alm_tm);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Getalarm time %u-%02u-%02u %02u:%02u:%02u\n",
						  alm_tm->tm_year+1900,
						  alm_tm->tm_mon+1,
						  alm_tm->tm_mday,
						  alm_tm->tm_hour,
						  alm_tm->tm_min,
						  alm_tm->tm_sec));

	return 0;
}

static int bcm59040_476x_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time		*alm_tm = &alrm->time;
	u8			rtcReg[BCM59040_NUM_RTC_REG];
	int			rc;
	struct pmu_client	*pclient = to_pmu_client(dev);
	unsigned long		alm_timeval;

	// disable previous alarms
	bcm59040_476x_rtc_setaie(dev, 0);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "Setalarm time 0x%02x.0x%02x.0x%02x 0x%02x/0x%02x/0x%02x enabled %d\n",
						  alm_tm->tm_year,
						  alm_tm->tm_mon,
						  alm_tm->tm_mday,
						  alm_tm->tm_hour,
						  alm_tm->tm_min,
						  alm_tm->tm_sec,
						  alrm->enabled));

	rtcReg[0] = alm_tm->tm_sec;
	rtcReg[1] = alm_tm->tm_min;
	rtcReg[2] = alm_tm->tm_hour;
	rtcReg[3] = 0; // don't want to wake up on a specific weekday
	rtcReg[4] = alm_tm->tm_mday;
	rtcReg[5] = alm_tm->tm_mon + 1;
	rtcReg[6] = alm_tm->tm_year - 100;

	rtc_tm_to_time(alm_tm, &alm_timeval);
	alm_timeval -= safe_rtc_offset;
	rtc_time_to_tm(alm_timeval, alm_tm);

	printk(KERN_INFO "Setting alarm: %d, %d, %d, %d, %d, %d, %d\n",
		rtcReg[0], rtcReg[1], rtcReg[2], rtcReg[3], rtcReg[4], rtcReg[5], rtcReg[6]);

	if((rc = pmu_bus_seqwrite(pclient,
			BCM59040_REG_RTCSC_A1,
			&rtcReg,
			BCM59040_NUM_RTC_REG)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_INFO PFX "error writing BCM59040_REG_RTCSC_A1 registers.\n"));
		return -EINVAL;
	}

	if (alrm->enabled)
		bcm59040_476x_rtc_setaie(dev, 1);

	return 0;
}

static void bcm59040_476x_rtc_setaie(struct device *dev, int enabled)
{
	int			rc;
	struct pmu_client	*pclient=to_pmu_client(dev);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "%s: aie=%d\n", __func__, enabled));

	spin_lock_irq(&bcm59040_476x_rtc_lock);

	if(enabled) {
		// clear interrupts that might be present
		pmu_bus_set_bit(pclient, BCM59040_REG_VINT1, BCM59040_VINT1_RTC_ALARM_BIT);
		rc = pmu_bus_clear_bit(pclient, BCM59040_REG_VINT1M, BCM59040_VINT1_RTC_ALARM_BIT);
		if(rc < 0) {
			PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error clearing interrupt mask.\n"));
			spin_unlock_irq(&bcm59040_476x_rtc_lock);
			return;
		}
	} else {
		if (!signal_pending(current)) {
			rc = pmu_bus_set_bit(pclient, BCM59040_REG_VINT1M, BCM59040_VINT1_RTC_ALARM_BIT);
			if(rc < 0) {
				PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error clearing interrupt mask.\n"));
				spin_unlock_irq(&bcm59040_476x_rtc_lock);
				return;
			}
		}
	}

	spin_unlock_irq( &bcm59040_476x_rtc_lock );
}

static int bcm59040_476x_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct pmu_client		*pclient=to_pmu_client(dev);

	switch(cmd) {
		case RTC_AIE_OFF:
			bcm59040_476x_rtc_setaie(dev, 0);
			return 0;

		case RTC_AIE_ON:
			bcm59040_476x_rtc_setaie(dev, 1);
			return 0;

		case RTC_PIE_OFF:
			bcm59040_476x_rtc_setpie(dev, 0);
			return 0;

		case RTC_PIE_ON:
			bcm59040_476x_rtc_setpie(dev, 1);
			return 0;

		case RTC_IRQP_READ:
		{
			int freq;
			int ret = bcm59040_476x_rtc_getfreq(pclient, &freq);

			if(ret != 0) {
				return ret;
			}
			return put_user(freq, (unsigned long *)arg);
		}

		case RTC_IRQP_SET:
			return bcm59040_476x_rtc_setfreq(&pclient->dev, (int)arg);

		case RTC_IOCTL_IS_RTC_GPS_TIME:
			if(safe_rtc)
				return put_user(1, (int *)arg);
			else
				return put_user(0, (int *)arg);

		case RTC_IOCTL_GET_HW_RTC:
		{
			struct rtc_time		ret_rtcval;

			if(bcm59040_getrtc(dev, &ret_rtcval)) {
				return -EIO;
			} else {
				if(copy_to_user((void __user *)arg,
									&ret_rtcval,
									sizeof(ret_rtcval)))
					return -EFAULT;
				else
					return 0;
			}

		}

		case RTC_IOCTL_GET_GPS_TIME:
		{
			struct rtc_time		ret_rtcval;

			if(safe_rtc) {
				if(bcm59040_getrtc(dev, &ret_rtcval)) {
					return -EIO;
				} else {
					if(copy_to_user((void __user *)arg,
										&ret_rtcval,
										sizeof(ret_rtcval)))
						return -EFAULT;
					else
						return 0;
				}
			} else {
				return -EINVAL;
			}
		}

		case RTC_IOCTL_RESET_GPS_TIME:
		{
			if(bcm59040_resetrtc(dev))
				return -EIO;

			return 0;
		}

		case RTC_IOCTL_SET_GPS_TIME:
		{
			struct rtc_time		in_rtcval;

			if(copy_from_user(&in_rtcval,
							  (void __user *)arg,
							  sizeof(in_rtcval))) {
				return -EFAULT;
			} else {
				if(bcm59040_setrtc(dev, &in_rtcval))
					return -EIO;
			}

			return 0;
		}

	}

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "From:%s cmd:%x\n",__FUNCTION__,cmd));
	return -ENOIOCTLCMD;
}

static irqreturn_t bcm59040_rtc_isr(int irq, void *dev_id)
{
	bcm59040_rtc_config_t		*rtc_config = dev_id;
	u8				data;
	int				rc;

	if((rc = pmu_bus_seqread(rtc_config->pclient,
					BCM59040_REG_INT5,
					&data,
					1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_INT5 register.\n"));
	}
	else {
		PMU_DEBUG(DBG_DATA, (KERN_INFO PFX "int status 0x%x.\n", data));
		if(data & BCM59040_INT5_RTCA1)
		{
			rtc_update_irq(rtc_config->rtc, 1, RTC_AF|RTC_IRQF);
		}
		if(data & (BCM59040_INT5_RTC1S|BCM59040_INT5_RTC60S))
		{
			rtc_update_irq(rtc_config->rtc, 1, RTC_PF|RTC_IRQF);
		}
	}

	data = (1 << BCM59040_VINT1_RTC_BIT) | (1 << BCM59040_VINT1_RTC_ALARM_BIT);
	pmu_bus_seqwrite(rtc_config->pclient, BCM59040_REG_VINT1, &data, 1 );

	return IRQ_HANDLED;
}

static void bcm59040_476x_rtc_release(struct device *dev)
{
	struct pmu_client		*pclient=to_pmu_client(dev);

	if(!signal_pending(current)) {
	   PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "From:%s\n",__FUNCTION__));
	   bcm59040_476x_rtc_setpie(&pclient->dev, 0);
   }
}

static const struct rtc_class_ops bcm59040_476x_rtcops = {
	.open = NULL,
	.release = bcm59040_476x_rtc_release,
	.ioctl = bcm59040_476x_rtc_ioctl,
	.read_time = bcm59040_476x_rtc_gettime,
	.set_time = bcm59040_476x_rtc_settime,
	.read_alarm = bcm59040_476x_rtc_getalarm,
	.set_alarm = bcm59040_476x_rtc_setalarm,
	.proc = NULL,
	.set_mmss = NULL,
	.irq_set_state = bcm59040_476x_rtc_setpie,
	.irq_set_freq = bcm59040_476x_rtc_setfreq,
	.read_callback = NULL,
};

static int bcm59040_rtc_probe(struct pmu_client *pclient,
		const struct pmu_device_id *id)
{
	int			rc;
	bcm59040_rtc_config_t	*rtc_config;
	u8			data;
	unsigned long		timeval=0;

	if((rtc_config = kzalloc(sizeof(*rtc_config), GFP_KERNEL)) == NULL) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "Out of memory allocating rtc_config\n"));
		return -ENOMEM;
	}
	rtc_config->pclient = pclient;
	pmu_set_drvdata(pclient, rtc_config);

	PMU_DEBUG(DBG_TRACE, (KERN_INFO PFX "%s: probe=%p\n", __func__, pclient));

	device_init_wakeup(&pclient->dev, 1);
	rtc_config->rtc = rtc_device_register("bcm59040_rtc",
			&pclient->dev,
			&bcm59040_476x_rtcops,
			THIS_MODULE);

	if(IS_ERR(rtc_config->rtc)) {
		rc = PTR_ERR(rtc_config->rtc);
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "cannot attach rtc\n"));
		rtc_device_unregister(rtc_config->rtc);
		pmu_set_drvdata(pclient, NULL);
		return rc;
	}

	rtc_config->freq4760 = 1;			//1s

	rc = request_irq(BCM59040_IRQ_RTC,
			bcm59040_rtc_isr,
			0,
			"bcm59040_rtc",
			rtc_config);
	if (rc < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_rtc_probe failed to attach interrupt, rc = %d\n", rc));
		kfree(rtc_config);
		pmu_set_drvdata(pclient, NULL);
		return rc;
	}

	rc = request_irq(BCM59040_IRQ_RTC_ALARM,
			bcm59040_rtc_isr,
			0,
			"bcm59040_rtc",
			rtc_config);
	if (rc < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "bcm59040_rtc_probe failed to attach alarm interrupt, rc = %d\n", rc));
		free_irq(BCM59040_IRQ_RTC, rtc_config);
		kfree(rtc_config);
		pmu_set_drvdata(pclient, NULL);
		return rc;
	}
	// there's no alarm pending
	disable_irq(BCM59040_IRQ_RTC_ALARM);

	bcm59040_476x_rtc_setfreq(&pclient->dev, (int)rtc_config->freq4760);

	/*
	 * Initialize RTC offset correctly if RTC has been
	 * set securely.
	 */

	if((rc = pmu_bus_seqread(pclient,
			BCM59040_REG_FGGNRL1,
			&timeval,
			3)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error writing BCM59040_REG_FGGNRL1 registers.\n"));
		free_irq(BCM59040_IRQ_RTC, rtc_config);
		free_irq(BCM59040_IRQ_RTC_ALARM, rtc_config);
		kfree(rtc_config);
		pmu_set_drvdata(pclient, NULL);
		return -EINVAL;
	}

	if((rc = pmu_bus_seqread(pclient,
			BCM59040_REG_RTCSECURE,
			&data,
			1)) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading BCM59040_REG_RTCSECURE registers.\n"));
		free_irq(BCM59040_IRQ_RTC, rtc_config);
		free_irq(BCM59040_IRQ_RTC_ALARM, rtc_config);
		kfree(rtc_config);
		pmu_set_drvdata(pclient, NULL);
		return -EINVAL;
	}

	if(data & BCM59040_SAFE_RTC_BIT_RTCVALID) {
		if(data & BCM59040_SAFE_RTC_BIT_OFFSETSIGN) {
			safe_rtc = 1;
			safe_rtc_offset = -timeval;
		} else {
			safe_rtc = 1;
			safe_rtc_offset = timeval;
		}
	} else {
		safe_rtc = 0;
	}

	return 0;
}

static int bcm59040_rtc_remove(struct pmu_client *pclient)
{
	bcm59040_rtc_config_t	*rtc_config;
	struct device 		*dev = &pclient->dev;

	rtc_config = pmu_get_drvdata(pclient);

	device_init_wakeup(dev, 0);

	bcm59040_476x_rtc_setpie(dev, 0);
	bcm59040_476x_rtc_setaie(dev, 0);
	rtc_device_unregister(rtc_config->rtc);

	free_irq(BCM59040_IRQ_RTC, rtc_config);
	free_irq(BCM59040_IRQ_RTC_ALARM, rtc_config);
	kfree(rtc_config);

	pmu_set_drvdata(pclient, NULL);

	return 0;
}

#ifdef CONFIG_PM

static int bcm59040_rtc_suspend(struct pmu_client *pclient, pm_message_t state)
{
	u8 intReg;

	if(pmu_bus_seqread(pclient,
				BCM59040_REG_VINT1M,
				&intReg,
				1) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading interrupt mask registers.\n"));
		return -EINVAL;
	}

	if (device_may_wakeup(&pclient->dev) && !(intReg & (1 << BCM59040_VINT1_RTC_ALARM_BIT))) {
		PMU_DEBUG(DBG_INFO, (KERN_INFO PFX "Enabling wakeup.\n"));
		enable_irq_wake(BCM59040_IRQ_RTC_ALARM);
	}
	else
		disable_irq(BCM59040_IRQ_RTC_ALARM);

	return 0;
}

static int bcm59040_rtc_resume(struct pmu_client *pclient)
{
	u8 intReg;

	if(pmu_bus_seqread(pclient,
				BCM59040_REG_VINT1M,
				&intReg,
				1) < 0) {
		PMU_DEBUG(DBG_ERROR, (KERN_ERR PFX "error reading interrupt mask registers.\n"));
		return -EINVAL;
	}

	if (device_may_wakeup(&pclient->dev) && !(intReg & (1 << BCM59040_VINT1_RTC_ALARM_BIT)))
		disable_irq_wake(BCM59040_IRQ_RTC_ALARM);
	else
		enable_irq(BCM59040_IRQ_RTC_ALARM);

	return 0;
}

#else

#define bcm59040_rtc_suspend	NULL
#define bcm59040_rtc_resume	NULL
#endif

struct pmu_device_id bcm59040_rtc_idtable[] = {
	{ "rtc" },
	{ }
};

/** @brief BCM59040 RTC driver registration data
 *
 */

struct pmu_driver bcm59040_rtc_driver =
{
	.driver	= {
		.name	= BCM59040_RTC_DRIVER,
		.owner	= THIS_MODULE,
	},
	.probe		= &bcm59040_rtc_probe,
	.remove		= &bcm59040_rtc_remove,
	.suspend	= &bcm59040_rtc_suspend,
	.resume		= &bcm59040_rtc_resume,
	.id_table	= bcm59040_rtc_idtable,
};


static int __init bcm59040_476x_rtc_init(void)
{
	printk(KERN_INFO "bcm59040_rtc (BCM4760), (c) 2009-2010 Broadcom Corporation\n");
	return pmu_register_driver(&bcm59040_rtc_driver);
}

static void __exit bcm59040_476x_rtc_exit(void)
{
	pmu_unregister_driver(&bcm59040_rtc_driver);
}

device_initcall(bcm59040_476x_rtc_init);
module_exit(bcm59040_476x_rtc_exit);

MODULE_DESCRIPTION( RTC_BCM59040_MODULE_DESC );
MODULE_AUTHOR( "Broadcom Corporation" );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( RTC_BCM59040_MODULE_VERSION );
MODULE_ALIAS( "pmu:bcm59040_rtc (BCM4760)" );
