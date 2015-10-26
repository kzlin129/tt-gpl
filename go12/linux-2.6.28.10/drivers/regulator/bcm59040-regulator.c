/*****************************************************************************
* Copyright 2009 - 2009 Broadcom Corporation.  All rights reserved.
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
*
*****************************************************************************
*
*  bcm59040-regulator.c
*
*  PURPOSE:
*
*     This implements the BCM59040 chip specific portion of the regulator
*     framework driver.
*
*  NOTES:
*
****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <linux/regulator/bcm59040-regulators.h>

#include <linux/broadcom/pmu_chip.h>
#include <linux/broadcom/pmu_bcm59040.h>

#include <asm/arch/reg_pwrseq.h>

struct bcm59040_regulator_info {
	struct regulator_desc desc;
	struct regulator_dev *rdev;
};

/* BCM59040 common operations */
static int bcm59040_set_voltage(
	struct regulator_dev *rdev,
	int min_uV,
	int max_uV)
{
	struct bcm59040_regulator_info *info = rdev_get_drvdata(rdev);

	if (PMU_regulator_set_voltage(current_pmu,
								  info->desc.id,
								  min_uV / 1000))
	{
		pr_err("invalid voltage range (%d, %d) uV", min_uV, max_uV);
		return -EINVAL;
	}
	return 0;
}

static int bcm59040_get_voltage(
	struct regulator_dev *rdev)
{
	int mV;
	u32 min_mV, max_mV, mV_step;
	struct bcm59040_regulator_info *info = rdev_get_drvdata(rdev);

	mV = (int)PMU_regulator_get_voltage(current_pmu,
										info->desc.id,
									    &min_mV,
									    &max_mV,
									    &mV_step);
	return (mV <= 0) ? mV : (mV * 1000);
}

static int bcm59040_enable(
	struct regulator_dev *rdev)
{
	int new_pm_state,rc;
	struct bcm59040_regulator_info *info = rdev_get_drvdata(rdev);

	new_pm_state = pwrseq_switch_pm_states_begin();
	rc = PMU_regulator_set_state_for_pm(current_pmu, info->desc.id,
								        new_pm_state, PMU_Regulator_On);
	pwrseq_switch_pm_states_finalize(new_pm_state);

	return rc;
}

static int bcm59040_disable(
	struct regulator_dev *rdev)
{
	int new_pm_state,rc;
	struct bcm59040_regulator_info *info = rdev_get_drvdata(rdev);

	new_pm_state = pwrseq_switch_pm_states_begin();
	rc = PMU_regulator_set_state_for_pm(current_pmu, info->desc.id,
								        new_pm_state, PMU_Regulator_Off);
	pwrseq_switch_pm_states_finalize(new_pm_state);

	return rc;
}

static int bcm59040_is_enabled(
	struct regulator_dev *rdev)
{
	int current_pm_state;

	struct bcm59040_regulator_info *info = rdev_get_drvdata(rdev);

	current_pm_state = pwrseq_get_current_pm_state();

	return (PMU_regulator_get_state_for_pm(current_pmu,
										   info->desc.id,
										   current_pm_state) !=
			PMU_Regulator_Off);
}

static int bcm59040_suspend_enable(struct regulator_dev *rdev)
{
    int rc;
    struct bcm59040_regulator_info *info = rdev_get_drvdata(rdev);

    rc = PMU_regulator_set_state_for_pm(current_pmu, info->desc.id,
                                        3, PMU_Regulator_On);

    return rc;
}

static int bcm59040_suspend_disable(struct regulator_dev *rdev)
{
    int rc;
    struct bcm59040_regulator_info *info = rdev_get_drvdata(rdev);

    rc = PMU_regulator_set_state_for_pm(current_pmu, info->desc.id,
                                        3, PMU_Regulator_Off);

    return rc;
}


static struct regulator_ops bcm59040_LDO_ops = {
    .get_voltage           = bcm59040_get_voltage,
    .enable                = bcm59040_enable,
    .disable               = bcm59040_disable,
    .is_enabled            = bcm59040_is_enabled,
    .set_suspend_enable    = bcm59040_suspend_enable,
    .set_suspend_disable   = bcm59040_suspend_disable,
};

static struct regulator_ops bcm59040_switcher_ops = {
    .set_voltage            = bcm59040_set_voltage,
    .get_voltage            = bcm59040_get_voltage,
    .enable                 = bcm59040_enable,
    .disable                = bcm59040_disable,
    .is_enabled             = bcm59040_is_enabled,
    .set_suspend_enable     = bcm59040_suspend_enable,
    .set_suspend_disable    = bcm59040_suspend_disable,
};

struct bcm59040_regulator_info bcm59040_regulators[] = {
    {
        .desc =
        {
            .name       = "LDO1",
            .id         = BCM59040_LDO1_ID,
            .ops        = &bcm59040_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO2",
            .id         = BCM59040_LDO2_ID,
            .ops        = &bcm59040_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO3",
            .id         = BCM59040_LDO3_ID,
            .ops        = &bcm59040_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO4",
            .id         = BCM59040_LDO4_ID,
            .ops        = &bcm59040_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO5",
            .id         = BCM59040_LDO5_ID,
            .ops        = &bcm59040_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "LDO6",
            .id         = BCM59040_LDO6_ID,
            .ops        = &bcm59040_LDO_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "CSR",
            .id         = BCM59040_CSR_ID,
            .ops        = &bcm59040_switcher_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        },
    },
    {
        .desc =
        {
            .name       = "IOSR",
            .id         = BCM59040_IOSR_ID,
            .ops        = &bcm59040_switcher_ops,
            .type       = REGULATOR_VOLTAGE,
            .owner      = THIS_MODULE,
        }
    },
};

static int __devinit bcm59040_regulator_probe(
	struct platform_device *pdev)
{
    struct regulator_dev *rdev;

    rdev = regulator_register(&bcm59040_regulators[pdev->id].desc, &pdev->dev,
                              &bcm59040_regulators[pdev->id]);

    if (IS_ERR(rdev))
        return PTR_ERR(rdev);

    return 0;
}

static int bcm59040_regulator_remove(
	struct platform_device *pdev)
{
    struct regulator_dev *rdev = platform_get_drvdata(pdev);

    regulator_unregister(rdev);

    return 0;
}

static struct platform_driver bcm59040_regulator_driver = {
	.driver	= {
		.name	= "bcmpmu_regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= bcm59040_regulator_probe,
	.remove		= bcm59040_regulator_remove,
};

static int __init bcm59040_regulator_init(void)
{
	return platform_driver_register(&bcm59040_regulator_driver);
}
module_init(bcm59040_regulator_init);

static void __exit bcm59040_regulator_exit(void)
{
	platform_driver_unregister(&bcm59040_regulator_driver);
}
module_exit(bcm59040_regulator_exit);
