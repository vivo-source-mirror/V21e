/* Copyright (c) 2019 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "PM8008: %s: " fmt, __func__

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/string.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>

#define pm8008_err(reg, message, ...) \
	pr_err("%s: " message, (reg)->rdesc.name, ##__VA_ARGS__)
#define pm8008_debug(reg, message, ...) \
	pr_debug("%s: " message, (reg)->rdesc.name, ##__VA_ARGS__)

#define STARTUP_DELAY_USEC		20
#define VSET_STEP_SIZE_MV		1
#define VSET_STEP_MV			8
#define VSET_STEP_UV			(VSET_STEP_MV * 1000)

#define MISC_BASE			0x900

#define MISC_CHIP_ENABLE_REG		(MISC_BASE + 0x50)
#define CHIP_ENABLE_BIT			BIT(0)

#define LDO_ENABLE_REG(base)		(base + 0x46)
#define ENABLE_BIT			BIT(7)

#define LDO_STATUS1_REG(base)		(base + 0x08)
#define VREG_READY_BIT			BIT(7)
#define MODE_STATE_MASK			GENMASK(1, 0)
#define MODE_STATE_NPM			3
#define MODE_STATE_LPM			2
#define MODE_STATE_BYPASS		0

#define LDO_VSET_LB_REG(base)		(base + 0x40)

#define LDO_VSET_VALID_LB_REG(base)	(base + 0x42)

#define LDO_MODE_CTL1_REG(base)		(base + 0x45)
#define MODE_PRIMARY_MASK		GENMASK(2, 0)
#define LDO_MODE_NPM			7
#define LDO_MODE_LPM			4
#define FORCED_BYPASS			2

#define LDO_STEPPER_CTL_REG(base)	(base + 0x3b)
#define STEP_RATE_MASK			GENMASK(1, 0)

#define LDO_PD_CTL_REG(base)		(base + 0xA0)
#define STRONG_PD_EN_BIT		BIT(7)

#define MAX_REG_NAME			20
#define PM8008_MAX_LDO			7

/*vivo hujin add to set debounce time from 240us to 960us begin*/
#define LDO_OCP_CTL1_REG(base)		(base + 0x88)
#define OCP_DBG			(BIT(5)+BIT(4))
/*vivo hujin add to set debounce time from 240us to 960us end*/

static int pm8008_reset(void);

struct reset_data {
	struct regmap *chip_rm;
	struct regmap *reg_rm;
	unsigned int reset_gpio;
	bool is_probe_successed;
	struct mutex		lock;
	u8	module_en;
	int volt[7];
	u8	mode[7];
	u8	en[7];
	u8	pd[7];
};

static struct reset_data rst_data = {0};

struct pm8008_chip {
	struct device		*dev;
	struct regmap		*regmap;
	struct regulator_dev	*rdev;
	struct regulator_desc	rdesc;

};

struct regulator_data {
	char		*name;
	char		*supply_name;
	int		hpm_min_load_ua;
	int		min_dropout_uv;
};

struct pm8008_regulator {
	struct device		*dev;
	struct regmap		*regmap;
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
	struct regulator	*parent_supply;
	struct regulator	*en_supply;
	struct device_node	*of_node;
	u16			base;
	int			hpm_min_load_ua;
	int			min_dropout_uv;
	int			step_rate;
};

static struct regulator_data reg_data[] = {
			/* name,        parent,  min load, headroom */
			{"pm8008_l1", "vdd_l1_l2", 10000, 225000},
			{"pm8008_l2", "vdd_l1_l2", 10000, 225000},
			{"pm8008_l3", "vdd_l3_l4", 10000, 200000},
			{"pm8008_l4", "vdd_l3_l4", 10000, 200000},
			{"pm8008_l5", "vdd_l5", 10000, 300000},
			{"pm8008_l6", "vdd_l6", 10000, 300000},
			{"pm8008_l7", "vdd_l7", 10000, 300000},
};

/* common functions */
static int pm8008_read(struct regmap *regmap,  u16 reg, u8 *val, int count)
{
	int rc;

retry:
	mutex_lock(&rst_data.lock);
	rc = regmap_bulk_read(regmap, reg, val, count);
	if (rc < 0)
		pr_err("failed to read 0x%04x\n", reg);
	mutex_unlock(&rst_data.lock);
	if (rc < 0) {
		if (rst_data.is_probe_successed == true && pm8008_reset() == 0)
			goto retry;
	}
	return rc;
}

static int pm8008_write(struct regmap *regmap, u16 reg, u8 *val, int count)
{
	int rc;

	pr_debug("Writing 0x%02x to 0x%04x\n", val, reg);
	rc = regmap_bulk_write(regmap, reg, val, count);
	if (rc < 0)
		pr_err("failed to write 0x%04x\n", reg);

	return rc;
}

static int pm8008_masked_write(struct regmap *regmap, u16 reg, u8 mask,
				u8 val)
{
	int rc;

retry:
	mutex_lock(&rst_data.lock);

	pr_debug("Writing 0x%02x to 0x%04x with mask 0x%02x\n", val, reg, mask);
	rc = regmap_update_bits(regmap, reg, mask, val);
	if (rc < 0)
		pr_err("failed to write 0x%02x to 0x%04x with mask 0x%02x\n",
				val, reg, mask);
	mutex_unlock(&rst_data.lock);
	if (rc < 0) {
		if (rst_data.is_probe_successed == true && pm8008_reset() == 0)
			goto retry;
	}
	return rc;
}

static int pm8008_reset(void)
{
	int rc = 0, mv;
	int i, j;
	u8 vset_raw[2];
	u16 reg;

	pr_err("Doing reset for pm8008\n");
	mutex_lock(&rst_data.lock);

	for (j = 3; j > 0; j--) {
		mdelay(5);
		gpio_direction_output(rst_data.reset_gpio, 0);
		pr_err("pm8008:pull down reset_gpio\n");
		mdelay(5);
		gpio_direction_output(rst_data.reset_gpio, 1);
		pr_err("pm8008:pull up reset_gpio\n");
		mdelay(5);

		reg = 0x950;
		pr_err("Writing 0x%02x to 0x%04x\n", rst_data.module_en, reg);
		rst_data.module_en = 0x01;
		rc = regmap_bulk_write(rst_data.chip_rm, reg, &rst_data.module_en, 1);
		if (rc < 0) {
			pr_err("failed to write 0x%04x\n", reg);
			goto unlock;
		}
		for (i = 0; i < 7; i++) {
			/* strong pd */
			reg = 0x40A0 + i*0x100;
			pr_err("Writing 0x%02x to 0x%04x\n", rst_data.pd[i], reg);
			rc = regmap_bulk_write(rst_data.reg_rm, reg, &rst_data.pd[i], 1);
			if (rc < 0) {
				pr_err("failed to write 0x%04x\n", reg);
				goto unlock;
			}

			/* voltage */
			reg = 0x4040 + i*0x100;
			if (rst_data.volt[i] != 0) {
				mv = DIV_ROUND_UP(rst_data.volt[i], 1000);
				mv = DIV_ROUND_UP(DIV_ROUND_UP(mv, VSET_STEP_MV) * VSET_STEP_MV,
						VSET_STEP_SIZE_MV);
				vset_raw[0] = mv & 0xff;
				vset_raw[1] = (mv & 0xff00) >> 8;
				pr_err("Writing 0x%02x 0x%02x to 0x%04x\n", vset_raw[0], vset_raw[1], reg);
				rc = regmap_bulk_write(rst_data.reg_rm, reg, vset_raw, 2);
				if (rc < 0) {
					pr_err("failed to write 0x%04x\n", reg);
					goto unlock;
				}
			}

			/* mode */
			reg = 0x4045 + i*0x100;
			if (rst_data.mode[i] != 0) {
				pr_err("Writing 0x%02x to 0x%04x\n", rst_data.mode[i], reg);
				rc = regmap_bulk_write(rst_data.reg_rm, reg, &rst_data.mode[i], 1);
				if (rc < 0) {
					pr_err("failed to write 0x%04x\n", reg);
					goto unlock;
				}
			}

			/* enable */
			reg = 0x4046 + i*0x100;
			pr_err("Writing 0x%02x to 0x%04x\n", rst_data.en[i], reg);
			rc = regmap_bulk_write(rst_data.reg_rm, reg, &rst_data.en[i], 1);
			if (rc < 0) {
				pr_err("failed to write 0x%04x\n", reg);
				goto unlock;
			}
		}

	unlock:
		if (rc == 0)
			break;
	}
	mutex_unlock(&rst_data.lock);
	return rc;
}

/* PM8008 LDO Regulator callbacks */
static int pm8008_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);
	u8 vset_raw[2];
	int rc;

	rc = pm8008_read(pm8008_reg->regmap,
			LDO_VSET_VALID_LB_REG(pm8008_reg->base),
			vset_raw, 2);
	if (rc < 0) {
		pm8008_err(pm8008_reg,
			"failed to read regulator voltage rc=%d\n", rc);
		return rc;
	}

	pm8008_debug(pm8008_reg, "VSET read [%x][%x]\n",
			vset_raw[1], vset_raw[0]);
	return (vset_raw[1] << 8 | vset_raw[0]) * 1000;
}

static int pm8008_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);
	int rc;
	u8 reg;

	rc = pm8008_read(pm8008_reg->regmap,
			LDO_ENABLE_REG(pm8008_reg->base), &reg, 1);
	if (rc < 0) {
		pm8008_err(pm8008_reg, "failed to read enable reg rc=%d\n", rc);
		return rc;
	}

	return !!(reg & ENABLE_BIT);
}

static int pm8008_regulator_enable(struct regulator_dev *rdev)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);
	int rc, rc2, current_uv, delay_us, delay_ms, retry_count = 10;
	u8 reg;

	current_uv = pm8008_regulator_get_voltage(rdev);
	if (current_uv < 0) {
		pm8008_err(pm8008_reg, "failed to get current voltage rc=%d\n",
			current_uv);
		return current_uv;
	}

	rc = regulator_enable(pm8008_reg->en_supply);
	if (rc < 0) {
		pm8008_err(pm8008_reg,
			"failed to enable en_supply rc=%d\n", rc);
		return rc;
	}

	if (pm8008_reg->parent_supply) {
		rc = regulator_set_voltage(pm8008_reg->parent_supply,
					current_uv + pm8008_reg->min_dropout_uv,
					INT_MAX);
		if (rc < 0) {
			pm8008_err(pm8008_reg, "failed to request parent supply voltage rc=%d\n",
				rc);
			goto remove_en;
		}

		rc = regulator_enable(pm8008_reg->parent_supply);
		if (rc < 0) {
			pm8008_err(pm8008_reg,
				"failed to enable parent rc=%d\n", rc);
			regulator_set_voltage(pm8008_reg->parent_supply, 0,
						INT_MAX);
			goto remove_en;
		}
	}

	rc = pm8008_masked_write(pm8008_reg->regmap,
				LDO_ENABLE_REG(pm8008_reg->base),
				ENABLE_BIT, ENABLE_BIT);
	if (rc < 0) {
		pm8008_err(pm8008_reg,
			"failed to enable regulator rc=%d\n", rc);
		goto remove_vote;
	}

	/*
	 * Wait for the VREG_READY status bit to be set using a timeout delay
	 * calculated from the current commanded voltage.
	 */
	delay_us = STARTUP_DELAY_USEC
			+ DIV_ROUND_UP(current_uv, pm8008_reg->step_rate);
	delay_ms = DIV_ROUND_UP(delay_us, 1000);

	/* Retry 10 times for VREG_READY before bailing out */
	while (retry_count--) {
		if (delay_ms > 20)
			msleep(delay_ms);
		else
			usleep_range(delay_us, delay_us + 100);

		rc = pm8008_read(pm8008_reg->regmap,
				LDO_STATUS1_REG(pm8008_reg->base), &reg, 1);
		if (rc < 0) {
			pm8008_err(pm8008_reg,
				"failed to read regulator status rc=%d\n", rc);
			goto disable_ldo;
		}
		if (reg & VREG_READY_BIT) {
			pm8008_debug(pm8008_reg, "regulator enabled\n");
			rst_data.en[(pm8008_reg->base-0x4000)/0x100] = 0x80;
			return 0;
		}
	}

	pm8008_err(pm8008_reg, "failed to enable regulator, VREG_READY not set\n");
	rc = -ETIME;

disable_ldo:
	pm8008_masked_write(pm8008_reg->regmap,
			LDO_ENABLE_REG(pm8008_reg->base), ENABLE_BIT, 0);

remove_vote:
	if (pm8008_reg->parent_supply) {
		rc2 = regulator_disable(pm8008_reg->parent_supply);
		if (rc2 < 0)
			pm8008_err(pm8008_reg, "failed to disable parent supply rc=%d\n",
				rc2);
		rc2 = regulator_set_voltage(pm8008_reg->parent_supply, 0,
						INT_MAX);
		if (rc2 < 0)
			pm8008_err(pm8008_reg, "failed to remove voltage vote for parent supply rc=%d\n",
				rc2);
	}

remove_en:
	rc2 = regulator_disable(pm8008_reg->en_supply);
	if (rc2 < 0)
		pm8008_err(pm8008_reg, "failed to disable en_supply rc=%d\n",
			rc2);

	return rc;
}

static int pm8008_regulator_disable(struct regulator_dev *rdev)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);
	int rc;

	rc = pm8008_masked_write(pm8008_reg->regmap,
				LDO_ENABLE_REG(pm8008_reg->base),
				ENABLE_BIT, 0);
	if (rc < 0) {
		pm8008_err(pm8008_reg,
			"failed to disable regulator rc=%d\n", rc);
		return rc;
	}

	/* remove voltage vote from parent regulator */
	if (pm8008_reg->parent_supply) {
		rc = regulator_disable(pm8008_reg->parent_supply);
		if (rc < 0) {
			pm8008_err(pm8008_reg, "failed to disable parent rc=%d\n",
				rc);
			return rc;
		}
		rc = regulator_set_voltage(pm8008_reg->parent_supply,
					0, INT_MAX);
		if (rc < 0) {
			pm8008_err(pm8008_reg, "failed to remove parent voltage rc=%d\n",
				rc);
			return rc;
		}
	}

	/* remove vote from chip enable regulator */
	rc = regulator_disable(pm8008_reg->en_supply);
	if (rc < 0) {
		pm8008_err(pm8008_reg, "failed to disable en_supply rc=%d\n",
			rc);
		return rc;
	}
	rst_data.en[(pm8008_reg->base-0x4000)/0x100] = 0;

	pm8008_debug(pm8008_reg, "regulator disabled\n");
	return 0;
}

static int pm8008_write_voltage(struct pm8008_regulator *pm8008_reg, int min_uv,
				int max_uv)
{
	int rc = 0, mv;
	u8 vset_raw[2];

	mv = DIV_ROUND_UP(min_uv, 1000);
	if (mv * 1000 > max_uv) {
		pm8008_err(pm8008_reg,
			"requested voltage above maximum limit\n");
		return -EINVAL;
	}

	/*
	 * Each LSB of regulator is 1mV and the voltage setpoint
	 * should be multiple of 8mV(step).
	 */
	mv = DIV_ROUND_UP(DIV_ROUND_UP(mv, VSET_STEP_MV) * VSET_STEP_MV,
				VSET_STEP_SIZE_MV);

	vset_raw[0] = mv & 0xff;
	vset_raw[1] = (mv & 0xff00) >> 8;
	rc = pm8008_write(pm8008_reg->regmap, LDO_VSET_LB_REG(pm8008_reg->base),
			vset_raw, 2);
	if (rc < 0) {
		pm8008_err(pm8008_reg, "failed to write voltage rc=%d\n", rc);
		return rc;
	}

	pm8008_debug(pm8008_reg, "VSET=[%x][%x]\n", vset_raw[1], vset_raw[0]);
	return 0;
}

static int pm8008_regulator_set_voltage_time(struct regulator_dev *rdev,
				int old_uV, int new_uv)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);

	return DIV_ROUND_UP(abs(new_uv - old_uV), pm8008_reg->step_rate);
}

static int pm8008_regulator_set_voltage(struct regulator_dev *rdev,
				int min_uv, int max_uv, unsigned int *selector)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);
	int rc = 0, current_uv = 0, rounded_uv = 0, enabled = 0;

	if (pm8008_reg->parent_supply) {
		enabled = pm8008_regulator_is_enabled(rdev);
		if (enabled < 0) {
			return enabled;
		} else if (enabled) {
			current_uv = pm8008_regulator_get_voltage(rdev);
			if (current_uv < 0)
				return current_uv;
			rounded_uv = roundup(min_uv, VSET_STEP_UV);
		}
	}

	/*
	 * Set the parent_supply voltage before changing the LDO voltage when
	 * the LDO voltage is being increased.
	 */
	if (pm8008_reg->parent_supply && enabled && rounded_uv >= current_uv) {
		/* Request parent voltage with headroom */
		rc = regulator_set_voltage(pm8008_reg->parent_supply,
					rounded_uv + pm8008_reg->min_dropout_uv,
					INT_MAX);
		if (rc < 0) {
			pm8008_err(pm8008_reg, "failed to request parent supply voltage rc=%d\n",
				rc);
			return rc;
		}
	}

	rc = pm8008_write_voltage(pm8008_reg, min_uv, max_uv);
	if (rc < 0)
		return rc;

	/*
	 * Set the parent_supply voltage after changing the LDO voltage when
	 * the LDO voltage is being reduced.
	 */
	if (pm8008_reg->parent_supply && enabled && rounded_uv < current_uv) {
		/*
		 * Ensure sufficient time for the LDO voltage to slew down
		 * before reducing the parent supply voltage.  The regulator
		 * framework will add the same delay after this function returns
		 * in all cases (i.e. enabled/disabled and increasing/decreasing
		 * voltage).
		 */
		udelay(pm8008_regulator_set_voltage_time(rdev, rounded_uv,
							current_uv));

		/* Request parent voltage with headroom */
		rc = regulator_set_voltage(pm8008_reg->parent_supply,
					rounded_uv + pm8008_reg->min_dropout_uv,
					INT_MAX);
		if (rc < 0) {
			pm8008_err(pm8008_reg, "failed to request parent supply voltage rc=%d\n",
				rc);
			return rc;
		}
	}
	rst_data.volt[(pm8008_reg->base-0x4000)/0x100] = min_uv;
	pm8008_debug(pm8008_reg, "voltage set to %d\n", min_uv);
	return rc;
}

static int pm8008_regulator_set_mode(struct regulator_dev *rdev,
				unsigned int mode)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);
	int rc;
	u8 val = LDO_MODE_LPM;

	if (mode == REGULATOR_MODE_NORMAL)
		val = LDO_MODE_NPM;
	else if (mode == REGULATOR_MODE_IDLE)
		val = LDO_MODE_LPM;

	rc = pm8008_masked_write(pm8008_reg->regmap,
				LDO_MODE_CTL1_REG(pm8008_reg->base),
				MODE_PRIMARY_MASK, val);
	if (!rc)
		pm8008_debug(pm8008_reg, "mode set to %d\n", val);

	rst_data.mode[(pm8008_reg->base-0x4000)/0x100] = val;

	return rc;
}

static unsigned int pm8008_regulator_get_mode(struct regulator_dev *rdev)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);
	int rc;
	u8 reg;

	rc = pm8008_read(pm8008_reg->regmap,
			LDO_STATUS1_REG(pm8008_reg->base), &reg, 1);
	if (rc < 0) {
		pm8008_err(pm8008_reg, "failed to get mode rc=%d\n", rc);
		return rc;
	}

	return ((reg & MODE_STATE_MASK) == MODE_STATE_NPM)
			? REGULATOR_MODE_NORMAL : REGULATOR_MODE_IDLE;
}

static int pm8008_regulator_set_load(struct regulator_dev *rdev, int load_uA)
{
	struct pm8008_regulator *pm8008_reg = rdev_get_drvdata(rdev);
	int mode;

	if (load_uA >= pm8008_reg->hpm_min_load_ua)
		mode = REGULATOR_MODE_NORMAL;
	else
		mode = REGULATOR_MODE_IDLE;

	return pm8008_regulator_set_mode(rdev, mode);
}

static struct regulator_ops pm8008_regulator_ops = {
	.enable			= pm8008_regulator_enable,
	.disable		= pm8008_regulator_disable,
	.is_enabled		= pm8008_regulator_is_enabled,
	.set_voltage		= pm8008_regulator_set_voltage,
	.get_voltage		= pm8008_regulator_get_voltage,
	.set_mode		= pm8008_regulator_set_mode,
	.get_mode		= pm8008_regulator_get_mode,
	.set_load		= pm8008_regulator_set_load,
	.set_voltage_time	= pm8008_regulator_set_voltage_time,
};

static int pm8008_register_ldo(struct pm8008_regulator *pm8008_reg,
						const char *name)
{
	struct regulator_config reg_config = {};
	struct regulator_init_data *init_data;
	struct device *dev = pm8008_reg->dev;
	struct device_node *reg_node = pm8008_reg->of_node;
	char buff[MAX_REG_NAME];
	int rc, i, init_voltage;
	u8 reg;

	/* get regulator data */
	for (i = 0; i < PM8008_MAX_LDO; i++)
		if (!strcmp(reg_data[i].name, name))
			break;

	if (i == PM8008_MAX_LDO) {
		pr_err("Invalid regulator name %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u16(reg_node, "reg", &pm8008_reg->base);
	if (rc < 0) {
		pr_err("%s: failed to get regulator base rc=%d\n", name, rc);
		return rc;
	}

	pm8008_reg->min_dropout_uv = reg_data[i].min_dropout_uv;
	of_property_read_u32(reg_node, "qcom,min-dropout-voltage",
						&pm8008_reg->min_dropout_uv);

	pm8008_reg->hpm_min_load_ua = reg_data[i].hpm_min_load_ua;
	of_property_read_u32(reg_node, "qcom,hpm-min-load",
						&pm8008_reg->hpm_min_load_ua);
	init_voltage = -EINVAL;
	of_property_read_u32(reg_node, "qcom,init-voltage", &init_voltage);

	if (of_property_read_bool(reg_node, "qcom,strong-pd")) {
		rc = pm8008_masked_write(pm8008_reg->regmap,
				LDO_PD_CTL_REG(pm8008_reg->base),
				STRONG_PD_EN_BIT, STRONG_PD_EN_BIT);
		if (rc < 0) {
			pr_err("%s: failed to configure pull down rc=%d\n",
				name, rc);
			return rc;
		}
		rst_data.pd[(pm8008_reg->base-0x4000)/0x100] = 0x80;
	}

	/*vivo hujin add to set debounce time from 240us to 960us begin*/
	rc = pm8008_masked_write(pm8008_reg->regmap, LDO_OCP_CTL1_REG(pm8008_reg->base),
			OCP_DBG, OCP_DBG);
	if (rc < 0) {
		pr_err("%s: failed to configure pull down rc=%d\n",
			name, rc);
		return rc;
	}
	/*vivo hujin add to set debounce time from 240us to 960us end*/

	/* get slew rate */
	rc = pm8008_read(pm8008_reg->regmap,
			LDO_STEPPER_CTL_REG(pm8008_reg->base), &reg, 1);
	if (rc < 0) {
		pr_err("%s: failed to read step rate configuration rc=%d\n",
				name, rc);
		return rc;
	}
	pm8008_reg->step_rate = 38400 >> (reg & STEP_RATE_MASK);

	scnprintf(buff, MAX_REG_NAME, "%s-supply", reg_data[i].supply_name);
	if (of_find_property(dev->of_node, buff, NULL)) {
		pm8008_reg->parent_supply = devm_regulator_get(dev,
						reg_data[i].supply_name);
		if (IS_ERR(pm8008_reg->parent_supply)) {
			rc = PTR_ERR(pm8008_reg->parent_supply);
			if (rc != -EPROBE_DEFER)
				pr_err("%s: failed to get parent regulator rc=%d\n",
					name, rc);
			return rc;
		}
	}

	/* pm8008_en should be present otherwise fail the regulator probe */
	pm8008_reg->en_supply = devm_regulator_get(dev, "pm8008_en");
	if (IS_ERR(pm8008_reg->en_supply)) {
		rc = PTR_ERR(pm8008_reg->en_supply);
		pr_err("%s: failed to get chip_en supply\n", name);
		return rc;
	}

	init_data = of_get_regulator_init_data(dev, reg_node,
						&pm8008_reg->rdesc);
	if (init_data == NULL) {
		pr_err("%s: failed to get regulator data\n", name);
		return -ENODATA;
	}
	if (!init_data->constraints.name) {
		pr_err("%s: regulator name missing\n", name);
		return -EINVAL;
	}

	/* configure the initial voltage for the regulator */
	if (init_voltage > 0) {
		rc = pm8008_write_voltage(pm8008_reg, init_voltage,
					init_data->constraints.max_uV);
		if (rc < 0)
			pr_err("%s: failed to set initial voltage rc=%d\n",
					name, rc);
		 rst_data.volt[(pm8008_reg->base-0x4000)/0x100] = init_voltage;

	}

	init_data->constraints.input_uV = init_data->constraints.max_uV;
	init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_STATUS
						| REGULATOR_CHANGE_VOLTAGE
						| REGULATOR_CHANGE_MODE
						| REGULATOR_CHANGE_DRMS;
	reg_config.dev = dev;
	reg_config.init_data = init_data;
	reg_config.driver_data = pm8008_reg;
	reg_config.of_node = reg_node;

	pm8008_reg->rdesc.owner = THIS_MODULE;
	pm8008_reg->rdesc.type = REGULATOR_VOLTAGE;
	pm8008_reg->rdesc.ops = &pm8008_regulator_ops;
	pm8008_reg->rdesc.name = init_data->constraints.name;
	pm8008_reg->rdesc.n_voltages = 1;

	pm8008_reg->rdev = devm_regulator_register(dev, &pm8008_reg->rdesc,
						&reg_config);
	if (IS_ERR(pm8008_reg->rdev)) {
		rc = PTR_ERR(pm8008_reg->rdev);
		pr_err("%s: failed to register regulator rc=%d\n",
				pm8008_reg->rdesc.name, rc);
		return rc;
	}

	pr_debug("%s regulator registered\n", name);

	return 0;
}

/* PMIC probe and helper function */
static int pm8008_parse_regulator(struct regmap *regmap, struct device *dev)
{
	int rc = 0;
	const char *name;
	struct device_node *child;
	struct pm8008_regulator *pm8008_reg;

	/* parse each subnode and register regulator for regulator child */
	for_each_available_child_of_node(dev->of_node, child) {
		pm8008_reg = devm_kzalloc(dev, sizeof(*pm8008_reg), GFP_KERNEL);
		if (!pm8008_reg)
			return -ENOMEM;

		pm8008_reg->regmap = regmap;
		pm8008_reg->of_node = child;
		pm8008_reg->dev = dev;

		rc = of_property_read_string(child, "regulator-name", &name);
		if (rc)
			continue;

		rc = pm8008_register_ldo(pm8008_reg, name);
		if (rc < 0) {
			pr_err("failed to register regulator %s rc=%d\n",
					name, rc);
			return rc;
		}
	}

	return 0;
}

static void pm8008_release_reset_gpio(void)
{
	if (gpio_is_valid(rst_data.reset_gpio)) {
		pinctrl_free_gpio(rst_data.reset_gpio);
		gpio_free(rst_data.reset_gpio);
		pr_info("pm8008 relase reset gpio");
	}
}

static int pm8008_regulator_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct regmap *regmap;
	pr_info("pm8008_regulator_probe enter\n");
	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!regmap) {
		pr_err("parent regmap is missing\n");
		pm8008_release_reset_gpio();
		rst_data.is_probe_successed = false;
		return -EINVAL;
	}
	rst_data.reg_rm = regmap;
	if (gpio_is_valid(rst_data.reset_gpio)) {
		if (gpio_get_value(rst_data.reset_gpio) == 0) {
			gpio_direction_output(rst_data.reset_gpio, 1);
			usleep_range(500, 600);
			pr_info("pm8008_regulator_probe  reset gpio set to high\n");
		}
	}
	rc = pm8008_parse_regulator(regmap, &pdev->dev);
	if (rc < 0) {
		pr_err("failed to parse device tree rc=%d\n", rc);
		pm8008_release_reset_gpio();
		rst_data.is_probe_successed = false;
		return rc;
	}
	rst_data.is_probe_successed = true;
	return 0;
}

/* PM8008 chip enable regulator callbacks */
static int pm8008_enable_regulator_enable(struct regulator_dev *rdev)
{
	struct pm8008_chip *chip = rdev_get_drvdata(rdev);
	int rc;

	rc = pm8008_masked_write(chip->regmap, MISC_CHIP_ENABLE_REG,
				CHIP_ENABLE_BIT, CHIP_ENABLE_BIT);
	if (rc  < 0) {
		pm8008_err(chip, "failed to enable chip rc=%d\n", rc);
		return rc;
	}
	rst_data.module_en = 0x1;
	pm8008_debug(chip, "regulator enabled\n");
	return 0;
}

static int pm8008_enable_regulator_disable(struct regulator_dev *rdev)
{
	struct pm8008_chip *chip = rdev_get_drvdata(rdev);
	int rc;

	rc = pm8008_masked_write(chip->regmap, MISC_CHIP_ENABLE_REG,
				CHIP_ENABLE_BIT, 0);
	if (rc  < 0) {
		pm8008_err(chip, "failed to disable chip rc=%d\n", rc);
		return rc;
	}
	rst_data.module_en = 0;
	pm8008_debug(chip, "regulator disabled\n");
	return 0;
}

static int pm8008_enable_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct pm8008_chip *chip = rdev_get_drvdata(rdev);
	int rc;
	u8 reg;

	rc = pm8008_read(chip->regmap, MISC_CHIP_ENABLE_REG, &reg, 1);
	if (rc  < 0) {
		pm8008_err(chip, "failed to get chip state rc=%d\n", rc);
		return rc;
	}

	return !!(reg & CHIP_ENABLE_BIT);
}

static struct regulator_ops pm8008_enable_reg_ops = {
	.enable = pm8008_enable_regulator_enable,
	.disable = pm8008_enable_regulator_disable,
	.is_enabled = pm8008_enable_regulator_is_enabled,
};

static int pm8008_init_enable_regulator(struct pm8008_chip *chip)
{
	struct regulator_config cfg = {};
	int rc = 0;

	cfg.dev = chip->dev;
	cfg.driver_data = chip;

	chip->rdesc.owner = THIS_MODULE;
	chip->rdesc.type = REGULATOR_VOLTAGE;
	chip->rdesc.ops = &pm8008_enable_reg_ops;
	chip->rdesc.of_match = "qcom,pm8008-chip-en";
	chip->rdesc.name = "qcom,pm8008-chip-en";

	chip->rdev = devm_regulator_register(chip->dev, &chip->rdesc, &cfg);
	if (IS_ERR(chip->rdev)) {
		rc = PTR_ERR(chip->rdev);
		chip->rdev = NULL;
		return rc;
	}

	return 0;
}

static int pm8008_chip_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct pm8008_chip *chip = NULL;

	pr_info("pm8008_chip_probe enter\n");
	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("err chip  null\n");
		return -ENOMEM;
	}
	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		pr_err("parent regmap is missing\n");
		return -EINVAL;
	}
	chip->dev = &pdev->dev;

	rst_data.chip_rm = chip->regmap;
	rst_data.reset_gpio = of_get_named_gpio(chip->dev->of_node, "qcom,reset-gpio", 0);
	pr_info("pm8008 reset gpio = %d \n", rst_data.reset_gpio);
	if (gpio_is_valid(rst_data.reset_gpio)) {
		rc = gpio_request(rst_data.reset_gpio, "reset_gpio");
		if (rc) {
			pr_err("pm8008 Fail to request reset gpio, rc=%d\n", rc);
			return rc;
		} else {
			gpio_direction_output(rst_data.reset_gpio, 1);
			usleep_range(500, 600);
			pr_info("pm8008_chip_probe  reset gpio set to high\n");
		}
	} else {
		pr_err("pm8008 Fail to request reset gpio\n");
	}
	mutex_init(&rst_data.lock);

	/* Register chip enable regulator */
	rc = pm8008_init_enable_regulator(chip);
	if (rc < 0) {
		pr_err("pm8008 Failed to register chip enable regulator rc=%d\n", rc);
		pm8008_release_reset_gpio();
		rst_data.is_probe_successed = false;
		return rc;
	}

	rst_data.is_probe_successed = true;
	pr_debug("PM8008 chip registered\n");
	return 0;
}

static int pm8008_chip_remove(struct platform_device *pdev)
{
	struct pm8008_chip *chip = platform_get_drvdata(pdev);
	int rc;

	rc = pm8008_masked_write(chip->regmap, MISC_CHIP_ENABLE_REG,
				CHIP_ENABLE_BIT, 0);
	if (rc  < 0)
		pr_err("failed to disable chip rc=%d\n", rc);

	return 0;
}

static const struct of_device_id pm8008_regulator_match_table[] = {
	{
		.compatible	= "qcom,pm8008-regulator",
	},
	{ },
};

static struct platform_driver pm8008_regulator_driver = {
	.driver	= {
		.name		= "qcom,pm8008-regulator",
		.owner		= THIS_MODULE,
		.of_match_table	= pm8008_regulator_match_table,
	},
	.probe		= pm8008_regulator_probe,
};
module_platform_driver(pm8008_regulator_driver);

static const struct of_device_id pm8008_chip_match_table[] = {
	{
		.compatible	= "qcom,pm8008-chip",
	},
	{ },
};

static struct platform_driver pm8008_chip_driver = {
	.driver	= {
		.name		= "qcom,pm8008-chip",
		.owner		= THIS_MODULE,
		.of_match_table	= pm8008_chip_match_table,
	},
	.probe		= pm8008_chip_probe,
	.remove		= pm8008_chip_remove,
};
module_platform_driver(pm8008_chip_driver);

MODULE_DESCRIPTION("QPNP PM8008 PMIC Regulator Driver");
MODULE_LICENSE("GPL v2");
