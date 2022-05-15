/*
 *	drivers/prox_vled_power/vled_power.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vledpower.h>
#include <linux/workqueue.h>


#include <linux/of_platform.h>

#include <linux/regulator/consumer.h>

#define TAG "VLED_POWER"

struct vled_power_vledpower_data {
	struct vledpower_dev sdev;
	const char *name;
	int enable;
	struct vled_power_vreg_data *sensor_vled_vdd;
};

static int vled_vreg_init(struct platform_device *pdev, struct vled_power_vreg_data *vreg)
{
	int rc = 0;
	struct device *dev = &pdev->dev;

	printk(KERN_ERR "[%s]:[%s] vreg_get for : %s \n", TAG, __func__, vreg->name);
	/* Get the regulator handle */
	vreg->reg = regulator_get(dev, vreg->name);
	if (IS_ERR(vreg->reg)) {
		rc = PTR_ERR(vreg->reg);
		printk(KERN_ERR "[%s]:[%s] regulator_get(%s) failed. rc=%d\n", TAG, __func__, vreg->name, rc);
		goto out;
	}

	if ((regulator_count_voltages(vreg->reg) > 0)
			&& (vreg->low_vol_level) && (vreg->high_vol_level))
		vreg->set_voltage_sup = 1;

out:
	return rc;
}

static int vled_vreg_enable(struct vled_power_vreg_data *vreg)
{
	int rc = 0;
	printk(KERN_ERR "[%s]:[%s] vreg_en for : %s %d %d\n", TAG, __func__, vreg->name, vreg->low_vol_level, vreg->high_vol_level);

	if (!vreg->is_enabled) {
		if (vreg->set_voltage_sup) {
			rc = regulator_set_voltage(vreg->reg,
								vreg->low_vol_level,
								vreg->high_vol_level);
			if (rc < 0) {
				printk(KERN_ERR "[%s]:[%s] vreg_set_vol(%s) failed. rc=%d\n", TAG, __func__, vreg->name, rc);
				goto out;
			}
		}

		if (vreg->load_uA >= 0) {
			rc = regulator_set_load(vreg->reg,
						vreg->load_uA);
			if (rc < 0) {
				printk(KERN_ERR "[%s]:[%s] vreg_set_mode(%s) failed. rc=%d\n", TAG, __func__, vreg->name, rc);
				goto out;
			}
		}

		rc = regulator_enable(vreg->reg);
		if (rc < 0) {
			printk(KERN_ERR "[%s]:[%s] regulator_enable(%s) failed. rc=%d\n", TAG, __func__, vreg->name, rc);
			goto out;
		}
		vreg->is_enabled = true;
	}
out:
	return rc;
}

static int vled_vreg_disable(struct vled_power_vreg_data *vreg)
{
	int rc = 0;

	if (!vreg)
		return rc;

	printk(KERN_ERR "[%s]:[%s] vreg_en for : %s \n", TAG, __func__, vreg->name);

	if (vreg->is_enabled) {
		rc = regulator_disable(vreg->reg);
		if (rc < 0) {
			printk(KERN_ERR "[%s]:[%s] regulator_disable(%s) failed. rc=%d\n", TAG, __func__, vreg->name, rc);
			goto out;
		}
		vreg->is_enabled = false;

		if (vreg->set_voltage_sup) {
			/* Set the min voltage to 0 */
			rc = regulator_set_voltage(vreg->reg, 0,
				vreg->high_vol_level);
			if (rc < 0) {
				printk(KERN_ERR "[%s]:[%s] vreg_set_vol(%s) failed. rc=%d\n", TAG, __func__, vreg->name, rc);
				goto out;
			}
		}
		if (vreg->load_uA >= 0) {
			rc = regulator_set_load(vreg->reg, 0);
			if (rc < 0) {
				printk(KERN_ERR "[%s]:[%s] vreg_set_mode(%s) failed. rc=%d\n", TAG, __func__, vreg->name, rc);
			}
		}
	}
out:
	return rc;
}

static int vled_configure_vreg(struct platform_device *pdev, struct vled_power_vreg_data *vreg)
{
	int rc = 0;

	printk(KERN_ERR "[%s]:[%s] config : %s \n", TAG, __func__, vreg->name);
	/* Get the regulator handle for vreg */
	if (!(vreg->reg)) {
		rc = vled_vreg_init(pdev, vreg);
		if (rc < 0)
			return rc;
	}
	rc = vled_vreg_enable(vreg);

	return rc;
}



static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
							const char *buf, size_t length)
{
	unsigned long val;
	struct vledpower_dev *sdev = (struct vledpower_dev *)
								dev_get_drvdata(dev);
	struct vled_power_vledpower_data *vledpower_data =
		container_of(sdev, struct vled_power_vledpower_data, sdev);

	if (!attr || !dev || !buf)
		return -EINVAL;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	vledpower_data->enable = (int)val;
	if (val == 1) {
		vled_vreg_enable(vledpower_data->sensor_vled_vdd);
	} else {
		vled_vreg_disable(vledpower_data->sensor_vled_vdd);
	}

	return length;
}
#define MAX_PROP_SIZE 32
static int vled_dt_parse_vreg_info(struct device *dev,
		struct vled_power_vreg_data **vreg_data, const char *vreg_name)
{
	int len, ret = 0;
	const __be32 *prop;
	char prop_name[MAX_PROP_SIZE];
	struct vled_power_vreg_data *vreg;
	struct device_node *np = dev->of_node;

	printk(KERN_ERR "[%s]:[%s] vreg dev tree parse fo : %s \n", TAG, __func__, vreg_name);
	*vreg_data = NULL;
	snprintf(prop_name, MAX_PROP_SIZE, "%s-supply", vreg_name);
	if (of_parse_phandle(np, prop_name, 0)) {
		vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
		if (!vreg) {
			printk(KERN_ERR "[%s]:[%s] No memory for vreg: %s \n", TAG, __func__, vreg_name);
			ret = -ENOMEM;
			goto err;
		}

		vreg->name = vreg_name;

		/* Parse voltage-level from each node */
		snprintf(prop_name, MAX_PROP_SIZE,
				"%s-voltage-level", vreg_name);
		prop = of_get_property(np, prop_name, &len);
		if (!prop || (len != (2 * sizeof(__be32)))) {
			printk(KERN_ERR "[%s]:[%s] %s %s property\n", TAG, __func__, prop ? "invalid format" : "no", prop_name);
		} else {
			vreg->low_vol_level = be32_to_cpup(&prop[0]);
			vreg->high_vol_level = be32_to_cpup(&prop[1]);
		}

		/* Parse current-level from each node */
		snprintf(prop_name, MAX_PROP_SIZE,
				"%s-current-level", vreg_name);
		ret = of_property_read_u32(np, prop_name, &vreg->load_uA);
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] %s property is not valid\n", TAG, __func__, prop_name);
			vreg->load_uA = -1;
			ret = 0;
		}

		*vreg_data = vreg;
		printk(KERN_ERR "[%s]:[%s] %s: vol=[%d %d]uV, current=[%d]uA\n", TAG, __func__,
			vreg->name, vreg->low_vol_level,
			vreg->high_vol_level,
			vreg->load_uA);
	} else{
		printk(KERN_ERR "[%s]:[%s] %s: is not provided in device tree\n", TAG, __func__, vreg_name);
	}

err:
	return ret;
}
static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
						   char *buf)
{
	struct vledpower_dev *sdev = (struct vledpower_dev *)
								dev_get_drvdata(dev);
	struct vled_power_vledpower_data *vledpower_data =
		container_of(sdev, struct vled_power_vledpower_data, sdev);

	return snprintf(buf, 10, "enable=%d\n\r", vledpower_data->enable);
}

static DEVICE_ATTR(enable, S_IRWXU, enable_show, enable_store);


static int vled_power_vledpower_probe(struct platform_device *pdev)
{

	struct vled_power_vledpower_data *vledpower_data;
	int ret = 0;
	/*enum of_gpio_flags flags;*/

	printk(KERN_ERR "[%s]:[%s] enter\n", TAG, __func__);
	vledpower_data = kzalloc(sizeof(struct vled_power_vledpower_data), GFP_KERNEL);
	if (!vledpower_data) {
		printk(KERN_ERR "[%s]:[%s] error 1\n", TAG, __func__);
		return -ENOMEM;
	}
	if (pdev->dev.of_node) {
		printk(KERN_ERR "[%s]:[%s] pdev->dev.of_node\n", TAG, __func__);
		ret = vled_dt_parse_vreg_info(&pdev->dev,
					&vledpower_data->sensor_vled_vdd,
					"sensor-vdd");
		if (ret < 0) {
			printk(KERN_ERR "[%s]:[%s] sensor-vdd not provided in device tree %d\n", TAG, __func__, ret);
			goto err_vledpower_parse_vreg;
		}
	}

	ret = vled_configure_vreg(pdev, vledpower_data->sensor_vled_vdd);
	if (ret < 0) {
		printk(KERN_ERR "[%s]:[%s] sensor-vdd enable fail %d\n", TAG, __func__, ret);
		goto err_vledpower_parse_vreg;
	}
	vledpower_data->sdev.name = "vled-power";
	vledpower_data->enable = -1;


	ret = vledpower_dev_register(&vledpower_data->sdev);
	if (ret < 0) {
		printk(KERN_ERR "vledpower[%s]:[%s] error 2 ret = %d\n", TAG, __func__, ret);
		goto err_vledpower_dev_register;

	}

	ret = device_create_file(vledpower_data->sdev.dev, &dev_attr_enable);
	if (ret < 0) {
		printk(KERN_ERR "[%s]:[%s] error 3 ret = %d\n", TAG, __func__, ret);
		goto err_vledpower_dev_register;

	}

	printk(KERN_ERR "[%s]:[%s] success\n", TAG, __func__);

	return 0;




err_vledpower_dev_register:
   regulator_put(vledpower_data->sensor_vled_vdd->reg);
err_vledpower_parse_vreg:
	kfree(vledpower_data);

	return ret;
}

static int vled_power_vledpower_remove(struct platform_device *pdev)
{
	struct vled_power_vledpower_data *vledpower_data = platform_get_drvdata(pdev);

	vledpower_dev_unregister(&vledpower_data->sdev);
	if (vledpower_data->sensor_vled_vdd)
		regulator_put(vledpower_data->sensor_vled_vdd->reg);
	kfree(vledpower_data);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id vled_power_match_table[] = {
	 { .compatible = "vled-power",},
	 {},
};
#endif

static struct platform_driver vled_power_vledpower_driver = {
	.probe		= vled_power_vledpower_probe,
	.remove		= vled_power_vledpower_remove,
	.driver		=  {
		.name		= "vled-power",
		.owner		= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = vled_power_match_table,
#endif
	},
};

static int __init vled_power_vledpower_init(void)
{
	printk(KERN_ERR "vledpower[%s]:[%s] enter\n", TAG, __func__);
	return platform_driver_register(&vled_power_vledpower_driver);
}

static void __exit vled_power_vledpower_exit(void)
{
	printk(KERN_ERR "vledpower[%s]:[%s] enter\n", TAG, __func__);
	platform_driver_unregister(&vled_power_vledpower_driver);
}

module_init(vled_power_vledpower_init);
module_exit(vled_power_vledpower_exit);

MODULE_AUTHOR("sensor xuguodong@vivo.com.cn");
MODULE_DESCRIPTION("VLED POWER vledpower driver");
MODULE_LICENSE("GPL");
