/*
 * Copyright (c) 2014-2015, 2017-2019, The Linux Foundation.
 * All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spmi.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>

#define PMIC_REV2		0x101
#define PMIC_REV3		0x102
#define PMIC_REV4		0x103
#define PMIC_TYPE		0x104
#define PMIC_SUBTYPE		0x105

#define PMIC_TYPE_VALUE		0x51

#define COMMON_SUBTYPE		0x00
#define PM8941_SUBTYPE		0x01
#define PM8841_SUBTYPE		0x02
#define PM8019_SUBTYPE		0x03
#define PM8226_SUBTYPE		0x04
#define PM8110_SUBTYPE		0x05
#define PMA8084_SUBTYPE		0x06
#define PMI8962_SUBTYPE		0x07
#define PMD9635_SUBTYPE		0x08
#define PM8994_SUBTYPE		0x09
#define PMI8994_SUBTYPE		0x0a
#define PM8916_SUBTYPE		0x0b
#define PM8004_SUBTYPE		0x0c
#define PM8909_SUBTYPE		0x0d

static const struct of_device_id pmic_spmi_id_table[] = {
	{ .compatible = "qcom,spmi-pmic", .data = (void *)COMMON_SUBTYPE },
	{ .compatible = "qcom,pm8941",    .data = (void *)PM8941_SUBTYPE },
	{ .compatible = "qcom,pm8841",    .data = (void *)PM8841_SUBTYPE },
	{ .compatible = "qcom,pm8019",    .data = (void *)PM8019_SUBTYPE },
	{ .compatible = "qcom,pm8226",    .data = (void *)PM8226_SUBTYPE },
	{ .compatible = "qcom,pm8110",    .data = (void *)PM8110_SUBTYPE },
	{ .compatible = "qcom,pma8084",   .data = (void *)PMA8084_SUBTYPE },
	{ .compatible = "qcom,pmi8962",   .data = (void *)PMI8962_SUBTYPE },
	{ .compatible = "qcom,pmd9635",   .data = (void *)PMD9635_SUBTYPE },
	{ .compatible = "qcom,pm8994",    .data = (void *)PM8994_SUBTYPE },
	{ .compatible = "qcom,pmi8994",   .data = (void *)PMI8994_SUBTYPE },
	{ .compatible = "qcom,pm8916",    .data = (void *)PM8916_SUBTYPE },
	{ .compatible = "qcom,pm8004",    .data = (void *)PM8004_SUBTYPE },
	{ .compatible = "qcom,pm8909",    .data = (void *)PM8909_SUBTYPE },
	{ }
};

static void pmic_spmi_show_revid(struct regmap *map, struct device *dev)
{
	unsigned int rev2, minor, major, type, subtype;
	const char *name = "unknown";
	int ret, i;

	ret = regmap_read(map, PMIC_TYPE, &type);
	if (ret < 0)
		return;

	if (type != PMIC_TYPE_VALUE)
		return;

	ret = regmap_read(map, PMIC_SUBTYPE, &subtype);
	if (ret < 0)
		return;

	for (i = 0; i < ARRAY_SIZE(pmic_spmi_id_table); i++) {
		if (subtype == (unsigned long)pmic_spmi_id_table[i].data)
			break;
	}

	if (i != ARRAY_SIZE(pmic_spmi_id_table))
		name = pmic_spmi_id_table[i].compatible;

	ret = regmap_read(map, PMIC_REV2, &rev2);
	if (ret < 0)
		return;

	ret = regmap_read(map, PMIC_REV3, &minor);
	if (ret < 0)
		return;

	ret = regmap_read(map, PMIC_REV4, &major);
	if (ret < 0)
		return;

	/*
	 * In early versions of PM8941 and PM8226, the major revision number
	 * started incrementing from 0 (eg 0 = v1.0, 1 = v2.0).
	 * Increment the major revision number here if the chip is an early
	 * version of PM8941 or PM8226.
	 */
	if ((subtype == PM8941_SUBTYPE || subtype == PM8226_SUBTYPE) &&
	    major < 0x02)
		major++;

	if (subtype == PM8110_SUBTYPE)
		minor = rev2;

	dev_dbg(dev, "%x: %s v%d.%d\n", subtype, name, major, minor);
}

static const struct regmap_config spmi_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
	.fast_io	= true,
};

static const struct regmap_config spmi_regmap_can_sleep_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
	.fast_io	= false,
};


/* vivo add for spmi_class(debugfs spmi operations) */
struct spmi_class_dev {
	struct class	spmi_class;
	struct regmap	*regmap;
	u16				address;
	u16				count;
};


static ssize_t data_store(struct class *c, struct class_attribute *attr,
						const char *buf, size_t count)
{
	struct spmi_class_dev *scdev = container_of(c, struct spmi_class_dev,
												spmi_class);
	u8 val = 0;

	if (kstrtou8(buf, 0, &val)) {
		pr_err("spmi_class: failed to convert data in buffer to u8 value\n");
		return -EINVAL;
	}

	regmap_write(scdev->regmap, scdev->address, val);

	return count;
}
#define REG_DATA_SIZE               224
static ssize_t data_show(struct class *c, struct class_attribute *attr,
						char *buf)
{
	struct spmi_class_dev *scdev = container_of(c, struct spmi_class_dev,
												spmi_class);
	char temp[REG_DATA_SIZE];
	unsigned int val = 0;
	int i, rc, pos = 0;
	u16 count = min(min(scdev->count, (u16)0xF), (u16)0xFFFF-scdev->address); //max 0xF bytes

	for (i = 0; i <= count; i++) {
		regmap_read(scdev->regmap, scdev->address + i, &val);
		rc = snprintf(temp + pos, REG_DATA_SIZE, "0x%04x: 0x%02x\n", scdev->address + i, val);
		pos += rc;
		if (pos >= (REG_DATA_SIZE - 9))
			break;
	}

	return scnprintf(buf, REG_DATA_SIZE, "%s", temp);
}
static CLASS_ATTR_RW(data);


static ssize_t count_store(struct class *c, struct class_attribute *attr,
							const char *buf, size_t count)
{
	struct spmi_class_dev *scdev = container_of(c, struct spmi_class_dev,
												spmi_class);

	if (kstrtou16(buf, 0, &scdev->count))
		return -EINVAL;

	return count;
}
static ssize_t count_show(struct class *c, struct class_attribute *attr,
							char *buf)
{
	struct spmi_class_dev *scdev = container_of(c, struct spmi_class_dev,
												spmi_class);

	return scnprintf(buf, 8, "%d\n", scdev->count);
}
static CLASS_ATTR_RW(count);


static ssize_t address_store(struct class *c, struct class_attribute *attr,
							const char *buf, size_t count)
{

	struct spmi_class_dev *scdev = container_of(c, struct spmi_class_dev,
												spmi_class);

	if (kstrtou16(buf, 0, &scdev->address))
		return -EINVAL;

	return count;

}
static ssize_t address_show(struct class *c, struct class_attribute *attr,
							char *buf)

{
	struct spmi_class_dev *scdev = container_of(c, struct spmi_class_dev,
												spmi_class);

	return scnprintf(buf, 8, "0x%x\n", scdev->address);
}
static CLASS_ATTR_RW(address);


static struct attribute *spmi_class_attrs[] = {
	&class_attr_address.attr,
	&class_attr_count.attr,
	&class_attr_data.attr,
	NULL,
};
ATTRIBUTE_GROUPS(spmi_class);

/* vivo add for spmi_class(debugfs spmi operations) end */

static int pmic_spmi_probe(struct spmi_device *sdev)
{
	struct device_node *root = sdev->dev.of_node;
	struct regmap *regmap;
	/* vivo add for spmi_class(debugfs spmi operations) */
	struct spmi_class_dev *scdev;
	int rc;
	/* vivo add for spmi_class(debugfs spmi operations) end */

	if (of_property_read_bool(root, "qcom,can-sleep"))
		regmap = devm_regmap_init_spmi_ext(sdev,
						&spmi_regmap_can_sleep_config);
	else
		regmap = devm_regmap_init_spmi_ext(sdev, &spmi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Only the first slave id for a PMIC contains this information */
	if (sdev->usid % 2 == 0)
		pmic_spmi_show_revid(regmap, &sdev->dev);

	/* vivo add for spmi_class(debugfs spmi operations) */
	scdev = devm_kzalloc(&sdev->dev, sizeof(*scdev), GFP_KERNEL);

	if (!scdev)
		pr_err("Failed to alloc spmi class\n");
	else {
		scdev->regmap = regmap;
		scdev->spmi_class.name = dev_name(&sdev->dev);
		scdev->spmi_class.class_groups = spmi_class_groups;
		rc = class_register(&scdev->spmi_class);
		if (rc < 0)
			pr_err("Failed to create spmi class rc=%d\n", rc);
	}
	/* vivo add for spmi_class(debugfs spmi operations) end */

	return devm_of_platform_populate(&sdev->dev);
}

MODULE_DEVICE_TABLE(of, pmic_spmi_id_table);

static struct spmi_driver pmic_spmi_driver = {
	.probe = pmic_spmi_probe,
	.driver = {
		.name = "pmic-spmi",
		.of_match_table = pmic_spmi_id_table,
	},
};

static int __init pmic_spmi_init(void)
{
	return spmi_driver_register(&pmic_spmi_driver);
}
arch_initcall(pmic_spmi_init);

MODULE_DESCRIPTION("Qualcomm SPMI PMIC driver");
MODULE_ALIAS("spmi:spmi-pmic");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Josh Cartwright <joshc@codeaurora.org>");
MODULE_AUTHOR("Stanimir Varbanov <svarbanov@mm-sol.com>");
