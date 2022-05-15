/*
 * add drivers/prox_vled_power/vledpower_class.c
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
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/vledpower.h>

struct class *vledpower_class;
//static atomic_t device_count;

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
						 char *buf)
{
	struct vledpower_dev *sdev = (struct vledpower_dev *)
								dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n", sdev->name);
}
static DEVICE_ATTR(name, S_IRUGO, name_show, NULL);
static int create_vledpower_class(void)
{
	printk(KERN_ERR "vledpower: create_vledpower_class\n");
	if (!vledpower_class) {
		vledpower_class = class_create(THIS_MODULE, "vledpower");
		if (IS_ERR(vledpower_class))
			return PTR_ERR(vledpower_class);
		//atomic_set(&device_count, 0);
	}

	return 0;
}
int vledpower_dev_register(struct vledpower_dev *sdev)
{
	int ret;
	printk(KERN_ERR "vledpower: vledpower_dev_register\n");
	if (!vledpower_class) {
		printk(KERN_ERR "vledpower: vledpower_dev_register 11111111\n");
		ret = create_vledpower_class();
		if (ret < 0)
			return ret;
	}
	sdev->dev = device_create(vledpower_class, NULL,
							  MKDEV(0, 0), NULL, sdev->name);
	if (IS_ERR(sdev->dev))
		return PTR_ERR(sdev->dev);
	ret = device_create_file(sdev->dev, &dev_attr_name);
	if (ret < 0)
		goto err_create_file_1;
	dev_set_drvdata(sdev->dev, sdev);
	return 0;
err_create_file_1:
	device_destroy(vledpower_class, MKDEV(0, 0));
	printk(KERN_ERR "vledpower: Failed to register driver %s\n", sdev->name);
	return ret;
}
EXPORT_SYMBOL_GPL(vledpower_dev_register);
void vledpower_dev_unregister(struct vledpower_dev *sdev)
{
	printk(KERN_ERR "vledpower: vledpower_dev_unregister\n");
	device_destroy(vledpower_class, MKDEV(0, 0));
	dev_set_drvdata(sdev->dev, NULL);
}
EXPORT_SYMBOL_GPL(vledpower_dev_unregister);
static int __init vledpower_class_init(void)
{
	return create_vledpower_class();
}
static void __exit vledpower_class_exit(void)
{
	class_destroy(vledpower_class);
}
module_init(vledpower_class_init);
module_exit(vledpower_class_exit);
MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("vledpower class driver");
MODULE_LICENSE("GPL");
