/*
 *	sarpower class driver
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

#ifndef __LINUX_vledpower_H__
#define __LINUX_vledpower_H__

struct vled_power_vreg_data {
	/* voltage regulator handle */
	struct regulator *reg;
	/* regulator name */
	const char *name;
	/* voltage levels to be set */
	unsigned int low_vol_level;
	unsigned int high_vol_level;
	/* current level to be set */
	unsigned int load_uA;
	
	bool set_voltage_sup;
	/* is this regulator enabled? */
	bool is_enabled;
};
struct vledpower_dev {
	const char	*name;
	struct device	*dev;
	int		index;
};

extern int vledpower_dev_register(struct vledpower_dev *sdev);
extern void vledpower_dev_unregister(struct vledpower_dev *sdev);


#endif /* __LINUX_vledpower_H__ */
