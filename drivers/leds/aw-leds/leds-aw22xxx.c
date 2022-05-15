/*
 * leds-aw22xxx.c	aw22xxx led module
 *
 * version: v1.0.5
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 *
 *	Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
//#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/leds.h>
#include "leds-aw22xxx.h"
#include "leds-aw22xxx-reg.h"
/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW22XXX_I2C_NAME "aw22xxx_led"

#define AW22XXX_VERSION "v1.0.6"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1
#define ERROR_CODE_ONE 1
#define ERROR_CODE_TWO 2


/******************************************************
 *
 * aw22xxx led parameter
 *
 ******************************************************/
#define AW22XXX_CFG_NAME_MAX		64
static char *aw22xxx_fw_name = "aw22xxx_fw.bin";
static char *aw22xxx_fw_test_name = "aw22xxx_fw_test.bin";
//struct aw22xxx *aw22xxx_a;
//EXPORT_SYMBOL(aw22xxx_a);
static char aw22xxx_cfg_name[][AW22XXX_CFG_NAME_MAX] = {
	
	{"aw22xxx_cfg_led_00.bin"},
	{"aw22xxx_cfg_led_01.bin"},
	{"aw22xxx_cfg_led_02.bin"},
	{"aw22xxx_cfg_led_03.bin"},
	{"aw22xxx_cfg_led_04.bin"},
	{"aw22xxx_cfg_led_05.bin"},
	{"aw22xxx_cfg_led_06.bin"},
	{"aw22xxx_cfg_led_07.bin"},
	/* On - All White foCamera People Mode */
	{"aw22xxx_cfg_led_08.bin"},
	/* On - Rainbow for mera People Mode */
	{"aw22xxx_cfg_led_09.bin"},
	/* On - Orange for Cera People Mode */
	{"aw22xxx_cfg_led_10.bin"},
	/* On - All White foCamera Fill Light */
	{"aw22xxx_cfg_led_11.bin"},
	/* Breath - White foCamera Countdown */
	{"aw22xxx_cfg_led_12.bin"},
	/* Breath - Blue foramera Countdown */
	{"aw22xxx_cfg_led_13.bin"},
	/* Breath - Purple f Camera Countdown */
	{"aw22xxx_cfg_led_14.bin"},
	{"aw22xxx_cfg_led_15.bin"},
	{"aw22xxx_cfg_led_16.bin"},
	{"aw22xxx_cfg_led_17.bin"},
	{"aw22xxx_cfg_led_18.bin"},
	{"aw22xxx_cfg_led_19.bin"},
	{"aw22xxx_cfg_led_20.bin"},
	{"aw22xxx_cfg_led_21.bin"},
	{"aw22xxx_cfg_led_22.bin"},
	{"aw22xxx_cfg_led_23.bin"},
	{"aw22xxx_cfg_led_24.bin"},
	{"aw22xxx_cfg_led_25.bin"},
	{"aw22xxx_cfg_led_26.bin"},
	{"aw22xxx_cfg_led_27.bin"},
	{"aw22xxx_cfg_led_28.bin"},
	{"aw22xxx_cfg_led_29.bin"},
	{"aw22xxx_cfg_led_30.bin"},
	{"aw22xxx_cfg_audio_00.bin"},
	{"aw22xxx_cfg_audio_01.bin"},
	{"aw22xxx_cfg_audio_02.bin"},
	{"aw22xxx_cfg_audio_03.bin"},
	{"aw22xxx_cfg_audio_04.bin"},
	{"aw22xxx_cfg_audio_05.bin"},
	{"aw22xxx_cfg_audio_06.bin"},
	{"aw22xxx_cfg_audio_07.bin"},
	{"aw22xxx_cfg_audio_08.bin"},
	{"aw22xxx_cfg_audio_09.bin"},
	{"aw22xxx_cfg_audio_10.bin"},
	{"aw22xxx_cfg_audio_11.bin"},
	{"aw22xxx_cfg_audio_12.bin"},
	{"aw22xxx_cfg_audio_13.bin"},
	{"aw22xxx_cfg_audio_14.bin"},
	{"aw22xxx_cfg_audio_15.bin"},
	{"aw22xxx_cfg_factory_00.bin"},
	{"aw22xxx_cfg_factory_01.bin"},
	{"aw22xxx_cfg_factory_02.bin"},
	{"aw22xxx_cfg_factory_03.bin"},
	{"aw22xxx_cfg_factory_04.bin"},
	{"aw22xxx_cfg_factory_05.bin"},
	{"aw22xxx_cfg_factory_06.bin"},
	{"aw22xxx_cfg_factory_07.bin"},
	{"aw22xxx_cfg_factory_08.bin"},
	{"aw22xxx_cfg_factory_09.bin"},
	{"aw22xxx_cfg_factory_10.bin"},
	{"aw22xxx_cfg_factory_11.bin"},
	{"aw22xxx_cfg_factory_12.bin"},
	{"aw22xxx_cfg_factory_13.bin"},
	{"aw22xxx_cfg_factory_14.bin"},
	{"aw22xxx_cfg_factory_15.bin"},
	{"aw22xxx_cfg_factory_16.bin"},
	{"aw22xxx_cfg_factory_17.bin"},
	{"aw22xxx_cfg_factory_18.bin"},
	{"aw22xxx_cfg_factory_19.bin"},
	{"aw22xxx_cfg_factory_20.bin"},
	{"aw22xxx_cfg_factory_21.bin"},
	{"aw22xxx_cfg_factory_22.bin"},
	{"aw22xxx_cfg_factory_23.bin"},
	{"aw22xxx_cfg_factory_24.bin"},
	{"aw22xxx_cfg_factory_25.bin"},
	{"aw22xxx_cfg_factory_26.bin"},
	{"aw22xxx_cfg_factory_27.bin"},
	{"aw22xxx_cfg_factory_28.bin"},
	{"aw22xxx_cfg_factory_29.bin"},
	{"aw22xxx_cfg_factory_30.bin"},
	{"aw22xxx_call_led_01.bin"},
	{"aw22xxx_msg_led_01.bin"},
	{"aw22xxx_msg_led_02.bin"},
	{"aw22xxx_msg_led_03.bin"},
	{"aw22xxx_msg_led_04.bin"},
	{"aw22xxx_msg_led_05.bin"},
	{"aw22xxx_charge_led_01.bin"},
	{"aw22xxx_chargefull_led_01.bin"},
	{"aw22xxx_calling_led_03.bin"},
	{"aw22xxx_music_led_01.bin"},
	{"aw22xxx_music_led_02.bin"},
	{"aw22xxx_music_led_03.bin"},
	{"aw22xxx_music_led_04.bin"},
	{"aw22xxx_music_led_05.bin"},
	{"aw22xxx_music_led_06.bin"},
	{"aw22xxx_music_led_07.bin"},
	{"aw22xxx_music_led_08.bin"},
	{"aw22xxx_music_led_09.bin"},
	{"aw22xxx_music_led_10.bin"},
	{"aw22xxx_music_led_11.bin"},
	{"aw22xxx_music_led_12.bin"},
	{"aw22xxx_music_led_13.bin"},
	{"aw22xxx_music_led_14.bin"},
	{"aw22xxx_music_led_15.bin"},
	{"aw22xxx_monster_led_01.bin"},
	{"aw22xxx_Jovi_led_01.bin"},
};

#define AW22XXX_IMAX_NAME_MAX		32
static char aw22xxx_imax_name[][AW22XXX_IMAX_NAME_MAX] = {
	{"AW22XXX_IMAX_2mA"},
	{"AW22XXX_IMAX_3mA"},
	{"AW22XXX_IMAX_4mA"},
	{"AW22XXX_IMAX_6mA"},
	{"AW22XXX_IMAX_9mA"},
	{"AW22XXX_IMAX_10mA"},
	{"AW22XXX_IMAX_15mA"},
	{"AW22XXX_IMAX_20mA"},
	{"AW22XXX_IMAX_30mA"},
	{"AW22XXX_IMAX_40mA"},
	{"AW22XXX_IMAX_45mA"},
	{"AW22XXX_IMAX_60mA"},
	{"AW22XXX_IMAX_75mA"},
};
static char aw22xxx_imax_code[] = {
	AW22XXX_IMAX_2mA,
	AW22XXX_IMAX_3mA,
	AW22XXX_IMAX_4mA,
	AW22XXX_IMAX_6mA,
	AW22XXX_IMAX_9mA,
	AW22XXX_IMAX_10mA,
	AW22XXX_IMAX_15mA,
	AW22XXX_IMAX_20mA,
	AW22XXX_IMAX_30mA,
	AW22XXX_IMAX_40mA,
	AW22XXX_IMAX_45mA,
	AW22XXX_IMAX_60mA,
	AW22XXX_IMAX_75mA,
};
/******************************************************
 *
 * aw22xxx i2c write/read
 *
 ******************************************************/
static int aw22xxx_i2c_write(struct aw22xxx *aw22xxx,
		 unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw22xxx->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw22xxx_i2c_read(struct aw22xxx *aw22xxx,
		unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw22xxx->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw22xxx_i2c_write_bits(struct aw22xxx *aw22xxx,
		 unsigned char reg_addr, unsigned char mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;

	aw22xxx_i2c_read(aw22xxx, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw22xxx_i2c_write(aw22xxx, reg_addr, reg_val);

	return 0;
}


#ifdef AW22XXX_FLASH_I2C_WRITES
static int aw22xxx_i2c_writes(struct aw22xxx *aw22xxx,
		unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len+1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return	-ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw22xxx->i2c, data, len+1);
	if (ret < 0) {
		pr_err("%s: i2c master send error\n", __func__);
	}

	kfree(data);

	return ret;
}

#endif

static int aw22xxx_i2c_reads(struct aw22xxx *aw22xxx,
		unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;

	ret = i2c_smbus_write_byte(aw22xxx->i2c, reg_addr);
	if (ret) {
		pr_err("%s: couldn't send request, ret=%d\n",
			__func__, ret);
		return ret;
	}

	ret = i2c_master_recv(aw22xxx->i2c, buf, len);
	if (ret != len) {
		pr_err("%s: couldn't read registers, return %d bytes\n",
			__func__,  ret);
		return ret;
	}

	return ret;
}
/*****************************************************
 *
 * aw22xxx led cfg
 *
 *****************************************************/
static int aw22xxx_reg_page_cfg(struct aw22xxx *aw22xxx, unsigned char page)
{
	aw22xxx_i2c_write(aw22xxx, REG_PAGE, page);
	return 0;
}

static int aw22xxx_sw_reset(struct aw22xxx *aw22xxx)
{
	aw22xxx_i2c_write(aw22xxx, REG_SRST, AW22XXX_SRSTW);
	msleep(2);
	return 0;
}

static int aw22xxx_chip_enable(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_GCR,
				BIT_GCR_CHIPEN_MASK, BIT_GCR_CHIPEN_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_GCR,
				BIT_GCR_CHIPEN_MASK, BIT_GCR_CHIPEN_DISABLE);
	}
	msleep(2);
	return 0;
}

/*
static int aw22xxx_mcu_wakeup(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_WAKEUP_MASK, BIT_MCUCTR_MCU_WAKEUP_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_WAKEUP_MASK, BIT_MCUCTR_MCU_WAKEUP_DISABLE);
	}
	return 0;
}
*/

static int aw22xxx_mcu_reset(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_RESET_MASK, BIT_MCUCTR_MCU_RESET_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_RESET_MASK, BIT_MCUCTR_MCU_RESET_DISABLE);
	}
	return 0;
}

static int aw22xxx_mcu_enable(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_WORK_MASK, BIT_MCUCTR_MCU_WORK_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_MCUCTR,
				BIT_MCUCTR_MCU_WORK_MASK, BIT_MCUCTR_MCU_WORK_DISABLE);
	}
	return 0;
}

static int aw22xxx_led_task0_cfg(struct aw22xxx *aw22xxx, unsigned char task)
{
	aw22xxx_i2c_write(aw22xxx, REG_TASK0, task);
	return 0;
}

static int aw22xxx_led_task1_cfg(struct aw22xxx *aw22xxx, unsigned char task)
{
	aw22xxx_i2c_write(aw22xxx, REG_TASK1, task);
	return 0;
}

static int aw22xxx_hw_reset(struct aw22xxx *aw22xxx)
{
	pr_info("%s enter.\n", __func__);

	if (aw22xxx && gpio_is_valid(aw22xxx->reset_gpio)) {
		gpio_set_value_cansleep(aw22xxx->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw22xxx->reset_gpio, 1);
		msleep(1);
	} else {
		dev_err(aw22xxx->dev, "%s:	failed\n", __func__);
	}
	pr_info("%s exit.\n", __func__);
	return 0;
}

/*
static int aw22xxx_get_pst(struct aw22xxx *aw22xxx, unsigned char *pst)
{
	int ret = -1;
	ret = aw22xxx_i2c_read(aw22xxx, REG_PST, pst);
	if (ret < 0) {
		pr_err("%s: error=%d\n", __func__, ret);
	}
	return ret;
}
*/

static int aw22xxx_imax_cfg(struct aw22xxx *aw22xxx, unsigned char imax)
{
	if (imax > 0x0f) {
		imax = 0x0f;
	}
	mutex_lock(&aw22xxx->cfg_lock);
	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_i2c_write(aw22xxx, REG_IMAX, imax);
	mutex_unlock(&aw22xxx->cfg_lock);
	return 0;
}

/*
static int aw22xxx_audio_enable(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_AUDCTR,
				BIT_AUDCTR_AUDEN_MASK, BIT_AUDCTR_AUDEN_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_AUDCTR,
				BIT_AUDCTR_AUDEN_MASK, BIT_AUDCTR_AUDEN_DISABLE);
	}
	return 0;
}

static int aw22xxx_agc_enable(struct aw22xxx *aw22xxx, bool flag)
{
	if (flag) {
		aw22xxx_i2c_write_bits(aw22xxx, REG_AUDCTR,
				BIT_AUDCTR_AGCEN_MASK, BIT_AUDCTR_AGCEN_ENABLE);
	} else {
		aw22xxx_i2c_write_bits(aw22xxx, REG_AUDCTR,
				BIT_AUDCTR_AGCEN_MASK, BIT_AUDCTR_AGCEN_DISABLE);
	}
	return 0;
}

static int aw22xxx_agc_igain_cfg(struct aw22xxx *aw22xxx, unsigned char igain)
{
	if (igain > 0x3f) {
		igain = 0x3f;
	}
	aw22xxx_i2c_write(aw22xxx, REG_IGAIN, igain);

	return 0;
}

static int aw22xxx_get_agc_gain(struct aw22xxx *aw22xxx, unsigned char *gain)
{
	int ret = -1;
	ret = aw22xxx_i2c_read(aw22xxx, REG_GAIN, gain);
	if (ret < 0) {
		pr_err("%s: error=%d\n", __func__, ret);
	}
	return ret;
}
*/

static int aw22xxx_dbgctr_cfg(struct aw22xxx *aw22xxx, unsigned char cfg)
{
	if (cfg >= (AW22XXX_DBGCTR_MAX-1)) {
		cfg = AW22XXX_DBGCTR_NORMAL;
	}
	aw22xxx_i2c_write(aw22xxx, REG_DBGCTR, cfg);

	return 0;
}

static int aw22xxx_addr_cfg(struct aw22xxx *aw22xxx, unsigned int addr)
{
	aw22xxx_i2c_write(aw22xxx, REG_ADDR1, (unsigned char)((addr>>0)&0xff));
	aw22xxx_i2c_write(aw22xxx, REG_ADDR2, (unsigned char)((addr>>8)&0xff));

	return 0;
}

static int aw22xxx_data_cfg(struct aw22xxx *aw22xxx, unsigned int data)
{
	aw22xxx_i2c_write(aw22xxx, REG_DATA, data);

	return 0;
}
/*
static int aw22xxx_get_fw_version(struct aw22xxx *aw22xxx)
{
	unsigned char reg_val = 0;
	unsigned char i = 0;

	pr_info("%s: enter\n", __func__);
	aw22xxx_hw_reset(aw22xxx);
	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_mcu_enable(aw22xxx, true);
	aw22xxx_led_task0_cfg(aw22xxx, 0xff);
	aw22xxx_mcu_reset(aw22xxx, false);
	msleep(2);
	aw22xxx_mcu_reset(aw22xxx, true);

	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE2);
	aw22xxx->fw_version = 0;
	for (i = 0; i < 4; i++) {
		aw22xxx_i2c_read(aw22xxx, 0xe9+i, &reg_val);
		aw22xxx->fw_version |= (reg_val<<((3-i)*8));
	}
	pr_info("%s: flash fw version: 0x%04x\n", __func__, aw22xxx->fw_version);

	aw22xxx_i2c_read(aw22xxx, 0xd0, &reg_val);
	switch (reg_val) {
	case AW22118_CHIPID:
		aw22xxx->chipid = AW22118;
		pr_info("%s: flash chipid: aw22118\n", __func__);
		break;
	case AW22127_CHIPID:
		aw22xxx->chipid = AW22127;
		pr_info("%s: flash chipid: aw22127\n", __func__);
		break;
	default:
		pr_err("%s: unknown id=0x%02x\n", __func__, reg_val);
	}

	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);

	return 0;
}
*/
static int aw22xxx_led_display(struct aw22xxx *aw22xxx)
{
	aw22xxx_addr_cfg(aw22xxx, 0x00e1);
	aw22xxx_dbgctr_cfg(aw22xxx, AW22XXX_DBGCTR_SFR);
	aw22xxx_data_cfg(aw22xxx, 0x3d);
	aw22xxx_dbgctr_cfg(aw22xxx, AW22XXX_DBGCTR_NORMAL);
	return 0;
}

static int aw22xxx_led_off(struct aw22xxx *aw22xxx)
{
	aw22xxx_led_task0_cfg(aw22xxx, 0xff);
	aw22xxx_mcu_reset(aw22xxx, true);
	return 0;
}

static void aw22xxx_brightness_work(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx,
		  brightness_work);

	pr_info("%s enter\n", __func__);

	aw22xxx_led_off(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, false);
	if (aw22xxx->cdev.brightness) {
		aw22xxx_chip_enable(aw22xxx, true);
		aw22xxx_mcu_enable(aw22xxx, true);

		aw22xxx_imax_cfg(aw22xxx, (unsigned char)aw22xxx->imax);
		aw22xxx_led_display(aw22xxx);

		aw22xxx_led_task0_cfg(aw22xxx, 0x82);
		aw22xxx_mcu_reset(aw22xxx, false);
	}
}

static void aw22xxx_set_brightness(struct led_classdev *cdev,
		   enum led_brightness brightness)
{
	struct aw22xxx *aw22xxx = container_of(cdev, struct aw22xxx, cdev);

	aw22xxx->cdev.brightness = brightness;

	schedule_work(&aw22xxx->brightness_work);
}

static void aw22xxx_task_work(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx,
		  task_work);

	pr_info("%s enter\n", __func__);

	aw22xxx_led_off(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, false);
	if (aw22xxx->task0) {
		aw22xxx_chip_enable(aw22xxx, true);
		aw22xxx_mcu_enable(aw22xxx, true);

		aw22xxx_imax_cfg(aw22xxx, (unsigned char)aw22xxx->imax);
		aw22xxx_led_display(aw22xxx);

		aw22xxx_led_task0_cfg(aw22xxx, aw22xxx->task0);
		aw22xxx_led_task1_cfg(aw22xxx, aw22xxx->task1);
		aw22xxx_mcu_reset(aw22xxx, false);
	}
}

static int aw22xxx_led_init(struct aw22xxx *aw22xxx)
{
	pr_info("%s: enter\n", __func__);

	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_imax_cfg(aw22xxx, aw22xxx_imax_code[aw22xxx->imax]);
	//aw22xxx_i2c_write(aw22xxx, 0x03, 0x20);
	aw22xxx_chip_enable(aw22xxx, false);

	pr_info("%s: exit\n", __func__);

	return 0;
}

static void aw22xxx_trim_work(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx,
		  trim_work);
	//unsigned char flash_data[512];
	unsigned int i = 0;
	unsigned int tmp_addr = 0;

	pr_info("%s: enter\n", __func__);
	mutex_lock(&aw22xxx->trim_lock);
#if 0
	/* read flash */
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);

	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < 512; i++) {
		tmp_addr = 0x4400 + i;
		aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(tmp_addr>>8));
		aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(tmp_addr>>0));
		aw22xxx_i2c_read(aw22xxx, 0x23, &flash_data[i]);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
	pr_info("%s: flash orignal data:\n", __func__);
	for (i = 0; i < 512; i += 16) {
		pr_info("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n", flash_data[i+0], flash_data[i+1], flash_data[i+2], flash_data[i+3], flash_data[i+4], flash_data[i+5], flash_data[i+6], flash_data[i+7], flash_data[i+8], flash_data[i+9], flash_data[i+10], flash_data[i+11], flash_data[i+12], flash_data[i+13], flash_data[i+14], flash_data[i+15]);
	}
	pr_info("\n");
#endif

	/* earse flash */
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);

	aw22xxx_i2c_write(aw22xxx, 0x22, 0x44);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(6);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);


	/* trim flash */
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);


	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < 512; i++) {
		tmp_addr = 0x4400 + i;
		aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(tmp_addr>>8));
		aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(tmp_addr>>0));
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
		aw22xxx_i2c_write(aw22xxx, 0x23, aw22xxx->trim_data[i]);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);


	/* check flash */
#if 0
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);

	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < 512; i++) {
		tmp_addr = 0x4400 + i;
		aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(tmp_addr>>8));
		aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(tmp_addr>>0));
		aw22xxx_i2c_read(aw22xxx, 0x23, &flash_data[i]);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
	pr_info("%s: flash trim data:\n", __func__);
	for (i = 0; i < 512; i += 16) {
		pr_info("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n", flash_data[i+0], flash_data[i+1], flash_data[i+2], flash_data[i+3], flash_data[i+4], flash_data[i+5], flash_data[i+6], flash_data[i+7], flash_data[i+8], flash_data[i+9], flash_data[i+10], flash_data[i+11], flash_data[i+12], flash_data[i+13], flash_data[i+14], flash_data[i+15]);
	}
	pr_info("\n");
#endif


	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	mutex_unlock(&aw22xxx->trim_lock);
	pr_info("%s: exit\n", __func__);				
}

/*****************************************************
 *
 * firmware/cfg update
 *
 *****************************************************/
static void aw22xxx_cfg_loaded(const struct firmware *cont, void *context)
{
	struct aw22xxx *aw22xxx = context;
	int i = 0;
	unsigned char page = 0;
	unsigned char reg_addr = 0;
	unsigned char reg_val = 0;

	pr_info("%s: enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw22xxx_cfg_name[aw22xxx->effect]);
		release_firmware(cont);
		mutex_unlock(&aw22xxx->cfg_lock);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw22xxx_cfg_name[aw22xxx->effect],
					cont ? cont->size : 0);
/*
	for (i = 0; i < cont->size; i++) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
	}
*/
	for (i = 0; i < cont->size; i += 2) {
		if (aw22xxx->cfg == 1) {
			aw22xxx_i2c_write(aw22xxx, *(cont->data+i), *(cont->data+i+1));
			pr_info("%s: addr:0x%02x, data:0x%02x\n", __func__, *(cont->data+i), *(cont->data+i+1));
		} else {
			if (*(cont->data+i) == 0xff) {
				page = *(cont->data+i+1);
			}
			if (page == 1) {
				reg_addr = *(cont->data+i);
				if ((reg_addr < 0x2b) && (reg_addr > 0x0f)) {
					reg_addr -= 0x10;
					reg_val = (unsigned char)(((aw22xxx->rgb[reg_addr/3])>>(8*(2-reg_addr%3)))&0xff);
					aw22xxx_i2c_write(aw22xxx, *(cont->data+i), reg_val);
					pr_info("%s: addr:0x%02x, data:0x%02x\n", __func__, *(cont->data+i), reg_val);
				} else {
					aw22xxx_i2c_write(aw22xxx, *(cont->data+i), *(cont->data+i+1));
					pr_info("%s: addr:0x%02x, data:0x%02x\n", __func__, *(cont->data+i), *(cont->data+i+1));
				}
			} else {
				aw22xxx_i2c_write(aw22xxx, *(cont->data+i), *(cont->data+i+1));
				if ((*(cont->data+i) == 0x05) && (*(cont->data+i+1) == 0xfe)) {
					msleep(2);
				}
				pr_info("%s: addr:0x%02x, data:0x%02x\n", __func__, *(cont->data+i), *(cont->data+i+1));
			}
		}
	}

	release_firmware(cont);

	pr_info("%s: cfg update complete\n", __func__);
	mutex_unlock(&aw22xxx->cfg_lock);
}

static int aw22xxx_cfg_update(struct aw22xxx *aw22xxx)
{
	int ret;
	pr_info("%s: enter\n", __func__);

	if (aw22xxx->effect < (sizeof(aw22xxx_cfg_name)/AW22XXX_CFG_NAME_MAX)) {
		pr_info("%s: cfg name=%s\n", __func__, aw22xxx_cfg_name[aw22xxx->effect]);
	} else {
		pr_err("%s: effect 0x%02x over max value \n", __func__, aw22xxx->effect);
		return -ERROR_CODE_ONE;
	}

	if (aw22xxx->fw_flags != AW22XXX_FLAG_FW_OK) {
		pr_err("%s: fw update error: not compelte \n", __func__);
		return -ERROR_CODE_TWO;
	}
	mutex_lock(&aw22xxx->cfg_lock);

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw22xxx_cfg_name[aw22xxx->effect], aw22xxx->dev, GFP_KERNEL,
				aw22xxx, aw22xxx_cfg_loaded);

	if (ret) {
		pr_err("%s: cfg load error.\n", __func__);
		mutex_unlock(&aw22xxx->cfg_lock);
	}

	return ret;
}

static int aw22xxx_container_update(struct aw22xxx *aw22xxx,
		struct aw22xxx_container *aw22xxx_fw)
{
	unsigned int i = 0;
	unsigned char reg_val = 0;
	unsigned int tmp_bist = 0;
	unsigned int tmp_len = 0;

	/* chip enable*/
	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	msleep(2);
	aw22xxx_mcu_enable(aw22xxx, true);

	/* flash cfg */
	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	//aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, aw22xxx_fw->key);

	/* flash erase*/
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(40);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x40);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(6);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x42);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(6);
#if 0
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x44);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(6);
#endif
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);

#if 0
	/* flash write */
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < aw22xxx_fw->len; i++) {
		aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
		aw22xxx_i2c_write(aw22xxx, 0x23, aw22xxx_fw->data[i]);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
#endif
#ifdef AW22XXX_FLASH_I2C_WRITES
	/* flash writes */
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < aw22xxx_fw->len; i += tmp_len) {
		aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x11, 0x01);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
		if ((aw22xxx_fw->len - i) < MAX_FLASH_WRITE_BYTE_SIZE) {
			tmp_len = aw22xxx_fw->len - i;
		} else {
			tmp_len = MAX_FLASH_WRITE_BYTE_SIZE;
		}
		aw22xxx_i2c_writes(aw22xxx, 0x23, &aw22xxx_fw->data[i], tmp_len);
		aw22xxx_i2c_write(aw22xxx, 0x11, 0x00);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
#else
	/* flash write */
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < aw22xxx_fw->len; i++) {
		if ((i%128) == 0) {
			aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
			aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
		}
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
		aw22xxx_i2c_write(aw22xxx, 0x23, aw22xxx_fw->data[i]);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
#endif

	/* bist check */
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_mcu_enable(aw22xxx, true);
	aw22xxx_i2c_write(aw22xxx, 0x22, (((aw22xxx_fw->len-1)>>8)&0xff));
	aw22xxx_i2c_write(aw22xxx, 0x21, (((aw22xxx_fw->len-1)>>0)&0xff));
	aw22xxx_i2c_write(aw22xxx, 0x24, 0x07);
	msleep(5);
	aw22xxx_i2c_read(aw22xxx, 0x24, &reg_val);
	if (reg_val == 0x05) {
		aw22xxx_i2c_read(aw22xxx, 0x25, &reg_val);
		tmp_bist = reg_val;
		aw22xxx_i2c_read(aw22xxx, 0x26, &reg_val);
		tmp_bist |= (reg_val<<8);
		if (tmp_bist == aw22xxx_fw->bist) {
			pr_info("%s: bist check pass, bist=0x%04x\n", __func__, aw22xxx_fw->bist);
		} else {
			pr_err("%s: bist check fail, bist=0x%04x\n", __func__, aw22xxx_fw->bist);
			pr_err("%s: fw update failed, please reset phone\n", __func__);
			return -ERROR_CODE_ONE;
		}
	} else {
		pr_err("%s: bist check is running, reg0x24=0x%02x\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);
	/*
	aw22xxx_get_fw_version(aw22xxx);
	if (aw22xxx_fw->version == aw22xxx->fw_version) {
		pr_info("%s: fw update ok, flash fw version=0x%04x\n",
			__func__, aw22xxx->fw_version);
	} else {
		pr_err("%s: fw update failed, flash fw version=0x%04x\n",
			__func__, aw22xxx->fw_version);
		return -ERROR_CODE_TWO;
	}
	*/
	return 0;
}

static void aw22xxx_fw_loaded(const struct firmware *cont, void *context)
{
	struct aw22xxx *aw22xxx = context;
	struct aw22xxx_container *aw22xxx_fw;
	int i = 0;
	int ret = -1;
	char tmp_buf[32] = {0};
	unsigned int shift = 0;
	unsigned short check_sum = 0;
	unsigned char reg_val = 0;
	unsigned int tmp_bist = 0;

	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw22xxx_fw_name);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw22xxx_fw_name,
					cont ? cont->size : 0);
/*
	for (i = 0; i < cont->size; i++) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
	}
*/
	/* check sum */
	for (i = 2; i < cont->size; i++) {
		check_sum += cont->data[i];
	}
	if (check_sum != (unsigned short)((cont->data[0]<<8)|(cont->data[1]))) {
		pr_err("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
		release_firmware(cont);
		return;
	} else {
		pr_info("%s: check sum pass : 0x%04x\n", __func__, check_sum);
	}

	/* get fw info */
	aw22xxx_fw = kzalloc(cont->size + 4*sizeof(unsigned int), GFP_KERNEL);
	if (!aw22xxx_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	shift += 2;

	pr_info("%s: fw chip_id : 0x%02x\n", __func__, cont->data[0+shift]);
	shift += 1;

	memcpy(tmp_buf, &cont->data[0+shift], 16);
	pr_info("%s: fw customer: %s\n", __func__, tmp_buf);
	shift += 16;

	memcpy(tmp_buf, &cont->data[0+shift], 8);
	pr_info("%s: fw project: %s\n", __func__, tmp_buf);
	shift += 8;

	aw22xxx_fw->version = (cont->data[0+shift]<<24) | (cont->data[1+shift]<<16) |
			(cont->data[2+shift]<<8) | (cont->data[3+shift]<<0);
	pr_info("%s: fw version : 0x%04x\n", __func__, aw22xxx_fw->version);
	shift += 4;

	// reserved
	shift += 3;

	aw22xxx_fw->bist = (cont->data[0+shift]<<8) | (cont->data[1+shift]<<0);
	pr_info("%s: fw bist : 0x%04x\n", __func__, aw22xxx_fw->bist);
	shift += 2;

	aw22xxx_fw->key = cont->data[0+shift];
	pr_info("%s: fw key : 0x%04x\n", __func__, aw22xxx_fw->key);
	shift += 1;

	// reserved
	shift += 1;

	aw22xxx_fw->len = (cont->data[0+shift]<<8) | (cont->data[1+shift]<<0);
	pr_info("%s: fw len : 0x%04x\n", __func__, aw22xxx_fw->len);
	aw22xxx->fw_len = aw22xxx_fw->len;
	shift += 2;
	aw22xxx->fw_shift = shift;

	memcpy(aw22xxx_fw->data, &cont->data[shift], aw22xxx_fw->len);
	release_firmware(cont);

	/* check version */
	//aw22xxx_get_fw_version(aw22xxx);
	//aw22xxx_hw_reset(aw22xxx);
	/* bist check */
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_mcu_enable(aw22xxx, true);
	aw22xxx_i2c_write(aw22xxx, 0x22, (((aw22xxx_fw->len-1)>>8)&0xff));
	aw22xxx_i2c_write(aw22xxx, 0x21, (((aw22xxx_fw->len-1)>>0)&0xff));
	aw22xxx_i2c_write(aw22xxx, 0x24, 0x07);
	msleep(5);
	aw22xxx_i2c_read(aw22xxx, 0x24, &reg_val);
	if (reg_val == 0x05) {
		aw22xxx_i2c_read(aw22xxx, 0x25, &reg_val);
		tmp_bist = reg_val;
		aw22xxx_i2c_read(aw22xxx, 0x26, &reg_val);
		tmp_bist |= (reg_val<<8);
		if (tmp_bist == aw22xxx_fw->bist) {
			pr_info("%s: bist check pass, bist=0x%04x\n",
					__func__, aw22xxx_fw->bist);
			/*
			if (aw22xxx_fw->version == aw22xxx->fw_version) {
				pr_info("%s: find no new fw, bin fw version: 0x%04x, use the flash fw\n",
					__func__, aw22xxx->fw_version);
			*/
				if (aw22xxx->fw_update == 0) {
					kfree(aw22xxx_fw);
					aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);
					aw22xxx_led_init(aw22xxx);
					aw22xxx->fw_flags = AW22XXX_FLAG_FW_OK;
					return;
				} else {
					pr_info("%s: fw version: 0x%04x, force update fw\n",
			/*			__func__, aw22xxx_fw->version);
			
				}
			} else {
				pr_info("%s: find new fw: 0x%04x, need update\n",
			*/
					__func__, aw22xxx_fw->version);
			}
		} else {
			pr_info("%s: bist check fail, fw bist=0x%04x, flash bist=0x%04x\n",
					__func__, aw22xxx_fw->bist, tmp_bist);
			pr_info("%s: find new fw: 0x%04x, need update\n",
					__func__, aw22xxx_fw->version);
		}
	} else {
		pr_err("%s: bist check is running, reg0x24=0x%02x\n", __func__, reg_val);
		pr_info("%s: fw need update\n", __func__);
	}
	aw22xxx_i2c_write(aw22xxx, 0x24, 0x00);

	/* fw update */
	ret = aw22xxx_container_update(aw22xxx, aw22xxx_fw);
	if (ret) {
		aw22xxx->fw_flags = AW22XXX_FLAG_FW_FAIL;
	} else {
		aw22xxx->fw_flags = AW22XXX_FLAG_FW_OK;
	}
	kfree(aw22xxx_fw);

	aw22xxx->fw_update = 0;

	aw22xxx_led_init(aw22xxx);

	pr_info("%s: exit\n", __func__);
}

static int aw22xxx_fw_update(struct aw22xxx *aw22xxx)
{
	pr_info("%s enter\n", __func__);

	aw22xxx->fw_flags = AW22XXX_FLAG_FW_UPDATE;

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw22xxx_fw_name, aw22xxx->dev, GFP_KERNEL,
				aw22xxx, aw22xxx_fw_loaded);
}

#ifdef AWINIC_FW_UPDATE_DELAY
static enum hrtimer_restart aw22xxx_fw_timer_func(struct hrtimer *timer)
{
	struct aw22xxx *aw22xxx = container_of(timer, struct aw22xxx, fw_timer);

	pr_info("%s enter\n", __func__);

	schedule_work(&aw22xxx->fw_work);

	return HRTIMER_NORESTART;
}
#endif

static void aw22xxx_fw_work_routine(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, fw_work);

	pr_info("%s enter\n", __func__);

	aw22xxx_fw_update(aw22xxx);

}

static void aw22xxx_cfg_work_routine(struct work_struct *work)
{
	struct aw22xxx *aw22xxx = container_of(work, struct aw22xxx, cfg_work);

	pr_info("%s enter\n", __func__);

	aw22xxx_cfg_update(aw22xxx);

}

static int aw22xxx_fw_init(struct aw22xxx *aw22xxx)
{
#ifdef AWINIC_FW_UPDATE_DELAY
	int fw_timer_val = 8000;

	hrtimer_init(&aw22xxx->fw_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw22xxx->fw_timer.function = aw22xxx_fw_timer_func;
	INIT_WORK(&aw22xxx->fw_work, aw22xxx_fw_work_routine);
	INIT_WORK(&aw22xxx->cfg_work, aw22xxx_cfg_work_routine);
	hrtimer_start(&aw22xxx->fw_timer,
			ktime_set(fw_timer_val/1000, (fw_timer_val%1000)*1000000),
			HRTIMER_MODE_REL);
#else
	INIT_WORK(&aw22xxx->fw_work, aw22xxx_fw_work_routine);
	INIT_WORK(&aw22xxx->cfg_work, aw22xxx_cfg_work_routine);
	schedule_work(&aw22xxx->fw_work);
#endif
	return 0;
}


/******************************************************
 *
 * irq
 *
 ******************************************************/

static void aw22xxx_interrupt_clear(struct aw22xxx *aw22xxx)
{
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw22xxx_i2c_read(aw22xxx, REG_INTST, &reg_val);
	pr_info("%s: reg INTST=0x%x\n", __func__, reg_val);
}

static void aw22xxx_interrupt_setup(struct aw22xxx *aw22xxx)
{
	pr_info("%s enter\n", __func__);

	aw22xxx_interrupt_clear(aw22xxx);

	aw22xxx_i2c_write_bits(aw22xxx, REG_INTEN,
			BIT_INTEN_FUNCMPE_MASK, BIT_INTEN_FUNCMPE_ENABLE);
}

static irqreturn_t aw22xxx_irq(int irq, void *data)
{
	struct aw22xxx *aw22xxx = data;
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw22xxx_i2c_read(aw22xxx, REG_INTST, &reg_val);
	pr_info("%s: reg INTST=0x%x\n", __func__, reg_val);

	if (reg_val & BIT_INTST_FUNCMPE) {
		pr_info("%s: functions compelte!\n", __func__);
		aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
		aw22xxx_mcu_reset(aw22xxx, true);
		aw22xxx_mcu_enable(aw22xxx, false);
		aw22xxx_chip_enable(aw22xxx, false);
		pr_info("%s: enter standby mode!\n", __func__);
	}
	pr_info("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw22xxx_parse_dt(struct device *dev, struct aw22xxx *aw22xxx,
		struct device_node *np)
{
	aw22xxx->aw22xxx_supply = regulator_get(dev, "aw22xxx");
	if (IS_ERR_OR_NULL(aw22xxx->aw22xxx_supply)) {
			dev_err(dev, "%s: aw22xxx no regulator provided.\n", __func__);
			return -ERROR_CODE_ONE;
	}

	aw22xxx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw22xxx->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
		return -ERROR_CODE_ONE;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw22xxx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw22xxx->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided, will not suppport intterupt\n", __func__);
		return -ERROR_CODE_ONE;
	} else {
		dev_info(dev, "%s: irq gpio provided ok\n", __func__);
	}

	return 0;
}



static int aw22xxx_hw_off(struct aw22xxx *aw22xxx)
{
	pr_info("%s enter\n", __func__);

	if (aw22xxx && gpio_is_valid(aw22xxx->reset_gpio)) {
		gpio_set_value_cansleep(aw22xxx->reset_gpio, 0);
		msleep(1);
	} else {
		dev_err(aw22xxx->dev, "%s:	failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw22xxx_read_chipid(struct aw22xxx *aw22xxx)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg_val = 0;

	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_sw_reset(aw22xxx);

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw22xxx_i2c_read(aw22xxx, REG_SRST, &reg_val);
		if (ret < 0) {
			dev_err(aw22xxx->dev,
				"%s: failed to read register AW22XXX_REG_ID: %d\n",
				__func__, ret);
			return -EIO;
		}
		switch (reg_val) {
		case AW22XXX_SRSTR:
			pr_info("%s aw22xxx detected\n", __func__);
			//aw22xxx->flags |= AW22XXX_FLAG_SKIP_INTERRUPTS;
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, reg_val);
			break;
		}
		cnt++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}


/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw22xxx_reg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	unsigned int databuf[2] = {0, 0};
	mutex_lock(&aw22xxx->cfg_lock);
	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		pr_info("VIVOTEST: %s: addr:0x%02x, data:0x%02x\n", __func__, databuf[0], databuf[1]);
		/*
		if ((databuf[0] == 0xff) && (databuf[1] == 0xff)) {
			if (aw22xxx && gpio_is_valid(aw22xxx->reset_gpio)) {
				gpio_set_value_cansleep(aw22xxx->reset_gpio, 0);
			}
		} else if ((databuf[0] == 0xee) && (databuf[1] == 0xee)) {
			if (aw22xxx && gpio_is_valid(aw22xxx->reset_gpio)) {
				gpio_set_value_cansleep(aw22xxx->reset_gpio, 1);
			}
		} else {
		*/
			aw22xxx_i2c_write(aw22xxx, databuf[0], databuf[1]);
		//}
	}
	mutex_unlock(&aw22xxx->cfg_lock);
	return count;
}

static ssize_t aw22xxx_reg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	unsigned char reg_page = 0;
	aw22xxx_i2c_read(aw22xxx, REG_PAGE, &reg_page);
	for (i = 0; i < AW22XXX_REG_MAX; i++) {
		if (!reg_page) {
			if (!(aw22xxx_reg_access[i]&REG_RD_ACCESS))
			   continue;
		}
		aw22xxx_i2c_read(aw22xxx, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}

static ssize_t aw22xxx_hwen_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	unsigned int databuf[1] = {0};

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (1 == databuf[0]) {
			aw22xxx_hw_reset(aw22xxx);
		} else {
			aw22xxx_hw_off(aw22xxx);
		}
	}

	return count;
}

static ssize_t aw22xxx_hwen_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "hwen=%d\n",
			gpio_get_value(aw22xxx->reset_gpio));

	return len;
}

static ssize_t aw22xxx_fw_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	unsigned int databuf[1] = {0};

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw22xxx->fw_update = databuf[0];
		if (1 == databuf[0]) {
			schedule_work(&aw22xxx->fw_work);
		}
	}

	return count;
}

static ssize_t aw22xxx_fw_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	char *pbuf = NULL;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int tmp_len = 0;
	char str[64];

	/* read flash */
	mutex_lock(&aw22xxx->cfg_lock);
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	//aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);

	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	pbuf = kzalloc(aw22xxx->fw_len, GFP_KERNEL);
	if (!pbuf) {
		pr_err("%s: Error allocating memory\n", __func__);
		mutex_unlock(&aw22xxx->cfg_lock);
		return len;
	}
	for (i = 0; i < aw22xxx->fw_len; i += tmp_len) {
		aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(i>>8));
		aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(i>>0));
		if ((aw22xxx->fw_len - i) < MAX_FLASH_WRITE_BYTE_SIZE)
			tmp_len = aw22xxx->fw_len - i;
		else
			tmp_len = MAX_FLASH_WRITE_BYTE_SIZE;
		aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
		aw22xxx_i2c_reads(aw22xxx, 0x23, &pbuf[i], tmp_len);
		aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
	}
#if 0
	for (i = 0; i < aw22xxx->fw_len; i++) {
		aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(i>>8));
		aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(i>>0));
		aw22xxx_i2c_read(aw22xxx, 0x23, &pbuf[i]);
	}
#endif
	for (i = 0; i < aw22xxx->fw_len; i++) {
		snprintf(&str[j*3], 16, "%02x ", pbuf[i]);
		j++;
		if ((((i + aw22xxx->fw_shift)%16) == 15) ||
			(i == (aw22xxx->fw_len-1))) {
			j = 0;
			pr_err("%s: 0x%04x: %s\n", __func__, (i+aw22xxx->fw_shift)-0x0f, str);
		}
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	kfree(pbuf);
	mutex_unlock(&aw22xxx->cfg_lock);
	return len;
}

static ssize_t aw22xxx_cfg_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	unsigned int databuf[1] = {0};

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		aw22xxx->cfg = databuf[0];
		if (aw22xxx->cfg) {
			schedule_work(&aw22xxx->cfg_work);
		}
	}

	return count;
}

static ssize_t aw22xxx_cfg_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	for (i = 0; i < sizeof(aw22xxx_cfg_name)/AW22XXX_CFG_NAME_MAX; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "cfg[%x] = %s\n", i, aw22xxx_cfg_name[i]);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "current cfg = %s\n", aw22xxx_cfg_name[aw22xxx->effect]);

	return len;
}

static ssize_t aw22xxx_effect_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int databuf[1];
	int rc = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw22xxx->effect = databuf[0];

	if (aw22xxx->effect != 0) {
		if (regulator_count_voltages(aw22xxx->aw22xxx_supply) > 0) {
			rc = regulator_set_voltage(aw22xxx->aw22xxx_supply, 3800000, 3800000);
			pr_info("%s: aw22xxx regulator set to 3800000.\n", __func__);
			if (rc) {
				printk("regulator set_vtg:aw22xxx_supply failed rc=%d", rc);
				return rc;
			}
		}
	} else {
		if (regulator_count_voltages(aw22xxx->aw22xxx_supply) > 0) {
			rc = regulator_set_voltage(aw22xxx->aw22xxx_supply, 3300000, 3300000);
			pr_info("%s: aw22xxx regulator set to 3300000.\n", __func__);
			if (rc) {
				printk("regulator set_vtg:aw22xxx_supply failed rc=%d", rc);
				return rc;
			}
		}
	}

	return len;
}

static ssize_t aw22xxx_effect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	len += snprintf(buf + len, PAGE_SIZE - len, "effect = 0x%02x\n", aw22xxx->effect);

	return len;
}

static ssize_t aw22xxx_imax_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw22xxx->imax = databuf[0];
	aw22xxx_imax_cfg(aw22xxx, aw22xxx_imax_code[aw22xxx->imax]);

	return len;
}

static ssize_t aw22xxx_imax_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	for (i = 0; i < sizeof(aw22xxx_imax_name)/AW22XXX_IMAX_NAME_MAX; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "imax[%x] = %s\n", i, aw22xxx_imax_name[i]);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "current id = 0x%02x, imax = %s\n",
		aw22xxx->imax, aw22xxx_imax_name[aw22xxx->imax]);

	return len;
}

static ssize_t aw22xxx_rgb_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int databuf[2];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	sscanf(buf, "%x %x", &databuf[0], &databuf[1]);
	aw22xxx->rgb[databuf[0]] = databuf[1];

	return len;
}

static ssize_t aw22xxx_rgb_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	unsigned int i;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	for (i = 0; i < AW22XXX_RGB_MAX; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "rgb[%d] = 0x%06x\n", i, aw22xxx->rgb[i]);
	}
	return len;
}

static ssize_t aw22xxx_task0_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw22xxx->task0 = databuf[0];
	schedule_work(&aw22xxx->task_work);

	return len;
}

static ssize_t aw22xxx_task0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	len += snprintf(buf + len, PAGE_SIZE - len, "task0 = 0x%02x\n", aw22xxx->task0);

	return len;
}

static ssize_t aw22xxx_task1_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	sscanf(buf, "%x", &databuf[0]);
	aw22xxx->task1 = databuf[0];

	return len;
}

static ssize_t aw22xxx_task1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	len += snprintf(buf + len, PAGE_SIZE - len, "task1 = 0x%02x\n", aw22xxx->task1);

	return len;
}
static ssize_t aw22xxx_trim_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	sscanf(buf, "%x", &databuf[0]);

	if (databuf[0]) {
		schedule_work(&aw22xxx->trim_work);
	}
	return len;
}
static ssize_t aw22xxx_mcu_check_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int databuf[1];
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	sscanf(buf, "%x", &databuf[0]);
	if (databuf[0] == 1) {
		aw22xxx->fw_flags = AW22XXX_FLAG_FW_UPDATE;
		request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			aw22xxx_fw_test_name, aw22xxx->dev, GFP_KERNEL,
			aw22xxx, aw22xxx_fw_loaded);
	}

	return len;
}

static ssize_t aw22xxx_mcu_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	uint8_t reg_val = 0;
	uint8_t fail_flag = 0;

	pr_info("%s: enter\n", __func__);

	mutex_lock(&aw22xxx->cfg_lock);
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	// IRAM
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x01);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);

	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x80) {
		fail_flag = 1;
		pr_err("%s: iram test fail, reg_val=%d\n", __func__, reg_val);
	}

	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// XRAM
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x82) {
		fail_flag = 1;
		pr_err("%s: xram test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// SFR
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x86) {
		fail_flag = 1;
		pr_err("%s: sfr test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// WDT
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x04);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(100);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x88) {
		fail_flag = 1;
		pr_err("%s: wdt test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// PLUS
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x05);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x89) {
		fail_flag = 1;
		pr_err("%s: plus test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// MINUS
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x06);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x8b) {
		fail_flag = 1;
		pr_err("%s: minus test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// LSHIFT
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x07);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x8d) {
		fail_flag = 1;
		pr_err("%s: lshift test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// MUL
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x08);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x8f) {
		fail_flag = 1;
		pr_err("%s: mul test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// DIV
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x09);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x91) {
		fail_flag = 1;
		pr_err("%s: div test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// RSHIFT
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x0a);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x93) {
		fail_flag = 1;
		pr_err("%s: rshift test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// TIMER0
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x0b);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x96) {
		fail_flag = 1;
		pr_err("%s: timer0 test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	// TIMER1
	aw22xxx_i2c_write(aw22xxx, 0x06, 0x0c);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x03);
	msleep(2);
	aw22xxx_i2c_read(aw22xxx, 0x07, &reg_val);
	if (reg_val != 0x98) {
		fail_flag = 1;
		pr_err("%s: timer1 test fail, reg_val=%d\n", __func__, reg_val);
	}
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);

	mutex_unlock(&aw22xxx->cfg_lock);

	if (fail_flag == 1)
		len += snprintf(buf + len, PAGE_SIZE - len, "NG\n");
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "OK\n");
	return len;
}

static ssize_t aw22xxx_flash_check_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	return len;
}
static ssize_t aw22xxx_flash_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	uint8_t reg_val = 0;

	unsigned int i = 0;

	unsigned int tmp_len = 0;
	unsigned int j = 0;
	char *pbuf = NULL;
	unsigned int fw_len = 0x4400;

	pr_info("%s: enter\n", __func__);

	mutex_lock(&aw22xxx->cfg_lock);

	pbuf = kzalloc(MAX_FLASH_WRITE_BYTE_SIZE, GFP_KERNEL);
	if (!pbuf) {
		pr_err("%s: Error allocating memory\n", __func__);
		mutex_unlock(&aw22xxx->cfg_lock);
		return len;
	}

	/* chip enable*/
	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_mcu_enable(aw22xxx, true);

	/* flash cfg */
	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);

	/* flash erase*/
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(40);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x40);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(6);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x42);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(6);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x44);
	aw22xxx_i2c_write(aw22xxx, 0x21, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x02);
	aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
	msleep(6);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);

	/* flash read */
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);

	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);

	for (i = 0; i < fw_len; i += tmp_len) {
		aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(i>>8));
		aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(i>>0));
		if ((fw_len - i) < MAX_FLASH_WRITE_BYTE_SIZE)
			tmp_len = fw_len - i;
		else
			tmp_len = MAX_FLASH_WRITE_BYTE_SIZE;
		aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
		aw22xxx_i2c_reads(aw22xxx, 0x23, pbuf, tmp_len);
		aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
		for (j = 0; j < tmp_len; j++) {
			if (pbuf[j] != 0xff) {
				len += snprintf(buf + len, PAGE_SIZE - len,
					"flash erase test fail, flash[0x%04x]=0x%02x\n", i+j, pbuf[i]);
				mutex_unlock(&aw22xxx->cfg_lock);
				kfree(pbuf);
				pbuf = NULL;
				return len;
			}
		}
	}
#if 0
	for (i = 0; i < fw_len; i++) {
		if ((i%128) == 0) {
			aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(i>>8));
			aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(i>>0));
		}
		aw22xxx_i2c_read(aw22xxx, 0x23, &reg_val);
		if (reg_val != 0xff) {
			len += snprintf(buf + len, PAGE_SIZE - len, "flash erase test fail, flash[0x%04x]=0x%02x\n", i, reg_val);
			mutex_unlock(&aw22xxx->cfg_lock);
			return len;
		}
	}
#endif

	/* flash write */

	/* chip enable*/
	aw22xxx_reg_page_cfg(aw22xxx, AW22XXX_REG_PAGE0);
	aw22xxx_sw_reset(aw22xxx);
	aw22xxx_chip_enable(aw22xxx, true);
	aw22xxx_mcu_enable(aw22xxx, true);

	/* flash cfg */
	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	//aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);
#ifdef AW22XXX_FLASH_I2C_WRITES
	memset(pbuf, 0x00, MAX_FLASH_WRITE_BYTE_SIZE);
	/* flash writes */
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < fw_len; i += tmp_len) {
		aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
		aw22xxx_i2c_write(aw22xxx, 0x11, 0x01);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
		if ((fw_len - i) < MAX_FLASH_WRITE_BYTE_SIZE)
			tmp_len = fw_len - i;
		else
			tmp_len = MAX_FLASH_WRITE_BYTE_SIZE;
		aw22xxx_i2c_writes(aw22xxx, 0x23, pbuf, tmp_len);
		aw22xxx_i2c_write(aw22xxx, 0x11, 0x00);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
#else
	/* flash write */
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	for (i = 0; i < fw_len; i++) {
		if ((i%128) == 0) {
			aw22xxx_i2c_write(aw22xxx, 0x22, ((i>>8)&0xff));
			aw22xxx_i2c_write(aw22xxx, 0x21, ((i>>0)&0xff));
		}
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x04);
		aw22xxx_i2c_write(aw22xxx, 0x23, 0x00);
		aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	}
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
#endif

	/* flash read */
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);

	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);

	for (i = 0; i < fw_len; i += tmp_len) {
		aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(i>>8));
		aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(i>>0));
		if ((fw_len - i) < MAX_FLASH_WRITE_BYTE_SIZE)
			tmp_len = fw_len - i;
		else
			tmp_len = MAX_FLASH_WRITE_BYTE_SIZE;
		aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
		aw22xxx_i2c_reads(aw22xxx, 0x23, pbuf, tmp_len);
		aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
		for (j = 0; j < tmp_len; j++) {
			if (pbuf[j] != 0x00) {
				len += snprintf(buf + len, PAGE_SIZE - len,
					"flash write0 test fail, flash[0x%04x]=0x%02x\n", i+j, reg_val);
				mutex_unlock(&aw22xxx->cfg_lock);
				kfree(pbuf);
				pbuf = NULL;
				return len;
			}
		}
	}
#if 0
	for (i = 0; i < fw_len; i++) {
		if ((i%128) == 0) {
			aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(i>>8));
			aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(i>>0));
		}
		aw22xxx_i2c_read(aw22xxx, 0x23, &reg_val);
		if (reg_val != 0x00) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"flash write0 test fail, flash[0x%04x]=0x%02x\n", i, reg_val);
			mutex_unlock(&aw22xxx->cfg_lock);
			return len;
		}
	}
#endif

	len += snprintf(buf + len, PAGE_SIZE - len, "flash check test pass\n");
#ifdef AW22XXX_FLASH_I2C_WRITES
	kfree(pbuf);
	pbuf = NULL;
#endif
	mutex_unlock(&aw22xxx->cfg_lock);

	pr_info("%s: exit\n", __func__);


	return len;
}

static ssize_t aw22xxx_ate_info_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	//unsigned int databuf[1];
	//struct led_classdev *led_cdev = dev_get_drvdata(dev);
	//struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	return len;

}

static ssize_t aw22xxx_ate_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	uint8_t i = 0;
	uint32_t lot_id = 0;
	uint8_t wafer_id = 0;
	uint16_t point_x = 0;
	uint16_t point_y = 0;
	char *pbuf = NULL;

	mutex_lock(&aw22xxx->cfg_lock);
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);

	aw22xxx_i2c_write(aw22xxx, 0x22, 0x48);
	aw22xxx_i2c_write(aw22xxx, 0x22, 0x11);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);

	pbuf = kzalloc(0x0d, GFP_KERNEL);
	if (!pbuf) {
		pr_err("%s: Error allocating memory\n", __func__);
		mutex_unlock(&aw22xxx->cfg_lock);
		return len;
	}
	for (i = 0; i < 0x0d; i++) {
		aw22xxx_i2c_read(aw22xxx, 0x23, &pbuf[i]);
	}
	lot_id = (pbuf[7]<<24) | (pbuf[6]<<16) | (pbuf[5]<<8) | (pbuf[4]<<0);
	wafer_id = pbuf[8];
	point_x = (pbuf[10]<<8) | (pbuf[9]<<0);
	point_y = (pbuf[12]<<8) | (pbuf[11]<<0);
	len += snprintf(buf + len, PAGE_SIZE - len, "lot_id = %d\n", lot_id);
	len += snprintf(buf + len, PAGE_SIZE - len, "wafer_id = %d\n", wafer_id);
	len += snprintf(buf + len, PAGE_SIZE - len, "point_x = %d\n", point_x);
	len += snprintf(buf + len, PAGE_SIZE - len, "point_y = %d\n", point_y);
	kfree(pbuf);
	pbuf = NULL;
	mutex_unlock(&aw22xxx->cfg_lock);

	return len;
}

static ssize_t aw22xxx_trim_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);
	unsigned int i = 0;
	unsigned int tmp_addr = 0;

	mutex_lock(&aw22xxx->trim_lock);
	/* read flash */
	aw22xxx_i2c_write(aw22xxx, 0xff, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x02, 0x01);
	msleep(2);
	aw22xxx_i2c_write(aw22xxx, 0x04, 0x01);
	msleep(2);

	aw22xxx_i2c_write(aw22xxx, 0x80, 0xec);
	aw22xxx_i2c_write(aw22xxx, 0x35, 0x29);
	aw22xxx_i2c_write(aw22xxx, 0x36, 0xcd);
	aw22xxx_i2c_write(aw22xxx, 0x37, 0xba);
	aw22xxx_i2c_write(aw22xxx, 0x38, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x39, 0xf2);

	aw22xxx_i2c_write(aw22xxx, 0x30, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x03);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw22xxx_flash_orignal_data:\n");
	for (i = 0; i < 512; i++) {
		tmp_addr = 0x4400 + i;
		aw22xxx_i2c_write(aw22xxx, 0x22, (unsigned char)(tmp_addr>>8));
		aw22xxx_i2c_write(aw22xxx, 0x21, (unsigned char)(tmp_addr>>0));
		aw22xxx_i2c_read(aw22xxx, 0x23, &aw22xxx->trim_data[i]);
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x,", aw22xxx->trim_data[i]);
		if ((i % 16) == 15) {
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	aw22xxx_i2c_write(aw22xxx, 0x20, 0x00);
	aw22xxx_i2c_write(aw22xxx, 0x01, 0x55);
	msleep(2);
	mutex_unlock(&aw22xxx->trim_lock);


	return len;
}


static ssize_t aw22xxx_trim_data_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	unsigned int databuf[2] = {0, 0};

	//len += snprintf(buf + len, PAGE_SIZE - len, "trim_data:\n");
	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		if (databuf[0] < 0x200) {
			aw22xxx->trim_data[databuf[0]] = (unsigned char)databuf[1];
		}
	}

	return len;
}

static ssize_t aw22xxx_trim_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int i = 0;
	ssize_t len = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw22xxx *aw22xxx = container_of(led_cdev, struct aw22xxx, cdev);

	for (i = 0; i < 0x200; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x,", aw22xxx->trim_data[i]);
		if ((i % 16) == 15) {
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw22xxx_reg_show, aw22xxx_reg_store);
static DEVICE_ATTR(hwen, S_IWUSR | S_IRUGO, aw22xxx_hwen_show, aw22xxx_hwen_store);
static DEVICE_ATTR(fw, S_IWUSR | S_IRUGO, aw22xxx_fw_show, aw22xxx_fw_store);
static DEVICE_ATTR(cfg, S_IWUSR | S_IRUGO, aw22xxx_cfg_show, aw22xxx_cfg_store);
static DEVICE_ATTR(effect, S_IWUSR | S_IRUGO, aw22xxx_effect_show, aw22xxx_effect_store);
static DEVICE_ATTR(imax, S_IWUSR | S_IRUGO, aw22xxx_imax_show, aw22xxx_imax_store);
static DEVICE_ATTR(rgb, S_IWUSR | S_IRUGO, aw22xxx_rgb_show, aw22xxx_rgb_store);
static DEVICE_ATTR(task0, S_IWUSR | S_IRUGO, aw22xxx_task0_show, aw22xxx_task0_store);
static DEVICE_ATTR(task1, S_IWUSR | S_IRUGO, aw22xxx_task1_show, aw22xxx_task1_store);
static DEVICE_ATTR(mcu_check, S_IWUSR | S_IRUGO, aw22xxx_mcu_check_show, aw22xxx_mcu_check_store);
static DEVICE_ATTR(flash_check, S_IWUSR | S_IRUGO, aw22xxx_flash_check_show, aw22xxx_flash_check_store);
static DEVICE_ATTR(ate_info, S_IWUSR | S_IRUGO, aw22xxx_ate_info_show, aw22xxx_ate_info_store);
static DEVICE_ATTR(trim, S_IWUSR | S_IRUGO, aw22xxx_trim_show, aw22xxx_trim_store);
static DEVICE_ATTR(trim_data, S_IWUSR | S_IRUGO, aw22xxx_trim_data_show, aw22xxx_trim_data_store);						   

static struct attribute *aw22xxx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_fw.attr,
	&dev_attr_cfg.attr,
	&dev_attr_effect.attr,
	&dev_attr_imax.attr,
	&dev_attr_rgb.attr,
	&dev_attr_task0.attr,
	&dev_attr_task1.attr,
	&dev_attr_mcu_check.attr,
	&dev_attr_flash_check.attr,
	&dev_attr_ate_info.attr,
	&dev_attr_trim.attr,
	&dev_attr_trim_data.attr,
	NULL
};

static struct attribute_group aw22xxx_attribute_group = {
	.attrs = aw22xxx_attributes
};


/******************************************************
 *
 * led class dev
 *
 ******************************************************/
static int aw22xxx_parse_led_cdev(struct aw22xxx *aw22xxx,
		struct device_node *np)
{
	struct device_node *temp;
	int ret = -1;

	pr_info("%s enter\n", __func__);

	for_each_child_of_node(np, temp) {
		ret = of_property_read_string(temp, "aw22xxx,name",
			&aw22xxx->cdev.name);
		if (ret < 0) {
			dev_err(aw22xxx->dev,
				"Failure reading led name, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw22xxx,imax",
			&aw22xxx->imax);
		if (ret < 0) {
			dev_err(aw22xxx->dev,
				"Failure reading imax, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw22xxx,brightness",
			&aw22xxx->cdev.brightness);
		if (ret < 0) {
			dev_err(aw22xxx->dev,
				"Failure reading brightness, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw22xxx,max_brightness",
			&aw22xxx->cdev.max_brightness);
		if (ret < 0) {
			dev_err(aw22xxx->dev,
				"Failure reading max brightness, ret = %d\n", ret);
			goto free_pdata;
		}
	}

	INIT_WORK(&aw22xxx->brightness_work, aw22xxx_brightness_work);
	INIT_WORK(&aw22xxx->task_work, aw22xxx_task_work);
	INIT_WORK(&aw22xxx->trim_work, aw22xxx_trim_work);
	aw22xxx->cdev.brightness_set = aw22xxx_set_brightness;
	ret = led_classdev_register(aw22xxx->dev, &aw22xxx->cdev);
	if (ret) {
		dev_err(aw22xxx->dev,
			"unable to register led ret=%d\n", ret);
		goto free_pdata;
	}

	ret = sysfs_create_group(&aw22xxx->cdev.dev->kobj,
			&aw22xxx_attribute_group);
	if (ret) {
		dev_err(aw22xxx->dev, "led sysfs ret: %d\n", ret);
		goto free_class;
	}

	return 0;

free_class:
	led_classdev_unregister(&aw22xxx->cdev);
free_pdata:
	return ret;
}

/******************************************************
 *
 * led input dev
 *
 * @author chenkai
 ******************************************************/
/*
 static int aw22xxx_input_dev_init(struct aw22xxx *aw22xxx_data)
{
	struct input_dev *dev;
	int err;

	dev = devm_input_allocate_device(aw22xxx_data->dev);
	if (!dev) {
		dev_err(aw22xxx_data->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	dev->name = AW22XXX_DRIVER_NAME;
	dev->id.bustype = BUS_I2C;
	input_set_drvdata(dev, aw22xxx_data);
	input_set_capability(dev, AW22XXX_EVENT_TYPE, AW22XXX_EVENT_CODE);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	aw22xxx_data->input = dev;
	return 0;
}
*/
/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw22xxx_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw22xxx *aw22xxx;
	struct device_node *np = i2c->dev.of_node;
	int ret;
	int irq_flags;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		pr_err("%s i2c function failed\n", __func__);
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	//pr_info("%s allocate ram for aw22xxx\n", __func__);
	aw22xxx = devm_kzalloc(&i2c->dev, sizeof(struct aw22xxx), GFP_KERNEL);
	if (aw22xxx == NULL)
		return -ENOMEM;

	aw22xxx->dev = &i2c->dev;
	aw22xxx->i2c = i2c;

	i2c_set_clientdata(i2c, aw22xxx);
	mutex_init(&aw22xxx->cfg_lock);

	/* aw22xxx input device init */
	/*
	ret = aw22xxx_input_dev_init(aw22xxx);
	if (ret) {
		dev_err(&i2c->dev, "aw22xxx_input_dev_init was failed(%d)", ret);
		goto err_parse_dt;
	}
	*/
	/* aw22xxx rst & int */
	//pr_info("%s start parsing device tree...\n", __func__);
	if (np) {
		ret = aw22xxx_parse_dt(&i2c->dev, aw22xxx, np);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
			goto err_parse_dt;
		}
	} else {
		aw22xxx->reset_gpio = -1;
		aw22xxx->irq_gpio = -1;
	}

	ret = regulator_enable(aw22xxx->aw22xxx_supply);
	if (ret) {
			dev_err(&i2c->dev, "Regulator aw22xxx_supply failed ret=%d", ret);
			goto err_gpio_request;
	}
	
	ret = regulator_set_voltage(aw22xxx->aw22xxx_supply, 3300000, 3300000);
	pr_info("%s: aw22xxx regulator set to 3300000.\n", __func__);
	if (ret) {
		printk("regulator set_vtg:aw22xxx_supply failed ret=%d", ret);
		return ret;
	}

	if (gpio_is_valid(aw22xxx->reset_gpio)) {
		//gpio_free(aw22xxx->reset_gpio);
		ret = devm_gpio_request_one(&i2c->dev, aw22xxx->reset_gpio,
			GPIOF_OUT_INIT_LOW, "aw22xxx_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
			goto err_gpio_request;
		}
	} else {
		dev_err(&i2c->dev, "%s: invalid rst gpio\n", __func__);
	}

	if (gpio_is_valid(aw22xxx->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw22xxx->irq_gpio,
			GPIOF_DIR_IN, "aw22xxx_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n", __func__);
			goto err_gpio_request;
		}
	} else {
		dev_err(&i2c->dev, "%s: invalid int gpio\n", __func__);
	}

	/* hardware reset */
	aw22xxx_hw_reset(aw22xxx);

	/* aw22xxx chip id */
	ret = aw22xxx_read_chipid(aw22xxx);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw22xxx_read_chipid failed ret=%d\n", __func__, ret);
		goto err_id;
	}

	/* aw22xxx irq */
	
	if (gpio_is_valid(aw22xxx->irq_gpio) &&
		!(aw22xxx->flags & AW22XXX_FLAG_SKIP_INTERRUPTS)) {

		aw22xxx_interrupt_setup(aw22xxx);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
					gpio_to_irq(aw22xxx->irq_gpio),
					NULL, aw22xxx_irq, irq_flags,
					"aw22xxx", aw22xxx);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
					__func__, gpio_to_irq(aw22xxx->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);

		aw22xxx->flags |= AW22XXX_FLAG_SKIP_INTERRUPTS;
	}
	
	dev_set_drvdata(&i2c->dev, aw22xxx);

	aw22xxx_parse_led_cdev(aw22xxx, np);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s error creating led class dev\n", __func__);
		goto err_sysfs;
	}
	aw22xxx_fw_init(aw22xxx);
	mutex_init(&aw22xxx->trim_lock);
	//aw22xxx_a = aw22xxx;

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw22xxx->irq_gpio), aw22xxx);
err_irq:
err_id:
	regulator_put(aw22xxx->aw22xxx_supply);
	devm_gpio_free(&i2c->dev, aw22xxx->reset_gpio);
	devm_gpio_free(&i2c->dev, aw22xxx->irq_gpio);
err_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, aw22xxx);
	aw22xxx = NULL;
	return ret;
}

static int aw22xxx_i2c_remove(struct i2c_client *i2c)
{
	struct aw22xxx *aw22xxx = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);
	sysfs_remove_group(&aw22xxx->cdev.dev->kobj,
			&aw22xxx_attribute_group);
	led_classdev_unregister(&aw22xxx->cdev);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw22xxx->irq_gpio), aw22xxx);

	if (gpio_is_valid(aw22xxx->reset_gpio))
		devm_gpio_free(&i2c->dev, aw22xxx->reset_gpio);
	if (gpio_is_valid(aw22xxx->irq_gpio))
		devm_gpio_free(&i2c->dev, aw22xxx->irq_gpio);
	if (!IS_ERR_OR_NULL(aw22xxx->aw22xxx_supply))
		regulator_put(aw22xxx->aw22xxx_supply);

	devm_kfree(&i2c->dev, aw22xxx);
	aw22xxx = NULL;

	return 0;
}

static const struct i2c_device_id aw22xxx_i2c_id[] = {
	{ AW22XXX_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw22xxx_i2c_id);

static struct of_device_id aw22xxx_dt_match[] = {
	{ .compatible = "awinic,aw22xxx_led" },
	{ },
};

static struct i2c_driver aw22xxx_i2c_driver = {
	.driver = {
		.name = AW22XXX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw22xxx_dt_match),
	},
	.probe = aw22xxx_i2c_probe,
	.remove = aw22xxx_i2c_remove,
	.id_table = aw22xxx_i2c_id,
};


static int __init aw22xxx_i2c_init(void)
{
	int ret = 0;

	pr_info("aw22xxx driver version %s\n", AW22XXX_VERSION);

	ret = i2c_add_driver(&aw22xxx_i2c_driver);
	if (ret) {
		pr_err("fail to add aw22xxx device into i2c\n");
		return ret;
	}

	return 0;
}
module_init(aw22xxx_i2c_init);


static void __exit aw22xxx_i2c_exit(void)
{
	i2c_del_driver(&aw22xxx_i2c_driver);
}
module_exit(aw22xxx_i2c_exit);


MODULE_DESCRIPTION("AW22XXX LED Driver");
MODULE_LICENSE("GPL v2");
