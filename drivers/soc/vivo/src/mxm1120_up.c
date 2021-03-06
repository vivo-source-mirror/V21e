/*
  * m1120.c - Linux kernel modules for hall switch
 *
  * Copyright (C) 2013 Seunghwan Park <seunghwan.park@magnachip.com>
  * Copyright (C) 2014 MagnaChip Semiconductor.
 *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
 *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
*/

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include "../inc/mxm1120.h"
#include "../inc/akm09918.h"

/***********************************************************/
/*customer config*/
/***********************************************************/
#define M1120_DBG_ENABLE					// for debugging
#define M1120_DETECTION_MODE				M1120_DETECTION_MODE_POLLING/*M1120_DETECTION_MODE_POLLING /James*/
#define M1120_INTERRUPT_TYPE				M1120_VAL_INTSRS_INTTYPE_BESIDE
#define M1120_SENSITIVITY_TYPE			  M1120_VAL_INTSRS_SRS_10BIT_0_068mT
#define M1120_PERSISTENCE_COUNT			 M1120_VAL_PERSINT_COUNT(4)
#define M1120_OPERATION_FREQUENCY		   M1120_VAL_OPF_FREQ_80HZ
#define M1120_OPERATION_RESOLUTION		  M1120_VAL_OPF_BIT_10
#define M1120_DETECT_RANGE_HIGH			 (60)/*Need change via test.*/
#define M1120_DETECT_RANGE_LOW			  (50)/*Need change via test.*/
#define M1120_RESULT_STATUS_A			   (0x01)  // result status A ----> ==180Degree.
#define M1120_RESULT_STATUS_B			   (0x02)  // result status B ----> != 180Degree.
#define M1120_EVENT_TYPE					EV_ABS  // EV_KEY
#define M1120_EVENT_CODE					ABS_X   // KEY_F1
#define M1120_EVENT_DATA_CAPABILITY_MIN	 (-32768)
#define M1120_EVENT_DATA_CAPABILITY_MAX	 (32767)

/*MagnaChip Hall Sensor power supply VDD 2.7V~3.6V, VIO 1.65~VDD*/
#define M1120_VDD_MIN_UV	   2700000
#define M1120_VDD_MAX_UV	   3600000
#define M1120_VIO_MIN_UV	   1650000
#define M1120_VIO_MAX_UV	   3600000

/***********************************************************/
/*debug macro*/
/***********************************************************/
#ifdef M1120_DBG_ENABLE
#define dbg(fmt, args...)  printk("[M1120-DBG] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define dbgn(fmt, args...)  printk(fmt, ##args)
#else
#define dbg(fmt, args...)
#define dbgn(fmt, args...)
#endif // M1120_DBG_ENABLE
#define dbg_func_in()	   dbg("[M1120-DBG-F.IN] %s", __func__)
#define dbg_func_out()	  dbg("[M1120-DBG-F.OUT] %s", __func__)
#define dbg_line()		  dbg("[LINE] %d(%s)", __LINE__, __func__)
/***********************************************************/


/***********************************************************/
/*error display macro*/
/***********************************************************/
#define mxerr(pdev, fmt, args...)		   \
	dev_err(pdev, "[M1120-ERR] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define mxinfo(pdev, fmt, args...)		  \
	dev_info(pdev, "[M1120-INFO] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
/***********************************************************/

/***********************************************************/
/*static variable*/
/***********************************************************/
static m1120_data_t *p_m1120_data;
/***********************************************************/


/***********************************************************/
/*function protyps*/
/***********************************************************/
/*i2c interface*/
static int  m1120_i2c_read(struct i2c_client *client, u8 reg, u8 *rdata, u8 len);
static int  m1120_i2c_get_reg(struct i2c_client *client, u8 reg, u8 *rdata);
static int  m1120_i2c_write(struct i2c_client *client, u8 reg, u8 *wdata, u8 len);
static int  m1120_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata);
/*vdd / vid power control*/
static int m1120_set_power(struct device *dev, bool on);
/*scheduled work*/
static void m1120_work_func(struct work_struct *work);
static void ak09918_work_func(struct work_struct *work);
/*interrupt handler*/
//static irqreturn_t m1120_irq_handler(int irq, void *dev_id);
/*configuring or getting configured status*/
static void m1120_get_reg(struct device *dev, int *regdata);
static void m1120_set_reg(struct device *dev, int *regdata);
static int  m1120_get_enable(struct device *dev);
static void m1120_set_enable(struct device *dev, int enable);
static int  m1120_get_delay(struct device *dev);
static void m1120_set_delay(struct device *dev, int delay);
static int  m1120_get_debug(struct device *dev);
static void m1120_set_debug(struct device *dev, int debug);
static int  m1120_clear_interrupt(struct device *dev);
static int  m1120_update_interrupt_threshold(struct device *dev, short raw);
static int  m1120_set_operation_mode(struct device *dev, int mode);
static int  m1120_set_detection_mode(struct device *dev, u8 mode);
static int  m1120_init_device(struct device *dev);
static int  m1120_reset_device(struct device *dev);
static int  m1120_set_calibration(struct device *dev);
static int  m1120_get_calibrated_data(struct device *dev, int *data);
static int  m1120_measure(m1120_data_t *p_data, short *raw);
static int  m1120_get_result_status(m1120_data_t *p_data, int raw);
static int  m1120_power_ctl(m1120_data_t *data, bool on);
static int  mag_get_rawdata(struct device *dev, int16_t *data);
/***********************************************************/


/***********************************************************/
/*functions for i2c interface*/
/***********************************************************/
#define M1120_I2C_BUF_SIZE				  (17)
static int m1120_i2c_read(struct i2c_client *client, u8 reg, u8 *rdata, u8 len)
{
	int rc;

	/* Add By James for i2c_smbus_read_i2c_block_data */
	rc = i2c_smbus_read_i2c_block_data(client, reg, len, rdata);
	if (rc < 0) {
		mxerr(&client->dev, "i2c_transfer was failed(%d)", rc);
		return rc;
	}

	return 0;
}

static int  m1120_i2c_get_reg(struct i2c_client *client, u8 reg, u8 *rdata)
{
	return m1120_i2c_read(client, reg, rdata, 1);
}

static int m1120_i2c_write(struct i2c_client *client, u8 reg, u8 *wdata, u8 len)
{
	m1120_data_t *p_data = i2c_get_clientdata(client);
	u8  buf[M1120_I2C_BUF_SIZE];
	int rc;
	int i;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len+1,
			.buf = buf,
		},
	};

	if (client == NULL) {
		printk("[ERROR] %s : i2c client is NULL.\n", __func__);
		return -ENODEV;
	}

	buf[0] = reg;
	if (len > M1120_I2C_BUF_SIZE) {
		mxerr(&client->dev, "i2c buffer size must be less than %d", M1120_I2C_BUF_SIZE);
		return -EIO;
	}
	for (i = 0 ; i < len; i++)
		buf[i+1] = wdata[i];

	rc = i2c_transfer(client->adapter, msg, 1);
	if (rc < 0) {
		mxerr(&client->dev, "i2c_transfer was failed (%d)", rc);
		return rc;
	}

	if (len == 1) {
		switch (reg) {
		case M1120_REG_PERSINT:
			p_data->reg.map.persint = wdata[0];
			break;
		case M1120_REG_INTSRS:
			p_data->reg.map.intsrs = wdata[0];
			break;
		case M1120_REG_LTHL:
			p_data->reg.map.lthl = wdata[0];
			break;
		case M1120_REG_LTHH:
			p_data->reg.map.lthh = wdata[0];
			break;
		case M1120_REG_HTHL:
			p_data->reg.map.hthl = wdata[0];
			break;
		case M1120_REG_HTHH:
			p_data->reg.map.hthh = wdata[0];
			break;
		case M1120_REG_I2CDIS:
			p_data->reg.map.i2cdis = wdata[0];
			break;
		case M1120_REG_SRST:
			p_data->reg.map.srst = wdata[0];
			msleep(1);
			break;
		case M1120_REG_OPF:
			p_data->reg.map.opf = wdata[0];
			break;
		}
	}

	for (i = 0; i < len; i++)
		dbg("reg=0x%02X data=0x%02X", buf[0]+(u8)i, buf[i+1]);

	return 0;
}

static int m1120_i2c_set_reg(struct i2c_client *client, u8 reg, u8 wdata)
{
	return m1120_i2c_write(client, reg, &wdata, sizeof(wdata));
}

/***********************************************************/



/***********************************************************/
/*vdd / vid power control*/
/***********************************************************/
static int m1120_set_power(struct device *dev, bool on)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev);
	if (on) {
		// to do for vdd power up
		mxinfo(&client->dev, "vdd power up");

		msleep(5); // wait 5ms
		dbg("waiting 5ms after vdd power up");

		// to do vid power up
		mxinfo(&client->dev, "vid power up");

		msleep(10); // wait 10ms
		dbg("waiting 10ms after vid power up");
	} else {
		// to do for vid power down
		mxinfo(&client->dev, "vid power down");

		// to do for vdd power down
		mxinfo(&client->dev, "vdd power down");
	}
#else
	m1120_power_ctl(p_m1120_data, on);
#endif
	return 0;
}
/***********************************************************/


/***********************************************************/
/*functions for scheduling*/
/***********************************************************/
static void m1120_work_func(struct work_struct *work)
{
	m1120_data_t *p_data = container_of((struct delayed_work *)work, m1120_data_t, work);
	unsigned long delay = msecs_to_jiffies(m1120_get_delay(&p_data->client->dev));
	short raw = 0;
	int err = 0;

	dbg_func_in();
	err = m1120_measure(p_data, &raw);

	printk("m1120_work_func : rawData--== %6d\n", raw);

	if (!err) {
		if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
			p_data->last_data = m1120_get_result_status(p_data, raw);
		} else {
		////James add here: polling mode:
		////to call m1120_get_result_status(p_data, raw) get the status of Camera here.
			p_data->last_data = (int)raw;
		}

#if (M1120_EVENT_TYPE == EV_ABS)
		input_report_abs(p_data->input_dev, M1120_EVENT_CODE, p_data->last_data);
#elif (M1120_EVENT_TYPE == EV_KEY)
		input_report_key(p_data->input_dev, M1120_EVENT_CODE, p_data->last_data);
#else
#error ("[ERR] M1120_EVENT_TYPE is not defined.")
#endif

		input_sync(p_data->input_dev);
	}

	if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
		dbg("run update_interrupt_threshold");
		m1120_update_interrupt_threshold(&p_data->client->dev, raw);
	} else {
		schedule_delayed_work(&p_data->work, delay);
		dbg("run schedule_delayed_work");
	}
}
static void ak09918_work_func(struct work_struct *work)
{
	int ret = 0;
	m1120_data_t *p_data = container_of((struct delayed_work *)work, m1120_data_t, work);
	unsigned long delay = msecs_to_jiffies(1);
	ret = mag_get_rawdata(&p_m1120_data->client->dev, &p_data->z_value);
	printk(KERN_ERR "ecompass z_value:%d", p_data->z_value);
	schedule_delayed_work(&p_data->work, delay);
}
/***********************************************************/


/***********************************************************/
/*functions for interrupt handler*/
/***********************************************************/
static irqreturn_t m1120_irq_handler(int irq, void *dev_id)
{
	dbg_func_in();
	if (p_m1120_data != NULL) {
		dbg("run schedule_delayed_work");
		schedule_delayed_work(&p_m1120_data->work, 0);
	}
	/*James add: the INTB has interrupt single now.*/
	printk("m1120_irq_handler : the INTB has interrupt single now\n");
	return IRQ_HANDLED;
}
/***********************************************************/


/***********************************************************/
/*functions for configuring or getting configured status*/
/***********************************************************/

static void m1120_get_reg(struct device *dev, int *regdata)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err;

	u8 rega = (((*regdata) >> 8) & 0xFF);
	u8 regd = 0;
	err = m1120_i2c_get_reg(client, rega, &regd);

	*regdata = 0;
	*regdata |= (err == 0) ? 0x0000 : 0xFF00;
	*regdata |= regd;
}

static void m1120_set_reg(struct device *dev, int *regdata)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err;

	u8 rega = (((*regdata) >> 8) & 0xFF);
	u8 regd = *regdata&0xFF;
	err = m1120_i2c_set_reg(client, rega, regd);

	*regdata = 0;
	*regdata |= (err == 0) ? 0x0000 : 0xFF00;
}


static int m1120_get_enable(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	return atomic_read(&p_data->atm.enable);
}


static void m1120_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	mutex_lock(&p_data->mtx.enable);

	if (enable) {				   /*enable if state will be changed*/
		if (!atomic_cmpxchg(&p_data->atm.enable, 0, 1)) {
			if (p_data->use_e_compass) {
				printk(KERN_ERR "m1120_set_enable:%d", enable);
				schedule_delayed_work(&p_data->work, msecs_to_jiffies(0));
			} else {
				m1120_set_detection_mode(dev, p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT);
				m1120_set_operation_mode(&p_m1120_data->client->dev, OPERATION_MODE_MEASUREMENT);
			}
		}
	} else {						/*disable if state will be changed*/
		if (atomic_cmpxchg(&p_data->atm.enable, 1, 0)) {
			cancel_delayed_work_sync(&p_data->work);
			if (!p_data->use_e_compass) {
				m1120_set_operation_mode(&p_m1120_data->client->dev, OPERATION_MODE_POWERDOWN);
			}
		}
	}
	atomic_set(&p_data->atm.enable, enable);

	mutex_unlock(&p_data->mtx.enable);
}

static int m1120_get_delay(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	int delay = 0;

	delay = atomic_read(&p_data->atm.delay);

	return delay;
}

static void m1120_set_delay(struct device *dev, int delay)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	if (delay < M1120_DELAY_MIN)
		delay = M1120_DELAY_MIN;
	atomic_set(&p_data->atm.delay, delay);

	mutex_lock(&p_data->mtx.enable);

	if (m1120_get_enable(dev)) {
		if (!(p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT)) {
			cancel_delayed_work_sync(&p_data->work);
			schedule_delayed_work(&p_data->work, msecs_to_jiffies(delay));
		}
	}

	mutex_unlock(&p_data->mtx.enable);
}

static int m1120_get_debug(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	return atomic_read(&p_data->atm.debug);
}

static void m1120_set_debug(struct device *dev, int debug)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	atomic_set(&p_data->atm.debug, debug);
}

static int m1120_clear_interrupt(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	int ret = 0;

	ret = m1120_i2c_set_reg(p_data->client, M1120_REG_PERSINT, p_data->reg.map.persint | 0x01);

	return ret;
}

static void m1120_convdata_short_to_2byte(u8 opf, short x, unsigned char *hbyte, unsigned char *lbyte)
{
	if ((opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		/*8 bit resolution*/
		if (x < -128)
			x = -128;
		else if (x > 127)
			x = 127;

		if (x >= 0) {
			*lbyte = x & 0x7F;
		} else {
			*lbyte = ((0x80 - (x*(-1))) & 0x7F) | 0x80;
		}
		*hbyte = 0x00;
	} else {
		/*10 bit resolution*/
		if (x < -512)
			x = -512;
		else if (x > 511)
			x = 511;

		if (x >= 0) {
			*lbyte = x & 0xFF;
			*hbyte = (((x&0x100)>>8)&0x01) << 6;
		} else {
			*lbyte = (0x0200 - (x*(-1))) & 0xFF;
			*hbyte = ((((0x0200 - (x*(-1))) & 0x100)>>8)<<6) | 0x80;
		}
	}
}

static short m1120_convdata_2byte_to_short(u8 opf, unsigned char hbyte, unsigned char lbyte)
{
	short x;

	if ((opf & M1120_VAL_OPF_BIT_8) == M1120_VAL_OPF_BIT_8) {
		/*8 bit resolution*/
		x = lbyte & 0x7F;
		if (lbyte & 0x80) {
			x -= 0x80;
		}
	} else {
		/*10 bit resolution*/
		x = (((hbyte & 0x40) >> 6) << 8) | lbyte;
		if (hbyte&0x80) {
			x -= 0x200;
		}
	}

	return x;
}

static int m1120_update_interrupt_threshold(struct device *dev, short raw)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	u8 lthh = 0, lthl = 0, hthh = 0, hthl = 0;
	int err = -1;

	if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {

		dbg("reg.map.intsrs = 0x%02X", p_data->reg.map.intsrs);
		if (p_data->reg.map.intsrs & M1120_VAL_INTSRS_INTTYPE_WITHIN) {
			// to do another condition
		} else {
			dbg("BESIDE raw = %d", raw);
			if ((raw >= -512) && (raw < p_data->thrhigh)) {
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrhigh, &hthh, &hthl);
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, -512, &lthh, &lthl);
			} else if ((raw >= p_data->thrlow) && (raw <= 511)) {
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, 511, &hthh, &hthl);
				m1120_convdata_short_to_2byte(p_data->reg.map.opf, p_data->thrlow, &lthh, &lthl);
			}
		}

		err = m1120_i2c_set_reg(p_data->client, M1120_REG_HTHH, hthh);
		if (err)
			return err;
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_HTHL, hthl);
		if (err)
			return err;
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_LTHH, lthh);
		if (err)
			return err;
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_LTHL, lthl);
		if (err)
			return err;

		dbg("threshold : (0x%02X%02X, 0x%02X%02X)\n", hthh, hthl, lthh, lthl);

		err = m1120_clear_interrupt(dev);
		if (err)
			return err;
	}

	return err;
}

static int m1120_set_operation_mode(struct device *dev, int mode)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	u8 opf = p_data->reg.map.opf;
	int err = -1;

	switch (mode) {
	case OPERATION_MODE_POWERDOWN:
		if (p_data->irq_enabled) {

			/*disable irq*/
			disable_irq(p_data->irq);
			free_irq(p_data->irq, NULL);
			p_data->irq_enabled = 0;
		}
		opf &= (0xFF - M1120_VAL_OPF_HSSON_ON);
		err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
		mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_POWERDOWN");
		break;
	case OPERATION_MODE_MEASUREMENT:
		opf &= (0xFF - M1120_VAL_OPF_EFRD_ON);
		opf |= M1120_VAL_OPF_HSSON_ON;
		err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
		if (p_data->reg.map.intsrs & M1120_DETECTION_MODE_INTERRUPT) {
			if (!p_data->irq_enabled) {
				/*enable irq*/
				err = request_irq(p_data->irq, &m1120_irq_handler, IRQF_TRIGGER_FALLING, M1120_IRQ_NAME, 0);
				if (err) {
					mxerr(dev, "request_irq was failed");
					return err;
				}
				mxinfo(dev, "request_irq was success");
				enable_irq(p_data->irq);
				p_data->irq_enabled = 1;
			}
		}
		mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_MEASUREMENT");
		break;
	case OPERATION_MODE_FUSEROMACCESS:
		opf |= M1120_VAL_OPF_EFRD_ON;
		opf |= M1120_VAL_OPF_HSSON_ON;
		err = m1120_i2c_set_reg(client, M1120_REG_OPF, opf);
		mxinfo(&client->dev, "operation mode was chnaged to OPERATION_MODE_FUSEROMACCESS");
		break;
	}

	return err;
}

static int m1120_set_detection_mode(struct device *dev, u8 mode)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	u8 data;
	int err = 0;

	if (mode & M1120_DETECTION_MODE_INTERRUPT) {

		/*config threshold*/
		m1120_update_interrupt_threshold(dev, p_data->last_data);

		/*write intsrs*/
		data = p_data->reg.map.intsrs | M1120_DETECTION_MODE_INTERRUPT;
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_INTSRS, data);
		if (err)
			return err;

	} else {

		/*write intsrs*/
		data = p_data->reg.map.intsrs & (0xFF - M1120_DETECTION_MODE_INTERRUPT);
		err = m1120_i2c_set_reg(p_data->client, M1120_REG_INTSRS, data);
		if (err)
			return err;
	}

	return err;
}


static int m1120_init_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	int err = -1;

	/*(1) vdd and vid power up*/
	err = m1120_set_power(dev, 1);
	if (err) {
		mxerr(&client->dev, "m1120 power-on was failed (%d)", err);
		return err;
	}

	/*(2) init variables*/
	atomic_set(&p_data->atm.enable, 0);
	atomic_set(&p_data->atm.delay, M1120_DELAY_MIN);
#ifdef M1120_DBG_ENABLE
	atomic_set(&p_data->atm.debug, 1);
#else
	atomic_set(&p_data->atm.debug, 0);
#endif
	p_data->calibrated_data = 0;
	p_data->last_data = 0;
	p_data->irq_enabled = 0;
	p_data->irq_first = 1;
	p_data->thrhigh = M1120_DETECT_RANGE_HIGH;
	p_data->thrlow = M1120_DETECT_RANGE_LOW;
	m1120_set_delay(&client->dev, M1120_DELAY_MAX);
	m1120_set_debug(&client->dev, 0);

	/*(3) reset registers*/
	err = m1120_reset_device(dev);
	if (err) {
		mxerr(&client->dev, "m1120_reset_device was failed (%d)", err);
		return err;
	}

	mxinfo(&client->dev, "initializing device was success");

	return 0;
}

static int m1120_reset_device(struct device *dev)
{
	int err = 0;
	u8  read_buffer[2];
	u8  id = 0xFF, data = 0x00;

	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	if ((p_data == NULL) || (p_data->client == NULL))
		return -ENODEV;

	/*(0) check if use compass*/
	err = m1120_i2c_read(p_data->client, AKM_MAG_REG_ADDR_WIA1_099XX, read_buffer, sizeof(read_buffer));
	if (err < 0)
		return err;
	printk(KERN_ERR"ecompass:akm099xx company ID:0x%x, device ID:0x%x\n", read_buffer[0], read_buffer[1]);

	/* Verify Company ID */
	if (read_buffer[0] != AKM_MAG_COMPANY_ID)
		printk(KERN_ERR"ecompass: not akm099xx product, company ID must be 0x48, current:0x%x\n", read_buffer[0]);

	/* Check Device ID */
	if ((read_buffer[1] != AKM_MAG09912_DEVICE_ID) &&
	(read_buffer[1] != AKM_MAG09911_DEVICE_ID) && (read_buffer[1] != AKM_MAG09918_DEVICE_ID)) {
	/* Unknown device. Return with nothing detected */
		printk(KERN_ERR"ecompass: not akm099xx product, device ID must be 0x05(akm09911) or 0x04(akm09912), current:0x%x", read_buffer[1]);
	} else {
		printk(KERN_ERR"ecompass:use compass(low cost plan), device ID:0x%x\n", read_buffer[1]);
		p_m1120_data->use_e_compass = true;
		goto finish;
	}

	/*(1) sw reset*/
	err = m1120_i2c_set_reg(p_data->client, M1120_REG_SRST, M1120_VAL_SRST_RESET);
	if (err) {
		mxerr(&client->dev, "sw-reset was failed(%d)", err);
		return err;
	}
	msleep(5);
	dbg("wait 5ms after vdd power up");

	/*(2) check id*/
	err = m1120_i2c_get_reg(p_data->client, M1120_REG_DID, &id);
	if (err < 0)
		return err;
	if (id != M1120_VAL_DID) {
		mxerr(&client->dev, "current device id(0x%02X) is not M1120 device id(0x%02X)", id, M1120_VAL_DID);
		return -ENXIO;
	}

	/*(3) init variables*/
	/*(3-1) persint*/
	data = M1120_PERSISTENCE_COUNT;
	err = m1120_i2c_set_reg(p_data->client, M1120_REG_PERSINT, data);
	/*(3-2) intsrs*/
	data = M1120_DETECTION_MODE | M1120_SENSITIVITY_TYPE;
	if (data & M1120_DETECTION_MODE_INTERRUPT) {
		data |= M1120_INTERRUPT_TYPE;
	}
	err = m1120_i2c_set_reg(p_data->client, M1120_REG_INTSRS, data);
	/*(3-3) opf*/
	data = M1120_OPERATION_FREQUENCY | M1120_OPERATION_RESOLUTION;
	err = m1120_i2c_set_reg(p_data->client, M1120_REG_OPF, data);

	/*(4) write variable to register*/
	err = m1120_set_detection_mode(dev, M1120_DETECTION_MODE);
	if (err) {
		mxerr(&client->dev, "m1120_set_detection_mode was failed(%d)", err);
		return err;
	}

	/*(5) set power-down mode*/
	err = m1120_set_operation_mode(dev, OPERATION_MODE_POWERDOWN);
	if (err) {
		mxerr(&client->dev, "m1120_set_detection_mode was failed(%d)", err);
		return err;
	}

finish:
	return err;
}

static int m1120_set_calibration(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);
	int retrycnt = 10, cnt = 0;

	short raw = 0;
	int err;

	m1120_set_operation_mode(dev, OPERATION_MODE_MEASUREMENT);
	for (cnt = 0; cnt < retrycnt; cnt++) {
		msleep(M1120_DELAY_FOR_READY);
		err = m1120_measure(p_data, &raw);
		if (!err) {
			p_data->calibrated_data = raw;
			break;
		}
	}
	if (!m1120_get_enable(dev))
		m1120_set_operation_mode(dev, OPERATION_MODE_POWERDOWN);

	return err;
}

static int m1120_get_calibrated_data(struct device *dev, int *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);

	int err = 0;

	if (p_data == NULL)
		err = -ENODEV;
	else
		*data = p_data->calibrated_data;

	return err;
}

static int m1120_measure(m1120_data_t *p_data, short *raw)
{
	struct i2c_client *client = p_data->client;
	int err;
	u8 buf[3];
	int st1_is_ok = 0;

	// (1) read data
	err = m1120_i2c_read(client, M1120_REG_ST1, buf, sizeof(buf));
	if (err)
		return err;

	/* (2) check st1 at polling mode */
	if (buf[0] & 0x01)
		st1_is_ok = 1;

	if (st1_is_ok) {
		*raw = m1120_convdata_2byte_to_short(p_data->reg.map.opf, buf[2], buf[1]);
		p_data->last_data = *raw;
	} else {
		/* if data not ready, use last measure result */
		*raw = p_data->last_data;
		mxerr(&client->dev, "st1(0x%02X) is not DRDY", buf[0]);
		err = -1;
	}

	if (m1120_get_debug(&client->dev)) {
		printk("raw data (%d)\n", *raw);
	}

	return err;
}


static int m1120_get_result_status(m1120_data_t *p_data, int raw)
{
	int status;

	if (p_data->thrhigh <= raw) {
		status = M1120_RESULT_STATUS_B;
	} else if (p_data->thrlow >= raw) {
		status = M1120_RESULT_STATUS_A;
	} else {
		status = p_data->last_data;
	}

	switch (status) {
	case M1120_RESULT_STATUS_A:
		dbg("Result is status [A]\n");
		break;
	case M1120_RESULT_STATUS_B:
		dbg("Result is status [B]\n");
		break;
	}

	return status;
}

/**************************************************
   input device interface
   **************************************************/
static int m1120_input_dev_init(m1120_data_t *p_data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev) {
		return -ENOMEM;
	}
	dev->name = M1120_DRIVER_NAME;
	dev->id.bustype = BUS_I2C;

#if (M1120_EVENT_TYPE == EV_ABS)
	input_set_drvdata(dev, p_data);
	input_set_capability(dev, M1120_EVENT_TYPE, ABS_MISC);
	input_set_abs_params(dev, M1120_EVENT_CODE, M1120_EVENT_DATA_CAPABILITY_MIN, M1120_EVENT_DATA_CAPABILITY_MAX, 0, 0);
#elif (M1120_EVENT_TYPE == EV_KEY)
	input_set_drvdata(dev, p_data);
	input_set_capability(dev, M1120_EVENT_TYPE, M1120_EVENT_CODE);
#else
#error ("[ERR] M1120_EVENT_TYPE is not defined.")
#endif

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		printk(KERN_ERR "m1120_input_dev_init register error(%d)\n", err);
		return err;
	}

	p_data->input_dev = dev;

	return 0;
}

static void m1120_input_dev_terminate(m1120_data_t *p_data)
{
	struct input_dev *dev = p_data->input_dev;

	input_unregister_device(dev);
	input_free_device(dev);
}





/**************************************************
   misc device interface
   **************************************************/

static int m1120_misc_dev_open(struct inode*, struct file*);
static int m1120_misc_dev_release(struct inode*, struct file*);
static long m1120_misc_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t m1120_misc_dev_read(struct file *filp, char *buf, size_t count, loff_t *ofs);
static ssize_t m1120_misc_dev_write(struct file *filp, const char *buf, size_t count, loff_t *ofs);
static unsigned int m1120_misc_dev_poll(struct file *filp, struct poll_table_struct *pwait);

static struct file_operations m1120_misc_dev_fops = {
	.owner = THIS_MODULE,
	.open = m1120_misc_dev_open,
	.unlocked_ioctl = m1120_misc_dev_ioctl,
	.release = m1120_misc_dev_release,
	.read = m1120_misc_dev_read,
	.write = m1120_misc_dev_write,
	.poll = m1120_misc_dev_poll,
};

static struct miscdevice m1120_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = M1120_DRIVER_NAME,
	.fops = &m1120_misc_dev_fops,
};
/*m1120 misc device file operation*/
static int m1120_misc_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int m1120_misc_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}



static long m1120_misc_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	void __user *argp = (void __user *)arg;
	int kbuf = 0;
	int caldata = 0;

	switch (cmd) {
	case M1120_IOCTL_SET_ENABLE:
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		dbg("M1120_IOCTL_SET_ENABLE(%d)\n", kbuf);
		m1120_set_enable(&p_m1120_data->client->dev, kbuf);
		break;
	case M1120_IOCTL_GET_ENABLE:
		kbuf = m1120_get_enable(&p_m1120_data->client->dev);
		dbg("M1120_IOCTL_GET_ENABLE(%d)\n", kbuf);
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		break;
	case M1120_IOCTL_SET_DELAY:
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		dbg("M1120_IOCTL_SET_DELAY(%d)\n", kbuf);
		m1120_set_delay(&p_m1120_data->client->dev, kbuf);
		break;
	case M1120_IOCTL_GET_DELAY:
		kbuf = m1120_get_delay(&p_m1120_data->client->dev);
		dbg("M1120_IOCTL_GET_DELAY(%d)\n", kbuf);
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		break;
	case M1120_IOCTL_SET_CALIBRATION:
		dbg("M1120_IOCTL_SET_CALIBRATION\n");
		kbuf = m1120_set_calibration(&p_m1120_data->client->dev);
		ret = copy_to_user(argp, &kbuf, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		break;
	case M1120_IOCTL_GET_CALIBRATED_DATA:
		dbg("M1120_IOCTL_GET_CALIBRATED_DATA\n");
		kbuf = m1120_get_calibrated_data(&p_m1120_data->client->dev, &caldata);
		ret = copy_to_user(argp, &caldata, sizeof(caldata));
		if (ret)
			return -EFAULT;
		dbg("calibrated data (%d)\n", caldata);
		break;
	case M1120_IOCTL_SET_REG:
		ret = copy_to_user(argp, &caldata, sizeof(caldata));
		if (ret)
			return -EFAULT;
		dbg("M1120_IOCTL_SET_REG([0x%02X] %02X", (u8)((kbuf>>8)&0xFF), (u8)(kbuf&0xFF));
		m1120_set_reg(&p_m1120_data->client->dev, &kbuf);
		dbgn(" (%s))\n", (kbuf&0xFF00)?"Not Ok":"Ok");
		ret = copy_to_user(argp, &caldata, sizeof(caldata));
		if (ret)
			return -EFAULT;
		break;
	case M1120_IOCTL_GET_REG:
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		dbg("M1120_IOCTL_GET_REG([0x%02X]", (u8)((kbuf>>8)&0xFF));
		m1120_get_reg(&p_m1120_data->client->dev, &kbuf);
		dbgn(" 0x%02X (%s))\n", (u8)(kbuf&0xFF), (kbuf&0xFF00)?"Not Ok":"Ok");
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		break;
	case M1120_IOCTL_SET_INTERRUPT:
		ret = copy_from_user(&kbuf, argp, sizeof(kbuf));
		if (ret)
			return -EFAULT;
		dbg("M1120_IOCTL_SET_INTERRUPT(%d)\n", kbuf);
		if (kbuf) {
			m1120_set_detection_mode(&p_m1120_data->client->dev, M1120_DETECTION_MODE_INTERRUPT);
		} else {
			m1120_set_detection_mode(&p_m1120_data->client->dev, M1120_DETECTION_MODE_POLLING);
		}
		break;
	case M1120_IOCTL_GET_INTERRUPT:
		break;
	case M1120_IOCTL_SET_THRESHOLD_HIGH:
		break;
	case M1120_IOCTL_GET_THRESHOLD_HIGH:
		break;
	case M1120_IOCTL_SET_THRESHOLD_LOW:
		break;
	case M1120_IOCTL_GET_THRESHOLD_LOW:
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static ssize_t m1120_misc_dev_read(struct file *filp, char *buf, size_t count, loff_t *ofs)
{
	return 0;
}

static ssize_t m1120_misc_dev_write(struct file *filp, const char *buf, size_t count, loff_t *ofs)
{
	return 0;
}

static unsigned int m1120_misc_dev_poll(struct file *filp, struct poll_table_struct *pwait)
{
	return 0;
}

/**************************************************
   sysfs attributes
   **************************************************/
static ssize_t m1120_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", m1120_get_enable(&p_m1120_data->client->dev));
}

static ssize_t m1120_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	if ((enable == 0) || (enable == 1)) {
		m1120_set_enable(&p_m1120_data->client->dev, enable);
	}

	return count;
}

static ssize_t m1120_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", m1120_get_delay(&p_m1120_data->client->dev));
}

static ssize_t m1120_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	if (delay > M1120_DELAY_MAX) {
		delay = M1120_DELAY_MAX;
	}

	m1120_set_delay(&p_m1120_data->client->dev, delay);

	return count;
}

static ssize_t m1120_debug_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", m1120_get_debug(&p_m1120_data->client->dev));
}

static ssize_t m1120_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long debug = simple_strtoul(buf, NULL, 10);

	m1120_set_debug(&p_m1120_data->client->dev, debug);

	return count;
}

static ssize_t m1120_wake_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	return 0;
}

/* chenkai add for getting rawdata by electronic compass*/
static int mag_get_rawdata(struct device *dev, int16_t *data)
{
	int err = 0;
	u8 akm_write_buffer = 0;
	u8 akm_read_buffer = 0;
	int16_t sae_d[3] = {0, 0, 0};
	u8 rw_buffer[AKM_MAG099XX_NUM_DATA_BYTES] = {0};
	struct i2c_client *client = to_i2c_client(dev);

	/* get cntl */
	err = m1120_i2c_read(client, AKM_MAG_REG_ADDR_CNTL2_099XX, &akm_read_buffer, sizeof(akm_read_buffer));
	if (err < 0) {
		printk(KERN_ERR "ecompass: rawdata flag read failed 1");
		return err;
	}
	usleep_range(50, 100);

	/*1:switch to single measurement mode*/
	akm_write_buffer = 0x01;
	err = m1120_i2c_set_reg(client, AKM_MAG_REG_ADDR_CNTL2_099XX, akm_write_buffer);
	printk(KERN_ERR "ecompass err2:%d", err);
	if (err < 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 2");
		return err;
	}
	usleep_range(AKM_MAG09918_MEASUREMENT_TIME_TYP, AKM_MAG09918_MEASUREMENT_TIME_MAX);

	/*2:check DRDY bit*/
	err = m1120_i2c_get_reg(client, AKM_MAG_REG_ADDR_STATUS1_099XX, &rw_buffer[0]);
	printk(KERN_ERR "ecompass err3:%d", err);
	if (err < 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 3");
		return err;
	}
	usleep_range(50, 100);
	if ((rw_buffer[0] & AKM_MAG_DRDY_BIT_MASK) == 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 4");
	}

	/*3: read data*/
	err = m1120_i2c_read(client, AKM_MAG_REG_ADDR_STATUS1_099XX, rw_buffer, sizeof(rw_buffer));
	printk(KERN_ERR "ecompass err4:%d", err);
	if (err < 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 5");
		return err;
	}

	/*4: read st2 register and checkout hoft bit*/
	if (rw_buffer[AKM_MAG099XX_NUM_DATA_BYTES - 1] & AKM_MAG_HOFL_BIT_MASK) {
		printk(KERN_ERR "ecompass rawdata overflow!");
		*data = AKM_MAG09918_MAX_VALUE;
		return true;
	}
	printk(KERN_ERR " ecompass AK099xx get_rawdata rw_buffer[1] %d rw_buffer[2] %d rw_buffer[3] %d",
						 rw_buffer[1],
						 rw_buffer[2],
						 rw_buffer[3]);
	printk(KERN_ERR " ecompass AK099xx get_rawdata rw_buffer[4] %d rw_buffer[5] %d rw_buffer[6] %d",
						 rw_buffer[4],
						 rw_buffer[5],
						 rw_buffer[6]);
	sae_d[0] = (int16_t) (((uint16_t)rw_buffer[1]) | ((uint16_t)rw_buffer[2] << 8));
	sae_d[1] = (int16_t) (((uint16_t)rw_buffer[3]) | ((uint16_t)rw_buffer[4] << 8));
	sae_d[2] = (int16_t) (((uint16_t)rw_buffer[5]) | ((uint16_t)rw_buffer[6] << 8));
	/*data[0] = sae_d[0] * 1;
	data[1] = sae_d[1] * 1;
	data[2] = sae_d[2] * 1;*/
	*data = sae_d[2] * 1;

	err = m1120_i2c_set_reg(client, AKM_MAG_REG_ADDR_CNTL2_099XX, akm_read_buffer);
	printk(KERN_ERR "ecompass err5:%d", err);
	if (err < 0) {
		printk(KERN_ERR "ecompass rawdata flag read failed 7");
		return err;
	}
	usleep_range(50, 100);
	return true;
}

/*dengkangkai added for getting measure data*/
static ssize_t m1120_data_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	short value = 0;
	if (p_m1120_data->use_e_compass) {
		value = 0 - p_m1120_data->z_value/(5*30);
	} else {
		m1120_measure(p_m1120_data, &value);
	}

	return snprintf(buf, 10, "%d\n", value);
}

static ssize_t m1120_dump_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	   int reg = 0;
	   int reg_l = M1120_REG_HSL;
	   int reg_h = M1120_REG_HSH;
	   int i = 0;

	   for (i = 0; i < 11; i++) {
		   reg = i<<8;
		   m1120_get_reg(&p_m1120_data->client->dev, &reg);
		   printk(KERN_ERR"dkk: the reg 0x%02X value: 0x%02X\n", i, reg);
	   }

	   m1120_get_reg(&p_m1120_data->client->dev, &reg_l);
	   printk(KERN_ERR"dkk: the reg_l is 0x%02X\n", (u8)(reg_l&0xFF));

	   m1120_get_reg(&p_m1120_data->client->dev, &reg_h);
	   printk(KERN_ERR"dkk: the reg_h is 0x%02X", (u8)(reg_h&0xFF));

	   reg = ((reg_h&0xC0) << 2)|reg_l;
	   printk(KERN_ERR"dkk: the up hall reg measure is 0x%02X\n", reg);

	   return snprintf(buf, 10, "%d\n", reg);
	   //return 0;
}
/*dengkangkai added end*/

/* vivo sensor team chenkai add for lowcost plan start*/
static ssize_t m1120_plan_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 20, "%d\n", p_m1120_data->use_e_compass);
}
/* vivo sensor team chenkai add for lowcost plan end*/

static DEVICE_ATTR(enable,  S_IRUGO|S_IWUSR|S_IWGRP, m1120_enable_show, m1120_enable_store);
static DEVICE_ATTR(delay,   S_IRUGO|S_IWUSR|S_IWGRP, m1120_delay_show,  m1120_delay_store);
static DEVICE_ATTR(debug,   S_IRUGO|S_IWUSR|S_IWGRP, m1120_debug_show,  m1120_debug_store);
static DEVICE_ATTR(wake,	S_IWUSR|S_IWGRP,		 NULL,			  m1120_wake_store);
static DEVICE_ATTR(data,	S_IRUGO|S_IWUSR|S_IWGRP, m1120_data_show,   NULL);
static DEVICE_ATTR(dump,	S_IRUGO|S_IWUSR|S_IWGRP, m1120_dump_show,   NULL);
static DEVICE_ATTR(plan,    S_IRUGO|S_IWUSR|S_IWGRP, m1120_plan_show,   NULL);

static struct attribute *m1120_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_debug.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_dump.attr,
	&dev_attr_plan.attr,
	NULL
};

static struct attribute_group m1120_attribute_group = {
	.attrs = m1120_attributes
};

static int m1120_power_ctl(m1120_data_t *data, bool on)
{
	int ret = 0;
	int err = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			err = regulator_enable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}
			  msleep(8);////>=5ms OK.
		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			err = regulator_disable(data->vdd);
			return ret;
		}
		msleep(10); // wait 10ms
		data->power_enabled = on;
	} else {
		dev_info(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int m1120_power_init(m1120_data_t *data)
{
	int ret;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		ret = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		ret = regulator_set_voltage(data->vdd,
				M1120_VDD_MIN_UV,
				M1120_VDD_MAX_UV);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vdd_put;
		}
	}

	data->vio = regulator_get(&data->client->dev, "vio");
	if (IS_ERR(data->vio)) {
		ret = PTR_ERR(data->vio);
		dev_err(&data->client->dev,
			"Regulator get failed vio ret=%d\n", ret);
		goto reg_vdd_set;
	}

	if (regulator_count_voltages(data->vio) > 0) {
		ret = regulator_set_voltage(data->vio,
				M1120_VIO_MIN_UV,
				M1120_VIO_MAX_UV);
		if (ret) {
			dev_err(&data->client->dev,
			"Regulator set failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, M1120_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}


static int m1120_parse_dt(struct device *dev,
			m1120_data_t *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	struct i2c_client *client = to_i2c_client(dev);
	m1120_data_t *p_data = i2c_get_clientdata(client);


	rc = of_property_read_u32(np, "magnachip,init-interval", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read init-interval\n");
		return rc;
	} else {
		if (temp_val < M1120_DELAY_MIN)
			temp_val = M1120_DELAY_MIN;
		atomic_set(&p_data->atm.delay, temp_val);
	}

	p_data->int_en = of_property_read_bool(np, "magnachip,use-interrupt");

	p_data->igpio = of_get_named_gpio_flags(dev->of_node,
				"magnachip,gpio-int", 0, NULL);

	p_data->use_hrtimer = of_property_read_bool(np, "magnachip,use-hrtimer");

	return 0;
}

/**************************************************
   i2c client
   **************************************************/

static int m1120_i2c_drv_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	m1120_platform_data_t   *p_platform;
	m1120_data_t			*p_data;
	int					 err = 0;
	struct vivo_hall_dev *hall_dev;

	dbg_func_in();

	if (p_m1120_data != NULL) {
		printk(KERN_ERR "[M1120-DBG] p_m1120_data is existed, so return ok\n");
		return 0;
	}

	/*(1) allocation memory for p_m1120_data*/
	p_data = kzalloc(sizeof(m1120_data_t), GFP_KERNEL);
	if (!p_data) {
		mxerr(&client->dev, "kernel memory alocation was failed");
		err = -ENOMEM;
		goto error_0;
	}

	/*(2) init mutex variable*/
	mutex_init(&p_data->mtx.enable);
	mutex_init(&p_data->mtx.data);
	p_data->power_enabled = false;

	/*(3) config i2c client*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		mxerr(&client->dev, "i2c_check_functionality was failed");
		err = -ENODEV;
		goto error_1;
	}
	i2c_set_clientdata(client, p_data);
	p_data->client = client;
	p_m1120_data = p_data;

	if (client->dev.of_node) {
		dev_err(&client->dev, "Use client->dev.of_node\n");
		err = m1120_parse_dt(&client->dev, p_data);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			goto error_1;
		}
	} else {
		p_platform = client->dev.platform_data;
		dev_err(&client->dev, "Use platform data\n");
	}
#if 0////replaced by using m1120_parse_dt() dts tree...
	/*(4) get platform data*/
	p_platform = client->dev.platform_data;
	if (p_platform) {
		p_data->power_vi2c	  = p_platform->power_vi2c;
		p_data->power_vdd	   = p_platform->power_vdd;
		p_data->igpio		   = p_platform->interrupt_gpio;
		p_data->irq			 = p_platform->interrupt_irq;
	} else {
		p_data->power_vi2c = -1;
		p_data->power_vdd = -1;
		p_data->igpio = -1;
	}
#endif
	/*(5) setup interrupt gpio*/
	/*if (p_data->igpio != -1) {
		err = gpio_request(p_data->igpio, "m1120_irq");
		if (err) {
			mxerr(&client->dev, "gpio_request was failed(%d)", err);
			goto error_1;
		}
		mxinfo(&client->dev, "gpio_request was success");
		err = gpio_direction_input(p_data->igpio);
		if (err < 0) {
			mxerr(&client->dev, "gpio_direction_input was failed(%d)", err);
			goto error_2;
		}
		mxinfo(&client->dev, "gpio_direction_input was success");
	}*/

	err = m1120_power_init(p_data);
	if (err) {
		dev_err(&client->dev, "Failed to get sensor regulators\n");
		err = -EINVAL;
		goto error_1;
	}
	err = m1120_power_ctl(p_data, true);
	if (err) {
		dev_err(&client->dev, "Failed to enable sensor power\n");
		err = -EINVAL;
		goto error_1;
	}


	/*(6) reset and init device*/
	err = m1120_init_device(&p_data->client->dev);
	if (err) {
		mxerr(&client->dev, "m1120_init_device was failed(%d)", err);
		goto error_1;
	}
	mxinfo(&client->dev, "%s was found", id->name);

	/*(7) config work function*/
	if (p_data->use_e_compass) {
		INIT_DELAYED_WORK(&p_data->work, ak09918_work_func);
	} else {
		INIT_DELAYED_WORK(&p_data->work, m1120_work_func);
	}

	/*(8) init input device*/
	err = m1120_input_dev_init(p_data);
	if (err) {
		mxerr(&client->dev, "m1120_input_dev_init was failed(%d)", err);
		/* goto error_1; */
	} else {
		mxerr(&client->dev, "m1120_input_dev_init (%d)", err);
	}
	/* TODO DONOT CARE INPUT DEVICE*/
	hall_dev = &(p_data->hall_dev);
	hall_dev->name = UP_HALL_NAME;
	err = vivo_hall_dev_register(hall_dev);
	if (err < 0) {
		pr_err("vivo hall registration failed (%d)\n", err);
		goto error_1;
	}
	mxinfo(&client->dev, "%s was initialized", M1120_DRIVER_NAME);

	/*(9) create sysfs group*/
	err = sysfs_create_group(&hall_dev->dev->kobj, &m1120_attribute_group);
	if (err) {
		mxerr(&client->dev, "sysfs_create_group was failed(%d)", err);
		goto error_3;
	}

	/*(10) register misc device*/
	err = misc_register(&m1120_misc_dev);
	if (err) {
		mxerr(&client->dev, "misc_register was failed(%d)", err);
		goto error_4;
	}

	/*(11) imigrate p_data to p_m1120_data*/
	dbg("%s : %s was probed.\n", __func__, M1120_DRIVER_NAME);

	return 0;

error_4:
	sysfs_remove_group(&hall_dev->dev->kobj, &m1120_attribute_group);

error_3:
	m1120_input_dev_terminate(p_data);

error_1:
	kfree(p_data);

error_0:
	p_m1120_data = NULL;
	return err;
}

static int m1120_i2c_drv_remove(struct i2c_client *client)
{
	m1120_data_t *p_data = i2c_get_clientdata(client);
	struct vivo_hall_dev *hall_dev = &(p_data->hall_dev);

	m1120_set_enable(&client->dev, 0);
	misc_deregister(&m1120_misc_dev);
	sysfs_remove_group(&hall_dev->dev->kobj, &m1120_attribute_group);
	m1120_input_dev_terminate(p_data);
	if (p_data->igpio != -1) {
		gpio_free(p_data->igpio);
	}
	kfree(p_data);

	return 0;
}

static const struct i2c_device_id m1120_i2c_drv_id_table[] = {
	{M1120_DRIVER_NAME, 0 },
	{ }
};


static const struct of_device_id m1120_of_match[] = {
	{ .compatible = "magnachip,mxm1120,up", },
	{ },
};

static struct i2c_driver m1120_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = M1120_DRIVER_NAME,
		.of_match_table = m1120_of_match,
	},
	.probe	  = m1120_i2c_drv_probe,
	.remove	 = m1120_i2c_drv_remove,
	.id_table   = m1120_i2c_drv_id_table,
	//.suspend  = m1120_i2c_drv_suspend,
	//.resume	   = m1120_i2c_drv_resume,
};

static int __init m1120_driver_init_up(void)
{
	printk(KERN_INFO "%s\n", __func__);
	return i2c_add_driver(&m1120_driver);
}
module_init(m1120_driver_init_up);

static void __exit m1120_driver_exit_up(void)
{
	printk(KERN_INFO "%s\n", __func__);
	i2c_del_driver(&m1120_driver);
}
module_exit(m1120_driver_exit_up);

MODULE_AUTHOR("vivo sensor <goodpeople@programer.com>");
MODULE_VERSION(M1120_DRIVER_VERSION);
MODULE_DESCRIPTION("M1120 hallswitch driver");
MODULE_LICENSE("GPL");

