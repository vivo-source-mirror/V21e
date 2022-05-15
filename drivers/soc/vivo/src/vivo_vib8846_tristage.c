#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/log2.h>
#include <linux/qpnp/qpnp-misc.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <uapi/asm-generic/unistd.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include "../inc/timed_output.h"

struct vib_pwm_setting {
	u64	period_ns;
	u64	duty_ns;
};


struct qpnp_hap {
	struct pwm_device    *pwm_dev;
	struct platform_device      *pdev;
	struct timed_output_dev     timed_dev;
	struct mutex            lock;
	char *cali_data;
	int adec;
	int dir;
	int nenable;
	int nsleep;
	int dcen;
	int timeout;
	int stage;
	struct delayed_work delayed_work;
	struct vib_pwm_setting pwm_setting;
};

static struct workqueue_struct *motor_work_queue;
static int vivo_step[4] = {0, 0, 0, 0};
static struct qpnp_hap *p_hap;
/*motor period */
/*static int motor_period = 54000;*/
static int motor_period = 26000;
static int motor_runtime = 1080;
static int current_dir;
static int total_time;
static int true_runtime;
static const char *camera_push_pull_type;

static int qpnp_hap_parse_dt (struct qpnp_hap *hap)
{
	struct platform_device *pdev = hap->pdev;
	int rc = 0;
	enum of_gpio_flags flags;

	/*get pwm device*/
	hap->pwm_dev = of_pwm_get(pdev->dev.of_node, NULL);
	if (IS_ERR(hap->pwm_dev)) {
		rc = PTR_ERR(hap->pwm_dev);
		printk(KERN_ERR "ccm_vibrator: Cannot get PWM device rc:(%d)\n", rc);
		hap->pwm_dev = NULL;
		return rc;
	}
	/* end */
	rc = of_property_read_string(pdev->dev.of_node, "vivo,camera-push-pull-type", &camera_push_pull_type);
	if (rc) {
		printk(KERN_ERR "%s:vivo,camera-push-pull-type property do not find,set default type positive\n", __func__);
			camera_push_pull_type = "positive";
		}
	printk(KERN_ERR "%s:vivo,camera-push-pull-type is %s\n", __func__, camera_push_pull_type);

	/*adec input-PD*/
	hap->adec = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-adec", 0, &flags);
	if (hap->adec < 0) {
		/* there's no adec pin */
		printk(KERN_ERR "ccm_vibrator: there's no adec pin, the ret is %d\n", hap->adec);
	} else {
		/* use adec pin */
		rc = gpio_request(hap->adec, "vivo,motor-adec");
		if (rc < 0)
			printk(KERN_ERR "ccm_vibrator: request vivo,motot-adec error: %d\n", rc);
		rc = gpio_direction_input(hap->adec);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
		gpio_set_value(hap->adec, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
	}

	/*dir input-PD*/
	hap->dir = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-dir", 0, &flags);
	if (hap->dir < 0) {
		printk(KERN_ERR "ccm_vibrator: parse dir gpio error, the ret is %d\n", hap->dir);
	}
	rc = gpio_request(hap->dir, "vivo,motor-dir");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request dir gpio error: %d\n", rc);
	rc = gpio_direction_input(hap->dir);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		return rc;
	}
	gpio_set_value(hap->dir, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		return rc;
	}

	/* nenable PD*/
	hap->nenable = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-nenable", 0, &flags);
	if (hap->nenable < 0) {
		printk(KERN_ERR "ccm_vibrator: parse motor-nenable error, the ret is %d\n", hap->nenable);
		return hap->nenable;
	}
	rc = gpio_request(hap->nenable, "vivo,motor-nenable");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request motor-nenable gpio error: %d\n", rc);
	rc = gpio_direction_input(hap->nenable);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}
	gpio_set_value(hap->nenable, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}

	/* nsleep PD*/
	hap->nsleep = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-nsleep", 0, &flags);
	if (hap->nsleep < 0) {
		printk(KERN_ERR "ccm_vibrator: parse nsleep error, the ret is %d\n", hap->nsleep);
		return hap->nsleep;
	}
	rc = gpio_request(hap->nsleep, "vivo,motor-nsleep");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request nsleep gpio error: %d\n", rc);
	rc = gpio_direction_input(hap->nsleep);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}
	gpio_set_value(hap->nsleep, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}

	/* dcen output-low*/
	hap->dcen = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-dcen", 0, &flags);
	if (hap->dcen < 0) {
		printk(KERN_ERR "ccm_vibrator: parse dcen error, the ret is %d\n", hap->dcen);
		return hap->dcen;
	}
	rc = gpio_request(hap->dcen, "vivo,motor-dcen");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request dcen gpio error: %d\n", rc);
	rc = gpio_direction_output(hap->dcen, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
		return rc;
	}

	/* timeout output-low*/
	hap->timeout = of_get_named_gpio_flags(
				pdev->dev.of_node, "vivo,motor-timeout", 0, &flags);
	if (hap->timeout < 0) {
		printk(KERN_ERR "ccm_vibrator: parse timeout error, the ret is %d\n", hap->timeout);
		return hap->timeout;
	}
	rc = gpio_request(hap->timeout, "vivo,motor-timeout");
	if (rc < 0)
		printk(KERN_ERR "ccm_vibrator: request timeout gpio error: %d\n", rc);
	rc = gpio_direction_output(hap->timeout, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		return rc;
	}
	return rc;
}

static void vivo_set_pwm(bool enable, struct vib_pwm_setting pwm_setting)
{
	int rc;
	struct pwm_state pstate;
	printk(KERN_ERR "ccm_vibrator: set pwm:%d, period:%d, duty_ns:%d", enable, pwm_setting.period_ns, pwm_setting.duty_ns);
	pwm_get_state(p_hap->pwm_dev, &pstate);
	pstate.enabled = enable;
	pstate.period = pwm_setting.period_ns;
	pstate.duty_cycle = pwm_setting.duty_ns;
	pstate.output_type = PWM_OUTPUT_FIXED;
	pstate.output_pattern = NULL;
	rc = pwm_apply_state(p_hap->pwm_dev, &pstate);
	printk(KERN_ERR"ccm_vibrator: set pwm rc(%d)!\n", rc);
}

static int vivo_enable_vib(struct qpnp_hap *hap)
{
	int rc = 0;

	printk(KERN_ERR"ccm_vibrator: Enter the %s\n", __func__);

	/* 1.pull dcen high */
	rc = gpio_direction_output(hap->dcen, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
		return rc;
	}

	/* 2.pull nENBL high hold 1.5ms*/
	rc = gpio_direction_output(hap->nenable, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}
	usleep_range(1500, 1600);

	/* 2.pull ADEC high, hold 1us */
	if (hap->adec > 0) {
		rc = gpio_direction_output(hap->adec, 1);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
	}

	usleep_range(1, 3);

	/* 3.pull nsleep high, hold 1ms */
	rc = gpio_direction_output(hap->nsleep, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}
	usleep_range(1000, 1100);
	return rc;
}

static int vivo_disable_vib(struct qpnp_hap *hap)
{
	int rc = 0;

	/* pull nenable high */
	rc = gpio_direction_output(hap->nenable, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}

	/* set nsleep input-PD */
	rc = gpio_direction_input(hap->nsleep);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}
	gpio_set_value(hap->nsleep, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nsleep, rc);
		return rc;
	}

	/* pull timeout high, delay 0.5ms */
	rc = gpio_direction_output(hap->timeout, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		return rc;
	}
	usleep_range(1, 3);
	/* set dir input-PD*/
	rc = gpio_direction_input(hap->dir);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		return rc;
	}
	gpio_set_value(hap->dir, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dir, rc);
		return rc;
	}

	/* set adec input-PD*/
	if (hap->adec > 0) {
		rc = gpio_direction_input(hap->adec);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
		gpio_set_value(hap->adec, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->adec, rc);
			return rc;
		}
	}

	/* pull dcen low */
	rc = gpio_direction_output(hap->dcen, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->dcen, rc);
		return rc;
	}

	/* set nenable input-PD*/
	rc = gpio_direction_input(hap->nenable);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}
	gpio_set_value(hap->nenable, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		return rc;
	}
	usleep_range(20000, 21000);

	/*after 20ms, pull timeout low */
	rc = gpio_direction_output(hap->timeout, 0);
	if (rc < 0) {
		printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->timeout, rc);
		return rc;
	}

	return 0;
}

static void stop_motor_work(struct qpnp_hap *hap)
{
	int rc;
	/*disable the pwm*/
	vivo_set_pwm(false, hap->pwm_setting);

	/*reset stage */
	hap->stage = 0;
	total_time = 0;
	true_runtime = 0;

	/*disable motor gpio*/
	rc = vivo_disable_vib(hap);
	if (rc < 0) {
		printk(KERN_ERR"ccm_vibrator: disable vib fail");
	}
}

static void vivo_vib_stage_enable(struct qpnp_hap *hap, int time_ms, bool call_by_service)
{
	int rc;
	/* direction of motor's movement, 0:pop up, 1:withdraw*/
	int dir = -1;
	int runtime = 0;
	int temp_period = 0;
	bool is_jammed = false;
	int first_p = 40000;//1562pps-25khz
	int second_p = 26666;//2343pps-37.5khz
	int third_p = 80000;//781pps-12.5khz
	char *caller = NULL;
	motor_runtime = time_ms;
	dir = time_ms >> 16;
	runtime = time_ms & 0xFFFF;
	caller = call_by_service?"service":"driver";
	printk(KERN_ERR "ccm_vibrator: caller:%s vivo_vib_stage_enable time_ms:%d, runtime:%d, total:%d, true:%d",
		caller, time_ms, runtime, total_time, true_runtime);

	if (runtime == 0xEEEE) {
		printk(KERN_ERR "ccm_vibrator: vibrator jammed too many times, switch to 800pps");
		is_jammed = true;
	}

	/* 0.cancel all task when call by service*/
	if (call_by_service) {
		if (!is_jammed) {
			/* normal routine */
			rc = cancel_delayed_work_sync(&hap->delayed_work);
			if (rc) {
				printk(KERN_ERR "ccm_vibrator: call_by_service cancel delayed work successful");
			} else {
				printk(KERN_ERR "ccm_vibrator: call_by_service no delayed work or cancel failed");
			}
			if (time_ms == 0) {
				printk(KERN_ERR "ccm_vibrator: stop motor by service");
				stop_motor_work(hap);
				/*sleep 10ms*/
				usleep_range(10000, 12000);
			}
			hap->stage = 0;
			current_dir = dir;
			total_time = runtime;
			true_runtime = 0;
		} else {
			/* Enter stage 2 directly and use 800pps when jammed*/
			rc = cancel_delayed_work_sync(&hap->delayed_work);
			if (rc) {
				printk(KERN_ERR "ccm_vibrator: is_jammed delayed work successful");
				stop_motor_work(hap);
				/*sleep 10ms*/
				usleep_range(10000, 12000);
			} else {
				printk(KERN_ERR "ccm_vibrator: is_jammed cancel failed");
			}
			hap->stage = 2;
			current_dir = dir;
			total_time = 1500;
			runtime = 1500;
			true_runtime = 0;
		}
	}

	printk(KERN_ERR "ccm_vibrator: caller:%s stage %d start", caller, hap->stage);

	if (total_time <= 0) {
		printk(KERN_ERR "ccm_vibrator: caller:%s stop motor because total_time<=0", caller);
		stop_motor_work(hap);
		usleep_range(10000, 12000);
		return;
	}

	/* 1.set gpio and direction when call by service or jammed*/
	if (call_by_service || is_jammed) {
		/* set gpio */
		pr_err("ccm_vibrator: only run in stage 0 set vib gpio\n");
		rc = vivo_enable_vib(hap);
		if (rc < 0) {
			printk(KERN_ERR "ccm_vibrator: set mode error\n");
			return;
		}

		/* set motor direction */
		if (!strncmp(camera_push_pull_type, "negative", 8)) {
			if (dir == 1) {
				rc = gpio_direction_output(hap->dir, 1);
				if (rc < 0) {
					printk(KERN_ERR "ccm_vibrator: Unable to set gpio: %d to 1, the rc is %d \n", hap->dir, rc);
				}
			} else if (dir == 0) {
				rc = gpio_direction_output(hap->dir, 0);
				if (rc < 0) {
					printk(KERN_ERR "%s Unable to set gpio: %d to 0, the rc is %d \n", __func__, hap->dir, rc);
				}
			}
		} else {
			if (dir == 0) {
				rc = gpio_direction_output(hap->dir, 1);
				if (rc < 0) {
					printk(KERN_ERR "ccm_vibrator: Unable to set gpio: %d to 1, the rc is %d \n", hap->dir, rc);
				}
			} else if (dir == 1) {
				rc = gpio_direction_output(hap->dir, 0);
				if (rc < 0) {
					printk(KERN_ERR "%s Unable to set gpio: %d to 0, the rc is %d \n", __func__, hap->dir, rc);
				}
			}
		}

		/* pull nenable low, hold 200ns*/
		rc = gpio_direction_output(hap->nenable, 0);
		if (rc < 0) {
			printk(KERN_ERR "%s Unable to set gpio %d, the rc is %d \n", __func__, hap->nenable, rc);
		}
		usleep_range(1, 3);
	}

	/* 3.set pwm duty cyle:100, period: 415us(2.4KHz) */
	if (hap->stage == 0) {
		/* first stage */
		if (runtime > 70) {
			runtime = 70;
		}
		temp_period = first_p;
	} else if (hap->stage == 1) {
		/* second stage */
		if (runtime > 391) {
			runtime = 391;
		}
		temp_period = second_p;
	} else if (hap->stage == 2) {
		/* third stage */
		temp_period = third_p;
	}

	/* 4.enable pwm */
	printk(KERN_ERR"ccm_vibrator: caller:%s pwm_enable, temp_period:%d\n", caller, temp_period);
	hap->pwm_setting.period_ns = temp_period;
	hap->pwm_setting.duty_ns = (126 * temp_period)/255;
	vivo_set_pwm(true, hap->pwm_setting);

	true_runtime = runtime;
	rc = queue_delayed_work(motor_work_queue, &hap->delayed_work, msecs_to_jiffies(runtime));
	if (!rc) {
		printk(KERN_ERR "ccm_vibrator: The queue_work return is %d\n", rc);
		return;
	}
}

static void vivo_vib_enable(struct timed_output_dev *dev, int time_ms)
{
	struct qpnp_hap *hap = container_of(dev, struct qpnp_hap,
				 timed_dev);
	printk(KERN_ERR "ccm_vibrator: enter vivo_vib_enable\n");
	mutex_lock(&hap->lock);
	vivo_vib_stage_enable(hap, time_ms, true);
	mutex_unlock(&hap->lock);
}

/* work func */
static void motor_work_func(struct work_struct *work)
{
	struct qpnp_hap *hap = container_of(work, struct qpnp_hap, delayed_work.work);
	int time_ms;
	printk(KERN_ERR "ccm_vibrator: motor_work_func\n");

	printk(KERN_ERR "ccm_vibrator: stage:%d end.total:%d, true:%d\n", hap->stage, total_time, true_runtime);
	if (hap->stage == 0 || hap->stage == 1) {
		hap->stage++;
		total_time = total_time - true_runtime;
		time_ms = ((current_dir << 16) | (total_time & 0xFFFF));
		/*disable the pwm*/
		vivo_set_pwm(false, hap->pwm_setting);
		vivo_vib_stage_enable(hap, time_ms, false);
	} else if (hap->stage == 2) {
		stop_motor_work(hap);
	}
}
/* end */

/*sysfs attributes*/
static ssize_t motor_period_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long period = 0;
	int rc = 0;

	rc = kstrtoul(buf, 10, &period);
	if (rc)
		return rc;

	motor_period = (int)period*1000;
	printk(KERN_ERR"ccm_vibrator: motor period is %d(ns)\n", motor_period);

	return count;
}

static ssize_t motor_runtime_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long time = 0;
	int rc = 0;

	rc = kstrtoul(buf, 10, &time);
	if (rc)
		return rc;

	motor_runtime = (int)time;
	printk(KERN_ERR"ccm_vibrator: motor_runtime is %d\n", motor_runtime);

	return count;
}

static ssize_t motor_step_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long step = 0;
	int rc = 0,  i = 0;

	printk(KERN_ERR"ccm_vibrator: motor step is %s\n", buf);
	rc = kstrtoul(buf, 10, &step);
	if (rc)
		return rc;
	printk(KERN_ERR "ccm_vibrator: The step number is %lu\n", step);
	for (i = 3; i >= 0; i--) {
		vivo_step[i] = (int)step%10;
		step = step/10;
	}
	printk(KERN_ERR "ccm_vibrator: step is %d, %d, %d, %d\n", vivo_step[0], vivo_step[1], vivo_step[2], vivo_step[3]);

	return count;
}

static ssize_t cali_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk(KERN_ERR"ccm_vibrator: cali_data is %s, count:%zd\n", buf, count);
	memset(p_hap->cali_data, 0, 30);
	memcpy(p_hap->cali_data, buf, count < 30 ? count : 30);

	return count;
}
static ssize_t cali_data_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 30, "%s\n", p_hap->cali_data);
}
static struct device_attribute qpnp_hap_attrs[] = {
	__ATTR(period, 0664, NULL, motor_period_store),
	__ATTR(runtime, 0664, NULL, motor_runtime_store),
	__ATTR(step, 0664, NULL, motor_step_store),
	__ATTR(cali_data, 0664, cali_data_show, cali_data_store),
};
/*sysfs attributes end*/

/* get time api to know the remaining time */
static int qpnp_hap_get_time(struct timed_output_dev *dev)
{
	return 0;
}


static int qpnp_haptic_probe(struct platform_device *pdev)
{
	struct qpnp_hap *hap;
	int rc, i = 0;

	printk("ccm_vibrator: Enter the probe(tristage version), platform_device name is %s\n", pdev->name);
	hap = devm_kzalloc(&pdev->dev, sizeof(*hap), GFP_KERNEL);

	if (!hap)
		return -ENOMEM;

	hap->pdev = pdev;

	dev_set_drvdata(&pdev->dev, hap);

	printk("ccm_vibrator: before qpnp_hap_parse_dt\n");
	rc = qpnp_hap_parse_dt(hap);
	if (rc < 0) {
		pr_err("DT parsing failed\n");
		return rc;
	}

	mutex_init(&hap->lock);
	/* init delay work */
	motor_work_queue = create_singlethread_workqueue("motor_queue");
	if (!motor_work_queue) {
		pr_err("Fail to create gpio_work_queue\n");
		goto timed_output_fail;
	}
	INIT_DELAYED_WORK(&hap->delayed_work, motor_work_func);

	/*function*/
	hap->timed_dev.name = "motor";
	hap->timed_dev.get_time = qpnp_hap_get_time;
	hap->timed_dev.enable = vivo_vib_enable;

	/*register*/
	printk("ccm_vibrator: before timed_output_dev_register\n");
	rc = timed_output_dev_register(&hap->timed_dev);
	if (rc < 0) {
		pr_err("timed_output registration failed\n");
		goto timed_output_fail;
	}

	printk("ccm_vibrator: before create attributes\n");
	for (i = 0; i < ARRAY_SIZE(qpnp_hap_attrs); i++) {
		rc = sysfs_create_file(&hap->timed_dev.dev->kobj,
				&qpnp_hap_attrs[i].attr);
		if (rc < 0) {
			pr_err("sysfs creation failed\n");
			goto sysfs_fail;
		}
	}

	hap->cali_data = devm_kzalloc(&pdev->dev, 30, GFP_KERNEL);
	p_hap = hap;
	printk("ccm_vibrator: Probe ok!\n");
	return 0;

timed_output_fail:
	cancel_delayed_work_sync(&hap->delayed_work);
	mutex_destroy(&hap->lock);

sysfs_fail:
   for (; i > 0; i--)
	   sysfs_remove_file(&hap->timed_dev.dev->kobj,
			   &qpnp_hap_attrs[i].attr);
   timed_output_dev_unregister(&hap->timed_dev);

	return rc;
}

/* suspend routines to turn off haptics */
#ifdef CONFIG_PM
static int qpnp_haptic_suspend(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_haptic_pm_ops, qpnp_haptic_suspend, NULL);

static int qpnp_haptic_remove(struct platform_device *pdev)
{
	struct qpnp_hap *hap = dev_get_drvdata(&pdev->dev);
	timed_output_dev_unregister(&hap->timed_dev);
	mutex_destroy(&hap->lock);

	return 0;
}

static const struct of_device_id spmi_match_table[] = {
	{ .compatible = "vivo,vib_8846_tristage", },
	{ },
};

static struct platform_driver qpnp_haptic_driver = {
	.driver	 = {
		.name	   = "vivo,vib_8846_tristage",
		.of_match_table = spmi_match_table,
		.pm	 = &qpnp_haptic_pm_ops,
	},
	.probe	  = qpnp_haptic_probe,
	.remove	 = qpnp_haptic_remove,
};

static int __init qpnp_haptic_init(void)
{
	return platform_driver_register(&qpnp_haptic_driver);
}
module_init(qpnp_haptic_init);

static void __exit qpnp_haptic_exit(void)
{
	return platform_driver_unregister(&qpnp_haptic_driver);
}
module_exit(qpnp_haptic_exit);

MODULE_DESCRIPTION("qpnp haptic driver");
MODULE_LICENSE("GPL v2");
