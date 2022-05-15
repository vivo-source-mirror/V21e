/*
 * Copyright (C) 2016 vivo Co., Ltd.
 * YangChun <yangchun@vivo.com.cn>
 *
 * This driver is used to export hardware board information for users
 *
**/

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/sys_soc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <soc/qcom/socinfo.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/soc/qcom/smem.h>
#define BOARD_REV_LEN 16
#define BOARD_NAME_LEN 24
#define VIVO_VENDOR_LEN 8
#define VIVO_CPU_TYPE_LEN 10
#define FREQ_STR_LEN 8
#define INVALID_CPU_FREQ "0"
#define INVALID_CPU_TYPE "unkown"
#define CPU_REVISION_ADDR 0x000A607C
#define VIVO_HW_VERSION_MASK (0xF<<28)
#define VIVO_PM_STATUS_LEN 32

//store in shared memory
struct boardinfo_smem{
	uint32_t board_id;
	char board_rev[BOARD_REV_LEN];
	uint32_t size;
	uint32_t type;
	char board_name[BOARD_NAME_LEN];
	uint32_t ddr_manufacture_id;
	uint32_t lcm_id;
	uint32_t dp_status;	/*add wuyuanhan, dp image load or not.*/
	char pmic_status[VIVO_PM_STATUS_LEN];
	uint32_t reserved;//make len as a multiple of 8
} *boardinfo_smem;

struct boardinfo_ext{
	char vendor[VIVO_VENDOR_LEN];
	unsigned int cpu_freq;
	char cpu_type[VIVO_CPU_TYPE_LEN];
    char user_cpu_freq[FREQ_STR_LEN];				// max cpu_freq|string|unit GHz|for setting app display to user... 2018/10/25  wuyuanhan, 
	unsigned int core_num;
} *boardinfo_ext;

typedef struct freq_base_map {
	uint32_t board_id;
	uint32_t act_freq;
	char user_freq[FREQ_STR_LEN];
    char cpu_type[VIVO_CPU_TYPE_LEN];
} freq_base_map_t;

extern unsigned int cpuinfo_max_freq_cached;

#if 0

//维护原则:
//1、用户显示CPU频率来源于cpu芯片手册 /sys/bus/soc/devices/soc1/user_cpu_freq，单位GHz.
//2、实际CPU工作频率，以/sys/bus/cpu/devices/cpu7/cpufreq/cpuinfo_max_freq & /sys/bus/soc/devices/soc1/cpu_freq 为准
//3、freq_maps 映射表支持一个boardid，使用不同频率不同CPU的情况。
//4、每个Boarid 都需要配置，不然读取到的CPU频率为0，CPU型号为 unkonw.
//5、map表匹配原则:
//   5.1 匹配boardid，未找到 设置无效频率"0"
//   5.2 只匹配到boardid，未配置到实际工作频率cpu,获取默认显示cpu频率，CPU型号信息
//   5.3 匹配到boardid&配置到实际工作频率cpu,对应实际频率cpu频率的显示CPU频率，CPU型号信息
//
///* -----------------  1.  该平台涉及到的所有CPU手册 CPU工作频率信息                   ----------------------- */
//sm6150:
//2xKryo 360 Gold 2.0GHz
//
//sm7150:
//2xKryo 360 Gold 2.2GHz
//
///* -----------------  2.  该平台xbl/sbl下 board_id 信息拷贝，新增board_id时，需要同步更新  ------------------ */
//static vivo_board_id_t board_ids[] = {
//	{0,"SDM670",},
//	{1,"PD1832F_EX",},
//

#endif

static freq_base_map_t freq_maps[] = {
	{0, 0, "2.0", "675"},//SM6150
	{1, 0, "2.0", "675"},//PD1832F_EX_ME
	{2, 0, "2.0", "675"},//PD1838
	{6, 0, "2.0", "675"},//PD1948F_EX
	{9, 0, "2.0", "675"},//PD1832F_EX_MA
	{10, 0, "2.0", "675"},//PD1931F_EX
	{7, 0, "2.84", "855"},//ETD1816
	{11, 0, "2.84", "855"},//EXP1823
	{12, 0, "2.84", "855"},//EXP1831
	{13, 0, "2.84", "855"},//PD1824
	{14, 0, "2.96", "855 Plus"},//PD1916
	{15, 0, "2.96", "855 Plus"},//PD1922
	{16, 0, "2.96", "855 Plus"},//PD1923
	{17, 0, "2.96", "855 Plus"},//PD1924
	{18, 0, "2.96", "855 Plus"},//PD1923F_EX
	{19, 0, "2.96", "855 Plus"},//PD1924F_EX
	{20, 2841600, "2.84", "855"},//PD1936ABCD
	{20, 2956800, "2.96", "855 Plus"},//PD1936E
	{21, 0, "2.96", "855 Plus"},//PD1936F_EX
	{22, 0, "2.96", "855 Plus"},//PD1973F_EX
	{33, 0, "2.0", "675"},//PD1832F_EX UFS SM6150
	{38, 0, "2.0", "675"},//PD1948F_EX-0   SM6150
	{41, 0, "2.0", "675"},//PD1832ME       SM6150
	{48, 0, "2.96", "855 Plus"},//PD1923-o
	{49, 0, "2.96", "855 Plus"},//PD1924-o
	{50, 0, "2.96", "855 Plus"},//PD1923F_EX-o
	{51, 0, "2.96", "855 Plus"},//PD1924F_EX-o
	{60, 0, "2.3", "720G"},//start 7125
	{63, 0, "2.3", "720G"},//PD2023
	{64, 0, "2.3", "720G"},//PD2039F_EX
	{80, 0, "2.2", "730"},//SM7150
	{81, 0, "2.2", "730"},//PD2006F_EX
	{82, 0, "2.2", "730"},//PD2067F_EX
	{83, 0, "2.3", "720G"}, //PD2090F_EX
	{84, 0, "2.3", "720G"}, //PD2107F_EX
};

static ssize_t vivo_show_board_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", boardinfo_smem->board_id);
}

static ssize_t vivo_show_board_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_smem->board_name);
}
static ssize_t vivo_show_ddr_manufacture_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", boardinfo_smem->ddr_manufacture_id);
}
static ssize_t vivo_show_vendor(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->vendor);
}
static ssize_t vivo_show_cpu_freq(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", boardinfo_ext->cpu_freq);
}
static ssize_t vivo_show_cpu_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->cpu_type);
}
static ssize_t vivo_show_user_cpu_freq(struct device *dev, struct device_attribute *attr, char *buf)
{ 
    return snprintf(buf, PAGE_SIZE, "%s\n", boardinfo_ext->user_cpu_freq);
}

static ssize_t vivo_show_core_num(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", boardinfo_ext->core_num);
}

static struct device_attribute vivo_vendor = __ATTR(vendor, S_IRUGO, vivo_show_vendor,  NULL);

static struct device_attribute vivo_board_id = __ATTR(board_id, S_IRUGO, vivo_show_board_id, NULL);

static struct device_attribute vivo_board_name = __ATTR(board_name, S_IRUGO, vivo_show_board_name, NULL);

static struct device_attribute vivo_ddrinfo = __ATTR(ddrinfo, S_IRUGO, vivo_show_ddr_manufacture_id,  NULL);

static struct device_attribute vivo_cpu_freq = __ATTR(cpu_freq, S_IRUGO, vivo_show_cpu_freq, NULL);

static struct device_attribute vivo_cpu_user_freq = __ATTR(user_cpu_freq, S_IRUGO, vivo_show_user_cpu_freq, NULL);

static struct device_attribute vivo_cpu_type = __ATTR(cpu_type, S_IRUGO, vivo_show_cpu_type, NULL);	

static struct device_attribute vivo_core_num = __ATTR(core_num, S_IRUGO, vivo_show_core_num, NULL);			

static void __init populate_soc_sysfs_files(struct device *vivo_soc_device)
{
	device_create_file(vivo_soc_device, &vivo_board_id);
	device_create_file(vivo_soc_device, &vivo_board_name);
	device_create_file(vivo_soc_device, &vivo_vendor);
	device_create_file(vivo_soc_device, &vivo_ddrinfo);
	device_create_file(vivo_soc_device, &vivo_cpu_freq);
    device_create_file(vivo_soc_device, &vivo_cpu_user_freq);
	device_create_file(vivo_soc_device, &vivo_cpu_type);
	device_create_file(vivo_soc_device, &vivo_core_num);
	return;
}
	
static unsigned int vivo_get_cpu_freq(void)
{
	int cpu_id = 0;
	unsigned int max_freq = 0;
	unsigned int cur_freq = 0;
	int num_cpus = num_possible_cpus();
	for (cpu_id = 0; cpu_id < num_cpus; cpu_id++) {
		cur_freq = cpufreq_quick_get_max(cpu_id);
		if (cur_freq > max_freq) {
			max_freq = cur_freq;
		}
	}

	return max_freq;
}
static void get_user_cpu_freq_and_type(void)
{
    
	int i = 0;
	int default_index = -1;
 
	for (i = 0; i < (sizeof (freq_maps) / sizeof (freq_maps[0])); i++) {
		if (freq_maps[i].board_id == boardinfo_smem->board_id) {
			if ((freq_maps[i].act_freq == boardinfo_ext->cpu_freq) && (freq_maps[i].act_freq != 0)) {				
				pr_err("vivo board_info:Set user cpu max freq : %sGHz\n", freq_maps[i].user_freq); 
				strlcpy(boardinfo_ext->user_cpu_freq, freq_maps[i].user_freq, FREQ_STR_LEN);
				strlcpy(boardinfo_ext->cpu_type, freq_maps[i].cpu_type, VIVO_CPU_TYPE_LEN);
				return;
			} else if (freq_maps[i].act_freq == 0) {
			    default_index = i;
			}
		}
	}
	
	if (default_index >= 0) {
		pr_err("vivo board_info:Set user cpu max freq : %sGHz\n", freq_maps[default_index].user_freq);
		strlcpy(boardinfo_ext->user_cpu_freq, freq_maps[default_index].user_freq, VIVO_CPU_TYPE_LEN);
		strlcpy(boardinfo_ext->cpu_type, freq_maps[default_index].cpu_type, VIVO_CPU_TYPE_LEN);
		return;
	} else {
		strlcpy(boardinfo_ext->user_cpu_freq, INVALID_CPU_FREQ, FREQ_STR_LEN);
		strlcpy(boardinfo_ext->cpu_type, INVALID_CPU_TYPE, VIVO_CPU_TYPE_LEN);
		pr_err("vivo board_info: error: Need to set cpu max freq for user!!!\n"); 
		return;
	}
}
char *vivo_get_pmic_status(void)
{
	printk("PM OCP check: %s\n", boardinfo_smem->pmic_status);
	return boardinfo_smem->pmic_status;
}
EXPORT_SYMBOL(vivo_get_pmic_status);
static void vivo_boardinfo_ext_init(void)
{
	
	boardinfo_ext = kzalloc(sizeof(*boardinfo_ext), GFP_KERNEL);
	if (!boardinfo_ext) {
		pr_err("boardinfo_ext alloc failed!\n");
		return;
	}
    //cpu max frequency
    boardinfo_ext->cpu_freq = vivo_get_cpu_freq();
    
	//user frequency & type of cpu
	get_user_cpu_freq_and_type();
	//core number
	boardinfo_ext->core_num = num_possible_cpus();
	
	//vendor
	strlcpy(boardinfo_ext->vendor, "vivo", VIVO_VENDOR_LEN); //vivo
	pr_err("vivo cpu_freq:%u user_cpu_freq:%sGHz core_num=%u cpu_type=%s\n", boardinfo_ext->cpu_freq,
			boardinfo_ext->user_cpu_freq,
			boardinfo_ext->core_num,
			boardinfo_ext->cpu_type);
}
static int __init vivo_boardinfo_init_sysfs(void)
{
	struct device *vivo_soc_device;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;

	if (!boardinfo_smem) {
		pr_err("No boardinfo found!\n");
		return -ENODEV;
	}

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		pr_err("Soc Device alloc failed!\n");
		return -ENOMEM;
	}

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR_OR_NULL(soc_dev)) {
		kfree(soc_dev_attr);
		pr_err("Soc device register failed\n");
		return -EIO;
	}
		
	vivo_soc_device = soc_device_to_device(soc_dev);
	
	populate_soc_sysfs_files(vivo_soc_device);
	
	//extra information init
	vivo_boardinfo_ext_init();
	
	return 0;
}
late_initcall(vivo_boardinfo_init_sysfs);

static void vivo_boardinfo_print(void)
{
	pr_info("board_id=%d, board_version=%s, type=%d, board_name:%s\n",
		boardinfo_smem->board_id, boardinfo_smem->board_rev, 
		boardinfo_smem->type, boardinfo_smem->board_name);
}
#define SMEM_ID_VENDOR0 134
int __init vivo_boardinfo_init(void)
{
	static bool boardinfo_init_done;
	size_t  size;
	if (boardinfo_init_done)
		return 0;
 #if 0
 interface from :
	socinfo = smem_get_entry(SMEM_HW_SW_BUILD_ID, &size, 0,
				 SMEM_ANY_HOST_FLAG);
 to:
	socinfo = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_HW_SW_BUILD_ID, &size);
 #endif

	boardinfo_smem = (struct boardinfo_smem *)qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_ID_VENDOR0, &size);

	if (IS_ERR_OR_NULL(boardinfo_smem))
		BUG_ON("Can't find SMEM_ID_VENDOR0 for vivo boardinfo!\n");

	vivo_boardinfo_print();
	vivo_get_pmic_status();
	boardinfo_init_done = true;
	return 0;
}
subsys_initcall(vivo_boardinfo_init);
