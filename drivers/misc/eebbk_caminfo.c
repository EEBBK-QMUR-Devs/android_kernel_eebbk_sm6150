// SPDX-License-Identifier: GPL-2.0
/*
 * eebbk_caminfo.c
 * EEBBK S6 (P20H130) vendor camera support.
 *
 * The stock vendor CamX HAL (2023, camera.qcom.so) requires the kernel to
 * export /proc/driver/BackCamera_info and /proc/driver/FrontCamera_info.
 * It reads the module/sensor token from these files to select the matching
 * com.qti.sensormodule.*.bin.  Without them CamX reports zero camera
 * devices:
 *   camximagesensormoduledatamanager.cpp ... error to open: /proc/driver/
 *   BackCamera_info, No such file or directory
 *   ... check module vendor failed ... Invalid number of sensor module
 *   managers
 *
 * This driver recreates those proc entries late in boot (matching the
 * vendor kernel's cam_sensor probe timing).  The contents are writable at
 * runtime (echo token > /proc/driver/BackCamera_info) so tokens can be
 * calibrated on-device.  Tokens are of the form <vendor>_<sensor>, e.g.
 * qtech_s5k3l6 (rear: Samsung s5k3l6 13M AF) and qtech_ov16a10 (front:
 * OmniVision ov16a10 16M FF).
 */
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#define EEBBK_INFO_SIZE 256

/* Real values captured from the OEM kernel on this unit (S6/P20H130):
 * back  -> Module Vendor: Q Tech (2742EJ36Q0F002DT), Image Sensor:
 *          Samsung s5k3l6(13M)(AF)(RAW)(MIPI)          -> qtech_s5k3l6.bin
 * front -> Module Vendor: TSP , Image Sensor: OmniVision
 *          ov16a10(16M)(FF)(RAW)(MIPI)                 -> tsp_ov16a10.bin
 * CamX HAL reads a fixed 100 bytes and parses vendor/sensor from this text.
 */
static char eebbk_back_info[EEBBK_INFO_SIZE] =
	"Module Vendor: Q Tech (2742EJ36Q0F002DT), Image Sensor: Samsung "
	"s5k3l6(13M)(AF)(RAW)(MIPI)";
static char eebbk_front_info[EEBBK_INFO_SIZE] =
	"Module Vendor: TSP , Image Sensor: OmniVision "
	"ov16a10(16M)(FF)(RAW)(MIPI)";

static ssize_t eebbk_info_read(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos)
{
	char *p = PDE_DATA(file_inode(file));
	return simple_read_from_buffer(buf, count, ppos, p, strlen(p));
}

static ssize_t eebbk_info_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	char *p = PDE_DATA(file_inode(file));
	size_t n = count;
	if (n >= EEBBK_INFO_SIZE)
		n = EEBBK_INFO_SIZE - 1;
	if (copy_from_user(p, buf, n))
		return -EFAULT;
	p[n] = '\0';
	if (n > 0 && p[n - 1] == '\n')
		p[n - 1] = '\0';
	pr_info("eebbk_caminfo: %s <- '%s'\n",
		(p == eebbk_back_info) ? "BackCamera_info"
				       : "FrontCamera_info", p);
	return count;
}

static const struct file_operations eebbk_info_fops = {
	.read = eebbk_info_read,
	.write = eebbk_info_write,
	.owner = THIS_MODULE,
};

static int __init eebbk_caminfo_init(void)
{
	struct proc_dir_entry *f1, *f2;

	/*
	 * proc_create_data() accepts slash-paths: the "driver" dir is
	 * resolved by xlate_proc_name, like rtc.c's "driver/rtc".
	 * Creating the dir manually with proc_mkdir() during early init
	 * crashed (proc_register), so full paths are used instead.
	 */
	f1 = proc_create_data("driver/BackCamera_info", 0666, NULL,
			      &eebbk_info_fops, eebbk_back_info);
	f2 = proc_create_data("driver/FrontCamera_info", 0666, NULL,
			      &eebbk_info_fops, eebbk_front_info);
	if (!f1 || !f2) {
		pr_err("eebbk_caminfo: failed to create proc entries "
		       "(back=%p front=%p)\n", f1, f2);
		return -ENOMEM;
	}
	pr_info("eebbk_caminfo: /proc/driver/BackCamera_info='%s' "
		"FrontCamera_info='%s'\n", eebbk_back_info, eebbk_front_info);
	return 0;
}

static void __exit eebbk_caminfo_exit(void)
{
	remove_proc_entry("driver/BackCamera_info", NULL);
	remove_proc_entry("driver/FrontCamera_info", NULL);
}

/* vendor kernel creates these during cam_sensor probe; match late timing */
late_initcall(eebbk_caminfo_init);
module_exit(eebbk_caminfo_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EEBBK S6 camera info proc support");
