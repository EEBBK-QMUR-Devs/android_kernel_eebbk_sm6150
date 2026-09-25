/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include "cam_eeprom_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_eeprom_soc.h"
#include "cam_eeprom_core.h"
#include "cam_debug_util.h"

static long cam_eeprom_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int                       rc     = 0;
	struct cam_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_eeprom_driver_cmd(e_ctrl, arg);
		break;
	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

static int cam_eeprom_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_eeprom_ctrl_t *e_ctrl =
		v4l2_get_subdevdata(sd);

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "e_ctrl ptr is NULL");
			return -EINVAL;
	}

	mutex_lock(&(e_ctrl->eeprom_mutex));
	cam_eeprom_shutdown(e_ctrl);
	mutex_unlock(&(e_ctrl->eeprom_mutex));

	return 0;
}

int32_t cam_eeprom_update_i2c_info(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_eeprom_i2c_info_t *i2c_info)
{
	struct cam_sensor_cci_client        *cci_client = NULL;

	if (e_ctrl->io_master_info.master_type == CCI_MASTER) {
		cci_client = e_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			CAM_ERR(CAM_EEPROM, "failed: cci_client %pK",
				cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = e_ctrl->cci_i2c_master;
		cci_client->sid = (i2c_info->slave_addr) >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
	} else if (e_ctrl->io_master_info.master_type == I2C_MASTER) {
		e_ctrl->io_master_info.client->addr = i2c_info->slave_addr;
		CAM_DBG(CAM_EEPROM, "Slave addr: 0x%x", i2c_info->slave_addr);
	}
	return 0;
}

/* ---------------------------------------------------------------------------
 * EEBBK (BBK) camera module information
 *
 * The factory firmware publishes the camera module identification of both
 * cameras through /proc/driver/BackCamera_info and
 * /proc/driver/FrontCamera_info.  The content is the EEPROM memory map that
 * is read once by the kernel during probe; the module vendor is derived from
 * the module id bytes and the whole map is dumped as register/value pairs.
 * ---------------------------------------------------------------------------
 */
#define BBK_BACK_INFO_PROC	"driver/BackCamera_info"
#define BBK_FRONT_INFO_PROC	"driver/FrontCamera_info"
#define BBK_INFO_PROC_MODE	0664
#define BBK_MODULE_ID_LEN	20
#define BBK_BACK_REG_BASE	0xADD
#define BBK_FRONT_REG_BASE	0x6EF
#define BBK_INFO_ID_OFFSET	21
#define BBK_INFO_ID_LEN		16
#define BBK_INFO_ID_MIN_LEN	0x26
#define BBK_INFO_ID_FLAG	0x01

static uint8_t  *bbk_back_info_data;
static uint32_t  bbk_back_info_size;
static uint8_t  *bbk_front_info_data;
static uint32_t  bbk_front_info_size;

/* Rear camera: Samsung s5k3l6 (13M) */
#define BBK_BACK_S5K3L6(vendor) \
	"Module Vendor: " vendor " %s, Image Sensor: " \
	"Samsung s5k3l6(13M)(AF)(RAW)(MIPI)\n"

/* Front camera: OmniVision ov16a10 (16M) / Samsung s5k4h7 (8M) */
#define BBK_FRONT_OV16A10(vendor) \
	"Module Vendor: " vendor " %s, Image Sensor: " \
	"OmniVision ov16a10(16M)(FF)(RAW)(MIPI)\n"
#ifdef CONFIG_MACH_EEBBK_P22NH220
/*
 * T3FIX_FRONT_OV8856: the T3 (P22NH220) front module is an OmniVision
 * OV8856.  The vendor CamX HAL only ships
 *   /vendor/lib64/camera/com.qti.sensor.ov8856.so
 *   /vendor/lib64/camera/com.qti.sensormodule.tsp_ov8856.bin
 *   /vendor/lib64/camera/com.qti.eeprom.tsp_p24c64g_ov8856.so
 * and the factory kernel reports "Module Vendor: TSP, Image Sensor:
 * OmniVision ov8856(8M)" for this very module, which is what makes CamX
 * match tsp_ov8856.bin and enumerate the front camera.  Reporting ov16a10
 * made every sensormodule bin fail its vendor check, so only the rear
 * camera ever reached the camera provider.
 */
#define BBK_FRONT_OV8856(vendor) \
	"Module Vendor: " vendor " %s, Image Sensor: " \
	"OmniVision ov8856(8M)(FF)(RAW)(MIPI)\n"
#endif
#define BBK_FRONT_H110_OV16A10(vendor) \
	"H110 Module Vendor: " vendor " %s, Image Sensor: " \
	"OmniVision ov16a10(16M)(FF)(RAW)(MIPI)\n"
#define BBK_FRONT_S5K4H7(vendor) \
	"Module Vendor: " vendor " %s, Image Sensor: " \
	"Samsung s5k4h7(8M)(FF)(RAW)(MIPI)\n"

/**
 * bbk_back_info_vendor - resolve the rear camera module vendor
 * @buf: module id bytes read from the EEPROM
 * @len: number of valid bytes in @buf
 *
 * Returns the format string describing the detected module.
 */
static const char *bbk_back_info_vendor(uint8_t *buf, uint32_t len)
{
	if (!len)
		return BBK_BACK_S5K3L6("Q Tech");

	if (buf[0] == 0x05) {			/* Coe125 */
		if (len <= 1)
			return BBK_BACK_S5K3L6("Coe125");
		if (buf[1])
			return BBK_BACK_S5K3L6("unknown");
		if (len < 7)
			return BBK_BACK_S5K3L6("Coe125");
		if (buf[6])
			return BBK_BACK_S5K3L6("unknown");
		if (len < 8)
			return BBK_BACK_S5K3L6("Coe125");
		if (buf[7])
			return BBK_BACK_S5K3L6("unknown");
		if (len < 9)
			return BBK_BACK_S5K3L6("Coe125");
		if (buf[8])
			return BBK_BACK_S5K3L6("unknown");
		if (len < 10)
			return BBK_BACK_S5K3L6("Coe125");
		if (buf[9])
			return BBK_BACK_S5K3L6("unknown");

		return BBK_BACK_S5K3L6("Coe125");
	}

	if (buf[0] == 0x02) {			/* Truly */
		if (len <= 1)
			return BBK_BACK_S5K3L6("Truly");
		if (buf[1])
			return BBK_BACK_S5K3L6("unknown");
		if (len < 7)
			return BBK_BACK_S5K3L6("Truly");
		if (buf[6] != 0x64)
			return BBK_BACK_S5K3L6("unknown");
		if (len < 8)
			return BBK_BACK_S5K3L6("Truly");
		if (buf[7] != 0x71)
			return BBK_BACK_S5K3L6("unknown");
		if (len < 9)
			return BBK_BACK_S5K3L6("Truly");
		if (buf[8] != 0x18)
			return BBK_BACK_S5K3L6("unknown");
		if (len < 10)
			return BBK_BACK_S5K3L6("Truly");
		if (buf[9] != 0xc1)
			return BBK_BACK_S5K3L6("unknown");

		return BBK_BACK_S5K3L6("Truly");
	}

	if (buf[0] != 0x01)
		return BBK_BACK_S5K3L6("unknown");

	if (len <= 1)
		return BBK_BACK_S5K3L6("Q Tech");

	if (buf[1] == 0xa0) {			/* TSP */
		if (len < 6)
			return BBK_BACK_S5K3L6("TSP");
		if (buf[5])
			return BBK_BACK_S5K3L6("unknown");
		if (len < 7)
			return BBK_BACK_S5K3L6("TSP");
		if (buf[6])
			return BBK_BACK_S5K3L6("unknown");
		if (len < 8)
			return BBK_BACK_S5K3L6("TSP");
		if (buf[7])
			return BBK_BACK_S5K3L6("unknown");

		return BBK_BACK_S5K3L6("TSP");
	}

	if (buf[1] != 0x06)			/* Q Tech */
		return BBK_BACK_S5K3L6("unknown");

	if (len < 6)
		return BBK_BACK_S5K3L6("Q Tech");
	if (buf[5])
		return BBK_BACK_S5K3L6("unknown");
	if (len < 7)
		return BBK_BACK_S5K3L6("Q Tech");
	if (buf[6])
		return BBK_BACK_S5K3L6("unknown");
	if (len < 8)
		return BBK_BACK_S5K3L6("Q Tech");
	if (buf[7])
		return BBK_BACK_S5K3L6("unknown");

	return BBK_BACK_S5K3L6("Q Tech");
}

/**
 * bbk_front_info_vendor - resolve the front camera module vendor
 * @buf: module id bytes read from the EEPROM
 * @len: number of valid bytes in @buf
 *
 * Returns the format string describing the detected module.
 */
static const char *bbk_front_info_vendor(uint8_t *buf, uint32_t len)
{
	if (!len)
	    #ifdef CONFIG_MACH_EEBBK_P22NH220
	    return BBK_FRONT_OV8856("TSP");
	    #else
		return BBK_FRONT_OV16A10("TSP");
		#endif

	if (buf[0] == 0xa0) {			/* H110 / TSP ov16a10 */
		if (len <= 1)
			return BBK_FRONT_H110_OV16A10("TSP");
		if (buf[1])
			return BBK_FRONT_S5K4H7("unknown");
		if (len < 7)
			return BBK_FRONT_H110_OV16A10("TSP");
		if (buf[6] != 0x41)
			return BBK_FRONT_S5K4H7("unknown");
		if (len < 8)
			return BBK_FRONT_H110_OV16A10("TSP");
		if (buf[7] != 0x16)
			return BBK_FRONT_S5K4H7("unknown");

		return BBK_FRONT_H110_OV16A10("TSP");
	}

	if (buf[0] != 0x01)
		return BBK_FRONT_S5K4H7("unknown");

	if (len <= 1)
	    #ifdef CONFIG_MACH_EEBBK_P22NH220
	    return BBK_FRONT_OV8856("TSP");
	    #else
		return BBK_FRONT_OV16A10("TSP");
		#endif

	if (buf[1] == 0x06) {			/* Q Tech ov16a10 */
		if (len < 8)
			return BBK_FRONT_OV16A10("Q Tech");
		if (!buf[7]) {
			if (len < 9)
				return BBK_FRONT_OV16A10("Q Tech");
			if (!buf[8]) {
				if (len < 10)
					return BBK_FRONT_OV16A10("Q Tech");
				if (!buf[9])
					return BBK_FRONT_OV16A10("Q Tech");
			}
		}
	} else if (buf[1] == 0x0a) {		/* TSP ov16a10 */
		if (len < 7)
		    #ifdef CONFIG_MACH_EEBBK_P22NH220
		    return BBK_FRONT_OV8856("TSP");
		    #else
			return BBK_FRONT_OV16A10("TSP");
			#endif
		if (!buf[6]) {
			if (len < 8)
			    #ifdef CONFIG_MACH_EEBBK_P22NH220
			    return BBK_FRONT_OV8856("TSP");
			    #else
				return BBK_FRONT_OV16A10("TSP");
				#endif
			if (!buf[7]) {
				if (len < 9)
				    #ifdef CONFIG_MACH_EEBBK_P22NH220
				    return BBK_FRONT_OV8856("TSP");
				    #else
					return BBK_FRONT_OV16A10("TSP");
					#endif
				if (!buf[8]) {
					if (len < 10)
					    #ifdef CONFIG_MACH_EEBBK_P22NH220
					    return BBK_FRONT_OV8856("TSP");
					    #else
						return BBK_FRONT_OV16A10("TSP");
						#endif
					if (!buf[9])
					    #ifdef CONFIG_MACH_EEBBK_P22NH220
					    return BBK_FRONT_OV8856("TSP");
					    #else
						return BBK_FRONT_OV16A10("TSP");
						#endif
				}
			}
		}
	}

	/* Samsung s5k4h7 (8M) family */
	if (buf[1] != 0x01)
		return BBK_FRONT_S5K4H7("unknown");

	if (len < 3)
		return BBK_FRONT_S5K4H7("LiteArray");
	if (buf[2] == 0x10) {
		if (len < 4)
			return BBK_FRONT_S5K4H7("LiteArray");
		if (buf[3] == 0x0c) {
			if (len < 5)
				return BBK_FRONT_S5K4H7("LiteArray");
			if (buf[4] == 0xd0) {
				if (len < 6)
					return BBK_FRONT_S5K4H7("LiteArray");
				if (buf[5] == 0x09) {
					if (len < 7)
						return BBK_FRONT_S5K4H7(
							"LiteArray");
					if (buf[6] == 0xa0) {
						if (len < 0xc)
							return
							BBK_FRONT_S5K4H7(
							"LiteArray");
						if (!buf[11]) {
							if (len < 0xd)
								return
								BBK_FRONT_S5K4H7(
								"LiteArray");
							if (buf[12] != 0x04)
								goto second;
							if (len < 0xe)
								return
								BBK_FRONT_S5K4H7(
								"LiteArray");
							if (buf[13])
								goto second;
							if (len < 0xf)
								return
								BBK_FRONT_S5K4H7(
								"LiteArray");
							if (buf[14])
								goto second;

							return
							BBK_FRONT_S5K4H7(
								"LiteArray");
						}
					}
				}
			}
		}
	}

second:
	/* Truly s5k4h7 */
	if (buf[2] != 0x10)
		return BBK_FRONT_S5K4H7("unknown");
	if (len < 4)
		return BBK_FRONT_S5K4H7("Truly");
	if (buf[3] != 0x0c)
		return BBK_FRONT_S5K4H7("unknown");
	if (len < 5)
		return BBK_FRONT_S5K4H7("Truly");
	if (buf[4] != 0xd0)
		return BBK_FRONT_S5K4H7("unknown");
	if (len < 6)
		return BBK_FRONT_S5K4H7("Truly");
	if (buf[5] != 0x09)
		return BBK_FRONT_S5K4H7("unknown");
	if (len < 7)
		return BBK_FRONT_S5K4H7("Truly");
	if (buf[6] != 0xa0)
		return BBK_FRONT_S5K4H7("unknown");
	if (len < 0xc)
		return BBK_FRONT_S5K4H7("Truly");
	if (!buf[11]) {
		if (len < 0xd)
			return BBK_FRONT_S5K4H7("Truly");
		if (buf[12] != 0x12)
			return BBK_FRONT_S5K4H7("unknown");
		if (len < 0xe)
			return BBK_FRONT_S5K4H7("Truly");
		if (buf[13])
			return BBK_FRONT_S5K4H7("unknown");
		if (len < 0xf)
			return BBK_FRONT_S5K4H7("Truly");
		if (buf[14])
			return BBK_FRONT_S5K4H7("unknown");

		return BBK_FRONT_S5K4H7("Truly");
	}

	return BBK_FRONT_S5K4H7("unknown");
}

/**
 * bbk_camera_info_suffix - build the "(<module id>)" suffix
 * @suffix: destination buffer
 * @size:   size of @suffix
 * @buf:    module id bytes
 * @len:    number of valid bytes
 *
 * The factory firmware appends the 16 byte module id string (when the byte
 * at offset 20 is 0x01) to the module vendor line.
 */
static void bbk_camera_info_suffix(char *suffix, size_t size,
	uint8_t *buf, uint32_t len)
{
	char id[BBK_INFO_ID_LEN + 1];

	suffix[0] = '\0';
	if (!buf || len < BBK_INFO_ID_MIN_LEN ||
		buf[BBK_MODULE_ID_LEN] != BBK_INFO_ID_FLAG)
		return;

	memcpy(id, &buf[BBK_INFO_ID_OFFSET], BBK_INFO_ID_LEN);
	id[BBK_INFO_ID_LEN] = '\0';
	snprintf(suffix, size, "(%s)", id);
}

/**
 * bbk_camera_info_dump - print the module id/calibration bytes
 * @m:     seq_file
 * @buf:   module id bytes
 * @len:   number of valid bytes
 * @base:  first register number of the second half of the map
 */
static void bbk_camera_info_dump(struct seq_file *m, uint8_t *buf,
	uint32_t len, uint32_t base)
{
	uint32_t i;

	if (!buf)
		return;

	for (i = 0; i < len; i++) {
		if (i < BBK_MODULE_ID_LEN)
			seq_printf(m, "reg[0x%04x] = 0x%02x, ",
				(unsigned int)i, (unsigned int)buf[i]);
		else
			seq_printf(m, "reg[0x%04x] = 0x%02x, ",
				(unsigned int)(i + base),
				(unsigned int)buf[i]);
	}
}

static int back_camera_info_show(struct seq_file *m, void *v)
{
	char suffix[BBK_INFO_ID_LEN + 8];

	bbk_camera_info_suffix(suffix, sizeof(suffix), bbk_back_info_data,
		bbk_back_info_size);

	seq_printf(m, bbk_back_info_vendor(bbk_back_info_data,
		bbk_back_info_size), suffix);

	bbk_camera_info_dump(m, bbk_back_info_data, bbk_back_info_size,
		BBK_BACK_REG_BASE);

	return 0;
}

static int front_camera_info_show(struct seq_file *m, void *v)
{
	char suffix[BBK_INFO_ID_LEN + 8];

	bbk_camera_info_suffix(suffix, sizeof(suffix), bbk_front_info_data,
		bbk_front_info_size);

	seq_printf(m, bbk_front_info_vendor(bbk_front_info_data,
		bbk_front_info_size), suffix);

	bbk_camera_info_dump(m, bbk_front_info_data, bbk_front_info_size,
		BBK_FRONT_REG_BASE);

	return 0;
}

static int back_camera_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, back_camera_info_show, NULL);
}

static int front_camera_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, front_camera_info_show, NULL);
}

static const struct file_operations back_camera_info_fops = {
	.owner = THIS_MODULE,
	.open = back_camera_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations front_camera_info_fops = {
	.owner = THIS_MODULE,
	.open = front_camera_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int back_camera_info_create_proc(void)
{
	if (!proc_create(BBK_BACK_INFO_PROC, BBK_INFO_PROC_MODE, NULL,
		&back_camera_info_fops)) {
		CAM_ERR(CAM_EEPROM, "%s creat proc %s fail! %d", __func__,
			BBK_BACK_INFO_PROC, -1);
		return -1;
	}

	return 0;
}

static int front_camera_info_create_proc(void)
{
	if (!proc_create(BBK_FRONT_INFO_PROC, BBK_INFO_PROC_MODE, NULL,
		&front_camera_info_fops)) {
		CAM_ERR(CAM_EEPROM, "%s creat proc %s fail! %d", __func__,
			BBK_FRONT_INFO_PROC, -1);
		return -1;
	}

	return 0;
}

/**
 * bbk_camera_info_setup - publish the camera module information
 * @e_ctrl: eeprom control structure
 *
 * Creates /proc/driver/BackCamera_info or /proc/driver/FrontCamera_info
 * depending on the eeprom instance and stores a copy of the EEPROM memory
 * map that was read during probe.  The proc entries are created even when no
 * data is available yet, so that userspace always finds them.
 */
static void bbk_camera_info_setup(struct cam_eeprom_ctrl_t *e_ctrl)
{
	uint8_t *data = e_ctrl->cal_data.mapdata;
	uint32_t size = e_ctrl->cal_data.num_data;
	int32_t id = e_ctrl->soc_info.pdev->id;

	if (id != 0 && id != 1)
		return;

	if (data && size) {
		uint8_t *copy = kmemdup(data, size, GFP_KERNEL);

		if (!copy) {
			CAM_ERR(CAM_EEPROM, "failed: no memory for camera info");
			return;
		}

		if (id == 0) {
			kfree(bbk_back_info_data);
			bbk_back_info_data = copy;
			bbk_back_info_size = size;
		} else {
			kfree(bbk_front_info_data);
			bbk_front_info_data = copy;
			bbk_front_info_size = size;
		}
	}

	if (id == 0)
		back_camera_info_create_proc();
	else
		front_camera_info_create_proc();
}

/**
 * bbk_camera_info_read_module_id - read the module id out of the eeprom
 * @e_ctrl: eeprom control structure
 *
 * The factory firmware powers the camera module up once during probe, reads
 * the EEPROM memory map (module id + calibration) and feeds it to the
 * /proc/driver/*Camera_info entries.
 */
static void bbk_camera_info_read_module_id(struct cam_eeprom_ctrl_t *e_ctrl)
{
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct device_node *of_node = e_ctrl->soc_info.dev->of_node;
	uint32_t temp;
	int32_t rc;

	if (!soc_private || !of_node)
		return;

	rc = of_property_read_u32(of_node, "cell-index",
		&e_ctrl->soc_info.pdev->id);
	if (rc < 0)
		CAM_DBG(CAM_EEPROM, "cell-index not found rc %d", rc);

	rc = of_property_read_u32(of_node, "qcom,slave-addr", &temp);
	if (!rc) {
		soc_private->i2c_info.slave_addr = temp;
		rc = of_property_read_u32(of_node, "qcom,i2c-freq-mode", &temp);
		if (!rc)
			soc_private->i2c_info.i2c_freq_mode = temp;
		else
			CAM_DBG(CAM_EEPROM, "qcom,i2c-freq-mode rc %d", rc);
	} else {
		CAM_DBG(CAM_EEPROM, "qcom,slave-addr rc %d", rc);
	}

	CAM_ERR(CAM_EEPROM,
		"KERNEL_CHECK_MODULE_ID e_ctrl->soc_info.pdev->id: %d, slave-addr: 0x%x, i2c-freq-mode: %d",
		e_ctrl->soc_info.pdev->id, soc_private->i2c_info.slave_addr,
		soc_private->i2c_info.i2c_freq_mode);

	rc = cam_eeprom_update_i2c_info(e_ctrl, &soc_private->i2c_info);
	if (rc)
		CAM_ERR(CAM_EEPROM, "failed: to update i2c info rc %d", rc);

	rc = cam_get_dt_power_setting_data(of_node, &e_ctrl->soc_info,
		&soc_private->power_info);
	if (rc < 0)
		CAM_ERR(CAM_EEPROM,
			"KERNEL_CHECK_MODULE_ID failed in getting power settings");

	rc = cam_eeprom_parse_read_memory_map(of_node, e_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM,
			"KERNEL_CHECK_MODULE_ID Failed: rc : %d", rc);
		bbk_camera_info_setup(e_ctrl);
		return;
	}

	bbk_camera_info_setup(e_ctrl);

	vfree(e_ctrl->cal_data.mapdata);
	vfree(e_ctrl->cal_data.map);
	e_ctrl->cal_data.mapdata = NULL;
	e_ctrl->cal_data.map = NULL;
	e_ctrl->cal_data.num_data = 0;
	e_ctrl->cal_data.num_map = 0;

	CAM_INFO(CAM_EEPROM, "KERNEL_CHECK_MODULE_ID Done");
}

#ifdef CONFIG_COMPAT
static long cam_eeprom_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_EEPROM,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_eeprom_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc < 0) {
			CAM_ERR(CAM_EEPROM,
				"Failed in eeprom suddev handling rc %d",
				rc);
			return rc;
		}
		break;
	default:
		CAM_ERR(CAM_EEPROM, "Invalid compat ioctl: %d", cmd);
		rc = -EINVAL;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_EEPROM,
				"Failed to copy from user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

static const struct v4l2_subdev_internal_ops cam_eeprom_internal_ops = {
	.close = cam_eeprom_subdev_close,
};

static struct v4l2_subdev_core_ops cam_eeprom_subdev_core_ops = {
	.ioctl = cam_eeprom_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_eeprom_init_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_eeprom_subdev_ops = {
	.core = &cam_eeprom_subdev_core_ops,
};

static int cam_eeprom_init_subdev(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int rc = 0;

	e_ctrl->v4l2_dev_str.internal_ops = &cam_eeprom_internal_ops;
	e_ctrl->v4l2_dev_str.ops = &cam_eeprom_subdev_ops;
	strlcpy(e_ctrl->device_name, CAM_EEPROM_NAME,
		sizeof(e_ctrl->device_name));
	e_ctrl->v4l2_dev_str.name = e_ctrl->device_name;
	e_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	e_ctrl->v4l2_dev_str.ent_function = CAM_EEPROM_DEVICE_TYPE;
	e_ctrl->v4l2_dev_str.token = e_ctrl;

	rc = cam_register_subdev(&(e_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_SENSOR, "Fail with cam_register_subdev");

	return rc;
}

static int cam_eeprom_i2c_driver_probe(struct i2c_client *client,
	 const struct i2c_device_id *id)
{
	int                             rc = 0;
	struct cam_eeprom_ctrl_t       *e_ctrl = NULL;
	struct cam_eeprom_soc_private  *soc_private = NULL;
	struct cam_hw_soc_info         *soc_info = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_EEPROM, "i2c_check_functionality failed");
		goto probe_failure;
	}

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "kzalloc failed");
		rc = -ENOMEM;
		goto probe_failure;
	}

	soc_private = kzalloc(sizeof(*soc_private), GFP_KERNEL);
	if (!soc_private)
		goto ectrl_free;

	e_ctrl->soc_info.soc_private = soc_private;

	i2c_set_clientdata(client, e_ctrl);

	mutex_init(&(e_ctrl->eeprom_mutex));

	soc_info = &e_ctrl->soc_info;
	soc_info->dev = &client->dev;
	soc_info->dev_name = client->name;
	e_ctrl->io_master_info.master_type = I2C_MASTER;
	e_ctrl->io_master_info.client = client;
	e_ctrl->eeprom_device_type = MSM_CAMERA_I2C_DEVICE;
	e_ctrl->cal_data.mapdata = NULL;
	e_ctrl->cal_data.map = NULL;
	e_ctrl->userspace_probe = false;

	rc = cam_eeprom_parse_dt(e_ctrl);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: soc init rc %d", rc);
		goto free_soc;
	}

	rc = cam_eeprom_update_i2c_info(e_ctrl, &soc_private->i2c_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: to update i2c info rc %d", rc);
		goto free_soc;
	}

	rc = cam_eeprom_init_subdev(e_ctrl);
	if (rc)
		goto free_soc;

	if (soc_private->i2c_info.slave_addr != 0)
		e_ctrl->io_master_info.client->addr =
			soc_private->i2c_info.slave_addr;

	e_ctrl->bridge_intf.device_hdl = -1;
	e_ctrl->bridge_intf.ops.get_dev_info = NULL;
	e_ctrl->bridge_intf.ops.link_setup = NULL;
	e_ctrl->bridge_intf.ops.apply_req = NULL;
	e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;

	return rc;
free_soc:
	kfree(soc_private);
ectrl_free:
	kfree(e_ctrl);
probe_failure:
	return rc;
}

static int cam_eeprom_i2c_driver_remove(struct i2c_client *client)
{
	int                             i;
	struct v4l2_subdev             *sd = i2c_get_clientdata(client);
	struct cam_eeprom_ctrl_t       *e_ctrl;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_hw_soc_info         *soc_info;

	if (!sd) {
		CAM_ERR(CAM_EEPROM, "Subdevice is NULL");
		return -EINVAL;
	}

	e_ctrl = (struct cam_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "eeprom device is NULL");
		return -EINVAL;
	}

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_EEPROM, "soc_info.soc_private is NULL");
		return -EINVAL;
	}

	CAM_INFO(CAM_EEPROM, "i2c driver remove invoked");
	soc_info = &e_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(e_ctrl->eeprom_mutex));
	cam_eeprom_shutdown(e_ctrl);
	mutex_unlock(&(e_ctrl->eeprom_mutex));
	mutex_destroy(&(e_ctrl->eeprom_mutex));
	cam_unregister_subdev(&(e_ctrl->v4l2_dev_str));
	kfree(soc_private);
	v4l2_set_subdevdata(&e_ctrl->v4l2_dev_str.sd, NULL);
	kfree(e_ctrl);

	return 0;
}

static int cam_eeprom_spi_setup(struct spi_device *spi)
{
	struct cam_eeprom_ctrl_t       *e_ctrl = NULL;
	struct cam_hw_soc_info         *soc_info = NULL;
	struct cam_sensor_spi_client   *spi_client;
	struct cam_eeprom_soc_private  *eb_info;
	struct cam_sensor_power_ctrl_t *power_info = NULL;
	int                             rc = 0;

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl)
		return -ENOMEM;

	soc_info = &e_ctrl->soc_info;
	soc_info->dev = &spi->dev;
	soc_info->dev_name = spi->modalias;

	e_ctrl->v4l2_dev_str.ops = &cam_eeprom_subdev_ops;
	e_ctrl->userspace_probe = false;
	e_ctrl->cal_data.mapdata = NULL;
	e_ctrl->cal_data.map = NULL;

	spi_client = kzalloc(sizeof(*spi_client), GFP_KERNEL);
	if (!spi_client) {
		kfree(e_ctrl);
		return -ENOMEM;
	}

	eb_info = kzalloc(sizeof(*eb_info), GFP_KERNEL);
	if (!eb_info)
		goto spi_free;
	e_ctrl->soc_info.soc_private = eb_info;

	e_ctrl->eeprom_device_type = MSM_CAMERA_SPI_DEVICE;
	e_ctrl->io_master_info.spi_client = spi_client;
	e_ctrl->io_master_info.master_type = SPI_MASTER;
	spi_client->spi_master = spi;

	power_info = &eb_info->power_info;
	power_info->dev = &spi->dev;

	/* set spi instruction info */
	spi_client->retry_delay = 1;
	spi_client->retries = 0;

	/* Initialize mutex */
	mutex_init(&(e_ctrl->eeprom_mutex));

	rc = cam_eeprom_parse_dt(e_ctrl);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: spi soc init rc %d", rc);
		goto board_free;
	}

	rc = cam_eeprom_spi_parse_of(spi_client);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "Device tree parsing error");
		goto board_free;
	}

	rc = cam_eeprom_init_subdev(e_ctrl);
	if (rc)
		goto board_free;

	e_ctrl->bridge_intf.device_hdl = -1;
	e_ctrl->bridge_intf.ops.get_dev_info = NULL;
	e_ctrl->bridge_intf.ops.link_setup = NULL;
	e_ctrl->bridge_intf.ops.apply_req = NULL;

	v4l2_set_subdevdata(&e_ctrl->v4l2_dev_str.sd, e_ctrl);
	return rc;

board_free:
	kfree(e_ctrl->soc_info.soc_private);
spi_free:
	kfree(spi_client);
	kfree(e_ctrl);
	return rc;
}

static int cam_eeprom_spi_driver_probe(struct spi_device *spi)
{
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi_setup(spi);

	CAM_DBG(CAM_EEPROM, "irq[%d] cs[%x] CPHA[%x] CPOL[%x] CS_HIGH[%x]",
		spi->irq, spi->chip_select, (spi->mode & SPI_CPHA) ? 1 : 0,
		(spi->mode & SPI_CPOL) ? 1 : 0,
		(spi->mode & SPI_CS_HIGH) ? 1 : 0);
	CAM_DBG(CAM_EEPROM, "max_speed[%u]", spi->max_speed_hz);

	return cam_eeprom_spi_setup(spi);
}

static int cam_eeprom_spi_driver_remove(struct spi_device *sdev)
{
	int                             i;
	struct v4l2_subdev             *sd = spi_get_drvdata(sdev);
	struct cam_eeprom_ctrl_t       *e_ctrl;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_hw_soc_info         *soc_info;

	if (!sd) {
		CAM_ERR(CAM_EEPROM, "Subdevice is NULL");
		return -EINVAL;
	}

	e_ctrl = (struct cam_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "eeprom device is NULL");
		return -EINVAL;
	}

	soc_info = &e_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(e_ctrl->eeprom_mutex));
	cam_eeprom_shutdown(e_ctrl);
	mutex_unlock(&(e_ctrl->eeprom_mutex));
	mutex_destroy(&(e_ctrl->eeprom_mutex));
	cam_unregister_subdev(&(e_ctrl->v4l2_dev_str));
	kfree(e_ctrl->io_master_info.spi_client);
	e_ctrl->io_master_info.spi_client = NULL;
	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	if (soc_private) {
		kfree(soc_private->power_info.gpio_num_info);
		soc_private->power_info.gpio_num_info = NULL;
		kfree(soc_private);
		soc_private = NULL;
	}
	v4l2_set_subdevdata(&e_ctrl->v4l2_dev_str.sd, NULL);
	kfree(e_ctrl);

	return 0;
}

static int32_t cam_eeprom_platform_driver_probe(
	struct platform_device *pdev)
{
	int32_t                         rc = 0;
	struct cam_eeprom_ctrl_t       *e_ctrl = NULL;
	struct cam_eeprom_soc_private  *soc_private = NULL;

	e_ctrl = kzalloc(sizeof(struct cam_eeprom_ctrl_t), GFP_KERNEL);
	if (!e_ctrl)
		return -ENOMEM;

	e_ctrl->soc_info.pdev = pdev;
	e_ctrl->soc_info.dev = &pdev->dev;
	e_ctrl->soc_info.dev_name = pdev->name;
	e_ctrl->eeprom_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	e_ctrl->cal_data.mapdata = NULL;
	e_ctrl->cal_data.map = NULL;
	e_ctrl->userspace_probe = false;

	e_ctrl->io_master_info.master_type = CCI_MASTER;
	e_ctrl->io_master_info.cci_client = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!e_ctrl->io_master_info.cci_client) {
		rc = -ENOMEM;
		goto free_e_ctrl;
	}

	soc_private = kzalloc(sizeof(struct cam_eeprom_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_cci_client;
	}
	e_ctrl->soc_info.soc_private = soc_private;
	soc_private->power_info.dev = &pdev->dev;

	/* Initialize mutex */
	mutex_init(&(e_ctrl->eeprom_mutex));
	rc = cam_eeprom_parse_dt(e_ctrl);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: soc init rc %d", rc);
		goto free_soc;
	}
	rc = cam_eeprom_update_i2c_info(e_ctrl, &soc_private->i2c_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: to update i2c info rc %d", rc);
		goto free_soc;
	}

	/* EEBBK: read the camera module id and publish it through /proc */
	bbk_camera_info_read_module_id(e_ctrl);

	rc = cam_eeprom_init_subdev(e_ctrl);
	if (rc)
		goto free_soc;

	e_ctrl->bridge_intf.device_hdl = -1;
	e_ctrl->bridge_intf.ops.get_dev_info = NULL;
	e_ctrl->bridge_intf.ops.link_setup = NULL;
	e_ctrl->bridge_intf.ops.apply_req = NULL;
	platform_set_drvdata(pdev, e_ctrl);
	e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;

	return rc;
free_soc:
	kfree(soc_private);
free_cci_client:
	kfree(e_ctrl->io_master_info.cci_client);
free_e_ctrl:
	kfree(e_ctrl);

	return rc;
}

static int cam_eeprom_platform_driver_remove(struct platform_device *pdev)
{
	int                        i;
	struct cam_eeprom_ctrl_t  *e_ctrl;
	struct cam_hw_soc_info    *soc_info;

	e_ctrl = platform_get_drvdata(pdev);
	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "eeprom device is NULL");
		return -EINVAL;
	}

	CAM_INFO(CAM_EEPROM, "Platform driver remove invoked");
	soc_info = &e_ctrl->soc_info;

	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(e_ctrl->eeprom_mutex));
	cam_eeprom_shutdown(e_ctrl);
	mutex_unlock(&(e_ctrl->eeprom_mutex));
	mutex_destroy(&(e_ctrl->eeprom_mutex));
	cam_unregister_subdev(&(e_ctrl->v4l2_dev_str));
	kfree(soc_info->soc_private);
	kfree(e_ctrl->io_master_info.cci_client);
	platform_set_drvdata(pdev, NULL);
	v4l2_set_subdevdata(&e_ctrl->v4l2_dev_str.sd, NULL);
	kfree(e_ctrl);

	return 0;
}

static const struct of_device_id cam_eeprom_dt_match[] = {
	{ .compatible = "qcom,eeprom" },
	{ }
};


MODULE_DEVICE_TABLE(of, cam_eeprom_dt_match);

static struct platform_driver cam_eeprom_platform_driver = {
	.driver = {
		.name = "qcom,eeprom",
		.owner = THIS_MODULE,
		.of_match_table = cam_eeprom_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = cam_eeprom_platform_driver_probe,
	.remove = cam_eeprom_platform_driver_remove,
};

static const struct i2c_device_id cam_eeprom_i2c_id[] = {
	{ "msm_eeprom", (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver cam_eeprom_i2c_driver = {
	.id_table = cam_eeprom_i2c_id,
	.probe  = cam_eeprom_i2c_driver_probe,
	.remove = cam_eeprom_i2c_driver_remove,
	.driver = {
		.name = "msm_eeprom",
	},
};

static struct spi_driver cam_eeprom_spi_driver = {
	.driver = {
		.name = "qcom_eeprom",
		.owner = THIS_MODULE,
		.of_match_table = cam_eeprom_dt_match,
	},
	.probe = cam_eeprom_spi_driver_probe,
	.remove = cam_eeprom_spi_driver_remove,
};
static int __init cam_eeprom_driver_init(void)
{
	int rc = 0;

	rc = platform_driver_register(&cam_eeprom_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "platform_driver_register failed rc = %d",
			rc);
		return rc;
	}

	rc = spi_register_driver(&cam_eeprom_spi_driver);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "spi_register_driver failed rc = %d", rc);
		return rc;
	}

	rc = i2c_add_driver(&cam_eeprom_i2c_driver);
	if (rc < 0) {
		CAM_ERR(CAM_EEPROM, "i2c_add_driver failed rc = %d", rc);
		return rc;
	}

	return rc;
}

static void __exit cam_eeprom_driver_exit(void)
{
	platform_driver_unregister(&cam_eeprom_platform_driver);
	spi_unregister_driver(&cam_eeprom_spi_driver);
	i2c_del_driver(&cam_eeprom_i2c_driver);
}

module_init(cam_eeprom_driver_init);
module_exit(cam_eeprom_driver_exit);
MODULE_DESCRIPTION("CAM EEPROM driver");
MODULE_LICENSE("GPL v2");
