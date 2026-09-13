/*
 * Magnachip MXM1120 hall / magnetic sensor driver for the EEBBK S6 (P20H130).
 *
 * Reconstructed from the vendor kernel binary in the factory boot image: the
 * vendor sources are not part of this tree, so the register map, the access
 * protocol and the measurement decoding were recovered by disassembling the
 * factory Image (its kallsyms still carries the symbol names).
 *
 * Recovered symbols
 *   m1120_i2c_drv_probe (x2: up/down), m1120_i2c_set_reg(_up), m1120_measure(_up),
 *   m1120_irq_handler, m1120_set_enable, m1120_set_delay, m1120_set_operation_mode,
 *   m1120_update_interrupt_threshold(_up), m1120_misc_dev_ioctl, m1120_data_show,
 *   m1120_dump_show, m1120_input_dev_init
 *
 * I2C protocol
 *   write: buf[0] = reg, buf[1] = value, one i2c_msg (len 2) via i2c_transfer()
 *   read : i2c_smbus_read_i2c_block_data(client, reg, len, buf)
 *
 * Register map
 *   0x00  device id / status   read 1 byte, expected 0x9c on a healthy part;
 *                              bits 7:6 are the two high bits of the field in
 *                              10-bit mode
 *   0x01  control              written 0x00, then (value & 0x7f)
 *   0x07  control              written 0x01
 *   0x08  operation mode       written 0x40 (measurement), (value & 0xfe)
 *   0x10  3-byte measurement   buf[0] = status (bit 0 = DRDY), buf[1] = low
 *                              bits, buf[2] = high bits
 *   0x00..0x0a                 dumped by the vendor's debug path
 *
 * Init sequence recovered from m1120_i2c_drv_probe
 *   write 0x07 = 0x01
 *   read  0x00 -> must be 0x9c
 *   write 0x00 = 0x40
 *   write 0x01 = 0x00
 *   write 0x08 = 0x40
 *   write 0x01 = (old & 0x7f)
 *   write 0x08 = (old & 0xfe)
 *
 * Measurement decoding recovered from m1120_measure
 *   10-bit mode (flags & 2 == 0):  v = buf[1] | ((buf[2] >> 6) << 8), sign
 *                                  extended from bit 9
 *    8-bit mode (flags & 2 != 0):  v = buf[2] & 0x7f,  sign extended from bit 7
 *   the result is a signed 16-bit value, which is what the factory's input
 *   device reported on ABS_X (-32768..32767)
 *
 * The sensor is exposed as an input device named "m1120_up" / "m1120_down"
 * (exactly the names the factory kernel created, see /proc/bus/input/devices)
 * plus sysfs attributes for enable/delay/data/vendor.
 *
 * SPDX-License-Identifier: GPL-2.0
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>

/*
 * The BBK hall framework.  Its header is not part of this tree, so the two
 * structures the registration protocol needs are declared here, with the layout
 * recovered from the factory binary: hall_ops is {get_data, set_enable} and
 * hall_dev is {char name[24]; struct hall_ops *ops; void *data;}, and the name
 * must start with "up" or "down" for the framework to accept it.
 */
struct hall_ops {
	int (*get_data)(void *data, s16 *value);
	int (*set_enable)(void *data, int on);
};

struct hall_dev {
	char		name[24];
	struct hall_ops	*ops;
	void		*data;
};

extern int bbk_hall_core_register_device(struct hall_dev *dev);
extern void bbk_hall_core_unregister_device(struct hall_dev *dev);

#define M1120_DRV_NAME		"mxm1120"

#define M1120_REG_CMD		0x00
#define M1120_REG_CHIPID	0x09
#define M1120_REG_CTRL1		0x01
#define M1120_REG_CTRL2		0x07
#define M1120_REG_OPMODE	0x08
#define M1120_REG_DATA		0x10

/*
 * [RE] the factory probe reads one byte from register 0x09 and refuses the
 * part unless it is 0x9c; both instances on this board (i2c 0x0c and 0x0f)
 * answer 0x9c, which is what the earlier reconstruction got wrong when it
 * looked at register 0x00 instead.
 */
#define M1120_ID_EXPECT		0x9c
#define M1120_DATA_LEN		3
#define M1120_DUMP_LEN		0x14
#define M1120_OP_MEASUREMENT	0x40

enum m1120_pos {
	M1120_POS_UP = 0,
	M1120_POS_DOWN,
	M1120_POS_MIDDLE,
};

struct m1120_data {
	struct i2c_client	*client;
	struct device		*dev;
	struct input_dev	*input;
	struct mutex		lock;
	struct delayed_work	work;
	struct regulator	*vdd;
	struct regulator	*vddio;
	int			reset_gpio;
	enum m1120_pos		pos;
	u8			device_id;
	bool			enabled;
	bool			ten_bit;
	bool			use_interrupt;
	bool			use_hrtimer;
	unsigned int		init_interval;
	unsigned int		delay_ms;
	char			line[128];
	s16			last;
	/* the BBK hall framework registration */
	struct hall_ops		hall_ops;
	struct hall_dev		hall;
};

static int m1120_i2c_set_reg(struct m1120_data *d, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	struct i2c_msg msg = {
		.addr	= d->client->addr,
		.flags	= 0,
		.len	= sizeof(buf),
		.buf	= buf,
	};
	int ret;

	/* [RE] the vendor helper issues a single message carrying reg+value */
	ret = i2c_transfer(d->client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(d->dev, "%s: i2c write 0x%02x=0x%02x failed (%d)\n",
			__func__, reg, val, ret);
		return ret;
	}
	return 0;
}

static int m1120_read_block(struct m1120_data *d, u8 reg, u8 len, u8 *buf)
{
	int ret = i2c_smbus_read_i2c_block_data(d->client, reg, len, buf);

	if (ret < 0) {
		dev_err(d->dev, "%s: i2c read 0x%02x failed (%d)\n",
			__func__, reg, ret);
		return ret;
	}
	return 0;
}

static int m1120_get_id(struct m1120_data *d, u8 *id)
{
	return m1120_read_block(d, M1120_REG_CHIPID, 1, id);
}

static int m1120_init_device(struct m1120_data *d)
{
	u8 v = 0;
	int ret;

	/* [RE] the exact sequence the vendor probe used */
	ret = m1120_i2c_set_reg(d, M1120_REG_CTRL2, 0x01);
	if (ret)
		return ret;

	ret = m1120_get_id(d, &d->device_id);
	if (ret)
		return ret;
	if (d->device_id != M1120_ID_EXPECT)
		dev_warn(d->dev,
			 "%s: device id 0x%02X, expected 0x%02X, continuing\n",
			 __func__, d->device_id, M1120_ID_EXPECT);

	ret = m1120_i2c_set_reg(d, M1120_REG_CMD, 0x40);
	if (ret)
		return ret;
	ret = m1120_i2c_set_reg(d, M1120_REG_CTRL1, 0x00);
	if (ret)
		return ret;
	ret = m1120_i2c_set_reg(d, M1120_REG_OPMODE, M1120_OP_MEASUREMENT);
	if (ret)
		return ret;

	/* read-modify-write of 0x01 and 0x08, as in the vendor probe */
	ret = m1120_read_block(d, M1120_REG_CTRL1, 1, &v);
	if (!ret)
		ret = m1120_i2c_set_reg(d, M1120_REG_CTRL1, v & 0x7f);
	if (ret)
		return ret;
	ret = m1120_read_block(d, M1120_REG_OPMODE, 1, &v);
	if (!ret)
		ret = m1120_i2c_set_reg(d, M1120_REG_OPMODE, v & 0xfe);

	dev_info(d->dev, "%s: initializing device was success (id 0x%02x)\n",
		 __func__, d->device_id);
	return ret;
}

static int m1120_set_operation_mode(struct m1120_data *d, bool measure)
{
	u8 v = 0;
	int ret;

	ret = m1120_read_block(d, M1120_REG_OPMODE, 1, &v);
	if (ret)
		return ret;
	v = measure ? (v | M1120_OP_MEASUREMENT) : (v & ~M1120_OP_MEASUREMENT);
	return m1120_i2c_set_reg(d, M1120_REG_OPMODE, v);
}

/*
 * One measurement.  [RE] m1120_measure in the factory kernel reads three bytes
 * from register 0x10 and refuses the sample unless bit 0 of the first byte is
 * set; the caller turns that refusal into the value -2000.  The reading is
 * buf[1] plus bit 8 taken from the top bits of buf[2], sign extended from bit 9
 * in the 10 bit mode and a plain signed byte of buf[1] in the 8 bit mode.
 *
 * Reading the high bits from register 0x00 instead - as this driver first did -
 * returns a constant: register 0x00 holds 0x40, so every sample decoded to 256.
 */
static int m1120_measure(struct m1120_data *d, s16 *value)
{
	u8 buf[M1120_DATA_LEN];
	int v;
	int ret = m1120_read_block(d, M1120_REG_DATA, M1120_DATA_LEN, buf);

	if (ret)
		return ret;

	if (!(buf[0] & 0x01)) {
		dev_dbg(d->dev, "%s: damon st1(0x%02X) is not DRDY\n", __func__, buf[0]);
		return -1;
	}

	if (d->ten_bit) {
		v = buf[1] | (((buf[2] >> 6) & 0x1) << 8);
		if (buf[2] & 0x80)
			v |= ~0x3ff;			/* sign extend bit 9 */
	} else {
		v = buf[1] & 0x7f;
		if (buf[1] & 0x80)
			v |= ~0x7f;			/* sign extend bit 7 */
	}

	*value = (s16)v;
	dev_dbg(d->dev, "%s: raw = %d (st 0x%02x lo 0x%02x hi 0x%02x)\n",
		__func__, *value, buf[0], buf[1], buf[2]);
	return 0;
}

static void m1120_work(struct work_struct *work)
{
	struct m1120_data *d = container_of(to_delayed_work(work),
					    struct m1120_data, work);
	s16 value;
	int ret;

	mutex_lock(&d->lock);
	ret = m1120_measure(d, &value);
	if (!ret) {
		d->last = value;
		if (d->input)
			input_report_abs(d->input, ABS_X, value);
		if (d->input)
			input_sync(d->input);
		scnprintf(d->line, sizeof(d->line), "m1120 [%s] raw = %d\n",
			  d->pos == M1120_POS_UP ? "up" :
			  (d->pos == M1120_POS_DOWN ? "down" : "middle"), value);
	}
	if (d->enabled)
		schedule_delayed_work(&d->work, msecs_to_jiffies(d->delay_ms));
	mutex_unlock(&d->lock);
}

/* ------------------------------------------------ BBK hall framework ops -- */

/*
 * [RE] the framework calls get_data() to take one sample and set_enable() to
 * turn the sensor on and off; both take the hall_dev's data pointer, which is
 * this driver's private data.
 */
static int m1120_hall_get_data(void *data, s16 *value)
{
	struct m1120_data *d = data;
	int ret;

	if (!d || !value)
		return -EINVAL;
	mutex_lock(&d->lock);
	ret = m1120_measure(d, value);
	if (!ret) {
		d->last = *value;
		scnprintf(d->line, sizeof(d->line), "m1120 [%s] raw = %d\n",
			  d->pos == M1120_POS_UP ? "up" :
			  (d->pos == M1120_POS_DOWN ? "down" : "middle"), *value);
	}
	mutex_unlock(&d->lock);
	return ret;
}

static int m1120_hall_set_enable(void *data, int on)
{
	struct m1120_data *d = data;

	if (!d)
		return -EINVAL;
	mutex_lock(&d->lock);
	d->enabled = !!on;
	if (d->enabled) {
		m1120_set_operation_mode(d, true);
		schedule_delayed_work(&d->work, 0);
	} else {
		cancel_delayed_work(&d->work);
		m1120_set_operation_mode(d, false);
	}
	mutex_unlock(&d->lock);
	return 0;
}

/*
 * Register with the framework.  The factory DT names the two instances
 * "magnachip,mxm1120,up" and "magnachip,mxm1120,down", and the factory driver
 * registered them as the framework's up and down sensors, so the name is taken
 * from the device tree compatible.
 */
static int m1120_hall_register(struct m1120_data *d)
{
	d->hall_ops.get_data = m1120_hall_get_data;
	d->hall_ops.set_enable = m1120_hall_set_enable;
	d->hall.ops = &d->hall_ops;
	d->hall.data = d;

	if (d->pos == M1120_POS_UP)
		strlcpy(d->hall.name, "up-mxm1120", sizeof(d->hall.name));
	else if (d->pos == M1120_POS_DOWN)
		strlcpy(d->hall.name, "down-mxm1120", sizeof(d->hall.name));
	else
		strlcpy(d->hall.name, "mid-mxm1120", sizeof(d->hall.name));

	return bbk_hall_core_register_device(&d->hall);
}

/* ------------------------------------------------------------------ sysfs -- */

static ssize_t data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m1120_data *d = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s", d->line);
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m1120_data *d = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", d->enabled ? 1 : 0);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct m1120_data *d = dev_get_drvdata(dev);
	unsigned long v;

	if (kstrtoul(buf, 0, &v))
		return -EINVAL;

	mutex_lock(&d->lock);
	d->enabled = !!v;
	if (d->enabled) {
		m1120_set_operation_mode(d, true);
		schedule_delayed_work(&d->work, 0);
	} else {
		cancel_delayed_work(&d->work);
		m1120_set_operation_mode(d, false);
	}
	mutex_unlock(&d->lock);
	return count;
}

static ssize_t delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m1120_data *d = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", d->delay_ms);
}

static ssize_t delay_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct m1120_data *d = dev_get_drvdata(dev);
	unsigned long v;

	if (kstrtoul(buf, 0, &v))
		return -EINVAL;
	d->delay_ms = (unsigned int)v;
	return count;
}

/* [RE] the vendor's dump attribute: registers 0x00 to 0x13, byte by byte */
static ssize_t dump_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m1120_data *d = dev_get_drvdata(dev);
	u8 regs[M1120_DUMP_LEN];
	u8 blk[M1120_DATA_LEN];
	int i, n = 0, ret;

	mutex_lock(&d->lock);
	for (i = 0; i < M1120_DUMP_LEN; i++) {
		ret = m1120_read_block(d, (u8)i, 1, &regs[i]);
		if (ret) {
			mutex_unlock(&d->lock);
			return scnprintf(buf, PAGE_SIZE,
					 "read reg 0x%02x failed (%d)\n", i, ret);
		}
	}
	mutex_unlock(&d->lock);

	for (i = 0; i < M1120_DUMP_LEN; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n, "[%02x]=%02x ", i, regs[i]);
	n += scnprintf(buf + n, PAGE_SIZE - n, "\n");

	/* the block the measurement actually uses */
	mutex_lock(&d->lock);
	ret = m1120_read_block(d, M1120_REG_DATA, M1120_DATA_LEN, blk);
	mutex_unlock(&d->lock);
	if (!ret)
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "0x10 block: st=0x%02x lo=0x%02x hi=0x%02x ",
			       blk[0], blk[1], blk[2]);
	n += scnprintf(buf + n, PAGE_SIZE - n, "ten_bit=%d enabled=%d id=0x%02x\n",
		       d->ten_bit, d->enabled, d->device_id);
	return n;
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "magnachip\n");
}

static DEVICE_ATTR_RO(data);
static DEVICE_ATTR_RO(dump);
static DEVICE_ATTR_RW(enable);
static DEVICE_ATTR_RW(delay);
static DEVICE_ATTR_RO(vendor);

static struct attribute *m1120_attrs[] = {
	&dev_attr_data.attr,
	&dev_attr_dump.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_vendor.attr,
	NULL,
};
ATTRIBUTE_GROUPS(m1120);

static int m1120_input_init(struct m1120_data *d)
{
	struct input_dev *input;
	const char *name;
	int ret;

	name = (d->pos == M1120_POS_UP) ? "m1120_up" :
	       (d->pos == M1120_POS_DOWN) ? "m1120_down" : "m1120_middle";

	input = devm_input_allocate_device(d->dev);
	if (!input)
		return -ENOMEM;

	input->name = name;
	input->phys = "m1120/input0";
	input->id.bustype = BUS_I2C;
	input->dev.parent = d->dev;

	__set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_X, -32768, 32767, 0, 0);

	ret = input_register_device(input);
	if (ret) {
		dev_err(d->dev, "%s: m1120_input_dev_init was failed(%d)\n", __func__, ret);
		return ret;
	}
	d->input = input;

	dev_info(d->dev, "%s: %s was initialized\n", __func__, name);
	return 0;
}

static int m1120_parse_dt(struct m1120_data *d)
{
	struct device_node *np = d->dev->of_node;
	u32 v;

	/*
	 * [RE] the vendor probe parsed exactly these properties:
	 *   magnachip,init-interval   u32  -> data->init_interval (0 becomes 1)
	 *   magnachip,use-interrupt   bool -> data->use_interrupt
	 *   magnachip,gpio-int        gpio -> data->gpio_int
	 *   magnachip,use-hrtimer     bool -> data->use_hrtimer
	 * The factory DT only sets init-interval = <1>, i.e. interrupt and
	 * hrtimer mode are off and the sensor is polled.
	 *
	 * [RE] the vendor queued its work with a delay of 20, so one interval
	 * unit is taken as 20 ms; the factory value of 1 therefore polls at
	 * 20 ms, which is also the vendor's default when the property is
	 * missing (it forces the value to 1).
	 */
	d->init_interval = 1;
	d->delay_ms = 20;
	d->ten_bit = true;

	if (np && !of_property_read_u32(np, "magnachip,init-interval", &v) && v)
		d->init_interval = v;
	d->delay_ms = d->init_interval * 20;

	/* parsed for completeness; interrupt and hrtimer modes are not wired up */
	d->use_interrupt = np && of_property_read_bool(np, "magnachip,use-interrupt");
	d->use_hrtimer = np && of_property_read_bool(np, "magnachip,use-hrtimer");
	if (d->use_interrupt || d->use_hrtimer)
		dev_info(d->dev,
			 "%s: interrupt/hrtimer mode requested (interrupt %d, hrtimer %d), falling back to polling\n",
			 __func__, d->use_interrupt, d->use_hrtimer);

	d->reset_gpio = np ? of_get_named_gpio(np, "magnachip,gpio-int", 0) : -ENOENT;
	if (gpio_is_valid(d->reset_gpio)) {
		/* the vendor used this gpio as its interrupt line */
		dev_info(d->dev, "%s: magnachip,gpio-int = %d (polled, gpio left as is)\n",
			 __func__, d->reset_gpio);
		d->reset_gpio = -ENOENT;
	}
	return 0;
}

static const struct of_device_id m1120_of_match[] = {
	{ .compatible = "magnachip,mxm1120,up",     .data = (void *)M1120_POS_UP },
	{ .compatible = "magnachip,mxm1120,down",   .data = (void *)M1120_POS_DOWN },
	{ .compatible = "magnachip,mxm1120,middle", .data = (void *)M1120_POS_MIDDLE },
	{ }
};
MODULE_DEVICE_TABLE(of, m1120_of_match);

static const struct i2c_device_id m1120_id[] = {
	{ "mxm1120_up", M1120_POS_UP },
	{ "mxm1120_down", M1120_POS_DOWN },
	{ }
};
MODULE_DEVICE_TABLE(i2c, m1120_id);

static int m1120_i2c_drv_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct m1120_data *d;
	const struct of_device_id *match;
	int ret;

	d = devm_kzalloc(&client->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->client = client;
	d->dev = &client->dev;
	mutex_init(&d->lock);
	INIT_DELAYED_WORK(&d->work, m1120_work);
	i2c_set_clientdata(client, d);
	dev_set_drvdata(&client->dev, d);
	strlcpy(d->line, "m1120: no data yet\n", sizeof(d->line));

	match = i2c_of_match_device(m1120_of_match, client);
	d->pos = match ? (enum m1120_pos)(uintptr_t)match->data : M1120_POS_UP;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_I2C)) {
		dev_err(d->dev, "%s: i2c_check_functionality was failed\n", __func__);
		return -EOPNOTSUPP;
	}

	/* optional supplies: the vendor probe requested two regulators */
	d->vdd = devm_regulator_get_optional(d->dev, "vdd");
	if (IS_ERR(d->vdd))
		d->vdd = NULL;
	d->vddio = devm_regulator_get_optional(d->dev, "vddio");
	if (IS_ERR(d->vddio))
		d->vddio = NULL;
	if (d->vdd)
		regulator_enable(d->vdd);
	if (d->vddio)
		regulator_enable(d->vddio);

	m1120_parse_dt(d);

	ret = m1120_init_device(d);
	if (ret) {
		dev_err(d->dev, "%s: m1120_init_device was failed(%d)\n", __func__, ret);
		goto err_regs;
	}

	ret = m1120_input_init(d);
	if (ret)
		goto err_regs;

	ret = devm_device_add_groups(d->dev, m1120_groups);
	if (ret) {
		dev_err(d->dev, "%s: sysfs_create_group was failed(%d)\n", __func__, ret);
		goto err_regs;
	}

	d->enabled = true;
	schedule_delayed_work(&d->work, msecs_to_jiffies(d->delay_ms));

	ret = m1120_hall_register(d);
	if (ret)
		dev_err(d->dev, "%s: hall framework registration failed (%d)\n",
			__func__, ret);

	dev_info(d->dev, "%s: %s-%s was found\n", __func__,
		 d->pos == M1120_POS_UP ? "up" : (d->pos == M1120_POS_DOWN ? "down" : "middle"),
		 M1120_DRV_NAME);
	return 0;

err_regs:
	if (d->vddio)
		regulator_disable(d->vddio);
	if (d->vdd)
		regulator_disable(d->vdd);
	return ret;
}

static int m1120_i2c_drv_remove(struct i2c_client *client)
{
	struct m1120_data *d = i2c_get_clientdata(client);

	if (!d)
		return 0;
	bbk_hall_core_unregister_device(&d->hall);
	d->enabled = false;
	cancel_delayed_work_sync(&d->work);
	if (d->vddio)
		regulator_disable(d->vddio);
	if (d->vdd)
		regulator_disable(d->vdd);
	return 0;
}

static int m1120_suspend(struct device *dev)
{
	struct m1120_data *d = dev_get_drvdata(dev);

	if (!d)
		return 0;
	cancel_delayed_work_sync(&d->work);
	m1120_set_operation_mode(d, false);
	return 0;
}

static int m1120_resume(struct device *dev)
{
	struct m1120_data *d = dev_get_drvdata(dev);

	if (!d)
		return 0;
	if (d->enabled) {
		m1120_set_operation_mode(d, true);
		schedule_delayed_work(&d->work, msecs_to_jiffies(d->delay_ms));
	}
	return 0;
}

static const struct dev_pm_ops m1120_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(m1120_suspend, m1120_resume)
};

static struct i2c_driver m1120_i2c_driver = {
	.driver = {
		.name		= M1120_DRV_NAME,
		.owner		= THIS_MODULE,
		.pm		= &m1120_pm_ops,
		.of_match_table	= of_match_ptr(m1120_of_match),
	},
	.probe		= m1120_i2c_drv_probe,
	.remove		= m1120_i2c_drv_remove,
	.id_table	= m1120_id,
};
module_i2c_driver(m1120_i2c_driver);

MODULE_AUTHOR("EEBBK S6 kernel reconstruction");
MODULE_DESCRIPTION("Magnachip MXM1120 hall sensor (reconstructed from the factory binary)");
MODULE_LICENSE("GPL v2");
