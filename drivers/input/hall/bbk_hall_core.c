/*
 * BBK hall sensor framework for the EEBBK S6 (P20H130).
 *
 * The elevator camera of this device is positioned with hall sensors (Magnachip
 * MXM1120 for the two end points, iSentek IST8801 as the 3D switch) and this
 * framework is the glue between them and the motor controller: it polls the
 * sensors, keeps their recent samples in small ring buffers, recognises when the
 * mechanism is pushed by hand or stalls, and carries the calibration data the
 * Android daemon feeds in through a misc device.
 *
 * The vendor source of this driver is not part of the tree, so everything below
 * was recovered from the factory boot image: the misc device name and its four
 * ioctls, the calibration blob layouts, the six sysfs attributes, the exported
 * API and the sensor registration protocol.  Where the vendor's code addresses
 * its private data by offset, that offset is recorded in a comment.
 *
 * Device tree: none.  The vendor driver has no of_match_table; it is a plain
 * platform device called "bbk_hall_core", and this driver registers that device
 * itself once both the up and the down sensor have registered:
 *
 *   bbk_hall_core: register up down hall Done,so add platform devices
 *
 * Userspace interface that must stay identical:
 *   /dev/bbk_hall_core          misc device, ioctls 0x40046000 .. 0x40046003
 *   /sys/devices/platform/bbk_hall_core/
 *     bbk_hall_delay      0644  poll period in ms, clamped to at least 20
 *     bbk_hall_enable     0644  show the enabled flag, store takes only 0/1
 *     bbk_hall_data       0444  "up:%d down:%d\n", one fresh sample per sensor
 *     bbk_hall_vendor     0444  "up:%s down:%s\n", the registered sensor names
 *     bbk_hall_cali_time  0644  internal unit is 0.6 of the user unit
 *     bbk_mhall_version   0444  "version:%d\n", the mhall blob validity flag
 *
 * SPDX-License-Identifier: GPL-2.0
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/errno.h>

#define BBK_HALL_MISC_NAME	"bbk_hall_core"
#define BBK_HALL_PLAT_NAME	"bbk_hall_core"

#define CALI_PATH_HALL		"/mnt/vendor/persist/sensors/cali_hall"
#define CALI_PATH_MHALL		"/mnt/vendor/persist/sensors/cali_mhall_final"
#define CALI_PATH_COUNT		"/mnt/vendor/persist/sensors/up_down_count"

/* [RE] the four ioctls, all built from the same base */
#define BBK_HALL_CORE_IOCTL_SET_CALI		0x40046000
#define BBK_HALL_CORE_IOCTL_TRANS_CALI		0x40046001
#define BBK_HALL_CORE_IOCTL_SET_MHALL_CALI	0x40046002
#define BBK_HALL_CORE_IOCTL_SET_MHALL_CALI_DAEMON 0x40046003

/* the sizes the vendor copies, in bytes */
#define HALL_CALI_SIZE		56
#define MHALL_CALI_SIZE		48

/* the queue sentinel and the poll period bounds */
#define HALL_QUEUE_SENTINEL	10000
#define HALL_DELAY_DEFAULT	50
#define HALL_DELAY_MIN		20
#define HALL_QUEUE_MAX		32

/* [RE] the vendor's sample queues are 10 and 5 entries deep */
#define HALL_QUEUE_DEPTH	10
#define MOVE_QUEUE_DEPTH	5

/* ------------------------------------------------------------- layouts --- */

/* [RE] one calibration point, 8 bytes */
struct hall_cali_pos {
	s16 hall_up;		/* +0x00 */
	s16 hall_down;		/* +0x02 */
	int hall_up_down_diff;	/* +0x04 */
};

/* [RE] /mnt/vendor/persist/sensors/cali_hall: 56 bytes, no magic */
struct hall_cali_data {
	int			unknown0;	/* +0x00 */
	struct hall_cali_pos	position0;	/* +0x04 */
	struct hall_cali_pos	position24;	/* +0x0c */
	struct hall_cali_pos	position48;	/* +0x14 */
	struct hall_cali_pos	position696;	/* +0x1c */
	struct hall_cali_pos	position1;	/* +0x24 */
	struct hall_cali_pos	position6;	/* +0x2c */
	int			cali_time;	/* +0x34 */
};

/* [RE] /mnt/vendor/persist/sensors/cali_mhall_final: 48 bytes, no magic */
struct mhall_cali_data {
	s16	hall_up;		/* +0x00 */
	s16	hall_down;		/* +0x02 */
	int	valid;			/* +0x04, must be 1 to be accepted */
	s16	hall_up_1;		/* +0x08 */
	u8	pad0[6];		/* +0x0a */
	s16	hall_up_2;		/* +0x10 */
	u8	pad1[6];		/* +0x12 */
	s16	hall_up_3;		/* +0x18 */
	u8	pad2[6];		/* +0x1a */
	s16	hall_up_4;		/* +0x20 */
	u8	pad3[6];		/* +0x22 */
	int	f2;			/* +0x28 */
	int	add_time;		/* +0x2c */
};

/* [RE] what a sensor driver hands to bbk_hall_core_register_device() */
struct hall_ops {
	int (*get_data)(void *data, s16 *value);	/* +0x00 */
	int (*set_enable)(void *data, int on);		/* +0x08 */
};

struct hall_dev {
	char		name[24];	/* +0x00, must start with "up" or "down" */
	struct hall_ops	*ops;		/* +0x18 */
	void		*data;		/* +0x20 */
};

/* [RE] a sample ring buffer, 32 bytes, filled with 10000 at init */
struct hall_queue {
	int	*up;		/* +0x00 */
	int	*down;		/* +0x08 */
	int	size;		/* +0x10 */
	int	index;		/* +0x14 */
	int	count;		/* +0x18 */
	int	pad;		/* +0x1c */
};

/*
 * [RE] the vendor's private data is 168 bytes; the offsets it is addressed by
 * are +0x00 hall_up, +0x08 hall_down, +0x18 register flag, +0x1c enabled,
 * +0x20 delay, +0x24 hall cali valid, +0x28 mhall cali valid, +0x30 work,
 * +0x90 hall queue, +0x98 move queue, +0xa0 inited, +0xa4 busy.  Nothing
 * outside this file dereferences it, so the fields are named here.
 */
struct mhall_data {
	struct hall_dev		*hall_up;
	struct hall_dev		*hall_down;
	atomic_t		register_flag;
	atomic_t		enabled;
	int			delay;
	int			cali_valid;
	int			mhall_cali_valid;
	struct delayed_work	work;
	struct hall_queue	*hall_queue;
	struct hall_queue	*move_queue;
	int			inited;
	int			busy;
};

/* the press threshold, a module parameter of the queue unit in the vendor */
static int mhall_press = 8;
module_param(mhall_press, int, 0644);

/* [RE] the exported globals the motor driver reads and writes */
struct mhall_data *mhall_data;
EXPORT_SYMBOL_GPL(mhall_data);

struct hall_cali_data g_hall_cali_data;
EXPORT_SYMBOL_GPL(g_hall_cali_data);

struct mhall_cali_data g_mhall_cali_data;
EXPORT_SYMBOL_GPL(g_mhall_cali_data);

/* 12 bytes in the vendor image, cleared at init, otherwise unused */
u8 camera_mhall[12];
EXPORT_SYMBOL_GPL(camera_mhall);

/*
 * The motor driver supplies these.  They are weak here so that this framework
 * links and works on its own; the strong definitions in the elevator driver
 * take over as soon as CONFIG_VIB_PWM is set.  They are deliberately not
 * exported here: the elevator driver already exports the real ones, and
 * exporting the same name from two objects would collide on the __crc_ symbol.
 */
void __weak set_vib_all_time(int t)
{
}

void __weak set_vib_up_down_count(int up, int all)
{
}

void __weak cancel_vib_hrtimer(int arg)
{
}

bool __weak vib_is_in_cali(void)
{
	return false;
}

void __weak vib_update_key(int key)
{
}

int __weak init_hall_data_fake(const void *p)
{
	return -1;
}

int __weak get_camera_state(void)
{
	return 0;
}

/* --------------------------------------------------------------- queues -- */

int init_hall_queue_array(struct hall_queue *q)
{
	int i;

	if (!q || !q->up || !q->down)
		return -1;
	for (i = 0; i < q->size; i++) {
		q->up[i] = HALL_QUEUE_SENTINEL;
		q->down[i] = HALL_QUEUE_SENTINEL;
	}
	q->index = 0;
	q->count = 0;
	return 1;
}
EXPORT_SYMBOL_GPL(init_hall_queue_array);

struct hall_queue *init_hall_queue(int n)
{
	struct hall_queue *q;

	if (n <= 0 || n > HALL_QUEUE_MAX)
		return NULL;
	q = kzalloc(sizeof(*q), GFP_KERNEL);
	if (!q)
		return NULL;
	q->up = kzalloc(sizeof(int) * n, GFP_KERNEL);
	q->down = kzalloc(sizeof(int) * n, GFP_KERNEL);
	if (!q->up || !q->down) {
		kfree(q->up);
		kfree(q->down);
		kfree(q);
		return NULL;
	}
	q->size = n;
	init_hall_queue_array(q);
	return q;
}
EXPORT_SYMBOL_GPL(init_hall_queue);

void in_hall_queue(struct hall_queue *q, int up, int down)
{
	if (!q || !q->up || !q->down || q->size <= 0)
		return;
	q->up[q->index] = up;
	q->down[q->index] = down;
	q->index = (q->index + 1 == q->size) ? 0 : q->index + 1;
	if (q->count < q->size)
		q->count++;
}
EXPORT_SYMBOL_GPL(in_hall_queue);

int out_hall_queue_up(struct hall_queue *q, int off)
{
	if (!q || !q->up || q->size <= 0)
		return HALL_QUEUE_SENTINEL;
	return q->up[((q->index + off) % q->size + q->size) % q->size];
}
EXPORT_SYMBOL_GPL(out_hall_queue_up);

int out_hall_queue_down(struct hall_queue *q, int off)
{
	if (!q || !q->down || q->size <= 0)
		return HALL_QUEUE_SENTINEL;
	return q->down[((q->index + off) % q->size + q->size) % q->size];
}
EXPORT_SYMBOL_GPL(out_hall_queue_down);

/* [RE] a queue counts as ready once no sentinel is left in it */
int hall_queue_is_inited(struct hall_queue *q)
{
	int i;

	if (!q || !q->up || !q->down)
		return 0;
	for (i = 0; i < q->size; i++) {
		if (q->up[i] == HALL_QUEUE_SENTINEL ||
		    q->down[i] == HALL_QUEUE_SENTINEL)
			return 0;
	}
	return 1;
}
EXPORT_SYMBOL_GPL(hall_queue_is_inited);

/* copy the queue in chronological order, newest last */
static int queue_snapshot(struct hall_queue *q, int *up, int *down)
{
	int i, idx;

	if (!q || !q->up || !q->down || q->size > HALL_QUEUE_MAX)
		return 0;
	if (!hall_queue_is_inited(q))
		return 0;
	idx = q->index;			/* the write position holds the oldest */
	for (i = 0; i < q->size; i++) {
		up[i] = q->up[idx];
		down[i] = q->down[idx];
		idx = (idx + 1) % q->size;
	}
	return 1;
}

/*
 * [RE] queue_data_is_press: the vendor compares the newest, the previous and
 * the oldest sample of both channels with the mhall_press threshold (8).
 */
int queue_data_is_press(struct hall_queue *q)
{
	int size, idx, newest, prev, oldest;
	int dnewest, dprev, doldest;
	int rise, fall;

	if (!q || !hall_queue_is_inited(q) || q->size < 3)
		return 0;
	size = q->size;
	idx = q->index;
	newest = q->up[(idx + size - 1) % size];
	prev = q->up[(idx + size - 2) % size];
	oldest = q->up[idx % size];
	dnewest = q->down[(idx + size - 1) % size];
	dprev = q->down[(idx + size - 2) % size];
	doldest = q->down[idx % size];

	rise = ((prev - oldest) > mhall_press) && (prev > oldest) &&
	       ((newest - oldest) > mhall_press);
	fall = ((doldest - dprev) > mhall_press) && (doldest > dprev) &&
	       ((doldest - dnewest) > mhall_press);

	return rise && fall;
}
EXPORT_SYMBOL_GPL(queue_data_is_press);

/*
 * [RE] queue_data_is_move_press: every neighbouring pair and the whole span of
 * both channels stays within one count, i.e. the mechanism has stopped.
 */
int queue_data_is_move_press(struct hall_queue *q)
{
	int up[HALL_QUEUE_MAX], down[HALL_QUEUE_MAX];
	int i, n;

	if (!q || q->size > HALL_QUEUE_MAX || !queue_snapshot(q, up, down))
		return 0;
	n = q->size;
	for (i = 1; i < n; i++) {
		if ((up[i] - up[i - 1]) * (up[i] - up[i - 1]) > 1)
			return 0;
		if ((down[i] - down[i - 1]) * (down[i] - down[i - 1]) > 1)
			return 0;
	}
	if ((up[n - 1] - up[0]) * (up[n - 1] - up[0]) > 1)
		return 0;
	if ((down[n - 1] - down[0]) * (down[n - 1] - down[0]) > 1)
		return 0;
	return 1;
}
EXPORT_SYMBOL_GPL(queue_data_is_move_press);

/*
 * [RE] queue_data_is_move_strong_press: while the elevator is moving (state 4)
 * a large enough up/down divergence means the mechanism is being pushed by
 * hand.  polarity selects the direction of the comparison and comes from the
 * hall calibration.
 */
int queue_data_is_move_strong_press(struct hall_queue *q, int state, int polarity)
{
	int up[HALL_QUEUE_MAX], down[HALL_QUEUE_MAX];
	int j, last;

	if (!q || q->size > HALL_QUEUE_MAX || state != 4)
		return 0;
	if (!queue_snapshot(q, up, down))
		return 0;

	last = q->size - 1;
	for (j = 0; j < q->size; j++) {
		int d = down[last] - up[last] + up[j] - down[j];

		if (polarity == 1) {
			if (d > 30)
				return 1;
		} else {
			if (d >= 31)
				return 1;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(queue_data_is_move_strong_press);

void queue_data_print(struct hall_queue *q)
{
	int up[HALL_QUEUE_MAX], down[HALL_QUEUE_MAX];
	int i;

	if (!q || !queue_snapshot(q, up, down)) {
		pr_info("[LYQ-damon-hall]:%s queue not ready\n", __func__);
		return;
	}
	pr_info("[LYQ-damon-hall]:%s up", __func__);
	for (i = 0; i < q->size; i++)
		pr_cont(" %d", up[i]);
	pr_cont(" down");
	for (i = 0; i < q->size; i++)
		pr_cont(" %d", down[i]);
	pr_cont("\n");
}
EXPORT_SYMBOL_GPL(queue_data_print);

static void hall_core_reset_queues(void)
{
	if (!mhall_data)
		return;
	if (mhall_data->hall_queue)
		init_hall_queue_array(mhall_data->hall_queue);
	if (mhall_data->move_queue)
		init_hall_queue_array(mhall_data->move_queue);
}

void bbk_hall_core_init_queue(void)
{
	if (mhall_data && mhall_data->hall_queue)
		init_hall_queue_array(mhall_data->hall_queue);
}
EXPORT_SYMBOL_GPL(bbk_hall_core_init_queue);

void bbk_hall_core_init_move_queue(void)
{
	if (mhall_data && mhall_data->move_queue)
		init_hall_queue_array(mhall_data->move_queue);
}
EXPORT_SYMBOL_GPL(bbk_hall_core_init_move_queue);

int hall_clear_cali_data(void)
{
	memset(&g_hall_cali_data, 0, sizeof(g_hall_cali_data));
	if (mhall_data)
		mhall_data->cali_valid = 0;
	pr_info("[LYQ-damon-hall]:%s hall calibration cleared\n", __func__);
	return 0;
}
EXPORT_SYMBOL_GPL(hall_clear_cali_data);

/* ----------------------------------------------------------- public API -- */

/* [RE] this reports the inited flag, not the enabled one */
int bbk_hall_get_status(void)
{
	return mhall_data ? mhall_data->inited : 0;
}
EXPORT_SYMBOL_GPL(bbk_hall_get_status);

static int hall_read_one(struct hall_dev *dev, s16 *value)
{
	if (!dev || !dev->ops || !dev->ops->get_data)
		return -1;
	return dev->ops->get_data(dev->data, value);
}

/*
 * [RE] bbk_hall_core_read_data: the framework must be inited and enabled, each
 * channel gets up to five attempts a millisecond apart, and a failure is
 * reported as -2000 in both outputs.
 */
int bbk_hall_core_read_data(s16 *hall_up, s16 *hall_down)
{
	struct mhall_data *d = mhall_data;
	int retry, ok_up, ok_down;

	if (!d || d->inited != 1)
		return -1;

	if (atomic_read(&d->enabled) == 0) {
		pr_info("[LYQ-damon-hall]:damon hall is not enable\n");
		if (hall_up)
			*hall_up = -2000;
		if (hall_down)
			*hall_down = -2000;
		return -1;
	}

	d->busy = 1;
	if (hall_up)
		*hall_up = -2000;
	if (hall_down)
		*hall_down = -2000;

	ok_up = 0;
	ok_down = 0;
	for (retry = 0; retry < 5; retry++) {
		if (!ok_up && hall_read_one(d->hall_up, hall_up) == 0)
			ok_up = 1;
		if (!ok_down && hall_read_one(d->hall_down, hall_down) == 0)
			ok_down = 1;
		if (ok_up && ok_down)
			break;
		msleep(1);
	}
	d->busy = 0;

	if (!ok_up || !ok_down) {
		if (hall_up)
			*hall_up = -2000;
		if (hall_down)
			*hall_down = -2000;
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bbk_hall_core_read_data);

int bbk_hall_core_set_delay(int ms)
{
	if (!mhall_data)
		return -1;
	mhall_data->delay = ms < HALL_DELAY_MIN ? HALL_DELAY_MIN : ms;
	pr_info("[LYQ-damon-hall]:%s delay %d\n", __func__, mhall_data->delay);
	return 0;
}
EXPORT_SYMBOL_GPL(bbk_hall_core_set_delay);

/* [RE] the polling worker: sample, store, recognise presses, reschedule */
static void bbk_hal_work_func(struct work_struct *work)
{
	struct mhall_data *d = container_of(to_delayed_work(work),
					    struct mhall_data, work);
	s16 up = -2000, down = -2000;

	if (!d->hall_up || !d->hall_down)
		goto resched;
	if (vib_is_in_cali() || d->cali_valid != 1)
		goto resched;

	if (bbk_hall_core_read_data(&up, &down) == 0) {
		int state = get_camera_state();
		int polarity;

		/* the moving states feed the move queue, everything else the hall queue */
		if (state == 4 || state == 5)
			in_hall_queue(d->move_queue, up, down);
		else
			in_hall_queue(d->hall_queue, up, down);

		polarity = (g_hall_cali_data.position0.hall_down <
			    g_hall_cali_data.position1.hall_down);

		if (queue_data_is_press(d->hall_queue)) {
			/* [RE] a press by hand: the motor driver is asked to stop */
			queue_data_print(d->hall_queue);
			hall_core_reset_queues();
			vib_update_key(635);
		} else if (queue_data_is_move_strong_press(d->move_queue, state,
							   polarity)) {
			if (d->move_queue->count >= 14) {
				pr_info("[LYQ-damon-hall]:damon mechanism pushed by hand, abort\n");
				hall_core_reset_queues();
				cancel_vib_hrtimer(0);
			} else {
				cancel_vib_hrtimer(1);
			}
		}
	}

resched:
	/*
	 * Only re-arm while the framework is still enabled.  The disable path
	 * clears the flag before it cancels, so a self requeue here would make
	 * cancel_delayed_work_sync() wait for a work that keeps re-scheduling
	 * itself - a livelock that hangs whoever called enable(0), which is the
	 * elevator driver on every move.
	 */
	if (atomic_read(&d->enabled))
		queue_delayed_work_on(8 /* WORK_CPU_UNBOUND */, system_wq,
				      &d->work, msecs_to_jiffies(d->delay));
}

/* [RE] enable is idempotent through an atomic compare and swap */
int bbk_hall_core_enable(int on)
{
	struct mhall_data *d = mhall_data;
	int want = on ? 1 : 0;

	if (!d || d->inited != 1)
		return 0;

	if (want) {
		if (atomic_cmpxchg(&d->enabled, 0, 1) != 0)
			return 0;
		if (d->hall_up && d->hall_up->ops && d->hall_up->ops->set_enable)
			d->hall_up->ops->set_enable(d->hall_up->data, 1);
		if (d->hall_down && d->hall_down->ops && d->hall_down->ops->set_enable)
			d->hall_down->ops->set_enable(d->hall_down->data, 1);
		hall_core_reset_queues();
		queue_delayed_work_on(8, system_wq, &d->work, 10);
		pr_info("[LYQ-damon-hall]:%s hall enabled\n", __func__);
	} else {
		if (atomic_cmpxchg(&d->enabled, 1, 0) != 1)
			return 0;
		cancel_delayed_work_sync(&d->work);
		if (d->hall_up && d->hall_up->ops && d->hall_up->ops->set_enable)
			d->hall_up->ops->set_enable(d->hall_up->data, 0);
		if (d->hall_down && d->hall_down->ops && d->hall_down->ops->set_enable)
			d->hall_down->ops->set_enable(d->hall_down->data, 0);
		hall_core_reset_queues();
		pr_info("[LYQ-damon-hall]:%s hall disabled\n", __func__);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bbk_hall_core_enable);

/* ------------------------------------------------------------- sysfs ----- */

static ssize_t bbk_hall_delay_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	return sprintf(buf, "delay=%d\n", mhall_data ? mhall_data->delay : 0);
}

static ssize_t bbk_hall_delay_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ms;

	if (kstrtoint(buf, 10, &ms))
		return count;
	if (mhall_data)
		mhall_data->delay = ms < HALL_DELAY_MIN ? HALL_DELAY_MIN : ms;
	return count;
}

static ssize_t bbk_hall_enable_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	return sprintf(buf, "enable=%d\n",
		       mhall_data ? atomic_read(&mhall_data->enabled) : 0);
}

/* [RE] the vendor forwards zero and one only */
static ssize_t bbk_hall_enable_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int v;

	if (kstrtoint(buf, 10, &v))
		return count;
	if (v == 0 || v == 1)
		bbk_hall_core_enable(v);
	return count;
}

/* [RE] one fresh sample per sensor, up to ten attempts each, 2 ms apart */
static ssize_t bbk_hall_data_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct mhall_data *d = mhall_data;
	s16 up = -2000, down = -2000;
	int i;

	if (d) {
		for (i = 0; i < 10; i++) {
			s16 u = -2000, dn = -2000;

			if (hall_read_one(d->hall_up, &u) == 0 &&
			    hall_read_one(d->hall_down, &dn) == 0) {
				up = u;
				down = dn;
				break;
			}
			msleep(2);
		}
	}
	return sprintf(buf, "up:%d down:%d\n", up, down);
}

static ssize_t bbk_hall_vendor_show(struct device *dev, struct device_attribute *attr,
				    char *buf)
{
	struct mhall_data *d = mhall_data;
	const char *up = (d && d->hall_up) ? d->hall_up->name : "none";
	const char *down = (d && d->hall_down) ? d->hall_down->name : "none";

	return sprintf(buf, "up:%s down:%s\n", up, down);
}

static ssize_t bbk_hall_cali_time_show(struct device *dev, struct device_attribute *attr,
				       char *buf)
{
	if (!mhall_data || mhall_data->cali_valid == 0)
		return sprintf(buf, "cali_time=0\n");
	return sprintf(buf, "cali_time=%d\n", g_hall_cali_data.cali_time * 6 / 10);
}

/* [RE] the internal unit is 0.6 of the user unit, and a write needs a blob */
static ssize_t bbk_hall_cali_time_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long v;
	int t;

	if (kstrtoul(buf, 10, &v))
		return count;
	t = (int)((10 * v + 5) / 6);

	if (mhall_data && mhall_data->cali_valid != 0 && v >= 1) {
		g_hall_cali_data.cali_time = t;
		set_vib_all_time(t);
		pr_info("[LYQ-damon-hall]:damon bbk_hall_cali_time_store Set cali time:%d Success\n",
			t);
	} else {
		pr_info("[LYQ-damon-hall]:damon bbk_hall_cali_time_store Set cali time:%d failed\n",
			t);
	}
	return count;
}

static ssize_t bbk_mhall_version_show(struct device *dev, struct device_attribute *attr,
				      char *buf)
{
	return sprintf(buf, "version:%d\n", g_mhall_cali_data.valid);
}

static DEVICE_ATTR_RW(bbk_hall_delay);
static DEVICE_ATTR_RW(bbk_hall_enable);
static DEVICE_ATTR_RO(bbk_hall_data);
static DEVICE_ATTR_RO(bbk_hall_vendor);
static DEVICE_ATTR_RW(bbk_hall_cali_time);
static DEVICE_ATTR_RO(bbk_mhall_version);

/* ---------------------------------------------------------- misc device -- */

static int bbk_hall_core_misc_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int bbk_hall_core_misc_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t bbk_hall_core_misc_dev_read(struct file *file, char __user *buf,
					   size_t count, loff_t *pos)
{
	return 0;
}

static ssize_t bbk_hall_core_misc_dev_write(struct file *file, const char __user *buf,
					    size_t count, loff_t *pos)
{
	return 0;
}

static unsigned int bbk_hall_core_misc_dev_poll(struct file *file, poll_table *wait)
{
	return 0;
}

/* [RE] store a calibration blob, optionally persisting it */
static int hall_core_save_blob(const char *path, const void *blob, size_t size)
{
	struct file *f;
	ssize_t wrote;

	f = filp_open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (IS_ERR(f)) {
		pr_err("[LYQ-damon-hall]:open %s failed %ld\n", path, PTR_ERR(f));
		return -1;
	}
	wrote = __kernel_write(f, blob, size, &f->f_pos);
	filp_close(f, NULL);
	if (wrote != size) {
		pr_err("[LYQ-damon-hall]:write %s failed %ld\n", path, (long)wrote);
		return -1;
	}
	return 0;
}

/* [RE] the transform ioctl also restores the up/down counter */
static void hall_core_restore_count(void)
{
	struct file *f;
	char buf[50];
	ssize_t n;
	unsigned int up = 0, all = 0;

	memset(buf, 0, sizeof(buf));
	f = filp_open(CALI_PATH_COUNT, O_RDONLY, 0);
	if (IS_ERR(f)) {
		pr_err("[LYQ-damon-hall]:read %s failed,%ld\n",
		       CALI_PATH_COUNT, PTR_ERR(f));
		return;
	}
	n = kernel_read(f, buf, sizeof(buf) - 1, &f->f_pos);
	filp_close(f, NULL);
	if (n <= 0) {
		pr_err("[LYQ-damon-hall]:read %s failed,%ld\n",
		       CALI_PATH_COUNT, (long)n);
		return;
	}
	if (sscanf(buf, "%u-%u", &up, &all) == 2) {
		set_vib_up_down_count(up, all);
		pr_info("[LYQ-damon-hall]:%s up_down_count %u-%u\n",
			__func__, up, all);
	}
}

static long bbk_hall_core_misc_dev_ioctl(struct file *file, unsigned int cmd,
					 unsigned long arg)
{
	void __user *uarg = (void __user *)arg;

	pr_info("[LYQ-damon-hall]:---------ioctl cmd 0x%x -------\n", cmd);

	switch (cmd) {
	case BBK_HALL_CORE_IOCTL_SET_CALI:
		memset(&g_hall_cali_data, 0, sizeof(g_hall_cali_data));
		if (copy_from_user(&g_hall_cali_data, uarg, HALL_CALI_SIZE))
			return -EFAULT;
		if (hall_core_save_blob(CALI_PATH_HALL, &g_hall_cali_data,
					HALL_CALI_SIZE))
			return -1;
		if (mhall_data)
			mhall_data->cali_valid = 1;
		set_vib_all_time(g_hall_cali_data.cali_time);
		pr_info("[LYQ-damon-hall]:---------ioctl: BBK_HALL_CORE_IOCTL_SET_CALI after Hall Cali-------\n");
		return 0;

	case BBK_HALL_CORE_IOCTL_TRANS_CALI:
		memset(&g_hall_cali_data, 0, sizeof(g_hall_cali_data));
		if (copy_from_user(&g_hall_cali_data, uarg, HALL_CALI_SIZE))
			return -EFAULT;
		if (mhall_data)
			mhall_data->cali_valid = 1;
		set_vib_all_time(g_hall_cali_data.cali_time);
		hall_core_restore_count();
		pr_info("[LYQ-damon-hall]:---------ioctl: BBK_HALL_CORE_IOCTL_TRANS_CALI -------\n");
		return 0;

	case BBK_HALL_CORE_IOCTL_SET_MHALL_CALI:
		memset(&g_mhall_cali_data, 0, sizeof(g_mhall_cali_data));
		if (copy_from_user(&g_mhall_cali_data, uarg, MHALL_CALI_SIZE))
			return -EFAULT;
		g_mhall_cali_data.valid = 1;
		if (hall_core_save_blob(CALI_PATH_MHALL, &g_mhall_cali_data,
					MHALL_CALI_SIZE)) {
			g_mhall_cali_data.valid = 0;
			return -1;
		}
		if (init_hall_data_fake(&g_mhall_cali_data) != 1) {
			pr_err("[LYQ-damon-hall]:zyhc set mhall cali data failed\n");
			return -1;
		}
		if (mhall_data)
			mhall_data->mhall_cali_valid = 1;
		pr_info("[LYQ-damon-hall]:---------ioctl: BBK_HALL_CORE_IOCTL_SET_MHALL_CALI after Hall Cali-------\n");
		return 0;

	case BBK_HALL_CORE_IOCTL_SET_MHALL_CALI_DAEMON:
		memset(&g_mhall_cali_data, 0, sizeof(g_mhall_cali_data));
		if (copy_from_user(&g_mhall_cali_data, uarg, MHALL_CALI_SIZE))
			return -EFAULT;
		g_mhall_cali_data.valid = 1;
		init_hall_data_fake(&g_mhall_cali_data);
		if (mhall_data)
			mhall_data->mhall_cali_valid = 1;
		pr_info("[LYQ-damon-hall]:---------ioctl: BBK_HALL_CORE_IOCTL_SET_MHALL_CALI_DAEMON -------\n");
		return 0;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations bbk_hall_core_misc_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= bbk_hall_core_misc_dev_open,
	.release	= bbk_hall_core_misc_dev_release,
	.read		= bbk_hall_core_misc_dev_read,
	.write		= bbk_hall_core_misc_dev_write,
	.poll		= bbk_hall_core_misc_dev_poll,
	.unlocked_ioctl	= bbk_hall_core_misc_dev_ioctl,
};

static struct miscdevice bbk_hall_core_misc_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= BBK_HALL_MISC_NAME,
	.fops	= &bbk_hall_core_misc_dev_fops,
};

/* ------------------------------------------------------- platform glue --- */

static struct platform_device bbk_hall_core_device = {
	.name	= BBK_HALL_PLAT_NAME,
	.id	= -1,
};

static int bbk_hall_core_probe(struct platform_device *pdev)
{
	int ret;

	/* [RE] the vendor probe only creates the attributes and the misc device */
	ret = device_create_file(&pdev->dev, &dev_attr_bbk_hall_delay);
	ret |= device_create_file(&pdev->dev, &dev_attr_bbk_hall_enable);
	ret |= device_create_file(&pdev->dev, &dev_attr_bbk_hall_data);
	ret |= device_create_file(&pdev->dev, &dev_attr_bbk_hall_vendor);
	ret |= device_create_file(&pdev->dev, &dev_attr_bbk_hall_cali_time);
	ret |= device_create_file(&pdev->dev, &dev_attr_bbk_mhall_version);
	if (ret)
		pr_info("[LYQ-damon-hall]:device_create_file failed\n");

	if (misc_register(&bbk_hall_core_misc_dev)) {
		pr_err("[LYQ-damon-hall]:%s misc_register failed\n", __func__);
		return -1;
	}
	mhall_data->inited = 1;
	pr_info("[LYQ-damon-hall]:%s success\n", __func__);
	return 0;
}

static struct platform_driver bbk_hall_core_driver = {
	.probe	= bbk_hall_core_probe,
	.driver	= {
		.name = BBK_HALL_PLAT_NAME,
	},
};

/*
 * [RE] bbk_hall_core_register_device: a sensor driver hands in a hall_dev whose
 * name starts with "up" or "down".  Once both are present the framework
 * registers its own platform device and driver, exactly once.
 */
int bbk_hall_core_register_device(struct hall_dev *dev)
{
	struct mhall_data *d = mhall_data;

	if (!dev)
		return -1;
	if (!d) {
		pr_err("[LYQ-damon-hall]:bbk_hall failed kmalloc hall_data failed\n");
		return -1;
	}

	if (d->inited == 1) {
		pr_info("[LYQ-damon-hall]:damon bbk_hall_core is inited\n");
		return 0;
	}

	if (!strncmp(dev->name, "up", 2)) {
		d->hall_up = dev;
		pr_info("[LYQ-damon-hall]:register up hall %s\n", dev->name);
	} else if (!strncmp(dev->name, "down", 4)) {
		d->hall_down = dev;
		pr_info("[LYQ-damon-hall]:register down hall %s\n", dev->name);
	} else {
		pr_err("[LYQ-damon-hall]:unknown hall name %s\n", dev->name);
		return -1;
	}

	if (d->hall_up && d->hall_down &&
	    atomic_cmpxchg(&d->register_flag, 1, 0) == 1) {
		pr_info("[LYQ-damon-hall]:bbk_hall_core: register up down hall Done,so add platform devices\n");
		platform_device_register(&bbk_hall_core_device);
		platform_driver_register(&bbk_hall_core_driver);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bbk_hall_core_register_device);

void bbk_hall_core_unregister_device(struct hall_dev *dev)
{
	struct mhall_data *d = mhall_data;

	if (!d || !dev)
		return;
	if (d->hall_up == dev)
		d->hall_up = NULL;
	if (d->hall_down == dev)
		d->hall_down = NULL;
}
EXPORT_SYMBOL_GPL(bbk_hall_core_unregister_device);

/* ------------------------------------------------------ legacy helpers --- */

/*
 * [RE] the older text based framework the IST8801 driver used in the factory
 * image.  Nothing in this tree calls it: our IST8801 driver registers through
 * bbk_hall_core_register_device() like the vendor's newer path.  The symbols
 * are kept so that out of tree users keep linking.
 */
int dhall_register_hall(char *name, void *ops, void *data)
{
	pr_info("[LYQ-damon-hall]:%s name %s\n", __func__, name ? name : "?");
	return 0;
}
EXPORT_SYMBOL_GPL(dhall_register_hall);

int dhall_unregister_hall(char *name)
{
	return 0;
}
EXPORT_SYMBOL_GPL(dhall_unregister_hall);

irqreturn_t dhall_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(dhall_irq_handler);

/* --------------------------------------------------------------- init ---- */

static int __init bbk_hall_core_init(void)
{
	mhall_data = kzalloc(sizeof(*mhall_data), GFP_KERNEL);
	if (!mhall_data) {
		pr_err("[LYQ-damon-hall]:bbk_hall failed kmalloc hall_data failed\n");
		return -1;
	}

	memset(camera_mhall, 0, sizeof(camera_mhall));
	memset(&g_hall_cali_data, 0, sizeof(g_hall_cali_data));
	memset(&g_mhall_cali_data, 0, sizeof(g_mhall_cali_data));

	mhall_data->delay = HALL_DELAY_DEFAULT;
	atomic_set(&mhall_data->register_flag, 1);
	atomic_set(&mhall_data->enabled, 0);
	mhall_data->cali_valid = 0;
	mhall_data->mhall_cali_valid = 0;
	mhall_data->inited = 0;
	mhall_data->busy = 0;

	mhall_data->hall_queue = init_hall_queue(HALL_QUEUE_DEPTH);
	mhall_data->move_queue = init_hall_queue(MOVE_QUEUE_DEPTH);
	if (!mhall_data->hall_queue || !mhall_data->move_queue) {
		pr_err("[LYQ-damon-hall]:%s queue allocation failed\n", __func__);
		return -1;
	}

	INIT_DELAYED_WORK(&mhall_data->work, bbk_hal_work_func);

	pr_info("[LYQ-damon-hall]:zyhc mhall BBK_HALL_CORE_IOCTL_SET_MHALL_CALI:%u\n",
		BBK_HALL_CORE_IOCTL_SET_MHALL_CALI);
	pr_info("[LYQ-damon-hall]:bbk_hall_core_init success\n");
	return 0;
}
module_init(bbk_hall_core_init);

static void __exit bbk_hall_core_exit(void)
{
	if (!mhall_data)
		return;
	cancel_delayed_work_sync(&mhall_data->work);
	misc_deregister(&bbk_hall_core_misc_dev);
	if (mhall_data->hall_queue) {
		kfree(mhall_data->hall_queue->up);
		kfree(mhall_data->hall_queue->down);
		kfree(mhall_data->hall_queue);
	}
	if (mhall_data->move_queue) {
		kfree(mhall_data->move_queue->up);
		kfree(mhall_data->move_queue->down);
		kfree(mhall_data->move_queue);
	}
	kfree(mhall_data);
	mhall_data = NULL;
}
module_exit(bbk_hall_core_exit);

MODULE_AUTHOR("EEBBK S6 kernel reconstruction");
MODULE_DESCRIPTION("BBK hall sensor framework (reconstructed from the factory binary)");
MODULE_LICENSE("GPL v2");
