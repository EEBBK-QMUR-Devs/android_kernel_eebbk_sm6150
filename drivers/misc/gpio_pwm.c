/*
 * BBK elevator camera motor controller for the EEBBK S6 (P20H130).
 *
 * The front camera of this device sits on a motorised elevator: it is pushed
 * out of the body when the camera application starts and retracted again
 * afterwards.  The controller driver is called vib_pwm in the factory kernel
 * (its source file is drivers/input/hall/vib_pwm/gpio_pwm.c and its module
 * name is gpio_pwm) and it is built in, so its sources are not part of this
 * tree.  The device tree binding, the sysfs interface, the state machine and
 * the motor timing below were recovered from the factory boot image, whose
 * kallsyms still carries every symbol name, and from the factory device tree.
 *
 * Hardware, from the node soc:bbk_vib_pwm (compatible "bbk,vib_pwm_control")
 *   boost-gpio  gpio24  driver supply boost, one while moving, zero idle
 *   enable-gpio gpio23  driver enable, active low: zero while moving
 *   sleep-gpio  gpio29  driver nSLEEP, one while moving, zero idle
 *   dir-gpio    gpio26  zero moves the elevator up, one moves it down
 *   id-gpio     gpio25  elevator module id strap, read through a pull-up
 *   clocks = <&gcc GCC_GP2_CLK>, clock-names = "gp2_clk"
 *
 * There is no pwm controller in the binding: the drive waveform is the general
 * purpose clock gcc_gp2_clk, which the "vib_pwm_active" pinctrl state routes
 * out on gpio21, so the chopper frequency is simply the clock rate handed to
 * clk_set_rate.  GCC_GP2_CLK is clock id 36 in qcom,gcc-sdmmagpie.h and
 * gcc_gp2_clk_src is a fractional (mnd) rcg, so the 19200, 32000, 41600 and
 * 4800 Hz the vendor uses are all synthesizable even though the preset table
 * only lists 19.2, 25, 50, 100 and 200 MHz.
 *
 * Camera positions.  The vendor measures the travel on a scale that runs from
 * 0 (retracted) to 15950 (fully extended), with anchors at 3828 (24%),
 * 7656 (48%) and 11101 (69.6%), and it exposes these values through
 * vib_pwm_camera_state:
 *   0  retracted                 4  moving up        (busy)
 *   1  69.6%, the camera mid row 5  moving down      (busy)
 *   2  fully extended            6  stopped by the anti-pinch logic
 *   7  holder position, between 1 and 2
 *
 * The vendor driver's private data is 680 bytes and every handler addresses it
 * by offset, so the offsets are recorded here for traceability:
 *   +0x10 boost gpio  +0x14 enable gpio  +0x18 sleep gpio  +0x1C dir gpio
 *   +0x20 id gpio     +0x28 pinctrl      +0x30/38/40 pins active/suspend/id
 *   +0x48 struct clk *gp2_clk            +0x50 struct wakeup_source
 *   +0x118 struct hrtimer                +0x158 ktime_t of the current move
 *   +0x160 int time   +0x164 int pre_time +0x168 u8 is_subsection
 *   +0x190 u64 freq   +0x198 int count    +0x19C int enable (1 up, 2 down)
 *   +0x1A0 int camera_state               +0x1A4 int target_state
 *   +0x1A8 int key_idle                   +0x1AC u8 pending_notify
 *   +0x1B0 int holder_mode                +0x1B4 int elevator_mode
 *   +0x1B8 char notify_str[32]            +0x1D8 int move_count
 *   +0x1DC int sub_phase                  +0x1E0 int is_move
 *   +0x218 u8 stop_flag                   +0x219 u8 abort_flag
 *   +0x21C int is_in_cali                 +0x2A0 int up_down_count
 *   +0x2A4 int all_count
 * The vendor allocates it with devm_kmalloc(), which leaves enable, target,
 * holder_mode, all_count and notify_str uninitialised; this rewrite uses
 * devm_kzalloc() instead.
 *
 * One move, replicated from vib_pwm_set_camera_state:
 *   wake lock if not held
 *   clk_set_rate(gp2_clk, freq)          (32000 for every normal move)
 *   bbk_hall_core_enable(0)
 *   clk_prepare(gp2_clk)
 *   dir = 0 for up, 1 for down
 *   boost = 1; msleep(3); enable = 0; sleep = 1; msleep(3)
 *   clk_enable(gp2_clk)
 *   hrtimer_start_range_ns(&timer, move_time, 0, HRTIMER_MODE_REL_PINNED)
 *   bbk_hall_core_enable(1)
 * and the stop path, used from the timer callback and from the emergency
 * thread, is boost = 0, msleep(5), enable = 1, sleep = 0, clk_disable, later
 * clk_unprepare.
 *
 * Move durations.  all_time (module parameter mhall_control7, 2839 by default)
 * is the duration of a full travel, and the intermediate positions are scaled
 * from it: the 69.6% position takes all_time * 696 / 1000.  Every duration is
 * then scaled by the ratio between the nominal 19200 Hz and the 32000 Hz the
 * move actually runs at, that is time * 6 / 10, before it is converted to
 * nanoseconds for the hrtimer.  Durations above 65536 run in two stages: a
 * 50 ms preliminary stage at 32000 Hz and then the remainder at 41600 Hz,
 * scaled by 6 / 13.
 *
 * SPDX-License-Identifier: GPL-2.0
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/pinctrl/consumer.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/pm_wakeup.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/kthread.h>

#define VIB_PWM_DRV_NAME	"bbk_vib_pwm"

/* camera_state values, exactly as the vendor driver uses them */
#define CAM_STATE_DOWN		0	/* retracted, travel 0 */
#define CAM_STATE_MID		1	/* 69.6% of the travel */
#define CAM_STATE_FULL		2	/* fully extended, travel 15950 */
#define CAM_STATE_BUSY_UP	4	/* moving up */
#define CAM_STATE_BUSY_DOWN	5	/* moving down */
#define CAM_STATE_ABNORMAL	6	/* stopped by the anti-pinch logic */
#define CAM_STATE_HOLDER	7	/* holder position, between 1 and 2 */

/* position anchors of computer_distance() */
#define VIB_PWM_TRAVEL_FULL	15950
#define VIB_PWM_TRAVEL_MID	11101

/* clock rates the vendor programs */
#define VIB_PWM_FREQ_DEFAULT	19200
#define VIB_PWM_FREQ_MOVE	32000
#define VIB_PWM_FREQ_SLOW	4800
#define VIB_PWM_FREQ_STAGE2	41600

#define VIB_PWM_TICK_MS		50
#define VIB_PWM_TICK_LIMIT	65536
#define VIB_PWM_TIME_MAGIC	1300
#define VIB_PWM_MOVE_NUM	6	/* main stage: time * 6 / 10 */
#define VIB_PWM_MOVE_DEN	10
#define VIB_PWM_STAGE2_NUM	6	/* second stage: time * 6 / 13 */
#define VIB_PWM_STAGE2_DEN	13
#define VIB_PWM_MID_NUM		696	/* the 69.6% position */
#define VIB_PWM_MID_DEN		1000
#define VIB_PWM_DEFAULT_RETRY	3
#define VIB_PWM_CALI_FILE	"/mnt/vendor/persist/sensors/up_down_count"

/* the key codes the vendor injects through its input device */
#define VIB_KEY_PRESS		635
#define VIB_KEY_DROP		636
#define VIB_KEY_ANGLE		637
#define VIB_KEY_HOME		638
#define VIB_KEY_NOT_HOLDER	639
#define VIB_KEY_HOLDER		640
#define VIB_KEY_NOTIFY1		641
#define VIB_KEY_NOTIFY2		642
#define VIB_KEY_NOTIFY3		643
#define VIB_KEY_RESTART		644
#define VIB_KEY_STREAM_ON	645
#define VIB_KEY_STREAM_OFF	646
#define VIB_KEY_FIRST		VIB_KEY_PRESS
#define VIB_KEY_LAST		VIB_KEY_STREAM_OFF

/*
 * Module parameters.  The factory driver is gpio_pwm, so these appear as
 * /sys/module/gpio_pwm/parameters/mhall_control*; the names and the default
 * values are the ones recovered from its kernel_param array.
 */
static int mhall_control_up;		/* holder detection dead zone, up */
static int mhall_control_down;		/* holder detection dead zone, down */
static int mhall_control2;		/* holder time correction */
static int mhall_control3;		/* holder target hall reading */
static int mhall_control4 = 1000;	/* holder time coefficient, /600 */
static int mhall_control5 = 3;		/* retry_time for the holder moves */
static int mhall_control6 = 20;		/* add_time for the holder moves */
static int mhall_control7 = 2839;	/* all_time, a full travel */

module_param(mhall_control_up, int, 0664);
module_param(mhall_control_down, int, 0664);
module_param(mhall_control2, int, 0664);
module_param(mhall_control3, int, 0664);
module_param(mhall_control4, int, 0664);
module_param(mhall_control5, int, 0664);
module_param(mhall_control6, int, 0664);
module_param(mhall_control7, int, 0664);

/*
 * The hall framework is a separate driver.  Declaring these weak keeps this
 * driver linkable and working on its own, and lets bbk_hall_core take over as
 * soon as it is built in.
 */
int __weak bbk_hall_core_enable(int enable)
{
	return 0;
}

int __weak bbk_hall_core_read_data(s16 *up, s16 *down)
{
	if (up)
		*up = -2000;
	if (down)
		*down = -2000;
	return -1;
}

int __weak bbk_hall_get_status(void)
{
	return 0;
}

void __weak bbk_hall_core_init_queue(void)
{
}

void __weak bbk_hall_core_init_move_queue(void)
{
}

struct vib_pwm {
	struct device		*dev;
	struct mutex		lock;
	struct clk		*gp2_clk;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_active;
	struct pinctrl_state	*pins_suspend;
	struct pinctrl_state	*pins_idconfig;
	int			boost_gpio;
	int			enable_gpio;
	int			sleep_gpio;
	int			dir_gpio;
	int			id_gpio;
	struct wakeup_source	wakeup;
	bool			ws_active;
	wait_queue_head_t	wait;
	struct hrtimer		timer;
	/*
	 * [RE] the vendor's hrtimer handler only stops the electrical part and
	 * queues its work one jiffy later; the work does everything that may
	 * sleep.  Splitting it is not cosmetic: sleeping inside the hrtimer
	 * callback corrupts the kernel ("bad: scheduling from the idle thread!"
	 * with msleep <- vib_pwm_brake <- vib_pwm_timer_func in the trace).
	 */
	struct work_struct	finish_work;
	/* move state */
	unsigned int		time_ms;
	unsigned int		pre_time;
	bool			is_subsection;
	unsigned long		freq;
	int			enable;		/* 1 up, 2 down, else idle */
	int			camera_state;
	int			target_state;
	int			move_count;
	int			retry_time;
	bool			is_move;
	/* userspace visible values */
	int			id;
	int			count;
	int			state_init;
	int			holder_mode;
	int			elevator_mode;
	int			elevator_row_shift;
	int			up_down_count;
	int			all_count;
	int			is_in_cali;
	int			key_idle;
	bool			pending_notify;
	char			notify_str[32];
	/* key injection */
	struct input_dev	*input;
	/*
	 * The clock has to be tracked by hand: the timer callback stops the
	 * waveform and the deferred work then finishes the move, so without
	 * these flags clk_disable() runs twice and the clock core warns about
	 * an unbalanced disable (drivers/clk/clk.c:903).
	 */
	bool			clk_on;
	bool			clk_prepared;
};

static struct vib_pwm *vib_pwm_dev;

/* ------------------------------------------------------- key injection -- */

/* [RE] vib_update_key: the vendor injects a press and a release per event */
void vib_update_key(int key)
{
	struct vib_pwm *d = vib_pwm_dev;

	if (!d || !d->input)
		return;
	if (key < VIB_KEY_FIRST || key > VIB_KEY_LAST)
		return;

	input_report_key(d->input, key, 1);
	input_sync(d->input);
	input_report_key(d->input, key, 0);
	input_sync(d->input);
	dev_info(d->dev, "%s: key %d injected\n", __func__, key);
}
EXPORT_SYMBOL_GPL(vib_update_key);

/* ----------------------------------------------------------- motor core -- */

static void vib_pwm_gpio_set(int gpio, int value)
{
	if (gpio_is_valid(gpio))
		gpiod_direction_output_raw(gpio_to_desc(gpio), value);
}

static void vib_pwm_gpio_input(int gpio)
{
	if (gpio_is_valid(gpio))
		gpiod_direction_input(gpio_to_desc(gpio));
}

/*
 * [RE] the device tree carries three pinctrl states for this driver:
 * "vib_pwm_active" muxes gpio21 to the gcc_gp2 clock output, "vib_pwm_suspend"
 * turns it back into a pulled down gpio and "vib_pwm_idconfig" configures the id
 * strap on gpio25.  pinctrl_select_state() *replaces* the selected state, so
 * leaving idconfig selected after the probe also dropped the clock mux and the
 * motor received no waveform at all: the driver ran its whole move sequence
 * while the elevator never moved.  Select active for a move and suspend after
 * it, which is what these two states exist for.
 */
static void vib_pwm_select_pins(struct vib_pwm *d, bool active)
{
	struct pinctrl_state *state;

	if (IS_ERR_OR_NULL(d->pinctrl))
		return;
	state = active ? d->pins_active : d->pins_suspend;
	if (IS_ERR_OR_NULL(state))
		return;
	if (pinctrl_select_state(d->pinctrl, state))
		dev_err(d->dev, "%s: cannot select the %s pinctrl state\n",
			__func__, active ? "active" : "suspend");
	else
		dev_info(d->dev, "%s: pinctrl %s\n", __func__,
			 active ? "active" : "suspend");
}

static void vib_pwm_brake(struct vib_pwm *d)
{
	/* [RE] boost = 0, msleep(5), enable = 1, sleep = 0; dir is not touched */
	vib_pwm_gpio_set(d->boost_gpio, 0);
	msleep(5);
	vib_pwm_gpio_set(d->enable_gpio, 1);
	vib_pwm_gpio_set(d->sleep_gpio, 0);
}

/* the vendor converts a nominal-19200 ms value into hrtimer nanoseconds */
static ktime_t vib_pwm_ms_to_ktime(unsigned int ms)
{
	return ktime_set(0, (u64)ms * NSEC_PER_MSEC);
}

static ktime_t vib_pwm_move_time(struct vib_pwm *d)
{
	unsigned int ms;

	if (d->is_subsection)
		return vib_pwm_ms_to_ktime(VIB_PWM_TICK_MS);

	ms = d->time_ms * VIB_PWM_MOVE_NUM / VIB_PWM_MOVE_DEN;
	if (!ms)
		ms = 1;
	d->time_ms = ms;	/* [RE] the vendor stores the scaled value back */
	return vib_pwm_ms_to_ktime(ms);
}

/*
 * The chopper frequency lives in the RCG behind the branch clock this driver
 * was given, so setting it on the branch alone does nothing: measured on the
 * device, the pin kept outputting 19.2 MHz whatever freq was written.  The
 * vendor has the same latent bug, which is why the stock kernel never moved the
 * elevator.  Program the parent and report what the framework ends up with.
 */
static void vib_pwm_set_clock(struct vib_pwm *d)
{
	struct clk *src = clk_get_parent(d->gp2_clk);
	int ret;

	ret = clk_set_rate(d->gp2_clk, d->freq);
	if (ret)
		dev_err(d->dev, "%s: can't set pwm_clk rate ret=%d\n", __func__, ret);

	if (src && !IS_ERR(src)) {
		ret = clk_set_rate(src, d->freq);
		if (ret)
			dev_err(d->dev, "%s: can't set %s rate ret=%d\n", __func__,
				__clk_get_name(src), ret);
		dev_info(d->dev, "%s: freq %lu, %s -> %lu Hz, branch %lu Hz\n",
			 __func__, d->freq, __clk_get_name(src),
			 clk_get_rate(src), clk_get_rate(d->gp2_clk));
	}
}

/*
 * [RE] vib_pwm_set_camera_state: program the clock, bring the driver up in the
 * order the vendor used and arm the move timer.  enable is 1 for up and 2 for
 * down; any other value only brakes.
 */
static int vib_pwm_set_camera_state(struct vib_pwm *d)
{
	ktime_t when;

	dev_info(d->dev, "%s: enable %d, time %u, freq %lu\n", __func__,
		 d->enable, d->time_ms, d->freq);

	d->is_subsection = false;
	d->pre_time = 0;
	if ((d->enable == 1 || d->enable == 2) && d->time_ms > VIB_PWM_TICK_LIMIT) {
		d->is_subsection = true;
		d->pre_time = VIB_PWM_TICK_MS;
		dev_info(d->dev,
			 "%s: set is_subsection = true, pre_time:%u, current freq: %lu, all_time:%d\n",
			 __func__, d->pre_time, d->freq, mhall_control7);
	}

	if (d->enable != 1 && d->enable != 2) {
		vib_pwm_brake(d);
		return 0;
	}

	/* drop a finish that is still pending: it belongs to the previous move */
	cancel_work_sync(&d->finish_work);

	/* mux gpio21 back to the gcc_gp2 clock output before driving */
	vib_pwm_select_pins(d, true);

	if (!d->ws_active) {
		__pm_stay_awake(&d->wakeup);
		d->ws_active = true;
	}

	when = vib_pwm_move_time(d);


	bbk_hall_core_enable(0);
	clk_prepare(d->gp2_clk);
	d->clk_prepared = true;
	vib_pwm_set_clock(d);

	/* direction first: zero raises the elevator, one lowers it */
	vib_pwm_gpio_set(d->dir_gpio, d->enable == 1 ? 0 : 1);
	vib_pwm_gpio_set(d->boost_gpio, 1);
	msleep(3);
	vib_pwm_gpio_set(d->enable_gpio, 0);
	vib_pwm_gpio_set(d->sleep_gpio, 1);
	msleep(3);

	clk_enable(d->gp2_clk);
	d->clk_on = true;

	d->move_count++;
	hrtimer_start(&d->timer, when, HRTIMER_MODE_REL_PINNED);

	bbk_hall_core_enable(1);
	d->is_move = true;
	return 0;
}

/* the second stage of a long move: 41600 Hz for the remaining time */
static void vib_pwm_stage2(struct vib_pwm *d)
{
	unsigned int ms;

	d->freq = VIB_PWM_FREQ_STAGE2;
	vib_pwm_set_clock(d);
	ms = (d->time_ms - d->pre_time) * VIB_PWM_STAGE2_NUM / VIB_PWM_STAGE2_DEN;
	d->time_ms = ms;
	dev_info(d->dev,
		 "%s: clk_set_rate=%lu is_subsection==true & Reset to false\n",
		 __func__, d->freq);
	d->is_subsection = false;
	hrtimer_start(&d->timer, vib_pwm_ms_to_ktime(ms), HRTIMER_MODE_REL_PINNED);
}

/*
 * [RE] stop the waveform and the driver from interrupt context: three gpio
 * writes and clk_disable, exactly the steps the vendor's hrtimer handler does.
 * The vendor's 5 ms brake delay, clk_unprepare, the hall framework calls and the
 * second stage all belong to the deferred work, because they may sleep.
 */
static void vib_pwm_stop_waveform(struct vib_pwm *d)
{
	d->is_move = false;
	if (d->clk_on) {
		clk_disable(d->gp2_clk);
		d->clk_on = false;
	}
	vib_pwm_gpio_set(d->boost_gpio, 0);
	vib_pwm_gpio_set(d->enable_gpio, 1);
	vib_pwm_gpio_set(d->sleep_gpio, 0);
}

/* stop the electrical part of a move; must not cancel the timer */
static void vib_pwm_motor_down(struct vib_pwm *d)
{
	d->is_move = false;
	if (d->clk_on) {
		clk_disable(d->gp2_clk);
		d->clk_on = false;
	}
	if (d->clk_prepared) {
		clk_unprepare(d->gp2_clk);
		d->clk_prepared = false;
	}
	vib_pwm_brake(d);
}

/*
 * Finish a move: [RE] the tail of the vendor's handler.  This runs inside the
 * timer callback, so it must never call hrtimer_cancel() on our own timer.
 * Callers outside the callback cancel the timer themselves.
 */
static void vib_pwm_finish_move(struct vib_pwm *d, bool keep_state)
{
	vib_pwm_motor_down(d);
	vib_pwm_select_pins(d, false);
	d->move_count = 0;

	if (d->target_state <= CAM_STATE_FULL) {
		bbk_hall_core_init_queue();
		bbk_hall_core_init_move_queue();
		if (d->target_state == CAM_STATE_DOWN && !d->is_in_cali)
			bbk_hall_core_enable(0);
	}

	if (d->ws_active) {
		__pm_relax(&d->wakeup);
		d->ws_active = false;
	}

	/*
	 * [RE] the move states 4 and 5 are only markers: the real state is the
	 * target, and leaving it out makes vib_pwm_set_camera refuse every
	 * later request because (state & ~1) == 4 looks busy forever.
	 */
	if (!keep_state)
		d->camera_state = d->target_state;

	dev_info(d->dev, "%s: vib location is standby & Stop vib +++ camera_state %d\n",
		 __func__, d->camera_state);
}

/*
 * [RE] the vendor's handler: stop what can be stopped without sleeping and hand
 * the rest to the work.  Nothing here may sleep - no msleep, no clk_unprepare,
 * no clk_set_rate, no cancel of another work.
 */
static enum hrtimer_restart vib_pwm_timer_func(struct hrtimer *timer)
{
	struct vib_pwm *d = container_of(timer, struct vib_pwm, timer);

	if (d->is_subsection) {
		/* the preliminary stage is over: the second stage's clk_set_rate
		 * may sleep, so it runs in the work */
		dev_info(d->dev,
			 "++++++++++++ %s: is_subsection = true & wait_up_interrupt to start next vib_freq\n",
			 __func__);
		schedule_work(&d->finish_work);
		return HRTIMER_NORESTART;
	}

	vib_pwm_stop_waveform(d);
	schedule_work(&d->finish_work);
	return HRTIMER_NORESTART;
}

/* the deferred half: everything that may sleep */
static void vib_pwm_finish_work(struct work_struct *work)
{
	struct vib_pwm *d = container_of(work, struct vib_pwm, finish_work);

	if (d->is_subsection) {
		vib_pwm_stage2(d);
		return;
	}

	vib_pwm_finish_move(d, false);
	vib_pwm_select_pins(d, false);
	wake_up_interruptible(&d->wait);
}

/* ------------------------------------------------------ state machine ---- */

/* [RE] the move durations of the vendor's transition table */
static unsigned int vib_pwm_time_for(int want, int cur)
{
	unsigned int full = mhall_control7;
	unsigned int mid = full * VIB_PWM_MID_NUM / VIB_PWM_MID_DEN;
	unsigned int correction = (mhall_control2 * 10 / 6) * 15 / 100;

	switch (want) {
	case CAM_STATE_MID:
		return (cur == CAM_STATE_FULL) ? full - mid : mid;
	case CAM_STATE_FULL:
		return (cur == CAM_STATE_MID) ? full - mid : full;
	case CAM_STATE_HOLDER:
		if (cur == CAM_STATE_FULL)
			return mhall_control2 * 10 / 6;
		if (cur == CAM_STATE_MID)
			return (full - mid) - correction;
		return full - correction;
	case CAM_STATE_DOWN:
		/* [RE] (0,1) takes the mid duration, (0,2) and (0,7) a full one */
		if (cur == CAM_STATE_MID)
			return mid;
		return full;
	default:
		return full;
	}
}

/* [RE] the frequencies of the vendor's transition table */
static unsigned long vib_pwm_freq_for(int want, int cur)
{
	if ((want == CAM_STATE_HOLDER && cur == CAM_STATE_FULL) ||
	    (want == CAM_STATE_FULL && cur == CAM_STATE_HOLDER))
		return VIB_PWM_FREQ_SLOW;
	return VIB_PWM_FREQ_MOVE;
}

static int vib_pwm_move_to(struct vib_pwm *d, int want)
{
	int cur = d->camera_state;

	/* states 4 and 5 are busy, and a calibration blocks a move */
	if ((cur & ~1) == CAM_STATE_BUSY_UP || d->is_in_cali) {
		dev_info(d->dev,
			 "%s: want %d, current %d, is_in_cali %d: busy, ignored\n",
			 __func__, want, cur, d->is_in_cali);
		return 0;
	}

	if (cur == want)
		return 0;

	d->target_state = want;
	d->time_ms = vib_pwm_time_for(want, cur);
	d->freq = vib_pwm_freq_for(want, cur);
	d->retry_time = (want == CAM_STATE_HOLDER || cur == CAM_STATE_HOLDER) ?
			mhall_control5 : VIB_PWM_DEFAULT_RETRY;

	/* up for the mid, full and holder positions, down for a retraction */
	if (want == CAM_STATE_DOWN) {
		d->enable = 2;
		d->camera_state = CAM_STATE_BUSY_DOWN;
	} else {
		d->enable = 1;
		d->camera_state = CAM_STATE_BUSY_UP;
	}

	if (cur == CAM_STATE_DOWN) {
		d->up_down_count++;
		d->all_count++;
	}

	dev_info(d->dev,
		 "%s: want %d, current %d -> %s for %u ms, retry %d\n",
		 __func__, want, cur, d->enable == 1 ? "up" : "down",
		 d->time_ms, d->retry_time);

	return vib_pwm_set_camera_state(d);
}

/* [RE] vib_pwm_set_camera, the entry point userspace reaches */
int vib_pwm_set_camera(int want)
{
	struct vib_pwm *d = vib_pwm_dev;
	int ret;

	if (!d)
		return -ENODEV;
	if (want != CAM_STATE_DOWN && want != CAM_STATE_MID &&
	    want != CAM_STATE_FULL && want != CAM_STATE_ABNORMAL &&
	    want != CAM_STATE_HOLDER) {
		dev_info(d->dev, "%s: unrecognised camera state %d\n", __func__, want);
		return -EINVAL;
	}

	mutex_lock(&d->lock);
	/*
	 * [RE] the vendor sets the chopper frequency to 32000 before looking at
	 * the transition table, and a request for the abnormal state only stops
	 * the motor.
	 */
	d->freq = VIB_PWM_FREQ_MOVE;
	if (want == CAM_STATE_ABNORMAL) {
		d->camera_state = CAM_STATE_ABNORMAL;
		d->enable = 0;
		ret = vib_pwm_set_camera_state(d);
		mutex_unlock(&d->lock);
		return ret;
	}

	ret = vib_pwm_move_to(d, want);
	mutex_unlock(&d->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(vib_pwm_set_camera);

int get_camera_state(void)
{
	return vib_pwm_dev ? vib_pwm_dev->camera_state : -1;
}
EXPORT_SYMBOL_GPL(get_camera_state);

void cancel_vib_hrtimer(int arg)
{
	struct vib_pwm *d = vib_pwm_dev;

	if (!d)
		return;
	hrtimer_cancel(&d->timer);
	cancel_work_sync(&d->finish_work);
	bbk_hall_core_init_queue();
	bbk_hall_core_init_move_queue();
	vib_pwm_finish_move(d, true);
	if (arg == 0) {
		d->camera_state = CAM_STATE_ABNORMAL;
		dev_info(d->dev, "---------only_stop_vib & not do anything, camera_state:6 ----\n");
	}
	wake_up_interruptible(&d->wait);
}
EXPORT_SYMBOL_GPL(cancel_vib_hrtimer);

bool vib_is_move(void)
{
	return vib_pwm_dev ? vib_pwm_dev->is_move : false;
}
EXPORT_SYMBOL_GPL(vib_is_move);

bool vib_is_in_cali(void)
{
	return vib_pwm_dev ? !!vib_pwm_dev->is_in_cali : false;
}
EXPORT_SYMBOL_GPL(vib_is_in_cali);

void set_vib_all_time(int t)
{
	mhall_control7 = t;
}
EXPORT_SYMBOL_GPL(set_vib_all_time);

void set_vib_up_down_count(int up, int all)
{
	struct vib_pwm *d = vib_pwm_dev;

	if (!d)
		return;
	d->up_down_count = up;
	d->all_count = all;
}
EXPORT_SYMBOL_GPL(set_vib_up_down_count);

/* ------------------------------------------------------------- sysfs ---- */

static ssize_t vib_pwm_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	int id = 0;

	/* [RE] the vendor guards on boost_gpio but reads id_gpio */
	if (d && gpio_is_valid(d->boost_gpio)) {
		if (gpio_is_valid(d->id_gpio))
			id = gpiod_get_raw_value(gpio_to_desc(d->id_gpio));
		d->id = id;
	} else {
		dev_info(dev, "%s: id_gpio is null\n", __func__);
	}
	return sprintf(buf, "vib_id=%d\n", id);
}

static ssize_t vib_pwm_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "vib_freq=%d\n", (int)d->freq);
}

static ssize_t vib_pwm_freq_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	int v;

	if (kstrtoint(buf, 10, &v) == 0)
		d->freq = v;
	return count;
}

static ssize_t vib_pwm_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "count=%d\n", d->count);
}

static ssize_t vib_pwm_count_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	int v;

	if (kstrtoint(buf, 10, &v) == 0)
		d->count = v;
	return count;
}

static ssize_t vib_pwm_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "enable=%d\n", d->enable);
}

/*
 * [RE] one is up, two is down, anything else only brakes: the vendor does not
 * touch the move state, the hall framework or the timer here.
 */
static ssize_t vib_pwm_enable_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	int v;

	if (kstrtoint(buf, 10, &v))
		return count;

	mutex_lock(&d->lock);
	d->enable = (v == 1 || v == 2) ? v : 0;
	if (d->enable == 0) {
		hrtimer_cancel(&d->timer);
		cancel_work_sync(&d->finish_work);
	}
	vib_pwm_set_camera_state(d);
	mutex_unlock(&d->lock);
	return count;
}

static ssize_t vib_pwm_dir_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "dir=%d\n",
		       gpio_is_valid(d->dir_gpio) ?
		       gpiod_get_raw_value(gpio_to_desc(d->dir_gpio)) : 0);
}

/* [RE] the vendor looks at the first character only: '1' is one, else zero */
static ssize_t vib_pwm_dir_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	if (buf && count)
		vib_pwm_gpio_set(d->dir_gpio, buf[0] == '1');
	return count;
}

static ssize_t vib_pwm_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "time=%d all_time=%d\n", d->time_ms, mhall_control7);
}

/* [RE] writing 1300 means "use all_time plus ten" */
static ssize_t vib_pwm_time_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	int v;

	if (kstrtoint(buf, 10, &v))
		return count;

	if (v == VIB_PWM_TIME_MAGIC) {
		v = mhall_control7 + 10;
		dev_info(d->dev, "%s: time set 1300 ,change %d\n", __func__, v);
	}
	d->time_ms = v;
	return count;
}

static ssize_t vib_pwm_camera_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", d->camera_state);
}

static ssize_t vib_pwm_camera_state_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int state = 0;

	dev_info(dev, "-------%s *%s*\n", __func__, buf ? buf : "");
	if (!buf || !count)
		return count;
	if (kstrtoint(buf, 10, &state)) {
		dev_err(dev, "%s get error state\n", __func__);
		return count;
	}
	vib_pwm_set_camera(state);
	return count;
}

static ssize_t vib_pwm_state_init_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", d->camera_state);
}

/* [RE] the vendor only accepts zero here, and it re-homes the elevator */
static ssize_t vib_pwm_state_init_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	int v;

	if (kstrtoint(buf, 10, &v) || v != 0)
		return count;

	d->state_init = v;
	if (bbk_hall_get_status() == 1) {
		bbk_hall_core_enable(1);
		msleep(10);
		d->target_state = CAM_STATE_DOWN;
		d->camera_state = CAM_STATE_DOWN;
		bbk_hall_core_enable(0);
	}
	return count;
}

static ssize_t vib_pwm_abort_notify_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", d->notify_str);
}

/*
 * [RE] the vendor's notify protocol is a list of keywords, not a number: the
 * camera application writes plain strings and the driver reacts to each of
 * them, injecting the key its userspace expects.
 */
static ssize_t vib_pwm_abort_notify_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	char line[40];

	if (!buf || !count)
		return count;
	strlcpy(line, buf, sizeof(line));

	if (!strncmp(line, "username:", 9)) {
		strlcpy(d->notify_str, line + 9, sizeof(d->notify_str));
		return count;
	}
	if (!strncmp(line, "com.eebbk.askhomework", 21)) {
		if (strstr(d->notify_str, "com.eebbk.askhomework")) {
			dev_info(d->dev, "%s: maintain blocking\n", __func__);
			vib_update_key(VIB_KEY_RESTART);
			return count;
		}
		vib_pwm_set_camera(CAM_STATE_DOWN);
		vib_update_key(VIB_KEY_RESTART);
		return count;
	}
	if (!strncmp(line, "stream on", 9)) {
		vib_update_key(VIB_KEY_STREAM_ON);
		return count;
	}
	if (!strncmp(line, "stream off", 10)) {
		vib_update_key(VIB_KEY_STREAM_OFF);
		return count;
	}

	d->pending_notify = true;

	if (!strncmp(line, "0x27c", 5) || !strncmp(line, "636", 3)) {
		vib_pwm_set_camera(CAM_STATE_DOWN);
		vib_update_key(VIB_KEY_DROP);
	} else if (!strncmp(line, "0x27d", 5) || !strncmp(line, "637", 3)) {
		vib_pwm_set_camera(CAM_STATE_DOWN);
		vib_update_key(VIB_KEY_ANGLE);
	} else if (!strncmp(line, "0x27f", 5) || !strncmp(line, "639", 3)) {
		vib_update_key(VIB_KEY_NOT_HOLDER);
	} else if (!strncmp(line, "0x280", 5) || !strncmp(line, "640", 3)) {
		vib_update_key(VIB_KEY_HOLDER);
	} else if (!strncmp(line, "0x282", 5) || !strncmp(line, "642", 3)) {
		vib_update_key(VIB_KEY_NOTIFY2);
	} else if (!strncmp(line, "0x283", 5) || !strncmp(line, "643", 3)) {
		vib_pwm_set_camera(CAM_STATE_DOWN);
		vib_update_key(VIB_KEY_NOTIFY3);
	} else if (!strncmp(line, "0x284", 5) || !strncmp(line, "644", 3) ||
		   !strncmp(line, "666", 3)) {
		/* RESTART_CAMERA_ELEVATOR, following elevator_mode */
		int mode = d->elevator_mode;

		dev_info(d->dev, "%s: RESTART_CAMERA_ELEVATOR to elevator_mode %d\n",
			 __func__, mode);
		switch (mode) {
		case 0:
		case 5:
			break;
		case 2:
		case 4:
			vib_pwm_set_camera(CAM_STATE_FULL);
			break;
		case 6:
			vib_pwm_set_camera(CAM_STATE_HOLDER);
			break;
		default:
			vib_pwm_set_camera(CAM_STATE_MID);
			break;
		}
		vib_update_key(mode == 5 ? VIB_KEY_NOTIFY2 : VIB_KEY_RESTART);
	} else if (!strncmp(line, "888", 3)) {
		/* nothing to do, the vendor only clears its pending flag */
	}
	return count;
}

static ssize_t vib_pwm_holder_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", d->holder_mode);
}

/* [RE] string values only, and two and three send the elevator home */
static ssize_t vib_pwm_holder_mode_store(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	int v;

	if (!buf || !count || buf[0] < '0' || buf[0] > '3')
		return count;
	v = buf[0] - '0';
	d->holder_mode = v;
	if (v == 2 || v == 3)
		vib_pwm_set_camera(CAM_STATE_DOWN);
	return count;
}

/* [RE] the vendor reports five while it is holding a restart back */
static ssize_t vib_pwm_elevator_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", d->elevator_mode);
}

static ssize_t vib_pwm_elevator_mode_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	if (!buf || !count)
		return count;
	switch (buf[0]) {
	case '0': case '1': case '2': case '3': case '4': case '6':
		d->elevator_mode = buf[0] - '0';
		break;
	default:
		break;
	}
	return count;
}

static ssize_t vib_pwm_elevator_row_shift_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	s16 up = -2000, down = -2000;
	int ret;

	ret = bbk_hall_core_read_data(&up, &down);
	if (ret) {
		dev_info(d->dev, "zya damon %s bbk_hall_core_read_data failed\n", __func__);
		return sprintf(buf, "%d", ret);
	}
	d->elevator_row_shift = up;
	return sprintf(buf, "%d %d %d %d %d\n", d->elevator_row_shift,
		       mhall_control3, abs(up), abs(down), mhall_control3 - abs(up));
}

static ssize_t vib_pwm_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", d->is_in_cali);
}

/* [RE] "0" leaves calibration, "1" enters it and homes the elevator */
static ssize_t vib_pwm_cali_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	int v;

	if (!buf || !count || kstrtoint(buf, 10, &v))
		return count;
	if (v == 0) {
		d->is_in_cali = 0;
		bbk_hall_core_enable(0);
	} else {
		d->is_in_cali = 1;
		d->camera_state = CAM_STATE_DOWN;
		d->target_state = CAM_STATE_DOWN;
		bbk_hall_core_enable(1);
	}
	return count;
}

static ssize_t vib_pwm_up_down_count_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", (unsigned)d->up_down_count);
}

/* [RE] the count is also written to persistent storage in one text line */
static ssize_t vib_pwm_up_down_count_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	struct file *f;
	char line[49];
	unsigned int v;
	int n;

	if (sscanf(buf, "%u", &v) != 1)
		return count;
	d->up_down_count = v;

	n = snprintf(line, sizeof(line), "%u-%u", d->up_down_count, d->all_count);
	f = filp_open(VIB_PWM_CALI_FILE, O_RDWR | O_CREAT | O_TRUNC, 0600);
	if (IS_ERR(f)) {
		dev_err(d->dev, "damon save up_down_count file %s failed\n", VIB_PWM_CALI_FILE);
		return count;
	}
	__kernel_write(f, line, n, &f->f_pos);
	filp_close(f, NULL);
	dev_info(d->dev, "damon up_down_count %u,all count:%d\n",
		 d->up_down_count, d->all_count);
	return count;
}

static ssize_t vib_pwm_clear_cali_data_show(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	return sprintf(buf, "clear");
}

static ssize_t vib_pwm_clear_cali_data_store(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	unsigned int v;

	if (sscanf(buf, "%u", &v) != 1 || v != 1)
		return count;
	d->is_in_cali = 0;
	d->up_down_count = 0;
	d->all_count = 0;
	dev_info(d->dev, "%s: hall calibration data cleared\n", __func__);
	return count;
}

static DEVICE_ATTR_RO(vib_pwm_id);
/*
 * [RE] the attribute is called vib_pwm_freq, not vib_pwm_frequency: the factory
 * name string at 0x99f7617 is "vib_pwm_freq" and userspace opens that path.
 */
static DEVICE_ATTR_RW(vib_pwm_freq);
static DEVICE_ATTR_RW(vib_pwm_count);
static DEVICE_ATTR_RW(vib_pwm_enable);
static DEVICE_ATTR_RW(vib_pwm_dir);
static DEVICE_ATTR_RW(vib_pwm_time);
static DEVICE_ATTR_RW(vib_pwm_camera_state);
static DEVICE_ATTR_RW(vib_pwm_state_init);
static DEVICE_ATTR_RW(vib_pwm_abort_notify);
static DEVICE_ATTR_RW(vib_pwm_holder_mode);
static DEVICE_ATTR_RW(vib_pwm_elevator_mode);
static DEVICE_ATTR_RO(vib_pwm_elevator_row_shift);
static DEVICE_ATTR_RW(vib_pwm_cali);
static DEVICE_ATTR_RW(vib_pwm_up_down_count);
static DEVICE_ATTR_RW(vib_pwm_clear_cali_data);

#ifdef CONFIG_BBK_DEBUG_BRINGUP
/*
 * Bring-up only.  The one thing that cannot be seen from userspace is the rate
 * the clock framework ends up programming for the chopper: the vendor sets
 * 32000 for a normal move, and because the travel is a pulse count a wrong rate
 * moves the elevator a wrong distance.
 */
static ssize_t vib_pwm_debug_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	struct clk *p = clk_get_parent(d->gp2_clk);
	int n;

	n = sprintf(buf,
		    "freq=%d time_ms=%u enable=%d state=%d target=%d clk_on=%d clk_prepared=%d\n",
		    (int)d->freq, d->time_ms, d->enable, d->camera_state,
		    d->target_state, d->clk_on, d->clk_prepared);
	n += sprintf(buf + n,
		     "gp2_clk rate=%lu round(%d)=%ld enabled=%d parent=%s parent_rate=%lu\n",
		     clk_get_rate(d->gp2_clk), (int)d->freq,
		     clk_round_rate(d->gp2_clk, d->freq),
		     __clk_is_enabled(d->gp2_clk),
		     (p && !IS_ERR(p)) ? __clk_get_name(p) : "?",
		     (p && !IS_ERR(p)) ? clk_get_rate(p) : 0);
	n += sprintf(buf + n, "boost=%d en=%d sleep=%d dir=%d id=%d\n",
		     gpiod_get_raw_value(gpio_to_desc(d->boost_gpio)),
		     gpiod_get_raw_value(gpio_to_desc(d->enable_gpio)),
		     gpiod_get_raw_value(gpio_to_desc(d->sleep_gpio)),
		     gpiod_get_raw_value(gpio_to_desc(d->dir_gpio)),
		     gpiod_get_raw_value(gpio_to_desc(d->id_gpio)));
	return n;
}

static ssize_t vib_pwm_debug_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct vib_pwm *d = dev_get_drvdata(dev);
	unsigned int ms;
	int v;

	if (sscanf(buf, "time %u", &ms) == 1)
		d->time_ms = ms;
	else if (sscanf(buf, "freq %d", &v) == 1)
		d->freq = v;
	else if (sscanf(buf, "round %d", &v) == 1)
		dev_info(d->dev, "%s: round_rate(%d) = %ld\n", __func__, v,
			 clk_round_rate(d->gp2_clk, v));
	return count;
}
static DEVICE_ATTR_RW(vib_pwm_debug);
#endif

static struct attribute *vib_pwm_attrs[] = {
	&dev_attr_vib_pwm_id.attr,
	&dev_attr_vib_pwm_freq.attr,
	&dev_attr_vib_pwm_count.attr,
	&dev_attr_vib_pwm_enable.attr,
	&dev_attr_vib_pwm_dir.attr,
	&dev_attr_vib_pwm_time.attr,
	&dev_attr_vib_pwm_camera_state.attr,
	&dev_attr_vib_pwm_state_init.attr,
	&dev_attr_vib_pwm_abort_notify.attr,
	&dev_attr_vib_pwm_holder_mode.attr,
	&dev_attr_vib_pwm_elevator_mode.attr,
	&dev_attr_vib_pwm_elevator_row_shift.attr,
	&dev_attr_vib_pwm_cali.attr,
	&dev_attr_vib_pwm_up_down_count.attr,
	&dev_attr_vib_pwm_clear_cali_data.attr,
#ifdef CONFIG_BBK_DEBUG_BRINGUP
	&dev_attr_vib_pwm_debug.attr,
#endif
	NULL,
};
ATTRIBUTE_GROUPS(vib_pwm);

/* --------------------------------------------------------- input device -- */

/* [RE] the vendor registers h110-vib-input and injects keys 635 to 646 */
static int vib_pwm_input_init(struct vib_pwm *d)
{
	struct input_dev *input;
	int key;

	input = devm_input_allocate_device(d->dev);
	if (!input)
		return -ENOMEM;

	input->name = "h110-vib-input";
	input->phys = "h110-vib/input0";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 1;
	input->id.product = 1;
	input->id.version = 0x0100;
	input->dev.parent = d->dev;

	__set_bit(EV_KEY, input->evbit);
	for (key = VIB_KEY_FIRST; key <= VIB_KEY_LAST; key++)
		__set_bit(key, input->keybit);

	if (input_register_device(input)) {
		dev_err(d->dev, "%s: input_register_device failed\n", __func__);
		return -ENODEV;
	}
	d->input = input;
	return 0;
}

/* ------------------------------------------------------------- probe ---- */

static int vib_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct vib_pwm *d;
	int ret;

	/* [RE] the vendor uses devm_kmalloc and leaves fields uninitialised */
	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->dev = dev;
	mutex_init(&d->lock);
	init_waitqueue_head(&d->wait);
	platform_set_drvdata(pdev, d);
	dev_set_drvdata(dev, d);

	/* [RE] the vendor parses the gpios in device tree order */
	d->sleep_gpio = of_get_named_gpio(np, "sleep-gpio", 0);
	d->enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	d->dir_gpio = of_get_named_gpio(np, "dir-gpio", 0);
	d->boost_gpio = of_get_named_gpio(np, "boost-gpio", 0);
	d->id_gpio = of_get_named_gpio(np, "id-gpio", 0);

	if (!gpio_is_valid(d->boost_gpio) || !gpio_is_valid(d->enable_gpio) ||
	    !gpio_is_valid(d->sleep_gpio) || !gpio_is_valid(d->dir_gpio)) {
		dev_err(dev, "%s: one of the motor gpios is missing\n", __func__);
		return -ENODEV;
	}

	ret = devm_gpio_request(dev, d->boost_gpio, "vib_pwm_boost");
	ret |= devm_gpio_request(dev, d->enable_gpio, "vib_pwm_enable");
	ret |= devm_gpio_request(dev, d->sleep_gpio, "vib_pwm_sleep");
	ret |= devm_gpio_request(dev, d->dir_gpio, "vib_pwm_dir");
	if (ret) {
		dev_err(dev, "%s: gpio request failed (%d)\n", __func__, ret);
		return -ENODEV;
	}
	if (gpio_is_valid(d->id_gpio))
		devm_gpio_request(dev, d->id_gpio, "vib_pwm_id");

	/* [RE] the vendor's initial levels */
	vib_pwm_gpio_set(d->boost_gpio, 0);
	vib_pwm_gpio_set(d->enable_gpio, 1);
	vib_pwm_gpio_set(d->sleep_gpio, 0);
	vib_pwm_gpio_set(d->dir_gpio, 1);
	if (gpio_is_valid(d->id_gpio))
		vib_pwm_gpio_input(d->id_gpio);

	d->gp2_clk = devm_clk_get(dev, "gp2_clk");
	if (IS_ERR(d->gp2_clk)) {
		dev_err(dev, "%s: vib_pwm_data->pwm_clk is error\n", __func__);
		return PTR_ERR(d->gp2_clk);
	}

	d->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(d->pinctrl)) {
		d->pins_active = pinctrl_lookup_state(d->pinctrl, "vib_pwm_active");
		d->pins_suspend = pinctrl_lookup_state(d->pinctrl, "vib_pwm_suspend");
		d->pins_idconfig = pinctrl_lookup_state(d->pinctrl, "vib_pwm_idconfig");
		/* read the id strap first, then leave the pins in their idle
		 * state; a move selects the active state again */
		if (!IS_ERR(d->pins_idconfig))
			pinctrl_select_state(d->pinctrl, d->pins_idconfig);
		if (!IS_ERR(d->pins_suspend))
			pinctrl_select_state(d->pinctrl, d->pins_suspend);
	}

	wakeup_source_init(&d->wakeup, "vib_wake_lock");
	hrtimer_init(&d->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
	d->timer.function = vib_pwm_timer_func;
	INIT_WORK(&d->finish_work, vib_pwm_finish_work);

	ret = vib_pwm_input_init(d);
	if (ret)
		return ret;

	/* [RE] the vendor's initial values */
	d->freq = VIB_PWM_FREQ_DEFAULT;
	d->time_ms = mhall_control7;
	d->camera_state = CAM_STATE_DOWN;
	d->target_state = CAM_STATE_DOWN;
	d->enable = 0;
	d->retry_time = VIB_PWM_DEFAULT_RETRY;
	d->key_idle = 1;

	vib_pwm_dev = d;

	ret = sysfs_create_groups(&dev->kobj, vib_pwm_groups);
	if (ret) {
		dev_err(dev, "%s: sysfs_create_groups failed (%d)\n", __func__, ret);
		return ret;
	}

	if (gpio_is_valid(d->id_gpio)) {
		d->id = gpiod_get_raw_value(gpio_to_desc(d->id_gpio));
		dev_info(dev, "%s: elevator id = %d\n", __func__, d->id);
	}
	dev_info(dev, "%s: probed, gp2_clk %lu Hz, all_time %d\n",
		 __func__, clk_get_rate(d->gp2_clk), mhall_control7);
	return 0;
}

static int vib_pwm_remove(struct platform_device *pdev)
{
	struct vib_pwm *d = platform_get_drvdata(pdev);

	if (d) {
		hrtimer_cancel(&d->timer);
		cancel_work_sync(&d->finish_work);
		vib_pwm_brake(d);
		wakeup_source_destroy(&d->wakeup);
		sysfs_remove_groups(&pdev->dev.kobj, vib_pwm_groups);
	}
	vib_pwm_dev = NULL;
	return 0;
}

static int __maybe_unused vib_pwm_suspend(struct device *dev)
{
	struct vib_pwm *d = dev_get_drvdata(dev);

	/* [RE] the vendor only logs here and never stops the motor */
	dev_info(dev, "damon %s\n", __func__);
	(void)d;
	return 0;
}

static const struct dev_pm_ops vib_pwm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(vib_pwm_suspend, NULL)
};

static const struct of_device_id vib_pwm_of_match[] = {
	{ .compatible = "bbk,vib_pwm_control" },
	{ }
};
MODULE_DEVICE_TABLE(of, vib_pwm_of_match);

static struct platform_driver vib_pwm_driver = {
	.probe		= vib_pwm_probe,
	.remove		= vib_pwm_remove,
	.driver		= {
		.name		= VIB_PWM_DRV_NAME,
		.of_match_table	= vib_pwm_of_match,
		.pm		= &vib_pwm_pm_ops,
	},
};

/* ------------------------------------------------- mhall calibration ----- */

/*
 * [RE] the 48 byte blob the framework persists as cali_mhall_final.  Its layout
 * is fixed by the file userspace writes, so it is repeated here rather than
 * shared through a header:
 *   +0x00 s16 hall_up   +0x02 s16 hall_down   +0x04 int valid (must be 1)
 *   +0x08 s16 hall_up_1 +0x10 s16 hall_up_2   +0x18 s16 hall_up_3
 *   +0x20 s16 hall_up_4 +0x28 int f2          +0x2c int add_time
 */
struct mhall_cali_blob {
	s16	hall_up;
	s16	hall_down;
	int	valid;
	s16	hall_up_1;
	u8	pad0[6];
	s16	hall_up_2;
	u8	pad1[6];
	s16	hall_up_3;
	u8	pad2[6];
	s16	hall_up_4;
	u8	pad3[6];
	int	f2;
	int	add_time;
};

/*
 * [RE] init_hall_data_fake: the hall framework calls this after it accepted a
 * calibration blob, and the value comes back as one of our module parameters so
 * that it survives in /sys/module/gpio_pwm/parameters/.  The vendor returns 1 on
 * success and -1 when the blob is not marked valid.
 */
int init_hall_data_fake(const void *p)
{
	const struct mhall_cali_blob *b = p;

	if (!b || b->valid != 1) {
		pr_err("zyhc set mhall cali data failed\n");
		return -1;
	}

	mhall_control_up = b->hall_up;
	mhall_control_down = b->hall_down;
	mhall_control3 = b->hall_up_1;
	mhall_control2 = b->f2;
	mhall_control6 = b->add_time;

	pr_info("zyhc data_up:%d,data_down:%d,fake_data2:%d,fake_data3:%d,add_time:%d\n",
		mhall_control_up, mhall_control_down, mhall_control2,
		mhall_control3, mhall_control6);
	return 1;
}
EXPORT_SYMBOL_GPL(init_hall_data_fake);

module_platform_driver(vib_pwm_driver);

MODULE_AUTHOR("EEBBK S6 kernel reconstruction");
MODULE_DESCRIPTION("BBK elevator camera motor controller (reconstructed from the factory binary)");
MODULE_LICENSE("GPL v2");
