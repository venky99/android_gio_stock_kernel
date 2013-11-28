/*
 * drivers/cpufreq/cpufreq_zen.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Alin
 *
 * Based on SmartassV2 by Erasmux
 *
 * Based on the Interactive governor By Mike Chan (mike@android.com)
 * which was adapted to 2.6.29 kernel by Nadlabak (pavel@doshaska.net)
 *
 * SMP support based on mod by faux123
 *
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <asm/cputime.h>
#include <linux/earlysuspend.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/sweep.h>


/******************** Tunable parameters: ********************/

/*
 * The "ideal" frequency to use when awake. The governor will ramp up faster
 * towards the ideal frequency and slower after it has passed it. Similarly,
 * lowering the frequency towards the ideal frequency is faster than below it.
 */
#define DEFAULT_IDEAL_STEP_ONE 122880
static unsigned int ideal_step_one;

#define DEFAULT_IDEAL_STEP_TWO 245760
static unsigned int ideal_step_two;
#define DEFAULT_IDEAL_STEP_THREE 320000
static unsigned int ideal_step_three;

#define DEFAULT_IDEAL_STEP_FOUR 480000
static unsigned int ideal_step_four;
static int pulse_freq;

/*
 * Frequency delta when ramping up above the ideal frequency.
 * Zero disables and causes to always jump straight to max frequency.
 * When below the ideal frequency we always ramp up to the ideal freq.
 */
static unsigned int ramp_up_step;

/*
 * CPU freq will be increased if measured load > dynamics_thd;
 */
#define DEFAULT_PERFORMANCE_BIAS 2
static unsigned short perf_bias;
static unsigned short dynamics_thd;
static unsigned short old_load;

#define DEFAULT_SLEEP_THRESHOLD 75
static unsigned short sleep_thd;

/*
 * CPU freq will be decreased if measured load < min_cpu_load;
 */
static unsigned short min_cpu_load;

/*
 * The minimum amount of time to spend at a frequency before we can ramp up.
 * Notice we ignore this when we are below the ideal frequency.
 */
#define DEFAULT_UP_RATE_US 31000;
static unsigned short up_rate_us;

/*
 * Sampling rate, I highly recommend to leave it at 2.
 */
#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;

#define TOUCH_LOAD				75
#define TOUCH_LOAD_THRESHOLD			10
#define TOUCH_LOAD_DURATION			1200
static unsigned int touch_load_duration;
static unsigned int touch_load;
static unsigned int touch_load_threshold;


/*************** End of tunables ***************/

static void (*pm_idle_old)(void);
static atomic_t active_count = ATOMIC_INIT(0);

struct timer_list touchpulse;
static unsigned timer_delay;
static bool touch;

struct zen_info_s {
	struct cpufreq_policy *cur_policy;
	struct cpufreq_frequency_table *freq_table;
	struct timer_list timer;
	u64 time_in_idle;
	u64 idle_exit_time;
	u64 freq_change_time;
	u64 freq_change_time_in_idle;
	int cur_cpu_load;
	int old_freq;
	int ramp_dir;
	unsigned int enable;
	int ideal_speed;
};
static DEFINE_PER_CPU(struct zen_info_s, zen_info);

/* Workqueues handle frequency scaling */
static struct workqueue_struct *up_wq;
static struct workqueue_struct *down_wq;
static struct work_struct freq_scale_work;

static cpumask_t work_cpumask;
static spinlock_t cpumask_lock;

static unsigned int suspended;

static int cpufreq_governor_zen(struct cpufreq_policy *policy,
		unsigned int event);

struct cpufreq_governor cpufreq_gov_zen = {
	.name = "Zen",
	.governor = cpufreq_governor_zen,
	.max_transition_latency = 9000000,
	.owner = THIS_MODULE,
};

inline static void zen_dynamics_suspend(struct zen_info_s *this_zen, struct cpufreq_policy *policy) {
		this_zen->ideal_speed = policy->min;
		up_rate_us = dynamics_thd*700;
}

inline static void zen_dynamics_awake(struct zen_info_s *this_zen, struct cpufreq_policy *policy) {

	if (old_load != this_zen->cur_cpu_load) {
		if (this_zen->ideal_speed != ideal_step_one && this_zen->cur_cpu_load < (dynamics_thd - 80))
				this_zen->ideal_speed = ideal_step_one;
			
		if (this_zen->ideal_speed != ideal_step_two && this_zen->cur_cpu_load >= (dynamics_thd - 80) && 
			this_zen->cur_cpu_load < (dynamics_thd - 60)) 
				this_zen->ideal_speed = ideal_step_two;

		if (this_zen->ideal_speed != ideal_step_three && this_zen->cur_cpu_load > (dynamics_thd - 60) && 
			this_zen->cur_cpu_load < (dynamics_thd - 35))
				this_zen->ideal_speed = ideal_step_three;

		if (this_zen->ideal_speed != ideal_step_four && this_zen->cur_cpu_load > (dynamics_thd - 35)) 
			this_zen->ideal_speed = ideal_step_four; 
		
		if (policy->max > pulse_freq && this_zen->ideal_speed != pulse_freq &&
			this_zen->cur_cpu_load > (dynamics_thd - 14))
			this_zen->ideal_speed = pulse_freq;

		if (this_zen->cur_cpu_load < 90) {
			up_rate_us = (110 - this_zen->cur_cpu_load)*dynamics_thd*7; 
		} else {
			up_rate_us = 18000;
		}
	}
	old_load = this_zen->cur_cpu_load;
}

inline static void zen_dynamics_update(void) {
	dynamics_thd = 100-perf_bias;
	min_cpu_load = dynamics_thd - 25;
	ramp_up_step = 28000*(102-dynamics_thd);
}

inline static unsigned int validate_freq(struct cpufreq_policy *policy, int freq) {
	if (freq > (int)policy->max)
		return policy->max;
	if (freq < (int)policy->min)
		return policy->min;
	return freq;
}

inline static void reset_timer(unsigned long cpu, struct zen_info_s *this_zen) {
	this_zen->time_in_idle = get_cpu_idle_time_us(cpu, &this_zen->idle_exit_time);
	mod_timer(&this_zen->timer, jiffies + sample_rate_jiffies);
}

inline static void work_cpumask_set(unsigned long cpu) {
	unsigned long flags;
	spin_lock_irqsave(&cpumask_lock, flags);
	cpumask_set_cpu(cpu, &work_cpumask);
	spin_unlock_irqrestore(&cpumask_lock, flags);
}

inline static int work_cpumask_test_and_clear(unsigned long cpu) {
	unsigned long flags;
	int res = 0;
	spin_lock_irqsave(&cpumask_lock, flags);
	res = cpumask_test_and_clear_cpu(cpu, &work_cpumask);
	spin_unlock_irqrestore(&cpumask_lock, flags);
	return res;
}

inline static int target_freq(struct cpufreq_policy *policy, struct zen_info_s *this_zen,
			      int new_freq, int old_freq, int prefered_relation) {
	int index, target;
	struct cpufreq_frequency_table *table = this_zen->freq_table;

	if (new_freq == old_freq)
		return 0;
	new_freq = validate_freq(policy,new_freq);
	if (new_freq == old_freq)
		return 0;

	if (table &&
	    !cpufreq_frequency_table_target(policy,table,new_freq,prefered_relation,&index))
	{
		target = table[index].frequency;
		if (target == old_freq) {
			// if for example we are ramping up to *at most* current + ramp_up_step
			// but there is no such frequency higher than the current, try also
			// to ramp up to *at least* current + ramp_up_step.
			if (new_freq > old_freq && prefered_relation==CPUFREQ_RELATION_H
			    && !cpufreq_frequency_table_target(policy,table,new_freq,
							       CPUFREQ_RELATION_L,&index))
				target = table[index].frequency;
			// simlarly for ramping down:
			else if (new_freq < old_freq && prefered_relation==CPUFREQ_RELATION_L
				&& !cpufreq_frequency_table_target(policy,table,new_freq,
								   CPUFREQ_RELATION_H,&index))
				target = table[index].frequency;
		}

		if (target == old_freq) {
			// We should not get here:
			// If we got here we tried to change to a validated new_freq which is different
			// from old_freq, so there is no reason for us to remain at same frequency.
			return 0;
		}
	}
	else target = new_freq;

	__cpufreq_driver_target(policy, target, prefered_relation);


	return target;
}

static void cpufreq_zen_timer(unsigned long cpu)
{
	u64 delta_idle;
	u64 delta_time;
	int cpu_load;
	int old_freq;
	u64 update_time;
	u64 now_idle;
	int queued_work = 0;
	struct zen_info_s *this_zen = &per_cpu(zen_info, cpu);
	struct cpufreq_policy *policy = this_zen->cur_policy;

	now_idle = get_cpu_idle_time_us(cpu, &update_time);
	old_freq = policy->cur;

	if (this_zen->idle_exit_time == 0 || update_time == this_zen->idle_exit_time)
		return;

	delta_idle = cputime64_sub(now_idle, this_zen->time_in_idle);
	delta_time = cputime64_sub(update_time, this_zen->idle_exit_time);

	// If timer ran less than 1ms after short-term sample started, retry.
	if (delta_time < 1000) {
		if (!timer_pending(&this_zen->timer))
			reset_timer(cpu,this_zen);
		return;
	}

	if ((delta_time == 0) || (delta_idle > delta_time)) {
		cpu_load = 0;
	} else {
		cpu_load = 100 * (unsigned int)(delta_time - delta_idle) / (unsigned int)delta_time;
	}

	/* Boost only CPUs with load > touch_load_thershold */
	  if (touch && cpu_load < touch_load &&
	      cpu_load > touch_load_threshold)
		    cpu_load = touch_load;
	
	this_zen->cur_cpu_load = cpu_load;
	this_zen->old_freq = old_freq;

	// Scale up if load is above max or if there where no idle cycles since coming out of idle,
	// additionally, if we are at or above the ideal_speed, verify we have been at this frequency
	// for at least up_rate_us:
	if (cpu_load > 99 || delta_idle == 0)
	{
		if (old_freq < policy->max &&
			 (old_freq < this_zen->ideal_speed || delta_idle == 0 ||
			  cputime64_sub(update_time, this_zen->freq_change_time) >= up_rate_us))
		{
			this_zen->ramp_dir = 1;
			work_cpumask_set(cpu);
			queue_work(up_wq, &freq_scale_work);
			queued_work = 1;
		}
		else this_zen->ramp_dir = 0;
	}
	// Similarly for scale down: load should be below min
	else if (cpu_load < min_cpu_load && old_freq > policy->min &&
		 (old_freq > this_zen->ideal_speed ||
		  cputime64_sub(update_time, this_zen->freq_change_time) >= 10000))
	{
		this_zen->ramp_dir = -1;
		work_cpumask_set(cpu);
		queue_work(down_wq, &freq_scale_work);
		queued_work = 1;
	}
	else this_zen->ramp_dir = 0;

	// To avoid unnecessary load when the CPU is already at high load, we don't
	// reset ourselves if we are at max speed. If and when there are idle cycles,
	// the idle loop will activate the timer.
	// Additionally, if we queued some work, the work task will reset the timer
	// after it has done its adjustments.
	if (!queued_work && old_freq < policy->max)
		reset_timer(cpu,this_zen);
}

static void cpufreq_idle(void)
{
	struct zen_info_s *this_zen = &per_cpu(zen_info, smp_processor_id());
	struct cpufreq_policy *policy = this_zen->cur_policy;

	if (!this_zen->enable) {
		pm_idle_old();
		return;
	}

	if (policy->cur == policy->min && timer_pending(&this_zen->timer))
		del_timer(&this_zen->timer);

	pm_idle_old();

	if (!timer_pending(&this_zen->timer))
		reset_timer(smp_processor_id(), this_zen);
}

/* We use the same work function to scale up and down */
static void cpufreq_zen_freq_change_time_work(struct work_struct *work)
{
	unsigned int cpu;
	int new_freq;
	int old_freq;
	int ramp_dir;
	struct zen_info_s *this_zen;
	struct cpufreq_policy *policy;
	unsigned int relation = CPUFREQ_RELATION_L;

	for_each_possible_cpu(cpu) {
		this_zen = &per_cpu(zen_info, cpu);
		if (!work_cpumask_test_and_clear(cpu))
			continue;

		ramp_dir = this_zen->ramp_dir;
		this_zen->ramp_dir = 0;

		old_freq = this_zen->old_freq;
		policy = this_zen->cur_policy;

	
		if (!suspended)
			zen_dynamics_awake(this_zen,policy);

		if (pulse)
			new_freq = pulse_freq;
		else {
			if (old_freq != policy->cur) {
				// frequency was changed by someone else?
				new_freq = old_freq;
			}
				else if (ramp_dir > 0 && nr_running() > 1) {
				// ramp up logic:
				if (old_freq < this_zen->ideal_speed)
					new_freq = this_zen->ideal_speed;
				else {
					new_freq = old_freq + ramp_up_step;
					relation = CPUFREQ_RELATION_H;
				}
			}
			else if (ramp_dir < 0) {
				// ramp down logic:
				if (old_freq > this_zen->ideal_speed) {
					new_freq = this_zen->ideal_speed;
					relation = CPUFREQ_RELATION_H;
				}
				else {
					new_freq = policy->min;
				}
			}
			else { // ramp_dir==0 ?! Could the timer change its mind about a queued ramp up/down
				// before the work task gets to run?
				// This may also happen if we refused to ramp up because the nr_running()==1
				new_freq = old_freq;
			}
		}

		// do actual ramp up (returns 0, if frequency change failed):
		new_freq = target_freq(policy,this_zen,new_freq,old_freq,relation);
		if (new_freq)
			this_zen->freq_change_time_in_idle =
				get_cpu_idle_time_us(cpu,&this_zen->freq_change_time);

		// reset timer:
		if (new_freq < policy->max)
			reset_timer(cpu,this_zen);
		// if we are maxed out, it is pointless to use the timer
		// (idle cycles wake up the timer when the timer comes)
		else if (timer_pending(&this_zen->timer))
			del_timer(&this_zen->timer);
	}
}

static ssize_t show_up_rate_us(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", up_rate_us);
}

static ssize_t store_up_rate_us(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input >= 0 && input <= 100000000)
		up_rate_us = input;
	return count;
}

static ssize_t show_ideal_step_one(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ideal_step_one);
}

static ssize_t store_ideal_step_one(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input >= 0) 
		ideal_step_one = input;
	return count;
}

static ssize_t show_ideal_step_two(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ideal_step_two);
}

static ssize_t store_ideal_step_two(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input >= 0) 
		ideal_step_two = input;
	return count;
}

static ssize_t show_ideal_step_three(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ideal_step_three);
}

static ssize_t store_ideal_step_three(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input >= 0) {
		ideal_step_three = input;
	}
	return count;
}

static ssize_t show_ideal_step_four(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ideal_step_four);
}

static ssize_t store_ideal_step_four(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input >= 0) {
		ideal_step_four = input;
	}
	return count;
}

static ssize_t show_sample_rate_jiffies(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sample_rate_jiffies);
}

static ssize_t store_sample_rate_jiffies(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input > 0 && input <= 1000)
		sample_rate_jiffies = input;
	return count;
}

static ssize_t show_ramp_up_step(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ramp_up_step);
}

static ssize_t store_ramp_up_step(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input >= 0)
		ramp_up_step = input;
	return count;
}

static ssize_t show_dynamics_thd(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dynamics_thd);
}

static ssize_t store_dynamics_thd(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input > 0 && input <= 100) {
		dynamics_thd = input;
		zen_dynamics_update();
	}
	return count;
}

static ssize_t show_perf_bias(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", perf_bias);
}

static ssize_t store_perf_bias(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input > 0 && input <= 15) {
		perf_bias = input;
		zen_dynamics_update();
	}
	return count;
}

static ssize_t show_sleep_thd(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sleep_thd);
}

static ssize_t store_sleep_thd(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input > 0 && input <= 100) {
		sleep_thd = input;
	}
	return count;
}

static ssize_t show_min_cpu_load(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_cpu_load);
}

static ssize_t store_min_cpu_load(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &input);
	if (res < 0)
		return -EINVAL;
	if (input > 0 && input < 100)
		min_cpu_load = input;
	return count;
}


#define define_global_rw_attr(_name)		\
static struct global_attr _name##_attr =	\
	__ATTR(_name, 0666, show_##_name, store_##_name)

#define define_global_ro_attr(_name)		\
static struct global_attr _name##_attr =	\
	__ATTR(_name, 0444, show_##_name, store_##_name)

define_global_ro_attr(up_rate_us);
define_global_rw_attr(ideal_step_one);
define_global_rw_attr(ideal_step_two);
define_global_rw_attr(ideal_step_three);
define_global_rw_attr(ideal_step_four);
define_global_ro_attr(sample_rate_jiffies);
define_global_ro_attr(ramp_up_step);
define_global_ro_attr(dynamics_thd);
define_global_rw_attr(perf_bias);
define_global_rw_attr(sleep_thd);
define_global_ro_attr(min_cpu_load);

static struct attribute * zen_attributes[] = {
	&up_rate_us_attr.attr,
	&ideal_step_one_attr.attr,
	&ideal_step_two_attr.attr,
	&ideal_step_three_attr.attr,
	&ideal_step_four_attr.attr,
	&sample_rate_jiffies_attr.attr,
	&ramp_up_step_attr.attr,
	&dynamics_thd_attr.attr,
	&perf_bias_attr.attr,
	&sleep_thd_attr.attr,
	&min_cpu_load_attr.attr,
	NULL,
};

static struct attribute_group zen_attr_group = {
	.attrs = zen_attributes,
	.name = "zen",
};

static void input_timeout(unsigned long timeout)
{
	touch = false;
}

static void zen_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	if (!touch) {
		touch = true;
		touchpulse.expires = jiffies + timer_delay;
		add_timer(&touchpulse);
	} else
		mod_timer(&touchpulse, jiffies + timer_delay);
}

static int zen_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void zen_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id zen_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler zen_input_handler = {
	.event		= zen_input_event,
	.connect	= zen_input_connect,
	.disconnect	= zen_input_disconnect,
	.name		= "cpufreq_ond",
	.id_table	= zen_ids,
};

static int cpufreq_governor_zen(struct cpufreq_policy *new_policy,
		unsigned int event)
{
	unsigned int cpu = new_policy->cpu;
	int rc;
	struct zen_info_s *this_zen = &per_cpu(zen_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!new_policy->cur))
			return -EINVAL;

		this_zen->cur_policy = new_policy;

		this_zen->enable = 1;
		if (suspended)
			zen_dynamics_suspend(this_zen,new_policy);
		else
			this_zen->ideal_speed = ideal_step_three;
			up_rate_us = 31000;

		this_zen->freq_table = cpufreq_frequency_get_table(cpu);

		smp_wmb();

		// Do not register the idle hook and create sysfs
		// entries if we have already done so.
		if (atomic_inc_return(&active_count) <= 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&zen_attr_group);
			if (rc)
				return rc;

			pm_idle_old = pm_idle;
			pm_idle = cpufreq_idle;
		}

		if (!cpu)
			rc = input_register_handler(&zen_input_handler);

		if (this_zen->cur_policy->cur < new_policy->max && !timer_pending(&this_zen->timer))
			reset_timer(cpu,this_zen);

		break;

	case CPUFREQ_GOV_LIMITS:
		if (suspended)
			zen_dynamics_suspend(this_zen,new_policy);
		else
			this_zen->ideal_speed = ideal_step_three;
			up_rate_us = 31000;

		if (this_zen->cur_policy->cur > new_policy->max) {
			__cpufreq_driver_target(this_zen->cur_policy,
						new_policy->max, CPUFREQ_RELATION_H);
		}
		else if (this_zen->cur_policy->cur < new_policy->min) {
			__cpufreq_driver_target(this_zen->cur_policy,
						new_policy->min, CPUFREQ_RELATION_L);
		}

		if (this_zen->cur_policy->cur < new_policy->max && !timer_pending(&this_zen->timer))
			reset_timer(cpu,this_zen);

		break;

	case CPUFREQ_GOV_STOP:
		this_zen->enable = 0;
		if (!cpu)
			input_unregister_handler(&zen_input_handler);
		smp_wmb();
		del_timer(&this_zen->timer);
		flush_work(&freq_scale_work);
		this_zen->idle_exit_time = 0;

		if (atomic_dec_return(&active_count) <= 1) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &zen_attr_group);
			pm_idle = pm_idle_old;
		}
		break;
	}

	return 0;
}

static void zen_suspend(int cpu, int suspend)
{
	struct zen_info_s *this_zen = &per_cpu(zen_info, smp_processor_id());
	struct cpufreq_policy *policy = this_zen->cur_policy;
	unsigned int new_freq;

	if (!this_zen->enable)
		return;

	if (suspend) {
		zen_dynamics_suspend(this_zen,policy);
		// to avoid wakeup issues with quick sleep/wakeup don't change actual frequency when entering sleep
		// to allow some time to settle down. Instead we just reset our statistics (and reset the timer).
		// Eventually, the timer will adjust the frequency if necessary.

		this_zen->freq_change_time_in_idle =
			get_cpu_idle_time_us(cpu,&this_zen->freq_change_time);
	} else {
		this_zen->ideal_speed = ideal_step_three;
		up_rate_us = 20000;

		new_freq = policy->max;

		__cpufreq_driver_target(policy, new_freq,
					CPUFREQ_RELATION_L);
	}

	reset_timer(smp_processor_id(),this_zen);
}

static void zen_early_suspend(struct early_suspend *handler) {
	int i;
	if (suspended)
		return;
	suspended = 1;
	for_each_online_cpu(i)
		zen_suspend(i,1);
}

static void zen_late_resume(struct early_suspend *handler) {
	int i;
	if (!suspended) // already not suspended so nothing to do
		return;
	suspended = 0;
	for_each_online_cpu(i)
		zen_suspend(i,0);
}

static struct early_suspend zen_power_suspend = {
	.suspend = zen_early_suspend,
	.resume = zen_late_resume,
#ifdef CONFIG_MACH_HERO
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
#endif
};

static int __init cpufreq_gov_zen_init(void)
{
	unsigned int i;
	struct zen_info_s *this_zen;
	up_rate_us = DEFAULT_UP_RATE_US;
	ideal_step_one = DEFAULT_IDEAL_STEP_ONE;
	ideal_step_two = DEFAULT_IDEAL_STEP_TWO;
	ideal_step_three = DEFAULT_IDEAL_STEP_THREE;
	ideal_step_four = DEFAULT_IDEAL_STEP_FOUR;
	sample_rate_jiffies = DEFAULT_SAMPLE_RATE_JIFFIES;
	perf_bias = DEFAULT_PERFORMANCE_BIAS;
	dynamics_thd = 100-perf_bias;
	sleep_thd = DEFAULT_SLEEP_THRESHOLD;
	old_load = 100-perf_bias;
	ramp_up_step = 28000*(102-dynamics_thd);
	min_cpu_load = dynamics_thd-25;
	touch_load_duration = TOUCH_LOAD_DURATION;
	touch_load = TOUCH_LOAD;
	touch_load_threshold = TOUCH_LOAD_THRESHOLD;
	pulse_freq = 600000;

	spin_lock_init(&cpumask_lock);

	suspended = 0;

	/* Initialize per-cpu data: */
	for_each_possible_cpu(i) {
		this_zen = &per_cpu(zen_info, i);
		this_zen->enable = 0;
		this_zen->cur_policy = 0;
		this_zen->ramp_dir = 0;
		this_zen->time_in_idle = 0;
		this_zen->idle_exit_time = 0;
		this_zen->freq_change_time = 0;
		this_zen->freq_change_time_in_idle = 0;
		this_zen->cur_cpu_load = 0;
		// initialize timer:
		init_timer_deferrable(&this_zen->timer);
		this_zen->timer.function = cpufreq_zen_timer;
		this_zen->timer.data = i;
		work_cpumask_test_and_clear(i);
	}

	// Scale up is high priority
	up_wq = create_workqueue("zen_up");
	down_wq = create_workqueue("zen_down");
	if (!up_wq || !down_wq)
		return -ENOMEM;

	init_timer(&touchpulse);
	touchpulse.function = input_timeout;

	INIT_WORK(&freq_scale_work, cpufreq_zen_freq_change_time_work);

	register_early_suspend(&zen_power_suspend);

	return cpufreq_register_governor(&cpufreq_gov_zen);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ZEN
fs_initcall(cpufreq_gov_zen_init);
#else
module_init(cpufreq_gov_zen_init);
#endif

static void __exit cpufreq_gov_zen_exit(void)
{
	del_timer_sync(&touchpulse);
	cpufreq_unregister_governor(&cpufreq_gov_zen);
	destroy_workqueue(up_wq);
	destroy_workqueue(down_wq);
}

module_exit(cpufreq_gov_zen_exit);

MODULE_AUTHOR ("Alin");
MODULE_DESCRIPTION ("'cpufreq_zen' - A dynamic SmartassV2 based cpufreq governor");
MODULE_LICENSE ("GPL");
