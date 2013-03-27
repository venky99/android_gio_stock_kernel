/*
 * drivers/cpufreq/cpufreq_rage.c
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
 
static unsigned int step[6];
 
#define DEFAULT_IDEAL_STEP_ONE 122880

#define DEFAULT_IDEAL_STEP_TWO 245760

#define DEFAULT_IDEAL_STEP_THREE 320000

#define DEFAULT_IDEAL_STEP_FOUR 480000
static int pulse_freq;

/*
 * CPU freq will be increased if measured load > 20;
 */
static unsigned short old_load;

/*
 * The minimum amount of time to spend at a frequency before we can ramp up.
 * Notice we ignore this when we are below the ideal frequency.
 */
#define DEFAULT_UP_RATE_US 30000;
static unsigned int up_rate_us;

/*
 * Sampling rate, I highly recommend to leave it at 2.
 */
#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;

/*************** End of tunables ***************/

static void (*pm_idle_old)(void);
static atomic_t active_count = ATOMIC_INIT(0);

struct rage_info_s {
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
static DEFINE_PER_CPU(struct rage_info_s, rage_info);

/* Workqueues handle frequency scaling */
static struct workqueue_struct *up_wq;
static struct work_struct freq_scale_work;

static cpumask_t work_cpumask;
static spinlock_t cpumask_lock;

static unsigned int suspended;

static int cpufreq_governor_rage(struct cpufreq_policy *policy,
		unsigned int event);

struct cpufreq_governor cpufreq_gov_rage = {
	.name = "rage",
	.governor = cpufreq_governor_rage,
	.max_transition_latency = 9000000,
	.owner = THIS_MODULE,
};

inline static unsigned int validate_freq(struct cpufreq_policy *policy, int freq) {
	if (freq > (int)policy->max)
		return policy->max;
	if (freq < (int)policy->min)
		return policy->min;
	return freq;
}

inline static void reset_timer(unsigned long cpu, struct rage_info_s *this_rage) {
	this_rage->time_in_idle = get_cpu_idle_time_us(cpu, &this_rage->idle_exit_time);
	mod_timer(&this_rage->timer, jiffies + sample_rate_jiffies);
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

inline static int target_freq(struct cpufreq_policy *policy, struct rage_info_s *this_rage,
			      int new_freq, int old_freq, int prefered_relation) {
	int index, target;
	struct cpufreq_frequency_table *table = this_rage->freq_table;

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

static void cpufreq_rage_timer(unsigned long cpu)
{
	u64 delta_idle;
	u64 delta_time;
	int cpu_load;
	int old_freq;
	u64 update_time;
	u64 now_idle;
	int queued_work = 0;
	int index;
	struct rage_info_s *this_rage = &per_cpu(rage_info, cpu);
	struct cpufreq_policy *policy = this_rage->cur_policy;
	
	old_load = this_rage->cur_cpu_load;

	now_idle = get_cpu_idle_time_us(cpu, &update_time);
	old_freq = policy->cur;

	if (this_rage->idle_exit_time == 0 || update_time == this_rage->idle_exit_time)
		return;

	delta_idle = cputime64_sub(now_idle, this_rage->time_in_idle);
	delta_time = cputime64_sub(update_time, this_rage->idle_exit_time);

	// If timer ran less than 1ms after short-term sample started, retry.
	if (delta_time < 1000) {
		if (!timer_pending(&this_rage->timer))
			reset_timer(cpu,this_rage);
		return;
	}

	if ((delta_time == 0) || (delta_idle > delta_time)) {
		cpu_load = 0;
	} else {
		cpu_load = 100 * (unsigned int)(delta_time - delta_idle) / (unsigned int)delta_time;
	}

	this_rage->cur_cpu_load = cpu_load;
	this_rage->old_freq = old_freq;

	// Scale up if load is above max or if there where no idle cycles since coming out of idle,
	// additionally, if we are at or above the ideal_speed, verify we have been at this frequency
	// for at least up_rate_us:
	if (cpu_load != old_load || delta_idle == 0)
	{
		if (step[5] != policy->max)
			step[5] = policy->max;

		if (policy->max > step[4])
		{
			if (suspended)
				index = (int)cpu_load/46;
			else
				index = (int)cpu_load/19;
		} else
		{
			if (suspended)
				index = (int)cpu_load/46;
			else
				index = (int)cpu_load/22;
		}
		if (step[index] != policy->cur && (step[index] < policy->cur || cputime64_sub(update_time, this_rage->freq_change_time) >= up_rate_us || delta_idle == 0))
		{
			this_rage->ramp_dir = 1;
			work_cpumask_set(cpu);
			queue_work(up_wq, &freq_scale_work);
			queued_work = 1;
			this_rage->ideal_speed = step[index];
		} else this_rage->ramp_dir = 0;
	} else this_rage->ramp_dir = 0;

	// To avoid unnecessary load when the CPU is already at high load, we don't
	// reset ourselves if we are at max speed. If and when there are idle cycles,
	// the idle loop will activate the timer.
	// Additionally, if we queued some work, the work task will reset the timer
	// after it has done its adjustments.
	if (!queued_work && old_freq < policy->max)
		reset_timer(cpu,this_rage);
}

static void cpufreq_idle(void)
{
	struct rage_info_s *this_rage = &per_cpu(rage_info, smp_processor_id());
	struct cpufreq_policy *policy = this_rage->cur_policy;

	if (!this_rage->enable) {
		pm_idle_old();
		return;
	}

	if (policy->cur == policy->min && timer_pending(&this_rage->timer))
		del_timer(&this_rage->timer);

	pm_idle_old();

	if (!timer_pending(&this_rage->timer))
		reset_timer(smp_processor_id(), this_rage);
}

/* We use the same work function to scale up and down */
static void cpufreq_rage_freq_change_time_work(struct work_struct *work)
{
	unsigned int cpu;
	int new_freq;
	int old_freq;
	int ramp_dir;
	struct rage_info_s *this_rage;
	struct cpufreq_policy *policy;
	unsigned int relation = CPUFREQ_RELATION_L;

	for_each_possible_cpu(cpu) {
		this_rage = &per_cpu(rage_info, cpu);
		if (!work_cpumask_test_and_clear(cpu))
			continue;

		ramp_dir = this_rage->ramp_dir;
		this_rage->ramp_dir = 0;

		old_freq = this_rage->old_freq;
		policy = this_rage->cur_policy;


		if (pulse)
			new_freq = pulse_freq;
		else {
			if (old_freq != policy->cur) {
				// frequency was changed by someone else?
				new_freq = old_freq;
			}
				else if (ramp_dir > 0 && nr_running() > 1) {
				// ramp up logic:
					new_freq = this_rage->ideal_speed;
			} else { // ramp_dir==0 ?! Could the timer change its mind about a queued ramp up/down
				// before the work task gets to run?
				// This may also happen if we refused to ramp up because the nr_running()==1
				new_freq = old_freq;
			}
		}

		// do actual ramp up (returns 0, if frequency change failed):
		new_freq = target_freq(policy,this_rage,new_freq,old_freq,relation);
		if (new_freq)
			this_rage->freq_change_time_in_idle =
				get_cpu_idle_time_us(cpu,&this_rage->freq_change_time);

		// reset timer:
		if (new_freq < policy->max)
			reset_timer(cpu,this_rage);
		// if we are maxed out, it is pointless to use the timer
		// (idle cycles wake up the timer when the timer comes)
		else if (timer_pending(&this_rage->timer))
			del_timer(&this_rage->timer);
	}
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

#define define_global_rw_attr(_name)		\
static struct global_attr _name##_attr =	\
	__ATTR(_name, 0666, show_##_name, store_##_name)

#define define_global_ro_attr(_name)		\
static struct global_attr _name##_attr =	\
	__ATTR(_name, 0444, show_##_name, store_##_name)

define_global_ro_attr(sample_rate_jiffies);

static struct attribute * rage_attributes[] = {
	&sample_rate_jiffies_attr.attr,
	NULL,
};

static struct attribute_group rage_attr_group = {
	.attrs = rage_attributes,
	.name = "rage",
};

static int cpufreq_governor_rage(struct cpufreq_policy *new_policy,
		unsigned int event)
{
	unsigned int cpu = new_policy->cpu;
	int rc;
	struct rage_info_s *this_rage = &per_cpu(rage_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!new_policy->cur))
			return -EINVAL;

		this_rage->cur_policy = new_policy;

		this_rage->enable = 1;
	
		this_rage->ideal_speed = pulse_freq;

		this_rage->freq_table = cpufreq_frequency_get_table(cpu);

		smp_wmb();

		// Do not register the idle hook and create sysfs
		// entries if we have already done so.
		if (atomic_inc_return(&active_count) <= 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&rage_attr_group);
			if (rc)
				return rc;

			pm_idle_old = pm_idle;
			pm_idle = cpufreq_idle;
		}

		if (this_rage->cur_policy->cur < new_policy->max && !timer_pending(&this_rage->timer))
			reset_timer(cpu,this_rage);

		break;

	case CPUFREQ_GOV_LIMITS:
		this_rage->ideal_speed = pulse_freq;

		if (this_rage->cur_policy->cur > new_policy->max) {
			__cpufreq_driver_target(this_rage->cur_policy,
						new_policy->max, CPUFREQ_RELATION_H);
		}
		else if (this_rage->cur_policy->cur < new_policy->min) {
			__cpufreq_driver_target(this_rage->cur_policy,
						new_policy->min, CPUFREQ_RELATION_L);
		}

		if (this_rage->cur_policy->cur < new_policy->max && !timer_pending(&this_rage->timer))
			reset_timer(cpu,this_rage);

		break;

	case CPUFREQ_GOV_STOP:
		this_rage->enable = 0;
		smp_wmb();
		del_timer(&this_rage->timer);
		flush_work(&freq_scale_work);
		this_rage->idle_exit_time = 0;

		if (atomic_dec_return(&active_count) <= 1) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &rage_attr_group);
			pm_idle = pm_idle_old;
		}
		break;
	}

	return 0;
}

static void rage_early_suspend(struct early_suspend *handler) {
	if (suspended)
		return;
	suspended = 1;
	
}

static void rage_late_resume(struct early_suspend *handler) {
	struct rage_info_s *this_rage = &per_cpu(rage_info, smp_processor_id());
	struct cpufreq_policy *policy = this_rage->cur_policy;
	unsigned int new_freq;

	if (!suspended) // already not suspended so nothing to do
		return;
	suspended = 0;
	
	if (!this_rage->enable)
		return;
		
	this_rage->ideal_speed = policy->max;

	new_freq = policy->max;

	__cpufreq_driver_target(policy, new_freq,
					CPUFREQ_RELATION_L);
	reset_timer(smp_processor_id(),this_rage);
}

static struct early_suspend rage_power_suspend = {
	.suspend = rage_early_suspend,
	.resume = rage_late_resume,
#ifdef CONFIG_MACH_HERO
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
#endif
};

static int __init cpufreq_gov_rage_init(void)
{
	unsigned int i;
	struct rage_info_s *this_rage;
	up_rate_us = DEFAULT_UP_RATE_US;
	step[0] = DEFAULT_IDEAL_STEP_ONE;
	step[1] = DEFAULT_IDEAL_STEP_TWO;
	step[2] = DEFAULT_IDEAL_STEP_THREE;
	step[3] = DEFAULT_IDEAL_STEP_FOUR;
	step[4] = 600000;
	step[5] = 600000;
	sample_rate_jiffies = DEFAULT_SAMPLE_RATE_JIFFIES;
	old_load = 99;
	pulse_freq = 600000;

	spin_lock_init(&cpumask_lock);

	suspended = 0;

	/* Initialize per-cpu data: */
	for_each_possible_cpu(i) {
		this_rage = &per_cpu(rage_info, i);
		this_rage->enable = 0;
		this_rage->cur_policy = 0;
		this_rage->ramp_dir = 0;
		this_rage->time_in_idle = 0;
		this_rage->idle_exit_time = 0;
		this_rage->freq_change_time = 0;
		this_rage->freq_change_time_in_idle = 0;
		this_rage->cur_cpu_load = 0;
		// initialize timer:
		init_timer_deferrable(&this_rage->timer);
		this_rage->timer.function = cpufreq_rage_timer;
		this_rage->timer.data = i;
		work_cpumask_test_and_clear(i);
	}

	// Scale up is high priority
	up_wq = create_rt_workqueue("rage_up");

	if (!up_wq)
		return -ENOMEM;

	INIT_WORK(&freq_scale_work, cpufreq_rage_freq_change_time_work);

	register_early_suspend(&rage_power_suspend);

	return cpufreq_register_governor(&cpufreq_gov_rage);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_RAGE
fs_initcall(cpufreq_gov_rage_init);
#else
module_init(cpufreq_gov_rage_init);
#endif

static void __exit cpufreq_gov_rage_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_rage);
	destroy_workqueue(up_wq);
}

module_exit(cpufreq_gov_rage_exit);

MODULE_AUTHOR ("Alin");
MODULE_DESCRIPTION ("'cpufreq_rage' - A dynamic SmartassV2 based cpufreq governor");
MODULE_LICENSE ("GPL");
