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
#define DEFAULT_UP_RATE_US 31000;
static unsigned int up_rate_us;

/*
 * Sampling rate, I highly recommend to leave it at 2.
 */
#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;

/*************** End of tunables ***************/

static void (*pm_idle_old)(void);
static atomic_t active_count = ATOMIC_INIT(0);

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
	int index;
	struct zen_info_s *this_zen = &per_cpu(zen_info, cpu);
	struct cpufreq_policy *policy = this_zen->cur_policy;
	
	old_load = this_zen->cur_cpu_load;

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

	this_zen->cur_cpu_load = cpu_load;
	this_zen->old_freq = old_freq;

	// Scale up if load is above max or if there where no idle cycles since coming out of idle,
	// additionally, if we are at or above the ideal_speed, verify we have been at this frequency
	// for at least up_rate_us:
	if (cpu_load != old_load || delta_idle == 0)
	{
		step[5] = policy->max;
		if (this_zen->cur_cpu_load < 90) {
			up_rate_us = (110 - this_zen->cur_cpu_load)*700; 
		} else {
			up_rate_us = 18000;
		}
		if (policy->max > step[4])
		{
			index = (int)cpu_load/19;
		} else
		{
			index = (int)cpu_load/23;
		}
		if (cputime64_sub(update_time, this_zen->freq_change_time) >= up_rate_us || delta_idle == 0)
		{
			this_zen->ramp_dir = 1;
			work_cpumask_set(cpu);
			queue_work(up_wq, &freq_scale_work);
			queued_work = 1;
			this_zen->ideal_speed = step[index];
		} else this_zen->ramp_dir = 0;
	} else this_zen->ramp_dir = 0;

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


		if (pulse)
			new_freq = pulse_freq;
		else {
			if (old_freq != policy->cur) {
				// frequency was changed by someone else?
				new_freq = old_freq;
			}
				else if (ramp_dir > 0 && nr_running() > 1) {
				// ramp up logic:
					new_freq = this_zen->ideal_speed;
			} else { // ramp_dir==0 ?! Could the timer change its mind about a queued ramp up/down
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

static struct attribute * zen_attributes[] = {
	&sample_rate_jiffies_attr.attr,
	NULL,
};

static struct attribute_group zen_attr_group = {
	.attrs = zen_attributes,
	.name = "zen",
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
	
		this_zen->ideal_speed = pulse_freq;
		up_rate_us = 0;

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

		if (this_zen->cur_policy->cur < new_policy->max && !timer_pending(&this_zen->timer))
			reset_timer(cpu,this_zen);

		break;

	case CPUFREQ_GOV_LIMITS:
		this_zen->ideal_speed = pulse_freq;
		up_rate_us = 0;

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

/*static void zen_suspend(int cpu, int suspend)
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
*/

static void zen_early_suspend(struct early_suspend *handler) {
	if (suspended)
		return;
	suspended = 1;
	
}

static void zen_late_resume(struct early_suspend *handler) {
	struct zen_info_s *this_zen = &per_cpu(zen_info, smp_processor_id());
	struct cpufreq_policy *policy = this_zen->cur_policy;
	unsigned int new_freq;

	if (!suspended) // already not suspended so nothing to do
		return;
	suspended = 0;
	
	if (!this_zen->enable)
		return;
		
	this_zen->ideal_speed = policy->max;
	up_rate_us = 0;

	new_freq = policy->max;

	__cpufreq_driver_target(policy, new_freq,
					CPUFREQ_RELATION_L);
	reset_timer(smp_processor_id(),this_zen);
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
	up_wq = create_rt_workqueue("zen_up");

	if (!up_wq)
		return -ENOMEM;

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
	cpufreq_unregister_governor(&cpufreq_gov_zen);
	destroy_workqueue(up_wq);
}

module_exit(cpufreq_gov_zen_exit);

MODULE_AUTHOR ("Alin");
MODULE_DESCRIPTION ("'cpufreq_zen' - A dynamic SmartassV2 based cpufreq governor");
MODULE_LICENSE ("GPL");
