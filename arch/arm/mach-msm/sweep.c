/*
 * Author: Chad Froebel <chadfroebel@gmail.com>
 *
 * Simple port to Nexus 4 : motley <motley.slate@gmail.com>
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
 */

/*
 * Possible values for "sweeptowake" are :
 *
 *   0 - disabled
 *   1 - enabled  (default)
*/

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/sweep.h>
#include <linux/module.h>
#include <linux/input.h>

int sweeptowake;
int pocket_keyguard;
int doubletap;
int mediacontrol;

/* sysfs interface for "sweeptowake" */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)          \
{									\
	return sprintf(buf, "%d\n", object);				\
}

show_one(sweeptowake, sweeptowake);
show_one(pocket_keyguard, pocket_keyguard);
show_one(doubletap, doubletap);
show_one(mediacontrol, mediacontrol);

static ssize_t sweeptowake_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int input;
	int ret;
	ret = sscanf(buf, "%d", &input);

	if (ret != 1 || input > 1 || input < 0) {
		return -EINVAL;
	}

	sweeptowake = input;
	return count;
}
static ssize_t pocket_keyguard_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int input;
	int ret;
	ret = sscanf(buf, "%d", &input);

	if (ret != 1 || input > 1 || input < 0) {
		return -EINVAL;
	}

	pocket_keyguard = input;
	return count;
}
static ssize_t doubletap_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int input;
	int ret;
	ret = sscanf(buf, "%d", &input);

	if (ret != 1 || input > 1 || input < 0) {
		return -EINVAL;
	}

	doubletap = input;
	return count;
}
static ssize_t mediacontrol_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int input;
	int ret;
	ret = sscanf(buf, "%d", &input);

	if (ret != 1 || input > 1 || input < 0) {
		return -EINVAL;
	}

	mediacontrol = (doubletap)?input:0;
	return count;
}

#define define_kobj_rw_attr(_name)		\
static struct kobj_attribute _name##_attribute =	\
	__ATTR(_name, 0666, show_##_name, _name##_store)

define_kobj_rw_attr(sweeptowake);
define_kobj_rw_attr(pocket_keyguard);
define_kobj_rw_attr(doubletap);
define_kobj_rw_attr(mediacontrol);

static struct attribute *sweeptowake_attrs[] = {
&sweeptowake_attribute.attr,
&pocket_keyguard_attribute.attr,
&doubletap_attribute.attr,
&mediacontrol_attribute.attr,
NULL,
};

static struct attribute_group sweeptowake_attr_group = {
.attrs = sweeptowake_attrs,
};

/* Initialize sweep sysfs folder */
static struct kobject *sweeptowake_kobj;

static int __init sweep_init(void)
{
	int sweeptowake_retval;

	sweeptowake = 1; /* Sweep2Wake enabled by default */
	pocket_keyguard = 1; /* Pocket Keyguard enabled by default */
	doubletap = 1; /* DoubleTap2Wake enabled by default */
	mediacontrol = (doubletap)?1:0; /* DoubleTap2PlayPause enabled by default */

	sweeptowake_kobj = kobject_create_and_add("sweep", kernel_kobj);
	if (!sweeptowake_kobj) {
			return -ENOMEM;
	}

	sweeptowake_retval = sysfs_create_group(sweeptowake_kobj, &sweeptowake_attr_group);

	if (sweeptowake_retval)
		kobject_put(sweeptowake_kobj);

	return (sweeptowake_retval);
}

static void __exit sweep_exit(void)
{
	kobject_put(sweeptowake_kobj);
}

module_init(sweep_init);
module_exit(sweep_exit);


