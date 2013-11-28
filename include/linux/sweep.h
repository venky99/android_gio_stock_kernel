/*
 * Author: Chad Froebel <chadfroebel@gmail.com>
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

#ifndef _LINUX_SWEEP_H
#define _LINUX_SWEEP_H

extern int sweeptowake;
extern int pocket_keyguard;
extern int doubletap;
extern int mediacontrol;
extern int wake_start;
extern int wake_end;
extern int area_start;
extern int area_end;
extern bool pulse;
extern bool force_locked;
extern bool scr_suspended;
extern bool covered;
extern void out_of_pocket(void);
extern void in_pocket(void);
extern bool disabled;

#endif

