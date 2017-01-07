/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#ifndef _LGE_BOARD_REVISION_H
#define _LGE_BOARD_REVISION_H
#ifdef CONFIG_MACH_MSM8952_B3_ATT_US
enum hw_rev_type {
	HW_REV_HDKA = 0,
	HW_REV_HDKB,
	HW_REV_RDK,
	HW_REV_A,
	HW_REV_B_1,
	HW_REV_B,
	HW_REV_C,
	HW_REV_1_0,
	HW_REV_MAX
};
#elif defined(CONFIG_MACH_MSM8952_B5_JP_KDI)
enum hw_rev_type {
	HW_REV_A = 0,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_F,
	HW_REV_1_0,
    HW_REV_1_1,
	HW_REV_MAX
};
#else
enum hw_rev_type {
	HW_REV_HDKA = 0,
	HW_REV_HDKB,
	HW_REV_RDK,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
};
#endif

extern char *rev_str[];

enum hw_rev_type lge_get_board_revno(void);
char *lge_get_board_rev(void);

#endif
