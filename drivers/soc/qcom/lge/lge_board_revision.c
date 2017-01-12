/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/string.h>
#include <soc/qcom/lge/lge_board_revision.h>

static enum hw_rev_type lge_bd_rev = HW_REV_MAX;
#ifdef CONFIG_MACH_MSM8952_B3_ATT_US
/* CAUTION: These strings are come from LK. */
char *rev_str[] = {"hdk_a", "hdk_b", "rdk", "rev_a", "rev_b1", "rev_b",
	"rev_c", "rev_10", "revserved"};
#elif defined(CONFIG_MACH_MSM8952_B5_JP_KDI)
char *rev_str[] = {"rev_a", "rev_b", "rev_c", "rev_d", "rev_e", "rev_f",
	"rev_10", "rev_11", "reserved"};
#else
/* CAUTION: These strings are come from LK. */
char *rev_str[] = {"hdk_a", "hdk_b", "rdk", "rev_a", "rev_b", "rev_c",
	"rev_10", "rev_11", "revserved"};
#endif
static int __init board_revno_setup(char *rev_info)
{
	int i;

	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = i;
			break;
		}
	}

	pr_info("BOARD : LGE %s\n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

enum hw_rev_type lge_get_board_revno(void)
{
	return lge_bd_rev;
}

char *lge_get_board_rev(void)
{
	char *name;
	name = rev_str[lge_bd_rev];
	return name;
}
