/*
 * arch/arm/mach-tegra/board-molly-powermon.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation. All Rights Reserved.
 * Copyright (c) 2013, Google, Inc. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/i2c.h>
#include <linux/ina3221.h>

#include "board.h"
#include "board-molly.h"

static struct ina3221_platform_data power_mon_info = {
	.rail_name = {
		"VIN_5V0",     /* total system power */
	},
	.shunt_resistor = {5}, /* 5 mOhm */
	/* Enable ch1.  ch2, ch3 are unused */
	.cont_conf_data = (INA3221_ENABLE_CHAN1 | INA3221_AVG |
			   INA3221_VBUS_CT | INA3221_VSHUNT_CT |
			   INA3221_CONT_MODE),
	/* Enable ch1.  ch2, ch3 are unused */
	.trig_conf_data = (INA3221_ENABLE_CHAN1 | INA3221_TRIG_MODE),
	/* ch1 (Total Power) 3000 mA = 15W
	 * ch2 (Unused)
	 * ch3 (Unused)
	 */
	.crit_alert_limit_curr = {3000},
};

static struct i2c_board_info molly_i2c0_ina3221_board_info[] = {
	{
		I2C_BOARD_INFO("ina3221", 0x40),
		.platform_data = &power_mon_info,
		.irq = -1,
	},
};

int __init molly_pmon_init(void)
{
	i2c_register_board_info(0, molly_i2c0_ina3221_board_info,
		ARRAY_SIZE(molly_i2c0_ina3221_board_info));

	return 0;
}

