/*
 * arch/arm/mach-tegra/board-molly-sensors.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/nct1008.h>
#include <linux/therm_est.h>
#include <linux/of_platform.h>
#include <linux/pid_thermal_gov.h>
#include <linux/tegra-fuse.h>

#include <mach/edp.h>
#include <linux/edp.h>

#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#include <media/tps61310.h>

#include <linux/platform_device.h>
#include <linux/generic_adc_thermal.h>
#include <generated/mach-types.h>
#include <linux/power/sbs-battery.h>

#include "gpio-names.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "board.h"
#include "board-common.h"
#include "board-molly.h"
#include "tegra-board-id.h"
#include "dvfs.h"

#define TEMP_ALERT_GPIO TEGRA_GPIO_PJ0

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init molly_throttle_init(void)
{
	pr_info("%s:\n", __func__);
	balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(molly_throttle_init);

static struct nct1008_platform_data molly_nct1008_pdata = {
	.supported_hwrev = true,
	.extended_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */

	.sensors = {
		[LOC] = {
			.shutdown_limit = 120, /* C */
			.num_trips = 0,
			.tzp = NULL,
		},
		[EXT] = {
			.shutdown_limit = 105, /* C */
			.num_trips = 1,
			.tzp = NULL,
			.trips = {
				{
					.cdev_type = "suspend_soctherm",
					.trip_temp = 50000,
					.trip_type = THERMAL_TRIP_ACTIVE,
					.upper = 1,
					.lower = 1,
					.hysteresis = 5000,
					.mask = 1,
				},
			},
		}
	}
};

static struct i2c_board_info molly_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &molly_nct1008_pdata,
		.irq = -1,
	}
};

static int molly_nct1008_init(void)
{
    int nct1008_port;
    int ret = 0;

    nct1008_port = TEGRA_GPIO_PJ0;

	tegra_add_all_vmin_trips(molly_nct1008_pdata.sensors[EXT].trips,
				&molly_nct1008_pdata.sensors[EXT].num_trips);

    molly_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
    pr_info("%s: nct1008 irq %d", __func__, molly_nct1008_board_info[0].irq);

    ret = gpio_request(nct1008_port, "temp_alert");
    if (ret < 0) {
        pr_err("%s: gpio_request() for nct1008_port %d failed\n",
		       __func__, nct1008_port);
        return ret;
    }

    ret = gpio_direction_input(nct1008_port);
    if (ret < 0) {
        pr_info("%s: calling gpio_free(nct1008_port)", __func__);
        gpio_free(nct1008_port);
        return ret;
    }

	/* molly thermal sensor on I2C3/CAM_I2C, i.e. instance 2 */
	i2c_register_board_info(2, molly_nct1008_board_info,
				ARRAY_SIZE(molly_nct1008_board_info));

    return ret;
}

int __init molly_sensors_init(void)
{
    molly_nct1008_init();

    return 0;
}
