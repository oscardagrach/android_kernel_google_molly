/*
 * arch/arm/mach-tegra/board-molly-sdhci.c
 *
 * Copyright (C) 2010-2013 Google, Inc.
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/mbtc_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/mmc/host.h>
#include <linux/wl12xx.h> /* needed? */
#include <linux/platform_data/mmc-sdhci-tegra.h>
#include <linux/tegra-fuse.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/gpio-tegra.h>
#include <mach/io_dpd.h>

#include "tegra-board-id.h"
#include "gpio-names.h"
#include "board.h"
#include "board-molly.h"
#include "dvfs.h"
#include "iomap.h"

#define MOLLY_WLAN_PWR	TEGRA_GPIO_PCC5
#define MOLLY_WLAN_RST	TEGRA_GPIO_PW5
#define MOLLY_WLAN_WOW	TEGRA_GPIO_PU5
#define MOLLY_BT_HOST_WAKE	TEGRA_GPIO_PU6
#define FUSE_CORE_SPEEDO_0	0x134

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int molly_wifi_status_register(void (*callback)(int , void *), void *);

static int molly_wifi_reset(int on);
static int molly_wifi_power(int on);
static int molly_wifi_set_carddetect(int val);

static struct wifi_platform_data molly_wifi_control = {
	.set_power	= molly_wifi_power,
	.set_reset	= molly_wifi_reset,
	.set_carddetect	= molly_wifi_set_carddetect,
	.host_sleep_cond = 0xa, /* UNICAST + MULTICAST */
			        /* bit 0: broadcast, 1: unicast, 2: mac,
			               3: multicast, 6: mgmt frame,
				       31: don't wakeup on ipv6 */
	.host_sleep_gpio = 16,  /* GPIO 16 */
	.host_sleep_gap	= 0xff,  /* level trigger */
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "mrvl_wlan_irq",
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device molly_wifi_device = {
	.name		= "mrvl_wlan",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(wifi_resource),
	.resource	= wifi_resource,
	.dev		= {
		.platform_data = &molly_wifi_control,
	},
};

static struct mbtc_platform_data molly_mbtc_control = {
	.gpio_gap	= 0x04ff, /* GPIO 4, GAP 0xff (level) */
};

static struct resource bt_resource[] = {
	[0] = {
		.name	= "mrvl_bt_irq",
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device molly_bt_device = {
	.name		= "mrvl_bt",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(bt_resource),
	.resource	= bt_resource,
	.dev		= {
		.platform_data = &molly_mbtc_control,
	},
};

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct embedded_sdio_data embedded_sdio_data0 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor	 = 0x02d0,
		.device	 = 0x4329,
	},
};
#endif

struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= molly_wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio = &embedded_sdio_data0,
#endif
		.built_in = 0,
		.ocr_mask = MMC_OCR_1V8_MASK,
	},
#ifndef CONFIG_MMC_EMBEDDED_SDIO
#endif
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x2,
	.trim_delay = 0x2,
	.ddr_clk_limit = 50000000,
	.max_clk_limit = 100000000,
	.uhs_mask = MMC_UHS_MASK_DDR50,
};

struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x5,
	.trim_delay = 0x3,
	.ddr_clk_limit = 41000000,
	.max_clk_limit = 200000000,
	.mmc_data = {
		.built_in = 1,
		.ocr_mask = MMC_OCR_1V8_MASK,
	},
};

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int molly_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int molly_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int molly_wifi_power(int on)
{
	gpio_set_value(MOLLY_WLAN_PWR, on);
	gpio_set_value(MOLLY_WLAN_RST, on);
	mdelay(100);

	return 0;
}

static int molly_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int __init molly_wifi_init(void)
{
	int rc;

	rc = gpio_request(MOLLY_WLAN_PWR, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);
	rc = gpio_request(MOLLY_WLAN_RST, "wlan_rst");
	if (rc)
		pr_err("WLAN_RST gpio request failed:%d\n", rc);
	rc = gpio_request(MOLLY_WLAN_WOW, "wlan_wow");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);
	rc = gpio_request(MOLLY_BT_HOST_WAKE, "bt_host_wake");
	if (rc)
		pr_err("BT_HOST_WAKE gpio request failed:%d\n", rc);

	rc = gpio_direction_output(MOLLY_WLAN_PWR, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	gpio_direction_output(MOLLY_WLAN_RST, 0);
	if (rc)
		pr_err("WLAN_RST gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(MOLLY_WLAN_WOW);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(MOLLY_BT_HOST_WAKE);
	if (rc)
		pr_err("BT_HOST_WAKE gpio direction configuration failed:%d\n", rc);

	wifi_resource[0].start = wifi_resource[0].end =
		gpio_to_irq(MOLLY_WLAN_WOW);
	bt_resource[0].start = bt_resource[0].end =
		gpio_to_irq(MOLLY_BT_HOST_WAKE);

	platform_device_register(&molly_wifi_device);
	platform_device_register(&molly_bt_device);
	return 0;
}

#ifdef CONFIG_TEGRA_PREPOWER_WIFI
static int __init molly_wifi_prepower(void)
{
	molly_wifi_power(1);

	return 0;
}

fs_initcall(molly_wifi_prepower);
#endif

int __init molly_sdhci_init(void)
{
	int nominal_core_mv;
	int min_vcore_override_mv;
    int boot_vcore_mv;
	u32 speedo;

	nominal_core_mv =
		tegra_dvfs_rail_get_nominal_millivolts(tegra_core_rail);
	if (nominal_core_mv) {
		tegra_sdhci_platform_data0.nominal_vcore_mv = nominal_core_mv;
		tegra_sdhci_platform_data3.nominal_vcore_mv = nominal_core_mv;
	}
	min_vcore_override_mv =
		tegra_dvfs_rail_get_override_floor(tegra_core_rail);
	if (min_vcore_override_mv) {
		tegra_sdhci_platform_data0.min_vcore_override_mv =
			min_vcore_override_mv;
		tegra_sdhci_platform_data3.min_vcore_override_mv =
			min_vcore_override_mv;
	}
	if ((tegra_sdhci_platform_data3.uhs_mask & MMC_MASK_HS200)
		&& (!(tegra_sdhci_platform_data3.uhs_mask &
		MMC_UHS_MASK_DDR50)))
		tegra_sdhci_platform_data3.trim_delay = 0;

	boot_vcore_mv = tegra_dvfs_rail_get_boot_level(tegra_core_rail);
	if (boot_vcore_mv) {
		tegra_sdhci_platform_data0.boot_vcore_mv = boot_vcore_mv;
        tegra_sdhci_platform_data3.boot_vcore_mv = boot_vcore_mv;
	}

    tegra_sdhci_platform_data0.disable_clock_gate = 1;

	speedo = tegra_fuse_readl(FUSE_CORE_SPEEDO_0);
	tegra_sdhci_platform_data3.cpu_speedo = speedo;
	tegra_sdhci_platform_data0.cpu_speedo = speedo;

	/* device0 uses resource0, which is sdmmc1.  used for WiFi
	 * device2 uses resource2, which is sdmmc3.
	 *      molly needs SDMMC3 lines for ubik SPI so not
	 *      registering as a sdhci device.
	 * device3 uses resource3, which is sdmmc4.  used for eMMC.
	 *
	 * sdmmc2 is used for modem on dalmore, not configured here.  no
	 * modem in molly
	 */
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device0);
	molly_wifi_init();
	return 0;
}
