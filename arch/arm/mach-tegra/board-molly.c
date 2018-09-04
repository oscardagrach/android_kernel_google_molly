/*
 * arch/arm/mach-tegra/board-molly.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2013, Google, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/memblock.h>
#include <linux/moduleparam.h>
#include <linux/reboot.h>
#include <linux/spi/spi-tegra.h>
#include <linux/rfkill-gpio.h>
#include <linux/skbuff.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/tegra_usb_phy.h>
#include <linux/of_platform.h>
#include <linux/edp.h>
#include <linux/irqchip/tegra.h>
#include <linux/clk/tegra.h>
#include <linux/clocksource.h>
#include <linux/tegra_fiq_debugger.h>
#include <linux/irqchip.h>
#include <linux/leds.h>

#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t11.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/system_info.h>
#include <mach/gpio-tegra.h>
#include <linux/aah_io.h>
#include <linux/athome_radio.h>
#include <linux/nct1008.h>

#include <mach/hardware.h>
#include <mach/thermal.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/isomgr.h>
#include <mach/gpio-tegra.h>
#include <mach/xusb.h>

#include "board.h"
#include "board-common.h"
#include "clock.h"
#include "board-molly.h"
#include "devices.h"
#include "gpio-names.h"
#include "iomap.h"
#include "pm.h"
#include "common.h"
#include "tegra-board-id.h"
#include "tegra-of-dev-auxdata.h"

static struct board_info board_info;

int molly_hw_rev;
module_param(molly_hw_rev, int, S_IRUGO);
MODULE_PARM_DESC(molly_hw_rev, "hardware revision");

static const char const *molly_hw_name[] = {
    [MOLLY_REV_PROTO1] = "Molly PROTO1",
    [MOLLY_REV_PROTO2] = "Molly PROTO2",
    [MOLLY_REV_EVT1]   = "Molly EVT1",
    [MOLLY_REV_EVT2]   = "Molly EVT2",
    [MOLLY_REV_EVT3]   = "Molly EVT3",
    [MOLLY_REV_DVT1]   = "Molly DVT1",
    [MOLLY_REV_DVT2]   = "Molly DVT2",
    [MOLLY_REV_PVT1]   = "Molly PVT1",
    [MOLLY_REV_PROD]   = "Molly PROD",
};

static const char *molly_hw_rev_name(void)
{
	int num = ARRAY_SIZE(molly_hw_name);

	if (molly_hw_rev >= num ||
	    !molly_hw_name[molly_hw_rev])
		return "Molly unknown version";

	return molly_hw_name[molly_hw_rev];
}

static void __init molly_init_hw_rev(void)
{
	/* ATAG_REVISION is not processed when device trees are used
	 * which causes system_rev to not be set.  Set system_rev based
	 * on the Molly board revision.
	 */
	system_rev = molly_hw_rev;

	pr_info("Molly HW revision: %02x (%s)\n",
		molly_hw_rev, molly_hw_rev_name());
}

static __initdata struct tegra_clk_init_table molly_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	/* Setting vi_sensor-clk to true for validation purpose, will imapact
	 * power, later set to be false.*/
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

/******************************************************************************
 *                                                                            *
 *           aah_io driver platform data                                      *
 *                                                                            *
 ******************************************************************************/
static struct aah_io_platform_data aah_io_data = {
	.key_gpio = TEGRA_GPIO_PQ5, /* molly's UI_SWITCH, KB_COL5/GPIO_PQ5 */
	.key_code = KEY_CONNECT, /* hardware pairing button */
};

static struct i2c_board_info __initdata aah_io_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("aah-io", 0x32),
		.platform_data = &aah_io_data,
	},
};

static void __init molly_i2c_init(void)
{
	/* Tegra4 has five i2c controllers:
	 * I2C_1 is called GEN1_I2C in pinmux/schematics
	 * I2C_2 is called GEN2_I2C in pinmux/schematics
	 * I2C_3 is called CAM_I2C in pinmux/schematics
	 * I2C_4 is called DDC_I2C in pinmux/schematics
	 * I2C_5/PMU is called PWR_I2C in pinmux/schematics
	 *
	 * I2C1/GEN1 is for INA3221 current and bus voltage monitor
	 * I2C2/GEN2 is for LED
	 * I2C3/CAM is for TMP451 (nct1008 temp sensor for
	 *  dalmore is on I2C1)
	 * I2C4 is for HDMI/DDC
	 * I2C5/PWR is for PMIC TPS65913B2B5
	 */

	i2c_register_board_info(1, aah_io_i2c_board_info,
				ARRAY_SIZE(aah_io_i2c_board_info));
}

static struct tegra_serial_platform_data molly_uartd_pdata = {
	.dma_req_selector = 19,
	.modem_interrupt = false,
};

static void __init molly_uart_init(void)
{
#ifndef CONFIG_USE_OF
	tegra_uartd_device.dev.platform_data = &molly_uartd_pdata;
	platform_add_devices(molly_uart_devices,
			ARRAY_SIZE(molly_uart_devices));
#endif
	tegra_uartd_device.dev.platform_data = &molly_uartd_pdata;
	if (!is_tegra_debug_uartport_hs()) {
		int debug_port_id = uart_console_debug_init(3);
		if (debug_port_id < 0)
			return;

#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
		tegra_serial_debug_init_irq_mode(TEGRA_UARTD_BASE, INT_UARTD, NULL, -1, -1);
#else
		platform_device_register(uart_console_debug_device);
#endif
	} else {
		tegra_uartd_device.dev.platform_data = &molly_uartd_pdata;
		platform_device_register(&tegra_uartd_device);
	}
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct platform_device *molly_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra11_se_device,
#endif
	&tegra_hda_device,
#if defined(CONFIG_TEGRA_CEC_SUPPORT)
	&tegra_cec_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.unaligned_dma_buf_supported = false,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static void __init molly_wake_sources_init(void)
{
	/* Set HDMI HPD GPIO as wakeup source */
	tegra_set_wake_gpio(4, MOLLY_HDMI_HPD);
}

static void __init molly_usb_init(void)
{
	/* Set USB wake sources for molly */
	tegra_set_usb_wake_source();
	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
}

/* SMSC LAN9730 ethernet controller.
 * Initially reset is asserted.
 * TODO: How to use the phy_int_n signal?  SMSC driver doesn't take
 * platform data.  Maybe just hook up here as a irq for wake?
 */
#define GPIO_ETHERNET_PHY_INT_N     TEGRA_GPIO_PR4  /* KB_ROW4/GPIO_PR4 */
#define GPIO_ETHERNET_PHY_INT_N_3V3 TEGRA_GPIO_PEE4 /* SDMMC3_CLK_LB_OUT/GPIO_PEE4 */
#define GPIO_ETHERNET_RESET_N       TEGRA_GPIO_PEE5 /* SDMMC3_CLK_LB_IN/GPIO_PEE5 */

static struct gpio ethernet_gpios[] __initdata = {
	{GPIO_ETHERNET_PHY_INT_N, GPIOF_IN,           "ethernet_phy_int_n" },
	{GPIO_ETHERNET_PHY_INT_N_3V3, GPIOF_IN,       "ethernet_phy_int_n_3v3" },
	{GPIO_ETHERNET_RESET_N,   GPIOF_OUT_INIT_LOW, "ethernet_reset_n" },
};

static struct gpio ethernet_gpios_dvt1[] __initdata = {
	{GPIO_ETHERNET_PHY_INT_N_3V3, GPIOF_IN,       "ethernet_phy_int_n" },
	{GPIO_ETHERNET_RESET_N,   GPIOF_OUT_INIT_LOW, "ethernet_reset_n" },
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = false,
	},
};

static void __init molly_hsic_init(void)
{
	int ret;

	switch (molly_hw_rev) {
	case MOLLY_REV_PROTO1:
	case MOLLY_REV_PROTO2:
	case MOLLY_REV_EVT1:
	case MOLLY_REV_DVT1:
		ret = gpio_request_array(ethernet_gpios_dvt1,
				ARRAY_SIZE(ethernet_gpios_dvt1));
		break;
	default:
		ret = gpio_request_array(ethernet_gpios,
				ARRAY_SIZE(ethernet_gpios));
	};

	if (ret) {
		pr_warn("%s:gpio request failed\n", __func__);
		return;
	}

	/* delay after reset asserted by gpio_request_array() */
	udelay(100);

	/* take out of reset */
	gpio_set_value(GPIO_ETHERNET_RESET_N, 1);

	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_hsic_pdata;
	platform_device_register(&tegra_ehci2_device);
}

#define ATHOME_RADIO_INT_GPIO     TEGRA_GPIO_PS2 /* KB_ROW10/GPIO_PS2 */
#define ATHOME_RADIO_INT_3V3_GPIO TEGRA_GPIO_PB4 /* SDMMC3_DAT3/GPIO_PB4 */
#define ATHOME_RADIO_RESET_N_GPIO TEGRA_GPIO_PB5 /* SDMMC3_DAT2/GPIO_PB5 */
#define ATHOME_RADIO_SPI_CS_GPIO  TEGRA_GPIO_PA7 /* SDMMC3_CMD/GPIO_PA7 */

static struct athome_platform_data radio_pdata = {
	.gpio_num_irq = ATHOME_RADIO_INT_GPIO,
	.gpio_num_rst = ATHOME_RADIO_RESET_N_GPIO,
	.gpio_spi_cs  = ATHOME_RADIO_SPI_CS_GPIO,
};

static struct athome_platform_data radio_pdata_dvt1 = {
	.gpio_num_irq = ATHOME_RADIO_INT_3V3_GPIO,
	.gpio_num_rst = ATHOME_RADIO_RESET_N_GPIO,
	.gpio_spi_cs  = ATHOME_RADIO_SPI_CS_GPIO,
};

#define ATHOME_RADIO_SPI_BUS_NUM 2 /* bus 2 == spi3 */
#define ATHOME_RADIO_SPI_CS      0
/* 2MHZ is max for sim3 right now.  Need to verify
 * clock values available to SPI for Tegra.
 * Depends on clks (dalmore pll_p is 408MHz and clk_m is 12MHz)
 * and dividers available.
 * 1.5MHz was setting we used in wolfie.
 */
#define ATHOME_RADIO_SPI_MAX_HZ  1500000

static struct spi_board_info molly_radio_spi_info[] __initdata = {
	{
		.modalias	= ATHOME_RADIO_MOD_NAME,
		.platform_data  = &radio_pdata,
		.irq		= -1,
		.max_speed_hz   = ATHOME_RADIO_SPI_MAX_HZ,
		.bus_num	= ATHOME_RADIO_SPI_BUS_NUM,
		.chip_select	= ATHOME_RADIO_SPI_CS,
		.mode           = SPI_MODE_0,
	},
};

static void __init molly_radio_init(void)
{
	switch (molly_hw_rev) {
	case MOLLY_REV_PROTO1:
	case MOLLY_REV_PROTO2:
	case MOLLY_REV_EVT1:
	case MOLLY_REV_DVT1:
		molly_radio_spi_info[0].platform_data = &radio_pdata_dvt1;
	};

	spi_register_board_info(molly_radio_spi_info,
				ARRAY_SIZE(molly_radio_spi_info));
}

static struct tegra_spi_platform_data molly_spi_pdata = {
	.is_clkon_always        = false,
	.spi_max_frequency      = 25000000,
};

static void __init molly_spi_init(void)
{
	tegra11_spi_device3.dev.platform_data = &molly_spi_pdata;
    platform_device_register(&tegra11_spi_device3);
}

#define MOLLY_BOOT_EMC_RATE 792000000
static void __init molly_boost_emc_clk_for_boot(bool if_boost)
{
	static struct clk *boot_emc_clk;

	if (if_boost) {
		if (IS_ERR_OR_NULL(boot_emc_clk)) {
			boot_emc_clk = clk_get_sys("boot", "emc");
			if (IS_ERR_OR_NULL(boot_emc_clk)) {
				pr_err("%s: failed to get boot.emc clk\n",
					__func__);
				return;
			}
			clk_prepare_enable(boot_emc_clk);
			clk_set_rate(boot_emc_clk, MOLLY_BOOT_EMC_RATE);
			pr_info("molly: enable emc boost for booting (%d Hz)\n",
				MOLLY_BOOT_EMC_RATE);
		}
	} else {
		if (!IS_ERR_OR_NULL(boot_emc_clk)) {
			clk_set_rate(boot_emc_clk, 0);
			clk_disable_unprepare(boot_emc_clk);
			clk_put(boot_emc_clk);
			boot_emc_clk = NULL;
			pr_info("molly: disable emc boost for booting\n");
		}
	}
}

static void molly_power_off(void)
{
	pr_emerg("power off requested, rebooting into bootloader to "
			"complete shutdown\n");
	machine_restart("shutdown");
}

static void __init tegra_molly_early_init(void)
{
    molly_init_hw_rev();
	tegra_clk_init_from_table(molly_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("molly");
}

static void __init tegra_molly_late_init(void)
{
    //platform_device_register(&tegra114_pinctrl_device);
    molly_pinmux_init();
	molly_i2c_init();
	molly_spi_init();
	molly_radio_init();
	molly_wake_sources_init();
	molly_usb_init();
	/* molly_xusb_init(); temporarily disable while fixing */
	molly_hsic_init();
	molly_uart_init();
	platform_add_devices(molly_devices, ARRAY_SIZE(molly_devices));
	tegra_io_dpd_init();
	molly_regulator_init();
	molly_sdhci_init();
	molly_suspend_init();
	molly_boost_emc_clk_for_boot(true);
	molly_emc_init();
	molly_edp_init();
	isomgr_init();
	molly_panel_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
    molly_sensors_init();
	molly_soctherm_init();

	pm_power_off = molly_power_off;
    molly_boost_emc_clk_for_boot(false);
}

#ifdef CONFIG_USE_OF
struct of_dev_auxdata molly_auxdata_lookup[] __initdata = {
    OF_DEV_AUXDATA("nvidia,tegra114-sdhci", 0x78000600, "sdhci-tegra.3", &tegra_sdhci_platform_data3),
    OF_DEV_AUXDATA("nvidia,tegra114-sdhci", 0x78000000, "sdhci-tegra.0", &tegra_sdhci_platform_data0),
	OF_DEV_AUXDATA("nvidia,tegra114-host1x", TEGRA_HOST1X_BASE, "host1x",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-gr3d", TEGRA_GR3D_BASE, "gr3d",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-gr2d", TEGRA_GR2D_BASE, "gr2d",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-msenc", TEGRA_MSENC_BASE, "msenc",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-vi", TEGRA_VI_BASE, "vi",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-isp", TEGRA_ISP_BASE, "isp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-tsec", TEGRA_TSEC_BASE, "tsec",
				NULL),
	T114_SPI_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra114-apbdma", 0x6000a000, "tegra-apbdma",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006000, "serial-tegra.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006300, "serial-tegra.3",
				NULL),
    T114_I2C_OF_DEV_AUXDATA,
	T114_SPI_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra114-xhci", 0x70090000, "tegra-xhci",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-nvavp", 0x60001000, "nvavp",
				NULL),
    OF_DEV_AUXDATA("nvidia,tegra114-pwm", 0x7000a000, "tegra-pwm", NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
				NULL),
	{}
};
#endif

static void __init tegra_molly_dt_init(void)
{
    tegra_get_board_info(&board_info);

    tegra_molly_early_init();

	of_platform_populate(NULL,
		of_default_bus_match_table, molly_auxdata_lookup,
		&platform_bus);

	tegra_molly_late_init();
}

static void __init tegra_molly_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 1920*1080*4*2 = 16588800 bytes (recovery uses
	 *                 double buffered miniui)
	 * 3840*2160*4 = 33177600 bytes (boot animation
	 *               uses single buffer, SurfaceFlinger
	 *               and hwcomposer will allocate its
	 *               own buffers
	 * Use the larger of the two to cover both cases.
	 */
	tegra_reserve(0, /* carveout */
		      PAGE_ALIGN(33177600), /* fb_size */
		      0); /* fb2_size: 0, not used */
#else
	tegra_reserve(SZ_128M,  /* carveout */
		      PAGE_ALIGN(33177600), /* fb_size */
		      0); /* fb2_size: 0, not used */
#endif
	/* Allocate at a fixed place, 1GB into RAM
	 * so it doesn't move when other blocks near
	 * end of RAM change size.  This makes it easier
	 * for the bootloader to keep in sync with it for
	 * implementing "fastboot oem kmsg".
	 */
}

static const char * const molly_dt_board_compat[] = {
	"google,molly",
	NULL
};

MACHINE_START(MOLLY, "molly")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_molly_reserve,
	.init_early	= tegra11x_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_molly_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= molly_dt_board_compat,
	.init_late	= tegra_init_late,
MACHINE_END
