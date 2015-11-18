/*
 * Reachtech G2C1 board
 *
 * Copyright (C) 2015 Graeme Russ <gruss@tss-engineering.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * Author: Graeme Russ <gruss@tss-engineering.com>
 *
 * Based on m28evk.c:
 * Copyright (C) 2011 Marek Vasut <marek.vasut@gmail.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/mii.h>
#include <miiphy.h>
#include <netdev.h>
#include <errno.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Functions
 */
int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);
	/* IO1 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK1, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
	/* SSP2 clock at 160MHz */
	mxs_set_sspclk(MXC_SSPCLK2, 160000, 0);

	/* Turn the Run LED On */
	gpio_direction_output(MX28_PAD_SPDIF__GPIO_3_27, 0);

	/* Turn off software write protect of NAND */
	gpio_direction_output(MX28_PAD_GPMI_RESETN__GPIO_0_28, 1);

	/* Power on LCD */
	gpio_direction_output(MX28_PAD_LCD_RESET__GPIO_3_30, 1);

	/* Enable the LCD display */
	gpio_direction_output(MX28_PAD_GPMI_CE3N__GPIO_0_19, 1);

	/* Turn on the LCD backlight */
	gpio_direction_output(MX28_PAD_AUART0_CTS__GPIO_3_2, 1);

	/* TODO: Set the LCD backlight to maximum brightness */

	/* Disbale power to the USB OTG Port */
	gpio_direction_output(MX28_PAD_SSP0_DATA4__GPIO_2_4, 0);

	/* Turn on DEBUG port CTS */
	gpio_direction_output(MX28_PAD_SSP0_DATA5__GPIO_2_5, 1);

	/* Configure the Ethernet Interrupt Input */
	gpio_direction_input(MX28_PAD_SSP0_DATA6__GPIO_2_6);

	/* Configure the 8-bit GPIO Interrupt Input */
	gpio_direction_input(MX28_PAD_GPMI_CE2N__GPIO_0_18);


	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

#ifdef CONFIG_CMD_MMC

int board_mmc_init(bd_t *bis)
{
	return mxsmmc_initialize(bis, 0, NULL, NULL);
}
#endif

#ifdef	CONFIG_CMD_NET

#define	MII_OPMODE_STRAP_OVERRIDE	0x16
#define	MII_PHY_CTRL1			0x1e
#define	MII_PHY_CTRL2			0x1f

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	int ret;

	ret = cpu_eth_init(bis);
	if (ret)
		return ret;

	clrsetbits_le32(&clkctrl_regs->hw_clkctrl_enet,
			CLKCTRL_ENET_TIME_SEL_MASK,
			CLKCTRL_ENET_TIME_SEL_RMII_CLK |
			CLKCTRL_ENET_CLK_OUT_EN);

	/* Reset the PHY */
	gpio_direction_output(MX28_PAD_SAIF0_MCLK__GPIO_3_20, 0);
	mdelay(10);
	gpio_set_value(MX28_PAD_SAIF0_MCLK__GPIO_3_20, 1);
	mdelay(10);

	ret = fecmxc_initialize_multi(bis, 0, 0, MXS_ENET0_BASE);
	if (ret) {
		printf("FEC MXS: Unable to init FEC0\n");
		return ret;
	}

	dev = eth_get_dev_by_name("FEC0");
	if (!dev) {
		printf("FEC MXS: Unable to get FEC0 device entry\n");
		return -EINVAL;
	}

	return ret;
}

#endif
