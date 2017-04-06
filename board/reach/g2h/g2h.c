/*
 * Copyright (C) 2013 Reach Technology.
 *
 * Author: Jeff Hormn <jhorn@reachtech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <malloc.h>
#include <mmc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>

/* Special MXCFB sync flags are here. */
#include "../drivers/video/mxcfb.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define USDHC3_CD_GPIO		IMX_GPIO_NR(1, 2)
#define USDHC2_CD_GPIO		IMX_GPIO_NR(1, 4)
#define ETH_PHY_RESET		IMX_GPIO_NR(1, 25)
#define BUZZER_VOLUME		IMX_GPIO_NR(1, 19)

#define DIP_PIN1		IMX_GPIO_NR(1, 20)
#define DIP_PIN2		IMX_GPIO_NR(1, 17)
#define DIP_PIN3		IMX_GPIO_NR(1, 19)
#define DIP_PIN4		IMX_GPIO_NR(1, 21)

#define LCD_PAR_ENABLE		IMX_GPIO_NR(2, 11)
#define BACKLIGHT_PAR_ENABLE	IMX_GPIO_NR(2, 8)

#define LVDS0_EN		IMX_GPIO_NR(2, 8)		
#define DISP0_VDDEN		IMX_GPIO_NR(2, 11)
#define BACKLIGHT_PWM		IMX_GPIO_NR(2, 9)

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

static iomux_v3_cfg_t const dipswitch_pads[] = {
	MX6_PAD_SD1_CLK__GPIO1_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD1_DAT1__GPIO1_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD1_DAT2__GPIO1_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* Carrier MicroSD Card Detect */
	MX6_PAD_GPIO_4__GPIO1_IO04      | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* SOM MicroSD Card Detect */
	MX6_PAD_GPIO_2__GPIO1_IO02      | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* AR8031 PHY Reset */
	MX6_PAD_ENET_CRS_DV__GPIO1_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const ecspi1_pads[] = {
	/* GPIO CS */
	MX6_PAD_EIM_D24__GPIO3_IO24  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static iomux_v3_cfg_t const display_enable_pads[] = {
	MX6_PAD_SD4_DAT0__GPIO2_IO08  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_DAT3__GPIO2_IO11  | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_DAT1__GPIO2_IO09  | MUX_PAD_CTRL(NO_PAD_CTRL),  
};

static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 26)
	},
};

static void setup_iomux_dipswitch(void)
{
	imx_iomux_v3_setup_multiple_pads(dipswitch_pads,
					 ARRAY_SIZE(dipswitch_pads));
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset AR8031 PHY */
	gpio_direction_output(ETH_PHY_RESET, 0);
	/* the datasheet states a 10ms delay */
	udelay(1000 * 10);
	gpio_set_value(ETH_PHY_RESET, 1);
}

static iomux_v3_cfg_t const pwm1_pads[] = {
	/* Buzzer Volume */
	MX6_PAD_SD1_DAT2__GPIO1_IO19	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_pwm(void)
{
	imx_iomux_v3_setup_multiple_pads(pwm1_pads, ARRAY_SIZE(pwm1_pads));
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
	{USDHC2_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;

	/*
	 * Following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SOM MicroSD
	 * mmc1                    Carrier board MicroSD
	 */
	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			usdhc_cfg[0].max_bus_width = 4;
			gpio_direction_input(USDHC3_CD_GPIO);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			usdhc_cfg[1].max_bus_width = 4;
			gpio_direction_input(USDHC2_CD_GPIO);
			break;
		default:
			printf("Invalid SDHC port");
			return -EINVAL;
		}

		status = fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
		if (status)
			return status;
	}

	return 0;
}

void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
}

static int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

/* On G2H Ethernet PHY can be located at address 0x1 */
#define ETH_PHY_MASK	(1 << 0x1)

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;

	setup_iomux_enet();

	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* our phy is at addr 1, just look there */
	phydev = phy_find_by_mask(bus, ETH_PHY_MASK, PHY_INTERFACE_MODE_RGMII);
	if (!phydev) {
		ret = -EINVAL;
		goto free_bus;
	}

	debug("using phy at address %d\n", phydev->addr);
	ret = fec_probe(bis, -1, IMX_FEC_BASE, bus, phydev);
	if (ret)
		goto free_phydev;

	return 0;

free_phydev:
	free(phydev);
free_bus:
	free(bus);
	return ret;
}

#if defined(CONFIG_VIDEO_IPUV3)
static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00,
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01,
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02,
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03,
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04,
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05,
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06,
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07,
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08,
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09,
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10,
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11,
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12,
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13,
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14,
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15,
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16,
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17,
};

static int detect_7_panel(struct display_info_t const *dev)
{
	if (IS_ENABLED(CONFIG_LCD_7))
		return 1;
	else
		return 0;
}

static int detect_5_7_panel(struct display_info_t const *dev)
{
	if (IS_ENABLED(CONFIG_LCD_5_7))
		return 1;
	else
		return 0;
}

static int detect_10_1_panel(struct display_info_t const *dev)
{
	if (IS_ENABLED(CONFIG_LCD_10_1))
		return 1;
	else
		return 0;
}

static int detect_10_4_panel(struct display_info_t const *dev)
{
	if (IS_ENABLED(CONFIG_LCD_10_4))
		return 1;
	else
		return 0;
}

static void enable_rgb(struct display_info_t const *dev)
{	
	imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));
}

struct display_info_t const displays[] = {{
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_7_panel,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "LCD_7",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 35714,
		.left_margin    = 100,
		.right_margin   = 100,
		.upper_margin   = 56,
		.lower_margin   = 15,
		.hsync_len      = 5,
		.vsync_len      = 10,
		.sync           = FB_SYNC_CLK_LAT_FALL,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_5_7_panel,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "LCD_5_7",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = KHZ2PICOS(25175),
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 31,
		.lower_margin   = 11,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = FB_SYNC_CLK_LAT_FALL,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_10_1_panel,
	.enable	= NULL,
	.mode	= {
		.name           = "LCD_10_1",
		.refresh        = 60,
		.xres           = 1280,
		.yres           = 800,
		.pixclock       = KHZ2PICOS(65000),
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= detect_10_4_panel,
	.enable	= NULL,
	.mode	= {
		.name           = "LCD_10_4",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = KHZ2PICOS(60000),
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	imx_iomux_v3_setup_multiple_pads(display_enable_pads,
					 ARRAY_SIZE(display_enable_pads));

	enable_ipu_clock();
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT
	     | IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
	       | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
		  << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif

#ifdef CONFIG_NAND_MXS
static iomux_v3_cfg_t gpmi_pads[] = {
	MX6_PAD_NANDF_CLE__NAND_CLE	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_ALE__NAND_ALE	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_WP_B__NAND_WP_B	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_RB0__NAND_READY_B	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_CS0__NAND_CE0_B	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_CS1__NAND_CE1_B	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_CMD__NAND_RE_B	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_CLK__NAND_WE_B	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D0__NAND_DATA00	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D1__NAND_DATA01	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D2__NAND_DATA02	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D3__NAND_DATA03	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D4__NAND_DATA04	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D5__NAND_DATA05	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D6__NAND_DATA06	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_NANDF_D7__NAND_DATA07	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* config gpmi nand iomux */
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

	/* gate ENFC_CLK_ROOT clock first,before clk source switch */
	clrbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

	/* config gpmi and bch clock to 100 MHz */
	clrsetbits_le32(&mxc_ccm->cs2cdr,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED_MASK |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL_MASK,
			MXC_CCM_CS2CDR_ENFC_CLK_PODF(0) |
			MXC_CCM_CS2CDR_ENFC_CLK_PRED(3) |
			MXC_CCM_CS2CDR_ENFC_CLK_SEL(3));

	/* enable ENFC_CLK_ROOT clock */
	setbits_le32(&mxc_ccm->CCGR2, MXC_CCM_CCGR2_IOMUX_IPT_CLK_IO_MASK);

	/* enable gpmi and bch clock gating */
	setbits_le32(&mxc_ccm->CCGR4,
		     MXC_CCM_CCGR4_RAWNAND_U_BCH_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_BCH_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_MASK |
		     MXC_CCM_CCGR4_RAWNAND_U_GPMI_INPUT_APB_MASK |
		     MXC_CCM_CCGR4_PL301_MX6QPER1_BCH_OFFSET);

	/* enable apbh clock gating */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#endif

int board_early_init_f(void)
{
	gpio_direction_output(BACKLIGHT_PWM, 0);
	gpio_direction_output(DISP0_VDDEN, 0);
	gpio_direction_output(LVDS0_EN, 0);

	setup_iomux_uart();

#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
#ifdef CONFIG_NAND_MXS
	setup_gpmi_nand();
#endif
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

static char board_dipswitch(void)
{
	char val = 0;

	/* read the pin and toggle, the reads are flipped , investigate */
	val |= ((gpio_get_value(DIP_PIN1)) ^ (1 << 0)) << 0;
	val |= ((gpio_get_value(DIP_PIN2)) ^ (1 << 0)) << 1;
	val |= ((gpio_get_value(DIP_PIN3)) ^ (1 << 0)) << 2;
	val |= ((gpio_get_value(DIP_PIN4)) ^ (1 << 0)) << 3;

	return val;
}

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(3, 24)) : -1;
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{ "mmc0",	  MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{ "mmc1",	  MAKE_CFGVAL(0x40, 0x20, 0x00, 0x00)},
	{ "spi1",	  MAKE_CFGVAL(0x30, 0x20, 0x00, 0x28)},
	{ NULL,	 0},
};
#endif

bool has_evervision(void)
{
	if (!i2c_probe(0x38))
		return true;

	return false;
}

bool has_base_io(void)
{
    char val = 0;

    /* 
     * on the base IO board only PIN3 and PIN4 are available; we are 
     * using PIN4 to determine Base IO from Full IO
     */
    val |= ((gpio_get_value(DIP_PIN4)) ^ (1 << 0)) << 3;

    if (val == 0x8) {
		setenv("board_dip", simple_itoa(val));

        return true;
    }

    /* if PIN4 is low this is a full stuff so read the other pins */
    val |= ((gpio_get_value(DIP_PIN1)) ^ (1 << 0)) << 0;
    val |= ((gpio_get_value(DIP_PIN2)) ^ (1 << 0)) << 1;
    val |= ((gpio_get_value(DIP_PIN3)) ^ (1 << 0)) << 2;

	setenv("board_dip", simple_itoa(val));

    return false;
}

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (has_evervision()) {
		setenv("touch_rev", "EVERVISION");
    } else {
		setenv("touch_rev", "RESISTIVE");
    }

    if (has_base_io()) {
		setenv("base_io", "true");
    } else {
		setenv("base_io", "false");
    }
#ifdef CONFIG_LCD_5_7
	setenv("board_rev", "LCD_5_7");
#endif
#ifdef CONFIG_LCD_7
	setenv("board_rev", "LCD_7");
#endif
#ifdef CONFIG_LCD_10_1
	setenv("board_rev", "LCD_10_1");
#endif
#ifdef CONFIG_LCD_10_4
	setenv("board_rev", "LCD_10_4");
#endif

#endif
	gpio_set_value(BACKLIGHT_PWM, 1);
	gpio_set_value(DISP0_VDDEN, 1);
#ifdef CONFIG_LCD_5_7
	mdelay(400);
#elif CONFIG_LCD_10_1
	mdelay(250);
#else
	mdelay(150);
#endif
	gpio_set_value(LVDS0_EN, 1);

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_spi();
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_iomux_dipswitch();
	setup_iomux_pwm();

	/* Set buzzer volume to the maximum */
	gpio_set_value(BUZZER_VOLUME, 1);

	return 0;
}

int checkboard(void)
{
	puts("Board: G2H\n");

	return 0;
}

void shutdown_lcd(void)
{
	gpio_set_value(LVDS0_EN, 0);
	gpio_set_value(BACKLIGHT_PWM, 0);
	gpio_set_value(DISP0_VDDEN, 0);
}

static int do_dip(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	setenv_hex("dipswitch", board_dipswitch());

	printf("dipswitch 0x%X\n", board_dipswitch());

	return 0;
}

U_BOOT_CMD(
	dip, 1, 1, do_dip,
	"Reads DIP, sets 'dipswitch' environment variable",
	"Returns 0 (true) on read"
);
