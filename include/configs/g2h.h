/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the G2H.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/mach-imx/gpio.h>
#include <linux/sizes.h>

#define MACH_TYPE_HAWTHORNE		4901
#define CONFIG_MACH_TYPE		MACH_TYPE_HAWTHORNE

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART_BASE		UART1_BASE

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			0x12000000

/* MMC Configuration */
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* Ethernet Configuration */
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1

/* SPI NOR */
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		20000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_SPEED		  100000

#define CONFIG_CONSOLE_DEV		"ttymxc0"

/* Framebuffer */
#define CONFIG_VIDEO_SHUTDOWN_LCD
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_IPUV3_CLK 198000000
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* NAND support */
#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* kernel device tree options */
#if defined(CONFIG_RD6_7P)
#define CONFIG_DT "imx6dl-g2h-7p.dtb"
#elif defined(CONFIG_RD6_7R)
#define CONFIG_DT "imx6dl-g2h-7r.dtb"
#elif defined(CONFIG_RD6_57R)
#define CONFIG_DT "imx6dl-g2h-57r.dtb"
#elif defined(CONFIG_RD6_57P)
#define CONFIG_DT "imx6dl-g2h-57p.dtb"
#elif defined(CONFIG_RD6_10P)
#define CONFIG_DT "imx6dl-g2h-10p.dtb"
#elif defined(CONFIG_RD6_10U)
#define CONFIG_DT "imx6dl-g2h-10u.dtb"
#else
#define CONFIG_DT ""
#endif

#define CONFIG_BOOT_DEV "mmcboot"

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_addr_r=0x18000000\0" \
	"kernel_addr_r=0x12000000\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"bootargs=rw rootfstype=ext4 console=${console},${baudrate} " \
		"consoleblank=0 vt.global_cursor_default=0\0" \
	"splash=boot/splash.bmp\0" \
	"splashpos=m,m\0" \
	"mender_pre_setup_commands=setenv panel ${board_rev}; " \
		"load mmc ${mender_uboot_dev}:${mender_boot_part} ${kernel_addr_r} ${splash}; " \
		"bmp display ${kernel_addr_r};\0 " \

#define CONFIG_BOOTCOMMAND	""

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			SZ_512M

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#if defined(CONFIG_BOOT_FROM_SPI)
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_OFFSET		(512 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#else
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(1024 * 1024)
#endif

#endif			       /* __CONFIG_H * */
