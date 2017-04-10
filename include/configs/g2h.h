/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the G2H.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H


/* Set Boot Delay to Zero*/
#define CONFIG_BOOTDELAY    0
/* Stops autoboot process even when bootdelay is set to 0 */
#define CONFIG_ZERO_BOOTDELAY_CHECK

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <linux/sizes.h>

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define MACH_TYPE_HAWTHORNE		4901
#define CONFIG_MACH_TYPE		MACH_TYPE_HAWTHORNE

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

#define CONFIG_CMD_BMODE

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)
#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE		0x17800000

/* MMC Configuration */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* Ethernet Configuration */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

/* SPI NOR */
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		20000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1
#define CONFIG_SYS_I2C_SPEED		  100000

#define CONFIG_CONSOLE_DEV		"ttymxc0"
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"

/* Framebuffer */
#define CONFIG_VIDEO_SHUTDOWN_LCD
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 198000000
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

/* NAND flash command */
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND support */
#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8

/* kernel device tree options */
#if defined(CONFIG_SOLO_3)
#define CONFIG_DT "imx6dl-g2h-3.dtb"
#elif defined(CONFIG_SOLO_3f)
#define CONFIG_DT "imx6dl-g2h-3f.dtb"
#elif defined(CONFIG_SOLO_4)
#define CONFIG_DT "imx6dl-g2h-4.dtb"
#elif defined(CONFIG_SOLO_4f)
#define CONFIG_DT "imx6dl-g2h-4f.dtb"
#elif defined(CONFIG_SOLO_13)
#define CONFIG_DT "imx6dl-g2h-13.dtb"
#elif defined(CONFIG_SOLO_13f)
#define CONFIG_DT "imx6dl-g2h-13f.dtb"
#elif defined(CONFIG_SOLO_14)
#define CONFIG_DT "imx6dl-g2h-14.dtb"
#elif defined(CONFIG_SOLO_14f)
#define CONFIG_DT "imx6dl-g2h-14f.dtb"
#elif defined(CONFIG_SOLO_11f)
#define CONFIG_DT "imx6dl-g2h-11f.dtb"
#elif defined(CONFIG_SOLO_12f)
#define CONFIG_DT "imx6dl-g2h-12f.dtb"
#elif defined(CONFIG_SOLO_6)
#define CONFIG_DT "imx6dl-g2h-6.dtb"
#else
#define CONFIG_DT ""
#endif

/* panel and touch options */
#define EVERVISION_5_7    "test $touch_rev = EVERVISION && test $board_rev = LCD_5_7"
#define RESISTIVE_5_7     "test $touch_rev = RESISTIVE && test $board_rev = LCD_5_7"

#define EVERVISION_7    "test $touch_rev = EVERVISION && test $board_rev = LCD_7"
#define RESISTIVE_7     "test $touch_rev = RESISTIVE && test $board_rev = LCD_7"

#define EVERVISION_10_1 "test $touch_rev = EVERVISION && test $board_rev = LCD_10_1"
#define RESISTIVE_10_1  "test $touch_rev = RESISTIVE && test $board_rev = LCD_10_1"

#define RESISTIVE_10_4  "test $touch_rev = RESISTIVE && test $board_rev = LCD_10_4"

#define BASE_IO "test $base_io = true"

#define DIP_0 "test $board_dip = 0"
#define DIP_1 "test $board_dip = 1"
#define DIP_2 "test $board_dip = 2"
#define DIP_3 "test $board_dip = 3"
#define DIP_4 "test $board_dip = 4"
#define DIP_5 "test $board_dip = 5"
#define DIP_6 "test $board_dip = 6"
#define DIP_7 "test $board_dip = 7"

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"image=zImage\0" \
	"fdt_addr_r=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"splashpos=m,m\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw rootfstype=ext4\0" \
    "fdtfile=" CONFIG_DT "\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} quiet " \
		"root=${mmcroot} consoleblank=0 vt.global_cursor_default=0\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr_r} ${fdtfile}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr_r}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} quiet " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr_r} ${fdtfile}; then " \
				"bootz ${loadaddr} - ${fdt_addr_r}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
    "nandargs=setenv bootargs console=${console},${baudrate} quiet " \
        "${nandroot} consoleblank=0 vt.global_cursor_default=0 \0" \
    "nandroot=ubi.mtd=2,2048 root=ubi0:rootfs0 rootfstype=ubifs \0" \
    "nandboot=echo Booting from nand ...; " \
        "run nandargs; " \
        "nand read ${fdt_addr_r} 0x0000000 0x0080000; " \
        "nand read ${loadaddr} 0x0080000 0x0A00000; " \
        "bootz ${loadaddr} - ${fdt_addr_r}; \0" \

#ifdef CONFIG_BOOT_FROM_SPI
#define CONFIG_BOOTCOMMAND \
	"setenv panel ${board_rev}; "\
	"run nandboot; "
#else
#define CONFIG_BOOTCOMMAND \
	"setenv panel ${board_rev}; "\
	"mmc dev ${mmcdev};" \
	"mmc rescan; " \
	"run loadimage; " \
	"run mmcboot; "
#endif
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
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
#define CONFIG_ENV_OFFSET		(768 * 1024)
#endif

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#endif			       /* __CONFIG_H * */
