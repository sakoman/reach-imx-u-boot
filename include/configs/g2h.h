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
#define CONFIG_MMCROOT			"/dev/mmcblk2p1"

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

#ifdef CONFIG_BOOT_FROM_SPI
#define CONFIG_BOOT_DEV "nandboot"
#define CONFIG_RESCUE_BOOT "rescue_nor_boot"
#else
#define CONFIG_BOOT_DEV "mmcboot"
#define CONFIG_RESCUE_BOOT "rescue_sd_boot"
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"image=boot/zImage\0" \
	"splash=/boot/splash.bmp\0" \
	"splashpos=m,m\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rw rootfstype=ext4\0" \
    "fdt_image=boot/" CONFIG_DT "\0" \
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
	"loadimage=load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_image}\0" \
	"mmcboot=echo Booting from mmc ...; mmc dev ${mmcdev}; mmc rescan; " \
        "run loadimage; run loadfdt; run mmcargs; " \
            "bootz ${loadaddr} - ${fdt_addr}; \0" \
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
			"if ${get_cmd} ${fdt_addr} ${fdtfile}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
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
    "check_update=if test ${swupdate} = true; then run " CONFIG_RESCUE_BOOT "; " \
        "else run " CONFIG_BOOT_DEV "; fi; \0" \
    "nandargs=setenv bootargs console=${console},${baudrate} quiet " \
        "${nandroot} consoleblank=0 vt.global_cursor_default=0 \0" \
    "nandroot=ubi.mtd=2,2048 root=ubi0:rootfs0 rootfstype=ubifs \0" \
    "nandboot=echo Booting from nand ...; " \
        "run nandargs; " \
        "nand read ${fdt_addr} 0x0000000 0x0080000; " \
        "nand read ${loadaddr} 0x0080000 0x0A00000; " \
        "bootz ${loadaddr} - ${fdt_addr}; \0" \
	"swupdate=false \0" \
	"rescue_addr=0x19000000 \0" \
    "rescue_image=rescue-image.ext3.gz \0" \
	"load_sd_rescue_image=fatload mmc ${mmcdev}:${mmcpart} ${rescue_addr} ${rescue_image}\0" \
    "rescue_root=/dev/ram0 rw initrd=0x19000000,5M \0" \
    "rescue_args=setenv bootargs console=${console},${baudrate} quiet rdinit=/sbin/init " \
        "root=${rescue_root} consoleblank=0 vt.global_cursor_default=0 \0" \
    "rescue_sd_boot=echo Rescue boot ...; run rescue_args; run load_sd_rescue_image; " \
        "run loadimage; run loadfdt; bootz ${loadaddr} - ${fdt_addr}; \0" \

#define CONFIG_BOOTCOMMAND \
    "setenv panel ${board_rev}; " \
    "load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${splash}; " \
    "bmp display ${loadaddr}; " \
    "run check_update; "

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
#define CONFIG_ENV_OFFSET		(768 * 1024)
#endif

#endif			       /* __CONFIG_H * */
