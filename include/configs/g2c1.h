/*
 * Copyright (C) 2015 Graeme Russ <gruss@tss-engineering.com>
 * on behalf of DENX Software Engineering GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIGS_G2C1_H__
#define __CONFIGS_G2C1_H__

#ifdef DEBUG
#define CONFIG_SPL_SERIAL_SUPPORT
#endif

/* System configurations */
#define CONFIG_MX28			/* i.MX28 SoC */
#define CONFIG_SYS_MXS_VDD5V_ONLY	/* No Battery */

/* TODO: Remove all references to MACH_TYPE
#define MACH_TYPE_G2C1	3613
#define CONFIG_MACH_TYPE	MACH_TYPE_G2C1
*/

#define CONFIG_FIT

#define CONFIG_TIMESTAMP		/* Print image info with timestamp */

/* U-Boot Commands */
#define CONFIG_SYS_NO_FLASH
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DOS_PARTITION
#define CONFIG_FAT_WRITE

#define CONFIG_CMD_ASKENV
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_BMP
#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DATE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_EEPROM
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FAT
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_CMD_FUSE
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_GREPENV
#define CONFIG_CMD_I2C
#define CONFIG_CMD_MII
#define CONFIG_CMD_MMC
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS
#define CONFIG_CMD_PING
#define CONFIG_CMD_SF
#define CONFIG_CMD_SPI
#define CONFIG_CMD_USB
#define	CONFIG_VIDEO

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x08000000	/* Max 128 MB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Environment */
#define CONFIG_ENV_SIZE			(16 * 1024)

#if defined(CONFIG_ENV_IS_IN_NAND) && defined(CONFIG_ENV_IS_IN_MMC)
#error "CONFIG_ENV_IS_IN_NAND and CONFIG_ENV_IS_IN_MMC cannot both be defined"
#elif defined(CONFIG_ENV_IS_IN_NAND)
/* Environment is in NAND - CONFIG_CMD_NAND must be defined*/
#if !defined(CONFIG_CMD_NAND)
#define CONFIG_CMD_NAND
#endif
#define CONFIG_ENV_SIZE_REDUND		CONFIG_ENV_SIZE
#define CONFIG_ENV_SECT_SIZE		(128 * 1024)
#define CONFIG_ENV_RANGE		(4 * CONFIG_ENV_SECT_SIZE)
#define CONFIG_ENV_OFFSET		(24 * CONFIG_ENV_SECT_SIZE) /* 3 MiB */
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + CONFIG_ENV_RANGE)
#elif defined(CONFIG_ENV_IS_IN_MMC)
/* Environment is in NAND - CONFIG_CMD_MMC must be defined*/
#if !defined(CONFIG_CMD_MMC)
#define CONFIG_CMD_MMC
#endif
#define CONFIG_ENV_OFFSET		(-CONFIG_ENV_SIZE)
#define CONFIG_SYS_MMC_ENV_DEV		0
#else
#error "Either CONFIG_ENV_IS_IN_NAND or CONFIG_ENV_IS_IN_MMC must be defined"
#endif

#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_MTDPARTS
#define CONFIG_RBTREE
#define CONFIG_LZO
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define MTDIDS_DEFAULT			"nand0=gpmi-nand"
#define MTDPARTS_DEFAULT			\
	"mtdparts=gpmi-nand:"			\
		"3m(u-boot),"			\
		"512k(env1),"			\
		"512k(env2),"			\
		"14m(boot),"			\
		"238m(data),"			\
		"-@4096k(UBI)"

/* FEC Ethernet on SoC */
#ifdef CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MX28_FEC_MAC_IN_OCOTP
#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC
#endif

/* EEPROM */
#ifdef CONFIG_CMD_EEPROM
#define CONFIG_SYS_I2C_MULTI_EEPROMS
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	2
#endif

/* RTC */
#ifdef CONFIG_CMD_DATE
#define CONFIG_RTC_MXS
#endif

/* USB */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_MXS_PORT0
#define CONFIG_EHCI_MXS_PORT1
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_USB_STORAGE
#endif

/* SPI */
#ifdef CONFIG_CMD_SPI
#define CONFIG_DEFAULT_SPI_BUS		2
#define CONFIG_DEFAULT_SPI_CS		0
#define CONFIG_DEFAULT_SPI_MODE		SPI_MODE_0

/* SPI FLASH */
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SF_DEFAULT_BUS		2
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		40000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0

#define CONFIG_ENV_SPI_BUS		2
#define CONFIG_ENV_SPI_CS		0
#define CONFIG_ENV_SPI_MAX_HZ		40000000
#define CONFIG_ENV_SPI_MODE		SPI_MODE_0
#endif

#endif

/* LCD */
#ifdef CONFIG_VIDEO
#define	CONFIG_VIDEO_LOGO
#define	CONFIG_SPLASH_SCREEN
#define	CONFIG_CMD_BMP
#define	CONFIG_BMP_16BPP
#define	CONFIG_VIDEO_BMP_RLE8
#define	CONFIG_VIDEO_BMP_GZIP
#define	CONFIG_SYS_VIDEO_LOGO_MAX_SIZE	(2 << 20)
#endif

/* Boot Linux */
#define CONFIG_BOOTDELAY	1
#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR

/* Extra Environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"ubifs_file=filesystem.ubifs\0" \
	"update_nand_full_filename=u-boot.nand\0" \
	"update_nand_firmware_filename=u-boot.sb\0"	\
	"update_nand_firmware_maxsz=0x100000\0"	\
	"update_nand_stride=0x40\0"	/* MX28 datasheet ch. 12.12 */ \
	"update_nand_count=0x4\0"	/* MX28 datasheet ch. 12.12 */ \
	"update_nand_get_fcb_size="	/* Get size of FCB blocks */ \
		"nand device 0 ; " \
		"nand info ; " \
		"setexpr fcb_sz ${update_nand_stride} * ${update_nand_count};" \
		"setexpr update_nand_fcb ${fcb_sz} * ${nand_writesize}\0" \
	"update_nand_firmware_full=" /* Update FCB, DBBT and FW */ \
		"if tftp ${update_nand_full_filename} ; then " \
		"run update_nand_get_fcb_size ; " \
		"nand scrub -y 0x0 ${filesize} ; " \
		"nand write.raw ${loadaddr} 0x0 ${fcb_sz} ; " \
		"setexpr update_off ${loadaddr} + ${update_nand_fcb} ; " \
		"setexpr update_sz ${filesize} - ${update_nand_fcb} ; " \
		"nand write ${update_off} ${update_nand_fcb} ${update_sz} ; " \
		"fi\0" \
	"update_nand_firmware="		/* Update only firmware */ \
		"if tftp ${update_nand_firmware_filename} ; then " \
		"run update_nand_get_fcb_size ; " \
		"setexpr fcb_sz ${update_nand_fcb} * 2 ; " /* FCB + DBBT */ \
		"setexpr fw_sz ${update_nand_firmware_maxsz} * 2 ; " \
		"setexpr fw_off ${fcb_sz} + ${update_nand_firmware_maxsz};" \
		"nand erase ${fcb_sz} ${fw_sz} ; " \
		"nand write ${loadaddr} ${fcb_sz} ${filesize} ; " \
		"nand write ${loadaddr} ${fw_off} ${filesize} ; " \
		"fi\0" \
	"update_nand_kernel="		/* Update kernel */ \
		"mtdparts default; " \
		"nand erase.part kernel; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"nand write ${loadaddr} kernel ${filesize}\0" \
	"update_nand_fdt="		/* Update fdt */ \
		"mtdparts default; " \
		"nand erase.part fdt; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${fdt_file}; " \
		"nand write ${loadaddr} fdt ${filesize}\0" \
	"update_nand_filesystem="		/* Update filesystem */ \
		"mtdparts default; " \
		"nand erase.part filesystem; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${ubifs_file}; " \
		"ubi part filesystem; " \
		"ubi create filesystem; " \
		"ubi write ${loadaddr} filesystem ${filesize}\0" \
	"nandargs=setenv bootargs console=${console_mainline},${baudrate} " \
		"rootfstype=ubifs ubi.mtd=6 root=ubi0_0 ${mtdparts}\0" \
	"nandboot="		/* Boot from NAND */ \
		"mtdparts default; " \
		"run nandargs; " \
		"nand read ${loadaddr} kernel 0x00400000; " \
		"if test ${boot_fdt} = yes; then " \
			"nand read ${fdt_addr} fdt 0x00080000; " \
			"bootz ${loadaddr} - ${fdt_addr}; " \
		"else " \
			"if test ${boot_fdt} = no; then " \
				"bootz; " \
			"else " \
				"echo \"ERROR: Set boot_fdt to yes or no.\"; " \
			"fi; " \
		"fi\0" \
	"update_sd_firmware_filename=u-boot.sd\0" \
	"update_sd_firmware="		/* Update the SD firmware partition */ \
		"if mmc rescan ; then "	\
		"if tftp ${update_sd_firmware_filename} ; then " \
		"setexpr fw_sz ${filesize} / 0x200 ; "	/* SD block size */ \
		"setexpr fw_sz ${fw_sz} + 1 ; "	\
		"mmc write ${loadaddr} 0x800 ${fw_sz} ; " \
		"fi ; "	\
		"fi\0" \
	"script=boot.scr\0"	\
	"image=zImage\0" \
	"console_mainline=ttyAMA0\0" \
	"fdt_file=imx28-g2c1.dtb\0" \
	"fdt_addr=0x41000000\0" \
	"boot_fdt=yes\0" \
	"ip_dyn=yes\0" \
	"mmcdev=0\0" \
	"mmcpart=2\0" \
	"mmcroot=/dev/mmcblk0p3 rw rootwait\0" \
	"mmcargs=setenv bootargs console=${console_mainline},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadbootscript="  \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; "	\
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
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
	"netargs=setenv bootargs console=${console_mainline},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; "	\
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi;" \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev}; if mmc rescan; then " \
		"if run loadbootscript; then " \
			"run bootscript; " \
		"else " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else run netboot; fi"

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_G2C1_H__ */
