/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Placeholder wrapper to allow addressing Allwinner F-series (suniv) CPU
 * based devices separately. Please do not add anything in here.
 */
#ifndef __CONFIG_H
#define __CONFIG_H

#include <configs/sunxi-common.h>

#ifdef CONFIG_BOOTCOMMAND
#undef CONFIG_BOOTCOMMAND
#endif

#define CONFIG_BOOTCOMMAND 	"mtd read spi-nand0 0x80C00000 0x100000 0x4000; " \
				"mtd read spi-nand0 0x80008000 0x110000 0x400000; " \
				"bootz 0x80008000 - 0x80C00000"

#endif /* __CONFIG_H */
