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

#define CONFIG_BOOTCOMMAND "bootz 0x80008000 0x80D00000 0x80C00000"

#endif /* __CONFIG_H */
