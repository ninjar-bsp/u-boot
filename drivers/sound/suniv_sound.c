// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 Google, LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#define LOG_CATEGORY UCLASS_SOUND

#include <common.h>
#include <audio_codec.h>
#include <clk.h>
#include <dm.h>
#include <i2s.h>
#include <log.h>
#include <misc.h>
#include <sound.h>
#include <dm/pinctrl.h>

static int suniv_sound_play(struct udevice *dev, void *data, uint data_size)
{
	struct sound_uc_priv *uc_priv = dev_get_uclass_priv(dev);

	return i2s_tx_data(uc_priv->i2s, data, data_size);
}

static int suniv_sound_probe(struct udevice *dev)
{
	// struct sound_uc_priv *uc_priv = dev_get_uclass_priv(dev);
	// struct ofnode_phandle_args args;
	// struct udevice *pinctrl;
	// struct clk clk;
	// ofnode node;
	// int ret;
    printk("%s\n", __func__);
	return 0;
}

static const struct sound_ops suniv_sound_ops = {
	.play	= suniv_sound_play,
};

static const struct udevice_id suniv_sound_ids[] = {
	{ .compatible = "allwinner,suniv-f1c100s-sound" },
	{ }
};

U_BOOT_DRIVER(suniv_sound) = {
	.name		= "suniv_sound",
	.id		= UCLASS_SOUND,
	.of_match	= suniv_sound_ids,
	.probe		= suniv_sound_probe,
	.ops		= &suniv_sound_ops,
};
