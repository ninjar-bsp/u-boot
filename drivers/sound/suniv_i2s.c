// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 embeddedboys, Ltd.
 */

#define LOG_CATEGORY UCLASS_I2S

#include <common.h>
#include <dm.h>
#include <i2s.h>
#include <clk.h>
#include <log.h>
#include <sound.h>
#include <reset.h>
#include <asm/io.h>
#include <linux/bitops.h>

// #include "piano.wav.h"
#include "test.wav.h"

#define PLL_AUDIO_CTRL_REG    (0x01c20000 + 0x008)
#define PLL_AUDIO_N_SHIFT   8
#define PLL_AUDIO_N_MASK    GENMASK(14, 8)
#define PLL_AUDIO_M_SHIFT   0
#define PLL_AUDIO_M_MASK    GENMASK(4, 0)
#define PLL_AUDIO_EN        BIT(31)

/* Suniv I2S Ctrl register bits */
#define I2S_CTRL_SDO_EN       BIT(8)
#define I2S_CTRL_ASS          BIT(6)
#define I2S_CTRL_PCM_SELECT   BIT(4)
#define I2S_CTRL_TX_EN        BIT(2)
#define I2S_CTRL_RX_EN        BIT(1)
#define I2S_CTRL_G_EN         BIT(0)

#define I2S_EN (I2S_CTRL_SDO_EN | I2S_CTRL_TX_EN | \
                I2S_CTRL_G_EN)

#define I2S_FIFO_DEPTH      128
#define I2S_TXFIFO_FLUSH    BIT(25)
#define I2S_RXFIFO_FLUSH    BIT(24)

/* Digital Audio Clock Divide Register */
#define I2S_MCLKO_EN        BIT(7)
#define I2S_BCLK_DIV_MASK   GENMASK(6, 4)
#define I2S_BCLK_DIV_SHIFT  4

#define I2S_MCLK_DIV_MASK   GENMASK(3, 0)
#define I2S_MCLK_DIV_SHIFT  0

#define suniv_i2s_reg_dump(__reg) \
    pr_debug(#__reg ": 0x%x\n", readl(__reg))

struct suniv_i2s_regs {
    u32     ctrl;       /* DA_CTRL_REG   0x00 */
    u32     fat0;       /* DA_FAT_REG0   0x04 */
    u32     fat1;       /* DA_FAT_REG1   0x08 */
    u32     txfifo;     /* DA_TXFIFO_REG 0x0C */
    u32     rxfifo;     /* DA_RXFIFO_REG 0x10 */
    u32     fctl;       /* DA_FCTL_REG   0x14 */
    u32     fsta;       /* DA_FSTA_REG   0x18 */
    u32     intc;       /* DA_INT_REG    0x1C */
    u32     ista;       /* DA_ISTA_REG   0x20 */
    u32     clkd;
    u32     txcnt;
    u32     rxcnt;
    u32     txchsel;
    u32     txchmap;
    u32     rxchsel;
    u32     rxchmap;
};

struct wavfile_header {
    char    riff_tag[4];
    int     riff_length;
    char    wave_tag[4];
    char    fmt_tag[4];
    int     fmt_length;
    short   audio_format;
    short   num_channels;
    int     sample_rate;
    int     byte_rate;
    short   block_align;
    short   bits_per_sample;
    char    data_tag[4];
    int     data_length;
};

/* Suniv Audio PLL clocks */
enum {
    SUNIV_AUDIO_PLL_24_571MHz,
    SUNIV_AUDIO_PLL_22_579MHz,
};

/* Suniv I2S Sample Rates */
enum {
    I2S_SR_16BIT,
    I2S_SR_20BIT,
    I2S_SR_24BIT,
    RESERVED,
};
#define I2S_SR_SHIFT    4

enum {
    I2S_WSS_16BCLK,
    I2S_WSS_20BCLK,
    I2S_WSS_24BCLK,
    I2S_WSS_32BCLK,
};
#define I2S_WSS_SHIFT   2

static int suniv_i2s_test(struct i2s_uc_priv *priv)
{
    struct suniv_i2s_regs *regs = (struct suniv_i2s_regs *)priv->base_address;
    /*
     * PD7  : MCLK
     * PD8  : BCLK
     * PD9  : LRCK
     * PD10 : SDI
     * PD11 : SDO0
     */
    
    int i;
    size_t to_send, tx_array_size = I2S_FIFO_DEPTH;
    size_t remain = sizeof(test_wav);
    u8 const *txbuf8 = test_wav;
    
    txbuf8 += 44;
    remain -= 44;
    
    setbits_le32(&regs->ctrl, I2S_EN);
    suniv_i2s_reg_dump(&regs->ctrl);
    
    pr_debug("%s start.\n", __func__);
    
    while (remain) {
        to_send = min(tx_array_size, remain);
        
        for (i = 0; i < to_send; i += 2)
            writew(txbuf8[i + 1] << 8 | txbuf8[i], &regs->txfifo);
        
        /* wait for fifo spaces */
        while (((readl(&regs->fsta) >> 16) & 0xff) < 0x3f);
        txbuf8 += to_send;
        remain -= to_send;
    }
    
    /* waiting for fifo empty */
    while (!(readl(&regs->ista) & BIT(4)));
    setbits_le16(&regs->ista, BIT(4));
    
    pr_debug("%s done.\n", __func__);
    clrbits_le32(&regs->ctrl, I2S_EN);
    return 0;
}

static int suniv_i2s_clock_setup(struct udevice *dev)
{
    int ret;
    struct clk clk_bus, clk_mod;
    
    ret = clk_get_by_index(dev, 0, &clk_bus);
    if (ret) {
        pr_debug("%s: Cannot find get BUS clk\n", __func__);
        return ret;
    }
    ret = clk_prepare_enable(&clk_bus);
    if (ret) {
        pr_debug("%s: Cannot find enable BUS clk\n", __func__);
        return ret;
    }
    
    ret = clk_get_by_index(dev, 1, &clk_mod);
    if (ret) {
        pr_debug("%s: Cannot find get Module clk\n", __func__);
        return ret;
    }
    ret = clk_prepare_enable(&clk_mod);
    if (ret) {
        pr_debug("%s: Cannot find enable Module clk\n", __func__);
        return ret;
    }
    
    return 0;
}

static int suniv_i2s_set_audio_pll(struct i2s_uc_priv *priv, u8 clk)
{
    u8 factor_n, factor_m;
    /* default PLL_AUDIO is 24.571MHz */
    /* But we need 22.5792MHz in this demo */
    /* N/M rate is 0.47 */
    clrbits_le32(PLL_AUDIO_CTRL_REG, ~(PLL_AUDIO_N_MASK & PLL_AUDIO_M_MASK));

    switch (clk) {
    /* FIXME: support 24_571MHz Audio PLL clock */
    case SUNIV_AUDIO_PLL_24_571MHz:
        factor_n = 0x06;
        factor_m = 0x0f;
        pr_debug("SUNIV_AUDIO_PLL_24_571MHz Not supported yet!\n");
        break;
    case SUNIV_AUDIO_PLL_22_579MHz:
        factor_n = 0x06;
        factor_m = 0x0f;
    }

    writel((factor_n + 1) << PLL_AUDIO_N_SHIFT |
           (factor_m + 1) << PLL_AUDIO_M_SHIFT,
           PLL_AUDIO_CTRL_REG);
    setbits_le32(PLL_AUDIO_CTRL_REG, PLL_AUDIO_EN);

    return 0;
}

static int suniv_i2s_reset(struct udevice *dev)
{
    struct reset_ctl *reset = devm_reset_control_get_by_index(dev, 0);
    reset_assert(reset);
    reset_deassert(reset);
    return 0;
}

/* FIXME: support other formats */
static int suniv_i2s_init(struct i2s_uc_priv *priv)
{
    struct suniv_i2s_regs *regs = (struct suniv_i2s_regs *)priv->base_address;
    u32 bps = priv->bitspersample;  /* sample rate */
    // u32 lrf = priv->rfs;
    // u32 chn = priv->channels;
    u32 mode = 0;

    suniv_i2s_set_audio_pll(priv, SUNIV_AUDIO_PLL_22_579MHz);

    /* set Digital Audio Interface to I2S mode */
    clrbits_le32(&regs->ctrl, I2S_CTRL_PCM_SELECT);
    suniv_i2s_reg_dump(&regs->ctrl);
    
    switch (bps) {
    case 16:
        mode = I2S_SR_16BIT;
        break;
    case 20:
        mode = I2S_SR_20BIT;
        pr_debug("I2S_SR_20BIT Not supported yet!\n");
        break;
    case 24:
        mode = I2S_SR_24BIT;
        pr_debug("I2S_SR_24BIT Not supported yet!\n");
        break;
    default:
        log_err("Invalid sample size input %d\n", priv->bitspersample);
    }
    clrbits_le32(&regs->fat0, ~((I2S_WSS_16BCLK) << I2S_WSS_SHIFT));
    setbits_le32(&regs->fat0, (mode << I2S_SR_SHIFT));
    suniv_i2s_reg_dump(&regs->fat0);

    /* set mclk divider */
    clrbits_le32(&regs->clkd, ~I2S_MCLK_DIV_MASK);
    setbits_le32(&regs->clkd, (0x4) << I2S_MCLK_DIV_SHIFT);
    setbits_le32(&regs->clkd, I2S_MCLKO_EN);
    suniv_i2s_reg_dump(&regs->clkd);

    /* clear fifo and fifo counter */
    writel(0, &regs->txcnt);
    writel(0, &regs->rxcnt);
    setbits_le32(&regs->fctl, I2S_TXFIFO_FLUSH);
    setbits_le32(&regs->fctl, I2S_RXFIFO_FLUSH);

    /* enable fifo empty interrupt */
    writel(0, &regs->intc);
    setbits_le32(&regs->intc, BIT(4));
    
    return 0;
}

/* FIXME: add this routine from testing function */
static int i2s_send_data(struct suniv_i2s_regs *regs, u32 *data, uint length)
{
    return 0;
}

static int suniv_i2s_tx_data(struct udevice *dev, void *data, uint data_size)
{
    struct i2s_uc_priv *priv = dev_get_uclass_priv(dev);
    struct suniv_i2s_regs *regs = (struct suniv_i2s_regs *)priv->base_address;
    
    return i2s_send_data(regs, data, data_size / sizeof(u32));
}

static int suniv_i2s_probe(struct udevice *dev)
{
    struct i2s_uc_priv *priv = dev_get_uclass_priv(dev);
    ulong base;
    
    base = dev_read_addr(dev);
    pr_debug("%s, base : 0x%lx\n", __func__, base);
    if (base == FDT_ADDR_T_NONE) {
        log_debug("Missing i2s base\n");
        return -EINVAL;
    }
    
    priv->base_address = base;
    priv->id = 1;
    priv->audio_pll_clk = 24571000;
    priv->samplingrate = 44100;
    priv->bitspersample = 16;
    priv->channels = 2;
    priv->rfs = 256;
    priv->bfs = 32;
    
    suniv_i2s_clock_setup(dev);
    suniv_i2s_reset(dev);
    suniv_i2s_init(priv);
    
    return suniv_i2s_test(priv);
}

static const struct i2s_ops suniv_i2s_ops = {
    .tx_data    = suniv_i2s_tx_data,
};

static const struct udevice_id suniv_i2s_ids[] = {
    { .compatible = "allwinner,suniv-f1c100s-i2s" },
    { }
};

U_BOOT_DRIVER(suniv_i2s) = {
    .name       = "suniv_i2s",
    .id         = UCLASS_I2S,
    .of_match   = suniv_i2s_ids,
    .probe      = suniv_i2s_probe,
    .ops        = &suniv_i2s_ops,
};
