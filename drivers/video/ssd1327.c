// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) 2023 Iota Hydrae <writeforever@foxmail.com>
 */

/*
 * Support for the SSD1327, which can operate behavior though
 * SPI interface for driving a TFT display.
 */

#define pr_fmt(fmt) "ssd1327: " fmt

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <env.h>
#include <log.h>
#include <asm/cache.h>
#include <dm/devres.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <malloc.h>
#include <video.h>

#include <spi.h>
#include <spi-mem.h>
#include <asm/gpio.h>
#include <mipi_display.h>
#include <asm/arch/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * SSD1327 Command Table
 */
#define SSD1327_CMD_SET_COL_ADDR        0x15
#define SSD1327_CMD_SET_ROW_ADDR        0x75
#define SSD1327_CMD_SET_CONTRAST        0x81

/* 0x84 ~ 0x86 are no operation commands */

#define SSD1327_CMD_SET_REMAP                   0xA0
#define SSD1327_CMD_SET_DISPLAY_START_LINE      0xA1
#define SSD1327_CMD_SET_DISPLAY_OFFSET          0xA2

/* SSD1327 Display mode setting */
#define SSD1327_CMD_SET_DISPLAY_MODE_NORMAL          0xA4
#define SSD1327_CMD_SET_DISPLAY_MODE_ON              0xA5    /* All pixel at grayscale level GS15 */
#define SSD1327_CMD_SET_DISPLAY_MODE_OFF             0xA6    /* All pixel at grayscale level GS0 */
#define SSD1327_CMD_SET_DISPLAY_MODE_INVERSE         0xA7

#define SSD1327_CMD_SET_MULTIPLEX_RATIO         0xA8
#define SSD1327_CMD_FUNCTION_SELETION_A         0xAB

#define SSD1327_CMD_SET_DISPLAY_ON               0xAE
#define SSD1327_CMD_SET_DISPLAY_OFF              0xAF

#define SSD1327_CMD_SET_PHASE_LENGTH                0xB1
#define SSD1327_CMD_NOP                             0xB2
#define SSD1327_CMD_SET_FCLK_OSC_FREQ               0xB3
#define SSD1327_CMD_SET_GPIO                        0xB5
#define SSD1327_CMD_SET_SEC_PRE_CHARGE_PERIOD       0xB6
#define SSD1327_CMD_SET_GRAY_SCALE_TABLE            0xB8
#define SSD1327_CMD_SEL_DEF_LINEAR_GC_TABLE         0xB9

/* SSD1327 Default settings */
#define SSD1327_DEF_GAMMA_LEVEL         32

#define DRV_NAME "ssd1327"

struct ssd1327_priv;

struct ssd1327_operations {
    int (*init_display)(struct ssd1327_priv *priv);
    int (*reset)(struct ssd1327_priv *priv);
    int (*clear)(struct ssd1327_priv *priv, int clear);
    int (*idle)(struct ssd1327_priv *priv, bool on);
    int (*blank)(struct ssd1327_priv *priv, bool on);
    int (*sleep)(struct ssd1327_priv *priv, bool on);
    int (*set_var)(struct ssd1327_priv *priv);
    int (*set_gamma)(struct ssd1327_priv *priv, u8 gamma);
    int (*set_cursor)(struct ssd1327_priv *priv, int x, int y);
    int (*set_addr_win)(struct ssd1327_priv *priv, int xs, int ys, int xe, int ye);
};

struct ssd1327_display {
    u32                     xres;
    u32                     yres;
    u32                     bpp;
    u32                     fps;
    u32                     rotate;
    u32                     xs_off;
    u32                     xe_off;
    u32                     ys_off;
    u32                     ye_off;
};

struct ssd1327_priv {
    struct udevice          *dev;
    struct spi_slave        *spi;
    u8                      *buf;
    struct {
        void *buf;
        size_t len;
    } txbuf;
    struct {
        struct gpio_desc *reset;
        struct gpio_desc *dc;
        struct gpio_desc *cs;
        struct gpio_desc *blk;
    } gpio;
    
    /* device specific */
    const struct ssd1327_operations        *tftops;
    
    struct ssd1327_display           *display;
    
    u32             dirty_lines_start;
    u32             dirty_lines_end;
    u8              contrast;
};

static int ssd1327_ofdata_to_platdata(struct udevice *dev)
{
    struct ssd1327_priv *priv = dev_get_priv(dev);
    
    priv->spi = dev_get_parent_priv(dev);
    
    return 0;
}

static int ssd1327_spi_write(struct ssd1327_priv *priv, void *buf, size_t len)
{
    return dm_spi_xfer(priv->dev, (len * BITS_PER_BYTE),
                       buf, NULL, SPI_XFER_ONCE);
}

static int write_buf_dc(struct ssd1327_priv *priv, void *buf, size_t len, int dc)
{
    int ret;
    
    ret = dm_gpio_set_value(priv->gpio.dc, dc);
    if (ret) {
        dev_err(priv->dev, "Failed to handle dc\n");
        return ret;
    }
    
    ret = ssd1327_spi_write(priv, buf, len);
    if (ret)
        dev_err(priv->dev, "write() failed and returned %d\n", ret);
    return ret;
}

static int request_one_gpio(struct ssd1327_priv *priv,
                            const char *name, int index,
                            struct gpio_desc **gpiop)
{
    struct udevice *dev = priv->dev;
    int ret;
    pr_debug("requesting gpio : %s\n", name);
    
    ret = gpio_request_by_name(priv->dev, name, 0, *gpiop, GPIOD_IS_OUT);
    if (ret) {
        dev_err(dev, "failed to request gpio : %s ret:%d", name, ret);
        return ret;
    }
    
    return 0;
}

static int request_gpios(struct ssd1327_priv *priv)
{
    int ret;
    pr_debug("%s\n", __func__);
    
    ret = request_one_gpio(priv, "reset-gpios", 0, &priv->gpio.reset);
    if (ret)
        return ret;
    ret = request_one_gpio(priv, "dc-gpios", 0, &priv->gpio.dc);
    if (ret)
        return ret;
        
    return 0;
}

static inline int ssd1327_write_byte(struct ssd1327_priv *priv, u8 byte, int dc)
{
    return write_buf_dc(priv, &byte, 1, dc);
}

#define write_cmd(__par, __c) ssd1327_write_byte(__par, __c, 0);
#define write_data(__par, __c) ssd1327_write_byte(__par, __c, 1);

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
static int ssd1327_write_reg(struct ssd1327_priv *priv, int len, ...)
{
    u8 *buf = (u8 *)priv->buf;
    va_list args;
    int i;
    
    va_start(args, len);
    
    *buf = (u8)va_arg(args, unsigned int);
    write_buf_dc(priv, buf, sizeof(u8), 0);
    len--;
    
    /* if there no params */
    if (len == 0)
        return 0;
        
    for (i = 0; i < len; i++)
        *buf++ = (u8)va_arg(args, unsigned int);
        
    write_buf_dc(priv, priv->buf, len, 1);
    va_end(args);
    
    return 0;
}
#define write_reg(priv, ...) \
    ssd1327_write_reg(priv, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int ssd1327_reset(struct ssd1327_priv *priv)
{
    dm_gpio_set_value(priv->gpio.reset, 1);
    mdelay(10);
    dm_gpio_set_value(priv->gpio.reset, 0);
    mdelay(50);
    dm_gpio_set_value(priv->gpio.reset, 1);
    mdelay(10);
    return 0;
}

static int ssd1327_init_display(struct ssd1327_priv *priv)
{
    pr_debug("%s, initializing display ...\n", __func__);
    
    write_reg(priv, 0xae);//--turn off oled panel
    
    write_reg(priv, 0x15);    //   set column address
    write_reg(priv, 0x00);    //  start column   0
    write_reg(priv, 0x7f);    //  end column   127
    
    write_reg(priv, 0x75);    //   set row address
    write_reg(priv, 0x00);    //  start row   0
    write_reg(priv, 0x7f);    //  end row   127
    
    write_reg(priv, 0x81);  // set contrast control
    write_reg(priv, 0x80);
    
    write_reg(priv, 0xa0);    // gment remap
    write_reg(priv, 0x51);   //51
    
    write_reg(priv, 0xa1);  // start line
    write_reg(priv, 0x00);
    
    write_reg(priv, 0xa2);  // display offset
    write_reg(priv, 0x00);
    
    write_reg(priv, 0xa4);    // rmal display
    write_reg(priv, 0xa8);    // set multiplex ratio
    write_reg(priv, 0x7f);
    
    write_reg(priv, 0xb1);  // set phase leghth
    write_reg(priv, 0xf1);
    
    write_reg(priv, 0xb3);  // set dclk
    /* 80Hz:0xc1   90Hz:0xe1
     * 100Hz:0x00  110Hz:0x30
     * 120Hz:0x50   130Hz:0x70 */
    write_reg(priv, 0x00);

    write_reg(priv, 0xab);  //
    write_reg(priv, 0x01);  //
    
    write_reg(priv, 0xb6);  // set phase leghth
    write_reg(priv, 0x0f);
    
    write_reg(priv, 0xbe);
    write_reg(priv, 0x0f);
    
    write_reg(priv, 0xbc);
    write_reg(priv, 0x08);
    
    write_reg(priv, 0xd5);
    write_reg(priv, 0x62);
    
    write_reg(priv, 0xfd);
    write_reg(priv, 0x12);
    
    write_reg(priv, 0xaf);
    
    pr_debug("%s, Done.\n", __func__);
    
    return 0;
}

static int ssd1327_blank(struct ssd1327_priv *priv, bool on)
{
    if (on)
        write_reg(priv, SSD1327_CMD_SET_DISPLAY_MODE_OFF);
    else
        write_reg(priv, SSD1327_CMD_SET_DISPLAY_MODE_ON);
    return 0;
}

static int ssd1327_set_addr_win(struct ssd1327_priv *priv, int xs, int ys, int xe,
                                int ye)
{
    /* set column adddress */
    write_reg(priv, SSD1327_CMD_SET_COL_ADDR);
    
    /* 16grayscale cost 4bit which means one byte present 2 pixel */
    write_reg(priv, xs / 2);
    write_reg(priv, xe / 2);
    
    /* set row address */
    write_reg(priv, SSD1327_CMD_SET_ROW_ADDR);
    write_reg(priv, ys);
    write_reg(priv, ye);
    return 0;
}

static int ssd1327_clear(struct ssd1327_priv *priv, int clear)
{
    int width = priv->display->xres;
    int height = priv->display->yres;
    int bpp = priv->display->bpp;
    int i;
    
    ssd1327_set_addr_win(priv, 0, 0, width - 1, height - 1);
    
    dm_gpio_set_value(priv->gpio.dc, 1);
    for (i = 0; i < width * height * bpp / BITS_PER_BYTE; i++)
        ssd1327_spi_write(priv, &clear, 1);

    return 0;
}

static int ssd1327_idle(struct ssd1327_priv *priv, bool on)
{
    return 0;
}

static int ssd1327_sleep(struct ssd1327_priv *priv, bool on)
{
    if (on)
        write_reg(priv, SSD1327_CMD_SET_DISPLAY_OFF);
    else
        write_reg(priv, SSD1327_CMD_SET_DISPLAY_ON);
        
    pr_debug("%s, panel was %s", __func__, on ? "off" : "on");
    return 0;
}

static int __maybe_unused ssd1327_show_img(struct ssd1327_priv *priv, const u8 *img)
{
    int i, j;
    int width = priv->display->xres;
    int height = priv->display->yres;
    int byte;
    ssd1327_set_addr_win(priv, 0, 0, 127, 127);
    
    dm_gpio_set_value(priv->gpio.dc, 1);
    for (i = 0; i < height; i++) {
        for (j = 0; j < width / 2; j++) {
            /* 64 byte means one line length */
            byte = img[j + i * 64];
            ssd1327_spi_write(priv, &byte, 1);
        }
    }
    return 0;
}

static int ssd1327_set_var(struct ssd1327_priv *priv)
{
    return 0;
}

static int ssd1327_set_gamma(struct ssd1327_priv *priv, u8 gamma)
{
    /* The chip has 256 contrast steps from 0x00 to 0xFF */
    gamma &= 0xFF;
    
    write_reg(priv, SSD1327_CMD_SET_CONTRAST);
    write_reg(priv, gamma);
    
    return 0;
}

static const struct ssd1327_operations default_ssd1327_ops = {
    .init_display = ssd1327_init_display,
    .idle         = ssd1327_idle,
    .reset        = ssd1327_reset,
    .clear        = ssd1327_clear,
    .blank        = ssd1327_blank,
    .sleep        = ssd1327_sleep,
    .set_var      = ssd1327_set_var,
    .set_gamma    = ssd1327_set_gamma,
    .set_addr_win = ssd1327_set_addr_win,
};

static int ssd1327_hw_init(struct ssd1327_priv *priv)
{
    int ret;
    pr_debug("%s\n", __func__);
    
    ret = dm_spi_claim_bus(priv->dev);
    if (ret) {
        dev_err(priv->dev, "failed to claim spi bus : %d", ret);
        return ret;
    }
    
    priv->tftops->reset(priv);
    priv->tftops->init_display(priv);
    // priv->tftops->set_var(priv);
    // priv->tftops->clear(priv);

    // priv->tftops->blank(priv, false);
    // priv->tftops->blank(priv, true);
    
    // ssd1327_show_img(priv, gImage_1in5);
    
    dm_spi_release_bus(priv->dev);
    
    return 0;
}

static int ssd1327_of_config(struct ssd1327_priv *priv)
{
    int ret;
    u32 width, height, bpp;
    u32 fps, buswidth;
    
    pr_debug("%s\n", __func__);
    ret = request_gpios(priv);
    if (ret) {
        dev_err(priv->dev, "Request gpio failed!");
        return ret;
    }
    
    width = dev_read_u32_default(priv->dev, "width", 0);
    if (width > 0 && width <= 128)
        priv->display->xres = width;
        
    height = dev_read_u32_default(priv->dev, "height", 0);
    if (width > 0 && width <= 128)
        priv->display->yres = height;
        
    buswidth = dev_read_u32_default(priv->dev, "buswidth", 0);
    if (buswidth != 8) {
        dev_err(priv->dev, "Only 8bit buswidth is supported now");
        return -EINVAL;
    }
    
    bpp = dev_read_u32_default(priv->dev, "bpp", 0);
    if (bpp != 16) {
        dev_err(priv->dev, "Only RGB565 is supported now");
        return -EINVAL;
    } else {
        priv->display->bpp = bpp;
    }
    
    /* FIXME: fps doesn't used anywhere */
    fps = dev_read_u32_default(priv->dev, "fps", 0);
    
    return 0;
}

#define RED(a)      ((((a) & 0xf800) >> 11) << 3)
#define GREEN(a)    ((((a) & 0x07e0) >> 5) << 2)
#define BLUE(a)     (((a) & 0x001f) << 3)
static inline u8 rgb565_to_4bit_grayscale(u16 rgb565)
{
    int r, g, b;
    u16 gray;
    
    /* get each channel and expand them to 8 bit */
    r = RED(rgb565);
    g = GREEN(rgb565);
    b = BLUE(rgb565);
    
    /* convert rgb888 to grayscale */
    gray = ((r * 77 + g * 151 + b * 28) >> 8); // 0 ~ 255
    if (gray == 0)
        return gray;
        
    /*
     * To 4-bit grayscale
     *   B3  B2  B1  B0
     *   0   0   0   0
     * which means have 16 kind of gray
     */
    return (gray / 16);
}

static int ssd1327_video_sync(struct udevice *vid)
{
    struct video_priv *uc_priv = dev_get_uclass_priv(vid);
    struct ssd1327_priv *priv = dev_get_priv(vid);
    
    int i, j, ret;
    u8 p0, p1;
    size_t remain;
    size_t to_copy;
    size_t tx_array_size;
    u8 *txbuf8 = priv->txbuf.buf;
    u16 *vmem16 = uc_priv->fb;
    
    priv->tftops->idle(priv, false);
    
    ret = dm_spi_claim_bus(priv->dev);
    if (ret) {
        dev_err(priv->dev, "failed to claim spi bus : %d", ret);
        return ret;
    }
    
    priv->tftops->set_addr_win(priv, 0, 0,
                               priv->display->xres - 1,
                               priv->display->yres - 1);
                               
    pr_debug("uc_priv->xsize : %d, uc_priv->ysize : %d\n", uc_priv->xsize, uc_priv->ysize);
    for (i = 0, j = 0; i < (uc_priv->xsize * uc_priv->ysize); i += 2, j++) {
        p0 = rgb565_to_4bit_grayscale(vmem16[i]);
        p1 = rgb565_to_4bit_grayscale(vmem16[i + 1]);
        txbuf8[j] = p0 << 4 | p1;
    }
    
    remain = j;
    tx_array_size = priv->txbuf.len;
    while (remain) {
        pr_debug("%s, remain : %d\n", __func__, remain);
        to_copy = min(tx_array_size, remain);
        write_buf_dc(priv, txbuf8, to_copy, 1);
        txbuf8 += to_copy;
        remain -= to_copy;
    }
    
    dm_spi_release_bus(priv->dev);
    
    priv->tftops->idle(priv, true);
    
    return 0;
}

static struct ssd1327_display default_ssd1327_display = {
    .xres   = 128,
    .yres   = 128,
    .bpp    = 16,
    .fps    = 60,
    .rotate = 0,
};

static int ssd1327_probe(struct udevice *dev)
{
    struct video_uc_plat *plat = dev_get_uclass_plat(dev);
    struct video_priv *uc_priv = dev_get_uclass_priv(dev);
    
    struct spi_slave *slave = dev_get_parent_priv(dev);
    struct ssd1327_priv *priv = dev_get_priv(dev);
    
    pr_debug("%s() plat: base 0x%lx, size 0x%x\n",
             __func__, plat->base, plat->size);
             
    priv->spi = slave;
    
    priv->buf = (u8 *)malloc(PAGE_SIZE);
    
    priv->txbuf.buf = (u8 *)malloc(PAGE_SIZE);
    priv->txbuf.len = PAGE_SIZE;
    
    priv->display = &default_ssd1327_display;
    priv->tftops = &default_ssd1327_ops;
    
    priv->dev = dev;
    ssd1327_of_config(priv);
    
    uc_priv->bpix = VIDEO_BPP16;
    uc_priv->xsize = priv->display->xres;
    uc_priv->ysize = priv->display->yres;
    uc_priv->rot = 0;
    
    ssd1327_hw_init(priv);
    
    return 0;
}

static int ssd1327_bind(struct udevice *dev)
{
    struct video_uc_plat *plat = dev_get_uclass_plat(dev);
    struct ssd1327_priv *priv = dev_get_priv(dev);
    
    plat->size = priv->display->xres * priv->display->yres * 16;
    
    return 0;
}

static const struct video_ops ssd1327_ops = {
    .video_sync = ssd1327_video_sync,
};

static const struct udevice_id ssd1327_ids[] = {
    { .compatible = "solomon,ssd1327" },
    { }
};

U_BOOT_DRIVER(ssd1327) = {
    .name = DRV_NAME "_video",
    .id = UCLASS_VIDEO,
    .ops = &ssd1327_ops,
    .bind = ssd1327_bind,
    .of_match = ssd1327_ids,
    .of_to_plat = ssd1327_ofdata_to_platdata,
    .plat_auto = sizeof(struct video_uc_plat),
    .probe = ssd1327_probe,
    .priv_auto = sizeof(struct ssd1327_priv),
};