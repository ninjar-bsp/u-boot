// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) 2023 Iota Hydrae <writeforever@foxmail.com>
 */

/*
 * Support for the ST7567, which can operate behavior though
 * SPI interface for driving a TFT display.
 */

#define pr_fmt(fmt)	"st7567: " fmt

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
 * ST7567 Command Table
 */
#define ST7567_CMD_SET_COL_ADDR        0x15
#define ST7567_CMD_SET_ROW_ADDR        0x75
#define ST7567_CMD_SET_CONTRAST        0x81

/* 0x84 ~ 0x86 are no operation commands */

#define ST7567_CMD_SET_REMAP                   0xA0
#define ST7567_CMD_SET_DISPLAY_START_LINE      0xA1
#define ST7567_CMD_SET_DISPLAY_OFFSET          0xA2

/* ST7567 Display mode setting */
#define ST7567_CMD_SET_DISPLAY_MODE_NORMAL          0xA4
#define ST7567_CMD_SET_DISPLAY_MODE_ON              0xA5    /* All pixel at grayscale level GS15 */
#define ST7567_CMD_SET_DISPLAY_MODE_OFF             0xA6    /* All pixel at grayscale level GS0 */
#define ST7567_CMD_SET_DISPLAY_MODE_INVERSE         0xA7

#define ST7567_CMD_SET_MULTIPLEX_RATIO         0xA8
#define ST7567_CMD_FUNCTION_SELETION_A         0xAB

#define ST7567_CMD_SET_DISPLAY_ON               0xAE
#define ST7567_CMD_SET_DISPLAY_OFF              0xAF

#define ST7567_CMD_SET_PHASE_LENGTH                0xB1
#define ST7567_CMD_NOP                             0xB2
#define ST7567_CMD_SET_FCLK_OSC_FREQ               0xB3
#define ST7567_CMD_SET_GPIO                        0xB5
#define ST7567_CMD_SET_SEC_PRE_CHARGE_PERIOD       0xB6
#define ST7567_CMD_SET_GRAY_SCALE_TABLE            0xB8
#define SSd1327_CMD_SEL_DEF_LINEAR_GC_TABLE         0xB9

/* ST7567 Default settings */
#define ST7567_DEF_GAMMA_LEVEL         32
#define SUNIV_FIFO_DEPTH 128

#define DRV_NAME "st7567"

struct st7567_par;

struct st7567_operations {
        int (*init_display)(struct st7567_par *par);
        int (*reset)(struct st7567_par *par);
        int (*clear)(struct st7567_par *par);
        // int (*idle)(struct st7567_par *par, bool on);
        int (*blank)(struct st7567_par *par, bool on);
        int (*sleep)(struct st7567_par *par, bool on);
        int (*set_var)(struct st7567_par *par);
        // int (*set_gamma)(struct st7567_par *par, u8 gamma);
        // int (*set_addr_win)(struct st7567_par *par, int xs, int ys, int xe, int ye);
        int (*set_cursor)(struct st7567_par *par, int x, int y);
};

struct st7567_display {
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

struct st7567_par {
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
        const struct st7567_operations        *tftops;
        const struct st7567_display           *display;

        // struct fb_info          *fbinfo;
        // struct fb_ops           *fbops;

        u32             pseudo_palette[16];

        u32             dirty_lines_start;
        u32             dirty_lines_end;
        // u8              contrast;
};

static int st7567_ofdata_to_platdata(struct udevice *dev)
{
    struct st7567_par *par = dev_get_priv(dev);

    par->spi = dev_get_parent_priv(dev);

    return 0;
}

static int st7567_spi_write(struct st7567_par *par, void *buf, size_t len)
{
    return dm_spi_xfer(par->dev, (len * BITS_PER_BYTE),
        buf, NULL, SPI_XFER_ONCE);
}

static int write_buf_dc(struct st7567_par *par, void *buf, size_t len, int dc)
{
    int ret;

    // u8 *buf8 = (u8 *)buf;
    // for (int i =0;i < len ; i++) {
    //     printk("dc : %d, val : 0x%x\n", dc, buf8[i]);
    // }

    ret = dm_gpio_set_value(par->gpio.dc, dc);
    if (ret) {
        dev_err(par->dev, "Failed to handle dc\n");
        return ret;
    }

    ret = st7567_spi_write(par, buf, len);
    if (ret)
        dev_err(par->dev, "write() failed and returned %d\n", ret);
    return ret;
}

static int request_one_gpio(struct st7567_par *par,
                            const char *name, int index,
                            struct gpio_desc **gpiop)
{
    struct udevice *dev = par->dev;
    int ret;
    printk("requesting gpio : %s\n", name);
    // ret = dm_gpio_request(*gpiop, name);
    ret = gpio_request_by_name(par->dev, name, 0, *gpiop, GPIOD_IS_OUT);
    if (ret) {
        dev_err(dev, "failed to request gpio : %s ret:%d", name, ret);
        return ret;
    }

    return 0;
}

static int request_gpios(struct st7567_par *par)
{
    int ret;
    printk("%s, ============\n", __func__);

    ret = request_one_gpio(par, "reset-gpios", 0, &par->gpio.reset);
    if (ret)
        return ret;
    ret = request_one_gpio(par, "dc-gpios", 0, &par->gpio.dc);
    if (ret)
        return ret;
    // ret = request_one_gpio(par, "blk-gpios", 0, &par->gpio.reset);
    // if (ret)
    //     return ret;
    // ret = request_one_gpio(par, "cs-gpios", 0, &par->gpio.reset);
    // if (ret)
    //     return ret;

    return 0;
}

//#ifndef SPI_MEM_OP
#if 1
static inline int st7567_write_byte(struct st7567_par *par, u8 byte, int dc)
{
    // int ret;

    // ret = dm_gpio_set_value(par->gpio.dc, dc);
    // if (ret) {
    //     dev_err(par->dev, "failed to handle dc : %d\n", ret);
    //     return ret;
    // }

    // ret = dm_spi_xfer(par->dev, 8, &byte, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
    // if (ret) {
    //     dev_err(par->dev, "failed to write byte : %d\n", ret);
    //     return ret;
    // }
    // return 0;
    return write_buf_dc(par, &byte, 1, dc);
}
#else
static int st7567_write_byte(struct st7567_par *par, u8 byte, int dc)
{
    struct spi_mem_op op = SPI_MEM_OP(SPI_MEM_OP_CMD(byte, 1),
                                    SPI_MEM_OP_NO_ADDR,
                                    SPI_MEM_OP_NO_DUMMY,
                                    SPI_MEM_OP_NO_DATA);
    int ret;

    printk("val : 0x%x, dc : %d\n", byte, dc);

    ret = dm_gpio_set_value(par->gpio.dc, dc);
    if (ret) {
        dev_err(par->dev, "failed to handle dc : %d\n", ret);
        return ret;
    }

    ret = spi_mem_exec_op(par->spi, &op);
    if (ret) {
        dev_err(par->spi->dev, "failed to exec spi_mem op : %d", ret);
		return ret;
    }

    return 0;
}
#endif
#define write_cmd(__par, __c) st7567_write_byte(__par, __c, 0);
#define write_data(__par, __c) st7567_write_byte(__par, __c, 1);

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__}) / sizeof(int))
static int st7567_write_reg(struct st7567_par *par, int len, ...)
{
        u8 *buf = (u8 *)par->buf;
        va_list args;
        int i;

        va_start(args, len);

        *buf = (u8)va_arg(args, unsigned int);
        write_buf_dc(par, buf, sizeof(u8), 0);
        len--;

        /* if there no params */
        if (len == 0)
                return 0;

        for (i = 0; i < len; i++)
                *buf++ = (u8)va_arg(args, unsigned int);

        write_buf_dc(par, par->buf, len, 1);
        va_end(args);

        return 0;
}
#define write_reg(par, ...) \
        st7567_write_reg(par, NUMARGS(__VA_ARGS__), __VA_ARGS__)

static int st7567_reset(struct st7567_par *par)
{
    dm_gpio_set_value(par->gpio.reset, 1);
    mdelay(10);
    dm_gpio_set_value(par->gpio.reset, 0);
    mdelay(200);
    dm_gpio_set_value(par->gpio.reset, 1);
    mdelay(10);
    return 0;
}

#if 0
static int st7567_init_display(struct st7567_par *par)
{
    printk("%s, initializing display ...\n", __func__);
    st7567_reset(par);

    write_reg(par, 0xe2);
    write_reg(par, 0xa6);
    write_reg(par, 0xa1);
    write_reg(par, 0xc8);
    write_reg(par, 0x2f);
    write_reg(par, 0x26);
    write_reg(par, 0xa2);
    write_reg(par, 0xf8);
    write_reg(par, 0x00);
    write_reg(par, 0x81);
    write_reg(par, 0x09);
    write_reg(par, 0xa5);       /* test pixel command */
    write_reg(par, 0xaf);

    printk("%s, Done.\n", __func__);

    return 0;
}
#else
static int st7567_init_display(struct st7567_par *par)
{
    printk("%s, initializing display ...\n", __func__);

    write_reg(par, 0x11); //Sleep out
    mdelay(120);     //Delay 120ms
    //************* Start Initial Sequence **********//
    write_reg(par, 0x36, 0x00);
    write_reg(par, 0x3A, 0x05);
    write_reg(par, 0xB2,0x0C,0x0C,0x00,0x33,0x33);
    write_reg(par, 0xB7,0x35);
    write_reg(par, 0xBB, 0x32);     //Vcom=1.35V
    write_reg(par, 0xC2, 0x01);
    write_reg(par, 0xC3,0x15);     //GVDD=4.8V
    write_reg(par, 0xC4, 0x20);     //VDV, 0x20:0v
    write_reg(par, 0xC6,0x0F);    //0x0F:60Hz
    write_reg(par, 0xD0, 0xA4, 0xA1);
    write_reg(par, 0xE0,0xD0,0x08,0x0E,0x09,0x09,0x05,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34);
    write_reg(par, 0xE1,0xD0,0x08,0x0E,0x09,0x09,0x15,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34);
    write_reg(par, 0x21);
    write_reg(par, 0x29);

    printk("%s, Done.\n", __func__);

    return 0;
}
#endif

static int st7567_clear(struct st7567_par *par)
{
    printk("%s\n", __func__);
    par->tftops->set_cursor(par, 0, 0);

    for (int x =0; x < 240; x++)
        for (int y =0; y < 280; y++) {
            write_data(par, 0);
            write_data(par, 0);
        }
    return 0;
}

static int st7567_blank(struct st7567_par *par, bool on)
{
    return 0;
}

static int st7567_sleep(struct st7567_par *par, bool on)
{
    return 0;
}

static int st7567_set_var(struct st7567_par *par)
{
    return 0;
}

static int st7567_set_cursor(struct st7567_par *par, int x, int y)
{
	write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
		  (x >> 8) & 0xFF, x & 0xFF, 0x00, 0xEF);

	write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
		  (y >> 8) & 0xFF, y & 0xFF, 0x00, 0xEF);

	write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
    return 0;
}

static const struct st7567_operations default_st7567_ops = {
    .init_display = st7567_init_display,
    .reset        = st7567_reset,
    .clear        = st7567_clear,
    .blank        = st7567_blank,
    .sleep        = st7567_sleep,
    .set_var      = st7567_set_var,
    .set_cursor   = st7567_set_cursor,
};

static int st7567_hw_init(struct st7567_par *par)
{
    int ret;
    printk("%s\n", __func__);

    // ret = dm_spi_claim_bus(par->dev);
    // if (ret) {
    //     dev_err(par->dev, "Failed to claim SPI BUS: %d\n", ret);
    //     return ret;
    // }

    par->tftops->reset(par);
    par->tftops->init_display(par);
    // par->tftops->set_var(par);
    par->tftops->clear(par);

    // dm_spi_release_bus(par->dev);

    return 0;
}

static int st7567_of_config(struct st7567_par *par)
{
    int ret;

    printk("%s\n", __func__);
    ret = request_gpios(par);
    if (ret) {
        dev_err(par->dev, "Request gpio failed!");
        return ret;
    }

    return 0;
}

static const struct st7567_display default_st7567_display = {
    .xres = 128,
    .yres = 64,
    .bpp = 1,
    .fps = 60,
    .rotate = 0,
};

static int st7567_video_sync(struct udevice *vid)
{
    
}

static int st7567_probe(struct udevice *dev)
{
    struct video_uc_plat *plat = dev_get_uclass_plat(dev);
    struct video_priv *uc_priv = dev_get_uclass_priv(dev);

    struct spi_slave *slave = dev_get_parent_priv(dev);
    struct st7567_par *par = dev_get_priv(dev);
    u32 fb_start, fb_end;
    u32 buswidth;
    int ret;

	printk("%s() plat: base 0x%lx, size 0x%x\n",
	       __func__, plat->base, plat->size);

    buswidth = dev_read_u32_default(dev, "buswidth", 0);
    if (buswidth != 8) {
        dev_err(dev, "Only 8bit buswidth is supported now");
        return -EINVAL;
    }

    par->spi = slave;
    par->buf = (u8 *)malloc(PAGE_SIZE);

    ret = dm_spi_claim_bus(dev);
    // ret = spi_claim_bus(par->spi);
    if (ret) {
        dev_err(dev, "failed to claim spi bus : %d", ret);
        return ret;
    }

    par->display = &default_st7567_display;
    par->tftops = &default_st7567_ops;

    switch(par->display->bpp) {
    case 32:
    case 24:
    case 16:
    case 8:
        dev_err(par->dev, "Invaild bpp specified (bpp = %i)", par->display->bpp);
        break;
    case 1:
        break;
    }

    par->dev = dev;
    st7567_of_config(par);

    uc_priv->bpix = VIDEO_BPP8;
    uc_priv->xsize = par->display->xres;
    uc_priv->ysize = par->display->yres;
    uc_priv->rot = 0;

    fb_start = plat->base;
    fb_end = plat->base + plat->size;

    st7567_hw_init(par);

    dm_spi_release_bus(par->dev);
    // spi_release_bus(par->spi);

    gd->fb_base = plat->base;
    return 0;
}

static const struct video_ops st7567_ops = {
    .video_sync = NULL,
};

static const struct udevice_id st7567_ids[] = {
    { .compatible = "sitronix,st7567" },
    { }
};

U_BOOT_DRIVER(st7567) = {
    .name = DRV_NAME"_video",
    .id = UCLASS_VIDEO,
    .ops = &st7567_ops,
    .of_match = st7567_ids,
    .of_to_plat = st7567_ofdata_to_platdata,
    .probe = st7567_probe,
    .priv_auto = sizeof(struct st7567_par),
};