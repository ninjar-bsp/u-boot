// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 iotah <writeforever@foxmail.com>
 */

#include <common.h>
#include <image.h>
#include <malloc.h>
#include <log.h>
#include <spl.h>
#include <asm/arch/spl.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/libfdt.h>

#include <u-boot/crc.h>
#include <hang.h>

#ifdef CONFIG_SPL_OS_BOOT
    #error CONFIG_SPL_OS_BOOT is not supported yet
#endif

/*****************************************************************************/
/* SUN4I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN4I_SPI0_CCTL             0x1C
#define SUN4I_SPI0_CTL              0x08
#define SUN4I_SPI0_RX               0x00
#define SUN4I_SPI0_TX               0x04
#define SUN4I_SPI0_FIFO_STA         0x28
#define SUN4I_SPI0_BC               0x20
#define SUN4I_SPI0_TC               0x24

#define SUN4I_CTL_ENABLE            BIT(0)
#define SUN4I_CTL_MASTER            BIT(1)
#define SUN4I_CTL_TF_RST            BIT(8)
#define SUN4I_CTL_RF_RST            BIT(9)
#define SUN4I_CTL_XCH               BIT(10)

/*****************************************************************************/
/* SUN6I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN6I_SPI0_CCTL             0x24
#define SUN6I_SPI0_GCR              0x04
#define SUN6I_SPI0_TCR              0x08
#define SUN6I_SPI0_FIFO_STA         0x1C
#define SUN6I_SPI0_MBC              0x30
#define SUN6I_SPI0_MTC              0x34
#define SUN6I_SPI0_BCC              0x38
#define SUN6I_SPI0_TXD              0x200
#define SUN6I_SPI0_RXD              0x300

#define SUN6I_CTL_ENABLE            BIT(0)
#define SUN6I_CTL_MASTER            BIT(1)
#define SUN6I_CTL_SRST              BIT(31)
#define SUN6I_TCR_XCH               BIT(31)

/*****************************************************************************/

#define CCM_AHB_GATING0             (0x01C20000 + 0x60)
#define CCM_H6_SPI_BGR_REG          (0x03001000 + 0x96c)
#ifdef CONFIG_SUN50I_GEN_H6
    #define CCM_SPI0_CLK                (0x03001000 + 0x940)
#else
    #define CCM_SPI0_CLK                (0x01C20000 + 0xA0)
#endif
#define SUN6I_BUS_SOFT_RST_REG0     (0x01C20000 + 0x2C0)

#define AHB_RESET_SPI0_SHIFT        20
#define AHB_GATE_OFFSET_SPI0        20

#define SPI0_CLK_DIV_BY_2           0x1000
#define SPI0_CLK_DIV_BY_4           0x1001
#define SPI0_CLK_DIV_BY_32          0x100f

#define SPI_READ_MAX_SIZE 60 /* FIFO size, minus 4 bytes of the header */

struct sunxi_spi_reg_offsets {
    ulong spi_ctl_reg;
    ulong spi_ctl_xch_bitmask;
    ulong spi_fifo_reg;
    ulong spi_tx_reg;
    ulong spi_rx_reg;
    ulong spi_bc_reg;
    ulong spi_tc_reg;
    ulong spi_bcc_reg;
};

struct sunxi_spi_reg_offsets sun6i_spi_reg_offsets = {
    .spi_ctl_reg         = SUN6I_SPI0_TCR,
    .spi_fifo_reg        = SUN6I_SPI0_FIFO_STA,
    .spi_tx_reg          = SUN6I_SPI0_TXD,
    .spi_rx_reg          = SUN6I_SPI0_RXD,
    .spi_bc_reg          = SUN6I_SPI0_MBC,
    .spi_tc_reg          = SUN6I_SPI0_MTC,
    .spi_bcc_reg         = SUN6I_SPI0_BCC,
};

struct sunxi_spi_reg_offsets sun4i_spi_reg_offsets = {
    .spi_ctl_reg         = SUN4I_SPI0_CTL,
    .spi_fifo_reg        = SUN4I_SPI0_FIFO_STA,
    .spi_tx_reg          = SUN4I_SPI0_TX,
    .spi_rx_reg          = SUN4I_SPI0_RX,
    .spi_bc_reg          = SUN4I_SPI0_BC,
    .spi_tc_reg          = SUN4I_SPI0_TC,
    .spi_bcc_reg         = 0,
};
#define to_sunxi_spi_reg(spi, reg) \
    (spi->base + spi->reg_offsets->spi_##reg)

struct spi_nand_device {
    char *name;
    unsigned page;

    size_t page_size;
    size_t block_size;
};

struct sunxi_spi {
    uintptr_t base;
    
    size_t fifo_depth;
    
    bool is_sun6i;
    struct sunxi_spi_reg_offsets *reg_offsets;
    ulong spi_ctl_xch_bitmask;

    struct spi_nand_device nand_dev;
};

/*****************************************************************************/
/* Winbond SPI NAND Instructions Table                                     */
/*****************************************************************************/
#define SPI_NAND_RESET          0xff    /* device reset */
#define SPI_NAND_RD_ID          0x9f    /* read jedec id */
#define SPI_NAND_RD_STAT_REG    0x0f    /* read status register */
#define SPI_NAND_WD_STAT_REG    0x1f    /* write status register */
#define SPI_NAND_WD_ON          0x06    /* write enable */
#define SPI_NAND_WD_OFF         0x04    /* write disable */
#define SPI_NAND_RD_PAGE_DATA   0x13    /* read page into data buffer */
#define SPI_NAND_RD_DATA        0x03    /* read from data buffer */

/*****************************************************************************/

/*
 * Allwinner A10/A20 SoCs were using pins PC0,PC1,PC2,PC23 for booting
 * from SPI Flash, everything else is using pins PC0,PC1,PC2,PC3.
 * The H6 uses PC0, PC2, PC3, PC5, the H616 PC0, PC2, PC3, PC4.
 */
static void spi0_pinmux_setup(unsigned int pin_function)
{
    /* All chips use PC0 and PC2. */
    sunxi_gpio_set_cfgpin(SUNXI_GPC(0), pin_function);
    sunxi_gpio_set_cfgpin(SUNXI_GPC(2), pin_function);
    
    /* All chips except H6 and H616 use PC1. */
    if (!IS_ENABLED(CONFIG_SUN50I_GEN_H6))
        sunxi_gpio_set_cfgpin(SUNXI_GPC(1), pin_function);
        
    if (IS_ENABLED(CONFIG_MACH_SUN50I_H6))
        sunxi_gpio_set_cfgpin(SUNXI_GPC(5), pin_function);
    if (IS_ENABLED(CONFIG_MACH_SUN50I_H616))
        sunxi_gpio_set_cfgpin(SUNXI_GPC(4), pin_function);
        
    /* Older generations use PC23 for CS, newer ones use PC3. */
    if (IS_ENABLED(CONFIG_MACH_SUN4I) || IS_ENABLED(CONFIG_MACH_SUN7I) ||
        IS_ENABLED(CONFIG_MACH_SUN8I_R40))
        sunxi_gpio_set_cfgpin(SUNXI_GPC(23), pin_function);
    else
        sunxi_gpio_set_cfgpin(SUNXI_GPC(3), pin_function);
}

static bool is_sun6i_gen_spi(void)
{
    return IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I) ||
           IS_ENABLED(CONFIG_SUN50I_GEN_H6);
}

static uintptr_t spi0_base_address(void)
{
    if (IS_ENABLED(CONFIG_MACH_SUN8I_R40))
        return 0x01C05000;
        
    if (IS_ENABLED(CONFIG_SUN50I_GEN_H6))
        return 0x05010000;
        
    if (!is_sun6i_gen_spi() ||
        IS_ENABLED(CONFIG_MACH_SUNIV))
        return 0x01C05000;
        
    return 0x01C68000;
}

/*
 * Setup 6 MHz from OSC24M (because the BROM is doing the same).
 */
static void spi0_enable_clock(void)
{
    uintptr_t base = spi0_base_address();
    
    /* Deassert SPI0 reset on SUN6I */
    if (IS_ENABLED(CONFIG_SUN50I_GEN_H6))
        setbits_le32(CCM_H6_SPI_BGR_REG, (1U << 16) | 0x1);
    else if (is_sun6i_gen_spi())
        setbits_le32(SUN6I_BUS_SOFT_RST_REG0,
                     (1 << AHB_RESET_SPI0_SHIFT));
                     
    /* Open the SPI0 gate */
    if (!IS_ENABLED(CONFIG_SUN50I_GEN_H6))
        setbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));
        
    if (IS_ENABLED(CONFIG_MACH_SUNIV)) {
        /* Divide by 32, clock source is AHB clock 200MHz */
        writel(SPI0_CLK_DIV_BY_32, base + SUN6I_SPI0_CCTL);
    } else {
        /* Divide by 4 */
        writel(SPI0_CLK_DIV_BY_4, base + (is_sun6i_gen_spi() ?
                                          SUN6I_SPI0_CCTL : SUN4I_SPI0_CCTL));
        /* 24MHz from OSC24M */
        writel((1 << 31), CCM_SPI0_CLK);
    }
    
    if (is_sun6i_gen_spi()) {
        /* Enable SPI in the master mode and do a soft reset */
        setbits_le32(base + SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
                     SUN6I_CTL_ENABLE | SUN6I_CTL_SRST);
        /* Wait for completion */
        while (readl(base + SUN6I_SPI0_GCR) & SUN6I_CTL_SRST)
            ;
    } else {
        /* Enable SPI in the master mode and reset FIFO */
        setbits_le32(base + SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
                     SUN4I_CTL_ENABLE |
                     SUN4I_CTL_TF_RST |
                     SUN4I_CTL_RF_RST);
    }
}

static void spi0_disable_clock(void)
{
    uintptr_t base = spi0_base_address();
    
    /* Disable the SPI0 controller */
    if (is_sun6i_gen_spi())
        clrbits_le32(base + SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
                     SUN6I_CTL_ENABLE);
    else
        clrbits_le32(base + SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
                     SUN4I_CTL_ENABLE);
                     
    /* Disable the SPI0 clock */
    if (!IS_ENABLED(CONFIG_MACH_SUNIV))
        writel(0, CCM_SPI0_CLK);
        
    /* Close the SPI0 gate */
    if (!IS_ENABLED(CONFIG_SUN50I_GEN_H6))
        clrbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));
        
    /* Assert SPI0 reset on SUN6I */
    if (IS_ENABLED(CONFIG_SUN50I_GEN_H6))
        clrbits_le32(CCM_H6_SPI_BGR_REG, (1U << 16) | 0x1);
    else if (is_sun6i_gen_spi())
        clrbits_le32(SUN6I_BUS_SOFT_RST_REG0,
                     (1 << AHB_RESET_SPI0_SHIFT));
}

static void spi0_init(struct sunxi_spi *spi)
{
    unsigned int pin_function = SUNXI_GPC_SPI0;
    
    if (IS_ENABLED(CONFIG_MACH_SUN50I) ||
        IS_ENABLED(CONFIG_SUN50I_GEN_H6))
        pin_function = SUN50I_GPC_SPI0;
    else if (IS_ENABLED(CONFIG_MACH_SUNIV))
        pin_function = SUNIV_GPC_SPI0;
        
    spi0_pinmux_setup(pin_function);
    spi0_enable_clock();
    
    spi->base = spi0_base_address();
    spi->is_sun6i = is_sun6i_gen_spi();
    spi->fifo_depth = SPI_READ_MAX_SIZE;
    
    if (spi->is_sun6i) {
        spi->reg_offsets = &sun6i_spi_reg_offsets;
        spi->spi_ctl_xch_bitmask = SUN6I_TCR_XCH;
    }
    else {
        spi->reg_offsets = &sun4i_spi_reg_offsets;
        spi->spi_ctl_xch_bitmask = SUN4I_CTL_XCH;
    }
}

static void spi0_deinit(struct sunxi_spi *spi)
{
    /* New SoCs can disable pins, older could only set them as input */
    unsigned int pin_function = SUNXI_GPIO_INPUT;
    
    if (is_sun6i_gen_spi())
        pin_function = SUNXI_GPIO_DISABLE;
        
    spi0_disable_clock();
    spi0_pinmux_setup(pin_function);
}

static ssize_t spi0_write_then_read(struct sunxi_spi *spi,
                                    const void *txbuf, unsigned n_tx,
                                    void *rxbuf, unsigned n_rx)
{
    ssize_t  len     = 0;
    u8       *txbuf8 = (u8 *)txbuf;
    u8       *rxbuf8 = (u8 *)rxbuf;

    ulong spi_bc_reg   = to_sunxi_spi_reg(spi, bc_reg);
    ulong spi_tc_reg   = to_sunxi_spi_reg(spi, tc_reg);
    ulong spi_bcc_reg  = to_sunxi_spi_reg(spi, bcc_reg);
    ulong spi_rx_reg   = to_sunxi_spi_reg(spi, rx_reg);
    ulong spi_tx_reg   = to_sunxi_spi_reg(spi, tx_reg);
    ulong spi_ctl_reg  = to_sunxi_spi_reg(spi, ctl_reg);
    ulong spi_fifo_reg = to_sunxi_spi_reg(spi, fifo_reg);

    printf("%s, n_tx : %d, r_tx : %d\n", __func__, n_tx, n_rx);

    /* Burst counter (total bytes) */
    writel(n_tx + n_tx, spi_bc_reg);
    /* Transfer counter (bytes to send) */
    writel(n_tx, spi_tc_reg);

    if (spi->is_sun6i)
        writel(n_tx, spi_bcc_reg);

    for (int i = 0; i < n_tx; i++) {
        printf("writing 0x%02x into 0x%lx\n", txbuf8[i], spi_tx_reg);
        writeb(txbuf8[i], spi_tx_reg);
    }

    // printf("start transfer ctl reg : %lx\n", spi_ctl_reg);
    /* Start the data transfer */
    setbits_le32(spi_ctl_reg, spi->spi_ctl_xch_bitmask);
    udelay(300);

    // printf("waiting for rx fifo ready\n");
    /* Wait until everything is received in the RX FIFO */
    while ((readl(spi_fifo_reg) & 0x7F) < n_tx + n_tx)
        ;

    /* Skip 4 bytes */
    readl(spi_rx_reg);

    /* if only need to write */
    if (n_rx <= 0)
        return 0;

    // printf("reading out from fifo\n");
    u8 db = 0;
    while (n_rx-- > 0) {
        db = readb(spi_rx_reg);
        *rxbuf8++ = db;
        len++;
        printf(" %02x", db);
    }
    puts("\n");

    return (len == n_rx) ? -1 : len;
}

static inline ssize_t spi_w8r8(struct sunxi_spi *spi, u8 cmd)
{
    ssize_t status;
    u8      result;
    
    status = spi0_write_then_read(spi, &cmd, 1, &result, 1);
    
    return (status < 0) ? status : result;
}

static inline ssize_t spi_w8r16(struct sunxi_spi *spi, u8 cmd)
{
    ssize_t  status;
    u16      result;
    
    status = spi0_write_then_read(spi, &cmd, 1, &result, 2);
    
    return (status < 0) ? status : result;
}

static inline u32 spi_nand_read_id(struct sunxi_spi *spi)
{
    u8 cmds[] = {0x9f, 0x00};
    u32     result;
    ssize_t status;
    
    status = spi0_write_then_read(spi, cmds, sizeof(cmds), &result, 3);
    result = cpu_to_be32(result) >> 8;
    
    return (status < 0) ? status : result;
}

static inline u8
spi_nand_read_status_reg(struct sunxi_spi *spi, u8 reg_addr)
{
    return spi_w8r8(spi, reg_addr);
}

static inline int
spi_nand_write_status_reg(struct sunxi_spi *spi, u8 reg_addr, u8 reg_val)
{
    u8 txbuf[] = { reg_addr, reg_val };

    return spi0_write_then_read(spi, txbuf, sizeof(txbuf), NULL, 0);
}

static inline int spi_nand_init(struct sunxi_spi *spi)
{
    puts("loading payload from SPI NAND ...\n");
    
    u32 id = spi_nand_read_id(spi);
    printf("JEDEC ID : 0x%08x\n", id);

    if (id == 0x00efaa00) {
        spi->nand_dev.name = "w25n01g";
        spi->nand_dev.page_size = (1 << 11);    /* 2048 */
        spi->nand_dev.block_size = (1 << 17);   /* 128KB */
    } else {
        puts("### NAND Device Not supported yet!\n");
        // hang();
    }

    /* in the beginning, there's no page was read into buf */
    spi->nand_dev.page = -1;

    return 0;
}

static int spi_nand_load_page_op(struct sunxi_spi *spi, unsigned page)
{
    u8 txbuf[] = {
        SPI_NAND_RD_PAGE_DATA,
        0x00,   /* dummy clock */
        (u8)(page >> 8),
        (u8)(page),
    };

    return spi0_write_then_read(spi, txbuf, sizeof(txbuf), NULL, 0);
}

static int
spi_nand_read_from_cache_op(struct sunxi_spi *spi, unsigned column,
                                    void *rxbuf, size_t len)
{
    u8 txbuf[] = {
        SPI_NAND_RD_DATA,
        (u8)(column >> 8),
        (u8)(column),
        0x00,   /* dummy clock */
    };

    return spi0_write_then_read(spi, txbuf, sizeof(txbuf), rxbuf, len);

    return 0;
}

static int sunxi_spi0_nand_read_data(struct sunxi_spi *spi, void *buf, u32 addr, u32 len)
{
    unsigned page = addr >> 11;

    /* load page to data buffer */
    if (spi->nand_dev.page != page) {
        spi->nand_dev.page = page;

        printf("loading page %d into cache\n", spi->nand_dev.page);
        spi_nand_load_page_op(spi, page);
    }

    /*
     * the check the addr, make sure it's in data buffer
     * usually be used to load the endless of the page
     */
    addr &= 0x7FF;
    if (len + addr > spi->nand_dev.page_size)
        len = spi->nand_dev.page_size - addr;

    return spi_nand_read_from_cache_op(spi, addr, buf, len);
}

static int spi0_nand_read_data(struct sunxi_spi *spi, void *buf, u32 addr, u32 len)
{
	u8 *buf8 = buf;
	u32 chunk_len;

	size_t loop_count = 0;

	printf("read into %p, from flash 0x%08x, length: %d\n", buf, addr, len);

	while (len > 0) {
		chunk_len = len;
		if (chunk_len > SPI_READ_MAX_SIZE)
			chunk_len = SPI_READ_MAX_SIZE;

		sunxi_spi0_nand_read_data(spi, buf8, addr, chunk_len);
		len  -= chunk_len;
		buf8 += chunk_len;
		addr += chunk_len;

		loop_count++;
	}

	printf(">>> with %d rounds copy loop, payload has been loaded to memory\n", loop_count);

	return 0;
}

static int spl_spi_load_image(struct spl_image_info *spl_image,
                              struct spl_boot_device *bootdev)
{
    int ret = 0;
    struct legacy_img_hdr *header;
    uint32_t load_offset = sunxi_get_spl_size();
    
    header = (struct legacy_img_hdr *)CONFIG_TEXT_BASE;
    load_offset = max_t(uint32_t, load_offset, CONFIG_SYS_SPI_U_BOOT_OFFS);
    
    struct sunxi_spi *spi = (struct sunxi_spi *)malloc(sizeof(struct sunxi_spi));
    
    spi0_init(spi);

    spi_nand_init(spi);

    spi0_nand_read_data(spi, (void *)header, load_offset, 0x40);

    printf("image header infomation :\n");
	printf("\t ih_magic;  -->> 0x%x/* Image Header Magic Number  */  \n", header->ih_magic);
	printf("\t ih_hcrc;  -->> 0x%x /* Image Header CRC Checksum  */  \n", header->ih_hcrc);
	printf("\t ih_time;  -->> 0x%x /* Image Creation Timestamp	*/    \n", header->ih_time);
	printf("\t ih_size;  -->> 0x%x /* Image Data Size		*/        \n", header->ih_size);
	printf("\t ih_load;  -->> 0x%08x /* Data	 Load  Address		*/    \n", header->ih_load);
	printf("\t ih_ep;	  -->> 0x%08x /* Entry Point Address		*/    \n", header->ih_ep);
	printf("\t ih_dcrc;  -->> 0x%x /* Image Data CRC Checksum	*/    \n", header->ih_dcrc);
	printf("\t ih_os;	  -->> 0x%x /* Operating System		*/        \n", header->ih_os);
	printf("\t ih_arch;  -->> 0x%x /* CPU architecture		*/        \n", header->ih_arch);
	printf("\t ih_type;  -->> 0x%x /* Image Type			*/        \n", header->ih_type);
	printf("\t ih_comp;  -->> 0x%x /* Compression Type		*/        \n", header->ih_comp);

    hang();

    if (IS_ENABLED(CONFIG_SPL_LOAD_FIT) &&
		image_get_magic(header) == FDT_MAGIC) {
		debug("Not supported FIT image yet!\n");
	} else {
		spl_image->flags |= SPL_COPY_PAYLOAD_ONLY;
		ret = spl_parse_image_header(spl_image, bootdev, header);
		if (ret)
			return ret;

		// printf("u-boot name : %s\n", spl_image->name);
		printf("\t load addr : 0x%lx\n", spl_image->load_addr);
		printf("\t     entry : 0x%lx\n", spl_image->entry_point);
		printf("\t load offt : 0x%x\n", load_offset);
		printf("\t      size : 0x%x\n", spl_image->size);
		printf("\t dcrc data : 0x%lx\n", spl_image->dcrc_data);
		printf("\t dcrc len : %lu\n", spl_image->dcrc_length);
		// printf("image payload start address : 0x%lx\n", image_get_data(header));

		memset((void *)CONFIG_TEXT_BASE, 0x00, spl_image->size);
		// spi0_read_data((void *)spl_image->load_addr,
		// 	       load_offset, spl_image->size);

		spi0_nand_read_data(spi, (void *)spl_image->load_addr,
				   load_offset, spl_image->size);

		/* one more step, make sure data is correct */
		// size_t dump_len = 0x800;
		// printf(">>> dump 0x%x of starting\n", dump_len);

		// hexdump(0x81700000, 0, dump_len + 0x40);
		// printf(">>> dump 0x%x of ending\n", dump_len);
		// hexdump(0x81700000, spl_image->dcrc_length - dump_len, dump_len);

		/* mem_flash_cmp(void *mem_start, ulong flash_start, size_t len) */
		uint32_t crc = crc32(0, (void *)0x81700000, spl_image->dcrc_length);
		if (crc != header->ih_dcrc) {
			printf(">>> crc of payload is 0x%08x which should be 0x%08x\n", crc, header->ih_dcrc);
			puts("the crc of payload is mismatch, we don't need go futher!\n");
			hang();
		} else {
			puts("crc of payload matched, prepare for launching!\n");
		}
	}

    hang();

    spi0_deinit(spi);

    return ret;
}

/* Use priorty 0 to override the default if it happens to be linked in */
SPL_LOAD_IMAGE_METHOD("sunxi SPI-NAND", 0, BOOT_DEVICE_NAND, spl_spi_load_image);