#include <common.h>
#include <dm.h>
#include <mmc.h>
#include <asm/addrspace.h>
#include <clk.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/cache.h>
#include <linux/io.h>

#define MISC_CTRL               PHYS_TO_UNCACHED(0x1fd00424)
#define DMA_REG_BASE            PHYS_TO_UNCACHED(0x1fd01160)
#define SDIO_DMA_DESCADDR       PHYS_TO_UNCACHED(0x10000)

#define BLOCK_MAX               2048

#define SDICON                  0x00
#define SDIPRE                  0x04
#define SDICMDARG               0x08
#define SDICMDCON               0x0c
#define SDICMDSTA               0x10
#define SDIRSP0                 0x14
#define SDIRSP1                 0x18
#define SDIRSP2                 0x1C
#define SDIRSP3                 0x20
#define SDIDTIMER               0x24
#define SDIBSIZE                0x28
#define SDIDATCON               0x2C
#define SDIDATCNT               0x30
#define SDIDATSTA               0x34
#define SDIFSTA                 0x38
#define SDIINTMSK               0x3C
#define SDIWRDAT                0x40
#define SDISTAADD0              0x44
#define SDISTAADD1              0x48
#define SDISTAADD2              0x4c
#define SDISTAADD3              0x50
#define SDISTAADD4              0x54
#define SDISTAADD5              0x58
#define SDISTAADD6              0x5c
#define SDISTAADD7              0x60
#define SDIINTEN                0x64
#define SDIO_CON_ENCLK          BIT(0)
#define SDIO_CON_DIACLK         0
#define SDIO_CON_SOFTRST        BIT(8)

#define SDIO_PRE_REVCLOCK       BIT(31)

#define SDIO_CMDCON_CMST        BIT(8)
#define SDIO_CMDCON_WAITRSP     BIT(9)
#define SDIO_CMDCON_LONGRSP     BIT(10)
#define SDIO_CMDCON_AUTOSTOP    BIT(12)
#define SDIO_CMDCON_CHECKCRC    BIT(13)
#define SDIO_CMDCON_SDIOEN      BIT(14)

#define SDIO_DATCON_DTST        BIT(14)
#define SDIO_DATCON_DMAEN       BIT(15)
#define SDIO_DATCON_WIDEMODE    BIT(16)
#define SDIO_DATCON_WIDEMODE8B  BIT(26)

#define SDIO_INTMSK_DATFIN      BIT(0)
#define SDIO_INTMSK_DATTOUT     BIT(1)
#define SDIO_INTMSK_DATCRC      BIT(2)
#define SDIO_INTMSK_CRCSTA      BIT(3)
#define SDIO_INTMSK_PROGERR     BIT(4)
#define SDIO_INTMSK_SDIO        BIT(5)
#define SDIO_INTMSK_CMDFIN      BIT(6)
#define SDIO_INTMSK_CMDTOUT     BIT(7)
#define SDIO_INTMSK_RSPCRC      BIT(8)
#define SDIO_INTMSK_R1BFIN      BIT(9)

#define SDIO_MAKE_CMD(c,f)      ((c & 0x3f) | 0x140 | (f))
#define SDIO_GET_CMDINT(c)      (c & 0x3c0)
#define SDIO_GET_DATAINT(c)     (c & 0x1f)
#define SDIO_GET_CMDIDX(c)      (c & 0x3f)
#define SDIO_GET_CMD(c)         ((c>>8) & 0x3f)
#define DMA_CMD_INTMSK          BIT(0)
#define DMA_CMD_WRITE           BIT(12)

/*this driver use dma_channel_one-bit 0 = 1*/
#define DMA_CHANNEL_SELECT(x,y) ((x<<1)|(y<<0))

#define DMA_SINGLE_TRANS_OVER   BIT(2)
#define DMA_OREDER_ASK          BIT(2)
#define DMA_OREDER_START        BIT(3)
#define DMA_OREDER_STOP         BIT(4)
#define DMA_ALIGNED             32
#define DMA_ALIGNED_MSK         (~(DMA_ALIGNED - 1))

struct sdio_mmc_plat {
    struct mmc_config cfg;
    struct mmc mmc;
};

struct sdio_host {
    const char *name;
    fdt_addr_t ioaddr;
    struct mmc *mmc;
    struct mmc_config *cfg;
    void *dma_desc_addr;
    void *wdma_order_addr;
    void *rdma_order_addr;
    unsigned int sdio_bus_clk;
    unsigned int clock;
};

struct dma_desc {
    volatile unsigned int order_addr_low;
    volatile unsigned int saddr_low;
    volatile unsigned int daddr;
    volatile unsigned int length;
    volatile unsigned int step_length;
    volatile unsigned int step_times;
    volatile unsigned int cmd;
    volatile unsigned int order_addr_high;
    volatile unsigned int saddr_high;
};

static inline void sdio_writel(struct sdio_host *host, int reg, u32 val)
{
    iowrite32(val, host->ioaddr + reg);
}

static inline u32 sdio_readl(struct sdio_host *host, int reg)
{
    return ioread32(host->ioaddr + reg);
}

static void dma_desc_start(struct sdio_host *host)
{
    iowrite32(VA_TO_PHYS((unsigned int)host->dma_desc_addr | DMA_OREDER_START | DMA_CHANNEL_SELECT(0, 1)), DMA_REG_BASE);
}

static void misc_set_dma(void)
{
    unsigned int misc_ctrl;
    misc_ctrl =  ioread32(MISC_CTRL);
    misc_ctrl &= ~(0x3 << 23);
    misc_ctrl |= 0x1010000;  //sdio use dma1
    iowrite32(misc_ctrl, MISC_CTRL);
}

static void dma_ask_valid_done(void)
{
    iowrite32(0x10004, DMA_REG_BASE);
    while (ioread32(DMA_REG_BASE) & DMA_OREDER_ASK);
    while (ioread32(0x80010018) & DMA_SINGLE_TRANS_OVER);
}

static unsigned int sdio_cmd_prepare_flag(struct mmc_cmd *cmd)
{
    unsigned int   flag = 0;
    unsigned short cmd_index;
    unsigned int   rsp;

    cmd_index = cmd->cmdidx;
    rsp       = cmd->resp_type;

    if (rsp == MMC_RSP_NONE) {
        return flag;
    }

    if (rsp & MMC_RSP_PRESENT) {
        flag |= SDIO_CMDCON_WAITRSP;
    }

// if response type is R2 and the crc bit set on, some unknow error will happen
    if ((rsp & MMC_RSP_CRC) && !(cmd_index == 2 || cmd_index == 9)) {
        flag |= SDIO_CMDCON_CHECKCRC;
    }

    if (rsp & MMC_RSP_136) {
        flag |= SDIO_CMDCON_LONGRSP;
    }

    if (cmd_index == MMC_CMD_READ_MULTIPLE_BLOCK || cmd_index == MMC_CMD_WRITE_MULTIPLE_BLOCK) {
        flag |= SDIO_CMDCON_AUTOSTOP;
    }

    return flag;
}

static int sdio_prepare_dma(struct sdio_host *host, struct mmc_data *data)
{
    struct dma_desc *desc = (struct dma_desc *)host->dma_desc_addr;
    int data_size = data->blocksize * data->blocks;
    unsigned int data_phy_addr;
    unsigned int sdio_des_addr = host->ioaddr + SDIWRDAT;

    if (data->flags == MMC_DATA_READ) {
        flush_dcache_range((unsigned int)data->dest, (unsigned int)data->dest + data_size);
        data_phy_addr = (unsigned int)data->dest;
    } else {
        flush_dcache_range((unsigned int)data->src, (unsigned int)data->src + data_size);
        data_phy_addr = (unsigned int)data->src;
    }

    if (desc == NULL) {
        pr_debug("Invalid DMA address.\n");
        return -EINVAL;
    }

    desc->order_addr_low    = 0x0;
    desc->saddr_low     = VA_TO_PHYS(data_phy_addr);
    desc->daddr     = VA_TO_PHYS(sdio_des_addr);
    desc->length        = data_size / 4;
    desc->step_length   = 0x1;
    desc->step_times    = 0x1;
    if (data->flags == MMC_DATA_READ) {
        desc->cmd  = DMA_CMD_INTMSK;
    } else {
        desc->cmd  = DMA_CMD_INTMSK | DMA_CMD_WRITE;
    }

    dma_desc_start(host);
    return 0;
}

static int sdio_transfer_data(struct sdio_host *host, struct mmc_data *data)
{
    int data_fin     = 0;
    int sdiintmsk    = 0;
    unsigned int err = 0;
    int timeout = 20000000;

    while (data_fin == 0 && timeout--) {
        sdiintmsk = sdio_readl(host, SDIINTMSK);
        data_fin  = SDIO_GET_DATAINT(sdiintmsk);
    }

    if (timeout < 0) {
        pr_debug("wait sdio data irq timeout.\n");
        return -ETIMEDOUT;
    }

    if (sdiintmsk & SDIO_INTMSK_CRCSTA) {
        pr_debug("DATA crc state error.\n");
        err = -ETIMEDOUT;
    } else if (sdiintmsk & SDIO_INTMSK_DATCRC) {
        pr_debug("DATA CRC error.\n");
        err = -1;
    } else if (sdiintmsk & SDIO_INTMSK_PROGERR) {
        pr_debug("DATA program error.\n");
        err = -1;
    } else if (sdiintmsk & SDIO_INTMSK_DATTOUT) {
        pr_debug("DATA timeout.\n");
        err = -ETIMEDOUT;
    }

    sdio_writel(host, SDIINTMSK, sdiintmsk & 0x1e);

    return err;
}

static int ls_mmc_send_cmd(struct udevice *dev, struct mmc_cmd *cmd, struct mmc_data *data)
{
    struct sdio_host *host = dev_get_priv(dev);
    struct mmc *mmc = mmc_get_mmc_dev(dev);

    unsigned int   sdicmdcon;
    unsigned int   sdiintmsk;
    unsigned int   flag = 0;
    unsigned short cmd_index;
    unsigned int   cmd_fin = 0;
    unsigned int   rsp_cmdindex;
    int timeout = 20000;
    int err = 0;

    cmd_index = cmd->cmdidx;
    if (cmd_index == 12) 
        return 0;
    flag = sdio_cmd_prepare_flag(cmd);
    if (data) {
        if (data->blocks > BLOCK_MAX || data->blocks == 0) {
            pr_debug("DATA block argument is invalid\n");
            return -EINVAL;
        }

        misc_set_dma();
        unsigned int sdidatcon = SDIO_DATCON_DTST | SDIO_DATCON_DMAEN | data->blocks;

        if (mmc->bus_width == 4 || mmc->selected_mode & MMC_MODE_4BIT) {
            sdidatcon |= SDIO_DATCON_WIDEMODE;
        } else if (mmc->bus_width == 8 || mmc->selected_mode & MMC_MODE_8BIT)
            sdidatcon |= SDIO_DATCON_WIDEMODE8B;

        sdio_writel(host, SDIBSIZE, data->blocksize);
        sdio_writel(host, SDIDTIMER, 0xffffff);
        sdio_writel(host, SDIDATCON, sdidatcon);
        sdio_writel(host, SDIINTMSK, 0xfffffe);

        err = sdio_prepare_dma(host, data);
        if (err) {
            return err;
        }
    }

    sdio_writel(host, SDICMDARG, cmd->cmdarg);

    sdio_writel(host, SDICMDCON, SDIO_MAKE_CMD(cmd_index, flag));

    while ((cmd_fin == 0) && timeout--) {
        sdiintmsk = sdio_readl(host, SDIINTMSK);
        cmd_fin = SDIO_GET_CMDINT(sdiintmsk);
        if (timeout == 0) {
            pr_debug("CMD check timeout!\n");
            err = -ETIMEDOUT;
            goto out;
        }
    }

    if (sdiintmsk & SDIO_INTMSK_RSPCRC) {
        pr_debug("CMD %d CRC error.\n", cmd_index);
        err = -1;
    } else if (sdiintmsk & SDIO_INTMSK_CMDTOUT) {
        pr_debug("CMD %d TIMEOUT error.\n", cmd_index);
        err = -ETIMEDOUT;
    } else if (sdiintmsk & SDIO_INTMSK_CMDFIN) {
        int i;
        int count = (flag & SDIO_CMDCON_LONGRSP) ? 4 : 1;
        for (i = 0; i < count; i++) {
            cmd->response[i] = sdio_readl(host, SDIRSP0 + i * 4);
            pr_debug("response[%d] = 0x%x\n",i,cmd->response[i]);
        }
    } else {
        pr_debug("CMD %d NOT finished, INT_MSK reg is 0x%x\n", cmd_index, sdiintmsk);
        err = -1;
    }

    if (data) {
        err = sdio_transfer_data(host, data);
        pr_debug("line %d err = %d in send_cmd\n", __LINE__, err);
    }
    dma_ask_valid_done();
out:
    sdio_writel(host, SDIINTMSK, (sdiintmsk & 0x1fe));

    return err;
}

static void sdio_set_clk(struct sdio_host *host, unsigned int clk)
{
    unsigned int clk_pre;

    if (clk == 0) {
        sdio_writel(host, SDICON, SDIO_CON_DIACLK);
        return;
    }
    if (clk == 25000000)
        clk_pre = 2;
    else
        clk_pre = host->sdio_bus_clk / clk;

    sdio_writel(host, SDICON, SDIO_CON_ENCLK);
    sdio_writel(host, SDIPRE, clk_pre | SDIO_PRE_REVCLOCK);
    host->clock = clk;
}

static int ls_mmc_set_ios(struct udevice *dev)
{
    struct sdio_host *host = dev_get_priv(dev);
    struct mmc *mmc = mmc_get_mmc_dev(dev);
    if (mmc->clock != host->clock) {
        sdio_set_clk(host, mmc->clock);
    }
    if (mmc->clk_disable) {
        sdio_set_clk(host, 0);
    }
    return 0;
}

static int ls_mmc_get_cd(struct udevice *dev)
{
    return 1;
}

static const struct dm_mmc_ops ls_mmc_ops = {
    .send_cmd = ls_mmc_send_cmd,
    .set_ios = ls_mmc_set_ios,
    .get_cd = ls_mmc_get_cd,
};

static int ls_mmc_probe(struct udevice *dev)
{
    struct sdio_mmc_plat *plat = dev_get_platdata(dev);
    struct sdio_host *host = dev_get_priv(dev);
    struct mmc_config *cfg = &plat->cfg;
    struct clk clk;
    int ret;

    cfg->name = dev->name;

    ret = clk_get_by_index(dev, 0, &clk);
    if (ret) {
        pr_debug("lsmmc: clk_get_by_index() failed: %d\n", ret);
        return ret;
    }
    host->sdio_bus_clk = clk_get_rate(&clk);
    host->clock = host->sdio_bus_clk;
    host->ioaddr = (phys_addr_t)devfdt_get_addr(dev);
    host->dma_desc_addr = SDIO_DMA_DESCADDR;
    cfg->host_caps = SD_HS | MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_4BIT;
    cfg->voltages =  MMC_VDD_33_34;
    cfg->f_min = host->clock / 255;
    cfg->f_max = host->clock;
    cfg->b_max = BLOCK_MAX; /*block max once a time*/

    misc_set_dma();

    sdio_writel(host, SDICON, SDIO_CON_SOFTRST);
    mdelay(1);
    sdio_writel(host, SDICON, SDIO_CON_ENCLK);
    sdio_writel(host, SDIPRE, 0x800000ff);
    mdelay(1);
    sdio_writel(host, SDIINTEN, 0x3ff);
    return mmc_init(&plat->mmc);
}

static int ls_mmc_bind(struct udevice *dev)
{
    struct sdio_mmc_plat *plat = dev_get_platdata(dev);

    return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct udevice_id ls_mmc_match[] = {
    { .compatible = "ls,ls-mmc" },
    { }
};

U_BOOT_DRIVER(ls_mmc) = {
    .name = "ls_mmc",
    .id = UCLASS_MMC,
    .of_match = ls_mmc_match,
    .bind = ls_mmc_bind,
    .probe = ls_mmc_probe,
    .ops = &ls_mmc_ops,
    .platdata_auto_alloc_size = sizeof(struct sdio_mmc_plat),
    .priv_auto_alloc_size = sizeof(struct sdio_host),
};
