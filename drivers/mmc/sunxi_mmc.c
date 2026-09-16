// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Aaron <leafy.myeh@allwinnertech.com>
 *
 * MMC driver for allwinner sunxi platform.
 *
 * This driver is used by the (ARM) SPL with the legacy MMC interface, and
 * by U-Boot proper using the full DM interface. The actual hardware access
 * code is common, and comes first in this file.
 * The legacy MMC interface implementation comes next, followed by the
 * proper DM_MMC implementation at the end.
 */

#include <cpu_func.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <mmc.h>
#include <clk.h>
#include <reset.h>
#include <asm/cache.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>
#if !CONFIG_IS_ENABLED(DM_MMC)
#include <asm/arch/mmc.h>
#endif
#include <linux/delay.h>
#include <linux/kernel.h>
#include <sunxi_gpio.h>

#include "sunxi_mmc.h"

#ifndef CCM_MMC_CTRL_MODE_SEL_NEW
#define CCM_MMC_CTRL_MODE_SEL_NEW	0
#endif

struct sunxi_mmc_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct sunxi_mmc_priv {
	unsigned mmc_no;
	uint32_t *mclkreg;
	unsigned fatal_err;
	struct gpio_desc cd_gpio;	/* Change Detect GPIO */
	struct sunxi_mmc *reg;
	struct mmc_config cfg;
};

/*
 * All A64 and later MMC controllers feature auto-calibration. This would
 * normally be detected via the compatible string, but we need something
 * which works in the SPL as well.
 */
static bool sunxi_mmc_can_calibrate(void)
{
	return IS_ENABLED(CONFIG_MACH_SUN50I) ||
	       IS_ENABLED(CONFIG_MACH_SUN50I_H5) ||
	       IS_ENABLED(CONFIG_SUN50I_GEN_H6) ||
	       IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2) ||
	       IS_ENABLED(CONFIG_MACH_SUN8I_R40);
}

static int mmc_set_mod_clk(struct sunxi_mmc_priv *priv, unsigned int hz)
{
	unsigned int pll, pll_hz, div, n, oclk_dly, sclk_dly;
	bool new_mode = IS_ENABLED(CONFIG_MMC_SUNXI_HAS_NEW_MODE);
	u32 val = 0;

	/* A83T support new mode only on eMMC */
	if (IS_ENABLED(CONFIG_MACH_SUN8I_A83T) && priv->mmc_no != 2)
		new_mode = false;

	if (hz <= 24000000) {
		pll = CCM_MMC_CTRL_OSCM24;
		pll_hz = 24000000;
	} else {
#ifdef CONFIG_MACH_SUN9I
		pll = CCM_MMC_CTRL_PLL_PERIPH0;
		pll_hz = clock_get_pll4_periph0();
#else
		/*
		 * SoCs since the A64 (H5, H6, H616) actually use the doubled
		 * rate of PLL6/PERIPH0 as an input clock, but compensate for
		 * that with a fixed post-divider of 2 in the mod clock.
		 * This cancels each other out, so for simplicity we just
		 * pretend it's always PLL6 without a post divider here.
		 */
		pll = CCM_MMC_CTRL_PLL6;
		pll_hz = clock_get_pll6();
#endif
		/*
		 * On the D1/R528/T113 mux source 1 refers to PLL_PERIPH0(1x),
		 * like for the older SoCs. However we still have the hidden
		 * divider of 2x, so compensate for that here.
		 */
		if (IS_ENABLED(CONFIG_MACH_SUN8I_R528))
			pll_hz /= 2;

		/*
		 * The A523/T527 uses PERIPH0_400M as the MMC0/1 input clock,
		 * and PERIPH0_800M for MMC2. There is also the hidden divider
		 * of 2. The clock code reports 600 MHz for PERIPH0.
		 * Adjust the calculation accordingly: 600 * hidden2 / 3 for
		 * MMC0/1, and 600 * hidden2 / 3 * 2 for MMC2.
		 */
		if (IS_ENABLED(CONFIG_MACH_SUN55I_A523)) {
			pll_hz /= 3;
			if (priv->mmc_no == 2)
				pll_hz *= 2;
		}
		/*
		 * The H713 keeps PLL_PERIPH0(2x) at mux 1 for MMC0/1, but its
		 * MMC2 (the eMMC) has PERIPH0_800M there, with the hidden
		 * divider of 2: 400 MHz effective, not the 600 MHz assumed
		 * above. Measured 16.09.2026 on the HY310: asked for 52 MHz,
		 * the register held M = 12 and the eMMC ran at 33 MHz.
		 */
		if (IS_ENABLED(CONFIG_MACH_SUN50I_H713) && priv->mmc_no == 2)
			pll_hz = pll_hz * 2 / 3;
	}

	div = pll_hz / hz;
	if (pll_hz % hz)
		div++;

	n = 0;
	while (div > 16) {
		n++;
		div = (div + 1) / 2;
	}

	if (n > 3) {
		printf("mmc %u error cannot set clock to %u\n", priv->mmc_no,
		       hz);
		return -1;
	}

	/* determine delays */
	if (hz <= 400000) {
		oclk_dly = 0;
		sclk_dly = 0;
	} else if (hz <= 25000000) {
		oclk_dly = 0;
		sclk_dly = 5;
	} else {
		if (IS_ENABLED(CONFIG_MACH_SUN9I)) {
			if (hz <= 52000000)
				oclk_dly = 5;
			else
				oclk_dly = 2;
		} else {
			if (hz <= 52000000)
				oclk_dly = 3;
			else
				oclk_dly = 1;
		}
		sclk_dly = 4;
	}

	if (new_mode) {
		val |= CCM_MMC_CTRL_MODE_SEL_NEW;
		setbits_le32(&priv->reg->ntsr, SUNXI_MMC_NTSR_MODE_SEL_NEW);
	}

	if (!sunxi_mmc_can_calibrate()) {
		/*
		 * Use hardcoded delay values if controller doesn't support
		 * calibration
		 */
		val = CCM_MMC_CTRL_OCLK_DLY(oclk_dly) |
			CCM_MMC_CTRL_SCLK_DLY(sclk_dly);
	}

	/* The A523 has a second divider, not a shift. */
	if (IS_ENABLED(CONFIG_MACH_SUN55I_A523))
		n = (1U << n) - 1;

	writel(CCM_MMC_CTRL_ENABLE| pll | CCM_MMC_CTRL_N(n) |
	       CCM_MMC_CTRL_M(div) | val, priv->mclkreg);

	debug("mmc %u set mod-clk req %u parent %u n %u m %u rate %u\n",
	      priv->mmc_no, hz, pll_hz, 1u << n, div, pll_hz / (1u << n) / div);

	return 0;
}

static int mmc_update_clk(struct sunxi_mmc_priv *priv)
{
	unsigned int cmd;
	unsigned timeout_msecs = 2000;
	unsigned long start = get_timer(0);

	cmd = SUNXI_MMC_CMD_START |
	      SUNXI_MMC_CMD_UPCLK_ONLY |
	      SUNXI_MMC_CMD_WAIT_PRE_OVER;

	writel(cmd, &priv->reg->cmd);
	while (readl(&priv->reg->cmd) & SUNXI_MMC_CMD_START) {
		if (get_timer(start) > timeout_msecs)
			return -1;
	}

	/* clock update sets various irq status bits, clear these */
	writel(readl(&priv->reg->rint), &priv->reg->rint);

	return 0;
}

static int mmc_config_clock(struct sunxi_mmc_priv *priv, struct mmc *mmc)
{
	unsigned rval = readl(&priv->reg->clkcr);

	/* Disable Clock */
	rval &= ~SUNXI_MMC_CLK_ENABLE;
	writel(rval, &priv->reg->clkcr);
	if (mmc_update_clk(priv))
		return -1;

	/* Set mod_clk to new rate */
	if (mmc_set_mod_clk(priv, mmc->clock))
		return -1;

	/* Clear internal divider */
	rval &= ~SUNXI_MMC_CLK_DIVIDER_MASK;
	writel(rval, &priv->reg->clkcr);

#if defined(CONFIG_SUNXI_GEN_SUN6I) || defined(CONFIG_SUN50I_GEN_H6) || defined(CONFIG_SUNXI_GEN_NCAT2)
	/* A64 supports calibration of delays on MMC controller and we
	 * have to set delay of zero before starting calibration.
	 * Allwinner BSP driver sets a delay only in the case of
	 * using HS400 which is not supported by mainline U-Boot or
	 * Linux at the moment
	 */
	if (sunxi_mmc_can_calibrate())
		writel(SUNXI_MMC_CAL_DL_SW_EN, &priv->reg->samp_dl);
#endif

	/* Re-enable Clock */
	rval |= SUNXI_MMC_CLK_ENABLE;
	writel(rval, &priv->reg->clkcr);
	if (mmc_update_clk(priv))
		return -1;

	return 0;
}

static int sunxi_mmc_set_ios_common(struct sunxi_mmc_priv *priv,
				    struct mmc *mmc)
{
	debug("set ios: bus_width: %x, clock: %d\n",
	      mmc->bus_width, mmc->clock);

	/* Change clock first */
	if (mmc->clock && mmc_config_clock(priv, mmc) != 0) {
		priv->fatal_err = 1;
		return -EINVAL;
	}

	/* Change bus width */
	if (mmc->bus_width == 8)
		writel(0x2, &priv->reg->width);
	else if (mmc->bus_width == 4)
		writel(0x1, &priv->reg->width);
	else
		writel(0x0, &priv->reg->width);

	return 0;
}

static int mmc_trans_data_by_cpu(struct sunxi_mmc_priv *priv, struct mmc *mmc,
				 struct mmc_data *data)
{
	const int reading = !!(data->flags & MMC_DATA_READ);
	const uint32_t status_bit = reading ? SUNXI_MMC_STATUS_FIFO_EMPTY :
					      SUNXI_MMC_STATUS_FIFO_FULL;
	unsigned i;
	unsigned *buff = (unsigned int *)(reading ? data->dest : data->src);
	unsigned word_cnt = (data->blocksize * data->blocks) >> 2;
	unsigned timeout_msecs = word_cnt >> 6;
	uint32_t status;
	unsigned long  start;

	if (timeout_msecs < 2000)
		timeout_msecs = 2000;

	/* Always read / write data through the CPU */
	setbits_le32(&priv->reg->gctrl, SUNXI_MMC_GCTRL_ACCESS_BY_AHB);

	start = get_timer(0);

	for (i = 0; i < word_cnt;) {
		unsigned int in_fifo;

		while ((status = readl(&priv->reg->status)) & status_bit) {
			if (get_timer(start) > timeout_msecs)
				return -1;
		}

		/*
		 * For writing we do not easily know the FIFO size, so have
		 * to check the FIFO status after every word written.
		 * TODO: For optimisation we could work out a minimum FIFO
		 * size across all SoCs, and use that together with the current
		 * fill level to write chunks of words.
		 */
		if (!reading) {
			writel(buff[i++], &priv->reg->fifo);
			continue;
		}

		/*
		 * The status register holds the current FIFO level, so we
		 * can be sure to collect as many words from the FIFO
		 * register without checking the status register after every
		 * read. That saves half of the costly MMIO reads, effectively
		 * doubling the read performance.
		 * Some SoCs (A20) report a level of 0 if the FIFO is
		 * completely full (value masked out?). Use a safe minimal
		 * FIFO size in this case.
		 */
		in_fifo = SUNXI_MMC_STATUS_FIFO_LEVEL(status);
		if (in_fifo == 0 && (status & SUNXI_MMC_STATUS_FIFO_FULL))
			in_fifo = 32;
		for (; in_fifo > 0; in_fifo--)
			buff[i++] = readl_relaxed(&priv->reg->fifo);
		dmb();
	}

	return 0;
}

#if CONFIG_IS_ENABLED(MMC_SUNXI_IDMA)

/*
 * Internal DMA (IDMAC).  The controller walks a chain of 16 byte descriptors
 * in main memory and moves the data itself, which takes the CPU out of the
 * per-word FIFO loop of mmc_trans_data_by_cpu().
 *
 * Everything below follows two sources that agree on this hardware:
 *   - Linux sunxi-mmc (sunxi_mmc_init_idma_des(), sunxi_mmc_start_dma()),
 *     the driver that runs this eMMC at HS400 on the same board;
 *   - the vendor U-Boot 2018.05 of the HY310 (hy310-u-boot.fex, sha256
 *     b8f40b86fe726145afbb50c067a3fda6b1ee7898340d48af5c5baf6d647f3a88,
 *     load address 0x4a000000), function sunxi_mmc_do_send_cmd_common at
 *     0x4a01db68, whose descriptor loop is at 0x4a01dfd4 and whose
 *     start-up sequence runs from 0x4a01e23e to 0x4a01e346.
 * Where they differ, the comment says so.
 */

/*
 * Bytes per descriptor.  The vendor U-Boot splits every transfer into
 * 4096 byte chunks (0x4a01e26a is bytecnt >> 12, 0x4a01e266 the
 * remainder), which is
 * below the 8192 byte limit that Linux allows for this controller
 * (sun50i_h713_emmc_cfg, .idma_des_size_bits = 13).  Stay with the vendor.
 */
#define SUNXI_MMC_IDMA_MAX_LEN		4096

/*
 * Length of the descriptor chain.  256 descriptors cover a 1 MiB transfer
 * and cost 4 KiB of .bss; the vendor keeps a 256 KiB chain
 * (memalign(64, 0x40000) at 0x4a01e5fe), which we do not need because
 * b_max caps a single command at the size of our chain.
 */
#define SUNXI_MMC_IDMA_DESCS		256
#define SUNXI_MMC_IDMA_MAX_BYTES	\
	((unsigned int)SUNXI_MMC_IDMA_MAX_LEN * SUNXI_MMC_IDMA_DESCS)
#define SUNXI_MMC_IDMA_MAX_BLOCKS	(SUNXI_MMC_IDMA_MAX_BYTES / 512)

/*
 * Descriptor and buffer addresses are stored as word addresses, i.e. shifted
 * right by two.  Sources: Linux sun50i_h713_emmc_cfg, .idma_des_shift = 2;
 * vendor U-Boot at 0x4a01e02c, 0x4a01e064 and 0x4a01e31e, which shifts the
 * buffer
 * and the next-descriptor pointer for every SoC revision above 0x000502ff
 * (the H713 is one of them).
 */
#define SUNXI_MMC_IDMA_DES_SHIFT	2
#define SUNXI_MMC_IDMA_MAX_ADDR		\
	(0x100000000ULL << SUNXI_MMC_IDMA_DES_SHIFT)

/*
 * U-Boot runs one MMC transfer at a time, so a single chain serves every
 * controller.  It has to own whole cache lines, hence the alignment.
 */
static struct sunxi_idma_desc
sunxi_mmc_idma_chain[SUNXI_MMC_IDMA_DESCS] __aligned(ARCH_DMA_MINALIGN);

static uintptr_t sunxi_mmc_data_buf(struct mmc_data *data)
{
	if (data->flags & MMC_DATA_READ)
		return (uintptr_t)data->dest;

	return (uintptr_t)data->src;
}

static u32 sunxi_mmc_dma_addr(uintptr_t addr)
{
	return (u32)((u64)addr >> SUNXI_MMC_IDMA_DES_SHIFT);
}

/*
 * Cache maintenance, but only while there is a cache to maintain.
 *
 * arch/arm/cpu/armv8/cache_v8.c hands flush_dcache_range() and
 * invalidate_dcache_range() straight to the "dc civac" and "dc ivac" loops of
 * cache.S without ever looking at dcache_status(); they walk one cache line at
 * a time no matter what the MMU is doing.  With the cache off each of those
 * instructions is a no-op - nothing can be allocated into a cache that is not
 * looked up - so the loop is pure cost: the SPL's largest read alone is 13792
 * lines, walked twice, out of code that is itself fetched uncached.
 *
 * In the SPL the cache is off for the whole of the SPL's life: no mmu_setup()
 * and no dcache_enable() are linked into spl/u-boot-spl (M3 report 2), and
 * with the MMU off every access is Device-nGnRnE, which is exactly why the
 * DMA needs no maintenance there.  In U-Boot proper the cache is on and both
 * calls happen as before.
 */
static void sunxi_mmc_dma_flush(uintptr_t start, uintptr_t end)
{
	if (dcache_status())
		flush_dcache_range(start, end);
}

static void sunxi_mmc_dma_invalidate(uintptr_t start, uintptr_t end)
{
	if (dcache_status())
		invalidate_dcache_range(start, end);
}

static bool sunxi_mmc_dma_capable(struct mmc_data *data, unsigned int bytecnt)
{
	uintptr_t buf = sunxi_mmc_data_buf(data);

	/*
	 * The vendor U-Boot takes the PIO path for 64 bytes and less
	 * (0x4a01e23e compares against 0x40, 0x4a01e244 branches to the PIO
	 * path), the IDMA above that, so every
	 * block sized transfer is a DMA transfer.
	 */
	if (bytecnt <= 64 || bytecnt > SUNXI_MMC_IDMA_MAX_BYTES)
		return false;

	/*
	 * Cache maintenance works on whole cache lines.  A buffer that does
	 * not start and end on a line boundary shares its first or last line
	 * with someone else's data, and flushing or invalidating would take
	 * that data with it - such a transfer stays on the PIO path.
	 */
	if ((buf | bytecnt) & (ARCH_DMA_MINALIGN - 1))
		return false;

	/* The descriptor holds a word address, so 34 bits of range. */
	if ((u64)buf + bytecnt > SUNXI_MMC_IDMA_MAX_ADDR)
		return false;

	return true;
}

static int mmc_start_dma(struct sunxi_mmc_priv *priv, struct mmc_data *data,
			 unsigned int bytecnt)
{
	struct sunxi_idma_desc *chain = sunxi_mmc_idma_chain;
	uintptr_t buf = sunxi_mmc_data_buf(data);
	unsigned int count = DIV_ROUND_UP(bytecnt, SUNXI_MMC_IDMA_MAX_LEN);
	unsigned int last = bytecnt - (count - 1) * SUNXI_MMC_IDMA_MAX_LEN;
	unsigned int i, timeout = 1000;
	u32 val;

	for (i = 0; i < count; i++) {
		u32 config = SUNXI_MMC_IDMA_DES0_CH |
			     SUNXI_MMC_IDMA_DES0_DIC |
			     SUNXI_MMC_IDMA_DES0_OWN;

		if (i == 0)
			config |= SUNXI_MMC_IDMA_DES0_FD;
		if (i == count - 1)
			config = (config & ~SUNXI_MMC_IDMA_DES0_DIC) |
				 SUNXI_MMC_IDMA_DES0_LD |
				 SUNXI_MMC_IDMA_DES0_ER;

		chain[i].config = cpu_to_le32(config);
		chain[i].buf_size = cpu_to_le32(i == count - 1 ? last :
						SUNXI_MMC_IDMA_MAX_LEN);
		chain[i].buf_addr = cpu_to_le32(sunxi_mmc_dma_addr(
				buf + (uintptr_t)i * SUNXI_MMC_IDMA_MAX_LEN));
		chain[i].next_desc = (i == count - 1) ? 0 :
			cpu_to_le32(sunxi_mmc_dma_addr(
					(uintptr_t)&chain[i + 1]));
	}

	/*
	 * The engine reads the descriptors and, for a write, the payload out
	 * of main memory, so both have to be out of the caches first.  For a
	 * read the flush evicts dirty lines that would otherwise land on top
	 * of what the engine wrote.  sunxi_mmc_dma_capable() has already
	 * established that the payload owns whole cache lines.
	 */
	sunxi_mmc_dma_flush(buf, buf + bytecnt);
	sunxi_mmc_dma_flush((uintptr_t)chain,
			    (uintptr_t)chain +
			    roundup(count * sizeof(*chain), ARCH_DMA_MINALIGN));

	/* Take the data path off the AHB FIFO and hand it to the IDMA. */
	clrbits_le32(&priv->reg->gctrl, SUNXI_MMC_GCTRL_ACCESS_BY_AHB);
	setbits_le32(&priv->reg->gctrl, SUNXI_MMC_GCTRL_DMA_RESET |
					SUNXI_MMC_GCTRL_DMA_ENABLE);

	/* Vendor 0x4a01e2bc, polled at 0x4a01e2c2; Linux has
	 * sunxi_mmc_reset_dmactl().
	 */
	writel(SUNXI_MMC_IDMAC_RESET, &priv->reg->dmac);
	while (readl(&priv->reg->dmac) & SUNXI_MMC_IDMAC_RESET) {
		if (!timeout--) {
			debug("mmc %u: IDMA reset timeout\n", priv->mmc_no);
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	/* Vendor 0x4a01e2d6 writes 0x82, Linux writes the same two bits. */
	writel(SUNXI_MMC_IDMAC_ENABLE | SUNXI_MMC_IDMAC_FIXBURST,
	       &priv->reg->dmac);

	/*
	 * We poll the raw interrupt status, so the IDMA interrupt is never
	 * taken; the vendor arms it all the same (0x4a01e2da to 0x4a01e2fa) and
	 * so do we,
	 * because the status bit it gates is what the engine sets on
	 * completion.
	 */
	val = readl(&priv->reg->idie) &
	      ~(SUNXI_MMC_IDIE_TXIRQ | SUNXI_MMC_IDIE_RXIRQ);
	val |= (data->flags & MMC_DATA_WRITE) ? SUNXI_MMC_IDIE_TXIRQ :
						SUNXI_MMC_IDIE_RXIRQ;
	writel(val, &priv->reg->idie);

	/* Vendor 0x4a01e31e and 0x4a01e326; Linux in init_host(). */
	writel(sunxi_mmc_dma_addr((uintptr_t)chain), &priv->reg->dlba);

	return 0;
}

static int mmc_finish_dma(struct sunxi_mmc_priv *priv, struct mmc_data *data,
			  unsigned int bytecnt)
{
	uintptr_t buf = sunxi_mmc_data_buf(data);
	u32 idst = readl(&priv->reg->idst);

	/* Vendor 0x4a01dc10: status is write-one-to-clear, then stop. */
	writel(idst, &priv->reg->idst);
	writel(0, &priv->reg->idie);
	writel(0, &priv->reg->dmac);
	clrbits_le32(&priv->reg->gctrl, SUNXI_MMC_GCTRL_DMA_ENABLE);

	/*
	 * Vendor 0x4a01df28 invalidates the destination after a read, so the
	 * CPU sees what the engine put there instead of a stale line.
	 */
	if (data->flags & MMC_DATA_READ)
		sunxi_mmc_dma_invalidate(buf, buf + bytecnt);

	if (idst & SUNXI_MMC_IDST_ERROR) {
		debug("mmc %u: IDMA error, idst %08x\n", priv->mmc_no, idst);
		return -EIO;
	}

	return 0;
}

/*
 * The fixed 120 ms of the PIO path only ever had to cover the tail of a
 * transfer the CPU had already moved.  With the IDMA the whole transfer
 * happens inside this wait, so it has to scale: one millisecond per 256
 * bytes, which is the budget mmc_trans_data_by_cpu() gives itself
 * (word_cnt >> 6), but never less than the 120 ms of before.  An error
 * still leaves mmc_rint_wait() immediately; this only bounds a silent hang.
 */
static uint sunxi_mmc_dma_timeout(unsigned int bytecnt)
{
	uint timeout_msecs = bytecnt >> 8;

	return timeout_msecs < 120 ? 120 : timeout_msecs;
}

#else /* !MMC_SUNXI_IDMA */

/* Without the IDMA nothing caps a single command but the MMC core does. */
#define SUNXI_MMC_IDMA_MAX_BLOCKS	CONFIG_SYS_MMC_MAX_BLK_COUNT

static bool sunxi_mmc_dma_capable(struct mmc_data *data, unsigned int bytecnt)
{
	return false;
}

static int mmc_start_dma(struct sunxi_mmc_priv *priv, struct mmc_data *data,
			 unsigned int bytecnt)
{
	return -ENOSYS;
}

static int mmc_finish_dma(struct sunxi_mmc_priv *priv, struct mmc_data *data,
			  unsigned int bytecnt)
{
	return 0;
}

static uint sunxi_mmc_dma_timeout(unsigned int bytecnt)
{
	return 120;
}

#endif /* MMC_SUNXI_IDMA */

static int mmc_rint_wait(struct sunxi_mmc_priv *priv, struct mmc *mmc,
			 uint timeout_msecs, uint done_bit, const char *what)
{
	unsigned int status;
	unsigned long start = get_timer(0);

	do {
		status = readl(&priv->reg->rint);
		if ((get_timer(start) > timeout_msecs) ||
		    (status & SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT)) {
			debug("%s timeout %x\n", what,
			      status & SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT);
			return -ETIMEDOUT;
		}
	} while (!(status & done_bit));

	return 0;
}

/*
 * One attempt at one command.  @allow_dma is false for the PIO repeat that
 * sunxi_mmc_send_cmd_common() makes after an IDMA fault; *@dma_fault comes
 * back true only when the engine, and not the card, is what went wrong.
 */
static int sunxi_mmc_send_cmd_once(struct sunxi_mmc_priv *priv,
				   struct mmc *mmc, struct mmc_cmd *cmd,
				   struct mmc_data *data, bool allow_dma,
				   bool *dma_fault)
{
	unsigned int cmdval = SUNXI_MMC_CMD_START;
	unsigned int timeout_msecs;
	int error = 0;
	unsigned int status = 0;
	unsigned int bytecnt = 0;
	bool use_dma = false;

	if (priv->fatal_err)
		return -1;
	if (cmd->resp_type & MMC_RSP_BUSY)
		debug("mmc cmd %d check rsp busy\n", cmd->cmdidx);
	if (cmd->cmdidx == 12)
		return 0;

	if (!cmd->cmdidx)
		cmdval |= SUNXI_MMC_CMD_SEND_INIT_SEQ;
	if (cmd->resp_type & MMC_RSP_PRESENT)
		cmdval |= SUNXI_MMC_CMD_RESP_EXPIRE;
	if (cmd->resp_type & MMC_RSP_136)
		cmdval |= SUNXI_MMC_CMD_LONG_RESPONSE;
	if (cmd->resp_type & MMC_RSP_CRC)
		cmdval |= SUNXI_MMC_CMD_CHK_RESPONSE_CRC;

	if (data) {
		if ((u32)(long)data->dest & 0x3) {
			error = -1;
			goto out;
		}

		cmdval |= SUNXI_MMC_CMD_DATA_EXPIRE|SUNXI_MMC_CMD_WAIT_PRE_OVER;
		if (data->flags & MMC_DATA_WRITE)
			cmdval |= SUNXI_MMC_CMD_WRITE;
		if (data->blocks > 1)
			cmdval |= SUNXI_MMC_CMD_AUTO_STOP;
		writel(data->blocksize, &priv->reg->blksz);
		writel(data->blocks * data->blocksize, &priv->reg->bytecnt);
	}

	debug("mmc %d, cmd %d(0x%08x), arg 0x%08x\n", priv->mmc_no,
	      cmd->cmdidx, cmdval | cmd->cmdidx, cmd->cmdarg);
	writel(cmd->cmdarg, &priv->reg->arg);

	if (!data)
		writel(cmdval | cmd->cmdidx, &priv->reg->cmd);

	/*
	 * transfer data and check status
	 * STATREG[2] : FIFO empty
	 * STATREG[3] : FIFO full
	 */
	if (data) {
		int ret = 0;

		bytecnt = data->blocksize * data->blocks;
		debug("trans data %d bytes\n", bytecnt);
		use_dma = allow_dma && sunxi_mmc_dma_capable(data, bytecnt);
		if (use_dma) {
			/*
			 * The descriptor chain and the DMA registers have to
			 * stand before the command starts the transfer, as
			 * they do in the vendor U-Boot (0x4a01e24c to
			 * 0x4a01e346, the command write is the last one).
			 */
			ret = mmc_start_dma(priv, data, bytecnt);
			if (ret) {
				/*
				 * Leave use_dma set: the engine is half
				 * programmed and mmc_finish_dma() below is
				 * what puts it back.  No command has been
				 * written yet, so the PIO repeat is a clean
				 * first attempt.
				 */
				*dma_fault = true;
				error = ret;
				goto out;
			}
			writel(cmdval | cmd->cmdidx, &priv->reg->cmd);
		} else {
			writel(cmdval | cmd->cmdidx, &priv->reg->cmd);
			ret = mmc_trans_data_by_cpu(priv, mmc, data);
			if (ret) {
				error = readl(&priv->reg->rint) &
					SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT;
				error = -ETIMEDOUT;
				goto out;
			}
		}
	}

	error = mmc_rint_wait(priv, mmc, 1000, SUNXI_MMC_RINT_COMMAND_DONE,
			      "cmd");
	if (error)
		goto out;

	if (data) {
		timeout_msecs = use_dma ? sunxi_mmc_dma_timeout(bytecnt) : 120;
		debug("cacl timeout %x msec\n", timeout_msecs);
		error = mmc_rint_wait(priv, mmc, timeout_msecs,
				      data->blocks > 1 ?
				      SUNXI_MMC_RINT_AUTO_COMMAND_DONE :
				      SUNXI_MMC_RINT_DATA_OVER,
				      "data");
		if (error) {
			/*
			 * The data phase ran out of time and the controller
			 * reports nothing wrong with the card - then it is the
			 * engine that did not finish, and the CPU can try.  A
			 * card error (CRC, timeout) is the card's answer and
			 * repeating it on the PIO path would only ask twice;
			 * the tuning sweep of HS200 lives on exactly that
			 * answer 63 times out of 64.
			 */
			if (use_dma && !(readl(&priv->reg->rint) &
					 SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT))
				*dma_fault = true;
			goto out;
		}
	}

	if (cmd->resp_type & MMC_RSP_BUSY) {
		unsigned long start = get_timer(0);
		timeout_msecs = 2000;

		do {
			status = readl(&priv->reg->status);
			if (get_timer(start) > timeout_msecs) {
				debug("busy timeout\n");
				error = -ETIMEDOUT;
				goto out;
			}
		} while (status & SUNXI_MMC_STATUS_CARD_DATA_BUSY);
	}

	if (cmd->resp_type & MMC_RSP_136) {
		cmd->response[0] = readl(&priv->reg->resp3);
		cmd->response[1] = readl(&priv->reg->resp2);
		cmd->response[2] = readl(&priv->reg->resp1);
		cmd->response[3] = readl(&priv->reg->resp0);
		debug("mmc resp 0x%08x 0x%08x 0x%08x 0x%08x\n",
		      cmd->response[3], cmd->response[2],
		      cmd->response[1], cmd->response[0]);
	} else {
		cmd->response[0] = readl(&priv->reg->resp0);
		debug("mmc resp 0x%08x\n", cmd->response[0]);
	}
out:
	if (use_dma) {
		int dma_error = mmc_finish_dma(priv, data, bytecnt);

		/* mmc_finish_dma() only fails on the engine's own error bits. */
		if (dma_error) {
			*dma_fault = true;
			if (!error)
				error = dma_error;
		}
	}

	if (error < 0) {
		writel(SUNXI_MMC_GCTRL_RESET, &priv->reg->gctrl);
		mmc_update_clk(priv);
	}
	writel(0xffffffff, &priv->reg->rint);
	writel(readl(&priv->reg->gctrl) | SUNXI_MMC_GCTRL_FIFO_RESET,
	       &priv->reg->gctrl);

	return error;
}

static int sunxi_mmc_send_cmd_common(struct sunxi_mmc_priv *priv,
				     struct mmc *mmc, struct mmc_cmd *cmd,
				     struct mmc_data *data)
{
	bool dma_fault = false;
	int error;

	error = sunxi_mmc_send_cmd_once(priv, mmc, cmd, data, true, &dma_fault);
	if (!error || !dma_fault)
		return error;

	/*
	 * The IDMA engine, not the card, is what failed, and the path out of
	 * sunxi_mmc_send_cmd_once() has already reset the controller and the
	 * engine.  Do the transfer again on the CPU: in U-Boot proper that
	 * costs a slow command, in the SPL it is the difference between a
	 * device that boots and a device that needs FEL.  Say so on the UART
	 * either way - a boot that quietly takes ten times as long is a bug
	 * report nobody can make sense of.
	 */
	printf("mmc %u: IDMA failed (%d), repeating the transfer on the CPU\n",
	       priv->mmc_no, error);

	return sunxi_mmc_send_cmd_once(priv, mmc, cmd, data, false, &dma_fault);
}

static void sunxi_mmc_reset(void *regs)
{
	/* Reset controller */
	writel(SUNXI_MMC_GCTRL_RESET, regs + SUNXI_MMC_GCTRL);
	udelay(1000);

	if (IS_ENABLED(CONFIG_SUN50I_GEN_H6) || IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2)) {
		/* Reset card */
		writel(SUNXI_MMC_HWRST_ASSERT, regs + SUNXI_MMC_HWRST);
		udelay(10);
		writel(SUNXI_MMC_HWRST_DEASSERT, regs + SUNXI_MMC_HWRST);
		udelay(300);

		/* Setup FIFO R/W threshold. Needed on H616. */
		writel(SUNXI_MMC_THLDC_READ_THLD(512) |
		       SUNXI_MMC_THLDC_WRITE_EN |
		       SUNXI_MMC_THLDC_READ_EN, regs + SUNXI_MMC_THLDC);
	}
}

/* non-DM code here is used by the (ARM) SPL only */

#if !CONFIG_IS_ENABLED(DM_MMC)
/* support 4 mmc hosts */
struct sunxi_mmc_priv mmc_host[4];

static int mmc_resource_init(int sdc_no)
{
	struct sunxi_mmc_priv *priv = &mmc_host[sdc_no];
	void *ccm = (void *)SUNXI_CCM_BASE;

	debug("init mmc %d resource\n", sdc_no);

	switch (sdc_no) {
	case 0:
		priv->reg = (struct sunxi_mmc *)SUNXI_MMC0_BASE;
		priv->mclkreg = ccm + CCU_MMC0_CLK_CFG;
		break;
	case 1:
		priv->reg = (struct sunxi_mmc *)SUNXI_MMC1_BASE;
		priv->mclkreg = ccm + CCU_MMC1_CLK_CFG;
		break;
#ifdef SUNXI_MMC2_BASE
	case 2:
		priv->reg = (struct sunxi_mmc *)SUNXI_MMC2_BASE;
		priv->mclkreg = ccm + CCU_MMC2_CLK_CFG;
		break;
#endif
#ifdef SUNXI_MMC3_BASE
	case 3:
		priv->reg = (struct sunxi_mmc *)SUNXI_MMC3_BASE;
		priv->mclkreg = ccm + CCU_MMC3_CLK_CFG;
		break;
#endif
	default:
		printf("Wrong mmc number %d\n", sdc_no);
		return -1;
	}
	priv->mmc_no = sdc_no;

	return 0;
}

static int sunxi_mmc_core_init(struct mmc *mmc)
{
	struct sunxi_mmc_priv *priv = mmc->priv;

	sunxi_mmc_reset(priv->reg);

	return 0;
}

static int sunxi_mmc_set_ios_legacy(struct mmc *mmc)
{
	struct sunxi_mmc_priv *priv = mmc->priv;

	return sunxi_mmc_set_ios_common(priv, mmc);
}

static int sunxi_mmc_send_cmd_legacy(struct mmc *mmc, struct mmc_cmd *cmd,
				     struct mmc_data *data)
{
	struct sunxi_mmc_priv *priv = mmc->priv;

	return sunxi_mmc_send_cmd_common(priv, mmc, cmd, data);
}

/* .getcd is not needed by the SPL */
static const struct mmc_ops sunxi_mmc_ops = {
	.send_cmd	= sunxi_mmc_send_cmd_legacy,
	.set_ios	= sunxi_mmc_set_ios_legacy,
	.init		= sunxi_mmc_core_init,
};

struct mmc *sunxi_mmc_init(int sdc_no)
{
	void *ccm = (void *)SUNXI_CCM_BASE;
	struct sunxi_mmc_priv *priv = &mmc_host[sdc_no];
	struct mmc_config *cfg = &priv->cfg;
	int ret;

	memset(priv, '\0', sizeof(struct sunxi_mmc_priv));

	cfg->name = "SUNXI SD/MMC";
	cfg->ops  = &sunxi_mmc_ops;

	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	cfg->host_caps = MMC_MODE_4BIT;

	if ((IS_ENABLED(CONFIG_MACH_SUN50I) || IS_ENABLED(CONFIG_MACH_SUN8I) ||
	    IS_ENABLED(CONFIG_SUN50I_GEN_H6) || IS_ENABLED(CONFIG_MACH_SUN55I_A523)) &&
	    (sdc_no == 2))
		cfg->host_caps = MMC_MODE_8BIT;

	cfg->host_caps |= MMC_MODE_HS_52MHz | MMC_MODE_HS;
	/*
	 * One command has to fit into one descriptor chain, as in the DM probe
	 * below.  Without the IDMA this is CONFIG_SYS_MMC_MAX_BLK_COUNT again
	 * and nothing changes.  It matters here because the SPL's largest read
	 * is the U-Boot image: 1724 blocks today, but a chain that cannot hold
	 * the whole command would take that read back to the PIO path without
	 * saying so, and mmc_bread() splits at b_max for free.
	 */
	cfg->b_max = min_t(unsigned int, CONFIG_SYS_MMC_MAX_BLK_COUNT,
			   SUNXI_MMC_IDMA_MAX_BLOCKS);

	cfg->f_min = 400000;
	cfg->f_max = 52000000;

	if (mmc_resource_init(sdc_no) != 0)
		return NULL;

	/* config ahb clock */
	debug("init mmc %d clock and io\n", sdc_no);
#if !defined(CONFIG_SUN50I_GEN_H6) && !defined(CONFIG_SUNXI_GEN_NCAT2)
	setbits_le32(ccm + CCU_AHB_GATE0, 1 << AHB_GATE_OFFSET_MMC(sdc_no));

#ifdef CONFIG_SUNXI_GEN_SUN6I
	/* unassert reset */
	setbits_le32(ccm + CCU_AHB_RESET0_CFG, 1 << AHB_RESET_OFFSET_MMC(sdc_no));
#endif
#if defined(CONFIG_MACH_SUN9I)
	/* sun9i has a mmc-common module, also set the gate and reset there */
	writel(SUNXI_MMC_COMMON_CLK_GATE | SUNXI_MMC_COMMON_RESET,
	       SUNXI_MMC_COMMON_BASE + 4 * sdc_no);
#endif
#else /* CONFIG_SUN50I_GEN_H6 */
	/*
	 * The H713 SMHC bus interface locks up (register accesses hang
	 * the CPU) if the bus clock gate opens while the module is still
	 * held in reset, so release the reset first, as the BROM does.
	 */
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H713)) {
		setbits_le32(ccm + CCU_H6_MMC_GATE_RESET,
			     1 << (RESET_SHIFT + sdc_no));
		setbits_le32(ccm + CCU_H6_MMC_GATE_RESET, 1 << sdc_no);
	} else {
		setbits_le32(ccm + CCU_H6_MMC_GATE_RESET, 1 << sdc_no);
		/* unassert reset */
		setbits_le32(ccm + CCU_H6_MMC_GATE_RESET,
			     1 << (RESET_SHIFT + sdc_no));
	}
#endif
	ret = mmc_set_mod_clk(priv, 24000000);
	if (ret)
		return NULL;

	return mmc_create(cfg, priv);
}

#else /* CONFIG_DM_MMC code below, as used by U-Boot proper */

static int sunxi_mmc_set_ios(struct udevice *dev)
{
	struct sunxi_mmc_plat *plat = dev_get_plat(dev);
	struct sunxi_mmc_priv *priv = dev_get_priv(dev);

	return sunxi_mmc_set_ios_common(priv, &plat->mmc);
}

static int sunxi_mmc_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	struct sunxi_mmc_plat *plat = dev_get_plat(dev);
	struct sunxi_mmc_priv *priv = dev_get_priv(dev);

	return sunxi_mmc_send_cmd_common(priv, &plat->mmc, cmd, data);
}

static int sunxi_mmc_getcd(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct sunxi_mmc_priv *priv = dev_get_priv(dev);

	/* If polling, assume that the card is always present. */
	if ((mmc->cfg->host_caps & MMC_CAP_NONREMOVABLE) ||
	    (mmc->cfg->host_caps & MMC_CAP_NEEDS_POLL))
		return 1;

	if (dm_gpio_is_valid(&priv->cd_gpio)) {
		int cd_state = dm_gpio_get_value(&priv->cd_gpio);

		if (mmc->cfg->host_caps & MMC_CAP_CD_ACTIVE_HIGH)
			return !cd_state;
		else
			return cd_state;
	}
	return 1;
}

#if CONFIG_IS_ENABLED(MMC_SUPPORTS_TUNING)
/*
 * The MMC core refuses HS200 and HS400 unless the host offers a tuning
 * step, so here it is. The Linux driver for this controller has none: its
 * sunxi_mmc_calibrate() writes the sample delay line to step 0 with the
 * software enable bit and leaves it there for every mode, and that is what
 * carries this eMMC to HS400 on this board. mmc_config_clock() above
 * already does exactly that, so start by confirming step 0 holds.
 *
 * Only if it does not do we look further. The vendor U-Boot searches too
 * (sunxi_tuning_speed_mode at 0x4a01f378: it programs a delay, reads a
 * pattern back and keeps the middle of the widest run that passed), but it
 * reads back a pattern of its own that it first WROTE to the card at block
 * 0x5fc0 (sunxi_read_tuning at 0x4a01eb78, sunxi_mmc_tuning_init at
 * 0x4a01ef44). We will not write to anybody's eMMC to find a delay, so the
 * pattern here is the card's own tuning block, fetched with the opcode the
 * core hands us - the same search over the same delay line, read only.
 */
static int sunxi_mmc_read_tuning_block(struct sunxi_mmc_priv *priv,
				       struct mmc *mmc, uint opcode)
{
	ALLOC_CACHE_ALIGN_BUFFER(u8, buf, 128);
	struct mmc_cmd cmd;
	struct mmc_data data;

	cmd.cmdidx = opcode;
	cmd.cmdarg = 0;
	cmd.resp_type = MMC_RSP_R1;

	/* 128 bytes on an 8 bit bus, 64 on a 4 bit one (JEDEC 84-B51). */
	data.dest = (char *)buf;
	data.blocks = 1;
	data.blocksize = mmc->bus_width == 8 ? 128 : 64;
	data.flags = MMC_DATA_READ;

	return sunxi_mmc_send_cmd_common(priv, mmc, &cmd, &data);
}

static void sunxi_mmc_set_samp_dl(struct sunxi_mmc_priv *priv, uint step)
{
	writel(SUNXI_MMC_CAL_DL_SW_EN | (step & SUNXI_MMC_CAL_DL_SW_MASK),
	       &priv->reg->samp_dl);
}

static int sunxi_mmc_execute_tuning(struct udevice *dev, uint opcode)
{
	struct sunxi_mmc_plat *plat = dev_get_plat(dev);
	struct sunxi_mmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = &plat->mmc;
	uint step, run = 0, best_len = 0, best_end = 0;

	sunxi_mmc_set_samp_dl(priv, 0);
	if (!sunxi_mmc_read_tuning_block(priv, mmc, opcode))
		return 0;

	for (step = 0; step < SUNXI_MMC_CAL_DL_STEPS; step++) {
		sunxi_mmc_set_samp_dl(priv, step);
		if (sunxi_mmc_read_tuning_block(priv, mmc, opcode)) {
			run = 0;
			continue;
		}
		if (++run > best_len) {
			best_len = run;
			best_end = step;
		}
	}

	if (!best_len) {
		/* Back to the setting Linux uses; the core drops a mode. */
		sunxi_mmc_set_samp_dl(priv, 0);
		debug("mmc %u: no sample delay passed tuning\n", priv->mmc_no);
		return -EIO;
	}

	step = best_end - best_len / 2;
	sunxi_mmc_set_samp_dl(priv, step);
	debug("mmc %u: sample delay %u, window %u..%u\n", priv->mmc_no, step,
	      best_end + 1 - best_len, best_end);

	return 0;
}
#endif /* MMC_SUPPORTS_TUNING */

static const struct dm_mmc_ops sunxi_mmc_ops = {
	.send_cmd	= sunxi_mmc_send_cmd,
	.set_ios	= sunxi_mmc_set_ios,
	.get_cd		= sunxi_mmc_getcd,
#if CONFIG_IS_ENABLED(MMC_SUPPORTS_TUNING)
	.execute_tuning	= sunxi_mmc_execute_tuning,
#endif
};

static unsigned get_mclk_offset(void)
{
	if (IS_ENABLED(CONFIG_MACH_SUN9I_A80))
		return 0x410;

	if (IS_ENABLED(CONFIG_SUN50I_GEN_H6) || IS_ENABLED(CONFIG_SUNXI_GEN_NCAT2))
		return 0x830;

	return 0x88;
};

static int sunxi_mmc_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct sunxi_mmc_plat *plat = dev_get_plat(dev);
	struct sunxi_mmc_priv *priv = dev_get_priv(dev);
	struct reset_ctl_bulk reset_bulk;
	struct clk gate_clk;
	struct mmc_config *cfg = &plat->cfg;
	struct ofnode_phandle_args args;
	u32 *ccu_reg;
	int ret;

	cfg->name = dev->name;

	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34;
	cfg->host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS;
	/*
	 * One command must fit into one descriptor chain, so the chain is
	 * what limits a transfer once the IDMA is in use. Without it this is
	 * CONFIG_SYS_MMC_MAX_BLK_COUNT as before.
	 */
	cfg->b_max = min_t(unsigned int, CONFIG_SYS_MMC_MAX_BLK_COUNT,
			   SUNXI_MMC_IDMA_MAX_BLOCKS);

	cfg->f_min = 400000;
	cfg->f_max = 52000000;

	ret = mmc_of_parse(dev, cfg);
	if (ret)
		return ret;

	/*
	 * f_max above stands for the 52 MHz of MMC high speed, which is all
	 * this driver used to reach. A node that asks for HS200 wants the
	 * 200 MHz that goes with it; every slower mode stays capped by its
	 * own frequency, so this only lifts the ceiling.
	 */
	if (CONFIG_IS_ENABLED(MMC_HS200_SUPPORT) &&
	    (cfg->host_caps & MMC_MODE_HS200))
		cfg->f_max = 200000000;

	priv->reg = dev_read_addr_ptr(dev);

	/* We don't have a sunxi clock driver so find the clock address here */
	ret = dev_read_phandle_with_args(dev, "clocks", "#clock-cells", 0,
					  1, &args);
	if (ret)
		return ret;
	ccu_reg = (u32 *)(uintptr_t)ofnode_get_addr(args.node);

	priv->mmc_no = ((uintptr_t)priv->reg - SUNXI_MMC0_BASE) / 0x1000;
	priv->mclkreg = (void *)ccu_reg + get_mclk_offset() + priv->mmc_no * 4;

	/*
	 * The H713 SMHC bus interface locks up (register accesses hang
	 * the CPU) if the bus clock gate opens while the module is still
	 * held in reset, so release the reset first, as the BROM does.
	 */
	if (IS_ENABLED(CONFIG_MACH_SUN50I_H713)) {
		ret = reset_get_bulk(dev, &reset_bulk);
		if (!ret)
			reset_deassert_bulk(&reset_bulk);

		ret = clk_get_by_name(dev, "ahb", &gate_clk);
		if (!ret)
			clk_enable(&gate_clk);
	} else {
		ret = clk_get_by_name(dev, "ahb", &gate_clk);
		if (!ret)
			clk_enable(&gate_clk);

		ret = reset_get_bulk(dev, &reset_bulk);
		if (!ret)
			reset_deassert_bulk(&reset_bulk);
	}

	ret = mmc_set_mod_clk(priv, 24000000);
	if (ret)
		return ret;

	/* This GPIO is optional */
	gpio_request_by_name(dev, "cd-gpios", 0, &priv->cd_gpio,
			     GPIOD_IS_IN | GPIOD_PULL_UP);

	upriv->mmc = &plat->mmc;

	sunxi_mmc_reset(priv->reg);

	return 0;
}

static int sunxi_mmc_bind(struct udevice *dev)
{
	struct sunxi_mmc_plat *plat = dev_get_plat(dev);

	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct udevice_id sunxi_mmc_ids[] = {
	{ .compatible = "allwinner,sun4i-a10-mmc" },
	{ .compatible = "allwinner,sun5i-a13-mmc" },
	{ .compatible = "allwinner,sun7i-a20-mmc" },
	{ .compatible = "allwinner,sun8i-a83t-emmc" },
	{ .compatible = "allwinner,sun9i-a80-mmc" },
	{ .compatible = "allwinner,sun20i-d1-mmc" },
	{ .compatible = "allwinner,sun50i-a64-mmc" },
	{ .compatible = "allwinner,sun50i-a64-emmc" },
	{ .compatible = "allwinner,sun50i-h6-mmc" },
	{ .compatible = "allwinner,sun50i-h6-emmc" },
	{ .compatible = "allwinner,sun50i-a100-mmc" },
	{ .compatible = "allwinner,sun50i-a100-emmc" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(sunxi_mmc_drv) = {
	.name		= "sunxi_mmc",
	.id		= UCLASS_MMC,
	.of_match	= sunxi_mmc_ids,
	.bind		= sunxi_mmc_bind,
	.probe		= sunxi_mmc_probe,
	.ops		= &sunxi_mmc_ops,
	.plat_auto	= sizeof(struct sunxi_mmc_plat),
	.priv_auto	= sizeof(struct sunxi_mmc_priv),
};
#endif
