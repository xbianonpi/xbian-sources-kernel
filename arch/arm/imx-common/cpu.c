/*
 * (C) Copyright 2007
 * Sascha Hauer, Pengutronix
 *
 * (C) Copyright 2009 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/crm_regs.h>
#include <ipu_pixfmt.h>
#include <asm/arch/iomux.h>

#ifdef CONFIG_FSL_ESDHC
#include <fsl_esdhc.h>
#endif

char *get_reset_cause(u32 imxtype)
{
	u32 cause, gpr1, gpr12;
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;

	cause = readl(&src_regs->srsr);
	writel(cause, &src_regs->srsr);

	switch (cause) {
	case 0x00001:
	case 0x00011:
		return "POR";
	case 0x00004:
		return "CSU";
	case 0x00008:
		return "IPP USER";
	case 0x00010: {
		struct iomuxc_base_regs *const iomuxc_regs
			= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;

		//if (imxtype == MXC_CPU_MX6SOLO || imxtype == MXC_CPU_MX6Q) {
		{
			printf("  Reset PCIe regs\n");
			gpr1 = readl(&iomuxc_regs->gpr[1]);
			gpr12 = readl(&iomuxc_regs->gpr[12]);
			gpr1 &= ~(1 << 16);
			gpr12 &= ~(1 << 10);
			writel(gpr1, &iomuxc_regs->gpr[1]);
			writel(gpr12, &iomuxc_regs->gpr[12]);
		}
		return "WDOG";
	}
	case 0x00020:
		return "JTAG HIGH-Z";
	case 0x00040:
		return "JTAG SW";
	case 0x10000:
		return "WARM BOOT";
	default:
		return "unknown reset";
	}
}

#if defined(CONFIG_MX53) || defined(CONFIG_MX6)
#if defined(CONFIG_MX53)
#define MEMCTL_BASE	ESDCTL_BASE_ADDR;
#else
#define MEMCTL_BASE	MMDC_P0_BASE_ADDR;
#endif
static const unsigned char col_lookup[] = {9, 10, 11, 8, 12, 9, 9, 9};
static const unsigned char bank_lookup[] = {3, 2};

struct esd_mmdc_regs {
	uint32_t	ctl;
	uint32_t	pdc;
	uint32_t	otc;
	uint32_t	cfg0;
	uint32_t	cfg1;
	uint32_t	cfg2;
	uint32_t	misc;
	uint32_t	scr;
	uint32_t	ref;
	uint32_t	rsvd1;
	uint32_t	rsvd2;
	uint32_t	rwd;
	uint32_t	or;
	uint32_t	mrr;
	uint32_t	cfg3lp;
	uint32_t	mr4;
};

#define ESD_MMDC_CTL_GET_ROW(mdctl)	((ctl >> 24) & 7)
#define ESD_MMDC_CTL_GET_COLUMN(mdctl)	((ctl >> 20) & 7)
#define ESD_MMDC_CTL_GET_WIDTH(mdctl)	((ctl >> 16) & 3)
#define ESD_MMDC_CTL_GET_CS1(mdctl)	((ctl >> 30) & 1)
#define ESD_MMDC_MISC_GET_BANK(mdmisc)	((misc >> 5) & 1)

unsigned imx_ddr_size(void)
{
	struct esd_mmdc_regs *mem = (struct esd_mmdc_regs *)MEMCTL_BASE;
	unsigned ctl = readl(&mem->ctl);
	unsigned misc = readl(&mem->misc);
	int bits = 11 + 0 + 0 + 1;      /* row + col + bank + width */

	bits += ESD_MMDC_CTL_GET_ROW(ctl);
	bits += col_lookup[ESD_MMDC_CTL_GET_COLUMN(ctl)];
	bits += bank_lookup[ESD_MMDC_MISC_GET_BANK(misc)];
	bits += ESD_MMDC_CTL_GET_WIDTH(ctl);
	bits += ESD_MMDC_CTL_GET_CS1(ctl);
	return 1 << bits;
}
#endif

const char *get_imx_type(u32 imxtype)
{
	switch (imxtype) {
	case MXC_CPU_MX6Q:
		return "6Q";	/* Quad-core version of the mx6 */
	case MXC_CPU_MX6DL:
		return "6DL";	/* Dual Lite version of the mx6 */
	case MXC_CPU_MX6SOLO:
		return "6SOLO";	/* Solo version of the mx6 */
	case MXC_CPU_MX6SL:
		return "6SL";	/* Solo-Lite version of the mx6 */
	case MXC_CPU_MX51:
		return "51";
	case MXC_CPU_MX53:
		return "53";
	default:
		return "??";
	}
}

#if defined(CONFIG_DISPLAY_CPUINFO)

int print_cpuinfo(void)
{
	u32 cpurev, imxtype;

	cpurev = get_cpu_rev();

	imxtype = (cpurev & 0xFF000) >> 12;
	printf("CPU:   Freescale i.MX%s rev%d.%d at %d MHz\n",
		get_imx_type(imxtype),
		(cpurev & 0x000F0) >> 4,
		(cpurev & 0x0000F) >> 0,
		mxc_get_clock(MXC_ARM_CLK) / 1000000);
	printf("Reset cause: %s\n", get_reset_cause(imxtype));
	return 0;
}
#endif

int cpu_eth_init(bd_t *bis)
{
	int rc = -ENODEV;

#if defined(CONFIG_FEC_MXC)
	rc = fecmxc_initialize(bis);
#endif

	return rc;
}

#ifdef CONFIG_FSL_ESDHC
/*
 * Initializes on-chip MMC controllers.
 * to override, implement board_mmc_init()
 */
int cpu_mmc_init(bd_t *bis)
{
	return fsl_esdhc_mmc_init(bis);
}
#endif

u32 get_ahb_clk(void)
{
	struct mxc_ccm_reg *imx_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 reg, ahb_podf;

	reg = __raw_readl(&imx_ccm->cbcdr);
	reg &= MXC_CCM_CBCDR_AHB_PODF_MASK;
	ahb_podf = reg >> MXC_CCM_CBCDR_AHB_PODF_OFFSET;

	return get_periph_clk() / (ahb_podf + 1);
}

#if defined(CONFIG_VIDEO_IPUV3)
void arch_preboot_os(void)
{
	/* disable video before launching O/S */
	ipuv3_fb_shutdown();

	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	setbits_le32(&ccm->analog_pll_video, BM_ANADIG_PLL_VIDEO_POWERDOWN);

}
#endif
