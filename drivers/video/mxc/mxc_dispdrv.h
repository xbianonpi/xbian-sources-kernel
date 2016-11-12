/*
 * Copyright (C) 2011-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MXC_DISPDRV_H__
#define __MXC_DISPDRV_H__
#include <linux/ipu.h>
#include <linux/ipu-v3.h>
#include <linux/fb.h>
#include "crtc.h"

struct mxc_dispdrv_handle {
	struct mxc_dispdrv_driver *drv;
};

struct mxc_dispdrv_setting {
	/*input-feedback parameter*/
	struct fb_info *fbi;
	int if_fmt;
	int default_bpp;
	char *dft_mode_str;

	/* feedback parameter */
	enum crtc crtc;
};

struct mxc_dispdrv_driver {
	const char *name;
	int (*init) (struct mxc_dispdrv_handle *, struct mxc_dispdrv_setting *);
	void (*deinit) (struct mxc_dispdrv_handle *);
	/* display driver enable function for extension */
	int (*enable) (struct mxc_dispdrv_handle *, struct fb_info *);
	/* display driver disable function, called at early part of fb_blank */
	void (*disable) (struct mxc_dispdrv_handle *, struct fb_info *);
	/* display driver setup function, called at early part of fb_set_par */
	int (*setup) (struct mxc_dispdrv_handle *, struct fb_info *fbi);
};

/*!
 * Structure containing the MXC specific framebuffer information.
 */
struct mxcfb_info {
	int default_bpp;
	int cur_blank;
	int next_blank;
	ipu_channel_t ipu_ch;
	int ipu_id;
	int ipu_di;
	u32 ipu_di_pix_fmt;
	bool ipu_int_clk;
	bool overlay;
	bool alpha_chan_en;
	bool late_init;
	bool first_set_par;
	dma_addr_t alpha_phy_addr0;
	dma_addr_t alpha_phy_addr1;
	void *alpha_virt_addr0;
	void *alpha_virt_addr1;
	uint32_t alpha_mem_len;
	uint32_t ipu_ch_irq;
	uint32_t ipu_ch_nf_irq;
	uint32_t ipu_alp_ch_irq;
	uint32_t cur_ipu_buf;
	uint32_t cur_ipu_alpha_buf;

	u32 pseudo_palette[16];

	bool mode_found;
	struct completion flip_complete;
	struct completion alpha_flip_complete;
	struct completion vsync_complete;

	void *ipu;
	struct fb_info *ovfbi;

	struct mxc_dispdrv_handle *dispdrv;

	struct fb_var_screeninfo cur_var;
};

struct mxc_dispdrv_handle *mxc_dispdrv_register(struct mxc_dispdrv_driver *drv);
int mxc_dispdrv_unregister(struct mxc_dispdrv_handle *handle);
struct mxc_dispdrv_handle *mxc_dispdrv_gethandle(char *name,
	struct mxc_dispdrv_setting *setting);
void mxc_dispdrv_puthandle(struct mxc_dispdrv_handle *handle);
int mxc_dispdrv_setdata(struct mxc_dispdrv_handle *handle, void *data);
void *mxc_dispdrv_getdata(struct mxc_dispdrv_handle *handle);
#endif
