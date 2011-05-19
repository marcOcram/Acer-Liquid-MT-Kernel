/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"

extern struct semaphore mddi_host_mutex;
static struct mdp4_overlay_pipe *mddi_pipe;
static struct mdp4_overlay_pipe *pending_pipe;
static struct msm_fb_data_type *mddi_mfd;

#ifdef USE_DMAS
static struct completion mddi_comp;
static char *blt_virt;
static char *blt_phy;
#define BLT_SIZE (480 * 800 * 4 * 2)  /* X_RES x Y_RES x Color_Depth x Double_Buffer */
#endif

#ifndef CONFIG_MACH_ACER_A4
static int vsync_start_y_adjust = 4;
static int dmap_vsync_enable;
#else
static int dmap_vsync_enable = 1;	/* for enable vsync alignment in overlay */

void mdp_dmap_vsync_set(int enable)
{
	dmap_vsync_enable = enable;
}

int mdp_dmap_vsync_get(void)
{
	return dmap_vsync_enable;
}
#endif

void mdp4_mddi_vsync_enable(struct msm_fb_data_type *mfd,
		struct mdp4_overlay_pipe *pipe, int which)
{
#ifndef CONFIG_MACH_ACER_A4
	uint32 start_y, data, tear_en;
#else
	uint32 data, tear_en;
#endif

	tear_en = (1 << which);

	if ((mfd->use_mdp_vsync) && (mfd->ibuf.vsync_enable) &&
		(mfd->panel_info.lcd.vsync_enable)) {

#if defined(MDP4_MDDI_DMA_SWITCH) || defined(USE_DMAS)
		if (which == 0 && dmap_vsync_enable == 0) /* dma_p */
			return;
#endif
#ifndef CONFIG_MACH_ACER_A4
		if (vsync_start_y_adjust <= pipe->dst_y)
			start_y = pipe->dst_y - vsync_start_y_adjust;
		else
			start_y = (mfd->total_lcd_lines - 1) -
				(vsync_start_y_adjust - pipe->dst_y);
		if (which == 0)
			MDP_OUTP(MDP_BASE + 0x210, start_y);	/* primary */
		else
			MDP_OUTP(MDP_BASE + 0x214, start_y);	/* secondary */
#else
		if (which == 0)
			MDP_OUTP(MDP_BASE + 0x210, 3);	/* primary */
		else
			MDP_OUTP(MDP_BASE + 0x214, 3);	/* secondary */
#endif

		data = inpdw(MDP_BASE + 0x20c);
		data |= tear_en;
		MDP_OUTP(MDP_BASE + 0x20c, data);
	} else {
		data = inpdw(MDP_BASE + 0x20c);
		data &= ~tear_en;
		MDP_OUTP(MDP_BASE + 0x20c, data);
	}
}

#define WHOLESCREEN

void mdp4_overlay_update_lcd(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	uint8 *src;
	int ptype;
	uint32 mddi_ld_param;
	uint16 mddi_vdo_packet_reg;
	struct mdp4_overlay_pipe *pipe;

	if (mfd->key != MFD_KEY)
		return;

	mddi_mfd = mfd;		/* keep it */

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);


	if (mddi_pipe == NULL) {
		ptype = mdp4_overlay_format2type(mfd->fb_imgType);
		pipe = mdp4_overlay_pipe_alloc(ptype);
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = mfd->fb_imgType;
		mdp4_overlay_format2pipe(pipe);

		mddi_pipe = pipe; /* keep it */

#ifdef USE_DMAS
		blt_virt = (char *) kmalloc(BLT_SIZE, GFP_KERNEL);
		if (blt_virt) {
			blt_phy = (char *)virt_to_phys(blt_virt);
			mddi_pipe->blt_addr = (uint32)blt_phy;
			mddi_pipe->blt_cnt = 0;
			printk("%s: blt virt=%x phy=%x\n", __func__,
					(int)blt_virt, (int)blt_phy);
		} else
			printk("%s: blt kmalloc FAILED\n", __func__);

		init_completion(&mddi_comp);
#endif

		mddi_ld_param = 0;
		mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

#ifdef USE_DMAS
		MDP_OUTP(MDP_BASE + 0x00090, 1);/* secondary mddi */
#else
		if (mfd->panel_info.type == MDDI_PANEL) {
			if (mfd->panel_info.pdest == DISPLAY_1)
				mddi_ld_param = 0;
			else
				mddi_ld_param = 1;
		} else {
			mddi_ld_param = 2;
		}

		MDP_OUTP(MDP_BASE + 0x00090, mddi_ld_param);
#endif

		if (mfd->panel_info.bpp == 24)
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_24 << 16) | mddi_vdo_packet_reg);
		else if (mfd->panel_info.bpp == 16)
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_16 << 16) | mddi_vdo_packet_reg);
		else
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC << 16) | mddi_vdo_packet_reg);

		MDP_OUTP(MDP_BASE + 0x00098, 0x01);
	} else {
		pipe = mddi_pipe;
	}


	src = (uint8 *) iBuf->buf;

#ifdef WHOLESCREEN

	{
		struct fb_info *fbi;

		fbi = mfd->fbi;
		pipe->src_height = fbi->var.yres;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres;
		pipe->src_w = fbi->var.xres;
		pipe->src_y = 0;
		pipe->src_x = 0;
		pipe->dst_h = fbi->var.yres;
		pipe->dst_w = fbi->var.xres;
		pipe->dst_y = 0;
		pipe->dst_x = 0;
		pipe->srcp0_addr = (uint32)src;
		pipe->srcp0_ystride = fbi->fix.line_length;
	}

#else
	if (mdp4_overlay_active(MDP4_MIXER0)) {
		struct fb_info *fbi;

		fbi = mfd->fbi;
		pipe->src_height = fbi->var.yres;
		pipe->src_width = fbi->var.xres;
		pipe->src_h = fbi->var.yres;
		pipe->src_w = fbi->var.xres;
		pipe->src_y = 0;
		pipe->src_x = 0;
		pipe->dst_h = fbi->var.yres;
		pipe->dst_w = fbi->var.xres;
		pipe->dst_y = 0;
		pipe->dst_x = 0;
		pipe->srcp0_addr = (uint32) src;
		pipe->srcp0_ystride = fbi->fix.line_length;
	} else {
		/* starting input address */
		src += (iBuf->dma_x + iBuf->dma_y * iBuf->ibuf_width)
					* iBuf->bpp;

		pipe->src_height = iBuf->dma_h;
		pipe->src_width = iBuf->dma_w;
		pipe->src_h = iBuf->dma_h;
		pipe->src_w = iBuf->dma_w;
		pipe->src_y = 0;
		pipe->src_x = 0;
		pipe->dst_h = iBuf->dma_h;
		pipe->dst_w = iBuf->dma_w;
		pipe->dst_y = iBuf->dma_y;
		pipe->dst_x = iBuf->dma_x;
		pipe->srcp0_addr = (uint32) src;
		pipe->srcp0_ystride = iBuf->ibuf_width * iBuf->bpp;
	}
#endif

	pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;

	mdp4_overlay_rgb_setup(pipe);

	mdp4_mixer_stage_up(pipe);

	mdp4_overlayproc_cfg(pipe);

#ifdef USE_DMAS
	mdp4_overlay_dmas_xy(pipe);

	mdp4_overlay_dmas_cfg(mfd, 0);

	mdp4_mddi_vsync_enable(mfd, pipe, 1);
#else
	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(mfd, 0);

	mdp4_mddi_vsync_enable(mfd, pipe, 0);
#endif

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

}

#ifdef USE_DMAS
void mdp4_blt_xy_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr;
	char *overlay_base;


	if (pipe->blt_addr == 0)
		return;

	/* overlay ouput is RGB888 */
	off = 0;
	if (pipe->blt_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * 3;

	addr = pipe->blt_addr + off;

	/* dmas */
	MDP_OUTP(MDP_BASE + 0xA0008, addr);

	mddi_pipe->blt_cnt++;
	off = 0;
	if (pipe->blt_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * 3;

	addr = pipe->blt_addr + off;

	/* overlay 0 */
	overlay_base = MDP_BASE + MDP4_OVERLAYPROC0_BASE;/* 0x10000 */
	outpdw(overlay_base + 0x000c, addr);
	outpdw(overlay_base + 0x001c, addr);
}
#endif

/*
 * mdp4_overlay0_done_mddi: called from isr
 */
void mdp4_overlay0_done_mddi()
{
#ifdef USE_DMAS
	mdp_pipe_ctrl(MDP_DMA_S_BLOCK, MDP_BLOCK_POWER_ON, TRUE);
	mdp4_blt_xy_update(mddi_pipe);
	outpdw(MDP_BASE + 0x0010, 0x0); /* start DMAS */
#endif

#ifdef MDP4_NONBLOCKING
	mdp_disable_irq_nosync(MDP_OVERLAY0_TERM);
#endif

	if (pending_pipe)
		complete(&pending_pipe->comp);
}

void mdp4_mddi_overlay_restore(void)
{
#ifdef MDP4_MDDI_DMA_SWITCH
	mdp4_mddi_overlay_dmas_restore();
#else
	/* mutex holded by caller */
	if (mddi_mfd && mddi_pipe) {
		mdp4_overlay_update_lcd(mddi_mfd);
		mdp4_mddi_overlay_kickoff(mddi_mfd, mddi_pipe);
	}
#endif
}

static ulong mddi_last_kick;
static ulong mddi_kick_interval;

#ifdef USE_DMAS
int mdp4_mddi_overlay_kickoff(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
#else
void mdp4_mddi_overlay_kickoff(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
#endif
{
#ifdef MDP4_NONBLOCKING
	unsigned long flag;

	if (pipe == mddi_pipe) {  /* base layer */
		if (mdp4_overlay_pipe_staged(pipe->mixer_num) > 1) {
			if (time_before(jiffies,
				(mddi_last_kick + mddi_kick_interval/2))) {
				mdp4_stat.kickoff_mddi_skip++;
				return;	/* let other pipe to kickoff */
			}
		}
	}

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (mfd->dma->busy == TRUE) {
		INIT_COMPLETION(pipe->comp);
		pending_pipe = pipe;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (pending_pipe != NULL) {
		/* wait until DMA finishes the current job */
		wait_for_completion_killable(&pipe->comp);
		pending_pipe = NULL;
	}
	down(&mfd->sem);
	mdp_enable_irq(MDP_OVERLAY0_TERM);
	mfd->dma->busy = TRUE;
	/* start OVERLAY pipe */
	mdp_pipe_kickoff(MDP_OVERLAY0_TERM, mfd);
	if (pipe != mddi_pipe) { /* non base layer */
		int intv;

		if (mddi_last_kick == 0)
			intv = 0;
		else
			intv = jiffies - mddi_last_kick;

		mddi_kick_interval += intv;
		mddi_kick_interval /= 2;	/* average */
		mddi_last_kick = jiffies;
	}
	up(&mfd->sem);
#else
#ifdef CONFIG_MACH_ACER_A4
	struct mdp_dma_data *dma;

	if (pipe == mddi_pipe && mfd->skip_fb == 1) {  /* base layer */
		if (mdp4_overlay_pipe_staged(pipe->mixer_num) > 1) {
			if (time_before(jiffies,
				(mddi_last_kick + mddi_kick_interval + 3))) {
				mdp4_stat.kickoff_mddi_skip++;
#ifdef USE_DMAS
				return -1;	/* let other pipe to kickoff */
#else
				return;	/* let other pipe to kickoff */
#endif
			}
		}
	}
#endif
	down(&mfd->sem);
	mdp_enable_irq(MDP_OVERLAY0_TERM);
	mfd->dma->busy = TRUE;
	INIT_COMPLETION(pipe->comp);
	pending_pipe = pipe;

	/* start OVERLAY pipe */
	mdp_pipe_kickoff(MDP_OVERLAY0_TERM, mfd);
	up(&mfd->sem);

	/* wait until DMA finishes the current job */
#ifndef CONFIG_MACH_ACER_A4
	wait_for_completion_killable(&pipe->comp);
#else
	if (wait_for_completion_interruptible_timeout(&pipe->comp, 5*HZ) <= 0) {
		/* recovery */
		dma = &dma2_data;
		dma->busy = FALSE;
		pending_pipe = NULL;
		mdp_disable_irq(MDP_OVERLAY0_TERM);

		if (pipe != mddi_pipe) pr_err("Overlay Timeout\n");
		else pr_err("Framebuffer Timeout\n");
		return -1;
	}
#endif
	pending_pipe = NULL;
	mdp_disable_irq(MDP_OVERLAY0_TERM);
#ifdef CONFIG_MACH_ACER_A4
	if (pipe != mddi_pipe && mfd->skip_fb == 1) { /* non base layer */
		int intv;

		if (mddi_last_kick == 0)
			intv = 0;
		else
			intv = jiffies - mddi_last_kick;

		mddi_kick_interval += intv;
		mddi_kick_interval /= 2;	/* average */
		mddi_last_kick = jiffies;
	}
#endif
#endif
#ifdef USE_DMAS
	return 0;
#endif
}

#ifdef USE_DMAS
void mdp4_dma_s_blt_done_mddi()
{
	if (pending_pipe == mddi_pipe)
		complete(&mddi_comp);
}
#endif

#ifdef MDP4_MDDI_DMA_SWITCH

void mdp4_dma_s_done_mddi()
{
	if (pending_pipe)
		complete(&pending_pipe->dmas_comp);
}
void mdp4_dma_s_update_lcd(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	uint32 outBpp = iBuf->bpp;
	uint16 mddi_vdo_packet_reg;
	uint32 dma_s_cfg_reg;

	dma_s_cfg_reg = 0;

	if (mfd->fb_imgType == MDP_RGBA_8888)
		dma_s_cfg_reg |= DMA_PACK_PATTERN_BGR; /* on purpose */
	else if (mfd->fb_imgType == MDP_BGR_565)
		dma_s_cfg_reg |= DMA_PACK_PATTERN_BGR;
	else
		dma_s_cfg_reg |= DMA_PACK_PATTERN_RGB;

	if (outBpp == 4)
		dma_s_cfg_reg |= (1 << 26); /* xRGB8888 */
	else if (outBpp == 2)
		dma_s_cfg_reg |= DMA_IBUF_FORMAT_RGB565;

	dma_s_cfg_reg |= DMA_DITHER_EN;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	/* PIXELSIZE */
	MDP_OUTP(MDP_BASE + 0xa0004, (pipe->dst_h << 16 | pipe->dst_w));
	MDP_OUTP(MDP_BASE + 0xa0008, pipe->srcp0_addr);	/* ibuf address */
	MDP_OUTP(MDP_BASE + 0xa000c, pipe->srcp0_ystride);/* ystride */

	if (mfd->panel_info.bpp == 24) {
		dma_s_cfg_reg |= DMA_DSTC0G_8BITS |	/* 666 18BPP */
		    DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;
	} else if (mfd->panel_info.bpp == 18) {
		dma_s_cfg_reg |= DMA_DSTC0G_6BITS |	/* 666 18BPP */
		    DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
	} else {
		dma_s_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
		    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	}

	MDP_OUTP(MDP_BASE + 0xa0010, (pipe->dst_y << 16) | pipe->dst_x);
	MDP_OUTP(MDP_BASE + 0x00090, 1); /* do not change this */

	mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

	if (mfd->panel_info.bpp == 24)
		MDP_OUTP(MDP_BASE + 0x00094,
			(MDDI_VDO_PACKET_DESC_24 << 16) | mddi_vdo_packet_reg);
	else if (mfd->panel_info.bpp == 16)
		MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_16 << 16) | mddi_vdo_packet_reg);
	else
		MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC << 16) | mddi_vdo_packet_reg);

	MDP_OUTP(MDP_BASE + 0x00098, 0x01);

	MDP_OUTP(MDP_BASE + 0xa0000, dma_s_cfg_reg);

	mdp4_mddi_vsync_enable(mfd, pipe, 1);

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}

void mdp4_mddi_dma_s_kickoff(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	down(&mfd->sem);
	down(&mddi_host_mutex);
	mdp_enable_irq(MDP_DMA_S_TERM);
	mfd->dma->busy = TRUE;
	INIT_COMPLETION(pipe->dmas_comp);
	mfd->ibuf_flushed = TRUE;
	pending_pipe = pipe;
	/* start dma_s pipe */
	mdp_pipe_kickoff(MDP_DMA_S_TERM, mfd);
	up(&mfd->sem);

	/* wait until DMA finishes the current job */
	wait_for_completion_killable(&pipe->dmas_comp);
	up(&mddi_host_mutex);
	pending_pipe = NULL;
	mdp_disable_irq(MDP_DMA_S_TERM);
}

void mdp4_mddi_overlay_dmas_restore(void)
{
	/* mutex holded by caller */
	if (mddi_mfd && mddi_pipe) {
		mdp4_dma_s_update_lcd(mddi_mfd, mddi_pipe);
		mdp4_mddi_dma_s_kickoff(mddi_mfd, mddi_pipe);
	}
}
#endif

void mdp4_mddi_overlay(struct msm_fb_data_type *mfd)
{
#ifndef CONFIG_MACH_ACER_A4
	mutex_lock(&mfd->dma->ov_mutex);
#endif

#ifdef MDP4_NONBLOCKING
	if (mfd && mfd->panel_power_on) {
#else
	if ((mfd) && (!mfd->dma->busy) && (mfd->panel_power_on)) {
#endif
#ifdef CONFIG_MACH_ACER_A4
		mutex_lock(&mfd->dma->ov_mutex);
#endif
		mdp4_overlay_update_lcd(mfd);

#ifdef MDP4_MDDI_DMA_SWITCH
		if (mdp4_overlay_pipe_staged(mddi_pipe->mixer_num) <= 1) {
			mdp4_dma_s_update_lcd(mfd, mddi_pipe);
			mdp4_mddi_dma_s_kickoff(mfd, mddi_pipe);
		} else
#endif
#ifdef USE_DMAS
		{
			/* UI only */
			INIT_COMPLETION(mddi_comp);
			if (mdp4_mddi_overlay_kickoff(mfd, mddi_pipe) >= 0) {
				pending_pipe = mddi_pipe;
#ifdef CONFIG_MACH_ACER_A4
				mdp_enable_irq(MDP_DMA_S_TERM);
				down(&mddi_host_mutex);
				if( wait_for_completion_interruptible_timeout(&mddi_comp, HZ) <= 0)
					pr_emerg("DMA_S wait completion timeout or interrupted\n");
#else
				down(&mddi_host_mutex);
				mdp_enable_irq(MDP_DMA_S_TERM);
				wait_for_completion_killable(&mddi_comp);
#endif
				mdp_disable_irq(MDP_DMA_S_TERM);
				up(&mddi_host_mutex);
			}
		}
#else
			mdp4_mddi_overlay_kickoff(mfd, mddi_pipe);
#endif
		mdp4_stat.kickoff_mddi++;

	/* signal if pan function is waiting for the update completion */
		if (mfd->pan_waiting) {
			mfd->pan_waiting = FALSE;
			complete(&mfd->pan_comp);
		}
#ifdef CONFIG_MACH_ACER_A4
	mutex_unlock(&mfd->dma->ov_mutex);
	}
#else
	}

	mutex_unlock(&mfd->dma->ov_mutex);
#endif
}
