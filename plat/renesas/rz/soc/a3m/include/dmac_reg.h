/*
 * Copyright (c) 2025, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __DMAC_REGS_H__
#define __DMAC_REGS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define	DMAC_BASE					(0x11820000)		/* DMAC base address */


#define DMAC_N0SA(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0000)
#define DMAC_N0DA(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0004)
#define DMAC_N0TB(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0008)
#define DMAC_N1SA(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x000C)
#define DMAC_N1DA(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0010)
#define DMAC_N1TB(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0014)
#define DMAC_CRSA(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0018)
#define DMAC_CRDA(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x001C)
#define DMAC_CRTB(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0020)
#define DMAC_CHSTAT(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0024)
#define DMAC_CHCTRL(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0028)
#define DMAC_CHCFG(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x002C)
#define DMAC_CHITVL(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0030)
#define DMAC_CHEXT(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0034)
#define DMAC_NXLA(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x0038)
#define DMAC_CRLA(ch)				(DMAC_BASE + ((ch % 8) * 0x40) + ((ch / 8) * 0x400) + 0x003C)

#define DMAC_DCTRL_0_7				(DMAC_BASE + 0x0300)
#define DMAC_DSTAT_EN_0_7			(DMAC_BASE + 0x0310)
#define DMAC_DSTAT_ER_0_7			(DMAC_BASE + 0x0314)
#define DMAC_DSTAT_END_0_7			(DMAC_BASE + 0x0318)
#define DMAC_DSTAT_TC_0_7			(DMAC_BASE + 0x031C)
#define DMAC_DSTAT_SUS_0_7			(DMAC_BASE + 0x0320)

#define DMAC_DCTRL_8_15				(DMAC_BASE + 0x0700)
#define DMAC_DSTAT_EN_8_15			(DMAC_BASE + 0x0710)
#define DMAC_DSTAT_ER_8_15			(DMAC_BASE + 0x0714)
#define DMAC_DSTAT_END_8_15			(DMAC_BASE + 0x0718)
#define DMAC_DSTAT_TC_8_15			(DMAC_BASE + 0x071C)
#define DMAC_DSTAT_SUS_8_15			(DMAC_BASE + 0x0720)

/* CHSTAT field */
#define DMAC_CHSTAT_INIMSK_POS		16
#define DMAC_CHSTAT_INIMSK			(1u << DMAC_CHSTAT_INIMSK_POS)
#define DMAC_CHSTAT_MODE_POS		11
#define DMAC_CHSTAT_MODE			(1u << DMAC_CHSTAT_MODE_POS)
#define DMAC_CHSTAT_DER_POS			10
#define DMAC_CHSTAT_DER				(1u << DMAC_CHSTAT_DER_POS)
#define DMAC_CHSTAT_DW_POS			9
#define DMAC_CHSTAT_DW				(1u << DMAC_CHSTAT_DW_POS)
#define DMAC_CHSTAT_DL_POS			8
#define DMAC_CHSTAT_DL				(1u << DMAC_CHSTAT_DL_POS)
#define DMAC_CHSTAT_SR_POS			7
#define DMAC_CHSTAT_SR				(1u << DMAC_CHSTAT_SR_POS)
#define DMAC_CHSTAT_TC_POS			6
#define DMAC_CHSTAT_TC				(1u << DMAC_CHSTAT_TC_POS)
#define DMAC_CHSTAT_END_POS			5
#define DMAC_CHSTAT_END				(1u << DMAC_CHSTAT_END_POS)
#define DMAC_CHSTAT_ER_POS			4
#define DMAC_CHSTAT_ER				(1u << DMAC_CHSTAT_ER_POS)
#define DMAC_CHSTAT_SUS_POS			3
#define DMAC_CHSTAT_SUS				(1u << DMAC_CHSTAT_SUS_POS)
#define DMAC_CHSTAT_TACT_POS		2
#define DMAC_CHSTAT_TACT			(1u << DMAC_CHSTAT_TACT_POS)
#define DMAC_CHSTAT_RQST_POS		1
#define DMAC_CHSTAT_RQST			(1u << DMAC_CHSTAT_RQST_POS)
#define DMAC_CHSTAT_EN_POS			0
#define DMAC_CHSTAT_EN				(1u << DMAC_CHSTAT_EN_POS)

/* CHCTRL field */
#define DMAC_CHCTRL_CLRINTMSK_POS	17
#define DMAC_CHCTRL_CLRINTMSK		(1u << DMAC_CHCTRL_CLRINTMSK_POS)
#define DMAC_CHCTRL_SETINTMSK_POS	16
#define DMAC_CHCTRL_SETINTMSK		(1u << DMAC_CHCTRL_SETINTMSK_POS)
#define DMAC_CHCTRL_CLRSUS_POS		9
#define DMAC_CHCTRL_CLRSUS			(1u << DMAC_CHCTRL_CLRSUS_POS)
#define DMAC_CHCTRL_SETSUS_POS		8
#define DMAC_CHCTRL_SETSUS			(1u << DMAC_CHCTRL_SETSUS_POS)
#define DMAC_CHCTRL_CLRTC_POS		6
#define DMAC_CHCTRL_CLRTC			(1u << DMAC_CHCTRL_CLRTC_POS)
#define DMAC_CHCTRL_CLREND_POS		5
#define DMAC_CHCTRL_CLREND			(1u << DMAC_CHCTRL_CLREND_POS)
#define DMAC_CHCTRL_CLRRQ_POS		4
#define DMAC_CHCTRL_CLRRQ			(1u << DMAC_CHCTRL_CLRRQ_POS)
#define DMAC_CHCTRL_SWRST_POS		3
#define DMAC_CHCTRL_SWRST			(1u << DMAC_CHCTRL_SWRST_POS)
#define DMAC_CHCTRL_STG_POS			2
#define DMAC_CHCTRL_STG				(1u << DMAC_CHCTRL_STG_POS)
#define DMAC_CHCTRL_CLREN_POS		1
#define DMAC_CHCTRL_CLREN			(1u << DMAC_CHCTRL_CLREN_POS)
#define DMAC_CHCTRL_SETEN_POS		0
#define DMAC_CHCTRL_SETEN			(1u << DMAC_CHCTRL_SETEN_POS)

/* CHCFG field */
#define DMAC_CHCFG_DMS_POS			31
#define DMAC_CHCFG_DMS				(1u << DMAC_CHCFG_DMS_POS)
#define DMAC_CHCFG_REN_POS			30
#define DMAC_CHCFG_REN				(1u << DMAC_CHCFG_REN_POS)
#define DMAC_CHCFG_RSW_POS			29
#define DMAC_CHCFG_RSW				(1u << DMAC_CHCFG_RSW_POS)
#define DMAC_CHCFG_RSEL_POS			28
#define DMAC_CHCFG_RSEL				(1u << DMAC_CHCFG_RSEL_POS)
#define DMAC_CHCFG_SBE_POS			27
#define DMAC_CHCFG_SBE				(1u << DMAC_CHCFG_SBE_POS)
#define DMAC_CHCFG_DEM_POS			24
#define DMAC_CHCFG_DEM				(1u << DMAC_CHCFG_DEM_POS)
#define DMAC_CHCFG_TM_POS			22
#define DMAC_CHCFG_TM				(1u << DMAC_CHCFG_TM_POS)
#define DMAC_CHCFG_DAD_POS			21
#define DMAC_CHCFG_DAD				(1u << DMAC_CHCFG_DAD_POS)
#define DMAC_CHCFG_SAD_POS			20
#define DMAC_CHCFG_SAD				(1u << DMAC_CHCFG_SAD_POS)
#define DMAC_CHCFG_DDS_POS			16
#define DMAC_CHCFG_DDS				(15u << DMAC_CHCFG_DDS_POS)
#define DMAC_CHCFG_SDS_POS			12
#define DMAC_CHCFG_SDS				(15u << DMAC_CHCFG_SDS_POS)
#define DMAC_CHCFG_AM_POS			8
#define DMAC_CHCFG_AM				(7u << DMAC_CHCFG_AM_POS)
#define DMAC_CHCFG_LVL_POS			6
#define DMAC_CHCFG_LVL				(1u << DMAC_CHCFG_LVL_POS)
#define DMAC_CHCFG_HIEN_POS			5
#define DMAC_CHCFG_HIEN				(1u << DMAC_CHCFG_HIEN_POS)
#define DMAC_CHCFG_LOEN_POS			4
#define DMAC_CHCFG_LOEN				(1u << DMAC_CHCFG_LOEN_POS)
#define DMAC_CHCFG_REQD_POS			3
#define DMAC_CHCFG_REQD				(1u << DMAC_CHCFG_REQD_POS)
#define DMAC_CHCFG_SEL_POS			0
#define DMAC_CHCFG_SEL				(7u << DMAC_CHCFG_SEL_POS)

#ifdef __cplusplus
}
#endif

#endif	/* __DMAC_REGS_H__ */
