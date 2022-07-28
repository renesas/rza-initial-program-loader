/*
 * Copyright (c) 2022, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdbool.h>

#include <assert.h>
#include <endian.h>
#include <common/debug.h>

#include <arch_helpers.h>
#include <xspi_api.h>
#include <spim.h>
#include <spim_regs.h>
#include <octa_regs.h>
#include <cpg.h>
#include <cpg_regs.h>
#include <sys_regs.h>
#include <pfc_regs.h>
#include <lib/mmio.h>
#include <drivers/delay_timer.h>

/* Defaults */
#define DEFAULT_SPI_FREQUENCY 66666667
#define RESET_DURATION_US 10
#define DEFAULT_VOLTAGE_IS_1800MV 1
#define DEFAULT_CALIBRATION_OFFSET UINT32_MAX

/* Static function pre-definition */
static int spim_open(xspi_ctrl_t * ctrl, xspi_cfg_t const * const cfg);
static int spim_close(xspi_ctrl_t * const ctrl);
static int spim_post_init(xspi_ctrl_t * const ctrl);
static int spim_exec_op(xspi_ctrl_t * const ctrl, xspi_op_t const * const op, bool is_write);
static int spim_configure_xip(xspi_ctrl_t * const ctrl, xspi_op_t const * const rop, xspi_op_t const * const wop);
static int spim_start_xip(xspi_ctrl_t * const ctrl);
static int spim_stop_xip(xspi_ctrl_t * const ctrl);
static int spim_run_manual_calibration(xspi_ctrl_t * const ctrl);
static int spim_enable_auto_calibration(xspi_ctrl_t * const ctrl);
static int spim_disable_auto_calibration(xspi_ctrl_t * const ctrl);
static int spim_set_frequency(xspi_ctrl_t * const ctrl, int frequency_hz);
static int spim_clean_mmap(xspi_ctrl_t * const ctrl);
static int spim_inv_mmap(xspi_ctrl_t * const ctrl);
static uintptr_t spim_get_mmap_base(xspi_ctrl_t * const ctrl);
static size_t spim_get_mmap_size(xspi_ctrl_t * const ctrl);

/* API function table definition */
const xspi_api_t spim_api = {
	.open = spim_open,
	.close = spim_close,
	.post_init = spim_post_init,
	.exec_op = spim_exec_op,
	.configure_xip = spim_configure_xip,
	.start_xip = spim_start_xip,
	.stop_xip = spim_stop_xip,
	.run_manual_calibration = spim_run_manual_calibration,
	.enable_auto_calibration = spim_enable_auto_calibration,
	.disable_auto_calibration = spim_disable_auto_calibration,
	.set_frequency = spim_set_frequency,
	.clean_mmap = spim_clean_mmap,
	.inv_mmap = spim_inv_mmap,
	.get_mmap_base = spim_get_mmap_base,
	.get_mmap_size = spim_get_mmap_size,
};

/* DTR test pattern */
static const uint32_t dtr_pattern = 0xAA00FF55;

/* Static variables */
static bool globally_initialised = false;
static bool calibration_done = false;
static int spi_clock = DEFAULT_SPI_FREQUENCY;

static const uintptr_t load_base = BL2_BASE;

/* Function definitions */
static void wait_until_32(uintptr_t addr, uint32_t mask, uint32_t data)
{
	while((mmio_read_32(addr) & mask) != data) {};
}

static void select_spim(spim_ctrl_t *myctrl, xspi_cfg_t const * const cfg)
{
	if (RZ_XSPI_EXCLUSIVE_SELECTOR) {
		/* Check if OCTA selected */
		uint32_t ipcont_spi_octa = mmio_read_32(SYS_IPCONT);
		if ((ipcont_spi_octa & IPCONT_SEL_SPI_OCTA) == IPCONT_SEL_SPI_OCTA_OCTA) {
			/* Reset the OCTA controller and the devices connected,
			 *  required to change the device mode from OPI to SPI.
			 * Otherwise, the octa devices can not communicate with the SPIM controller.
			 */
			/* Assert reset line */
			mmio_clrbits_32(myctrl->reg_base + OCTA_RSTCNT, OCTA_RSTCNT_RSTVAL);

			/* Reset OCTA controller */
			cpg_reset_on(CPG_CLOCK_OCTA);

			/* Stop OCTA clock */
			cpg_clock_off(CPG_CLOCK_OCTA);
		}
	}

	/* Wait for reset SPI device */
	udelay(RESET_DURATION_US);

	if (RZ_XSPI_FORCE_VOLTAGE_SETTING) {
		/* force voltage setting
		 * Note: This is required if the boot mode is neither 3 nor 4.
		 */
		uint8_t voltage = DEFAULT_VOLTAGE_IS_1800MV;
		if (cfg->extend)  {
			const spim_ext_t * ext = (const spim_ext_t*)cfg->extend;
			if (ext->voltage_is_3300mv) voltage = 0;
		}
		mmio_write_8(PFC_QSPI, voltage);
		mmio_read_8(PFC_QSPI);
	}

	/* Supply SPIM clock (currently not implemented) */
	cpg_clock_on(CPG_CLOCK_SPIM);

	/* Resume SPIM controller (currently not implemented) */
	cpg_reset_off(CPG_CLOCK_SPIM);

	if (RZ_XSPI_EXCLUSIVE_SELECTOR) {
		/* Select SPIM for SPI controller */
		uint32_t ipcont_spi_octa = mmio_read_32(SYS_IPCONT);
		ipcont_spi_octa &= ~IPCONT_SEL_SPI_OCTA;
		ipcont_spi_octa |= IPCONT_SEL_SPI_OCTA_SPI << IPCONT_SEL_SPI_OCTA_POS;
		mmio_write_32(SYS_IPCONT, ipcont_spi_octa);
		mmio_read_32(SYS_IPCONT);
	}
}

static void test_tend(const spim_ctrl_t * myctrl)
{
	wait_until_32(myctrl->reg_base + SPIM_CMNSR, SPIM_CMNSR_TEND, SPIM_CMNSR_TEND);
}

static void spim_init_phy(const spim_ctrl_t * myctrl)
{
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ2, 0xa5390000);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ1, 0x80000000);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ2, 0x00008080);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ1, 0x80000022);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ2, 0x00008080);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ1, 0x80000024);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ2, 0x00000030);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ1, 0x80000032);
}

static int spim_start_calibration(const spim_ctrl_t * myctrl)
{
	/* Error if not DTR is selected for XIP mode */
	uint32_t drdrenr = mmio_read_32(myctrl->reg_base + SPIM_DRDRENR);
	if (!(drdrenr & SPIM_DRDRENR_DRDRE)) return -1;

	/* Start sequence */
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ2, 0xa5390000);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ1, 0x80000000);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ2, 0x00009191);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ1, 0x80000022);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ2, 0x00009191);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ1, 0x80000024);

	/* Copy DRxxxx to SMxxxx for preparing to run read operation
	 * from manual mode
	 */
	mmio_write_32(myctrl->reg_base + SPIM_SMDRENR, drdrenr);
	uint32_t drcmr = mmio_read_32(myctrl->reg_base + SPIM_DRCMR);
	mmio_write_32(myctrl->reg_base + SPIM_SMCMR, drcmr);
	uint32_t dropr = mmio_read_32(myctrl->reg_base + SPIM_DROPR);
	mmio_write_32(myctrl->reg_base + SPIM_SMOPR, dropr);
	uint32_t smenr = mmio_read_32(myctrl->reg_base + SPIM_DRENR);
	smenr |= SPIM_SMENR_SPIDE_LONG << SPIM_SMENR_SPIDE_POS;
	mmio_write_32(myctrl->reg_base + SPIM_SMENR, smenr);
	uint32_t drdmcr = mmio_read_32(myctrl->reg_base + SPIM_DRDMCR);
	mmio_write_32(myctrl->reg_base + SPIM_SMDMCR, drdmcr);

	return 0;
}

static const uint32_t cmncr_init_mask =
	SPIM_CMNCR_BSZ |
	SPIM_CMNCR_IO0FV |
	SPIM_CMNCR_IO2FV |
	SPIM_CMNCR_IO3FV |
	SPIM_CMNCR_MOIIO0 |
	SPIM_CMNCR_MOIIO1 |
	SPIM_CMNCR_MOIIO2 |
	SPIM_CMNCR_MOIIO3;
/* IOn maintained as Hi-Z at idle
 * IO0 Value sets Hi-Z while dummy and 1-bit read transfer phase
 * IO2/IO3 Value sets high while 1-bit command/address/additional-data phase
 */
static const uint32_t cmncr_init_value =
	1u << SPIM_CMNCR_MD_POS |
	SPIM_CMNCR_BSZ_SINGLE << SPIM_CMNCR_BSZ_POS |
	SPIM_CMNCR_IO_HIZ << SPIM_CMNCR_IO0FV_POS |
	SPIM_CMNCR_IO_HIGH << SPIM_CMNCR_IO2FV_POS |
	SPIM_CMNCR_IO_HIGH << SPIM_CMNCR_IO3FV_POS |
	SPIM_CMNCR_IO_HIZ << SPIM_CMNCR_MOIIO0_POS |
	SPIM_CMNCR_IO_HIZ << SPIM_CMNCR_MOIIO1_POS |
	SPIM_CMNCR_IO_HIZ << SPIM_CMNCR_MOIIO2_POS |
	SPIM_CMNCR_IO_HIZ << SPIM_CMNCR_MOIIO3_POS;

static const uint32_t ssldr_init_mask =
	SPIM_SSLDR_SCKDL |
	SPIM_SSLDR_SLNDL |
	SPIM_SSLDR_SPNDL;
static const uint32_t ssldr_init_value =
	0u << SPIM_SSLDR_SCKDL_POS |
	0u << SPIM_SSLDR_SLNDL_POS |
	0u << SPIM_SSLDR_SPNDL_POS;

static const uint32_t drcr_init_mask =
	SPIM_DRCR_RBURST |
	SPIM_DRCR_RCF |
	SPIM_DRCR_RBE |
	SPIM_DRCR_SSLE;
static const uint32_t drcr_init_value =
	7u << SPIM_DRCR_RBURST_POS |
	1u << SPIM_DRCR_RCF_POS |
	1u << SPIM_DRCR_RBE_POS |
	1u << SPIM_DRCR_SSLE_POS;

static const uint32_t drear_init_mask =
	SPIM_DREAR_EAC |
	SPIM_DREAR_EAV;
static const uint32_t drear_init_value =
	3u << SPIM_DREAR_EAC_POS |
	0u << SPIM_DREAR_EAV_POS;

static const uint32_t drdrenr_init_mask =
	SPIM_DRDRENR_HYPE |
	SPIM_DRDRENR_ADDRE |
	SPIM_DRDRENR_OPDRE |
	SPIM_DRDRENR_DRDRE;
static const uint32_t drdrenr_init_value =
	0u << SPIM_DRDRENR_HYPE_POS |
	0u << SPIM_DRDRENR_ADDRE_POS |
	0u << SPIM_DRDRENR_OPDRE_POS |
	0u << SPIM_DRDRENR_DRDRE_POS;

static const uint32_t phycnt_init_mask =
	SPIM_PHYCNT_ALT_ALIGN |
	SPIM_PHYCNT_CAL |
	SPIM_PHYCNT_CKSEL |
	SPIM_PHYCNT_EXDS |
	SPIM_PHYCNT_HS |
	SPIM_PHYCNT_OCT |
	SPIM_PHYCNT_OCTA |
	SPIM_PHYCNT_PHYMEM |
	SPIM_PHYCNT_WBUF2 |
	SPIM_PHYCNT_WBUF;
static const uint32_t phycnt_init_value =
	0u << SPIM_PHYCNT_ALT_ALIGN_POS |
	0u << SPIM_PHYCNT_CAL_POS |
	3u << SPIM_PHYCNT_CKSEL_POS |
	0u << SPIM_PHYCNT_EXDS_POS |
	0u << SPIM_PHYCNT_HS_POS |
	0u << SPIM_PHYCNT_OCT_POS |
	0u << SPIM_PHYCNT_OCTA_POS |
	0u << SPIM_PHYCNT_PHYMEM_POS |
	0u << SPIM_PHYCNT_WBUF2_POS |
	0u << SPIM_PHYCNT_WBUF_POS;

static const uint32_t phyoffset1_init_mask =
	SPIM_PHYOFFSET1_DDRTMG;
static const uint32_t phyoffset1_init_value =
	SPIM_PHYOFFSET1_SDR << SPIM_PHYOFFSET1_DDRTMG_POS;

static const uint32_t phyoffset2_init_mask =
	SPIM_PHYOFFSET2_OCTTMG;
static const uint32_t phyoffset2_init_value =
	SPIM_PHYOFFSET2_SPI << SPIM_PHYOFFSET2_OCTTMG_POS;

static void spim_ip_init(spim_ctrl_t *myctrl)
{
	cpg_set_xspi_clock(XSPI_CLOCK_SPIM, spi_clock * 4);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_CMNCR, cmncr_init_mask, cmncr_init_value);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SSLDR, ssldr_init_mask, ssldr_init_value);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_DRCR, drcr_init_mask, drcr_init_value);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_DREAR, drear_init_mask, drear_init_value);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_DRDRENR, drdrenr_init_mask, drdrenr_init_value);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYCNT, phycnt_init_mask, phycnt_init_value|SPIM_PHYCNT_CAL);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYOFFSET1, phyoffset1_init_mask, phyoffset1_init_value);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYOFFSET2, phyoffset2_init_mask, phyoffset2_init_value);
	spim_init_phy(myctrl);
}

static int spim_open(xspi_ctrl_t * ctrl, xspi_cfg_t const * const cfg)
{
	int result = -1;
	assert(ctrl);
	assert(cfg);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	spim_ext_t * ext = (spim_ext_t*)cfg->extend;

	if (myctrl->opened) return -1;

	myctrl->reg_base = cfg->base;
	myctrl->mmap_base = 0x20000000;
	myctrl->mmap_size = 0x10000000;
	assert(myctrl->reg_base);

	if (!globally_initialised) {
		select_spim(myctrl, cfg);
		spim_ip_init(myctrl);

		globally_initialised = true;
	}

	/* Override calibration base if specified */
	uint32_t calibration_offset = DEFAULT_CALIBRATION_OFFSET;
	if (ext && ext->calibration_base != 0) {
		calibration_offset = ext->calibration_base;
	}
	if (calibration_offset == UINT32_MAX) {
		/* Calculate offset from Flash base */
		uintptr_t offset = (uintptr_t)&dtr_pattern;
		offset -= load_base;
		offset += 0x200;
		assert(offset <= UINT32_MAX);
		myctrl->calibration_base = (uint32_t)offset;
	}
	else {
		myctrl->calibration_base = calibration_offset;
	}

	result = 0;
	myctrl->opened = true;

	return result;
}

static int spim_close(xspi_ctrl_t * const ctrl)
{
	int result = -1;
	assert(ctrl);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	if (myctrl->opened) {
		myctrl->opened = false;
		result = 0;
	}
	return result;

}

static int spim_post_init(xspi_ctrl_t * const ctrl)
{
	int result = 0;
	return result;
}

static int spim_reduce_frequency(spim_ctrl_t * const ctrl, bool dtr)
{
	int old_freq = spi_clock;
	assert(ctrl);
	if (dtr && spi_clock > RZ_XSPI_DTR_FREQ_LIMIT) {
		spi_clock = RZ_XSPI_DTR_FREQ_LIMIT;
	}
	else if (!dtr && spi_clock > RZ_XSPI_STR_FREQ_LIMIT) {
		spi_clock = RZ_XSPI_STR_FREQ_LIMIT;
	}
	else {
		/* We do not need to reduce the current SPI frequency */
		return 0;
	}

	int result = cpg_set_xspi_clock(XSPI_CLOCK_SPIM, spi_clock * 4);
	if (result != 0) {
		return result;
	}

	int actual_freq = cpg_get_xspi_clock(XSPI_CLOCK_SPIM);
	if (actual_freq == -1) return -1;
	spi_clock = actual_freq / 4;

	WARN("SPIM: Reduces the SPI frequency from %d to %d.\n", old_freq, spi_clock);

	return 0;
}

static void spim_start_xip_internal(spim_ctrl_t * const ctrl)
{
	bool is_dtr = !!(mmio_read_32(SPIM_DRDRENR)&SPIM_DRDRENR_DRDRE);
	spim_reduce_frequency(ctrl, is_dtr);
	mmio_clrbits_32(ctrl->reg_base + SPIM_CMNCR, SPIM_CMNCR_MD);
	mmio_read_32(ctrl->reg_base + SPIM_CMNCR);
}

static void test_sslf(spim_ctrl_t * myctrl)
{
	wait_until_32(myctrl->reg_base + SPIM_CMNSR, SPIM_CMNSR_SSLF, 0);
}

static int spim_stop_xip_internal(spim_ctrl_t * myctrl)
{
	int result = 0;
	uint32_t drcr = mmio_read_32(myctrl->reg_base + SPIM_DRCR);
	if ((drcr & (SPIM_DRCR_RBE|SPIM_DRCR_SSLE)) == (SPIM_DRCR_RBE|SPIM_DRCR_SSLE)) {
		/* Set SSLN and wait for sslf */
		mmio_write_32(myctrl->reg_base + SPIM_DRCR, drcr|SPIM_DRCR_SSLN);
		test_sslf(myctrl);
	}
	test_tend(myctrl);

	/* Set MD bit */
	mmio_setbits_32(myctrl->reg_base + SPIM_CMNCR, SPIM_CMNCR_MD);

	return result;
}

static bool spim_stop_xip_temporarily(spim_ctrl_t * myctrl)
{
	/* Stop XIP and return previous state*/
	bool state = !(mmio_read_32(myctrl->reg_base + SPIM_CMNCR) & SPIM_CMNCR_MD);
	spim_stop_xip_internal(myctrl);
	return state;
}

struct delay_setting {
	uint8_t y:1;
	uint8_t :3;
	uint8_t x:1;
	uint8_t :1;
	uint8_t zz:2;
};

static const struct delay_setting adj_table[] = {
	{ .y=1, .zz=3, .x=0},
	{ .y=1, .zz=3, .x=1},
	{ .y=1, .zz=2, .x=0},
	{ .y=1, .zz=2, .x=1},
	{ .y=1, .zz=1, .x=0},
	{ .y=0, .zz=3, .x=0},
	{ .y=0, .zz=3, .x=1},
	{ .y=0, .zz=2, .x=0},
	{ .y=0, .zz=2, .x=1},
	{ .y=0, .zz=1, .x=0},
	{ .y=0, .zz=1, .x=1},
	{ .y=0, .zz=0, .x=0},
	{ .y=0, .zz=0, .x=1},
};
#define NUM_ADJ_TABLES ARRAY_SIZE(adj_table)

/* Update delay */
static void spim_set_delay(const spim_ctrl_t * myctrl, const struct delay_setting * delay)
{
	uint32_t cksel_mask = SPIM_PHYCNT_CKSEL;
	uint32_t cksel_value = delay->zz << SPIM_PHYCNT_CKSEL_POS;
	uint32_t phyadj2 = delay->x << 4 | delay->y << 0;
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYCNT, cksel_mask, cksel_value|SPIM_PHYCNT_CAL);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ2, phyadj2);
	mmio_write_32(myctrl->reg_base + SPIM_PHYADJ1, 0x80000032);
}

/* Run simple read op */
static uint32_t spim_internal_read32(const spim_ctrl_t * myctrl, const uint32_t adr)
{
	mmio_write_32(myctrl->reg_base + SPIM_SMADR, adr);
	uint32_t smcr = SPIM_SMCR_SPIE | SPIM_SMCR_SPIRE;
	mmio_write_32(myctrl->reg_base + SPIM_SMCR, smcr);
	test_tend(myctrl);
	return mmio_read_32(myctrl->reg_base + SPIM_SMRDR0);
}

/* Find the most longest good part of stat */
static int find_longest_part(bool *stat, int count, int *top, int *len)
{
	int last_top;
	int last_len;
	int end;
	last_top = 0;
	*len = 0;
	while (last_top < count) {
		for (; last_top < count && stat[last_top] == false; ++last_top);
		if (last_top >= count) break;
		for (end = last_top + 1; end < count && stat[end] == true; ++end);
		last_len = end - last_top;
		if (*len < last_len) {
			*len = last_len;
			*top = last_top;
		}
		last_top = end;
	}
	if (*len < 3) return -1;
	return 0;
}

/* Start calibration */
static int spim_internal_manual_calibration(spim_ctrl_t * myctrl)
{
	bool is_xip;
	int index;
	int result;
	bool delay_stat[NUM_ADJ_TABLES];
	int top;
	int len;

	is_xip = spim_stop_xip_temporarily(myctrl);
	result = spim_start_calibration(myctrl);

	INFO("SPIM calibration info\n");
	INFO("[y zz x]\n");

	if (result == 0) {
		/* Store the per-delay results */
		for (index = 0; index < NUM_ADJ_TABLES; index++) {
			spim_set_delay(myctrl, &adj_table[index]);
			uint32_t read = spim_internal_read32(myctrl, myctrl->calibration_base);
			INFO("[%d %d%d %d]=%08x\n",
				adj_table[index].y, adj_table[index].zz>1?1:0, (adj_table[index].zz&1)?1:0, adj_table[index].x,
				bswap32(read));
			delay_stat[index] = !!(read == dtr_pattern);
		}

		result = find_longest_part(delay_stat, NUM_ADJ_TABLES, &top, &len);
	}

	if (DEBUG) {
		char debugstr[NUM_ADJ_TABLES+1];
		for (index = 0; index < NUM_ADJ_TABLES; index++) {
			debugstr[index] = delay_stat[index] ? '1' : '0';
		}
		debugstr[index] = '\0';
		INFO("%s\n",debugstr);
	}

	if (result == 0) {
		/* Choose median to delay index */
		index = top + len / 2;
		spim_set_delay(myctrl, &adj_table[index]);
		INFO("center=%d\n", index);
	}

	if (is_xip) {
		/* Resume XIP state */
		spim_start_xip_internal(myctrl);
	}
	return result;
}

static int calibration_check(spim_ctrl_t * myctrl)
{
	if (!calibration_done) {
		int res = spim_internal_manual_calibration(myctrl);
		calibration_done = !(res < 0);
		return res;
	}
	return 0;
}

static const uint32_t smcmr_clearmask =
	SPIM_SMCMR_CMD |
	SPIM_SMCMR_OCMD;

static const uint32_t smenr_clearmask =
	SPIM_SMENR_ADB|
	SPIM_SMENR_ADE|
	SPIM_SMENR_CDB|
	SPIM_SMENR_CDE|
	SPIM_SMENR_DME|
	SPIM_SMENR_SPIDB|
	SPIM_SMENR_OCDB|
	SPIM_SMENR_OCDE|
	SPIM_SMENR_OPDB|
	SPIM_SMENR_OPDE|
	SPIM_SMENR_SPIDE;

static const uint32_t smdrenr_clearmask =
	SPIM_SMDRENR_ADDRE |
	SPIM_SMDRENR_DRDRE |
	SPIM_SMDRENR_HYPE |
	SPIM_SMDRENR_OPDRE;

static const uint32_t smdmcr_clearmask =
	SPIM_SMDMCR_DMCYC;

static const uint32_t ssldr_clearmask =
	SPIM_SSLDR_SCKDL |
	SPIM_SSLDR_SLNDL |
	SPIM_SSLDR_SPNDL;

/* SMENR value for 1-1-1 command */
static const uint32_t smenr_form_111 =
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_CDB_POS |
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_OCDB_POS |
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_ADB_POS |
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_OPDB_POS |
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_SPIDB_POS;

/* SMENR value for 1-1-4 command */
static const uint32_t smenr_form_114 =
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_CDB_POS |
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_OCDB_POS |
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_ADB_POS |
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_OPDB_POS |
	SPIM_SMENR_DB_4BIT << SPIM_SMENR_SPIDB_POS;

/* SMENR value for 1-4-4 command */
static const uint32_t smenr_form_144 =
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_CDB_POS |
	SPIM_SMENR_DB_1BIT << SPIM_SMENR_OCDB_POS |
	SPIM_SMENR_DB_4BIT << SPIM_SMENR_ADB_POS |
	SPIM_SMENR_DB_4BIT << SPIM_SMENR_OPDB_POS |
	SPIM_SMENR_DB_4BIT << SPIM_SMENR_SPIDB_POS;

/* SMENR value for no opcode */
static const uint32_t smenr_op_none =
	0u << SPIM_SMENR_CDE_POS |
	0u << SPIM_SMENR_OCDE_POS;

/* SMENR value for 1-byte opcode */
static const uint32_t smenr_op_1byte =
	1u << SPIM_SMENR_CDE_POS |
	0u << SPIM_SMENR_OCDE_POS;

/* SMENR value for 2-byte opcode */
static const uint32_t smenr_op_2byte =
	1u << SPIM_SMENR_CDE_POS |
	1u << SPIM_SMENR_OCDE_POS;

/* SMENR value for no address */
static const uint32_t smenr_addr_none =
	SPIM_SMENR_ADE_NONE << SPIM_SMENR_ADE_POS;

/* SMENR value for 3-byte address */
static const uint32_t smenr_addr_3byte =
	SPIM_SMENR_ADE_3BYTE << SPIM_SMENR_ADE_POS;

/* SMENR value for 4-byte address */
static const uint32_t smenr_addr_4byte =
	SPIM_SMENR_ADE_4BYTE << SPIM_SMENR_ADE_POS;

/* SMENR value for no additional data */
static const uint32_t smenr_additional_none =
	SPIM_SMENR_OPDE_NONE << SPIM_SMENR_OPDE_POS;

/* SMENR value for 1-byte additional data */
static const uint32_t smenr_additional_1byte =
	SPIM_SMENR_OPDE_1BYTE << SPIM_SMENR_OPDE_POS;

/* SMENR value for 2-byte additional data */
static const uint32_t smenr_additional_2byte =
	SPIM_SMENR_OPDE_2BYTE << SPIM_SMENR_OPDE_POS;

/* SMENR value for 3-byte additional data */
static const uint32_t smenr_additional_3byte =
	SPIM_SMENR_OPDE_3BYTE << SPIM_SMENR_OPDE_POS;

/* SMENR value for 4-byte additional data */
static const uint32_t smenr_additional_4byte =
	SPIM_SMENR_OPDE_4BYTE << SPIM_SMENR_OPDE_POS;

/* SMDRENR value that indicate the address phase is by DDR */
static const uint32_t smdrenr_address_is_ddr =
	1u << SPIM_SMDRENR_ADDRE_POS |
	1u << SPIM_SMDRENR_OPDRE_POS;

/* SMDRENR value that indicate the data phase is by DDR */
static const uint32_t smdrenr_data_is_ddr =
	1u << SPIM_SMDRENR_DRDRE_POS;

static void send_256(spim_ctrl_t * myctrl, xspi_transfer_form_t form, uintptr_t buffer, uint32_t smenr)
{
	/* Use wbuffer for transfer */
	mmio_setbits_32(myctrl->reg_base + SPIM_DRCR, SPIM_DRCR_RCF);
	mmio_setbits_32(myctrl->reg_base + SPIM_PHYCNT, SPIM_PHYCNT_WBUF2|SPIM_PHYCNT_WBUF|SPIM_PHYCNT_CAL);
	if (form == SPI_FORM_1_1_4 || form == SPI_FORM_1_4_4) {
		uint32_t phyoffset2_msk = SPIM_PHYOFFSET2_OCTTMG;
		uint32_t phyoffset2_set = SPIM_PHYOFFSET2_SPI_WBUF << SPIM_PHYOFFSET2_OCTTMG_POS;
		mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYOFFSET2, phyoffset2_msk, phyoffset2_set);
	}
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMENR, smenr_clearmask ,smenr | (SPIM_SMENR_SPIDE_LONG<<SPIM_SMENR_SPIDE_POS));
	/* Write data to buffer */
	for (int bytes = 0; bytes < 256; bytes += 8) {
		mmio_write_64(myctrl->reg_base + SPIM_BUFFER + bytes, *(uint64_t*)(buffer+bytes));
	}
}

static void send_4(spim_ctrl_t * myctrl, uintptr_t buffer, uint32_t smenr)
{
	/* Set SPIDE */
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMENR, smenr_clearmask, smenr | (SPIM_SMENR_SPIDE_LONG<<SPIM_SMENR_SPIDE_POS));
	/* Write data to SMWDR */
	mmio_write_32(myctrl->reg_base + SPIM_SMWDR0, *(uint32_t*)buffer);
}

static void send_2(spim_ctrl_t * myctrl, uintptr_t buffer, uint32_t smenr)
{
	/* Set SPIDE */
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMENR, smenr_clearmask, smenr | (SPIM_SMENR_SPIDE_WORD<<SPIM_SMENR_SPIDE_POS));
	/* Write data to SMWDR */
	mmio_write_16(myctrl->reg_base + SPIM_SMWDR0, *(uint16_t*)buffer);
}

static void send_1(spim_ctrl_t * myctrl, uintptr_t buffer, uint32_t smenr)
{
	/* Set SPIDE */
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMENR, smenr_clearmask, smenr | (SPIM_SMENR_SPIDE_BYTE<<SPIM_SMENR_SPIDE_POS));
	/* Write data to SMWDR */
	mmio_write_8(myctrl->reg_base + SPIM_SMWDR0, *(uint8_t*)buffer);
}

static void receive(spim_ctrl_t * myctrl, uintptr_t buffer, uint32_t smenr)
{
	/* Set SPIDE */
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMENR, smenr_clearmask, smenr | (SPIM_SMENR_SPIDE_LONG<<SPIM_SMENR_SPIDE_POS));
}

static void no_data(spim_ctrl_t * myctrl, uintptr_t buffer, uint32_t smenr)
{
	/* No transfer data */
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMENR, smenr_clearmask, smenr | (SPIM_SMENR_SPIDE_NONE<<SPIM_SMENR_SPIDE_POS));
}

static int spim_exec_op(xspi_ctrl_t * const ctrl, xspi_op_t const * const op, bool is_write)
{
	/* Check parameters */
	assert(ctrl);
	assert(op);
	assert(op->transfer_size == 0 || (op->transfer_size && op->transfer_buffer));
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	bool is_xip;

	switch (op->form) {
	case SPI_FORM_1_1_1:
	case SPI_FORM_1_1_4:
	case SPI_FORM_1_4_4:
		break;
	default:
		ERROR("Unsupported transfer form %d", op->form);
		return -1;
	}

	/* Save XIP state and stop XIP */
	is_xip = spim_stop_xip_temporarily(myctrl);

	/* Do calibration and reduce freq. if transfer is DTR read */
	if (op->transfer_is_ddr && !is_write && op->transfer_size) {
		spim_reduce_frequency(ctrl, true);
		int res = calibration_check(myctrl);
		if (res < 0) return res;
	}
	else {
		spim_reduce_frequency(ctrl, true);
	}

	/* Create values to write the registers */
	uint32_t smcmr_set = 0;
	uint32_t smenr_set = 0;
	uint32_t smdrenr_set = 0;
	uint32_t smopr = 0;
	uint32_t smadr = op->address;
	uint32_t smdmcr_set = 0;
	uint32_t ssldr_set = 0;
	uint32_t save_ssldr = mmio_read_32(myctrl->reg_base + SPIM_SSLDR);
	uint32_t save_phyoffset1 = mmio_read_32(myctrl->reg_base + SPIM_PHYOFFSET1);
	uint32_t save_phyoffset2 = mmio_read_32(myctrl->reg_base + SPIM_PHYOFFSET2);
	uint32_t save_phycnt = mmio_read_32(myctrl->reg_base + SPIM_PHYCNT);

	/* Command form */
	switch (op->form) {
	case SPI_FORM_1_1_1:
		smenr_set |= smenr_form_111;
		break;
	case SPI_FORM_1_1_4:
		smenr_set |= smenr_form_114;
		break;
	case SPI_FORM_1_4_4:
		smenr_set |= smenr_form_144;
		break;
	default:
		ERROR("Unsupported transfer form %d", op->form);
		return -1;
	}

	/* Opcode */
	switch (op->op_size) {
	case 0:
		smenr_set |= smenr_op_none;
		break;
	case 1:
		smenr_set |= smenr_op_1byte;
		smcmr_set |= (op->op & 0xff) << SPIM_SMCMR_CMD_POS;
		break;
	case 2:
		smenr_set |= smenr_op_2byte;
		smcmr_set |= (op->op & 0xff) << SPIM_SMCMR_OCMD_POS;
		smcmr_set |= (op->op & 0xff00) >> 8 << SPIM_SMCMR_CMD_POS;
		break;
	default:
		ERROR("Unsupported op size %d", op->op_size);
		return -1;
	}

	/* Address */
	switch (op->address_size) {
	case 0:
		smenr_set |= smenr_addr_none;
		break;
	case 3:
		smenr_set |= smenr_addr_3byte;
		break;
	case 4:
		smenr_set |= smenr_addr_4byte;
		break;
	default:
		ERROR("Unsupported address size %d", op->address_size);
		return -1;
	}

	/* Additional data */
	switch (op->additional_size) {
	case 0:
		smenr_set |= smenr_additional_none;
		break;
	case 1:
		smenr_set |= smenr_additional_1byte;
		smopr = (op->additional_value & 0xff) << SPIM_SMOPR_OPD3_POS;
		break;
	case 2:
		smenr_set |= smenr_additional_2byte;
		smopr = (op->additional_value & 0xff) << SPIM_SMOPR_OPD2_POS;
		smopr |= (op->additional_value & 0xff00) >> 8 << SPIM_SMOPR_OPD3_POS;
		break;
	case 3:
		smenr_set |= smenr_additional_3byte;
		smopr = (op->additional_value & 0xff) << SPIM_SMOPR_OPD1_POS;
		smopr |= (op->additional_value & 0xff00) >> 8 << SPIM_SMOPR_OPD2_POS;
		smopr |= (op->additional_value & 0xff0000) >> 16 << SPIM_SMOPR_OPD3_POS;
		break;
	case 4:
		smenr_set |= smenr_additional_4byte;
		smopr = (op->additional_value & 0xff) << SPIM_SMOPR_OPD0_POS;
		smopr |= (op->additional_value & 0xff00) >> 8 << SPIM_SMOPR_OPD1_POS;
		smopr |= (op->additional_value & 0xff0000) >> 16 << SPIM_SMOPR_OPD2_POS;
		smopr |= (op->additional_value & 0xff000000) >> 24 << SPIM_SMOPR_OPD3_POS;
		break;
	default:
		ERROR("Unsupported additional size %d", op->additional_size);
		return -1;
	}

	/* Dummy cycle */
	if (op->dummy_cycles == 0) {
		smenr_set |= 0u << SPIM_SMENR_DME_POS;
		smdmcr_set |= 0u << SPIM_SMDMCR_DMCYC_POS;
	}
	else if (op->dummy_cycles == 1 || op->dummy_cycles > 20) {
		ERROR("Unsupported dummy cycle count %d", op->dummy_cycles);
		return -1;
	}
	else {
		smenr_set |= 1u << SPIM_SMENR_DME_POS;
		smdmcr_set |= (op->dummy_cycles - 1) << SPIM_SMDMCR_DMCYC_POS;
	}

	/* DDR indicator for address */
	if (op->address_is_ddr) {
		smdrenr_set |= smdrenr_address_is_ddr;
	}

	/* DDR indicator for data */
	if (op->transfer_is_ddr) {
		smdrenr_set |= smdrenr_data_is_ddr;
	}

	/* PHYOFFSET1 setting */
	uint32_t phyoffset1_msk = SPIM_PHYOFFSET1_DDRTMG;
	uint32_t phyoffset1_set;
	if(op->transfer_is_ddr) {
		phyoffset1_set = SPIM_PHYOFFSET1_DDR << SPIM_PHYOFFSET1_DDRTMG_POS;
	}
	else {
		phyoffset1_set = SPIM_PHYOFFSET1_SDR << SPIM_PHYOFFSET1_DDRTMG_POS;
	}

	/* PHYCNT setting */
	uint32_t phycnt_msk = SPIM_PHYCNT_PHYMEM;
	uint32_t phycnt_set;
	if(op->address_is_ddr || op->transfer_is_ddr) {
		phycnt_set = SPIM_PHYCNT_DDR << SPIM_PHYCNT_PHYMEM_POS;
	}
	else {
		phycnt_set = SPIM_PHYCNT_SDR << SPIM_PHYCNT_PHYMEM_POS;
	}

	/* SLCH (SSL assert to CLK high) */
	if (op->slch_value < 8) {
		ssldr_set |= op->slch_value << SPIM_SSLDR_SCKDL_POS;
	}
	else {
		ERROR("Unsupported slch_value %d", op->slch_value);
		return -1;
	}

	/* CLSH (CLK low tp SSL negative) */
	if (op->clsh_value < 8) {
		ssldr_set |= op->clsh_value << SPIM_SSLDR_SLNDL_POS;
	}
	else {
		ERROR("Unsupported clsh_value %d", op->clsh_value);
		return -1;
	}

	/* SHSL (SSL negative to SSL assert) */
	if (op->shsl_value < 8) {
		ssldr_set |= op->shsl_value << SPIM_SSLDR_SPNDL_POS;
	}
	else {
		ERROR("Unsupported shsl_value %d", op->shsl_value);
		return -1;
	}

	/* Write the register */
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMCMR, smcmr_clearmask, smcmr_set);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMDRENR, smdrenr_clearmask, smdrenr_set);
	mmio_write_32(myctrl->reg_base + SPIM_SMADR, smadr);
	mmio_write_32(myctrl->reg_base + SPIM_SMOPR, smopr);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SMDMCR, smdmcr_clearmask, smdmcr_set);
	mmio_write_32(myctrl->reg_base + SPIM_SSLDR, (save_ssldr & ~ssldr_clearmask) | ssldr_set);
	mmio_write_32(myctrl->reg_base + SPIM_PHYOFFSET1, (save_phyoffset1 & ~phyoffset1_msk) | phyoffset1_set);
	mmio_write_32(myctrl->reg_base + SPIM_PHYCNT, (save_phycnt & ~phycnt_msk) | phycnt_set | SPIM_PHYCNT_CAL);

	int64_t remain = (int64_t)op->transfer_size;
	uintptr_t buffer = (uintptr_t)op->transfer_buffer;
	uint32_t smcr;

	/* Enable transmit */
	if (is_write && remain) smcr = SPIM_SMCR_SPIWE | SPIM_SMCR_SPIE;
	else if (remain) smcr = SPIM_SMCR_SPIRE | SPIM_SMCR_SPIE;
	else smcr = SPIM_SMCR_SPIE;

	do {
		uint32_t xfer_count;

		/* Block size per single transfer */
		if (is_write && remain >= 256) {
			send_256(myctrl, op->form, buffer, smenr_set);
			xfer_count = 256;
		}
		else if (is_write && remain >= 4) {
			send_4(myctrl, buffer, smenr_set);
			xfer_count = 4;
		}
		else if (is_write && remain >= 2) {
			send_2(myctrl, buffer, smenr_set);
			xfer_count = 2;
		}
		else if (is_write && remain) {
			send_1(myctrl, buffer, smenr_set);
			xfer_count = 1;
		}
		else if (remain) {
			receive(myctrl, buffer, smenr_set);
			xfer_count = 4;
		}
		else {
			no_data(myctrl, buffer, smenr_set);
			xfer_count = 0;
		}

		/* Set SSLKP if transaction is remained and write transfer */
		if (is_write && remain > xfer_count) {
			smcr |= SPIM_SMCR_SSLKP;
		}
		else {
			smcr &= ~SPIM_SMCR_SSLKP;
		}

		/* Exec transaction */
		mmio_write_32(myctrl->reg_base + SPIM_SMCR, smcr);
		test_tend(myctrl);

		/* Store received data */
		if (!is_write && remain) {
			uint32_t smrdr = mmio_read_32(myctrl->reg_base + SPIM_SMRDR0);
			if (remain > 3) {
				*(uint32_t*)buffer = smrdr;
			}
			else {
				*(uint8_t*)buffer = (smrdr >> 24) & 255;
				if (remain > 1) *(uint8_t*)(buffer+1) = (smrdr >> 16) & 255;
				if (remain > 2) *(uint8_t*)(buffer+2) = (smrdr >> 8) & 255;
			}

		}
		remain -= xfer_count;
		buffer += xfer_count;

		/* Clear write buffer flag and restore OCTTMG */
		mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYCNT, SPIM_PHYCNT_WBUF2|SPIM_PHYCNT_WBUF, SPIM_PHYCNT_CAL);
		mmio_write_32(myctrl->reg_base + SPIM_PHYOFFSET2, save_phyoffset2);

		if (remain > 0) {
			if (!is_write) {
				/* Increment address */
				smadr += xfer_count;
				mmio_write_32(myctrl->reg_base + SPIM_SMADR, smadr);
			}
			else {
				/* Clear enabler bits for continuous write access */
				smenr_set &= ~(SPIM_SMENR_DME|SPIM_SMENR_CDE|SPIM_SMENR_OCDE|SPIM_SMENR_ADE|SPIM_SMENR_OPDE);
			}
		}
	} while (remain > 0);

	/* Resume regs */
	mmio_write_32(myctrl->reg_base + SPIM_SSLDR, save_ssldr);
	mmio_write_32(myctrl->reg_base + SPIM_PHYCNT, save_phycnt|SPIM_PHYCNT_CAL);
	mmio_write_32(myctrl->reg_base + SPIM_PHYOFFSET1, save_phyoffset1);

	if (is_xip) {
		/* Resume XIP state */
		spim_start_xip_internal(ctrl);
	}

	return 0;
}

static const uint32_t drcmr_clearmask =
	SPIM_DRCMR_CMD |
	SPIM_DRCMR_OCMD;

static const uint32_t drenr_clearmask =
	SPIM_DRENR_ADB|
	SPIM_DRENR_ADE|
	SPIM_DRENR_CDB|
	SPIM_DRENR_CDE|
	SPIM_DRENR_DME|
	SPIM_DRENR_DRDB|
	SPIM_DRENR_OCDB|
	SPIM_DRENR_OCDE|
	SPIM_DRENR_OPDB|
	SPIM_DRENR_OPDE;

static const uint32_t drdrenr_clearmask =
	SPIM_DRDRENR_ADDRE |
	SPIM_DRDRENR_DRDRE |
	SPIM_DRDRENR_HYPE |
	SPIM_DRDRENR_OPDRE;

static const uint32_t drdmcr_clearmask =
	SPIM_DRDMCR_DMCYC;

/* DRENR value for 1-1-1 command */
static const uint32_t drenr_form_111 =
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_CDB_POS |
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_OCDB_POS |
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_ADB_POS |
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_OPDB_POS |
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_DRDB_POS;

/* DRENR value for 1-1-4 command */
static const uint32_t drenr_form_114 =
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_CDB_POS |
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_OCDB_POS |
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_ADB_POS |
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_OPDB_POS |
	SPIM_DRENR_DB_4BIT << SPIM_DRENR_DRDB_POS;

/* DRENR value for 1-4-4 command */
static const uint32_t drenr_form_144 =
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_CDB_POS |
	SPIM_DRENR_DB_1BIT << SPIM_DRENR_OCDB_POS |
	SPIM_DRENR_DB_4BIT << SPIM_DRENR_ADB_POS |
	SPIM_DRENR_DB_4BIT << SPIM_DRENR_OPDB_POS |
	SPIM_DRENR_DB_4BIT << SPIM_DRENR_DRDB_POS;

/* DRENR value for no opcode */
static const uint32_t drenr_op_none =
	0u << SPIM_DRENR_CDE_POS |
	0u << SPIM_DRENR_OCDE_POS;

/* DRENR value for 1-byte opcode */
static const uint32_t drenr_op_1byte =
	1u << SPIM_DRENR_CDE_POS |
	0u << SPIM_DRENR_OCDE_POS;

/* DRENR value for 2-byte opcode */
static const uint32_t drenr_op_2byte =
	1u << SPIM_DRENR_CDE_POS |
	1u << SPIM_DRENR_OCDE_POS;

/* DRENR value for no address */
static const uint32_t drenr_addr_none =
	SPIM_DRENR_ADE_NONE << SPIM_DRENR_ADE_POS;

/* DRENR value for 3-byte address */
static const uint32_t drenr_addr_3byte =
	SPIM_DRENR_ADE_3BYTE << SPIM_DRENR_ADE_POS;

/* DRENR value for 4-byte address */
static const uint32_t drenr_addr_4byte =
	SPIM_DRENR_ADE_4BYTE << SPIM_DRENR_ADE_POS;

/* DRENR value for no additional data */
static const uint32_t drenr_additional_none =
	SPIM_DRENR_OPDE_NONE << SPIM_DRENR_OPDE_POS;

/* DRENR value for 1-byte additional data */
static const uint32_t drenr_additional_1byte =
	SPIM_DRENR_OPDE_1BYTE << SPIM_DRENR_OPDE_POS;

/* DRENR value for 2-byte additional data */
static const uint32_t drenr_additional_2byte =
	SPIM_DRENR_OPDE_2BYTE << SPIM_DRENR_OPDE_POS;

/* DRENR value for 3-byte additional data */
static const uint32_t drenr_additional_3byte =
	SPIM_DRENR_OPDE_3BYTE << SPIM_DRENR_OPDE_POS;

/* DRENR value for 4-byte additional data */
static const uint32_t drenr_additional_4byte =
	SPIM_DRENR_OPDE_4BYTE << SPIM_DRENR_OPDE_POS;

/* DRDRENR value that indicate the address phase is by DDR */
static const uint32_t drdrenr_address_is_ddr =
	1u << SPIM_DRDRENR_ADDRE_POS |
	1u << SPIM_DRDRENR_OPDRE_POS;

/* DRDRENR value that indicate the data phase is by DDR */
static const uint32_t drdrenr_data_is_ddr =
	1u << SPIM_DRDRENR_DRDRE_POS;

static int spim_configure_xip(xspi_ctrl_t * const ctrl, xspi_op_t const * const rop, xspi_op_t const * const wop)
{
	assert(ctrl);
	assert(rop);
	assert(!wop);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;

	uint32_t drcmr = mmio_read_32(myctrl->reg_base + SPIM_DRCMR);
	uint32_t drenr = mmio_read_32(myctrl->reg_base + SPIM_DRENR);
	uint32_t drdrenr = mmio_read_32(myctrl->reg_base + SPIM_DRDRENR);
	uint32_t dropr = 0;
	uint32_t drdmcr = mmio_read_32(myctrl->reg_base + SPIM_DRDMCR);
	uint32_t ssldr_clear = 0;
	uint32_t ssldr_set = 0;
	uint32_t phyoffset1_msk = SPIM_PHYOFFSET1_DDRTMG;
	uint32_t phyoffset1_set = 0;
	uint32_t phyoffset2_msk = SPIM_PHYOFFSET2_OCTTMG;
	uint32_t phyoffset2_set = SPIM_PHYOFFSET2_SPI << SPIM_PHYOFFSET2_OCTTMG_POS;
	uint32_t phycnt_msk = SPIM_PHYCNT_PHYMEM;
	uint32_t phycnt_set = 0;

	drenr &= ~drenr_clearmask;
	drcmr &= ~drcmr_clearmask;
	drdrenr &= ~drdrenr_clearmask;
	drdmcr &= ~drdmcr_clearmask;

	/* Command form */
	switch (rop->form) {
	case SPI_FORM_1_1_1:
		drenr |= drenr_form_111;
		break;
	case SPI_FORM_1_1_4:
		drenr |= drenr_form_114;
		break;
	case SPI_FORM_1_4_4:
		drenr |= drenr_form_144;
		break;
	default:
		ERROR("Unsupported transfer form %d for rop", rop->form);
		return -1;
	}

	/* Opcode */
	switch (rop->op_size) {
	case 0:
		drenr |= drenr_op_none;
		break;
	case 1:
		drenr |= drenr_op_1byte;
		drcmr |= (rop->op & 0xff) << SPIM_DRCMR_CMD_POS;
		break;
	case 2:
		drenr |= drenr_op_2byte;
		drcmr |= (rop->op & 0xff) << SPIM_DRCMR_OCMD_POS;
		drcmr |= (rop->op & 0xff00) >> 8 << SPIM_DRCMR_CMD_POS;
		break;
	default:
		ERROR("Unsupported op size %d for rop", rop->op_size);
		return -1;
	}

	/* Address */
	switch (rop->address_size) {
	case 0:
		drenr |= drenr_addr_none;
		break;
	case 3:
		drenr |= drenr_addr_3byte;
		break;
	case 4:
		drenr |= drenr_addr_4byte;
		break;
	default:
		ERROR("Unsupported address size %d for rop", rop->address_size);
		return -1;
	}

	/* Additional data */
	switch (rop->additional_size) {
	case 0:
		drenr |= drenr_additional_none;
		break;
	case 1:
		drenr |= drenr_additional_1byte;
		dropr |= (rop->additional_value & 0xff) << SPIM_DROPR_OPD3_POS;
		break;
	case 2:
		drenr |= drenr_additional_2byte;
		dropr |= (rop->additional_value & 0xff) << SPIM_DROPR_OPD2_POS;
		dropr |= (rop->additional_value & 0xff00) >> 8 << SPIM_DROPR_OPD3_POS;
		break;
	case 3:
		drenr |= drenr_additional_3byte;
		dropr |= (rop->additional_value & 0xff) << SPIM_DROPR_OPD1_POS;
		dropr |= (rop->additional_value & 0xff00) >> 8 << SPIM_DROPR_OPD2_POS;
		dropr |= (rop->additional_value & 0xff0000) >> 16 << SPIM_DROPR_OPD3_POS;
		break;
	case 4:
		drenr |= drenr_additional_4byte;
		dropr |= (rop->additional_value & 0xff) << SPIM_DROPR_OPD0_POS;
		dropr |= (rop->additional_value & 0xff00) >> 8 << SPIM_DROPR_OPD1_POS;
		dropr |= (rop->additional_value & 0xff0000) >> 16 << SPIM_DROPR_OPD2_POS;
		dropr |= (rop->additional_value & 0xff000000) >> 24 << SPIM_DROPR_OPD3_POS;
		break;
	default:
		ERROR("Unsupported additional size %d for rop", rop->additional_size);
		return -1;
	}

	/* Dummy cycle */
	if (rop->dummy_cycles == 0) {
		drenr |= 0u << SPIM_DRENR_DME_POS;
		drdmcr |= 0u << SPIM_DRDMCR_DMCYC_POS;
	}
	else if (rop->dummy_cycles == 1 || rop->dummy_cycles > 20) {
		ERROR("Unsupported dummy cycle count %d for rop", rop->dummy_cycles);
		return -1;
	}
	else {
		drenr |= 1u << SPIM_DRENR_DME_POS;
		drdmcr |= (rop->dummy_cycles - 1) << SPIM_DRDMCR_DMCYC_POS;
	}

	/* DDR indicator for address */
	if (rop->address_is_ddr) {
		drdrenr |= drdrenr_address_is_ddr;
	}

	/* DDR indicator for data */
	if (rop->transfer_is_ddr) {
		drdrenr |= drdrenr_data_is_ddr;
	}

	/* DDR indicator for PHYOFFSET1 */
	if (rop->transfer_is_ddr) {
		phyoffset1_set = SPIM_PHYOFFSET1_DDR << SPIM_PHYOFFSET1_DDRTMG_POS;
	}
	else {
		phyoffset1_set = SPIM_PHYOFFSET1_SDR << SPIM_PHYOFFSET1_DDRTMG_POS;
	}

	/* DDR indicator for PHYCNT */
	if (rop->address_is_ddr || rop->transfer_is_ddr) {
		phycnt_set = SPIM_PHYCNT_DDR << SPIM_PHYCNT_PHYMEM_POS;
	}
	else {
		phycnt_set = SPIM_PHYCNT_SDR << SPIM_PHYCNT_PHYMEM_POS;
	}

	/* SLCH (SSL assert to CLK high) */
	if (rop->slch_value < 8) {
		ssldr_clear |= SPIM_SSLDR_SCKDL;
		ssldr_set |= rop->slch_value << SPIM_SSLDR_SCKDL_POS;
	}
	else {
		ERROR("Unsupported slch_value %d for rop", rop->slch_value);
		return -1;
	}

	/* CLSH (CLK low tp SSL negative) */
	if (rop->clsh_value < 8) {
		ssldr_clear |= SPIM_SSLDR_SLNDL;
		ssldr_set |= rop->clsh_value << SPIM_SSLDR_SLNDL_POS;
	}
	else {
		ERROR("Unsupported clsh_value %d for rop", rop->clsh_value);
		return -1;
	}

	/* SHSL (SSL negative to SSL assert) */
	if (rop->shsl_value < 8) {
		ssldr_clear |= SPIM_SSLDR_SPNDL;
		ssldr_set |= rop->shsl_value << SPIM_SSLDR_SPNDL_POS;
	}
	else {
		ERROR("Unsupported shsl_value %d for rop", rop->shsl_value);
		return -1;
	}

	/* Write the register */
	mmio_write_32(myctrl->reg_base + SPIM_DRCMR, drcmr);
	mmio_write_32(myctrl->reg_base + SPIM_DRENR, drenr);
	mmio_write_32(myctrl->reg_base + SPIM_DRDRENR, drdrenr);
	mmio_write_32(myctrl->reg_base + SPIM_DROPR, dropr);
	mmio_write_32(myctrl->reg_base + SPIM_DRDMCR, drdmcr);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_SSLDR, ssldr_clear, ssldr_set);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYOFFSET1, phyoffset1_msk, phyoffset1_set);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYOFFSET2, phyoffset2_msk, phyoffset2_set);
	mmio_clrsetbits_32(myctrl->reg_base + SPIM_PHYCNT, phycnt_msk, phycnt_set|SPIM_PHYCNT_CAL);

	/* Reduce the SPI freq. */
	spim_reduce_frequency(ctrl, rop->transfer_is_ddr);

	/* Do calibration if transfer is DTR */
	if (rop->transfer_is_ddr) {
		return calibration_check(myctrl);
	}

	return 0;
}

static int spim_start_xip(xspi_ctrl_t * const ctrl)
{
	assert(ctrl);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	mmio_setbits_32(myctrl->reg_base + SPIM_DRCR, SPIM_DRCR_RCF);
	spim_start_xip_internal(myctrl);
	spim_inv_mmap(ctrl);

	return 0;
}

static int spim_stop_xip(xspi_ctrl_t * const ctrl)
{
	assert(ctrl);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	return spim_stop_xip_internal(myctrl);
}

static int spim_run_manual_calibration(xspi_ctrl_t * const ctrl)
{
	assert(ctrl);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	return spim_internal_manual_calibration(myctrl);
}

static int spim_enable_auto_calibration(xspi_ctrl_t * const ctrl)
{
	return -1;

}

static int spim_disable_auto_calibration(xspi_ctrl_t * const ctrl)
{
	return -1;

}

static int spim_set_frequency(xspi_ctrl_t * const ctrl, int frequency_hz)
{
	assert(ctrl);

	int result = cpg_set_xspi_clock(XSPI_CLOCK_SPIM, frequency_hz * 4);
	if (result != 0) return result;
	int actual_freq = cpg_get_xspi_clock(XSPI_CLOCK_SPIM);
	if (actual_freq == -1) return -1;
	actual_freq = actual_freq / 4;
	if (spi_clock != actual_freq) {
		WARN("SPIM: Reduces the SPI frequency from %d to %d.\n", spi_clock, actual_freq);
	}
	spi_clock = actual_freq;

	return 0;
}

static int spim_clean_mmap(xspi_ctrl_t * const ctrl)
{
	return 0;
}

static int spim_inv_mmap(xspi_ctrl_t * const ctrl)
{
	assert(ctrl);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	inv_dcache_range(myctrl->mmap_base, myctrl->mmap_size);

	return 0;
}

static uintptr_t spim_get_mmap_base(xspi_ctrl_t * const ctrl)
{
	assert(ctrl);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	return myctrl->mmap_base;
}

static size_t spim_get_mmap_size(xspi_ctrl_t * const ctrl)
{
	assert(ctrl);
	spim_ctrl_t * myctrl = (spim_ctrl_t *)ctrl;
	return myctrl->mmap_size;
}
