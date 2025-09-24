/*
 * Copyright (c) 2025, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <endian.h>
#include <common/debug.h>
#include <arch_helpers.h>
#include "cpg.h"
#include "dmac.h"
#include "dmac_reg.h"

/*
 * Macro definitions
 */

static bool dmac_opend = false;
static bool dmac_ready = false;

/*
 * Internal variables
 */

/* Initial value of Channel Configuration Register for SW trigger */
static const uint32_t chcfg_value_sw_trigger = 
	((0 << DMAC_CHCFG_DMS_POS)  | /* Register mode */
	(0 << DMAC_CHCFG_REN_POS)   | /* Does not continue DMA transfers. */
	(0 << DMAC_CHCFG_RSW_POS)   | /* Does not invert RSEL. */
	(0 << DMAC_CHCFG_RSEL_POS)  | /* Next0 Register Set. */
	(0 << DMAC_CHCFG_SBE_POS)   | /* Stops the DMA transfer without sweeping the buffer. */
	(1 << DMAC_CHCFG_DEM_POS)   | /* Interrupt mask */
	(1 << DMAC_CHCFG_TM_POS)    | /* Block transfer */
	(4 << DMAC_CHCFG_AM_POS)    | /* DMA ack not to be output */
	(0 << DMAC_CHCFG_LVL_POS)   | /* This bit's initial value. It is not affected by SW trigger. */
	(0 << DMAC_CHCFG_HIEN_POS)  | /* This bit's initial value. It is not affected by SW trigger. */
	(0 << DMAC_CHCFG_LOEN_POS)  | /* This bit's initial value. It is not affected by SW trigger. */
	(0 << DMAC_CHCFG_REQD_POS));  /* This bit's initial value. It is not affected by SW trigger. */

/*
 * DMAC api functions
 */
int dmac_open(uint32_t unit)
{
	if (dmac_opend != false) return DMAC_ERR_ALEADY_OPEN;
	
	cpg_start_dmac(unit);
	
	dmac_opend = true;
	dmac_ready = false;
	
	return DMAC_SUCCESS;
}

int dmac_close(uint32_t unit)
{
	if (dmac_opend != false) return DMAC_ERR_NOT_OPEN;
	
	cpg_stop_dmac(unit);
	
	dmac_opend = false;
	dmac_ready = false;
	
	return DMAC_SUCCESS;
}

int dmac_set_transfer_setting(uint32_t unit, uint32_t ch, struct dmac_cfg_t const * cfg)
{
	uint32_t chcfg_value = chcfg_value_sw_trigger;
	
	if (dmac_opend == false) return DMAC_ERR_NOT_OPEN;
	
	if ((unit > DMAC_UNIT_MAX) || 
		(ch > DMAC_CH_MAX)     || 
		(cfg == NULL)) {
		return DMAC_ERR_INVALID_PARAM;
	}
	
	chcfg_value |= (chcfg_value_sw_trigger                                           |
					((cfg->dest_addr_count << DMAC_CHCFG_DAD_POS) & DMAC_CHCFG_DAD) |
					((cfg->src_addr_count << DMAC_CHCFG_SAD_POS) & DMAC_CHCFG_SAD)  |
					((cfg->dest_data_size << DMAC_CHCFG_DDS_POS) & DMAC_CHCFG_DDS)  |
					((cfg->src_data_size << DMAC_CHCFG_SDS_POS) & DMAC_CHCFG_SDS)   |
					(((ch % 8) << DMAC_CHCFG_SEL_POS) & DMAC_CHCFG_SEL));
	
	mmio_write_32(DMAC_CHCFG(ch), chcfg_value); 
	
	dmac_ready = true;
	
	return DMAC_SUCCESS;
	
}

int dmac_transfer_start(uint32_t unit, uint32_t ch, uint32_t src, uint32_t dest, uint32_t size)
{
	int result = DMAC_SUCCESS;
	uint32_t regVal;
	uint32_t i;

	if (dmac_opend == false) return DMAC_ERR_NOT_OPEN;
	
	if (dmac_ready == false) return DMAC_ERR_NOT_READY;

	if (( 0 == dest ) ||
		( 0 == src )  ||
		( 0 == size )) {
		return DMAC_ERR_INVALID_PARAM;
	}

	/* DMA transfer address settings */
	mmio_write_32(DMAC_N0SA(ch), src);		/* source address */
	mmio_write_32(DMAC_N0DA(ch), dest);		/* destination address */
	mmio_write_32(DMAC_N0TB(ch), size);		/* transfer bytes */ 

	/* DMAC software reset */
	regVal = mmio_read_32(DMAC_CHSTAT(ch));
	if (( regVal & ( DMAC_CHSTAT_EN | DMAC_CHSTAT_TACT ) ) == 0) {
		mmio_write_32(DMAC_CHCTRL(ch), DMAC_CHCTRL_SWRST);	/* SWRST=1 */
		mmio_write_32(DMAC_CHCTRL(ch), 0x00000000);	/* SWRST=0 */
	}
	else {
		return DMAC_ERR_READY_TIMEOUT;
	}

	/* DMAC enable */
	/* Only software trigger is supported */
	mmio_write_32(DMAC_CHCTRL(ch), (DMAC_CHCTRL_STG | DMAC_CHCTRL_SETEN)); /* STG=1(soft trigger)SETEN=1 */

	/* Wait for DMAC transfer */
	for ( i = 0 ; i < DMAC_DEFAULT_TRANSFER_WAIT_COUNT ; i++) {
		regVal = mmio_read_32(DMAC_CHSTAT(ch));
		/* Wait until bit[6]=1 (set TC bit) */
		if (( regVal & DMAC_CHSTAT_TC ) != 0 ) {
			break;
		}
	}

	if ( i >= DMAC_DEFAULT_TRANSFER_WAIT_COUNT ) {
		result = DMAC_ERR_READY_TIMEOUT;
	}

	flush_dcache_range((uintptr_t)dest, (uint64_t)size);

	return result;
}


