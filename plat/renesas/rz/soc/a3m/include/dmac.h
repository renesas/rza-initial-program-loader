/*
 * Copyright (c) 2025, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __DMAC_H__
#define __DMAC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Macro definitions
 */

#define DMAC_UNIT_MAX						(0)
#define DMAC_CH_MAX							(15)
#define DMAC_TRANS_FLASH_UNIT				(0)
#define DMAC_TRANS_FLASH_CH					(0)
#define DMAC_DEFAULT_TRANSFER_WAIT_COUNT	(0x00FFFFFF)
#define DMAC_DEFAULT_TRANSFER_UNIT_SIZE		(8*32)	/* SPI multi burst length */

/* Error code */
#define DMAC_SUCCESS				(0)
#define DMAC_ERR_INVALID_PARAM		(-301)
#define DMAC_ERR_READY_TIMEOUT		(-302)
#define DMAC_ERR_ALEADY_OPEN		(-303)
#define DMAC_ERR_NOT_OPEN			(-304)
#define DMAC_ERR_NOT_READY			(-305)

/*
 * DMA transfer information
 */
 
enum dmac_addr_direction_t{
	DMAC_ADDR_COUNT_INCREMENT = 0x00,
	DMAC_ADDR_COUNT_FIX       = 0x01,
};

enum dmac_trans_size_t{
	DMAC_TRANS_SIZE_8    = 0x00,
	DMAC_TRANS_SIZE_16   = 0x01,
	DMAC_TRANS_SIZE_32   = 0x02,
	DMAC_TRANS_SIZE_64   = 0x03,
	DMAC_TRANS_SIZE_128  = 0x04,
	DMAC_TRANS_SIZE_256  = 0x05,
	DMAC_TRANS_SIZE_512  = 0x06,
	DMAC_TRANS_SIZE_1024 = 0x07,
};

struct dmac_cfg_t{
	enum dmac_addr_direction_t dest_addr_count;	///< Destination address count increment / fix
	enum dmac_addr_direction_t src_addr_count;	///< Source address count increment / fix
	enum dmac_trans_size_t dest_data_size;		///< DMA transfer size for destination
	enum dmac_trans_size_t src_data_size;		///< DMA transfer size for source
	void     *extend;						///< Set additional device-dependent settings
};

/* API functions */
int dmac_open(uint32_t unit);
int dmac_close(uint32_t unit);
int dmac_set_transfer_setting(uint32_t unit, uint32_t ch, struct dmac_cfg_t const * cfg);
int dmac_transfer_start(uint32_t unit, uint32_t ch, uint32_t src, uint32_t dest, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* __DMAC_H__ */
