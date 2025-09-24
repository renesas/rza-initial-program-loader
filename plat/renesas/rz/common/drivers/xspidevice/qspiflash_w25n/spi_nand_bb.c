/*
 * Copyright (c) 2024, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file  spi_nand_bb.c
 * @brief Bad Block Management for SPI NAND
 *
 */

/* ************************ HEADER (INCLUDE) SECTION *********************** */

#include "qspiflash_w25n_api.h"
#include "spi_nand.h"
#include "W25N01GVZEIG.h"

/* ***************** MACROS, CONSTANTS, COMPILATION FLAGS ****************** */

#define SPI_NAND_BB_INVALID_BLOCK_NO	(0xFFFFFFFF)

/* ********************** STRUCTURES, TYPE DEFINITIONS ********************* */

/* ********************** DECLARATION OF EXTERNAL DATA ********************* */

/* ********************** DECLARATION OF INTERNAL DATA ********************* */

static uint32_t spi_nand_bb_l2p_block_table[SPI_NAND_NUM_OF_BLOCKS] = { SPI_NAND_BB_INVALID_BLOCK_NO };

/* ************************** FUNCTION PROTOTYPES ************************** */

static int spi_nand_bb_init_L2P_Block_Table(void);
static int spi_nand_bb_set_L2P_Block_Table(uint32_t lBlock, uint32_t pBlock);
static int spi_nand_bb_is_Bad_Block(xspidevice_ctrl_t* ctrl, uint32_t phyBlock, bool *result);

/* ********************************* CODE ********************************** */

/** @brief Initialize L2P Block Table
 *
 * - Pre-conditions:<BR>
 *
 * - Post-conditions:<BR>
 *
 */
static int spi_nand_bb_init_L2P_Block_Table(void)
{
	int ret = SPI_NAND_ERROR_NONE;
	uint32_t block;

	SPI_NAND_ENTERF("spi_nand_bb_init_L2P_Block_Table");

	for ( block = 0; block < SPI_NAND_NUM_OF_BLOCKS; block++ ) {
		spi_nand_bb_l2p_block_table[block] = SPI_NAND_BB_INVALID_BLOCK_NO;
	}

	SPI_NAND_EXITF( "spi_nand_bb_init_L2P_Block_Table", ret);
	return ret;
}

/** @brief Set L2P Block Table
 *
 * - Pre-conditions:<BR>
 *
 * - Post-conditions:<BR>
 *
 * @param[in] lBlock
 * @param[in] pBlock
 */
static int spi_nand_bb_set_L2P_Block_Table(uint32_t lBlock, uint32_t pBlock)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_bb_Set_L2P_Block_Table");

	if (( SPI_NAND_NUM_OF_BLOCKS <= lBlock ) ||
		( SPI_NAND_NUM_OF_BLOCKS <= pBlock )){
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_bb_Set_L2P_Block_Table", ret);
		return ret;
	}

	spi_nand_bb_l2p_block_table[lBlock] = pBlock;

	SPI_NAND_EXITF( "spi_nand_bb_Set_L2P_Block_Table", ret);
	return ret;
}

/** @brief Check if a phyBlock is bad from bad block markers on spare area.
 *
 * - Pre-conditions:<BR>
 *
 * - Post-conditions:<BR>
 *
 */
static int spi_nand_bb_is_Bad_Block(xspidevice_ctrl_t* ctrl, uint32_t phyBlock, bool *result)
{
	int ret = SPI_NAND_ERROR_NONE;
	uint32_t page;
	uint8_t readData[4];
	uint8_t checkData[4];

	SPI_NAND_ENTERF("spi_nand_bb_is_Bad_Block");

	if (( NULL == ctrl )||(SPI_NAND_NUM_OF_BLOCKS <= phyBlock)||( NULL == result )) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_bb_is_Bad_Block", ret);
		return ret;
	}

	memset(&checkData[0], 0xFF, sizeof(checkData));
	page = 0;
	ret = spi_nand_Read_Page_Spare(ctrl, &readData[0], phyBlock, page, 0, sizeof(readData), true);
	if ( SPI_NAND_ERROR_NONE != ret ) {
		if ( SPI_NAND_ERROR_ECC == ret ) {
			ret = spi_nand_Read_Page_Spare(ctrl, &readData[0], phyBlock, page, 0, sizeof(readData), false);
			if ( SPI_NAND_ERROR_NONE != ret ) {
				goto _FUNC_END;
			}
		}
		else {
			goto _FUNC_END;
		}
	}
	if ( memcmp(&readData[0], &checkData[0], sizeof(readData)) != 0 ) {
		/*
		 * phyBlock is bad.
		 */
		*result = true;
	}
	else {
		*result = false;
	}

_FUNC_END:
	SPI_NAND_EXITF( "spi_nand_bb_is_Bad_Block", ret);
	return ret;
}


/** @brief function of detecting bad blocks
 *
 * - Pre-conditions:<BR>
 *
 * - Post-conditions:<BR>
 *
 * @param[in] ctrl
 * @param[out] numOfLogBlocks
 */
int spi_nand_bb_Detect_Bad_Block(xspidevice_ctrl_t* ctrl, uint32_t * const numOfLogBlocks)
{
	int ret = SPI_NAND_ERROR_NONE;
	uint32_t phyBlock = 0;
	uint32_t logBlock = 0;
	bool isBad;

	SPI_NAND_ENTERF("spi_nand_bb_Detect_Bad_Block");

	if (( NULL == ctrl ) || ( NULL == numOfLogBlocks )) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_bb_Detect_Bad_Block", ret);
		return ret;
	}

	ret = spi_nand_bb_init_L2P_Block_Table();
	if ( SPI_NAND_ERROR_NONE != ret ) {
		SPI_NAND_EXITF("spi_nand_bb_Detect_Bad_Block", ret);
		return ret;
	}

	logBlock = 0;

	for (phyBlock = 0; phyBlock < SPI_NAND_NUM_OF_BLOCKS; phyBlock++) {
		ret = spi_nand_bb_is_Bad_Block( ctrl, phyBlock, &isBad );
		if ( SPI_NAND_ERROR_NONE != ret ) {
			goto _FUNC_END;
		}
		if ( false == isBad ) {
			ret = spi_nand_bb_set_L2P_Block_Table( logBlock, phyBlock );
			if ( SPI_NAND_ERROR_NONE != ret ) {
				goto _FUNC_END;
			}
			logBlock++;
		}
	}

	if ( SPI_NAND_ERROR_NONE == ret ) {
		*numOfLogBlocks = logBlock;
	}

_FUNC_END:
	SPI_NAND_EXITF( "spi_nand_bb_Detect_Bad_Block", ret);
	return ret;
}

/** @brief Get L2P Block Table
 *
 * - Pre-conditions:<BR>
 *
 * - Post-conditions:<BR>
 *
 * @param[in] lBlock
 * @param[in] pBlock
 */
int spi_nand_bb_Get_L2P_Block_Table(uint32_t lBlock, uint32_t * const pBlock)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_bb_Get_L2P_Block_Table");

	if (( SPI_NAND_NUM_OF_BLOCKS <= lBlock ) ||
		( NULL == pBlock )){
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_bb_Get_L2P_Block_Table", ret);
		return ret;
	}

	*pBlock = spi_nand_bb_l2p_block_table[lBlock];

	SPI_NAND_EXITF( "spi_nand_bb_Get_L2P_Block_Table", ret);
	return ret;
}

/** @brief Get P2L Block Table
 *
 * - Pre-conditions:<BR>
 *
 * - Post-conditions:<BR>
 *
 * @param[in] pBlock
 * @param[in] lBlock
 */
int spi_nand_bb_Get_P2L_Block_Table(uint32_t pBlock, uint32_t * const lBlock)
{
	int ret = SPI_NAND_ERROR_NONE;
	uint32_t target_block;

	SPI_NAND_ENTERF("spi_nand_bb_Get_P2L_Block_Table");

	if (( SPI_NAND_NUM_OF_BLOCKS <= pBlock ) ||
		( NULL == lBlock )){
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_bb_Get_P2L_Block_Table", ret);
		return ret;
	}

	for (target_block = 0; SPI_NAND_NUM_OF_BLOCKS > target_block; target_block++) {
		if (spi_nand_bb_l2p_block_table[target_block] == pBlock) {
			*lBlock = target_block;
			break;
		}
	}
	
	if (SPI_NAND_NUM_OF_BLOCKS == target_block) {
		ret = SPI_NAND_ERROR_FAIL;
	}

	SPI_NAND_EXITF( "spi_nand_bb_Get_L2P_Block_Table", ret);
	return ret;
}

