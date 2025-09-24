/*
 * Copyright (c) 2024, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file  spi_nand_bb.h
 * @brief Bad Block Management for SPI NAND
 *
 */

#ifndef __QSPIFLASH_W25N_SPI_NAND_BB_H__
#define __QSPIFLASH_W25N_SPI_NAND_BB_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* ************************ HEADER (INCLUDE) SECTION *********************** */

/* ***************** MACROS, CONSTANTS, COMPILATION FLAGS ****************** */

/* ********************** STRUCTURES, TYPE DEFINITIONS ********************* */

/* ********************** DECLARATION OF EXTERNAL DATA ********************* */

/* ************************** FUNCTION PROTOTYPES ************************** */

int spi_nand_bb_Detect_Bad_Block(xspidevice_ctrl_t* ctrl, uint32_t * const numOfLogBlocks);
int spi_nand_bb_Get_L2P_Block_Table(uint32_t lBlock, uint32_t * const pBlock);
int spi_nand_bb_Get_P2L_Block_Table(uint32_t pBlock, uint32_t * const lBlock);

#ifdef __cplusplus
}
#endif

#endif	/* __QSPIFLASH_W25N_SPI_NAND_BB_H__ */
