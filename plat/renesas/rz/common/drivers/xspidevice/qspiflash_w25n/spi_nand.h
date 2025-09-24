/*
 * Copyright (c) 2024, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _QSPIFLASH_W25N_SPI_NAND_H_
#define _QSPIFLASH_W25N_SPI_NAND_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* ************************ HEADER (INCLUDE) SECTION *********************** */

/* ***************** MACROS, CONSTANTS, COMPILATION FLAGS ****************** */
#define SPI_NAND_QUAD_READ_USE
#define SPI_NAND_EXEC_OP_READ_REPEAT_USE
#if (SUPPORT_DMAC == 1)
#define CONTINUOUS_READ_USE
#endif

/** @brief Return value(Error code) for external functions
 */
#define SPI_NAND_ERROR_NONE		(0)
#define SPI_NAND_ERROR_PARAM	(-100)
#define SPI_NAND_ERROR_BUSY		(-101)
#define SPI_NAND_ERROR_FAIL		(-102)
#define SPI_NAND_ERROR_ECC		(-103)
#define SPI_NAND_ERROR_SYS		(-200)

/*
 * Default is disable, enable at special case.
 */
//#define SPI_NAND_REFERENCE_BOARD

/** @brief Ener/Exit Macro for function
 */
#define SPI_NAND_ENTERF(x)		do{}while(0)
#define SPI_NAND_EXITF(x,y)		do{}while(0)

/* ********************** STRUCTURES, TYPE DEFINITIONS ********************* */

/* ********************** DECLARATION OF EXTERNAL DATA ********************* */

/* ************************** FUNCTION PROTOTYPES ************************** */

/*
 * spi_nand I/F
 */
int spi_nand_Open(xspidevice_ctrl_t* ctrl);
int spi_nand_Close(xspidevice_ctrl_t* ctrl);
int spi_nand_Read_Page_Spare(xspidevice_ctrl_t* ctrl, uint8_t* buffer, uint32_t block, uint32_t page, uint32_t offset, size_t length, bool eccEnable);
int spi_nand_Read_Page_Main(xspidevice_ctrl_t* ctrl, uint8_t* buffer, uint32_t block, uint32_t page, uint32_t offset, size_t length);

/*
 * Read performance up
 */
int spi_nand_Set_Cont_Enable(xspidevice_ctrl_t* ctrl);
int spi_nand_Clear_Cont_Enable(xspidevice_ctrl_t* ctrl);
int spi_nand_Issue_Read_Page_Command(xspidevice_ctrl_t* ctrl, uint32_t block, uint32_t page);
int spi_nand_Is_Ecc_Uncorrectable(xspidevice_ctrl_t* ctrl, bool *result);

#ifdef __cplusplus
}
#endif

#endif /* _QSPIFLASH_W25N_SPI_NAND_H_ */
