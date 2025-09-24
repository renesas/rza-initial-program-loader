/*
 * Copyright (c) 2024, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/* ************************ HEADER (INCLUDE) SECTION *********************** */

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <drivers/delay_timer.h>
#include <rza_printf.h>
#include <spim.h>

#include "qspiflash_w25n_api.h"
#include "spi_nand.h"
#include "spi_nand_bb.h"
#include "W25N01GVZEIG.h"
#if (SUPPORT_DMAC == 1)
#include "dmac.h"
#endif

/* ***************** MACROS, CONSTANTS, COMPILATION FLAGS ****************** */

/* Defaults */
#define DEFAULT_SPI_FREQUENCY 					(66666667)
#define SPI_POST_RESET_WAIT 					(50)
#define NUM_OF_PBLOCK_AREAS_BY_CONTINUOUS_READ	(SPI_NAND_NUM_OF_BAD_BLOCKS + 1)

/* ********************** STRUCTURES, TYPE DEFINITIONS ********************* */

typedef struct st_flash_read_area
{
	uint8_t * buffer;
	size_t address;
	size_t length;
} flash_read_area_t;

typedef struct st_pblock_area
{
	bool valid;
	uint32_t startBlock;
	uint32_t numOfBlocks;
} pblock_area_t;

/* ************************** FUNCTION PROTOTYPES ************************** */

/* external */
static int flash_open(xspidevice_ctrl_t * ctrl, xspidevice_cfg_t const * cfg);
static int flash_close(xspidevice_ctrl_t * ctrl);
static int flash_exec_op(xspidevice_ctrl_t * ctrl, xspi_op_t const * op, bool is_write);
static int flash_get_info(xspidevice_ctrl_t * ctrl, xspidevice_info_t * info);
static int flash_read(xspidevice_ctrl_t * ctrl, void * buffer, size_t address, size_t length);
static int flash_enter_xip(xspidevice_ctrl_t * ctrl);
static int flash_exit_xip(xspidevice_ctrl_t * ctrl);
static int flash_get_logical_address(xspidevice_ctrl_t * ctrl, size_t paddress, size_t * const laddress);

/* internal for Read performance up */
#ifdef CONTINUOUS_READ_USE
static int flash_internal_read_continuous_set_pblock_area_info( uint32_t lblock, uint32_t numOfBlocks, pblock_area_t *info);
static int flash_internal_read_middle_area(xspidevice_ctrl_t * ctrl, uint8_t * buffer, size_t address, size_t length);
#endif
static int flash_internal_get_physical_block_no( size_t address,  uint32_t *pblock);
static int flash_internal_read_manual_mode(xspidevice_ctrl_t * ctrl, uint8_t * buffer, size_t address, size_t length);

/* ********************** DECLARATION OF EXTERNAL DATA ********************* */

/* API function table definition */
const xspidevice_api_t qspiflash_w25n_api = {
	.open = flash_open,
	.close = flash_close,
	.exec_op = flash_exec_op,
	.enter_xip = flash_enter_xip,
	.exit_xip = flash_exit_xip,
	.get_info = flash_get_info,
	.read = flash_read,
	.write = NULL,
	.erase = NULL,
	.get_write_status = NULL,
	.get_logical_address = flash_get_logical_address,
};

/* ********************** DECLARATION OF INTERNAL DATA ********************* */

#ifdef CONTINUOUS_READ_USE
static const xspi_op_t op_fast_read_quad_io_continuous = {
#ifdef SPI_NAND_QUAD_READ_USE
	.form = SPI_FORM_1_4_4,
	.op = SPI_NAND_CMD_FAST_READ_QUAD_IO,
	.op_size = 1,
	.op_is_ddr = false,
	.address = 0,
	.address_is_ddr = false,
	.address_size = 0,
	.additional_value = 0,
	.additional_size = 0, /* Do not set the Column Address because The Fast Read Quad I/O command */
						  /* in Continuous Read Mode does not require a column address. */
						  /* Data reception will start from the first address of the page. */
	.dummy_cycles = 12,
	.transfer_buffer = NULL,
	.transfer_is_ddr = false,
	.transfer_size = 0,
	.force_idle_level_mask = 0x08,	// Keep IO3/HOLD pin to High
	.force_idle_level_value = 0x08,	// Keep IO3/HOLD pin to High
	.slch_value = 0,
	.clsh_value = 0,
	.shsl_value = 3
#else
	.form = SPI_FORM_1_1_1,
	.op = SPI_NAND_CMD_READ,
	.op_size = 1,
	.op_is_ddr = false,
	.address = 0,
	.address_is_ddr = false,
	.address_size = 0,
	.additional_value = 0,
	.additional_size = 2, /* The Read Data command in Continuous Read Mode does not require a Column Address. */
						  /* In this case, this command requires 24 dummy cycles, but SPIBSC only allows up to 20 cycles to be set. */
						  /* Therefore, 16 bits of additional data containing 0 are sent as dummy cycles, */
						  /* and then 8 bits of normal dummy cycles are sent. */
	.dummy_cycles = 8,
	.transfer_buffer = NULL,
	.transfer_is_ddr = false,
	.transfer_size = 0,
	.force_idle_level_mask = 0x08,	// Keep IO3/HOLD pin to High
	.force_idle_level_value = 0x08,	// Keep IO3/HOLD pin to High
	.slch_value = 0,
	.clsh_value = 0,
	.shsl_value = 3
#endif
};

static const struct dmac_cfg_t dmac_cfg = {
	.dest_addr_count = DMAC_ADDR_COUNT_INCREMENT,
	.src_addr_count = DMAC_ADDR_COUNT_INCREMENT,
	.dest_data_size = DMAC_TRANS_SIZE_1024,
	.src_data_size = DMAC_TRANS_SIZE_1024,
	.extend = NULL
};
#endif

static int spi_frequency = DEFAULT_SPI_FREQUENCY;
static uint8_t  pageMainBuffer[SPI_NAND_PAGE_MAIN_BYTES] = { 0 };
static uint32_t numOfLogBlocks = 0;
#ifdef CONTINUOUS_READ_USE
static pblock_area_t pblock_area_info[NUM_OF_PBLOCK_AREAS_BY_CONTINUOUS_READ];
#endif
static bool done_flash_open_close = false;

/* ********************************* CODE ********************************** */

#if 0
static void testUtile_PrintBuff(uint8_t* buff, uint32_t size)
{
	int32_t byte;

	if((buff == NULL) || (size == 0)){
		return;
	}

	for(byte=0; byte<size; byte++){
		if((byte % 16) == 0 ) {
			RZA_PRINTF_VERBOSE("0x%08x :", byte );
		}
		RZA_PRINTF_VERBOSE(" %02x", buff[byte]);
		if((byte % 16) == 15){
			RZA_PRINTF_VERBOSE("\n");
		}
	}
	RZA_PRINTF_VERBOSE("\n");
}
#endif

/*
 * INTERNAL FUNCTIONS
 */
#ifdef CONTINUOUS_READ_USE
static int flash_internal_read_continuous_set_pblock_area_info( uint32_t lblock, uint32_t numOfBlocks, pblock_area_t *info)
{
	int result = 0;
	uint32_t i;
	uint32_t pblock;
	uint32_t j;
	uint32_t ppblock;

	if ( NULL == info ) {
		result = SPI_NAND_ERROR_PARAM;
		return result;
	}

	/*
	 * Initialize pblock_area_info
	 */
	for ( i = 0; i < NUM_OF_PBLOCK_AREAS_BY_CONTINUOUS_READ; i++) {
		info[i].valid = false;
		info[i].startBlock = 0;
		info[i].numOfBlocks = 0;
	}

	/*
	 * Set pblock_area_info
	 */
	ppblock = 0x0000FFFF;
	j = -1;
	for ( i = 0; i < numOfBlocks; i++ ) {
		result = spi_nand_bb_Get_L2P_Block_Table(lblock + i, &pblock);
		if (SPI_NAND_ERROR_NONE != result) {
			goto _FUNC_END;
		}
		if ( (ppblock + 1) != pblock ) {
			j++;
			info[j].valid = true;
			info[j].startBlock = pblock;
			info[j].numOfBlocks = 1;
			if (j >= NUM_OF_PBLOCK_AREAS_BY_CONTINUOUS_READ) {
				result = SPI_NAND_ERROR_SYS;
				goto _FUNC_END;
			}
		}
		else {
			info[j].numOfBlocks++;
		}
		ppblock = pblock;
	}

	for ( i = 0; i < NUM_OF_PBLOCK_AREAS_BY_CONTINUOUS_READ; i++) {
		if ( true == info[i].valid ) {
			RZA_PRINTF_VERBOSE("%d: start %d blocks %d\n", i,
						info[i].startBlock,
						info[i].numOfBlocks);
		}
	}

_FUNC_END:
	return result;
}

static int flash_internal_read_middle_area(xspidevice_ctrl_t * ctrl, uint8_t * buffer, size_t address, size_t length)
{
	assert(ctrl);
	assert(buffer);
	int result = 0;
	uint32_t numOfBlocks = 0;
	uint8_t* copiedBuffer;
	uint32_t lblock;
	uint32_t pblock;
	int page = 0;
	size_t start;
	xspi_op_t op;
	bool isEccError;

	RZA_PRINTF_VERBOSE("-> flash_internal_read_middle_area() address=0x%lx length=%ld\n", address, length);

	if (( NULL == ctrl ) || ( NULL == buffer )) {
		return -1;
	}

	if (( 0 != (address % SPI_NAND_BLOCK_BYTES) ) || ( 0 != (length % SPI_NAND_BLOCK_BYTES) )) {
		return -1;
	}

	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = myctrl->xspi;
	uintptr_t mmap_base = xspi->api->get_mmap_base(xspi->ctrl);

	op = op_fast_read_quad_io_continuous;
	copiedBuffer = buffer;
	start = address;
	numOfBlocks = length / SPI_NAND_BLOCK_BYTES;
	lblock = ( start / SPI_NAND_PAGE_MAIN_BYTES) / SPI_NAND_NUM_OF_PAGES_IN_BLOCK;

#if (RZA_NAND_READ_UNIT_128BLOCK == 1)
	uint32_t numOfUnitBlocks;
	uint32_t numOfTransBlocks;

	result = dmac_set_transfer_setting(DMAC_TRANS_FLASH_UNIT, DMAC_TRANS_FLASH_CH, &dmac_cfg);
	if (result !=  DMAC_SUCCESS) {
		goto _FUNC_END;
	}

	while (0 < numOfBlocks)
	{
		if (SPI_NAND_NUM_OF_ONCE_TRANS_BLOCKS < numOfBlocks)
		{
			numOfUnitBlocks = SPI_NAND_NUM_OF_ONCE_TRANS_BLOCKS;
		} else {
			numOfUnitBlocks = numOfBlocks;
		}
		
		result = flash_internal_read_continuous_set_pblock_area_info( lblock, numOfUnitBlocks, &pblock_area_info[0] );
		if ( 0 != result ) {
			goto _FUNC_END;
		}

		for (int i = 0; i < NUM_OF_PBLOCK_AREAS_BY_CONTINUOUS_READ; i++) {

			if ( true == pblock_area_info[i].valid ) {
				page = 0;
				pblock = pblock_area_info[i].startBlock;
				numOfTransBlocks = pblock_area_info[i].numOfBlocks;
			}
			else {
				/* finish */
				break;
			}

			/*
			 * Set CONT bit
			 */
			result = spi_nand_Set_Cont_Enable( ctrl );
			if (SPI_NAND_ERROR_NONE != result) {
				goto _FUNC_END;
			}

			result = spi_nand_Issue_Read_Page_Command( ctrl, pblock, page );
			if (SPI_NAND_ERROR_NONE != result) {
				goto _FUNC_END;
			}

			result = xspi->api->configure_xip(xspi->ctrl, &op, NULL);
			if ( 0 != result ) {
				goto _FUNC_END;
			}

			result = xspi->api->set_frequency(xspi->ctrl, spi_frequency);
			if ( 0 != result ) {
				goto _FUNC_END;
			}

			result = xspi->api->start_xip(xspi->ctrl);
			if ( 0 != result ) {
				goto _FUNC_END;
			}

			/*
			 * Except the last page for ECC uncorrectable error.
			 */
			result = dmac_transfer_start(DMAC_TRANS_FLASH_UNIT,
										 DMAC_TRANS_FLASH_CH,
										 (uintptr_t)mmap_base,
										 (uintptr_t)copiedBuffer,
										 SPI_NAND_BLOCK_BYTES*numOfTransBlocks - SPI_NAND_PAGE_MAIN_BYTES);
			if ( 0 != result ) {
				/*
				 * from xip to manual.
				 */
				xspi->api->stop_xip(xspi->ctrl);
				goto _FUNC_END;
			}
			
			result =  xspi->api->stop_xip(xspi->ctrl);
			if ( 0 != result ) {
				goto _FUNC_END;
			}

			/*
			 * Check ecc uncorrectable error.
			 */
			result = spi_nand_Is_Ecc_Uncorrectable(ctrl, &isEccError);
			if (SPI_NAND_ERROR_NONE != result) {
				goto _FUNC_END;
			}
			if ( true == isEccError ) {
				RZA_PRINTF_VERBOSE("FAIL : Ecc uncorrectable error pblock=%d numOfTransBlocks=%d\n", pblock, numOfTransBlocks);
				result = -1;
				goto _FUNC_END;
			}

			/*
			 * Clear CONT bit for Buffer Read.
			 */
			result = spi_nand_Clear_Cont_Enable( ctrl );
			if (SPI_NAND_ERROR_NONE != result) {
				goto _FUNC_END;
			}

			/*
			 * for next...
			 */
			copiedBuffer += SPI_NAND_BLOCK_BYTES*numOfTransBlocks - SPI_NAND_PAGE_MAIN_BYTES;
			start += SPI_NAND_BLOCK_BYTES*numOfTransBlocks - SPI_NAND_PAGE_MAIN_BYTES;
			result = flash_internal_read_manual_mode( ctrl, copiedBuffer, start, SPI_NAND_PAGE_MAIN_BYTES );
			if ( 0 != result ) {
				goto _FUNC_END;
			}
			copiedBuffer += SPI_NAND_PAGE_MAIN_BYTES;
			start += SPI_NAND_PAGE_MAIN_BYTES;
		}
		lblock += numOfUnitBlocks;
		if (numOfBlocks > numOfUnitBlocks) {
			numOfBlocks -= numOfUnitBlocks;
		}
		else {
			/* fail-safe process */
			numOfBlocks = 0;
		}
	}
#else /* (RZA_NAND_READ_UNIT_128BLOCK == 1) */
	result = flash_internal_read_continuous_set_pblock_area_info( lblock, numOfBlocks, &pblock_area_info[0] );
	if ( 0 != result ) {
		goto _FUNC_END;
	}

    result = dmac_set_transfer_setting(DMAC_TRANS_FLASH_UNIT, DMAC_TRANS_FLASH_CH, &dmac_cfg);
    if (result !=  DMAC_SUCCESS) {
		goto _FUNC_END;
	}
	for (int i = 0; i < NUM_OF_PBLOCK_AREAS_BY_CONTINUOUS_READ; i++) {

		if ( true == pblock_area_info[i].valid ) {
			page = 0;
			pblock = pblock_area_info[i].startBlock;
			numOfBlocks = pblock_area_info[i].numOfBlocks;
		}
		else {
			/* finish */
			break;
		}

		/*
		 * Set CONT bit
		 */
		result = spi_nand_Set_Cont_Enable( ctrl );
		if (SPI_NAND_ERROR_NONE != result) {
			goto _FUNC_END;
		}

		result = spi_nand_Issue_Read_Page_Command( ctrl, pblock, page );
		if (SPI_NAND_ERROR_NONE != result) {
			goto _FUNC_END;
		}

		result = xspi->api->configure_xip(xspi->ctrl, &op, NULL);
		if ( 0 != result ) {
			goto _FUNC_END;
		}

		result = xspi->api->set_frequency(xspi->ctrl, spi_frequency);
		if ( 0 != result ) {
			goto _FUNC_END;
		}

		result = xspi->api->start_xip(xspi->ctrl);
		if ( 0 != result ) {
			goto _FUNC_END;
		}

		/*
		 * Except the last page for ECC uncorrectable error.
		 */
		result = dmac_transfer_start(DMAC_TRANS_FLASH_UNIT,
									 DMAC_TRANS_FLASH_CH,
									 (uintptr_t)mmap_base,
									 (uintptr_t)copiedBuffer,
									 SPI_NAND_BLOCK_BYTES*numOfBlocks - SPI_NAND_PAGE_MAIN_BYTES);
		if ( 0 != result ) {
			/*
			 * from xip to manual.
			 */
			xspi->api->stop_xip(xspi->ctrl);
			goto _FUNC_END;
		}
		
		result =  xspi->api->stop_xip(xspi->ctrl);
		if ( 0 != result ) {
			goto _FUNC_END;
		}

		/*
		 * Check ecc uncorrectable error.
		 */
		result = spi_nand_Is_Ecc_Uncorrectable(ctrl, &isEccError);
		if (SPI_NAND_ERROR_NONE != result) {
			goto _FUNC_END;
		}
		if ( true == isEccError ) {
			RZA_PRINTF_VERBOSE("FAIL : Ecc uncorrectable error pblock=%d numOfBlocks=%d\n", pblock, numOfBlocks);
			result = -1;
			goto _FUNC_END;
		}

		/*
		 * Clear CONT bit for Buffer Read.
		 */
		result = spi_nand_Clear_Cont_Enable( ctrl );
		if (SPI_NAND_ERROR_NONE != result) {
			goto _FUNC_END;
		}

		/*
		 * for next...
		 */
		copiedBuffer += SPI_NAND_BLOCK_BYTES*numOfBlocks - SPI_NAND_PAGE_MAIN_BYTES;
		start += SPI_NAND_BLOCK_BYTES*numOfBlocks - SPI_NAND_PAGE_MAIN_BYTES;
		result = flash_internal_read_manual_mode( ctrl, copiedBuffer, start, SPI_NAND_PAGE_MAIN_BYTES );
		if ( 0 != result ) {
			goto _FUNC_END;
		}
		copiedBuffer += SPI_NAND_PAGE_MAIN_BYTES;
		start += SPI_NAND_PAGE_MAIN_BYTES;
	}
#endif /* (RZA_NAND_READ_UNIT_128BLOCK == 1) */

_FUNC_END:
	/*
	 * Clear CONT bit
	 */
	spi_nand_Clear_Cont_Enable( ctrl );

	RZA_PRINTF_VERBOSE("<- flash_internal_read_middle_area() result=%d\n", result);
	return result;
}
#endif

static int flash_internal_get_physical_block_no( size_t address,  uint32_t *pblock)
{
	int result = 0;
	uint32_t lblock;

	if ( NULL == pblock ) {
		result = -1;
		return result;
	}

	lblock = ( address / SPI_NAND_PAGE_MAIN_BYTES) / SPI_NAND_NUM_OF_PAGES_IN_BLOCK;
	result = spi_nand_bb_Get_L2P_Block_Table(lblock, pblock);

	return result;
}

static int flash_internal_read_manual_mode(xspidevice_ctrl_t * ctrl, uint8_t * buffer, size_t address, size_t length)
{
	int result = 0;
	uint32_t offset = 0;
	uint8_t* copiedBuffer;
	size_t start;
	size_t remain;
	size_t copied = 0;
	uint32_t pblock;
	uint32_t page;

	RZA_PRINTF_VERBOSE("-> flash_internal_read_manual_mode() address=0x%lx length=%ld\n", address, length);

	if ((NULL == ctrl) || (NULL == buffer)) {
		result = -1;
		RZA_PRINTF_VERBOSE("<- flash_internal_read_manual_mode() result=%d\n", result);
		return result;
	}

	copiedBuffer = (uint8_t*)buffer;
	start = address;
	remain = length;
	copied = 0;

	while ( remain > 0 ) {
		/*
		 * Calculate physical block and page.
		 */
		page = ( start / SPI_NAND_PAGE_MAIN_BYTES) % SPI_NAND_NUM_OF_PAGES_IN_BLOCK;
		result = flash_internal_get_physical_block_no(start, &pblock);
		if (SPI_NAND_ERROR_NONE != result) {
			break;
		}

		/*
		 * Read page data from flash memory
		 */
		result = spi_nand_Read_Page_Main(ctrl, &pageMainBuffer[0], pblock, page, 0, SPI_NAND_PAGE_MAIN_BYTES);
		if (SPI_NAND_ERROR_NONE != result) {
			break;
		}

		/*
		 * Copy from read page main buffer to parameter buffer
		 */
		offset = start % SPI_NAND_PAGE_MAIN_BYTES;
		if ( (offset + remain) <= SPI_NAND_PAGE_MAIN_BYTES ) {
			copied = remain;
		}
		else {
			copied = SPI_NAND_PAGE_MAIN_BYTES - offset;
		}
		memcpy(copiedBuffer, &pageMainBuffer[offset], copied);

		/*
		 * Prepare next read and copy
		 */
		start += copied;
		copiedBuffer += copied;
		remain -= copied;
	}

	RZA_PRINTF_VERBOSE("<- flash_internal_read_manual_mode() result=%d\n", result);
	return result;
}

/*
 * EXTERNAL FUNCTIONS
 */
static int flash_open(xspidevice_ctrl_t * ctrl, xspidevice_cfg_t const * cfg)
{
	assert(ctrl);
	assert(cfg);
	int result = -1;

	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = cfg->xspi;

	RZA_PRINTF_VERBOSE("-> flash_open() \n");

	if (myctrl->opened) {
		result = -1;
		return result;
	}

	if ( false == done_flash_open_close ) {

		numOfLogBlocks = 0;
		myctrl->xspi = xspi;
		result = xspi->api->open(xspi->ctrl, xspi->cfg);
		if (result == 0) {
			result = xspi->api->set_frequency(xspi->ctrl, spi_frequency);
			udelay(SPI_POST_RESET_WAIT);
		}

		if (result == 0) {
			result = spi_nand_Open(ctrl);
		}

		/*
		 * Detect bad block in flash memory
		 */
		if (result == 0) {
			result = spi_nand_bb_Detect_Bad_Block( ctrl, &numOfLogBlocks );
		}

		/*
		 * Setup DMAC channel
		 */
		dmac_open(DMAC_TRANS_FLASH_UNIT);
	}
	else {
		result = 0;
	}

	if (result == 0) {
		myctrl->opened = true;
	}
	else {
		xspi->api->close(xspi->ctrl);
	}

	if (result != 0) {
		result = -1;
	}

	RZA_PRINTF_VERBOSE("<- flash_open() result=%d numOfLogBlocks=%d\n", result, numOfLogBlocks);
	return result;
}

static int flash_close(xspidevice_ctrl_t * ctrl)
{
	assert(ctrl);
	int result = -1;
	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = myctrl->xspi;

	RZA_PRINTF_VERBOSE("-> flash_close()\n");

	if (myctrl->opened) {
		dmac_close(DMAC_TRANS_FLASH_UNIT);
		result = spi_nand_Close(ctrl);
		if ( result == 0 ) {
			if ( false == done_flash_open_close ) {
				result = xspi->api->close(xspi->ctrl);
				done_flash_open_close = true;
			}
			else {
				result = 0;
			}
			myctrl->opened = false;
		}
	}

	RZA_PRINTF_VERBOSE("<- flash_close() result=%d\n", result);
	return result;


}
static int flash_exec_op(xspidevice_ctrl_t * ctrl, xspi_op_t const * op, bool is_write)
{
	assert(ctrl);
	assert(op);
	int result = -1;

	RZA_PRINTF_VERBOSE("-> flash_exec_op()\n");

	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = myctrl->xspi;

	result = xspi->api->exec_op(xspi->ctrl, op, is_write);

	RZA_PRINTF_VERBOSE("<- flash_exec_op() result=%d\n", result);

	return result;
}

static int flash_get_info(xspidevice_ctrl_t * ctrl, xspidevice_info_t * info)
{
	assert(ctrl);
	assert(info);

	info->capacity = 0;
	info->minimum_erase_size = SPI_NAND_BLOCK_BYTES;

	strlcpy(info->device_vendor, "Winbond", sizeof(info->device_vendor));
	strlcpy(info->device_product, "W25N01GVZEIG", sizeof(info->device_product));

	return 0;
}

static int flash_read(xspidevice_ctrl_t * ctrl, void * buffer, size_t address, size_t length)
{
	assert(ctrl);
	assert(buffer);
	int result = 0;

	uint32_t lblock;
	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	flash_read_area_t startBlock = { 0 };
	flash_read_area_t middleBlocks = { 0 };
	flash_read_area_t endBlock = { 0 };
	uint32_t blockRemainBytes;
	uint8_t * startBuffer = (uint8_t *)buffer;
	uint32_t middleNumOfBlocks;

	RZA_PRINTF_VERBOSE("-> flash_read() buffer=%p address=0x%lx length=%ld\n", buffer, address, length);

	if ( (myctrl->opened) == false) {
		result = -1;
		RZA_PRINTF_VERBOSE("<- flash_read() result=%d\n", result);
		return result;
	}

	/* Has the specified address exceeded the NAND area? */
	lblock = ( (address+length) / SPI_NAND_PAGE_MAIN_BYTES) / SPI_NAND_NUM_OF_PAGES_IN_BLOCK;
	if( lblock >= numOfLogBlocks ){
		result = -1;
		RZA_PRINTF_VERBOSE("<- flash_read() result=%d\n", result);
		return result;
	}

	startBlock.buffer = startBuffer;
	startBlock.address = address;
	blockRemainBytes = SPI_NAND_BLOCK_BYTES - (address%SPI_NAND_BLOCK_BYTES);
	if ( length <= blockRemainBytes ) {
		startBlock.length = length;

		middleBlocks.buffer = NULL;
		middleBlocks.address = 0;
		middleBlocks.length = 0;

		endBlock.buffer = NULL;
		endBlock.address = 0;
		endBlock.length = 0;
		
	}
	else {
		startBlock.length = blockRemainBytes;
		startBuffer += blockRemainBytes;

		middleNumOfBlocks = ( length - blockRemainBytes ) / SPI_NAND_BLOCK_BYTES;
		if ( middleNumOfBlocks > 0 ) {
			middleBlocks.buffer = startBuffer;
			middleBlocks.address = startBlock.address + blockRemainBytes;
			middleBlocks.length = middleNumOfBlocks*SPI_NAND_BLOCK_BYTES;

			startBuffer += middleNumOfBlocks*SPI_NAND_BLOCK_BYTES;
			if ( length > ( startBlock.length + middleBlocks.length ) ) {
				endBlock.buffer = startBuffer;
				endBlock.address = middleBlocks.address + middleBlocks.length;
				endBlock.length = length - ( startBlock.length + middleBlocks.length );
			}
			else {
				endBlock.buffer = NULL;
				endBlock.address = 0;
				endBlock.length = 0;
			}
		}
		else {
			middleBlocks.buffer = NULL;
			middleBlocks.address = 0;
			middleBlocks.length = 0;

			endBlock.buffer = startBuffer;
			endBlock.address = address + blockRemainBytes;
			endBlock.length = length - blockRemainBytes;
		}
	}

	if ( startBlock.length != 0 ) {
		result = flash_internal_read_manual_mode(ctrl, startBlock.buffer, startBlock.address, startBlock.length);
		if ( 0 != result ) {
			goto _FUNC_END;
		}
	}

	if ( middleBlocks.length != 0 ) {
#ifdef CONTINUOUS_READ_USE
		result = flash_internal_read_middle_area(ctrl, middleBlocks.buffer, middleBlocks.address, middleBlocks.length);
#else
		result = flash_internal_read_manual_mode(ctrl, middleBlocks.buffer, middleBlocks.address, middleBlocks.length);
#endif
		if ( 0 != result ) {
			goto _FUNC_END;
		}
	}

	if ( endBlock.length != 0 ) {
		result = flash_internal_read_manual_mode(ctrl, endBlock.buffer, endBlock.address, endBlock.length);
		if ( 0 != result ) {
			goto _FUNC_END;
		}
	}

_FUNC_END:
	if (result != 0) {
		result = -1;
	}
	RZA_PRINTF_VERBOSE("<- flash_read() result=%d\n", result);

	return result;
}

static int flash_enter_xip(xspidevice_ctrl_t * ctrl)
{
	int result = 0;

	return result;
}

static int flash_exit_xip(xspidevice_ctrl_t * ctrl)
{
	int result = 0;

	return result;
}

static int flash_get_logical_address(xspidevice_ctrl_t * ctrl, size_t paddress, size_t * const laddress)
{
	int result;
	
	uint32_t pblock;
	uint32_t lblock;
	size_t   offset;
	
	if ((NULL == ctrl) || (NULL == laddress)) {
		result = SPI_NAND_ERROR_PARAM;
		RZA_PRINTF_VERBOSE("<- flash_get_logical_address() result=%d\n", result);
		return result;
	}
	
	/* Has the specified address exceeded the NAND area? */
	pblock = paddress / (SPI_NAND_PAGE_MAIN_BYTES * SPI_NAND_NUM_OF_PAGES_IN_BLOCK);
	offset = paddress % (SPI_NAND_PAGE_MAIN_BYTES * SPI_NAND_NUM_OF_PAGES_IN_BLOCK);
	if( pblock >= numOfLogBlocks ){
		result = SPI_NAND_ERROR_FAIL;
		RZA_PRINTF_VERBOSE("<- get_logical_address() result=%d\n", result);
		return result;
	}
	
	result = spi_nand_bb_Get_P2L_Block_Table(pblock, &lblock);
	if (SPI_NAND_ERROR_NONE == result) {
		*laddress = (lblock * (SPI_NAND_PAGE_MAIN_BYTES * SPI_NAND_NUM_OF_PAGES_IN_BLOCK)) + offset;
	}
	
	RZA_PRINTF_VERBOSE("<- get_logical_address() logical address=0x%lx  , result=%d\n", *laddress, result);

	return result;
}
