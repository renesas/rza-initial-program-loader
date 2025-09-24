/*
 * Copyright (c) 2024, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/* ************************ HEADER (INCLUDE) SECTION *********************** */

#include "qspiflash_w25n_api.h"
#include "spi_nand.h"
#include "W25N01GVZEIG.h"

/* ***************** MACROS, CONSTANTS, COMPILATION FLAGS ****************** */

/* ********************** STRUCTURES, TYPE DEFINITIONS ********************* */

/* ********************** DECLARATION OF EXTERNAL DATA ********************* */

/* ********************** DECLARATION OF INTERNAL DATA ********************* */

/** @brief SPI NAND Flash Memory Command
 */

/* Reset */
static const xspi_op_t op_device_reset = {
	.form = SPI_FORM_1_1_1,
	.op = SPI_NAND_CMD_DEVICE_RESET,
	.op_size = 1,
	.op_is_ddr = false,
	.address = 0,
	.address_is_ddr = false,
	.address_size = 0,
	.additional_value = 0,
	.additional_size = 0,
	.dummy_cycles = 0,
	.transfer_buffer = NULL,
	.transfer_is_ddr = false,
	.transfer_size = 0,
	.force_idle_level_mask = 0x08,	// Keep IO3/HOLD pin to High
	.force_idle_level_value = 0x08,	// Keep IO3/HOLD pin to High
	.slch_value = 0,
	.clsh_value = 0,
	.shsl_value = 3
};

/* Read Status Regiser */
static const xspi_op_t op_read_status_register = {
	.form = SPI_FORM_1_1_1,
	.op = SPI_NAND_CMD_READ_STATUS_REGISTER,
	.op_size = 1,
	.op_is_ddr = false,
	.address = 0,
	.address_is_ddr = false,
	.address_size = 0,
	.additional_value = 0,
	.additional_size = 1, /* This command requires the Status register address (8 bits) to be set. */
						  /* However, in the normal SPIBSC format, the Address can only be set to 24 or 32 bits. */
						  /* Therefore, the Status register address is sent to the Flash memory as additional data. */
	.dummy_cycles = 0,
	.transfer_buffer = NULL,
	.transfer_is_ddr = false,
	.transfer_size = 1,
	.force_idle_level_mask = 0x08,	// Keep IO3/HOLD pin to High
	.force_idle_level_value = 0x08,	// Keep IO3/HOLD pin to High
	.slch_value = 0,
	.clsh_value = 0,
	.shsl_value = 3
};

/* Write Status Register */
static const xspi_op_t op_write_status_register = {
	.form = SPI_FORM_1_1_1,
	.op = SPI_NAND_CMD_WRITE_STATUS_REGISTER,
	.op_size = 1,
	.op_is_ddr = false,
	.address = 0,
	.address_is_ddr = false,
	.address_size = 0,
	.additional_value = 0,
	.additional_size = 1, /* This command requires the Status register address (8 bits) to be set. */
						  /* However, in the normal SPIBSC format, the Address can only be set to 24 or 32 bits. */
						  /* Therefore, the Status register address is sent to the Flash memory as additional data. */
	.dummy_cycles = 0,
	.transfer_buffer = NULL,
	.transfer_is_ddr = false,
	.transfer_size = 1,
	.force_idle_level_mask = 0x08,	// Keep IO3/HOLD pin to High
	.force_idle_level_value = 0x08,	// Keep IO3/HOLD pin to High
	.slch_value = 0,
	.clsh_value = 0,
	.shsl_value = 3
};

/* Page Data Read */
static const xspi_op_t op_page_data_read = {
	.form = SPI_FORM_1_1_1,
	.op = SPI_NAND_CMD_PAGE_DATA_READ,
	.op_size = 1,
	.op_is_ddr = false,
	.address = 0,
	.address_is_ddr = false,
	.address_size = 3, /* The Page Data Read command has a special format that requires a dummy cycle */
					   /* immediately after the command data. To reproduce this format in the SPIBSC, */
					   /* a total of 3 bytes of data are assigned to the address in the address area: */
					   /* 0 data as a dummy cycle + Page address data. */
					   /* address[0 :15]: Page address data */
					   /* address[16:23] : dummy cycle */
	.additional_value = 0,
	.additional_size = 0,
	.dummy_cycles = 0,
	.transfer_buffer = NULL,
	.transfer_is_ddr = false,
	.transfer_size = 0,
	.force_idle_level_mask = 0x08,	// Keep IO3/HOLD pin to High
	.force_idle_level_value = 0x08,	// Keep IO3/HOLD pin to High
	.slch_value = 0,
	.clsh_value = 0,
	.shsl_value = 3
};

#ifdef SPI_NAND_QUAD_READ_USE
/* Fast Read Quad I/O */
static const xspi_op_t op_fast_read_quad_io = {
	.form = SPI_FORM_1_4_4,
	.op = SPI_NAND_CMD_FAST_READ_QUAD_IO,
	.op_size = 1,
	.op_is_ddr = false,
	.address = 0,
	.address_is_ddr = false,
	.address_size = 0,
	.additional_value = 0,
	.additional_size = 2, /* The Fast Read Quad I/O command in Buffer Read Mode requires the Column Address[15:0] to be set. */
						  /* However, in the normal SPIBSC format, the Address can only be set to 24 or 32 bits. */
						  /* Therefore, the Status register address is sent to the Flash memory as additional data. */
	.dummy_cycles = 4,
	.transfer_buffer = NULL,
	.transfer_is_ddr = false,
	.transfer_size = 4,
	.force_idle_level_mask = 0x08,	// Keep IO3/HOLD pin to High
	.force_idle_level_value = 0x08,	// Keep IO3/HOLD pin to High
	.slch_value = 0,
	.clsh_value = 0,
	.shsl_value = 3
};
#else /* SPI_NAND_QUAD_READ_USE */
/* Read Data */
static const xspi_op_t op_read_data = {
	.form = SPI_FORM_1_1_1,
	.op = SPI_NAND_CMD_READ,
	.op_size = 1,
	.op_is_ddr = false,
	.address = 0,
	.address_is_ddr = false,
	.address_size = 0,
	.additional_value = 0,
	.additional_size = 2, /* The Read Data command in Buffer Read Mode requires the Column Address[15:0] to be set. */
						  /* However, in the normal SPIBSC format, the Address can only be set to 24 or 32 bits. */
						  /* Therefore, the Status register address is sent to the Flash memory as additional data. */
	.dummy_cycles = 8,
	.transfer_buffer = NULL,
	.transfer_is_ddr = false,
	.transfer_size = 4,
	.force_idle_level_mask = 0x08,	// Keep IO3/HOLD pin to High
	.force_idle_level_value = 0x08,	// Keep IO3/HOLD pin to High
	.slch_value = 0,
	.clsh_value = 0,
	.shsl_value = 3
};
#endif /* SPI_NAND_QUAD_READ_USE */

#ifdef SPI_NAND_EXEC_OP_READ_REPEAT_USE
static uint32_t tempBuffer[(SPI_NAND_PAGE_MAIN_BYTES + SPI_NAND_PAGE_SPARE_BYTES)/sizeof(uint32_t)];
#endif /* SPI_NAND_EXEC_OP_READ_REPEAT_USE */

/* ************************** FUNCTION PROTOTYPES ************************** */

#ifdef SPI_NAND_EXEC_OP_READ_REPEAT_USE
extern int spim_exec_op_read_repeat(xspi_ctrl_t * const ctrl,
									xspi_op_t const * const op, 
									bool is_write,
									int repeat); /* spim.c */
#endif /* SPI_NAND_EXEC_OP_READ_REPEAT_USE */

static int spi_nand_read_register(xspidevice_ctrl_t* ctrl, uint8_t address, uint8_t* value);
static int spi_nand_write_register(xspidevice_ctrl_t* ctrl, uint8_t address, uint8_t value);
static int spi_nand_read_page_manual_mode(xspidevice_ctrl_t* ctrl, uint8_t* buffer, uint32_t block, uint32_t page, uint32_t pageOffset, size_t length, bool eccEnable);
static int spi_nand_set_ecc_en(xspidevice_ctrl_t* ctrl);
static int spi_nand_clear_ecc_en(xspidevice_ctrl_t* ctrl);
static int spi_nand_clear_status_bit(xspidevice_ctrl_t* ctrl, uint8_t regAddr, uint8_t bitValue, uint8_t expectValue);
static int spi_nand_set_status_bit(xspidevice_ctrl_t* ctrl, uint8_t regAddr, uint8_t bitValue, uint8_t expectValue);
static int spi_nand_reset(xspidevice_ctrl_t* ctrl, uint32_t maxWaitTime);
static int spi_nand_unlock_block_protection(xspidevice_ctrl_t* ctrl);
static int spi_nand_set_quad_enable(xspidevice_ctrl_t* ctrl);

/* ********************************* CODE ********************************** */

/*
 * INTERNAL FUNCTIONS
 */

static inline void spi_nand_set_radd(uint32_t block, uint32_t page, uint32_t *radd)
{
	*radd = (block << SPI_NAND_RADD_BLOCK_ADDRESS_POS) | page;
}

static inline void spi_nand_set_caddr(uint32_t offset, uint32_t *caddr)
{
	*caddr = offset;
}

/** @brief Get feature
 *
 * - Pre-conditions:<BR>
 * .
 * - Post-conditions:<BR>
 * .
 *
 * @param[in] ctrl
 * @param[in] address
 * @param[out] value
 */
static int spi_nand_read_register(xspidevice_ctrl_t* ctrl, uint8_t address, uint8_t* value)
{
	int ret = SPI_NAND_ERROR_NONE;
	bool is_write = false;
	xspi_op_t op = op_read_status_register;
	uint8_t status = 0;
	int result;

	SPI_NAND_ENTERF("spi_nand_read_register");

	if ( (NULL == ctrl) || (NULL == value) ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_read_register", ret);
		return ret;
	}

	if (( SPI_NAND_SR_ADD_PROTECTION_REGISTER == address ) ||
		(SPI_NAND_SR_ADD_CONFIGURATION_REGISTER == address) ||
		(SPI_NAND_SR_ADD_STATUS_REGISTER == address) ) {
	}
	else {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_read_register", ret);
		return ret;
	}

	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = myctrl->xspi;

	op.additional_value = address;
	op.transfer_buffer = (void*)&status;
	result = xspi->api->exec_op(xspi->ctrl, &op, is_write);
	if (0 == result) {
		*value = status;
		ret = SPI_NAND_ERROR_NONE;
	}
	else {
		ret = SPI_NAND_ERROR_SYS;
	}

	SPI_NAND_EXITF("spi_nand_read_register", ret);
	return ret;
}

/** @brief Set feature
 *
 * - Pre-conditions:<BR>
 * .
 * - Post-conditions:<BR>
 * .
 *
 * @param[in] ctrl
 * @param[in] address
 * @param[in] value
 */
static int spi_nand_write_register(xspidevice_ctrl_t* ctrl, uint8_t address, uint8_t value)
{
	int ret = SPI_NAND_ERROR_NONE;
	bool is_write = true;
	xspi_op_t op = op_write_status_register;
	int result;

	SPI_NAND_ENTERF("spi_nand_write_register");

	if ( NULL == ctrl ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_write_register", ret);
		return ret;
	}

	/*
	 * Status Regster-3 (Status Only)
	 */
	if ((SPI_NAND_SR_ADD_PROTECTION_REGISTER == address) ||
		(SPI_NAND_SR_ADD_CONFIGURATION_REGISTER == address) ) {
	}
	else {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_write_register", ret);
		return ret;
	}

	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = myctrl->xspi;

	op.additional_value = address;
	op.transfer_buffer = (void*)&value;
	result = xspi->api->exec_op(xspi->ctrl, &op, is_write);
	if (0 != result) {
		ret = SPI_NAND_ERROR_SYS;
	}

	SPI_NAND_EXITF("spi_nand_write_register", ret);
	return ret;
}

/** @brief Read page data by manual mode
 *
 * - Pre-conditions:<BR>
 * .
 * - Post-conditions:<BR>
 * .
 *
 * @param[in] ctrl
 * @param[out] buffer
 * @param[int] block
 * @param[int] page
 * @param[int] pageOffset
 * @param[int] length
 * @param[int] eccCheck
 */
static int spi_nand_read_page_manual_mode(xspidevice_ctrl_t* ctrl, uint8_t* buffer, uint32_t block, uint32_t page, uint32_t pageOffset, size_t length, bool eccEnable)
{
	int ret = SPI_NAND_ERROR_NONE;

	uint32_t radd;
	bool is_write = false;
	xspi_op_t op;
	int i;
	uint8_t regAdd = SPI_NAND_SR_ADD_STATUS_REGISTER;
	uint8_t status = 0;
	uint32_t cadd;
#ifdef SPI_NAND_EXEC_OP_READ_REPEAT_USE
#else
	uint32_t readData;
#endif
	int result;

	SPI_NAND_ENTERF("spi_nand_read_page_manual_mode");

	if ( (NULL == ctrl) || (NULL == buffer) ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_read_page_manual_mode", ret);
		return ret;
	}

	if ( (SPI_NAND_NUM_OF_BLOCKS <= block) ||
		(SPI_NAND_NUM_OF_PAGES_IN_BLOCK <= page) ||
		((SPI_NAND_PAGE_MAIN_BYTES + SPI_NAND_PAGE_SPARE_BYTES) <= pageOffset) ||
		((SPI_NAND_PAGE_MAIN_BYTES + SPI_NAND_PAGE_SPARE_BYTES) < length ) ||
		(0 != (pageOffset % sizeof(uint32_t))) ||
		(0 != (length % sizeof(uint32_t))) ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_read_page_manual_mode", ret);
		return ret;
	}

	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = myctrl->xspi;

	if ( false == eccEnable ) {
		ret = spi_nand_clear_ecc_en( ctrl );
		if (SPI_NAND_ERROR_NONE != ret) {
			goto _FUNC_END;
		}
	}

	spi_nand_set_radd( block, page, &radd);
	op = op_page_data_read;
	op.address = radd;
	result = xspi->api->exec_op(xspi->ctrl, &op, is_write);
	if ( 0 != result ) {
		ret = SPI_NAND_ERROR_SYS;
		goto _FUNC_END;
	}

	/*
	 * Wait ready
	 */
	for (i = 0; i < SPI_NAND_TIME_WAIT_READY_PAGE_READ; i++ ) {
		ret = spi_nand_read_register(ctrl, regAdd, &status);
		if (SPI_NAND_ERROR_NONE != ret) {
			goto _FUNC_END;
		}
		if ((status & SPI_NAND_REG_STATUS_BUSY) == 0) {
			break;
		}
	}
	if (i >= SPI_NAND_TIME_WAIT_READY_PAGE_READ) {
		ret = SPI_NAND_ERROR_BUSY;
		goto _FUNC_END;
	}

	if ( true == eccEnable ) {
		/*
		 * Check ecc uncorrectable error
		 */
		if (((status & SPI_NAND_REG_STATUS_ECC) == (SPI_NAND_REG_STATUS_ECC_2BIT_SINGLE << SPI_NAND_REG_STATUS_ECC_POS)) ||
			((status & SPI_NAND_REG_STATUS_ECC) == (SPI_NAND_REG_STATUS_ECC_2BIT_MULTI << SPI_NAND_REG_STATUS_ECC_POS))){
			ret = SPI_NAND_ERROR_ECC;
			goto _FUNC_END;
		}
	}

	/*
	 * Read data from page cache
	 */
	spi_nand_set_caddr( pageOffset, &cadd);
#ifdef SPI_NAND_QUAD_READ_USE
	op = op_fast_read_quad_io;
#else /* SPI_NAND_QUAD_READ_USE */
	op = op_read_data;
#endif /* SPI_NAND_QUAD_READ_USE */

#ifdef SPI_NAND_EXEC_OP_READ_REPEAT_USE
	op.additional_value = cadd;
	op.transfer_buffer = (void*)&tempBuffer[0];
	result = spim_exec_op_read_repeat(xspi->ctrl, &op, false, length/sizeof(uint32_t));
	if ( 0 != result ) {
		ret = SPI_NAND_ERROR_SYS;
		goto _FUNC_END;
	}
	memcpy(buffer, &tempBuffer[0], length);
#else /* SPI_NAND_EXEC_OP_READ_REPEAT_USE */
	for (i = 0; i < length; i += sizeof(uint32_t)) {
		op.additional_value = cadd;
		op.transfer_buffer = (void*)&readData;
		result = xspi->api->exec_op(xspi->ctrl, &op, is_write);
		if ( 0 != result ) {
			ret = SPI_NAND_ERROR_SYS;
			goto _FUNC_END;
		}
		cadd += sizeof(uint32_t);
		memcpy(&buffer[i], &readData, sizeof(uint32_t));
	}
#endif /* SPI_NAND_EXEC_OP_READ_REPEAT_USE */

_FUNC_END:
	if ( false == eccEnable ) {
		spi_nand_set_ecc_en( ctrl );
	}
	SPI_NAND_EXITF("spi_nand_read_page_manual_mode", ret);
	return ret;
}

static int spi_nand_clear_status_bit(xspidevice_ctrl_t* ctrl, uint8_t regAddr, uint8_t bitValue, uint8_t expectValue)
{
	int ret = SPI_NAND_ERROR_NONE;
	uint8_t value = 0;
	uint8_t status = 0;

	SPI_NAND_ENTERF("spi_nand_clear_status_bit");

	if ( NULL == ctrl ) {
		ret = SPI_NAND_ERROR_PARAM;
		goto _FUNC_END;
	}

	ret = spi_nand_read_register(ctrl, regAddr, &value);
	if (SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	if ( (value & bitValue) == expectValue ) {
		ret = SPI_NAND_ERROR_NONE;
		goto _FUNC_END;
	}

	value &= ~(bitValue);

	ret = spi_nand_write_register(ctrl, regAddr, value);
	if (SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	ret = spi_nand_read_register(ctrl, regAddr, &status);
	if (SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	if ( value != status ) {
		ret = SPI_NAND_ERROR_SYS;
		goto _FUNC_END;
	}

_FUNC_END:
	SPI_NAND_EXITF("spi_nand_clear_status_bit", ret);
	return ret;
}

static int spi_nand_set_status_bit(xspidevice_ctrl_t* ctrl, uint8_t regAddr, uint8_t bitValue, uint8_t expectValue)
{
	int ret = SPI_NAND_ERROR_NONE;
	uint8_t value = 0;
	uint8_t status = 0;

	SPI_NAND_ENTERF("spi_nand_set_status_bit");

	if ( NULL == ctrl ) {
		ret = SPI_NAND_ERROR_PARAM;
		goto _FUNC_END;
	}

	ret = spi_nand_read_register(ctrl, regAddr, &value);
	if (SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	if ( (value & bitValue) == expectValue ) {
		ret = SPI_NAND_ERROR_NONE;
		goto _FUNC_END;
	}

	value |= (bitValue);

	ret = spi_nand_write_register(ctrl, regAddr, value);
	if (SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	ret = spi_nand_read_register(ctrl, regAddr, &status);
	if (SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	if ( value != status ) {
		ret = SPI_NAND_ERROR_SYS;
		goto _FUNC_END;
	}

_FUNC_END:
	SPI_NAND_EXITF("spi_nand_set_status_bit", ret);
	return ret;
}

static int spi_nand_set_ecc_en(xspidevice_ctrl_t* ctrl)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_set_ecc_en");

	ret = spi_nand_set_status_bit(	ctrl,
									SPI_NAND_SR_ADD_CONFIGURATION_REGISTER,
									SPI_NAND_REG_CONFIGURATION_ECC,
									(SPI_NAND_REG_CONFIGURATION_ECC_ENABLE << SPI_NAND_REG_CONFIGURATION_ECC_POS));

	SPI_NAND_EXITF("spi_nand_set_ecc_en", ret);

	return ret;
}

static int spi_nand_clear_ecc_en(xspidevice_ctrl_t* ctrl)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_clear_ecc_en");

	ret = spi_nand_clear_status_bit( ctrl,
									 SPI_NAND_SR_ADD_CONFIGURATION_REGISTER,
									 SPI_NAND_REG_CONFIGURATION_ECC,
									 (SPI_NAND_REG_CONFIGURATION_ECC_DISABLE << SPI_NAND_REG_CONFIGURATION_ECC_POS));

	SPI_NAND_EXITF("spi_nand_clear_ecc_en", ret);

	return ret;
}

int spi_nand_reset(xspidevice_ctrl_t* ctrl, uint32_t maxWaitTime)
{
	int ret = SPI_NAND_ERROR_NONE;
	xspi_op_t op = op_device_reset;
	uint8_t regAdd = SPI_NAND_SR_ADD_STATUS_REGISTER;
	uint8_t status = 0;
	uint32_t i;
	int result;

	SPI_NAND_ENTERF("spi_nand_reset");

	if (NULL == ctrl) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_reset", ret);
		return ret;
	}

	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = myctrl->xspi;

	/* Issue reset command */
	result = xspi->api->exec_op(xspi->ctrl, &op, false);
	if ( 0 != result ) {
		ret = SPI_NAND_ERROR_SYS;
		goto _FUNC_END;
	}

	for (i = 0; i < maxWaitTime; i++) {
		ret = spi_nand_read_register(ctrl, regAdd, &status);
		if (SPI_NAND_ERROR_NONE != ret) {
			goto _FUNC_END;
		}
		if ((status & SPI_NAND_REG_STATUS_BUSY) == 0) {
			break;
		}
	}
	if (i >= maxWaitTime) {
		ret = SPI_NAND_ERROR_BUSY;
	}

_FUNC_END:
	SPI_NAND_EXITF("spi_nand_reset", ret);
	return ret;
}

int spi_nand_unlock_block_protection(xspidevice_ctrl_t* ctrl)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_unlock_block_protection");

	ret = spi_nand_clear_status_bit( ctrl,
									 SPI_NAND_SR_ADD_PROTECTION_REGISTER,
									 SPI_NAND_REG_PROTECTION_BP,
									 (SPI_NAND_REG_PROTECTION_BP_POS_UNLOCKED << SPI_NAND_REG_PROTECTION_BP_POS));

	SPI_NAND_EXITF("spi_nand_unlock_block_protection", ret);

	return ret;
}

static int spi_nand_set_quad_enable(xspidevice_ctrl_t* ctrl)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_set_quad_enable");

	if (NULL == ctrl) {
		ret = SPI_NAND_ERROR_PARAM;
		goto _FUNC_END;
	}

	/*
	 * W25N is always enabled Quad Operation.
	 */

_FUNC_END:
	SPI_NAND_EXITF("spi_nand_set_quad_enable", ret);
	return ret;
}

/*
 * EXTERNAL FUNCTIONS
 */

/** @brief spi_nand_Open
 *
 * - Pre-conditions:<BR>
 * .
 * - Post-conditions:<BR>
 * .
 *
 * @param[in/out] ctrl
 */
int spi_nand_Open(xspidevice_ctrl_t* ctrl)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_Open");

	if ( NULL == ctrl ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_Open", ret);
		return ret;
	}

	ret = spi_nand_reset(ctrl, SPI_NAND_TIME_WAIT_READY_RESET);
	if ( SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	ret = spi_nand_unlock_block_protection( ctrl );
	if ( SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	ret = spi_nand_set_quad_enable( ctrl );
	if ( SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

_FUNC_END:
	SPI_NAND_EXITF("spi_nand_Open", ret);
	return ret;
}

/** @brief spi_nand_Close
 *
 * - Pre-conditions:<BR>
 * .
 * - Post-conditions:<BR>
 * .
 *
 * @param[in] ctrl
 */
int spi_nand_Close(xspidevice_ctrl_t* ctrl)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_Close");

	if ( NULL == ctrl ) {
		ret = SPI_NAND_ERROR_PARAM;
	}

	SPI_NAND_EXITF("spi_nand_Close", ret);
	return ret;
}

/** @brief Read data from page spare area by SPIBSC manual mode.
 *
 * - Pre-conditions:<BR>
 * .
 * - Post-conditions:<BR>
 * .
 *
 * @param[in] ctrl
 * @param[out] buffer
 * @param[in] block
 * @param[in] page
 * @param[in] offset
 * @param[in] length
 * @param[in] eccCheck
 */
int spi_nand_Read_Page_Spare(xspidevice_ctrl_t* ctrl, uint8_t* buffer, uint32_t block, uint32_t page, uint32_t offset, size_t length, bool eccCheck)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_Read_Page_Spare");

	if ( (NULL == ctrl) || (NULL == buffer) ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_Read_Page_Spare", ret);
		return ret;
	}

	if ( ( SPI_NAND_NUM_OF_BLOCKS <= block ) ||
		( SPI_NAND_NUM_OF_PAGES_IN_BLOCK <= page ) ||
		( SPI_NAND_PAGE_SPARE_BYTES <= offset ) ||
		( SPI_NAND_PAGE_SPARE_BYTES < length) ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_Read_Page_Spare", ret);
		return ret;
	}

	ret = spi_nand_read_page_manual_mode(ctrl, buffer, block, page, SPI_NAND_PAGE_MAIN_BYTES + offset, length, eccCheck);

	SPI_NAND_EXITF("spi_nand_Read_Page_Spare", ret);
	return ret;
}

/** @brief Read data from page main area by SPIBSC manual mode.
 *
 * - Pre-conditions:<BR>
 * .
 * - Post-conditions:<BR>
 * .
 *
 * @param[in] ctrl
 * @param[out] buffer
 * @param[in] block
 * @param[in] page
 * @param[in] offset
 * @param[in] length
 */
int spi_nand_Read_Page_Main(xspidevice_ctrl_t* ctrl, uint8_t* buffer, uint32_t block, uint32_t page, uint32_t offset, size_t length)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_Read_Page_Main");

	if ( (NULL == ctrl) || (NULL == buffer) ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_Read_Page_Main", ret);
		return ret;
	}

	if ( ( SPI_NAND_NUM_OF_BLOCKS <= block ) ||
		( SPI_NAND_NUM_OF_PAGES_IN_BLOCK <= page ) ||
		( SPI_NAND_PAGE_MAIN_BYTES <= offset ) ||
		( SPI_NAND_PAGE_MAIN_BYTES < length) ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_Read_Page_Main", ret);
		return ret;
	}

	ret = spi_nand_read_page_manual_mode(ctrl, buffer, block, page, offset, length, true);

	SPI_NAND_EXITF("spi_nand_Read_Page_Main", ret);
	return ret;
}

int spi_nand_Set_Cont_Enable(xspidevice_ctrl_t* ctrl)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_Set_Cont_Enable");

	if (NULL == ctrl) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_Set_Cont_Enable", ret);
		return ret;
	}

	/*
	 * Continuous Read Mode(BUF=0)
	 */
	ret = spi_nand_clear_status_bit( ctrl,
									 SPI_NAND_SR_ADD_CONFIGURATION_REGISTER,
									 SPI_NAND_REG_CONFIGURATION_BUF,
									 (SPI_NAND_REG_CONFIGURATION_BUF_DISABLE << SPI_NAND_REG_CONFIGURATION_BUF_POS));

	SPI_NAND_EXITF("spi_nand_Set_Cont_Enable", ret);
	return ret;
}

int spi_nand_Clear_Cont_Enable(xspidevice_ctrl_t* ctrl)
{
	int ret = SPI_NAND_ERROR_NONE;

	SPI_NAND_ENTERF("spi_nand_Clear_Cont_Enable");

	if (NULL == ctrl) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_Clear_Cont_Enable", ret);
		return ret;
	}

	/*
	 * Buffer Read Mode(BUF=1)
	 */
	ret = spi_nand_set_status_bit(	ctrl,
									SPI_NAND_SR_ADD_CONFIGURATION_REGISTER,
									SPI_NAND_REG_CONFIGURATION_BUF,
									(SPI_NAND_REG_CONFIGURATION_BUF_ENABLE << SPI_NAND_REG_PROTECTION_BP_POS));

	SPI_NAND_EXITF("spi_nand_Clear_Cont_Enable", ret);

	return ret;
}

int spi_nand_Issue_Read_Page_Command(xspidevice_ctrl_t* ctrl, uint32_t block, uint32_t page)
{
	int ret = SPI_NAND_ERROR_NONE;
	uint32_t radd;
	bool is_write = false;
	xspi_op_t op;
	int i;
	uint8_t regAdd = SPI_NAND_SR_ADD_STATUS_REGISTER;
	uint8_t status = 0;
	int result;

	qspiflash_w25n_ctrl_t * myctrl = (qspiflash_w25n_ctrl_t *)ctrl;
	const xspi_instance_t * xspi = myctrl->xspi;

	SPI_NAND_ENTERF("spi_nand_Issue_Read_Page_Command");

	if (NULL == ctrl) {
		ret = SPI_NAND_ERROR_PARAM;
		goto _FUNC_END;
	}

	if ( (SPI_NAND_NUM_OF_BLOCKS <= block) ||
		(SPI_NAND_NUM_OF_PAGES_IN_BLOCK <= page) ) {
		ret = SPI_NAND_ERROR_PARAM;
		goto _FUNC_END;
	}

	spi_nand_set_radd( block, page, &radd);
	op = op_page_data_read;
	op.address = radd;
	result = xspi->api->exec_op(xspi->ctrl, &op, is_write);
	if ( 0 != result ) {
		ret = SPI_NAND_ERROR_SYS;
		goto _FUNC_END;
	}

	/*
	 * Wait ready by reading status register.
	 */
	for (i = 0; i < SPI_NAND_TIME_WAIT_READY_PAGE_READ; i++ ) {
		ret = spi_nand_read_register(ctrl, regAdd, &status);
		if (SPI_NAND_ERROR_NONE != ret) {
			goto _FUNC_END;
		}
		if ((status & SPI_NAND_REG_STATUS_BUSY) == 0) {
			break;
		}
	}
	if (i >= SPI_NAND_TIME_WAIT_READY_PAGE_READ) {
		ret = SPI_NAND_ERROR_BUSY;
		goto _FUNC_END;
	}

_FUNC_END:
	SPI_NAND_EXITF("spi_nand_Issue_Read_Page_Command", ret);
	return ret;
}

int spi_nand_Is_Ecc_Uncorrectable(xspidevice_ctrl_t* ctrl, bool *result)
{
	int ret = SPI_NAND_ERROR_NONE;
	uint8_t regAdd = SPI_NAND_SR_ADD_STATUS_REGISTER;
	uint8_t status = 0;

	SPI_NAND_ENTERF("spi_nand_Is_Ecc_Uncorrectable");

	if ( (NULL == ctrl) || (NULL == result) ) {
		ret = SPI_NAND_ERROR_PARAM;
		SPI_NAND_EXITF("spi_nand_Is_Ecc_Uncorrectable", ret);
		return ret;
	}

	ret = spi_nand_read_register(ctrl, regAdd, &status);
	if (SPI_NAND_ERROR_NONE != ret) {
		goto _FUNC_END;
	}

	if (((status & SPI_NAND_REG_STATUS_ECC) == (SPI_NAND_REG_STATUS_ECC_2BIT_SINGLE << SPI_NAND_REG_STATUS_ECC_POS)) ||
		((status & SPI_NAND_REG_STATUS_ECC) == (SPI_NAND_REG_STATUS_ECC_2BIT_MULTI << SPI_NAND_REG_STATUS_ECC_POS))){
		*result = true;
	}
	else {
		*result = false;
	}

_FUNC_END:
	SPI_NAND_EXITF("spi_nand_Is_Ecc_Uncorrectable", ret);
	return ret;
}
