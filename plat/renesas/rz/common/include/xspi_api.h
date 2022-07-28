/*
 * Copyright (c) 2021, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _XSPI_API_H_
#define _XSPI_API_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/** xSPI Transfer form */
typedef enum e_xspi_transfer_form
{
	SPI_FORM_1_1_1,		/* Command executes at 1-1-1 form */
	SPI_FORM_1_1_2,		/* Command executes at 1-1-2 form */
	SPI_FORM_1_2_2,		/* Command executes at 1-2-2 form */
	SPI_FORM_2_2_2,		/* Command executes at 2-2-2 form */
	SPI_FORM_1_1_4,		/* Command executes at 1-1-4 form */
	SPI_FORM_1_4_4,		/* Command executes at 1-4-4 form */
	SPI_FORM_4_4_4,		/* Command executes at 4-4-4 form */
	SPI_FORM_SOPI,		/* Command executes at SOPI form */
	SPI_FORM_DOPI,		/* Command executes at DOPI form */
	SPI_FORM_OCTARAM,	/* Command executes at DOPI(OctaRAM) form */
} xspi_transfer_form_t;

/** xSPI interface configuration */
typedef struct st_xspi_cfg
{
	uint8_t channel;	/* Channel number to be used */
	uintptr_t base;		/* Regisgter base address */
	void const * extend;	/* Implementation specific extended configuration */
} xspi_cfg_t;

/** xSPI control block.  Allocate an implementation specific control block to pass into the xSPI API calls. */
typedef void xspi_ctrl_t;

/** xSPI operation table
 * *** CUT HERE BEFORE RELEASE, NOTICE FOR INTERNAL DEVELOPER ***
 * This table contains logical parameter, not a register value.
 * Do not modify this table as register value!
 * *** CUT HERE BEFORE RELEASE, NOTICE FOR INTERNAL DEVELOPER ***
 */
typedef struct st_xspi_op
{
	xspi_transfer_form_t form;	///< Transfer form */
	uint16_t op;			///< Operation code (for 2 byte op, highside used as 1st byte) */
	uint8_t  op_size;		///< Operation code size (0 to 2) */
	uint32_t address;		///< Address (ignored for configuring in-place access) */
	uint8_t  address_size;		///< Address size (0 to 4) */
	bool     address_is_ddr;	///< Address phase is DDR */
	uint8_t  additional_size;	///< Additional data size */
	uint32_t additional_value;	///< Additional data value */
	uint8_t  dummy_cycles;		///< Dummy cycle count (as clocks) */
	size_t   transfer_size;		///< Transfer size by bytes (ignored for configuring in-place access) */
	void *   transfer_buffer;	///< Transfer buffer pointer (ignored for configuring in-place access) */
	bool     transfer_is_ddr;	///< Transfer phase is DDR */
	uint8_t  slch_value;
	uint8_t  clsh_value;
	uint8_t  shsl_value;
} xspi_op_t;

/** Shared Interface definition for xSPI */
typedef struct st_xspi_api
{
	/** Prepare a channel for xSPI operation.
	 *
	 * @param[in, out] ctrl Pointer to user-provided storage for the control block.
	 * @param[in]      cfg  Pointer to xSPI configuration structure.
	 */
	int (* open)(xspi_ctrl_t * ctrl, xspi_cfg_t const * const cfg);

	/** Clean up the xSPI control block.
	 *
	 * @param[in]  ctrl  Pointer to the opened control block.
	 */
	int (* close)(xspi_ctrl_t * const ctrl);

	/** Post initialization routine
	 *
	 * @param[in]  ctrl  Pointer to the opened control block.
	 */
	int (* post_init)(xspi_ctrl_t * const ctrl);

	/** Execute operation.
	 *
	 * @param[in, out] ctrl Pointer to the control block.
	 * @param[in]      op     Operation structure
	 * @param[in]      is_write   Transaction phase direction is write
	 */
	int (* exec_op)(xspi_ctrl_t * const ctrl, xspi_op_t const * const op, bool is_write);

	/** Configure for in-place access.
	 * For enabling in-place access, start_xip must be called.
	 *
	 * @param[in, out] ctrl Pointer to the control block.
	 * @param[in]      rop    Operation structure for read operation.
	 * @param[in]      wop    Operation structure for write operation.
	 * @note
	 * For the flash, wop must be nullptr.
	 * @note
	 * For the OctaRAM, wop must be specified.
	 */
	int (* configure_xip)(xspi_ctrl_t * const ctrl, xspi_op_t const * const rop, xspi_op_t const * const wop);

	/** Starting in-place access.
	 * For configuring in-place access, configure_xip must be called.
	 *
	 * @param[in, out] ctrl Pointer to the control block.
	 */
	int (* start_xip)(xspi_ctrl_t * const ctrl);

	/** Stopping in-place access.
	 *
	 * @param[in, out] ctrl Pointer to the control block.
	 */
	int (* stop_xip)(xspi_ctrl_t * const ctrl);

	/** Do manual calibration.
	 *
	 * @param[in, out] ctrl Pointer to the control block.
	 */
	int (* run_manual_calibration)(xspi_ctrl_t * const ctrl);

	/** Enabling auto calibration.
	 *
	 * @param[in, out] ctrl Pointer to the control block.
	 */
	int (* enable_auto_calibration)(xspi_ctrl_t * const ctrl);

	/** Disabling auto calibration.
	 *
	 * @param[in, out] ctrl Pointer to the control block.
	 */
	int (* disable_auto_calibration)(xspi_ctrl_t * const ctrl);

	/** Setting SPI frequency.
	 *
	 * @param[in, out] ctrl Pointer to the control block.
	 * @param[in] frequency_hz Pointer to the control block.
	 */
	int (* set_frequency)(xspi_ctrl_t * const ctrl, int frequency_hz);

	/** Invalidate cache for entire MMAP
	 *
	 * @param[in] ctrl Pointer to the control block.
	 */
	int (* inv_mmap)(xspi_ctrl_t * const ctrl);

	/** Clean cache for entire MMAP
	 *
	 * @param[in] ctrl Pointer to the control block.
	 */
	int (* clean_mmap)(xspi_ctrl_t * const ctrl);

	/** Get MMAP base
	 *
	 * @param[in] ctrl Pointer to the control block.
	 */
	uintptr_t (* get_mmap_base)(xspi_ctrl_t * const ctrl);

	/** Get MMAP size
	 *
	 * @param[in] ctrl Pointer to the control block.
	 */
	size_t (* get_mmap_size)(xspi_ctrl_t * const ctrl);
} xspi_api_t;

/** This structure encompasses everything that is needed to use an instance of this interface. */
typedef struct st_xspi_instance
{
	xspi_ctrl_t      * ctrl;          ///< Pointer to the control structure for this instance
	xspi_cfg_t const * cfg;           ///< Pointer to the configuration structure for this instance
	xspi_api_t const * api;           ///< Pointer to the API structure for this instance
} xspi_instance_t;

/* external functions */
extern void xspi_setup(void);


#endif	/* _XSPI_API_H_ */
