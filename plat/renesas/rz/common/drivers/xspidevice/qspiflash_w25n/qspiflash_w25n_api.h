/*
 * Copyright (c) 2024, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _QSPIFLASH_W25N_API_H_
#define _QSPIFLASH_W25N_API_H_
#include <string.h>
#include <stdbool.h>

#include "xspidevice_api.h"

/** xSPI device control structure */
typedef struct st_qspiflash_w25n_ctrl
{
	bool opened;
	const xspi_instance_t * xspi;
} qspiflash_w25n_ctrl_t;

/** xSPI device extended configuration structure */
typedef struct st_qspiflash_w25n_ext
{
	/* No additional configurations */
} qspiflash_w25n_ext_t;

extern const xspidevice_api_t qspiflash_w25n_api;

#endif	/* _QSPIFLASH_W25N_API_H_ */

