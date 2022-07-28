#
# Copyright (c) 2021, Renesas Electronics Corporation. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

XSPI0_DEVICE?=qspiflash_at25
XSPI_DEVICE_TYPE:=QSPI

ifneq (${USE_SDRAM},0)
DDR_SOURCES	+=	plat/renesas/rz/soc/${PLAT}/drivers/ddr/param_mc_C-011_D4-01-2.c	\
				plat/renesas/rz/common/drivers/ddr/param_swizzle_T3bcud2.c

DDR_PLL4	:= 1600
$(eval $(call add_define,DDR_PLL4))
endif
$(eval $(call add_define_val,XSPI_DEVICE_TYPE,\"${XSPI_DEVICE_TYPE}\"))
