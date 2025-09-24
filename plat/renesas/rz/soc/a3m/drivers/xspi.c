#include <xspi_api.h>
#include <xspidevice_api.h>
#include <assert.h>
#include <common/debug.h>
#include <rza_printf.h>

static int sub_init_xip(const xspidevice_instance_t *inst)
{
	int result;
	result = inst->api->open(inst->ctrl, inst->cfg);
	if (!result) {
		result = inst->api->enter_xip(inst->ctrl);
		inst->api->close(inst->ctrl);
	}
	return result;
}

static int sub_post_init_xip(const xspi_instance_t *xspi)
{
	int result;
	result = xspi->api->open(xspi->ctrl, xspi->cfg);
	if (!result) {
		xspi->api->enable_auto_calibration(xspi->ctrl);
		result = xspi->api->post_init(xspi->ctrl);
		xspi->api->close(xspi->ctrl);
	}
	return result;
}

void xspi_setup(void)
{
	int i;
	int result = 0;

#ifdef XSPI_DEVICE_TYPE
	RZA_PRINTF("Configure %s Flash Memory\n", XSPI_DEVICE_TYPE);
#endif
	for (i = 0; result >= 0 && i < xspidevices_count; i++) {
		result = sub_init_xip(xspidevices[i]);
		if (result < 0) {
			ERROR("xspi.c: abort on sub_init_xip (#%d)\n", i);
			panic();
		}
	}
	for (i = 0; result >= 0 && i < xspidevices_count; i++) {
		result = sub_post_init_xip(xspidevices[i]->cfg->xspi);
		if (result < 0) {
			ERROR("xspi.c: abort on sub_post_init_xip (#%d)\n", i);
			panic();
		}
	}
}

/**
 * memcpy for xspi
 */
int rz_xspi_read(void* to, uint64_t offset, size_t length) {
	int result;
	size_t fsp_base_addr = offset;
#if (FLASH_MEMORY_TYPE == NAND_FLASH)
	size_t loffset;
#endif /* (FLASH_MEMORY_TYPE == NAND_FLASH) */
	const xspidevice_instance_t *inst = xspidevices[0];
	if (!inst) return -1;

	result = inst->api->open(inst->ctrl, inst->cfg);

#if (FLASH_MEMORY_TYPE == NAND_FLASH)
	if (!result) {
		result = inst->api->get_logical_address(inst->ctrl, offset, &loffset);
		if (!result) {
			fsp_base_addr = loffset;
		}
	}
#endif /* (FLASH_MEMORY_TYPE == NAND_FLASH) */

	if (!result) {
		inst->api->read(inst->ctrl, to, fsp_base_addr, length);
		inst->api->close(inst->ctrl);
	}
	return result;
}
