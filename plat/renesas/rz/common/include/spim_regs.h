/*
 * Copyright (c) 2022, Renesas Electronics Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SPIM_REG_H_
#define _SPIM_REG_H_

#define SPIM_CMNCR	0x0000
#define SPIM_SSLDR	0x0004
#define SPIM_DRCR	0x000c
#define SPIM_DRCMR	0x0010
#define SPIM_DREAR	0x0014
#define SPIM_DROPR	0x0018
#define SPIM_DRENR	0x001c
#define SPIM_SMCR	0x0020
#define SPIM_SMCMR	0x0024
#define SPIM_SMADR	0x0028
#define SPIM_SMOPR	0x002c
#define SPIM_SMENR	0x0030
#define SPIM_SMRDR0	0x0038
#define SPIM_SMRDR1	0x003c
#define SPIM_SMWDR0	0x0040
#define SPIM_SMWDR1	0x0044
#define SPIM_CMNSR	0x0048
#define SPIM_DRDMCR	0x0058
#define SPIM_DRDRENR	0x005c
#define SPIM_SMDMCR	0x0060
#define SPIM_SMDRENR	0x0064
#define SPIM_PHYADJ1	0x0070
#define SPIM_PHYADJ2	0x0074
#define SPIM_PHYCNT	0x007c
#define SPIM_PHYOFFSET1	0x0080
#define SPIM_PHYOFFSET2	0x0084
#define SPIM_PHYINT	0x0088
#define SPIM_BUFFER	0x10000

/* CMNCR field */
#define SPIM_CMNCR_MD_POS	31
#define SPIM_CMNCR_MD		(1u << SPIM_CMNCR_MD_POS)
#define SPIM_CMNCR_MOIIO3_POS	22
#define SPIM_CMNCR_MOIIO3	(3u << SPIM_CMNCR_MOIIO3_POS)
#define SPIM_CMNCR_MOIIO2_POS	20
#define SPIM_CMNCR_MOIIO2	(3u << SPIM_CMNCR_MOIIO2_POS)
#define SPIM_CMNCR_MOIIO1_POS	18
#define SPIM_CMNCR_MOIIO1	(3u << SPIM_CMNCR_MOIIO1_POS)
#define SPIM_CMNCR_MOIIO0_POS	16
#define SPIM_CMNCR_MOIIO0	(3u << SPIM_CMNCR_MOIIO0_POS)
#define SPIM_CMNCR_IO3FV_POS	14
#define SPIM_CMNCR_IO3FV	(3u << SPIM_CMNCR_IO3FV_POS)
#define SPIM_CMNCR_IO2FV_POS	12
#define SPIM_CMNCR_IO2FV	(3u << SPIM_CMNCR_IO2FV_POS)
#define SPIM_CMNCR_IO0FV_POS	8
#define SPIM_CMNCR_IO0FV	(3u << SPIM_CMNCR_IO0FV_POS)
#define SPIM_CMNCR_IO_LOW	0u
#define SPIM_CMNCR_IO_HIGH	1u
#define SPIM_CMNCR_IO_KEEP	2u
#define SPIM_CMNCR_IO_HIZ	3u
#define SPIM_CMNCR_BSZ_POS	0
#define SPIM_CMNCR_BSZ		(3u << SPIM_CMNCR_BSZ_POS)
#define SPIM_CMNCR_BSZ_SINGLE	0u
#define SPIM_CMNCR_BSZ_DUAL	1u
#define SPIM_CMNCR_BSZ_OCTA	1u
#define SPIM_CMNCR_BSZ_HYPER	1u

/* SSLDR field */
#define SPIM_SSLDR_SPNDL_POS	16
#define SPIM_SSLDR_SPNDL	(7u << SPIM_SSLDR_SPNDL_POS)
#define SPIM_SSLDR_SLNDL_POS	8
#define SPIM_SSLDR_SLNDL	(7u << SPIM_SSLDR_SLNDL_POS)
#define SPIM_SSLDR_SCKDL_POS	0
#define SPIM_SSLDR_SCKDL	(7u << SPIM_SSLDR_SCKDL_POS)

/* DRCR field */
#define SPIM_DRCR_SSLN_POS	24
#define SPIM_DRCR_SSLN		(1u << SPIM_DRCR_SSLN_POS)
#define SPIM_DRCR_RBURST_POS	16
#define SPIM_DRCR_RBURST	(31u << SPIM_DRCR_RBURST_POS)
#define SPIM_DRCR_RCF_POS	9
#define SPIM_DRCR_RCF		(1u << SPIM_DRCR_RCF_POS)
#define SPIM_DRCR_RBE_POS	8
#define SPIM_DRCR_RBE		(1u << SPIM_DRCR_RBE_POS)
#define SPIM_DRCR_SSLE_POS	0
#define SPIM_DRCR_SSLE		(1u << SPIM_DRCR_SSLE_POS)

/* DREAR field */
#define SPIM_DREAR_EAV_POS	16
#define SPIM_DREAR_EAV		(255u << SPIM_DREAR_EAV_POS)
#define SPIM_DREAR_EAC_POS	0
#define SPIM_DREAR_EAC		(7u << SPIM_DREAR_EAC_POS)

/* DRCMR field */
#define SPIM_DRCMR_CMD_POS	16
#define SPIM_DRCMR_CMD		(255u << SPIM_DRCMR_CMD_POS)
#define SPIM_DRCMR_OCMD_POS	0
#define SPIM_DRCMR_OCMD		(255u << SPIM_DRCMR_OCMD_POS)

/* DROPR field */
#define SPIM_DROPR_OPD3_POS	24
#define SPIM_DROPR_OPD3		(255u << SPIM_DROPR_OPD3_POS)
#define SPIM_DROPR_OPD2_POS	16
#define SPIM_DROPR_OPD2		(255u << SPIM_DROPR_OPD2_POS)
#define SPIM_DROPR_OPD1_POS	8
#define SPIM_DROPR_OPD1		(255u << SPIM_DROPR_OPD1_POS)
#define SPIM_DROPR_OPD0_POS	0
#define SPIM_DROPR_OPD0		(255u << SPIM_DROPR_OPD0_POS)

/* CMNSR field */
#define SPIM_CMNSR_SSLF_POS	1
#define SPIM_CMNSR_SSLF		(1u << SPIM_CMNSR_SSLF_POS)
#define SPIM_CMNSR_TEND_POS	0
#define SPIM_CMNSR_TEND		(1u << SPIM_CMNSR_TEND_POS)

/* DRDMCR field */
#define SPIM_DRDMCR_DMCYC_POS	0
#define SPIM_DRDMCR_DMCYC	(31u << SPIM_DRDMCR_DMCYC_POS)

/* DRENR field */
#define SPIM_DRENR_CDB_POS	30
#define SPIM_DRENR_CDB		(3u << SPIM_DRENR_CDB_POS)
#define SPIM_DRENR_OCDB_POS	28
#define SPIM_DRENR_OCDB		(3u << SPIM_DRENR_OCDB_POS)
#define SPIM_DRENR_ADB_POS	24
#define SPIM_DRENR_ADB		(3u << SPIM_DRENR_ADB_POS)
#define SPIM_DRENR_OPDB_POS	20
#define SPIM_DRENR_OPDB		(3u << SPIM_DRENR_OPDB_POS)
#define SPIM_DRENR_DRDB_POS	16
#define SPIM_DRENR_DRDB		(3u << SPIM_DRENR_DRDB_POS)
#define SPIM_DRENR_DB_4BIT	2u
#define SPIM_DRENR_DB_1BIT	0u
#define SPIM_DRENR_DME_POS	15
#define SPIM_DRENR_DME		(1u << SPIM_DRENR_DME_POS)
#define SPIM_DRENR_CDE_POS	14
#define SPIM_DRENR_CDE		(1u << SPIM_DRENR_CDE_POS)
#define SPIM_DRENR_OCDE_POS	12
#define SPIM_DRENR_OCDE		(1u << SPIM_DRENR_OCDE_POS)
#define SPIM_DRENR_ADE_POS	8
#define SPIM_DRENR_ADE		(15u << SPIM_DRENR_ADE_POS)
#define SPIM_DRENR_ADE_3BYTE	7u
#define SPIM_DRENR_ADE_4BYTE	15u
#define SPIM_DRENR_ADE_OPI	12u
#define SPIM_DRENR_ADE_HYPER	4u
#define SPIM_DRENR_ADE_NONE	0u
#define SPIM_DRENR_OPDE_POS	4
#define SPIM_DRENR_OPDE		(15u << SPIM_DRENR_OPDE_POS)
#define SPIM_DRENR_OPDE_NONE	0u
#define SPIM_DRENR_OPDE_1BYTE	8u
#define SPIM_DRENR_OPDE_2BYTE	12u
#define SPIM_DRENR_OPDE_3BYTE	14u
#define SPIM_DRENR_OPDE_4BYTE	15u

/* SMCR field */
#define SPIM_SMCR_SSLKP_POS	8
#define SPIM_SMCR_SSLKP		(1u << SPIM_SMCR_SSLKP_POS)
#define SPIM_SMCR_SPIRE_POS	2
#define SPIM_SMCR_SPIRE		(1u << SPIM_SMCR_SPIRE_POS)
#define SPIM_SMCR_SPIWE_POS	1
#define SPIM_SMCR_SPIWE		(1u << SPIM_SMCR_SPIWE_POS)
#define SPIM_SMCR_SPIE_POS	0
#define SPIM_SMCR_SPIE		(1u << SPIM_SMCR_SPIE_POS)

/* SMCMR field */
#define SPIM_SMCMR_CMD_POS	16
#define SPIM_SMCMR_CMD		(255u << SPIM_SMCMR_CMD_POS)
#define SPIM_SMCMR_OCMD_POS	0
#define SPIM_SMCMR_OCMD		(255u << SPIM_SMCMR_OCMD_POS)

/* SMOPR field */
#define SPIM_SMOPR_OPD3_POS	24
#define SPIM_SMOPR_OPD3		(255u << SPIM_SMOPR_OPD3_POS)
#define SPIM_SMOPR_OPD2_POS	16
#define SPIM_SMOPR_OPD2		(255u << SPIM_SMOPR_OPD2_POS)
#define SPIM_SMOPR_OPD1_POS	8
#define SPIM_SMOPR_OPD1		(255u << SPIM_SMOPR_OPD1_POS)
#define SPIM_SMOPR_OPD0_POS	0
#define SPIM_SMOPR_OPD0		(255u << SPIM_SMOPR_OPD0_POS)

/* SMENR field */
#define SPIM_SMENR_CDB_POS	30
#define SPIM_SMENR_CDB		(3u << SPIM_SMENR_CDB_POS)
#define SPIM_SMENR_OCDB_POS	28
#define SPIM_SMENR_OCDB		(3u << SPIM_SMENR_OCDB_POS)
#define SPIM_SMENR_ADB_POS	24
#define SPIM_SMENR_ADB		(3u << SPIM_SMENR_ADB_POS)
#define SPIM_SMENR_OPDB_POS	20
#define SPIM_SMENR_OPDB		(3u << SPIM_SMENR_OPDB_POS)
#define SPIM_SMENR_SPIDB_POS	16
#define SPIM_SMENR_SPIDB	(3u << SPIM_SMENR_SPIDB_POS)
#define SPIM_SMENR_DB_4BIT	2u
#define SPIM_SMENR_DB_1BIT	0u
#define SPIM_SMENR_DME_POS	15
#define SPIM_SMENR_DME		(1u << SPIM_SMENR_DME_POS)
#define SPIM_SMENR_CDE_POS	14
#define SPIM_SMENR_CDE		(1u << SPIM_SMENR_CDE_POS)
#define SPIM_SMENR_OCDE_POS	12
#define SPIM_SMENR_OCDE		(1u << SPIM_SMENR_OCDE_POS)
#define SPIM_SMENR_ADE_POS	8
#define SPIM_SMENR_ADE		(15u << SPIM_SMENR_ADE_POS)
#define SPIM_SMENR_ADE_3BYTE	7u
#define SPIM_SMENR_ADE_4BYTE	15u
#define SPIM_SMENR_ADE_OPI	12u
#define SPIM_SMENR_ADE_HYPER	4u
#define SPIM_SMENR_ADE_NONE	0u
#define SPIM_SMENR_OPDE_POS	4
#define SPIM_SMENR_OPDE		(15u << SPIM_SMENR_OPDE_POS)
#define SPIM_SMENR_OPDE_NONE	0u
#define SPIM_SMENR_OPDE_1BYTE	8u
#define SPIM_SMENR_OPDE_2BYTE	12u
#define SPIM_SMENR_OPDE_3BYTE	14u
#define SPIM_SMENR_OPDE_4BYTE	15u
#define SPIM_SMENR_SPIDE_POS	0
#define SPIM_SMENR_SPIDE	(15u << SPIM_SMENR_SPIDE_POS)
#define SPIM_SMENR_SPIDE_NONE	0u
#define SPIM_SMENR_SPIDE_BYTE	8u
#define SPIM_SMENR_SPIDE_WORD	12u
#define SPIM_SMENR_SPIDE_LONG	15u

/* DRDRENR field */
#define SPIM_DRDRENR_HYPE_POS	12
#define SPIM_DRDRENR_HYPE	(7u << SPIM_DRDRENR_HYPE_POS)
#define SPIM_DRDRENR_SPI	0u
#define SPIM_DRDRENR_DDR	5u
#define SPIM_DRDRENR_OCTADDR	4u
#define SPIM_DRDRENR_ADDRE_POS	8
#define SPIM_DRDRENR_ADDRE	(1u << SPIM_DRDRENR_ADDRE_POS)
#define SPIM_DRDRENR_OPDRE_POS	4
#define SPIM_DRDRENR_OPDRE	(1u << SPIM_DRDRENR_OPDRE_POS)
#define SPIM_DRDRENR_DRDRE_POS	0
#define SPIM_DRDRENR_DRDRE	(1u << SPIM_DRDRENR_DRDRE_POS)

/* SMDMCR field */
#define SPIM_SMDMCR_DMCYC_POS	0
#define SPIM_SMDMCR_DMCYC	(31u << SPIM_SMDMCR_DMCYC_POS)

/* SMDRENR field */
#define SPIM_SMDRENR_HYPE_POS	12
#define SPIM_SMDRENR_HYPE	(7u << SPIM_SMDRENR_HYPE_POS)
#define SPIM_SMDRENR_SPI	0u
#define SPIM_SMDRENR_DDR	5u
#define SPIM_SMDRENR_OCTADDR	4u
#define SPIM_SMDRENR_ADDRE_POS	8
#define SPIM_SMDRENR_ADDRE	(1u << SPIM_SMDRENR_ADDRE_POS)
#define SPIM_SMDRENR_OPDRE_POS	4
#define SPIM_SMDRENR_OPDRE	(1u << SPIM_SMDRENR_OPDRE_POS)
#define SPIM_SMDRENR_DRDRE_POS	0
#define SPIM_SMDRENR_DRDRE	(1u << SPIM_SMDRENR_DRDRE_POS)

/* PHYCNT field */
#define SPIM_PHYCNT_CAL_POS		31
#define SPIM_PHYCNT_CAL			(1u << SPIM_PHYCNT_CAL_POS)
#define SPIM_PHYCNT_ALT_ALIGN_POS	30
#define SPIM_PHYCNT_ALT_ALIGN		(1u << SPIM_PHYCNT_ALT_ALIGN_POS)
#define SPIM_PHYCNT_OCTA_POS		22
#define SPIM_PHYCNT_OCTA		(3u << SPIM_PHYCNT_OCTA_POS)
#define SPIM_PHYCNT_OCTA_DDR_ALT	1u
#define SPIM_PHYCNT_OCTA_DDR_SEQ	2u
#define SPIM_PHYCNT_EXDS_POS		21
#define SPIM_PHYCNT_EXDS		(1u << SPIM_PHYCNT_EXDS_POS)
#define SPIM_PHYCNT_OCT_POS		20
#define SPIM_PHYCNT_OCT			(1u << SPIM_PHYCNT_OCT_POS)
#define SPIM_PHYCNT_HS_POS		18
#define SPIM_PHYCNT_HS			(1u << SPIM_PHYCNT_HS_POS)
#define SPIM_PHYCNT_CKSEL_POS		16
#define SPIM_PHYCNT_CKSEL		(3u << SPIM_PHYCNT_CKSEL_POS)
#define SPIM_PHYCNT_WBUF2_POS		4
#define SPIM_PHYCNT_WBUF2		(1u << SPIM_PHYCNT_WBUF2_POS)
#define SPIM_PHYCNT_WBUF_POS		2
#define SPIM_PHYCNT_WBUF		(1u << SPIM_PHYCNT_WBUF_POS)
#define SPIM_PHYCNT_PHYMEM_POS		0
#define SPIM_PHYCNT_PHYMEM		(3u << SPIM_PHYCNT_PHYMEM_POS)
#define SPIM_PHYCNT_SDR			0u
#define SPIM_PHYCNT_DDR			1u
#define SPIM_PHYCNT_HYPER		3u

/* PHYOFFSET1 field */
#define SPIM_PHYOFFSET1_DDRTMG_POS	28
#define SPIM_PHYOFFSET1_DDRTMG		(7u << SPIM_PHYOFFSET1_DDRTMG_POS)
#define SPIM_PHYOFFSET1_DDR		2u
#define SPIM_PHYOFFSET1_SDR		3u

/* PHYOFFSET2 field */
#define SPIM_PHYOFFSET2_OCTTMG_POS	8
#define SPIM_PHYOFFSET2_OCTTMG		(7u << SPIM_PHYOFFSET2_OCTTMG_POS)
#define SPIM_PHYOFFSET2_SPI		4u
#define SPIM_PHYOFFSET2_HYPER		4u
#define SPIM_PHYOFFSET2_SPI_WBUF	0u
#define SPIM_PHYOFFSET2_OPI		3u




#endif	/* _SPIM_REG_H_ */

