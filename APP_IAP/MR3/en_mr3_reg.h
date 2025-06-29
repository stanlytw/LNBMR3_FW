/**
 * @file Registers and it's birs definition relating to MR3
 */
#ifndef EN_MR3_REG_H
#define EN_MR3_REG_H

/* ************************************************************************** */
/*                            MR3 REGISTER ADDRESS                            */
/* ************************************************************************** */

/* ******** MR3 Signal conditioning register SC (addr = 0x20 - 0x10) ******** */
#define EN_MR3_REG_SC1_02 ((uint8_t) 0x02U)
#define EN_MR3_REG_SC2_03 ((uint8_t) 0x03U)
#define EN_MR3_REG_SC3_04 ((uint8_t) 0x04U)
#define EN_MR3_REG_SC4_05 ((uint8_t) 0x05U)
#define EN_MR3_REG_SC5_06 ((uint8_t) 0x06U)
#define EN_MR3_REG_SC6_07 ((uint8_t) 0x07U)
#define EN_MR3_REG_SC7_08 ((uint8_t) 0x08U)
#define EN_MR3_REG_SC8_09 ((uint8_t) 0x09U)
#define EN_MR3_REG_SC9_0A ((uint8_t) 0x0AU)
#define EN_MR3_REG_SC10_0B ((uint8_t) 0x0BU)
#define EN_MR3_REG_SC11_0C ((uint8_t) 0x0CU)
#define EN_MR3_REG_SC12_0D ((uint8_t) 0x0DU)
#define EN_MR3_REG_SC13_0E ((uint8_t) 0x0EU)
#define EN_MR3_REG_SC14_0F ((uint8_t) 0x0FU)
#define EN_MR3_REG_SC15_10 ((uint8_t) 0x10U)

/* ************* MR3 Interface register IN (addr = 0x18 - 0x1A) ************* */
#define EN_MR3_REG_IN1_18 ((uint8_t) 0x18U)
#define EN_MR3_REG_IN2_19 ((uint8_t) 0x19U)
#define EN_MR3_REG_IN3_1A ((uint8_t) 0x1AU)

/* ****** MR3 Offset and Interpolator register OI (addr = 0x1B - 0x21) ****** */
#define EN_MR3_REG_OI1_1B ((uint8_t) 0x1BU)
#define EN_MR3_REG_OI2_1C ((uint8_t) 0x1CU)
#define EN_MR3_REG_OI3_1D ((uint8_t) 0x1DU)
#define EN_MR3_REG_OI4_1E ((uint8_t) 0x1EU)
#define EN_MR3_REG_OI5_1F ((uint8_t) 0x1FU)
#define EN_MR3_REG_OI6_20 ((uint8_t) 0x20U)
#define EN_MR3_REG_OI7_21 ((uint8_t) 0x21U)

/* ********** MR3 Mask register for error and warning: EMASK, WMASK ********* */
#define EN_MR3_REG_MEW1_22 ((uint8_t) 0x22U)
#define EN_MR3_REG_MEW2_23 ((uint8_t) 0x22U)

/* ************** MR3 Data Resolution register DR (addr = 0x24) ************* */
#define EN_MR3_REG_DR_24 ((uint8_t) 0x24U)

/* ********* MRˇ CRC configuration register CRC (addr = 0x27 - 0x29) ******** */

#define EN_MR3_REG_CRC1_27 ((uint8_t) 0x27U)
#define EN_MR3_REG_CRC2_28 ((uint8_t) 0x28U)
#define EN_MR3_REG_CRC3_29 ((uint8_t) 0x29U)

/* ********* MR3 Extended settings register ES (addr = 0x2A - 0x2D) ********* */
#define EN_MR3_REG_ES1_2A ((uint8_t) 0x2AU) /* fixed zeros */
#define EN_MR3_REG_ES2_2B ((uint8_t) 0x2BU)
#define EN_MR3_REG_ES3_2C ((uint8_t) 0x2CU)
#define EN_MR3_REG_ES4_2D ((uint8_t) 0x2DU)

/* ******* MR3 Status register / Command register SR, CR (addr = 0x60) ****** */
#define EN_MR3_REG_SR_60 ((uint8_t) 0x60U)
#define EN_MR3_REG_CR_60 ((uint8_t) 0x60U)

/* ********** MR3 Sign-of-life counter register SOLC (addr = 0x68) ********** */

#define EN_MR3_REG_SOLC_68 ((uint8_t) 0x68U)

/* ******************* MR3 Error register ER (addr = 0x69) ****************** */

#define EN_MR3_REG_ER_69 ((uint8_t) 0x69U)

/* ************************************************************************** */
/*                        MR3 REGISTER BITS DEFINITION                        */
/* ************************************************************************** */
/* ****************** Bits Definition for MR3_SC0register ***************** */

#define MR3_SC0_GFS_Pos (4U) /* GFS [3:0] */
#define MR3_SC0_GFS_Msk (0x0FU << MR3_SC0_GFS_Pos) 
#define MR3_SC0_GFS MR3_SC0_GFS_Msk

#define MR3_SC0_GR_Pos (0U) /* GR [2:0] */
#define MR3_SC0_GR_Msk (0x07U << MR3_SC0_GR_Pos) 
#define MR3_SC0_GR MR3_SC0_GR_Msk

/* ****************** Bits Definition for MR3_SC1 register ***************** */
#define MR3_SC1_GFS_Pos (0U) /* GFS [10:4] */
#define MR3_SC1_GFS_Msk (0x7FU << MR3_SC1_GFS_Pos) 
#define MR3_SC1_GFS MR3_SC1_GFS_Msk

/* ****************** Bits Definition for MR3_SC2 register ***************** */
#define MR3_SC2_GFC_Pos (0U) /* GFC [7:0] */
#define MR3_SC2_GFC_Msk (0xFFU << MR3_SC2_GFC_Pos) 
#define MR3_SC2_GFC MR3_SC3_GFC_Msk

/* ****************** Bits Definition for MR3_SC3 register ***************** */
#define MR3_SC3_GFC_Pos (0U) /* GFC [10:8] */
#define MR3_SC3_GFC_Msk (0x07U << MR3_SC3_GFC_Pos) 
#define MR3_SC3_GFC MR3_SC3_GFC_Msk

#define MR3_SC3_MPS_Pos (4U) /* MPS [3:0] */
#define MR3_SC3_MPS_Msk (0x0FU << MR3_SC3_MPS_Pos) 
#define MR3_SC3_MPS MR3_SC3_MPS_Msk

/* ****************** Bits Definition for MR3_SC4 register ***************** */
#define MR3_SC4_MPS_Pos (0U) /* MPS [9:4] */
#define MR3_SC4_MPS_Msk (0x3FU << MR3_SC4_MPS_Pos) 
#define MR3_SC4_MPS MR3_SC4_MPS_Msk


/* ****************** Bits Definition for MR3_SC5 register ***************** */
#define MR3_SC5_MPC_Pos (0U) /* MPC [7:0] */
#define MR3_SC5_MPC_Msk (0xFFU << MR3_SC5_MPC_Pos) 
#define MR3_SC5_MPC MR3_SC5_MPC_Msk

/* ****************** Bits Definition for MR3_SC6register ***************** */
#define MR3_SC6_MPC_Pos (0U) /* MPC [9:8] */
#define MR3_SC6_MPC_Msk (0x03U << MR3_SC6_MPC_Pos) 
#define MR3_SC6_MPC MR3_SC6_MPC_Msk

/* ****************** Bits Definition for MR3_SC12 register ***************** */
#define MR3_SC12_SELREF_Pos (6U) /* REFVOS [1:0] */
#define MR3_SC12_SELREF_Msk (0x03U << MR3_SC12_SELREF_Pos)
#define MR3_SC12_SELREF MR3_SC12_SELREF_Msk


/* ****************** Bits Definition for MR3_SC12 register ***************** */
#define MR3_SC12_UIN_Pos (0U)
#define MR3_SC12_UIN_Msk (0x01U << MR3_SC12_UIN_Pos)
#define MR3_SC12_UIN MR3_SC12_UIN_Msk

#define MR3_SC12_RIN_Pos (1U)
#define MR3_SC12_RIN_Msk (0x03U << MR3_SC12_RIN_Pos)
#define MR3_SC12_RIN MR3_SC12_RIN_Msk

#define MR3_SC12_TUIN_Pos (3U)
#define MR3_SC12_TUIN_Msk (0x03U << MR3_SC12_TUIN_Pos)
#define MR3_SC12_TUIN MR3_SC12_TUIN_Msk

#define MR3_SC12_DCPOS_Pos (6U)
#define MR3_SC12_DCPOS_Msk (0x01U << MR3_SC12_DCPOS_Pos)
#define MR3_SC12_DCPOS MR3_SC12_DCPOS_Msk

#define MR3_SC12_INMODE_Pos (7U)
#define MR3_SC12_INMODE_Msk (0x01U << MR3_SC12_INMODE_Pos)
#define MR3_SC12_INMODE MR3_SC12_INMODE_Msk

/* ****************** Bits Definition for MR3_SC11 register ***************** */
#define MR3_SC11_REFVOS_Pos (4U)
#define MR3_SC11_REFVOS_Msk (0x03U << MR3_SC11_REFVOS_Pos)
#define MR3_SC11_REFVOS MR3_SC11_REFVOS_Msk

/* ****************** Bits Definition for MR3_SC14 register ***************** */

#define MR3_SC14_MODE_Pos (0U)
#define MR3_SC14_MODE_Msk (0x03U << MR3_SC14_MODE_Pos) 
#define MR3_SC14_MODE MR3_SC14_MODE_Msk

#define MR3_SC14_CFGBIAS_Pos (4U)
#define MR3_SC14_CFGBIAS_Msk (0x0FU << MR3_SC14_CFGBIAS_Pos) 
#define MR3_SC14_CFGBIAS MR3_SC14_MODE_Msk

/* ****************** Bits Definition for MR3_OI1 register ****************** */

#define MR3_OI1_RESO_CC_Pos (0U)
#define MR3_OI1_RESO_CC_Msk (0x03U << MR3_OI1_RESO_CC_Pos) 
#define MR3_OI1_RESO_CC MR3_OI1_RESO_CC_Msk

#define MR3_OI1_DIR_Pos (2U)
#define MR3_OI1_DIR_Msk (0x01U << MR3_OI1_DIR_Pos) 
#define MR3_OI1_DIR MR3_OI1_DIR_Msk

#define MR3_OI1_OFFS_ST_Pos (6U) /* OFFS_ST [1:0] */
#define MR3_OI1_OFFS_ST_Msk (0x03U << MR3_OI1_OFFS_ST_Pos) 
#define MR3_OI1_OFFS_ST MR3_OI1_OFFS_ST_Msk

#define MR3_OI2_OFFS_ST_Pos (0U) /* OFFS_ST [9:2] */
#define MR3_OI2_OFFS_ST_Msk (0xFFU << MR3_OI2_OFFS_ST_Pos) 
#define MR3_OI2_OFFS_ST MR3_OI2_OFFS_ST_Msk

#define MR3_OI3_OFFS_ST_Pos (0U) /* OFFS_ST [17:10] */
#define MR3_OI3_OFFS_ST_Msk (0xFFU << MR3_OI3_OFFS_ST_Pos) 
#define MR3_OI3_OFFS_ST MR3_OI3_OFFS_ST_Msk

#define MR3_OI4_OFFS_ST_Pos (0U) /* OFFS_ST [26:18] */
#define MR3_OI4_OFFS_ST_Msk (0xFFU << MR3_OI4_OFFS_ST_Pos) 
#define MR3_OI4_OFFS_ST MR3_OI4_OFFS_ST_Msk

#define MR3_OI5_OFFS_MT_Pos (0U) /* OFFS_MT [7:0] */
#define MR3_OI5_OFFS_MT_Msk (0xFFU << MR3_OI5_OFFS_MT_Pos) 
#define MR3_OI5_OFFS_MT MR3_OI5_OFFS_MT_Msk

#define MR3_OI6_OFFS_MT_Pos (0U) /* OFFS_MT [15:8] */
#define MR3_OI6_OFFS_MT_Msk (0xFFU << MR3_OI6_OFFS_MT_Pos) 
#define MR3_OI6_OFFS_MT MR3_OI6_OFFS_MT_Msk

#define MR3_OI7_OFFS_MT_Pos (0U) /* OFFS_MT [23:16] */
#define MR3_OI7_OFFS_MT_Msk (0xFFU << MR3_OI7_OFFS_MT_Pos) 
#define MR3_OI7_OFFS_MT MR3_OI7_OFFS_MT_Msk

/* *********************** Bits Definition for MR3_IN1 ********************** */
#define MR3_IN1_CYC_ADI_Pos (2U)
#define MR3_IN1_CYC_ADI_Msk (0x01U << MR3_IN1_CYC_ADI_Pos)
#define MR3_IN1_CYC_ADI MR3_IN1_CYC_ADI_Msk

#define MR3_IN1_STP_ADI_Pos (3U)
#define MR3_IN1_STP_ADI_Msk (0x01U << MR3_IN1_STP_ADI_Pos)
#define MR3_IN1_STP_ADI MR3_IN1_STP_ADI_Msk

#define MR3_IN1_GET_ADI_Pos (4U)
#define MR3_IN1_GET_ADI_Msk (0x01U << MR3_IN1_GET_ADI_Pos)
#define MR3_IN1_GET_ADI MR3_IN1_GET_ADI_Msk

#define MR3_IN1_INTCFG_Pos (6U)
#define MR3_IN1_INTCFG_Msk (0x03U << MR3_IN1_INTCFG_Pos)
#define MR3_IN1_INTCFG MR3_IN1_INTCFG_Msk

/* *********************** Bits Definition for MR3_IN2 ********************** */

#define MR3_IN2_ACQMODE_Pos (6U)
#define MR3_IN2_ACQMODE_Msk (0x03U << MR3_IN2_ACQMODE_Pos)
#define MR3_IN2_ACQMODE MR3_IN2_ACQMODE_Msk

/* *********************** Bits Defiition for MR3_IN3 *********************** */

#define MR3_IN3_SSI_ADI_Pos (0U)
#define MR3_IN3_SSI_ADI_Msk (0x01U << MR3_IN3_SSI_ADI_Pos)
#define MR3_IN3_SSI_ADI MR3_IN3_SSI_ADI_Msk

#define MR3_IN3_SBL_ADI_Pos (1U)
#define MR3_IN3_SBL_ADI_Msk (0x03U << MR3_IN3_SBL_ADI_Pos)
#define MR3_IN3_SBL_ADI MR3_IN3_SBL_ADI_Msk

#define MR3_IN3_DL_ADI_Pos (3U)
#define MR3_IN3_DL_ADI_Msk (0x1FU << MR3_IN3_DL_ADI_Pos)
#define MR3_IN3_DL_ADI MR3_IN3_DL_ADI_Msk

/* ********************** Bits Definition for MR3_MEW1 ********************** */
/* TODO: */

/* ********************** Bits Definition for MR3_MEW2 ********************** */
/* TODO: */

/* *********************** Bits Definition for MR3_DR *********************** */

#define MR3_DR_RESO_ST_Pos (0U)
#define MR3_DR_RESO_ST_Msk (0x1FU << MR3_DR_RESO_ST_Pos)
#define MR3_DR_RESO_ST MR3_DR_RESO_ST_Msk

#define MR3_DR_RESO_MT_Pos (5U)
#define MR3_DR_RESO_MT_Msk (0x07U << MR3_DR_RESO_MT_Pos)
#define MR3_DR_RESO_MT MR3_DR_RESO_MT_Msk

/* ********************** Bits Definition for MR3_CRC1 ********************** */

#define MR3_CRC1_CHK_ADI_Pos (5U)
#define MR3_CRC1_CHK_ADI_Msk (0x01U << MR3_CRC1_CHK_ADI_Pos)
#define MR3_CRC1_CHK_ADI MR3_CRC1_CHK_ADI_Msk

#define MR3_CRC1_RES_ERR_Pos (6U)
#define MR3_CRC1_RES_ERR_Msk (0x01U << MR3_CRC1_RES_ERR_Pos)
#define MR3_CRC1_RES_ERR MR3_CRC1_RES_ERR_Msk

/* *********************** Bits Definition for MR3_ES2 *********************** */

#define MR3_ES2_EN_FAMP_Pos (0U)
#define MR3_ES2_EN_FAMP_Msk (0x01U << MR3_ES2_EN_FAMP_Pos)
#define MR3_ES2_EN_FAMP MR3_ES2_EN_FAMP_Msk

#define MR3_ES2_TO_FAMP_Pos (1U)
#define MR3_ES2_TO_FAMP_Msk (0x01U << MR3_ES2_TO_FAMP_Pos)
#define MR3_ES2_TO_FAMP MR3_ES2_TO_FAMP_Msk

#define MR3_ES2_THR_FAMP_Pos (2U)
#define MR3_ES2_THR_FAMP_Msk (0x03U << MR3_ES2_THR_FAMP_Pos)
#define MR3_ES2_THR_FAMP MR3_ES2_THR_FAMP_Msk

#define MR3_ES2_SLOW_ADI_Pos (7U)
#define MR3_ES2_SLOW_ADI_Msk (0x01U << MR3_ES2_SLOW_ADI_Pos)
#define MR3_ES2_SLOW_ADI MR3_ES2_SLOW_ADI_Msk

/* *********************** Bits Definition for MR3_ES3 ********************** */

#define MR3_ES3_SPO_ADI_Pos (3U)
#define MR3_ES3_SPO_ADI_Msk (0x0FU << MR3_ES3_SPO_ADI_Pos)
#define MR3_ES3_SPO_ADI MR3_ES3_SPO_ADI_Msk

/* *********************** Bits Definition for MR3_SR *********************** */

#define MR3_SR_PDV_Pos (0U)
#define MR3_SR_PDV_Msk (0x01U << MR3_SR_PDV_Pos)
#define MR3_SR_PDV MR3_SR_PDV_Msk

#define MR3_SR_ADV_Pos (1U)
#define MR3_SR_ADV_Msk (0x01U << MR3_SR_ADV_Pos)
#define MR3_SR_ADV MR3_SR_ADV_Msk

#define MR3_SR_BUSY_Pos (2U)
#define MR3_SR_BUSY_Msk (0x01U << MR3_SR_BUSY_Pos)
#define MR3_SR_BUSY MR3_SR_BUSY_Msk

#define MR3_SR_EWKL_Pos (3U)
#define MR3_SR_EWKL_Msk (0x01U << MR3_SR_EWKL_Pos)
#define MR3_SR_EWKL MR3_SR_EWKL_Msk

#define MR3_SR_EWKH_Pos (4U)
#define MR3_SR_EWKH_Msk (0x01U << MR3_SR_EWKH_Pos)
#define MR3_SR_EWKH MR3_SR_EWKH_Msk

#define MR3_SR_WARN_Pos (5U)
#define MR3_SR_WARN_Msk (0x01U << MR3_SR_WARN_Pos)
#define MR3_SR_WARN MR3_SR_WARN_Msk

#define MR3_SR_ERR_Pos (6U)
#define MR3_SR_ERR_Msk (0x01U << MR3_SR_ERR_Pos)
#define MR3_SR_ERR MR3_SR_ERR_Msk

#define MR3_SR_INIT_Pos (7U)
#define MR3_SR_INIT_Msk (0x01U << MR3_SR_INIT_Pos)
#define MR3_SR_INIT MR3_SR_INIT_Msk

/* *********************** Bits Definition for MR3_ER *********************** */

#define MR3_ER_ERR_RGL_Pos (0U)
#define MR3_ER_ERR_RGL_Msk (0x01U << MR3_ER_ERR_RGL_Pos)
#define MR3_ER_ERR_RGL MR3_ER_ERR_RGL_Msk

#define MR3_ER_ERR_AMP_Pos (1U)
#define MR3_ER_ERR_AMP_Msk (0x01U << MR3_ER_ERR_AMP_Pos)
#define MR3_ER_ERR_AMP MR3_ER_ERR_AMP_Msk

#define MR3_ER_ERR_TMP_Pos (2U)
#define MR3_ER_ERR_TMP_Msk (0x01U << MR3_ER_ERR_TMP_Pos)
#define MR3_ER_ERR_TMP MR3_ER_ERR_TMP_Msk

#define MR3_ER_ERR_SYNC_Pos (3U)
#define MR3_ER_ERR_SYNC_Msk (0x01U << MR3_ER_ERR_SYNC_Pos)
#define MR3_ER_ERR_SYNC MR3_ER_ERR_SYNC_Msk

#define MR3_ER_ERR_KNF_Pos (4U)
#define MR3_ER_ERR_KNF_Msk (0x01U << MR3_ER_ERR_KNF_Pos)
#define MR3_ER_ERR_KNF MR3_ER_ERR_KNF_Msk

#define MR3_ER_ERR_IPO_Pos (5U)
#define MR3_ER_ERR_IPO_Msk (0x01U << MR3_ER_ERR_IPO_Pos)
#define MR3_ER_ERR_IPO MR3_ER_ERR_IPO_Msk

#define MR3_ER_ERR_ABS_Pos (6U)
#define MR3_ER_ERR_ABS_Msk (0x01U << MR3_ER_ERR_ABS_Pos)
#define MR3_ER_ERR_ABS MR3_ER_ERR_ABS_Msk

#define MR3_ER_ERR_EXT_Pos (7U)
#define MR3_ER_ERR_EXT_Msk (0x01U << MR3_ER_ERR_EXT_Pos)
#define MR3_ER_ERR_EXT MR3_ER_ERR_EXT_Msk

/* ********************** Bits Definition for MR3_SOLC ********************** */

#define MR3_SOLC_LC_Pos (0U)
#define MR3_SOLC_LC_Msk (0xFFU << MR3_SOLC_LC_Pos)
#define MR3_SOLC_LC MR3_SOLC_LC_Msk

#endif
