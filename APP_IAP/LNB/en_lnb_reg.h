#ifndef EN_LNB_REG_H
#define EN_LNB_REG_H

/**
 * @brief Registers and its' bits definition relating to LNB
 * 
 * 
 * Content:
 * 
 * - LNB REGISTER ADDRESS
 * - LNB REGISTERS BITS DEFINITION
 * - LNB REGISTER DEFAULT VALUE (AFTER POWER ON RESET)
 * 
 */

/* ************************************************************************** */
/*                            LNB REGISTER ADDRESS                            */
/* ************************************************************************** */

/* ********* LNB Signal Calibration register SC (addr = 0x00 - 0x05) ******** */
#define EN_LNB_REG_SC_00 ((uint8_t)0x00U)
#define EN_LNB_REG_SC_01 ((uint8_t)0x01U)
#define EN_LNB_REG_SC_02 ((uint8_t)0x02U)
#define EN_LNB_REG_SC_03 ((uint8_t)0x03U)
#define EN_LNB_REG_SC_04 ((uint8_t)0x04U)
#define EN_LNB_REG_SC_05 ((uint8_t)0x05U)

/* ************ LNB LED Power Control Register LPC (addr = 0x06) ************ */
#define EN_LNB_REG_LED_PC_06 ((uint8_t)0x06U)

/* ******** LNB Output Configuration register OC (addr = 0x07 - 0x08) ******* */
#define EN_LNB_REG_OC_07 ((uint8_t)0x07U)
#define EN_LNB_REG_OC_08 ((uint8_t)0x08U)

/* ************** LNB Test Functions register TF (addr = 0x09) ************** */
#define EN_LNB_REG_TF_09 ((uint8_t)0x09U)

/* ************* LNB FlexCount register FC ( addr = 0x0A - 0x11) ************ */
#define EN_LNB_REG_FC_10 ((uint8_t)0x0AU)
#define EN_LNB_REG_FC_11 ((uint8_t)0x0BU)
#define EN_LNB_REG_FC_12 ((uint8_t)0x0CU)
#define EN_LNB_REG_FC_13 ((uint8_t)0x0DU)
#define EN_LNB_REG_FC_14 ((uint8_t)0x0EU)
#define EN_LNB_REG_FC_15 ((uint8_t)0x0FU)
#define EN_LNB_REG_FC_16 ((uint8_t)0x10U)
#define EN_LNB_REG_FC_17 ((uint8_t)0x11U)

/* ************* LNB Status(Read only) register ST (addr = 0x12) ************ */
#define EN_LNB_REG_ST_18 ((uint8_t)0x12U)

/* ************************************************************************** */
/*                        LNB REGISTERS BITS DEFINITION                       */
/* ************************************************************************** */

/* ****************** Bits Definition for LNB_SC1 register ****************** */
#define LNB_SC1_GS_Pos (0U)
#define LNB_SC1_GS_Msk (0x3FU << LNB_SC1_GS_Pos)
#define LNB_SC1_GS LNB_SC1_GS_Msk
#define LNB_SC1_NENSHIFT_Pos (6U)
#define LNB_SC1_NENSHIFT_Msk (0x01U << LNB_SC1_NENSHIFT_Pos)
#define LNB_SC1_NENSHIFT LNB_SC1_NENSHIFT_Msk

/* ****************** Bits Definition for LNB_SC2 register ****************** */
#define LNB_SC2_GC_Pos (0U)
#define LNB_SC2_GC_Msk (0x3FU << LNB_SC2_GC_Pos)
#define LNB_SC2_GC LNB_SC2_GC_Msk
#define LNB_SC2_LCMOD_Pos (6U)
#define LNB_SC2_LCMOD_Msk (0x01U << LNB_SC2_LCMOD_Pos)
#define LNB_SC2_LCMOD LNB_SC2_LCMOD_Msk

/* ****************** Bits Definition for LNB_SC3 register ****************** */
#define LNB_SC3_OSP_Pos (0U)
#define LNB_SC3_OSP_Msk (0x7FU << LNB_SC3_OSP_Pos)
#define LNB_SC3_OSP LNB_SC3_OSP_Msk

/* ****************** Bits Definition for LNB_SC4 register ****************** */
#define LNB_SC4_OSN_Pos (0U)
#define LNB_SC4_OSN_Msk (0x7FU << LNB_SC4_OSN_Pos)
#define LNB_SC4_OSN LNB_SC4_OSN_Msk

/* ****************** Bits Definition for LNB_SC5 register ****************** */
#define LNB_SC5_OCP_Pos (0U)
#define LNB_SC5_OCP_Msk (0x7FU << LNB_SC5_OCP_Pos)
#define LNB_SC5_OCP LNB_SC5_OCP_Msk

/* ****************** Bits Definition for LNB_SC6 register ****************** */
#define LNB_SC6_OCN_Pos (0U)
#define LNB_SC6_OCN_Msk (0x7FU << LNB_SC6_OCN_Pos)
#define LNB_SC6_OCN LNB_SC6_OCN_Msk

/* ****************** Bits Definition for LNB_LPC register ****************** */
#define LNB_LPC_LCSET_Pos (0U)
#define LNB_LPC_LCSET_Msk (0x3FU << LNB_LPC_LCSET_Pos)
#define LNB_LPC_LCSET LNB_LPC_LCSET_Msk
#define LNB_LPC_LCTYP_Pos (6U)
#define LNB_LPC_LCTYP_Msk (0x01U << LNB_LPC_LCTYP_Pos)
#define LNB_LPC_LCTYP LNB_LPC_LCTYP_Msk

/* ****************** Bits Definition for LNB_OC1 register ****************** */
#define LNB_OC1_GR_Pos (0U)
#define LNB_OC1_GR_Msk (0x03U << LNB_OC1_GR_Pos)
#define LNB_OC1_GR LNB_OC1_GR_Msk
#define LNB_OC1_OSZC_Pos (2U)
#define LNB_OC1_OSZC_Msk (0x03U << LNB_OC1_OSZC_Pos)
#define LNB_OC1_OSZC LNB_OC1_OSZC_Msk
#define LNB_OC1_EPG_Pos (4U)
#define LNB_OC1_EPG_Msk (0x01U << LNB_OC1_EPG_Pos)
#define LNB_OC1_EPG LNB_OC1_EPG_Msk
#define LNB_OC1_DIR_Pos (5U)
#define LNB_OC1_DIR_Msk (0x01U << LNB_OC1_DIR_Pos)
#define LNB_OC1_DIR LNB_OC1_DIR_Msk
#define LNB_OC1_NGRAY_Pos (6U)
#define LNB_OC1_NGRAY_Msk (0x01U << LNB_OC1_NGRAY_Pos)
#define LNB_OC1_NGRAY LNB_OC1_NGRAY_Msk

/* ****************** Bits Definition for LNB_OC2 register ****************** */
#define LNB_OC2_SRC_Pos (0U)
#define LNB_OC2_SRC_Msk (0x07U << LNB_OC2_SRC_Pos)
#define LNB_OC2_SRC LNB_OC2_SRC_Msk
#define LNB_OC2_RNF_Pos (3U)
#define LNB_OC2_RNF_Msk (0x01U << LNB_OC2_RNF_Pos)
#define LNB_OC2_RNF LNB_OC2_RNF_Msk
#define LNB_OC2_INC_Pos (4U)
#define LNB_OC2_INC_Msk (0x07U << LNB_OC2_INC_Pos)
#define LNB_OC2_INC LNB_OC2_INC_Msk

/* ****************** Bits Definition for LNB_TF register ****************** */
#define LNB_TF_TMUX_Pos (0U)
#define LNB_TF_TMUX_Msk (0x0FU << LNB_TF_TMUX_Pos)
#define LNB_TF_TMUX LNB_TF_TMUX_Msk
#define LNB_TF_TA_Pos (4U)
#define LNB_TF_TA_Msk (0x03U << LNB_TF_TA_Pos)
#define LNB_TF_TA LNB_TF_TA_Msk
#define LNB_TF_NENF_Pos (6U)
#define LNB_TF_NENF_Msk (0x01U << LNB_TF_NENF_Pos)
#define LNB_TF_NENF LNB_TF_NENF_Msk

/* ****************** Bits Definition for LNB_FC1 register ****************** */
#define LNB_FC1_HYS_Pos (0U)
#define LNB_FC1_HYS_Msk (0x07U << LNB_FC1_HYS_Pos)
#define LNB_FC1_HYS LNB_FC1_HYS_Msk

/* ****************** Bits Definition for LNB_FC2 register ****************** */
#define LNB_FC2_NENFLEX_Pos (1U)
#define LNB_FC2_NENFLEX_Msk (0x01U << LNB_FC2_NENFLEX_Pos)
#define LNB_FC2_NENFLEX LNB_FC2_NENFLEX_Msk
#define LNB_FC2_SELABS_Pos (2U)
#define LNB_FC2_SELABS_Msk (0x01U << LNB_FC2_SELABS_Pos)
#define LNB_FC2_SELABS LNB_FC2_SELABS_Msk
#define LNB_FC2_TRIABZ_Pos (3U)
#define LNB_FC2_TRIABZ_Msk (0x01U << LNB_FC2_TRIABZ_Pos)
#define LNB_FC2_TRIABZ LNB_FC2_TRIABZ_Msk
#define LNB_FC2_INVZ_Pos (4U)
#define LNB_FC2_INVZ_Msk (0x01U << LNB_FC2_INVZ_Pos)
#define LNB_FC2_INVZ LNB_FC2_INVZ_Msk
#define LNB_FC2_INVB_Pos (5U)
#define LNB_FC2_INVB_Msk (0x01U << LNB_FC2_INVB_Pos)
#define LNB_FC2_INVB LNB_FC2_INVB_Msk
#define LNB_FC2_INVA_Pos (6U)
#define LNB_FC2_INVA_Msk (0x01U << LNB_FC2_INVA_Pos)
#define LNB_FC2_INVA LNB_FC2_INVA_Msk

/* ****************** Bits Definition for LNB_FC3 register ****************** */
#define LNB_FC3_ZPOS_Pos (0U)
#define LNB_FC3_ZPOS_Msk (0x7FU << LNB_FC3_ZPOS_Pos)
#define LNB_FC3_ZPOS LNB_FC3_ZPOS_Msk

/* ****************** Bits Definition for LNB_FC4 register ****************** */
#define LNB_FC4_ZPOS_Pos (0U)
#define LNB_FC4_ZPOS_Msk (0x7FU << LNB_FC4_ZPOS_Pos)
#define LNB_FC4_ZPOS LNB_FC4_ZPOS_Msk

/* ****************** Bits Definition for LNB_FC5 register ****************** */
#define LNB_FC5_ZPOS_Pos (0U)
#define LNB_FC5_ZPOS_Msk (0x0FU << LNB_FC5_ZPOS_Pos)
#define LNB_FC5_ZPOS LNB_FC5_ZPOS_Msk
#define LNB_FC5_Z90_Pos (4U)
#define LNB_FC5_Z90_Msk (0x01U << LNB_FC5_Z90_Pos)
#define LNB_FC5_Z90 LNB_FC5_Z90_Msk
#define LNB_FC5_RESIPO_Pos (5U)
#define LNB_FC5_RESIPO_Msk (0x03U << LNB_FC5_RESIPO_Pos)
#define LNB_FC5_RESIPO LNB_FC5_RESIPO_Msk

/* ****************** Bits Definition for LNB_FC6 register ****************** */
#define LNB_FC6_RESSUB_Pos (0U)
#define LNB_FC6_RESSUB_Msk (0x7FU << LNB_FC6_RESSUB_Pos)
#define LNB_FC6_RESSUB LNB_FC6_RESSUB_Msk

/* ****************** Bits Definition for LNB_FC7 register ****************** */
#define LNB_FC7_RESSUB_Pos (0U)
#define LNB_FC7_RESSUB_Msk (0x7FU << LNB_FC7_RESSUB_Pos)
#define LNB_FC7_RESSUB LNB_FC7_RESSUB_Msk

/* ****************** Bits Definition for LNB_FC8 register ****************** */
#define LNB_FC8_RESSUB_Pos (0U)
#define LNB_FC8_RESSUB_Msk (0x0FU << LNB_FC8_RESSUB_Pos)
#define LNB_FC8_RESSUB LNB_FC8_RESSUB_Msk
#define LNB_FC8_ENIPO_Pos (4U)
#define LNB_FC8_ENIPO_Msk (0x01U << LNB_FC8_ENIPO_Pos)
#define LNB_FC8_ENIPO LNB_FC8_ENIPO_Msk
#define LNB_FC8_STOPFLEX_Pos (5U)
#define LNB_FC8_STOPFLEX_Msk (0x01U << LNB_FC8_STOPFLEX_Pos)
#define LNB_FC8_STOPFLEX LNB_FC8_STOPFLEX_Msk
#define LNB_FC8_NOUTLO_Pos (6U)
#define LNB_FC8_NOUTLO_Msk (0x01U << LNB_FC8_NOUTLO_Pos)
#define LNB_FC8_NOUTLO LNB_FC8_NOUTLO_Msk

/* ******************* Bits Definition for LNB_ST register ****************** */
#define LNB_ST_POSOK_Pos (0U)
#define LNB_ST_POSOK_Msk (0x01U << LNB_ST_POSOK_Pos)
#define LNB_ST_POSOK LNB_ST_POSOK_Msk
#define LNB_ST_ERRS_Pos (1U)
#define LNB_ST_ERRS_Msk (0x01U << LNB_ST_ERRS_Pos)
#define LNB_ST_ERRS LNB_ST_ERRS_Msk
#define LNB_ST_ERRP_Pos (2U)
#define LNB_ST_ERRP_Msk (0x01U << LNB_ST_ERRP_Pos)
#define LNB_ST_ERRP LNB_ST_ERRP_Msk
#define LNB_ST_CHIPVERSION_Pos (4U)
#define LNB_ST_CHIPVERSION_Msk (0x0FU << LNB_ST_CHIPVERSION_Pos)
#define LNB_ST_CHIPVERSION LNB_ST_CHIPVERSION_Msk

/* ************************************************************************** */
/*                         LNB REGISTER DEFAULT VALUE                         */
/* ************************************************************************** */

/**
 * @brief Data of internal registers of LNB after POR.
 * 
 * Register's bit numbering is in MSB first.
 */

/* *************************** Signal calibration *************************** */
//// 0x00  0xA0
//#define EN_LNB_REG_SC_00_DEFAULT ((uint8_t)0xA0)
//// 0x01  0xA0
//#define EN_LNB_REG_SC_01_DEFAULT ((uint8_t)0xA0)
//// 0x02  0xC0
//#define EN_LNB_REG_SC_02_DEFAULT ((uint8_t)0xC0)
//// 0x03  0xC0
//#define EN_LNB_REG_SC_03_DEFAULT ((uint8_t)0xC0)
//// 0x04  0xC0
//#define EN_LNB_REG_SC_04_DEFAULT ((uint8_t)0xC0)
//// 0x05  0xC0
//#define EN_LNB_REG_SC_05_DEFAULT ((uint8_t)0xC0)

// 0x00  0x80[GS=1.0]
#define EN_LNB_REG_SC_00_DEFAULT ((uint8_t)0x80)
// 0x01  0x80[GC=1.0]
#define EN_LNB_REG_SC_01_DEFAULT ((uint8_t)0x80)
// 0x02  0xC0
#define EN_LNB_REG_SC_02_DEFAULT ((uint8_t)0xC0)
// 0x03  0xC0
#define EN_LNB_REG_SC_03_DEFAULT ((uint8_t)0xC0)
// 0x04  0xC0
#define EN_LNB_REG_SC_04_DEFAULT ((uint8_t)0xC0)
// 0x05  0xC0
#define EN_LNB_REG_SC_05_DEFAULT ((uint8_t)0xC0)
/* **************************** LED Power Control *************************** */
// 0x06  0x60
#define EN_LNB_REG_LED_PC_06_DEFAULT ((uint8_t)0x60)

/* *************************** Output Configuation ************************** */
// 0x07  0x09
#define EN_LNB_REG_OC_07_DEFAULT ((uint8_t)0x09)
// 0x08  0x18
#define EN_LNB_REG_OC_08_DEFAULT ((uint8_t)0x18)

/* ***************************** Test Functions ***************************** */
// 0x09  0x00
#define EN_LNB_REG_TF_09_DEFAULT ((uint8_t)0x00)

/* ******************************** FlexCount ******************************* */
// 0x0A  0x00
#define EN_LNB_REG_FC_10_DEFAULT ((uint8_t)0x00)
// 0x0B  0x8E
#define EN_LNB_REG_FC_11_DEFAULT ((uint8_t)0x8E)
// 0x0C  0x00
#define EN_LNB_REG_FC_12_DEFAULT ((uint8_t)0x00)
// 0x0D  0x00
#define EN_LNB_REG_FC_13_DEFAULT ((uint8_t)0x00)
// 0x0E  0x00
#define EN_LNB_REG_FC_14_DEFAULT ((uint8_t)0x00)
// 0x0F  0x00
#define EN_LNB_REG_FC_15_DEFAULT ((uint8_t)0x00)
// 0x10  0x00
#define EN_LNB_REG_FC_16_DEFAULT ((uint8_t)0x00)
// 0x11  0xA0
#define EN_LNB_REG_FC_17_DEFAULT ((uint8_t)0xA0)

/* *************************** Status (Read Only) *************************** */
// 0x12 (Read only)

#endif
