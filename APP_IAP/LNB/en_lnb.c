#include "stm32f7xx_hal.h"

#include "en_lnb.h"
extern uint8_t LNBQC_OK;
uint32_t gray_to_binary(uint32_t gray);
LNB_TypeDef LNB = {
    .SC1 = EN_LNB_REG_SC_00_DEFAULT,
    .SC2 = EN_LNB_REG_SC_01_DEFAULT,
    .SC3 = EN_LNB_REG_SC_02_DEFAULT,
    .SC4 = EN_LNB_REG_SC_03_DEFAULT,
    .SC5 = EN_LNB_REG_SC_04_DEFAULT,
    .SC6 = EN_LNB_REG_SC_05_DEFAULT,
    .LPC = EN_LNB_REG_LED_PC_06_DEFAULT,
    .OC1 = EN_LNB_REG_OC_07_DEFAULT,
    .OC2 = EN_LNB_REG_OC_08_DEFAULT,
    .TF = EN_LNB_REG_TF_09_DEFAULT,
    .FC1 = EN_LNB_REG_FC_10_DEFAULT,
    .FC2 = EN_LNB_REG_FC_11_DEFAULT,
    .FC3 = EN_LNB_REG_FC_12_DEFAULT,
    .FC4 = EN_LNB_REG_FC_13_DEFAULT,
    .FC5 = EN_LNB_REG_FC_14_DEFAULT,
    .FC6 = EN_LNB_REG_FC_15_DEFAULT,
    .FC7 = EN_LNB_REG_FC_16_DEFAULT,
    .FC8 = EN_LNB_REG_FC_17_DEFAULT,
    .ST = 0x0U};

/**
 * @brief A minial subset of normal LNB Initialization process.
 * 
 * @note This is a NOT configurable Initialization process with FlexCount 
 *       Disable according to datasheet . The configurable options are selected
 *       just meet the project requirement.
 * 
 *       **Any configurations of the LNB object will be overwritten.**
 * 
 * @param hlnb pointer to a LNB_HandleTypeDef
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EN_LNB_Init_Minimal(LNB_HandleTypeDef *hlnb)
{
    /* ************************* reset the LNB Instance ************************* */

    hlnb->Instance->SC1 = EN_LNB_REG_SC_00_DEFAULT;
    hlnb->Instance->SC2 = EN_LNB_REG_SC_01_DEFAULT;
    hlnb->Instance->SC3 = EN_LNB_REG_SC_02_DEFAULT;
    hlnb->Instance->SC4 = EN_LNB_REG_SC_03_DEFAULT;
    hlnb->Instance->SC5 = EN_LNB_REG_SC_04_DEFAULT;
    hlnb->Instance->SC6 = EN_LNB_REG_SC_05_DEFAULT;
    hlnb->Instance->LPC = EN_LNB_REG_LED_PC_06_DEFAULT;
    hlnb->Instance->OC1 = EN_LNB_REG_OC_07_DEFAULT;
    hlnb->Instance->OC2 = EN_LNB_REG_OC_08_DEFAULT;
    hlnb->Instance->TF = EN_LNB_REG_TF_09_DEFAULT;
    hlnb->Instance->FC1 = EN_LNB_REG_FC_10_DEFAULT;
    hlnb->Instance->FC2 = EN_LNB_REG_FC_11_DEFAULT;
    hlnb->Instance->FC3 = EN_LNB_REG_FC_12_DEFAULT;
    hlnb->Instance->FC4 = EN_LNB_REG_FC_13_DEFAULT;
    hlnb->Instance->FC5 = EN_LNB_REG_FC_14_DEFAULT;
    hlnb->Instance->FC6 = EN_LNB_REG_FC_15_DEFAULT;
    hlnb->Instance->FC7 = EN_LNB_REG_FC_16_DEFAULT;
    hlnb->Instance->FC8 = EN_LNB_REG_FC_17_DEFAULT;
    hlnb->Instance->ST = 0x0U;

    /* ************************* set RESIPO, INC and SRC ************************ */

    /* RESIPO default 8-bit, set to 5-bit */
    LNB_WRITE_BIT(hlnb->Instance->FC5, LNB_FC5_RESIPO_Msk, EN_LNB_INTERPOLATOR_RESOLUTION_5BIT);

    /* INC default x2, unchanged*/

    /* SRC deault mode 0, set to mode 7 */
    /* length 16-bit, valid 12-bit for SPI */
    /* length 14-bit, valid 12-bit for SSI */
    LNB_WRITE_BIT(hlnb->Instance->OC2, LNB_OC2_SRC_Msk, EN_LNB_SSI_LENGTH_12BIT);

    /* **************************** set ENIPO, TRIABZ *************************** */

    /* ENIPO dafult Disable, set to Enable */
    LNB_WRITE_BIT(hlnb->Instance->FC8, LNB_FC8_ENIPO_Msk, EN_LNB_INTERPOLATOR_ENABLE);

    /* TRIABZ deault Disable, set ot Enable */
    LNB_WRITE_BIT(hlnb->Instance->FC2, LNB_FC2_TRIABZ_Msk, EN_LNB_ABZ_OUTPUT_TRISTATE_DISABLE);

    /**
     * @brief Figure 4. Typical Configuration Sequence, p18, LNB datasheet C1.
     * 
     * @note With FlecCount Disable
     */

    /* ************************* Write addr 0x00 - 0x09 ************************* */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 12;
    hlnb->pack.RxLength = 12;
    hlnb->pack.TxBuffer.data[0] = EN_LNB_REG_SC_00; /* addr */

    for (uint32_t i = 1; i < hlnb->pack.TxLength - 1; i++)
    {
        hlnb->pack.TxBuffer.data[i] = ((uint8_t *)hlnb->Instance)[i-1];
        EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[i]);
    }

    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* ***************************** Write addr 0x0E **************************** */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)0x0EU;
    hlnb->pack.TxBuffer.data[1] = ((uint8_t *)hlnb->Instance)[0x0EU];
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* ***************************** Write addr 0x11 **************************** */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)0x11U;
    hlnb->pack.TxBuffer.data[1] = ((uint8_t *)hlnb->Instance)[0x11U];
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* ***************************** Write addr 0x0B **************************** */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = EN_LNB_REG_FC_11;
    hlnb->pack.TxBuffer.data[1] = hlnb->Instance->FC2;
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* ******************************* Check POSOK ****************************** */

    uint32_t i = 0U;
    do
    {
        __EN_LNB_SPI_COMMAND_REGISTER_DATA(hlnb);

        hlnb->Instance->ST = hlnb->pack.RxBuffer.data[1];

        /* ***************************** Register Status **************************** */

        __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

        if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
        {
            return HAL_ERROR;
        }

        i++;
        if (i == 5)
            return HAL_ERROR;
        HAL_Delay(30U); /* 30ms */
    } while ((hlnb->Instance->ST & (uint8_t)0x01U) == 0U);

    /* ******************** activate Sensor data channel (PA) ******************* */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_ACTIVATE;
    hlnb->pack.TxLength = 2;
    hlnb->pack.RxLength = 2;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)(0x80U | 0x03U); /* PA and RA */

    __EN_LNB_SPI_TransmitReceive(hlnb);

    if ((hlnb->pack.RxBuffer.data[0] & ((uint8_t)(0x20U | 0x01U))) != ((uint8_t)(0x21U)))
    {
        /* activate Sensor data channel failed */
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief LNB position data comunication
 * @param hlnb pointer to LNB_HandleTypeDef
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EN_LNB_Position(LNB_HandleTypeDef *hlnb)
{
    /* TODO: add enable/disable following requests mechanism */

//    /* ******************** activate Sensor data channel (PA) ******************* */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_ACTIVATE;
    hlnb->pack.TxLength = 2;
    hlnb->pack.RxLength = 2;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)(0x80U | 0x03U); /* PA and RA */

    __EN_LNB_SPI_TransmitReceive(hlnb);

    if ((hlnb->pack.RxBuffer.data[0] & ((uint8_t)(0x20U | 0x01U))) == ((uint8_t)(0x21U)))
    {
        /* activate Sensor data channel failed */
        return HAL_ERROR;
    }

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* Position data comunication */

    do
    {
        /* SDAD status, command 1 */

        hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_POSITION_DATA_STATUS;
        hlnb->pack.TxLength = 2;
        hlnb->pack.RxLength = 2;
        hlnb->pack.TxBuffer.data[0] = (uint8_t)0x00U;

        __EN_LNB_SPI_TransmitReceive(hlnb);

        /* Disable following sensor data requests */

        /* SF == 1, Position data request failed */
        if (hlnb->pack.RxBuffer.data[0] & 0x40)
        {
            /* Enable following sensor data requests */

            goto error;
        }

    } while ((hlnb->pack.RxBuffer.data[0] & 0x80) == 0x00U); /* SV == 0, Position data invalid */

    /* SDAD transmission, command 2 */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_POSITION_READ;
    hlnb->pack.TxLength = 4;
    hlnb->pack.RxLength = 4;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)0x00U;
    hlnb->pack.TxBuffer.data[1] = (uint8_t)0x00U;
    hlnb->pack.TxBuffer.data[2] = (uint8_t)0x00U;

    __EN_LNB_SPI_TransmitReceive(hlnb);

//    /* FIXIT */
//    switch(((hlnb->pack.RxBuffer.data[0] | 0xC0U) ) | ((hlnb->Instance->AR << 6) | 0xC0U))
//    {
//        case 0x20000000U:
//        case 0xB0000000U:
//        case 0xE0000000U:
//        case 0x40000000U:
//            hlnb->Instance->MT += 1U;
//            break;
//        case 0x10000000U:
//        case 0x70000000U:
//        case 0xD0000000U:
//        case 0x80000000U:
//            hlnb->Instance->MT -= 1U;
//            break;
//        case 0x00000000U:
//        case 0x50000000U:
//        case 0xA0000000U:
//        case 0xF0000000U:
//            break; /* IDLE */
//        case 0x30000000U:
//        case 0x60000000U:
//        case 0x90000000U:
//        case 0xC0000000U:
//            /* Overrun Error */
//            break;
//        default:
//            break;
//    }

    hlnb->Instance->AR_b[0] = hlnb->pack.RxBuffer.data[0];
    hlnb->Instance->AR_b[1] = hlnb->pack.RxBuffer.data[1];
    hlnb->Instance->AR_b[2] = hlnb->pack.RxBuffer.data[2];

    hlnb->Instance->AR = (((((uint32_t)hlnb->pack.RxBuffer.data[0]) << 16) |
                         (((uint32_t)hlnb->pack.RxBuffer.data[1]) <<  8) |
                         ((uint32_t)hlnb->pack.RxBuffer.data[2]))>>6) | (0xAB000000U); /* 0xAB000000U is the data header */
		// (SD1|SD2|SD3)>>6, keep 18-bit(most)
    /*
    switch (hlnb->Init.SSILength)
    {
        case 0:
        case 1:
            hlnb->Instance->AR = (((uint32_t)hlnb->pack.RxBuffer.data[0]) << 16 |
                                  ((uint32_t)hlnb->pack.RxBuffer.data[1]) << 8  |
                                  ((uint32_t)hlnb->pack.RxBuffer.data[2]))
                                 >> (6U + hlnb->Init.SSILength);
            break;
        case 2:
        case 3:
        case 4:
            hlnb->Instance->AR = (((uint32_t)hlnb->pack.RxBuffer.data[0]) << 8 |
                                  ((uint32_t)hlnb->pack.RxBuffer.data[1]))
                                 >> (hlnb->Init.SSILength - 2U);
            break;
        case 5:
        case 6:
            hlnb->Instance->AR = (((uint32_t)hlnb->pack.RxBuffer.data[0]) << 8 |
                                  ((uint32_t)hlnb->pack.RxBuffer.data[1]))
                                 >> 3U;
            break;
        case 7:
            hlnb->Instance->AR = (((uint32_t)hlnb->pack.RxBuffer.data[0]) << 8 |
                                  ((uint32_t)hlnb->pack.RxBuffer.data[1]))
                                 >> 4U;
            break;
        default:
            hlnb->Instance->AR = 0x00000000U;
            
            return HAL_ERROR;
    }
    */
    /* 0x8070h = 0b1000000001110000*/

    /* Enable following sensor data requests */

    goto error;

    /* error handler*/

error:

    /* Register status or data */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    /* If error == 1 */
    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        /* error handler */

        hlnb->Instance->AR = (uint32_t)0x00U;
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief LNB Initialization process
 * @param hlnb pointer to a LNB_HandleTypeDef
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef EN_LNB_Init(LNB_HandleTypeDef *hlnb)
{
    if (hlnb == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * @brief Parallel Mode is not supported.
     */
    assert_param(IS_LNB_MODE(hlnb->Init.Mode));
    if (hlnb->Init.Mode == EN_LNB_MODE_INTERFACE)
    {
        LNB_WRITE_BIT(hlnb->Instance->OC1, LNB_OC1_EPG_Msk, hlnb->Init.Mode);
    }
    else
    {
        return HAL_ERROR; // PARALLEL MODE is NOT supported
    }

    /**
     * @brief Configure FlexCount and Interpolator.
     */
    assert_param(IS_LNB_FLEXCOUNT(hlnb->Init.FlexCount));
    if (hlnb->Init.FlexCount == EN_LNB_FLEXCOUNT_ENABLE)
    {
        return HAL_ERROR;
    }

    assert_param(IS_LNB_INTERPOLATOR(hlnb->Init.Interpolator));
    if (hlnb->Init.Interpolator == EN_LNB_INTERPOLATOR_DISABLE)
    {
        hlnb->Init.Interpolator = EN_LNB_INTERPOLATOR_ENABLE;
    }

    assert_param(IS_LNB_INTERPOLATOR_RESOLUTION(hlnb->Init.InterpolatorResolution));
    assert_param(IS_LNB_INTERPOLATOR_HYSTERESIS(hlnb->Init.InterpolatorHysteresis));
    assert_param(IS_LNB_INTERPOLATOR_FILTER(hlnb->Init.InterpolatorFilterDisable));

    LNB_WRITE_BIT(hlnb->Instance->FC5, LNB_FC5_RESIPO_Msk, hlnb->Init.InterpolatorResolution);
    LNB_WRITE_BIT(hlnb->Instance->FC1, LNB_FC1_HYS_Msk, hlnb->Init.InterpolatorHysteresis);
    LNB_WRITE_BIT(hlnb->Instance->TF, LNB_TF_NENF_Msk, hlnb->Init.InterpolatorFilterDisable);

    /**
     * @brief Shift Register
     */
    assert_param(IS_LNB_SSI(hlnb->Init.SSI));
    if (hlnb->Init.SSI == EN_LNB_SSI_ENABLE)
    {
        assert_param(IS_LNB_SSI_OUTPUT_DATA_FORMAT(hlnb->Init.SSIOutputDataFormat));
        assert_param(IS_LNB_SSI_IDLE_OUTPUT(hlnb->Init.SSIIdleOutput));
        assert_param(IS_LNB_SSI_LENGTH(hlnb->Init.SSILength));
        assert_param(IS_LNB_SSI_ROTATION_DIRECTION(hlnb->Init.SSIDirection));

        LNB_WRITE_BIT(hlnb->Instance->OC1, LNB_OC1_NGRAY_Msk, hlnb->Init.SSIOutputDataFormat);
        LNB_WRITE_BIT(hlnb->Instance->OC2, LNB_OC2_RNF_Msk, hlnb->Init.SSIIdleOutput);
        LNB_WRITE_BIT(hlnb->Instance->OC2, LNB_OC2_SRC_Msk, hlnb->Init.SSILength);
        LNB_WRITE_BIT(hlnb->Instance->OC1, LNB_OC1_DIR_Msk, hlnb->Init.SSIDirection);
    }
    else
    {
        return HAL_ERROR;
    }

    /**
     * @brief Signal Conditioning
     */
    assert_param(IS_LNB_DEFAULT_SIGNAL_CALIBRATION(hlnb->Init.DefaultSignalCalibration));
    if (hlnb->Init.DefaultSignalCalibration == EN_LNB_NO_DEFAULT_SIGNAL_CALIBRATION)
    {
        assert_param(IS_LNB_GAIN_RANGE(hlnb->Init.GainRange));
        assert_param(IS_LNB_SIN_GAIN(hlnb->Init.SinGain));
        assert_param(IS_LNB_POSITIVE_SIN_OFFSET(hlnb->Init.PositiveSinOffset));
        assert_param(IS_LNB_NEGATIVE_SIN_OFFSET(hlnb->Init.NegativeSinOffset));
        assert_param(IS_LNB_COS_GAIN(hlnb->Init.CosGain));
        assert_param(IS_LNB_POSITIVE_COS_OFFSET(hlnb->Init.PositiveCosOffset));
        assert_param(IS_LNB_NEGATIVE_COS_OFFSET(hlnb->Init.NegativeCosOffset));

        LNB_WRITE_BIT(hlnb->Instance->OC1, LNB_OC1_GR_Msk, hlnb->Init.GainRange);
        LNB_WRITE_BIT(hlnb->Instance->SC1, LNB_SC1_GS_Msk, hlnb->Init.SinGain);
        LNB_WRITE_BIT(hlnb->Instance->SC3, LNB_SC3_OSP_Msk, hlnb->Init.PositiveSinOffset);
        LNB_WRITE_BIT(hlnb->Instance->SC4, LNB_SC4_OSN_Msk, hlnb->Init.NegativeSinOffset);
        LNB_WRITE_BIT(hlnb->Instance->SC2, LNB_SC2_GC_Msk, hlnb->Init.CosGain);
        LNB_WRITE_BIT(hlnb->Instance->SC5, LNB_SC5_OCP_Msk, hlnb->Init.PositiveCosOffset);
        LNB_WRITE_BIT(hlnb->Instance->SC6, LNB_SC6_OCN_Msk, hlnb->Init.NegativeCosOffset);
    }

    /**
     * @brief LED Power Control
     */
    assert_param(IS_LNB_DEFAULT_LED_POWER_CONTROL(hlnb->Init.DefaultLEDPowerControl));
    if (hlnb->Init.DefaultLEDPowerControl == EN_LNB_NO_DEFAULT_LED_POWER_CONTROL)
    {
        assert_param(IS_LNB_LED_POWER_CONTROL_MODE(hlnb->Init.LEDPowerControlMode));
        assert_param(IS_LNB_LED_POWER_CONTROL_TYPE(hlnb->Init.LEDPowerControlType));
        assert_param(IS_LNB_LED_POWER_CONTROL_SETPOINT(hlnb->Init.LEDPowerControlSetpoint));

        LNB_WRITE_BIT(hlnb->Instance->SC2, LNB_SC2_LCMOD_Msk, hlnb->Init.LEDPowerControlMode);
        LNB_WRITE_BIT(hlnb->Instance->LPC, LNB_LPC_LCTYP_Msk, hlnb->Init.LEDPowerControlType);
        LNB_WRITE_BIT(hlnb->Instance->LPC, LNB_LPC_LCSET_Msk, hlnb->Init.LEDPowerControlSetpoint);
    }

    /**
     * @brief Test Function
     */
    assert_param(IS_LNB_TEST_MODE(hlnb->Init.TestMode));
    if (hlnb->Init.TestMode == EN_LNB_TEST_MODE_ENABLE)
    {
        return HAL_ERROR;
    }

    /* **************************** set ENIPO, TRIABZ *************************** */

    /* ENIPO dafult Disable, set to Enable */
    LNB_WRITE_BIT(hlnb->Instance->FC8, LNB_FC8_ENIPO_Msk, EN_LNB_INTERPOLATOR_ENABLE);

    /* TRIABZ deault Disable, set ot Enable */
    LNB_WRITE_BIT(hlnb->Instance->FC2, LNB_FC2_TRIABZ_Msk, EN_LNB_ABZ_OUTPUT_TRISTATE_DISABLE);

    /**
     * @brief Figure 4. Typical Configuration Sequence, p18, LNB datasheet C1.
     * 
     * @note With FlecCount Disable
     */

    /* ************************* Write addr 0x00 - 0x09 ************************* */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 12;
    hlnb->pack.RxLength = 12;
    hlnb->pack.TxBuffer.data[0] = EN_LNB_REG_SC_00; /* addr */

    for (uint32_t i = 1; i < hlnb->pack.TxLength - 1; i++)
    {
        hlnb->pack.TxBuffer.data[i] = ((uint8_t *)hlnb->Instance)[i-1];
        EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[i]);
    }

    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* ***************************** Write addr 0x0E **************************** */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)0x0EU;
    hlnb->pack.TxBuffer.data[1] = ((uint8_t *)hlnb->Instance)[0x0EU];
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* ***************************** Write addr 0x11 **************************** */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)0x11U;
    hlnb->pack.TxBuffer.data[1] = ((uint8_t *)hlnb->Instance)[0x11U];
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* ***************************** Write addr 0x0B **************************** */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = EN_LNB_REG_FC_11;
    hlnb->pack.TxBuffer.data[1] = hlnb->Instance->FC2;
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ***************************** Register Status **************************** */

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
    {
        return HAL_ERROR;
    }

    /* ******************************* Check POSOK ****************************** */

    uint32_t i = 0U;
    do
    {
        __EN_LNB_SPI_COMMAND_REGISTER_DATA(hlnb);

        hlnb->Instance->ST = hlnb->pack.RxBuffer.data[1];

        /* ***************************** Register Status **************************** */

        __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

        if (hlnb->pack.RxBuffer.data[0] & 0x8EU)
        {
            return HAL_ERROR;
        }

        i++;
        if (i == 5)
            return HAL_ERROR;
        HAL_Delay(30U); /* 30ms */
    } while ((hlnb->Instance->ST & (uint8_t)0x01U) == 0U);
		
		
		
		/* **Tunning LED Power to fit Sin/Cos Vpp=1V:Write addr 0x06 ******** */

    //1. Set LCTYP = 0, squared control mode
    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)0x06U;
    hlnb->pack.TxBuffer.data[1] = (uint8_t)0x00U;
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    //2. Set LED Power control set point
    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)0x06U;
    hlnb->pack.TxBuffer.data[1] = (uint8_t)0x3FU;
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

//    //1. Set LCTYP = 1, sum control mode
//    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
//    hlnb->pack.TxLength = 3;
//    hlnb->pack.RxLength = 3;
//    hlnb->pack.TxBuffer.data[0] = (uint8_t)0x06U;
//    hlnb->pack.TxBuffer.data[1] = (uint8_t)0x40U;
//    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

//    __EN_LNB_SPI_TransmitReceive(hlnb);

    /* ******************** activate Sensor data channel (PA) ******************* */

    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_ACTIVATE;
    hlnb->pack.TxLength = 2;
    hlnb->pack.RxLength = 2;
    hlnb->pack.TxBuffer.data[0] = (uint8_t)(0x80U | 0x03U); /* PA and RA */

    __EN_LNB_SPI_TransmitReceive(hlnb);

    if ((hlnb->pack.RxBuffer.data[0] & ((uint8_t)(0x20U | 0x01U))) != ((uint8_t)(0x21U)))
    {
        /* activate Sensor data channel failed */
        return HAL_ERROR;
    }
		//LNBQC_OK = 1;
    return HAL_OK;
}

/**
 * @brief Prepare for reconfiguration of LNB
 * @param hlnb pointor to a LNB_HandleTypeDef
 * @retval None
 * @warning NOT TESTED
 */
HAL_StatusTypeDef EN_LNB_DeInit(LNB_HandleTypeDef *hlnb)
{
    /* disable interpolator */
    LNB_WRITE_BIT(hlnb->Instance->FC8, LNB_FC8_ENIPO_Msk, EN_LNB_INTERPOLATOR_DISABLE);

    /* switch AZ interface to tri-state */
    LNB_WRITE_BIT(hlnb->Instance->FC2, LNB_FC2_TRIABZ_Msk, EN_LNB_ABZ_OUTPUT_TRISTATE_ENABLE);

    /* disable interpolator, spi command */
    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = EN_LNB_REG_FC_17;
    hlnb->pack.TxBuffer.data[1] = hlnb->Instance->FC8;
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[1] & 0x8E)
    {
        return HAL_ERROR;
    }

    /* switch ABZ interface to tri-state, spi command */
    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = EN_LNB_REG_FC_11;
    hlnb->pack.TxBuffer.data[1] = hlnb->Instance->FC2;
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);
    
    __EN_LNB_SPI_TransmitReceive(hlnb);

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[1] & 0x8E)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Caluculate multit-turn data
 * @param hlnb pointor to a LNB_HandleTypeDef
 * @retval None
 * @warning NOT TESTED
 */
HAL_StatusTypeDef EN_LNB_Enable(LNB_HandleTypeDef *hlnb)
{
    /* enable interpolator */
    LNB_WRITE_BIT(hlnb->Instance->FC8, LNB_FC8_ENIPO_Msk, EN_LNB_INTERPOLATOR_ENABLE);

    /* switch ABZ interface to normal */
    LNB_WRITE_BIT(hlnb->Instance->FC2, LNB_FC2_TRIABZ_Msk, EN_LNB_ABZ_OUTPUT_TRISTATE_DISABLE);

    /* enable interpolator, spi command */
    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = EN_LNB_REG_FC_17;
    hlnb->pack.TxBuffer.data[1] = hlnb->Instance->FC8;
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);

    __EN_LNB_SPI_TransmitReceive(hlnb);

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[1] & 0x8E)
    {
        return HAL_ERROR;
    }

    /* switch AZ interface to normal, spi command */
    hlnb->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;
    hlnb->pack.TxLength = 3;
    hlnb->pack.RxLength = 3;
    hlnb->pack.TxBuffer.data[0] = EN_LNB_REG_FC_11;
    hlnb->pack.TxBuffer.data[1] = hlnb->Instance->FC2;
    EVEN_PARITY_MSB(hlnb->pack.TxBuffer.data[1]);
    
    __EN_LNB_SPI_TransmitReceive(hlnb);

    __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb);

    if (hlnb->pack.RxBuffer.data[1] & 0x8E)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Caluculate multit-turn data
 * @param hlnb pointor to a LNB_HandleTypeDef
 * @retval None
 * @warning NOT TESTED
 */
HAL_StatusTypeDef EN_LNB_CycleCount(LNB_HandleTypeDef *hlnb)
{    
    // /* read INCA, and INCB */
    // static uint32_t INC_decode_past = 0;
    // static uint32_t INC_decode_cur = 0;
    // static uint32_t INC_CycleCounter = 0;
    // if (HAL_GPIO_ReadPin(INCA_GPIO_Port, INCA_Pin) == GPIO_PIN_RESET)
    //     INC_decode_cur = 0U;
    // else
    //     INC_decode_cur = 2U;
    // if (HAL_GPIO_ReadPin(INCB_GPIO_Port, INCB_Pin) == GPIO_PIN_SET)
    //     INC_decode_cur += 1U;

    // switch(INC_decode_past | INC_decode_cur)
    // {
    //     case 0x00000002U:
    //     case 0x0000000BU:
    //     case 0x0000000EU:
    //     case 0x00000004U:
    //         INC_CycleCounter += 1U;
    //         break;
    //     case 0x00000001U:
    //     case 0x00000007U:
    //     case 0x0000000DU:
    //     case 0x00000008U:
    //         INC_CycleCounter -= 1U;
    //         break;
    //     case 0x00000000U:
    //     case 0x00000005U:
    //     case 0x0000000AU:
    //     case 0x0000000FU:
    //         break;
    //     case 0x00000003U:
    //     case 0x00000006U:
    //     case 0x00000009U:
    //     case 0x0000000CU:
    //         /* Error */
    //         break;
    //     default:
    //         break;
    // }
  return HAL_OK;
}

/**
 * @brief even parity on MSB
 * 
 * @param n MSB of n should be 0.
 * @retval n even parity on MSB ex: 0x80U or 0x00U
 */
uint8_t even_parity(uint8_t n)
{
    n ^= (n << 4);
    n ^= (n << 2);
    n ^= (n << 1);
    return n & ((uint8_t)0x80U);
}

uint32_t gray_to_binary(uint32_t gray)
{
    gray = gray ^ (gray >> 16);
    gray = gray ^ (gray >> 8);
    gray = gray ^ (gray >> 4);
    gray = gray ^ (gray >> 2);
    gray = gray ^ (gray >> 1);
    return gray;
}

