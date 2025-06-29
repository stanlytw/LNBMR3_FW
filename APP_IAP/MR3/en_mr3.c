/**
 * @file en_mr3.c
 */
//#define CYC_READ_WITH_PREREQUEST
// Include

#include "en_mr3_reg.h"
#include "en_mr3.h"
#include "main.h"
#include "stm32f7xx_hal_def.h"
#include "stm32f7xx_hal.h"

MR3_TypeDef MR3;

HAL_StatusTypeDef EN_MR3_Init_Minimal(MR3_HandleTypeDef *hmr3)
{
    if (hmr3 == NULL)
    {
        return HAL_ERROR;
    }

    hmr3->ErrorCode = EN_MR3_ERROR_NONE;

    if (hmr3->State == MR3_STATE_RESET)
    {
        hmr3->Lock = HAL_UNLOCKED;
    }
    
    HAL_Delay(1);

    /* clear hmr3->Init */
    {
        MR3_InitTypeDef tmp = {0};
        hmr3->Init = tmp;
    }

    {
        MR3_TypeDef tmp = {0};
        *hmr3->Instance = tmp;
    }

    /* Options of OPERATING MODES */
    // hmr3->Init.Mode = EN_MR3_MODE_NORMAL;

    /* Options of BIAS CURRENT SOURCE */
    // hmr3->Init.BiasCurrentCalibration = EN_MR3_BIAS_CURRENT_100_PERCENT;
    
    /* Options od SIGNAL CONDITIONING */
    // hmr3->Init.AnalogInputConnectionMode = EN_MR3_ANALOG_INPUT_CONNECTION_MODE_DIFFERENTIAL;
    hmr3->Init.AnalogInputElectricalMode = EN_MR3_ANALOG_INPUT_ELECTRICAL_MODE_VOLTAGE;
    // hmr3->Init.AnalogInputResistanceType = // unused (for current mode)
    hmr3->Init.AnalogInputVoltageDividerType = EN_MR3_ANALOG_INPUT_VOLTAGE_DIVIDER_TYPE_1;
    // hmr3->Init.AnalogInputPolarityReferenceType = EN_MR3_ANALOG_INPUT_VOLTAGE_CURRENT_POLARITY_REFERENCE_TYPE_0;
    //hmr3->Init.AnalogInputReferenceType = EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_0;
    hmr3->Init.AnalogInputOffsetReferenceSourceType = EN_MR3_ANALOG_INPUT_OFFSET_REF_SOURCE_TYPE_V025;
		//hmr3->Init.AnalogInputCoarseGain = 0x00;//Factor:6.0
		hmr3->Init.AnalogInputFineGainSine = 0x0080;//0x03E0;
		hmr3->Init.AnalogInputFineGainCosine = 0x0000;//0x03E0;
		hmr3->Init.AnalogInputCenterPotentialSine = 0x0180;
		hmr3->Init.AnalogInputCenterPotentialCosine = 0x0200;
    /* Options of INTERPOLATION AND CYCLE COUNTING */
    hmr3->Init.CycleCounterLength = EN_MR3_CYCLE_COUNTER_LENGTH_34BIT;
    // hmr3->Init.PositionDataAdjustmentDirection = EN_MR3_POSITION_DATA_ADJUST_DIR_NORMAL;
    hmr3->Init.PositionDataAdjustmentDirection = EN_MR3_POSITION_DATA_ADJUST_DIR_NORMAL;
    // hmr3->Init.PositionDataSingleTurnOffset = 0x00000000U;
    // hmr3->Init.PositionDataMultiTurnOffset = 0x00000000U;
    hmr3->Init.PositionDataMultiTurnResolution = EN_MR3_SPI_MULTITURN_RESOLUTION_16BIT;
    hmr3->Init.PositionDataSingleTurnResolution = EN_MR3_SPI_SINGLE_TURN_RESOlUTION_23BIT;
    hmr3->Init.PositionDataAquisitionMode = EN_MR3_POSITION_DATA_AQUISITION_MODE_NORMAL;

//    /* Options of ADI */
//    hmr3->Init.ADIProtocol = EN_MR3_ADI_PROTO_SSI;
//    hmr3->Init.ADIClockSpeed = EN_MR3_ADI_CLOCK_RATE_SLOW;
//    hmr3->Init.ADIDataLength = 0x0DU << MR3_IN3_DL_ADI_Pos;/*10 + 3-bit for sync*/
//    hmr3->Init.ADISyncBitLength = EN_MR3_ADI_SYNC_BIT_LENGTH_3;
//    hmr3->Init.ADIPositionDataAdjustment = EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_0;
//    hmr3->Init.ADIStartMode = EN_MR3_ADI_START_MODE_WITHOUT_ABS_DATA;
//    hmr3->Init.ADICyclicRead = EN_MR3_ADI_CYCLIC_ABS_READ_DISABLE;
//    hmr3->Init.ADICyclicCheck = EN_MR3_ADI_CYCLIC_CHECK_ABS_DATA_DISABLE;
    // hmr3->Init.ADIInterface = EN_MR3_ADI_INTERFACE_NORMAL; /* interface daisy chain */

    /* Options of Monitoring and Safty Features */
    // hmr3->Init.ErrorRegisterResetAction = EN_MR3_ERROR_REGISTER_RESET_ACTION_READ_ERR_REGISTER;
    // hmr3->Init.SignalErrorFiltered = EN_MR3_SIGNAL_ERROR_FILTERED_DISABLE;
    // hmr3->Init.SignalErrorFilterTimeout = EN_MR3_SIGNAL_ERROR_FILTER_TIMEOUT_273us;
    // hmr3->Init.SignalErrorFilterThreshold = EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_17us;


    /* ## write options to virtual object */
    /* Options of OPERATING MODES */
    /* Options of BIAS CURRENT SOURCE */
    /* Options od SIGNAL CONDITIONING */
		/*GFS[3:0] 4-bit@ SC0*/
		//MR3_WRITE_BIT(hmr3->Instance->SC0, MR3_SC0_GR_Msk, (uint8_t)((hmr3->Init.AnalogInputCoarseGain & 0x0007)));
		/*GFS[3:0] 4-bit@ SC0*/
		MR3_WRITE_BIT(hmr3->Instance->SC0, MR3_SC0_GFS_Msk, (uint8_t)((hmr3->Init.AnalogInputFineGainSine & 0x000F)));
		/*GFS[10:4] 7-bit@ SC1*/
		MR3_WRITE_BIT(hmr3->Instance->SC1, MR3_SC1_GFS_Msk, (uint8_t)((hmr3->Init.AnalogInputFineGainSine>>4 & 0x007F)));
		/*GFC[7:0] 8-bit@ SC2*/
		MR3_WRITE_BIT(hmr3->Instance->SC2, MR3_SC2_GFC_Msk, (uint8_t)((hmr3->Init.AnalogInputFineGainCosine & 0x00FF)));
		/*GFC[10:8] 3-bit@ SC3*/
		MR3_WRITE_BIT(hmr3->Instance->SC3, MR3_SC3_GFC_Msk, (uint8_t)((hmr3->Init.AnalogInputFineGainCosine>>8 & 0x0007)));
    
		/*MPS[3:0] 4-bit@ SC3*/
		MR3_WRITE_BIT(hmr3->Instance->SC3, MR3_SC3_MPS_Msk, (uint8_t)((hmr3->Init.AnalogInputCenterPotentialSine & 0x000F)));
		/*MPS[9:4] 6-bit@ SC4*/
		MR3_WRITE_BIT(hmr3->Instance->SC4, MR3_SC4_MPS_Msk, (uint8_t)((hmr3->Init.AnalogInputCenterPotentialSine>>4 & 0x003F)));
		/*MPC[7:0] 8-bit@ SC5*/
		MR3_WRITE_BIT(hmr3->Instance->SC5, MR3_SC5_MPC_Msk, (uint8_t)((hmr3->Init.AnalogInputCenterPotentialCosine & 0x00FF)));
		/*MPC[10:8] 2-bit@ SC6*/
		MR3_WRITE_BIT(hmr3->Instance->SC6, MR3_SC6_MPC_Msk, (uint8_t)((hmr3->Init.AnalogInputCenterPotentialCosine>>8 & 0x0003)));
		
		MR3_WRITE_BIT(hmr3->Instance->SC12, MR3_SC12_UIN_Msk, hmr3->Init.AnalogInputElectricalMode);
		MR3_WRITE_BIT(hmr3->Instance->SC11, MR3_SC11_REFVOS_Msk, hmr3->Init.AnalogInputOffsetReferenceSourceType);
    
		/* Options of INTERPOLATION AND CYCLE COUNTING */
    MR3_WRITE_BIT(hmr3->Instance->OI1, MR3_OI1_RESO_CC_Msk, hmr3->Init.CycleCounterLength);
    MR3_WRITE_BIT(hmr3->Instance->OI1, MR3_OI1_DIR_Msk, hmr3->Init.PositionDataAdjustmentDirection);
    MR3_WRITE_BIT(hmr3->Instance->DR, MR3_DR_RESO_MT_Msk, hmr3->Init.PositionDataMultiTurnResolution);
    MR3_WRITE_BIT(hmr3->Instance->DR, MR3_DR_RESO_ST_Msk, hmr3->Init.PositionDataSingleTurnResolution);    
    MR3_WRITE_BIT(hmr3->Instance->IN2, MR3_IN2_ACQMODE_Msk, hmr3->Init.PositionDataAquisitionMode);

//    /* Options of ADI */
//    MR3_WRITE_BIT(hmr3->Instance->IN3, MR3_IN3_SSI_ADI_Msk, hmr3->Init.ADIProtocol);
//    MR3_WRITE_BIT(hmr3->Instance->IN3, MR3_IN3_DL_ADI_Msk, hmr3->Init.ADIDataLength);
//    MR3_WRITE_BIT(hmr3->Instance->IN3, MR3_IN3_SBL_ADI_Msk, hmr3->Init.ADISyncBitLength);
//    
//    MR3_WRITE_BIT(hmr3->Instance->IN1, MR3_IN1_STP_ADI_Msk, hmr3->Init.ADIStartMode);
//    MR3_WRITE_BIT(hmr3->Instance->IN1, MR3_IN1_CYC_ADI_Msk, hmr3->Init.ADICyclicRead);
//    
//    MR3_WRITE_BIT(hmr3->Instance->CRC1, MR3_CRC1_CHK_ADI_Msk, hmr3->Init.ADICyclicCheck);
//    
//    MR3_WRITE_BIT(hmr3->Instance->ES2, MR3_ES2_SLOW_ADI_Msk, hmr3->Init.ADIClockSpeed);
//    MR3_WRITE_BIT(hmr3->Instance->ES3, MR3_ES3_SPO_ADI_Msk, hmr3->Init.ADIPositionDataAdjustment);
    
    /* Options of Monitoring and Safty Features */
    MR3_WRITE_BIT(hmr3->Instance->ES2, MR3_ES2_EN_FAMP_Msk, hmr3->Init.SignalErrorFiltered);

    /* ## write options to MR3 through SPI */
    /* Options of OPERATING MODES */
    /* Options of BIAS CURRENT SOURCE */
    /* Options od SIGNAL CONDITIONING */
		
		/* Setting FineGainSine and FineGainCosine*/
		__EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC1_02, hmr3->Instance->SC1);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
		
		__EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC2_03, hmr3->Instance->SC2);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
		
		__EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC3_04, hmr3->Instance->SC3);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
		
		__EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC4_05, hmr3->Instance->SC4);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
		
		
		__EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC5_06, hmr3->Instance->SC5);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
		
			__EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC6_07, hmr3->Instance->SC6);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
		
		
		 __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC11_0C, hmr3->Instance->SC11);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
		
		
    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC12_0D, hmr3->Instance->SC12);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }

    /* Options of INTERPOLATION AND CYCLE COUNTING */
    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI1_1B, hmr3->Instance->OI1);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }

    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_DR_24, hmr3->Instance->DR);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }

    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_IN2_19, hmr3->Instance->IN2);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }

//    /* Options of ADI */
//    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_IN3_1A, hmr3->Instance->IN3);
//    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
//    {
//        return HAL_ERROR;
//    }
//    
//    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_IN1_18, hmr3->Instance->IN1);
//    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
//    {
//        return HAL_ERROR;
//    }

//    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_CRC1_27, hmr3->Instance->CRC1);
//    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
//    {
//        return HAL_ERROR;
//    }

//    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_ES2_2B, hmr3->Instance->ES2);
//    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
//    {
//        return HAL_ERROR;
//    }

//    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_ES3_2C, hmr3->Instance->ES3);
//    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
//    {
//        return HAL_ERROR;
//    }

    /* Options of Monitoring and Safty Features */
    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_ES2_2B, hmr3->Instance->ES2);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
		
		/* 20200207 Enable Signal Filtering Start*/
	  __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_SC14_0F, (0x01U<<2));
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
	  /* 20200207 Enable Signal Filtering End*/
		
    hmr3->State     = MR3_STATE_READY;
    hmr3->ErrorCode = EN_MR3_ERROR_NONE;
    
    return HAL_OK;
}

HAL_StatusTypeDef EN_MR3_ADI_CyclicCounterCheckEnable(MR3_HandleTypeDef *hmr3)
{
    hmr3->Init.ADICyclicRead = EN_MR3_ADI_CYCLIC_ABS_READ_ENABLE;
    hmr3->Init.ADICyclicCheck = EN_MR3_ADI_CYCLIC_CHECK_ABS_DATA_ENABLE;
  
    hmr3->Instance->IN1 = 0x80;

    MR3_WRITE_BIT(hmr3->Instance->CRC1, MR3_CRC1_CHK_ADI_Msk, hmr3->Init.ADICyclicCheck);
    MR3_WRITE_BIT(hmr3->Instance->IN1, MR3_IN1_CYC_ADI_Msk, hmr3->Init.ADICyclicRead);
  
    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_CRC1_27, hmr3->Instance->CRC1);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }

    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_IN1_18, hmr3->Instance->IN1);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef EN_MR3_ADI_CyclicCounterCheckDisable(MR3_HandleTypeDef *hmr3)
{
    hmr3->Init.ADICyclicRead = EN_MR3_ADI_CYCLIC_ABS_READ_DISABLE;
    hmr3->Init.ADICyclicCheck = EN_MR3_ADI_CYCLIC_CHECK_ABS_DATA_DISABLE;
    
    hmr3->Instance->IN1 = 0x80;

    MR3_WRITE_BIT(hmr3->Instance->IN1, MR3_IN1_CYC_ADI_Msk, hmr3->Init.ADICyclicRead);
    MR3_WRITE_BIT(hmr3->Instance->CRC1, MR3_CRC1_CHK_ADI_Msk, hmr3->Init.ADICyclicCheck);
    
    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_IN1_18, hmr3->Instance->IN1);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }

    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_CRC1_27, hmr3->Instance->CRC1);
    if (HAL_IS_BIT_SET(hmr3->ErrorCode, EN_MR3_ERROR_SPI))
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

#ifdef CYC_READ_WITH_PREREQUEST
HAL_StatusTypeDef EN_MR3_CyclicRead(MR3_HandleTypeDef *hmr3, uint8_t *pData)
{
    __EN_MR3_SPI_REQUEST_POSITION_DATA(hmr3);

    if (EN_MR3_GetError(hmr3) != 0U)
        __EN_MR3_SPI_SOFTWARE_RESET(hmr3);
		
//For Complete Data Read Flow//
		hmr3->pack.TxBuffer.cyc_addr = (uint8_t)(EN_MR3_SPI_ACCESS_MODE_REGISTER_ACCESS | 0x60);
    hmr3->pack.TxBuffer.opcode = EN_MR3_SPI_OPCODE_REQ_POSITION_DATA;
    hmr3->pack.TxLength = 2;

    //for(int i = 0;i < 13;i++)
    //    hmr3->pack.TxBuffer.data[i] = (uint8_t)0x00U;

    //HAL_Delay(1);

    if (pData == NULL)
    {
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_RESET);

        // HAL_SPI_Transmit(hmr3->hspi, (uint8_t *)&hmr3->pack.TxBuffer, 1, EN_MR3_SPI_TIMEOUT);
        // HAL_SPI_Receive(hmr3->hspi, (uint8_t *)&hmr3->Instance->CC_DATA.ST, 14, EN_MR3_SPI_TIMEOUT);
//        if (HAL_SPI_TransmitReceive(hmr3->hspi,
//                                    (uint8_t *)&hmr3->pack.TxBuffer,
//                                    (uint8_t *)&hmr3->Instance->CC_DATA,
//                                    hmr3->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK)
				 if (HAL_SPI_Transmit(hmr3->hspi,
                                    (uint8_t *)&hmr3->pack.TxBuffer,
                                     hmr3->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK)
            SET_BIT(hmr3->ErrorCode, EN_MR3_ERROR_SPI);
        
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_RESET);
        
        if (HAL_SPI_TransmitReceive(hmr3->hspi,
                                    (uint8_t *)&hmr3->pack.TxBuffer,pData,
                                    hmr3->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK)
            SET_BIT(hmr3->ErrorCode, EN_MR3_ERROR_SPI);
        
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_SET);

    }

//End for complete data Read Flow//
    hmr3->pack.TxBuffer.cyc_addr = (uint8_t)(EN_MR3_SPI_ACCESS_MODE_CYCLIC_READOUT | 0x60);
    //hmr3->pack.TxBuffer.opcode = EN_MR3_SPI_OPCODE_REQ_POSITION_DATA;
		hmr3->pack.TxBuffer.opcode = hmr3->pack.TxBuffer.cyc_addr;
    hmr3->pack.TxLength = 15;

    for(int i = 0;i < 13;i++)
        hmr3->pack.TxBuffer.data[i] = (uint8_t)0x00U;

    //HAL_Delay(1);

    if (pData == NULL)
    {
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_RESET);

        // HAL_SPI_Transmit(hmr3->hspi, (uint8_t *)&hmr3->pack.TxBuffer, 1, EN_MR3_SPI_TIMEOUT);
        // HAL_SPI_Receive(hmr3->hspi, (uint8_t *)&hmr3->Instance->CC_DATA.ST, 14, EN_MR3_SPI_TIMEOUT);
        if (HAL_SPI_TransmitReceive(hmr3->hspi,
                                    (uint8_t *)&hmr3->pack.TxBuffer.opcode,
                                    (uint8_t *)&hmr3->Instance->CC_DATA,
                                    hmr3->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK)
            SET_BIT(hmr3->ErrorCode, EN_MR3_ERROR_SPI);
        
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_RESET);
        
        if (HAL_SPI_TransmitReceive(hmr3->hspi,
                                    (uint8_t *)&hmr3->pack.TxBuffer,pData,
                                    hmr3->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK)
            SET_BIT(hmr3->ErrorCode, EN_MR3_ERROR_SPI);
        
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_SET);

    }

    return HAL_OK;
}
#else

HAL_StatusTypeDef EN_MR3_CyclicRead(MR3_HandleTypeDef *hmr3, uint8_t *pData)
{
    //__EN_MR3_SPI_REQUEST_POSITION_DATA(hmr3);
		
    if (EN_MR3_GetError(hmr3) != 0U)
		{
				__EN_MR3_SPI_SOFTWARE_RESET(hmr3);
		}
		HAL_GPIO_WritePin(GPIOA, MR3_NL_Pin, GPIO_PIN_RESET);//
    
		hmr3->pack.TxBuffer.cyc_addr = (uint8_t)(EN_MR3_SPI_ACCESS_MODE_CYCLIC_READOUT | 0x60);
    hmr3->pack.TxBuffer.opcode = EN_MR3_SPI_OPCODE_REQ_POSITION_DATA;
    hmr3->pack.TxLength = 15;

    for(int i = 0;i < 13;i++)
        hmr3->pack.TxBuffer.data[i] = (uint8_t)0x00U;

    //HAL_Delay(1);

    if (pData == NULL)
    {
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_RESET);

        // HAL_SPI_Transmit(hmr3->hspi, (uint8_t *)&hmr3->pack.TxBuffer, 1, EN_MR3_SPI_TIMEOUT);
        // HAL_SPI_Receive(hmr3->hspi, (uint8_t *)&hmr3->Instance->CC_DATA.ST, 14, EN_MR3_SPI_TIMEOUT);

//			if (HAL_SPI_TransmitReceive(hmr3->hspi,
//                                    (uint8_t *)&hmr3->pack.TxBuffer,
//                                    (uint8_t *)&hmr3->Instance->CC_DATA,
//                                    hmr3->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK)
				if (HAL_SPI_TransmitReceive_DMA(hmr3->hspi,
                                    (uint8_t *)&hmr3->pack.TxBuffer,
                                    (uint8_t *)&hmr3->Instance->CC_DATA,
                                    hmr3->pack.TxLength) != HAL_OK)SET_BIT(hmr3->ErrorCode, EN_MR3_ERROR_SPI);
        
          //HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_RESET);
#if 0        
        if (HAL_SPI_TransmitReceive(hmr3->hspi,
                                    (uint8_t *)&hmr3->pack.TxBuffer,pData,
                                    hmr3->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK)
        SET_BIT(hmr3->ErrorCode, EN_MR3_ERROR_SPI);    
#endif        
#if 1
			  if (HAL_SPI_TransmitReceive_DMA(hmr3->hspi,
                                    (uint8_t *)&hmr3->pack.TxBuffer,pData,
                                    hmr3->pack.TxLength) != HAL_OK)SET_BIT(hmr3->ErrorCode, EN_MR3_ERROR_SPI);
#endif 
				
				
        //HAL_GPIO_WritePin(hmr3->CS_Port, hmr3->CS, GPIO_PIN_SET);

    }

    return HAL_OK;
}


#endif
void EN_MR3_RotationCheck(MR3_HandleTypeDef *hmr3, uint32_t new_AR, uint16_t new_MT, uint32_t data)
{
    static uint32_t old_AR = 0U;
    static uint32_t old_MT = 0U;
    // uint32_t new_AR = (*((uint32_t *)new.AR));

        if (new_AR >= old_AR)
        {
            if ((new_AR - old_AR) >= data)
                if (old_MT == new_MT)
                    hmr3->Rotation = MR3_ENCODER_ROTATION_NOT_INVERTED;
                else 
                    hmr3->Rotation = MR3_ENCODER_ROTATION_INVERTED;
            else
                hmr3->Rotation = MR3_ENCODER_ROTATION_IDLE;
        }
        else if (old_AR >= new_AR)
        {
            if ((old_AR - new_AR) >= data)
                if (old_MT == new_MT)
                    hmr3->Rotation = MR3_ENCODER_ROTATION_INVERTED;
                else 
                    hmr3->Rotation = MR3_ENCODER_ROTATION_NOT_INVERTED;
            else
                hmr3->Rotation = MR3_ENCODER_ROTATION_IDLE;
        }
        else
        {
            hmr3->Rotation = MR3_ENCODER_ROTATION_IDLE;
        }

    old_MT = new_MT;
    old_AR = new_AR;
}

HAL_StatusTypeDef EN_MR3_ClearSingleTurnOffset(MR3_HandleTypeDef *hmr3)
{
    if (hmr3 == NULL)
    {
        return HAL_ERROR;
    }

    hmr3->ErrorCode = EN_MR3_ERROR_NONE;

    if (hmr3->State == MR3_STATE_RESET)
    {
        hmr3->Lock = HAL_UNLOCKED;
    }

    __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI1_1B);

    uint8_t tmp = hmr3->pack.RxBuffer.data[0];

    /* clear singleturn offset */

    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI2_1C, 0x00U | (tmp & 0x0F));

    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI2_1C, 0x00U);

    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI3_1D, 0x00U);    
        
    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI4_1E, 0x00U);

    hmr3->State     = MR3_STATE_READY;
    hmr3->ErrorCode = EN_MR3_ERROR_NONE;

    return HAL_OK;

}

HAL_StatusTypeDef EN_MR3_SetSingleTurnOffset(MR3_HandleTypeDef *hmr3, uint8_t *pData)
{
    if (hmr3 == NULL)
    {
        return HAL_ERROR;
    }

    hmr3->ErrorCode = EN_MR3_ERROR_NONE;

    if (hmr3->State == MR3_STATE_RESET)
    {
        hmr3->Lock = HAL_UNLOCKED;
    }

    // assert_param(IS_MR3_SPI_SINGLE_TURN_RESOlUTION(hmr3->Init.PositionDataSingleTurnResolution));

    __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI1_1B);

    uint8_t tmp = hmr3->pack.RxBuffer.data[0];

    // /* clear singleturn offset */

    // __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI2_1C, 0x00U | (tmp & 0x0F));

    // __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI2_1C, 0x00U);

    // __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI3_1D, 0x00U);    
    
    // __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI4_1E, 0x00U);


    if (hmr3->Init.PositionDataSingleTurnResolution >= EN_MR3_SPI_SINGLE_TURN_RESOlUTION_8BIT)
    {
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI4_1E, *pData);
    }
    else if (hmr3->Init.PositionDataSingleTurnResolution >= EN_MR3_SPI_SINGLE_TURN_RESOlUTION_16BIT)
    {
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI4_1E, *pData);

        pData += 1;

        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI3_1D, *pData);

    }
    else if (hmr3->Init.PositionDataSingleTurnResolution >= EN_MR3_SPI_SINGLE_TURN_RESOlUTION_24BIT)
    {
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI4_1E, *pData);

        pData += 1;

        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI3_1D, *pData);

        pData += 1;

        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI2_1C, *pData);

    }
    else if (hmr3->Init.PositionDataSingleTurnResolution >= EN_MR3_SPI_SINGLE_TURN_RESOlUTION_26BIT)
    {
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI4_1E, *pData);

        pData += 1;

        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI3_1D, *pData);

        pData += 1;

        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI2_1C, *pData);

        pData += 1;
        
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI1_1B, ((*pData) & 0xC0) | (tmp & 0x0F));
    }
    else 
    {
        /* error */
        return HAL_ERROR;

    }

    hmr3->State     = MR3_STATE_READY;
    hmr3->ErrorCode = EN_MR3_ERROR_NONE;

    return HAL_OK;
}

HAL_StatusTypeDef EN_MR3_GetSingleTurnOffset(MR3_HandleTypeDef *hmr3, uint8_t *pData)
{
    if (hmr3 == NULL)
    {
        return HAL_ERROR;
    }

    hmr3->ErrorCode = EN_MR3_ERROR_NONE;

    if (hmr3->State == MR3_STATE_RESET)
    {
        hmr3->Lock = HAL_UNLOCKED;
    }

    if(hmr3->Init.PositionDataSingleTurnResolution >= EN_MR3_SPI_SINGLE_TURN_RESOlUTION_8BIT)
    {
        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI4_1E);

        *pData = hmr3->pack.RxBuffer.data[0];
    }
    else if(hmr3->Init.PositionDataSingleTurnResolution >= EN_MR3_SPI_SINGLE_TURN_RESOlUTION_16BIT)
    {
        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI4_1E);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;

        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI3_1D);

        *pData = hmr3->pack.RxBuffer.data[0];

    }
    else if(hmr3->Init.PositionDataSingleTurnResolution >= EN_MR3_SPI_SINGLE_TURN_RESOlUTION_24BIT)
    {
        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI4_1E);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;

        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI3_1D);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;

        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI2_1C);

        *pData = hmr3->pack.RxBuffer.data[0];
    }
    else if(hmr3->Init.PositionDataSingleTurnResolution >= EN_MR3_SPI_SINGLE_TURN_RESOlUTION_26BIT)
    {
        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI4_1E);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;

        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI3_1D);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;

        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI2_1C);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;

        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI1_1B);

        *pData = hmr3->pack.RxBuffer.data[0] & 0xC0;
    }
    else
    {
        return HAL_ERROR;
    }

    hmr3->State     = MR3_STATE_READY;
    hmr3->ErrorCode = EN_MR3_ERROR_NONE;

    return HAL_OK;
}

HAL_StatusTypeDef EN_MR3_ClearMultiTurnOffset(MR3_HandleTypeDef *hmr3)
{
    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI5_1F, 0x00U);

    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI6_20, 0x00U);    
    
    __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI7_21, 0x00U);

    return HAL_OK;
}

HAL_StatusTypeDef EN_MR3_SetMultiTurnOffset(MR3_HandleTypeDef *hmr3, uint8_t *pData)
{
    // /* Clear MR3 Mt offset setting */

    // __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI5_1F, 0x00U);

    // __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI6_20, 0x00U);    
    
    // __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI7_21, 0x00U);

    /* */
    if (hmr3->Init.PositionDataMultiTurnResolution >= EN_MR3_SPI_MULTITURN_RESOLUTION_8BIT)
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI5_1F, *pData);
    else if(hmr3->Init.PositionDataMultiTurnResolution >= EN_MR3_SPI_MULTITURN_RESOLUTION_16BIT)
    {
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI5_1F, *pData);

        pData += 1;

        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI6_20, *pData);
    }
    else 
    {
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI5_1F, *pData);

        pData += 1;

        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI6_20, *pData);    

        pData += 1;
        
        __EN_MR3_SPI_REGISTER_WRITE(hmr3, EN_MR3_REG_OI7_21, *pData);
    }

    return HAL_OK;
}

HAL_StatusTypeDef EN_MR3_GetMultiTurnOffset(MR3_HandleTypeDef *hmr3, uint8_t *pData)
{
    /* */
    if (hmr3->Init.PositionDataMultiTurnResolution >= EN_MR3_SPI_MULTITURN_RESOLUTION_8BIT)
    {
        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI5_1F);

        *pData = hmr3->pack.RxBuffer.data[0];
    }
    else if(hmr3->Init.PositionDataMultiTurnResolution >= EN_MR3_SPI_MULTITURN_RESOLUTION_16BIT)
    {
        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI5_1F);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;

        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI6_20);

        *pData = hmr3->pack.RxBuffer.data[0];
    }
    else 
    {
        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI5_1F);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;

        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI6_20);

        *pData = hmr3->pack.RxBuffer.data[0];

        pData += 1;
        
        __EN_MR3_SPI_REGISTER_READ(hmr3, EN_MR3_REG_OI7_21);

        *pData = hmr3->pack.RxBuffer.data[0];
    }

    return HAL_OK;
}

HAL_StatusTypeDef EN_MR3_ZeroizeSingleTurn(MR3_HandleTypeDef *hmr3)
{
    volatile uint64_t position = 0;
    volatile uint64_t offset_now = 0;
    volatile uint64_t offset = 0;

    EN_MR3_GetSingleTurnOffset(hmr3, (uint8_t *)&offset_now+2);

    EN_MR3_CyclicRead(hmr3, NULL);

    position = (((uint64_t)(*(uint32_t *)hmr3->Instance->CC_DATA.AR) & 0xfffffE00) << 16);

    offset = (position & 0xfffffffffe000000) + (rev64(offset_now)&0x0000ffffffffffff);

    offset = rev64(offset);

    EN_MR3_SetSingleTurnOffset(hmr3, ((uint8_t *)&offset)+2);
  
    return HAL_OK;
}

HAL_StatusTypeDef EN_MR3_ZeroizeMultiTurn(MR3_HandleTypeDef *hmr3)
{
    volatile uint16_t CMTOffset = 0;
    volatile uint16_t NMTOffset = 0;
    volatile uint16_t mt = 0;

    EN_MR3_GetMultiTurnOffset(hmr3, (uint8_t *)&CMTOffset);
  
    EN_MR3_CyclicRead(hmr3, NULL);
    
    mt = (*((uint16_t *)&hmr3->Instance->CC_DATA.MT));
  
    NMTOffset = mt + CMTOffset;

    EN_MR3_SetMultiTurnOffset(hmr3, (uint8_t *)&NMTOffset);

    return HAL_OK;
}


EN_MR3_StateTypeDef EN_MR3_GetState(MR3_HandleTypeDef *hmr3)
{
    return hmr3->State;
}

uint32_t EN_MR3_GetError(MR3_HandleTypeDef *hmr3)
{
    return hmr3->ErrorCode;
}

/**
 * @brief Reaverse Bytes order of 64-bit unsigned integer
 */
uint64_t rev64(uint64_t val)
{
  return ((uint64_t)__rev((uint32_t) ((val & 0xffffffff00000000) >> 32))) | 
         (((uint64_t)__rev((uint32_t) (val & 0x00000000ffffffff))) << 32);
}
