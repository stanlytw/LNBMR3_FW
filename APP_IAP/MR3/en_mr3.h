/**
 * @file en_mr3.h
 */

#ifndef EN_MR3_H
#define EN_MR3_H

#include <stdint.h>

#include "stm32f7xx_hal.h"

typedef enum
{
    MR3_STATE_RESET = 0x00U,
    MR3_STATE_READY = 0x01U,
    MR3_STATE_BUSY = 0x02U,
    MR3_STATE_BUSY_SPI = 0x03U,
    MR3_STATE_ERROR = 0x03U,
} EN_MR3_StateTypeDef;

typedef enum 
{
    MR3_ENCODER_ROTATION_IDLE = 0,
    MR3_ENCODER_ROTATION_NOT_INVERTED,
    MR3_ENCODER_ROTATION_INVERTED
} EN_MR3_RotationStateTypeDef;

struct mr3_spi_packet_buffer
{
    uint8_t cyc_addr;
    uint8_t opcode;
    uint8_t data[14];
};

typedef struct _mr3_spi_packet
{
    uint32_t TxLength;
    uint32_t RxLength;
    struct mr3_spi_packet_buffer TxBuffer;
    struct mr3_spi_packet_buffer RxBuffer;
} MR3_Packet;

typedef struct __MR3_CyclicRead_Buffer
{
    uint8_t ZERO;
    uint8_t ST;
    uint8_t AR[4];
    uint8_t MT[3];
    uint8_t LC;
    uint8_t ERR;
    uint8_t TEMP[2];
    uint8_t CR[2];
} MR3_CyclicRead_Buffer;//15 bytes

typedef struct
{
    /*  */
    uint8_t data[128];
    /* SIGNAL CONDITIONING */
	  uint8_t SC0;
		uint8_t SC1;
		uint8_t SC2;
		uint8_t SC3;
		uint8_t SC4;
		uint8_t SC5;
		uint8_t SC6;
		uint8_t SC7;
    uint8_t SC11;
    uint8_t SC12;
    uint8_t SC14;
    /* Offset & interpolator */
    uint8_t OI1;
    uint8_t OI2;
    uint8_t OI3;
    uint8_t OI4;
    uint8_t OI5;
    uint8_t OI6;
    uint8_t OI7;
    /* Data resolution */
    uint8_t DR;
    /* Interfaces */
    uint8_t IN1;
    uint8_t IN2;
    uint8_t IN3;
    /* CRC configuration */
    uint8_t CRC1;
    /* Extended settings */
    uint8_t ES2;
    uint8_t ES3;

    /* received data */
		uint8_t DUMMY;
    MR3_CyclicRead_Buffer CC_DATA;
} MR3_TypeDef;

typedef struct
{
    /* Options of OPERATING MODES */
    uint8_t Mode; // SC14_MODE
    /* Options of BIAS CURRENT SOURCE */
    uint8_t BiasCurrentCalibration; // SC14_CFGBIAS
    /* Options od SIGNAL CONDITIONING */
		uint8_t AnalogInputCoarseGain;					  // SC00_CoarseGain
		uint16_t AnalogInputFineGainSine;					// SC01_02_FineGainSine
		uint16_t AnalogInputFineGainCosine;				// SC03_04_FineGainCosine
		uint16_t AnalogInputCenterPotentialSine;	// SC05_06_CenterPotentialSine
		uint16_t AnalogInputCenterPotentialCosine;// SC07_08_CenterPotentialCosine
		uint8_t AnalogInputConnectionMode;        // SC12_INMODE
    uint8_t AnalogInputElectricalMode;        // SC12_UIN
    uint8_t AnalogInputResistanceType;        // SC12_RIN
    uint8_t AnalogInputVoltageDividerType;    // SC12_TUIN
    uint8_t AnalogInputPolarityReferenceType; // SC12_DCPOS
	  uint8_t AnalogInputOffsetReferenceSourceType;   // SC12_REFVOS, add to check MR3 noise
    uint8_t AnalogInputReferenceType;         // SC11_SELREF
    /* Options of INTERPOLATION AND CYCLE COUNTING */
    uint8_t CycleCounterLength;               // OI1_RESO_CC
    uint8_t PositionDataAdjustmentDirection;  // OI1_DIR
    uint32_t PositionDataSingleTurnOffset;    // OI1_OFFS_ST[1:0], OI2_OFFS_ST[9:2], OI3_OFFS_ST[17:10], OI4_OFFS_ST[25:18]
    uint32_t PositionDataMultiTurnOffset;     // OI5_OFFS_MT[7:0], OI6_OFFS_MT[15:8], OI7_OFFS_MT[23:16]
    uint8_t PositionDataMultiTurnResolution;  // DR_RESO_MT
    uint8_t PositionDataSingleTurnResolution; // DR_RESO_ST
    uint8_t PositionDataAquisitionMode;       // IN2_ACQMODE
    /* Options of ADI */
    uint8_t ADIProtocol;               // IN3_SSI_ADI
    uint8_t ADIClockSpeed;             // ES2_SLOW_ADI
    uint8_t ADIDataLength;             // IN3_DL_ADI
    uint8_t ADISyncBitLength;          // IN3_SBL_ADI
    uint8_t ADIPositionDataAdjustment; // ES3_SPO_ADI
    uint8_t ADICyclicRead;             // IN1_CYC_ADI
    uint8_t ADICyclicCheck;            //CRC1_CHK_ADI
    uint8_t ADIStartMode;              // IN1_STP_ADI
    uint8_t ADIInterface;              // IN1_GET_ADI
    /* Options of Monitoring and Safty Features */
    uint8_t ErrorRegisterResetAction;   // CRC1_RES_ERR
    uint8_t SignalErrorFiltered;        // ES2_EN_FAMP
    uint8_t SignalErrorFilterTimeout;   // ES2_TO_FAMP
    uint8_t SignalErrorFilterThreshold; // ES2_THR_FAMP
} MR3_InitTypeDef;

typedef struct
{
    MR3_TypeDef *Instance;

    MR3_InitTypeDef Init;

    MR3_Packet pack;

    EN_MR3_RotationStateTypeDef Rotation;

    SPI_HandleTypeDef *hspi;

    /**
     * @brief SPI CS Port, low active
     */
    GPIO_TypeDef *CS_Port;

    /**
     * @brief SPI CS Pin, low active
     */
    uint16_t CS;

    HAL_LockTypeDef Lock;

    __IO EN_MR3_StateTypeDef State;

    __IO uint32_t ErrorCode;

} MR3_HandleTypeDef;

/* ************************************************************************** */
/*                               MR3 ERROR CODE                               */
/* ************************************************************************** */

#define EN_MR3_ERROR_NONE (0x00000000U)
#define EN_MR3_ERROR_SPI (0x00000001U)
#define EN_MR3_ERROR_CONFIG (0x00000020U)
#define EN_MR3_ERROR_FLAG (0x00000004U)

/* ************************************************************************** */
/*                          REGISTER OPERATION MACRO                          */
/* ************************************************************************** */

/**
 * @brief modify the register with mask and value.
 * @param REG target register varible
 * @param MASK bits mask
 * @param VALUE value of bits
 * @retval None 
 */
#define MR3_WRITE_BIT(REG, MASK, VALUE) ((REG) = ((((REG) & ~MASK) | ((VALUE)&MASK))))

/* ************************************************************************** */
/*                             Function Prototype                             */
/* ************************************************************************** */

int EN_MR3_Init(MR3_HandleTypeDef *hmr3);

HAL_StatusTypeDef EN_MR3_Init_Minimal(MR3_HandleTypeDef *hmr3);

HAL_StatusTypeDef EN_MR3_CyclicRead(MR3_HandleTypeDef *hmr3, uint8_t *pData);

HAL_StatusTypeDef EN_MR3_ADI_CyclicCounterCheckEnable(MR3_HandleTypeDef *hmr3);

HAL_StatusTypeDef EN_MR3_ADI_CyclicCounterCheckDisable(MR3_HandleTypeDef *hmr3);

void EN_MR3_RotationCheck(MR3_HandleTypeDef *hmr3, uint32_t new_AR, uint16_t new_MT, uint32_t data);

HAL_StatusTypeDef EN_MR3_SetSingleTurnOffset(MR3_HandleTypeDef *hmr3, uint8_t *pData);

HAL_StatusTypeDef EN_MR3_GetSingleTurnOffset(MR3_HandleTypeDef *hmr3, uint8_t *pData);

HAL_StatusTypeDef EN_MR3_ClearSingleTurnOffset(MR3_HandleTypeDef *hmr3);

HAL_StatusTypeDef EN_MR3_SetMultiTurnOffset(MR3_HandleTypeDef *hmr3, uint8_t *pData);

HAL_StatusTypeDef EN_MR3_GetMultiTurnOffset(MR3_HandleTypeDef *hmr3, uint8_t *pData);

HAL_StatusTypeDef EN_MR3_ClearMultiTurnOffset(MR3_HandleTypeDef *hmr3);

HAL_StatusTypeDef EN_MR3_ZeroizeSingleTurn(MR3_HandleTypeDef *hmr3);

HAL_StatusTypeDef EN_MR3_ZeroizeMultiTurn(MR3_HandleTypeDef *hmr3);

EN_MR3_StateTypeDef EN_MR3_GetState(MR3_HandleTypeDef *hmr3);

uint32_t EN_MR3_GetError(MR3_HandleTypeDef *hmr3);

uint64_t rev64(uint64_t val);
/* ************************************************************************** */
/*                               Inline function                              */
/* ************************************************************************** */

inline uint8_t getRegByte(MR3_HandleTypeDef *hmr3)
{
    return hmr3->pack.RxBuffer.data[0];
}

/* ************************************************************************** */
/*                              SPI Command Macro                             */
/* ************************************************************************** */

#define EN_MR3_SPI_TIMEOUT (uint16_t)65535U

/**
 * @brief HAL_SPI_TransmiteReceive() with high level activate SS.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @retval None 
 */
#define __EN_MR3_SPI_TransmitReceive(hmr3)                                                \
    do                                                                                    \
    {                                                                                     \
        HAL_GPIO_WritePin((hmr3)->CS_Port, (hmr3)->CS, GPIO_PIN_RESET);                   \
        if (HAL_SPI_TransmitReceive((hmr3)->hspi,                                         \
                                    (uint8_t *)&(hmr3)->pack.TxBuffer,                    \
                                    (uint8_t *)&(hmr3)->pack.RxBuffer,                    \
                                    (hmr3)->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK) \
            SET_BIT((hmr3)->ErrorCode, EN_MR3_ERROR_SPI);                                 \
        HAL_GPIO_WritePin((hmr3)->CS_Port, (hmr3)->CS, GPIO_PIN_SET);                     \
    } while (0U)

/**
 * @brief HAL_SPI_Transmite() with high level activate SS.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @retval None 
 */
#define __EN_MR3_SPI_Transmit(hmr3)                                                \
    do                                                                             \
    {                                                                              \
        HAL_GPIO_WritePin((hmr3)->CS_Port, (hmr3)->CS, GPIO_PIN_RESET);            \
        if (HAL_SPI_Transmit((hmr3)->hspi,                                         \
                             (uint8_t *)&(hmr3)->pack.TxBuffer,                    \
                             (hmr3)->pack.TxLength, EN_MR3_SPI_TIMEOUT) != HAL_OK) \
            SET_BIT((hmr3)->ErrorCode, EN_MR3_ERROR_SPI);                          \
        HAL_GPIO_WritePin((hmr3)->CS_Port, (hmr3)->CS, GPIO_PIN_SET);              \
    } while (0U)

/**
 * @brief Read a byte from MR3 register.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @param addr address of MR3 internal regsiter
 * @retval None 
 */
#define __EN_MR3_SPI_REGISTER_READ(hmr3, addr)                                                       \
    do                                                                                               \
    {                                                                                                \
        (hmr3)->pack.TxBuffer.cyc_addr = (uint8_t)(EN_MR3_SPI_ACCESS_MODE_REGISTER_ACCESS | (addr)); \
        (hmr3)->pack.TxBuffer.opcode = EN_MR3_SPI_OPCODE_REGISTER_READ;                              \
        (hmr3)->pack.TxLength = 3;                                                                   \
        (hmr3)->pack.TxBuffer.data[0] = (uint8_t)0x00U;                                              \
        __EN_MR3_SPI_TransmitReceive((hmr3));                                                        \
    } while (0U)

/**
 * @brief Write a byte to MR3 internal register.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @param addr address of MR3 internal regsiter
 * @param byte a byte of data
 * @retval None 
 */
#define __EN_MR3_SPI_REGISTER_WRITE(hmr3, addr, byte)                                                \
    do                                                                                               \
    {                                                                                                \
        (hmr3)->pack.TxBuffer.cyc_addr = (uint8_t)(EN_MR3_SPI_ACCESS_MODE_REGISTER_ACCESS | (addr)); \
        (hmr3)->pack.TxBuffer.opcode = EN_MR3_SPI_OPCODE_REGISTER_WRITE;                             \
        (hmr3)->pack.TxLength = 3;                                                                   \
        (hmr3)->pack.TxBuffer.data[0] = (uint8_t)(byte);                                             \
        __EN_MR3_SPI_Transmit((hmr3));                                                               \
    } while (0U)

/**
 * @brief Execute SPI command.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @param code MR3 SPI communication opcode
 * @retval None 
 */
#define __EN_MR3_SPI_COMMAND_EXECUTION(hmr3, code)                                                  \
    do                                                                                              \
    {                                                                                               \
        (hmr3)->pack.TxBuffer.cyc_addr = (uint8_t)(EN_MR3_SPI_ACCESS_MODE_REGISTER_ACCESS | 0x60U); \
        (hmr3)->pack.TxBuffer.opcode = (uint8_t)(code);                                             \
        (hmr3)->pack.TxLength = 2;                                                                  \
        __EN_MR3_SPI_Transmit((hmr3));                                                              \
    } while (0U)

/**
 * @brief Execute SPI command: Request of new position data.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @retval None 
 */
#define __EN_MR3_SPI_REQUEST_POSITION_DATA(hmr3) \
    __EN_MR3_SPI_COMMAND_EXECUTION(hmr3, EN_MR3_SPI_OPCODE_REQ_POSITION_DATA)

/**
 * @brief Execute SPI command: Read new data via ADI interface.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @retval None 
 */
#define __EN_MR3_SPI_ADI_READ_DATA(hmr3) \
    __EN_MR3_SPI_COMMAND_EXECUTION(hmr3, EN_MR3_SPI_OPCODE_READ_DATA_ADI)

/**
 * @brief Execute SPI command: Trigger software reset.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @retval None 
 */
#define __EN_MR3_SPI_SOFTWARE_RESET(hmr3) \
    __EN_MR3_SPI_COMMAND_EXECUTION(hmr3, EN_MR3_SPI_OPCODE_SOFT_RESET)

/**
 * @brief Execute SPI command: ERROR simulation: activate ERR bit.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @retval None 
 */
#define __EN_MR3_SPI_ERROR_SIM_ACTIVATE(hmr3) \
    __EN_MR3_SPI_COMMAND_EXECUTION(hmr3, EN_MR3_SPI_OPCODE_ERROR_SIM_ACTIVATE_ERR)

/**
 * @brief Execute SPI command: ERROR simulation: deactivate ERR bit.
 * @param hmr3 pointor to a MR3_HandleTypeDef
 * @retval None 
 */
#define __EN_MR3_SPI_ERROR_SIM_DEACTIVATE(hmr3) \
    __EN_MR3_SPI_COMMAND_EXECUTION(hmr3, EN_MR3_SPI_OPCODE_ERROR_SIM_DEACTIVATE_ERR)

extern MR3_TypeDef MR3;

/* ************************************************************************** */
/*                         Accetable Parameter Option                         */
/* ************************************************************************** */

/* *********************** Options of OPERATING MODES *********************** */
#define EN_MR3_MODE_NORMAL (0x00U << MR3_SC14_MODE_Pos)
#define EN_MR3_MODE_ANALOG_1 (0x01U << MR3_SC14_MODE_Pos)
#define EN_MR3_MODE_ANALOG_2 (0x02U << MR3_SC14_MODE_Pos)
#define EN_MR3_MODE_BYPASS (0x03U << MR3_SC14_MODE_Pos)

#define IS_MR3_MODE(__MODE__)                \
    (((__MODE__) == EN_MR3_MODE_NORMAL) ||   \
     ((__MODE__) == EN_MR3_MODE_ANALOG_1) || \
     ((__MODE__) == EN_MR3_MODE_ANALOG_1) || \
     ((__MODE__) == EN_MR3_MODE_BYPASS))

/* ********************* Options of BIAS CURRENT SOURCE ********************* */
#define EN_MR3_BIAS_CURRENT_100_PERCENT (0x00U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_103_PERCENT (0x01U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_107_PERCENT (0x02U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_111_PERCENT (0x03U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_115_PERCENT (0x04U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_119_PERCENT (0x05U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_124_PERCENT (0x06U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_129_PERCENT (0x07U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_79_PERCENT (0x08U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_81_PERCENT (0x09U << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_84_PERCENT (0x0AU << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_86_PERCENT (0x0BU << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_88_PERCENT (0x0CU << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_91_PERCENT (0x0DU << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_94_PERCENT (0x0EU << MR3_SC14_CFGBIAS_Pos)
#define EN_MR3_BIAS_CURRENT_97_PERCENT (0x0FU << MR3_SC14_CFGBIAS_Pos)

#define IS_MR3_BIAS_CURRENT(__BIAS__)                   \
    (((__BIAS__) == EN_MR3_BIAS_CURRENT_100_PERCENT) || \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_103_PERCENT) || \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_107_PERCENT) || \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_111_PERCENT) || \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_115_PERCENT) || \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_119_PERCENT) || \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_124_PERCENT) || \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_129_PERCENT) || \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_79_PERCENT) ||  \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_81_PERCENT) ||  \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_84_PERCENT) ||  \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_86_PERCENT) ||  \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_88_PERCENT) ||  \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_91_PERCENT) ||  \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_94_PERCENT) ||  \
     ((__BIAS__) == EN_MR3_BIAS_CURRENT_97_PERCENT))

/* ********************* Options of SIGNAL CONDITIONING ********************* */

#define EN_MR3_ANALOG_INPUT_CONNECTION_MODE_DIFFERENTIAL (0x00U << MR3_SC12_INMODE_Pos)
#define EN_MR3_ANALOG_INPUT_CONNECTION_MODE_SINGLE_ENDED (0x01U << MR3_SC12_INMODE_Pos)

#define IS_MR3_ANALOG_INPUT_CONNECTION_MODE(__MODE__)         \
    (((__MODE__) == EN_MR3_ANALOG_INPUT_MODE_DIFFERENTIAL) || \
     ((__MODE__) == EN_MR3_ANALOG_INPUT_MODE_SINGLE_ENDED))

#define EN_MR3_ANALOG_INPUT_ELECTRICAL_MODE_CURRENT (0x00U << MR3_SC12_UIN_Pos)
#define EN_MR3_ANALOG_INPUT_ELECTRICAL_MODE_VOLTAGE (0x01U << MR3_SC12_UIN_Pos)

#define IS_MR3_ANALOG_INPUT_ELECTRICAL_MODE(__MODE__)    \
    (((__MODE__) == EN_MR3_ANALOG_INPUT_MODE_CURRENT) || \
     ((__MODE__) == EN_MR3_ANALOG_INPUT_MODE_VOLTAGE))

#define EN_MR3_ANALOG_INPUT_RESISTANCE_TYPE_0 (0x00U << MR3_SC12_RIN_Pos)
#define EN_MR3_ANALOG_INPUT_RESISTANCE_TYPE_1 (0x01U << MR3_SC12_RIN_Pos)
#define EN_MR3_ANALOG_INPUT_RESISTANCE_TYPE_2 (0x02U << MR3_SC12_RIN_Pos)
#define EN_MR3_ANALOG_INPUT_RESISTANCE_TYPE_3 (0x03U << MR3_SC12_RIN_Pos)

#define IS_MR3_ANALOG_INPUT_RESISTANCE_TYPE(__TYPE__)         \
    (((__TYPE__) == EN_MR3_ANALOG_INPUT_RESISTANCE_TYPE_0) || \
     ((__TYPE__) == EN_MR3_ANALOG_INPUT_RESISTANCE_TYPE_1) || \
     ((__TYPE__) == EN_MR3_ANALOG_INPUT_RESISTANCE_TYPE_2) || \
     ((__TYPE__) == EN_MR3_ANALOG_INPUT_RESISTANCE_TYPE_3))

#define EN_MR3_ANALOG_INPUT_VOLTAGE_DIVIDER_TYPE_0 (0x00U << MR3_SC12_TUIN_Pos)
#define EN_MR3_ANALOG_INPUT_VOLTAGE_DIVIDER_TYPE_1 (0x01U << MR3_SC12_TUIN_Pos)

#define IS_MR3_ANALOG_INPUT_VOLTAGE_DIVIDER_TYPE(__TYPE__)         \
    (((__TYPE__) == EN_MR3_ANALOG_INPUT_VOLTAGE_DIVIDER_TYPE_0) || \
     ((__TYPE__) == EN_MR3_ANALOG_INPUT_VOLTAGE_DIVIDER_TYPE_1))

#define EN_MR3_ANALOG_INPUT_OFFSET_REF_SOURCE_TYPE_V025 (0x02U << MR3_SC11_REFVOS_Pos)


#define IS_MR3_ANALOG_INPUT_OFFSET_REF_SOURCE_TYPE(__TYPE__)         \
    (((__TYPE__) == EN_MR3_ANALOG_INPUT_OFFSET_REF_SOURCE_TYPE_V025)
    
		 
#define EN_MR3_ANALOG_INPUT_VOLTAGE_CURRENT_POLARITY_REFERENCE_TYPE_0 (0x00U << MR3_SC12_DCPOS_Pos)
#define EN_MR3_ANALOG_INPUT_VOLTAGE_CURRENT_POLARITY_REFERENCE_TYPE_1 (0x01U << MR3_SC12_DCPOS_Pos)

#define IS_MR3_ANALOG_INPUT_VOLTAGE_CURRENT_POLARITY_REFERENCE(__TYPE__)              \
    (((__TYPE__) == EN_MR3_ANALOG_INPUT_VOLTAGE_CURRENT_POLARITY_REFERENCE_TYPE_0) || \
     ((__TYPE__) == EN_MR3_ANALOG_INPUT_VOLTAGE_CURRENT_POLARITY_REFERENCE_TYPE_1))

#define EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_0 (0x00U << MR3_SC11_SELREF_Pos)
#define EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_1 (0x01U << MR3_SC11_SELREF_Pos)
#define EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_2 (0x02U << MR3_SC11_SELREF_Pos)
#define EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_3 (0x03U << MR3_SC11_SELREF_Pos)

#define IS_MR3_ANALOG_INPUT_REFERENCE_TYPE(__TYPE__)         \
    (((__TYPE__) == EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_0) || \
     ((__TYPE__) == EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_1) || \
     ((__TYPE__) == EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_2) || \
     ((__TYPE__) == EN_MR3_ANALOG_INPUT_REFERENCE_TYPE_3))

/* TODO: Options of Analog output */

/* *************** Options of INTERPOLATION AND CYCLE COUNTING ************** */
#define EN_MR3_CYCLE_COUNTER_LENGTH_37BIT (0x00U << MR3_OI1_RESO_CC_Pos)
#define EN_MR3_CYCLE_COUNTER_LENGTH_36BIT (0x01U << MR3_OI1_RESO_CC_Pos)
#define EN_MR3_CYCLE_COUNTER_LENGTH_35BIT (0x02U << MR3_OI1_RESO_CC_Pos)
#define EN_MR3_CYCLE_COUNTER_LENGTH_34BIT (0x03U << MR3_OI1_RESO_CC_Pos)

#define IS_MR3_CYCLE_COUNTER_LENGTH(__LENGTH__)                \
    ((__LENGTH__) == EN_MR3_CYCLE_COUNTER_LENGTH_37BIT) ||     \
        ((__LENGTH__) == EN_MR3_CYCLE_COUNTER_LENGTH_36BIT) || \
        ((__LENGTH__) == EN_MR3_CYCLE_COUNTER_LENGTH_35BIT) || \
        ((__LENGTH__) == EN_MR3_CYCLE_COUNTER_LENGTH_34BIT)

#define EN_MR3_POSITION_DATA_ADJUST_DIR_NORMAL (0x00U << MR3_OI1_DIR_Pos)
#define EN_MR3_POSITION_DATA_ADJUST_DIR_INVERTED (0x01U << MR3_OI1_DIR_Pos)

#define EN_MR3_PSITION_DATA_ADJUST_DIR(__DIR__)             \
    ((__DIR__) == EN_MR3_PSITION_DATA_ADJUST_DIR_NORMAL) || \
        ((__DIR__) == EN_MR3_PSITION_DATA_ADJUST_DIR_INVERTED)

#define EN_MR3_POSITION_SINGLE_TURN_OFFSET_MAX 0x3FFFFFFU
#define EN_MR3_POSITION_SINGLE_TURN_OFFSET_MIN 0x00U

#define IS_MR3_POSITION_SINGLE_TURN_OFFSET(__OFFSET__) \
    (((__OFFSET__)-EN_MR3_POSITION_SINGLE_TURN_OFFSET_MIN) <= (EN_MR3_POSITION_SINGLE_TURN_OFFSET_MAX - EN_MR3_POSITION_SINGLE_TURN_OFFSET_MIN))

#define EN_MR3_POSITION_MULTITURN_OFFSET_MAX 0xFFFFFFU
#define EN_MR3_POSITION_MULTITURN_OFFSET_MIN 0x00U

#define IS_MR3_POSITION_MULTITURN_OFFSET(__OFFSET__) \
    (((__OFFSET__)-EN_MR3_POSITION_MULTITURN_OFFSET_MIN) <= (EN_MR3_POSITION_MULTITURN_OFFSET_MAX - EN_MR3_POSITION_MULTITURN_OFFSET_MIN))

#define EN_MR3_SPI_MULTITURN_RESOLUTION_24BIT (0x00U << MR3_DR_RESO_MT_Pos)
#define EN_MR3_SPI_MULTITURN_RESOLUTION_20BIT (0x01U << MR3_DR_RESO_MT_Pos)
#define EN_MR3_SPI_MULTITURN_RESOLUTION_16BIT (0x02U << MR3_DR_RESO_MT_Pos)
#define EN_MR3_SPI_MULTITURN_RESOLUTION_12BIT (0x03U << MR3_DR_RESO_MT_Pos)
#define EN_MR3_SPI_MULTITURN_RESOLUTION_8BIT (0x04U << MR3_DR_RESO_MT_Pos)
#define EN_MR3_SPI_MULTITURN_RESOLUTION_4BIT (0x05U << MR3_DR_RESO_MT_Pos)
#define EN_MR3_SPI_MULTITURN_RESOLUTION_0BIT (0x06U << MR3_DR_RESO_MT_Pos)
// #define EN_MR3_SPI_MULTITURN_RESOLUTION_0BIT 0x07U

#define IS_MR3_SPI_MULTITURN_RESOLUTION(__RES__)                \
    ((__RES__) == EN_MR3_SPI_MULTITURN_RESOLUTION_24BIT) ||     \
        ((__RES__) == EN_MR3_SPI_MULTITURN_RESOLUTION_20BIT) || \
        ((__RES__) == EN_MR3_SPI_MULTITURN_RESOLUTION_16BIT) || \
        ((__RES__) == EN_MR3_SPI_MULTITURN_RESOLUTION_12BIT) || \
        ((__RES__) == EN_MR3_SPI_MULTITURN_RESOLUTION_8BIT) ||  \
        ((__RES__) == EN_MR3_SPI_MULTITURN_RESOLUTION_4BIT) ||  \
        ((__RES__) == EN_MR3_SPI_MULTITURN_RESOLUTION_0BIT)

#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_26BIT (0x00U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_25BIT (0x01U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_24BIT (0x02U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_23BIT (0x03U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_22BIT (0x04U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_21BIT (0x05U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_20BIT (0x06U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_19BIT (0x07U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_18BIT (0x08U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_17BIT (0x09U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_16BIT (0x0AU << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_15BIT (0x0BU << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_14BIT (0x0CU << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_13BIT (0x0DU << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_12BIT (0x0EU << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_11BIT (0x0FU << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_10BIT (0x10U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_9BIT (0x11U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_8BIT (0x12U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_7BIT (0x13U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_6BIT (0x14U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_5BIT (0x15U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_4BIT (0x16U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_3BIT (0x17U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_2BIT (0x18U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_1BIT (0x19U << MR3_DR_RESO_ST_Pos)
#define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_0BIT (0x1AU << MR3_DR_RESO_ST_Pos)
// #define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_0BIT 0x1BU
// #define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_0BIT 0x1CU
// #define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_0BIT 0x1DU
// #define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_0BIT 0x1EU
// #define EN_MR3_SPI_SINGLE_TURN_RESOlUTION_0BIT 0x1FU

#define IS_MR3_SPI_SINGLE_TURN_RESOlUTION(__RES__)                \
    ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_26BIT) ||     \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_25BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_24BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_23BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_22BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_21BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_20BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_19BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_18BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_17BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_16BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_15BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_14BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_13BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_12BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_11BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_10BIT) || \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_9BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_8BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_7BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_6BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_5BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_4BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_3BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_2BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_1BIT) ||  \
        ((__RES__) == EN_MR3_SPI_SINGLE_TURN_RESOlUTION_0BIT) ||

#define EN_MR3_POSITION_DATA_AQUISITION_MODE_NORMAL (0x00U << MR3_IN2_ACQMODE_Pos)
#define EN_MR3_POSITION_DATA_AQUISITION_MODE_PIPELINE (0x01U << MR3_IN2_ACQMODE_Pos)
#define EN_MR3_POSITION_DATA_AQUISITION_MODE_CONTINUOUS (0x02U << MR3_IN2_ACQMODE_Pos)
#define EN_MR3_POSITION_DATA_AQUISITION_MODE_CONTINUOUS_WITH_INTERPOLATION (0x03U << MR3_IN2_ACQMODE_Pos)

#define IS_MR3_POSITION_DATA_AQUISITION_MODE(__MODE__)                     \
    ((__MODE__) == EN_MR3_POSITION_DATA_AQUISITION_MODE_NORMAL) ||         \
        ((__MODE__) == EN_MR3_POSITION_DATA_AQUISITION_MODE_PIPELINE) ||   \
        ((__MODE__) == EN_MR3_POSITION_DATA_AQUISITION_MODE_CONTINUOUS) || \
        ((__MODE__) == EN_MR3_POSITION_DATA_AQUISITION_MODE_CONTINUOUS_WITH_INTERPOLATION)

// #define NL_ENABLE_AQUIRE_FUNCTION /* not for spi interface. */

/* ***************************** Options of ADI ***************************** */
#define EN_MR3_ADI_PROTO_BISS_C (0x00U << MR3_IN3_SSI_ADI_Pos)
#define EN_MR3_ADI_PROTO_SSI (0x01U << MR3_IN3_SSI_ADI_Pos)

#define IS_MR3_ADI_PROTO(__PROTO__)              \
    (((__PROTO__) == EN_MR3_ADI_PROTO_BISS_C) || \
     ((__PROTO__) == EN_MR3_ADI_PROTO_SSI))

#define EN_MR3_ADI_CLOCK_RATE_NORMAL (0x00U << MR3_ES2_SLOW_ADI_Pos)
#define EN_MR3_ADI_CLOCK_RATE_SLOW (0x01U << MR3_ES2_SLOW_ADI_Pos)

#define IS_MR3_ADI_CLOCK_RATE(__CLOCK__)              \
    (((__CLOCK__) == EN_MR3_ADI_CLOCK_RATE_NORMAL) || \
     ((__CLOCK__) == EN_MR3_ADI_CLOCK_RATE_SLOW))

#define EN_MR3_ADI_DATA_LENGTH_MAX 0x1FU
#define EN_MR3_ADI_DATA_LENGTH_MIN 0x00U

#define IS_MR3_ADI_DATA_LENGTH(__LENGTH__)                                           \
    (((__LENGTH__) - ((uint8_t)EN_MR3_ADI_DATA_LENGTH_MIN << MR3_IN3_DL_ADI_Pos)) <= \
     ((uint8_t)(EN_MR3_ADI_DATA_LENGTH_MAX - EN_MR3_ADI_DATA_LENGTH_MIN) << MR3_IN3_DL_ADI_Pos))

#define EN_MR3_ADI_SYNC_BIT_LENGTH_0 (0x00U << MR3_IN3_SBL_ADI_Pos)
#define EN_MR3_ADI_SYNC_BIT_LENGTH_1 (0x01U << MR3_IN3_SBL_ADI_Pos)
#define EN_MR3_ADI_SYNC_BIT_LENGTH_2 (0x02U << MR3_IN3_SBL_ADI_Pos)
#define EN_MR3_ADI_SYNC_BIT_LENGTH_3 (0x03U << MR3_IN3_SBL_ADI_Pos)

#define IS_MR3_ADI_SYNC_BIT_LENGTH(__LENGTH__)         \
    (((__LENGTH__) == EN_MR3_ADI_SYNC_BIT_LENGTH_0) || \
     ((__LENGTH__) == EN_MR3_ADI_SYNC_BIT_LENGTH_1) || \
     ((__LENGTH__) == EN_MR3_ADI_SYNC_BIT_LENGTH_2) || \
     ((__LENGTH__) == EN_MR3_ADI_SYNC_BIT_LENGTH_3))

#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_0 ((0x00U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_1 ((0x01U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_2 ((0x02U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_3 ((0x03U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_4 ((0x04U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_5 ((0x05U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_6 ((0x06U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_7 ((0x07U) << MR3_ES3_SPO_ADI_Pos)

#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_0 ((0x08U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_1 ((0x09U) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_2 ((0x0AU) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_3 ((0x0BU) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_4 ((0x0CU) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_5 ((0x0DU) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_6 ((0x0EU) << MR3_ES3_SPO_ADI_Pos)
#define EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_7 ((0x0FU) << MR3_ES3_SPO_ADI_Pos)

#define IS_MR3_ADI_ABS_DATA_ADJUSTMENT(__ADJ__)               \
    (((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_0) ||  \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_1) ||  \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_2) ||  \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_3) ||  \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_4) ||  \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_5) ||  \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_6) ||  \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_PLUS_7) ||  \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_0) || \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_1) || \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_2) || \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_3) || \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_4) || \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_5) || \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_6) || \
     ((__ADJ__) == EN_MR3_ADI_ABS_DATA_ADJUSTMENT_MINUS_7))

#define EN_MR3_ADI_CYCLIC_ABS_READ_DISABLE (0x00U << MR3_IN1_CYC_ADI_Pos)
#define EN_MR3_ADI_CYCLIC_ABS_READ_ENABLE (0x01U << MR3_IN1_CYC_ADI_Pos)

#define IS_MR3_ADI_CYCLIC_ABS_READ(__CYCLIC__)               \
    (((__CYCLIC__) == EN_MR3_ADI_CYCLIC_ABS_READ_DISABLE) || \
     ((__CYCLIC__) == EN_MR3_ADI_CYCLIC_ABS_READ_ENABLE))

#define EN_MR3_ADI_CYCLIC_CHECK_ABS_DATA_DISABLE (0x00U << MR3_CRC1_CHK_ADI_Pos)
#define EN_MR3_ADI_CYCLIC_CHECK_ABS_DATA_ENABLE (0x01U << MR3_CRC1_CHK_ADI_Pos)

#define IS_MR3_ADI_CYCLIC_CHECK_ABS_DATA(__CHECK__)               \
    (((__CHECK__) == EN_MR3_ADI_CYCLIC_CHECK_ABS_DATA_DISABLE) || \
     ((__CHECK__) == EN_MR3_ADI_CYCLIC_CHECK_ABS_DATA_ENABLE))

#define EN_MR3_ADI_START_MODE_WITHOUT_ABS_DATA (0x00U << MR3_IN1_STP_ADI_Pos)
#define EN_MR3_ADI_START_MODE_WITH_ABS_DATA (0x01U << MR3_IN1_STP_ADI_Pos)

#define IS_ADI_START_MODE(__MODE__)                      \
    (((__MODE__) == MR3_ADI_START_MODE_WITH_ABS_DATA) || \
     ((__MODE__) == MR3_ADI_START_MODE_WITHOUT_ABS_DATA))

#define EN_MR3_ADI_INTERFACE_NORMAL (0x00U << MR3_IN1_GET_ADI_Pos)
#define EN_MR3_ADI_INTERFACE_BISS_CHAIN (0x00U << MR3_IN1_GET_ADI_Pos)

#define IS_MR3_ADI_INTERFACE(__INTERFACE__)              \
    (((__INTERFACE__) == EN_MR3_ADI_INTERFACE_NORMAL) || \
     ((__INTERFACE__) == EN_MR3_ADI_INTERFACE_BISS_CHAIN))

/* **************** Options of Monitoring and Safty Features **************** */

#define EN_MR3_ERROR_REGISTER_RESET_ACTION_READ_ABS_DATA (0x00U << MR3_CRC1_RES_ERR_Pos)
#define EN_MR3_ERROR_REGISTER_RESET_ACTION_READ_ERR_REGISTER (0x01U << MR3_CRC1_RES_ERR_Pos)

#define IS_MR3_ERROR_REGISTER_RESET_ACTION(__ACTION__)                         \
    (((__ACTION__) == EN_MR3_ERROR_REGISTER_RESET_ACTION_READ_ERR_REGISTER) || \
     ((__ACTION__) == EN_MR3_ERROR_REGISTER_RESET_ACTION_READ_ABS_DATA))

#define EN_MR3_SIGNAL_ERROR_FILTERED_ENABLE (0x00U << MR3_ES2_EN_FAMP_Pos)
#define EN_MR3_SIGNAL_ERROR_FILTERED_DISABLE (0x01U << MR3_ES2_EN_FAMP_Pos)

#define IS_MR3_SIGNAL_ERROR_FILTERED(__FILTERED__)              \
    (((__FILTERED__) == EN_MR3_SIGNAL_ERROR_FILTERED_ENABLE) || \
     ((__FILTERED__) == EN_MR3_SIGNAL_ERROR_FILTERED_DISABLE))

#define EN_MR3_SIGNAL_ERROR_FILTER_TIMEOUT_273us (0x00U << MR3_ES2_TO_FAMP_Pos)
#define EN_MR3_SIGNAL_ERROR_FILTER_TIMEOUT_546us (0x01U << MR3_ES2_TO_FAMP_Pos)

#define IS_MR3_SIGNAL_ERROR_FILTER_TIMEOUT(__TIMEOUT__)             \
    (((__TIMEOUT__) == EN_MR3_SIGNAL_ERROR_FILTER_TIMEOUT_273us) || \
     ((__TIMEOUT__) == EN_MR3_SIGNAL_ERROR_FILTER_TIMEOUT_546us))

#define EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_17us (0x00U << MR3_ES2_THR_FAMP_Pos)
#define EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_34us (0x01U << MR3_ES2_THR_FAMP_Pos)
#define EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_51us (0x02U << MR3_ES2_THR_FAMP_Pos)
#define EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_68us (0x03U << MR3_ES2_THR_FAMP_Pos)

#define IS_MR3_SIGNAL_ERROR_FILTER_THRESHOLD(__THRESHOLD__)            \
    (((__THRESHOLD__) == EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_17us) || \
     ((__THRESHOLD__) == EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_34us) || \
     ((__THRESHOLD__) == EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_51us) || \
     ((__THRESHOLD__) == EN_MR3_SIGNAL_ERROR_FILTER_THRESHOLD_68us))
/* ************************************************************************** */
/*                         SPI access mode and opcode                         */
/* ************************************************************************** */

/**
 *  +======================+===============+
 *  | Bit 7                | Bit 6 - Bit 0 |
 *  +======================+===============+
 *  | CYC(SPI_ACCESS_MODE) | ADDR          |
 *  +======================+===============+
 */

#define EN_MR3_SPI_ACCESS_MODE_REGISTER_ACCESS ((uint8_t)0x00U)
#define EN_MR3_SPI_ACCESS_MODE_CYCLIC_READOUT ((uint8_t)0x80U)

#define EN_MR3_SPI_OPCODE_REQ_POSITION_DATA ((uint8_t)0x00U)
#define EN_MR3_SPI_OPCODE_WRITE_CONFIG_EEPROM ((uint8_t)0x01U)
#define EN_MR3_SPI_OPCODE_READ_DATA_ADI ((uint8_t)0x02U)
#define EN_MR3_SPI_OPCODE_SOFT_RESET ((uint8_t)0x03U)
#define EN_MR3_SPI_OPCODE_VERIFY_CRC ((uint8_t)0x04U)
#define EN_MR3_SPI_OPCODE_ERROR_SIM_ACTIVATE_ERR ((uint8_t)0x05U)
#define EN_MR3_SPI_OPCODE_ERROR_SIM_DEACTIVATE_ERR ((uint8_t)0x06U)
#define EN_MR3_SPI_OPCODE_REGISTER_WRITE ((uint8_t)0x10U)
#define EN_MR3_SPI_OPCODE_REGISTER_READ ((uint8_t)0x20U)

#endif
