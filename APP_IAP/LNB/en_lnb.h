#include <stdint.h>
#include <stddef.h>
#include "stm32f7xx_hal.h"
#include "en_lnb_reg.h"

#ifndef EN_LNB_H
#define EN_LNB_H

typedef enum
{
    LNB_STATE_RESET = 0x00U,

    LNB_STATE_READY = 0x01U,

    LNB_STATE_BUSY = 0x02U

} LNB_StateTypeDef;

/**
 * @brief A copy of LNB interal register (RAM)
 * 
 * @note First 19 byte are the corresponding Register of LNB (which are 
 *       arranged and named according to Table 6. Register layout, LNB
 *       datasheet C1) and elenment IR, AR are the sensor data from LNB
 *       after preprocessing.
 */
typedef struct
{
    /* Corresponding Registers of LNB */

    /* Signal Calibration */
    uint8_t SC1;
    uint8_t SC2;
    uint8_t SC3;
    uint8_t SC4;
    uint8_t SC5;
    uint8_t SC6;

    /* LED Power Control */
    uint8_t LPC;

    /* Output Configuration */
    uint8_t OC1;
    uint8_t OC2;

    /* Test Function */
    uint8_t TF;

    /* FlexCount*/
    uint8_t FC1;
    uint8_t FC2;
    uint8_t FC3;
    uint8_t FC4;
    uint8_t FC5;
    uint8_t FC6;
    uint8_t FC7;
    uint8_t FC8;

    /* Status (read only) */
    uint8_t ST;

    /* Registers For Calculation (Only on MCU) */
    uint32_t IR; /* increamental Data */
    uint32_t AR; /* Absolute Position */
    uint32_t MT; /* Multi-turn */

    /* Registers for Calculation (in byte)*/
    uint8_t AR_b[4];
    uint8_t MT_b[4];
} LNB_TypeDef;

/**
 * @brief Parameter for LNB Initialization Process
 */
typedef struct
{
    uint8_t Mode;          // EPG
    uint8_t FlexCount;     // ENFLEX
    uint8_t SSI;           // NENSHIFT
    uint8_t Interpolator;  // ENIPO
    uint8_t ABSOutputMode; // SELABS

    /* FlexCount Parameter */
    uint8_t FlexCountResolution; // RESSUB
    // uint8_t ABZOutputLow; // NOUTLO
    uint8_t FlexCountPositionOffset; // ZPOS
    uint8_t InvertINCA;              // INVA
    uint8_t InvertINCB;              // INVB
    uint8_t InvertINCZ;              // INVZ
    uint8_t INCZPulseWidth;          // Z90

    /* Interpolator Parameter */
    uint8_t InterpolatorResolution;    // RESIPO
    uint8_t InterpolatorHysteresis;    // HYS
    uint8_t InterpolatorFilterDisable; // NENF

    /* ABZ interface Parameter */
    uint8_t ABZOutputResolution; // INC
    // uint8_t ABZOutputTristateEnable; // TRIABZ

    /* SSI Parameter */
    uint8_t SSIOutputDataFormat; // NGARY
    uint8_t SSIIdleOutput;       // RNF
    uint8_t SSILength;           // SRC
    uint8_t SSIDirection;        //DIR

    /* Signal Calibration Parameter */
    uint8_t DefaultSignalCalibration;
    uint8_t GainRange;         // GR
    uint8_t SinGain;           // GS
    uint8_t PositiveSinOffset; // OSP
    uint8_t NegativeSinOffset; // OSN
    uint8_t CosGain;           // GC
    uint8_t PositiveCosOffset; // OCP
    uint8_t NegativeCosOffset; // OCN

    /* LED Power Control parameter */
    uint8_t DefaultLEDPowerControl;
    uint8_t LEDPowerControlMode;     // LCMOD
    uint8_t LEDPowerControlType;     // LCTYP
    uint8_t LEDPowerControlSetpoint; // LCSET

    /* Test Function Parameter */
    uint8_t TestMode;      // TA
    uint8_t TestSignalMux; // TMUX

} LNB_InitTypeDef;

struct lnb_spi_packet_buffer
{
    uint8_t opcode;
    uint8_t data[20];
};

typedef struct _lnb_spi_packet
{
    uint32_t TxLength;
    uint32_t RxLength;
    struct lnb_spi_packet_buffer TxBuffer;
    struct lnb_spi_packet_buffer RxBuffer;
} LNB_SPI_Packet;

typedef struct __LNB_HandleTypeDef
{
    LNB_TypeDef *Instance;

    LNB_InitTypeDef Init;

    uint32_t IncDiscResolution;

    LNB_SPI_Packet pack;

    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *CS_Port; /* SPI CS Port*/

    uint16_t CS; /* SPI CS Pin */

    GPIO_TypeDef *INCA_Port; /* INCA Port*/

    uint16_t INCA; /* INCA Pin */

    GPIO_TypeDef *INCB_Port; /* INCB Port*/

    uint16_t INCB; /* INCB Pin */

    GPIO_TypeDef *INCZ_Port; /* INCZ Port*/

    uint16_t INCZ; /* INCZ Pin */

    __IO LNB_StateTypeDef State;

    __IO uint8_t Error;

} LNB_HandleTypeDef;

/* ************************************************************************** */
/*                          REGISTER OPTERATION MACRO                         */
/* ************************************************************************** */

/**
 * @brief modify the register with mask and value.
 * @param REG target register varible
 * @param MASK bits mask
 * @param VALUE value of bits
 * @retval None 
 */
#define LNB_WRITE_BIT(REG, MASK, VALUE) ((REG) = ((((REG) & ~MASK) | ((VALUE)&MASK))))

/**
 * @brief modify the register with mask and value, also on LNB.
 * @param HANDLE pointor to a LNB_HandleTypeDef
 * @param ADDR address of destination register
 * @param REG target register varible
 * @param MASK bits mask
 * @param VALUE value of bits
 * @retval None 
 */
#define LNB_WRITE_BIT_TRACE(HANDLE, ADDR, REG, MASK, VALUE)         \
    do                                                              \
    {                                                               \
        LNB_WRITE_BIT((REG), (MASK), (VALUE));                      \
        __EN_LNB_SPI_COMMAND_REGISTER_WRITE_BYTE((HANDLE), (ADDR)); \
    } while (0U)

/**
 * @brief append event parity to MSB.
 * @param \_\_BYTE\_\_ target byte
 * @retval None 
 */
#define EVEN_PARITY_MSB(__BYTE__)              \
    do                                         \
    {                                          \
        (__BYTE__) &= ((uint8_t)0x7FU);        \
        (__BYTE__) |= even_parity((__BYTE__)); \
    } while (0U)

/* ************************************************************************** */
/*                              spi command macro                             */
/* ************************************************************************** */

/**
 * @brief HAL_SPI_TransmiteReceive() with high activate SS.
 * @param hlnb pointor to a LNB_HandleTypeDef
 * @retval None 
 */
#define __EN_LNB_SPI_TransmitReceive(hlnb)                                       \
    do                                                                           \
    {                                                                            \
        HAL_GPIO_WritePin((hlnb)->CS_Port, (hlnb)->CS, GPIO_PIN_SET);            \
        HAL_SPI_TransmitReceive((hlnb)->hspi, (uint8_t *)&(hlnb)->pack.TxBuffer, \
                                (uint8_t *)&(hlnb)->pack.RxBuffer,               \
                                (hlnb)->pack.TxLength, 65535);                   \
        HAL_GPIO_WritePin((hlnb)->CS_Port, (hlnb)->CS, GPIO_PIN_RESET);          \
    } while (0U)

/**
 * @brief read the register status from LNB to pack.Rxbuffer.
 * @param hlnb pointor to LNB_HandleTypeDef
 * @retval None
 */
#define __EN_LNB_SPI_COMMAND_REGISTER_STATUS(hlnb)                        \
    do                                                                    \
    {                                                                     \
        (hlnb)->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_STATUS; \
        (hlnb)->pack.TxLength = 3;                                        \
        (hlnb)->pack.RxLength = 3;                                        \
        (hlnb)->pack.TxBuffer.data[0] = (uint8_t)0x00U;                   \
        (hlnb)->pack.TxBuffer.data[1] = (uint8_t)0x00U;                   \
        __EN_LNB_SPI_TransmitReceive((hlnb));                             \
    } while (0U)

/**
 * @brief read the register(RAM) status(Read only) from LNB to pack.Rxbuffer.
 * @param hlnb pointor to LNB_HandleTypeDef
 * @retval None
 */
#define __EN_LNB_SPI_COMMAND_REGISTER_DATA(hlnb)                        \
    do                                                                  \
    {                                                                   \
        (hlnb)->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_READ; \
        (hlnb)->pack.TxLength = 3;                                      \
        (hlnb)->pack.RxLength = 3;                                      \
        (hlnb)->pack.TxBuffer.data[0] = EN_LNB_REG_ST_18;               \
        (hlnb)->pack.TxBuffer.data[1] = (uint8_t)0x00U;                 \
        __EN_LNB_SPI_TransmitReceive((hlnb));                           \
    } while (0U)

/**
 * @brief write a byte of hlnb->Instace to LNB
 * @param hlnb pointor to LNB_HandleTypeDef
 * @param addr address of the byte to write
 * @retval None
 */
#define __EN_LNB_SPI_COMMAND_REGISTER_WRITE_BYTE(hlnb, addr)                   \
    do                                                                         \
    {                                                                          \
        (hlnb)->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;       \
        (hlnb)->pack.TxLength = 3;                                             \
        (hlnb)->pack.RxLength = 3;                                             \
        (hlnb)->pack.TxBuffer.data[0] = (addr);                                \
        (hlnb)->pack.TxBuffer.data[1] = ((uint8_t *)(hlnb)->Instance)[(addr)]; \
        EVEN_PARITY_MSB((hlnb)->pack.TxBuffer.data[1]);                        \
        __EN_LNB_SPI_TransmitReceive((hlnb));                                  \
    } while (0U)

/**
 * @brief write a byte of hlnb->Instace to LNB
 * @param hlnb pointor to LNB_HandleTypeDef
 * @param addr address of the byte to write
 * @param num number of the bytes to write
 * @retval None
 */
#define __EN_LNB_SPI_COMMAND_REGISTER_WRITE(hlnb, addr, num)                               \
    do                                                                                     \
    {                                                                                      \
        (hlnb)->pack.TxBuffer.opcode = EN_LNB_SPI_OPCODE_REGISTER_WRITE;                   \
        (hlnb)->pack.TxLength = num;                                                       \
        (hlnb)->pack.RxLength = num;                                                       \
        (hlnb)->pack.TxBuffer.data[0] = (addr);                                            \
        for (uint8_t i = 0; i < num; i++)                                                  \
            (hlnb)->pack.TxBuffer.data[i + 1] = ((uint8_t *)(hlnb)->Instance)[(addr) + i]; \
        EVEN_PARITY_MSB((hlnb)->pack.TxBuffer.data[1]);                                    \
        __EN_LNB_SPI_TransmitReceive((hlnb));                                              \
    } while (0U)

/* ************************************************************************** */
/*                         Accetable Parameter Option                         */
/* ************************************************************************** */

#define EN_LNB_MODE_INTERFACE 0x00U
#define EN_LNB_MODE_PARALLEL 0x10U

#define IS_LNB_MODE(__MODE__) (((__MODE__) == EN_LNB_MODE_INTERFACE) || \
                               ((__MODE__) == EN_LNB_MODE_INTERFACE))

#define EN_LNB_FLEXCOUNT_ENABLE 0x00U
#define EN_LNB_FLEXCOUNT_DISABLE 0x02U

#define IS_LNB_FLEXCOUNT(__FLEXCOUNT__) (((__FLEXCOUNT__) == EN_LNB_FLEXCOUNT_ENABLE) || \
                                         ((__FLEXCOUNT__) == EN_LNB_FLEXCOUNT_DISABLE))

#define EN_LNB_INTERPOLATOR_ENABLE 0x10U
#define EN_LNB_INTERPOLATOR_DISABLE 0x00U

#define IS_LNB_INTERPOLATOR(__INTERPOLATOR__) (((__INTERPOLATOR__) == EN_LNB_INTERPOLATOR_ENABLE) || \
                                               ((__INTERPOLATOR__) == EN_LNB_INTERPOLATOR_DISABLE))

#define EN_LNB_SSI_ENABLE 0x00U
#define EN_LNB_SSI_DISABLE 0x40U

#define IS_LNB_SSI(__SSI__) (((__SSI__) == EN_LNB_SSI_ENABLE) || \
                             ((__SSI__) == EN_LNB_SSI_DISABLE))

#define EN_LNB_ABS_OUTPUT_MODE_FLEXCOUNT 0x00U
#define EN_LNB_ABS_OUTPUT_MODE_MAXIMUM 0x02U

#define IS_LNB_ABS_OUTPUT(__MODE__) (((__MODE__) == EN_LNB_ABS_OUTPUT_MODE_FLEXCOUNT) || \
                                     ((__MODE__) == EN_LNB_ABS_OUTPUT_MODE_MAXIMUM))

// #define EN_LNB_FLEXCOUNT_RESOLUTION

#define IS_LNB_FLEXCOUNT_RESOLUTION(__RESOLUTION__) (0U)
// TODO:Range Check. It's related to interpolator resolution

#define EN_LNB_ABZ_OUTPUT_LOW 0x00U
#define EN_LNB_ABZ_OUTPUT_NORMAL 0x80U

#define IS_LNB_ABZ_OUTPUT(__MODE__) (((__MODE__) == EN_LNB_ABZ_OUTPUT_LOW) || \
                                     ((__MODE__) == EN_LNB_ABZ_OUTPUT_LOW))

// #define EN_LNB_FLEXCOUNT_POSITION_OFFSET
#define IS_LNB_FLEXCOUNT_POSITION_OFFSET(__OFFSET__) (0U)

#define EN_LNB_INCA_NORMAL 0x00U
#define EN_LNB_INCA_INVERTED 0x40U

#define IS_LNB_INCA_INVERTED(__INVERTED__) (((__INVERTED__) == EN_LNB_INCA_NORMAL) || \
                                            ((__INVERTED__) == EN_LNB_INCA_INVERTED))

#define EN_LNB_INCB_NORMAL 0x00U
#define EN_LNB_INCB_INVERTED 0x20U

#define IS_LNB_INCB_INVERTED(__INVERTED__) (((__INVERTED__) == EN_LNB_INCB_NORMAL) || \
                                            ((__INVERTED__) == EN_LNB_INCB_INVERTED))

#define EN_LNB_INCZ_NORMAL 0x00U
#define EN_LNB_INCZ_INVERTED 0x10U

#define IS_LNB_INCZ_INVERTED(__INVERTED__) (((__INVERTED__) == EN_LNB_INCZ_NORMAL) || \
                                            ((__INVERTED__) == EN_LNB_INCZ_INVERTED))

#define EN_LNB_INCZ_PULSE_WIDTH_180 0x00U
#define EN_LNB_INCZ_PULSE_WIDTH_90 0x10U

#define IS_LNB_INCZ_PULSE_WIDTH(__WIDTH__) (((__WIDTH__) == EN_LNB_INCZ_PULSE_WIDTH_180) || \
                                            ((__WIDTH__) == EN_LNB_INCZ_PULSE_WIDTH_180))

#define EN_LNB_INTERPOLATOR_RESOLUTION_8BIT 0x00U
#define EN_LNB_INTERPOLATOR_RESOLUTION_7BIT 0x20U
#define EN_LNB_INTERPOLATOR_RESOLUTION_6BIT 0x40U
#define EN_LNB_INTERPOLATOR_RESOLUTION_5BIT 0x60U

#define IS_LNB_INTERPOLATOR_RESOLUTION(__BIT__) (((__BIT__) == EN_LNB_INTERPOLATOR_RESOLUTION_8BIT) || \
                                                 ((__BIT__) == EN_LNB_INTERPOLATOR_RESOLUTION_7BIT) || \
                                                 ((__BIT__) == EN_LNB_INTERPOLATOR_RESOLUTION_6BIT) || \
                                                 ((__BIT__) == EN_LNB_INTERPOLATOR_RESOLUTION_5BIT))

#define EN_LNB_INTERPOLATOR_HYSTERESIS_0 0x00U
#define EN_LNB_INTERPOLATOR_HYSTERESIS_1 0x01U
#define EN_LNB_INTERPOLATOR_HYSTERESIS_2 0x02U
#define EN_LNB_INTERPOLATOR_HYSTERESIS_3 0x03U
#define EN_LNB_INTERPOLATOR_HYSTERESIS_4 0x04U
#define EN_LNB_INTERPOLATOR_HYSTERESIS_5 0x05U
#define EN_LNB_INTERPOLATOR_HYSTERESIS_6 0x06U
#define EN_LNB_INTERPOLATOR_HYSTERESIS_7 0x07U

#define IS_LNB_INTERPOLATOR_HYSTERESIS(__DEGREE__) (((__DEGREE__) == EN_LNB_INTERPOLATOR_HYSTERESIS_0) || \
                                                    ((__DEGREE__) == EN_LNB_INTERPOLATOR_HYSTERESIS_1) || \
                                                    ((__DEGREE__) == EN_LNB_INTERPOLATOR_HYSTERESIS_2) || \
                                                    ((__DEGREE__) == EN_LNB_INTERPOLATOR_HYSTERESIS_3) || \
                                                    ((__DEGREE__) == EN_LNB_INTERPOLATOR_HYSTERESIS_4) || \
                                                    ((__DEGREE__) == EN_LNB_INTERPOLATOR_HYSTERESIS_5) || \
                                                    ((__DEGREE__) == EN_LNB_INTERPOLATOR_HYSTERESIS_6) || \
                                                    ((__DEGREE__) == EN_LNB_INTERPOLATOR_HYSTERESIS_7))

#define EN_LNB_INTERPOLATOR_FILTER_ENABLE 0x00U
#define EN_LNB_INTERPOLATOR_FILTER_DISABLE 0x40U

#define IS_LNB_INTERPOLATOR_FILTER(__FILTER__) (((__FILTER__) == EN_LNB_INTERPOLATOR_FILTER_ENABLE) || \
                                                ((__FILTER__) == EN_LNB_INTERPOLATOR_FILTER_DISABLE))

#define EN_LNB_ABZ_OUTPUT_RESOLUTION_0 0x00U
#define EN_LNB_ABZ_OUTPUT_RESOLUTION_1 0x10U
#define EN_LNB_ABZ_OUTPUT_RESOLUTION_2 0x20U
#define EN_LNB_ABZ_OUTPUT_RESOLUTION_3 0x30U
#define EN_LNB_ABZ_OUTPUT_RESOLUTION_4 0x40U
#define EN_LNB_ABZ_OUTPUT_RESOLUTION_5 0x50U
#define EN_LNB_ABZ_OUTPUT_RESOLUTION_6 0x60U
#define EN_LNB_ABZ_OUTPUT_RESOLUTION_7 0x70U

#define IS_LNB_ABZ_OUTPUT_RESOLUTION(__RESOLUTION__) (((__RESOLUTION__)EN_LNB_ABZ_OUTPUT_RESOLUTION_0) || \
                                                      ((__RESOLUTION__)EN_LNB_ABZ_OUTPUT_RESOLUTION_1) || \
                                                      ((__RESOLUTION__)EN_LNB_ABZ_OUTPUT_RESOLUTION_2) || \
                                                      ((__RESOLUTION__)EN_LNB_ABZ_OUTPUT_RESOLUTION_3) || \
                                                      ((__RESOLUTION__)EN_LNB_ABZ_OUTPUT_RESOLUTION_4) || \
                                                      ((__RESOLUTION__)EN_LNB_ABZ_OUTPUT_RESOLUTION_5) || \
                                                      ((__RESOLUTION__)EN_LNB_ABZ_OUTPUT_RESOLUTION_6) || \
                                                      ((__RESOLUTION__)EN_LNB_ABZ_OUTPUT_RESOLUTION_7))

#define EN_LNB_ABZ_OUTPUT_TRISTATE_ENABLE 0x08U
#define EN_LNB_ABZ_OUTPUT_TRISTATE_DISABLE 0x00U

#define IS_LNB_ABZ_OUTPUT_TRISTATE(__MODE__) (((__MODE__) == EN_LNB_ABZ_OUTPUT_TRISTATE_ENABLE) || \
                                              ((__MODE__) == EN_LNB_ABZ_OUTPUT_TRISTATE_DISABLE))

#define EN_LNB_SSI_OUTPUT_DATA_FORMAT_GRAY 0x00U
#define EN_LNB_SSI_OUTPUT_DATA_FORMAT_BIN 0x40U

#define IS_LNB_SSI_OUTPUT_DATA_FORMAT(__FORMAT__) (((__FORMAT__) == EN_LNB_SSI_OUTPUT_DATA_FORMAT_GRAY) || \
                                                   ((__FORMAT__) == EN_LNB_SSI_OUTPUT_DATA_FORMAT_BIN))

#define EN_LNB_SSI_IDLE_OUTPUT_ABSOLUTE_POSITION_MSB 0x00U
#define EN_LNB_SSI_IDLE_OUTPUT_HIGH 0x08U

#define IS_LNB_SSI_IDLE_OUTPUT(__OUTPUT__) (((__OUTPUT__) == EN_LNB_SSI_IDLE_OUTPUT_ABSOLUTE_POSITION_MSB) || \
                                            ((__OUTPUT__) == EN_LNB_SSI_IDLE_OUTPUT_HIGH))

#define EN_LNB_SSI_LENGTH_18BIT 0x00U /* 3-Bytes */
#define EN_LNB_SSI_LENGTH_17BIT 0x01U
#define EN_LNB_SSI_LENGTH_16BIT 0x02U /* 2-Bytes */
#define EN_LNB_SSI_LENGTH_15BIT 0x03U
#define EN_LNB_SSI_LENGTH_14BIT 0x04U
#define EN_LNB_SSI_LENGTH_13BIT 0x05U
// #define EN_LNB_SSI_LENGTH_13BIT 0x06U
#define EN_LNB_SSI_LENGTH_12BIT 0x07U

#define IS_LNB_SSI_LENGTH(__BIT__) (((__BIT__) == EN_LNB_SSI_LENGTH_18BIT) || \
                                    ((__BIT__) == EN_LNB_SSI_LENGTH_17BIT) || \
                                    ((__BIT__) == EN_LNB_SSI_LENGTH_16BIT) || \
                                    ((__BIT__) == EN_LNB_SSI_LENGTH_15BIT) || \
                                    ((__BIT__) == EN_LNB_SSI_LENGTH_14BIT) || \
                                    ((__BIT__) == EN_LNB_SSI_LENGTH_13BIT) || \
                                    ((__BIT__) == EN_LNB_SSI_LENGTH_13BIT) || \
                                    ((__BIT__) == EN_LNB_SSI_LENGTH_12BIT))

#define EN_LNB_SSI_ROTATION_DIRECTION_CW 0x00U
#define EN_LNB_SSI_ROTATION_DIRECTION_CCW 0x20U

#define IS_LNB_SSI_ROTATION_DIRECTION(__DIRECTION__) (((__DIRECTION__) == EN_LNB_SSI_ROTATION_DIRECTION_CW) || \
                                                      ((__DIRECTION__) == EN_LNB_SSI_ROTATION_DIRECTION_CCW))

#define EN_LNB_DEFAULT_SIGNAL_CALIBRATION 0x00U
#define EN_LNB_NO_DEFAULT_SIGNAL_CALIBRATION 0x01U

#define IS_LNB_DEFAULT_SIGNAL_CALIBRATION(__SIGNAL__) (((__SIGNAL__) == EN_LNB_DEFAULT_SIGNAL_CALIBRATION) || \
                                                       ((__SIGNAL__) == EN_LNB_NO_DEFAULT_SIGNAL_CALIBRATION))

#define EN_LNB_GAIN_RANGE_0 0x00U
#define EN_LNB_GAIN_RANGE_1 0x01U
#define EN_LNB_GAIN_RANGE_2 0x02U
#define EN_LNB_GAIN_RANGE_3 0x03U

#define IS_LNB_GAIN_RANGE(__RANGE__) (((__RANGE__) == EN_LNB_GAIN_RANGE_0) || \
                                      ((__RANGE__) == EN_LNB_GAIN_RANGE_1) || \
                                      ((__RANGE__) == EN_LNB_GAIN_RANGE_2) || \
                                      ((__RANGE__) == EN_LNB_GAIN_RANGE_3))

// #define EN_LNB_SIN_GAIN
#define IS_LNB_SIN_GAIN(__GAIN__) (((__GAIN__)-0x00U) <= (0x3FU - 0x00U))

// #define EN_LNB_POSITIVE_SIN_OFFSET
#define IS_LNB_POSITIVE_SIN_OFFSET(__OFFSET__) (((__OFFSET__)-0x00U) <= (0x7FU - 0x00U))

// #define EN_LNB_NEGATIVE_SIN_OFFSET
#define IS_LNB_NEGATIVE_SIN_OFFSET(__OFFSET__) (((__OFFSET__)-0x00U) <= (0x7FU - 0x00U))

// #define EN_LNB_COS_GAIN
#define IS_LNB_COS_GAIN(__GAIN__) (((__GAIN__)-0x00U) <= (0x3FU - 0x00U))

// #define EN_LNB_POSITIVE_COS_OFFSET
#define IS_LNB_POSITIVE_COS_OFFSET(__OFFSET__) (((__OFFSET__)-0x00U) <= (0x7FU - 0x00U))

// #define EN_LNB_NEGATIVE_COS_OFFSET
#define IS_LNB_NEGATIVE_COS_OFFSET(__OFFSET__) (((__OFFSET__)-0x00U) <= (0x7FU - 0x00U))

#define EN_LNB_DEFAULT_LED_POWER_CONTROL 0x00U
#define EN_LNB_NO_DEFAULT_LED_POWER_CONTROL 0x01U

#define IS_LNB_DEFAULT_LED_POWER_CONTROL(__POWER__) (((__POWER__) == EN_LNB_DEFAULT_LED_POWER_CONTROL) || \
                                                     ((__POWER__) == EN_LNB_NO_DEFAULT_LED_POWER_CONTROL))

#define EN_LNB_LED_POWER_CONTROL_MODE_CONTINUOUS 0x00U
#define EN_LNB_LED_POWER_CONTROL_MODE_DEADBAND 0x40U

#define IS_LNB_LED_POWER_CONTROL_MODE(__MODE__) (((__MODE__) == EN_LNB_LED_POWER_CONTROL_MODE_CONTINUOUS) || \
                                                 ((__MODE__) == EN_LNB_LED_POWER_CONTROL_MODE_DEADBAND))

#define EN_LNB_LED_POWER_CONTROL_TYPE_SQUARE 0x00U
#define EN_LNB_LED_POWER_CONTROL_TYPE_SUM 0x40U

#define IS_LNB_LED_POWER_CONTROL_TYPE(__TYPE__) (((__TYPE__) == EN_LNB_LED_POWER_CONTROL_TYPE_SQUARE) || \
                                                 ((__TYPE__) == EN_LNB_LED_POWER_CONTROL_TYPE_SUM))

#define EN_LNB_LED_POWER_CONTROL_SETPOINT(__POINT__) (((((uint32_t)__POINT__) - 0x00U) <= (0x3FU - 0x00U)) ? (__POINT__) : (0x20U))

#define IS_LNB_LED_POWER_CONTROL_SETPOINT(__POINT__) (((__POINT__) == EN_LNB_LED_POWER_CONTROL_SETPOINT(__POINT__)))

#define EN_LNB_TEST_MODE_DISABLE 0x00U
#define EN_LNB_TEST_MODE_ENABLE 0x10U

#define IS_LNB_TEST_MODE(__MODE__) (((__MODE__) == EN_LNB_TEST_MODE_DISABLE) || \
                                    ((__MODE__) == EN_LNB_TEST_MODE_ENABLE))

// #define EN_LNB_TEST_SIGANL_MUX
#define IS_LNB_TEST_SINAL_MUX(__SIGNAL__) (0U)

/* ************************************************************************** */
/*                               LNB SPI Opcode                               */
/* ************************************************************************** */

#define EN_LNB_SPI_OPCODE_ACTIVATE ((uint8_t)0xB0)
#define EN_LNB_SPI_OPCODE_POSITION_READ ((uint8_t)0xA6)
#define EN_LNB_SPI_OPCODE_POSITION_DATA_STATUS ((uint8_t)0xF5)
#define EN_LNB_SPI_OPCODE_REGISTER_READ ((uint8_t)0x8A)
#define EN_LNB_SPI_OPCODE_REGISTER_WRITE ((uint8_t)0xCF)
#define EN_LNB_SPI_OPCODE_REGISTER_STATUS ((uint8_t)0xAD)

#define IS_LNB_SPI_OPCODE(__OPCODE__) (((__OPCODE__) == EN_LNB_SPI_OPCODE_ACTIVATE) ||             \
                                       ((__OPCODE__) == EN_LNB_SPI_OPCODE_POSITION_READ) ||        \
                                       ((__OPCODE__) == EN_LNB_SPI_OPCODE_POSITION_DATA_STATUS) || \
                                       ((__OPCODE__) == EN_LNB_SPI_OPCODE_REGISTER_READ) ||        \
                                       ((__OPCODE__) == EN_LNB_SPI_OPCODE_REGISTER_WRITE) ||       \
                                       ((__OPCODE__) == EN_LNB_SPI_OPCODE_REGISTER_STATUS))

#define EN_LNB_SPI_STATUS_VALID_Pos (0U)
#define EN_LNB_SPI_STATUS_VALID_Msk (0x7FU << EN_LNB_SPI_STATUS_VALID_Pos)
#define EN_LNB_SPI_STATUS_VALID EN_LNB_SPI_STATUS_VALID_Msk

#define EN_LNB_SPI_STATUS_BUSY_Pos (1U)
#define EN_LNB_SPI_STATUS_BUSY_Msk (0x7FU << EN_LNB_SPI_STATUS_BUSY_Pos)
#define EN_LNB_SPI_STATUS_BUSY EN_LNB_SPI_STATUS_BUSY_Msk

#define EN_LNB_SPI_STATUS_FAIL_Pos (2U)
#define EN_LNB_SPI_STATUS_FAIL_Msk (0x7FU << EN_LNB_SPI_STATUS_FAIL_Pos)
#define EN_LNB_SPI_STATUS_FAIL EN_LNB_SPI_STATUS_FAIL_Msk

#define EN_LNB_SPI_STATUS_DISSMISS_Pos (3U)
#define EN_LNB_SPI_STATUS_DISSMISS_Msk (0x7FU << EN_LNB_SPI_STATUS_DISSMISS_Pos)
#define EN_LNB_SPI_STATUS_DISSMISS EN_LNB_SPI_STATUS_DISSMISS_Msk

#define EN_LNB_SPI_STATUS_ERROR_Pos (7U)
#define EN_LNB_SPI_STATUS_ERROR_Msk (0x7FU << EN_LNB_SPI_STATUS_ERROR_Pos)
#define EN_LNB_SPI_STATUS_ERROR EN_LNB_SPI_STATUS_ERROR_Msk

/* ************************************************************************** */
/*                             Function prototype                             */
/* ************************************************************************** */

HAL_StatusTypeDef EN_LNB_Init_Minimal(LNB_HandleTypeDef *hlnb);

HAL_StatusTypeDef EN_LNB_Init(LNB_HandleTypeDef *hlnb);

HAL_StatusTypeDef EN_LNB_DeInit(LNB_HandleTypeDef *hlnb);

HAL_StatusTypeDef EN_LNB_Enable(LNB_HandleTypeDef *hlnb);

HAL_StatusTypeDef EN_LNB_Position(LNB_HandleTypeDef *hlnb);

HAL_StatusTypeDef EN_LNB_CycleCount(LNB_HandleTypeDef *hlnb);

uint8_t even_parity(uint8_t n);

extern LNB_TypeDef LNB;

#endif
