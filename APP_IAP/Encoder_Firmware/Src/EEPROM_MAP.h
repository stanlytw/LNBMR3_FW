/* ***************************** EEPROM SUPPORT ***************************** */
#define EN_ADDR_MIN         0x00U
#define EN_ADDR_MAX         0x7EU

/**
 * @brief Check the ranage of address
 * @param ADDR Address from T-format request
*/
#define IS_EN_ADDR(ADDR)       (((ADDR) > 0) \
                             && ((((ADDR) > EN_ADDR_MIN) && ((ADDR) < EN_ADDR_MAX)) \
                                 || ((ADDR) == EN_ADDR_PAGE_SELECT)))

#define EN_ADDR_PAGE_SELECT 0xFFU
#define EN_ADDR_PAGE_SIZE   ((EN_ADDR_MAX - EN_ADDR_MIN) + 1U)
#define EN_ADDR_PAGE_MIN    0U
#define EN_ADDR_PAGE_MAX    5U
#define EN_ADDR_PAGE_0_BASE 0x0000U
#define EN_ADDR_PAGE_0_MAX  0x007FU
#define EN_ADDR_PAGE_1_BASE 0x0080U
#define EN_ADDR_PAGE_1_MAX  0x00FFU
#define EN_ADDR_PAGE_2_BASE 0x0100U
#define EN_ADDR_PAGE_2_MAX  0x017EU
#define EN_ADDR_PAGE_3_BASE 0x0180U
#define EN_ADDR_PAGE_3_MAX  0x01FFU
#define EN_ADDR_PAGE_4_BASE 0x0200U
#define EN_ADDR_PAGE_4_MAX  0x027FU
#define EN_ADDR_PAGE_5_BASE 0x0280U
#define EN_ADDR_PAGE_5_MAX  0x02FFU

/**
 * @brief Interpret the address from T-format request to onbord EEPROM
 * @param N Page number from T-format request
 * @param ADDR Address from T-format request
*/
#define EN_ADDR_PAGE_IN_PAGE(N, ADDR) (N)*(EN_ADDR_PAGE_SIZE + 1U) + (ADDR)

/**
 * @brief Check the ranage of page number 
 * @param N Page number from T-format request
*/
#define IS_EN_ADDR_PAGE(N)            ((N) > EN_ADDR_PAGE_MIN) && ((N) < EN_ADDR_PAGE_MAX)

/* NOTE: ID should be stored in one of the memory page above */
// #define EN_ADDR_ID_BASE 0x300U
// #define EN_ADDR_ID_MAX  0x30FU

/* ************************* POWER FAILURE FUNCTION ************************* */

#define EN_ADDR_ENCODER_ID         0x0300U
#define EN_ADDR_ZERO_POINT_ST_BASE 0x030DU
#define EN_ADDR_ZERO_POINT_ST_SIZE 3U
#define EN_ADDR_POWER_FAILURE_FLAG 0x0310U
#define EN_ADDR_ST_DATA_BASE       0x0311U
#define EN_ADDR_ST_DATA_SIZE       3U
#define EN_ADDR_MT_DATA_BASE       0x0314U
#define EN_ADDR_MT_DATA_SIZE       3U

#define IS_EN_ADDR_POWER_FAILURE_FLAG(FLAG) (((FLAG) == 1U) || ((FLAG)) == 0U))
