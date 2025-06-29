#ifndef EEP_24LC64__H
#define EEP_24LC64__H


#include "main.h"
#include "EEPROM_MAP.h"


/* Control Byte Format for 24LC64 : 0b1010{A2}{A1}{A0}{W,R}U */
#define EEPROM_24LC64_DEVICE_ADDRESS_BASE 0xA0U
#define EEPROM_24LC64_DEVICE_ADDRESS_CHIP_SELECT_BIT 3U
#define EEPROM_24LC64_PAGE_WRITE_SIZE_MAX 32U
#define EEPROM_24LC64_ADDRESS_MAX 0x1FFF
#define EEPROM_24LC64_ADDRESS_MIN 0x0000
#define EEPROM_24LC64_ADDRESS_SIZE 13U


void EEPROM_24LC64_Write(I2C_HandleTypeDef hi2c_eep, uint16_t wr_address, uint8_t* wr_data, uint16_t wr_len);
void EEPROM_24LC64_Read (I2C_HandleTypeDef hi2c_eep, uint16_t rd_address, uint8_t* rd_data, uint16_t rd_len);


#endif
