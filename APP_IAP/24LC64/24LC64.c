#include "24LC64.h"

/**
  * @brief  This function is used to write 24LC EEPROM
  * @retval None
  */
void EEPROM_24LC64_Write(I2C_HandleTypeDef hi2c_eep, uint16_t wr_address, uint8_t* wr_data, uint16_t wr_len)
{

	/* ## EEPROM Write */
	while(HAL_I2C_GetState(&hi2c_eep) != HAL_I2C_STATE_READY){} ;
 
    while (HAL_I2C_Mem_Write_IT(&hi2c_eep, EEPROM_24LC64_DEVICE_ADDRESS_BASE, wr_address, I2C_MEMADD_SIZE_16BIT, (uint8_t *)(wr_data), wr_len) != HAL_OK)
	
	{
     /* error */
    ;
    }

    while(HAL_I2C_GetState(&hi2c_eep) != HAL_I2C_STATE_READY) {};
}



/**
  * @brief  This function is used to read from 24LC EEPROM
  * @retval None
  */
void EEPROM_24LC64_Read(I2C_HandleTypeDef hi2c_eep, uint16_t rd_address, uint8_t* rd_data, uint16_t rd_len)
{

	/* ## Read Zero point */
	
    while (HAL_I2C_IsDeviceReady(&hi2c_eep, EEPROM_24LC64_DEVICE_ADDRESS_BASE, 30, 30) == HAL_TIMEOUT){};

    while(HAL_I2C_GetState(&hi2c_eep) != HAL_I2C_STATE_READY){} ;
  
    while (HAL_I2C_Mem_Read_IT(&hi2c_eep, EEPROM_24LC64_DEVICE_ADDRESS_BASE, rd_address, I2C_MEMADD_SIZE_16BIT, (uint8_t *)(rd_data), rd_len) != HAL_OK)
    {
     ;
    }
    while(HAL_I2C_GetState(&hi2c_eep) != HAL_I2C_STATE_READY){} ;
}


