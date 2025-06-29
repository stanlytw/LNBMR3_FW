void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//_______________________________________________//Instance==USART1
	if (huart->Instance == USART1)
	{
		 HAL_UART_Receive_IT(&huart1, &rx_data, 1);
		 //=========================================================//rx_index==0
		 if (rx_index==0)
		 {
		 //___________________________________________________//0xea
     if (rx_data == 0xea) 
		 {
			rx_buffer[rx_index] = rx_data; rx_index++; max_index=3;
		 }	
		 //___________________________________________________//0xea
		 //___________________________________________________//0x1a
     else if (rx_data == 0x1a) 
		 {
      if(HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS))
        CLEAR_BIT(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS);
      SET_BIT(encoderCF, ENCODER_COLLECTION_MODE_SINGLE);

      //__________________________________________//RS485 Communication Format
      //   Multiturn 16 bit data  (2 Bytes)
      //   Singleturn 23 bit data (3 Bytes)		
          //MultiT = (*((uint16_t *)&hmr3.Instance->CC_DATA.MT));
          MultiT = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
          // SingleT = (*((uint32_t *)&hmr3.Instance->CC_DATA.AR));
          SingleT = (*((uint32_t *)&PDBuffer.buffer[PDBuffer.front].AR));
          SingleT = (SingleT>>9);		
					CRC_Array[0]=0x1a; CRC_Array[1]=0x00; CRC_Array[5]=0x17; CRC_Array[9]=0x00;
			 
         //singleturn:
//______________________________________________________________//Not Reset
if (resetS_t==0)
{	
			CRC_Array[4] = (SingleT	>> 16) & 0xFF; 
			CRC_Array[3] = (SingleT	>> 8) & 0xFF;	 
			CRC_Array[2] =  SingleT & 0xFF;
}
//______________________________________________________________//Not Reset
//______________________________________________________________//Reset
else //resetS_t!=0
{
	     DSingle=SingleT-ZSingleT;
	     if (DSingle<0){DSingleT = 0x7fffff + DSingle;}
			 else{DSingleT = DSingle;}
			 DSingleT = DSingleT & 0x007fffffU;
	     CRC_Array[4] = (DSingleT	>> 16) & 0xFF; 
       CRC_Array[3] = (DSingleT	>> 8) & 0xFF;	 
       CRC_Array[2] =  DSingleT & 0xFF;
}
//______________________________________________________________//Reset
      //multiturn:
			CRC_Array[8] = (MultiT	>> 16) & 0xFF; 
			CRC_Array[7] = (MultiT	>> 8) & 0xFF;	 
			CRC_Array[6] =  MultiT & 0xFF;	

      TX_Array[0]=CRC_Array[0]; TX_Array[1]=CRC_Array[1]; TX_Array[5]=CRC_Array[5];
      TX_Array[2]=CRC_Array[2]; TX_Array[3]=CRC_Array[3]; TX_Array[4]=CRC_Array[4];
      TX_Array[6]=CRC_Array[6]; TX_Array[7]=CRC_Array[7]; TX_Array[8]=CRC_Array[8];
      TX_Array[9]=CRC_Array[9];

			rx_buffer[rx_index] = rx_data; max_index=1; resetSTurn=0;
      TX_Array[10] = HAL_CRC_Calculate(&hcrc, (uint32_t*)CRC_Array, 10);			 
      HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TX_Array, 11);

      if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_SINGLE))
      {
        if(!dataCollectionCF)
          SET_BIT(dataCollectionCF, DATA_COLLECTION_COLLECTION);
      }

		 }	
		 //___________________________________________________//0x1a
		 //___________________________________________________//0xc2
		 else if (rx_data == 0xc2)
		 {
        if(HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS))
          CLEAR_BIT(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS);
        SET_BIT(encoderCF, ENCODER_COLLECTION_MODE_SINGLE);

      /* PREPARING PART - START */
        //   Multiturn 16 bit data  (2 Bytes)
        //   Singleturn 23 bit data (3 Bytes)		
				//MultiT = (*((uint16_t *)&hmr3.Instance->CC_DATA.MT));
			  MultiT = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
				//SingleT = (*((uint32_t *)&hmr3.Instance->CC_DATA.AR));
			  SingleT = (*((uint32_t *)&PDBuffer.buffer[PDBuffer.front].AR));
				SingleT = (SingleT>>9);	

				CRC_Array[0]=0xc2; CRC_Array[1]=0x00;
        //==============================================================================//resetSTurn==1
        if (resetSTurn==0){ZSingleT=SingleT; resetSTurn=1; resetS_t=0;}
        //==============================================================================//resetSTurn==1
        //==============================================================================//resetSTurn==2
        if (resetSTurn==1)
        {
				 DSingle=SingleT-ZSingleT;
				 if (DSingle<0){DSingleT = 0x7fffff + DSingle;}
				 else{DSingleT = DSingle;}
         DSingleT = DSingleT & 0x007fffffU;
        //singleturn: 
				CRC_Array[4] = (DSingleT	>> 16) & 0xFF; 
				CRC_Array[3] = (DSingleT	>> 8) & 0xFF;	 
				CRC_Array[2] =  DSingleT & 0xFF;	
        }
        //==============================================================================//resetSTurn==2

      /* PREPARING PART - STOP */
		  rx_buffer[rx_index] = rx_data; max_index=1; 
      TX_RArray[0]=CRC_Array[0]; TX_RArray[1]=CRC_Array[1]; TX_RArray[2]=CRC_Array[2]; 
			TX_RArray[3]=CRC_Array[3]; TX_RArray[4]=CRC_Array[4];	
			TX_RArray[5] = HAL_CRC_Calculate(&hcrc, (uint32_t*)CRC_Array, 5);		
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TX_RArray, 6);
  
      if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_SINGLE))
      {
        if(!dataCollectionCF)
          SET_BIT(dataCollectionCF, DATA_COLLECTION_COLLECTION);
      }

			resetS_t++;
		 }
		 //___________________________________________________//0xc2
		 else{}
		 }
		 //=========================================================//rx_index==0
		 //=========================================================//rx_index>0
		 else//rx_index>0
		 {
			 rx_buffer[rx_index] = rx_data; rx_index++;

       if (rx_index >= max_index)
       {
       //==================================================//0xea 0x00 				 
			 if (rx_buffer[1]==0x00)
       {
			 HAL_UART_Transmit(&huart1, send2, 4, 0xFFFF);	
			 } 
       //==================================================//0xea 0x00 
       //==================================================//0xea 0x01 			 
			 if (rx_buffer[1]==0x01)
       {
			 HAL_UART_Transmit(&huart1, send3, 4, 0xFFFF);	
			 } 	
       //==================================================//0xea 0x01 				 			
       rx_index=0;			
			 }
		 }
     //=========================================================//rx_index>0	
      if (rx_index==0)
      {
			 ii=0; for (ii=0; ii<max_index; ii++){rx_buffer[ii]=0;}
			}				
	}
	//_______________________________________________//Instance==USART1
}
