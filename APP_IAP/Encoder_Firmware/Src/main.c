/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include "en_mr3.h"
#include "en_mr3_reg.h"
#include "en_lnb.h"
#include "en_lnb_reg.h"
#include "TMP116.h"
#include "24LC64.h"
#include "EEPROM_MAP.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum _encoderState EncoderState;
typedef enum _exceptionHandlingState ExceptionHandlingState;
typedef enum _dataCollectionState DataCollectionState;
typedef enum _monitoringState MonitoringState;
typedef enum _absDataSyncState AbsDataSyncState;
typedef enum _tfState TfState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Firmware States */
enum _exceptionHandlingState {
  EXCEPTION_HANDLING_STATE_IDLE = 0,
  EXCEPTION_HANDLING_STATE_POWER_FAILURE,
  EXCEPTION_HANDLING_STATE_POWER_FAILURE_RETURN,
  EXCEPTION_HANDLING_STATE_OVER_SPEED
};

enum _dataCollectionState {
  DATA_COLLECTION_STATE_IDLE = 0,
  DATA_COLLECTION_STATE_COLLECTION,
  DATA_COLLECTION_STATE_ENQUEUE,
};

enum _monitoringState {
  MONITORING_STATE_IDLE = 0,
  MONITORING_STATE_VOLTAGE,
  MONITORING_STATE_ROTATION,
  MONITORING_STATE_TMP_ALM
};

enum _encoderState {
  ENCODER_STATE_IDLE = 0,
  ENCODER_STATE_EXCEPTION_HANDLING,
  ENCODER_STATE_DATA_COLLECTION,
  ENCODER_STATE_MONITORING,
  ENCODER_STATE_ABSDATASYNC
};

enum _absDataSyncState
{
  ABS_DATA_SYNC_STATE_IDLE = 0,
  ABS_DATA_SYNC_STATE_PENDING,
  ABS_DATA_SYNC_STATE_RUNNING,
  ABS_DATA_SYNC_STATE_TIMEOUT
};

enum _tfState
{
  TF_STATE_READY = 0,
  TF_STATE_BUSY,
  TF_STATE_TIMEOUT
};

struct encoderPositionDataBuffer
{
  uint32_t front;
  uint32_t tail;
  uint32_t len;
  MR3_CyclicRead_Buffer buffer[2];
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* encoder condition flag */

#define ENCODER_SYSTEM_RESET  0x00000001U
#define ENCODER_TF_LATE       0x00000002U
#define ENCODER_TMP_ALM       0x00000004U
#define ENCODER_POWER_FAILURE 0x00000008U
#define ENCODER_OVERSPEED     0x00000010U
#define ENOCDER_COUNTING_ERR  0x00000020U
#define ENCODER_MT_ERR        0x00000040U
#define ENCODER_VBAT_WARN     0x00000080U
#define ENCODER_VBAT_ERR      0x00000100U
#define ENCODER_PD_INVALID    0x00000200U
#define ENCODER_COLLECTION_MODE_CONTINUOUS 0x00000400U
#define ENCODER_COLLECTION_MODE_SINGLE     0x00000800U

/* data collection thread condition flag */

// #define DATA_COLLECTION_IDLE       0x00000001U
#define DATA_COLLECTION_COLLECTION       0x00000001U
#define DATA_COLLECTION_ENQUEUE          0x00000002U
#define DATA_COLLECTION_DISABLE          0x00000004U

/* Monitoring thread condition flag */

#define MONITORING_ROTATION 0x00000001U
#define MONITORING_VOLTAGE  0x00000002U
#define MONITORING_TMP_ALM  0x00000004U

/* abs data Sync thread condition flag */
#define ABS_DATA_SYNC_ACTIVATE     0x00000001U
#define ABS_DATA_SYNC_TF_FINISHED  0x00000002U
#define ABS_DATA_SYNC_TIMEOUT      0x00000004U
#define ABS_DATA_SYNC_PREPARING    0x00000008U

/* Exception Handling thread condition flag */

#define EXCEPTION_HANDLING_POWER_FAILURE        0x00000001U
#define EXCEPTION_HANDLING_POWER_FAILURE_RETURN 0x00000002U
#define EXCEPTION_HANDLING_OVER_SPEED                    0x00000004U

/** @brief Uncomment the following line to activate DEBUG_MODE
 * 
 * To read Positiion data from LNB
 */

#define DEBUG_MODE 
//#define MEM_DUMP
//#define FIX_LNB_MSB
//#define MR3_ST

//#define MLT_BY_FW

//#define MR3_ST_23PAD5B0


//#define SCHEDULER_FULLFUNCTION
/**/
#define IQC
#define LED_POWER_CONTROL_CONFIG

/**
 * @brief
*/

#define DEBUG_MODE_CONTINUOUS
#define DBG_MSG printf
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

MR3_HandleTypeDef hmr3;

LNB_HandleTypeDef hlnb;

volatile uint8_t MR3_RST_flag;

/* Encoder */
volatile EncoderState encoderState = ENCODER_STATE_IDLE;
volatile uint32_t encoderCF = 0;
volatile ExceptionHandlingState exceptionHandlingState = EXCEPTION_HANDLING_STATE_IDLE;
volatile uint32_t exceptionHandlingCF = 0;
volatile DataCollectionState dataCollectionState = DATA_COLLECTION_STATE_IDLE;
volatile uint32_t dataCollectionCF = 0;
volatile MonitoringState monitoringState = MONITORING_STATE_IDLE;
volatile uint32_t monitoringCF = 0;
volatile AbsDataSyncState absDataSyncState = ABS_DATA_SYNC_STATE_IDLE;
volatile uint32_t absDataSyncCF = 0;

struct encoderPositionDataBuffer PDBuffer;

/* Estimation*/
volatile uint32_t latency = 0;
volatile uint32_t latency_max = 0;
volatile uint32_t latency_max_count = 0;

volatile uint32_t htmp = 0;
volatile uint32_t hlatency = 0;
volatile uint32_t hlatency_max = 0;

volatile uint32_t tpa = 0;
volatile uint32_t tpb = 0;

volatile uint32_t latency1_tmp = 0;
volatile uint32_t latency1 = 0;
volatile uint32_t latency1_datacollect = 0;

volatile uint32_t err_ipo_count = 0;
volatile uint32_t err_unsync_count = 0;
volatile uint32_t err_count = 0;
volatile uint32_t err_PDV_count = 0;
/* commandReactionThread */
uint32_t commandReactionThread_cycle_count = 0;

/* monitoring */
uint32_t betteryVoltageRaw = 0U;
float betteryVoltageRes = 0.000806;
float betteryVoltage = 0;

/* exceptionHandlingThread */
struct PositionDataBuf
{
  uint8_t AR[4];
  uint8_t MT[3];
} PFPDBuf = {0};

/* For test*/

uint32_t LNB_AR_RAW = 0;
uint32_t LNB_ST_Offset = 0;
double LNB_AR = 0;
double LNB_AR_k = 0.0439453; /* 13-bit */

uint32_t MR3_AR_RAW = 0;
double MR3_AR = 0;
double MR3_AR_k = 0.0000429; /* 23-bit */

uint32_t latencyddd = 0;
uint32_t latencyddd_tmp = 0;

// RS485 Communication
uint8_t rx_data=0; 
uint8_t rx_buffer[3];
uint8_t rx_index=0, max_index=3, ii=0;
uint8_t send1[] = {0x1a, 0x00, 0x81, 0x06, 0x22, 0x17, 0x02, 0x00, 0x00, 0x00, 0xaa};
uint8_t sendR[] = {0xc2, 0x00, 0x00, 0x00, 0x00, 0xc2};
uint8_t send2[] = {0xea, 0x00, 0x8d, 0x67};
uint8_t send3[] = {0xea, 0x01, 0x10, 0xfb};
uint8_t TX_Array[11], CRC_Array[10], TX_RArray[6];
uint32_t SingleT=0, L_SingleT=0, SingleT_AVG=0, SingleT_AVGout=0,ZSingleT=0,ZSingleT_W=0,ZSingleT_R=0, DSingleT=0 ;
uint32_t MultiT=0, L_MultiT=0, ZMultiT=0;
uint32_t SingleTBuf[16] = {0};
uint32_t SingleTBuf_idx = 0;
uint32_t resetMTurn = 0;
uint32_t MultiT2 = 0, SingleT2 = 0;
int64_t DSingle=0;
uint8_t resetSTurn=0, resetS_t=0;
uint32_t DSingleT_emu = 0, L_SingleT_emu;
volatile uint8_t DSingleT_MSB_emu = 0x01;
volatile uint32_t SingleT_emu = 0;

uint8_t zeross[4] = {0};
uint32_t LNB_ST_BUF[4096] ={0};
uint32_t LNB_ST_BUF_Idx = 0;
uint32_t MR3_ST_BUF[4096] ={0};
uint32_t MR3_ST_BUF_Idx = 0;
volatile uint32_t MR3_SPI_TxCplt_counter = 0;
volatile uint32_t OverSpeedSim = 0;

volatile uint32_t tmp_mr3_cycread = 0;
volatile uint32_t latency_mr3_cycread = 0;
volatile uint32_t tformat_last = 0;
volatile uint32_t tformat_current = 0;
volatile uint32_t tformat_latency = 0;
volatile uint32_t fsm_latency = 0;
volatile uint32_t mr3_spi_tx_count = 0;

volatile uint32_t clockofpositionstatrt = 0;
volatile uint32_t clockofpositionstop = 0;
volatile uint32_t clockofpositionstop2 = 0;
HAL_StatusTypeDef res;
uint8_t EEPROMQC_OK =0, TMPQC_OK = 0, PORQC_OK = 0, BATQC_OK = 0, LNBQC_OK = 0 , SWDLQC_OK = 0 ,MR3QC_OK = 0;
uint8_t IQC_buffer[32] = {0};
volatile uint8_t mem_dump_request = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void LNB_Init(void);
static HAL_StatusTypeDef MR3_Init(void);
static void MR3_Reset(void);

/* callback function prototype of TIMx */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM2_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM3_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM4_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM6_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM9_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
/* Function prototype of threads */
void Scheduler(void);
void dataCollectionThread(void);
void monitoringThread(void);
void exceptionHandlingThread(void);
void absDataSyncThread(void);

void eh_PowerFailureReturnHandler(void);
void eh_PowerFailureHandler(void);
void eh_OverSpeedHandler(void);
void eh_OverSpeedReturnHandler(void);

void MR3_AbsDataSync(void);
// uint64_t rev64(uint64_t val);

int isOverSpeed(uint32_t st1, uint32_t mt1, uint32_t st2, uint32_t mt2,uint32_t threshold, uint32_t st_max, uint32_t mt_max);
void DUMP_Memory(void);
#ifdef IQC
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CRC_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  //MX_TIM3_Init();
  //MX_ADC2_Init();
  //MX_TIM2_Init();
  //MX_TIM4_Init();
  //MX_TIM8_Init();
  //MX_TIM9_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  /* POR PROCCESS START */
  
  /* ## 1. Set EN_BOOST */
  //HAL_GPIO_WritePin(EN_BOOST_GPIO_Port, EN_BOOST_Pin, GPIO_PIN_SET);

  /* ## 2. Check LNB_OK */
  while (HAL_GPIO_ReadPin(LNB_OK_GPIO_Port, LNB_OK_Pin) != GPIO_PIN_SET)
    ;
  //DBG_MSG("LNB_OK Pin Status Check Pass.\r\n");
	
  LNB_Init();

  /* ## 3. Reset MR3 */
  
  MR3_Reset();
  
  /* ## 4. Initialization of MR3, LNB */

  res = MR3_Init();
  if(res == HAL_OK) MR3QC_OK =1;
  //DBG_MSG("MCU, LNB, MR3 Initialization End\r\n");
  /* POR PROCCESS FINISH */
  
  /* TEST START */
  
  /* TEST FINISH */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // __EN_MR3_SPI_ADI_READ_DATA(&hmr3);

	uint8_t zeros[4] = {0};
	
	EN_MR3_SetSingleTurnOffset(&hmr3, (uint8_t *)&zeros);	

  zeros[0] = 0xff;
	zeros[1] = 0xff;
  EN_MR3_SetMultiTurnOffset(&hmr3, (uint8_t *)&zeros);
	
  MR3_AbsDataSync();
	
  
  
	/* ## EEPROM CHK */
	/* ## Trail Write Zero point offset */
	ZSingleT_W= 1000;
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;

  //while (HAL_I2C_Mem_Write_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE,EN_ADDR_ZERO_POINT_ST_BASE, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&(zeross), 3) != HAL_OK)
  while (HAL_I2C_Mem_Write_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE,EN_ADDR_ZERO_POINT_ST_BASE, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&(ZSingleT_W), 3) != HAL_OK)
	
	{
     /* error */
    ;
  }

  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;
  
	/* ## Read Zero point */
	//DBG_MSG("EEPROM Functionality Check.\r\n");
  while (HAL_I2C_IsDeviceReady(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, 30, 30) == HAL_TIMEOUT);

  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;
  
  while (HAL_I2C_Mem_Read_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, EN_ADDR_ZERO_POINT_ST_BASE, I2C_MEMADD_SIZE_16BIT, (uint8_t *) &(ZSingleT_R), 3) != HAL_OK)
  {
    ;
  }
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;

	if( ZSingleT_R == ZSingleT_W) EEPROMQC_OK = 1;
	
	
	/* ## TMP116 CHK */
	
  //while (HAL_I2C_IsDeviceReady(&hi2c3, TMP116_DEVICE_ADDRESS_BASE, 30, 30) == HAL_TIMEOUT);
	do{
			res = HAL_I2C_IsDeviceReady(&hi2c3, TMP116_DEVICE_ADDRESS_BASE, 30, 100);
	}while (res == HAL_TIMEOUT);
	TMPQC_OK = 1;
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;
  
 
	
	
	
  
	/*
		TO DO: Load MultiTurn form EEPROM, 
	  Then Recover MultiTurn to last value
	  MultiT = ZMultiTurn;
	*/
  // RS485 Communication Initialization
	//1a 00 00 00 00 17 00 00 00 00
  resetS_t=0;
	TX_Array[0]=0x1a; TX_Array[1]=0x00; TX_Array[5]=0x17; TX_Array[9]=0x00; TX_Array[10]=0x0d;
	//TX_Array[2~4]= single turn //singleturn:  
	TX_Array[2]=0x00; TX_Array[3]=0x00; TX_Array[4]=0x00;
	//TX_Array[6-8]= multi turn //multiturn: 	
	TX_Array[6]=0x00; TX_Array[7]=0x00; TX_Array[8]=0x00;	
	ii=0; for(ii=0; ii<10; ii++){CRC_Array[ii]=TX_Array[ii];}


  encoderState = ENCODER_STATE_IDLE;

  dataCollectionState = DATA_COLLECTION_STATE_IDLE;
  monitoringState = MONITORING_STATE_IDLE;
  exceptionHandlingState = EXCEPTION_HANDLING_STATE_IDLE;
  
  
  // HAL_ADC_Start_IT(&hadc2);  
  //HAL_TIM_Base_Start(&htim8);
  
  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim3);
  // HAL_TIM_Base_Start_IT(&htim4); /* disable abs sync */
  // HAL_TIM_Base_Start_IT(&htim9);

  PDBuffer.front = 0U;
  PDBuffer.tail = 1U;
  
  uint32_t tmp = 0;
  // tpa = DWT->CYCCNT;
	
  eh_PowerFailureReturnHandler();
  
  SET_BIT(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS);
	IQC_buffer[0] = 0x65;
	IQC_buffer[1] = IQC_buffer[2] = IQC_buffer[3] = IQC_buffer[4] =0;
	
  while (1)
  { 
    /* USER CODE END WHILE */
#ifndef MEM_DUMP
    /* USER CODE BEGIN 3 */
    //DBG_MSG("FSM for encoderState!\r\n");
		HAL_Delay(20);
		PORQC_OK = HAL_GPIO_ReadPin(VM_VP_GPIO_Port, VM_VP_Pin);//?
		BATQC_OK = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);//?
		SWDLQC_OK = HAL_GPIO_ReadPin(IAP_DL_GPIO_Port, IAP_DL_Pin);//?
		IQC_buffer[5]  = EEPROMQC_OK;
		IQC_buffer[6]  = TMPQC_OK;
		IQC_buffer[7]  = LNBQC_OK;
		IQC_buffer[8]  = MR3QC_OK;
		IQC_buffer[9]  = PORQC_OK;
		IQC_buffer[10] = BATQC_OK;
		IQC_buffer[11] = SWDLQC_OK;
		  
		//LNB_AR_RAW:
		IQC_buffer[12] = (uint8_t)(LNB_AR_RAW&0x000000ff);	IQC_buffer[13] = (uint8_t)((LNB_AR_RAW>>8)&0x000000ff);	
		IQC_buffer[14] = (uint8_t)((LNB_AR_RAW>>16)&0x000000ff); IQC_buffer[15] = 0x00;
		//MR3_AR_RAW:
		IQC_buffer[16] = (uint8_t)(MR3_AR_RAW&0x000000ff);	IQC_buffer[17] = (uint8_t)((MR3_AR_RAW>>8)&0x000000ff);	
		IQC_buffer[18] = (uint8_t)((MR3_AR_RAW>>16)&0x000000ff); IQC_buffer[19] = 0x00;
		//MultiT
		IQC_buffer[20] = (uint8_t)(MultiT&0x000000ff);	IQC_buffer[21] = (uint8_t)((MultiT>>8)&0x000000ff);	
		//TAIL
		IQC_buffer[22] = 0x0A;
		
		HAL_UART_Transmit(&huart1, (uint8_t *)IQC_buffer, 23, 0xFFFF);
#endif
		switch (encoderState)
    {
    case ENCODER_STATE_IDLE:
      /* decide which thread to execute*/

      tmp = DWT->CYCCNT;
      tpa = DWT->CYCCNT;

      Scheduler();
      
      break;
    case ENCODER_STATE_EXCEPTION_HANDLING:

      /* ESTIMATION START */
      latency = DWT->CYCCNT - tmp;
      tpb = DWT->CYCCNT - tpa;

      if (latency_max < latency) 
      { 
        latency_max = latency;
        latency_max_count ++;
      }
      /* ESTIMATION STOP */

      //exceptionHandlingThread();

      /* Reset encoder state */
      encoderState = ENCODER_STATE_IDLE;
      break;
    case ENCODER_STATE_DATA_COLLECTION:

      /* ESTIMATION START */
      latency = DWT->CYCCNT - tmp;
      tpb = DWT->CYCCNT - tpa;

      if (latency_max < latency) 
      { 
        latency_max = latency;
        latency_max_count ++;
      }
      /* ESTIMATION STOP */

      dataCollectionThread();

      /* Reset encoder state */
      encoderState = ENCODER_STATE_IDLE;
      break;
    case ENCODER_STATE_MONITORING:

      /* ESTIMATION START */
      latency = DWT->CYCCNT - tmp;
      tpb = DWT->CYCCNT - tpa;

      if (latency_max < latency) 
      { 
        latency_max = latency;
        latency_max_count ++;
      }
      /* ESTIMATION STOP */

      // monitoringThread();

      /* Reset encoder state */
      encoderState = ENCODER_STATE_IDLE;
      break;

      // case ENCODER_STATE_ABSDATASYNC:

      // /* ESTIMATION START */
      // latency = DWT->CYCCNT - tmp;
      // tpb = DWT->CYCCNT - tpa;

//      if (latency_max < latency) 
//      { 
//        latency_max = latency;
//        latency_max_count ++;
//      }
//      /* ESTIMATION STOP */

//      // monitoringThread();

//      /* Reset encoder state */
//      encoderState = ENCODER_STATE_IDLE;
//      break;

//    
//    default:
//      /* Reset encoder state */
//      encoderState = ENCODER_STATE_IDLE;
//      break;
  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C3;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 1;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.InitValue = 0;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00909BEB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 119;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 59999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 159;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 79;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 99;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;//2500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);//SPI1_Rx
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);//SPI1_Tx
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1, 1);//USART1_Tx
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D5_Pin|CS1_Pin|CS2_V1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(IAP_DL_GPIO_Port, IAP_DL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSI_NSL_GPIO_Port, SSI_NSL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TMP_ALM_Pin INCB_Pin */
  GPIO_InitStruct.Pin = TMP_ALM_Pin|INCB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D5_Pin CS1_Pin CS2_V1_Pin */
  GPIO_InitStruct.Pin = D5_Pin|CS1_Pin|CS2_V1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : IAP_DL_Pin */
  GPIO_InitStruct.Pin = IAP_DL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IAP_DL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INCA_Pin INCZ_Pin LNB_OK_Pin MR3_RST_Pin */
  GPIO_InitStruct.Pin = INCA_Pin|INCZ_Pin|LNB_OK_Pin|MR3_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SSI_NSL_Pin */
  GPIO_InitStruct.Pin = SSI_NSL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSI_NSL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS2_Pin MR3_NL_Pin */
  GPIO_InitStruct.Pin = CS2_Pin|MR3_NL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SSI_CLK_Pin */
  GPIO_InitStruct.Pin = SSI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SSI_CLK_GPIO_Port, &GPIO_InitStruct);
	
	
	/*Configure GPIO pin : VM_VP_Pin */
  GPIO_InitStruct.Pin = VM_VP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VM_VP_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOA, MR3_NL_Pin, GPIO_PIN_SET);//
  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 1);
  //HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void Scheduler(void)
{
  /* reset DWT */
  TIM2->CNT = 0;

//  if(HAL_IS_BIT_SET(encoderCF, ENCODER_POWER_FAILURE) && 
//     HAL_IS_BIT_CLR(exceptionHandlingCF, EXCEPTION_HANDLING_POWER_FAILURE))
//    // SET_BIT(exceptionHandlingCF, EXCEPTION_HANDLING_POWER_FAILURE);
//  ;
//  else if (HAL_IS_BIT_SET(encoderCF, ENCODER_OVERSPEED) && 
//           HAL_IS_BIT_SET(exceptionHandlingCF, EXCEPTION_HANDLING_OVER_SPEED))
//  {
//    // SET_BIT(exceptionHandlingCF, EXCEPTION_HANDLING_OVER_SPEED);
//    ;
//  }

//  if(exceptionHandlingCF)
//  {
//    if(HAL_IS_BIT_SET(exceptionHandlingCF, EXCEPTION_HANDLING_POWER_FAILURE))
//    {
//      CLEAR_BIT(exceptionHandlingCF, EXCEPTION_HANDLING_POWER_FAILURE);
//      exceptionHandlingState = EXCEPTION_HANDLING_STATE_POWER_FAILURE;
//    }
//    else if(HAL_IS_BIT_SET(exceptionHandlingCF, EXCEPTION_HANDLING_OVER_SPEED))
//    {
//      CLEAR_BIT(exceptionHandlingCF, EXCEPTION_HANDLING_OVER_SPEED);
//      exceptionHandlingState = EXCEPTION_HANDLING_STATE_OVER_SPEED;
//    }
//    else
//    {
//      exceptionHandlingState = EXCEPTION_HANDLING_STATE_IDLE;
//    }
//    encoderState = ENCODER_STATE_EXCEPTION_HANDLING;
//  }
//  else if (absDataSyncCF)
//  {
//    if(HAL_IS_BIT_SET(absDataSyncCF, ABS_DATA_SYNC_TIMEOUT))
//    {
//      CLEAR_BIT(absDataSyncCF, ABS_DATA_SYNC_STATE_TIMEOUT);
//      absDataSyncState = ABS_DATA_SYNC_STATE_IDLE;
//       /* drop */
//    }
//    else if(HAL_IS_BIT_SET(absDataSyncCF, ABS_DATA_SYNC_ACTIVATE))
//    {
//      CLEAR_BIT(absDataSyncCF, ABS_DATA_SYNC_ACTIVATE | ABS_DATA_SYNC_TF_FINISHED);
//      absDataSyncState = ABS_DATA_SYNC_STATE_RUNNING;
//      encoderState = ENCODER_STATE_ABSDATASYNC;
//    }
//  }
//  else if (dataCollectionCF)
	if (dataCollectionCF)
  {
    /* Activate right after tf communication, to make sure no tf interrupt will come. */
    if (HAL_IS_BIT_SET(dataCollectionCF, DATA_COLLECTION_COLLECTION))
    {
      CLEAR_BIT(dataCollectionCF, DATA_COLLECTION_COLLECTION);
      dataCollectionState = DATA_COLLECTION_STATE_COLLECTION;
    }
    else if (HAL_IS_BIT_SET(dataCollectionCF, DATA_COLLECTION_ENQUEUE))
    {
      CLEAR_BIT(dataCollectionCF, DATA_COLLECTION_ENQUEUE);
      dataCollectionState = DATA_COLLECTION_STATE_ENQUEUE;
    }
    else
    {
      ;
    }
    encoderState = ENCODER_STATE_DATA_COLLECTION;
  }
//  else if (monitoringCF)
//  {
//    if (HAL_IS_BIT_SET(monitoringCF, MONITORING_ROTATION))
//    {
//      CLEAR_BIT(monitoringCF, MONITORING_ROTATION);
//      monitoringState = MONITORING_STATE_ROTATION;
//    }
//    else if (HAL_IS_BIT_SET(monitoringCF, MONITORING_VOLTAGE))
//    {
//      CLEAR_BIT(monitoringCF, MONITORING_VOLTAGE);
//      //monitoringState = MONITORING_STATE_VOLTAGE;
//    }
//    else if (HAL_IS_BIT_SET(monitoringCF, MONITORING_TMP_ALM))
//    {
//      CLEAR_BIT(monitoringCF, MONITORING_TMP_ALM);
//      monitoringState = MONITORING_STATE_TMP_ALM;
//    }
//    else
//    {
//      ;
//    }
//    encoderState = ENCODER_STATE_MONITORING;
//  }
#ifdef DEBUG_MODE
  else if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS))
  {
    if ((absDataSyncState != ABS_DATA_SYNC_STATE_RUNNING) && HAL_IS_BIT_CLR(absDataSyncCF, ABS_DATA_SYNC_PREPARING))
      SET_BIT(dataCollectionCF, DATA_COLLECTION_COLLECTION);
  }
#endif	
}


void dataCollectionThread(void)
{
  //volatile uint32_t clockofpositionstatrt = 0;
  //volatile uint32_t clockofpositionstop = 0;
    switch(dataCollectionState)
    {
      case DATA_COLLECTION_STATE_IDLE:

        /* Reset data collection state */
        dataCollectionState = DATA_COLLECTION_STATE_IDLE;

        break;
      case DATA_COLLECTION_STATE_COLLECTION:
      
        /* Update position data */
        //latency1 = DWT->CYCCNT - latency1_tmp;
//			  latency1 = DWT->CYCCNT - latency1_datacollect;
//       EN_MR3_CyclicRead(&hmr3, (uint8_t *)&(PDBuffer.buffer[PDBuffer.front]));
			  latency1_datacollect = DWT->CYCCNT - latency1;
#ifdef DEBUG_MODE
				clockofpositionstatrt = DWT->CYCCNT;
				
        
				
				res = EN_LNB_Position(&hlnb);
			  if(res==HAL_OK) LNBQC_OK =1;
        clockofpositionstop2 = DWT->CYCCNT - clockofpositionstatrt;

			
			  //LNB vs MR3
				//LNB_AR_RAW = (hlnb.Instance->AR & 0x00FFFFFF)<<5;
				EN_MR3_CyclicRead(&hmr3, (uint8_t *)&(PDBuffer.buffer[PDBuffer.front]));
#ifdef FIX_LNB_MSB
			
				
				SingleT_emu = LNB_AR_RAW;
						
				if (SingleT_emu >= L_SingleT_emu)
				{
						DSingleT_emu = SingleT_emu - L_SingleT_emu;
						if (DSingleT_emu > 0x20000U)//90 deg@ 23-bit
						{
								DSingleT_MSB_emu =~ DSingleT_MSB_emu; 
								DSingleT_MSB_emu &= 0x01;
								DSingleT_emu=0;
						}
				}
				else
				{
						DSingleT_emu = L_SingleT_emu - SingleT_emu;
						if (DSingleT_emu > 0x20000U)//90 deg@ 23-bit
						{
								DSingleT_MSB_emu =~ DSingleT_MSB_emu; 
							  DSingleT_MSB_emu &= 0x01;
								DSingleT_emu=0;
						}
				}
				L_SingleT_emu = SingleT_emu ;
				SingleT_emu |= (DSingleT_MSB_emu<<22);
#endif			
				//SingleT = LNB_AR_RAW - LNB_ST_Offset;
				DSingle = LNB_AR_RAW - LNB_ST_Offset;
        if (DSingle < 0)
        {
             DSingleT = 0x7fffff + DSingle;
        }
        else
        {
             DSingleT = DSingle;
        }
        DSingleT = DSingleT & 0x007fffffU;
				SingleT = DSingleT  & 0x007fffe0U;
			
			
        //LNB_AR_RAW = (hlnb.Instance->AR & 0x00FFFFFF)<<14;
				//LNB_AR_RAW = (hlnb.Instance->AR & 0x00FFFFFF)<<5;
        //LNB_AR = ((hlnb.Instance->AR & 0x00FFFFFF)>>5) * LNB_AR_k;

        //MR3_AR_RAW = (*((uint32_t *)&(PDBuffer.buffer[PDBuffer.tail].AR)));
				//MR3_AR_RAW >>=9;
#ifdef MEM_DUMP				
if(mem_dump_request)
{	
#endif
	if(LNB_ST_BUF_Idx<4096) 
				{
						LNB_ST_BUF[LNB_ST_BUF_Idx] = LNB_AR_RAW;//SingleT;
						MR3_ST_BUF[LNB_ST_BUF_Idx] = MR3_AR_RAW;
						//MR3_ST_BUF[LNB_ST_BUF_Idx] =(( LNB_AR_RAW & 0xFFFFE000 )|(MR3_AR_RAW & (~0xFFFFE000)));
					  LNB_ST_BUF_Idx++;
						//MR3_ST_BUF_Idx++;
				}	
				else
				{
						LNB_ST_BUF_Idx = 0;
					  mem_dump_request = 0;
					  printf("DUMP FINISH, START DUMP(MR3 ST in decimal)\r\n");
						DUMP_Memory();
						//MR3_ST_BUF_Idx = 0;
				}
#ifdef MEM_DUMP			  //MR3_AR = ((*((uint32_t *)&(PDBuffer.buffer[PDBuffer.tail].AR)))>>9)*MR3_AR_k;
}
#endif
#endif			
        



        // err_count ++;
        // if(PDBuffer.buffer[PDBuffer.front].ERR != 0x17)
        //   err_ipo_count ++;

        SET_BIT(dataCollectionCF, DATA_COLLECTION_ENQUEUE);

        CLEAR_BIT(encoderCF, ENCODER_PD_INVALID);

        /* Reset data collection state */
        dataCollectionState = DATA_COLLECTION_STATE_IDLE;
				latency1 = DWT->CYCCNT;
        break;
      case DATA_COLLECTION_STATE_ENQUEUE:
        
        /* check SPI finished */
        if ((PDBuffer.buffer[PDBuffer.front].ST && 0x01U) == 0x01U) /* PDV set */
        {
          /* Exchange tail and front */
          uint32_t tmp = PDBuffer.front; /* if front == tail ? */
          PDBuffer.front = PDBuffer.tail;
          PDBuffer.tail = tmp;
        }
				else
				{
					err_PDV_count++;
				}
        /* TEST BLOCK - START */

        err_count ++;
        if ((LNB_AR_RAW & 0xFFF80000) != (MR3_AR_RAW & 0xFFF80000))
          err_unsync_count++;

        /* TEST BLOCK - STOP */

        SET_BIT(monitoringCF, MONITORING_ROTATION);

        /* Reset data collection state */
        dataCollectionState = DATA_COLLECTION_STATE_IDLE;

        break;
      default:

        /* Reset data collection state */
        dataCollectionState = DATA_COLLECTION_STATE_IDLE;

        break;
    }
}

/**
 * @brief update voltage and temperature state.
 */
void monitoringThread(void)
{
  switch (monitoringState)
  {
  case MONITORING_STATE_IDLE:

    /* Reset monitoring state */
    monitoringState = MONITORING_STATE_IDLE;
    break;
  case MONITORING_STATE_VOLTAGE:

    /* VP, GPIO INPUT */
    if (HAL_GPIO_ReadPin(VM_VP_GPIO_Port, VM_VP_Pin) == GPIO_PIN_RESET)
      SET_BIT(encoderCF, ENCODER_POWER_FAILURE);
    else
      CLEAR_BIT(encoderCF, ENCODER_POWER_FAILURE);
  
    /* VB, ADC read */
    betteryVoltageRaw = HAL_ADC_GetValue(&hadc2);
    betteryVoltage = betteryVoltageRaw * betteryVoltageRes;

    /* update bettery voltage state */
    if (betteryVoltage < 2.27f) /* 2.5 divided by 100k and 10k resistor */
    {
      // SET_BIT(encoderCF, ENCODER_POWER_FAILURE); /* Simulate Power failure */
      SET_BIT(encoderCF, ENCODER_VBAT_ERR);
      SET_BIT(encoderCF, ENCODER_VBAT_WARN);
    }
    else if (betteryVoltage < 2.7f) /* 3.0 divided by 100k and 10k resistor */
      SET_BIT(encoderCF, ENCODER_VBAT_WARN);
    else
      {
        CLEAR_BIT(encoderCF, ENCODER_VBAT_ERR);
        CLEAR_BIT(encoderCF, ENCODER_VBAT_WARN);
      }

    /* Reset monitoring state */
    monitoringState = MONITORING_STATE_IDLE;

    break;
  case MONITORING_STATE_ROTATION:

    /* Update rotation state */
    EN_MR3_RotationCheck(&hmr3,
                         *((uint32_t *)&(PDBuffer.buffer[PDBuffer.front].AR)),
                         *((uint16_t *)&(PDBuffer.buffer[PDBuffer.front].MT)),
                         0x002000);

    /* Reset monitoring state */
    monitoringState = MONITORING_STATE_IDLE;

    break;
  case MONITORING_STATE_TMP_ALM:

    /* Update TMP_ALM state */
    if (HAL_GPIO_ReadPin(TMP_ALM_GPIO_Port, TMP_ALM_Pin) == GPIO_PIN_RESET)
      SET_BIT(encoderCF, ENCODER_TMP_ALM);
    else
      CLEAR_BIT(encoderCF, ENCODER_TMP_ALM);

    /* Reset monitoring state */
    monitoringState = MONITORING_STATE_IDLE;

    break;
  default:
    break;
  }
}

void exceptionHandlingThread(void)
{
  switch (exceptionHandlingState)
  {
  case EXCEPTION_HANDLING_STATE_IDLE:

    /* TODO: Force MCU into low-power mode, But how to restore */

    // if (HAL_GPIO_ReadPin(VM_VP_GPIO_Port, VM_VP_Pin) == GPIO_PIN_SET)
    //   NVIC_SystemReset();

    /* Reset exception handling state */ 
    exceptionHandlingState = EXCEPTION_HANDLING_STATE_IDLE;
    break;
  case EXCEPTION_HANDLING_STATE_POWER_FAILURE:
    /* Power failure Process */
    // eh_PowerFailureHandler();
    /* Reset exception handling state */ 
    exceptionHandlingState = EXCEPTION_HANDLING_STATE_IDLE;
    break;
  case EXCEPTION_HANDLING_STATE_POWER_FAILURE_RETURN:
    /* Power failure Process */
    // eh_PowerFailureReturnHandler();
    /* Reset exception handling state */ 
    exceptionHandlingState = EXCEPTION_HANDLING_STATE_IDLE;
    break;
  case EXCEPTION_HANDLING_STATE_OVER_SPEED:
    /* Over speed Process*/
    eh_OverSpeedHandler();
    /* Reset exception handling state */ 
    exceptionHandlingState = EXCEPTION_HANDLING_STATE_IDLE;
    break;
  default:
    /* Reset exception handling state */ 
    exceptionHandlingState = EXCEPTION_HANDLING_STATE_IDLE;
    break;
  }
}

void absDataSyncThread(void)
{
  switch (absDataSyncState)
  {

  case ABS_DATA_SYNC_STATE_RUNNING:

    SET_BIT(absDataSyncCF, ABS_DATA_SYNC_PREPARING); /* diable datacolleciton thread */
    SET_BIT(encoderCF, ENCODER_PD_INVALID); /* disable T-foramt response */

    MR3_AbsDataSync();

  
    CLEAR_BIT(encoderCF, ENCODER_PD_INVALID);
    CLEAR_BIT(absDataSyncCF, ABS_DATA_SYNC_PREPARING);
    CLEAR_BIT(absDataSyncCF, ABS_DATA_SYNC_STATE_RUNNING);
    latency1_tmp = DWT->CYCCNT;

    // for(uint32_t i = 0; i < 0x960;i++) ;
    /* Reset absolute data sync state */
    absDataSyncState = ABS_DATA_SYNC_STATE_IDLE;
    break;
  case ABS_DATA_SYNC_STATE_TIMEOUT:
    /* reset MR3 ADI */
    break;
  case ABS_DATA_SYNC_STATE_IDLE:
  case ABS_DATA_SYNC_STATE_PENDING:
  default:

    /* ERROR */

    /* Reset absolute data sync state */
    absDataSyncState = ABS_DATA_SYNC_STATE_IDLE;
    break;
  }
}

void eh_PowerFailureHandler(void)
{
  /* ## -1. reset FSM*/

  /* Check SSI is not running */
  while ((HAL_IS_BIT_SET(absDataSyncCF, ABS_DATA_SYNC_PREPARING) || (absDataSyncState == ABS_DATA_SYNC_STATE_RUNNING))) ;

  /* Check T-foramt is not running */
  
  /* ## 0. Read MR3 for certain times (optional) */
  /* ## 1. mask ISRs*/

  /* ## 2. Stop useless peripheral(timers)*/

  HAL_TIM_Base_Stop_IT(&htim3);
  HAL_TIM_Base_Stop_IT(&htim4);
  HAL_TIM_Base_Stop(&htim8);
  HAL_UART_Abort_IT(&huart1);

  /* ## 3. Stop data collection thread */

  /* ## 4. Write current data to EEPROM */

  /* Position data (singleturn, multiturm)*/
  while (HAL_I2C_Mem_Write_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE,EN_ADDR_ST_DATA_BASE, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&(PDBuffer.buffer[PDBuffer.front].AR[1]), 6) != HAL_OK)
  {
    /* error */
    ;
  }
  
  /* Zeropoint Singleturn */
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;

  while (HAL_I2C_Mem_Write_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE,EN_ADDR_ZERO_POINT_ST_BASE, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&(ZSingleT), 3) != HAL_OK)
  {
    /* error */
    ;
  }
  
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;

  /* Set POWER_FAILURE_FLAG */
  while (HAL_I2C_IsDeviceReady(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, 30, 30) == HAL_TIMEOUT);

  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;
  
  uint8_t tmp = 0x01;

  while (HAL_I2C_Mem_Write_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, EN_ADDR_POWER_FAILURE_FLAG, I2C_MEMADD_SIZE_16BIT, &tmp, 1) != HAL_OK)
  {
    ;
  }
  
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;

  /* ## 7. Force MCU into Low-power mode */
}

void eh_PowerFailureReturnHandler(void)
{
  uint32_t AR_R = 0;
  uint16_t MT_R = 0;

  uint32_t AR_C = 0;
  uint16_t MT_C = 0;
  uint16_t MT_N = 0;
  
  uint16_t offset_now = 0;

  /* ## Check EN_ADDR_POWER_FAILURE_FLAG */

  while (HAL_I2C_IsDeviceReady(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, 30, 30) == HAL_TIMEOUT);

  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;
  
  uint8_t tmp = 0;

  while (HAL_I2C_Mem_Read_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, EN_ADDR_POWER_FAILURE_FLAG, I2C_MEMADD_SIZE_16BIT, &tmp, 1) != HAL_OK)
  {
    ;
  }
  
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;

  if (tmp == 0) goto end;

  /* ## Read the PD in EEPROM */
  
  while (HAL_I2C_IsDeviceReady(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, 30, 30) == HAL_TIMEOUT);

  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;
  
  while (HAL_I2C_Mem_Read_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, EN_ADDR_ST_DATA_BASE, I2C_MEMADD_SIZE_16BIT, (uint8_t *) &(PFPDBuf.AR[1]), 6) != HAL_OK)
  {
    ;
  }
  
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;


  AR_R = *((uint32_t *) &PFPDBuf.AR);
  MT_R = *((uint16_t *) &PFPDBuf.MT);

  /* Read data from MR3 */

  EN_MR3_CyclicRead(&hmr3, NULL);

  AR_C = (*((uint32_t *)&hmr3.Instance->CC_DATA.AR));
  MT_C = *((uint16_t *)&hmr3.Instance->CC_DATA.MT);

  /* ## Cross zero point */

  if (AR_R > AR_C)
  {
    if (AR_R - AR_C > 0xFFFFFE)
    {
      if (MT_R == 0xFFFF)
        MT_N = 0;
      else
        MT_N = MT_R + 1U;
    }
    else 
      MT_N = MT_R;
  }
  else 
  {
    if (AR_C - AR_R > 0xFFFFFE)
    {
      if(MT_R == 0)
        MT_N = 0xFFFF;
      else 
        MT_N = MT_R - 1U;
    }
    else 
      MT_N = MT_R;
  }
  EN_MR3_GetMultiTurnOffset(&hmr3, (uint8_t *)&offset_now);

  MT_N = (MT_C - MT_N) + offset_now;

  EN_MR3_SetMultiTurnOffset(&hmr3, (uint8_t *)&MT_N);

  HAL_Delay(1);

  EN_MR3_CyclicRead(&hmr3, NULL);

end:
  /* Set POWER_FAILURE_FLAG */
  while (HAL_I2C_IsDeviceReady(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, 30, 30) == HAL_TIMEOUT);

  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;
  
  tmp = 0x00;

  while (HAL_I2C_Mem_Write_IT(&hi2c3, EEPROM_24LC64_DEVICE_ADDRESS_BASE, EN_ADDR_POWER_FAILURE_FLAG, I2C_MEMADD_SIZE_16BIT, &tmp, 1) != HAL_OK)
  {
    ;
  }
  
  while(HAL_I2C_GetState(&hi2c3) != HAL_I2C_STATE_READY) ;

  /* ## Move forware or backware*/
  ;
}

void eh_OverSpeedHandler(void)
{
  /*restore the position data after Over speed */
  //eh_PowerFailureHandler();

  /* reset multiturn */
}

static void MR3_Reset(void)
{
  /* Reset MR3 */
  if (hmr3.hspi == &hspi1)
  {
    __EN_MR3_SPI_SOFTWARE_RESET(&hmr3); // hardware reset or software reset ?
    HAL_Delay(1);
  }
  else
  {
    hmr3.hspi = &hspi1;
    hmr3.CS_Port = CS2_GPIO_Port;
    hmr3.CS = CS2_Pin;
    
    __EN_MR3_SPI_SOFTWARE_RESET(&hmr3);
    HAL_Delay(1);
  }
}

static HAL_StatusTypeDef MR3_Init(void)
{
//  if (hmr3.hspi != &hspi1)
//  {
//    hmr3.hspi = &hspi1;
//    hmr3.CS_Port = CS2_GPIO_Port;
//    hmr3.CS = CS2_Pin;
//  }
    
  if (HAL_OK != EN_MR3_Init_Minimal(&hmr3))
  {
    if (EN_MR3_GetError(&hmr3) == EN_MR3_ERROR_SPI)
    {
      return EN_MR3_Init_Minimal(&hmr3);
    }
  }

  return HAL_OK;
}

static void LNB_Init(void)
{
  hlnb.Instance = &LNB;
  hlnb.hspi = &hspi4;
  hlnb.CS_Port = CS1_GPIO_Port;
  hlnb.CS = CS1_Pin;

  hlnb.Init.Mode = EN_LNB_MODE_INTERFACE;              // EPG
  hlnb.Init.SSI = EN_LNB_SSI_ENABLE;                   // NENSHIFT
  hlnb.Init.Interpolator = EN_LNB_INTERPOLATOR_ENABLE; // ENIPO

  hlnb.Init.FlexCount = EN_LNB_FLEXCOUNT_DISABLE; // ENFLEX
  hlnb.Init.TestMode = EN_LNB_TEST_MODE_DISABLE;  // TA

  /* Interpolator Parameter */
  hlnb.Init.InterpolatorResolution = EN_LNB_INTERPOLATOR_RESOLUTION_6BIT;  // RESIPO
  hlnb.Init.InterpolatorHysteresis = EN_LNB_INTERPOLATOR_HYSTERESIS_0;     // HYS
  hlnb.Init.InterpolatorFilterDisable = EN_LNB_INTERPOLATOR_FILTER_ENABLE; // NENF

  /* ABZ interface Parameter */
  hlnb.Init.ABZOutputResolution = EN_LNB_ABZ_OUTPUT_RESOLUTION_0; // INC

  /* SSI Parameter */
  hlnb.Init.SSIOutputDataFormat = EN_LNB_SSI_OUTPUT_DATA_FORMAT_BIN; // NGARY
  hlnb.Init.SSIIdleOutput = EN_LNB_SSI_IDLE_OUTPUT_HIGH;              // RNF
  hlnb.Init.SSILength = EN_LNB_SSI_LENGTH_18BIT;                      // SRC
  hlnb.Init.SSIDirection = EN_LNB_SSI_ROTATION_DIRECTION_CW;          //DIR

  /* Signal Calibration Parameter */
#ifndef SIGNAL_CALIBRATION_CONFIG
  hlnb.Init.DefaultSignalCalibration = EN_LNB_DEFAULT_SIGNAL_CALIBRATION;
#else
  hlnb.Init.DefaultSignalCalibration = EN_LNB_NO_DEFAULT_SIGNAL_CALIBRATION;
  /* The configuration of following paramter, please refer to LNB datasheet C1, p24 - 25*/
  // hlnb.Init.GainRange = ;         // GR
  // hlnb.Init.SinGain;           // GS
  // hlnb.Init.PositiveSinOffset; // OSP
  // hlnb.Init.NegativeSinOffset; // OSN
  // hlnb.Init.CosGain;           // GC
  // hlnb.Init.PositiveCosOffset; // OCP
  // hlnb.Init.NegativeCosOffset; // OCN
#endif

  /* LED Power Control parameter */
#ifndef LED_POWER_CONTROL_CONFIG
  hlnb.Init.DefaultLEDPowerControl = EN_LNB_DEFAULT_LED_POWER_CONTROL;
#else
  hlnb.Init.DefaultLEDPowerControl = EN_LNB_NO_DEFAULT_LED_POWER_CONTROL;
  /* Configuration of following paramter, please refer to LNB datasheet C1, p37*/
  // hlnb.Init.LEDPowerControlMode ;     // LCMOD
  // hlnb.Init.LEDPowerControlType;     // LCTYP
  // hlnb.Init.LEDPowerControlSetpoint; // LCSET
#endif

  hlnb.Instance = &LNB;
  hlnb.hspi = &hspi4;
  hlnb.CS_Port = CS1_GPIO_Port;
  hlnb.CS = CS1_Pin;


  if ( EN_LNB_Init(&hlnb) == HAL_ERROR)
  {
			Error_Handler();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
    HAL_TIM4_PeriodElapsedCallback(htim);

}


/**
 * @brief Event trigger of Absolute date sync event, T = 584us
 * */
void HAL_TIM4_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (absDataSyncState == ABS_DATA_SYNC_STATE_IDLE)
    SET_BIT(absDataSyncCF, ABS_DATA_SYNC_ACTIVATE);
  else if ((absDataSyncState == ABS_DATA_SYNC_STATE_PENDING) || 
           (absDataSyncState == ABS_DATA_SYNC_STATE_RUNNING))
    SET_BIT(absDataSyncCF, ABS_DATA_SYNC_TIMEOUT);
  
  return ;
}




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC2)
  {
      SET_BIT(monitoringCF, MONITORING_VOLTAGE | MONITORING_TMP_ALM);
  }
}

// /**
//  * @brief Reaverse Bytes order of 64-bit unsigned integer
//  */
// uint64_t rev64(uint64_t val)
// {
//   return ((uint64_t)__rev((uint32_t) ((val & 0xffffffff00000000) >> 32))) | 
//          (((uint64_t)__rev((uint32_t) (val & 0x00000000ffffffff))) << 32);
// }

/**
 * @brief Synchronize absolute position data of MR3 cycle counter with LNB data
 */
void MR3_AbsDataSync(void)
{
  volatile uint32_t tp1 = 0;
  volatile uint32_t tp2 = 0;
  volatile uint64_t offset = 0;
  volatile uint64_t offset_now = 0;
  volatile uint64_t position_src = 0;
  volatile uint64_t position_dst = 0;
  volatile uint64_t tmp = 0;
  
  tp1 = DWT->CYCCNT;

  EN_MR3_GetSingleTurnOffset(&hmr3, (uint8_t *)&offset_now+2);

  EN_MR3_CyclicRead(&hmr3, NULL);
  EN_LNB_Position(&hlnb);

  position_src = (((uint64_t)(*(uint32_t *)&hmr3.Instance->CC_DATA.AR) & 0xfff80000) << 16);
  position_dst = ((uint64_t) __rev(*(uint32_t *)&(hlnb.Instance->AR_b)) << 16);

  offset = (((position_src - position_dst )&0xfffffffffe000000) + (rev64(offset_now)&0x0000ffffffffffff)) & 0x0000ffffffffffff;

  offset = (((uint64_t)ZSingleT ) << 25) + offset;
  
  offset = rev64(offset);

  EN_MR3_SetSingleTurnOffset(&hmr3, ((uint8_t *)&offset)+2);

  tp2 = DWT->CYCCNT - tp1;
  
  return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //_______________________________________________//Instance==USART1
    if (huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
        //=========================================================//rx_index==0
        if (rx_index == 0)
        {
            //___________________________________________________//0xea
            if(rx_data == 0x30)
						{
								mem_dump_request = 1;
							  rx_index = 0;
						}
						//___________________________________________________//0xea
            else if (rx_data == 0xea)
            {
                rx_buffer[rx_index] = rx_data;
                rx_index++;
                max_index = 3;
            }
            //___________________________________________________//0xea
            //___________________________________________________//0x1a
            else if (rx_data == 0x1a)
            {
              	  
							
							//fsm_latency =   DWT->CYCCNT - tformat_last;
							/* RESET ID8 INTERNAL STATE - START */
                resetMTurn = 0;
                resetSTurn = 0;
                /* RESET ID8 INTERNAL STATE - STOP */
                if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS))
                {
                  CLEAR_BIT(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS);
                  MultiT2 = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
                  SingleT2 = (*((uint32_t *)&PDBuffer.buffer[PDBuffer.front].AR));
                  SingleT2 = (SingleT2 >> 9);
                }
                SET_BIT(encoderCF, ENCODER_COLLECTION_MODE_SINGLE);

                //__________________________________________//RS485 Communication Format
                //   Multiturn 16 bit data  (2 Bytes)
                //   Singleturn 23 bit data (3 Bytes)
                //MultiT = (*((uint16_t *)&hmr3.Instance->CC_DATA.MT));
								
              
#ifdef MR3_ST	
			
								// SingleT = (*((uint32_t *)&hmr3.Instance->CC_DATA.AR));
                SingleT = (*((uint32_t *)&PDBuffer.buffer[PDBuffer.front].AR));
                SingleT = (SingleT >> 9);
								#ifdef MR3_ST_23PAD5B0
								SingleT &=0x007ffffe0;
								#endif
								
								MultiT = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
								
#else
								LNB_AR_RAW = (hlnb.Instance->AR & 0x00FFFFFF)<<5;
								
#ifdef FIX_LNB_MSB
			
				
				SingleT_emu = LNB_AR_RAW;
						
				if (SingleT_emu >= L_SingleT_emu)
				{
						DSingleT_emu = SingleT_emu - L_SingleT_emu;
						if (DSingleT_emu > 0x20000U)//90 deg@ 23-bit
						{
								DSingleT_MSB_emu =~ DSingleT_MSB_emu; 
								DSingleT_MSB_emu &= 0x01;
								DSingleT_emu=0;
						}
				}
				else
				{
						DSingleT_emu = L_SingleT_emu - SingleT_emu;
						if (DSingleT_emu > 0x20000U)//90 deg@ 23-bit
						{
								DSingleT_MSB_emu =~ DSingleT_MSB_emu; 
							  DSingleT_MSB_emu &= 0x01;
								DSingleT_emu=0;
						}
				}
				L_SingleT_emu = SingleT_emu ;
				SingleT_emu |= (DSingleT_MSB_emu<<22);
				LNB_AR_RAW = SingleT_emu;
#endif			 
								//SingleT = LNB_AR_RAW - LNB_ST_Offset;
								DSingle = LNB_AR_RAW - LNB_ST_Offset;
                if (DSingle < 0)
                {
                    DSingleT = 0x7fffff + DSingle;
                }
                else
                {
                     DSingleT = DSingle;
                }
                DSingleT = DSingleT & 0x007fffffU;
								SingleT = DSingleT  & 0x007fffe0U;
								
								if (SingleT >= L_SingleT)
								{
									DSingleT = SingleT - L_SingleT;
									if (DSingleT > 4194304)//180 deg
									{
										L_MultiT++; DSingleT=0;
										if (L_MultiT>=65536)
										{
											L_MultiT=0;
										}
									}
								}
								else
								{
									DSingleT = L_SingleT - SingleT;
									if (DSingleT > 4194304)
									{
										DSingleT=0;
										if (L_MultiT>=0)
										{
										L_MultiT--; 
										}
										else
										{
										L_MultiT=65536;	
										}
									}
								}
								MultiT = L_MultiT;
								MultiT =~ MultiT;
								MultiT &=0x0000ffff;
								
//								if(LNB_ST_BUF_Idx<2048) 
//								{	
//									LNB_ST_BUF[LNB_ST_BUF_Idx] = SingleT;
//									LNB_ST_BUF_Idx++;
//								}
//								else LNB_ST_BUF_Idx = 0;	
#endif
								//---------------------------------//
//								if(SingleTBuf_idx<4)SingleTBuf_idx++; 
//								else SingleTBuf_idx = 0;
//								
//								SingleTBuf[SingleTBuf_idx] = SingleT & 0x000000ff;
//								SingleT_AVG = SingleTBuf[0]+SingleTBuf[1]+SingleTBuf[2]+SingleTBuf[3];
//								SingleT_AVG>>=2;
//								SingleT_AVGout = (SingleT & 0xffffff00)|SingleT_AVG;
//								SingleT = SingleT_AVGout;
								//---------------------------------//
                CRC_Array[0] = 0x1a;
                CRC_Array[1] = 0x00;
                CRC_Array[5] = 0x17;
                CRC_Array[9] = 0x00;


                //singleturn:
                //______________________________________________________________//Not Reset
                if (resetS_t == 0)
                {
                    CRC_Array[4] = (SingleT >> 16) & 0xFF;
                    CRC_Array[3] = (SingleT >> 8) & 0xFF;
                    CRC_Array[2] = SingleT & 0xFF;
                }
                //______________________________________________________________//Not Reset
                //______________________________________________________________//Reset
                else //resetS_t!=0
                {
                    // DSingle = SingleT - ZSingleT;
                    // if (DSingle < 0)
                    // {
                    //     DSingleT = 0x7fffff + DSingle;
                    // }
                    // else
                    // {
                    //     DSingleT = DSingle;
                    // }
                    // DSingleT = DSingleT & 0x007fffffU;
                    // CRC_Array[4] = (DSingleT >> 16) & 0xFF;
                    // CRC_Array[3] = (DSingleT >> 8) & 0xFF;
                    // CRC_Array[2] = DSingleT & 0xFF;
                    CRC_Array[4] = (SingleT >> 16) & 0xFF;
                    CRC_Array[3] = (SingleT >> 8) & 0xFF;
                    CRC_Array[2] = SingleT & 0xFF;

                }
                //______________________________________________________________//Reset
                //multiturn:
                CRC_Array[8] = (MultiT >> 16) & 0xFF;
                CRC_Array[7] = (MultiT >> 8) & 0xFF;
                CRC_Array[6] = MultiT & 0xFF;

                TX_Array[0] = CRC_Array[0];
                TX_Array[1] = CRC_Array[1];
                TX_Array[5] = CRC_Array[5];
                TX_Array[2] = CRC_Array[2];
                TX_Array[3] = CRC_Array[3];
                TX_Array[4] = CRC_Array[4];
                TX_Array[6] = CRC_Array[6];
                TX_Array[7] = CRC_Array[7];
                TX_Array[8] = CRC_Array[8];
                TX_Array[9] = CRC_Array[9];

                rx_buffer[rx_index] = rx_data;
                max_index = 1;
                resetSTurn = 0;
                TX_Array[10] = HAL_CRC_Calculate(&hcrc, (uint32_t *)CRC_Array, 10);
								//tformat_current= DWT->CYCCNT;
								
                //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)TX_Array, 11);
								HAL_UART_Transmit(&huart1, (uint8_t *)TX_Array, 11, 0xFFFFFFFF);

#if !defined MR3_ST || defined MLT_BY_FW
								L_SingleT = SingleT; //L_MultiT = MultiT;
#endif

                if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_SINGLE))
                {
                    if (!dataCollectionCF)
                        SET_BIT(dataCollectionCF, DATA_COLLECTION_COLLECTION);
                }
								// MR3_READ_START
								//tmp_mr3_cycread  = DWT-> CYCCNT;
#ifdef MR3_ST								
								EN_MR3_CyclicRead(&hmr3, (uint8_t *)&(PDBuffer.buffer[PDBuffer.front]));
								//MR3_AR = ((*((uint32_t *)&(PDBuffer.buffer[PDBuffer.front].AR)))>>9)*MR3_AR_k;
#else
								EN_LNB_Position(&hlnb);
#endif								
								// MR3_READ_END , cost 45us
								//latency_mr3_cycread = DWT-> CYCCNT - tmp_mr3_cycread; 
								//tformat_latency = DWT->CYCCNT - tformat_current;
								
            }
            //___________________________________________________//0x1a
            //___________________________________________________//0xc2
            else if (rx_data == 0xc2)
            {
                if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS))
                    CLEAR_BIT(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS);
                SET_BIT(encoderCF, ENCODER_COLLECTION_MODE_SINGLE);

                /* PREPARING PART - START */
                //   Multiturn 16 bit data  (2 Bytes)
                //   Singleturn 23 bit data (3 Bytes)
                //MultiT = (*((uint16_t *)&hmr3.Instance->CC_DATA.MT));
#ifdef MR3_ST								
                
								
                //SingleT = (*((uint32_t *)&hmr3.Instance->CC_DATA.AR));
                SingleT = (*((uint32_t *)&PDBuffer.buffer[PDBuffer.front].AR));
                SingleT = (SingleT >> 9);
								
							
								MultiT = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
								
#else
								LNB_AR_RAW = (hlnb.Instance->AR & 0x00FFFFFF)<<5;
								#ifdef FIX_LNB_MSB
			
				
				SingleT_emu = LNB_AR_RAW;
						
				if (SingleT_emu >= L_SingleT_emu)
				{
						DSingleT_emu = SingleT_emu - L_SingleT_emu;
						if (DSingleT_emu > 0x20000U)//90 deg@ 23-bit
						{
								DSingleT_MSB_emu =~ DSingleT_MSB_emu; 
								DSingleT_MSB_emu &= 0x01;
								DSingleT_emu=0;
						}
				}
				else
				{
						DSingleT_emu = L_SingleT_emu - SingleT_emu;
						if (DSingleT_emu > 0x20000U)//90 deg@ 23-bit
						{
								DSingleT_MSB_emu =~ DSingleT_MSB_emu; 
							  DSingleT_MSB_emu &= 0x01;
								DSingleT_emu=0;
						}
				}
				L_SingleT_emu = SingleT_emu ;
				SingleT_emu |= (DSingleT_MSB_emu<<22);
				LNB_AR_RAW = SingleT_emu;
#endif			 
								//SingleT = LNB_AR_RAW - LNB_ST_Offset;
								DSingle = LNB_AR_RAW - LNB_ST_Offset;
                if (DSingle < 0)
                {
                    DSingleT = 0x7fffff + DSingle;
                }
                else
                {
                     DSingleT = DSingle;
                }
                DSingleT = DSingleT & 0x007fffffU;
								SingleT = DSingleT & 0x007fffe0U;
#endif
                CRC_Array[0] = 0xc2;
                CRC_Array[1] = 0x00;
                //==============================================================================//resetSTurn==1
                if (resetSTurn == 0)
                {
                    // ZSingleT = SingleT;
                    resetSTurn = 1;
                    resetS_t = 0;
                    // CRC_Array[4] = 0x00U;
                    // CRC_Array[3] = 0x00U;
                    // CRC_Array[2] = 0x00U;

                    CRC_Array[4] = (SingleT >> 16) & 0xFF;
                    CRC_Array[3] = (SingleT >> 8) & 0xFF;
                    CRC_Array[2] = SingleT & 0xFF;
#ifdef MR3_ST									
                    EN_MR3_ZeroizeSingleTurn(&hmr3);
                    ZSingleT = (*((uint32_t *)&hmr3.Instance->CC_DATA.AR)) >> 9;
										#ifdef MR3_ST_23PAD5B0
										SingleT &=0x007ffffe0;
										#endif
#else
										
										LNB_ST_Offset = LNB_AR_RAW & 0x007fffffU;;
									  ZSingleT = LNB_ST_Offset;
#endif																		
                   
                }
                //==============================================================================//resetSTurn==1
                //==============================================================================//resetSTurn==2
                else if (resetSTurn == 1)
                {
                    // DSingle = SingleT - ZSingleT;
                    // if (DSingle < 0)
                    // {
                    //     DSingleT = 0x7fffff + DSingle;
                    // }
                    // else
                    // {
                    //     DSingleT = DSingle;
                    // }
                    // DSingleT = DSingleT & 0x007fffffU;
                    // //singleturn:
                    // CRC_Array[4] = (DSingleT >> 16) & 0xFF;
                    // CRC_Array[3] = (DSingleT >> 8) & 0xFF;
                    // CRC_Array[2] = DSingleT & 0xFF;
                    CRC_Array[4] = (SingleT >> 16) & 0xFF;
                    CRC_Array[3] = (SingleT >> 8) & 0xFF;
                    CRC_Array[2] = SingleT & 0xFF;
                    // EN_MR3_ZeroizeSingleTurn(&hmr3);
                }
                //==============================================================================//resetSTurn==2

                /* PREPARING PART - STOP */
                rx_buffer[rx_index] = rx_data;
                max_index = 1;
                TX_RArray[0] = CRC_Array[0];
                TX_RArray[1] = CRC_Array[1];
                TX_RArray[2] = CRC_Array[2];
                TX_RArray[3] = CRC_Array[3];
                TX_RArray[4] = CRC_Array[4];
                TX_RArray[5] = HAL_CRC_Calculate(&hcrc, (uint32_t *)CRC_Array, 5);
                HAL_UART_Transmit_DMA(&huart1, (uint8_t *)TX_RArray, 6);

                MultiT2 = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
                SingleT2 = (*((uint32_t *)&PDBuffer.buffer[PDBuffer.front].AR));
                SingleT2 = (SingleT2 >> 9);

                if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_SINGLE))
                {
                    if (!dataCollectionCF)
                        SET_BIT(dataCollectionCF, DATA_COLLECTION_COLLECTION);
                }

                resetS_t++;
            }
            //___________________________________________________//0xc2
//            else if (rx_data == 0x62)
//            {
//                if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS))
//                    CLEAR_BIT(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS);
//                SET_BIT(encoderCF, ENCODER_COLLECTION_MODE_SINGLE);

//                /* PREPARING PART - START */
//                //   Multiturn 16 bit data  (2 Bytes)
//                //   Singleturn 23 bit data (3 Bytes)
//                //MultiT = (*((uint16_t *)&hmr3.Instance->CC_DATA.MT));
//                MultiT = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
//#ifdef MR3_ST								
//                //SingleT = (*((uint32_t *)&hmr3.Instance->CC_DATA.AR));
//                SingleT = (*((uint32_t *)&PDBuffer.buffer[PDBuffer.front].AR));
//                SingleT = (SingleT >> 9);
//#else
//								LNB_AR_RAW = (hlnb.Instance->AR & 0x00FFFFFF)<<5;
//								SingleT = LNB_AR_RAW - LNB_ST_Offset;				
//#endif
//                CLEAR_BIT(encoderCF, ENCODER_OVERSPEED);
//                CLEAR_BIT(exceptionHandlingCF, EXCEPTION_HANDLING_OVER_SPEED);

//                CRC_Array[0] = 0x62;
//                CRC_Array[1] = 0x00;

//                if (resetMTurn == 0)
//                {
//                  CRC_Array[4] = (SingleT >> 16) & 0xFF;
//                  CRC_Array[3] = (SingleT >> 8) & 0xFF;
//                  CRC_Array[2] = SingleT & 0xFF;
//                  EN_MR3_CyclicRead(&hmr3, NULL);
//                  EN_MR3_ZeroizeMultiTurn(&hmr3);
//                  resetMTurn = 1;
//                }
//                else if (resetMTurn == 1)
//                {
//                  CRC_Array[4] = (SingleT >> 16) & 0xFF;
//                  CRC_Array[3] = (SingleT >> 8) & 0xFF;
//                  CRC_Array[2] = SingleT & 0xFF;
//                }

//                /* PREPARING PART - STOP */
//                rx_buffer[rx_index] = rx_data;
//                max_index = 1;
//                TX_RArray[0] = CRC_Array[0];
//                TX_RArray[1] = CRC_Array[1];
//                TX_RArray[2] = CRC_Array[2];
//                TX_RArray[3] = CRC_Array[3];
//                TX_RArray[4] = CRC_Array[4];
//                TX_RArray[5] = HAL_CRC_Calculate(&hcrc, (uint32_t *)CRC_Array, 5);
//                HAL_UART_Transmit_DMA(&huart1, (uint8_t *)TX_RArray, 6);

//                if (HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_SINGLE))
//                {
//                    if (!dataCollectionCF)
//                        SET_BIT(dataCollectionCF, DATA_COLLECTION_COLLECTION);
//                }

//                resetS_t++;
//            }
        }
				//___________________________________________________//0xc2??
        //=========================================================//rx_index==0
        //=========================================================//rx_index>0
        else //rx_index>0
        {
            rx_buffer[rx_index] = rx_data;
            rx_index++;

            if (rx_index >= max_index)
            {
                //==================================================//0xea 0x00
                if (rx_buffer[1] == 0x00)
                {
                    HAL_UART_Transmit_DMA(&huart1, send2, 4);
                }
                //==================================================//0xea 0x00
                //==================================================//0xea 0x01
                if (rx_buffer[1] == 0x01)
                {
                    HAL_UART_Transmit_DMA(&huart1, send3, 4);
                }
                //==================================================//0xea 0x01
                rx_index = 0;
            }
        }
        //=========================================================//rx_index>0
        if (rx_index == 0)
        {
            ii = 0;
            for (ii = 0; ii < max_index; ii++)
            {
                rx_buffer[ii] = 0;
            }
        }
    }
    //_______________________________________________//Instance==USART1
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
  {
			//mr3_spi_tx_count++;
		  
			clockofpositionstop = DWT->CYCCNT - clockofpositionstatrt;
		  HAL_GPIO_WritePin(hmr3.CS_Port, hmr3.CS, GPIO_PIN_SET);
		  LNB_AR_RAW = (hlnb.Instance->AR & 0x00FFFFFF)<<5;
			//GPIOA->BSRR = CS2_Pin;//set 
			MR3_AR_RAW = (*((uint32_t *)&(PDBuffer.buffer[PDBuffer.tail].AR)));
			MR3_AR_RAW >>=9;
		  MultiT = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
		  //HAL_GPIO_WritePin(GPIOA, MR3_NL_Pin, GPIO_PIN_SET);//
			GPIOA->BSRR = MR3_NL_Pin;
	}
}




void DUMP_Memory(void)
{
	uint32_t mem_dump_idx = 0;
	for(mem_dump_idx=0;mem_dump_idx<4096;mem_dump_idx++)
	{
			printf("LNB:%d ", LNB_ST_BUF[mem_dump_idx]);
		  HAL_Delay(1);
		  printf("MR3:%d\r\n", MR3_ST_BUF[mem_dump_idx]);
		  HAL_Delay(1);
	}
}

// /**
//  * @brief T-format request period reference, T = 120 us
//  * */
// void HAL_TIM3_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if(HAL_IS_BIT_SET(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS))
//     CLEAR_BIT(encoderCF, ENCODER_COLLECTION_MODE_CONTINUOUS);
//   SET_BIT(encoderCF, ENCODER_COLLECTION_MODE_SINGLE);
//   // TIM9->CNT = 79;
// latencyddd_tmp = DWT->CYCCNT;
// #ifdef DEBUG_MODE_CONTINUOUS
//   MultiT = (*((uint16_t *)&PDBuffer.buffer[PDBuffer.front].MT));
//   SingleT = (*((uint32_t *)&PDBuffer.buffer[PDBuffer.front].AR));
//   SingleT = (SingleT>>9);
//   SingleT = SingleT & 0x007fffffU;
//   CRC_Array[0]=0x1a; CRC_Array[1]=0x00; CRC_Array[5]=0x17; CRC_Array[9]=0x00;

//   CRC_Array[4] = (SingleT	>> 16) & 0xFF; 
//   CRC_Array[3] = (SingleT	>> 8) & 0xFF;	 
//   CRC_Array[2] =  SingleT & 0xFF;

//   CRC_Array[8] = (MultiT	>> 16) & 0xFF; 
//   CRC_Array[7] = (MultiT	>> 8) & 0xFF;	 
//   CRC_Array[6] =  MultiT & 0xFF;	

//   TX_Array[0]=CRC_Array[0]; TX_Array[1]=CRC_Array[1]; TX_Array[5]=CRC_Array[5];
//   TX_Array[2]=CRC_Array[2]; TX_Array[3]=CRC_Array[3]; TX_Array[4]=CRC_Array[4];
//   TX_Array[6]=CRC_Array[6]; TX_Array[7]=CRC_Array[7]; TX_Array[8]=CRC_Array[8];
//   TX_Array[9]=CRC_Array[9];

//   // rx_buffer[rx_index] = rx_data; max_index=1; resetSTurn=0;
//   TX_Array[10] = HAL_CRC_Calculate(&hcrc, (uint32_t*)CRC_Array, 10);
//   HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TX_Array, 11);
// #endif
// SET_BIT(dataCollectionCF, DATA_COLLECTION_COLLECTION);
// latencyddd = DWT->CYCCNT - latencyddd_tmp;
//   SET_BIT(encoderCF, ENCODER_TF_LATE);
//   return ;
// }
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
