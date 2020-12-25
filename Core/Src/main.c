/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "nrf24.h"
#include "MPU6050.h"
#include "stm32f1xx_it.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t watch1,watch2,watch3,watch4,watch5,watch6;
float watch1fl,watch2fl,watch3fl,watch4fl;

//NRF24
uint8_t nRF24_payloadTX[32]; //TX buffer
uint8_t nRF24_payloadRX[32]; //RX buffer
const uint8_t nRF24_ADDR[3] = {5, 3, 5 }; //Address
uint32_t wifiOK;
uint8_t RXstpaketov=0;

//NRF24 data
uint32_t Ljoyupdown;
uint32_t Ljoyleftright;
uint32_t Djoyupdown;
uint32_t Djoyleftright;
uint32_t potenc1;
uint32_t potenc2;
uint32_t togg1;
uint32_t togg2;
uint32_t togg3;
uint32_t togg4;
uint32_t togg5;
uint32_t togg6;

//MPU6050
MPU6050_Result MPU6050rezulatat ;
MPU6050str	mpu6050DataStr;
int16_t GyroXcal,GyroYcal,GyroZcal;
int16_t GyroXOff,GyroYOff,GyroZOff;
int32_t SUMGyroX,SUMGyroY,SUMGyroZ;
uint32_t i=0;
uint8_t fifoBuffer[64];    // FIFO storage buffer
uint16_t packetSize=42;    // expected DMP packet size (default is 42 bytes)
uint32_t MPUintActive;


//UART DEBUG
char UartTXbuff0[100];

//MOTOR
uint32_t MotorStatus=MOTORINIT;
uint32_t GyroCalibStatus=0;

struct FlashDatastruct FlashDataDefault; //Default constant embedded in Code
struct FlashDatastruct FlashDataFlash;  //Constants read from Flash
struct FlashDatastruct FlashDataActive; //Active constants
struct Quaternions QuaternionMPU60500;
struct GravityVector GravityVectorMPU6050;
struct Angles AnglesMPU6050_DMP;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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

	//test timings DWT counter
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= 1;

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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start(&hadc1);

  //DEFAULT FLASH CONSTANTS--------------------------------------------------------------------------
  FlashDataDefault.controlData=CONTROLWORD;
  FlashDataDefault.pid_p_gain_pitch=5.0;
  FlashDataDefault.pid_i_gain_pitch=0.001;
  FlashDataDefault.pid_d_gain_pitch=500.0;
  FlashDataDefault.pid_p_gain_roll=5.0;
  FlashDataDefault.pid_i_gain_roll=0.001;
  FlashDataDefault.pid_d_gain_roll=500.0;
  FlashDataDefault.pid_p_gain_yaw=4;
  FlashDataDefault.pid_i_gain_yaw=0;
  FlashDataDefault.pid_d_gain_yaw=0;
  FlashDataDefault.pid_max_pitch = 400;
  FlashDataDefault.pid_i_max_pitch = 100;
  FlashDataDefault.pid_max_roll = 400;
  FlashDataDefault.pid_i_max_roll = 100;
  FlashDataDefault.pid_max_yaw = 0;
  FlashDataDefault.pid_i_max_yaw = 0;
  FlashDataDefault.maxpitchdegree=20; //degrees
  FlashDataDefault.maxrolldegree=20;  //degrees
  FlashDataDefault.maxyawdegree=180;  //degrees
  FlashDataDefault.minthrottle=80;    //80counts of 1000 to spin rotors
  FlashDataDefault.maxthrottle=800;   //800counts of 1000 (80%)


  if( CheckFlashData(FLASHCONSTADDR) == CONTROLWORD ) //Check if any Data is present
  {
	  //Read Data and Save parameters into ACTIVE structure
	  ReadFlashData(FLASHCONSTADDR, &FlashDataActive);
	  ReadFlashData(FLASHCONSTADDR, &FlashDataFlash);

  }
  else
  {
	  //Write default values into Flash, Read back data into Active Structure
	  WriteFlashData(FLASHCONSTADDR, &FlashDataDefault);
	  ReadFlashData(FLASHCONSTADDR, &FlashDataActive);
	  ReadFlashData(FLASHCONSTADDR, &FlashDataFlash);
  }//------------------------------------------------------------------------------------------------------


  HAL_Delay(400);//wait for stable power

  //MPU6050 Init
  HAL_Delay(2000);
  MPU6050rezulatat=MPU6050_check(&hi2c2);
  //HAL_Delay(2000);
  MPU6050_init(&hi2c2);
 // HAL_Delay(2000);
  MPU6050_DMP_Init(&hi2c2);
  //HAL_Delay(2000);

  HAL_Delay(400);//for stable MPU6050 readings after init

  //NRF24 INIT
  SPI2->CR1|=SPI_CR1_SPE; //enable SPI

  nRF24_CE_L(); // RX/TX disabled

  wifiOK=nRF24_Check();

  nRF24_Init(); //Default init

  // Disable ShockBurst for all RX pipes
  nRF24_DisableAA(0xFF);

  // Set RF channel
  nRF24_SetRFChannel(15); //2400Mhz + 15Mhz

  // Set data rate
  nRF24_SetDataRate(nRF24_DR_250kbps);

  // Set CRC scheme
  nRF24_SetCRCScheme(nRF24_CRC_1byte);

  // Set address width, its common for all pipes (RX and TX)
  nRF24_SetAddrWidth(3);

  nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); //PROGRAM PIPE1!! for RX

  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 8); // Auto-ACK: disabled


  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR);

  // Set TX power
  nRF24_SetTXPower(nRF24_TXPWR_18dBm);

  // Set operational mode
  nRF24_SetOperationalMode(nRF24_MODE_RX);

  // Clear any pending IRQ flags
  nRF24_ClearIRQFlags();

  // Wake the transceiver
  nRF24_SetPowerMode(nRF24_PWR_UP);

  nRF24_CE_H();//Enable RX

  //get GYRO offset
  /*HAL_Delay(2000);//wait to connect battery
  GyroCalibStatus=1;

  SUMGyroX=0;
  SUMGyroY=0;
  SUMGyroZ=0;

  for(i=0;i<1000;i++)
  {
	  MPU6050_gyroread(&hi2c2,&mpu6050DataStr);
	  SUMGyroX+=mpu6050DataStr.Gyroscope_X;
	  SUMGyroY+=mpu6050DataStr.Gyroscope_Y;
	  SUMGyroZ+=mpu6050DataStr.Gyroscope_Z;
	  HAL_Delay(1);
  }

  GyroXOff=SUMGyroX/1000;
  GyroYOff=SUMGyroY/1000;
  GyroZOff=SUMGyroZ/1000;

  GyroCalibStatus=0;

  //startup angles Accel to Gyro transfer--------------------------------------------------------------
  MPU6050_accread(&hi2c2,&mpu6050DataStr);

  Acc_vector=sqrt((mpu6050DataStr.Accelerometer_X * mpu6050DataStr.Accelerometer_X)+(mpu6050DataStr.Accelerometer_Y * mpu6050DataStr.Accelerometer_Y)+(mpu6050DataStr.Accelerometer_Z * mpu6050DataStr.Accelerometer_Z));
  AnglePitchAccel=asin((double)mpu6050DataStr.Accelerometer_Y/Acc_vector)*RADIANSTODEGREES;
  AngleRollAccel=-asin((double)mpu6050DataStr.Accelerometer_X/Acc_vector)*RADIANSTODEGREES;

  AnglePitchAccel-=ACCELPITCHMANUALOFFSET;
  AngleRollAccel-=ACCELROLLMANUALOFFSET;

  AnglePitchGyro=AnglePitchAccel;
  AngleRollGyro=AngleRollAccel;
  AngleYawGyro=0;*/

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  MotorStatus=MOTOROFF;

  MPU6050_DMP_Enable(&hi2c2,MPU6050_ADDRESS,1);//enable DMP (writing to FIFO)

  HAL_TIM_Base_Start_IT(&htim2);//Start at the END of Main Initialization

  ///-----------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //own function is used for UART TX, very basic function for direct Register write one char at the time
	  //HAL_UART_Transmit ( &huart1, UartTXbuff0, strlen( UartTXbuff0 ), 1 ); //removed ->missing bytes on occasion

	  HAL_Delay(100);

	  sprintf(UartTXbuff0,T_HIDE_CUR);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0,T_GO_TO,1,1); //Go to start Screen
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0,T_CLR_SCREEN);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Motor Status %u MPU=%u \n\r",MotorStatus,MPU6050rezulatat);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "ThrottleIN %.2f \n\r",ThrottleINscaled);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "PitchIN %.2f \n\r",PitchINscaled);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "RollIN %.2f \n\r",RollINscaled);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "YawIN %.2f \n\r",YawINscaled);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch DMP=%.2f \n\r",AnglesMPU6050_DMP.pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll DMP=%.2f \n\r",AnglesMPU6050_DMP.roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw DMP=%.2f \n\r",AnglesMPU6050_DMP.yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch=%.2f \n\r",AnglePitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll=%.2f \n\r",AngleRoll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw=%.2f \n\r",AngleYaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch ACC=%.2f GYRO=%.2f\n\r",AnglePitchAccel,AnglePitchGyro);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll ACC=%.2f GYRO=%.2f\n\r",AngleRollAccel,AngleRollGyro);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw GYRO=%.2f\n\r",AngleYawGyro);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "PWM 1:%u  2:%u  3:%u  4:%u   \n\r",PWM_Mot1,PWM_Mot2,PWM_Mot3,PWM_Mot4);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Toggle %d %d %d %d %d %d  ",togg1,togg2,togg3,togg4,togg5,togg6);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Potenc %d %d  ",potenc1,potenc2);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "YL %d %d  YD %d %d \n\r",Ljoyupdown, Ljoyleftright, Djoyupdown, Djoyleftright);
	  WriteString(UartTXbuff0);

	  //ACTIVE PID CONSTANTS
	  sprintf(UartTXbuff0, "\n\rPID Constants Active  \n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch P=%.2f I=%.5f D=%.2f \n\r",FlashDataActive.pid_p_gain_pitch, FlashDataActive.pid_i_gain_pitch, FlashDataActive.pid_d_gain_pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll P=%.2f I=%.5f D=%.2f \n\r",FlashDataActive.pid_p_gain_roll, FlashDataActive.pid_i_gain_roll, FlashDataActive.pid_d_gain_roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw P=%.2f I=%.5f D=%.2f \n\r",FlashDataActive.pid_p_gain_yaw, FlashDataActive.pid_i_gain_yaw, FlashDataActive.pid_d_gain_yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch Max %d Max I %d \n\r",FlashDataActive.pid_max_pitch, FlashDataActive.pid_i_max_pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll Max %d Max I %d \n\r",FlashDataActive.pid_max_roll, FlashDataActive.pid_i_max_roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw Max %d Max I %d \n\r",FlashDataActive.pid_max_yaw, FlashDataActive.pid_i_max_yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch Max Degree %.2f \n\r",FlashDataActive.maxpitchdegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll Max Degree %.2f \n\r",FlashDataActive.maxrolldegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw Max Degree %.2f \n\r",FlashDataActive.maxyawdegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Throttle Max %.2f Min I %.2f \n\r",FlashDataActive.maxthrottle, FlashDataActive.minthrottle);
	  WriteString(UartTXbuff0);


	  //FLASH PID CONSTANTS
	  sprintf(UartTXbuff0, "\n\rPID Constants Flash  \n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch P=%.2f I=%.5f D=%.2f \n\r",FlashDataFlash.pid_p_gain_pitch, FlashDataFlash.pid_i_gain_pitch, FlashDataFlash.pid_d_gain_pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll P=%.2f I=%.5f D=%.2f \n\r",FlashDataFlash.pid_p_gain_roll, FlashDataFlash.pid_i_gain_roll, FlashDataFlash.pid_d_gain_roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw P=%.2f I=%.5f D=%.2f \n\r",FlashDataFlash.pid_p_gain_yaw, FlashDataFlash.pid_i_gain_yaw, FlashDataFlash.pid_d_gain_yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch Max %d Max I %d \n\r",FlashDataFlash.pid_max_pitch, FlashDataFlash.pid_i_max_pitch);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll Max %d Max I %d \n\r",FlashDataFlash.pid_max_roll, FlashDataFlash.pid_i_max_roll);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw Max %d Max I %d \n\r",FlashDataFlash.pid_max_yaw, FlashDataFlash.pid_i_max_yaw);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Pitch Max Degree %.2f \n\r",FlashDataFlash.maxpitchdegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Roll Max Degree %.2f \n\r",FlashDataFlash.maxrolldegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Yaw Max Degree %.2f \n\r",FlashDataFlash.maxyawdegree);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "Throttle Max %.2f Min I %.2f \n\r",FlashDataFlash.maxthrottle, FlashDataFlash.minthrottle);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "\n\r" );
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "watch %d %d %d %d %d %d\n\r",watch1,watch2,watch3,watch4,watch5,watch6);
	  WriteString(UartTXbuff0);

	  sprintf(UartTXbuff0, "watch %.2f %.2f %.2f %.2f \n\r",watch1fl,watch2fl,watch3fl,watch4fl);
	  WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "%d %d %d %d\n\r",fifoBuffer[0],fifoBuffer[1],fifoBuffer[2],fifoBuffer[3]);
	  //WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "%d %d %d %d\n\r",fifoBuffer[4],fifoBuffer[5],fifoBuffer[6],fifoBuffer[7]);
	 // WriteString(UartTXbuff0);

	  //sprintf(UartTXbuff0, "%d %d %d %d\n\r",fifoBuffer[8],fifoBuffer[9],fifoBuffer[10],fifoBuffer[11]);
	 // WriteString(UartTXbuff0);

	 // sprintf(UartTXbuff0, "%d %d %d %d\n\r",fifoBuffer[12],fifoBuffer[13],fifoBuffer[14],fifoBuffer[15]);
	 // WriteString(UartTXbuff0);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF24_CE_Pin|TEST1_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU6050_INT_Pin */
  GPIO_InitStruct.Pin = MPU6050_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU6050_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_CE_Pin */
  GPIO_InitStruct.Pin = NRF24_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF24_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF24_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_CSN_Pin */
  GPIO_InitStruct.Pin = NRF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF24_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST1_PIN_Pin */
  GPIO_InitStruct.Pin = TEST1_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TEST1_PIN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
//Write Data into Flash starting from given address
void WriteFlashData(uint32_t StartAddr, struct FlashDatastruct *p)
{
	FLASH_EraseInitTypeDef EraseInitStruct;

	uint32_t PageError;

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = StartAddr;
	EraseInitStruct.NbPages     = 1;

	HAL_FLASH_Unlock();

	//FLASH_PageErase(0x800FC00); //doesn't handle all registers PER regiser in CR is not cleared

	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr, p->controlData);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+4, (uint32_t) ( p->pid_p_gain_pitch * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+8, (uint32_t) ( p->pid_i_gain_pitch * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+12,(uint32_t) ( p->pid_d_gain_pitch * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+16,(uint32_t) ( p->pid_p_gain_roll * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+20,(uint32_t) ( p->pid_i_gain_roll * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+24,(uint32_t) ( p->pid_d_gain_roll * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+28,(uint32_t) ( p->pid_p_gain_yaw * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+32,(uint32_t) ( p->pid_i_gain_yaw * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+36,(uint32_t) ( p->pid_d_gain_yaw * FLASHCONSTANTMULTIPLIER) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+40, p->pid_max_pitch);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+44, p->pid_i_max_pitch);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+48, p->pid_max_roll);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+52, p->pid_i_max_roll);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+56, p->pid_max_yaw);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+60, p->pid_i_max_yaw);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+64,(uint32_t)(p->maxpitchdegree) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+68,(uint32_t)(p->maxrolldegree) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+72,(uint32_t)(p->maxyawdegree) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+76,(uint32_t)(p->minthrottle) );
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,StartAddr+80,(uint32_t)(p->maxthrottle) );

	HAL_FLASH_Lock();
}

void EraseFlashData(uint32_t StartAddr)
{
	FLASH_EraseInitTypeDef EraseInitStruct;

	uint32_t PageError;

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = StartAddr;
	EraseInitStruct.NbPages     = 1;

	HAL_FLASH_Unlock();

	//FLASH_PageErase(0x800FC00); //doesn't handle all registers PER regiser in CR is not cleared

	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	HAL_FLASH_Lock();
}


//Check if Data on given address matches control word
uint32_t CheckFlashData(uint32_t StartAddr)
{
	return *(( uint32_t *) (StartAddr) );
}

//Read Data from Flash
void ReadFlashData(uint32_t StartAddr, struct FlashDatastruct *p)
{
	p->controlData= *(( uint32_t *) (StartAddr) );
	p->pid_p_gain_pitch=(float)( (*(( uint32_t *) (StartAddr+4) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_i_gain_pitch=(float)( (*(( uint32_t *) (StartAddr+8) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_d_gain_pitch=(float)( (*(( uint32_t *) (StartAddr+12) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_p_gain_roll=(float)( (*(( uint32_t *) (StartAddr+16) ))  )/FLASHCONSTANTMULTIPLIER;
	p->pid_i_gain_roll=(float)( (*(( uint32_t *) (StartAddr+20) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_d_gain_roll=(float)( (*(( uint32_t *) (StartAddr+24) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_p_gain_yaw=(float)( (*(( uint32_t *) (StartAddr+28) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_i_gain_yaw=(float)( (*(( uint32_t *) (StartAddr+32) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_d_gain_yaw=(float)( (*(( uint32_t *) (StartAddr+36) )) )/FLASHCONSTANTMULTIPLIER;
	p->pid_max_pitch=*(( uint32_t *) (StartAddr+40) );
	p->pid_i_max_pitch=*(( uint32_t *) (StartAddr+44) );
	p->pid_max_roll=*(( uint32_t *) (StartAddr+48) );
	p->pid_i_max_roll=*(( uint32_t *) (StartAddr+52) );
	p->pid_max_yaw=*(( uint32_t *) (StartAddr+56) );
	p->pid_i_max_yaw=*(( uint32_t *) (StartAddr+60) );
	p->maxpitchdegree=(float)( (*(( uint32_t *) (StartAddr+64) )) );
	p->maxrolldegree=(float)( (*(( uint32_t *) (StartAddr+68) )) );
	p->maxyawdegree=(float)( (*(( uint32_t *) (StartAddr+72) )) );
	p->minthrottle=(float)( (*(( uint32_t *) (StartAddr+76) )) );
	p->maxthrottle=(float)( (*(( uint32_t *) (StartAddr+80) )) );
}

void WriteString(char string[])
{
  unsigned int i=0;
  while (string[i])
	{
	PrintCharUart (string[i]);
	i++;
  }
}


void PrintCharUart (int ch) 	 /* Write character to Serial Port    */
{
	while (!(USART1->SR & USART_SR_TXE));
	USART1->DR = (USART_DR_DR & ch);
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
