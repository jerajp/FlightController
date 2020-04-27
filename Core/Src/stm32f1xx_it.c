/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
#include "MPU6050.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t i;
uint32_t LEDcount;

uint32_t PWM_Mot1;
uint32_t PWM_Mot2;
uint32_t PWM_Mot3;
uint32_t PWM_Mot4;

uint32_t BattmV;
uint32_t BattmVSUM=0;
uint32_t BattmVAVG=0;
uint32_t batthistindx=0;
uint32_t BAttmVhist[BATTAVERAGETIME];

uint32_t SendBackFlag=0;
uint32_t RXactiveFlag=1;
uint32_t BackTimer=0;

uint32_t LoopCounter;
uint32_t MSGprerSecond;
uint32_t MSGcount;
uint32_t ConnectWeakFlag;
uint32_t MSGLowCount;

float AnglePitch, AnglePitchGyro,AnglePitchAccel;
float AngleRoll, AngleRollGyro, AngleRollAccel;
float Acc_vector;
uint8_t StartupAngleSet=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  //blinky
  LEDcount++;
  if(LEDcount>=250)
  {
	  LEDcount=0;
	  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
  }

  HAL_GPIO_WritePin(TEST1_PIN_GPIO_Port,TEST1_PIN_Pin,GPIO_PIN_SET);

  //Read Battery Voltage-----------------------------------------------
  HAL_ADC_PollForConversion(&hadc1,10);
  BattmV=HAL_ADC_GetValue(&hadc1)*BATTADCTOMV;

  //Battery average value-----------------------------------------------
  BAttmVhist[batthistindx]=BattmV;
  batthistindx++;

  if(batthistindx >= BATTAVERAGETIME)batthistindx=0;

  BattmVSUM=0;

  for(i=0;i<BATTAVERAGETIME;i++)
  {
	  BattmVSUM+=BAttmVhist[i];
  }

  BattmVAVG=BattmVSUM/(BATTAVERAGETIME);
  //-------------------------------------------------------------------------


  //NRF24--------------------------------------------------------------------
  //Ping for RX data when RXflag is SET
  if(RXactiveFlag)
  {
  		if ((nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) )
  		{
  			watch1++;

  			// Get a payload from the transceiver
  			nRF24_ReadPayload(nRF24_payloadRX, &RXstpaketov);

  			// Clear all pending IRQ flags
  			nRF24_ClearIRQFlags();

  			Ljoyupdown=nRF24_payloadRX[0];
  			Ljoyleftright=nRF24_payloadRX[1];
  			Djoyupdown=nRF24_payloadRX[2];
  			Djoyleftright=nRF24_payloadRX[3];
  			potenc1=nRF24_payloadRX[4];
  			potenc2=nRF24_payloadRX[5];

  			togg1=nRF24_payloadRX[6]>>7;
  			togg2=(nRF24_payloadRX[6] & 64 )>>6;
  			butt1=(nRF24_payloadRX[6] & 32 )>>5;
  			butt2=(nRF24_payloadRX[6] & 16 )>>4;
  			butt3=(nRF24_payloadRX[6] & 8 )>>3;
  			butt4=(nRF24_payloadRX[6] & 4 )>>2;
  			buttL=(nRF24_payloadRX[6] & 2 )>>1;
  			buttD=(nRF24_payloadRX[6] & 1 );

  			SendBackFlag=1;
  			RXactiveFlag=0;

  			MSGcount++;
  		}
  }
  if(SendBackFlag)//Config between RX-TX
  {
    	BackTimer++;

      	switch(BackTimer)
      	{
  	 	 case 1:
  	 	 	 	 {
  	 	 	 		//SET TX MODE
  	 	 	 		nRF24_CE_L();//END RX
  					nRF24_SetOperationalMode(nRF24_MODE_TX);
  	 	 	 	 }break;

  	 	 case 4:
  	 	 	 	 {
  	 	 			//SEND DATA TO RC remote
  	 	 			nRF24_payloadTX[0] = (uint8_t)(BattmVAVG & 0xFF);
  	 	 			nRF24_payloadTX[1] = (uint8_t)((BattmVAVG & 0xFF00)>>8);

  	 	 			// Transmit a packet
  	 	 			nRF24_TransmitPacket(nRF24_payloadTX, 2);
  	 	 			watch2++;
  	 	 	 	 }break;

      	case 5:
      			{
  	 	 	 		//SET RX MODE
  	 				nRF24_SetOperationalMode(nRF24_MODE_RX);
  	 				nRF24_CE_H(); //Start RX)

  	 	 	 	 }break;

      	case 6:
  				{
      				RXactiveFlag=1; //start pinging for data
      				SendBackFlag=0; //Exit routine
      				BackTimer=0;	//reset counter

  				}break;
      	}
  }//End Send Back config routine

  //Communication diagnostics
  LoopCounter++;
  if(LoopCounter==1000)
  {
    		MSGprerSecond=MSGcount;

    		if(MSGcount<MINMSGPERSEC)
    		{
    			MSGLowCount++;
    			ConnectWeakFlag=1;
    		}
    		 else  ConnectWeakFlag=0;

    		MSGcount=0;
    		LoopCounter=0;
  }//-----------------------------------------------------------------

  //MPU 6050-----------------------------------------------------------

  MPU6050_accread(&hi2c2,&mpu6050DataStr);
  MPU6050_gyroread(&hi2c2,&mpu6050DataStr);

  GyroXcal=mpu6050DataStr.Gyroscope_X - GyroXOff;
  GyroYcal=mpu6050DataStr.Gyroscope_Y - GyroYOff;
  GyroZcal=mpu6050DataStr.Gyroscope_Z - GyroZOff;

  AnglePitchGyro+=GyroXcal*GYROFACTOR;
  AngleRollGyro+=GyroYcal*GYROFACTOR;

  //correct angles with jaw axis correction
  AnglePitchGyro+=AngleRollGyro * sin(GyroZcal * DEGREESTORADIANS * GYROFACTOR);
  AngleRollGyro-=AnglePitchGyro * sin(GyroZcal * DEGREESTORADIANS * GYROFACTOR);

  //Accelerometer angles
  Acc_vector=sqrt((mpu6050DataStr.Accelerometer_X * mpu6050DataStr.Accelerometer_X)+(mpu6050DataStr.Accelerometer_Y * mpu6050DataStr.Accelerometer_Y)+(mpu6050DataStr.Accelerometer_Z * mpu6050DataStr.Accelerometer_Z));
  AnglePitchAccel=asin((float)mpu6050DataStr.Accelerometer_Y/Acc_vector)*READIANSTODEGREES;
  AngleRollAccel=asin((float)mpu6050DataStr.Accelerometer_X/Acc_vector)*READIANSTODEGREES;

  AnglePitchAccel-=ACCELPITCHMANUALOFFSET;
  AngleRollAccel-=ACCELROLLMANUALOFFSET;

  if(!StartupAngleSet)
  {
	  AnglePitch=AnglePitchAccel;
	  AngleRoll=AngleRollAccel;
	  StartupAngleSet=1; //First angles set to accelorometer at startup
  }
  else
  {
	  AnglePitch=0.98*AnglePitchGyro + 0.02*AnglePitchAccel;
	  AngleRoll=0.98*AngleRollGyro + 0.02*AngleRollAccel;

  }
  //-------------------------------------------------------------------

  //testing------------------------------------------------------------
  if(togg1==1 && ConnectWeakFlag==0)MotorStatus=1;
  else MotorStatus=0;

  if(MotorStatus==0)
  {
	  PWM_Mot1=1000;
	  PWM_Mot2=1000;
	  PWM_Mot3=1000;
	  PWM_Mot4=1000;
  }
  else if(MotorStatus==1)
  {
	  PWM_Mot1=1000 + potenc1*10;
	  PWM_Mot2=1000 + potenc1*10;
	  PWM_Mot3=1000 + potenc1*10;
	  PWM_Mot4=1000 + potenc1*10;
  }//-------------------------------------------------------------------

  //SET PWM CHANNELS-----------------------------------------------------
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_Mot1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_Mot2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_Mot3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_Mot4);

  HAL_GPIO_WritePin(TEST1_PIN_GPIO_Port,TEST1_PIN_Pin,GPIO_PIN_RESET);

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
