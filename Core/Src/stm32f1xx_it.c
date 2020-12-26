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

uint32_t FlashEraseFlag;
uint32_t FlashWriteFlag;
uint32_t MSGSelector;
uint32_t ParamSelector;
uint32_t SendParamActiveFlag;
uint32_t SendParamFlashFlag;
uint32_t ParamDataTX;
uint32_t ParamDataRX;
struct FlashDatastruct FlashDataTemp;
uint32_t FlashWriteTimeoutCount;
uint32_t FlashEraseTimeoutCount;

uint32_t Gyrocalibcount=0;
uint8_t AnglePitchDIR,AngleRollDIR;		//+- direction of angles for NRF24 sending
uint8_t AngleRollNRF24,AnglePitchNRF24; //positive angles for NRF24 sending
double AnglePitch, AnglePitchGyro,AnglePitchAccel;
double AngleRoll, AngleRollGyro, AngleRollAccel;
double AngleYaw, AngleYawGyro;

double Acc_vector;
uint8_t StartupAngleSet=0;

//SCALED INPUTS
float ThrottleINscaled=0;
float PitchINscaled=0;
float RollINscaled=0;
float YawINscaled=0;

float PitchGyroPIDin=0;
float RollGyroPIDin=0;
float YawGyroPIDin=0;

//PID
float pid_output_pitch=0;
float pid_output_roll=0;
float pid_output_yaw=0;

float pitch_integral=0;
float pitch_diffErrHist=0;
float roll_integral=0;
float roll_diffErrHist=0;
float yaw_integral=0;
float yaw_diffErrHist=0;

uint32_t togg1hist;
uint32_t togg2hist;
uint32_t togg3hist;
uint32_t togg4hist;
uint32_t togg5hist;
uint32_t togg4hist;
uint32_t togg6hist;

uint16_t temp_ACC_X;
uint16_t temp_ACC_Y;
uint16_t temp_ACC_Z;
uint16_t temp_GYR_X;
uint16_t temp_GYR_Y;
uint16_t temp_GYR_Z;


uint8_t dataTEST[200];

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

  HAL_GPIO_WritePin(TEST1_PIN_GPIO_Port,TEST1_PIN_Pin,GPIO_PIN_SET);

  //blinky
  LEDcount++;
  if(LEDcount>=250)
  {
	  LEDcount=0;
	  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
  }



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

  //save OLD toggle values
  togg1hist=togg1;
  togg2hist=togg2;
  togg3hist=togg3;
  togg4hist=togg4;
  togg5hist=togg5;
  togg6hist=togg6;

  //NRF24--------------------------------------------------------------------
  //Ping for RX data when RXflag is SET
  if(RXactiveFlag)
  {
  		if ((nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) )
  		{

  			// Get a payload from the transceiver
  			nRF24_ReadPayload(nRF24_payloadRX, &RXstpaketov);

  			// Clear all pending IRQ flags
  			nRF24_ClearIRQFlags();

  			MSGSelector=(nRF24_payloadRX[0]);

  			switch(MSGSelector)
  			{
  				case COMMCONTROLDATA:
  									{
  										if(nRF24_payloadRX[1]<=100 && nRF24_payloadRX[2]<=100 && nRF24_payloadRX[3]<=100 && nRF24_payloadRX[4]<=100)//Check if Data is in correct ranges
  										{
  							  				Ljoyupdown=nRF24_payloadRX[1];
  							  				Ljoyleftright=nRF24_payloadRX[2];
  							  				Djoyupdown=nRF24_payloadRX[3];
  							  				Djoyleftright=nRF24_payloadRX[4];
  							  				potenc1=nRF24_payloadRX[5];
  							  				potenc2=nRF24_payloadRX[6];

  							  				togg1=nRF24_payloadRX[7]>>7;
  							  				togg2=(nRF24_payloadRX[7] & 64 )>>6;
  							  				togg3=(nRF24_payloadRX[7] & 32 )>>5;
  							  				togg4=(nRF24_payloadRX[7] & 16 )>>4;
  							  				togg5=(nRF24_payloadRX[7] & 8 )>>3;
  							  				togg6=(nRF24_payloadRX[7] & 4 )>>2;
  										}
  									}break;

  				case COMMERASEFLASHDR:
  									{
  										FlashEraseFlag=1;

  									}break;

  				case COMMWRITEFLASHDR:
  									{
  										FlashWriteFlag=1;

  									}break;

  				case COMMPARAMACTIVE:
  				  					{
  				  						ParamSelector=nRF24_payloadRX[1];
  				  						ParamDataRX=(nRF24_payloadRX[2]<<24) + (nRF24_payloadRX[3]<<16) + (nRF24_payloadRX[4]<<8) + (nRF24_payloadRX[5]);

  				  					}break;

  				case COMMPARAMFLASH:
  				  					{
  				  						ParamSelector=nRF24_payloadRX[1];

  				  					}break;


  			}


  			if(MSGSelector==COMMPARAMACTIVE)//save values in active structure
  			{
  				switch(ParamSelector)
  				{
					case  PARAM1 :{FlashDataActive.pid_p_gain_pitch=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM2 :{FlashDataActive.pid_i_gain_pitch=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM3 :{FlashDataActive.pid_d_gain_pitch=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM4 :{FlashDataActive.pid_p_gain_roll=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM5 :{FlashDataActive.pid_i_gain_roll=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM6 :{FlashDataActive.pid_d_gain_roll=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM7 :{FlashDataActive.pid_p_gain_yaw=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM8 :{FlashDataActive.pid_i_gain_yaw=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM9 :{FlashDataActive.pid_d_gain_yaw=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM10 :{FlashDataActive.pid_max_pitch=(int)(ParamDataRX/FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM11 :{FlashDataActive.pid_i_max_pitch=(int)(ParamDataRX/FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM12 :{FlashDataActive.pid_max_roll=(int)(ParamDataRX/FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM13 :{FlashDataActive.pid_i_max_roll=(int)(ParamDataRX/FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM14 :{FlashDataActive.pid_max_yaw=(int)(ParamDataRX/FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM15 :{FlashDataActive.pid_i_max_yaw=(int)(ParamDataRX/FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM16 :{FlashDataActive.maxpitchdegree=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM17 :{FlashDataActive.maxrolldegree=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM18 :{FlashDataActive.maxyawdegree=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM19 :{FlashDataActive.minthrottle=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
					case  PARAM20 :{FlashDataActive.maxthrottle=(float)(ParamDataRX)/(float)(FLASHCONSTANTMULTIPLIER);}break;
  				}
  			}



  			SendBackFlag=1;
  			RXactiveFlag=0;

  			MSGcount++;
  		}
  }
  if(SendBackFlag)//Config between RX-TX
  {
    	BackTimer++;

    	//Calculate param for transfer
    	 if(MSGSelector==COMMPARAMACTIVE || MSGSelector==COMMPARAMFLASH)
    	 {
    		 if(MSGSelector==COMMPARAMACTIVE)FlashDataTemp=FlashDataActive;
    		 else FlashDataTemp=FlashDataFlash;

    		 switch(ParamSelector)
    		 {
    	 	 	 case PARAM1: {ParamDataTX=FlashDataTemp.pid_p_gain_pitch*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM2: {ParamDataTX=FlashDataTemp.pid_i_gain_pitch*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM3: {ParamDataTX=FlashDataTemp.pid_d_gain_pitch*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM4: {ParamDataTX=FlashDataTemp.pid_p_gain_roll*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM5: {ParamDataTX=FlashDataTemp.pid_i_gain_roll*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM6: {ParamDataTX=FlashDataTemp.pid_d_gain_roll*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM7: {ParamDataTX=FlashDataTemp.pid_p_gain_yaw*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM8: {ParamDataTX=FlashDataTemp.pid_i_gain_yaw*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM9: {ParamDataTX=FlashDataTemp.pid_d_gain_yaw*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM10: {ParamDataTX=FlashDataTemp.pid_max_pitch*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM11: {ParamDataTX=FlashDataTemp.pid_i_max_pitch*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM12: {ParamDataTX=FlashDataTemp.pid_max_roll*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM13: {ParamDataTX=FlashDataTemp.pid_i_max_roll*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM14: {ParamDataTX=FlashDataTemp.pid_max_yaw*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM15: {ParamDataTX=FlashDataTemp.pid_i_max_yaw*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM16: {ParamDataTX=FlashDataTemp.maxpitchdegree*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM17: {ParamDataTX=FlashDataTemp.maxrolldegree*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM18: {ParamDataTX=FlashDataTemp.maxyawdegree*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM19: {ParamDataTX=FlashDataTemp.minthrottle*FLASHCONSTANTMULTIPLIER;}break;
    	 	 	 case PARAM20: {ParamDataTX=FlashDataTemp.maxthrottle*FLASHCONSTANTMULTIPLIER;}break;
    		 }
    	 }

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
  	 	 	 		 nRF24_payloadTX[0] = MSGSelector;

  	 	 	 		switch(MSGSelector)
  	 	 	 		{
  	 	 	 			case COMMCONTROLDATA:
  	 	 	 								{
  	 	 	 									nRF24_payloadTX[1] = (uint8_t)(BattmVAVG & 0xFF);
  	 	 	 									nRF24_payloadTX[2] = (uint8_t)((BattmVAVG & 0xFF00)>>8);

  	 	 	 								  	//save Angle for NRF24 transfer
  	 	 	 								  	if(AnglePitch<0)
  	 	 	 								  	{
  	 	 	 								  		AnglePitchDIR=1;
  	 	 	 								  	 	AnglePitchNRF24=AnglePitch*(-1);
  	 	 	 								  	}
  	 	 	 								  	else
  	 	 	 								  	{
  	 	 	 								  		AnglePitchDIR=0;
  	 	 	 								  	 	AnglePitchNRF24=AnglePitch;
  	 	 	 								  	}

  	 	 	 								  	if(AngleRoll<0)
  	 	 	 								  	{
  	 	 	 								  		AngleRollDIR=1;
  	 	 	 								  		AngleRollNRF24=AngleRoll*(-1);
  	 	 	 								  	}
  	 	 	 								  	else
  	 	 	 								  	{
  	 	 	 								  		AngleRollDIR=0;
  	 	 	 								  		AngleRollNRF24=AngleRoll;
  	 	 	 								  	}

  	 	 	 								  	nRF24_payloadTX[3] = (uint8_t)(AnglePitchNRF24);
  	 	 	 								  	nRF24_payloadTX[4] = (uint8_t)(AngleRollNRF24);
  	 	 	 								  	nRF24_payloadTX[5] = (uint8_t)(AnglePitchDIR + (AngleRollDIR<<1) + (GyroCalibStatus<<2) + ((MotorStatus & 0x7)<<3) ); //1bit Pitch DIR, 1bit Roll DIR, 1 bit GyroCalinFlag, 3 bit MotorStatus
  	 	 	 								}break;
  	 	 	 			case COMMERASEFLASHDR:
  	 	 	 								{
  	 	 	 									nRF24_payloadTX[1]=0;
  	 	 	 									nRF24_payloadTX[2]=0;
  	 	 	 							  	 	nRF24_payloadTX[3]=0;
  	 	 	 							  	 	nRF24_payloadTX[4]=0;
  	 	 	 							  	 	nRF24_payloadTX[5]=0;
  	 	 	 							  	 	FlashEraseTimeoutCount=FLASHOPERATIONTIMEOUT;//set timeout to prevent multiple calls
  	 	 	 								}break;

  	 	 	 			case COMMWRITEFLASHDR:
  	 	 	 								{
  	 	 	 									nRF24_payloadTX[1]=0;
  	 	 	 						  	 	  	nRF24_payloadTX[2]=0;
  	 	 	 						  	 	  	nRF24_payloadTX[3]=0;
  	 	 	 						  	 	  	nRF24_payloadTX[4]=0;
  	 	 	 						  	 	  	nRF24_payloadTX[5]=0;
  	 	 	 						  	 	  	FlashWriteTimeoutCount=FLASHOPERATIONTIMEOUT; //set timeout to prevent multiple calls
  	 	 	 								}break;

  	 	 	 			case COMMPARAMACTIVE:
											{
  	 	 	 									nRF24_payloadTX[1] = ParamSelector;
  	 	 	 									nRF24_payloadTX[2] = (ParamDataTX & 0xFF000000)>>24;
  	 	 	 									nRF24_payloadTX[3] = (ParamDataTX & 0x00FF0000)>>16;
  	 	 	 									nRF24_payloadTX[4] = (ParamDataTX & 0x0000FF00)>>8;
  	 	 	 									nRF24_payloadTX[5] = (ParamDataTX & 0x000000FF);
											}break;

  	 	 	 			case COMMPARAMFLASH:
  	 	 	 								{
  	 	 	 									nRF24_payloadTX[1] = ParamSelector;
  	 	 	 									nRF24_payloadTX[2] = (ParamDataTX & 0xFF000000)>>24;
  	 	 	 									nRF24_payloadTX[3] = (ParamDataTX & 0x00FF0000)>>16;
  	 	 	 									nRF24_payloadTX[4] = (ParamDataTX & 0x0000FF00)>>8;
  	 	 	 									nRF24_payloadTX[5] = (ParamDataTX & 0x000000FF);
  	 	 	 								}break;
  	 	 	 		}

  	 	 			// Transmit a packet
  	 	 			nRF24_TransmitPacket(nRF24_payloadTX, 6);
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
  MPU6050_GetCurrentFIFOPacket(&hi2c2,MPU6050_ADDRESS,fifoBuffer,packetSize);
  CalculateQuaternions(&QuaternionMPU60500,fifoBuffer);
  CalculateGravityVector(&QuaternionMPU60500, &GravityVectorMPU6050);
  CalculateYawPitchRoll(&QuaternionMPU60500, &GravityVectorMPU6050,&AnglesMPU6050_DMP);

  AnglePitch=AnglesMPU6050_DMP.pitch;
  AngleRoll=AnglesMPU6050_DMP.roll;
  AngleYaw=AnglesMPU6050_DMP.yaw;

  //PID input Filtered
  PitchGyroPIDin =  (PitchGyroPIDin * 0.7) + (AnglePitch * 0.3);
  RollGyroPIDin = (RollGyroPIDin * 0.7) + (AngleRoll * 0.3);
  YawGyroPIDin = (YawGyroPIDin * 0.7) + (AngleYaw * 0.3);
  //-------------------------------------------------------------------

  //SCALE DATA
  //Throttle UP->DOWN 0-100 ->scaling
  ThrottleINscaled=ScaleDataFl(Ljoyupdown,0,100,FlashDataActive.minthrottle,FlashDataActive.maxthrottle);

  //Pitch UP->DOWN 0-100 ->scaling
  PitchINscaled=ScaleDataFl(Djoyupdown,0,100,-FlashDataActive.maxpitchdegree,FlashDataActive.maxpitchdegree);
  //Invert
  PitchINscaled*=(-1);

  //Roll LEFT->RIGHT 0 -> 100 -> scaling
  RollINscaled=ScaleDataFl(Djoyleftright,0,100,-FlashDataActive.maxrolldegree,FlashDataActive.maxrolldegree);

  //Roll LEFT->RIGHT 0 -> 100 ->scaling
  YawINscaled=ScaleDataFl(Ljoyleftright,0,100,-FlashDataActive.maxyawdegree,FlashDataActive.maxyawdegree);

  //MOTOR CONTROL

  //PID
  pid_output_pitch = pid(PitchINscaled, PitchGyroPIDin, FlashDataActive.pid_p_gain_pitch, FlashDataActive.pid_i_gain_pitch, FlashDataActive.pid_d_gain_pitch, &pitch_integral, &pitch_diffErrHist, FlashDataActive.pid_i_max_pitch, FlashDataActive.pid_max_pitch);
  pid_output_roll = pid(RollINscaled, RollGyroPIDin, FlashDataActive.pid_p_gain_roll, FlashDataActive.pid_i_gain_roll, FlashDataActive.pid_d_gain_roll,&roll_integral,&roll_diffErrHist,FlashDataActive.pid_i_max_roll, FlashDataActive.pid_max_roll );
  pid_output_yaw = pid(YawINscaled, YawGyroPIDin, FlashDataActive.pid_p_gain_yaw, FlashDataActive.pid_i_gain_yaw, FlashDataActive.pid_d_gain_yaw, &yaw_integral,&yaw_diffErrHist,FlashDataActive.pid_i_max_roll, FlashDataActive.pid_max_yaw );

  //TESTING
  if(ConnectWeakFlag==1)MotorStatus=MOTOROFF;//if connection is lost!

  //Motor STATUS (TOGGLE 1)
  //ON toggle 0->1 front start motor ON sequence
  if(togg1hist!=togg1 && togg1==1 && ThrottleINscaled<MOTORSTARTBLOCKTHRESHOLD)MotorStatus=MOTORSTARTING;

  //ON toggle 0-> motor always OFF
  if(togg1==0)MotorStatus=MOTOROFF;

  //GYROCALIB-----------------------------------------------------------------------------------------
  if(togg2hist==0 && togg2==1 && GyroCalibStatus==0 && MotorStatus==MOTOROFF) //button 2 pressed Motor OFF Calib not in progress
  {
	  GyroCalibStatus=1;
	  SUMGyroX=0;
	  SUMGyroY=0;
	  SUMGyroZ=0;
	  Gyrocalibcount=0;
  }

  //Write and Erase Flash operation timeout to prevent multiple calls in sequence
  if(FlashWriteTimeoutCount>0)FlashWriteTimeoutCount--;
  if(FlashEraseTimeoutCount>0)FlashEraseTimeoutCount--;

  //Write active parameters in flash
  if(FlashWriteFlag && MotorStatus==MOTOROFF && FlashWriteTimeoutCount==0)
  {
	  WriteFlashData(FLASHCONSTADDR, &FlashDataActive);
	  ReadFlashData(FLASHCONSTADDR, &FlashDataFlash);//Read back values to Flash structure
	  FlashWriteFlag=0;//reset
	  watch1++;
  }

  //Erase Flash Data
  if(FlashEraseFlag && MotorStatus==MOTOROFF && FlashEraseTimeoutCount==0)
  {
	  EraseFlashData(FLASHCONSTADDR);
	  FlashEraseFlag=0;//reset
	  watch2++;
  }

  if(GyroCalibStatus==1)
  {
	  SUMGyroX+=mpu6050DataStr.Gyroscope_X;
	  SUMGyroY+=mpu6050DataStr.Gyroscope_Y;
	  SUMGyroZ+=mpu6050DataStr.Gyroscope_Z;
	  Gyrocalibcount++;

	  if(Gyrocalibcount==GYROCALIBVALUES)
	  {

		  GyroXOff=SUMGyroX/GYROCALIBVALUES;
		  GyroYOff=SUMGyroY/GYROCALIBVALUES;
		  GyroZOff=SUMGyroZ/GYROCALIBVALUES;

		  //startup angles Accel to Gyro transfer
		  AnglePitchGyro=AnglePitchAccel;
		  AngleRollGyro=AngleRollAccel;

		  GyroCalibStatus=0;
	  }
  }//--------------------------------------------------------------------------------------------------

  if(MotorStatus==MOTORSTARTING)
  {
	  //startup angles Accel to Gyro transfer
	  AnglePitchGyro=AnglePitchAccel;
	  AngleRollGyro=AngleRollAccel;

	  if(GyroCalibStatus==0)//only if calib is finished allow transition
	  MotorStatus=MOTORRUNNING;
  }

  //MOT 1 FRONT LEFT  CW
  //MOT 2 FRONT RIGHT CCW
  //MOT 3 BACK  RIGHT CW
  //MOT 4 BACK  LEFT  CCW
  switch(MotorStatus)
  {
  	  case MOTORRUNNING:
  	  	  	  {
  	  	  		  PWM_Mot1=1000 + ThrottleINscaled  - pid_output_pitch - pid_output_roll + pid_output_yaw;
  	  		  	  PWM_Mot2=1000 + ThrottleINscaled  - pid_output_pitch + pid_output_roll - pid_output_yaw;
  	  		  	  PWM_Mot3=1000 + ThrottleINscaled  + pid_output_pitch + pid_output_roll + pid_output_yaw;
  	  		  	  PWM_Mot4=1000 + ThrottleINscaled  + pid_output_pitch - pid_output_roll - pid_output_yaw;

  	  		  	  //MIN OBRATI
  	  		  	  if(PWM_Mot1 < (1000+ FlashDataActive.minthrottle))PWM_Mot1=(1000+ FlashDataActive.minthrottle);
				  if(PWM_Mot2 < (1000+ FlashDataActive.minthrottle))PWM_Mot2=(1000+ FlashDataActive.minthrottle);
				  if(PWM_Mot3 < (1000+ FlashDataActive.minthrottle))PWM_Mot3=(1000+ FlashDataActive.minthrottle);
				  if(PWM_Mot4 < (1000+ FlashDataActive.minthrottle))PWM_Mot4=(1000+ FlashDataActive.minthrottle);

			  	  //MAX OBRATI
	  	  		  if(PWM_Mot1 > 1950)PWM_Mot1=1950;
				  if(PWM_Mot2 > 1950)PWM_Mot2=1950;
				  if(PWM_Mot3 > 1950)PWM_Mot3=1950;
				  if(PWM_Mot4 > 1950)PWM_Mot4=1950;

  	  	  	  }break;

  	  default:
  	  	  	  {
  	  	  		  PWM_Mot1=900;
  		  	  	  PWM_Mot2=900;
  		  	  	  PWM_Mot3=900;
  		  	  	  PWM_Mot4=900;

  		  	  	  //Reset PID
  		  	  	  pitch_integral=0;
  		  	  	  pitch_diffErrHist=0;
  		  	  	  roll_integral=0;
  		  	  	  roll_diffErrHist=0;
  		  	  	  yaw_integral=0;
  		  	  	  yaw_diffErrHist=0;

  	  	  	  }break;
  }

  //SET PWM CHANNELS-----------------------------------------------------
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_Mot1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_Mot2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_Mot3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_Mot4);

  HAL_GPIO_WritePin(TEST1_PIN_GPIO_Port,TEST1_PIN_Pin,GPIO_PIN_RESET);

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
float ScaleDataFl(float in_value,float in_min,float in_max, float out_min, float out_max)
{
	float factor;
	float out;

	factor=(out_max-out_min)/(in_max-in_min);
	out=(in_value-in_min)*factor+out_min;
	if(out<out_min)out=out_min;

	return out;

}

float pid(float pid_reference, float pid_input, float pid_p, float pid_i, float pid_d, float *integral, float *diffErrHist, float PIDimax, float PIDmax)
{
	float out;
	float pid_error_temp;

	//Erro calculation
	pid_error_temp = pid_input - pid_reference;

	//Integral part + saturation
	*integral += pid_i * pid_error_temp;
	if(*integral > PIDimax)*integral = PIDimax;
	else if(*integral < PIDimax * -1)*integral = PIDimax * -1;

	out = pid_p * pid_error_temp + *integral + pid_d * (pid_error_temp - *diffErrHist);

	if(out > PIDmax)out = PIDmax;
	else if(out < PIDmax * -1)out = PIDmax * -1;

	//save Error for next cylce D calculation
	*diffErrHist = pid_error_temp;


	return out;
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
