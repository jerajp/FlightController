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
uint8_t TESTERFUCKING=0;

uint32_t Gyrocalibcount=0;
uint8_t AnglePitchDIR,AngleRollDIR;		//+- direction of angles for NRF24 sending
uint8_t AngleRollNRF24,AnglePitchNRF24; //positive angles for NRF24 sending
float AnglePitch, AnglePitchGyro,AnglePitchAccel;
float AngleRoll, AngleRollGyro, AngleRollAccel;
float Acc_vector;
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

  			//Check if Data is in correct ranges
  			if(nRF24_payloadRX[0]<=100 && nRF24_payloadRX[1]<=100 && nRF24_payloadRX[2]<=100 && nRF24_payloadRX[3]<=100)
  			{
  				Ljoyupdown=nRF24_payloadRX[0];
  				Ljoyleftright=nRF24_payloadRX[1];
  				Djoyupdown=nRF24_payloadRX[2];
  				Djoyleftright=nRF24_payloadRX[3];
  				potenc1=nRF24_payloadRX[4];
  				potenc2=nRF24_payloadRX[5];

  				togg1=nRF24_payloadRX[6]>>7;
  				togg2=(nRF24_payloadRX[6] & 64 )>>6;
  				togg3=(nRF24_payloadRX[6] & 32 )>>5;
  				togg4=(nRF24_payloadRX[6] & 16 )>>4;
  				togg5=(nRF24_payloadRX[6] & 8 )>>3;
  				togg6=(nRF24_payloadRX[6] & 4 )>>2;

  			}
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

  	 	 			nRF24_payloadTX[2] = (uint8_t)(AnglePitchNRF24);
  	 	 			nRF24_payloadTX[3] = (uint8_t)(AngleRollNRF24);
  	 	 			nRF24_payloadTX[4] = (uint8_t)(AnglePitchDIR + (AngleRollDIR<<1) + (GyroCalibStatus<<2) + ((MotorStatus & 0x7)<<3) ); //1bit Pitch DIR, 1bit Roll DIR, 1 bit GyroCalinFlag, 3 bit MotorStatus

  	 	 			// Transmit a packet
  	 	 			nRF24_TransmitPacket(nRF24_payloadTX, 5);
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

  AnglePitchGyro+=GyroXcal*GYROFACTORANGLE;
  AngleRollGyro+=GyroYcal*GYROFACTORANGLE;

  //correct angles with jaw axis correction
  AnglePitchGyro+=AngleRollGyro * sin(GyroZcal * DEGREESTORADIANS * GYROFACTORANGLE);
  AngleRollGyro-=AnglePitchGyro * sin(GyroZcal * DEGREESTORADIANS * GYROFACTORANGLE);

  //Accelerometer angles
  Acc_vector=sqrt((mpu6050DataStr.Accelerometer_X * mpu6050DataStr.Accelerometer_X)+(mpu6050DataStr.Accelerometer_Y * mpu6050DataStr.Accelerometer_Y)+(mpu6050DataStr.Accelerometer_Z * mpu6050DataStr.Accelerometer_Z));
  AnglePitchAccel=asin((float)mpu6050DataStr.Accelerometer_Y/Acc_vector)*READIANSTODEGREES;
  AngleRollAccel=-asin((float)mpu6050DataStr.Accelerometer_X/Acc_vector)*READIANSTODEGREES;

  AnglePitchAccel-=ACCELPITCHMANUALOFFSET;
  AngleRollAccel-=ACCELROLLMANUALOFFSET;

  AnglePitch=0.998*AnglePitchGyro + 0.002*AnglePitchAccel;
  AngleRoll=0.998*AngleRollGyro + 0.002*AngleRollAccel;


  //PID input Filtered
  PitchGyroPIDin =  (PitchGyroPIDin * 0.7) + (AnglePitch * 0.3);
  RollGyroPIDin = (RollGyroPIDin * 0.7) + (AngleRoll * 0.3);
  //YawGyroPIDin = (YawGyroPIDin * 0.7) + (GyroZcal * GYROFACTORANGLEDEG * 0.3);
  //-------------------------------------------------------------------

  //SCALE DATA
  //Input Controller Center to MAX 50 - >100  --->0-800 us
  ThrottleINscaled=ScaleDataFl(Ljoyupdown,0,100,MINTRHOTTLE,THROTTLESCALE);//throttle limit to 80%

  //TESTING potenciometer=throttle
  //ThrottleINscaled=ScaleDataFl(potenc1,0,100,0,1000);//direct 10-100 -->0-1000 testing

  //Pitch UP->DOWN 0-100 ->scaling
  PitchINscaled=ScaleDataFl(Djoyupdown,0,100,-MAXPITCHSCALE,+MAXPITCHSCALE);
  //Invert
  PitchINscaled*=(-1);

  //Roll LEFT->RIGHT 0 -> 100 -> scaling
  RollINscaled=ScaleDataFl(Djoyleftright,0,100,-MAXROLLSCALE,MAXROLLSCALE);

  //Roll LEFT->RIGHT 0 -> 100 ->scaling
  YawINscaled=ScaleDataFl(Ljoyleftright,0,100,-MAXYAWSCALE,MAXYAWSCALE);

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
  	  	  		  PWM_Mot1=1000 + ThrottleINscaled  - pid_output_pitch - pid_output_roll /*+ pid_output_yaw*/;
  	  		  	  PWM_Mot2=1000 + ThrottleINscaled  - pid_output_pitch + pid_output_roll /*- pid_output_yaw*/;
  	  		  	  PWM_Mot3=1000 + ThrottleINscaled  + pid_output_pitch + pid_output_roll /*+ pid_output_yaw*/;
  	  		  	  PWM_Mot4=1000 + ThrottleINscaled  + pid_output_pitch - pid_output_roll /*- pid_output_yaw*/;

  	  		  	  //MIN OBRATI
  	  		  	  if(PWM_Mot1 < (1000+ MINTRHOTTLE))PWM_Mot1=(1000+ MINTRHOTTLE);
				  if(PWM_Mot2 < (1000+ MINTRHOTTLE))PWM_Mot2=(1000+ MINTRHOTTLE);
				  if(PWM_Mot3 < (1000+ MINTRHOTTLE))PWM_Mot3=(1000+ MINTRHOTTLE);
				  if(PWM_Mot4 < (1000+ MINTRHOTTLE))PWM_Mot4=(1000+ MINTRHOTTLE);

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
