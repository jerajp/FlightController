/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define LED1_OFF  GPIOC->BSRR=0x00002000  	 // 1 << (13)  HW LED is inverted
#define LED1_ON GPIOC->BSRR=0x20000000       // 1 << (16+13)

//MotorStates
#define MOTORINIT 		0
#define MOTOROFF 		1
#define MOTORSTARTING 	2
#define MOTORRUNNING	3

//WIfi message type selector
#define CONTROLMOVEMENT 0
#define COMMERASEFLASH  1
#define COMWRITEFLASH	2
#define COMINPUTPARAM1	3

#define VDIVRESISTOR1	6800 //Battery voltage dividers
#define VDIVRESISTOR2 	1475
#define BATTADCTOMV	(float)( (float)((VDIVRESISTOR1 + VDIVRESISTOR2) * 3300) / (float)((VDIVRESISTOR2 * 4095)) )
#define BATTAVERAGETIME 50 //50 msec average
#define  MINMSGPERSEC   10
#define T_CLR_SCREEN 		"\x1b[2J"

#define GYROFACTORANGLE (float)( (1) / (65.5 * 500) )  // [deg]   units 2ms loop 500 readings per second
#define GYROFACTORANGLEDEG (float)( 1 / 65.5 )  	   // [deg/s] units 65.5 factor-from gyro set-up
#define DEGREESTORADIANS (float)( 0.017453292 )  //2conversion factor from degrees to radians
#define READIANSTODEGREES (float)(57.29578)
#define ACCELPITCHMANUALOFFSET (float)(0.2)		//spirit level offset in degrees
#define ACCELROLLMANUALOFFSET (float)(2.2)		//spirit level offset in degrees
#define GYROCALIBVALUES 1000

//SAFETY THROTTLE CHECK AT STARTUP
#define MOTORSTARTBLOCKTHRESHOLD	200  //max throttle stick position to allow start

//FLASH DATA SPECIFIC
#define FLASHCONSTADDR 0x800FC00
#define CONTROLWORD 7 //control word to check if Flash constants are present
#define FLASHCONSTANTMULTIPLIER 100000

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

struct FlashDatastruct
{
	uint32_t controlData;
	float pid_p_gain_pitch;  //Gain setting for the pitch P-controller
	float pid_i_gain_pitch;  //Gain setting for the pitch I-controller
	float pid_d_gain_pitch;  //Gain setting for the pitch D-controller
	float pid_p_gain_roll;   //Gain setting for the roll P-controller
	float pid_i_gain_roll;   //Gain setting for the roll I-controller
	float pid_d_gain_roll;   //Gain setting for the roll D-controller
	float pid_p_gain_yaw;    //Gain setting for the pitch P-controller
	float pid_i_gain_yaw;    //Gain setting for the pitch I-controller
	float pid_d_gain_yaw;    //Gain setting for the pitch D-controller
	int pid_max_pitch; 		 //Maximum output of the PID-controller (+/-)
	int pid_i_max_pitch; 	 //Maximum output of the Integral part
	int pid_max_roll;        //Maximum output of the PID-controller (+/-)
	int pid_i_max_roll;      //Maximum output of the Integral part
	int pid_max_yaw;         //Maximum output of the PID-controller (+/-)
	int pid_i_max_yaw;       //Maximum output of the Integral part
	float maxpitchdegree;    //degrees
	float maxrolldegree;     //degrees
	float maxyawdegree;      //degrees
	float minthrottle;       //80counts of 1000 to keep rotors spinning
	float maxthrottle;       //800counts of 1000 (80%)
};

void WriteFlashData(uint32_t flashstartaddr, struct FlashDatastruct *p);
void ReadFlashData(uint32_t flashstartaddr, struct FlashDatastruct *p);
void EraseFlashData(uint32_t StartAddr);
uint32_t CheckFlashData(uint32_t StartAddr);

void WriteString(char string[]);
void PrintCharUart (int ch);

extern uint32_t watch1,watch2,watch3,watch4,watch5,test1,test2,test3,test5;
extern uint16_t AdcDataArray[1];
extern uint32_t MotorStatus;
extern uint32_t GyroCalibStatus;

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;

//NRF24 data
extern uint8_t nRF24_payloadTX[32]; //TX buffer
extern uint8_t nRF24_payloadRX[32]; //RX buffer
extern uint8_t RXstpaketov;
extern const uint8_t nRF24_ADDR[3]; //Address
extern uint32_t Ljoyupdown;
extern uint32_t Ljoyleftright;
extern uint32_t Djoyupdown;
extern uint32_t Djoyleftright;
extern uint32_t potenc1;
extern uint32_t potenc2;
extern uint32_t togg1;
extern uint32_t togg2;
extern uint32_t togg3;
extern uint32_t togg4;
extern uint32_t togg5;
extern uint32_t togg6;

//MPU 6050
extern int16_t GyroXcal,GyroYcal,GyroZcal;
extern int16_t GyroXOff,GyroYOff,GyroZOff;
extern int32_t SUMGyroX,SUMGyroY,SUMGyroZ;
extern MPU6050str	mpu6050DataStr;

//Inputs
extern float ThrottleINscaled;
extern float PitchINscaled;
extern float RollINscaled;
extern float YawINscaled;

//PID motor control
extern uint32_t PWM_Mot1;
extern uint32_t PWM_Mot2;
extern uint32_t PWM_Mot3;
extern uint32_t PWM_Mot4;

//PID
extern float pid_output_pitch;
extern float pid_output_roll;
extern float pid_output_yaw;

//Flash structure
extern struct FlashDatastruct FlashDataActive;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define NRF24_CE_Pin GPIO_PIN_12
#define NRF24_CE_GPIO_Port GPIOB
#define NRF24_IRQ_Pin GPIO_PIN_12
#define NRF24_IRQ_GPIO_Port GPIOA
#define NRF24_CSN_Pin GPIO_PIN_15
#define NRF24_CSN_GPIO_Port GPIOA
#define TEST1_PIN_Pin GPIO_PIN_9
#define TEST1_PIN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
