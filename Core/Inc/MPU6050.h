#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_ADDRESS	0xD0

typedef enum {
	MPU6050_NOTDETECTED,
	MPU6050_DETECTED,
} MPU6050_Result;

typedef struct  {
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
} MPU6050str;

struct Quaternions
{
	float w;
	float x;
	float y;
	float z;
};

struct GravityVector
{
	float x;
	float y;
	float z;
};

struct Angles
{
	float yaw;
	float pitch;
	float roll;
};

void MPU6050_Write_Single_Bit(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddress, uint8_t RegisterAddress, uint8_t BitPosition, uint8_t BitValue);
void MPU6050_Write_Few_Bits(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddress, uint8_t RegisterAddress, uint8_t BitStart, uint8_t Length, uint8_t data);
uint8_t MPU6050_Read_Single_Bit(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddress, uint8_t RegisterAddress, uint8_t BitPosition);

MPU6050_Result MPU6050_check(I2C_HandleTypeDef* I2Cx);
void MPU6050_init(I2C_HandleTypeDef* I2Cx);
void MPU6050_accread(I2C_HandleTypeDef* I2Cx, MPU6050str* DataStruct);
void MPU6050_gyroread(I2C_HandleTypeDef* I2Cx, MPU6050str* DataStruct);

void MPU6050_DMP_Enable(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddress, uint8_t enable);
void MPU6050_DMP_Reset(I2C_HandleTypeDef* I2Cx,uint8_t DeviceAddres);
uint8_t  MPU6050_DMP_Get_Enable(I2C_HandleTypeDef* I2Cx);
uint8_t MPU6050_DMP_Init(I2C_HandleTypeDef* I2Cx);
void MPU6050_Set_Memory_Bank(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank);
void MPU6050_Set_Memory_Start_Address(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t address);
uint8_t MPU6050_Read_Memory_Bank(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);
uint8_t MPU6050_getOTPBankValid(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);
void MPU6050_Set_SlaveAddress(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t num, uint8_t address);
void MPU6050_Set_MasterModeEnable(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t enable);
void MPU6050_Reset_I2CMaster(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);
void MPU6050_Set_CLK_Source(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t source);
void MPU6050_Reset(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);
void MPU6050_SetSleepEnabled(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t enable);
void MPU6050_SetIntEnabled(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t enable);
void MPU6050_SetRate(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t rate);
void MPU6050_SetGyroRange(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t range);
void MPU6050_SetAccelRange(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t range);
uint8_t MPU6050_WriteMemoryBlock(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address,uint8_t verify, uint8_t useProgMem);
void SetExternalFrameSync(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t sync);
void SetDLPFMode(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t mode);
void MPU6050_DMPConfig1(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t config);
void MPU6050_DMPConfig2(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t config);
void MPU6050_SetOTPBankValid(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t enable);
void MPU6050_SetMotionDetectionThreshold(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t threshold);
void MPU6050_SetZeroMotionDetectionThreshold(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t threshold);
void MPU6050_SetMotionDetectionDuration(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t duration);
void MPU6050_SetZeroMotionDetectionDuration(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress, uint8_t duration);
void MPU6050_SetFIFOenabled(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t enable);
void MPU6050_ResetFIFO(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);
uint32_t MPU6050_GetCurrentFIFOPacket(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t *data,uint8_t length);
uint16_t MPU6050_GetFifoCount(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);
void MPU6050_GetFifoBytes(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress,uint8_t *data, uint8_t length);
uint8_t MPU6050_GetIntStatus(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);
uint8_t MPU6050_FifoOvreflowStatus(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);
uint8_t MPU6050_GetFIFOEnableStatus(I2C_HandleTypeDef* I2Cx, uint8_t DeviceAddress);

void CalculateQuaternions(struct Quaternions *q, uint8_t *fifodata);
void CalculateGravityVector(struct Quaternions *q, struct GravityVector *v);
void CalculateYawPitchRoll(struct Quaternions *q, struct GravityVector *v, struct Angles *ang);

#endif /* MPU6050_H */
