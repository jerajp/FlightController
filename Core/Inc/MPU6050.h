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

#define MPU6050_I_AM_VAL				0x68 //value of register WHO_AM_I

//registers
#define MPU6050_WHO_AM_I				0x75
#define MPU6050_RA_PWR_MGMT_1			0x6B
#define MPU6050_RA_SMPLRT_DIV			0x19
#define MPU6050_RA_GYRO_CONFIG			0x1B
#define MPU6050_RA_ACCEL_CONFIG			0x1C
#define MPU6050_RA_GYRO_XOUT_H			0x43
#define MPU6050_ACCEL_RA_XOUT_H			0x3B
#define MPU6050_RA_USER_CTRL      		0x6A
#define MPU6050_RA_BANK_SEL         	0x6D
#define MPU6050_RA_MEM_START_ADDR   	0x6E
#define MPU6050_RA_MEM_R_W       	    0x6F
#define MPU6050_RA_XG_OFFS_TC       	0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_I2C_SLV0_ADDR    	0x25
#define MPU6050_RA_INT_ENABLE     		0x38
#define MPU6050_RA_SMPLRT_DIV       	0x19
#define MPU6050_RA_CONFIG          		0x1A
#define MPU6050_RA_MOT_THR          	0x1F
#define MPU6050_RA_ZRMOT_THR       		0x21
#define MPU6050_RA_MOT_DUR         		0x20
#define MPU6050_RA_ZRMOT_DUR        	0x22
#define MPU6050_RA_FIFO_COUNTH    	    0x72
#define MPU6050_RA_FIFO_R_W        		0x74
#define MPU6050_RA_INT_STATUS       	0x3A
#define MPU6050_RA_DMP_CFG_1       		0x70
#define MPU6050_RA_DMP_CFG_2        	0x71

#define MPU6050_PWR1_DEVICE_RESET_BIT   	7
#define MPU6050_PWR1_SLEEP_BIT          	6
#define MPU6050_USERCTRL_DMP_EN_BIT			7
#define MPU6050_USERCTRL_FIFO_EN_BIT        6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT 	5
#define MPU6050_USERCTRL_DMP_RESET_BIT		3
#define MPU6050_USERCTRL_FIFO_RESET_BIT     2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT  1
#define MPU6050_WHO_AM_I_BIT        		6
#define MPU6050_TC_OTP_BNK_VLD_BIT			0
#define MPU6050_PWR1_CLKSEL_BIT         	2
#define MPU6050_GCONFIG_FS_SEL_BIT      	4
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_CFG_EXT_SYNC_SET_BIT  	    5
#define MPU6050_CFG_DLPF_CFG_BIT   			2
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_DMP_INT_BIT       1

#define MPU6050_WHO_AM_I_LENGTH   			6
#define MPU6050_PWR1_CLKSEL_LENGTH    	    3
#define MPU6050_GCONFIG_FS_SEL_LENGTH   	2
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 	3
#define MPU6050_CFG_DLPF_CFG_LENGTH 		3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_GYRO_FS_250        	0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define  PI (float)(3.1415926535897932384626433832795)

#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]

#define min(a,b)            (((a) < (b)) ? (a) : (b))

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))


typedef enum {
	MPU6050_NOTDETECTED,
	MPU6050_DETECTED,
} MPU6050_Result;

typedef struct  {
	int16_t Accelerometer_X_RAW;
	int16_t Accelerometer_Y_RAW;
	int16_t Accelerometer_Z_RAW;

	int16_t Gyroscope_X_RAW;
	int16_t Gyroscope_Y_RAW;
	int16_t Gyroscope_Z_RAW;

	float Offset_Gyro_X;
	float Offset_Gyro_Y;
	float Offset_Gyro_Z;

	float Gyroscope_X_Cal;
	float Gyroscope_Y_Cal;
	float Gyroscope_Z_Cal;

	float Angle_Accel_Pitch;
	float Angle_Accel_Roll;

	float Angle_Gyro_Pitch;
	float Angle_Gyro_Roll;
	float Angle_Gyro_Yaw;

	float AngleSpeed_Gyro_X;
	float AngleSpeed_Gyro_Y;
	float AngleSpeed_Gyro_Z;

	float pitch;
	float roll;
	float yaw;

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

void MPU6050_CalculateFromRAWData(MPU6050str* d,float timedelta);
void GetGyroOffset(I2C_HandleTypeDef* I2Cx, MPU6050str* d, int32_t Loops);

#endif /* MPU6050_H */
