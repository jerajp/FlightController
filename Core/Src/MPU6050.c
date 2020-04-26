// Functions to manage the MPU6050 sensor

#include "MPU6050.h"

#define MPU6050_I_AM					0x68 //value of register WHO_AM_I

//registers
#define MPU6050_ADDRESS					0xD0
#define MPU6050_WHO_AM_I_REG			0x75
#define MPU6050_PWR_MGMT_1_REG			0x6B
#define MPU6050_SMPLRT_DIV_REG			0x19
#define MPU6050_GYRO_CONFIG_REG			0x1B
#define MPU6050_ACCEL_CONFIG_REG		0x1C
#define MPU6050_GYRO_XOUT_H_REG			0x43
#define MPU6050_ACCEL_XOUT_H_REG		0x3B



MPU6050_Result MPU6050_check(I2C_HandleTypeDef* I2Cx)
{
	//I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t data;

	HAL_I2C_Mem_Read (I2Cx, MPU6050_ADDRESS,MPU6050_WHO_AM_I_REG,1, &data, 1, 1000);

	if(data==MPU6050_I_AM) return MPU6050_DETECTED;
	else return MPU6050_NOTDETECTED;

}

void MPU6050_init(I2C_HandleTypeDef* I2Cx)
{
	uint8_t data=0; //wake sensor, clk=8Mhz(internal)
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1_REG, 1,&data, 1, 1000);

	//Set sample rate
	data=7;//set to 1khz sample rate
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV_REG, 1,&data, 1, 1000);

	//Gyro config
	data=1<<3;//Set ± 500 °/s
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG_REG, 1,&data, 1, 1000);

	//Accel config
	data=2<<3;//Set +-8g
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG_REG, 1,&data, 1, 1000);
}

void MPU6050_accread(I2C_HandleTypeDef* I2Cx, MPU6050str* DataStruct)
{
	uint8_t data[6];
	HAL_I2C_Mem_Read (I2Cx, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H_REG, 1, data, 6, 1000);

	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data [1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data [3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data [5]);

}

void MPU6050_gyroread(I2C_HandleTypeDef* I2Cx, MPU6050str* DataStruct)
{
	uint8_t data[6];
	HAL_I2C_Mem_Read (I2Cx, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H_REG, 1, data, 6, 1000);

	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data [1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data [3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data [5]);
}

