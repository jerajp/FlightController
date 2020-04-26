#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#ifndef MPU6050_H_
#define MPU6050_H_

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


MPU6050_Result MPU6050_check(I2C_HandleTypeDef* I2Cx);
void MPU6050_init(I2C_HandleTypeDef* I2Cx);
void MPU6050_accread(I2C_HandleTypeDef* I2Cx, MPU6050str* DataStruct);
void MPU6050_gyroread(I2C_HandleTypeDef* I2Cx, MPU6050str* DataStruct);














#endif /* MPU6050_H */
