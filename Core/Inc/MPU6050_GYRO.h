#ifndef INC_MPU6050_GYRO_H_
#define INC_MPU6050_GYRO_H_

#include "stdint.h"
#include "main.h"
#define MPU6050_ADDR         0x68<<1 //have to shift to the left cause the first bit is read/write bit for I2C

// MPU6050 register addresses
#define PWR_MGMT_1           0x6B
#define SMPLRT_DIV           0x19
#define ACCEL_CONFIG         0x1C
#define GYRO_CONFIG          0x1B
#define ACCEL_XOUT_H         0x3B
#define ACCEL_XOUT_L         0x3C
#define ACCEL_YOUT_H         0x3D
#define ACCEL_YOUT_L         0x3E
#define ACCEL_ZOUT_H         0x3F
#define ACCEL_ZOUT_L         0x40
#define GYRO_XOUT_H          0x43
#define GYRO_XOUT_L          0x44
#define GYRO_YOUT_H          0x45
#define GYRO_YOUT_L          0x46
#define GYRO_ZOUT_H          0x47
#define GYRO_ZOUT_L          0x48

typedef struct {
	int16_t AccelX;
	int16_t AccelY;
	int16_t AccelZ;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
}MPU6050_Data;

extern I2C_HandleTypeDef hi2c1;

void MPU6050_INIT();

void MPU6050_GetData(MPU6050_Data* MPU6050Data);

#endif
