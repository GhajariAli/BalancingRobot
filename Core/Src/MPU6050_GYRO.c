#include "MPU6050_Gyro.h"

void MPU6050_INIT(){
	uint8_t Samplerate = 0x00; //this will be +1 in the MPU6050 (the chip is 8khz divided by this value +1
	uint8_t	PowerManagementOption= 0x01; // check datasheet this will clear all registers and set internal 8khz osc as clock
	uint8_t AccelConfig = 0x00; // sets range to 2g
	uint8_t GyroConfig = 0x00; // sets gyro range to +-500degree

	HAL_Delay(100);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &PowerManagementOption, 1, 100);
	HAL_Delay(1);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &Samplerate, 1, 100);
	HAL_Delay(1);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &AccelConfig, 1, 100);
	HAL_Delay(1);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &GyroConfig, 1, 100);
	HAL_Delay(1);
}

void MPU6050_GetData(MPU6050_Data* MPU6050Data){
	uint8_t AccelData[6] ;
	uint8_t GyroData[6] ;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, AccelData, 6, 100);
	MPU6050Data->AccelX= (AccelData[0]<<8) | AccelData[1];
	MPU6050Data->AccelY= (AccelData[2]<<8) | AccelData[3];
	MPU6050Data->AccelZ= (AccelData[4]<<8) | AccelData[5];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, GyroData, 6, 100);
	MPU6050Data->GyroX= (GyroData[0]<<8) | GyroData[1];
	MPU6050Data->GyroY= (GyroData[2]<<8) | GyroData[3];
	MPU6050Data->GyroZ= (GyroData[4]<<8) | GyroData[5];
}
