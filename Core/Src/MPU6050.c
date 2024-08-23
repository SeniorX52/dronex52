/*
 * MPU6050.c
 *
 *  Created on: Aug 16, 2024
 *      Author: biffb
 */


#include "MPU6050.h"
#include <math.h>

static int16_t ax_offset = 0;
static int16_t ay_offset = 0;
static int16_t az_offset = 0;
static int16_t gx_offset = 0;
static int16_t gy_offset = 0;
static int16_t gz_offset = 0;

#define ACCEL_SCALE_FACTOR 8192.0f

MPU6050_HandleTypeDef* mpuHandle;

void ComputeAccelAngles(int16_t ax, int16_t ay, int16_t az, float* pitch, float* roll) {
    float ax_g = (float)ax / ACCEL_SCALE_FACTOR;
    float ay_g = (float)ay / ACCEL_SCALE_FACTOR;
    float az_g = (float)az / ACCEL_SCALE_FACTOR;

    *pitch = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * (180.0f / M_PI);
    *roll = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * (180.0f / M_PI);
}
void MPU6050_CalculatePitchRoll(float* pitch, float* roll){
	int16_t accelerometer[3]={0,0,0};
	MPU6050_ReadAccel(mpuHandle,accelerometer);
	ComputeAccelAngles(accelerometer[0], accelerometer[1], accelerometer[2],pitch, roll);
}

HAL_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *mpu) {
    HAL_StatusTypeDef ret;
    mpuHandle=mpu;
    ret = MPU6050_WriteRegister(mpu, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }
    HAL_Delay(100);
    ret = MPU6050_WriteRegister(mpu, MPU6050_CONFIG, 0x06);
        if (ret != HAL_OK) {
            return ret;
    }
    HAL_Delay(100);
    ret = MPU6050_WriteRegister(mpu, MPU6050_ACCEL_CONFIG, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }
    ret = MPU6050_WriteRegister(mpu, MPU6050_GYRO_CONFIG, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAccel(MPU6050_HandleTypeDef *mpu, int16_t* accelData) {
    uint8_t rawData[6];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
    HAL_Delay(100);
    if (ret == HAL_OK) {
        accelData[0] = ((int16_t)(rawData[0] << 8 | rawData[1])) - ax_offset;
        accelData[1] = ((int16_t)(rawData[2] << 8 | rawData[3])) - ay_offset;
        accelData[2] = ((int16_t)(rawData[4] << 8 | rawData[5])) - az_offset;
    }
    return ret;
}

HAL_StatusTypeDef MPU6050_ReadGyro(int16_t* gyroData) {
    uint8_t rawData[6];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(mpuHandle->hi2c, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
    HAL_Delay(100);
    if (ret == HAL_OK) {
        gyroData[0] = ((int16_t)(rawData[0] << 8 | rawData[1])) - gx_offset;
        gyroData[1] = ((int16_t)(rawData[2] << 8 | rawData[3])) - gy_offset;
        gyroData[2] = ((int16_t)(rawData[4] << 8 | rawData[5])) - gz_offset;
    }
    return ret;
}

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_HandleTypeDef *mpu, uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(mpu->hi2c, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_HandleTypeDef *mpu, uint8_t reg, uint8_t* data) {
    return HAL_I2C_Mem_Read(mpu->hi2c, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

void MPU6050_Calibrate() {
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t gyro[3];

    for (int i = 0; i < 100; i++) {
        MPU6050_ReadGyro(gyro);
        gx_sum += gyro[0];
        gy_sum += gyro[1];
        gz_sum += gyro[2];

        HAL_Delay(10);
    }
gx_offset = gx_sum / 100;
gy_offset = gy_sum / 100;
gz_offset = gz_sum / 100;

}
