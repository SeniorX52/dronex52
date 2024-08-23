/*
 * BMP280.c
 *
 *  Created on: Aug 17, 2024
 *      Author: biffb
 */

#include "BMP280.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>
#define SEA_LEVEL_PRESSURE 101325.0
#define CTRL_MEAS_REG       0xF4
#define CONFIG_REG          0xF5
#define PRESSURE_DATA_REG   0xF7
#define BMP280_ADDRESS 0x76 << 1
I2C_HandleTypeDef* hi2c;
BMP280_CalibData *calibData;
HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *handlehi2c) {
	hi2c=handlehi2c;
    HAL_StatusTypeDef ret;
    uint8_t data[2];
    data[0] = 0xF4;
    data[1] = 0x27;
    ret = HAL_I2C_Master_Transmit(hi2c, BMP280_ADDRESS, data, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    data[0] = 0xF5;
    data[1] = 0xA0;
    ret = HAL_I2C_Master_Transmit(hi2c, BMP280_ADDRESS, data, 2, HAL_MAX_DELAY);
    BMP280_SetUltraHighRes(hi2c);
    return ret;
}


float CalculateAltitude(float pressure) {
    return (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.190284)) * 4433000.0;
}

HAL_StatusTypeDef BMP280_SetUltraHighRes(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];

    data[0] = CTRL_MEAS_REG;
    data[1] = 0xFF;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, BMP280_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    data[0] = CONFIG_REG;
    data[1] = 0xB0;
    ret = HAL_I2C_Master_Transmit(hi2c, BMP280_ADDRESS << 1, data, 2, HAL_MAX_DELAY);

    return ret;
}
HAL_StatusTypeDef BMP280_ReadCalibrationData(I2C_HandleTypeDef *hi2c, BMP280_CalibData *handlecalibData) {
	calibData=handlecalibData;
    uint8_t calib[24];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(hi2c, BMP280_ADDRESS, 0x88, I2C_MEMADD_SIZE_8BIT, calib, 24, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    calibData->dig_T1 = (calib[1] << 8) | calib[0];
    calibData->dig_T2 = (calib[3] << 8) | calib[2];
    calibData->dig_T3 = (calib[5] << 8) | calib[4];
    calibData->dig_P1 = (calib[7] << 8) | calib[6];
    calibData->dig_P2 = (calib[9] << 8) | calib[8];
    calibData->dig_P3 = (calib[11] << 8) | calib[10];
    calibData->dig_P4 = (calib[13] << 8) | calib[12];
    calibData->dig_P5 = (calib[15] << 8) | calib[14];
    calibData->dig_P6 = (calib[17] << 8) | calib[16];
    calibData->dig_P7 = (calib[19] << 8) | calib[18];
    calibData->dig_P8 = (calib[21] << 8) | calib[20];
    calibData->dig_P9 = (calib[23] << 8) | calib[22];

    return HAL_OK;
}

HAL_StatusTypeDef ReadPressureAndCalculateAltitude(float *altitude) {
    uint8_t rawData[6];
    HAL_StatusTypeDef ret;
    int32_t rawPressure, rawTemp;
    int32_t t_fine;
    ret = HAL_I2C_Mem_Read(hi2c, BMP280_ADDRESS, PRESSURE_DATA_REG, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    rawPressure = (int32_t)((rawData[0] << 12) | (rawData[1] << 4) | (rawData[2] >> 4));
    rawTemp = (int32_t)((rawData[3] << 12) | (rawData[4] << 4) | (rawData[5] >> 4));
    int32_t var1 = ((((rawTemp >> 3) - ((int32_t)calibData->dig_T1 << 1))) * ((int32_t)calibData->dig_T2)) >> 11;
    int32_t var2 = (((((rawTemp >> 4) - ((int32_t)calibData->dig_T1)) * ((rawTemp >> 4) - ((int32_t)calibData->dig_T1))) >> 12) * ((int32_t)calibData->dig_T3)) >> 14;
    t_fine = var1 + var2;
    int64_t var1p = ((int64_t)t_fine) - 128000;
    int64_t var2p = var1p * var1p * (int64_t)calibData->dig_P6;
    var2p = var2p + ((var1p * (int64_t)calibData->dig_P5) << 17);
    var2p = var2p + (((int64_t)calibData->dig_P4) << 35);
    var1p = ((var1p * var1p * (int64_t)calibData->dig_P3) >> 8) + ((var1p * (int64_t)calibData->dig_P2) << 12);
    var1p = (((((int64_t)1) << 47) + var1p)) * ((int64_t)calibData->dig_P1) >> 33;
    if (var1p == 0) return HAL_ERROR;
    int64_t p = 1048576 - rawPressure;
    p = (((p << 31) - var2p) * 3125) / var1p;
    var1p = (((int64_t)calibData->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2p = (((int64_t)calibData->dig_P8) * p) >> 19;
    int32_t pressure = (int32_t)(((p + var1p + var2p) >> 8) + (((int64_t)calibData->dig_P7) << 4));
    float pressure_Pa = pressure / 256.0;
    *altitude = CalculateAltitude(pressure_Pa);
    return HAL_OK;
}


