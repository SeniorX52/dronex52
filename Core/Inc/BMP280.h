/*
 * BMP280.h
 *
 *  Created on: Aug 17, 2024
 *      Author: biffb
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_
#include "stm32f1xx_hal.h"
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData;
HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMP280_SetUltraHighRes(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ReadPressureAndCalculateAltitude(float *altitude);
HAL_StatusTypeDef BMP280_ReadCalibrationData(I2C_HandleTypeDef *hi2c, BMP280_CalibData *calibData);

#endif /* INC_BMP280_H_ */
