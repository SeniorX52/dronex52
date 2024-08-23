/*
 * motor.h
 *
 *  Created on: Aug 10, 2024
 *      Author: biffb
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_tim.h"
// Initialize the motor
#define MOTORS_NUMBER 4
#define MOTOR1	TIM_CHANNEL_4
#define MOTOR2	TIM_CHANNEL_1
#define MOTOR3	TIM_CHANNEL_2
#define MOTOR4	TIM_CHANNEL_3
void Motor_Init(void);
// Set the speed of the motor (PWM duty cycle)
void Motor_SetSpeedM1();
void Motor_SetSpeedM2();
void Motor_SetSpeedM3();
void Motor_SetSpeedM4();
void Motor_setSpeedAll(int speed);
int* Motor_getMotorsSpeed(void);
void Motor_setRoll(int output_roll);
void updateSpeeds(void);
void Motor_setPitch(int output_pitch);
void Motor_setYaw(int output_yaw);
#endif /* INC_MOTOR_H_ */
