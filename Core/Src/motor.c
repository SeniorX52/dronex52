/*
 * motor.c
 *
 *  Created on: Aug 10, 2024
 *      Author: biffb
 */



#include "motor.h"



#define PWM_FREQUENCY 1000
#define CLOCK_FREQUENCY 8000000
#include "core_cm3.h"
#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
void enable_global_interrupts(void) {
    __enable_irq();
}

static TIM_HandleTypeDef htim1;
static int  PWM_PERIOD;
static int MOTORS_SPEED[MOTORS_NUMBER]={0,0,0,0};
static int throttle_base=0;
static int pitch=0;
static int roll=0;
static int yaw=0;

void Motor_Init(void)
{
	PWM_PERIOD=CLOCK_FREQUENCY/PWM_FREQUENCY - 1 ;
	enable_global_interrupts();
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&htim1);

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
    HAL_TIM_PWM_Init(&htim1);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

}
void Motor_SetSpeedM1()
{
	int speed=throttle_base+pitch+roll+yaw;
    if (speed > PWM_PERIOD)
    {
    	speed = PWM_PERIOD;
    }
    if(speed<0){
    	    	speed=0;
    	    }
    MOTORS_SPEED[0]=speed;

    __HAL_TIM_SET_COMPARE(&htim1, MOTOR1, speed);
}
void Motor_SetSpeedM2(){

	int speed=throttle_base-pitch+roll-yaw;
	    if (speed > PWM_PERIOD)
	    {
	    	speed = PWM_PERIOD;
	    }
	    if(speed<0){
	    	    	speed=0;
	    	    }
	    MOTORS_SPEED[1]=speed;

	    __HAL_TIM_SET_COMPARE(&htim1, MOTOR2, speed);
}
void Motor_SetSpeedM3(){

	int speed=throttle_base-pitch-roll+yaw;
	    if (speed > PWM_PERIOD)
	    {
	    	speed = PWM_PERIOD;
	    }
	    if(speed<0){
	    	    	speed=0;
	    	    }
	    MOTORS_SPEED[2]=speed;

	    __HAL_TIM_SET_COMPARE(&htim1, MOTOR3, speed);
}
void Motor_SetSpeedM4(){
	int speed=throttle_base+pitch-roll-yaw;
	    if (speed > PWM_PERIOD)
	    {
	    	speed = PWM_PERIOD;
	    }
	    if(speed<0){
	    	    	speed=0;
	    	    }
	    MOTORS_SPEED[3]=speed;

	    __HAL_TIM_SET_COMPARE(&htim1, MOTOR4, speed);
}
void Motor_setSpeedAll(int speed){

	throttle_base=(int)((((double)speed)/100.0) * PWM_PERIOD);

	if (throttle_base > PWM_PERIOD)
		    {
		    	throttle_base = PWM_PERIOD;
		    }
	updateSpeeds();
}
void updateSpeeds(){
	Motor_SetSpeedM1();
	Motor_SetSpeedM2();
	Motor_SetSpeedM3();
	Motor_SetSpeedM4();
}
int* Motor_getMotorsSpeed(void) {
    return MOTORS_SPEED;

}
void Motor_setRoll(int output_roll){
	roll=5.56*output_roll;
	updateSpeeds();
}
void Motor_setPitch(int output_pitch){
	pitch=5.56*output_pitch;
	updateSpeeds();
}
void Motor_setYaw(int output_yaw){
	yaw=5.56*output_yaw;
	updateSpeeds();
}





