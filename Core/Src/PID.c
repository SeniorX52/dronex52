/*
 * PID.c
 *
 *  Created on: Aug 18, 2024
 *      Author: study
 */


#include "PID.h"
#include <math.h>
#include "MPU6050.h"
#include "motor.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "BMP280.h"

#include <string.h>
#define KP  1.2
#define KI  0.001
#define KD  0.02
#define YAW_KP  0.6
#define YAW_KI  0.001
#define YAW_KD  0.02
#define DEADBAND 	0
#define GYRO_SENSITIVITY 16.4f
#define LOOP_PERIOD_MS 10

PIDController pitch_pid={KP,KI,KD,0,0};
PIDController roll_pid={KP,KI,KD,0,0};
PIDController yaw_pid={YAW_KP,YAW_KI,YAW_KD,0,0};
float desired_pitch = 0.0f;
float desired_roll = 0.0f;
float desired_yaw=0.0f;
float motor_pitch_output = 0.0f;
float motor_roll_output = 0.0f;
float motor_yaw_output = 0.0f;
float rollRate=0.0f;
float pitchRate=0.0f;
float yawRate=0.0f;
float desired_roll_rate=0.0f;
float desired_pitch_rate=0.0f;
float desired_yaw_rate=0.0f;
volatile uint32_t previousTime = 0;
volatile uint32_t currentTime = 0;

void UpdateTime() {
    previousTime = currentTime;
    currentTime = HAL_GetTick();
}

float GetDeltaTime() {
    return (currentTime - previousTime) / 1000.0f;
}
void PID_init(){
	previousTime = HAL_GetTick();
	MPU6050_Calibrate();
}
void PID_Update(PIDController* pid, float setpoint, float measurement, float* output){
	float error= setpoint-measurement;
	if (fabs(error) < DEADBAND) {
	        error = 0.0;
	}
	float prop=pid->kp*error;
	pid->integral+=pid->ki * error;;
	if (pid->integral > 1000.0f) pid->integral = 1000.0f;
	if (pid->integral < -1000.0f) pid->integral = -1000.0f;
	float derivative=pid->kd * (error - pid->last_error);
	*output=prop+pid->integral+derivative;
	pid->last_error=error;
}


void UpdateGyroStabilization() {
    int16_t gyroData[3] = {0, 0, 0};
    if (MPU6050_ReadGyro(gyroData) == HAL_OK) {
        rollRate = 	-gyroData[0] / GYRO_SENSITIVITY;
        pitchRate = -gyroData[1] / GYRO_SENSITIVITY;
        yawRate = 	gyroData[2] / GYRO_SENSITIVITY;
        PID_Update(&roll_pid, desired_roll_rate, rollRate, &motor_roll_output);
        PID_Update(&pitch_pid, desired_pitch_rate, pitchRate, &motor_pitch_output);
        PID_Update(&yaw_pid, desired_yaw_rate, yawRate, &motor_yaw_output);
    }
}

char uartBuffer[500];
int uartBufferLength = 0;

void CollectDataForUART(void) {
    if (uartBufferLength > 0) {

        uartBufferLength = 0;
    }
}

void PID_ControlLoop(void) {
    float realPitch = 0;
    float realRoll = 0;
    UpdateTime();
    float dt = GetDeltaTime();
    UpdateGyroStabilization();
    MPU6050_CalculatePitchRoll(&realPitch, &realRoll);
    float estimatedPitch = (0.98 * (realPitch + pitchRate * dt)) + (0.02 * realPitch);
    float estimatedRoll = (0.98 * (realRoll + rollRate * dt)) + (0.02 * realRoll);
    PID_Update(&pitch_pid, desired_pitch, estimatedPitch, &motor_pitch_output);
    PID_Update(&roll_pid, desired_roll, estimatedRoll, &motor_roll_output);
    PID_Update(&yaw_pid, desired_yaw, yawRate, &motor_yaw_output);
    Motor_setRoll((int)motor_roll_output);
    Motor_setPitch((int)motor_pitch_output);
    Motor_setYaw((int)motor_yaw_output);
    float altitude;
    ReadPressureAndCalculateAltitude(&altitude);
    uartBufferLength += sprintf(uartBuffer + uartBufferLength, "{");
    uartBufferLength += sprintf(uartBuffer + uartBufferLength, "\"CurrentPitch\":%d, \"CurrentRoll\":%d, ", (int)realPitch, (int)realRoll);
    uartBufferLength += sprintf(uartBuffer + uartBufferLength, "\"OutputPitch\":%d, \"OutputRoll\":%d, ", (int)motor_pitch_output, (int)motor_roll_output);
    uartBufferLength += sprintf(uartBuffer + uartBufferLength, "\"RollRate\":%d, \"PitchRate\":%d, ", (int)rollRate, (int)pitchRate);
    uartBufferLength += sprintf(uartBuffer + uartBufferLength, "\"YawRate\":%d, \"YawOutput\":%d,", (int)yawRate, (int)motor_yaw_output);
    uartBufferLength += sprintf(uartBuffer + uartBufferLength, "\"Altitude\":%d", (int)altitude);
    uartBufferLength += sprintf(uartBuffer + uartBufferLength, "}\n");
    UART_sendString(uartBuffer);
    memset(uartBuffer, 0, sizeof(uartBuffer));
    uartBufferLength=0;
}
void PID_updatePitch(float pitch){
	if(pitch!=desired_pitch){
			desired_pitch=pitch;
			pitch_pid.last_error=0;
			pitch_pid.integral=0;
		}
}
void PID_updateRoll(float roll){

	if(roll!=desired_roll){
		desired_roll=roll;
		roll_pid.last_error=0;
		roll_pid.integral=0;
	}
}
void PID_updateYaw(float yaw){
	if(yaw!=desired_yaw){
		desired_yaw=yaw;
		yaw_pid.last_error=0;
		yaw_pid.integral=0;
	}
}
void PID_Reset(){
	PID_Reset_Controller(&pitch_pid);
	PID_Reset_Controller(&roll_pid);
	PID_Reset_Controller(&yaw_pid);
}
void PID_Reset_Controller(PIDController* pid) {
    pid->kp = 0.0;
    pid->ki = 0.0;
    pid->kd = 0.0;
    pid->integral = 0.0;
    pid->last_error = 0.0;
}
void PID_SetKP(float kp) {
    char buffer[50];
    pitch_pid.kp = kp;
    roll_pid.kp = kp;
    yaw_pid.kp=kp;
    int kp_int = (int)(kp * 10000);
    snprintf(buffer, sizeof(buffer), "KP value=%d.%04d\n", kp_int / 10000, kp_int % 10000);
    UART_sendString(buffer);
}

void PID_SetKI(float ki) {
    char buffer[50];
    pitch_pid.ki = ki;
    roll_pid.ki = ki;
    yaw_pid.ki=ki;
    int ki_int = (int)(ki * 10000);
    snprintf(buffer, sizeof(buffer), "KI value=%d.%04d\n", ki_int / 10000, ki_int % 10000);
    UART_sendString(buffer);
}

void PID_SetKD(float kd) {
    char buffer[50];
    pitch_pid.kd = kd;
    roll_pid.kd = kd;
    yaw_pid.kd=kd;
    int kd_int = (int)(kd * 10000);
    snprintf(buffer, sizeof(buffer), "KD value=%d.%04d\n", kd_int / 10000, kd_int % 10000);
    UART_sendString(buffer);
}

void PID_SetYawKP(float kp) {
    char buffer[50];
    yaw_pid.kp = kp;
    int kp_int = (int)(kp * 10000);
    snprintf(buffer, sizeof(buffer), "Yaw KP value=%d.%04d\n", kp_int / 10000, kp_int % 10000);
    UART_sendString(buffer);
}


void PID_SetYawKI(float ki) {
    char buffer[50];
    yaw_pid.ki = ki;
    int ki_int = (int)(ki * 10000);
    snprintf(buffer, sizeof(buffer), "Yaw KI value=%d.%04d\n", ki_int / 10000, ki_int % 10000);
    UART_sendString(buffer);
}

void PID_SetYawKD(float kd) {
    char buffer[50];
    yaw_pid.kd = kd;
    int kd_int = (int)(kd * 10000);
    snprintf(buffer, sizeof(buffer), "Yaw KD value=%d.%04d\n", kd_int / 10000, kd_int % 10000);
    UART_sendString(buffer);
}







