/*
 * PID.h
 *
 *  Created on: Aug 18, 2024
 *      Author: study
 */

#ifndef INC_PID_H_
#define INC_PID_H_
typedef struct{
	float kp,ki,kd;
	float integral;
	float last_error;
} PIDController;
void PID_ControlLoop(void);
void PID_Update(PIDController* pid, float setpoint, float measurement, float* output);
void PID_updateRoll(float roll);
void PID_updatePitch(float pitch);
void PID_updateYaw(float yaw);
void PID_SetKP(float kp);
void PID_SetKI(float ki);
void PID_SetKD(float kd);
void PID_Reset();
void PID_Reset_Controller(PIDController* pid);
void UpdateGyroStabilization(void);
void UpdateTime(void);
float GetDeltaTime(void);
void PID_init(void);
void PID_SetYawKP(float kp);

void PID_SetYawKI(float ki);
void PID_SetYawKD(float kd);
#endif /* INC_PID_H_ */
