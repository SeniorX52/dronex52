/*
 * controller.c
 *
 *  Created on: Aug 11, 2024
 *      Author: biffb
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "UART.h"
#include "motor.h"
#include "PID.h"
#include "main.h"
void Controller_executeCommand(char* data){
	char* token=strtok(data,"$");
	if (token != NULL && strcmp(token, "#MOTOR") == 0) {
        token = strtok(NULL, "$");
        if (token != NULL)
        {
            char* endptr;
            int speed = strtol(token, &endptr, 10);
            if (*endptr != '\0')
            {
                UART_sendString("Invalid speed value\n");
            }
            else
            {
                Motor_setSpeedAll(speed);
            }
        }
        else
        {
            UART_sendString("Speed value missing\n");
        }

        token = strtok(NULL, "$");
        if (token != NULL)
        {
            char* endptr;
            int roll = strtol(token, &endptr, 10);
            if (*endptr != '\0')
            {
                UART_sendString("Invalid roll value\n");
            }
            else
            {
                PID_updateRoll(roll);
            }
        }
        else
        {
            UART_sendString("Roll value missing\n");
        }

        token = strtok(NULL, "$");
        if (token != NULL)
        {
            char* endptr;
            int pitch = strtol(token, &endptr, 10);
            if (*endptr != '\0')
            {
                UART_sendString("Invalid pitch value\n");
            }
            else
            {
                PID_updatePitch(pitch);

            }
        }
        else
        {
            UART_sendString("Pitch value missing\n");
        }
        token = strtok(NULL, "$");
        if (token != NULL)
        {
            char* endptr;
            int yaw = strtol(token, &endptr, 10);
            if (*endptr != '\0')
            {
                UART_sendString("Invalid yaw value\n");
            }
            else
            {
            	PID_updateYaw(yaw);

            }
        }
        else
        {
            UART_sendString("yaw value missing\n");
        }
	}
	else{
	    if (token != NULL && strcmp(token, "#SET") == 0) {
	        token = strtok(NULL, "$");
	        if (token != NULL && strcmp(token, "PID") == 0) {

	            token = strtok(NULL, "$");
	            if (token != NULL) {
	                char* endptr;
	                float kp = strtof(token, &endptr);
	                if (*endptr != '\0') {
	                    UART_sendString("Invalid KP value\n");
	                    return;
	                }
	                PID_SetKP(kp);
	            } else {
	                UART_sendString("KP value missing\n");
	            }
	            // Process KI value
	            token = strtok(NULL, "$");
	            if (token != NULL) {
	                char* endptr;
	                float ki = strtof(token, &endptr);
	                if (*endptr != '\0') {
	                    UART_sendString("Invalid KI value\n");
	                }
	                PID_SetKI(ki);
	            } else {
	                UART_sendString("KI value missing\n");
	            }

	            // Process KD value
	            token = strtok(NULL, "$");
	            if (token != NULL) {
	                char* endptr;
	                float kd = strtof(token, &endptr);
	                if (*endptr != '\0') {
	                    UART_sendString("Invalid KD value\n");
	                    return;
	                }
	                PID_SetKD(kd);
	            } else {
	                UART_sendString("KD value missing\n");
	            }

	        }
	        else{
		        if (token != NULL && strcmp(token, "YAW") == 0) {

		            token = strtok(NULL, "$");
		            if (token != NULL) {
		                char* endptr;
		                float kp = strtof(token, &endptr);
		                if (*endptr != '\0') {
		                    UART_sendString("Invalid KP value\n");
		                    return;
		                }
		                PID_SetYawKP(kp);
		            } else {
		                UART_sendString("KP value missing\n");
		            }

		            // Process KI value
		            token = strtok(NULL, "$");
		            if (token != NULL) {
		                char* endptr;
		                float ki = strtof(token, &endptr);
		                if (*endptr != '\0') {
		                    UART_sendString("Invalid KI value\n");
		                    return;
		                }
		                PID_SetYawKI(ki);
		            } else {
		                UART_sendString("KI value missing\n");
		            }

		            // Process KD value
		            token = strtok(NULL, "$");
		            if (token != NULL) {
		                char* endptr;
		                float kd = strtof(token, &endptr);
		                if (*endptr != '\0') {
		                    UART_sendString("Invalid KD value\n");
		                    return;
		                }
		                PID_SetYawKD(kd);
		            } else {
		                UART_sendString("KD value missing\n");
		            }

		        }
	        }

	    }

	    else {
	    	 if (token != NULL && strcmp(token, "#RESET") == 0) {
	    		 token = strtok(NULL, "$");
	    			        if (token != NULL && strcmp(token, "PID") == 0) {
	    			        	PID_Reset();
	    			        }
	    			        else{
	    			            UART_sendString("Invalid command after #RESET\n");
	    			        }
	    	 }
	    }
	}

}
