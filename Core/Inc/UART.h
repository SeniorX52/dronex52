/*
 * UART.h
 *
 *  Created on: Aug 11, 2024
 *      Author: biffb
 */

#ifndef INC_UART_H_
#define INC_UART_H_


void UART_Init(void);
void UART_receive_Data(char* data,uint16_t size);
void UART_send_Data(char* data);
void UART_sendString(char *str);
void UART_send_Char(char data);
void UART_sendNumber(int32_t TxNumber);
#endif /* INC_UART_H_ */
