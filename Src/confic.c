/*
 * confic.c
 *
 *  Created on: 30.12.2019
 *      Author: tomat
 */

#include "config.h"


//Funktionen für schnelleren Zugriff
void led2_t() {
	HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
}

void led3_t() {
	HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
}

void leds_reset() {
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, 1);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 1);
}

void leds_set() {
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, 0);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 0);
}

void blink(uint8_t count){
	leds_reset();
	for(uint8_t i = 0;i<count;i++){
		leds_set();
		HAL_Delay(250);
		leds_reset();
		HAL_Delay(250);
	}
}

/*/
void dump_values_on_uart() {
	uint8_t textbuff[10] = "";
	itoa(last_acc[0], textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " ", 2, 160);
	itoa(last_acc[1], textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " ", 2, 160);
	itoa(last_acc[2], textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " || ", 5, 160);
	itoa((int16_t) rot[0], textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " ", 2, 160);
	itoa((int16_t) rot[1], textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " ", 2, 160);
	itoa((int16_t) rot[2], textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " || ", 5, 160);
	itoa((int16_t) q0 * 10.0, textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " ", 2, 160);
	itoa((int16_t) q1 * 10.0, textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " ", 2, 160);
	itoa((int16_t) q2 * 10.0, textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, " ", 2, 160);
	itoa((int16_t) q3 * 10.0, textbuff, 10);
	HAL_UART_Transmit(&huart1, &textbuff, strlen(textbuff), 160);
	HAL_UART_Transmit(&huart1, "\n\r", 4, 160);
}
// */

void print_uart(uint8_t *text) {
#ifdef DEBUG
	//Länge ermitteln
	int i = 0;
	while(text[i] != '\0' && i < 150){
		i++;
	}
	HAL_UART_Transmit(&huart1, text, i+1, 160);
#endif
}

