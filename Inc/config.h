#ifndef __CONFIG_H
#define __CONFIG_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define LOOPTIME 0.001

extern UART_HandleTypeDef huart1;


//Funktionen für schnelleren Zugriff
void led2_t();

void led3_t();

void leds_reset();

void leds_set();

void blink(uint8_t count);

/*/
void dump_values_on_uart();
// */

void print_uart(uint8_t *text);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
