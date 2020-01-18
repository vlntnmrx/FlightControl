#ifndef __SERIAL_H
#define __SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

void serial_print(UART_HandleTypeDef * huart, uint8_t * text);
void serial_println(UART_HandleTypeDef * huart, uint8_t * text);
void serial_print_num(UART_HandleTypeDef * huart, int16_t number);

void serial_write(UART_HandleTypeDef * huart, uint8_t * buff);

void serial_read(UART_HandleTypeDef * huart, uint8_t * buff, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif
