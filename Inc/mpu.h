#ifndef __MPU_H
#define __MPU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MPU_ADDR (0x68*2)
#define ACC_SCALE_2g 16.384f
#define ACC_SCALE_4g 8.192f
#define ACC_SCALE_8g 4.096f
#define ACC_SCALE_16g 2.048f

#define GYRO_SCALE_250dps 131.0f
#define GYRO_SCALE_500dps 65.5f
#define GYRO_SCALE_1000dps 32.8f
#define GYRO_SCALE_2000dps 16.4f

#define OFFSET_1G 1010 // etwa 1000mG
#define SPI_SELECT 0
#define SPI_DESELECT 1

uint8_t buff_acc[6]; //Super Wichtig: Vorzeichenlos!!!
uint8_t buff_gyro[6];

uint8_t gruff[14];

void mpu_init_std(I2C_HandleTypeDef *hi2c);
void mpu_init_dma(I2C_HandleTypeDef *hi2c);
void mpu_init_spi(SPI_HandleTypeDef *hspi);

void mpu_calibrate(I2C_HandleTypeDef *hi2c, int8_t *gyro_offset, float *rot_start);

void mpu_get_acc(I2C_HandleTypeDef *hi2c, int16_t *buffer);
void mpu_get_gyro(I2C_HandleTypeDef *hi2c, int16_t *buffer);
int16_t mpu_get_temp(I2C_HandleTypeDef *hi2c);


void mpu_init_dma(I2C_HandleTypeDef *hi2c);
void mpu_get_acc_dma(I2C_HandleTypeDef *hi2c);
void mpu_get_gyro_dma(I2C_HandleTypeDef *hi2c);
void mpu_get_all_dma(I2C_HandleTypeDef *hi2c);


void mpu_udate_data(int16_t* last_acc, int16_t* last_gyro);

void mpu_cs(uint8_t hilo);

#ifdef __cplusplus
}
#endif

#endif
