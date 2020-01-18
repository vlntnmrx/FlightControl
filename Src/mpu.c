/*
 * mpu.c
 *
 *  Created on: 30.10.2019
 *      Author: Valentin Marx
 */
#include "main.h"
#include "mpu.h"
#include <math.h>
#include "MadgwickAHRS.h"
#include "quaternion.h"
#include "control.h"

uint16_t acc_scale = 1;
uint16_t gyro_scale = 1;

void mpu_init_std(I2C_HandleTypeDef *hi2c) {
	print_uart("MPU init");
	/*Scale Setting:
	 * 00 = +250dps, 01= +500 dps, 10 = +1000 dps, 11 = +2000 dps
	 * 0b000xx000
	 * */
	//Set scale gyro 500dps
	gyro_scale = 2000;
	uint8_t da = 0b11 << 3; //0b00010000
	HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 27, I2C_MEMADD_SIZE_8BIT, &da, 1,
			HAL_I2C_STATE_TIMEOUT);
	while (hi2c->State != HAL_I2C_STATE_READY)
		;
	/*Scale Setting:
	 * ±2g (00), ±4g (01), ±8g (10), ±16g (11)
	 * 0b000xx000
	 * */
	//set scale acc +-4g
	acc_scale = 4000;
	da = 0b01 << 3;
	HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 28, I2C_MEMADD_SIZE_8BIT, &da, 1,
			HAL_I2C_STATE_TIMEOUT);
	while (hi2c->State != HAL_I2C_STATE_READY)
		;
	//Start Sensor DONT!
	//HAL_I2C_Mem_Write(hi2c, MPU_ADDR, 107, I2C_MEMADD_SIZE_8BIT, 0x00, 1,
	//		HAL_I2C_STATE_TIMEOUT);
	//while (hi2c->State != HAL_I2C_STATE_READY);
}

void mpu_init_dma(I2C_HandleTypeDef *hi2c) {
	/*//Start Sensor
	 HAL_StatusTypeDef state = HAL_I2C_Mem_Write_DMA(hi2c, MPU_ADDR, 107,
	 I2C_MEMADD_SIZE_8BIT, 0x00, 1);
	 while (hi2c->State != HAL_I2C_STATE_READY)
	 ;
	 // */
	//Set scale gyro 500dps
	HAL_StatusTypeDef state = HAL_I2C_Mem_Write_DMA(hi2c, MPU_ADDR, 27,
	I2C_MEMADD_SIZE_8BIT, 0b00001000, 1);
	while (hi2c->State != HAL_I2C_STATE_READY)
		;
	//set scale acc +-4g
	state = HAL_I2C_Mem_Write_DMA(hi2c, MPU_ADDR, 28, I2C_MEMADD_SIZE_8BIT,
			0b00001000, 1);
	while (hi2c->State != HAL_I2C_STATE_READY)
		;
}

void mpu_init_spi(SPI_HandleTypeDef *hspi) {
	mpu_cs(SPI_SELECT);
	uint8_t sb[2] = { 107 | 0x00, 0 };
	HAL_SPI_Transmit(hspi, sb, 2, 160);
	HAL_Delay(5);
	sb[0] = 27 | 0x00;
	sb[1] = 0x08;
	mpu_cs(SPI_SELECT);
	HAL_SPI_Transmit(hspi, sb, 2, 160);

	sb[0] = 28 | 0x00;
	sb[1] = 0x08;
	mpu_cs(SPI_SELECT);
	HAL_SPI_Transmit(hspi, sb, 2, 160);
	mpu_cs(SPI_DESELECT);
}

void mpu_calibrate(I2C_HandleTypeDef *hi2c, int8_t *gyro_offset,
		float *rot_start) {
	//Offset des Gyro ermitteln
	//30 Gyrowerte aus allen achsen holen
	int16_t gygy[3] = { 0, 0, 0 };
	int16_t acac[3] = { 0, 0, 0 };
	uint8_t bubu[6];
	uint8_t i;
	for (i = 0; i < 30; i++) {
		HAL_I2C_Mem_Read(hi2c, MPU_ADDR, 67, I2C_MEMADD_SIZE_8BIT, bubu, 6,
				160);
		gygy[0] += ((int16_t) bubu[0] << 8) | bubu[1];
		gygy[1] += ((int16_t) bubu[2] << 8) | bubu[3];
		gygy[2] += ((int16_t) bubu[4] << 8) | bubu[5];
		HAL_Delay(10);
	}
	//Offset mitteln
	gyro_offset[0] = gygy[0] / i;
	gyro_offset[1] = gygy[1] / i;
	gyro_offset[2] = gygy[2] / i;

	//Aktuelle Position aus Acc bestimmen, mittelwert aus 3 Messungen
	for (i = 0; i < 3; i++) {
		HAL_I2C_Mem_Read(hi2c, MPU_ADDR, 59, I2C_MEMADD_SIZE_8BIT, bubu, 6,
				160);
		acac[0] += ((int16_t) bubu[0] << 8) | bubu[1];
		acac[1] += ((int16_t) bubu[2] << 8) | bubu[3];
		acac[2] += ((int16_t) bubu[4] << 8) | bubu[5];
		HAL_Delay(5);
	}
	acac[0] = acac[0] / i;
	acac[1] = acac[1] / i;
	acac[2] = acac[2] / i;
	float length = sqrtf( // Berechne die laenge des Vektors
			(acac[0] * acac[0]) + (acac[1] * acac[1]) + (acac[2] * acac[2]));
	float ax, ay, az;
	ax = acac[0] / length;
	ay = acac[1] / length;
	az = acac[2] / length;
	rot_start[0] = 360.0 * asin(ay) / (M_PI * 2.0);
	rot_start[1] = 360.0 * asin(-ax) / (M_PI * 2.0);
	rot_start[2] = 0; //360.0 * asin(az) / (M_PI * 2.0); //Z-Rot ist egal

	//Quaternionen Kalibrieren
	float alpha = acos(acac[2] / length);
	float sin_alpha = sin(alpha / 2.0f);
	q_ist.q0 = cos(alpha / 2.0f);
	//Normalen Vektor berechnen zu acc und z-achse (0,0,-1)
	float v_norm[3];
	v_norm[0] = -acac[1] / length;
	v_norm[1] = acac[0] / length;
	v_norm[2] = 0;

	q_ist.q1 = v_norm[0] * sin_alpha;
	q_ist.q2 = v_norm[1] * sin_alpha;
	q_ist.q3 = v_norm[2] * sin_alpha;

	q_pos.q0 = q_ist.q0;
	q_pos.q1 = q_ist.q1;
	q_pos.q2 = q_ist.q2;
	q_pos.q3 = q_ist.q3;

	// */

}

void mpu_get_acc(I2C_HandleTypeDef *hi2c, int16_t *buffer) {
	uint8_t red[6];
	//Read reg 59-64
	HAL_I2C_Mem_Read(hi2c, MPU_ADDR, 59, I2C_MEMADD_SIZE_8BIT, red, 6,
			HAL_I2C_STATE_TIMEOUT);
	while (hi2c->State == HAL_I2C_STATE_BUSY)
		;

	buffer[0] = ((int16_t) red[0] << 8) | red[1];
	buffer[1] = ((int16_t) red[2] << 8) | red[3];
	buffer[2] = ((int16_t) red[4] << 8) | red[5];
}
void mpu_get_gyro(I2C_HandleTypeDef *hi2c, int16_t *buffer) {
	uint8_t red[6];
	//Read reg 67-72
	HAL_I2C_Mem_Read(hi2c, MPU_ADDR, 67, I2C_MEMADD_SIZE_8BIT, red, 6,
			HAL_I2C_STATE_TIMEOUT);
	while (hi2c->State == HAL_I2C_STATE_BUSY)
		;

	buffer[0] = ((int16_t) red[0] << 8) | red[1];
	buffer[1] = ((int16_t) red[2] << 8) | red[3];
	buffer[2] = ((int16_t) red[4] << 8) | red[5];
}
int16_t mpu_get_temp(I2C_HandleTypeDef *hi2c) {
	uint8_t red[2];
	HAL_I2C_Mem_Read(hi2c, MPU_ADDR, 65, I2C_MEMADD_SIZE_8BIT, red, 2,
			HAL_I2C_STATE_TIMEOUT);
	return (red[0] << 8) | red[1];
}

void mpu_get_acc_dma(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_Mem_Read_DMA(hi2c, MPU_ADDR, 59, I2C_MEMADD_SIZE_8BIT, buff_acc, 6);
}

void mpu_get_gyro_dma(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_Mem_Read_DMA(hi2c, MPU_ADDR, 67, I2C_MEMADD_SIZE_8BIT, buff_gyro,
			6);
}

void mpu_get_all_dma(I2C_HandleTypeDef *hi2c) {
	//mpu_cs(SPI_SELECT);
	/*
	 uint8_t ap = 59 | 0x80;
	 HAL_SPI_Transmit(hspi, &ap, 1, 160);
	 HAL_SPI_Receive_DMA(hspi, gruff, 2);
	 //mpu_cs(SPI_DESELECT); // */
	HAL_I2C_Mem_Read_DMA(hi2c, MPU_ADDR, 59, I2C_MEMADD_SIZE_8BIT, gruff, 14);
}

void mpu_udate_data(int16_t *last_acc, int16_t *last_gyro) {
	last_acc[0] = ((int16_t) buff_acc[0] << 8) | buff_acc[1];
	last_acc[0] = (int16_t) ((float) last_acc[0] / ACC_SCALE_4g);
	last_acc[1] = ((int16_t) buff_acc[2] << 8) | buff_acc[3];
	last_acc[1] = (int16_t) ((float) last_acc[1] / ACC_SCALE_4g);
	last_acc[2] = ((int16_t) buff_acc[4] << 8) | buff_acc[5];
	last_acc[2] = (int16_t) ((float) last_acc[2] / ACC_SCALE_4g);

	last_gyro[0] = ((int16_t) buff_gyro[0] << 8) | buff_gyro[1];
	last_gyro[0] = (int16_t) ((float) last_gyro[0] / GYRO_SCALE_1000dps);
	last_gyro[1] = ((int16_t) buff_gyro[2] << 8) | buff_gyro[3];
	last_gyro[1] = (int16_t) ((float) last_gyro[1] / GYRO_SCALE_1000dps);
	last_gyro[2] = ((int16_t) buff_gyro[4] << 8) | buff_gyro[5];
	last_gyro[2] = (int16_t) ((float) last_gyro[2] / GYRO_SCALE_1000dps);
}

void mpu_cs(uint8_t hilo) {
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, hilo);
}
