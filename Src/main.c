/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu.h"
#ifndef STM32F4
#define STM32F4
#endif
#include "serial.h"
#include "config.h"
#include "WiMOD_HCI_Layer.h"
#include <string.h>
#include "quaternion.h"
#include "control.h"
#include "MadgwickAHRS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

float rot[3] = { 0, 0, 0 };
float vel[3] = { 0, 0, 0 };
float trans[3] = { 0, 0, 0 };

extern Quaternion q_soll;

extern uint8_t buff_acc[6];
extern uint8_t buff_gyro[6];
extern uint8_t gruff[14]; //Great buffer

uint8_t watch_check = 0;
extern uint16_t servo[5];
uint8_t curr_servo;

int8_t gyro_offset[3];
uint8_t flag_mpu;
extern float looptime;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/*
	 if (htim->Instance == TIM10) {
	 } // */
	if (htim->Instance == TIM11) {
		//HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
		HAL_GPIO_WritePin(SERVO_1_GPIO_Port, SERVO_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SERVO_2_GPIO_Port, SERVO_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SERVO_3_GPIO_Port, SERVO_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SERVO_4_GPIO_Port, SERVO_4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SERVO_5_GPIO_Port, SERVO_5_Pin, GPIO_PIN_RESET);
		if (flag_mpu) {
			//HAL_TIM_Base_Stop_IT(htim);
			if (watch_check) {
				led3_t();
				HAL_DMA_Abort(&hdma_i2c1_tx);
				HAL_DMA_Abort(&hdma_i2c1_rx);

				hi2c1.ErrorCode = 0;
				flag_mpu = 0;
				watch_check = 0;
			} else {
				htim->Instance->ARR = 5000; //5ms timeout
				mpu_get_all_dma(&hi2c1);
				watch_check = 1;
			}
			return;
		}
		switch (curr_servo) {
		case 0:
			HAL_GPIO_WritePin(SERVO_1_GPIO_Port, SERVO_1_Pin, GPIO_PIN_SET);
			htim->Instance->ARR = servo[0];
			break;
		case 1:
			HAL_GPIO_WritePin(SERVO_2_GPIO_Port, SERVO_2_Pin, GPIO_PIN_SET);
			htim->Instance->ARR = servo[1];
			break;
		case 2:
			HAL_GPIO_WritePin(SERVO_3_GPIO_Port, SERVO_3_Pin, GPIO_PIN_SET);
			htim->Instance->ARR = servo[2];
			break;
		case 3:
			HAL_GPIO_WritePin(SERVO_4_GPIO_Port, SERVO_4_Pin, GPIO_PIN_SET);
			htim->Instance->ARR = servo[3];
			break;
		case 4:
			HAL_GPIO_WritePin(SERVO_5_GPIO_Port, SERVO_5_Pin, GPIO_PIN_SET);
			htim->Instance->ARR = servo[4];
			break;
		}
		curr_servo++;
		flag_mpu = 1;
		if (curr_servo > 4) {
			curr_servo = 0;
		}
	}
	while (0)
		;
}

/**
 * @brief  Master Rx Transfer completed callback.
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	//Zeit messen
	HAL_TIM_Base_Stop(&htim10);
	led2_t();
	looptime = htim10.Instance->CNT * 0.000001; //In s umrechnen
	htim10.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim10);
	//Daten aus Buffer gruff holen und zusammensetzen
	int16_t acc[3], gyro[3], tempt;
	float gx, gy, gz, temp;
	acc[0] = (((int16_t) gruff[0] << 8) | gruff[1]);
	acc[0] = acc[0] / ACC_SCALE_4g;
	acc[1] = (((int16_t) gruff[2] << 8) | gruff[3]);
	acc[1] = acc[1] / ACC_SCALE_4g;
	acc[2] = (((int16_t) gruff[4] << 8) | gruff[5]);
	acc[2] = acc[2] / ACC_SCALE_4g;
	tempt = ((int16_t) gruff[6] << 8) | gruff[7];
	temp = (tempt / 340.0f) + 36.53f;
	gyro[0] = (((int16_t) gruff[8] << 8) | gruff[9]) - gyro_offset[0];
	gx = ((float) gyro[0] / GYRO_SCALE_2000dps) * (M_PI / 180.0f);
	gyro[1] = (((int16_t) gruff[10] << 8) | gruff[11]) - gyro_offset[1];
	gy = ((float) gyro[1] / GYRO_SCALE_2000dps) * (M_PI / 180.0f);
	gyro[2] = (((int16_t) gruff[12] << 8) | gruff[13]) - gyro_offset[2];
	gz = ((float) gyro[2] / GYRO_SCALE_2000dps) * (M_PI / 180.0f);

	MadgwickAHRSupdateIMU(gx, gy, gz, acc[0], acc[1], acc[2]);

	pid_yaw();
	pid_pitch();
	pid_roll();
	watch_check = 0;
	flag_mpu = 0;
	htim11.Instance->ARR = 10; // Damit er gleich zurück zum Servo springt
	htim11.Instance->CNT = 0;
	//HAL_TIM_Base_Start_IT(&htim11);

}

void dump_quat_on_uart() {
	uint8_t buff[8];
	int16_t iq0, iq1, iq2, iq3;
	iq0 = (int16_t) (q_ist.q0 * 10000.0f);
	iq1 = (int16_t) (q_ist.q1 * 10000.0f);
	iq2 = (int16_t) (q_ist.q2 * 10000.0f);
	iq3 = (int16_t) (q_ist.q3 * 10000.0f);
	buff[0] = iq0 >> 8;
	buff[1] = iq0;
	buff[2] = iq1 >> 8;
	buff[3] = iq1;
	buff[4] = iq2 >> 8;
	buff[5] = iq2;
	buff[6] = iq3 >> 8;
	buff[7] = iq3;
	//itoa((int16_t) (q0 * 1000.f), buff, 10);
	//HAL_UART_Transmit(&huart1, buff, 8, 160);
	Quaternion qm;
	quat_copy(&q_ist, &qm);
	Quaternion qx = { 0, 1, 0, 0 }, qy = { 0, 0, 1, 0 }, qz = { 0, 0, 0, 1 };
	//Achsen auf aktuelle Position projezieren
	quat_qpq(&qm, &qx, &qx);
	quat_qpq(&qm, &qy, &qy);
	quat_qpq(&qm, &qz, &qz);

	char txt_buff[] = "   :   :   :   ";
	itoa((int) (qx.q0 * 10.0f), &txt_buff[0], 10);
	itoa((int) (qx.q1 * 10.0f), &txt_buff[4], 10);
	itoa((int) (qx.q2 * 10.0f), &txt_buff[8], 10);
	itoa((int) (qx.q3 * 10.0f), &txt_buff[12], 10);
	HAL_UART_Transmit(&huart1, txt_buff, 16, 160);
	HAL_UART_Transmit(&huart1, "\n\r", 2, 160);
}

TWiMOD_HCI_Message lora_cb(TWiMOD_HCI_Message msg) {
	TWiMOD_HCI_Message ret;
	return ret;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	print_uart("Startup...");
	HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
	//mpu_init_spi(&hspi1);
	mpu_init_std(&hi2c1);
	//mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_XYZ_COMPASS);

	pid_init();
	leds_reset(); //LEDs ausschalten
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET); // MPU Select auf HIGH
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET); // Flash Select auf HIGH

	HAL_Delay(10);
	blink(4);
	mpu_calibrate(&hi2c1, gyro_offset, rot);
	//
	//TWiMOD_HCI_CbRxMessage cbm = (&lora_cb);
	TWiMOD_HCI_Message iMsg = { 0, 0x01, 0x01, 0, 0 };
	WiMOD_HCI_Init(&lora_cb, &iMsg);
	HAL_Delay(5);
	// */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	curr_servo = 0;
	flag_mpu = 0;
	center_off(); //Servos mittig, motor aus
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	//HAL_TIM_Base_Start(&htim10); //MPU/IMU Timer
	HAL_TIM_Base_Start_IT(&htim11); //Servo Timer

	uint8_t state = 1, demo = 0;
	uint32_t tick = 0, tack = 0;

	Quaternion q180 = {0,0,0,1};

	while (1) {
		switch (state) {
		case 0:	//IDLE
			servo[0] = 1000; //Motor aus
			break;

		case 1: //DEMO
			switch (demo) {
			case 0:
				//Startup Motor
				if (servo[2] < 1500) {
					servo[2]++;
				} else {
					demo++;
					tick = HAL_GetTick();
				}
				break;

			case 1:
				//Strecke fliegen,
				if (HAL_GetTick() - tick >= 15000) {
					demo++;
				}
				break;
			case 2:
				//Ne Kurve machen um 180 Grad
				quat_multiply(&q_soll, &q180, &q_soll);
				//yaw(300);
				demo = 0;
				break;

			default:
				break;
			}
			break;
		default:
			break;
		}

		//Sekündlich Dinge tun
		if (HAL_GetTick() - tack >= 1000) {
			TWiMOD_HCI_Message wiMsg;
			wiMsg.MsgID = RADIOLINK_MSG_SEND_U_DATA_REQ;
			wiMsg.SapID = RADIOLINK_ID;
			wiMsg.Payload[0] = 0x17;
			wiMsg.Payload[1] = 0xBB;
			wiMsg.Payload[2] = 0xAA;

			wiMsg.Payload[3] = (uint8_t) (q_ist.q0 + 1.0) * 100.0;
			wiMsg.Payload[4] = (uint8_t) (q_ist.q1 + 1.0) * 100.0;
			wiMsg.Payload[5] = (uint8_t) (q_ist.q2 + 1.0) * 100.0;
			wiMsg.Payload[6] = (uint8_t) (q_ist.q3 + 1.0) * 100.0;
			wiMsg.Length = 7;
			WiMOD_HCI_SendMessage(&wiMsg);

			dump_quat_on_uart();

			//Checksumme über Sensorwerte bilden

			tack = HAL_GetTick();
		}

		if (!HAL_GPIO_ReadPin(BTN_0_GPIO_Port, BTN_0_Pin)) {
			state = 0;
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 3);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* TIM1_TRG_COM_TIM11_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */
//PRE: 42-1 ; Period: 16000
  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 167;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0xFFFF;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */
//PRE: 168-1 ; Period: 5000
  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 167;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1500;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_2_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FLASH_CS_Pin|MPU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SERVO_1_Pin|SERVO_2_Pin|SERVO_3_Pin|SERVO_4_Pin 
                          |SERVO_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN_1_Pin BTN_0_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin|BTN_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_2_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_CS_Pin MPU_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|MPU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO_1_Pin SERVO_2_Pin SERVO_3_Pin SERVO_4_Pin 
                           SERVO_5_Pin */
  GPIO_InitStruct.Pin = SERVO_1_Pin|SERVO_2_Pin|SERVO_3_Pin|SERVO_4_Pin 
                          |SERVO_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
