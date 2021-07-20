/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "tim.h"
#include "sensor.h"
#include "motor_controller.h"
#include "Alarm.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern void Report_to_PC(void);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern void Service_Input_Command(uint8_t* RxBuffer);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint8_t ethernet_Tx_Bytes[512];
extern uint8_t ethernet_Rx_Bytes[512];
extern __IO uint16_t ADC1_Value[1];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOA
#define Test_con603_pin3_Pin GPIO_PIN_5
#define Test_con603_pin3_GPIO_Port GPIOA
#define TILT_DRV_DIR0_Pin GPIO_PIN_0
#define TILT_DRV_DIR0_GPIO_Port GPIOB
#define PAN_ENC_A_Pin GPIO_PIN_12
#define PAN_ENC_A_GPIO_Port GPIOB
#define PAN_DRV_DIR0_Pin GPIO_PIN_13
#define PAN_DRV_DIR0_GPIO_Port GPIOB
#define PAN_ENC_B_Pin GPIO_PIN_14
#define PAN_ENC_B_GPIO_Port GPIOB
#define TEST_Pin GPIO_PIN_15
#define TEST_GPIO_Port GPIOB
#define LMT_UP_M1_Pin GPIO_PIN_3
#define LMT_UP_M1_GPIO_Port GPIOG
#define LMT_DOWN_M1_Pin GPIO_PIN_9
#define LMT_DOWN_M1_GPIO_Port GPIOG
#define PAN_DRV_DIR1_Pin GPIO_PIN_5
#define PAN_DRV_DIR1_GPIO_Port GPIOB
#define TILT_DRV_DIR1_Pin GPIO_PIN_7
#define TILT_DRV_DIR1_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_8
#define LED_RED_GPIO_Port GPIOB
#define LED_GRN_Pin GPIO_PIN_9
#define LED_GRN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
