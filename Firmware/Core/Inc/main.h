/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>

#define CAN_ID_SERVO_ZERO         0x702  //
#define CAN_ID_SERVO_ROTATE       0x703  // D0: dir 0/1, D1: angle 0..270
#define CAN_ID_SOLENOID_ACTUATE   0x704  // D0: bitmask (A8..A10)
#define CAN_ID_SOLENOID_STATUS    0x705  // TX only: D0 = bitmask (LOW=connected)
#define CAN_ID_LED 				  0x706


#define SOLENOID_PIN_1 8
#define SOLENOID_PIN_2 9
#define SOLENOID_PIN_3 10

#define SOL_PIN_1_DETECT 15
#define SOL_PIN_2_DETECT 14
#define SOL_PIN_3_DETECT 13



extern const uint16_t solenoidPins[3];
extern const uint16_t solenoidDetectPins[3];


#define MOTOR_TIMER_HANDLE htim3
#define MOTOR_TIMER_CHANNEL TIM_CHANNEL_1

void setPWM(TIM_HandleTypeDef *timer_handle, uint32_t timer_channel, float duty);
void servoZero(void);
void servoRotate(float angle);
void pyroActuate(uint8_t state);
void pyroDetect(void);
void can_send_std(uint16_t id, const uint8_t *data, uint8_t len);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t it);
uint8_t getDataLength(FDCAN_RxHeaderTypeDef *hdr);


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
