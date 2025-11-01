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

/* CAN commands */

#define CAN_ID_SERVO_ZERO         0x702  //
#define CAN_ID_SERVO_ROTATE       0x703  // D0: dir 0/1, D1: angle 0..270
#define CAN_ID_SOLENOID_ACTUATE   0x704  // D0: bitmask (A8..A10)
#define CAN_ID_SOLENOID_STATUS    0x705  // TX only: D0 = bitmask (LOW=connected)
#define CAN_ID_LED 				  0x706



/* Pyro Channels */

#define NUM_PYROS 3

extern GPIO_TypeDef* const pyroPort;
extern const uint16_t pyroPins[NUM_PYROS];

extern GPIO_TypeDef* const pyroDetectPort;
extern const uint16_t pyroDetectPins[NUM_PYROS];


/* motor stuff */
#define MOTOR_TIMER_HANDLE htim3
#define MOTOR_TIMER_CHANNEL TIM_CHANNEL_1


/* Motor functions */
void setPWM(TIM_HandleTypeDef *timer_handle, uint32_t timer_channel, float duty);
void servoZero(void);
void servoRotate(float angle);

/* Pyro channel functions */
void pyroActuate(uint8_t index, uint8_t state);
uint8_t pyroDetect(uint8_t index);


/* State machine functions  */
uint8_t systemIdle(void);
uint8_t systemReady(void);
void handleServo(void);
void handlePyro(int i);

//not bothered to declare everything, will do some other time


/* CAN functions */
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
