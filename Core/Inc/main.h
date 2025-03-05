/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void Timer14_Elapsed(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ON_OFF_BUTTON_Pin GPIO_PIN_13
#define ON_OFF_BUTTON_GPIO_Port GPIOC
#define ON_OFF_BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define DEBUG_3_Pin GPIO_PIN_14
#define DEBUG_3_GPIO_Port GPIOC
#define NOZZLE_INPUT_Pin GPIO_PIN_15
#define NOZZLE_INPUT_GPIO_Port GPIOC
#define NOZZLE_INPUT_EXTI_IRQn EXTI4_15_IRQn
#define DEBUG_4_Pin GPIO_PIN_0
#define DEBUG_4_GPIO_Port GPIOF
#define DEBUG_5_Pin GPIO_PIN_1
#define DEBUG_5_GPIO_Port GPIOF
#define ENCODER_CW_Pin GPIO_PIN_0
#define ENCODER_CW_GPIO_Port GPIOA
#define ACCEL_INT_Pin GPIO_PIN_1
#define ACCEL_INT_GPIO_Port GPIOA
#define ENCODER_CCW_Pin GPIO_PIN_2
#define ENCODER_CCW_GPIO_Port GPIOA
#define ENCODER_CCW_EXTI_IRQn EXTI2_3_IRQn
#define SPI1_NSS_Pin GPIO_PIN_3
#define SPI1_NSS_GPIO_Port GPIOA
#define AC_50_60HZ_Pin GPIO_PIN_4
#define AC_50_60HZ_GPIO_Port GPIOA
#define AC_50_60HZ_EXTI_IRQn EXTI4_15_IRQn
#define DEBUG_8_Pin GPIO_PIN_5
#define DEBUG_8_GPIO_Port GPIOA
#define HALL_SENSOR_Pin GPIO_PIN_6
#define HALL_SENSOR_GPIO_Port GPIOA
#define DEBUG_9_Pin GPIO_PIN_7
#define DEBUG_9_GPIO_Port GPIOA
#define INTERLOCK_Pin GPIO_PIN_0
#define INTERLOCK_GPIO_Port GPIOB
#define INTERLOCK_EXTI_IRQn EXTI0_1_IRQn
#define THERMOSTAT_Pin GPIO_PIN_1
#define THERMOSTAT_GPIO_Port GPIOB
#define THERMOSTAT_EXTI_IRQn EXTI0_1_IRQn
#define DEBUG_10_Pin GPIO_PIN_2
#define DEBUG_10_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_10
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_11
#define EEPROM_SDA_GPIO_Port GPIOB
#define MOTOR_RELAY_Pin GPIO_PIN_12
#define MOTOR_RELAY_GPIO_Port GPIOB
#define SPI2_CLK_DNC_Pin GPIO_PIN_13
#define SPI2_CLK_DNC_GPIO_Port GPIOB
#define DISPLAY_SCLK_Pin GPIO_PIN_8
#define DISPLAY_SCLK_GPIO_Port GPIOA
#define DEBUG_13_Pin GPIO_PIN_9
#define DEBUG_13_GPIO_Port GPIOA
#define MOTOR_CW_Pin GPIO_PIN_6
#define MOTOR_CW_GPIO_Port GPIOC
#define DEBUG_14_Pin GPIO_PIN_7
#define DEBUG_14_GPIO_Port GPIOC
#define MOTOR_CCW_Pin GPIO_PIN_10
#define MOTOR_CCW_GPIO_Port GPIOA
#define TRIAC_CONTROL_Pin GPIO_PIN_12
#define TRIAC_CONTROL_GPIO_Port GPIOA
#define DIAL_ARROW_LED_Pin GPIO_PIN_15
#define DIAL_ARROW_LED_GPIO_Port GPIOA
#define DEBUG_16_Pin GPIO_PIN_0
#define DEBUG_16_GPIO_Port GPIOD
#define NOZZLE_LED_Pin GPIO_PIN_1
#define NOZZLE_LED_GPIO_Port GPIOD
#define DEBUG_17_Pin GPIO_PIN_2
#define DEBUG_17_GPIO_Port GPIOD
#define DISPENSING_LED_Pin GPIO_PIN_3
#define DISPENSING_LED_GPIO_Port GPIOD
#define PROCESSING_LED_Pin GPIO_PIN_4
#define PROCESSING_LED_GPIO_Port GPIOB
#define VOLTAGE_SUPPLY_Pin GPIO_PIN_8
#define VOLTAGE_SUPPLY_GPIO_Port GPIOB
#define BUTTON_TEXT_LED_Pin GPIO_PIN_9
#define BUTTON_TEXT_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SPI2_MOSI_Pin GPIO_PIN_14
#define SPI2_MOSI_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_Pin_Number 5
#define SPI1_MOSI_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
