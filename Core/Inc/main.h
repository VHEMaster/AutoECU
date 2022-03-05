/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_NSS_OUTS1_Pin GPIO_PIN_3
#define SPI4_NSS_OUTS1_GPIO_Port GPIOE
#define SPI4_NSS_OUTS2_Pin GPIO_PIN_4
#define SPI4_NSS_OUTS2_GPIO_Port GPIOE
#define SPI4_NSS_O2_Pin GPIO_PIN_13
#define SPI4_NSS_O2_GPIO_Port GPIOC
#define BT_KEY_Pin GPIO_PIN_0
#define BT_KEY_GPIO_Port GPIOC
#define BT_LED_Pin GPIO_PIN_1
#define BT_LED_GPIO_Port GPIOC
#define BT_PWR_Pin GPIO_PIN_2
#define BT_PWR_GPIO_Port GPIOC
#define BT_NRST_Pin GPIO_PIN_3
#define BT_NRST_GPIO_Port GPIOC
#define TIM5_CH1_SENS_CSPS_Pin GPIO_PIN_0
#define TIM5_CH1_SENS_CSPS_GPIO_Port GPIOA
#define SPI4_NSS_KNOCK_Pin GPIO_PIN_1
#define SPI4_NSS_KNOCK_GPIO_Port GPIOA
#define TIM9_CH1_O2_HEATER_Pin GPIO_PIN_2
#define TIM9_CH1_O2_HEATER_GPIO_Port GPIOA
#define O2_NRST_Pin GPIO_PIN_3
#define O2_NRST_GPIO_Port GPIOA
#define KNOCK_INT_Pin GPIO_PIN_4
#define KNOCK_INT_GPIO_Port GPIOA
#define KNOCK_INT_EXTI_IRQn EXTI4_IRQn
#define TIM13_CH1_INJ_4_Pin GPIO_PIN_6
#define TIM13_CH1_INJ_4_GPIO_Port GPIOA
#define TIM14_CH1_INJ_3_Pin GPIO_PIN_7
#define TIM14_CH1_INJ_3_GPIO_Port GPIOA
#define INJ_CH2_Pin GPIO_PIN_4
#define INJ_CH2_GPIO_Port GPIOC
#define INJ_CH1_Pin GPIO_PIN_5
#define INJ_CH1_GPIO_Port GPIOC
#define SPI4_NSS_INJ_Pin GPIO_PIN_0
#define SPI4_NSS_INJ_GPIO_Port GPIOB
#define STEP_ERR_Pin GPIO_PIN_1
#define STEP_ERR_GPIO_Port GPIOB
#define STEP_I0_Pin GPIO_PIN_11
#define STEP_I0_GPIO_Port GPIOE
#define STEP_I1_Pin GPIO_PIN_12
#define STEP_I1_GPIO_Port GPIOE
#define STEP_PH1_Pin GPIO_PIN_13
#define STEP_PH1_GPIO_Port GPIOE
#define STEP_PH2_Pin GPIO_PIN_14
#define STEP_PH2_GPIO_Port GPIOE
#define LOGIC_OE_Pin GPIO_PIN_15
#define LOGIC_OE_GPIO_Port GPIOE
#define IGN_NALLOW_Pin GPIO_PIN_10
#define IGN_NALLOW_GPIO_Port GPIOB
#define IGN_4_Pin GPIO_PIN_11
#define IGN_4_GPIO_Port GPIOB
#define SPI2_NSS_FLASH_Pin GPIO_PIN_12
#define SPI2_NSS_FLASH_GPIO_Port GPIOB
#define SPI2_WP_Pin GPIO_PIN_8
#define SPI2_WP_GPIO_Port GPIOD
#define IGN_3_Pin GPIO_PIN_9
#define IGN_3_GPIO_Port GPIOD
#define IGN_2_Pin GPIO_PIN_10
#define IGN_2_GPIO_Port GPIOD
#define IGN_1_Pin GPIO_PIN_11
#define IGN_1_GPIO_Port GPIOD
#define SENS_RSVD2_Pin GPIO_PIN_12
#define SENS_RSVD2_GPIO_Port GPIOD
#define SENS_RSVD1_Pin GPIO_PIN_13
#define SENS_RSVD1_GPIO_Port GPIOD
#define SENS_CHARGE_Pin GPIO_PIN_14
#define SENS_CHARGE_GPIO_Port GPIOD
#define SENS_HANDBRAKE_Pin GPIO_PIN_15
#define SENS_HANDBRAKE_GPIO_Port GPIOD
#define SENS_OIL_Pin GPIO_PIN_6
#define SENS_OIL_GPIO_Port GPIOC
#define TIM8_CH2_SENS_SPEED_Pin GPIO_PIN_7
#define TIM8_CH2_SENS_SPEED_GPIO_Port GPIOC
#define SENS_TSPS_Pin GPIO_PIN_8
#define SENS_TSPS_GPIO_Port GPIOC
#define SENS_STARTER_Pin GPIO_PIN_9
#define SENS_STARTER_GPIO_Port GPIOC
#define CAN1_LBK_Pin GPIO_PIN_8
#define CAN1_LBK_GPIO_Port GPIOA
#define OUT_FAN_Pin GPIO_PIN_9
#define OUT_FAN_GPIO_Port GPIOA
#define OUT_RSVD1_Pin GPIO_PIN_10
#define OUT_RSVD1_GPIO_Port GPIOA
#define OUT_RSVD2_Pin GPIO_PIN_11
#define OUT_RSVD2_GPIO_Port GPIOA
#define OUT_STARTER_Pin GPIO_PIN_12
#define OUT_STARTER_GPIO_Port GPIOA
#define TACHOMETER_Pin GPIO_PIN_3
#define TACHOMETER_GPIO_Port GPIOD
#define CHECKENGINE_Pin GPIO_PIN_4
#define CHECKENGINE_GPIO_Port GPIOD
#define SPEEDMETER_Pin GPIO_PIN_5
#define SPEEDMETER_GPIO_Port GPIOD
#define FUEL_PUMP_Pin GPIO_PIN_6
#define FUEL_PUMP_GPIO_Port GPIOD
#define SPI1_NRST_Pin GPIO_PIN_7
#define SPI1_NRST_GPIO_Port GPIOD
#define SPI1_NSS_ADC_Pin GPIO_PIN_6
#define SPI1_NSS_ADC_GPIO_Port GPIOB
#define SW_NRST_Pin GPIO_PIN_7
#define SW_NRST_GPIO_Port GPIOB
#define TIM10_CH1_INJ_1_Pin GPIO_PIN_8
#define TIM10_CH1_INJ_1_GPIO_Port GPIOB
#define TIM11_CH1_INJ_2_Pin GPIO_PIN_9
#define TIM11_CH1_INJ_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define NVIC_PRIO_SPI1_ADC 7
#define NVIC_PRIO_SPI1_ADC_DMA_TX 7
#define NVIC_PRIO_SPI1_ADC_DMA_RX 7
#define NVIC_PRIO_SPI2_FLASH 8
#define NVIC_PRIO_SPI2_FLASH_DMA_TX 8
#define NVIC_PRIO_SPI2_FLASH_DMA_RX 8
#define NVIC_PRIO_SPI4_MISC 8
#define NVIC_PRIO_SPI4_MISC_DMA_TX 8
#define NVIC_PRIO_SPI4_MISC_DMA_RX 8
#define NVIC_PRIO_UART4_IMMO 7
#define NVIC_PRIO_UART4_IMMO_DMA_RX 15
#define NVIC_PRIO_UART5_COMM 7
#define NVIC_PRIO_UART5_COMM_DMA_TX 14
#define NVIC_PRIO_UART5_COMM_DMA_RX 15
#define NVIC_PRIO_UART8_BT 8
#define NVIC_PRIO_CAN1 13
#define NVIC_PRIO_RNG 14
#define NVIC_PRIO_TIM3_1K 6
#define NVIC_PRIO_TIM4_15US 5
#define NVIC_PRIO_TIM5_CSPS 3
#define NVIC_PRIO_TIM8_SPEED 6
#define NVIC_PRIO_TIM9_O2PWM 1
#define NVIC_PRIO_EXTI0_CSPS 2
#define NVIC_PRIO_EXTI4_KNOCK_INT 9

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
