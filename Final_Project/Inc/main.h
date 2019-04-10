/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define AUDIO_SAMPLE_SIZE 2000 //2000 samples
#define ONE_OVER_TOTAL_SAMPLE_SIZE 0.00003125 // 1/32000
#define AUDIO_SAMPLE_SIZE_SHORT 4000 // one sample is 2 bytes when converted to 12 bit resolution integer for DAC
#define AUDIO_SAMPLE_SIZE_FLOAT 8000 // each sample is 4 bytes when stored as a float
#define AUDIO_TWO_SECONDS 32000
#define ONE_OVER_AUDIO_TWO_SECONDS_FLOAT 0.00003125097
#define AUDIO_FOUR_SECONDS 64000
#define AUDIO_TWO_SECONDS_8BIT 64000 // number of bytes to represent 32000 samples of a 16 bit number
#define AUDIO_TWO_SECONDS_FLOAT 128000 // number of bytes to represent 32000 samples of a float
#define AUDIO_STORAGE_SIZE 2*AUDIO_TWO_SECONDS_FLOAT
#define TWO_PI_DIVIDED_BY_16000 0.00039269908

#define NUMBER_OF_AUDIO_BUFFERS_FOR_4_SECONDS_OF_AUDIO 32 // 64000/AUDIO_SAMPLE_SIZE = 32
#define ROW_SIZE 1000
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

// ############

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
