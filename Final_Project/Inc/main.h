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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define AUDIO_BUFFER_SIZE 2000 //2000 samples
#define AUDIO_BUFFER_SIZE_8BIT 4000
#define AUDIO_BUFFER_SIZE_FLOAT 8000
#define AUDIO_TWO_SECONDS 32000
#define AUDIO_FOUR_SECONDS 2*AUDIO_TWO_SECONDS
#define AUDIO_TWO_SECONDS_8BIT 64000 // number of bytes to represent 32000 samples of a 16 bit number
#define AUDIO_TWO_SECONDS_FLOAT 128000 // number of bytes to represent 32000 samples of a float
#define AUDIO_STORAGE_SIZE 2*AUDIO_TWO_SECONDS_FLOAT
#define TWO_PI_DIVIDED_BY_16000 0.00039269908

#define NUMBER_OF_AUDIO_BUFFERS_FOR_4_SECONDS_OF_AUDIO 32 // 64000/AUDIO_BUFFER_SIZE = 32
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

//buffers
float flashBuffer[AUDIO_BUFFER_SIZE];
uint16_t audioBufferLeft[AUDIO_BUFFER_SIZE];
uint16_t audioBufferRight[AUDIO_BUFFER_SIZE];

// ############
// matrices
float matrixBuffer[AUDIO_BUFFER_SIZE];
arm_matrix_instance_f32 matrix;

float transposeBuffer[AUDIO_BUFFER_SIZE];
arm_matrix_instance_f32 transposeMatrix;

float meanMatrixBuffer[AUDIO_BUFFER_SIZE];
arm_matrix_instance_f32 meanMatrix;

float resultMatrixBuffer[4];
arm_matrix_instance_f32 resultMatrix;

float eigValueMatrixBuffer[4];
arm_matrix_instance_f32 eigValueMatrix;

float eigVectorMatrixBuffer[4];
arm_matrix_instance_f32 eigVectorMatrix;

float tempMatrixBuffer[4];
arm_matrix_instance_f32 tempMatrix;

float covarianceMatrixBuffer[4];
arm_matrix_instance_f32 covarianceMatrix;
// ############


// flags
int get_covariance=0;
int rx_cplt=1;
int tx_cplt=1;
int cmd_cplt=1;
int left_qspi=0;
int right_qspi=0;
int get_mean=1;
int flashAddr = 0;

int mean = 0;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
