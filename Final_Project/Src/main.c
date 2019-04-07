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

#include "stm32l475e_iot01_qspi.h"
#include "arm_math.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int i;

//buffers
float32_t flashBuffer[AUDIO_SAMPLE_SIZE];
uint16_t audioBufferLeft[AUDIO_SAMPLE_SIZE];
uint16_t audioBufferRight[AUDIO_SAMPLE_SIZE];

// ############
// matrices
float matrixBuffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 matrix;

float matrix2Buffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 matrix2;

float transposeMatrixBuffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 transposeMatrix;

float meanMatrixBuffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 meanMatrix;

float whiteMatrixBuffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 whiteMatrix;

float singleRowMatrixBuffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 singleRowMatrix;

float singleColMatrixBuffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 singleColMatrix;

float singleCol2MatrixBuffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 singleCol2Matrix;

float singleCol3MatrixBuffer[AUDIO_SAMPLE_SIZE];
arm_matrix_instance_f32 singleCol3Matrix;

float eigValueMatrixBuffer[4];
arm_matrix_instance_f32 eigValueMatrix;

float temp2by2MatrixBuffer[4];
arm_matrix_instance_f32 temp2by2Matrix;

float icaFilterMatrixBuffer[4];
arm_matrix_instance_f32 icaFilterMatrix;

float eigVectorMatrixBuffer[4];
arm_matrix_instance_f32 eigVectorMatrix;

float whiteningMatrixBuffer[4];
arm_matrix_instance_f32 whiteningMatrix;

float weightMatrixBuffer[2];
arm_matrix_instance_f32 weightMatrix;

float weightOldMatrixBuffer[2];
arm_matrix_instance_f32 weightOldMatrix;

float temp2by1MatrixBuffer[2];
arm_matrix_instance_f32 temp2by1Matrix;

float secondTemp2by1MatrixBuffer[2];
arm_matrix_instance_f32 secondTemp2by1Matrix;

float thirdTemp2by1MatrixBuffer[2];
arm_matrix_instance_f32 thirdTemp2by1Matrix;

float tempMatrixBuffer[4];
arm_matrix_instance_f32 tempMatrix;

float temp2MatrixBuffer[4];
arm_matrix_instance_f32 temp2Matrix;

float covarianceMatrixBuffer[4];
arm_matrix_instance_f32 covarianceMatrix;

int read_flash_find_min_max=0;
int read_flash=0;
int rx_cplt=1;
int tx_cplt=1;
int cmd_cplt=1;
int left_qspi=0;
int right_qspi=0;
int get_mean=1;
int flashAddr = 0;

float32_t mu1 = 0;
float32_t mu2 = 0;
float maxVal1 = -INFINITY;
float minVal1 = INFINITY;
float maxVal2 = -INFINITY;
float minVal2 = INFINITY;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;
DMA_HandleTypeDef hdma_dac_ch2;

QSPI_HandleTypeDef hqspi;
DMA_HandleTypeDef hdma_quadspi;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

#define MAX(a,b) (((a)>(b))?(a):(b))
#define EPSILON 0.001

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void getNorm(float arr[], int size, float* norm) {
	float sum;
	
	for(int j=0;j<size;j++) {
		sum += arr[j]*arr[j];
	}
	
	arm_sqrt_f32(sum,norm);
	
}


// calculate eigenValues and eigenVectors
void calculateEigen() {
	float a = covarianceMatrixBuffer[0];
	float b = covarianceMatrixBuffer[1];
	float c = covarianceMatrixBuffer[2];
	float d = covarianceMatrixBuffer[3];
	
	// calculate eigen values
	float tr = a+d;
	float tr2 = tr*tr;
	float det = a*d - b*c;
	
	float root;
	arm_sqrt_f32(tr2 - 4*det,&root);
	
	float eig1 = (tr + root )/2;
	float eig2 = (tr - root )/2;
	
	// create eigenValue matrix
	eigValueMatrixBuffer[0] = eig1;
  eigValueMatrixBuffer[1] = 0;
  eigValueMatrixBuffer[2] = 0;
	eigValueMatrixBuffer[3] = eig2;

	
	// create eigenVector matrix
	eigVectorMatrixBuffer[0] = a - eig1;
  eigVectorMatrixBuffer[1] = b;
	eigVectorMatrixBuffer[2] = c;
	eigVectorMatrixBuffer[3] = d - eig2;
	
	
	// normalize first row
	float norm;
	float temp = eigVectorMatrixBuffer[0]*eigVectorMatrixBuffer[0] + eigVectorMatrixBuffer[2]*eigVectorMatrixBuffer[2];
	arm_sqrt_f32(temp, &norm);
	eigVectorMatrixBuffer[0] /= norm;
	eigVectorMatrixBuffer[2] /= norm;
	// normalize second row
	temp = eigVectorMatrixBuffer[1]*eigVectorMatrixBuffer[1] + eigVectorMatrixBuffer[3]*eigVectorMatrixBuffer[3];
	arm_sqrt_f32(temp, &norm);
	eigVectorMatrixBuffer[1] /= norm;
	eigVectorMatrixBuffer[3] /= norm;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	QSPI_CommandTypeDef sCommand;

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
  MX_DAC1_Init();
  MX_QUADSPI_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_Base_Start(&htim6);
	
	BSP_QSPI_Init();
	int reload=0;	//set reload to 0 to save time if flash memory is already filled

	if(reload)BSP_QSPI_Erase_Chip(); // this can take like 30 seconds. 
	
	// matrix coefficients
	int a11 = 1;
	int a12 = 0;
	int a21 = 1;
	int a22 = 0;
	float32_t angle1,angle2;
	float32_t sinOne,sinTwo;
	
	// create mixed signal and transfer over to flash
	if(reload)
	for(i=0;i<AUDIO_FOUR_SECONDS;i+=2) {														
		// 400 and 700 hz frequency spread over 16000 samples per second for two seconds
		angle1 = TWO_PI_DIVIDED_BY_16000*((400*i)%16000);
		angle2 = TWO_PI_DIVIDED_BY_16000*((700*i)%16000);
		sinOne = arm_sin_f32(angle1); // EVEN FLASH ADDR IS THE FIRST SIGNAL
		sinTwo = arm_sin_f32(angle2); // ODD FLASH ADDR IS THE SECOND SIGNAL
		flashBuffer[i%AUDIO_SAMPLE_SIZE] = (a11*sinOne + a12*sinTwo)/(a11+a12); // combine and normalize between 0 and 1
		flashBuffer[(i+1)%AUDIO_SAMPLE_SIZE] = (a21*sinOne + a22*sinTwo)/(a21+a22);
		if (!((i+2)%AUDIO_SAMPLE_SIZE)) {
			BSP_QSPI_Write((uint8_t*) flashBuffer,flashAddr,AUDIO_SAMPLE_SIZE_FLOAT); // write 4000 bytes at a time
			flashAddr += AUDIO_SAMPLE_SIZE_FLOAT;
		}
	}
	
	// this is here to demonstrate that buffers are cleared and it is really reading from flash
	for(i=0;i<AUDIO_SAMPLE_SIZE;i++) {
		flashBuffer[i] = 0;
	}
	
	// reset the read addr from flash
	flashAddr = 0;
	
	// get the mean
	// since we are using DMA, there's no point using CMSIS since we're bottlenecked by the speed of reading from flash
	while (flashAddr < AUDIO_STORAGE_SIZE) {
		get_mean = 1; // set flag so that rxCallback knows which code to execute
	
		BSP_QSPI_Read_DMA((uint8_t *) flashBuffer,flashAddr,AUDIO_SAMPLE_SIZE_FLOAT);

		while(get_mean); // wait for read to finish before calling next one
	}
	mu1 /= AUDIO_TWO_SECONDS; // This single division is surely faster than calling CMSIS for mean after using processor for QSPI
	mu2 /= AUDIO_TWO_SECONDS;
	
	
	// mean matrix is 32000 by 2, even indices are first column, odd indices are second column.
	for(i=0;i<AUDIO_SAMPLE_SIZE;i++){ 
		if (i%2) {
			meanMatrixBuffer[i] = mu2; // init the meanMatrixBuffer with the means calculated
		}
		else {
			meanMatrixBuffer[i] = mu1;
		}
	}
	
	// IMPORTANT TO KNOW: In flash, it's stored as a 32000x2 matrix, meaning it goes sig1, sig2, sig1, sig2. Every 2 values is a row. This is so that when we read from flash
	// we get both signals. Otherwise, we'd only get the first signal from a single read and would need to do 2 reads every time we processed info because they'd be in different 
	// places in flash, and we'd also have to keep track of these indices which would be complicated. 
	
	arm_mat_init_f32(&matrix, ROW_SIZE, 2, matrixBuffer); // ROW_SIZE x 2
	arm_mat_init_f32(&matrix2, ROW_SIZE, 2, matrix2Buffer); // ROW_SIZE x 2
	arm_mat_init_f32(&transposeMatrix, 2, ROW_SIZE, transposeMatrixBuffer); // 2 x ROW_SIZE
	arm_mat_init_f32(&meanMatrix, ROW_SIZE, 2, meanMatrixBuffer); // to be subtracted from "matrix" to centralize it.
	arm_mat_init_f32(&covarianceMatrix, 2, 2, covarianceMatrixBuffer);
	arm_mat_init_f32(&tempMatrix, ROW_SIZE, 2, tempMatrixBuffer);
	arm_mat_init_f32(&temp2Matrix, ROW_SIZE, 2, temp2MatrixBuffer);
	arm_mat_init_f32(&eigValueMatrix, 2, 2, eigValueMatrixBuffer);
	arm_mat_init_f32(&eigVectorMatrix, 2, 2, eigVectorMatrixBuffer);
	arm_mat_init_f32(&whiteningMatrix, 2, 2, whiteningMatrixBuffer);
	arm_mat_init_f32(&weightMatrix, 2, 1, weightMatrixBuffer);
	arm_mat_init_f32(&weightOldMatrix, 2, 1, weightOldMatrixBuffer);
	arm_mat_init_f32(&temp2by1Matrix,2,1,temp2by1MatrixBuffer);
	arm_mat_init_f32(&secondTemp2by1Matrix,2,1,secondTemp2by1MatrixBuffer);
	arm_mat_init_f32(&thirdTemp2by1Matrix,2,1,thirdTemp2by1MatrixBuffer);
	arm_mat_init_f32(&whiteMatrix,ROW_SIZE,2,whiteMatrixBuffer);
	arm_mat_init_f32(&singleColMatrix,AUDIO_SAMPLE_SIZE,1,singleColMatrixBuffer); // AUDIO_SAMPLE_SIZE x 1
	arm_mat_init_f32(&singleCol2Matrix,AUDIO_SAMPLE_SIZE,1,singleCol2MatrixBuffer); // AUDIO_SAMPLE_SIZE x 1
	arm_mat_init_f32(&singleCol3Matrix,AUDIO_SAMPLE_SIZE,1,singleCol3MatrixBuffer); // AUDIO_SAMPLE_SIZE x 1
	arm_mat_init_f32(&temp2by2Matrix,2,2,temp2by2MatrixBuffer); // 2x2
	arm_mat_init_f32(&icaFilterMatrix,2,2,icaFilterMatrixBuffer); // 2x2
	
	flashAddr = 0; // reset the read addr from flash
	
	BSP_QSPI_Read_DMA( (uint8_t *) flashBuffer, flashAddr, AUDIO_SAMPLE_SIZE_FLOAT);
	 
	while (flashAddr < AUDIO_STORAGE_SIZE) {
		while (read_flash_find_min_max); // wait for read to complete
		read_flash_find_min_max = 1; // set flag so that rxCallback knows which code to execute. Find min and max of signal for later
		BSP_QSPI_Read_DMA((uint8_t *) flashBuffer,flashAddr,AUDIO_SAMPLE_SIZE_FLOAT); // start next DMA read
		
		// calculate the covariance matrix for this chunk
		arm_mat_sub_f32(&matrix,&meanMatrix,&tempMatrix); // centralize matrix (need to store in temp because we can't write where we read from)
		arm_mat_trans_f32(&tempMatrix, &transposeMatrix); // fills transpose matrix with transpose (2x32000)
		arm_mat_mult_f32(&transposeMatrix, &tempMatrix,&matrix); // multiply transpose matrix with centralized input matrix -> 2x32000 x 32000x2 ~ 2x2
		arm_mat_add_f32(&matrix, &covarianceMatrix,&tempMatrix); // add partial sum to total 
		for(i=0;i<4;i++) { covarianceMatrixBuffer[i] = tempMatrixBuffer[i]; } // store total in covarianceMatrix (this may be unnecessary or can be optimized to use different buffer every time to eliminate copying)
	}
	
	arm_mat_scale_f32(&covarianceMatrix,AUDIO_TWO_SECONDS,&covarianceMatrix); // divide by N to get covariance
	
	calculateEigen();
	
	// get the whitening matrix
	
	// take the square root of the eigenvalues
	arm_sqrt_f32(eigValueMatrixBuffer[0],&whiteningMatrixBuffer[0]);
	whiteningMatrixBuffer[1] = 0;
	arm_sqrt_f32(eigValueMatrixBuffer[3],&whiteningMatrixBuffer[3]);
	whiteningMatrixBuffer[2] = 0;
	arm_mat_inverse_f32(&whiteningMatrix,&tempMatrix); // store inverse of root eigenvalue matrix in temp
	arm_mat_inverse_f32(&eigVectorMatrix,&temp2Matrix); // store inverse of eigenvector matrix in temp2
	arm_mat_mult_f32(&tempMatrix, &temp2Matrix, &whiteningMatrix);
	
	float norm;
	
	temp2by1MatrixBuffer[0] = rand()%100; // use modulo 100 so that the integer isn't too large which can make computation lengthy
	temp2by1MatrixBuffer[1] = rand()%100; // use modulo 100 so that the integer isn't too large which can make computation lengthy
	
	getNorm(temp2by1MatrixBuffer,2,&norm);
	arm_mat_scale_f32(&temp2by1Matrix,(1/norm),&weightMatrix); // normalize the weight matrix
	weightOldMatrixBuffer[0] = 0; //init oldWeight to 0
	weightOldMatrixBuffer[1] = 0;
	
	
	
	for(i=0;i<1000;i++) {
		
		// test for convergence
		arm_sub_f32(weightMatrixBuffer,weightOldMatrixBuffer,temp2by1MatrixBuffer,2);
		getNorm(temp2by1MatrixBuffer,2,&norm);
		if(norm < EPSILON) {
			break; 
		}
		arm_add_f32(weightMatrixBuffer,weightOldMatrixBuffer,temp2by1MatrixBuffer,2);
		getNorm(temp2by1MatrixBuffer,2,&norm);
		if(norm < EPSILON) {
			break; 
		}
		
		// store the weight into oldWeight
		weightOldMatrixBuffer[0] = weightMatrixBuffer[0]; // store old weight
		weightOldMatrixBuffer[1] = weightMatrixBuffer[1];
		
		// set weight to 0
		weightMatrixBuffer[0] = 0;
		weightMatrixBuffer[1] = 0;
		
		// update the weight chunk by chunk
		BSP_QSPI_Read_DMA( (uint8_t *) flashBuffer, flashAddr, AUDIO_SAMPLE_SIZE_FLOAT);
		while (flashAddr < AUDIO_STORAGE_SIZE) {
			while (read_flash); // wait for read to complete
			read_flash = 1; // set flag so that rxCallback knows which code to execute
			BSP_QSPI_Read_DMA((uint8_t *) flashBuffer,flashAddr,AUDIO_SAMPLE_SIZE_FLOAT); // start next DMA read
			
			// white_mat = center_mat' * whitening_mat
			// in the case that DMA finishes before calculations are all done, never use "matrix" after we have matrix2, which is the centralized matrix
			arm_mat_sub_f32(&matrix,&meanMatrix,&matrix2); // centralize matrix, matrix2=center_mat (need to store in temp because we can't write where we read from)
			arm_mat_mult_f32(&matrix2,&whiteningMatrix,&whiteMatrix); // white matrix is transpose of jerry's whiteMatrix cuz of how we store it
																														// whitening_mat * center_mat = (center_mat' * whitening_mat)' since whitening_mat is symmetric
			
			arm_mat_mult_f32(&whiteMatrix,&weightMatrix,&singleColMatrix); // white_mat * weight ~ ROW_SIZEx1
			
			// take result to the third power
			arm_mult_f32(singleColMatrixBuffer,singleColMatrixBuffer,singleCol2MatrixBuffer, AUDIO_SAMPLE_SIZE);
			arm_mult_f32(singleColMatrixBuffer,singleCol2MatrixBuffer,singleCol3MatrixBuffer, AUDIO_SAMPLE_SIZE);
			
			arm_mat_trans_f32(&whiteMatrix, &transposeMatrix); // fills transposeMatrix with transpose of whiteMatrix
			
			// transpose is first argument because we store it in memory as "transpose" already
			arm_mat_mult_f32(&transposeMatrix,&singleCol3Matrix, &temp2by1Matrix); // 2xROW_SIZE * ROW_SIZEx1 ~ 2x1
			
			arm_scale_f32(temp2by1MatrixBuffer,ONE_OVER_TOTAL_SAMPLE_SIZE,secondTemp2by1MatrixBuffer,2); // divide by num_samples
			arm_scale_f32(weightMatrixBuffer,3,temp2by1MatrixBuffer,2); // 3*weight
			arm_sub_f32(secondTemp2by1MatrixBuffer,temp2by1MatrixBuffer,thirdTemp2by1MatrixBuffer,2); // finish weight update for this chunk
			arm_add_f32(thirdTemp2by1MatrixBuffer,weightMatrixBuffer, secondTemp2by1MatrixBuffer, 2); // add chunnk to total (can't write to weightMatrixBuffer directly since we are reading from there)
			for(i=0;i<4;i++) { weightMatrixBuffer[i] = secondTemp2by1MatrixBuffer[i]; } // store total in weightMatrixBuffer 
		}
		// normalize the weight
			getNorm(thirdTemp2by1MatrixBuffer,2,&norm);
			arm_scale_f32(thirdTemp2by1MatrixBuffer,(1/norm),weightMatrixBuffer,2);
	}
	
	// create basis set transpose after weights have converged
	// if weight matrix is [a;b] then basis set is [a -b; b a], and basis set transpose is therefore [a b; -b a]
	temp2by2MatrixBuffer[0] = weightMatrixBuffer[0];
	temp2by2MatrixBuffer[1] = weightMatrixBuffer[1];
	temp2by2MatrixBuffer[2] = -weightMatrixBuffer[1];
	temp2by2MatrixBuffer[3] = weightMatrixBuffer[0];
	
	arm_mat_mult_f32(&temp2by2Matrix,&whiteningMatrix,&icaFilterMatrix);
	
	read_flash = 1;
	BSP_QSPI_Read_DMA((uint8_t *) flashBuffer,flashAddr,AUDIO_SAMPLE_SIZE_FLOAT); // start next DMA read
	while (read_flash); // wait for read to complete
	// start DMA transfer to DAC
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,(uint32_t*)audioBufferLeft,AUDIO_SAMPLE_SIZE, DAC_ALIGN_12B_R);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2,(uint32_t*)audioBufferRight,AUDIO_SAMPLE_SIZE, DAC_ALIGN_12B_R);
	
	// max value recorded needs to be adjusted by offset that will be applied later to normalize it
	maxVal1 -= minVal1;
	maxVal2 -= minVal2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
		if (flashAddr >= AUDIO_STORAGE_SIZE) {
			flashAddr = 0;
		}
		while (read_flash); // wait for read to complete
		read_flash = 1; // set flag so that rxCallback knows which code to execute
		BSP_QSPI_Read_DMA( (uint8_t *) flashBuffer,flashAddr,AUDIO_SAMPLE_SIZE_FLOAT); // start next DMA read
		arm_mat_trans_f32(&matrix, &transposeMatrix); // fills transposeMatrix with transpose of matrix. Need to do this because stored as transpose in memory
		arm_mat_mult_f32(&icaFilterMatrix,&transposeMatrix,&matrix2); // store result of filtering in matrix2 which is 2xROW_SIZE
		// store signal as 0-4095 in DAC buffer
		for (i=0;i<ROW_SIZE;i++) {
			matrix2Buffer[i] -= minVal1;
			matrix2Buffer[i] *= (4095/maxVal1);
			audioBufferLeft[i] = matrix2Buffer[i];
		}
		for (i=ROW_SIZE;i<AUDIO_SAMPLE_SIZE;i++) {
			matrix2Buffer[i] -= minVal2;
			matrix2Buffer[i] *= (4095/maxVal2);
			audioBufferRight[i] = matrix2Buffer[i];
		}
		
		
    /* USER CODE BEGIN 3 */
  }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */
	HAL_QSPI_DeInit(&hqspi);

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 0;
  hqspi.Init.FifoThreshold = 2;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 22;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

	while(1);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
