/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THROTTLE_MIN 82
#define THROTTLE_MAX 7950
#define SCALING_FACTOR 65535
#define TB_PRD 4000
#define offset 5

#define STOP 0
#define STARTUP 1
#define RUNNING 2
#define M_PI 3.14159265358979323846
#define sqrtThree 1.732050807568877293
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t counter=0;


int value=0;
int state = STOP,pstate = STOP;
float T1 = 0, T2 = 0, T0 = 0;
int enc_pos = 0;	// It is encoder position of the rotor
int last_tick = 0 ,current_tick = 0;
int throttle=0;
int enc_elecpos=0;
int sector=0;


float sin_thetaelec[33] ;
		/*
{0,
									6423,
									12785,
									19023,
									25079,
									30892,
									36409,
									41574,
									46340,
									50659,
									54490,
									2144,
									8554,
									14881,
									21065,
									27046,
									32767,
									38172,
									43210,
									47831,
									51992,
									55652,
									4286,
									10675,
									16961,
									23084,
									28985,
									34606,
									39895,
									44799,
									49271,
									53269,
									56754};

									*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
inline int capture_throttle();
//void Signal_Commutation(int enc_elecpos);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  for(int i=0;i<33;i++)
  {

	  sin_thetaelec[i] = sin(i*M_PI/33 + 90*M_PI/360);

  }

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_ADC_Start(&hadc1);
  last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      throttle= 4000;
      enc_elecpos=enc_pos%64;

	  if(enc_elecpos < 33)
	  	{
	  		T1 = (sin_thetaelec[enc_elecpos])*(sqrtThree*1000);
	  		T2 = (sin_thetaelec[32 - enc_elecpos])*(sqrtThree*1000) ;
	  	}
	  	else
	  	{
	  		T1 = (sin_thetaelec[enc_elecpos - 32])*(sqrtThree*1000) ;
	  		T2 = (sin_thetaelec[64 - enc_elecpos])*(sqrtThree*1000) ;
	  	}
	  	T0 = TB_PRD - T1 - T2;
	  	sector = enc_elecpos/10;
	  	switch(sector)
	  	{
	  	case 0:
	  		TIM1->CCR3 = T0/2;
	  		TIM1->CCR2 = T0/2 + T1 + T2;
	  		TIM1->CCR1 = T0/2 + T1;
	  	break;
	  	case 1:
	  		TIM1->CCR3 = T0/2 + T2;
	  		TIM1->CCR2 = T0/2 + T2 + T1;
	  		TIM1->CCR1 = T0/2;
	  	break;
	  	case 2:
	  		TIM1->CCR3 = T0/2 + T1 + T2;
	  		TIM1->CCR2 = T0/2 + T1;
	  		TIM1->CCR1 = T0/2;
	  	break;
	  	case 3:
	  		TIM1->CCR3 = T0/2 + T2 + T1;
	  		TIM1->CCR2 = T0/2;
	  		TIM1->CCR3 = T0/2 + T2;
	  	break;
	  	case 4:
	  		TIM1->CCR3 = T0/2 + T1;
	  		TIM1->CCR2 = T0/2;
	  		TIM1->CCR1 = T0/2 + T1 + T2;
	  	break;
	  	case 5:
	  		TIM1->CCR3 = T0/2;
	  		TIM1->CCR2 = T0/2 + T2;
	  		TIM1->CCR1 = T0/2 + T2 + T1;
	  	break;
	  	default:
	  		TIM1->CCR3 = 0;
	  		TIM1->CCR2 = 0;
	  		TIM1->CCR1 = 0;
	  	}


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int capture_throttle()
{
	  int c_throttle = 0;
	  int adc_val = 0;
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  adc_val = HAL_ADC_GetValue(&hadc1);

	  if (adc_val < 1300)
	  {
		c_throttle = THROTTLE_MIN;
	  }
	  else if (adc_val <= 2900)
	  {
		c_throttle = THROTTLE_MIN + ((adc_val - 1300)*THROTTLE_MAX)/(THROTTLE_MAX - THROTTLE_MIN);
	  }
	  else
	  {
		c_throttle = THROTTLE_MAX;
	  }
	  return c_throttle;
}

/*
void Signal_Commutation(int enc_elecpos)
{

	//throttle = capture_throttle();
	int sector;
	counter++;
	if(enc_elecpos < 33)
	{
		T1 = (sin_thetaelec[enc_elecpos])/SCALING_FACTOR;
		T2 = (sin_thetaelec[32 - enc_elecpos])/SCALING_FACTOR ;
	}
	else
	{
		T1 = (sin_thetaelec[enc_elecpos - 32])/SCALING_FACTOR ;
		T2 = (sin_thetaelec[64 - enc_elecpos])/SCALING_FACTOR ;
	}
	T0 = TB_PRD - T1 - T2;
	sector = enc_elecpos/10;
	switch(sector)
	{
	case 0:
		TIM1->CCR3 = T0/2;
		TIM1->CCR2 = T0/2 + T1 + T2;
		TIM1->CCR1 = T0/2 + T1;
	break;
	case 1:
		TIM1->CCR3 = T0/2 + T2;
		TIM1->CCR2 = T0/2 + T2 + T1;
		TIM1->CCR1 = T0/2;
	break;
	case 2:
		TIM1->CCR3 = T0/2 + T1 + T2;
		TIM1->CCR2 = T0/2 + T1;
		TIM1->CCR1 = T0/2;
	break;
	case 3:
		TIM1->CCR3 = T0/2 + T2 + T1;
		TIM1->CCR2 = T0/2;
		TIM1->CCR3 = T0/2 + T2;
	break;
	case 4:
		TIM1->CCR3 = T0/2 + T1;
		TIM1->CCR2 = T0/2;
		TIM1->CCR1 = T0/2 + T1 + T2;
	break;
	case 5:
		TIM1->CCR3 = T0/2;
		TIM1->CCR2 = T0/2 + T2;
		TIM1->CCR1 = T0/2 + T2 + T1;
	break;
	default:
		TIM1->CCR3 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR1 = 0;
	}
}
*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	switch(state)
	{
	case STOP:
		if(throttle > 0)
			state = STARTUP;
	break;
	case STARTUP:
		pstate = STARTUP;
		enc_pos++;
		if(enc_pos == 512)
			enc_pos = 0;
	//	Signal_Commutation(enc_pos % 64);
	break;
	case RUNNING:
		pstate = RUNNING;
	//	Signal_Commutation(enc_pos % 64);
	break;
	default:
		state = STOP;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_1)
	{
		state = RUNNING;
		enc_pos = 0;
		last_tick = HAL_GetTick();
	}
	if(state == RUNNING)
	if(GPIO_Pin == GPIO_PIN_6)
	{
		current_tick = HAL_GetTick();
		enc_pos++;
		if(enc_pos == 512)
			enc_pos = 0;
		last_tick = current_tick;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
