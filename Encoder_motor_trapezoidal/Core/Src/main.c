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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CPT 512
#define STARTUP 1
#define RUNNING 2
#define MEASURE 3
#define PAUSE 4
#define EXC_TIME 100
#define EXC_THROTTLE 85
#define STOP 0
#define comm_in_a_rev 48
//#define DELTA_C_MAX 42
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float adc_val=0;
float throttle=0;
uint8_t et, p_et=0;
int state = STOP, pstate = STOP;
volatile int delta_c = 0;
int h1 = 0, h2 = 0, h3 = 0;
int seq[6] = { 6, 4, 5, 1, 3, 2};
int order = 5;
volatile int A = 0, B = 0, I = 0;
int check = 0;
int shift = 0;
int enc_state = 0;
int n_enc_state = -1;
int delta_c_captured = 0;
//int DELTA_C_MAX = 42;
int DELTA_C_MAX = 11;
int count = 100;
uint8_t dir = 0; // 0-> left tyre, 1-> right tyre
int offset = 0;
int position = 0, position_captured = 0;
int sector = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
inline int capture_throttle(void);
void UpdateCommutation(void);
void SignalCommutation(int);
void nextState(void);
//void switch_enc_state(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//26 45 47 52
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
  /* USER CODE BEGIN 2 */
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_ADC_Start(&hadc1);
  A = (GPIOB->IDR & GPIO_PIN_6)?	(1)	:	(0);
  B = (GPIOB->IDR & GPIO_PIN_4)?	(1)	:	(0);
  p_et = A << 1 | B ;
//  switch_enc_state();
//  UpdateCommutation();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 6 -> 4 -> 5 -> 1 -> 3 -> 2
//  state = -1;
  while (1)
  {
	  switch(state)
	  {
	  	  case STOP :
	  		  throttle = capture_throttle();
	  		  SignalCommutation(7);
	  		  if(throttle > 0)
	  			  state = STARTUP;
	  		  pstate = STOP;
	  		  check = 0;
	      break;


	  	  case STARTUP:
	  		  throttle = EXC_THROTTLE;
//	  		  dir = 1;//1-> left tyre, 0 -> right tyre
			  nextState();
			  SignalCommutation(order);
			  HAL_Delay(EXC_TIME);
	  		  pstate = STARTUP;
	  		  throttle = capture_throttle();
	  		  if(throttle == 0) state = STOP;
	  		  else if(state != RUNNING) state = STARTUP;
	  	  break;

	  	  case RUNNING://26 45 47 52

	  		  throttle = capture_throttle();
	  		  //UpdateCommutation();
//	  		  SignalCommutation(order);
//	  		 state = (throttle == 0) ? STOP : RUNNING;
	  		 if(throttle == 0) state = PAUSE;
	  		 pstate = RUNNING;
	      break;

	  	  case PAUSE:
	  		  throttle = capture_throttle();
	  		  UpdateCommutation();
	  		  if(throttle > 100)
	  			  state = RUNNING;
	  	  break;
	  	  default:
	  		  state = STOP;

	  }
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int capture_throttle()
{
	  int c_throttle = 0.0;
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  adc_val = HAL_ADC_GetValue(&hadc1);

	  if (adc_val < 1300)
		  {
			c_throttle = 0;
		  }
	  else if (adc_val <= 2900)
		  {
			c_throttle = ((adc_val- 1300)*780)/1650;
		  }
	  else
		  {
			c_throttle = 780;
		  }
	  return c_throttle;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == GPIO_PIN_6) || (GPIO_Pin == GPIO_PIN_2))//TX = PA2
	{
		if(state == RUNNING)
			if((delta_c == 0) || (delta_c == 11) || (delta_c == 21) || (delta_c == 32) || (delta_c == 42) || (delta_c == 53))
			{
				UpdateCommutation();
			}
		if(delta_c >= 0 && delta_c < 63)
		{
			delta_c = delta_c + 1;
		}
		else
		{
			delta_c = 0;
		}
//		delta_c = delta_c + 1;
//		delta_c %= 63;

	}
	if((GPIO_Pin == GPIO_PIN_15) || (GPIO_Pin == GPIO_PIN_3))//RX = PA3
	{
		if(pstate == STARTUP)
		{
			delta_c = 0;
			state = RUNNING;
		}
		position_captured = position;
		if(position_captured >= 500 && position_captured <= 520)
		{
			delta_c = 0;
		}
		position = 0;
	}
}
void UpdateCommutation()
{
//	A = (GPIOB->IDR & GPIO_PIN_6)?	(1)	:	(0);
//	B = (GPIOB->IDR & GPIO_PIN_4)?	(1)	:	(0);
//	I = (GPIOA->IDR & GPIO_PIN_15)?	(1)	:	(0);
//	et = A << 1 | B;

//	delta_c += 1;

	if(adc_val < 1100 || delta_c < 0)
	  {
		SignalCommutation(7);
//		  TIM1 -> CCR1 = 0;				//ghc off
//		  TIM1 -> CCR4 = 0;				//gha off
//		  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
//		  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
//		  GPIOA -> BSRR	 = GPIO_BSRR_BR8; //glc on
//		  TIM1 -> CCR2 = 0;		//ghb on
	  }
	else if(delta_c >= 42 && delta_c <= 52)//6
	 {//GLC GHB
		if(dir)
			SignalCommutation(5);
		else
			SignalCommutation(5);
		  //GLA GHB
//		  TIM1 -> CCR1 = 0;				//ghc off
//		  TIM1 -> CCR4 = 0;				//gha off
//		  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
//		  GPIOA -> BSRR = GPIO_BSRR_BR8;//glc off
//		  GPIOA -> BSRR	 = GPIO_BSRR_BS10; //gla on
//		  TIM1 -> CCR2 = throttle;		//ghb on
	 }      		// GLC PWM ON

	else if(delta_c >= 53 && delta_c <= 63)//4
	  {
		  //GLC GHA
		if(dir)
			SignalCommutation(0);
		else
			SignalCommutation(4);
//		  TIM1 -> CCR1 = 0;				//ghc off
//		  TIM1 -> CCR4 = 0;				//gha off
//		  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
//		  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
//		  GPIOA -> BSRR	 = GPIO_BSRR_BS8; //glc on
//		  TIM1 -> CCR2 = throttle;		//ghb on
	  }

	else if(delta_c >= 0 && delta_c <= 10){//5
	  //GLB GHA
		if(dir)
			SignalCommutation(1);
		else
			SignalCommutation(3);
		  //GLC GHA
//		  TIM1 -> CCR1 = 0;				//ghc off
//		  TIM1 -> CCR2 = 0;				//ghb off
//		  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
//		  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
//		  GPIOA -> BSRR	 = GPIO_BSRR_BS8; //glc on
//		  TIM1 -> CCR4 = throttle;		//gha on
	}
	else if(delta_c >= 11 && delta_c <= 20){//1
		  //GLB GHC
		if(dir)
			SignalCommutation(2);
		else
			SignalCommutation(2);
		  //GLB GHA
//		  TIM1 -> CCR1 = 0;				//ghc off
//		  TIM1 -> CCR2 = 0;				//ghb off
//		  GPIOA -> BSRR	= GPIO_BSRR_BR8;//glc off
//		  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
//		  GPIOA -> BSRR	 = GPIO_BSRR_BS9; //glb on
//		  TIM1 -> CCR4 = throttle;		//gha on
		}

	else if(delta_c >= 21 && delta_c <= 31){//3
		  //GLA GHC
		if(dir)
			SignalCommutation(3);
		else
			SignalCommutation(1);
		  //GLB GHC
//		  TIM1 -> CCR2 = 0;				//gha off
//		  TIM1 -> CCR4 = 0;				//gha off
//		  GPIOA -> BSRR	= GPIO_BSRR_BR8;//glc off
//		  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
//		  GPIOA -> BSRR	 = GPIO_BSRR_BS9; //glb on
//		  TIM1 -> CCR1 = throttle;		//ghc on
		}

	else if(delta_c >= 32 && delta_c <= 41){//2
		  //GLA GHB
		if(dir)
			SignalCommutation(4);
		else
			SignalCommutation(0);
		  //GLA GHC
//		  TIM1 -> CCR2 = 0;				//ghb off
//		  TIM1 -> CCR4 = 0;				//gha off
//		  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
//		  GPIOA -> BSRR = GPIO_BSRR_BR8;//glc off
//		  GPIOA -> BSRR	 = GPIO_BSRR_BS10; //gla on
//		  TIM1 -> CCR1 = throttle;		//ghc on
	}

}

void nextState(){
	if(dir == 0){
	  if(order == 5) order = 0;
	  else order++;
	}
	else{
	  if(order == 0) order = 5;
	  else order--;
	}
}

void SignalCommutation(int ind)
{
		//uint8_t t = seq[ind];
		int t = ind;
//		sector = t;
		if(t==7 || adc_val < 1200)		//hall failure condition(t==7)
		  {
			  TIM1 -> CCR1 = 0;				//ghc off
			  TIM1 -> CCR4 = 0;				//gha off
			  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
			  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
			  GPIOA -> BSRR	 = GPIO_BSRR_BR8; //glc on
			  TIM1 -> CCR2 = 0;		//ghb on
		  }

		/* for commutation stage sequence refer
		 * asynchronous 1xPWM from DRV8323
		 * datasheet table5 revised march22
		*/
		else if(t==0)//6
		 {//GLC GHB
			  TIM1 -> CCR1 = 0;				//ghc off
			  TIM1 -> CCR4 = 0;				//gha off
			  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
			  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
			  GPIOA -> BSRR	 = GPIO_BSRR_BS8; //glc on
			  TIM1 -> CCR2 = throttle;		//ghb on
		 }      		// GLC PWM ON

		else if(t==1)//4
		  {
			  //GLC GHA
			  TIM1 -> CCR1 = 0;				//ghc off
			  TIM1 -> CCR2 = 0;				//ghb off
			  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
			  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
			  GPIOA -> BSRR	 = GPIO_BSRR_BS8; //glc on
			  TIM1 -> CCR4 = throttle;		//gha on
		  }

		else if(t==2){//5
		  //GLB GHA
			  TIM1 -> CCR1 = 0;				//ghc off
			  TIM1 -> CCR2 = 0;				//ghb off
			  GPIOA -> BSRR	= GPIO_BSRR_BR8;//glc off
			  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
			  GPIOA -> BSRR	 = GPIO_BSRR_BS9; //glb on
			  TIM1 -> CCR4 = throttle;		//gha on
		}

		else if(t==5){//2
			  //GLA GHB
			  TIM1 -> CCR1 = 0;				//ghc off
			  TIM1 -> CCR4 = 0;				//gha off
			  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
			  GPIOA -> BSRR = GPIO_BSRR_BR8;//glc off
			  GPIOA -> BSRR	 = GPIO_BSRR_BS10; //gla on
			  TIM1 -> CCR2 = throttle;		//ghb on
		}

		else if(t==3){//1
			  //GLB GHC
			  TIM1 -> CCR2 = 0;				//gha off
			  TIM1 -> CCR4 = 0;				//gha off
			  GPIOA -> BSRR	= GPIO_BSRR_BR8;//glc off
			  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
			  GPIOA -> BSRR	 = GPIO_BSRR_BS9; //glb on
			  TIM1 -> CCR1 = throttle;		//ghc on
			}

		else if(t==4){//3
			  //GLA GHC
			  TIM1 -> CCR2 = 0;				//ghb off
			  TIM1 -> CCR4 = 0;				//gha off
			  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
			  GPIOA -> BSRR = GPIO_BSRR_BR8;//glc off
			  GPIOA -> BSRR	 = GPIO_BSRR_BS10; //gla on
			  TIM1 -> CCR1 = throttle;		//ghc on
			}
		else{
			  TIM1 -> CCR1 = 0;				//ghc off
			  TIM1 -> CCR4 = 0;				//gha off
			  GPIOA -> BSRR	= GPIO_BSRR_BR9;//glb off
			  GPIOA -> BSRR = GPIO_BSRR_BR10;//gla off
			  GPIOA -> BSRR	 = GPIO_BSRR_BR8; //glc on
			  TIM1 -> CCR2 = 0;		//ghb on
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
