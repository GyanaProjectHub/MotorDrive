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

#define TIMEOUT_THRESHOLD 100000000
#define M_PI 3.14159265358979323846
#define INDEX 64

#define STARTUP 1
#define RUNNING 2
#define MEASURE 3
#define PAUSE 4
#define EXC_TIME 100
#define EXC_THROTTLE 85
#define STOP 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t counter1=0;
uint32_t counter2=0;


uint32_t prevTime = 0;
uint32_t currTime = 0;
float speed = 0;

int adc_val=0;
float throttle=0;

int state = STARTUP, pstate = STARTUP;

float angle=80;
float value1=0 , value2=0 , value3=0;
int sin_table1[INDEX];
int sin_table2[INDEX];
int sin_table3[INDEX];
int position = 0, position_captured = 0;
int dir=1;
int const_angle=40;
int sign=1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void spwm_commutation();
int catch_throttle();


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
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

 //

  if(dir==0)
  {
  	const_angle=50;
  	sign=1;
  }

  else
  	{
  	const_angle=19;
  	sign=-1;
  	}

for(int i=0 ; i< INDEX; i++)
{
	sin_table1[i]=4650*(sin(i*2*M_PI/64)+ sin(3*(i*2*M_PI/64))/6);
	sin_table2[i]=4650*(sin(i*2*M_PI/64 + sign*2*M_PI/3)+sin(3*(i*2*M_PI/64))/6);
	sin_table3[i]=4650*(sin(i*2*M_PI/64 - sign*2*M_PI/3)+sin(3*(i*2*M_PI/64))/6);
}



  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_ADC_Start(&hadc1);

 HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    //currTime = __HAL_TIM_GET_COUNTER(&htim2);
	   // if ((currTime - prevTime) > TIMEOUT_THRESHOLD)
	   // {
	    //  speed = 0;
	   // }

	   // value1 = sin_table1(counter1);
	    //value2 = sin_table2(counter1);
	    //value3 = sin_table3(counter1);
	 //HAL_ADC_Start(&hadc1) ;
	 //HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
	 //adc_val=HAL_ADC_GetValue(&hadc1);
	// throttle = 1;
/*
	  value1= 2000 + sin_table1[counter1];
	  value2= 2000 + sin_table2[counter1];
	  value3= 2000 + sin_table3[counter1];
	  switch(state)
		 	 	  {
		 	       case STARTUP:
		              TIM1->CCR1 = value1;
		              TIM1->CCR2 = value2;
		              TIM1->CCR3 = value3;
		 	    	   break;

		 	       case RUNNING:
			              TIM1->CCR1 = value1;
			              TIM1->CCR2 = value2;
			              TIM1->CCR3 = value3;
		              break;


		 		   default:
		 		  	  state = STARTUP;
		 	 	  }
*/
//throttle=catch_throttle();

spwm_commutation();
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	throttle=catch_throttle();

}







int catch_throttle()
{

	int c_throttle = 0;
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adc_val = HAL_ADC_GetValue(&hadc1);

	if (adc_val < 1300)
	{
		c_throttle = 40;
	}
	else if (adc_val <= 2900)
	{
		c_throttle = 40 + ((adc_val - 1300)*(600-40))/(2900-1300);
	}
	else
	{
		c_throttle = 600;
	}
	return c_throttle;

}



void spwm_commutation()
{

	  value1= 2000 + throttle*sin_table1[counter1]/1200;
	  value2= 2000 + throttle*sin_table2[counter1]/1200;
	  value3= 2000 + throttle*sin_table3[counter1]/1200;
	  switch(state)
		 	 	  {
		 	       case STARTUP:
		              TIM1->CCR1 = value1;
		              TIM1->CCR2 = value2;
		              TIM1->CCR3 = value3;
		 	    	   break;

		 	       case RUNNING:
			              TIM1->CCR1 = value1;
			              TIM1->CCR2 = value2;
			              TIM1->CCR3 = value3;
		              break;


		 		   default:
		 		  	  state = STARTUP;
		 	 	  }



}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

      if(GPIO_Pin == GPIO_PIN_6)
      {

		 if(counter1>=0 && counter1< 63)
		 {
			 counter1++;

		 }

		 else counter1=0;


	 position = position + 1;
	}



	if(GPIO_Pin == GPIO_PIN_1)
	{

		if(pstate == STARTUP)
		{
			counter1 = 0 + const_angle;
			state = RUNNING;
		}
		position_captured = position;
		if(position_captured >= 508 && position_captured <= 518)
		{
		     counter1 = 0 + const_angle;

		}
		position = 0;

  counter2++;

	}



}



/*

		//if(GPIO_Pin == GPIO_PIN_6)
	//	{


		    currTime = __HAL_TIM_GET_COUNTER(&htim2); // Get the current time from the timer

		    if (prevTime != 0)
		    {
		      uint32_t timeInterval = (currTime > prevTime) ? (currTime - prevTime) : ((0xFFFF - prevTime) + currTime + 1);
		      speed = (float)HAL_RCC_GetPCLK1Freq() / (htim2.Instance->PSC + 1) / timeInterval; // Calculate speed
		    }

		    prevTime = currTime;

		    */
	//	}

	//	t1 =(GPIOA->IDR & GPIO_PIN_5)? (1):(0); //PA5
	//	t2 =(GPIOA->IDR & GPIO_PIN_6)? (1):(0);

	//	state = t1<<1 |t2<<0;
   /*

		if(prevstate == 2)
		{
			if(state == 3) counter1++;
			else if(state==0) counter1--;
		}

		else if(prevstate == 3)
		{
			if(state == 1) counter1++;
			else if(state==2) counter1--;
		}

		else if(prevstate == 1)
		{
			if(state == 0) counter1++;
			else if(state==3) counter1--;
		}

		else if(prevstate == 0)
		{
			if(state == 2) counter1++;
			else if(state==1) counter1--;
		}

		prevstate=state;
*/






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
