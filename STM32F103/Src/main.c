
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"
#include "controle.h"

/* USER CODE BEGIN Includes */
#define SUM ARRAY2D_SUM_REVERSE
#define SUB ARRAY2D_SUB_REVERSE
#define MUL ARRAY2D_MUL_REVERSE
#define GAIN ARRAY2D_GAIN_REVERSE
#define GAINSUM ARRAY2D_GAINSUM_REVERSE
#define COPY ARRAY2D_COPY_REVERSE
#define ZERO ARRAY2D_ZERO_REVERSE
#define INT ARRAY2D_GAINSUM_REVERSE
#define MULSUM ARRAY2D_MULSUM_REVERSE
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ADC_BUFFER_LEN 1

#define ENTRADA_10 1241
#define ENTRADA_15 1861

typedef enum estado{ALTO, BAIXO}estado_t;

volatile uint32_t ADC_buffer[ADC_BUFFER_LEN];
volatile uint32_t valor_ADC[ADC_BUFFER_LEN];
volatile estado_t flag = BAIXO;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile const float Ts = 1/2000.0f;

volatile const float A[2][2] = {{96.9795, 9.0109},
		   	   	   	   	   	   	{-16.1789e3 , -666.666666666667}};

volatile const float B[2][1] = {{11.9331e-3},
		   	   	   	   	   	   	{-787.5538e-3}};

volatile const float C[3][2] = {{0, 0},
		   	   	   	   	   	    {1, 0},
								{0, 1}};

volatile const float D[3][1] = {{1},
		   	   	   	   	   	    {-17.9880e-6},
								{1.6179e-3}};

volatile const float F[2][1] = {{0},
		   	   	   	   	   	    {66.6670e-3}};

volatile const float K[1][3] = {{10.9836, 467.2270e3, 624.9290}};

volatile const float Ki = 1.3924e3;

void controlador(){

	volatile uint32_t ADCValue;
	volatile static uint32_t pwm_u;
	volatile static float y, r = 1.0;
	volatile static float u = 1.0, xi = 0, xi_, ki_xi, k_x[1][1], eta[2][1], eta_[2][1], x[3][1];

	//HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_SET);

	ARRAY2D_INDEX_INIT();
	HAL_ADC_Start(&hadc1);
	ADCValue = HAL_ADC_GetValue(&hadc1);

	y = (float)(ADCValue * 8.056641e-4f);

	if(flag == ALTO)
		r = 1.5;
	else if(flag == BAIXO)
		r = 1.0;

	/* OBSERVADOR */
	// eta_ = F*u
	GAIN(F, u, eta_);
	// eta_ += B*y
	GAINSUM(B, y, eta_);
	// eta_ += A*eta
	MULSUM(A, eta, eta_);

	// eta = INT(eta_)
	INT(eta_, Ts, eta);

	// x = eta*C
	MUL(C, eta, x);
	// x += y*D
	GAINSUM(D, y, x);

	/* CONTROLADOR */
	//xi_ = r - y
	xi_ = r - y;
	// xi = INT(xi_)
	xi = xi + Ts*xi_;
	// ki_xi = Ki*xi
	ki_xi = Ki*xi;

	// k_x = K*x
	MUL(K, x, k_x);

	// u = ki_xi - k_x
	u = ki_xi - k_x[0][0];

	if(u < 0)
		u = 0;
	if(u > 3.3)
		u = 3.3;

/*
	if(flag == ALTO)
		u = 1.5;
	else if(flag == BAIXO)
		u = 1.0;
*/

	pwm_u = (uint32_t)(302.7272f*u);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_u);

	//HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);

}

extern void initialise_monitor_handles(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//initialise_monitor_handles();
	//printf("teste\n");
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //printf("b \n");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);

  //printf("c \n");
  //HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_buffer,ADC_BUFFER_LEN);
  HAL_ADC_Start_IT(&hadc1);


  //printf("a \n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 333);
  while (1)
  {
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_SET);
	  flag = ALTO;

	  HAL_Delay(500);
	  HAL_GPIO_WritePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin, GPIO_PIN_RESET);
	  flag = BAIXO;
	//printf("%d\n", ADC_buffer[0]);
	//printf("a \n");

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		/*
		static uint32_t i;

		//HAL_GPIO_TogglePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin);
		valor_ADC[i]=ADC_buffer[i];

		i++;
		if(i >= ADC_BUFFER_LEN)
			i=0;
	*/
	}

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == htim2.Instance)
	{
	  //HAL_GPIO_TogglePin(PIN_TEST_GPIO_Port, PIN_TEST_Pin);
	  controlador();
	}

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
