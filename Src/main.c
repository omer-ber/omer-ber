
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
  * COPYRIGHT(c) 2020 STMicroelectronics
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

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t phy_to_dll_rx_bus = 0; // saves the data recived from the phy to phy and moves it to the dll_rx

uint8_t phy_to_dll_rx_bus_valid = 0; // checks if there is data to be transferd to the dll_rx from the phy layer

uint8_t dll_to_phy_tx_bus = 0; // saves the original data we want to transfer to the phy layer from the dll tx.

uint8_t dll_to_phy_tx_bus_valid = 0; // checks if theres data allready transferd from the dll to the [hy layer

uint8_t phy_tx_busy = 0; // checks if there is data being transmited at the moment.

int delay =0;

static uint8_t Rx_value = 0;
static uint8_t Tx_value = 0;
static uint8_t Rx_clock_value ;
static uint8_t Tx_clock_value ;
static uint16_t t_data =0;
uint8_t data=0;

uint8_t senders[] = {201,0,192,0,223};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void dll_TX()
{
	static int space=0;
	if (!phy_tx_busy && !dll_to_phy_tx_bus_valid)
	{
		if (delay == 0)
		{
			
			dll_to_phy_tx_bus = senders[space]; // takes diffrent data from the array in each run
			dll_to_phy_tx_bus_valid =1; 
			delay = 500; // input random number;
			space++;
		}
		else
			delay --;
	}
	space = space==5? 0:space; // reset the space if we finished all of the array
}	
	
void dll_RX()
{
	if ( phy_to_dll_rx_bus_valid)
	{
		t_data = phy_to_dll_rx_bus; // takes transferd data from the bus 
		phy_to_dll_rx_bus_valid =0; // reset the valid that means that we took the data
	}
	
}	


void phy_Tx()
{
	static uint16_t shifter =1; // for the masking in order to iso the bit
	static uint8_t transfer = 0; // var to the masked bit
	static uint8_t p_clock_val = 0; // saves the prev clock value in order to check that we are in rising edge
	static uint8_t temp=0;	// takes the data from the bus
	if(dll_to_phy_tx_bus_valid ){  // checkes if we are in a new byte
		temp = dll_to_phy_tx_bus;
		phy_tx_busy = 1;
		dll_to_phy_tx_bus_valid =0;
		HAL_TIM_Base_Start(&htim2); //turn on timer
		HAL_TIM_Base_Start_IT(&htim2);
	}
	Tx_clock_value = HAL_GPIO_ReadPin(Tx_clock_GPIO_Port, Tx_clock_Pin); // reads the current value of the clcok
	if (Tx_clock_value && (!p_clock_val)) // checks if we are in a rising edge 
	{
		if (shifter ==1) // first bit
		{
	
			transfer = ((dll_to_phy_tx_bus) &	(shifter));
			if (transfer == shifter)
			{
				HAL_GPIO_WritePin (Tx_data_GPIO_Port, Tx_data_Pin, GPIO_PIN_SET);	//inserts one to the data
				Tx_value =1;
			}	
			else
			{
				HAL_GPIO_WritePin (Tx_data_GPIO_Port, Tx_data_Pin, GPIO_PIN_RESET); // inserts zero to data 
				Tx_value =0;
			}
			shifter = shifter*2;		
		}
		else if ( shifter > 128) // after the last bit
		{
			HAL_TIM_Base_Stop(&htim2); //turn off timer
			HAL_TIM_Base_Stop_IT(&htim2); //turn off interupts
			HAL_GPIO_WritePin(Tx_clock_GPIO_Port, Tx_clock_Pin ,GPIO_PIN_RESET); //set clock to zero
			phy_tx_busy =0; // reset busy
			shifter =1; // reset the masker 
		}
		else
		{	
			transfer = ((dll_to_phy_tx_bus) &	(shifter)); // for the reset of the bits
			if (transfer == shifter){
				HAL_GPIO_WritePin (Tx_data_GPIO_Port, Tx_data_Pin, GPIO_PIN_SET);	
				Tx_value =1;
			}	
			else{
				HAL_GPIO_WritePin (Tx_data_GPIO_Port, Tx_data_Pin, GPIO_PIN_RESET);					
				Tx_value =0;
			}
			shifter = shifter*2;		
		}
	}
	p_clock_val = Tx_clock_value; // current clock into previos clock
}

void phy_Rx()
{
	
	static uint16_t s_counter =0; // counter for inserting the number 
	static uint8_t pr_clock_val=1; // prev clock
	Rx_clock_value = HAL_GPIO_ReadPin(Rx_clock_GPIO_Port, Rx_clock_Pin); // reads the current clock value
	if ( (!Rx_clock_value) && pr_clock_val) // checks that we are in falling edge
	{
		Rx_value = HAL_GPIO_ReadPin (Rx_data_GPIO_Port, Rx_data_Pin); // reads the data from the pin
		phy_to_dll_rx_bus += Rx_value << s_counter;	// moves the bit for the correct spot and inserts in to the bus
		s_counter ++;
	}
	pr_clock_val = Rx_clock_value;
	if (s_counter == 8){
		s_counter = 0;
		phy_to_dll_rx_bus_valid =1;
	}
}







void phy_layer()
{
	phy_Tx();
	phy_Rx();
	
}

void dll_layer()
{
	dll_TX();
	dll_RX();
}
 


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(Tx_clock_GPIO_Port, Tx_clock_Pin ,GPIO_PIN_RESET); //set clock to zero
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		dll_layer();
		phy_layer();	

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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 19999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Tx_clock_Pin|Tx_data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Tx_clock_Pin Tx_data_Pin */
  GPIO_InitStruct.Pin = Tx_clock_Pin|Tx_data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Rx_clock_Pin */
  GPIO_InitStruct.Pin = Rx_clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Rx_clock_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Rx_data_Pin */
  GPIO_InitStruct.Pin = Rx_data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Rx_data_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
