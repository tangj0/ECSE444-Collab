#include "main.h"
#include "stm32l4xx_hal.h"

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
//static void HAL_ADC_Init(void);
int UART_Print_String(UART_HandleTypeDef* huart, char* array_p, int array_len);
int ADCTemp0, ADCTemp;

int main(void)
{
	char ch[5] = {'j','o','b','s','\n'};
	char tempArr[25] = {'T', 'e', 'm', 'p', 'e', 'r', 'a', 't', 'u', 'r', 'e', ' ', '=', ' ', '3', '0', ' ', 'C', '\n'};
	char transmit[2] = {'a', 'y'};
	char adc_val;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
	
	//initialize temperature channel 
	//MX_ADC_Init();
	HAL_ADC_Start(&hadc1);
	
  /* Infinite loop */
  while (1)
  {

		
		//code that was given to us
		
		//HAL_UART_Transmit(&huart1, (uint8_t *)&ch[0], 5, 30000); //P1037
		
		//the first pre part of the code. basically, the revieve writes to the element of the array, and then we check to see
		//if the letter we typed matches what's been written there. transmit then prints the letter y. The delays are weird,
		//and if you push x too many times, it overflows and you need to hit the reset button on the board.
		
		//HAL_UART_Receive(&huart1, (uint8_t *)&transmit[0], 1, 30000);
		
//		if (transmit[0] == 'x') {
//			HAL_UART_Transmit(&huart1, (uint8_t *)&transmit[1], 1, 30000);
//		}
		
		
		//first section
		//HAL_Delay(100);
		//UART_Print_String(&huart1, &ch[0], 5); 
		
		
		
		//second section
		HAL_Delay(1000);
		HAL_ADC_PollForConversion(&hadc1, 30000); //P107, ADC = Analog to Digital Converter
		ADCTemp0 = HAL_ADC_GetValue(&hadc1); //update ADCTemp with current processor core temperature
		// 3300 = analog reference voltage = 3.3V; ADCTemp = value of digital data to convert; ADC resolution
		ADCTemp = __HAL_ADC_CALC_TEMPERATURE(3300, ADCTemp0, LL_ADC_RESOLUTION_10B); //P130
	
		if (ADCTemp < 10) { //convert currTemp into 2 chars
			tempArr[14] = '0';
			tempArr[15] = (char)ADCTemp;
		}
		else {
			tempArr[14] = (char)ADCTemp/10; //int division throws away remainder
			tempArr[15] = (char)ADCTemp%10; //remainder
		}


		UART_Print_String(&huart1, &tempArr[0], 19);
		
  }
}

//UART = Universal Serial Receiver/Transmitter 
int UART_Print_String(UART_HandleTypeDef* huart, char* array_p, int array_len) {
	int i;
	for (i = 0; i< array_len; i++) { //P1037 for HAL_UART_Transmit
		if (!HAL_UART_Transmit(huart, (uint8_t *)array_p, array_len, 30000) && !HAL_UART_Receive(&huart1, (uint8_t *)array_p, 1, 30000)){
			return 0;
		}
	}
	return 1;
}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 32;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000); //=1ms

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); //enable interrupt
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
}


void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
}



#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
}
#endif /* USE_FULL_ASSERT */
