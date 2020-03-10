
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <string.h>

#include "stm32l475e_iot01_hsensor.h" //humidity
#include "stm32l475e_iot01_tsensor.h" //temperature
#include "stm32l475e_iot01_magneto.h" //magnetometer
#include "stm32l475e_iot01_psensor.h" //pressure
#include "stm32l475e_iot01_accelero.h" //accelerometer
#include "stm32l475e_iot01_gyro.h" //gyroscope
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//osThreadId defaultTaskHandle;

osThreadId defaultTaskHandle;
osThreadId usartThreadId;
osThreadId buttonThreadId;
osThreadId temperatureThreadId;
osThreadId humidityThreadId;
osThreadId magnetometerThreadId;
osThreadId accelerometerThreadId;
osThreadId gyroscopeThreadId;
osThreadId pressureThreadId;
osThreadId currentThreadId;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef handle;


//mag, temp,humidity, acceleration,gyro, pressure
enum sensor {MAGNETOMETER, TEMPERATURE, HUMIDITY, ACCELOROMETER, GYROSCOPE, PRESSURE} sensor_thread;
//enum sensor sensorChoice;
enum state {ON, OFF} usart_state;
enum state button_state;

osMutexId mutexId;
int timer = 0;
int userButtonPressed = 0;
int chooseThread = 0;
int isInitialized = 0;
char buffer[100];

/*sensor outputs*/
float tempHumPresOutput; //temperature, humidity, pressure 
float gyroOutput[3]; //gyroscope
int16_t accMagOutput[3]; //accelerometer, magnetometer 
float xyz[3];



int fputc(int ch, FILE *f) {
  while (HAL_OK != HAL_UART_Transmit(&handle, (uint8_t *) &ch, 1, 30000));
  return ch;
}
int fgetc(FILE *f) {
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&handle, (uint8_t *)&ch, 1, 30000));
  return ch;
}

void UART_init() {	
	GPIO_InitTypeDef gpio_init;
	
	handle.Instance = USART1;
	handle.Init.BaudRate = 115200;
  handle.Init.WordLength = UART_WORDLENGTH_8B;
  handle.Init.StopBits = UART_STOPBITS_1;
  handle.Init.Parity = UART_PARITY_NONE;
  handle.Init.Mode = UART_MODE_TX_RX;
  handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  handle.Init.OverSampling = UART_OVERSAMPLING_16;
  handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	
  gpio_init.Pin = GPIO_PIN_6;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &gpio_init);
	
	gpio_init.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &gpio_init);
	
	HAL_UART_Init(&handle);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void StartDefaultTask(void const * argument);
void temperatureThread(void const * argument);
void humidityThread(void const * argument);
void pressureThread(void const * argument);
void accelerationThread(void const * argument);
void magnetoThread(void const * argument);
void gyroscopeThread(void const * argument);
void buttonThread(void const * argument);
void usartTransmitThread(void const * str);

int fputc(int ch, FILE *f);
int fgetc(FILE *f);

/* Jessie code */
//void StartUsartTask(void const * argument);
//void StartSensorTask(void const * argument);
//void StartUserInputTask(void const * argument);

//void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
	MX_GPIO_Init();	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	osMutexDef(Mutex1);
	mutexId = osMutexCreate(osMutex(Mutex1));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	
	//osThreadDef(usartTask, StartUSARTTask, osPriorityNormal, 0, 128);
	//osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
	osThreadDef(userInputTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(userInputTask), NULL);
	

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */
static void MX_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/* USER CODE END 4 */

/* user made thread definitions */

//Temperature Thread
void temperatureThread(void const * argument){
	BSP_TSENSOR_Init();
	float temperature;
	char tempBuffer[18];
	for(;;) {
		//makes sure we have 100ms between each read
		osDelay(100);
		//waits forever for the mutex to become available
		osMutexWait(mutexId, osWaitForever);
		temperature = BSP_TSENSOR_ReadTemp();
		sprintf(tempBuffer, "Temperature (C): %.2f \n", temperature);
		//clears the string, releases mutex
		memset(buffer, 0, strlen(buffer));
		strcat(buffer, tempBuffer);
		osMutexRelease(mutexId);
	}
}

//humidity Thread
void humidityThread(void const * argument){
	BSP_HSENSOR_Init();
	float humidity;
	char humidityBuffer[20];
	for(;;) {
		//makes sure we have 100ms between each read
		osDelay(100);
		//waits forever for the mutex to become available
		osMutexWait(mutexId, osWaitForever);
		humidity = BSP_HSENSOR_ReadHumidity();
		sprintf(humidityBuffer, "Humidity (%%): %.2f \n", humidity);
		//clears the string, releases mutex
		memset(buffer, 0, strlen(buffer));
		strcat(buffer, humidityBuffer);
		osMutexRelease(mutexId);
	}
}

//Magnetometer Thread
void magnetometerThread(void const * argument) {
	BSP_MAGNETO_Init();
	BSP_MAGNETO_LowPower(1);
	int16_t xyz[3];
	char x[100];
	char y[20];
	char z[20];
	
	for(;;) {
		osDelay(100);
		osMutexWait(mutexId, osWaitForever);
		BSP_MAGNETO_GetXYZ(xyz);
		sprintf(x, "Field Intensity (gauss) x: %.3d", xyz[0]);
		sprintf(y, ", y: %.3d", xyz[1]);
		sprintf(z, ", z: %.3d \n", xyz[2]);
		memset(buffer, 0, strlen(buffer));
		strcat(buffer, x);
		strcat(buffer, y);
		strcat(buffer, z);
		osMutexRelease(mutexId);
	}
}

//Pressure Thread
void pressureThread(void const * argument){
	BSP_PSENSOR_Init();
	float pressure;
	char pressureBuffer[20];
	for(;;) {
		osDelay(100);
		osMutexWait(mutexId, osWaitForever);
		pressure = BSP_PSENSOR_ReadPressure();
		sprintf(pressureBuffer, "Pressure (kPa): %.2f \n", pressure/10);
		memset(buffer, 0, strlen(buffer));
		strcat(buffer, pressureBuffer);
		osMutexRelease(mutexId);
	}
}

//Acceleration Thread
void accelerometerThread(void const * argument) {
	BSP_ACCELERO_Init();
	BSP_ACCELERO_LowPower(1);
	int16_t xyz[3];
	char x[30];
	char y[20];
	char z[20];
	for(;;) {
		osDelay(100);
		osMutexWait(mutexId, osWaitForever);
		BSP_ACCELERO_AccGetXYZ(xyz);
		sprintf(x, "Acceleraion (mg) x: %.5d", xyz[0]);
		sprintf(y, ", y: %.5d", xyz[1]);
		sprintf(z, ", z: %.5d \n", xyz[2]);
		memset(buffer, 0, strlen(buffer));
		strcat(buffer, x);
		strcat(buffer, y);
		strcat(buffer, z);
		osMutexRelease(mutexId);
	}
}

//Gyroscope Thread
void gyroscopeThread( void const * argument) {
	BSP_GYRO_Init();
	BSP_GYRO_LowPower(1);
	//float xyz[3];
	char x[80];
	char y[20];
	char z[20];
	for(;;) {
		osDelay(100);
		osMutexWait(mutexId, osWaitForever);
		BSP_GYRO_GetXYZ(xyz);
		sprintf(x, "Rotation (degrees) x: %.1f", xyz[0]/10);
		sprintf(y, ", y: %.1f", xyz[1]/10);
		sprintf(z, ", z: %.1f \n" , xyz[2]/10);
		memset(buffer, 0, strlen(buffer));
		strcat(buffer, x);
		strcat(buffer, y);
		strcat(buffer, z);
		osMutexRelease(mutexId);
	}
}

void usartTransmitThread(void const *str) {
	UART_init();
	while(1){
		osMutexWait(mutexId, osWaitForever);
		HAL_UART_Transmit(&handle,(uint8_t *)buffer, strlen(buffer), 30000);
		osMutexRelease(mutexId);
		osDelay(100);
	}
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	button_state = OFF;
	usart_state = OFF;
	
	//Defining threads
	osThreadDef(gyroscopeTask, gyroscopeThread, osPriorityNormal, 0, 150);
	osThreadDef(temperatureTask, temperatureThread, osPriorityNormal, 0, 128);
	osThreadDef(humidityTask, humidityThread, osPriorityNormal, 0, 128);
	osThreadDef(pressureTask, pressureThread, osPriorityNormal, 0, 128);
	osThreadDef(magnetometerTask, magnetometerThread, osPriorityNormal, 0, 128);
	osThreadDef(accelerometerTask, accelerometerThread, osPriorityNormal, 0, 128);
	osThreadDef(usartTransmitTask, usartTransmitThread, osPriorityNormal, 0, 100);
	//sensor_thread = MAGNETOMETER;
	while(1){
		
		if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
			button_state = ON;
			while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){}
			button_state = OFF;
			if (timer >= 2000) {
				BSP_ACCELERO_DeInit();
				BSP_MAGNETO_DeInit();
				BSP_GYRO_DeInit();
				osThreadTerminate(currentThreadId);
				osThreadTerminate(usartThreadId);
				usart_state = OFF;
				sensor_thread = MAGNETOMETER;
				timer = 0;
			}
			else {
				if(sensor_thread == MAGNETOMETER) {
					if(usart_state == ON) {
						BSP_GYRO_DeInit();
						osThreadTerminate(gyroscopeThreadId);
					}
					else {
						usart_state = ON;
						usartThreadId = osThreadCreate(osThread(usartTransmitTask), NULL);
					}
					//start the magnometer thread
					magnetometerThreadId = osThreadCreate(osThread(magnetometerTask), NULL);
					currentThreadId = magnetometerThreadId;
					sensor_thread = TEMPERATURE;
				}
				else if(sensor_thread == TEMPERATURE) {
					BSP_MAGNETO_DeInit();
					osThreadTerminate(magnetometerThreadId);
					
					temperatureThreadId = osThreadCreate(osThread(temperatureTask), NULL);
					currentThreadId = temperatureThreadId;
					sensor_thread = HUMIDITY;
				}
				else if(sensor_thread == HUMIDITY) {
					osThreadTerminate(temperatureThreadId);
					BSP_ACCELERO_DeInit();
					
					humidityThreadId = osThreadCreate(osThread(humidityTask), NULL);
					currentThreadId = humidityThreadId;
					sensor_thread = ACCELOROMETER;
				}
				else if(sensor_thread == ACCELOROMETER) {
					osThreadTerminate(humidityThreadId);
					
					accelerometerThreadId = osThreadCreate(osThread(accelerometerTask), NULL);
					currentThreadId = accelerometerThreadId;
					sensor_thread = PRESSURE;
				}
				else if(sensor_thread == PRESSURE){
					//BSP_GYRO_DeInit();
					//BSP_ACCELERO_DeInit();
					osThreadTerminate(accelerometerThreadId);
					//osThreadTerminate(gyroscopeThreadId);
					
					pressureThreadId = osThreadCreate(osThread(pressureTask), NULL);
					currentThreadId = pressureThreadId;
					sensor_thread = GYROSCOPE;
				}
				else if(sensor_thread == GYROSCOPE) {
					osThreadTerminate(pressureThreadId);
					
					gyroscopeThreadId = osThreadCreate(osThread(gyroscopeTask), NULL);
					currentThreadId = gyroscopeThreadId;
					sensor_thread = MAGNETOMETER;
					}
			}
		}
	}
	
	
	/*
  //USER CODE BEGIN 5 
	UART_init();
	BSP_ACCELERO_Init();
	int16_t XYZ[3];
  //nfinite loop 
  for(;;)
  {
		osDelay(100);
		BSP_ACCELERO_AccGetXYZ(XYZ);
		printf("X = %d, Y = %d, Z = %d\n", XYZ[0], XYZ[1], XYZ[2]);
  }
  //USER CODE END 5 
	*/
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(button_state == ON){
		timer++; 
	}
	if(button_state == OFF){
		timer = 0; //timer reset
	}
	
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
