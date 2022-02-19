/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
uint8_t array_states[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void initial_system_state(void);
void normal_state(uint8_t);
uint8_t manager_auth(void);
void direction(uint8_t);//O-to open;C- to close
void brake(uint8_t);//1- to brake;0- to release brake
void move_actuator(uint8_t);// L- to lock; U-to unlock;S-to stop
uint8_t actuator_state(void);// returns high when locked
uint8_t door_state(void);// returns high when closed
uint8_t select_system(void);//returns 1-normal operation;return 2-wind sensor master
uint8_t person_present(uint8_t);//input-F-to check front sensor;B-to check back sensor --> returns 1-person present;0-no person
uint8_t wind_sensor_state(void);//returns 1-wind speed high;returns 2-wind speed low
void emergency_close(void);
void remained_open(void);
uint8_t countstates(uint8_t []);
void move_motor(uint8_t,uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TOTAL_PULSES 2015
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
  MX_USB_PCD_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
initial_system_state();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GPIO_ReadPin(GPIOD, remained_open_Pin))
	  {
       remained_open();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t select_switch =select_system();           // Selection between motion sensor and wind sensor

	 switch(select_switch)
	 {
	 case 1:

		 	 	 normal_state(0);
		 		  break;

	 case 2:
		 if(wind_sensor_state())
		 {
			 if(!manager_auth()){
			 emergency_close();
			 while(!manager_auth());
			 }
		 }
		 normal_state(1);

		  break;
   }
  }
}
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, direction_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(brake_GPIO_Port, brake_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, linear_actuator1_Pin|linear_actuator2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : manager_button_Pin select_switch_Pin */
  GPIO_InitStruct.Pin = manager_button_Pin|select_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : direction_Pin PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = direction_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : brake_Pin */
  GPIO_InitStruct.Pin = brake_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(brake_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MISOA7_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : proximity_sensor_Pin */
  GPIO_InitStruct.Pin = proximity_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(proximity_sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : limit_switch_Pin motion_sensor_Pin wind_sensor_Pin */
  GPIO_InitStruct.Pin = limit_switch_Pin|motion_sensor_Pin|wind_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : remained_open_Pin PD6 */
  GPIO_InitStruct.Pin = remained_open_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : closing_motion_sensor_Pin */
  GPIO_InitStruct.Pin = closing_motion_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(closing_motion_sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : linear_actuator1_Pin linear_actuator2_Pin PC8 PC9 */
  GPIO_InitStruct.Pin = linear_actuator1_Pin|linear_actuator2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint8_t person_present(uint8_t side)
{

	HAL_Delay(20);
	if(side == 'F')
		return HAL_GPIO_ReadPin(GPIOB, motion_sensor_Pin);
	else if(side == 'B')
		return !HAL_GPIO_ReadPin(GPIOD, closing_motion_sensor_Pin);
}




void move_actuator(uint8_t state){
	if(state == 'L'){
		HAL_GPIO_WritePin(GPIOC, linear_actuator1_Pin, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOC, linear_actuator2_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
	}
	else if(state == 'U'){
		HAL_GPIO_WritePin(GPIOC, linear_actuator2_Pin, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOC, linear_actuator1_Pin, GPIO_PIN_SET);
		HAL_Delay(50);


	}
	else if(state == 'S'){
		HAL_GPIO_WritePin(GPIOC, linear_actuator2_Pin, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOC, linear_actuator1_Pin, GPIO_PIN_RESET);
		HAL_Delay(50);
	}

}
uint8_t actuator_state(void){

	HAL_Delay(20);
	return HAL_GPIO_ReadPin(GPIOB, proximity_sensor_Pin);

}
uint8_t door_state(void){


	HAL_Delay(20);
	return HAL_GPIO_ReadPin(GPIOB, limit_switch_Pin);

}
void direction(uint8_t dir){
	if(dir == 'C')
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	else if(dir == 'O')
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET );

}

void brake(uint8_t state){

	if(state)
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);


}
uint8_t wind_sensor_state(void){

	HAL_Delay(10);
	return HAL_GPIO_ReadPin(GPIOB, wind_sensor_Pin);


}


void initial_system_state(void)
{
	  brake(1);  // Break while door opening
	  HAL_Delay(50);

	 direction('C');
	  HAL_Delay(20);
	  brake(0);
	  HAL_Delay(20);

   	  while(!door_state()){
		HAL_Delay(10);
	  }
	brake(1);
	HAL_Delay(20);
	move_actuator('L');
	while(!actuator_state()){
		HAL_Delay(10);
		}
	move_actuator('S');


}

uint8_t select_system(void){
	HAL_Delay(10);
	uint8_t system_state = HAL_GPIO_ReadPin(GPIOA, select_switch_Pin);

	if(system_state)
		return 2;
	else
		return 1;

}



void normal_state(uint8_t override)
{
 	while(!HAL_GPIO_ReadPin(GPIOD, remained_open_Pin))
	{

	if(person_present('F') && (!HAL_GPIO_ReadPin (GPIOD, GPIO_PIN_6)))
			 	 	 {
			 	 		 if(actuator_state())
			 	 		 {
			 	 			 move_actuator('U');
 			 	 			 while(actuator_state())
			 	 			 {
			 	 				 HAL_Delay(20);
			 	 			 }
			 	 			 move_actuator('S');
			 	 		 }
			 	 		/* direction('O');
			 	 		 brake(0);
			 	 		 HAL_Delay(17000);
  			 	 		 brake(1);*/
			 	 		 move_motor('O',90);
			 	 		 HAL_Delay(5000);
			 	 		 direction('C');
			 	 		 brake(0);
			 	 		 while(!door_state())
			 	 		 {

			 	 			 while(person_present('B'))
			 	 			 {
			 	 				 direction('O');
			 	 				 brake(0);
			 	 				 HAL_Delay(2000);
			 	 				 if(!person_present('B'))
			 	 				 {

			 	 				 }
			 	 				//HAL_Delay(200);

			 	 				HAL_Delay(200); }

			 	 			 direction('C');
			 	 		 }
			 	 		 brake(1);
			 	 		 if(door_state() && (!override)){

			 	 			 move_actuator('L');
			 	 			 while(!actuator_state()){
			 	 				 HAL_Delay(20);
			 	 			 }
 			 	 		move_actuator('S');

			 	 		 }
			 	 	 }
	else
		{
			if(HAL_GPIO_ReadPin (GPIOD, GPIO_PIN_6))
			{
			if(person_present('B'))
			{
			 if(actuator_state())
				{
				 move_actuator('U');
				 while(actuator_state())
				  {
					HAL_Delay(20);
				  }
				move_actuator('S');
			     }
			 direction('O');
			 brake(0);
			 HAL_Delay(15000);
			 brake(1);
			 HAL_Delay(5000);
			 direction('C');
			 brake(0);
			 while(!door_state())
				{
				  HAL_Delay(50);
				}
			 brake(1);
			 if(door_state() && (!override)){

				move_actuator('L');
				while(!actuator_state())
				{
				    HAL_Delay(20);
				}
				move_actuator('S');

			}
		}

		}
		}



}
}
void emergency_close(void){
	 if(!door_state()){

					 if(actuator_state())
					 {
						 move_actuator('U');
						 while(actuator_state()){HAL_Delay(20);}
						 move_actuator('S');

					 }
					 direction('C');
					 brake(0);
					 while(!door_state()){HAL_Delay(20);}
					 brake(1);
					 move_actuator('L');
					 while(!actuator_state()){HAL_Delay(20);}
					 move_actuator('S');


				 }

}
void remained_open(void)
{
	if(HAL_GPIO_ReadPin(GPIOD, remained_open_Pin))
		  {
			  if(actuator_state())
			  	 {
			  		move_actuator('U');
			  		while(actuator_state())
			  		 {
			  			 HAL_Delay(20);
			  		  }
			  		move_actuator('S');
			  	  }
			  	brake(0);
			  	direction('O');
			  	HAL_Delay(17000);
			  	brake(1);
			  	while(HAL_GPIO_ReadPin(GPIOD, remained_open_Pin))
			  	{
			  		HAL_Delay(50);
			  	}
			  	 direction('C');
			     brake(0);
			     while(!door_state())
			     {
			    	 HAL_Delay(50);
			     }
			     brake(1);
			     move_actuator('L');
			     while(!actuator_state()){
			     		HAL_Delay(20);
			     		}
			     move_actuator('S');
		  }
}
uint8_t manager_auth(void){


	HAL_Delay(10);
	return HAL_GPIO_ReadPin(GPIOA, manager_button_Pin);
}
uint8_t countstates(uint8_t states[]){
	uint8_t countpos=0;
	for(int i=0;i< (sizeof(states)/sizeof(states[0]));i++){
		if(states[i] < 1)
			{countpos=0;
			break;}
		else
			countpos =1;
		}
	return countpos;


}
void move_motor(uint8_t dir,uint8_t degree){
	char uart_buf[50];
    int uart_buf_len;
	uint32_t rising_flag = 0;
	uint32_t count= 0;
	uint32_t pulses=0;
	pulses = (degree*TOTAL_PULSES)/360;
	direction(dir);
	brake(0);

while(count < pulses)
{
    if(!rising_flag){

    	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5)){

    		count++;
    		rising_flag= 1;
    	}
    }
    else
    {
    	if(!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5)){

    		rising_flag = 0;
    	}

    }

	//for(uint32_t i=0;i<10000;i++);
    uart_buf_len = sprintf(uart_buf,"%lu count\r\n", count);
    HAL_UART_Transmit(&huart1,&uart_buf, uart_buf_len+1, 100);

}

brake(1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
