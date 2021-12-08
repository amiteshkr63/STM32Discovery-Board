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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MY_FLASH.h"
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
//MT29F1G01AAADD instructions
const uint8_t WRITE_ENABLE=0x06;
const uint8_t READ_ENABLE=0x13;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int ECC_enable();
uint8_t Sensor_Data_buff[300];         //I am assuming in this i am storing Sensor_data_buff which i will write to SPI NAND FLASH
int latest_Nand_flash_addr=0x0;  //I am assuming in this i am storing address of SPI NAND FLASH in which i am about write Sensor Data
int timer_val;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
uint8_t count=0;
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
  MX_SPI2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */


  //Set Sector Adress
  MY_FLASH_SetSectorAddrs(11, 0x080E0000);

  //Here i am initializing Sensor_data with some value
  int j=0;
  for(int i=0;i<300;i++)
  {
	Sensor_Data_buff[i]=j++;
  }

  //Running BAD BLOCK DETECTION MECHANISM WITH SKIP BAD BLOCK MECHANISM
	uint8_t check_data;
	int starting_addr=0x0;
	int last_addr=0x7ff;
	int current_addr;
	static bad_block_addr_even[100];
	static uint8_t i=0;
	HAL_SPI_Transmit(&hspi2, (uint8_t*)READ_ENABLE,1,10);
	for(current_addr=starting_addr;current_addr<=last_addr;current_addr++)
	{
		HAL_SPI_Receive(&hspi2, (int*)current_addr,1,10);

		HAL_SPI_Receive(&hspi2, (int*)check_data,1,10);
		if (check_data!=0xff)
			{
				//Bad Block detected
			bad_block_addr_even[i]=current_addr;// Thought Process:
												//i am storing bad blocks at even place of bad_block_addr_even[i]
												//i am storing very next good blocks after bad blocks at odd place of bad_block_addr_even[i]
			while(check_data!=0xff)
			{
				current_addr++;
				HAL_SPI_Receive(&hspi2, (int*)current_addr,1,10);

				HAL_SPI_Receive(&hspi2, (int*)check_data,1,10);

			}
			if (check_data==0xff)
			{
				//Good block detected
				bad_block_addr_even[i+1]=current_addr;
			}
			}
		i+=2;
	}
	//END--Running BAD BLOCK DETECTION MECHANISM WITH SKIP BAD BLOCK MECHANISM

  //start timer
  HAL_TIM_Base_Start(&htim5);
  //current time
 timer_val=__HAL_TIM_GET_COUNTER(&htim5);

 //Mechanism to detect device run before or it is running for first time
	//read flash
	MY_FLASH_ReadN(0, latest_Nand_flash_addr, 1, DATA_TYPE_32);
if(latest_Nand_flash_addr==0xffffffff)
{
	  latest_Nand_flash_addr=0x0;
}

	  //END--Mechanism to detect system runned before or it is running first time
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //comparing latest_Nand_flash_addr with Bad block addresses so that we can skip those addresses
		  if (latest_Nand_flash_addr==last_addr)
			  latest_Nand_flash_addr=starting_addr;
	  if((latest_Nand_flash_addr==bad_block_addr_even[count]) ||(count<=i))//initially count=0
	  {
		  latest_Nand_flash_addr=bad_block_addr_even[count+1];
		  count+=2;
	  }
	  if(__HAL_TIM_GET_COUNTER(&htim5)-timer_val>=200)
	  {
		  //Enabling WRITE for MT29F1G01AAADD SPI NAND FLASH and i am not using #CHIP SELECT# here.
		  //So, i directly give WRITE command to MT29F1G01AAADD SPI NAND FLASH

		  HAL_SPI_Transmit(&hspi2, (uint8_t*)WRITE_ENABLE,1,10); //1-1 byte we sending
		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  //10- 10msec means a timeout here

		  ECC_enable(); //I am enabling ECC to easy access of user data blocks

		  HAL_SPI_Transmit(&hspi2, (uint8_t*)&latest_Nand_flash_addr,1,10);//First i am providing the address of NAND FLASH
		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  //where i am going to WRITE

		  HAL_SPI_Transmit(&hspi2, (uint8_t*)Sensor_Data_buff,300, 100);//Then, i am writing the sensor data to given address of
		  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //NAND FLASH

		  MY_FLASH_WriteN(0, latest_Nand_flash_addr, 1, DATA_TYPE_32);//Updating the STM32 non-volatile memory aadress from latest_Nand_flash_addr

		  latest_Nand_flash_addr+=0x12c; //0x12c=decimal 300
		  	  	  	  	  	  	  	  	  	//ReInitializing the Nand_flash_addr by adding 300 to upper address bcz we write data for 300Bytes
		  	  	  	  	  	  	  	  	  //in NAND_FLASH


	  }
    /* USER CODE END WHILE */
//incrementing latest_Nand_flash_addr
	  latest_Nand_flash_addr++;
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

int ECC_enable()
{
	const uint8_t SET_features=0x1f;
	const uint8_t OTP_features=0xb0;
	const uint8_t ECC_Enable=1<<4;
	HAL_SPI_Transmit(&hspi2, (uint8_t*)SET_features,1,10);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)OTP_features,1,10);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)ECC_Enable,1,10);


}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
