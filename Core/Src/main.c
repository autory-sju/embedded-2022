/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  * uart 6 global interupt, rx dma circular, tx dma normal for logger
  * uart 3 disable interupt and dma for data check
  * uart 2 disable interupt and dma for display
  * GPIO PC8 for logger relay
  * timer 11 activated
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h" // for printf
#include "stdlib.h"
#include "string.h"
#include "math.h"
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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
uint8_t rxBuffer[150];
const uint8_t End[] = {0xFF,0xFF,0xFF};



uint8_t rcvFlag = 0;
uint8_t sndFlag = 0;


char gps_long[11];
char gps_lat[10];
char gps_speed[6];


float vol; // for battery voltage
float vol_per; // for battery percent
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __cplusplus //for printf
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len) {
#else
int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
    if( HAL_UART_Transmit(&huart2, ptr, len, 0xFFFF) == HAL_OK ) return len; // printf for Display
    else return 0;
}

/*
void checkData() {
	for (int i = 0; i < 10; i++) {
		if ('0' > gps_long[i] || gps_long[i] > '9') {
			gps_long[0] = '0';
			gps_long[1] = '\0';

			break;
		}
		//printf("error count%d\n", i);
	}
	for (int i = 0; i < 9; i++) {
		if ('0' > gps_lat[i] || gps_lat[i] > '9') {
			gps_lat[0] = '0';
			gps_lat[1] = '\0';
			//printf("lat error\n");
			break;
		}
	}
	for (int i = 0; i < strlen(gps_speed); i++) {
		if ('0' > gps_speed[i] || gps_speed[i] > '9') {
			gps_speed[0] = '0';
			gps_speed[1] = '\0';
			break;
		}
	}
	//printf("BeforeCal gps_speed %s\n", gps_speed);
	// *3.6/1000=0.0036  //  97.2km/h = 27m/s = 27,000 mm/s -> 0.1m/s = 100 mm/s
	if (strlen(gps_speed) > 1) {
		int num = atoi(gps_speed) * 0.0036;
		if (num > 9) {
			gps_speed[0] = num / 10 + '0';
			gps_speed[1] = num % 10 + '0';
			gps_speed[2] = '\0';
		}
		else {
			gps_speed[0] = num + '0';
			gps_speed[1] = '\0';
		}
	}

}


void updateData(char* buffer) {
	int len = strlen(buffer);
	int j = 0;
	char* p;
	p = buffer;
	int colon_cnt = 0;
	int comma_cnt = 0;
	int got_lat = 0, got_long = 0, got_speed = 0; // ?���?? ?��?��?���?? 받았?��?��, ?��?�� 콤마 ?��치�? �??까워 ?��?��?���?? ?�� 받을까봐
	gps_long[0] = '0';
	gps_long[1] = '\0';
	gps_lat[0]= '0';
	gps_lat[1] = '\0';
	gps_speed[0]= '0';
	gps_speed[1] = '\0';

	//printf("updateData\n");

	for (p = buffer; p < buffer + len; p++) { // : ?���?? 찾음
		if (*p == ':') {
			colon_cnt++;
		}
		if (colon_cnt == 2) break; // *p�?? : ?���?? ?��?�� 주소?�� ?�� break
	}

	for (p; p < buffer + len; p++) { // ':' ?���?? ?��?�� ?��치�??�� ?��?��?�� ','?���?? 찾음
		if (colon_cnt < 2) break;
		if (*p == ',') comma_cnt++;
		if (comma_cnt == 1 && *(p + 10) == ',' && got_lat == 0) { // : 찾�? �???���???�� ?��?��?�� , �?? 첫번째이�?? 10�???�� ?��?���?? 콤마?���??
			for (int i = 0; i < 9; i++) { // 9�???�� �???��?��
				gps_lat[i] = *(p + i + 1);
				got_lat++;
			}


		}
		if (comma_cnt == 2 && *(p + 11) == ',' && got_long == 0) { // : 찾�? �???���???�� ?��?��?�� , �?? ?��번째?���?? 11�???�� ?��?���?? 콤마?���??
			for (int i = 0; i < 10; i++) { // 10�???�� �???��?��
				gps_long[i] = *(p + i + 1);
				got_long++;
			}


		}
		for (int i = 0; i < 5; i++) {
			if (comma_cnt == 7 && *(p + i+2) == ',' && got_speed == 0) {// 찾�? �???���???�� ?��?��?��','�?? 7번째 i+2�???�� ?��?�� 콤마?���??
				for (j = 0; j < i + 1; j++) {
					gps_speed[j] = *(p + j + 1);
				}
				gps_speed[j+1] = '\0';
				got_speed++;
				break;
			}
		}
	}
	checkData();
}
*/
float Cal_percent(float vol){
    if(vol>=57.4){
        return (float)(1.428*vol)+16;
    } else if(vol>=51.9) {
        return (float)6.91 * vol - 298.58;
    } else if(vol>=51.24){
        return (float)500/33*vol-7990/11;
    } else if(vol>=49){
        return (float)125/7*vol-865;
    } else if(vol>=46.2){
        return (float)5/14*vol-15/2;
    } else{
        return (float)45/14*vol-279/2;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint16_t timer_val; // timer value
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
  MX_USART6_UART_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart6, rxBuffer, 150); // loger recieve at buffer
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_SET); // loger reset relay on
  HAL_Delay(2000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,GPIO_PIN_RESET); // loger reset relay off
  HAL_Delay(2000);

  //HAL_TIM_Base_Start(&htim11); // timer 11 start

  //timer_val = __HAL_TIM_GET_COUNTER(&htim11);// Get current time (microseconds)
  float input = 1023.0;
  //char vol_str[4];
  //vol_str[3] = 'V';
  //char vol_per_str[4];
  //vol_per_str[3]='%';
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //printf("%d\n",timer_val);
	 //timer_val = __HAL_TIM_GET_COUNTER(&htim11);// - timer_val;
	  	  		  // If enough time has passed (1 second), toggle LED and get new timestamp
	 //if(timer_val>30000){
	//	 count_tim++;
	 //}
	  //if (count_tim>100000)
	  	  		    //  {
	  	  		        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); // LD2
	  	  		        HAL_ADC_Start(&hadc1);//AD변환 시작
	  	  		    	HAL_ADC_PollForConversion(&hadc1,100); //AD변환 완료될때까지 대기
	  	  		    	input = HAL_ADC_GetValue(&hadc1); //AD변환결과값을 받는다
	  	  		        float voltage = (input/1023)*3.3; // ADC to Voltage
	  	  		    	//printf("Input Voltage: %dV\n",voltage);
	  	  		    	vol = voltage/0.881198; // calculate max 3.3 to 3.7
	  	  		    	//printf("Voltage Sensor(before register): %dV\n",vol);
	  	  		    	vol = vol/0.03985; // calculate battery voltage
	  	  		        vol_per = Cal_percent(vol);
	  	  		    	//printf("Battery Voltage: %dV\nBattery Percent: %d%%\n",(int)round(vol),(int)round(vol_per));
	  	  		      if(vol_per<0||vol_per>100){
	  	  		    	  vol_per = 0;
	  	  		      }
	  	  		        printf("t4.txt=\"%d\%\"\n", (int)(round(vol_per)));
	  	  		      fflush (stdout);
	  	  		      HAL_UART_Transmit(&huart2,End, sizeof(End),10);
	  	  		      printf("t5.txt=\"%dV\"\n", (int)(round(vol)));
	  	  		  	  fflush (stdout);
	  	  		  	  HAL_UART_Transmit(&huart2,End, sizeof(End),10);
	  	  		  printf("Battery.val=\"%dV\"\n", (int)(round(vol_per)));
	  	  		  	  	  		  	  fflush (stdout);
	  	  		  	  	  		  	  HAL_UART_Transmit(&huart2,End, sizeof(End),10);
/*
	  	  		  	  	  		  	  vol_str[0]= ((int)vol)/100+48;
	  	  		  	  	  	vol_str[1]= ((int)vol)/10+48;
	  	  		  	  vol_str[2]= ((int)round(vol))/1+48;
	  	  		  	  vol_str[4] = '\0';
	  	  		  vol_per_str[0]= ((int)vol_per)/100+48;
	  	  		  	  	  		  	  	  	vol_per_str[1]= ((int)vol_per)/10+48;
	  	  		  	  	  		  	  vol_per_str[2]= ((int)round(vol_per))/1+48;
	  	  		  	  	  	vol_per_str[4] = '\0';

	  	  		  	    	HAL_UART_Transmit(&huart6,vol_str, 4,10);
	  	  		  	  HAL_UART_Transmit(&huart6,vol_per_str, 4,10);
*/
	  	  		  	 HAL_Delay(1000);
	  	  		  	 // count_tim = 0;

	  	  		        //timer_val = __HAL_TIM_GET_COUNTER(&htim11);
	  	  		     // }

/*
	  if(sndFlag)//
	  		  	      {
	  		  	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	  		  	        HAL_Delay(5);
	  		  	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	  		  	        sndFlag = 0;
	  		  	      }

	  		  	      if(rcvFlag)
	  		  	      {
	  		  	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  		  	        HAL_Delay(5);
	  		  	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  		  	        rcvFlag = 0;
	  		  	      }
*/
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoggerRelay_GPIO_Port, LoggerRelay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LoggerRelay_Pin */
  GPIO_InitStruct.Pin = LoggerRelay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoggerRelay_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)// Tx interupt
{
  sndFlag = 1;
  //933.11,-44.43,-300.29,-0.11,-0.81,-1.34,-48.15,55.35,32.55,29.56,50.78,25.43,19:04:37.500,375521219,1270743420,55976,37329,0,0,[speed]0,0,9999,241496000,1.001,
  //aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,imu_degC,humidity_%,degC,gps_Time,gps_Lat,gps_Long,gps_Alt,gps_AltMSL,gps_SIV,gps_FixType,gps_GroundSpeed,gps_Heading,gps_pDOP,gps_iTOW,output_Hz,
  /*
  printf("%s\n",rxBuffer);
  fflush (stdout);
  printf("//////////////////////////////////////////\n\n\n");
  //txLen = sizeof(txBuffer) - 1;
  //HAL_UART_Transmit(&huart3, txBuffer, txLen, 0xFFFF);
    */



      //HAL_UART_Transmit(&huart2,End, sizeof(End),10);
      //HAL_UART_Transmit(&huart3,Data, strlen(Data),10);
      //HAL_UART_Transmit(&huart3,"\n", 2,10);

//}

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)// Rx interupt (When Buffer is full)
{
  rcvFlag = 1;
  HAL_UART_Transmit_DMA(&huart6, rxBuffer, 300);
  updateData(rxBuffer);

  printf("t3.txt=\"%s,%s\"",gps_long, gps_lat);
  fflush (stdout);
  //HAL_UART_Transmit(&huart2,End, sizeof(End),10);
  printf("t0.txt=\"%s\"", gps_speed);
  fflush (stdout);
 // HAL_UART_Transmit(&huart2,End, sizeof(End),10);
}
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
