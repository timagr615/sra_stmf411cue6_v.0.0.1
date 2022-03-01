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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "ili9488.h"
#include "ili9488_gfx.h"
#include "MPU9250.h"
#include "micros.h"
#include "gps.h"
#include "utils.h"
#include "fusion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_REFERENCE_VOLTAGE                                           1.21
#define ADC_MAX                                                         0xFFF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

float mcuVoltage = 0;
float batteryVoltage = 0;
uint16_t adcData[2];

MPU9250_t mpu;
Model model = {CTRV, 6, 5};
EKFParams params = {5.0, 3.0, 2.0, 0.5, 0.3, 2.5, 0.2, 0.5, 0.8, 0.1};
Fusion fusion;

volatile uint8_t SPI1_TX_completed_flag = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern char SDPath[4];


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	SPI1_TX_completed_flag = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t ii = 0, ret;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(5000);
  printf("AFTER SysCLOCK INIT \n\r");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_SPI1_Init();

  //MX_USB_OTG_FS_PCD_Init();
  //MX_SDIO_SD_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, 2);
  DWT_Init();

  //printf("AFTER dwt INIT \n\r");

  printf("AFTER INIT \n\r");
  /*FATFS fileSystem;
  FIL testFile;
  uint8_t testBuffer[16] = "SD write success";
  UINT testBytes;
  FRESULT res;
  res = f_mount(&fileSystem, SDPath, 1);
  if(res == FR_OK)
    {
      uint8_t path[13] = "testfile.txt";
      path[12] = '\0';
      res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
      printf("OPEN FILE %u", res);
      HAL_Delay(3000);
      res = f_write(&testFile, testBuffer, 16, &testBytes);
      printf("WRITE to FILE %u", res);
      HAL_Delay(3000);
      res = f_close(&testFile);
      printf("CLOSE FILE %u",res);
      HAL_Delay(3000);
    }
  else
  {
	  printf("ERROR in mount filesystem %u", res);
  }*/


  NEO6_Init(&GpsState, &huart2);
  uint32_t Timer = HAL_GetTick();
  uint32_t Timer1 = HAL_GetTick();
  uint16_t FusionTimer;



  //////////////////////////// INIT DISPLAY ////////////////////////////
  //__HAL_SPI_ENABLE(DISP_SPI_PTR);
  //DISP_CS_UNSELECT;
  HAL_Delay(3000);


  ILI9341_Init(); // инициализация дисплея
  ILI9341_Fill_Screen(WHITE);


  //////////////////////// INIT MPU /////////////////////////
  for(ii=1; ii<128; ii++)
      {
          ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(ii<<1), 3, 5);
          if (ret != HAL_OK)
          {
              printf(" ");
          }
          else if(ret == HAL_OK)
          {
              printf("0x%X", ii);
              //HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), 10000);
          }
      }
  HAL_Delay(2000);
  if (!(setupMPU(&mpu, MPU9250_ADDRESS)==1))
  {  // change to your own address
	  char badmpu[] = "Check MPU\n\r";
   	  	  while (1)
   	  	  {
   	  		  ILI9341_DrawText("CHECK MPU", FONT4, 100, 150, BLACK, WHITE);
   	  		  printf("Check MPU\n\r");
   	  		  //HAL_UART_Transmit(&huart1, badmpu, strlen((char *)badmpu), 0xFFFF);
   	  		  HAL_Delay(5000);
          }
  }
  printf("MPU OK");
  setMPUSettings(&mpu);
  //calibrate(&mpu);
  //HAL_Delay(10000);


  ILI9341_Fill_Screen(WHITE);
  ILI9341_DrawText("Waiting for gps.", FONT4, 20, 35, BLACK, WHITE);
  /*NEO6_FirstPoint(&GpsState);
  InitFusion(&fusion, &model, &params);

  uint8_t Message[64];
  uint8_t MessageLength;

  float heading, pitch, roll;
  float la, ax, ay, az, gx, gy, gz;

  int i = 0;
  uint32_t prev_ms = 0;
  uint8_t str[100];
  uint8_t head[100];
  uint8_t rl[100];
  uint8_t ptc[100];
  uint8_t st0[100];
  uint8_t st1[100];
  uint8_t st2[100];
  uint8_t st3[100];

  float heading30[30] = {0.0};
  float avgMean = 0.0;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ILI9341_Fill_Screen(WHITE);
  ILI9341_Draw_Rectangle(0, 290, 480, 3, BLACK);
  ILI9341_DrawText("Heading  ", FONT4, 10, 0, BLACK, WHITE);
  HAL_Delay(1000);
  FusionTimer = HAL_GetTick();/*
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    mcuVoltage = ADC_MAX * ADC_REFERENCE_VOLTAGE / adcData[0];
	    batteryVoltage = 2 * adcData[1] * mcuVoltage / ADC_MAX;
	    if((HAL_GetTick() - Timer) > 500){
	    	 if (HAL_GPIO_ReadPin(GPIOA, BTN2_Pin) == GPIO_PIN_SET)
	    		  	  {
	    		  		  printf("Pressed\n\r");
	    		  	  }
	    		  	  else
	    		  	  {
	    		  		  printf("Released\n\r");
	    		  	  }
	    	 printf("Battery volts %.2f \n\r", batteryVoltage);
	    	 printf("mcuVoltage %.2f \n\r", mcuVoltage);
	    	 Timer = HAL_GetTick();
	    }


	  /*if (updateMPU(&mpu)==1)
	  {
		  heading = getYaw(&mpu);
		  roll = getRoll(&mpu);
		  pitch = getPitch(&mpu);
		  ax = getEarthAccX(&mpu);
		  ay = getEarthAccY(&mpu);
		  az = getEarthAccZ(&mpu);
		  la = getLonAcc(&mpu);

		  gx = getEarthGyroX(&mpu);
		  gy = getEarthGyroY(&mpu);
		  gz = getEarthGyroZ(&mpu);
	  }

	  NEO6_Task(&GpsState);
	  FusionTimer = HAL_GetTick() - FusionTimer;
	  processFusion(&fusion, &model, GpsState.Latitude, GpsState.Longitude, GpsState.Altitude, GpsState.SpeedKilometers/3.6,
			  GpsState.Course*DEG2RAD, heading*DEG2RAD, gz, la, GpsState.Hdop, FusionTimer);
	  FusionTimer = HAL_GetTick();
	  if((HAL_GetTick() - Timer) > 500)
	  {
		  printf("UTC Time: %02d:%02d:%02d\n\r", GpsState.Hour, GpsState.Minute, GpsState.Second);
		  printf("Date: %02d.%02d.20%02d\n\r", GpsState.Day, GpsState.Month, GpsState.Year);
		  printf("Latitude: %.6f %c\n\r", GpsState.Latitude, GpsState.LatitudeDirection);
		  printf("Longitude: %.6f %c\n\r", GpsState.Longitude, GpsState.LongitudeDirection);
		  printf("Altitude: %.2f m above sea level\n\r", GpsState.Altitude);
		  printf("Speed: %.2f knots, %f km/h\n\r", GpsState.SpeedKnots, GpsState.SpeedKilometers);
		  printf("Course: %.2f degrees\n\r", GpsState.Course);
		  printf("MagneticDeclination: %.2f degrees\n\r", GpsState.MagneticDeclination);
		  printf("Satelites: %d\n\r", GpsState.SatelitesNumber);
		  printf("Dilution of precision: %.2f\n\r", GpsState.Dop);
		  printf("Horizontal dilution of precision: %.2f\n\r", GpsState.Hdop);
		  printf("Vertical dilution of precision: %.2f\n\r", GpsState.Vdop);
		  //char ok[] = "\n\r";
		  //HAL_UART_Transmit(&huart1, ok, strlen((char *)ok), 0xFFFF);
		  print_float(head, gx);
		  print_float(rl, gy);
		  print_float(ptc, gz);
		  print_float(st0, (float)fusion.Point.mx);
		  print_float(st1, (float)fusion.Point.my);
		  print_float(st2, (float)fusion.KF.state.data[0]);
		  print_float(st3, (float)fusion.KF.state.data[0]);
		  Timer = HAL_GetTick();
	  }/*




	  if ((HAL_GetTick() - prev_ms) > 1000) {

		  /*for (int i = 0; i < 30; i++)
		  {

		  }*/
		  //sprintf(str, "  %.4f  ", data);

		  //sprintf(str, "%d  ", (int16_t)heading);
		  //ILI9341_DrawNumberText(str, FONT00, 20, 35, BLACK, WHITE, 1);

		  //ILI9341_Draw_Rectangle(390, 0, 90, 290, WHITE);

		  	  /*if (avgSpeed > speed1)
		  	  {
		  		  h = 150;
		  		  ILI9341_Draw_Rectangle(460, 290-h, 20, h, RED);
		  	  }
		  	  else
		  	  {
		  		  h = 50;
		  		  ILI9341_Draw_Rectangle(460, 290-h, 20, h, GREEN);
		  	  }
		  	  ILI9341_Draw_Rectangle(0, 293, 480, 27, WHITE);
		  	  if (dAvg > 0)
		  	  {
		  		  ILI9341_Draw_Rectangle(240, 293, hC, 27, GREEN);
		  	  }
		  	  else
		  	  {
		  		  ILI9341_Draw_Rectangle(240-hC, 293, +hC, 27, RED);
		  	  }
		  	//  ILI9341_Draw_Rectangle(239, 290, 2, 30, BLACK);
		  //prev_ms = HAL_GetTick();
	  }*/

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 12;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_DC_Pin|TFT_RST_Pin|TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_DC_Pin */
  GPIO_InitStruct.Pin = TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RST_Pin TFT_CS_Pin */
  GPIO_InitStruct.Pin = TFT_RST_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch){
	ITM_SendChar(ch);
	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == GpsState.neo6_huart)
	{
		NEO6_ReceiveUartChar(&GpsState);
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
	//printf("Error Handler");
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	 printf("Error Handler");
	  HAL_Delay(1000);
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

