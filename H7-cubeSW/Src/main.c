/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "i2c.h"
#include "quadspi.h"
#include "rng.h"
#include "sai.h"
#include "spi.h"
#include "tim.h"
//#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "audiostream.h"
#include "lcd.h"
#include "vl53l1_platform.h"
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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define NUM_ADC_CHANNELS 6
uint16_t myADC[NUM_ADC_CHANNELS] __ATTR_RAM_D2;

//VL53L1X laser;

VL53L1_Dev_t dev;
int status=0;
volatile int IntCount;

#define isInterrupt 0 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */
uint8_t byteData, sensorState=0;
uint16_t wordData;
uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
uint16_t Distance;
uint16_t SignalRate;
uint16_t AmbientRate;
uint8_t RangeStatus;
uint8_t dataReady;
int16_t offset;
uint16_t xtalk;







uint32_t counter = 0;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void MPU_Conf(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int isButtonOneDown, isButtonTwoDown;

static void buttonOneDown(void)
{
	isButtonOneDown = 1;
	ftMode = FTFeedback;

	LCD_setCursor(&hi2c4, 0x40);
	LCD_sendChar(&hi2c4, 'F');
	LCD_sendChar(&hi2c4, 'B');
}

static void buttonOneUp(void)
{
	isButtonOneDown = 0;
}

static void buttonTwoDown(void)
{
	isButtonTwoDown = 1;
	ftMode = FTSynthesisOne;

	LCD_setCursor(&hi2c4, 0x40);
	LCD_sendChar(&hi2c4, 'S');
	LCD_sendChar(&hi2c4, '1');
}

static void buttonTwoUp(void)
{
	isButtonTwoDown = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  MPU_Conf();

  SystemInit();
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_BDMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  //MX_QUADSPI_Init();
  MX_RNG_Init();
  MX_SAI1_Init();
  //MX_SPI4_Init();
  MX_I2C4_Init();
  //MX_USB_HOST_Init();
  //MX_TIM3_Init();
  MX_DMA_Init();


  //if (HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&myADC, NUM_ADC_CHANNELS) != HAL_OK)
	{
		//Error_Handler();

	}
	//audioInit(&hi2c2, &hsai_BlockA1, &hsai_BlockB1, &hrng, ((uint16_t*)&myADC));


	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); //led Green
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //led amber

	LCD_init(&hi2c4);
	HAL_Delay(10);
	LCD_home(&hi2c4);
	LCD_sendInteger(&hi2c4, 11111, 5);
  /* USER CODE BEGIN 2 */
  dev.I2cHandle = &hi2c2;
  dev.I2cDevAddr = 0x52;
  /* Those basic I2C read functions can be used to check your own I2C functions */
    //status = VL53L1_RdByte(&dev, 0x010F, &byteData);
    //printf("VL53L1X Model_ID: %X\n", byteData);
    //status = VL53L1_RdByte(&dev, 0x0110, &byteData);
    //printf("VL53L1X Module_Type: %X\n", byteData);
    //status = VL53L1_RdWord(&dev, 0x010F, &wordData);
   // printf("VL53L1X: %X\n", wordData);

    while(sensorState==0)
    {
    	status = VL53L1X_BootState(dev, &sensorState);
    	HAL_Delay(2);
    }
    status = VL53L1X_SensorInit(dev);
      status = VL53L1X_SetDistanceMode(dev, 1); /* 1=short, 2=long */
      status = VL53L1X_SetTimingBudgetInMs(dev, 20); /* in ms possible values [20, 50, 100, 200, 500] */
      status = VL53L1X_SetInterMeasurementInMs(dev, 20);
      	//status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
      	//HAL_Delay(4000);
      	//status = VL53L1X_CalibrateXtalk(dev, 1400, &xtalk);
      	//HAL_Delay(4000);
      //HAL_Delay(1000);
    status = VL53L1X_StartRanging(dev);




/*
	VL53L1X_setTimeout(&laser,500);
	HAL_Delay(2);
	VL53L1X_init(&laser);
	VL53L1X_start(&laser, 1);
	HAL_Delay(2);

	  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
	  // You can change these settings to adjust the performance of the sensor, but
	  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
	  // medium and long distance modes. See the VL53L1X datasheet for more
	  // information on range and timing limits.
	VL53L1X_setDistanceMode(&laser,Short);
	HAL_Delay(2);
	VL53L1X_setMeasurementTimingBudget(&laser,20000);
	HAL_Delay(2);
	  // Start continuous readings at a rate of one measurement every 50 ms (the
	  // inter-measurement period). This period should be at least as long as the
	  // timing budget.
	VL53L1X_startContinuous(&laser,50);
	HAL_Delay(2);
*/



	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //led white
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //while (dataReady == 0){
	  		  //status = VL53L1X_CheckForDataReady(dev, &dataReady);
	  		  //HAL_Delay(10);
	  	  //}
	  	  dataReady = 0;
	  	  //status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  	//HAL_Delay(100);
	  	  status = VL53L1X_GetDistance(dev, &Distance);

	  	LCD_home(&hi2c4);
	  	LCD_sendInteger(&hi2c4, Distance, 5);
		 LCD_sendChar(&hi2c4, ' ');
		 LCD_sendChar(&hi2c4, ' ');
	  	  //Distance = (_I2CBuffer[0] << 8) + _I2CBuffer[1];
	  	  //status = VL53L1X_GetSignalRate(dev, &SignalRate);
	  	  //status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
	  	  //status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	  	  //printf("%u, %u, %u, %u\n", RangeStatus, Distance, SignalRate, AmbientRate);
	  	  //if (RangeStatus == 0)
	  	  {
	  		  //laserRange = Distance;
	  	  }

	  //HAL_Delay(100);
	  //if (VL53L1X_dataReady(&laser) == 0x00)
	  //{
	  //VL53L1X_init(&laser);

/*
	  //VL53L1X_startContinuous(&laser,50);
	  VL53L1X_read(&laser, 0);
		  if (laser.ranging_data.range_status == RangeValid)
		  {
			  laserRange = laser.ranging_data.range_mm;
		  }
		  else
		  {
			  laser.calibrated = 0;
		  }

*/
	  //ADC values are =
	  // [0] = joystick x
	  // [1] = breath
	  // [2] = open
	  // [3] = joy Y
	  // [4] = pedal
	  // [5] = knob

	 //HAL_Delay(20);
	 //LCD_home(&hi2c4);

	 //LCD_sendInteger(&hi2c4, intHarmonic, 2);
	 LCD_sendChar(&hi2c4, ' ');
	 LCD_sendChar(&hi2c4, ' ');
	 //LCD_sendFixedFloat(&hi2c2, dist, 3, 1);

	 //LCD_sendInteger(&hi2c4, myADC[1], 5);
	 LCD_sendChar(&hi2c4, ' ');
	 LCD_sendChar(&hi2c4, ' ');

	 //LCD_sendInteger(&hi2c4, laserRange, 5);

	 //button1
	if (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13))
	{
		if (!isButtonOneDown)	buttonOneDown();
		LCD_sendChar(&hi2c4, '1');
	}
	else
	{
		LCD_sendChar(&hi2c4, ' ');
		if (isButtonOneDown) 	buttonOneUp();
	}

	//button2
	if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0))
	{
		if (!isButtonTwoDown)	buttonTwoDown();
		LCD_sendChar(&hi2c4, '2');
	}
	else
	{

		LCD_sendChar(&hi2c4, ' ');
		if (isButtonTwoDown)	buttonTwoUp();
	}

	//P button
	if (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11))
	{
		LCD_sendChar(&hi2c4, 'P');
		//if (!isPresetButtonDown) 	presetButtonDown();
	}
	else
	{
		LCD_sendChar(&hi2c4, ' ');
		//if (isPresetButtonDown)		presetButtonUp();
	}
    /* USER CODE END WHILE */
    //MX_USB_HOST_Process();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2C4
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_QSPI
                              |RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 344;
  PeriphClkInitStruct.PLL2.PLL2P = 7;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 4;
  PeriphClkInitStruct.PLL3.PLL3Q = 4;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Returns random floating point value [0.0,1.0)
float randomNumber(void) {
	
	uint32_t rand;
	HAL_RNG_GenerateRandomNumber(&hrng, &rand);
	float num = (((float)(rand >> 16))- 32768.f) * INV_TWO_TO_15;
	return num;
}



void MPU_Conf(void)
{
	//code from Keshikan https://github.com/keshikan/STM32H7_DMA_sample
  //Thanks, Keshikan! This solves the issues with accessing the SRAM in the D2 area properly. -JS
	
	MPU_Region_InitTypeDef MPU_InitStruct;

	  HAL_MPU_Disable();

	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;

	  //D2 Domain�SRAM1
	  MPU_InitStruct.BaseAddress = 0x30000000;
	  // Increased region size to 256k. In Keshikan's code, this was 512 bytes (that's all that application needed).
	  // Each audio buffer takes up the frame size * 8 (16 bits makes it *2 and stereo makes it *2 and double buffering makes it *2)
	  // So a buffer size for read/write of 4096 would take up 64k = 4096*8 * 2 (read and write).
	  // I increased that to 256k so that there would be room for the ADC knob inputs and other peripherals that might require DMA access (for instance the OLED screen).
	  // we have a total of 256k in SRAM1 (128k, 0x30000000-0x30020000) and SRAM2 (128k, 0x30020000-0x3004000) of D2 domain. 
		// There is an SRAM3 in D2 domain as well (32k, 0x30040000-0x3004800) that is currently not mapped by the MPU (memory protection unit) controller. 
	  
	  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;

	  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

	  //AN4838
	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
	  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

	  //Shared Device
//	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
//	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
//	  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
//	  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;


	  MPU_InitStruct.Number = MPU_REGION_NUMBER0;

	  MPU_InitStruct.SubRegionDisable = 0x00;


	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;


	  HAL_MPU_ConfigRegion(&MPU_InitStruct);




	  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
