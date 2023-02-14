/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "dac.h"
#include "dma.h"
#include "ltdc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "dac.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// #ifndef HSEM_ID_0
// #define HSEM_ID_0 (0U) /* HW semaphore 0*
// #endif
#define SHARED_ADDRESS 0x30040000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t *sharedMemoryAddress = (uint16_t *)SHARED_ADDRESS; // shared memory address pointer, points to the shared memory array in SDRAM
uint16_t array[ADC_SIZE];
uint16_t DAC_Table[ADC_SIZE];
float ftmp[ADC_SIZE] = {0};
uint32_t wave[480];
uint32_t Notif_Recieved = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void User_Tim6_Freq(uint32_t _freq) // 形参为频�?
{
  uint16_t _period;
  uint16_t _prescaler = 2;
  float ftmp;
  ftmp = 240000000 / (_prescaler + 1) / _freq - 1;
  // 频率计算
  _period = (uint16_t)ftmp;

  // _period = APB2_FREQ / (_prescaler + 1) / _freq - 1;
  htim6.Init.Prescaler = _prescaler;
  htim6.Init.Period = _period;
  HAL_TIM_Base_Init(&htim6);
  // MX_TIM6_Init(_prescaler,_period);
}
// 根据DAC要求的频率，计算TIM6的频率
void Set_DAC_Freq(uint32_t f_dac)
{
  uint32_t f_tim6;

  f_tim6 = f_dac * ADC_SIZE;
  User_Tim6_Freq(f_tim6);
}
void linear_interpolation(uint16_t *array, size_t array_len, uint32_t *converted_array, size_t converted_array_len)
{
  // 计算相邻元素之间的步长
  double step = (double)(array_len - 1) / (converted_array_len - 1);

  // 执行线性插值
  for (size_t i = 0; i < converted_array_len; i++)
  {
    double index = i * step;
    size_t idx1 = (size_t)index;
    size_t idx2 = idx1 + 1;
    double frac = index - idx1;
    converted_array[i] = (uint16_t)((1.0 - frac) * array[idx1] + frac * array[idx2]);
  }
}
// void ButterLPF_5000Hz(float *src_arr, float *dest_array)
// {
//   for (int n = 0; n < ADC_SIZE; n++)
//   {
//     if (n == 0)
//     {
//       dest_array[n] = b[0] * src_arr[n];
//     }
//     else
//     {
//       dest_array[n] = b[0] * src_arr[n] + b[1] * src_arr[n - 1] - a[1] * dest_array[n - 1];
//     }
//   }
// }
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  /* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
  /* USER CODE END Boot_Mode_Sequence_0 */

  /* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  // timeout = 0xFFFF;
  // while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0))
  //   ;
  // if (timeout < 0)
  // {
  //   Error_Handler();
  // }
  /* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN Boot_Mode_Sequence_2 */
  /* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
  HSEM notification */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /*Take HSEM */
  // HAL_HSEM_FastTake(HSEM_ID_0_ADCOK);
  // /*Release HSEM in order to notify the CPU2(CM4)*/
  // HAL_HSEM_Release(HSEM_ID_0_ADCOK, 0);
  // /* wait until CPU2 wakes up from stop mode */
  // timeout = 0xFFFF;
  // while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0))
  //   ;
  // if (timeout < 0)
  // {
  //   Error_Handler();
  // }
  /* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_LTDC_Init();
  /* USER CODE BEGIN 2 */

  int start_index = 480 * 136 / 2;

  //
  for (int i = start_index; i < start_index + 480 * 3; i++)
  {
    RGB565_480x272[i] = 0x8400;
  }

  // Enable the LCD display, DE signal and BL signal

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // PD7 DISP signal
  // HAL_GPIO_WritePin(GPIOK,GPIO_PIN_7,GPIO_PIN_SET); //PK7 DE Signal

  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0_ADCOK));
  // // Start the PWM timer and ADC DMA to start 60kHz PWM and ADC sampling
  // HAL_TIM_Base_Start(&htim2);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  // HAL_ADC_Start_DMA(&hadc3, (uint32_t *)rawADCValue, ADC_SIZE);
  Set_DAC_Freq(1000);
  HAL_TIM_Base_Start(&htim6);
  // HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)DAC_Table, ADC_SIZE, DAC_ALIGN_12B_R);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // for (size_t i = 0; i < 1000; i++)
    // {
    //   array[i] = sharedMemoryAddress[i];
    // }
    // LD8 flashes when functioning properly
    Notif_Recieved = 0;
    HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0_ADCOK));
    while (HAL_HSEM_IsSemTaken(HSEM_ID_0_ADCOK))
    {
    }
    // take the semaphore to read the shared memory
    HAL_HSEM_FastTake(HSEM_ID_0_ADCOK);
    for (size_t i = 0; i < ADC_SIZE; i++)
    {
      array[i] = sharedMemoryAddress[i];
    }
    // convert the 16 bit ADC value to 12 bit DAC value and output through dma
    for (size_t i = 0; i < ADC_SIZE; i++)
    {
      ftmp[i] = array[i] * 4096.0f / 65536.0f;
      DAC_Table[i] = (uint16_t)ftmp[i];
    };
    // perform linear interpolation to get 480 points on LCD
    linear_interpolation(array, ADC_SIZE, wave, 480);
    // map from 0-65535 to 0-271
    for (size_t i = 0; i < 480; i++)
    {
      wave[i] = (uint16_t)((float)wave[i] * 271.0f / 65535.0f);
    }

    const uint16_t pixels_per_row = 480;

    // // 遍历每一行
    // for (uint16_t row = 0; row < 136; row++)
    // {
    //   // 计算屏幕上的像素点的索引
    //   uint32_t pixel_index = row * pixels_per_row;

    //   // 将波形幅度值映射到屏幕上的行数
    //   uint16_t mapped_row = wave[row];

    //   // 在第一列的对应位置画出一个点
    //   if (row == mapped_row)
    //   {
    //     RGB565_480x272[pixel_index] = 0x8400; // 设置为黑色
    //   }
    //   else
    //   {
    //     RGB565_480x272[pixel_index] = 0x00; // 设置为黄色
    //   }
    // }

    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)DAC_Table, ADC_SIZE, DAC_ALIGN_12B_R);
    HAL_HSEM_Release(HSEM_ID_0_ADCOK, 0);

    // HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
    // HAL_Delay(5000);
    // HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
    // HAL_Delay(500);
    // // LD7 flashes when functioning properly
    // HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
    // HAL_Delay(5000);
    // HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
    // HAL_Delay(10);

    // HAL_UART_Transmit(&huart1, (uint8_t *)"OUT", sizeof("OUT"), 10);
    // HAL_Delay(1000);
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

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Macro to configure the PLL clock source
   */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Once the HSEM is released, store the data in the shared memory
void HAL_HSEM_FreeCallback(uint32_t SemMask)
{
  // HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
  Notif_Recieved = 1;
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
    // Toggles at 1Hz on error (LD6,Red LED)
    HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_13);
    HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
