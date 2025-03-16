/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "lsm6dso32x_reg.h" // Include the LSM6DSO32X driver header
#include <string.h>
#include <stdio.h>

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
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static stmdev_ctx_t dev_ctx; // LSM6DSO32X device context
static uint8_t whoamI, rst;  // Variables for device ID and reset status
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Platform-specific I2C functions for LSM6DSO32X */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len); // UART transmit helper
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Platform-specific I2C write function */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LSM6DSO32X_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
  return 0;
}

/* Platform-specific I2C read function */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSO32X_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/* UART transmit helper function */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
}

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize LSM6DSO32X driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;

  /* Check device ID */
  lsm6dso32x_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DSO32X_ID) {
    uint8_t msg[] = "LSM6DSO32X not found\r\n";
    tx_com(msg, strlen((char*)msg));
    while (1); // Hang if sensor not detected
  } else {
    uint8_t msg[] = "LSM6DSO32X detected\r\n";
    tx_com(msg, strlen((char*)msg));
  }

  /* Restore default configuration */
  lsm6dso32x_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dso32x_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable block data update */
  lsm6dso32x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set output data rate */
  lsm6dso32x_xl_data_rate_set(&dev_ctx, LSM6DSO32X_XL_ODR_104Hz);
  lsm6dso32x_gy_data_rate_set(&dev_ctx, LSM6DSO32X_GY_ODR_104Hz);

  /* Set full scale */
  lsm6dso32x_xl_full_scale_set(&dev_ctx, LSM6DSO32X_2g);
  lsm6dso32x_gy_full_scale_set(&dev_ctx, LSM6DSO32X_2000dps);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t reg;
    /* Check if accelerometer data is ready */
    lsm6dso32x_xl_flag_data_ready_get(&dev_ctx, &reg);
    if (reg) {
      int16_t data_raw[3];
      float data[3];
      uint8_t buffer[64];

      /* Read accelerometer data */
      lsm6dso32x_acceleration_raw_get(&dev_ctx, data_raw);
      data[0] = lsm6dso32x_from_fs2g_to_mg(data_raw[0]);
      data[1] = lsm6dso32x_from_fs2g_to_mg(data_raw[1]);
      data[2] = lsm6dso32x_from_fs2g_to_mg(data_raw[2]);
      sprintf((char*)buffer, "ACC [mg]: X: %.2f Y: %.2f Z: %.2f\r\n", data[0], data[1], data[2]);
      tx_com(buffer, strlen((char*)buffer));

      /* Read gyroscope data */
      lsm6dso32x_angular_rate_raw_get(&dev_ctx, data_raw);
      data[0] = lsm6dso32x_from_fs2000dps_to_mdps(data_raw[0]);
      data[1] = lsm6dso32x_from_fs2000dps_to_mdps(data_raw[1]);
      data[2] = lsm6dso32x_from_fs2000dps_to_mdps(data_raw[2]);
      sprintf((char*)buffer, "GYR [mdps]: X: %.2f Y: %.2f Z: %.2f\r\n", data[0], data[1], data[2]);
      tx_com(buffer, strlen((char*)buffer));

      /* Read temperature data */
      lsm6dso32x_temperature_raw_get(&dev_ctx, data_raw);
      data[0] = lsm6dso32x_from_lsb_to_celsius(data_raw[0]);
      sprintf((char*)buffer, "TEMP [degC]: %.2f\r\n", data[0]);
      tx_com(buffer, strlen((char*)buffer));

      /* Small delay to avoid flooding the UART */
      HAL_Delay(100);
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
