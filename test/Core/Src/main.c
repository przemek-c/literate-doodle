/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dso32x_reg.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Adres IMU (LSM6DSO32X) - 0x6A, dla funkcji HAL przesunięcie w lewo o 1 bit
#define IMU_ADDRESS (0x6A << 1)

// Wartość WHO_AM_I dla LSM6DSO32X to 0x6C
#define WHO_AM_I_VALUE 0x6C

// Zmienne IMU
#define ACCEL_SENSITIVITY_UG 244
#define GYRO_SENSITIVITY_UDPS 17500

// Zmienne UART
#define RX_BUFFER_SIZE 32
#define START_MARKER '['
#define END_MARKER ']'

// Zmienne pomiaru dystansu
#define CALCULATION_INTERVAL_MS 100 
#define PULSES_PER_REVOLUTION 4 // Rozdzielczość enkodera (4 impulsy na obrót)
#define WHEEL_DIAMETER_M 0.075 // Średnica koła w metrach (75 mm)
#define PI 3.1415926535f
#define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_M * PI)
#define MAX_RPS_THRESHOLD 10.0f 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */
uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxIndex = 0;
volatile uint8_t messageComplete = 0;

// Flagi sterujące
volatile char Steering = 'N';    // L/R/S/N (Left/Right/Straight/None)
volatile char Gear = 'N';        // F/B/N (Forward/Backward/None)
volatile uint16_t Duration = 0;   // Czas trwania manewru w ms
volatile char Lifting = 'N';

// Zmienna do śledzenia, czy jest segment jest w trakcie wykonywania
volatile uint8_t commandActive = 0;  // 0: oczekujący, 1: aktywny

// Pomiar dystansu
volatile uint32_t encoderPulseCount = 0;
volatile float linear_mps = 0.0f;
volatile float currentVelocity = 0.0f;
volatile uint32_t lastPulseCount = 0;
volatile uint32_t lastCalcTime = 0;
float rps = 0.0f;
volatile float lastGoodVelocity = 0.0f;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void imu_write_register(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, IMU_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}

void imu_read_registers(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 1000);
}

// Inicjalizacja IMU
void imu_init(void) {
    uint8_t whoami;
    imu_read_registers(0x0F, &whoami, 1);
    if (whoami != WHO_AM_I_VALUE) {
        printf("Nie wykryto układu IMU: 0x%02X\n\r", whoami);
        while (1);
    }
    imu_write_register(0x12, 0x40);
    imu_write_register(0x10, 0x64);
    imu_write_register(0x11, 0x64);
}

void imu_read_accel(imu_data_t *accel) {
    uint8_t buffer[6];
    imu_read_registers(0x28, buffer, 6);
    accel->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    accel->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    accel->z = (int16_t)(buffer[4] | (buffer[5] << 8));
}

void imu_read_gyro(imu_data_t *gyro) {
    uint8_t buffer[6];
    imu_read_registers(0x22, buffer, 6);
    gyro->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    gyro->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    gyro->z = (int16_t)(buffer[4] | (buffer[5] << 8));
}

int32_t readGyroZ(){
    imu_data_t accel, gyro;
    imu_read_accel(&accel);
    imu_read_gyro(&gyro);

    return (int32_t)gyro.z * GYRO_SENSITIVITY_UDPS;
}

void parseMessage(char* msg) {
  if (!msg) {
      printf("Zły format wiadomości\n\r");
      return;
  }

  printf("Otrzymano wiadomość: %s\n\r", msg);
  char* ptr = msg;
  ptr += 2;
  
  // Iteracja po wiadomości aż do znaku ']' lub końca stringa
  while (*ptr != ']' && *ptr != '\0') {
      if (strncmp(ptr, "S:", 2) == 0) {
          ptr += 2;  // Pominięcie "S:"
          if (*ptr != '\0') { 
              Steering = *ptr;
              ptr++;
          }
      }
      else if (strncmp(ptr, "G:", 2) == 0) {
          ptr += 2;  // Pominięcie "G:"
          if (*ptr != '\0') {
              Gear = *ptr;
              ptr++;
          }
      }
      
      else if (strncmp(ptr, "D:", 2) == 0) {
          ptr += 2;  // Pominięcie "D:"
          char* endPtr;
          long temp = strtol(ptr, &endPtr, 10);
          if (endPtr != ptr) {
        	  Duration = (uint16_t)temp;
            ptr = endPtr;
          }
      }

      else if (strncmp(ptr, "L:", 2) == 0) {
          ptr += 2;  // Pominięcie "L:"
          if (*ptr != '\0') {
        	  Lifting = *ptr;
              ptr++;
          }
      }

      // Pominęcie przecinka
      if (*ptr == ',') {
        ptr++;
      }
  }
  
  if (Steering == 'N' && Gear == 'N' && Lifting == 'N') {
    commandActive = 0;
  } 
  else {
    commandActive = 1;
    lastCalcTime = HAL_GetTick();
  }
  
  
  printf("Przypisane flagi sterujące - S:%c G:%c D:%d L:%c\n\r",
		  Steering, Gear, Duration, Lifting);
}

void sendDataToPlot(float desiredVelocity, float currentVelocity, float error, float output) {
  int32_t desiredV_scaled = (int32_t)(desiredVelocity * 1000.0f);
  int32_t currentV_scaled = (int32_t)(currentVelocity * 1000.0f);
  int32_t error_scaled = (int32_t)(error * 1000.0f);
  int32_t output_scaled = (int32_t)(output * 1000.0f);

  int32_t gyro_udps_z = readGyroZ();

   // Bufor na wiadomość
   char rpi_buffer[100];

   // Formatowanie wiadomosci do wyslania przez USART1
   int len = sprintf(rpi_buffer, "%lu,%ld,%ld,%ld,%ld,%ld\n",
                     HAL_GetTick(),      // Millisekundy 
                     desiredV_scaled,
                     currentV_scaled,
                     error_scaled,
                     output_scaled,
                     gyro_udps_z);
 
   if (len > 0) {
     HAL_UART_Transmit(&huart1, (uint8_t*)rpi_buffer, len, HAL_MAX_DELAY);
   }

}


void PIcontroller(volatile float m_desiredVelocity, volatile float m_currentVelocity){

	float current_velocity_truncated = (float)((int)(m_currentVelocity * 100.0f)) / 100.0f;
	float desired_velocity_truncated = (float)((int)(m_desiredVelocity * 100.0f)) / 100.0f;

	float error = desired_velocity_truncated - current_velocity_truncated;

	float pTerm = Kp * error;
	integralTerm += Ki * error * (CALCULATION_INTERVAL_MS / 1000.0f);

	// Ograniczenie całki (anti windup)
	if (integralTerm > maxIntegral) integralTerm = maxIntegral;
	else if (integralTerm < -maxIntegral) integralTerm = -maxIntegral;

	float output = pTerm + integralTerm;

	if (output > maxPWM) output = maxPWM;
	else if (output < minPWM) output = minPWM;

	// sendDataToPlot(m_desiredVelocity, m_currentVelocity, error, output);

  TIM8->CCR2 = (uint32_t)output;

}


void Drive() {

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  switch (Gear)
  {
  case 'F': // F w ASCII 70
	  TIM2->CCR3 = 0;
    if (Controller == 0) {
      TIM8->CCR2 = 50;
      // sendDataToPlot(desiredVelocity, currentVelocity, 0, 50);
    }
    else if (Controller == 1) {
      PIcontroller(desiredVelocity, currentVelocity);
    }
    else {
      TIM8->CCR2 = 0;
    }

    break;
  case 'B': // 66
	  TIM8->CCR2 = 0;
	  TIM2->CCR3 = 30;
	  break;
  default:
    TIM8->CCR2 = 0;
    TIM2->CCR3 = 0;
    break;
  }
}

void Steer() {

	int PWMtoSteer = 100;
  
    switch (Steering)
    {
    case 'L': // 76
        TIM3->CCR2 = 0;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        TIM4->CCR1 = PWMtoSteer;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      break;
    case 'R': // 82
        TIM4->CCR1 = 0;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        TIM3->CCR2 = PWMtoSteer;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
      break;
    default:
        TIM4->CCR1 = 0;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        TIM3->CCR2 = 0;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
      break;
    }
}

void Lift(){
	switch(Lifting)
	{
    case 'U': // 85
    	TIM17->CCR1 = 0;
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        TIM1->CCR3 = 30;

        HAL_Delay(2000);
        TIM1->CCR3 = 0;
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        Lifting = 'N';
        break;
    case 'D': // 68
    	TIM1->CCR3 = 0;
        HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
        TIM17->CCR1 = 30;

        HAL_Delay(2000);
        TIM17->CCR1 = 0;
        HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
        Lifting = 'N';
        break;
    default:
        if (TIM1->CCR3 != 0) {
        	TIM1->CCR3 = 0;
        }

        if (TIM17->CCR1 != 0) {
        	TIM17->CCR1 = 0;
        }
      break;
	}
}

void calculateDistance() {
    float distance_per_pulse = WHEEL_CIRCUMFERENCE_M / PULSES_PER_REVOLUTION;
    
    float distance = (float)encoderPulseCount * distance_per_pulse * 1000;
    
    // Wysłanie wiadomości o dystansie przez UART
    int distanceINT = (int) distance;
    char distance_buffer[50];
    int len = sprintf(distance_buffer, "Dystans: %d\n", distanceINT);
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)distance_buffer, len, HAL_MAX_DELAY);
    }
    // Wyzerowanie enkodera
    encoderPulseCount = 0;
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rxBuffer[0], 1);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Inicializacja COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

  // Test wysyłania wiadomości COM1 port
  // printf("Hello world !\n\r");

  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_GREEN);

  imu_init();  // Inicjalizacja IMU

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (messageComplete) {
        parseMessage((char*)rxBuffer);
        messageComplete = 0;
    }

    uint32_t now = HAL_GetTick();
    
    if (commandActive && (now - lastCalcTime < Duration)) {
        Drive();
        Steer();
        Lift();
        sendDataToPlot(0,0,0,0); // atrybuty 0, ponieważ zrezygnowano z regulacji prędkości

    } else if (commandActive) {
        Gear = 'N';
        Steering = 'N';
        Lifting = 'N';
        Drive();
        Steer();
        Lift();
        
        HAL_Delay(500);

        // Wysłanie znaku '1' do RPi oznaczającego koniec danego manewru
        char signal[3] = "1\n";  // Bufor wiadomości
        HAL_UART_Transmit(&huart1, (uint8_t*)signal, strlen(signal), HAL_MAX_DELAY);

        calculateDistance();
        
        // Reset
        commandActive = 0;
        Duration = 0;

    }

    //Część testowa z działania układów peryferyjnych z wykorzystaniem przycisku 
    if (BspButtonState == BUTTON_PRESSED)
    {
      /* Update button state */
      BspButtonState = BUTTON_RELEASED;
      /* -- Sample board code to toggle leds ---- */
      BSP_LED_Toggle(LED_GREEN);

      /* ..... Perform your action ..... */
      printf("Hello World!\n\r");
      // readIMU();

      imu_data_t accel, gyro;
      imu_read_accel(&accel);
      imu_read_gyro(&gyro);

      // Compute scaled values using integer arithmetic
      int32_t accel_ug_x = (int32_t)accel.x * ACCEL_SENSITIVITY_UG;
      int32_t accel_ug_y = (int32_t)accel.y * ACCEL_SENSITIVITY_UG;
      int32_t accel_ug_z = (int32_t)accel.z * ACCEL_SENSITIVITY_UG;
      int32_t gyro_udps_x = (int32_t)gyro.x * GYRO_SENSITIVITY_UDPS;
      int32_t gyro_udps_y = (int32_t)gyro.y * GYRO_SENSITIVITY_UDPS;
      int32_t gyro_udps_z = (int32_t)gyro.z * GYRO_SENSITIVITY_UDPS;

      // printing
      printf("Accel [ug]: X=%ld, Y=%ld, Z=%ld\n\r",
              accel_ug_x, accel_ug_y, accel_ug_z);
      printf("Gyro [mdps]: X=%ld, Y=%ld, Z=%ld\n\r",
                    gyro_udps_x, gyro_udps_y, gyro_udps_z);


      HAL_Delay(100);

      // runMotor(Gear, Type, Velocity);

      // printf("Pulses counted: %ld\n\r", encoderPulseCount);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
      if (rxIndex == 0 && rxBuffer[0] != START_MARKER) {
          HAL_UART_Receive_IT(&huart1, &rxBuffer[0], 1);
          return;
      }
      
      if (rxBuffer[rxIndex] == END_MARKER) {
          rxBuffer[rxIndex + 1] = '\0';
          messageComplete = 1;
          rxIndex = 0;
      } else if (rxIndex < RX_BUFFER_SIZE - 2) {
          rxIndex++;
      }
      
      HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_9) { // Przerwanie od enkodera
        encoderPulseCount++;
        BSP_LED_Toggle(LED_GREEN);
    }
}
/* USER CODE END 4 */

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

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
