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
#include <stdio.h> // Ensure stdio.h is included for sprintf
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
// IMU I2C address (LSM6DSO32X uses 0x6A or 0x6B depending on SDO pin; adjust if needed)
#define IMU_ADDRESS (0x6A << 1)  // 7-bit address shifted left for HAL (0xD4)

// Expected WHO_AM_I value for LSM6DSO32X
#define WHO_AM_I_VALUE 0x6C

// Sensitivity factors for converting raw data to physical units
/*
#define ACCEL_SENSITIVITY 0.061f  // mg/LSB for ±2g full scale
#define GYRO_SENSITIVITY 70.0f    // mdps/LSB for ±2000 dps full scale
*/
// Sensitivity factors (integer)
#define ACCEL_SENSITIVITY_UG 244  // micro-g per LSB for ±8g range
// #define GYRO_SENSITIVITY_MDPS 70  // mdps per LSB for ±2000 dps range
#define GYRO_SENSITIVITY_UDPS 17500 // micro-dps per LSB for ±500 dps

// UART STAFF
#define RX_BUFFER_SIZE 32
#define START_MARKER '['
#define END_MARKER ']'

// Velocity calculation
#define CALCULATION_INTERVAL_MS 100 // Calculate velocity every 100ms
#define PULSES_PER_REVOLUTION 1 // Example: Encoder resolution
#define WHEEL_DIAMETER_M 0.075 // Example: Wheel diameter in meters (e.g., 65mm)
#define PI 3.1415926535f
#define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_M * PI)

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

// Variables to store parsed values
volatile char Steering = 'N';    // L/R/S/N (Left/Right/Straight/None)
volatile char Gear = 'N';        // F/B/N (Forward/Backward/None)
volatile char Type = 'N';        // A/D/C/N (Acceleration/Deceleration/Constant/None)
volatile uint8_t Velocity = 0;   // 0-100
volatile uint8_t Duration = 0;   // seconds

// motor controller
// velocity calculation
volatile uint32_t encoderPulseCount = 0;
volatile float linear_mps = 0.0f;
volatile float currentVelocity = 0.0f;
volatile uint32_t lastPulseCount = 0;
volatile uint32_t lastCalcTime = 0;

//PI Controller
volatile float desiredVelocity = 0.0f;
float Kp = 10.0f; // Proportional gain (NEEDS TUNING)
float Ki = 5.0f;  // Integral gain (NEEDS TUNING)
float integralTerm = 0.0f;
float maxPWM = 99.0f; // Adjust based on selected Timer's ARR register value
float minPWM = 0.0f; // int would be fine I guess
float maxIntegral = 49.5f; // (maxPWM / 2) Example anti-windup limit (NEEDS TUNING)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Write to an IMU register
void imu_write_register(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, IMU_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}

// Read from IMU registers
void imu_read_registers(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 1000);
}

// Initialize the IMU
void imu_init(void) {
    uint8_t whoami;
    imu_read_registers(0x0F, &whoami, 1);  // Read WHO_AM_I register
    if (whoami != WHO_AM_I_VALUE) {
        printf("IMU not found: 0x%02X\n\r", whoami);
        while (1);  // Hang if IMU not detected
    }
    imu_write_register(0x12, 0x40);  // CTRL3_C: Enable Block Data Update (BDU)
    imu_write_register(0x10, 0x64);  // CTRL1_XL: 104 Hz, ±8g (was 0x60 for ±2g)
    imu_write_register(0x11, 0x64);  // CTRL2_G: 104 Hz, ±500 dps (was 0x6C for ±2000 dps)
}

// Read accelerometer data
void imu_read_accel(imu_data_t *accel) {
    uint8_t buffer[6];
    imu_read_registers(0x28, buffer, 6);  // OUTX_L_XL to OUTZ_H_XL
    accel->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    accel->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    accel->z = (int16_t)(buffer[4] | (buffer[5] << 8));
}

// Read gyroscope data
void imu_read_gyro(imu_data_t *gyro) {
    uint8_t buffer[6];
    imu_read_registers(0x22, buffer, 6);  // OUTX_L_G to OUTZ_H_G
    gyro->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    gyro->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    gyro->z = (int16_t)(buffer[4] | (buffer[5] << 8));
}

int32_t readGyroZ(){
    imu_data_t accel, gyro;
    imu_read_accel(&accel);
    imu_read_gyro(&gyro);

    // Compute scaled values using integer arithmetic
    /*
    int32_t accel_ug_x = (int32_t)accel.x * ACCEL_SENSITIVITY_UG;
    int32_t accel_ug_y = (int32_t)accel.y * ACCEL_SENSITIVITY_UG;
    int32_t accel_ug_z = (int32_t)accel.z * ACCEL_SENSITIVITY_UG;
    int32_t gyro_udps_x = (int32_t)gyro.x * GYRO_SENSITIVITY_UDPS;
    int32_t gyro_udps_y = (int32_t)gyro.y * GYRO_SENSITIVITY_UDPS;
    int32_t gyro_udps_z = (int32_t)gyro.z * GYRO_SENSITIVITY_UDPS;
    */

    return (int32_t)gyro.z * GYRO_SENSITIVITY_UDPS;

    // printing
    // printf("Accel [ug]: X=%ld, Y=%ld, Z=%ld\n\r",
    //        accel_ug_x, accel_ug_y, accel_ug_z);
    // printf("Gyro [mdps]: X=%ld, Y=%ld, Z=%ld\n\r",
    //              gyro_udps_x, gyro_udps_y, gyro_udps_z);
}

// UART parsing message
// there was a problem with first char so I change ptr++ to ptr += 2
// and Python code sends two [[ but here it sees only one
// weird but it works like that

void parseMessage(char* msg) {
  if (!msg) {
      printf("Error: Null message pointer\n\r");
      return;
  }

  printf("Original message: %s\n\r", msg);  // Debug original message
  char* ptr = msg;
  

  // First, let's check and skip the opening bracket
  if (*ptr == '[') {
      // ptr++;
      ptr += 2;
      printf("After bracket check, ptr points to: %s\n\r", ptr);
  } else {
      printf("Error: Message doesn't start with [\n\r");
      // ptr++;
      ptr += 2;
  }
  
  // Parse all fields in sequence
  while (*ptr != ']' && *ptr != '\0') {
      printf("Current parsing position: '%s'\n\r", ptr);  // Show exactly what we're looking at
      
      // Print the first few characters for debugging
      printf("Next 3 chars: '%c%c%c'\n\r", ptr[0], ptr[1], ptr[2]);
      
      if (strncmp(ptr, "S:", 2) == 0) {
          ptr += 2;  // Skip "S:"
          if (*ptr != '\0') {  // Safety check
              Steering = *ptr;
              printf("Found Steering: %c\n\r", Steering);
              ptr++;
          }
      }
      else if (strncmp(ptr, "G:", 2) == 0) {
          ptr += 2;  // Skip "G:"
          if (*ptr != '\0') {  // Safety check
              Gear = *ptr;
              printf("Found Gear: %c\n\r", Gear);
              ptr++;
          }
      }
      /*
      else if (strncmp(ptr, "T:", 2) == 0) {
          ptr += 2;  // Skip "T:"
          if (*ptr != '\0') {  // Safety check
              Type = *ptr;
              printf("Found Type: %c\n\r", Type);
              ptr++;
          }
      }
      */
      else if (strncmp(ptr, "V:", 2) == 0) {
          ptr += 2;  // Skip "V:"
          char* endPtr;
          long temp = strtol(ptr, &endPtr, 10);
          if (endPtr != ptr) {
              Velocity = (uint8_t)temp;
              printf("Found Velocity: %d\n\r", Velocity);
              ptr = endPtr;
          }
      }
      /*
      else if (strncmp(ptr, "D:", 2) == 0) {
          ptr += 2;  // Skip "D:"
          char* endPtr;
          long temp = strtol(ptr, &endPtr, 10);
          if (endPtr != ptr) {
              Duration = (uint8_t)temp;
              printf("Found Duration: %d\n\r", Duration);
              ptr = endPtr;
          }
      }
      */
      else {
          // If we don't recognize the field, skip one character
          printf("Skipping unknown character: %c\n\r", *ptr);
          ptr++;
      }
      
      // Skip comma if present
      if (*ptr == ',') {
          ptr++;
          printf("Skipped comma, now at: %s\n\r", ptr);
      }
  }
  
  printf("Final parsed values - S:%c G:%c V:%d\n\r",
         // Steering, Gear, Type, Velocity, Duration);
		  Steering, Gear, Velocity);
}
/*
volatile float calculateCurrentVelocity(volatile uint32_t encoderPulseCount){
  uint32_t now = HAL_GetTick();

  if (now - lastCalcTime >= CALCULATION_INTERVAL_MS) {
      // --- Velocity Calculation Logic (as shown previously) ---
      uint32_t currentPulseCount = encoderPulseCount; // Read volatile variable safely
      uint32_t pulsesElapsed = currentPulseCount - lastPulseCount;
      float deltaTime_s = (now - lastCalcTime) / 1000.0f;

      if (deltaTime_s > 0.0001f) {
          float rps = (float)pulsesElapsed / PULSES_PER_REVOLUTION / deltaTime_s;
          linear_mps = rps * WHEEL_CIRCUMFERENCE_M;
          // currentVelocity = linear_mps;
      } else {
          // Handle zero/small delta time
          // linear_mps = 0.0f;
      }
      lastPulseCount = currentPulseCount;
      lastCalcTime = now;

      // what do I expect here?
      // --- End Velocity Calculation Logic ---

      // Call PI controller update *here* if using this approach
      // updatePIController(desiredVelocity);
  }
  return linear_mps;
}
*/
void sendDataToPlot(float desiredVelocity, float currentVelocity, float error, float output) {
  // Scale floats to integers (e.g., multiply by 1000 to keep 3 decimal places)
  int32_t desiredV_scaled = (int32_t)(desiredVelocity * 1000.0f);
  int32_t currentV_scaled = (int32_t)(currentVelocity * 1000.0f);
  int32_t error_scaled = (int32_t)(error * 1000.0f);
  int32_t output_scaled = (int32_t)(output * 1000.0f); // Scale output as well

  int32_t gyro_udps_z = readGyroZ();

  /*
  // Print scaled integers using %ld format specifier for int32_t
  printf("%lu,%ld,%ld,%ld,%ld,%ld\n",
         HAL_GetTick(),      // Timestamp in milliseconds (uint32_t -> %lu)
         desiredV_scaled,
         currentV_scaled,
         error_scaled,
         output_scaled,
		 gyro_udps_z);
  */
   // Buffer to hold the formatted string for USART1
   char rpi_buffer[100]; // Adjust size if necessary, 100 should be safe for these values

   // Format the string into rpi_buffer
   int len = sprintf(rpi_buffer, "%lu,%ld,%ld,%ld,%ld,%ld\n",
                     HAL_GetTick(),      // Timestamp in milliseconds
                     desiredV_scaled,
                     currentV_scaled,
                     error_scaled,
                     output_scaled,
                     gyro_udps_z);
 
   // Transmit the formatted string via USART1 (huart1) to the Raspberry Pi
   if (len > 0) {
     HAL_UART_Transmit(&huart1, (uint8_t*)rpi_buffer, len, HAL_MAX_DELAY); // Using huart1 for USART1
   }

}


void PIcontroller(volatile float m_desiredVelocity, volatile float m_currentVelocity){
	float error = m_desiredVelocity - m_currentVelocity;

	float pTerm = Kp * error;
	integralTerm += Ki * error * (CALCULATION_INTERVAL_MS / 1000.0f);
	// Clamp integral term
	if (integralTerm > maxIntegral) integralTerm = maxIntegral;
	else if (integralTerm < -maxIntegral) integralTerm = -maxIntegral;

	float output = pTerm + integralTerm;

	if (output > maxPWM) output = maxPWM;
	else if (output < minPWM) output = minPWM;

  sendDataToPlot(m_desiredVelocity, m_currentVelocity, error, output);

	// Timers configuration
    TIM8->CCR2 = (uint32_t)output;
    // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // Start PWM only once, not repeatedly here
}

// void runMotor(char gear, char type, uint8_t velocity) {
void runMotor() {
  //Gear = *gear;
  
  switch (Gear)
  {
  case 'F': // F in ASCII is 70
    // motorForward
    // regulator(Velocity, Type);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    PIcontroller(desiredVelocity, currentVelocity);

    break;
  case 'B':
	  // motorBackward
	  TIM8->CCR2 = 0;
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	  break;
  default:
    TIM8->CCR2 = 0;
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    break;
  }
}

void Steer() {
	int PWMtoSteer = 30;
  //Gear = *gear;
  
    switch (Steering)
    {
    case 'L':
    	// one to zero
        TIM3->CCR2 = 0;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    	// second to run
        TIM4->CCR1 = PWMtoSteer;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
      break;
    case 'R':
    	// one to zero
        TIM4->CCR1 = 0;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    	// second to run
        TIM3->CCR2 = PWMtoSteer;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
      break;
    default:
    	// both to zero
        TIM4->CCR1 = 0;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    	// both to zero
        TIM3->CCR2 = 0;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
      break;
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
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rxBuffer[0], 1);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
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

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");

  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_GREEN);

  imu_init();  // Initialize the IMU after peripherals are set up
  // char message[64];

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (messageComplete) {
      parseMessage((char*)rxBuffer);
      messageComplete = 0;
    }
    Steer();
    // currentVelocity = calculateCurrentVelocity(encoderPulseCount);
    /*
    if (currentVelocity == 0){
    	printf("Current velocity is 0\n\r");
    }
    */
    // desiredVelocity = 2.5;
    // without plotting it's pointless

    uint32_t now = HAL_GetTick();

    // Controlling motor with interval
    if (now - lastCalcTime >= CALCULATION_INTERVAL_MS) {
        // --- Velocity Calculation Logic (as shown previously) ---
        uint32_t currentPulseCount = encoderPulseCount; // Read volatile variable safely
        uint32_t pulsesElapsed = currentPulseCount - lastPulseCount;
        float deltaTime_s = (now - lastCalcTime) / 1000.0f;
  
        if (deltaTime_s > 0.0001f) {
            float rps = (float)pulsesElapsed / PULSES_PER_REVOLUTION / deltaTime_s;
            currentVelocity = rps * WHEEL_CIRCUMFERENCE_M;
            // currentVelocity = linear_mps;
        } else {
            // Handle zero/small delta time
            // linear_mps = 0.0f;
        }
        lastPulseCount = currentPulseCount;
        lastCalcTime = now;
  
        // what do I expect here?
        // --- End Velocity Calculation Logic ---
  
        // Call PI controller update *here* if using this approach
        // updatePIController(desiredVelocity);
        runMotor();

    }


    // runMotor(Gear, Type, Velocity); //
    // runMotor();
    Steer(Steering);


    /* -- Sample board code for User push-button in interrupt mode ---- */
    if (BspButtonState == BUTTON_PRESSED)
    {
      /* Update button state */
      BspButtonState = BUTTON_RELEASED;
      /* -- Sample board code to toggle leds ---- */
      BSP_LED_Toggle(LED_GREEN);

      /* ..... Perform your action ..... */
      printf("Let's do this !\n\r");
      // grok code starts
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

      printf("Pulses counted: %ld\n\r", encoderPulseCount);
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
          // Wait for start marker
          HAL_UART_Receive_IT(&huart1, &rxBuffer[0], 1);
          return;
      }
      
      if (rxBuffer[rxIndex] == END_MARKER) {
          // Message complete
          rxBuffer[rxIndex + 1] = '\0';  // Null terminate
          messageComplete = 1;
          rxIndex = 0;
      } else if (rxIndex < RX_BUFFER_SIZE - 2) {
          rxIndex++;
      }
      
      // Continue receiving
      HAL_UART_Receive_IT(&huart1, &rxBuffer[rxIndex], 1);
  }
}

// motor controller EXTI handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_9) { // Check if it's PA9's interrupt
        encoderPulseCount++;
        BSP_LED_Toggle(LED_GREEN);
        // printf("Pulses counted: %ld\n\r", encoderPulseCount);
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
