### Plan
1.  Use PA9 for counting impulses via External Interrupt (EXTI). (USART1 moved to PC4/PC5).
2.  Check available timers for PWM output and periodic velocity calculation.
    *   datasheet [link](https://www.st.com/resource/en/datasheet/stm32g431c6.pdf)
    *   image
        ![alt text]({F81946C4-166E-4B76-A8D4-E9DB81621787}.png)
3.  Controller Implementation:
    *   Count impulses using EXTI on PA9.
    *   Calculate velocity periodically (e.g., every 100ms using a timer interrupt or `HAL_GetTick()`).
    *   **Simulate the DC Motor:** Model and simulate offline (e.g., MATLAB/Simulink) to aid controller design and tuning.
    *   Design PI controller (Kp, Ki tuning needed).
    *   Implement direction control logic using configured GPIOs for the DRV8833 driver.
    *   Configure and use a Timer channel for PWM output to control motor speed via DRV8833.

### Gemini:
Implementing a PI controller using encoder pulses for feedback is a standard and effective way to achieve accurate motor speed control. Here's a breakdown of the steps involved and what you'll need to consider:

1.  **Encoder Input Configuration (PA9):**
    *   **External Interrupt (EXTI):** Configure PA9 as a GPIO Input with an EXTI line associated with it (e.g., EXTI9_5). You'll count pulses in the EXTI interrupt handler. This is suitable for single-pulse feedback.
    *   **CubeMX:** Configure the GPIO EXTI in CubeMX and regenerate the code.

2.  **Pulse Counting & Velocity Calculation:**
    *   **EXTI Handler (if using EXTI):**
        ```c
        // Add in USER CODE BEGIN PV
        volatile uint32_t encoderPulseCount = 0;

        // Add in USER CODE BEGIN 4 (or stm32xxxx_it.c)
        void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
            if (GPIO_Pin == GPIO_PIN_9) { // Check if it's PA9's interrupt
                encoderPulseCount++;
            }
        }
        ```

    *   **Velocity Calculation:** You need a periodic task to calculate velocity based on the `encoderPulseCount`. Here are the common approaches:

        *   **Approach 1: Using `HAL_GetTick()` in the Main Loop:**
            *   Check `HAL_GetTick()` inside your `while(1)` loop in `main.c`.
            *   Simple, but timing depends on main loop execution speed.
            *   **Code Location:** Inside the `while(1)` loop in `main.c`.

            ```c
            // --- Example Code Snippet for main.c ---
            // Add near USER CODE BEGIN PV
            volatile float currentVelocity = 0.0f;
            volatile uint32_t lastPulseCount = 0;
            volatile uint32_t lastCalcTime = 0;
            extern volatile uint32_t encoderPulseCount; // Defined elsewhere (e.g., stm32g4xx_it.c)
            #define CALCULATION_INTERVAL_MS 100 // Calculate velocity every 100ms
            #define PULSES_PER_REVOLUTION 1000 // Example: Encoder resolution
            #define WHEEL_DIAMETER_M 0.065 // Example: Wheel diameter in meters (e.g., 65mm)
            #define PI 3.1415926535f
            #define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_M * PI)

            // Add inside the main while(1) loop, USER CODE BEGIN WHILE
            uint32_t now = HAL_GetTick();
            if (now - lastCalcTime >= CALCULATION_INTERVAL_MS) {
                // --- Velocity Calculation Logic ---
                uint32_t currentPulseCount = encoderPulseCount; // Read volatile variable safely
                uint32_t pulsesElapsed = currentPulseCount - lastPulseCount;
                float deltaTime_s = (now - lastCalcTime) / 1000.0f;

                if (deltaTime_s > 0.0001f) {
                    // Calculate rotational velocity (Revolutions Per Second - RPS)
                    float rps = (float)pulsesElapsed / PULSES_PER_REVOLUTION / deltaTime_s;
                    // Optionally calculate linear velocity (Meters Per Second - m/s)
                    float linear_mps = rps * WHEEL_CIRCUMFERENCE_M;
                    // Assign the desired velocity type (RPS or m/s) to currentVelocity
                    currentVelocity = linear_mps; // Or currentVelocity = rps;
                } else {
                    // Handle zero/small delta time
                    currentVelocity = 0.0f; // Set velocity to zero if no time elapsed
                }
                lastPulseCount = currentPulseCount;
                lastCalcTime = now;
                // --- End Velocity Calculation Logic ---

                // Call PI controller update *here* if using this approach
                // updatePIController(desiredVelocity); // Ensure desiredVelocity has the same units as currentVelocity
            }
            // --- Other main loop code ---
            ```

        *   **Approach 2: Using a Dedicated Hardware Timer Interrupt (Recommended):**
            *   Configure a timer (e.g., TIM6, TIM7) in CubeMX to generate an interrupt every `CALCULATION_INTERVAL_MS`.
            *   Enable the timer's interrupt (`NVIC` settings in CubeMX).
            *   Place the calculation logic inside the timer's callback function.
            *   Provides accurate, consistent timing independent of the main loop.
            *   **Code Location:** Inside `HAL_TIM_PeriodElapsedCallback` function (usually in `stm32g4xx_it.c`).

            ```c
            // --- Example Code Snippet for stm32g4xx_it.c ---
            // Add near USER CODE BEGIN PV
            volatile float currentVelocity = 0.0f; // Units depend on calculation (e.g., m/s or RPS)
            volatile uint32_t lastPulseCount = 0;
            // lastCalcTime is not needed here as the timer interrupt provides the interval
            extern volatile uint32_t encoderPulseCount; // Defined elsewhere or in this file
            #define CALCULATION_INTERVAL_MS 100 // Must match timer configuration
            #define PULSES_PER_REVOLUTION 1000 // Example: Encoder resolution
            #define WHEEL_DIAMETER_M 0.065 // Example: Wheel diameter in meters (e.g., 65mm)
            #define PI 3.1415926535f
            #define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_M * PI)
            // Assuming TIM6 is configured for 100ms interval
            extern TIM_HandleTypeDef htim6; // Make sure your timer handle is accessible

            // Add in USER CODE BEGIN 4 or find existing HAL_TIM_PeriodElapsedCallback
            void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
            {
              if (htim->Instance == TIM6) // Check if it's the correct timer interrupt
              {
                // --- Velocity Calculation Logic ---
                uint32_t currentPulseCount = encoderPulseCount; // Read volatile variable safely
                // Note: Rollover needs careful handling if the counter could wrap within the interval
                uint32_t pulsesElapsed = currentPulseCount - lastPulseCount;
                const float deltaTime_s = CALCULATION_INTERVAL_MS / 1000.0f; // Fixed interval

                // Calculate rotational velocity (Revolutions Per Second - RPS)
                float rps = (float)pulsesElapsed / PULSES_PER_REVOLUTION / deltaTime_s;
                // Optionally calculate linear velocity (Meters Per Second - m/s)
                float linear_mps = rps * WHEEL_CIRCUMFERENCE_M;
                // Assign the desired velocity type (RPS or m/s) to currentVelocity
                currentVelocity = linear_mps; // Or currentVelocity = rps;

                lastPulseCount = currentPulseCount;
                // --- End Velocity Calculation Logic ---

                // Call PI controller update *here* if using this approach
                // updatePIController(desiredVelocity); // Ensure desiredVelocity has the same units as currentVelocity
              }
              // else if (htim->Instance == ...) { /* Handle other timer interrupts */ }

              // Add USER CODE BEGIN Callback_Body
              /* */
              // Add USER CODE END Callback_Body
            }
            ```

3.  **PI Controller Implementation:**
    *   Modify your `regulator` function or create a new `updatePIController` function.
    *   **Important:** This function should be called *at the same frequency* as the velocity calculation (i.e., every `CALCULATION_INTERVAL_MS`). If using Approach 1, call it inside the `if` block. If using Approach 2, call it at the end of the timer interrupt handler.
    *   **Crucial:** The units of `desiredVelocity` passed to this function *must match* the units you chose for `currentVelocity` (e.g., both RPS or both m/s).
    *   ```c
        // Add near USER CODE BEGIN Includes or USER CODE BEGIN PV
        #include <stdio.h> // Required for printf

        // Add in USER CODE BEGIN PV
        float Kp = 10.0f; // Proportional gain (NEEDS TUNING)
        float Ki = 5.0f;  // Integral gain (NEEDS TUNING)
        float integralTerm = 0.0f;
        float maxPWM = 999.0f; // Adjust based on selected Timer's ARR register value
        float minPWM = 0.0f;
        float maxIntegral = maxPWM / 2.0f; // Example anti-windup limit (NEEDS TUNING)
        // Make sure currentVelocity is accessible (e.g., global volatile or passed as argument)
        extern volatile float currentVelocity;

        // Modify regulator or create new function
        // desiredVelocity units must match currentVelocity units (e.g., RPS or m/s)
        void updatePIController(float desiredVelocity) {
            // Ensure currentVelocity is accessed safely (volatile read)
            float actualVelocity = currentVelocity; // Read volatile variable once
            float error = desiredVelocity - actualVelocity; // Error calculation

            // Proportional Term
            float pTerm = Kp * error;

            // Integral Term with Anti-Windup
            // Note: Ensure CALCULATION_INTERVAL_MS is accessible here or pass dt as argument
            #define CALCULATION_INTERVAL_S (CALCULATION_INTERVAL_MS / 1000.0f)
            integralTerm += Ki * error * CALCULATION_INTERVAL_S;
            // Clamp integral term
            if (integralTerm > maxIntegral) integralTerm = maxIntegral;
            else if (integralTerm < -maxIntegral) integralTerm = -maxIntegral;

            // Controller Output
            float output = pTerm + integralTerm;

            // Clamp output to PWM limits (Timer CCR value)
            if (output > maxPWM) output = maxPWM;
            else if (output < minPWM) output = minPWM;

            // Update PWM Duty Cycle (Ensure TIM8 and CCR2 are correct for your setup)
            // Example: Assumes TIM8 Channel 2 controls PWM
            // Make sure TIM8 handle (e.g., htim8) is accessible if using HAL functions
             __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)output);
            // Or direct register access if preferred and configured:
            // TIM8->CCR2 = (uint32_t)output;

            // Data Logging via UART (printf redirected)
            // Ensure UART is initialized and printf is redirected (e.g., via syscalls.c)
            // Format: Time(ms), DesiredV, CurrentV, Error, PTerm, ITerm, OutputPWM
            printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.0f\n",
                   HAL_GetTick(),      // Timestamp in milliseconds
                   desiredVelocity,
                   actualVelocity,
                   error,
                   pTerm,
                   integralTerm,
                   output);
        }
        ```

4.  **Integration:**
    *   **Choose an approach** for periodic calculation (Timer Interrupt recommended).
    *   Implement the chosen approach.
    *   Call `updatePIController(desiredVelocity)` from the *same periodic task* that calculates the velocity.
    *   Ensure the PWM Timer channel is initialized and started (`HAL_TIM_PWM_Start`).
    *   Implement motor direction control logic (setting GPIOs for DRV8833 IN1/IN2).

5.  **Data Logging for MATLAB:**
    *   **STM32 Setup:**
        *   Enable a UART peripheral in CubeMX (e.g., USART2 connected to the ST-LINK VCP).
        *   Configure the project to redirect `printf` to that UART. In CubeIDE, this often involves ensuring `syscalls.c` is included and correctly implemented, or enabling specific settings under Project Properties -> C/C++ Build -> Settings -> Tool Settings -> MCU GCC Linker -> Miscellaneous -> Use float with printf.
        *   Add the `printf` statement shown in the `updatePIController` example to send data periodically.
        *   **Important:** Calling `printf` frequently, especially from an interrupt, can consume significant CPU time and potentially disrupt real-time behavior. If logging causes issues:
            *   Log less frequently.
            *   Buffer the data in the interrupt/periodic task and send the buffer from the main loop.
            *   Consider using ITM/SWO tracing (see "Further Considerations") which has lower overhead.
    *   **MATLAB Setup:**
        *   Connect the Nucleo board to your PC via USB. Identify the Virtual COM Port assigned to the ST-LINK.
        *   Use MATLAB's `serialport` interface:
          ```matlab
          % Example MATLAB code
          device = serialport("COMx", 115200); % Replace COMx with your port, match baud rate
          configureTerminator(device, "LF"); % Or "CR/LF" depending on printf newline

          dataLog = [];
          numSamples = 500; % Collect 500 data points, for example
          disp("Collecting data...");

          for i = 1:numSamples
              try
                  line = readline(device);
                  data = sscanf(line, '%lu,%f,%f,%f,%f,%f,%f'); % Parse CSV
                  if length(data) == 7 % Check if parsing was successful
                      dataLog = [dataLog; data'];
                  else
                      disp(['Warning: Could not parse line: ' line]);
                  end
              catch ME
                  disp(['Error reading from serial port: ' ME.message]);
                  break; % Exit loop on error
              end
          end
          clear device; % Close the serial port
          disp("Data collection complete.");

          % --- Now analyze dataLog ---
          % Columns: Time(ms), DesiredV, CurrentV, Error, PTerm, ITerm, OutputPWM
          time_ms = dataLog(:,1);
          desiredV = dataLog(:,2);
          currentV = dataLog(:,3);
          errorV = dataLog(:,4);
          pTerm = dataLog(:,5);
          iTerm = dataLog(:,6);
          outputPWM = dataLog(:,7);

          figure;
          plot(time_ms / 1000, desiredV, 'r--', 'LineWidth', 1.5); hold on;
          plot(time_ms / 1000, currentV, 'b-', 'LineWidth', 1.5);
          xlabel('Time (s)');
          ylabel('Velocity (units)'); % Adjust units label
          title('Velocity Control Performance');
          legend('Desired Velocity', 'Actual Velocity');
          grid on;

          figure;
          plot(time_ms / 1000, errorV, 'm-');
          xlabel('Time (s)');
          ylabel('Error (units)');
          title('Control Error');
          grid on;

          % Plot other terms as needed...
          ```

**Summary:**
*   **Data Logging:** Implement UART communication to send control loop variables to a PC for analysis in MATLAB. Be mindful of the performance impact of `printf`.

**Missing Pieces / Further Considerations:**
*   **ITM/SWO Tracing:** As an alternative to UART `printf` for logging with lower performance overhead, investigate using the ITM peripheral and SWO pin via the ST-LINK debugger. CubeIDE's "SWV ITM Data Console" can display and log this data.