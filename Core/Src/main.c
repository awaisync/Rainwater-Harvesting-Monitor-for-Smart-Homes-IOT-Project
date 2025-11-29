/* USER CODE BEGIN Header  */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define SHT40_ADDR (0x44 << 1) // HAL expects 8-bit address

#define XIAO_I2C_ADDR (0x28 << 1)   // 7-bit 0x28 -> 8-bit for HAL




// HC-SR04 pins
#define HCSR04_TRIG_PORT GPIOA
#define HCSR04_TRIG_PIN  GPIO_PIN_4   // PA4 -> Trigger

#define HCSR04_ECHO_PORT GPIOA
#define HCSR04_ECHO_PIN  GPIO_PIN_7   // PA7 -> Echo







/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;   // USART1 -> Wio-E5
UART_HandleTypeDef huart2;   // USART2 -> PC (ST-LINK)

volatile uint8_t btn_ble_trigger = 0; //flag for ext interrupt for BLE Uplink



uint8_t sht40_data[6];
uint8_t lora_rx[64];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER prototypes */
int  SHT40_Read(float *temperature, float *humidity);
void LoRa_Send(const char *s);
void LoRa_ReadReply(void);
int  LoRa_ReadReply_Long(void);

//Hcro4 prototypes

void  DWT_Init(void);
void  delay_us(uint32_t us);
float HCSR04_ReadDistance_cm(void);

/* USER CODE BEGIN 0 */
// Redirect printf to UART2 (PC)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 100);
    return ch;
}


// call back fucntion for external interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)   // PB0
    {
        btn_ble_trigger = 1;      // just set flag, no heavy work here
    }
}


//BLE function sir jeee

void BLE_I2C_Send(float temp, float hum, float dist_cm)
{
    // Scale and clamp to int16
    int16_t t_x100 = (int16_t)(temp * 100.0f);    // °C * 100
    int16_t h_x100 = (int16_t)(hum  * 100.0f);    // %RH * 100
    int16_t d_x10  = (int16_t)(dist_cm * 10.0f);  // cm * 10

    uint8_t buf[7];
    buf[0] = 0x55;                 // header
    buf[1] = (t_x100 >> 8) & 0xFF; // T high byte (big-endian)
    buf[2] =  t_x100       & 0xFF; // T low byte
    buf[3] = (h_x100 >> 8) & 0xFF; // H high
    buf[4] =  h_x100       & 0xFF; // H low
    buf[5] = (d_x10  >> 8) & 0xFF; // D high
    buf[6] =  d_x10        & 0xFF; // D low

    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
                               &hi2c1,
                               XIAO_I2C_ADDR,
                               buf,
                               sizeof(buf),
                               100   // timeout ms
                           );

    if (st == HAL_OK) {
        printf("BLE_I2C_Send OK: T=%.2f H=%.2f D=%.1f\r\n",
               temp, hum, dist_cm);
    } else {
        printf("BLE_I2C_Send FAILED (status=%d)\r\n", st);
    }
}



// SHT40 read function
int SHT40_Read(float *temperature, float *humidity)
{
    uint8_t cmd = 0xFD; // high precision measurement
    if (HAL_I2C_Master_Transmit(&hi2c1, SHT40_ADDR, &cmd, 1, 100) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10); // wait measurement

    if (HAL_I2C_Master_Receive(&hi2c1, SHT40_ADDR, sht40_data, 6, 100) != HAL_OK)
        return HAL_ERROR;

    uint16_t t_raw = (sht40_data[0] << 8) | sht40_data[1];
    uint16_t h_raw = (sht40_data[3] << 8) | sht40_data[4];

    *temperature = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    *humidity    =       100.0f * ((float)h_raw / 65535.0f);

    return HAL_OK;
}

// Send string to Wio-E5 over USART1
void LoRa_Send(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 1000);
}

// Read reply from Wio-E5 (short, simple)
void LoRa_ReadReply(void)
{
    memset(lora_rx, 0, sizeof(lora_rx));

    // Read up to buffer size - 1, timeout 500 ms
    HAL_UART_Receive(&huart1, lora_rx, sizeof(lora_rx) - 1, 500);

    // Print whatever we got to the PC console
    printf("LoRa reply: %s\r\n", lora_rx);
}



// Read reply from Wio-E5 (blocking, very simple)
int LoRa_ReadReply_Long(void)
{
    memset(lora_rx, 0, sizeof(lora_rx));

    uint32_t start = HAL_GetTick();
    uint16_t idx = 0;
    uint8_t ch;

    while ((HAL_GetTick() - start) < 10000)  // 10 s window
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 500) == HAL_OK)
        {
            if (idx < sizeof(lora_rx) - 1)
                lora_rx[idx++] = ch;
        }
        else
        {
            // 500 ms no data -> probably end of message
            break;
        }
    }

    lora_rx[idx] = 0;
    printf("LoRa long reply:\r\n%s\r\n", lora_rx);

    // crude check: if reply contains "joined", treat as success
    if (strstr((char*)lora_rx, "joined") != NULL ||
        strstr((char*)lora_rx, "Join Succeeded") != NULL)
    {
        return 1;
    }
    return 0;
}

//ultrassonic sensor reads
// --- DWT-based microsecond timing for HC-SR04 ---
// enables CPU cycle counter
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// busy-wait delay in microseconds
void delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { }
}

// --- HC-SR04 distance measurement ---
// returns distance in cm, or -1.0f on timeout
float HCSR04_ReadDistance_cm(void)
{
    uint32_t start_cycles, echo_cycles;
    uint32_t timeout_us = 30000;   // 30 ms echo timeout

    // ensure trigger low
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);

    // 10 us trigger pulse
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    // wait for echo to go high (start)
    uint32_t t0 = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_RESET)
    {
        if ((DWT->CYCCNT - t0) > timeout_us * (SystemCoreClock / 1000000U))
            return -1.0f; // no echo
    }

    // measure high time
    start_cycles = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - start_cycles) > timeout_us * (SystemCoreClock / 1000000U))
            break; // clamp too-long pulse
    }
    echo_cycles = DWT->CYCCNT - start_cycles;

    // convert cycles -> microseconds
    float echo_us = (float)echo_cycles / (SystemCoreClock / 1000000.0f);

    // HC-SR04: ~58 µs per cm (round trip)
    float distance_cm = echo_us / 58.0f;
    return distance_cm;
}

//




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();   // PC UART
  MX_I2C1_Init();          // SHT40
  MX_USART1_UART_Init();   // Wio-E5 UART (9600 baud)

  DWT_Init();  // enable cycle counter for microsecond timing

  HAL_Delay(1000); // let Wio-E5 boot

  printf("Booted. LoRa + SHT40 firmware...\r\n");
  printf("Using stored OTAA credentials\r\n");

  // Show device IDs once at startup
  LoRa_Send("AT+ID\r\n");
  LoRa_ReadReply_Long();

  // Wait 5 seconds before attempting join
  HAL_Delay(5000);

  // Make sure we are in OTAA mode and region
  LoRa_Send("AT+MODE=LWOTAA\r\n");
  HAL_Delay(300);
  LoRa_ReadReply();

  LoRa_Send("AT+DR=EU868\r\n");
  HAL_Delay(300);
  LoRa_ReadReply();

  int joined_ok = 0;
  uint32_t last_join_attempt = 0;

  float temp, hum;

  while (1)
  {
      // --- JOIN LOGIC ---
      if (!joined_ok)
      {
          uint32_t now = HAL_GetTick();
          // try join every 30 s
          if (now - last_join_attempt > 90000 || last_join_attempt == 0)
          {
              printf("Trying AT+JOIN...\r\n");
              LoRa_Send("AT+JOIN\r\n");
              joined_ok = LoRa_ReadReply_Long();
              last_join_attempt = now;

              if (joined_ok)
                  printf("JOIN SUCCESS, will start sending data.\r\n");
              else
                  printf("JOIN FAILED (no downlink). Will retry after 90s.\r\n");
          }

          // Even if not joined, we can still read sensor just for debug
          if (SHT40_Read(&temp, &hum) == HAL_OK)
          {
              float dist = HCSR04_ReadDistance_cm();
              printf("T = %.2f  H = %.2f  Dist = %.1f cm (not joined)\r\n",
                     temp, hum, dist);

              // after you have valid temp, hum, dist:
              if (btn_ble_trigger)
              {
                  btn_ble_trigger = 0;  // clear flag

                  // optional: debounce simple guard
                  static uint32_t last_btn_time = 0;
                  uint32_t now_btn = HAL_GetTick();
                  if (now_btn - last_btn_time > 200)   // 200 ms debounce
                  {
                      last_btn_time = now_btn;

                      BLE_I2C_Send(temp, hum, dist);   // send current values via I2C to XIAO
                      printf("Button Pressed, Send BLE DATA 0\r\n");
                  }
              }

          }



          HAL_Delay(5000);  // sensor interval
          continue;   // skip sending AT+MSGHEX until joined
      }

      // --- WE ARE JOINED: send data every 5 s ---

    	  if (SHT40_Read(&temp, &hum) == HAL_OK)
    	  {
    	      float dist = HCSR04_ReadDistance_cm();
    	      printf("T = %.2f  H = %.2f  Dist = %.1f cm\r\n", temp, hum, dist);

    	      // --- send to XIAO every 30 s ---
    	      // after you have valid temp, hum, dist:
    	      if (btn_ble_trigger)
    	      {
    	          btn_ble_trigger = 0;  // clear flag

    	          // optional: debounce simple guard
    	          static uint32_t last_btn_time = 0;
    	          uint32_t now_btn = HAL_GetTick();
    	          if (now_btn - last_btn_time > 200)   // 200 ms debounce
    	          {
    	              last_btn_time = now_btn;

    	              BLE_I2C_Send(temp, hum, dist);   // send current values via I2C to XIAO
    	              printf("Button: forced BLE send\r\n");
    	          }
    	      }


    	      // --- your existing LoRa packing & AT+MSGHEX ---
    	      int16_t t_int = (int16_t)(temp * 100);
    	      int16_t h_int = (int16_t)(hum  * 100);

    	      uint8_t p[4];
    	      p[0] = t_int >> 8;
    	      p[1] = t_int & 0xFF;
    	      p[2] = h_int >> 8;
    	      p[3] = h_int & 0xFF;

    	      char cmd[64];
    	      sprintf(cmd, "AT+MSGHEX=%02X%02X%02X%02X\r\n",
    	              p[0], p[1], p[2], p[3]);

    	      printf("Sending LoRa: %s\r\n", cmd);
    	      LoRa_Send(cmd);
    	      LoRa_ReadReply();
    	  }

      else
      {
          printf("SHT40 read error!\r\n");
      }

      HAL_Delay(5000); // 5 s between uplinks (for testing)
  }
}



/* Peripheral initialization functions generated by CubeMX below */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

    HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00707CBB; // 100 kHz standard mode
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;  // Wio-E5 baud rate
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
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
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};     //PB0 Configuration

    // LD3
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

    // HC-SR04 Trigger: PA4 output
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // HC-SR04 Echo: PA7 input
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //PB0 Configuration
    // Button PB0: input with pull-up + EXTI on falling edge
      GPIO_InitStruct.Pin  = GPIO_PIN_0;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      // Enable EXTI0 interrupt in NVIC
      HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);


}


void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
