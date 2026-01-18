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
#define SHT40_ADDR (0x44 << 1)      // HAL expects 8-bit address
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
RTC_HandleTypeDef hrtc;

volatile uint8_t btn_ble_trigger = 0; // PB0 EXTI
volatile uint8_t rtc_wakeup_flag = 0; // RTC wakeup

static uint8_t rtc_tick_count = 0;

static uint32_t last_btn_tick = 0;


uint8_t sht40_data[6];
uint8_t lora_rx[64];

/* cached last sensor values (BLE uses these) */
static float last_temp = 0.0f;
static float last_hum  = 0.0f;
static float last_dist = 0.0f;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);

/* USER prototypes */
int  SHT40_Read(float *temperature, float *humidity);
void LoRa_Send(const char *s);
void LoRa_ReadReply(void);
int  LoRa_ReadReply_Long(void);
void BLE_I2C_Send(float temp, float hum, float dist_cm);

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

// EXTI callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        uint32_t now = HAL_GetTick();

        // 250 ms debounce
        if ((now - last_btn_tick) > 250)
        {
            last_btn_tick = now;
            btn_ble_trigger = 1;
        }
    }
}


// RTC wake callback
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_)
{
    (void)hrtc_;
    rtc_wakeup_flag = 1;
}

// BLE I2C send
void BLE_I2C_Send(float temp, float hum, float dist_cm)
{
    int16_t t_x100 = (int16_t)(temp * 100.0f);
    int16_t h_x100 = (int16_t)(hum  * 100.0f);
    int16_t d_x10  = (int16_t)(dist_cm * 10.0f);

    uint8_t buf[7];
    buf[0] = 0x55;
    buf[1] = (t_x100 >> 8) & 0xFF;
    buf[2] =  t_x100       & 0xFF;
    buf[3] = (h_x100 >> 8) & 0xFF;
    buf[4] =  h_x100       & 0xFF;
    buf[5] = (d_x10  >> 8) & 0xFF;
    buf[6] =  d_x10        & 0xFF;

    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, XIAO_I2C_ADDR, buf, sizeof(buf), 100);

    if (st == HAL_OK) {
        printf("BLE OK: T=%.2f H=%.2f D=%.1f\r\n", temp, hum, dist_cm);
    } else {
        printf("BLE FAIL (st=%d)\r\n", st);
    }
}

// SHT40 read
int SHT40_Read(float *temperature, float *humidity)
{
    uint8_t cmd = 0xFD;
    if (HAL_I2C_Master_Transmit(&hi2c1, SHT40_ADDR, &cmd, 1, 100) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);

    if (HAL_I2C_Master_Receive(&hi2c1, SHT40_ADDR, sht40_data, 6, 100) != HAL_OK)
        return HAL_ERROR;

    uint16_t t_raw = (sht40_data[0] << 8) | sht40_data[1];
    uint16_t h_raw = (sht40_data[3] << 8) | sht40_data[4];

    *temperature = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    *humidity    = 100.0f * ((float)h_raw / 65535.0f);

    return HAL_OK;
}

// LoRa send
void LoRa_Send(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 1000);
}

// LoRa short reply
void LoRa_ReadReply(void)
{
    memset(lora_rx, 0, sizeof(lora_rx));
    HAL_UART_Receive(&huart1, lora_rx, sizeof(lora_rx) - 1, 500);
    printf("LoRa reply: %s\r\n", lora_rx);
}

// LoRa long reply (join success detection)
int LoRa_ReadReply_Long(void)
{
    memset(lora_rx, 0, sizeof(lora_rx));

    uint32_t start = HAL_GetTick();
    uint16_t idx = 0;
    uint8_t ch;

    while ((HAL_GetTick() - start) < 15000)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 50) == HAL_OK)
        {
            if (idx < sizeof(lora_rx) - 1) lora_rx[idx++] = ch;
            HAL_UART_Transmit(&huart2, &ch, 1, 10);
        }
    }

    lora_rx[idx] = 0;
    printf("\r\n[LoRa long reply buffer]\r\n%s\r\n", lora_rx);

    if (strstr((char*)lora_rx, "Network joined") != NULL ||
        strstr((char*)lora_rx, "Join Succeeded") != NULL)
        return 1;

    return 0;
}

// DWT init
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { }
}

float HCSR04_ReadDistance_cm(void)
{
    uint32_t start_cycles, echo_cycles;
    uint32_t timeout_us = 30000;

    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);

    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    uint32_t t0 = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_RESET)
    {
        if ((DWT->CYCCNT - t0) > timeout_us * (SystemCoreClock / 1000000U))
            return -1.0f;
    }

    start_cycles = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        if ((DWT->CYCCNT - start_cycles) > timeout_us * (SystemCoreClock / 1000000U))
            break;
    }
    echo_cycles = DWT->CYCCNT - start_cycles;

    float echo_us = (float)echo_cycles / (SystemCoreClock / 1000000.0f);
    return echo_us / 58.0f;
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    HAL_SuspendTick();   // <--- ADD THIS


    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_RTC_Init();

    DWT_Init();

    HAL_Delay(1000);

    printf("Booted. STOP2 + RTC 15s + PB0 BLE\r\n");

    LoRa_Send("AT+ID\r\n");
    LoRa_ReadReply_Long();

    HAL_Delay(5000);

    LoRa_Send("AT+MODE=LWOTAA\r\n");
    HAL_Delay(300);
    LoRa_ReadReply();

    LoRa_Send("AT+DR=EU868\r\n");
    HAL_Delay(300);
    LoRa_ReadReply();

    int joined_ok = 0;
    uint32_t last_join_attempt = 0;

    float temp = 0.0f, hum = 0.0f;

    while (1)
    {
        if (!rtc_wakeup_flag && !btn_ble_trigger)
        {
            __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
            __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

            HAL_SuspendTick();
            HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
            HAL_ResumeTick();

            SystemClock_Config();
            continue;
        }

        if (rtc_wakeup_flag)
                {
                    rtc_wakeup_flag = 0;

                    uint32_t now = HAL_GetTick();

                    if (!joined_ok)
                    {
                        LoRa_Send("AT+JOIN\r\n");
                        joined_ok = LoRa_ReadReply_Long();
                    }


                    if (SHT40_Read(&temp, &hum) == HAL_OK)
                    {
                        last_temp = temp;
                        last_hum  = hum;
                        last_dist = HCSR04_ReadDistance_cm();

                        if (joined_ok)
                        {
                            int16_t t_int = (int16_t)(last_temp * 100);
                            int16_t h_int = (int16_t)(last_hum  * 100);
                            int16_t d_int = (int16_t)(last_dist * 10);

                            uint8_t p0 = (uint8_t)(t_int >> 8);
                            uint8_t p1 = (uint8_t)(t_int & 0xFF);
                            uint8_t p2 = (uint8_t)(h_int >> 8);
                            uint8_t p3 = (uint8_t)(h_int & 0xFF);
                            uint8_t p4 = (uint8_t)(d_int >> 8);
                            uint8_t p5 = (uint8_t)(d_int & 0xFF);

                            char cmd[64];
                            sprintf(cmd, "AT+MSGHEX=%02X%02X%02X%02X%02X%02X\r\n", p0,p1,p2,p3,p4,p5);

                            LoRa_Send(cmd);
                            LoRa_ReadReply();
                        }
                    }
                }

        // BLE event: use cached values ONLY
        if (btn_ble_trigger)
        {
            btn_ble_trigger = 0;
            BLE_I2C_Send(last_temp, last_hum, last_dist);
        }

        // RTC tick: measure sensors + join/tx

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
    hi2c1.Init.Timing = 0x00707CBB;
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
    huart1.Init.BaudRate = 9600;
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

static void MX_RTC_Init(void)
{
    __HAL_RCC_RTC_ENABLE();

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv  = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }

    /* STM32L412: 4-arg API, AutoClr NOT exposed → use 0 */
    if (HAL_RTCEx_SetWakeUpTimer_IT(
            &hrtc,
            (15U * 32768U) / 16U,
            RTC_WAKEUPCLOCK_RTCCLK_DIV16,
            0) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
}



static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
