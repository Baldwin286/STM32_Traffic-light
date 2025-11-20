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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO_PIN_6

#define LED_YELLOW_PORT GPIOC
#define LED_YELLOW_PIN  GPIO_PIN_13

#define LED_RED_PORT GPIOA
#define LED_RED_PIN  GPIO_PIN_5

#define BUTTON_PORT GPIOB
#define BUTTON_PIN GPIO_PIN_12

#define GREEN_MIN 15
#define GREEN_MAX 45
#define YELLOW_MIN 3
#define YELLOW_MAX 5
#define RED_MIN 15
#define RED_MAX 90
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint16_t time_green = 5;
static uint16_t time_yellow = 3;
static uint16_t time_red = 5;
static uint8_t just_switched_mode2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum { MODE_1 = 1, MODE_2 = 2 } device_mode_t;
static device_mode_t current_mode = MODE_1;

typedef enum { PHASE_GREEN = 0, PHASE_YELLOW, PHASE_RED } phase_t;
static phase_t curr_phase = PHASE_GREEN;
static uint16_t phase_remaining = 0;

static uint8_t yellow_state = 0;

#define RX_BUF_SIZE 64
static uint8_t rx_byte;
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_idx = 0;

void set_leds(uint8_t g, uint8_t y, uint8_t r);
void apply_mode(device_mode_t m);
void toggle_mode(void);
void send_uart(const char *s);
void process_cmd(const char *s);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim != &htim2) return;

    if (current_mode == MODE_1) {
        yellow_state = !yellow_state;
        set_leds(0, yellow_state, 0);
        return;
    }

    if (just_switched_mode2) {
        just_switched_mode2 = 0;
        curr_phase = PHASE_GREEN;
        phase_remaining = time_green;
        set_leds(1,0,0);
        return;
    }

    if (phase_remaining > 0) phase_remaining--;

    if (phase_remaining == 0) {
        switch (curr_phase) {
            case PHASE_GREEN:
                curr_phase = PHASE_YELLOW;
                phase_remaining = time_yellow;
                break;
            case PHASE_YELLOW:
                curr_phase = PHASE_RED;
                phase_remaining = time_red;
                break;
            case PHASE_RED:
                curr_phase = PHASE_GREEN;
                phase_remaining = time_green;
                break;
        }

        switch (curr_phase) {
            case PHASE_GREEN:  set_leds(1,0,0); break;
            case PHASE_YELLOW: set_leds(0,1,0); break;
            case PHASE_RED:    set_leds(0,0,1); break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BUTTON_PIN) {
        static uint32_t last = 0;
        uint32_t now = HAL_GetTick();

        if (now - last > 200) {
            toggle_mode();
        }
        last = now;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        if (rx_byte == '\n')
        {
            rx_buf[rx_idx] = '\0';
            if (rx_idx > 0)
            {
                process_cmd(rx_buf);
            }

            rx_idx = 0;
            memset(rx_buf, 0, RX_BUF_SIZE);
        }
        else if (rx_byte != '\r')
        {
            if (rx_idx < RX_BUF_SIZE - 1)
            {
                rx_buf[rx_idx++] = rx_byte;
            }
            else
            {
                send_uart("ERROR: Buffer Full\r\n");
                rx_idx = 0;
                memset(rx_buf, 0, RX_BUF_SIZE);
            }
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void process_cmd(const char *cmd) {
    const char *p = cmd;

    send_uart("\r\n");
    send_uart("DBG RAW: ");
    for (const char *q = p; *q; q++) {
        char h[5];
        sprintf(h, "%02X ", (unsigned char)*q);
        send_uart(h);
    }
    send_uart("\r\n");

    send_uart("DBG CMD: ");
    send_uart(p);
    send_uart("\r\n");

    // -------- CFGTIME ----------
    if (strncmp(p, "CFGTIME", 7) == 0) {
        const char *num_str = p + 7;
        int g, y, r;

        if (strlen(num_str) == 6 &&
        		isdigit((unsigned char)num_str[0]) &&
        		isdigit((unsigned char)num_str[1]) &&
        		isdigit((unsigned char)num_str[2]) &&
        		isdigit((unsigned char)num_str[3]) &&
        		isdigit((unsigned char)num_str[4]) &&
        		isdigit((unsigned char)num_str[5])) {

            g = (num_str[0]-'0')*10 + (num_str[1]-'0');
            y = (num_str[2]-'0')*10 + (num_str[3]-'0');
            r = (num_str[4]-'0')*10 + (num_str[5]-'0');

            if (g >= GREEN_MIN && g <= GREEN_MAX &&
                y >= YELLOW_MIN && y <= YELLOW_MAX &&
                r >= RED_MIN && r <= RED_MAX) {

                time_green = g;
                time_yellow = y;
                time_red = r;

                send_uart("CFGTIME OK\r\n");

                if(current_mode == MODE_2){
                    curr_phase = PHASE_GREEN;
                    phase_remaining = time_green;
                    set_leds(1,0,0);
                }
                return;
            }
        }

        send_uart("CFGTIME FAIL\r\n");
        return;
    }

    // -------- CFGMODE ----------
    if (strncmp(p, "CFGMODE", 7) == 0) {
        char m = '0';

        if (sscanf(p, "CFGMODE %c", &m) != 1 &&
            sscanf(p, "CFGMODE%c", &m) != 1) {
            send_uart("CFGMODE FAIL\r\n");
            return;
        }

        if (m=='1' || m=='2') {
            current_mode = (m=='1') ? MODE_1 : MODE_2;
            apply_mode(current_mode);

            send_uart("CFGMODE OK\r\n");

            char out[20];
            sprintf(out, "MODE%c SELECTED\r\n", m);
            send_uart(out);
            return;
        }

        send_uart("CFGMODE FAIL\r\n");
        return;
    }

    send_uart("ERROR\r\n");
}


void toggle_mode(void) {
    current_mode = (current_mode == MODE_1) ? MODE_2 : MODE_1;
    apply_mode(current_mode);

    char msg[20];
    sprintf(msg, "MODE%d SELECTED\r\n", current_mode);
    send_uart(msg);
}

void apply_mode(device_mode_t m) {
    if (m == MODE_1) {
        yellow_state = 0;
        set_leds(0,0,0);
    } else {
        HAL_TIM_Base_Stop_IT(&htim2);

        set_leds(0,0,0);

        curr_phase = PHASE_GREEN;
        phase_remaining = time_green;

        set_leds(1,0,0);

        just_switched_mode2 = 1;
        HAL_TIM_Base_Start_IT(&htim2);
    }
}



void set_leds(uint8_t g, uint8_t y, uint8_t r) {
    HAL_GPIO_WritePin(LED_GREEN_PORT,  LED_GREEN_PIN,  g ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_YELLOW_PORT, LED_YELLOW_PIN, y ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RED_PORT,    LED_RED_PIN,    r ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void send_uart(const char *s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  HAL_TIM_Base_Start_IT(&htim2);
    current_mode = MODE_1;
    apply_mode(current_mode);
    if (current_mode == MODE_2) {
        curr_phase = PHASE_GREEN;
        phase_remaining = time_green;
    }
    send_uart("UART OK\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
