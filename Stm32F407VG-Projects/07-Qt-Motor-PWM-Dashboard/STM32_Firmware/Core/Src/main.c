/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Qt Motor PWM Dashboard İletişim Uygulaması
  *                   TIM4 ile PWM okuma/yazma simülasyonu ve UART üzerinden
  *                   Qt Arayüzüne PWM:25,50,75,100\n formatında aktarımı.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
// TIM_HandleTypeDef htim4; // (Kendi projenizde aktif edebilirsiniz)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
// static void MX_TIM4_Init(void); // (Kendi projenizde aktif edebilirsiniz)

/* USER CODE BEGIN 0 */
// GCC sprintf icin __io_putchar (UART Tx)
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 0 */

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  // MX_TIM4_Init();
  // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* USER CODE BEGIN 2 */
  int pwm1 = 0;
  int pwm2 = 25;
  int pwm3 = 50;
  int pwm4 = 75;
  int dir = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. Motorların o anki PWM değerlerini okuyun (Gerçek Proje)
      // pwm1 = TIM4->CCR1;
      // pwm2 = TIM4->CCR2;
      // pwm3 = TIM4->CCR3;
      // pwm4 = TIM4->CCR4;

      // ---- DEMO DEĞER ÜRETİCİ (Gerçekte silin) ----
      pwm1 += dir; pwm2 += dir; pwm3 += dir; pwm4 += dir;
      if (pwm1 > 100 || pwm1 < 0) dir = -dir;
      // --------------------------------------------

      // 2. Qt Arayüzüne veriyi beklenen formatta gönder (Örn: PWM:50,25,80,100\n)
      printf("PWM:%d,%d,%d,%d\n", pwm1, pwm2, pwm3, pwm4);

      // 3. 100ms yenileme hızı (Qt tarafındaki simülasyon frekansıyla ayni)
      HAL_Delay(100);

    /* USER CODE END WHILE */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  // Standart CubeIDE Config Kodları buraya gelecek
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200; // Qt Arayüzü ile aynı BaudRate!
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
