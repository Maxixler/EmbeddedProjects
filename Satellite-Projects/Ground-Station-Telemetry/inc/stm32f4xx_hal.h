#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H

#include <stdint.h>
#include <stdbool.h>

#define __IO volatile

/* Exported types */
typedef enum {
    HAL_OK = 0x00,
    HAL_ERROR = 0x01,
    HAL_BUSY = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

/* GPIO definitions */
typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET   = 1
} GPIO_PinState;

#define GPIO_PIN_0                 ((uint16_t)0x0001)
#define GPIO_PIN_1                 ((uint16_t)0x0002)
#define GPIO_PIN_2                 ((uint16_t)0x0004)
#define GPIO_PIN_3                 ((uint16_t)0x0008)
#define GPIO_PIN_4                 ((uint16_t)0x0010)
#define GPIO_PIN_5                 ((uint16_t)0x0020)
#define GPIO_PIN_6                 ((uint16_t)0x0040)
#define GPIO_PIN_7                 ((uint16_t)0x0080)
#define GPIO_PIN_8                 ((uint16_t)0x0100)
#define GPIO_PIN_9                 ((uint16_t)0x0200)
#define GPIO_PIN_10                ((uint16_t)0x0400)
#define GPIO_PIN_11                ((uint16_t)0x0800)
#define GPIO_PIN_12                ((uint16_t)0x1000)
#define GPIO_PIN_13                ((uint16_t)0x2000)
#define GPIO_PIN_14                ((uint16_t)0x4000)
#define GPIO_PIN_15                ((uint16_t)0x8000)

typedef enum {
    GPIO_MODE_INPUT         = 0x00000000U,
    GPIO_MODE_OUTPUT_PP     = 0x00000001U,
    GPIO_MODE_OUTPUT_OD     = 0x00000002U,
    GPIO_MODE_AF_PP         = 0x00000004U,
    GPIO_MODE_AF_OD         = 0x00000005U,
    GPIO_MODE_ANALOG        = 0x00000008U,
    GPIO_MODE_IT_RISING     = 0x00001001U,
    GPIO_MODE_IT_FALLING    = 0x00001002U,
    GPIO_MODE_IT_RISING_FALLING = 0x00001003U
} GPIOMode_TypeDef;

#define GPIO_NOPULL           ((uint32_t)0x00000000U)
#define GPIO_PULLUP           ((uint32_t)0x00000001U)
#define GPIO_PULLDOWN         ((uint32_t)0x00000002U)

typedef enum {
    GPIO_SPEED_FREQ_LOW     = 0x00000000U,
    GPIO_SPEED_FREQ_MEDIUM  = 0x00000001U,
    GPIO_SPEED_FREQ_HIGH    = 0x00000002U,
    GPIO_SPEED_FREQ_VERY_HIGH = 0x00000003U
} GPIOSpeed_TypeDef;

typedef struct {
    uint32_t Pin;
    GPIOMode_TypeDef Mode;
    uint32_t Pull;
    GPIOSpeed_TypeDef Speed;
} GPIO_InitTypeDef;

/* UART handle definition */
typedef struct {
    uint32_t Instance;          /* UART registers base address    */
    uint32_t Init;              /* UART communication parameters  */
    uint8_t *pTxBuffPtr;        /* Pointer to UART Tx transfer Buffer */
    uint16_t TxXferSize;        /* UART Tx Transfer size          */
    __IO uint16_t TxXferCount;  /* UART Tx Transfer Counter       */
    uint8_t *pRxBuffPtr;        /* Pointer to UART Rx transfer Buffer */
    uint16_t RxXferSize;        /* UART Rx Transfer size          */
    __IO uint16_t RxXferCount;  /* UART Rx Transfer Counter       */
    void (*RxISR)(void);        /* UART Rx ISR                    */
} UART_HandleTypeDef;

/* Exported constants */
#define USART2                            ((uint32_t)0x40004400U)
#define GPIOD                             ((uint32_t)0x40020C00U)
#define GPIOA                             ((uint32_t)0x40020000U)

/* Exported functions */
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
void HAL_GPIO_WritePin(uint32_t GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(uint32_t GPIOx, uint16_t GPIO_Pin);
void HAL_Delay(uint32_t Delay);
void __HAL_RCC_GPIOD_CLK_ENABLE(void);
void __HAL_RCC_GPIOA_CLK_ENABLE(void);
void __HAL_RCC_USART2_CLK_ENABLE(void);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);

#endif /* STM32F4XX_HAL_H */