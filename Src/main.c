/**
  ******************************************************************************
  * @file    main.c
  * @brief   VL53L0X 激光测距与移动物体检测主程序（STM32F103 + FreeRTOS）
  *
  *   三个任务：
  *     DataAcqTask  —— 数据采集：周期读取 VL53L0X 距离，入队
  *     CalcTask     —— 距离计算：滑动平均滤波 + 移动检测，入队
  *     DisplayTask  —— 屏幕显示：出队并通过串口显示距离与移动标志
  *
  *   硬件：I2C1(PB6/PB7) 接 VL53L0X；USART1(PA9/PA10) 输出，115200。
  ******************************************************************************
  */
#include "stm32f1xx_hal.h"
#include "vl53l0x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

/* 外设句柄 */
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart1;

/* 移动检测参数 */
#define WINDOW_SIZE      5     /* 滑动平均窗口长度 */
#define MOVE_THRESHOLD   50    /* 与历史均值差超过 50mm 判定为移动 */

/* 任务间传递的结果 */
typedef struct {
    uint16_t distance_mm;
    uint8_t  moving;            /* 0=静止 1=移动 */
} Result_t;

/* 队列 */
QueueHandle_t rawQueue;         /* 采集任务 -> 计算任务：uint16_t */
QueueHandle_t resultQueue;      /* 计算任务 -> 显示任务：Result_t */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void Error_Handler(void);

static void DataAcqTask(void *arg);
static void CalcTask(void *arg);
static void DisplayTask(void *arg);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    if (VL53L0X_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    rawQueue    = xQueueCreate(16, sizeof(uint16_t));
    resultQueue = xQueueCreate(16, sizeof(Result_t));
    if (rawQueue == NULL || resultQueue == NULL) {
        Error_Handler();
    }

    xTaskCreate(DataAcqTask, "acq",  256, NULL, 3, NULL);
    xTaskCreate(CalcTask,    "calc", 256, NULL, 2, NULL);
    xTaskCreate(DisplayTask, "disp", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) { }
}

/* 数据采集任务：周期测距 */
static void DataAcqTask(void *arg)
{
    (void)arg;
    uint16_t dist;

    for (;;) {
        dist = VL53L0X_ReadRangeSingle();
        if (dist != 0xFFFF) {
            xQueueSend(rawQueue, &dist, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* 距离计算任务：滑动平均 + 移动检测 */
static void CalcTask(void *arg)
{
    (void)arg;
    uint16_t history[WINDOW_SIZE] = {0};
    uint8_t  idx = 0, count = 0;
    uint32_t sum = 0;
    int32_t  prev_avg = 0;
    uint16_t dist;

    for (;;) {
        if (xQueueReceive(rawQueue, &dist, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 滑动窗口维护 */
        sum += dist - history[idx];
        history[idx] = dist;
        idx = (idx + 1) % WINDOW_SIZE;
        if (count < WINDOW_SIZE) {
            count++;
        }

        /* 移动检测：与上一次均值比较 */
        int32_t avg = (int32_t)(sum / count);
        Result_t r;
        r.distance_mm = (uint16_t)avg;
        r.moving = 0;
        if (count == WINDOW_SIZE) {
            int32_t diff = avg - prev_avg;
            r.moving = (diff > MOVE_THRESHOLD || diff < -MOVE_THRESHOLD) ? 1 : 0;
        }
        prev_avg = avg;

        xQueueSend(resultQueue, &r, 0);
    }
}

/* 屏幕显示任务：串口输出距离与移动标志 */
static void DisplayTask(void *arg)
{
    (void)arg;
    Result_t r;
    char buf[64];

    for (;;) {
        if (xQueueReceive(resultQueue, &r, portMAX_DELAY) == pdTRUE) {
            int len = snprintf(buf, sizeof(buf),
                               "dist=%u mm  moving=%s\r\n",
                               r.distance_mm, r.moving ? "YES" : "NO");
            HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
        }
    }
}

/* ==================== HAL 初始化（可由 CubeMX 生成替换） ==================== */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* HSE 8MHz -> PLL x9 -> 72MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* USART1：PA9(TX)/PA10(RX)，复用推挽 */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
    }
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
