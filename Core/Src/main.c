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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CTRL_CODE 0b1010
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
enum TYPE
{
    CHAR = 0,
};

enum RW_COMMANDS
{
    WRITE = 0,
    READ = 1
};

enum BLOCK_SELECT
{
    BLK_ZERO = 0b0,
    BLK_ONE = 0b1
};

// These values correspond to what lines are pulled high on the EEPROM chip inputs A0 and A1
enum CHIP_SELECT
{
    CHIP_ZERO = 0x0,
    CHIP_ONE = 0x1,
    CHIP_TWO = 0x2,
    CHIP_THREE = 0x3
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_RNG_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void USR_LogMessage(UART_HandleTypeDef *uart_port, char *message, int size);

static void USR_LogVariable_char(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname, char variable);
static void USR_LogVariable_ptrChar(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                    char *variable, int length);
static void USR_LogVariable_short(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                  short variable);
static void USR_LogVariable_ptrShort(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                     short *variable, int length);
static void USR_LogVariable_int(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname, int variable);
static void USR_LogVariable_ptrInt(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                   int *variable, int length);
static void USR_LogVariable_long(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname, long variable);
static void USR_LogVariable_ptrLong(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                    long *variable, int length);
static void USR_LogVariable_longLong(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                     long long variable);
static void USR_LogVariable_ptrLongLong(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                        long long *variable, int length);
static void USR_LogVariable_float(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                  float variable);
static void USR_LogVariable_ptrFloat(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                     float *variable, int length);
static void USR_LogVariable_double(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                   double variable);
static void USR_LogVariable_ptrDouble(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                      double *variable, int length);

uint8_t control_byte_builder(uint8_t control_code, uint8_t block_select, uint8_t chip_select, uint8_t read_write);
void WriteByteRequestHandler(I2C_HandleTypeDef I2C_Line, uint8_t hb_address, uint8_t lb_address, uint8_t data);
void WritePageRequestHandler(I2C_HandleTypeDef I2C_Line, uint8_t hb_address, uint8_t lb_address, uint8_t data[128]);
uint8_t CurrentAddressReadHandler(I2C_HandleTypeDef I2C_Line);
uint8_t RandomAddressReadHandler(I2C_HandleTypeDef I2C_Line);
uint8_t SequentialAddressReadHandler(I2C_HandleTypeDef I2C_Line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */
    uint8_t TxBuffer[50];
    uint8_t RxBuffer[50];

    for (int i = 0; i < 50; ++i)
    {
        TxBuffer[i] = 0;
        RxBuffer[i] = 0;
    }

    (void)RxBuffer[0];
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
    MX_I2C3_Init();
    MX_RNG_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    char ptrChar[5] = "HIII";
    short ptrShort[5] = {1, 2, 3, 4, 5};
    int ptrInt[5] = {1, 2, 3, 4, 5};
    long ptrLong[5] = {1, 2, 3, 4, 5};
    long long ptrLongLong[5] = {1, 2, 3, 4, 5};
    float ptrFloat[5] = {1, 2, 3, 4, 5};
    double ptrDouble[5] = {1, 2, 3, 4, 5};

    uint32_t float_pos = 0x70008040;
    uint32_t float_neg = 0xD2008040;

    uint64_t double_pos = 0x70000000D2008040;
    uint64_t double_neg = 0x80000000D2008040;

    USR_LogVariable_char(&huart3, "INT8_MAX", 7, INT8_MAX);
    USR_LogVariable_char(&huart3, "INT8_MIN", 7, INT8_MIN);
    USR_LogVariable_short(&huart3, "INT16_MAX", 7, INT16_MAX);
    USR_LogVariable_short(&huart3, "INT16_MIN", 7, INT16_MIN);
    USR_LogVariable_int(&huart3, "INT32_MAX", 7, INT32_MAX);
    USR_LogVariable_int(&huart3, "INT32_MIN", 7, INT32_MIN);
    USR_LogVariable_long(&huart3, "INT32_MAX", 7, INT32_MAX);
    USR_LogVariable_long(&huart3, "INT32_MIN", 7, INT32_MIN);
    USR_LogVariable_longLong(&huart3, "INT64_MAX", 7, INT64_MAX);
    USR_LogVariable_longLong(&huart3, "INT64_MIN", 7, INT64_MIN);
    USR_LogVariable_float(&huart3, "float_pos", 7, *(float *)&float_pos);
    USR_LogVariable_float(&huart3, "float_neg", 7, *(float *)&float_neg);
    USR_LogVariable_double(&huart3, "double_pos", 7, *(double *)&float_pos);
    USR_LogVariable_double(&huart3, "double_neg", 7, *(double *)&float_neg);

    USR_LogVariable_ptrChar(&huart3, "ptr_char", 7, ptrChar, 5);
    USR_LogVariable_ptrShort(&huart3, "ptr_short", 7, ptrShort, 5);
    USR_LogVariable_ptrInt(&huart3, "ptr_int", 7, ptrInt, 5);
    USR_LogVariable_ptrLong(&huart3, "ptr_long", 7, ptrLong, 5);
    USR_LogVariable_ptrLongLong(&huart3, "ptr_long_long", 7, ptrLongLong, 5);
    USR_LogVariable_ptrFloat(&huart3, "ptr_float", 7, ptrFloat, 5);
    USR_LogVariable_ptrDouble(&huart3, "ptr_float", 7, ptrDouble, 5);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        HAL_Delay(500);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_GPIO_TogglePin(LED_PC10_GPIO_Port, LED_PC10_Pin);
        HAL_GPIO_TogglePin(LED_PC11_GPIO_Port, LED_PC11_Pin);
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
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

    /* USER CODE BEGIN I2C3_Init 0 */

    /* USER CODE END I2C3_Init 0 */

    /* USER CODE BEGIN I2C3_Init 1 */

    /* USER CODE END I2C3_Init 1 */
    hi2c3.Instance = I2C3;
    hi2c3.Init.Timing = 0x20E16892;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C3_Init 2 */

    /* USER CODE END I2C3_Init 2 */
}

/**
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void)
{

    /* USER CODE BEGIN RNG_Init 0 */

    /* USER CODE END RNG_Init 0 */

    /* USER CODE BEGIN RNG_Init 1 */

    /* USER CODE END RNG_Init 1 */
    hrng.Instance = RNG;
    if (HAL_RNG_Init(&hrng) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RNG_Init 2 */

    /* USER CODE END RNG_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 399;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 49999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
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
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
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
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, OUTPUT_PC5_Pin | OUTPUT_PC6_Pin | OUTPUT_PC8_Pin | LED_PC10_Pin | LED_PC11_Pin,
                      GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : OUTPUT_PC5_Pin OUTPUT_PC6_Pin OUTPUT_PC8_Pin LED_PC10_Pin
                             LED_PC11_Pin */
    GPIO_InitStruct.Pin = OUTPUT_PC5_Pin | OUTPUT_PC6_Pin | OUTPUT_PC8_Pin | LED_PC10_Pin | LED_PC11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void USR_LogMessage(UART_HandleTypeDef *uart_port, char *message, int size)
{
    HAL_UART_Transmit(uart_port, message, size, 10);
}
static void USR_LogVariable_char(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname, char variable)
{
    char var[10] = {0};

    HAL_UART_Transmit(uart_port, variable_name, sizeof_varname, 10);
    HAL_UART_Transmit(uart_port, ": ", 2, 10);
    sprintf(var, "%c (0x%02X)\n", variable, variable);
    HAL_UART_Transmit(uart_port, var, 10, 10);
}
static void USR_LogVariable_ptrChar(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                    char *variable, int length)
{
    char temp[6] = {0};
    int i = 0;

    HAL_UART_Transmit(uart_port, variable_name, sizeof_varname, 10);
    HAL_UART_Transmit(uart_port, ": [ ", 4, 10);
    for (; i < length; i++)
    {
        sprintf(temp, "0x%02X ", variable[i]);
        HAL_UART_Transmit(uart_port, temp, 5, 10);
    }
    HAL_UART_Transmit(uart_port, "]\n", 2, 10);
}
static void USR_LogVariable_short(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                  short variable)
{
    char var[17] = {0};

    HAL_UART_Transmit(uart_port, variable_name, sizeof_varname, 10);
    HAL_UART_Transmit(uart_port, ": ", 2, 10);
    sprintf(var, "%hd (0x%02X)\n", variable, variable);
    HAL_UART_Transmit(uart_port, var, 16, 10);
}
static void USR_LogVariable_ptrShort(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                     short *variable, int length)
{
    char temp[8] = {0};
    int i = 0;

    HAL_UART_Transmit(uart_port, variable_name, sizeof_varname, 10);
    HAL_UART_Transmit(uart_port, ": [ ", 4, 10);
    for (; i < length; i++)
    {
        sprintf(temp, "0x%04X ", variable[i]);
        HAL_UART_Transmit(uart_port, temp, 7, 10);
    }
    HAL_UART_Transmit(uart_port, "]\n", 2, 10);
}
static void USR_LogVariable_int(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname, int variable)
{
    char var[26] = {0};

    HAL_UART_Transmit(uart_port, variable_name, sizeof_varname, 10);
    HAL_UART_Transmit(uart_port, ": ", 2, 10);
    sprintf(var, "%d (0x%04X)\n", variable, variable);
    HAL_UART_Transmit(uart_port, var, 25, 10);
}
static void USR_LogVariable_ptrInt(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                   int *variable, int length)
{
    char var[26] = {0};
    int i = 0;

    HAL_UART_Transmit(uart_port, variable_name, sizeof_varname, 10);
    HAL_UART_Transmit(uart_port, ": [ ", 4, 10);
    for (; i < length, i++)
        sprintf(var, "%d \n", variable, variable);
    HAL_UART_Transmit(uart_port, var, 25, 10);
}
static void USR_LogVariable_long(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname, long variable)
{
}
static void USR_LogVariable_ptrLong(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                    long *variable, int length)
{
}
static void USR_LogVariable_longLong(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                     long long variable)
{
}
static void USR_LogVariable_ptrLongLong(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                        long long *variable, int length)
{
}
static void USR_LogVariable_float(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                  float variable)
{
}
static void USR_LogVariable_ptrFloat(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                     float *variable, int length)
{
}
static void USR_LogVariable_double(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                   double variable)
{
}
static void USR_LogVariable_ptrDouble(UART_HandleTypeDef *uart_port, char *variable_name, int sizeof_varname,
                                      double *variable, int length)
{
}

uint8_t control_byte_builder(uint8_t control_code, uint8_t block_select, uint8_t chip_select, uint8_t read_write)
{
}
void WriteByteRequestHandler(I2C_HandleTypeDef I2C_Line, uint8_t hb_address, uint8_t lb_address, uint8_t data)
{
}
void WritePageRequestHandler(I2C_HandleTypeDef I2C_Line, uint8_t hb_address, uint8_t lb_address, uint8_t data[128])
{
}
uint8_t CurrentAddressReadHandler(I2C_HandleTypeDef I2C_Line)
{
}
uint8_t RandomAddressReadHandler(I2C_HandleTypeDef I2C_Line)
{
}
uint8_t SequentialAddressReadHandler(I2C_HandleTypeDef I2C_Line)
{
}
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

#ifdef USE_FULL_ASSERT
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
