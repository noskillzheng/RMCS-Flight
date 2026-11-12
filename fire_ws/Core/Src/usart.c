/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#define UART1_RX_RING_SIZE 64U

static volatile uint8_t s_uart1_rx_ring[UART1_RX_RING_SIZE];
static volatile uint8_t s_uart1_rx_head = 0U;
static volatile uint8_t s_uart1_rx_tail = 0U;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
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
  s_uart1_rx_head = 0U;
  s_uart1_rx_tail = 0U;
  (void)USART1->SR;
  (void)USART1->DR;
  SET_BIT(USART1->CR1, USART_CR1_RXNEIE);
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable & clean reset (ensures clean state after ISP Go jump) */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE BEGIN USART1_MspInit 1 */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void USART1_RxIrqHandler(void)
{
  uint32_t sr = USART1->SR;
  if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
  {
    uint8_t data = (uint8_t)(USART1->DR & 0xFFU);
    if ((sr & (USART_SR_NE | USART_SR_FE | USART_SR_PE)) == 0U && (sr & USART_SR_RXNE) != 0U)
    {
      uint8_t next_head = (uint8_t)((s_uart1_rx_head + 1U) & (UART1_RX_RING_SIZE - 1U));
      if (next_head != s_uart1_rx_tail)
      {
        s_uart1_rx_ring[s_uart1_rx_head] = data;
        s_uart1_rx_head = next_head;
      }
    }
  }
}

uint8_t USART1_TryReadByte(uint8_t *out_byte)
{
  uint8_t tail = s_uart1_rx_tail;
  if (tail == s_uart1_rx_head)
  {
    return 0U;
  }
  *out_byte = s_uart1_rx_ring[tail];
  s_uart1_rx_tail = (uint8_t)((tail + 1U) & (UART1_RX_RING_SIZE - 1U));
  return 1U;
}

void USART1_WriteBytes(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0U; i < len; i++)
  {
    uint32_t start = HAL_GetTick();
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
      if ((HAL_GetTick() - start) > 10U)
      {
        return;
      }
    }
    USART1->DR = data[i];
  }
}
/* USER CODE END 1 */
