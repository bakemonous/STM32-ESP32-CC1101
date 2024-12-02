/**
  ******************************************************************************
  * @file           : onewire.c
  * @brief          : 1-Wire driver
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "onewire.h"
/* Declarations and definitions ----------------------------------------------*/
/* Functions -----------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static void SetBaudrate(UART_HandleTypeDef *huart, uint32_t baudrate)
{
  uint32_t pclk = 0;
  huart->Init.BaudRate = baudrate;
#if defined(USART6) && defined(UART9) && defined(UART10)
    if ((huart->Instance == USART1) || (huart->Instance == USART6) ||
        (huart->Instance == UART9)  || (huart->Instance == UART10))
    {
      pclk = HAL_RCC_GetPCLK2Freq();
    }
#elif defined(USART6)
    if ((huart->Instance == USART1) || (huart->Instance == USART6))
    {
      pclk = HAL_RCC_GetPCLK2Freq();
    }
#else
    if (huart->Instance == USART1)
    {
      pclk = HAL_RCC_GetPCLK2Freq();
    }
#endif /* USART6 */
    else
    {
      pclk = HAL_RCC_GetPCLK1Freq();
    }
  if (huart->Init.OverSampling == UART_OVERSAMPLING_8)
  {
    huart->Instance->BRR = UART_BRR_SAMPLING8(pclk, huart->Init.BaudRate);
  }
  else
  {
    huart->Instance->BRR = UART_BRR_SAMPLING16(pclk, huart->Init.BaudRate);
  }
}
/*----------------------------------------------------------------------------*/
uint8_t OneWire_ProcessBit(UART_HandleTypeDef *huart, uint8_t bit)
{
  uint8_t txData = 0xFF;
  uint8_t rxData = 0x00;
  if (bit == 0)
  {
    txData = 0x00;
  }
  HAL_UART_Transmit(huart, &txData, 1, ONEWIRE_UART_TIMEOUT);
  HAL_UART_Receive(huart, &rxData, 1, ONEWIRE_UART_TIMEOUT);
  return rxData;
}
/*----------------------------------------------------------------------------*/
uint8_t OneWire_ProcessByte(UART_HandleTypeDef *huart, uint8_t byte)
{
  uint8_t rxByte = 0x00;
  for (uint8_t i = 0; i < ONEWIRE_BITS_NUM; i++)
  {
    uint8_t txBit = (byte >> i) & 0x01;
    uint8_t rxBit = 0;
    uint8_t tempRxData = OneWire_ProcessBit(huart, txBit);
    if (tempRxData == 0xFF)
    {
      rxBit = 1;
    }
    rxByte |= (rxBit << i);
  }
  return rxByte;
}
/*----------------------------------------------------------------------------*/
ONEWIRE_Status OneWire_Reset(UART_HandleTypeDef *huart)
{
  ONEWIRE_Status status = ONEWIRE_OK;
  uint8_t txByte = ONEWIRE_RESET_BYTE;
  uint8_t rxByte = 0x00;
  SetBaudrate(huart, ONEWIRE_RESET_BAUDRATE);
  HAL_UART_Transmit(huart, &txByte, 1, ONEWIRE_UART_TIMEOUT);
  HAL_UART_Receive(huart, &rxByte, 1, ONEWIRE_UART_TIMEOUT);
  SetBaudrate(huart, ONEWIRE_BAUDRATE);
  if (rxByte == txByte)
  {
    status = ONEWIRE_ERROR;
  }
  return status;
}
/*----------------------------------------------------------------------------*/