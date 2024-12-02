/**
  ******************************************************************************
  * @file           : ds18b20.h
  * @brief          : DS18B20 driver
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */
#ifndef DS18B20_H
#define DS18B20_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Declarations and definitions ----------------------------------------------*/
#define DS18B20_SERIAL_NUMBER_LEN_BYTES                                     6
#define DS18B20_SERIAL_NUMBER_OFFSET_BYTES                                  1
#define DS18B20_SCRATCHPAD_T_LSB_BYTE_IDX                                   0
#define DS18B20_SCRATCHPAD_T_MSB_BYTE_IDX                                   1
#define DS18B20_SCRATCHPAD_T_LIMIT_H_BYTE_IDX                               2
#define DS18B20_SCRATCHPAD_T_LIMIT_L_BYTE_IDX                               3
#define DS18B20_SCRATCHPAD_CONFIG_BYTE_IDX                                  4
#define DS18B20_9_BITS_CONFIG                                               0x1F
#define DS18B20_10_BITS_CONFIG                                              0x3F
#define DS18B20_11_BITS_CONFIG                                              0x5F
#define DS18B20_12_BITS_CONFIG                                              0x7F
#define DS18B20_9_BITS_DELAY_MS                                             94
#define DS18B20_10_BITS_DELAY_MS                                            188
#define DS18B20_11_BITS_DELAY_MS                                            375
#define DS18B20_12_BITS_DELAY_MS                                            750
#define DS18B20_9_BITS_DATA_MASK                                            0x7F8
#define DS18B20_10_BITS_DATA_MASK                                           0x7FC
#define DS18B20_11_BITS_DATA_MASK                                           0x7FE
#define DS18B20_12_BITS_DATA_MASK                                           0x7FF
#define DS18B20_SIGN_MASK                                                   0xF800
#define DS18B20_T_STEP                                                      0.0625
#define DS18B20_READ_ROM_RX_BYTES_NUM                                       8
#define DS18B20_READ_SCRATCHPAD_RX_BYTES_NUM                                9
typedef struct DS18B20
{
  uint8_t isInitialized;
  uint8_t isConnected;
  UART_HandleTypeDef *uart;
  uint8_t serialNumber[DS18B20_SERIAL_NUMBER_LEN_BYTES];
  uint8_t temperatureLimitLow;
  uint8_t temperatureLimitHigh;
  uint8_t configRegister;
  float temperature;
} DS18B20;
typedef struct DS18B20_Command
{
  uint8_t code;
  uint8_t rxBytesNum;
  uint8_t txBytesNum;
} DS18B20_Command;
typedef enum
{
  DS18B20_OK              = 0x00,
  DS18B20_ERROR           = 0x01,
} DS18B20_Status;
typedef enum
{
  DS18B20_NONE            = 0x00,
  DS18B20_DATA            = 0x01,
  DS18B20_DELAY           = 0x02,
} DS18B20_WaitCondition;
/* Functions -----------------------------------------------------------------*/
extern DS18B20_Status DS18B20_ConvertT(DS18B20 *sensor, DS18B20_WaitCondition waitCondition);
extern DS18B20_Status DS18B20_ReadScratchpad(DS18B20 *sensor);
extern DS18B20_Status DS18B20_WriteScratchpad(DS18B20 *sensor, uint8_t *data);
extern DS18B20_Status DS18B20_InitializationCommand(DS18B20 *sensor);
extern DS18B20_Status DS18B20_ReadRom(DS18B20 *sensor);
extern DS18B20_Status DS18B20_SkipRom(DS18B20 *sensor);
extern void DS18B20_Init(DS18B20 *sensor, UART_HandleTypeDef *huart);
#endif // #ifndef DS18B20_H