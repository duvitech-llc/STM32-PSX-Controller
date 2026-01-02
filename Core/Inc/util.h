/*
 * utils.h
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_


#include "main.h"
#include <stdint.h>

#define SPI_CS_LOW()     HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define SPI_CS_HIGH()    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)

#define SPI_CLK_LOW()    HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_RESET)
#define SPI_CLK_HIGH()   HAL_GPIO_WritePin(SPI_CLK_GPIO_Port, SPI_CLK_Pin, GPIO_PIN_SET)

#define SPI_MOSI_LOW()   HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, GPIO_PIN_RESET)
#define SPI_MOSI_HIGH()  HAL_GPIO_WritePin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, GPIO_PIN_SET)

#define SPI_MISO_READ()  HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port, SPI_MISO_Pin)

extern TIM_HandleTypeDef htim3;

void printBuffer(const uint8_t* buffer, uint32_t size);
uint16_t util_crc16(const uint8_t* buf, uint32_t size);
uint16_t util_hw_crc16(uint8_t* buf, uint32_t size);
uint8_t crc_test(void);
void get_unique_identifier(uint32_t* uid);
uint32_t fnv1a_32(const uint8_t *data, size_t len);
void DWT_Init(void);
void delay_us(uint32_t us);

#endif /* INC_UTIL_H_ */
