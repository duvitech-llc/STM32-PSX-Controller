#ifndef BITBANG_SPI_H
#define BITBANG_SPI_H

#include <stdint.h>
#include <stdbool.h>

/*
 * GPIO control macros must be defined by the user BEFORE including this header.
 *
 * Required macros:
 *
 *   SPI_CS_LOW()
 *   SPI_CS_HIGH()
 *
 *   SPI_CLK_LOW()
 *   SPI_CLK_HIGH()
 *
 *   SPI_MOSI_LOW()
 *   SPI_MOSI_HIGH()
 *
 *   SPI_MISO_READ()   -> returns 0 or 1
 *
 * util.h must provide:
 *   void delay_us(uint32_t us);
 */

void bitbang_spi_init(uint32_t bitrate_hz);
uint8_t bitbang_spi_transfer(uint8_t out);
void bitbang_spi_cs_assert(void);
void bitbang_spi_cs_deassert(void);

#endif /* BITBANG_SPI_H */
