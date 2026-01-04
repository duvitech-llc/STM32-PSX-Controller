#include "bitbang_spi.h"
#include "util.h"

/*
 * PSX-compatible SPI:
 *  - Clock idle HIGH
 *  - Data set BEFORE falling edge
 *  - Data sampled AFTER rising edge
 *  - LSB first
 */

static uint32_t half_period_us = 5;

/*
 * bitrate_hz: desired SPI clock rate
 * Example:
 *   bitbang_spi_init(25000); // 25 kHz (safe for PSX)
 */
void bitbang_spi_init(uint32_t bitrate_hz)
{
    if (bitrate_hz == 0)
        bitrate_hz = 1000;

    half_period_us = (1000000UL / bitrate_hz) / 2;  // FIX: was 10000UL
    if (half_period_us < 1)
        half_period_us = 1;

    SPI_CS_HIGH();
    SPI_CLK_HIGH();
    SPI_MOSI_HIGH();
}

void bitbang_spi_cs_assert(void)
{
    SPI_MOSI_HIGH();
    SPI_CLK_HIGH();
    delay_us(2);
    SPI_CS_LOW();
    delay_us(20);   // PSX ATTN delay
}

void bitbang_spi_cs_deassert(void)
{
    delay_us(20);
    SPI_CS_HIGH();
    SPI_CLK_HIGH();
    SPI_MOSI_HIGH();
}

/*
 * Transfer one byte, LSB first
 */
uint8_t bitbang_spi_transfer(uint8_t out)
{
    uint8_t in = 0;

    for (uint8_t bit = 0; bit < 8; bit++)
    {
        // MOSI setup while CLK HIGH
        if (out & (1 << bit))
            SPI_MOSI_HIGH();
        else
            SPI_MOSI_LOW();

        delay_us(half_period_us);

        // Falling edge - controller samples MOSI
        SPI_CLK_LOW();
        
        // Wait most of the low period
        delay_us(half_period_us);

        // Rising edge - controller drives MISO
        SPI_CLK_HIGH();
        
        // Sample MISO immediately after rising edge
        // (or add tiny delay if needed)
        delay_us(1);  // Small delay for signal to settle
        
        if (SPI_MISO_READ())
            in |= (1 << bit);
            
        delay_us(half_period_us - 1);
    }

    return in;
}
