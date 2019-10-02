/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation with FreeRTOS on MX1733 DevKit
 *
 * \author    Ogulcan Bal ( MatchX )
 *
 */

#include "spi-board.h"

#include <hw_spi.h>

#include "hw/hw.h"

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
  // Not used on this platform.
}

void SpiDeInit( Spi_t *obj )
{
  // Not used on this platform.
}

void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
  // Not used on this platform.
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
  // Not used on this platform.
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
  uint8_t rxData = 0;

  hw_spi_wait_while_busy(HW_LORA_SPI);
  hw_spi_fifo_write8(HW_LORA_SPI, ( uint16_t ) ( outData & 0xFF ));
  hw_spi_wait_while_busy(HW_LORA_SPI);
  rxData = ( uint16_t ) hw_spi_fifo_read8(HW_LORA_SPI);

  return( rxData );
}

