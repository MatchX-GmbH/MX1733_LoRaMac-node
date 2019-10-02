/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation with FreeRTOS on MX1733 DevKit
 *
 * \author    Ogulcan Bal ( MatchX )
 *
 */

#include "gpio-board.h"

#include <hw_spi.h>

#include "hw/hw.h"

// Not used on this platform.
//static Gpio_t *GpioIrq[16];

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
  // Not used on this platform.
}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
  obj->Context = context;
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
  // Not used on this platform.
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
  // Not used on this platform.
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
  if((obj->portIndex == HW_LORA_SPI_CS_PORT) && \
    (obj->pinIndex == HW_LORA_SPI_CS_PIN)){
    if(value){
      //NSS = 1;
      hw_spi_set_cs_high(HW_LORA_SPI);
    }else{
      //NSS = 0;
      hw_spi_set_cs_low(HW_LORA_SPI);
    }
  }
}

void GpioMcuToggle( Gpio_t *obj )
{
  // Not used on this platform.
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
  // Not used on this platform.
  return 0;
}
