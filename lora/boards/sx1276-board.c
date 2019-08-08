/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "sx1276-board.h"

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby, 
    SX1276SetRx,
    SX1276StartCad,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength
};

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntSwitchLf;
Gpio_t AntSwitchHf;

void SX1276IoInit( void )
{
  hw_gpio_configure_pin(HW_LORA_SPI_CS_PORT, HW_LORA_SPI_CS_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO, true);

  hw_gpio_set_pin_function(HW_LORA_DIO0_PORT, HW_LORA_DIO0_PIN,
    HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
  hw_gpio_set_pin_function(HW_LORA_DIO1_PORT, HW_LORA_DIO1_PIN,
    HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
  hw_gpio_set_pin_function(HW_LORA_DIO2_PORT, HW_LORA_DIO2_PIN,
    HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
  lora_hw_init(irqHandlers);
}

void SX1276IoDeInit( void )
{
  hw_gpio_configure_pin(HW_LORA_SPI_CS_PORT, HW_LORA_SPI_CS_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO, false);
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
  if( channel < RF_MID_BAND_THRESH )
  {
    return RF_PACONFIG_PASELECT_PABOOST;
  }
  else
  {
    return RF_PACONFIG_PASELECT_RFO;
  }
}

void SX1276SetAntSwLowPower( bool status )
{
  if( RadioIsActive != status )
  {
    RadioIsActive = status;

    if( status == false )
    {
      SX1276AntSwInit( );
    }
    else
    {
      SX1276AntSwDeInit( );
    }
  }
}

void SX1276AntSwInit( void )
{
  hw_gpio_set_pin_function(HW_LORA_RX_PORT, HW_LORA_RX_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
  hw_gpio_set_pin_function(HW_LORA_TX_PORT, HW_LORA_TX_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
}

void SX1276AntSwDeInit( void )
{
  hw_gpio_set_inactive(HW_LORA_RX_PORT, HW_LORA_RX_PIN);
  hw_gpio_set_inactive(HW_LORA_TX_PORT, HW_LORA_TX_PIN);
}

void SX1276SetAntSw( uint8_t rxTx )
{
  if( rxTx != 0 ) // 1: TX, 0: RX
  {
    hw_gpio_set_inactive(HW_LORA_RX_PORT, HW_LORA_RX_PIN);
    hw_gpio_set_active(  HW_LORA_TX_PORT, HW_LORA_TX_PIN);
  }
  else
  {
    hw_gpio_set_inactive(HW_LORA_TX_PORT, HW_LORA_TX_PIN);
    hw_gpio_set_active(  HW_LORA_RX_PORT, HW_LORA_RX_PIN);
  }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
  (void)(frequency);
  // Implement check. Currently all frequencies are supported
  return true;
}
