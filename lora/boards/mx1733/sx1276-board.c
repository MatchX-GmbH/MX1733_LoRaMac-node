/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation with FreeRTOS on MX1733 DevKit
 *
 * \author    Ogulcan Bal ( MatchX )
 *
 */

#include <math.h>
#include <stdlib.h>
#include "delay.h"
#include "radio.h"
#include "sx1276-board.h"

#include "hw_spi.h"

#include "hw/hw.h"
#include "lora/lora.h"

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      5

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] power Selects the right PA according to the wanted power.
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX1276GetPaSelect( int8_t power );

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
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime,
    NULL, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntSwitchRx;
Gpio_t AntSwitchTxBoost;
Gpio_t AntSwitchTxRfo;

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif

static spi_config  spi_conf= {
  .cs_pad   = {
    .port = HW_LORA_SPI_CS_PORT,
    .pin  = HW_LORA_SPI_CS_PIN,
  },
  .word_mode      = HW_SPI_WORD_8BIT,
  .smn_role       = HW_SPI_MODE_MASTER,
  .polarity_mode  = HW_SPI_POL_LOW,
  .phase_mode     = HW_SPI_PHA_MODE_0,
  .mint_mode      = HW_SPI_MINT_DISABLE,
  .xtal_freq      = HW_SPI_FREQ_DIV_8,
  .fifo_mode      = HW_SPI_FIFO_RX_TX,
};

void SX1276IoInit( void )
{
  hw_gpio_configure_pin(HW_LORA_REST_PORT, HW_LORA_REST_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO, true);
  hw_gpio_set_pin_function(HW_LORA_SPI_CLK_PORT, HW_LORA_SPI_CLK_PIN,
    HW_GPIO_MODE_OUTPUT, HW_LORA_GPIO_FUNC_SPI_CLK);
  hw_gpio_set_pin_function(HW_LORA_SPI_DI_PORT, HW_LORA_SPI_DI_PIN,
    HW_GPIO_MODE_INPUT, HW_LORA_GPIO_FUNC_SPI_DI);
  hw_gpio_set_pin_function(HW_LORA_SPI_DO_PORT, HW_LORA_SPI_DO_PIN,
    HW_GPIO_MODE_OUTPUT, HW_LORA_GPIO_FUNC_SPI_DO);
  hw_gpio_configure_pin(HW_LORA_SPI_CS_PORT, HW_LORA_SPI_CS_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO, true);
  hw_gpio_set_pin_function(HW_LORA_RX_PORT, HW_LORA_RX_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
  hw_gpio_set_pin_function(HW_LORA_TX_PORT, HW_LORA_TX_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);

  hw_gpio_set_pin_function(HW_LORA_DIO0_PORT, HW_LORA_DIO0_PIN,
    HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
  hw_gpio_set_pin_function(HW_LORA_DIO1_PORT, HW_LORA_DIO1_PIN,
    HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
  hw_gpio_set_pin_function(HW_LORA_DIO2_PORT, HW_LORA_DIO2_PIN,
    HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);

  hw_spi_init(HW_LORA_SPI, &spi_conf);

  // Important for GPIO implementation to know it's SPI pin.
  SX1276.Spi.Nss.portIndex = HW_LORA_SPI_CS_PORT;
  SX1276.Spi.Nss.pinIndex = HW_LORA_SPI_CS_PIN;
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
  lora_hw_init(irqHandlers);
}

void SX1276IoDeInit( void )
{

}

void SX1276IoDbgInit( void )
{

}

void SX1276IoTcxoInit( void )
{

}

void SX1276SetBoardTcxo( uint8_t state )
{

}

uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX1276Reset( void )
{
  // Set RESET pin to 0
  hw_gpio_configure_pin(HW_LORA_REST_PORT, HW_LORA_REST_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO, false);
  // Wait 1 ms
  DelayMs( 1 );
  // Configure RESET as input
  hw_gpio_set_pin_function(HW_LORA_REST_PORT, HW_LORA_REST_PIN,
    HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
  // Wait 6 ms
  DelayMs( 6 );
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( power );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power > 0 )
        {
            if( power > 15 )
            {
                power = 15;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 7 << 4 ) | ( power );
        }
        else
        {
            if( power < -4 )
            {
                power = -4;
            }
            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 0 << 4 ) | ( power + 4 );
        }
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

static uint8_t SX1276GetPaSelect( int8_t power )
{
  return RF_PACONFIG_PASELECT_PABOOST;
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
//  hw_gpio_set_pin_function(HW_LORA_RX_PORT, HW_LORA_RX_PIN,
//    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
//  hw_gpio_set_pin_function(HW_LORA_TX_PORT, HW_LORA_TX_PIN,
//    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
}

void SX1276AntSwDeInit( void )
{
//  hw_gpio_set_inactive(HW_LORA_RX_PORT, HW_LORA_RX_PIN);
//  hw_gpio_set_inactive(HW_LORA_TX_PORT, HW_LORA_TX_PIN);
}

void SX1276SetAntSw( uint8_t opMode )
{
  switch( opMode )
  {
  case RFLR_OPMODE_TRANSMITTER:
    hw_gpio_set_inactive(HW_LORA_RX_PORT, HW_LORA_RX_PIN);
    hw_gpio_set_active(  HW_LORA_TX_PORT, HW_LORA_TX_PIN);
    break;
  case RFLR_OPMODE_RECEIVER:
  case RFLR_OPMODE_RECEIVER_SINGLE:
  case RFLR_OPMODE_CAD:
  default:
    hw_gpio_set_inactive(HW_LORA_TX_PORT, HW_LORA_TX_PIN);
    hw_gpio_set_active(  HW_LORA_RX_PORT, HW_LORA_RX_PIN);
    break;
  }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
  (void)(frequency);
  // Implement check. Currently all frequencies are supported
  return true;
}
