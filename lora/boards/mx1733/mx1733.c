/*!
 * \file      mx1733.c
 *
 * \brief     Target board general functions implementation with FreeRTOS on MX1733 DevKit
 *
 * \author    Ogulcan Bal ( MatchX )
 *
 */
#include "osal.h"

#include "utilities.h"
#include "gpio.h"
#include "spi.h"
#include "timer.h"
#include "rtc-board.h"
#include "sx1276-board.h"
#include "board.h"

#include "hw/hw.h"
#include "sensor/bat.h"

void BoardCriticalSectionBegin( uint32_t *mask )
{
//  *mask = __get_PRIMASK( );
//  __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
//  __set_PRIMASK( *mask );
}

void BoardInitPeriph( void )
{

}

void BoardInitMcu( void )
{
  RtcInit();
  //SpiInit( &SX1276.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
  SX1276IoInit( );
}

void BoardResetMcu( void )
{
  CRITICAL_SECTION_BEGIN( );

  //Restart system
  NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
  SpiDeInit( &SX1276.Spi );
  SX1276IoDeInit( );
}

uint16_t BoardBatteryMeasureVolage( void )
{
  return 0;
}

uint32_t BoardGetBatteryVoltage( void )
{
  return 0;
}

uint8_t BoardGetBatteryLevel( void )
{
  return bat_level();
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line) */

    printf( "Wrong parameters value: file %s on line %lu\r\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
