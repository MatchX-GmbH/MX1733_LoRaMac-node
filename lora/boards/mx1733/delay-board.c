/*!
 * \file      delay-board.c
 *
 * \brief     Target board delay implementation with FreeRTOS on MX1733 DevKit
 *
 * \author    Ogulcan Bal ( MatchX )
 *
 */
#include "osal.h"
#include "delay-board.h"

void DelayMsMcu( uint32_t ms )
{
  OS_DELAY_MS( ms );
}
