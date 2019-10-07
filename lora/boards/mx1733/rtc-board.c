/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management with FreeRTOS on MX1733 DevKit
 *
 * \author    Ogulcan Bal ( MatchX )
 *
 */
#include "osal.h"
#include "sys_rtc.h"

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "utilities.h"
#include "delay.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"
#include "rtc-board.h"

#include "lora/lora.h"

// MCU Wake Up Time
#define MIN_ALARM_DELAY     3 // in ticks

#define RTC_TICKS_IN_SEC    32768

/*!
 * RTC timer context
 */
typedef struct
{
  uint32_t  Time;         // Reference time
  OS_TIMER  timer_handle; // FreeRTOS timer handle
}RtcTimerContext_t;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

/*!
 * \brief Function executed on rtc_timer_cb Timeout event
 */
static void rtc_timer_cb(OS_TIMER timer)
{
  lora_task_notify_event(EVENT_NOTIF_LORAMAC, TimerIrqHandler);
}

void RtcInit( void )
{
  if(RtcTimerContext.timer_handle == NULL)
  {
    RtcTimerContext.timer_handle = OS_TIMER_CREATE("rtctimer", \
      OS_MS_2_TICKS(MIN_ALARM_DELAY), OS_TIMER_FAIL, \
      (void *) OS_GET_CURRENT_TASK(), rtc_timer_cb);
    OS_ASSERT(RtcTimerContext.timer_handle);
  }
}

/*!
 * \brief Sets the RTC timer reference, sets also the RTC_DateStruct and RTC_TimeStruct
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcSetTimerContext( void )
{
  RtcTimerContext.Time = RtcGetTimerValue();
  return ( uint32_t )RtcTimerContext.Time;
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcGetTimerContext( void )
{
  return RtcTimerContext.Time;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
  return( MIN_ALARM_DELAY );
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( uint32_t milliseconds )
{
  return ( (milliseconds * RTC_TICKS_IN_SEC) / 1000 );
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
uint32_t RtcTick2Ms( uint32_t tick )
{
  return ( (tick * 1000) / RTC_TICKS_IN_SEC );
}

/*!
 * \brief a delay of delay ms by polling RTC
 *
 * \param[IN] delay in ms
 */
void RtcDelayMs( uint32_t delay )
{
  uint64_t delayTicks = 0;
  uint64_t refTicks = RtcGetTimerValue( );

  delayTicks = RtcMs2Tick( delay );

  // Wait delay ms
  while( ( ( RtcGetTimerValue( ) - refTicks ) ) < delayTicks )
  {
    __NOP( );
  }
}

void RtcSetAlarm( uint32_t timeout )
{
  OS_ASSERT(RtcTimerContext.timer_handle);

  RtcStartAlarm( timeout );
}

void RtcStopAlarm( void )
{
  OS_ASSERT(RtcTimerContext.timer_handle);

  OS_TIMER_STOP(RtcTimerContext.timer_handle, OS_TIMER_FOREVER);
}

void RtcStartAlarm( uint32_t timeout )
{
  OS_ASSERT(RtcTimerContext.timer_handle);

  uint32_t period = OS_MS_2_TICKS(RtcTick2Ms(timeout));

  OS_TIMER_CHANGE_PERIOD(RtcTimerContext.timer_handle, \
    period, OS_TIMER_FOREVER);

  OS_TIMER_START(RtcTimerContext.timer_handle, OS_TIMER_FOREVER);

#ifdef DEBUG
  printf("RTC Start Alarm:%ld + %ld\r\n", TimerGetCurrentTime(), RtcTick2Ms(timeout));
#endif
}

uint32_t RtcGetTimerValue( void )
{
  return (rtc_get() & 0xFFFFFFFF);
}

uint32_t RtcGetTimerElapsedTime( void )
{
  return( ( uint32_t )( RtcGetTimerValue() - RtcTimerContext.Time ) );
}

uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
  uint32_t ticks = RtcGetTimerValue();
  *milliseconds = RtcTick2Ms( ticks ) % 1000;
  return (RtcTick2Ms(ticks) / 1000);
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
  // Not used on this platform.
}

void RtcBkupRead( uint32_t *data0, uint32_t *data1 )
{
  // Not used on this platform.
}
