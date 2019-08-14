/* LoRa task */

#include <stdint.h>
#include <stdio.h>

#include "osal.h"
#include "sys_watchdog.h"

#include "board.h"
#include "ble/ble.h"
#include "hw/button.h"
#include "hw/cons.h"
#include "hw/hw.h"
#include "hw/iox.h"
#include "hw/led.h"
#include "lora/ad_lora.h"
#include "lora/lora.h"
#include "lora/param.h"
#include "lora/proto.h"
#include "lora/upgrade.h"
#include "lora/util.h"
#include "lora/mac/LoRaMac.h"
#include "sensor/sensor.h"
#include "sensor/bat.h"
#include "sensor/gps.h"

#define DEBUG
#define DEBUG_TIME
//#define BLE_ALWAYS_ON

#define MAX_SENSOR_SAMPLE_TIME	OS_MS_2_TICKS(2 * 1000)
PRIVILEGED_DATA static TickType_t	sampling_since;

#define JOIN_TIMEOUT		  OS_MS_2_TICKS(2 * 60 * 60 * 1000)
#define REJOIN_TIMEOUT		OS_MS_2_TICKS(15 * 60 * 1000)
#define TX_TIMEOUT		    OS_MS_2_TICKS(12 * 1000)
#define TX_PERIOD_TIMEOUT	OS_MS_2_TICKS(10 * 60 * 1000)
#define ALIVE_TX_PERIOD		OS_MS_2_TICKS(60 * 1000)
#define SEND_RETRY_TIME		OS_MS_2_TICKS(10 * 1000)

#define MAX_RESETS		8

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     1

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                      true

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( USE_BAND_868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
static uint8_t AppSKey[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

/*!
 * Device address
 */
static uint32_t DevAddr = ( uint32_t )0x00000000;

#endif

/*!
 * Device states
 */
PRIVILEGED_DATA static enum eDevicState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_PREPARE_TX,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP,
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
PRIVILEGED_DATA struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

typedef void ( loramac_cb_t )( void );

PRIVILEGED_DATA static OS_TASK lora_task_handle;
PRIVILEGED_DATA static OS_TIMER next_tx_timer;
PRIVILEGED_DATA static OS_TIMER prepare_tx_timer;

PRIVILEGED_DATA static DioIrqHandler **gp_irqHandlers;
PRIVILEGED_DATA static loramac_cb_t *gp_loramac_cb;

/*!
 * Indicates if a new packet can be sent
 */
INITIALISED_PRIVILEGED_DATA static bool NextTx = true;

#ifdef DEBUG

static uint8_t pre_state = DEVICE_STATE_INIT;

#ifdef DEBUG_TIME
static void
debug_time(void)
{
	uint32_t	now = OS_TICKS_2_MS(OS_GET_TICK_COUNT());

	printf("%lu:%02lu.%03lu ", now / 60000, \
	  now / 1000 % 60, now % 1000);
}
#else
#define debug_time()
#endif

#endif

void
lora_send(uint8_t *data, size_t len)
{
  McpsReq_t mcpsReq;
  LoRaMacTxInfo_t txInfo;

  if(LoRaMacQueryTxPossible(len, &txInfo) != LORAMAC_STATUS_OK)
  {
    // Send empty frame in order to flush MAC commands
    mcpsReq.Type = MCPS_UNCONFIRMED;
    mcpsReq.Req.Unconfirmed.fBuffer = NULL;
    mcpsReq.Req.Unconfirmed.fBufferSize = 0;
    mcpsReq.Req.Unconfirmed.Datarate = DR_0;
  }
  else
  {
    if( ComplianceTest.IsTxConfirmed == false )
    {
      mcpsReq.Type = MCPS_UNCONFIRMED;
      mcpsReq.Req.Unconfirmed.fPort = 1;
      mcpsReq.Req.Unconfirmed.fBuffer = data;
      mcpsReq.Req.Unconfirmed.fBufferSize = len;
      mcpsReq.Req.Unconfirmed.Datarate = DR_0;
    }
    else
    {
      mcpsReq.Type = MCPS_CONFIRMED;
      mcpsReq.Req.Confirmed.fPort = 1;
      mcpsReq.Req.Confirmed.fBuffer = data;
      mcpsReq.Req.Confirmed.fBufferSize = len;
      mcpsReq.Req.Confirmed.NbTrials = 8;
      mcpsReq.Req.Confirmed.Datarate = DR_0;
    }
  }

  if(LoRaMacMcpsRequest(&mcpsReq) == LORAMAC_STATUS_OK)
  {
    NextTx = false;
  }else{
    NextTx = true;
  }
}

/*!
 * \brief Function executed on next_tx_timer Timeout event
 */
static void next_tx_cb(OS_TIMER timer)
{
  MibRequestConfirm_t mibReq;
  LoRaMacStatus_t status;

  OS_TIMER_STOP_FROM_ISR(timer);

  mibReq.Type = MIB_NETWORK_JOINED;
  status = LoRaMacMibGetRequestConfirm( &mibReq );

  if( status == LORAMAC_STATUS_OK )
  {
    if( mibReq.Param.IsNetworkJoined == true )
    {
      DeviceState = DEVICE_STATE_PREPARE_TX;
      NextTx = true;
    }
    else
    {
      DeviceState = DEVICE_STATE_JOIN;
    }
  }
  lora_task_notify_event(EVENT_NOTIF_LORAMAC, NULL);
}

/*!
 * \brief Function executed on next_tx_timer Timeout event
 */
static void lora_tx_ready_cb(OS_TIMER timer)
{
  TickType_t delay;
  if (OS_GET_TICK_COUNT() < (sampling_since + MAX_SENSOR_SAMPLE_TIME) && \
      (delay = sensor_data_ready()) != 0) {
    OS_TIMER_START(timer, OS_TIMER_FOREVER);
    ad_lora_suspend_sleep(LORA_SUSPEND_LORA, delay);
  } else {
    DeviceState = DEVICE_STATE_SEND;
  }
  lora_task_notify_event(EVENT_NOTIF_LORAMAC, NULL);
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
  if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
  {
    switch( mcpsConfirm->McpsRequest )
    {
      case MCPS_UNCONFIRMED:
      {
        // Check Datarate
        // Check TxPower
        break;
      }
      case MCPS_CONFIRMED:
      {
        // Check Datarate
        // Check TxPower
        // Check AckReceived
        // Check NbTrials
        break;
      }
      case MCPS_PROPRIETARY:
      {
        break;
      }
      default:
        break;
    }
  }
  NextTx = true;
  lora_task_notify_event(EVENT_NOTIF_LORAMAC, NULL);
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
  if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
  {
      return;
  }

  switch( mcpsIndication->McpsIndication )
  {
      case MCPS_UNCONFIRMED:
      {
          break;
      }
      case MCPS_CONFIRMED:
      {
          break;
      }
      case MCPS_PROPRIETARY:
      {
          break;
      }
      case MCPS_MULTICAST:
      {
          break;
      }
      default:
          break;
  }

  // Check Multicast
  // Check Port
  // Check Datarate
  // Check FramePending
  // Check Buffer
  // Check BufferSize
  // Check Rssi
  // Check Snr
  // Check RxSlot

  if( ComplianceTest.Running == true )
  {
      ComplianceTest.DownLinkCounter++;
  }

  if( mcpsIndication->RxData == true )
  {
    switch( mcpsIndication->Port )
    {
    case 1:
    case 2:
      break;
    case 224:
      if( ComplianceTest.Running == false )
      {
        // Check compliance test enable command (i)
        if( ( mcpsIndication->BufferSize == 4 ) &&
            ( mcpsIndication->Buffer[0] == 0x01 ) &&
            ( mcpsIndication->Buffer[1] == 0x01 ) &&
            ( mcpsIndication->Buffer[2] == 0x01 ) &&
            ( mcpsIndication->Buffer[3] == 0x01 ) )
        {
          ComplianceTest.IsTxConfirmed = false;
          ComplianceTest.AppPort = 224;
          ComplianceTest.AppDataSize = 2;
          ComplianceTest.DownLinkCounter = 0;
          ComplianceTest.LinkCheck = false;
          ComplianceTest.DemodMargin = 0;
          ComplianceTest.NbGateways = 0;
          ComplianceTest.Running = true;
          ComplianceTest.State = 1;

          MibRequestConfirm_t mibReq;
          mibReq.Type = MIB_ADR;
          mibReq.Param.AdrEnable = true;
          LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
          LoRaMacTestSetDutyCycleOn( false );
#endif
        }
      }
      else
      {
        ComplianceTest.State = mcpsIndication->Buffer[0];
        switch( ComplianceTest.State )
        {
        case 0: // Check compliance test disable command (ii)
          ComplianceTest.IsTxConfirmed = false;
          ComplianceTest.AppPort = LORAWAN_APP_PORT;
          ComplianceTest.AppDataSize = LORAWAN_APP_DATA_SIZE;
          ComplianceTest.DownLinkCounter = 0;
          ComplianceTest.Running = false;

          MibRequestConfirm_t mibReq;
          mibReq.Type = MIB_ADR;
          mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
          LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
          LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
          break;
        case 1: // (iii, iv)
          ComplianceTest.AppDataSize = 2;
          break;
        case 2: // Enable confirmed messages (v)
          ComplianceTest.IsTxConfirmed = true;
          ComplianceTest.State = 1;
          break;
        case 3:  // Disable confirmed messages (vi)
          ComplianceTest.IsTxConfirmed = false;
          ComplianceTest.State = 1;
          break;
        case 4: // (vii)
          ComplianceTest.AppDataSize = mcpsIndication->BufferSize;

//          AppData[0] = 4;
//          for( uint8_t i = 1; i < AppDataSize; i++ )
//          {
//              AppData[i] = mcpsIndication->Buffer[i] + 1;
//          }
          break;
        case 5: // (viii)
            {
              MlmeReq_t mlmeReq;
              mlmeReq.Type = MLME_LINK_CHECK;
              LoRaMacMlmeRequest( &mlmeReq );
            }
            break;
        case 6: // (ix)
            {
              MlmeReq_t mlmeReq;

              mlmeReq.Type = MLME_JOIN;

              mlmeReq.Req.Join.DevEui = param_get_addr(PARAM_DEV_EUI);
              mlmeReq.Req.Join.AppEui = param_get_addr(PARAM_APP_EUI);
              mlmeReq.Req.Join.AppKey = param_get_addr(PARAM_DEV_KEY);

              LoRaMacMlmeRequest( &mlmeReq );
              DeviceState = DEVICE_STATE_SLEEP;
            }
            break;
        default:
            break;
        }
      }
      break;
    default:
      break;
    }
  }
  lora_task_notify_event(EVENT_NOTIF_LORAMAC, NULL);
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
  if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
  {
    switch( mlmeConfirm->MlmeRequest )
    {
      case MLME_JOIN:
      {
        // Status is OK, node has joined the network
        DeviceState = DEVICE_STATE_PREPARE_TX;
        NextTx = true;
        break;
      }
      case MLME_LINK_CHECK:
      {
        // Check DemodMargin
        // Check NbGateways
        if( ComplianceTest.Running == true )
        {
            ComplianceTest.LinkCheck = true;
            ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
            ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
        }
        break;
      }
      default:
        break;
    }
  }
  NextTx = true;
  lora_task_notify_event(EVENT_NOTIF_LORAMAC, NULL);
}

static void
lora_wkup_int_cb(void)
{
  if (hw_gpio_get_pin_status(HW_LORA_DIO0_PORT, HW_LORA_DIO0_PIN))
  {
    lora_task_notify_event(EVENT_NOTIF_LORA_DIO0, NULL);
  }
  if (hw_gpio_get_pin_status(HW_LORA_DIO1_PORT, HW_LORA_DIO1_PIN))
  {
    lora_task_notify_event(EVENT_NOTIF_LORA_DIO1, NULL);
  }
  if (hw_gpio_get_pin_status(HW_LORA_DIO2_PORT, HW_LORA_DIO2_PIN))
  {
    lora_task_notify_event(EVENT_NOTIF_LORA_DIO2, NULL);
  }
#ifdef FEATURE_USER_BUTTON
  if (hw_gpio_get_pin_status(HW_USER_BTN_PORT, HW_USER_BTN_PIN))
  {
    lora_task_notify_event(EVENT_NOTIF_BTN_PRESS, NULL);
  }
#endif
  hw_wkup_reset_interrupt();
}

void lora_task_notify_event(uint32_t event, void *cb)
{
  OS_TASK_NOTIFY_FROM_ISR(lora_task_handle, event, eSetBits);
  if(cb)
  {
    gp_loramac_cb = cb;
  }
}

void
lora_hw_init(void *irq)
{
#ifdef FEATURE_USER_BUTTON
  hw_gpio_set_pin_function(HW_USER_BTN_PORT, HW_USER_BTN_PIN,
    HW_GPIO_MODE_INPUT, HW_GPIO_FUNC_GPIO);
#endif

  hw_wkup_init(NULL);
  hw_wkup_set_counter_threshold(1);
  hw_wkup_configure_pin(HW_LORA_DIO0_PORT, HW_LORA_DIO0_PIN, true,
    HW_WKUP_PIN_STATE_HIGH);
  hw_wkup_configure_pin(HW_LORA_DIO1_PORT, HW_LORA_DIO1_PIN, true,
    HW_WKUP_PIN_STATE_HIGH);
  hw_wkup_configure_pin(HW_LORA_DIO2_PORT, HW_LORA_DIO2_PIN, true,
    HW_WKUP_PIN_STATE_HIGH);
#ifdef FEATURE_USER_BUTTON
  hw_wkup_configure_pin(HW_USER_BTN_PORT, HW_USER_BTN_PIN, true,
    HW_USER_BTN_ACTIVE);
#endif
  hw_wkup_register_interrupt(lora_wkup_int_cb, 1);

  gp_irqHandlers = irq;
  OS_ASSERT(irq);
}

void
lora_task_func(void *param)
{
  int8_t wdog_id;
	(void)param;
	param_init();
	ad_lora_init();
	led_notify(LED_STATE_BOOTING);
	/* register lora task to be monitored by watchdog */
	wdog_id = sys_watchdog_register(false);
	lora_task_handle = OS_GET_CURRENT_TASK();
	// check if the suota upgrade bit was set before reboot.
	upgrade_init();
#ifdef BLE_ALWAYS_ON
	ble_on();
#endif
	//Initialize LoRaMAC
	LoRaMacPrimitives_t LoRaMacPrimitives;
  LoRaMacCallback_t   LoRaMacCallbacks;
  MibRequestConfirm_t mibReq;

  DeviceState = DEVICE_STATE_INIT;
  SX1276IoInit();

	// start main loop of lora task.
  for (;;) {
    OS_BASE_TYPE ret;
    uint32_t notif = 0;

    /* notify watchdog on each loop */
    sys_watchdog_notify(wdog_id);

    switch( DeviceState )
    {
      case DEVICE_STATE_INIT:
      {
        LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
        LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
        LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
        LoRaMacCallbacks.GetBatteryLevel = bat_level;
        LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

        if(next_tx_timer == NULL){
          next_tx_timer = OS_TIMER_CREATE("nexttx", sensor_period(), \
            OS_TIMER_FAIL, (void *) OS_GET_CURRENT_TASK(), next_tx_cb);

          OS_ASSERT(next_tx_timer);
        }

        if(prepare_tx_timer == NULL){
          prepare_tx_timer = OS_TIMER_CREATE("preparetx", OS_MS_2_TICKS(100), \
            OS_TIMER_FAIL, (void *) OS_GET_CURRENT_TASK(), lora_tx_ready_cb);

          OS_ASSERT(prepare_tx_timer);
        }

        mibReq.Type = MIB_ADR;
        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_PUBLIC_NETWORK;
        mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
        LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
        LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
        LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
        LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
        LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
        LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
        LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
        LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );
#endif

#endif
        // TODO:Class C device if needed. Implement later.
//        mibReq.Type = MIB_DEVICE_CLASS;
//        mibReq.Param.Class = CLASS_C;
//        LoRaMacMibSetRequestConfirm( &mibReq );

        DeviceState = DEVICE_STATE_JOIN;
        break;
        }
        case DEVICE_STATE_JOIN:
        {
          led_notify(LED_STATE_JOINING);
#if( OVER_THE_AIR_ACTIVATION != 0 )
          MlmeReq_t mlmeReq;

          mlmeReq.Type = MLME_JOIN;

          mlmeReq.Req.Join.DevEui = param_get_addr(PARAM_DEV_EUI);
          mlmeReq.Req.Join.AppEui = param_get_addr(PARAM_APP_EUI);
          mlmeReq.Req.Join.AppKey = param_get_addr(PARAM_DEV_KEY);

          if( NextTx == true )
          {
              LoRaMacMlmeRequest( &mlmeReq );
          }
          DeviceState = DEVICE_STATE_SLEEP;
#else
          // Choose a random device address if not already defined in Comissioning.h
          if( DevAddr == 0 )
          {
              // Random seed initialization
              srand1( BoardGetRandomSeed( ) );

              // Choose a random device address
              DevAddr = randr( 0, 0x01FFFFFF );
          }

          mibReq.Type = MIB_NET_ID;
          mibReq.Param.NetID = LORAWAN_NETWORK_ID;
          LoRaMacMibSetRequestConfirm( &mibReq );

          mibReq.Type = MIB_DEV_ADDR;
          mibReq.Param.DevAddr = DevAddr;
          LoRaMacMibSetRequestConfirm( &mibReq );

          mibReq.Type = MIB_NWK_SKEY;
          mibReq.Param.NwkSKey = NwkSKey;
          LoRaMacMibSetRequestConfirm( &mibReq );

          mibReq.Type = MIB_APP_SKEY;
          mibReq.Param.AppSKey = AppSKey;
          LoRaMacMibSetRequestConfirm( &mibReq );

          mibReq.Type = MIB_NETWORK_JOINED;
          mibReq.Param.IsNetworkJoined = true;
          LoRaMacMibSetRequestConfirm( &mibReq );

          DeviceState = DEVICE_STATE_SEND;
#endif
          break;
      }
      case DEVICE_STATE_PREPARE_TX:
      {
#ifdef DEBUG
        printf("netid = %06lx\r\n", mibReq.Param.NetID);
#endif
        ad_lora_suspend_sleep(LORA_SUSPEND_LORA, TX_TIMEOUT);
        proto_txstart();
        sampling_since = OS_GET_TICK_COUNT();
        led_notify(LED_STATE_SAMPLING_SENSOR);
        sensor_prepare();
        OS_TIMER_START(prepare_tx_timer, OS_TIMER_FOREVER);
        DeviceState = DEVICE_STATE_SLEEP;
        break;
      }
      case DEVICE_STATE_SEND:
      {
        if( NextTx == true )
        {
#ifdef DEBUG
          debug_time();
          printf("lora state %d\r\n", DEVICE_STATE_SEND);
#endif
          led_notify(LED_STATE_IDLE);
          proto_send_data();
        }

        DeviceState = DEVICE_STATE_CYCLE;
        break;
      }
      case DEVICE_STATE_CYCLE:
      {
        DeviceState = DEVICE_STATE_SLEEP;

        ad_lora_allow_sleep(LORA_SUSPEND_LORA);

        // Schedule next packet transmission
        OS_TIMER_START(next_tx_timer, OS_TIMER_FOREVER);
        break;
      }
      case DEVICE_STATE_SLEEP:
      {
        // Wake up through events
        /* suspend watchdog while blocking on OS_TASK_NOTIFY_WAIT() */
        sys_watchdog_suspend(wdog_id);

        /*
         * Wait on any of the notification bits, then clear them all
         */
        ret = OS_TASK_NOTIFY_WAIT(0, OS_TASK_NOTIFY_ALL_BITS, &notif, OS_TASK_NOTIFY_FOREVER);
        /* Blocks forever waiting for task notification. The return value must be OS_OK */
        OS_ASSERT(ret == OS_OK);

        /* resume watchdog */
        sys_watchdog_notify_and_resume(wdog_id);
        break;
      }
      default:
      {
        DeviceState = DEVICE_STATE_INIT;
        break;
      }
    }

    if (notif & EVENT_NOTIF_LORA_DIO0) {
      gp_irqHandlers[0]();
    }

    if (notif & EVENT_NOTIF_LORA_DIO1) {
      gp_irqHandlers[1]();
    }

    if (notif & EVENT_NOTIF_LORA_DIO2) {
      gp_irqHandlers[2]();
    }

    if (notif & EVENT_NOTIF_BTN_PRESS) {
      button_press(OS_GET_TICK_COUNT());
    }

    if (notif & EVENT_NOTIF_CONS_RX) {
      cons_rx();
    }

    if (notif & EVENT_NOTIF_GPS_RX) {
#ifdef FEATURE_SENSOR_GPS
      gps_rx();
#endif
    }

    if (notif & EVENT_NOTIF_LORAMAC) {
      if(gp_loramac_cb != NULL){
        gp_loramac_cb();
        gp_loramac_cb = NULL;
      }
    }

#ifdef DEBUG
    if(pre_state != DeviceState){
      debug_time();
      printf("state %d\r\n", DeviceState);
      printf("Debug SX1276: %d\r\n", SX1276Read( REG_OPMODE ));
      pre_state = DeviceState;
    }
#endif
  }
}
