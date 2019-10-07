/* LoRa task */

#include <stdint.h>
#include <stdio.h>

#include "osal.h"
#include "sys_watchdog.h"
#include "hw_gpio.h"
#include "hw_wkup.h"

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
#include "lora/boards/board.h"
#include "lora/boards/sx1276-board.h"
#include "lora/mac/LoRaMac.h"
#include "sensor/sensor.h"
#include "sensor/bat.h"
#include "sensor/gps.h"

#define DEBUG
#define DEBUG_TIME
//#define DEBUG_STATE
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

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_5

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

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
PRIVILEGED_DATA static loramac_cb_t *gp_loramac_cb;

PRIVILEGED_DATA static OS_TASK lora_task_handle;
PRIVILEGED_DATA static OS_TIMER next_tx_timer;
PRIVILEGED_DATA static OS_TIMER prepare_tx_timer;

PRIVILEGED_DATA static DioIrqHandler **gp_irqHandlers;

/*!
 * Indicates if a new packet can be sent
 */
INITIALISED_PRIVILEGED_DATA static bool NextTx = true;

#ifdef DEBUG

#ifdef DEBUG_STATE
static uint8_t pre_state = DEVICE_STATE_INIT;
#endif

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

/*!
 * Executes the network Join request
 */
static void JoinNetwork( void )
{
  LoRaMacStatus_t status;
  MlmeReq_t mlmeReq;
  mlmeReq.Type = MLME_JOIN;
  mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

  // Starts the join procedure
  status = LoRaMacMlmeRequest( &mlmeReq );
#ifdef DEBUG
  debug_time();
  printf( "MLME-Request - MLME_JOIN\r\n" );
#ifdef DEBUG_STATE
  printf( "STATUS      : %d\r\n", status );
#endif
#endif

  if( status == LORAMAC_STATUS_OK )
  {
#ifdef DEBUG
    printf( "JOINING\r\n" );
#endif
    DeviceState = DEVICE_STATE_SLEEP;
  }
  else
  {
    DeviceState = DEVICE_STATE_CYCLE;
  }
}

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

  mibReq.Type = MIB_NETWORK_ACTIVATION;
  status = LoRaMacMibGetRequestConfirm( &mibReq );

  if( status == LORAMAC_STATUS_OK )
  {
    if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
    {
      // Network not joined yet. Try to join again
      JoinNetwork( );
    }
    else
    {
      DeviceState = DEVICE_STATE_PREPARE_TX;
      NextTx = true;
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
  if( mcpsIndication->FramePending == true )
  {
      // The server signals that it has pending data to be sent.
      // We schedule an uplink as soon as possible to flush the server.
      proto_send_data();
  }
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
    // Implementation of the downlink messages from server.
    if(mcpsIndication->BufferSize > 1){
      debug_time();
      proto_handle(mcpsIndication->Port, mcpsIndication->Buffer, \
        mcpsIndication->BufferSize);
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
        MibRequestConfirm_t mibReq;
        mibReq.Type = MIB_NET_ID;
        LoRaMacMibGetRequestConfirm( &mibReq );
#ifdef DEBUG
        debug_time();
        printf("JOINED: netid = %06lx\r\n", mibReq.Param.NetID);
#endif
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
  }else{
    // Join was not successful. Try to join again
    JoinNetwork( );
  }
  NextTx = true;
  lora_task_notify_event(EVENT_NOTIF_LORAMAC, NULL);
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
  if( mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
  {
#ifdef DEBUG
    debug_time();
    printf( "MLME-Indication\r\n" );
#ifdef DEBUG_STATE
    printf( "STATUS      : %s\r\n", mlmeIndication->Status);
#endif
#endif
  }
  if( mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
  {
  }
  switch( mlmeIndication->MlmeIndication )
  {
    case MLME_SCHEDULE_UPLINK:
    {
      // The MAC signals that we shall provide an uplink as soon as possible
      proto_send_data( );
      break;
    }
    default:
      break;
  }
}

void OnMacProcessNotify( void )
{
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
  LoRaMacStatus_t     status;

  DeviceState = DEVICE_STATE_INIT;
  BoardInitMcu();

	// start main loop of lora task.
  for (;;) {
    OS_BASE_TYPE ret;
    uint32_t notif = 0;

    /* notify watchdog on each loop */
    sys_watchdog_notify(wdog_id);

    // Process Radio IRQ
    if( Radio.IrqProcess != NULL )
    {
        Radio.IrqProcess( );
    }
    // Processes the LoRaMac events
    LoRaMacProcess( );

    switch( DeviceState )
    {
      case DEVICE_STATE_INIT:
      {
        LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
        LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
        LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
        LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
        LoRaMacCallbacks.GetBatteryLevel = NULL;
        LoRaMacCallbacks.GetTemperatureLevel = NULL;
        LoRaMacCallbacks.NvmContextChange = NULL;
        LoRaMacCallbacks.MacProcessNotify = OnMacProcessNotify;
        status = LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION );

#ifdef DEBUG_STATE
        printf("LoRaMacInitialization status: %d\r\n", status);
#endif
        mibReq.Type = MIB_DEV_EUI;
        mibReq.Param.DevEui = param_get_addr(PARAM_DEV_EUI);
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_JOIN_EUI;
        mibReq.Param.JoinEui = param_get_addr(PARAM_APP_EUI);
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_APP_KEY;
        mibReq.Param.AppKey = param_get_addr(PARAM_DEV_KEY);
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_NWK_KEY;
        mibReq.Param.NwkKey = param_get_addr(PARAM_DEV_KEY);
        LoRaMacMibSetRequestConfirm( &mibReq );

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

        mibReq.Type = MIB_PUBLIC_NETWORK;
        mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_ADR;
        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
        LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif

        mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
        mibReq.Param.SystemMaxRxError = 20;
        LoRaMacMibSetRequestConfirm( &mibReq );

        LoRaMacStart( );

        mibReq.Type = MIB_NETWORK_ACTIVATION;
        status = LoRaMacMibGetRequestConfirm( &mibReq );

        if( status == LORAMAC_STATUS_OK )
        {
            if( mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE )
            {
                DeviceState = DEVICE_STATE_JOIN;
            }
            else
            {
                DeviceState = DEVICE_STATE_SEND;
                NextTx = true;
            }
        }
        break;
        }
        case DEVICE_STATE_JOIN:
        {
          led_notify(LED_STATE_JOINING);
#if( OVER_THE_AIR_ACTIVATION == 0 )
          printf( "###### ===== JOINED ==== ######\r\n" );
          printf( "\r\nABP\r\n\r\n" );
          printf( "DevAddr     : %08lX\r\n", DevAddr );
          printf( "NwkSKey     : %02X", FNwkSIntKey[0] );
          for( int i = 1; i < 16; i++ )
          {
              printf( " %02X", FNwkSIntKey[i] );
          }
          printf( "\r\n" );
          printf( "AppSKey     : %02X", AppSKey[0] );
          for( int i = 1; i < 16; i++ )
          {
              printf( " %02X", AppSKey[i] );
          }
          printf( "\n\r\n" );

          mibReq.Type = MIB_NETWORK_ACTIVATION;
          mibReq.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
          LoRaMacMibSetRequestConfirm( &mibReq );

          DeviceState = DEVICE_STATE_SEND;
#else
          JoinNetwork( );
#endif
          break;
      }
      case DEVICE_STATE_PREPARE_TX:
      {
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
#endif
          led_notify(LED_STATE_SENDING);
          proto_send_data();
        }

        DeviceState = DEVICE_STATE_CYCLE;
        break;
      }
      case DEVICE_STATE_CYCLE:
      {
        DeviceState = DEVICE_STATE_SLEEP;

        ad_lora_allow_sleep(LORA_SUSPEND_LORA);
        led_notify(LED_STATE_IDLE);

        // Schedule next packet transmission
        OS_TIMER_CHANGE_PERIOD(next_tx_timer, \
          sensor_period(), OS_TIMER_FOREVER);
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
      gp_irqHandlers[0](NULL);
    }

    if (notif & EVENT_NOTIF_LORA_DIO1) {
      gp_irqHandlers[1](NULL);
    }

    if (notif & EVENT_NOTIF_LORA_DIO2) {
      gp_irqHandlers[2](NULL);
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

#ifdef DEBUG_STATE
    if(pre_state != DeviceState){
      debug_time();
      printf("state: %d, SX1276: %d, LR_CONF: %d\r\n", DeviceState, SX1276Read( REG_OPMODE ), SX1276Read(REG_LR_PACONFIG));
      pre_state = DeviceState;
    }
#endif
  }
}
