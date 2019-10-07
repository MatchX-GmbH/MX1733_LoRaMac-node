/* LoRa MatchX protocol */

#include <stdint.h>

#include <osal.h>

#include "hw/led.h"
#include "ble/ble.h"
#include "lora/param.h"
#include "lora/util.h"

/* CMD_REBOOT_UPGRADE */
#define UPGRADE_BIT		0x80
#define REBOOT_TIMEOUT_MASK	0x07

PRIVILEGED_DATA static OS_TIMER upgrade_timer;

static const TickType_t	reboot_timeouts[] = {
  OS_MS_2_TICKS(2 * 1000),
  OS_MS_2_TICKS(5 * 60 * 1000),	/* UPGRADE_DEFAULT */
  OS_MS_2_TICKS(15 * 60 * 1000),
  OS_MS_2_TICKS(30 * 60 * 1000),
  OS_MS_2_TICKS(60 * 60 * 1000),
  OS_MS_2_TICKS(2 * 60 * 60 * 1000),
  OS_MS_2_TICKS(4 * 60 * 60 * 1000),
  /* TBD */
};

static void
do_reboot(OS_TIMER timer)
{
  PRIVILEGED_DATA static uint8_t	cnt;

  /* If SUOTA is active, delay reboot, but force it after
   * 256 * 5s (21 min 20 sec). */
  if (ble_is_suota_ongoing() && --cnt) {
    OS_TIMER_CHANGE_PERIOD(timer, OS_MS_2_TICKS(5000), OS_TIMER_FOREVER);
    OS_TIMER_START(timer, OS_TIMER_FOREVER);
    return;
  }
  hw_cpm_reboot_system();
}

static void
schedule_reboot(uint8_t flags)
{
  uint8_t				tmout;

  tmout = flags & REBOOT_TIMEOUT_MASK;
  if (tmout >= ARRAY_SIZE(reboot_timeouts))
    return; // XXX
  OS_TIMER_CHANGE_PERIOD(upgrade_timer, reboot_timeouts[tmout], OS_TIMER_FOREVER);
  OS_TIMER_START(upgrade_timer, OS_TIMER_FOREVER);
}

void
upgrade_reboot(uint8_t flags)
{
  if ((flags & REBOOT_TIMEOUT_MASK) >= ARRAY_SIZE(reboot_timeouts))
    return; // XXX
  if (flags & UPGRADE_BIT) {
    param_set(PARAM_SUOTA, &flags, sizeof(flags));
    led_notify(LED_STATE_REBOOTING);
    schedule_reboot(0);
  } else {
    schedule_reboot(flags);
  }
}

void
upgrade_init()
{
  uint8_t	flags;

  /* create timer for the upgrade. */
  if(upgrade_timer == NULL){
    upgrade_timer = OS_TIMER_CREATE("upgradetimer", reboot_timeouts[1],
      OS_TIMER_FAIL, (void *) OS_GET_CURRENT_TASK(), do_reboot);

    OS_ASSERT(upgrade_timer);
  }

  if (param_get(PARAM_SUOTA, &flags, sizeof(flags)) == 0 || flags == 0)
    return;
  /*
   * Device will be ready for upgrade if suota parameter is set.
   * Last 3 bits of the parameter define the timeout of reset if
   * suota upgrade doesn't start in reboot_timeouts.
   * Example: If the suota parameter is 0x81, BLE will be open for
   * 5 minutes and then will reboot if suota upgrade doesn't start.
   */
  if (flags & UPGRADE_BIT) {
    ble_on();
    schedule_reboot(flags);
  }
  flags = 0;
  param_set(PARAM_SUOTA, &flags, sizeof(flags));
}
