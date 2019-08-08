#include <hw_wkup.h>

#include "osal.h"

#include "hw/button.h"
#include "hw/hw.h"
#include "lora/proto.h"
#include "lora/upgrade.h"

#ifdef FEATURE_USER_BUTTON

#define LONG_PRESS_TIME	OS_MS_2_TICKS(10 * 1000)

#if HW_USER_BTN_ACTIVE == HW_WKUP_PIN_STATE_HIGH
#define USER_BTN_DOWN(port, pin)	hw_gpio_get_pin_status(port, pin)
#else
#define USER_BTN_DOWN(port, pin)	!hw_gpio_get_pin_status(port, pin)
#endif

PRIVILEGED_DATA static OS_TIMER button_timer;
PRIVILEGED_DATA static TickType_t	press_time;

static void
button_cb(OS_TIMER timer)
{
  (void)timer;
  TickType_t	now = OS_GET_TICK_COUNT();

  if (now - press_time >= LONG_PRESS_TIME)
  {// If the button is pressed long enough start upgrade.
    upgrade_reboot(UPGRADE_DEFAULT);
  }else if (!USER_BTN_DOWN(HW_USER_BTN_PORT, HW_USER_BTN_PIN))
  {// If the button is released already send lora package.
    proto_send_data();
  }
  else
  {// If the button is still pressed check until released.
    OS_TIMER_RESET(button_timer, OS_TIMER_FOREVER);
  }
}

void
button_press(uint32_t time)
{
  // Save the time when the button is pressed to check duration.
  press_time = time;
  if(button_timer == NULL){
    button_timer = OS_TIMER_CREATE("button", OS_MS_2_TICKS(20), \
      OS_TIMER_FAIL, (void *) OS_GET_CURRENT_TASK(), button_cb);

    OS_ASSERT(button_timer);
  }else{
    OS_TIMER_START(button_timer, OS_TIMER_FOREVER);
  }
}

#endif /* FEATURE_USER_BUTTON */
