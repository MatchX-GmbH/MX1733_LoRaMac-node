#include <FreeRTOS.h>
#include <hw_gpio.h>

#include "hw.h"
#include "power.h"

void
power_init()
{
#ifdef FEATURE_POWER_SUPPLY
	hw_gpio_configure_pin(HW_PS_EN_PORT, HW_PS_EN_PIN,
	  HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO, true);
#endif /* FEATURE_POWER_SUPPLY */
}

void
power(bool on)
{
#ifdef FEATURE_POWER_SUPPLY
  hw_gpio_configure_pin(HW_PS_EN_PORT, HW_PS_EN_PIN,
    HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO, on);
#endif /* FEATURE_POWER_SUPPLY */
}


