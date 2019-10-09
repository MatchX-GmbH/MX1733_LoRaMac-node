#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <osal.h>
#include <resmgmt.h>
#include <sys_power_mgr.h>
#include <sys_watchdog.h>

#include "hw/cons.h"
#include "hw/hw.h"
#include "hw/i2c.h"
#include "hw/led.h"
#include "hw/power.h"
#include "lora/lora.h"
#include "sensor/sensor.h"

#define BARRIER()   __asm__ __volatile__ ("":::"memory")

#ifdef CONFIG_RETARGET
extern void	retarget_init(void);
#endif

#if dg_configUSE_WDOG
INITIALISED_PRIVILEGED_DATA int8_t	idle_task_wdog_id = -1;
#endif

PRIVILEGED_DATA static OS_TASK lora_task_handle;

__attribute__((__noreturn__)) void
app_failed()
{
  write(1, "app failed\r\n", 12);
  hw_cpm_reboot_system();
  for (;;)
    ;
}

void
vApplicationMallocFailedHook(void)
{
  write(1, "malloc\r\n", 8);
  app_failed();
}

void
vApplicationStackOverflowHook(OS_TASK pxTask, char *pcTaskName)
{
  (void)pxTask;
  (void)pcTaskName;
  write(1, "stack\r\n", 7);
  app_failed();
}

#if dg_configUSE_WDOG
void
vApplicationIdleHook(void)
{
  sys_watchdog_notify(idle_task_wdog_id);
}
#endif

static void
periph_setup(void)
{
  led_init();
  power_init();
  cons_reinit();
  i2c_init();
  sensor_init();
}

static void
sysinit_task_func(void *param)
{
  (void)param;
  cm_sys_clk_init(sysclk_XTAL16M);
  cm_apb_set_clock_divider(apb_div1);
  cm_ahb_set_clock_divider(ahb_div1);
  cm_lp_clk_init();
#if dg_configUSE_WDOG
  sys_watchdog_init();
  // Register the Idle task first.
  idle_task_wdog_id = sys_watchdog_register(false);
  ASSERT_WARNING(idle_task_wdog_id != -1);
  sys_watchdog_configure_idle_id(idle_task_wdog_id);
#endif
  cons_init();
  pm_system_init(periph_setup);
  printf("*** " HW_VER_STRING " ***\r\n");
  resource_init();
#ifdef CONFIG_RETARGET
  retarget_init();
#endif
  pm_set_wakeup_mode(true);
  /*
   * Be aware that pm_mode_extended_sleep as the
   * sleeping mode doesn't work in DEVELOPMENT_MODE.
   * This is due to dg_configENABLE_DEBUGGER
   * in the config file. Therefore, always set
   * sleep mode to pm_mode_idle if debugging.
   * Only decide if you want extended sleep for
   * production builds where debugging is not enabled.
   * */
#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
  pm_set_sleep_mode(pm_mode_idle);
#else
  pm_set_sleep_mode(INITIAL_SLEEP_MODE);
#endif
  cm_sys_clk_set(sysclk_XTAL16M);
  // Create the main loop function and delete the other.
  OS_TASK_CREATE("LoRa Task", lora_task_func, (void *)0,
      2048, OS_TASK_PRIORITY_NORMAL, lora_task_handle);
  OS_TASK_DELETE(OS_GET_CURRENT_TASK());
}

int
main()
{
  OS_TASK	handle;

  cm_clk_init_low_level();
  OS_TASK_CREATE("sysinit", sysinit_task_func, (void *)0,
      1024, OS_TASK_PRIORITY_HIGHEST, handle);
  vTaskStartScheduler();
  for (;;)
    ;
  return 0;
}
