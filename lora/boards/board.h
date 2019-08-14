/*
 * board.h
 *
 *  Created on: 7 Aug 2019
 *      Author: balog
 */

#ifndef LORA_BOARD_H_
#define LORA_BOARD_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <hw_gpio.h>
#include <hw_spi.h>
#include <hw_wkup.h>

#include "osal.h"

#include "lora/lora.h"

/*!
 * Structure for the GPIO
 */
typedef struct
{
  HW_GPIO_PORT port;
  HW_GPIO_PIN pin;
}Gpio_t;

/*!
 * Structure for the SPI
 */
typedef struct
{
  spi_config *spi_conf;
}Spi_t;

/*!
 * \brief Timer time variable definition
 */
#ifndef TimerTime_t
typedef uint32_t TimerTime_t;
#endif

#include "hw/hw.h"
#include "lora/lora.h"
#include "lora/utilities.h"
#include "lora/radio/radio.h"
#include "lora/radio/sx1276/sx1276.h"
#include "lora/boards/sx1276-board.h"

#define DelayMs(ms)  OS_DELAY_MS(ms)

#define USE_BAND_868

#endif /* LORA_BOARD_H_ */
