#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "hw/hw.h"

uint32_t sensor_period(void);

#ifdef FEATURE_SENSOR

#define SENSOR_MAX	4

void sensor_init(void);
void sensor_prepare(void);
uint32_t sensor_data_ready(void);
size_t sensor_get_data(int idx, char *buf, int len);
void sensor_txstart(void);

#else /* !FEATURE_SENSOR */

#define SENSOR_MAX	0

#define sensor_init()
#define sensor_prepare()
#define sensor_data_ready()		((uint32_t)0)
#define sensor_get_data(idx, buf, len)	((size_t)0)
#define sensor_txstart()

#endif /* FEATURE_SENSOR */

#endif /* __SENSOR_H__ */
