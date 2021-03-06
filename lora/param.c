/* Parameter handling */

#include <stdint.h>

#include <osal.h>
#include <ad_nvparam.h>
#include <platform_nvparam.h>

#include "lora/param.h"
#include "lora/util.h"

#define DEBUG

/* EUI-64: 78af58fffe040000 */
INITIALISED_PRIVILEGED_DATA static uint8_t  deveui64[8] = {
  0x78, 0xaf, 0x58, 0xff, 0xfe, 0x04, 0x00, 0x00,
};
/* EUI-48: 78af58040000 */
INITIALISED_PRIVILEGED_DATA static uint8_t	deveui48[6] = {
  0x00, 0x00, 0x04, 0x58, 0xaf, 0x78,
};
/* 78af580000040000 */
INITIALISED_PRIVILEGED_DATA static uint8_t	appeui[8] = {
  0x78, 0xaf, 0x58, 0x00, 0x00, 0x04, 0x00, 0x00,
};
/* df89dc73d9f52c0609edb2185efa4a34 */
INITIALISED_PRIVILEGED_DATA static uint8_t	devkey[16] = {
  0xdf, 0x89, 0xdc, 0x73, 0xd9, 0xf5, 0x2c, 0x06,
  0x09, 0xed, 0xb2, 0x18, 0x5e, 0xfa, 0x4a, 0x34,
};
PRIVILEGED_DATA static uint8_t			suota, sensor_period, min_sf;

/* NVPARAM "ble_platform" */
#define PARAM_DEV_EUI_OFF	TAG_BLE_PLATFORM_BD_ADDRESS
#define PARAM_DEV_EUI_LEN	sizeof(deveui48)

/* VES */
#define PARAM_APP_EUI_OFF	0
#define PARAM_APP_EUI_LEN	sizeof(appeui)

#define PARAM_DEV_KEY_OFF	(PARAM_APP_EUI_OFF + PARAM_APP_EUI_LEN)
#define PARAM_DEV_KEY_LEN	sizeof(devkey)

#define PARAM_SUOTA_OFF		(PARAM_DEV_KEY_OFF + PARAM_DEV_KEY_LEN)
#define PARAM_SUOTA_LEN		sizeof(suota)

#define PARAM_SENSOR_PERIOD_OFF	(PARAM_SUOTA_OFF + PARAM_SUOTA_LEN)
#define PARAM_SENSOR_PERIOD_LEN	sizeof(sensor_period)

#define PARAM_MIN_SF_OFF	(PARAM_SENSOR_PERIOD_OFF + \
         PARAM_SENSOR_PERIOD_LEN)
#define PARAM_MIN_SF_LEN	sizeof(min_sf)

#define PARAM_FLAG_BLE_NV	0x01	/* Stored in BLE NVPARAM area */
#define PARAM_FLAG_REVERSE	0x02	/* Reversed in protocol */
#define PARAM_FLAG_WRITE_ONLY	0x04	/* "Get param" disallowed */

struct param_def {
  void		*mem;		/* Location in memory */
  uint16_t	 offset;	/* Location in permanent storage */
  uint8_t		 len;		/* Length */
  uint8_t		 flags;		/* Flags */
};

static const struct param_def	params[] = {
  [PARAM_DEV_EUI] = {
    .mem	= deveui48,
    .offset	= PARAM_DEV_EUI_OFF,
    .len	= PARAM_DEV_EUI_LEN,
    .flags	= PARAM_FLAG_BLE_NV | PARAM_FLAG_REVERSE,
  },
  [PARAM_APP_EUI] = {
    .mem	= appeui,
    .offset	= PARAM_APP_EUI_OFF,
    .len	= PARAM_APP_EUI_LEN,
  },
  [PARAM_DEV_KEY] = {
    .mem	= devkey,
    .offset	= PARAM_DEV_KEY_OFF,
    .len	= PARAM_DEV_KEY_LEN,
    .flags	= PARAM_FLAG_WRITE_ONLY,
  },
  [PARAM_SENSOR_PERIOD] = {
    .mem	= &sensor_period,
    .offset	= PARAM_SENSOR_PERIOD_OFF,
    .len	= PARAM_SENSOR_PERIOD_LEN,
  },
  [PARAM_MIN_SF] = {
    .mem	= &min_sf,
    .offset	= PARAM_MIN_SF_OFF,
    .len	= PARAM_MIN_SF_LEN,
  },
  [PARAM_SUOTA] = {
    .mem	= &suota,
    .offset	= PARAM_SUOTA_OFF,
    .len	= PARAM_SUOTA_LEN,
  },
};

static inline void
reverse_memcpy(void *dest, void *src, size_t len)
{
  char	*d = dest, *s = src;

  while (len) {
    d[--len] = *s++;
  }
}

static void
param_copyDevEui64(void)
{
  reverse_memcpy(deveui64 + 5, deveui48, 3);
  deveui64[3] = 0xff;
  deveui64[4] = 0xfe;
  reverse_memcpy(deveui64, deveui48 + 3, 3);
}

/* Read param from permanent storage into memory */
static void
read_param(const struct param_def *param)
{
  uint8_t	buf[PARAM_MAX_LEN];

  if (param->flags & PARAM_FLAG_BLE_NV) {
    nvparam_t	nvparam;
    uint16_t	param_len;
    uint8_t		valid;

    nvparam = ad_nvparam_open("ble_platform");
    param_len = ad_nvparam_get_length(nvparam, param->offset, NULL);
    OS_ASSERT(param_len == param->len + 1);
    ad_nvparam_read_offset(nvparam, param->offset,
        param_len - sizeof(valid), sizeof(valid), &valid);
    if (valid != 0x00)
      return;
    ad_nvparam_read(nvparam, param->offset, param->len, param->mem);
  } else {
    nvms_t	nvms;
    int	i;

    OS_ASSERT(sizeof(buf) >= param->len);
    nvms = ad_nvms_open(NVMS_GENERIC_PART);
    ad_nvms_read(nvms, param->offset, buf, param->len);
    for (i = 0; i < param->len; i++) {
      if (buf[i] != 0xff) {
        memcpy(param->mem, buf, param->len);
        return;
      }
    }
  }
}

/* Set param in memory and write it to permanent storage */
static void
write_param(const struct param_def *param, void *data)
{
  uint8_t		buf[PARAM_MAX_LEN + 1];

  OS_ASSERT(param->len <= sizeof(buf));
  if (param->flags & PARAM_FLAG_REVERSE)
    reverse_memcpy(buf, data, param->len);
  else
    memcpy(buf, data, param->len);
  memcpy(param->mem, buf, param->len);
  if (param->flags & PARAM_FLAG_BLE_NV) {
    nvparam_t	nvparam;
    uint16_t	param_len;

    nvparam = ad_nvparam_open("ble_platform");
    param_len = ad_nvparam_get_length(nvparam, param->offset, NULL);
    OS_ASSERT(param_len == param->len + 1);
    OS_ASSERT(param_len <= sizeof(buf));
    (void)param_len;
    buf[param->len] = 0x00;
    ad_nvparam_write(nvparam, param->offset, param->len + 1, buf);
  } else {
    nvms_t		nvms;

    nvms = ad_nvms_open(NVMS_GENERIC_PART);
    ad_nvms_write(nvms, param->offset, buf, param->len);
  }
}

int
param_get(int idx, uint8_t *data, uint8_t len)
{
  if (idx >= (int)ARRAY_SIZE(params) || params[idx].len > len ||
      (params[idx].flags & PARAM_FLAG_WRITE_ONLY)) {
    return 0;
  }
  if (params[idx].flags & PARAM_FLAG_REVERSE)
    reverse_memcpy(data, params[idx].mem, params[idx].len);
  else
    memcpy(data, params[idx].mem, params[idx].len);
  return params[idx].len;
}

int
param_set(int idx, uint8_t *data, uint8_t len)
{
  if (idx >= (int)ARRAY_SIZE(params) || params[idx].len != len)
    return -1;
  write_param(params + idx, data);
  return 0;
}

void
param_init(void)
{
  int	i;

  for (i = 0; i < (int)ARRAY_SIZE(params); i++) {
    read_param(params + i);
#ifdef DEBUG
    if (params[i].flags & PARAM_FLAG_REVERSE) {
      for (int j = params[i].len - 1; j >= 0; j--)
        printf("%02x", ((uint8_t *)params[i].mem)[j]);
    } else {
      for (int j = 0; j < params[i].len; j++)
        printf("%02x", ((uint8_t *)params[i].mem)[j]);
    }
    printf("\r\n");
#endif
  }
#ifdef DEBUG
  {
    param_copyDevEui64();
    for (uint8_t j = 0 ; j < sizeof(deveui64) ; j++)
      printf("%02x", deveui64[j]);
    printf("\r\n");
  }
#endif
}

uint8_t* param_get_addr(int idx)
{
  uint8_t* addr;
  switch(idx){
  case PARAM_DEV_EUI:
    addr = deveui64;
    break;
  case PARAM_APP_EUI:
    addr = appeui;
      break;
  case PARAM_DEV_KEY:
    addr = devkey;
      break;
  default:
    addr = NULL;
    break;
  }
  return addr;
}
