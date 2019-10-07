#include <string.h>
#include <limits.h>

#include <hw_gpio.h>
#include <hw_uart.h>

#include "osal.h"

#include "hw/hw.h"
#include "hw/power.h"
#include "lora/lora.h"
#include "lora/util.h"
#include "gps.h"

#ifdef FEATURE_SENSOR_GPS

//#define DEBUG

#ifdef DEBUG
//#define DEBUG_GSV
#include <stdio.h>
#include <unistd.h>
#endif

PRIVILEGED_DATA static char	rxbuf[128];
PRIVILEGED_DATA static int	rxlen;
PRIVILEGED_DATA static OS_TIMER gps_rx_timer;

/* $GPGGA,155058.000,,,,,0,0,,,M,,M,,*44 */
/* $GPGGA,135704.000,5231.1618,N,01324.2888,E,1,3,5.64,105.3,M,44.7,M,,*59 */
enum {
  GPGGA_MSGID,		/* Message ID */
  GPGGA_TIME,		/* UTC time */
  GPGGA_LAT,		/* Latitude */
  GPGGA_NS,		/* N/S Indicator */
  GPGGA_LON,		/* Longitude */
  GPGGA_EW,		/* E/W Indicator */
  GPGGA_FIX,		/* Position Fix Indicator */
  GPGGA_SAT,		/* Satellites Used */
  GPGGA_HDOP,		/* HDOP, Hotizontal Dilution of Precision */
  GPGGA_MSL_ALT,		/* MSL Altitude */
  GPGGA_MSL_UNITS,	/* Units ("M") */
  GPGGA_GEOSEP,		/* Geoid Separation */
  GPGGA_GEOSEP_UNITS,	/* Units ("M") */
  GPGGA_AGE_DIFF,		/* Age of Diff. Corr. */
  GPGGA_STATION_ID,	/* Diff. Ref. Station ID */
};

#ifdef DEBUG_GSV

/* $GPGSV,1,1,00*79 */
/* $GPGSV,3,1,11,09,77,234,,23,73,076,,06,55,267,,03,37,114,*75 */
/* $GPGSV,3,1,11,26,68,023,37,15,64,251,33,05,45,058,34,29,33,253,33*75 */
enum {
  GPGSV_MSGID,    /* Message ID */
  GPGSV_MSGS,   /* Number of Messages */
  GPGSV_MSGNO,    /* Message Number */
  GPGSV_SAT,    /* Satellites in View */
};
#define GPGSV_SAT_BASE    (GPGSV_SAT + 1)

enum {
  GPGSV_SAT_ID,   /* Satellite ID */
  GPGSV_SAT_ELEV,   /* Elevation */
  GPGSV_SAT_AZ,   /* Asimuth */
  GPGSV_SAT_SNR,    /* SNR (C/N0) */
};
#define GPGSV_SAT_FIELDS  (GPGSV_SAT_SNR + 1)
#define GPGSV_MAX_SAT   4

#define MAX_SATS  32

struct sat {
  uint8_t id;
  uint8_t snr;
} __attribute__((packed));

PRIVILEGED_DATA struct sat  sat[MAX_SATS];
PRIVILEGED_DATA uint8_t   sats;

#define SATS_START  0
#define SATS_RECEIVING  1
#define SATS_DONE 2
PRIVILEGED_DATA static uint8_t  sats_status;

static const char gpgsv[] = "$GPGSV";

#define MAX_FIELDS  (GPGSV_SAT_BASE + GPGSV_SAT_FIELDS * GPGSV_MAX_SAT)

#else

#define MAX_FIELDS	(GPGGA_STATION_ID + 1)

#endif

struct datapart {
  char	*s;
  int	len;
};

struct gps_fix {
  uint8_t	fix;
  int32_t	lat;	/* Positive: North */
  int32_t	lon;	/* Positive: East */
  int16_t	alt;	/* MSL altitude */
} __attribute__((packed));

PRIVILEGED_DATA static struct gps_fix	last_fix;

#define STATUS_CONNECTED          0x01
#define STATUS_GPS_INFO_RECEIVED  0x02
#define STATUS_GPS_FIX_FOUND      0x04
PRIVILEGED_DATA static uint8_t	status;

static const char	hex[] = {
  '0', '1', '2', '3', '4', '5', '6', '7',
  '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
};
static const char	crlf[] = { '\r', '\n', };
static const char	gpgga[] = "$GPGGA";

static bool
valid_crc(char *msg, int len)
{
  char	*p;
  int	 ccrc, icrc; /* computed and indicated CRCs */

  if (*msg != '$')
    return false;
  /* String must end with "*XX\r\n", where XX are two hex digits. */
  p = memchr(msg, '*', len);
  if (p != msg + len - 5)
    return false;
  if (memcmp(msg + len - sizeof(crlf), crlf, sizeof(crlf)) != 0)
    return false;
  if ((p = memchr(hex, msg[len - 4], sizeof(hex))) == NULL)
    return false;
  icrc = (p - hex) << 4;
  if ((p = memchr(hex, msg[len - 3], sizeof(hex))) == NULL)
    return false;
  icrc |= p - hex;
  ccrc = 0;
  for (p = msg + 1; p < msg + len - 5; p++) {
    ccrc ^= *p;
  }
  return ccrc == icrc;
}

#define MAXLAT	9000
#define MAXLON	18000
static int32_t
parse_latlon(char *s, int max, bool negate)
{
  const char	*errstr;
  char		*frac;
  int32_t		 d, m, f;	/* degrees, minutes, fraction */
  int		 i;

  if ((frac = strchr(s, '.')) == NULL)
    return 0;
  *frac++ = '\0';
  m = strtonum(s, 0, max, &errstr);
  if (errstr)
    return 0;
  d = m / 100;
  m %= 100;
  f = strtonum(frac, 0, 9999, &errstr);
  if (errstr)
    return 0;
  for (i = 4 - (int)strlen(frac); i > 0; i--)
    f *= 10;
  return ((d * 60 + m) * 10000 + f) * (negate ? -1 : 1);
}

static int16_t
parse_alt(char *s)
{
  const char	*errstr;
  char		*frac;
  int16_t		 m, dm;
  bool		 negate = false;

  if (*s == '-') {
    negate = true;
    s++;
  }
  if ((frac = strchr(s, '.')) == NULL)
    return 0;
  *frac++ = '\0';
  m = strtonum(s, 0, SHRT_MAX / 10 - 1, &errstr);
  if (errstr)
    return 0;
  dm = strtonum(frac, 0, 9, &errstr);
  if (errstr)
    return 0;
  return (m * 10 + dm) * (negate ? -1 : 1);
}

static void
proc_gpgga(char *data[])
{
  struct gps_fix	 fix = {};
  const char	*errstr;

#ifdef DEBUG
  printf("gga\r\n");
#endif
  fix.fix = strtonum(data[GPGGA_FIX], 0, 6, &errstr);
  if (fix.fix) {
    fix.lat = parse_latlon(data[GPGGA_LAT], MAXLAT,
        data[GPGGA_NS][0] == 'S');
    fix.lon = parse_latlon(data[GPGGA_LON], MAXLON,
        data[GPGGA_EW][0] == 'W');
    fix.alt = parse_alt(data[GPGGA_MSL_ALT]);
    memcpy(&last_fix, &fix, sizeof(fix));
  }
}

#ifdef DEBUG_GSV

static void
gps_print_sats()
{
  int i;
  for (i = 0; i < sats; i++)
    printf("gps satellite %2d, snr %2d\r\n", sat[i].id, sat[i].snr);
}

static void
proc_gpgsv(char *data[], int fields)
{
  const char  *errstr;
  int    i, msgs, msgno, tots;

#ifdef DEBUG
  printf("gsv\r\n");
#endif
  msgs = strtonum(data[GPGSV_MSGS], 0, 16, &errstr);
  if (errstr)
    return;
  msgno = strtonum(data[GPGSV_MSGNO], 0, 16, &errstr);
  if (errstr)
    return;
  tots = strtonum(data[GPGSV_SAT], 0, MAX_SATS, &errstr);
  if (errstr)
    return;
  if (msgno == 1 && sats_status != SATS_DONE) {
    sats_status = SATS_RECEIVING;
    sats = 0;
  } else if (sats_status != SATS_RECEIVING)
    return;
  for (i = GPGSV_SAT_BASE;
      i + GPGSV_SAT_FIELDS - 1 <= fields && sats < tots;
      i += GPGSV_SAT_FIELDS, sats++) {
    sat[sats].id = strtonum(data[i + GPGSV_SAT_ID], 1, MAX_SATS,
        &errstr);
    if (errstr)
      return;
    if (fields <= i + GPGSV_SAT_SNR ||
        *data[i + GPGSV_SAT_SNR] == '\0') {
      sat[sats].snr = -1; // XXX
    } else {
      sat[sats].snr = strtonum(data[i + GPGSV_SAT_SNR], 1, 99,
          &errstr);
      if (errstr)
        return;
    }
  }
  if (msgno == msgs) {
    sats_status = SATS_DONE;
    gps_print_sats();
  }
}

#endif

static bool
msgproc(char *msg, int len)
{
  char	*data[MAX_FIELDS];
  char	*s, *p;
  int	 i;

#ifdef DEBUG
  write(1, msg, len);
#endif
  if (!valid_crc(msg, len))
    return false;
  msg[len - 5] = '\0';
  i = 0;
  for (s = msg; *s != '\0'; s = p + 1) {
    data[i++] = s;
    if (i == ARRAY_SIZE(data) || (p = strchr(s, ',')) == NULL)
      break;
    *p = '\0';
  }
  if (i) {
    if (strcmp(data[0], gpgga) == 0) {
      proc_gpgga(data);
      return true;
    }
#ifdef DEBUG_GSV
    else if (strcmp(data[0], gpgsv) == 0) {
      proc_gpgsv(data, i);
      return false;
    }
#endif
  }
  return false;
}

static void
gps_uart_rx(OS_TIMER timer)
{
  (void)(timer);
  lora_task_notify_event(EVENT_NOTIF_GPS_RX, NULL);
}

void gps_rx(void)
{
#ifdef DEBUG
  if((!hw_uart_read_buf_empty(HW_UART2)) \
    && (rxlen == 0)){
    printf("gps rx:\r\n");
  }
#endif
  while (!hw_uart_read_buf_empty(HW_UART2)) {
    uint8_t	c;

    if (rxlen >= (int)sizeof(rxbuf)) {
      rxlen = 0;
      continue;
    }
    c = hw_uart_read(HW_UART2);
    if ((rxbuf[rxlen++] = c) == '\n') {
      if (msgproc(rxbuf, rxlen)){
        if(last_fix.fix != 0){
          status |= STATUS_GPS_FIX_FOUND;
        }
        status |= STATUS_GPS_INFO_RECEIVED;
      }
      rxlen = 0;
    }
  }
  if (!(status & STATUS_GPS_INFO_RECEIVED))
  {
    OS_TIMER_START(gps_rx_timer, OS_TIMER_FOREVER);
  }
#ifdef DEBUG_GSV
  if (sats_status != SATS_DONE)
  {
    OS_TIMER_START(gps_rx_timer, OS_TIMER_FOREVER);
  }
#endif
#ifdef DEBUG
  else
  {
    printf("gps rx end.\r\n");
  }
#endif
}

void
gps_init()
{
  const uart_config	uart2_cfg = {
    .baud_rate		= HW_UART_BAUDRATE_9600,
    .data			= HW_UART_DATABITS_8,
    .parity			= HW_UART_PARITY_NONE,
    .stop			= HW_UART_STOPBITS_1,
    .auto_flow_control	= 0,
    .use_dma		= 0,
    .use_fifo		= 1,
  };

  hw_gpio_set_pin_function(HW_SENSOR_UART_TX_PORT, HW_SENSOR_UART_TX_PIN,
      HW_GPIO_MODE_INPUT,  HW_GPIO_FUNC_GPIO);
#if 0
  hw_gpio_set_pin_function(HW_SENSOR_UART_TX_PORT, HW_SENSOR_UART_TX_PIN,
      HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_UART2_TX);
#endif
  hw_gpio_set_pin_function(HW_SENSOR_UART_RX_PORT, HW_SENSOR_UART_RX_PIN,
      HW_GPIO_MODE_INPUT,  HW_GPIO_FUNC_UART2_RX);
  hw_uart_init(HW_UART2, &uart2_cfg);

  if(gps_rx_timer == NULL){
    gps_rx_timer = OS_TIMER_CREATE("gpsuartrx", OS_MS_2_TICKS(10), \
      OS_TIMER_FAIL, (void *) OS_GET_CURRENT_TASK(), gps_uart_rx);

    OS_ASSERT(gps_rx_timer);
  }
}

void
gps_prepare()
{
  status |= STATUS_CONNECTED;
#ifdef DEBUG_GSV
  sats_status = SATS_START;
#endif
#ifdef DEBUG
  printf("gps status %02x\r\n", status);
#endif
  rxlen = 0;
  memset(&last_fix, 0, sizeof(last_fix));
  status &= ~STATUS_GPS_INFO_RECEIVED;
  status &= ~STATUS_GPS_FIX_FOUND;
  while (!hw_uart_read_buf_empty(HW_UART2))
    hw_uart_read(HW_UART2);
  OS_TIMER_START(gps_rx_timer, OS_TIMER_FOREVER);
}

TickType_t
gps_data_ready()
{
#ifdef DEBUG_GSV
  return (status & (STATUS_GPS_FIX_FOUND | STATUS_GPS_INFO_RECEIVED)) && \
    sats_status == SATS_DONE ? 0 : OS_MS_2_TICKS(100);
#else
  return status & (STATUS_GPS_FIX_FOUND | STATUS_GPS_INFO_RECEIVED) ? \
    0 : OS_MS_2_TICKS(100);
#endif
}

int
gps_read(char *buf, int len)
{
  OS_TIMER_STOP(gps_rx_timer, OS_TIMER_FOREVER);
  if (len < (int)sizeof(last_fix) || last_fix.fix == 0) {
    if (len < 1 || !(status & STATUS_CONNECTED))
      return 0;
    buf[0] = !(status & STATUS_GPS_FIX_FOUND);
    return 1;
  }
  memcpy(buf, &last_fix, sizeof(last_fix));
  return sizeof(last_fix);
}

#endif /* FEATURE_SENSOR_GPS */
