#ifndef __GPS_H__
#define __GPS_H__

void gps_init(void);
void gps_prepare(void);
uint32_t gps_data_ready(void);
int	gps_read(char *, int);
void gps_txstart(void);
void gps_rx(void);

#endif /* __GPS_H__ */
