#ifndef __PROTO_H__
#define __PROTO_H__

void	proto_handle(uint8_t port, uint8_t *data, uint8_t len);
void	proto_send_periodic_data(osjob_t *job);
void	proto_txstart(void);

#endif /* __PROTO_H__ */
