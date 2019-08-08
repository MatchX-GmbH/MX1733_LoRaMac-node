#ifndef __LORA_H__
#define __LORA_H__

/*
 * Notification bits reservation.
 * First 4 bits are reserved for BLE events.
 */
#define EVENT_NOTIF_LORA_DIO  (1 << 5)
#define EVENT_NOTIF_BTN_PRESS (1 << 6)
#define EVENT_NOTIF_CONS_RX   (1 << 7)

void lora_hw_init(void *irq);
void lora_task_func(void *param);
void lora_send(uint8_t *data, size_t len);

#endif /* __LORA_H__ */
