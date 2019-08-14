#ifndef __LORA_H__
#define __LORA_H__

/*
 * Notification bits reservation.
 * First 5 bits are reserved for BLE events.
 */
#define EVENT_NOTIF_LORA_DIO0 (1 << 5)
#define EVENT_NOTIF_LORA_DIO1 (1 << 6)
#define EVENT_NOTIF_LORA_DIO2 (1 << 7)
#define EVENT_NOTIF_BTN_PRESS (1 << 8)
#define EVENT_NOTIF_CONS_RX   (1 << 9)
#define EVENT_NOTIF_GPS_RX    (1 << 10)
#define EVENT_NOTIF_LORAMAC   (1 << 11)

void lora_hw_init(void *irq);
void lora_task_func(void *param);
void lora_task_notify_event(uint32_t event, void *cb);
void lora_send(uint8_t *data, size_t len);

#endif /* __LORA_H__ */
