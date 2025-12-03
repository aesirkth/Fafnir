#ifndef __CAN_COM_H
#define __CAN_COM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <zephyr/drivers/can.h>

int init_can();

void submit_can_pkt(const void *packet, size_t length);

int add_filter_can(can_rx_callback_t rx_callback, struct can_filter filter, void *can_user_data);


#ifdef __cplusplus
}
#endif

#endif /* __CAN_COM_H */