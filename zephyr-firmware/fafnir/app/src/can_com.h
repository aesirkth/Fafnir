#ifndef __CAN_COM_H
#define __CAN_COM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

int init_can(void *can_user_data);

void submit_can_pkt(const void *packet, size_t length);


#ifdef __cplusplus
}
#endif

#endif /* __CAN_COM_H */