#ifndef __CAN_COM_H
#define __CAN_COM_H

#ifdef __cplusplus
extern "C" {
#endif


int init_can(void *can_user_data);

// void submit_can_pkt(const void *packet, unsigned int type);


#ifdef __cplusplus
}
#endif

#endif /* __CAN_COM_H */