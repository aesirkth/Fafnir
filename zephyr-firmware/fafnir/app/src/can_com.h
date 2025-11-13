#ifndef __CAN_COM_H
#define __CAN_COM_H

#ifdef __cplusplus
extern "C" {
#endif


int init_can(void *can_user_data);
void send_test_frame(void);


#ifdef __cplusplus
}
#endif

#endif /* __CAN_COM_H */