
#include <zephyr/kernel.h>
#include "can_com.h"

static void heartbeat_thread(void *arg1, void *arg2, void *arg3)
{
    (void)arg1;
    (void)arg2;
    (void)arg3;
    
    while (1) {
        k_sleep(K_MSEC(1000));
        uint8_t data[1] = {0xAB};
        submit_can_pkt(data, 1);
    }
}

#define LOG_THREAD_STACK_SIZE 512
K_THREAD_DEFINE(log_tid, LOG_THREAD_STACK_SIZE, heartbeat_thread, NULL, NULL, NULL, 7, 0, 0);