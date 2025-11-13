#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <errno.h>

LOG_MODULE_REGISTER(can_com);

K_MSGQ_DEFINE(can_tx_queue,
              sizeof(struct can_frame),
              8,            /* queue length (frames) */
              1);            /* alignment */
K_SEM_DEFINE(can_tx_done, 1, 1);

const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));


// TODO: Check these filter values
const struct can_filter filter = {
    .flags = 0,
    .id = 0x800,
    .mask = 0b11110000000 // match from 0x800 to 0x87F - maybe?
};

void can_rx_cb(const struct device *const device, struct can_frame *frame, void *user_data) {
    LOG_DBG("rx: %#X: frame->dlc: %d", frame->id, frame->dlc);

     

    // TODO: should probably check size.
    // if (frame->dlc != pkt_size[pkt_type]) {
    //     LOG_ERR("received packet %#x has length %d but should be length %d", pkt_type, frame->dlc, pkt_size[pkt_type]);
    // }

    memcpy(user_data, frame->data, frame->dlc);
    
}

void can_tx_cb(const struct device *device, int error, void *user_data) {
    k_sem_give(&can_tx_done);
    if (error) {
        LOG_ERR("can send callback error %d", error);
    }
}

// int submit_can_pkt(const void *packet, unsigned int type) {
//     const int length = pkt_size[type];
//     struct can_frame frame = {
//             .flags = 0,
//             .id = type + 0x700,
//             .dlc = length,
//     };
//     memcpy(frame.data, packet, length);
//     int ret = k_msgq_put(&can_tx_queue, &frame, K_NO_WAIT);
//     if (ret == -EAGAIN) {
//         LOG_ERR("can tx queue full");
//     }
//     return 0;
// }

// void can_thread_fn(void) {
//     int ret;
//     struct can_frame frame;

//     while (1) {
//         ret = k_msgq_get(&can_tx_queue, &frame, K_FOREVER);
//         if (ret == -ENOMSG) { // queue purged during wait
//             continue; 
//         }

//         // wait until previous frame is sent
//         k_sem_take(&can_tx_done, K_MSEC(200));

//         LOG_INF("sending can message 0x%02X", frame.id);

//         // send frame (blocking until mailbox is available)
//         ret = can_send(can_dev, &frame,
//                            K_MSEC(100),
//                             can_tx_cb,
//                            NULL);

//         if (ret) { // callback not called
//             LOG_ERR("can send error %d", ret);
//             k_sem_give(&can_tx_done);
//         }
//     }
// }

// K_THREAD_DEFINE(can_thread,
//                 1024,
//                 can_thread_fn,
//                 NULL, NULL, NULL,
//                 1, // priority
//                 0,
//                 -1); // do not start

int init_can(void *can_user_data) {
    int ret;
 
    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN device not ready");
        return -1;
    }
    
    ret = can_add_rx_filter(can_dev, can_rx_cb, can_user_data, &filter);
    if (ret) {
        LOG_ERR("adding can filter failed: %d", ret);
    } else {
        LOG_INF("adding can filter success");
    }

    ret = can_set_bitrate(can_dev, 500000); // 500 kb/s
    if (ret) {
        LOG_ERR("failed to set CAN nominal bitrate: [%d]", ret);
        return -1;
    } else {
        LOG_INF("CAN nominal bitrate successfully set to 500kb/s");
    }

    ret = can_set_bitrate_data(can_dev, 500000); // 500 kb/s
    if (ret) {
        LOG_ERR("failed to set CAN bitrate: [%d]", ret);
        return -1;
    } else {
        LOG_INF("CAN bitrate successfully set to 500kb/s");
    }

    ret = can_start(can_dev);
    if (ret) {
        LOG_ERR("failed to start CAN: %d", ret);
        return -1;
    } else{
        LOG_INF("CAN started");
    }

    // k_thread_start(can_thread);

    return 0;
}