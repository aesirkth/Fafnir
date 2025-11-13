#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(can_com);

const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

static const struct can_filter filter = {
    .flags = 0,
    .id = 0x123,
    .mask = 0b11110000000
};

static void can_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
    memcpy(user_data, frame->data, frame->dlc);
}

void send_test_frame(void)
{
    struct can_frame frame = {
        .id = 0x123,
        .dlc = 2,
        .flags = 0
    };

    frame.data[0] = 0xAB;
    frame.data[1] = 0xCD;

    int ret = can_send(can_dev, &frame, K_MSEC(10), NULL, NULL);
    if (ret < 0) {
        LOG_ERR("CAN TX error %d", ret);
    }
}

int init_can(void *rx_buffer)
{
    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN device not ready");
        return -1;
    }

    int ret = can_add_rx_filter(can_dev, can_rx_cb, rx_buffer, &filter);
    if (ret < 0) {
        LOG_ERR("RX filter error %d", ret);
        return -1;
    }

    ret = can_set_bitrate(can_dev, 500000);
    if (ret < 0) {
        LOG_ERR("Bitrate error %d", ret);
        return -1;
    }

    ret = can_start(can_dev);
    if (ret < 0) {
        LOG_ERR("Start error %d", ret);
        return -1;
    }

    return 0;
}
