#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can_test, LOG_LEVEL_INF);

#define CAN_ID_CMD   0x123   // any arbitrary ID
#define CAN_BITRATE  500000

// Define your LED
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// CAN device
const struct device *const can_dev = DEVICE_DT_GET(DT_ALIAS(canbus));

// RX callback
static void can_rx_callback(const struct device* dev, struct can_frame* frame, void* user_data)
{
    if (frame->dlc < 1) {
        LOG_WRN("Empty CAN frame received");
        return;
    }

    uint8_t cmd = frame->data[0];

    if (cmd == 0x01) {
        gpio_pin_set_dt(&led, 1);
        LOG_INF("LED ON via CAN");
    }
    else if (cmd == 0x00) {
        gpio_pin_set_dt(&led, 0);
        LOG_INF("LED OFF via CAN");
    }
    else {
        LOG_WRN("Unknown command: 0x%02X", cmd);
    }
}

int main(void)
{
    int ret;

    // Configure LED
    if (!device_is_ready(led.port)) {
        LOG_ERR("LED device not ready");
        return 0;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED");
        return 0;
    }

    // Configure CAN
    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN device not ready");
        return 0;
    }

    ret = can_set_bitrate(can_dev, CAN_BITRATE);
    if (ret) {
        LOG_ERR("Failed to set CAN bitrate: %d", ret);
        return 0;
    }

    ret = can_start(can_dev);
    if (ret) {
        LOG_ERR("Failed to start CAN: %d", ret);
        return 0;
    }

    struct can_filter filter = {
        .id = CAN_ID_CMD,
        .mask = 0x7FF,
        .flags = 0
    };

    ret = can_add_rx_filter(can_dev, can_rx_callback, NULL, &filter);
    if (ret < 0) {
        LOG_ERR("Failed to add RX filter: %d", ret);
        return 0;
    }

    LOG_INF("CAN test ready. Listening for ID 0x%03X...", CAN_ID_CMD);

    while (1) {
        k_sleep(K_FOREVER);  // nothing else to do
    }
}
