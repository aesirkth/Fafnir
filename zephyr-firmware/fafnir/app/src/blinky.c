#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include "can_com.h"

LOG_MODULE_REGISTER(blinky_test);




static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(rxled), gpios);

static const struct gpio_dt_spec N2_valve = GPIO_DT_SPEC_GET(DT_ALIAS(n2valve), gpios);
static const struct gpio_dt_spec vent_valve = GPIO_DT_SPEC_GET(DT_ALIAS(ventvalve), gpios);
static const struct gpio_dt_spec abort_valve = GPIO_DT_SPEC_GET(DT_ALIAS(abortvalve), gpios);
static const struct gpio_dt_spec extra_valve = GPIO_DT_SPEC_GET(DT_ALIAS(extravalve), gpios);

#define NUM_CHANNELS 4
const struct gpio_dt_spec pyroPins[NUM_CHANNELS] = {N2_valve, vent_valve, abort_valve, extra_valve};

#if !defined(CONFIG_BOARD_NATIVE_SIM) 
static const struct pwm_dt_spec pwm_servo = PWM_DT_SPEC_GET(DT_ALIAS(servo));
#define SERVO_PERIOD PWM_MSEC(20)
#endif

void servoRotate(float angle) {
	//The angle is mapped to -135 to 135 to properly represent CW and CCW rotations
	//Input of +90 == 90 deg rotation CW from the zero position.
    LOG_INF("setting angle of pwm_servo = %d\n", (int) angle);

    {
        if (angle < -135 || angle > 135) return;
        angle = 135.0f + angle; //135 degrees is the zero/middle position, since the servo motor can rotate 270 deg

        float degRatio = angle / 270.0f;

        float duty = degRatio * 0.10f + 0.025f;  //mapping to 2.5%–12.5% duty cycle

        pwm_set_dt(&pwm_servo, SERVO_PERIOD, (uint32_t) SERVO_PERIOD*duty); // move it a bit
    }
}

void servoZero(void) {
	servoRotate(0.0f);
}

void configure_output_pin(const struct gpio_dt_spec *pin) {
	int ret;
    int ready = gpio_is_ready_dt(pin);
    if (!ready) {
        LOG_ERR("pin not ready");
        return;
    }

    ret = gpio_pin_configure_dt(pin, GPIO_OUTPUT);

    if (ret < 0) {
        LOG_ERR("pin configure failed %d", ret);
        return;
    }
}

const struct can_filter filter_off = {
    .flags = 0,
    .id = 0x124,
    .mask = 0b11111111111 
    // .mask = 0x0
};

const struct can_filter filter_on = {
    .flags = 0,
    .id = 0x125,
    .mask = 0b11111111111 
    // .mask = 0x0
};

bool led_state = false;

void can_rx_led_on_cb(const struct device *const device, struct can_frame *frame, void *user_data) {
    led_state = true;
    // gpio_pin_toggle_dt(&led);

    // servoZero();
    // gpio_pin_toggle_dt(&led);
    // // servoRotate(90);
    // k_msleep(1001);
    // gpio_pin_toggle_dt(&led);
}

void can_rx_led_off_cb(const struct device *const device, struct can_frame *frame, void *user_data) {
    led_state = false;
    // gpio_pin_toggle_dt(&led);

    // servoZero();
    // gpio_pin_toggle_dt(&led);
    // // servoRotate(90);
    // k_msleep(1001);
    // gpio_pin_toggle_dt(&led);
}

int main() {
    gpio_pin_configure_dt(&led, GPIO_OUTPUT);
    for (size_t i = 0; i < NUM_CHANNELS; i++) {
        configure_output_pin(&pyroPins[i]);
    }

	if (!pwm_is_ready_dt(&pwm_servo)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_servo.dev->name);
		return 0;
	}

    init_can();
    add_filter_can(can_rx_led_off_cb, filter_off, NULL);
    add_filter_can(can_rx_led_on_cb, filter_on, NULL);
    
    while(true) {
        // gpio_pin_toggle_dt(&led);
        // for (size_t i = 0; i < NUM_CHANNELS; i++) {
        //     gpio_pin_toggle_dt(&pyroPins[i]);
        // }
        gpio_pin_toggle_dt(&abort_valve);
        gpio_pin_toggle_dt(&extra_valve);
        gpio_pin_set_dt(&led, led_state);

        uint8_t data[3] = {5, 6, 7};
        submit_can_pkt(data, 3);
        k_msleep(1001);

    }
    

    return 0;

}
