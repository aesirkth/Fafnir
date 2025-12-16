/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_com.h"

#if defined(CONFIG_BOARD_NATIVE_SIM) 
#include "gpio_emul_shell.h"
#endif

#include "main.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   4000


LOG_MODULE_REGISTER(main_func);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(rxled), gpios);

// Disable servo pwm if using board Native Sim 
// #if !defined(CONFIG_BOARD_NATIVE_SIM) 
// static const struct pwm_dt_spec pwm_servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
// #define SERVO_PERIOD PWM_MSEC(20)
// #endif

#if !defined(CONFIG_BOARD_NATIVE_SIM) 
static const struct pwm_dt_spec pwm_servo = PWM_DT_SPEC_GET(DT_ALIAS(servo));
#define SERVO_PERIOD PWM_MSEC(20)
#endif

static const struct gpio_dt_spec N2_valve = GPIO_DT_SPEC_GET(DT_ALIAS(n2valve), gpios);
static const struct gpio_dt_spec vent_valve = GPIO_DT_SPEC_GET(DT_ALIAS(ventvalve), gpios);
static const struct gpio_dt_spec abort_valve = GPIO_DT_SPEC_GET(DT_ALIAS(abortvalve), gpios);
static const struct gpio_dt_spec extra_valve = GPIO_DT_SPEC_GET(DT_ALIAS(extravalve), gpios);

// static const struct gpio_dt_spec N2_sense = GPIO_DT_SPEC_GET(DT_ALIAS(n2sense), gpios);
// static const struct gpio_dt_spec vent_sense = GPIO_DT_SPEC_GET(DT_ALIAS(ventsense), gpios);
// static const struct gpio_dt_spec main_sense = GPIO_DT_SPEC_GET(DT_ALIAS(mainsense), gpios);
// static const struct gpio_dt_spec abort_sense = GPIO_DT_SPEC_GET(DT_ALIAS(abortsense), gpios);
// static const struct gpio_dt_spec ignition_sense = GPIO_DT_SPEC_GET(DT_ALIAS(ignitionsense), gpios);

#define NUM_CHANNELS 4
// const struct gpio_dt_spec solenoidSense[NUM_CHANNELS] = {N2_sense, vent_sense, abort_sense, ignition_sense};
const struct gpio_dt_spec pyroPins[NUM_CHANNELS] = {N2_valve, vent_valve, abort_valve, extra_valve};


typedef enum {
    STATE_IDLE,
    STATE_INIT,
    STATE_FILL,
    STATE_STOP_FILL,
    STATE_UMBILICAL,
    STATE_N2_PRESSURIZATION,
    STATE_IGNITION_1,
    STATE_IGNITION_2,
    STATE_IGNITION_3,
    STATE_IGNITION_4,
    STATE_SAFE,
    STATE_ABORT_BEFORE_COUNTDOWN,
    STATE_ABORT_AFTER_COUNTDOWN
} State_t;

State_t systemState = STATE_IDLE;
bool trigger = true;

bool isStateBeforeCountdown(State_t state) {
    switch (state) {
    case STATE_IDLE:
        return true;
    case STATE_INIT:
        return true;
    case STATE_FILL:
        return true;
    case STATE_STOP_FILL:
        return true;
    case STATE_UMBILICAL:
        return true;
    case STATE_N2_PRESSURIZATION:
        return true;
    case STATE_IGNITION_1:
        return true;
    case STATE_IGNITION_2:
        return true;
    case STATE_IGNITION_3:
        return false; // TODO: What counts as "before countdown"?
    case STATE_IGNITION_4:
        return false;
    case STATE_SAFE:
        return false;
    case STATE_ABORT_BEFORE_COUNTDOWN:
        return false;
    case STATE_ABORT_AFTER_COUNTDOWN:
        return false;
    }
    LOG_ERR("Invalid state passed!");
    return true;
}

/* 
 * A CAN message with `data`, is interpreted as a command meaning:
 * "Change your state to `state_can_id_map[data]`".
*/
State_t state_can_id_map[] = {
    STATE_INIT,
    STATE_FILL,
    STATE_STOP_FILL,
    STATE_UMBILICAL,
    STATE_N2_PRESSURIZATION,
    STATE_IGNITION_1,
    STATE_SAFE,
    STATE_ABORT_BEFORE_COUNTDOWN // Used for any abort message, not just before countdown.
};

void changeStateToIgnition2_cb() { 
    if(systemState != STATE_IGNITION_1) return;

    systemState = STATE_IGNITION_2;
    LOG_INF("changeStateToIgnition2_cb triggered going into STATE_IGNITION_2!");
    trigger = true;
}

void changeStateToIgnition3_cb() { 
    if(systemState != STATE_IGNITION_2) return;

    systemState = STATE_IGNITION_3;
    LOG_INF("changeStateToIgnition3_cb triggered going into STATE_IGNITION_3!");
    trigger = true;
}

void changeStateToIgnition4_cb() { 
    if(systemState != STATE_IGNITION_3) return;

    systemState = STATE_IGNITION_4;
    LOG_INF("changeStateToIgnition4_cb triggered going into STATE_IGNITION_4!");
    trigger = true;
}

void changeStateToSafeing_cb() { 
    if(systemState != STATE_IGNITION_4) return;

    systemState = STATE_SAFE;
    LOG_INF("changeStateToSafeing_cb triggered going into STATE_SAFE!");
    trigger = true;
}

// uint8_t pyroSense(uint8_t index) {
// 	if (index >= NUM_PYROS) return 69; //69 means error
//     //HIGH = continuity detected (return 1)
//     //LOW = open circuit (return 0)

// 	int state = gpio_pin_get_dt(&solenoidSense[index]);

//     return state;
// }

void servoZero(void) {
	servoRotate(0.0f);
}

void servoRotate(float angle) {
	//The angle is mapped to -135 to 135 to properly represent CW and CCW rotations
	//Input of +90 == 90 deg rotation CW from the zero position.
    LOG_INF("setting angle of pwm_servo = %d\n", (int) angle);

    #if !defined(CONFIG_BOARD_NATIVE_SIM)
    {
        if (angle < -135 || angle > 135) return;
        angle = 135.0f + angle; //135 degrees is the zero/middle position, since the servo motor can rotate 270 deg

        float degRatio = angle / 270.0f;

        float duty = degRatio * 0.10f + 0.025f;  //mapping to 2.5%–12.5% duty cycle

        pwm_set_dt(&pwm_servo, SERVO_PERIOD, (uint32_t) SERVO_PERIOD*duty); // move it a bit
    }
    #endif
}

void set_pin(const struct gpio_dt_spec *pin, bool value) {
    LOG_INF("PIN[%d] set to value %d", pin->pin, value);
    gpio_pin_set_dt(pin, value);
}


const struct can_filter filter = {
    .flags = 0,
    .id = 0x123,
    .mask = 0b11111111111 
};
void can_rx_cb(const struct device *const device, struct can_frame *frame, void *user_data) {

    if (frame->dlc != 1) {
        LOG_ERR("received packet with id %d has length %d. Fafnir expects all packets to have size 1.", frame->id, frame->dlc);
    }

    if(systemState == STATE_ABORT_BEFORE_COUNTDOWN || systemState == STATE_ABORT_AFTER_COUNTDOWN) {
        LOG_ERR("CAN message recieved but ignored because system is aborted.");
        return;
    }

    size_t message_data = frame->data[0];
    State_t message_state = state_can_id_map[message_data];

    if (message_state == STATE_ABORT_BEFORE_COUNTDOWN) { // STATE_ABORT_BEFORE_COUNTDOWN represents any abort
        if(isStateBeforeCountdown(systemState)) {
            systemState = STATE_ABORT_BEFORE_COUNTDOWN;
        } else {
            systemState = STATE_ABORT_AFTER_COUNTDOWN;
        }
    } else {
        systemState = state_can_id_map[message_data];
    }

    LOG_INF("Recieved CAN message rx: %#X: frame->dlc: %d. Switching state to %d", frame->id, frame->dlc, systemState);

    trigger = true;
}

const struct can_filter override_filter = {
    .flags = 0,
    .id = 0x124,
    .mask = 0b11111111111 
};
void can_rx_override_cb(const struct device *const device, struct can_frame *frame, void *user_data) {

    if (frame->dlc != 2) {
        LOG_ERR("received packet with id %d has length %d. Override requires 2 arguments PIN, VALUE.", frame->id, frame->dlc);
    }
    size_t pin = frame->data[0];
    size_t value = frame->data[1];

    LOG_INF("Recieved CAN message rx: %#X: frame->dlc: %d. Setting pin[%d] := %d", frame->id, frame->dlc, pin, value);
    switch (pin) {
    case 0:
        set_pin(&N2_valve, value);
        break;
    case 1:
        set_pin(&vent_valve, value);
        break;
    case 2:
        set_pin(&abort_valve, value);
        break;
    case 3:
        set_pin(&extra_valve, value);
        break;
    case 4:
        int8_t signed_value = (int8_t) value;
        servoRotate(90.0 * ((double) signed_value/127.0) );
        break;

    }
}

static uint8_t data[2];


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

void configure_input_pin(const struct gpio_dt_spec *pin) {
	int ret;
    int ready = gpio_is_ready_dt(pin);
    if (!ready) {
        LOG_ERR("pin not ready");
        return;
    }

    ret = gpio_pin_configure_dt(pin, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("pin configure failed %d", ret);
        return;
    }
}

K_TIMER_DEFINE(TMinus7_timer, changeStateToIgnition2_cb, NULL);
K_TIMER_DEFINE(TMinus2_timer, changeStateToIgnition3_cb, NULL);
K_TIMER_DEFINE(TMinus0_timer, changeStateToIgnition4_cb, NULL);
K_TIMER_DEFINE(TPlus30_timer, changeStateToSafeing_cb, NULL);

void evaluateState() { 
    LOG_INF("Evaluating state: %d", systemState);
    switch(systemState) {
        case STATE_IDLE:
            set_pin(&N2_valve, 0);
            set_pin(&vent_valve, 0);
            set_pin(&abort_valve, 0);
            servoZero(); // Close Main Valve TODO: Is this correct? 
            set_pin(&led, 0);
        break;
        case STATE_INIT:
            set_pin(&N2_valve, 0);
            set_pin(&vent_valve, 1);
            set_pin(&abort_valve, 0);
            servoZero();
            set_pin(&led, 0);
        break;
        case STATE_FILL:
            // Same as STATE_INIT
            set_pin(&N2_valve, 0);
            set_pin(&vent_valve, 1);
            set_pin(&abort_valve, 0);
            servoZero();
            set_pin(&led, 0);
        break;

        case STATE_STOP_FILL:
            set_pin(&N2_valve, 0);
            set_pin(&vent_valve, 0);
            set_pin(&abort_valve, 0);
            servoZero();
            set_pin(&led, 0);
        break;

        case STATE_UMBILICAL:
            // Same as STATE_STOP_FILL
            set_pin(&N2_valve, 0);
            set_pin(&vent_valve, 0);
            set_pin(&abort_valve, 0);
            servoZero();
            set_pin(&led, 0);
        break;

        case STATE_N2_PRESSURIZATION:
            set_pin(&N2_valve, 1);
            set_pin(&vent_valve, 0);
            set_pin(&abort_valve, 0);
            servoZero();
            set_pin(&led, 0);
        break;

        case STATE_IGNITION_1:
            set_pin(&N2_valve, 1);
            set_pin(&vent_valve, 0);
            set_pin(&abort_valve, 0);
            servoZero();
            set_pin(&led, 1);
            // TODO: Buzzer
            // TODO: Are the times correct? Maybe confused about T-10, T-7, etc...
            // It is now T-10, 3000ms until T-7.
            k_timer_start(&TMinus7_timer, K_MSEC(3000), K_NO_WAIT);
        break;

        case STATE_IGNITION_2: 
            set_pin(&N2_valve, 1);
            set_pin(&vent_valve, 0);
            set_pin(&abort_valve, 0);
            servoZero();
            set_pin(&led, 1);
            // It is now T-7, 5000ms until T-2.
            k_timer_start(&TMinus2_timer, K_MSEC(5000), K_NO_WAIT);
        break;

        case STATE_IGNITION_3:
            set_pin(&N2_valve, 1);
            set_pin(&vent_valve, 0);
            set_pin(&abort_valve, 0);
            servoRotate(0.1 * 90.0); // TODO: What angle is 10%?
            set_pin(&led, 1);
            // It is now T-2, 2000ms until T-0.
            k_timer_start(&TMinus0_timer, K_MSEC(2000), K_NO_WAIT);
            
        break;

        case STATE_IGNITION_4:
            set_pin(&N2_valve, 1);
            set_pin(&vent_valve, 0);
            set_pin(&abort_valve, 0);
            servoRotate(1.0 * 90.0); // TODO: What angle is 100%?
            set_pin(&led, 1);
            // It is now T-0, T+30 in 30000 ms
            k_timer_start(&TPlus30_timer, K_MSEC(30000), K_NO_WAIT);
        break;

        case STATE_SAFE:
            set_pin(&N2_valve, 1);
            set_pin(&vent_valve, 1);
            set_pin(&abort_valve, 1);
            servoRotate(1.0 * 90.0);
            set_pin(&led, 1);
        break;

        case STATE_ABORT_BEFORE_COUNTDOWN:
            set_pin(&N2_valve, 1);
            set_pin(&vent_valve, 1);
            set_pin(&abort_valve, 1);
            servoZero();
            set_pin(&led, 1);
        break;

        case STATE_ABORT_AFTER_COUNTDOWN:
            set_pin(&N2_valve, 1);
            set_pin(&vent_valve, 1);
            set_pin(&abort_valve, 1);
            servoRotate(1.0 * 90.0);
            set_pin(&led, 1);
        break;

    }

    // TODO: What should we do if the sensing pins report no continuity?
    // TODO: Send back response CAN messages? What should they say?
}


int main(void)
{
	int ret;


    #if !defined(CONFIG_BOARD_NATIVE_SIM) 
	if (!pwm_is_ready_dt(&pwm_servo)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_servo.dev->name);
		return 0;
	}
    #endif

    init_can();
    add_filter_can(can_rx_cb, filter, NULL);
    add_filter_can(can_rx_override_cb, override_filter, NULL);



    // data[0] = 39;
    // data[1] = 59;
    // submit_can_pkt(data, 2);

    configure_output_pin(&led);


    for (size_t i = 0; i < NUM_CHANNELS; i++) {
        configure_output_pin(&pyroPins[i]);
        // configure_input_pin(&solenoidSense[i]);
    }

    
	while (1) {
		// ret = gpio_pin_toggle_dt(&led);
		// uint8_t senseState = pyroSense(0);

        if(trigger) {
            evaluateState();
            trigger = false;
        }

        uint8_t data[3] = {2, 3, 4};
        submit_can_pkt(data, 3);
		k_msleep(1000);
	}
	return 0;
}
