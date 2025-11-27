/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_com.h"
#include "gpio_emul_shell.h"
#include "main.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   4000


LOG_MODULE_REGISTER(main_func);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */


static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);

// Disable servo pwm if using board Native Sim 
#if !defined(CONFIG_BOARD_NATIVE_SIM) 
static const struct pwm_dt_spec pwm_servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
#define SERVO_PERIOD PWM_MSEC(20)
#endif


#if !defined(CONFIG_BOARD_NATIVE_SIM) 
static const struct gpio_dt_spec main_valve = GPIO_DT_SPEC_GET(DT_ALIAS(mainvalve), gpios);
#endif

static const struct gpio_dt_spec N2_valve = GPIO_DT_SPEC_GET(DT_ALIAS(n2valve), gpios);
static const struct gpio_dt_spec vent_valve = GPIO_DT_SPEC_GET(DT_ALIAS(ventvalve), gpios);
static const struct gpio_dt_spec abort_valve = GPIO_DT_SPEC_GET(DT_ALIAS(abortvalve), gpios);
static const struct gpio_dt_spec ignition = GPIO_DT_SPEC_GET(DT_ALIAS(ignition), gpios);

static const struct gpio_dt_spec N2_sense = GPIO_DT_SPEC_GET(DT_ALIAS(n2sense), gpios);
static const struct gpio_dt_spec vent_sense = GPIO_DT_SPEC_GET(DT_ALIAS(ventsense), gpios);
static const struct gpio_dt_spec main_sense = GPIO_DT_SPEC_GET(DT_ALIAS(mainsense), gpios);
static const struct gpio_dt_spec abort_sense = GPIO_DT_SPEC_GET(DT_ALIAS(abortsense), gpios);
static const struct gpio_dt_spec ignition_sense = GPIO_DT_SPEC_GET(DT_ALIAS(ignitionsense), gpios);

#define NUM_CHANNELS 4
const struct gpio_dt_spec solenoidSense[NUM_CHANNELS] = {N2_sense, vent_sense, abort_sense, ignition_sense};
const struct gpio_dt_spec pyroPins[NUM_CHANNELS] = {N2_valve, vent_valve, abort_valve, ignition};
// const struct gpio_dt_spec pyroSensePins[NUM_PYROS] = {pyro0_sense, pyro1_sense, pyro2_sense};
// const struct gpio_dt_spec pyroPins[NUM_PYROS] = {pyro0, pyro1, pyro2};



size_t CAN_IDS[] = {
    
};

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
    STATE_SAFE,
    STATE_ABORT
} State;

State systemState = STATE_IDLE;
bool trigger = true;

void timerCallback_1() { 
    if(systemState == STATE_ABORT) return;

    systemState = STATE_IGNITION_2;
    LOG_INF("timer callback 1 triggered going into STATE_IGNITION_2!");
    trigger = true;
}

void timerCallback_2() { 
    if(systemState == STATE_ABORT) return;

    systemState = STATE_IGNITION_3;
    LOG_INF("timer callback 2 triggered going into STATE_IGNITION_3!");
    trigger = true;
}

uint8_t pyroSense(uint8_t index) {
	if (index >= NUM_PYROS) return 69; //69 means error
    //HIGH = continuity detected (return 1)
    //LOW = open circuit (return 0)

	int state = gpio_pin_get_dt(&solenoidSense[index]);

    return state;
}

void servoZero(void) {
	servoRotate(0.0f);
}

void servoRotate(float angle) {
	//The angle is mapped to -135 to 135 to properly represent CW and CCW rotations
	//Input of +90 == 90 deg rotation CW from the zero position.
    LOG_INF("setting angle of pwm_servo = %f\n", (double) angle);

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

void can_rx_cb(const struct device *const device, struct can_frame *frame, void *user_data) {

    // if(frame->id == STATE_IGNITION 
    // ) return;

    //     systemState = frame->id;

    LOG_INF("Recieved CAN message rx: %#X: frame->dlc: %d. Switching state to %d", frame->id, frame->dlc, systemState);


    trigger = true;

    // TODO: should probably check size.
    // if (frame->dlc != pkt_size[pkt_type]) {
    //     LOG_ERR("received packet %#x has length %d but should be length %d", pkt_type, frame->dlc, pkt_size[pkt_type]);
    // }

    memcpy(user_data, frame->data, frame->dlc);
}

static uint8_t data[2];


void configure_output_pin(const struct gpio_dt_spec *pin) {
    // gpio_pin_configure_dt(pin, )

	int ret;
    int ready = gpio_is_ready_dt(pin);
    if (!ready) {
        LOG_ERR("pin not ready");
        return;
    }

    #if defined(CONFIG_BOARD_NATIVE_SIM) 
    ret = gpio_pin_configure_dt(pin, GPIO_OUTPUT);
    #else
    ret = gpio_pin_configure_dt(pin, GPIO_OUTPUT);
    #endif
    if (ret < 0) {
        LOG_ERR("pin configure failed %d", ret);
        return;
    }
}

void configure_input_pin(const struct gpio_dt_spec *pin) {
    // gpio_pin_configure_dt(pin, )

	int ret;
    int ready = gpio_is_ready_dt(pin);
    if (!ready) {
        LOG_ERR("pin not ready");
        return;
    }

    #if defined(CONFIG_BOARD_NATIVE_SIM) 
    ret = gpio_pin_configure_dt(pin, GPIO_INPUT);
    #else
    ret = gpio_pin_configure_dt(pin, GPIO_INPUT);
    #endif
    if (ret < 0) {
        LOG_ERR("pin configure failed %d", ret);
        return;
    }
}

void set_pin(const struct gpio_dt_spec *pin, bool value) {
    LOG_INF("PIN[%d] set to value %d", pin->pin, value);
    gpio_pin_set_dt(pin, value);
}

void resetState() {
    // reset state
}

K_TIMER_DEFINE(timer_1, timerCallback_1, NULL);
K_TIMER_DEFINE(timer_2, timerCallback_2, NULL);

void evaluateState() { 
    LOG_INF("Evaluating state: %d", systemState);
    switch(systemState) {
        case STATE_INIT:
            set_pin(&vent_valve, 1);
        break;

        case STATE_FILL:
        //nothing :)
        break;

        case STATE_STOP_FILL:
            set_pin(&vent_valve, 0);
        break;

        case STATE_UMBILICAL:
        //nothing :)
        break;

        case STATE_N2_PRESSURIZATION:
            set_pin(&N2_valve, 1);
        break;

        case STATE_IGNITION_1:
        //T-10 seconds
            //wait
        //buzzer and LED
            k_timer_start(&timer_1, K_MSEC(7000), K_NO_WAIT);

        break;

        case STATE_IGNITION_2: 
            //T-3 seconds
            //servoRotate(10);
            //delay(50ms)
            k_timer_start(&timer_2, K_MSEC(3000), K_NO_WAIT);
            //wait
        break;

        case STATE_IGNITION_3:
            
        break;
 
        //T-0 Seconds
        //servoRotate(90);
        break;
    }
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

    volatile uint8_t can_scratchpad[100];
    can_scratchpad[0] = 0;
    init_can(can_rx_cb, (void *) can_scratchpad);


	// k_msleep(SLEEP_TIME_MS);

    // data[0] = 39;
    // data[1] = 59;
    // submit_can_pkt(data, 2);

    configure_output_pin(&led);


    for (size_t i = 0; i < NUM_CHANNELS; i++) {
        configure_output_pin(&pyroPins[i]);
        configure_input_pin(&solenoidSense[i]);
    }

    
    resetState();
    set_pin(&N2_valve, 1);

	while (1) {
		// ret = gpio_pin_toggle_dt(&led);
		// uint8_t senseState = pyroSense(0);
        // // if(can_scratchpad[0])
        // // LOG_INF("can_scratchpad[0]= %d", can_scratchpad[0]);

		// if (senseState == 69) {
		// 	printk("pyro0_sense failed");
		// 	return 0;
		// }

        if(trigger) {
            evaluateState();
            trigger = false;
        }
        // set_pin(&led, can_scratchpad[0]);


		// set_pin(&led, can_scratchpad[0] ? 1 : 0);
		// LOG_INF("gpio_pin = %d and can_scratchpad[0]=%d 1 or 0 = %d", gpio_pin_get_dt(&led), can_scratchpad[0], can_scratchpad[0] ? 1 : 0);
        // LOG_INF("can_scratchpad[0]= %d", can_scratchpad[0]);


		k_msleep(50);
	}
	return 0;
}
