/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "can_com.h"
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

#define NUM_CHANNELS 5

static const struct gpio_dt_spec N2_valve = GPIO_DT_SPEC_GET(DT_NODEALIAS(N2_valve), gpios);
static const struct gpio_dt_spec vent_valve = GPIO_DT_SPEC_GET(DT_NODEALIAS(vent_valve), gpios);
static const struct gpio_dt_spec main_valve = GPIO_DT_SPEC_GET(DT_NODEALIAS(main_valve), gpios);
static const struct gpio_dt_spec abort_valve = GPIO_DT_SPEC_GET(DT_NODEALIAS(abort_valve), gpios);
static const struct gpio_dt_spec ignition = GPIO_DT_SPEC_GET(DT_NODEALIAS(ignition), gpios);

static const struct gpio_dt_spec N2_sense = GPIO_DT_SPEC_GET(DT_NODEALIAS(N2_sense), gpios);
static const struct gpio_dt_spec vent_sense = GPIO_DT_SPEC_GET(DT_NODEALIAS(vent_sense), gpios);
static const struct gpio_dt_spec main_sense = GPIO_DT_SPEC_GET(DT_NODEALIAS(main_sense), gpios);
static const struct gpio_dt_spec abort_sense = GPIO_DT_SPEC_GET(DT_NODEALIAS(abort_sense), gpios);
static const struct gpio_dt_spec ignition_sense = GPIO_DT_SPEC_GET(DT_NODEALIAS(ignition_sense), gpios);


const struct gpio_dt_spec solenoidSense[NUM_CHANNELS] = {N2_valve, vent_valve, main_valve, abort_valve, ignition};
const struct gpio_dt_spec solenoidPin[NUM_CHANNELS] = {N2_sense, vent_valve, main_sense, abort_sense, ignition_sense};

typedef enum {
    STATE_IDLE,
    STATE_INIT,
    STATE_FILL,
    STATE_STOP_FILL,
    STATE_UMBILICAL,
    STATE_N2_PRESSURIZATION,
    STATE_IGNITION,
    STATE_SAFE,
    STATE_ABORT,
} State;

State fafnir_state = STATE_IDLE;
bool trigger = false;



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

void resetState(); { 
    //reset all pins
}

void evaluateState() { 
    switch(fafnir_state) {
        case STATE_INIT:
        gpio_pin_set_dt(&vent_valve, 1);
        break;

        case STATE_FILL:
        //nothing :)
        break;

        case STATE_STOP_FILL:
        gpio_pin_set_dt(&vent_valve, 0);
        break;

        case STATE_UMBILICAL:
        //nothing :)
        break;

        case N2_PRESSURIZATION:
        gpio_pin_set_dt(&N2_valve, 1);
        break;

        case IGNITION:
        //T-10 seconds
            //wait
        //T-3 seconds
            //servoRotate(10);
            //Open main valve 
            //wait...
        //T-0 Seconds
        //servoRotate(90);
        break;
    }
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

    #if defined(CONFIG_BOARD_NATIVE_SIM) 
    // gpio loopback mode
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE | GPIO_INPUT);
	if (ret < 0) {
		return 0;
	}
    #else
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
    #endif

    #if !defined(CONFIG_BOARD_NATIVE_SIM) 
	if (!pwm_is_ready_dt(&pwm_servo)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_servo.dev->name);
		return 0;
	}
    #endif


	// ret = gpio_pin_configure_dt(&pyro0_sense, GPIO_INPUT);
	// if (ret != 0) {
	// 	printk("Error %d: failed to configure %s pin %d\n",
	// 	       ret, pyro0_sense.port->name, pyro0_sense.pin);
	// 	return 0;
	// }

    // LOG_INF("zeroing servo\n");
	// servoZero();
	// k_msleep(SLEEP_TIME_MS);
	// servoRotate(90);


    volatile uint8_t can_scratchpad[100];
    can_scratchpad[0] = 0;
    init_can((void *) can_scratchpad);

    LOG_INF("inititialised CAN\n");

	// k_msleep(SLEEP_TIME_MS);

    LOG_INF("submitting CAN packet\n");
    data[0] = 39;
    data[1] = 59;
    submit_can_pkt(data, 2);

    LOG_INF("running the while loop\n");

    resetState();

	while (1) {

        
	}
	return 0;
}