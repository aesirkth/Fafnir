/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include "main.h"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   4000

/* The devicetree node identifier for the "led0" alias. */
// #define LED0_NODE DT_ALIAS(led0)
#define LED0_NODE DT_NODELABEL(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct pwm_dt_spec pwm_servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
#define SERVO_PERIOD PWM_MSEC(20)

static const struct gpio_dt_spec pyro0_sense = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro0_sense), gpios);
static const struct gpio_dt_spec pyro1_sense = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro1_sense), gpios);
static const struct gpio_dt_spec pyro2_sense = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro2_sense), gpios);
const struct gpio_dt_spec pyroSensePins[NUM_PYROS] = {pyro0_sense, pyro1_sense, pyro2_sense};


typedef enum {
    STATE_IDLE,
    STATE_INIT,
    STATE_READY,
    STATE_ACTUATE
} State;

typedef enum {
    CMD_NONE,
    CMD_SERVO_ZERO,
    CMD_SERVO_ROTATE,
    CMD_PYRO_ACTUATE,
    CMD_PYRO_DISABLE
} CommandType;

typedef struct {
    CommandType type;
    uint8_t pyroIndex;
    uint16_t angle;
} Command;

// State servoState = STATE_IDLE;
// State pyroState[NUM_PYROS] = {STATE_IDLE, STATE_IDLE, STATE_IDLE};
// uint8_t pyroMask = 0b000;
// uint16_t targetAngle = 0;

// GPIO_TypeDef* const pyroPort = GPIOA;
// const uint16_t pyroPins[NUM_PYROS] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10};

// GPIO_TypeDef* const pyroDetectPort = GPIOB;
// const uint16_t pyroDetectPins[NUM_PYROS] = {GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13};

// void processCommand(Command cmd) {

// 	if (systemIdle()) return; //not so sure abt this


//     switch (cmd.type) {
//         case CMD_SERVO_ZERO:
//             servoState = STATE_INIT;
//             break;

//         case CMD_SERVO_ROTATE:
//             targetAngle = cmd.angle;
//             if (servoState == STATE_READY) servoState = STATE_ACTUATE;
//             break;

//         case CMD_PYRO_ACTUATE:
//             if (cmd.pyroIndex < NUM_PYROS && pyroState[cmd.pyroIndex] == STATE_READY)
//                 pyroState[cmd.pyroIndex] = STATE_ACTUATE;
//             break;

//         case CMD_PYRO_DISABLE:
//             if (cmd.pyroIndex < NUM_PYROS)
//                 pyroState[cmd.pyroIndex] = STATE_INIT;
//             break;

//         default:
//             break;
//     }
// }

// uint8_t systemReady(void) {
//     if (servoState != STATE_READY)
//         return 0;

//     for (int i = 0; i < NUM_PYROS; i++) {
//         if (pyroState[i] != STATE_READY)
//             return 0;
//     }

//     return 1;
// } //essentially just check that everything is in the READY state for the first time, send this via CAN to Fjalar

// uint8_t systemIdle(void) {
// 	  if (servoState == STATE_IDLE) return 1;

// 	    for (int i = 0; i < NUM_PYROS; i++) {
// 	        if (pyroState[i] == STATE_IDLE)
// 	            return 1;
// 	    }

// 	    return 0;
// }

// void handleServo(void) {
//     switch (servoState) {
//         case STATE_IDLE:
//             //Do nothing until we receive some initial command
//             break;

//         case STATE_INIT:
//             servoZero();
//             servoState = STATE_READY;
//             break;

//         case STATE_READY:
//             //Wait for rotation command
//             break;

//         case STATE_ACTUATE:
//             servoRotate(targetAngle);
//             printf("Servo Rotated\n");
//             servoState = STATE_READY;
//             break;
//     }
// }

// void handlePyro(int i) {
//     switch (pyroState[i]) {
//         case STATE_IDLE:
//             //Do nothing until we receive some initial command
//             break;

//         case STATE_INIT:
//             pyroActuate(i, 0);
//             if (pyroDetect(i) == 0) { //Low = continuity detected
//                 pyroState[i] = STATE_READY;
//                 printf("[Pyro %d] Continuity OK\n", i);
//             }
//             break;

//         case STATE_READY:
//             // WAit for actuation command
//             break;

//         case STATE_ACTUATE:
//             pyroActuate(i, 1);
//             //remain active until the CMD_PYRO_DISABLE is received
//             break;
//     }
// }

// void forceInit(void) {
// 	//USED ONLY FOR DEBUGGING
// 	servoState = STATE_INIT;
// 	for (int i = 0; i < NUM_PYROS; i++)
// 	    pyroState[i] = STATE_INIT;
// }

// void pyroActuate(uint8_t index, uint8_t state) {
// 	if (index >= NUM_PYROS) return;
// 	HAL_GPIO_WritePin(pyroPort, pyroPins[index], state ? GPIO_PIN_SET : GPIO_PIN_RESET);

//     if (state) {
//     	pyroMask |=  (1 << index); //set union
//     } else {
//     	pyroMask &= ~(1 << index); //set intersection
//     }

// }

uint8_t pyroSense(uint8_t index) {
	if (index >= NUM_PYROS) return 69; //69 means error
    //HIGH = continuity detected (return 1)
    //LOW = open circuit (return 0)

	int state = gpio_pin_get_dt(&pyroSensePins[index]);

    return state;
}

void servoZero(void) {
	servoRotate(0.0f);
}

void servoRotate(float angle) {
	//The angle is mapped to -135 to 135 to properly represent CW and CCW rotations
	//Input of +90 == 90 deg rotation CW from the zero position.

	if (angle < -135 || angle > 135) return;
	angle = 135.0f + angle; //135 degrees is the zero/middle position, since the servo motor can rotate 270 deg

	float degRatio = angle / 270.0f;

	float duty = degRatio * 0.10f + 0.025f;  //mapping to 2.5%–12.5% duty cycle
	// setPWM(htim, MOTOR_TIMER_CHANNEL, duty);
	pwm_set_dt(&pwm_servo, SERVO_PERIOD, (uint32_t) SERVO_PERIOD*duty); // move it a bit

}










int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!pwm_is_ready_dt(&pwm_servo)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_servo.dev->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&pyro0_sense, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, pyro0_sense.port->name, pyro0_sense.pin);
		return 0;
	}

	// ret = pwm_set_dt(&pwm_motor, 20000000, 500000); // 0 motor
	// ret = pwm_set_dt(&pwm_servo, 20000000, 2500000); // move it a bit
	// if (ret) {
	// 	return 0;
	// }
	servoZero();
	k_msleep(SLEEP_TIME_MS);
	servoRotate(90);

	while (1) {
		// ret = gpio_pin_toggle_dt(&led);
		ret = gpio_pin_set_dt(&led, pyroSense(0));
		if (ret < 0) {
			return 0;
		}

		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(10);
	}
	return 0;
}