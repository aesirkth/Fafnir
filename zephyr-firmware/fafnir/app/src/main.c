#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include "main.h"


LOG_MODULE_REGISTER(fafnir, LOG_LEVEL_INF);


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   5000
#define NUM_PYROS 3
#define IGNITION_CHANNEL 0

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);

static const struct pwm_dt_spec pwm_servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
#define SERVO_PERIOD PWM_MSEC(20)

static const struct gpio_dt_spec pyro0_sense = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro0_sense), gpios);
static const struct gpio_dt_spec pyro1_sense = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro1_sense), gpios);
static const struct gpio_dt_spec pyro2_sense = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro2_sense), gpios);

static const struct gpio_dt_spec pyro0 = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro0), gpios);
static const struct gpio_dt_spec pyro1 = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro1), gpios);
static const struct gpio_dt_spec pyro2 = GPIO_DT_SPEC_GET(DT_NODELABEL(pyro2), gpios);


//uint16_t targetAngle = 0;

const struct gpio_dt_spec pyroSensePins[NUM_PYROS] = {pyro0_sense, pyro1_sense, pyro2_sense};
const struct gpio_dt_spec pyroPins[NUM_PYROS] = {pyro0, pyro1, pyro2};


typedef enum {
    SYS_IDLE,
    SYS_INIT,
    SYS_READY,
    SYS_ACTUATE,
    SYS_DEACTUATE
} fafnir_state;


typedef enum {
    CMD_NONE,
    CMD_SERVO_ZERO,
    CMD_SERVO_ROTATE,
    CMD_PYRO_ACTUATE,
    CMD_PYRO_DISABLE
} fafnir_commands;

typedef enum {
    SERVO_IDLE,
    SERVO_ZERO,
    SERVO_ROTATE
} servo_state;

typedef enum {
    PYRO_OFF,
    PYRO_SENSE,
    PYRO_READY,
    PYRO_ON
} pyro_state;

fafnir_state systemState = SYS_IDLE;
servo_state servoState = SERVO_IDLE;
pyro_state pyroState[NUM_PYROS] = { PYRO_OFF, PYRO_OFF, PYRO_OFF };


int main(void)
{
    if (!initializePins()) {
        LOG_ERR("Pin initialization failed");
        return 0;
    }

  
    servoZero();

    LOG_INF("System booted, in IDLE");

    while (1) {
        switch (systemState) {
            case SYS_IDLE:
                // Wait for a CAN command to start INIT
                break;


            case SYS_INIT: {
                static bool init_started = false;

                if (!init_started) {
                    for (int i = 0; i < NUM_PYROS; i++)
                        pyroState[i] = PYRO_SENSE;
                    servoState = SERVO_ZERO;
                    init_started = true;
                }

                if (systemReady()) {
                    k_msleep(250);
                    systemState = SYS_READY;
                    LOG_INF("SYSTEM READY");
                    init_started = false;
                }
                break;
            }

            case SYS_READY:
                // Waiting for ACTUATE command
                break;

            case SYS_ACTUATE:
                LOG_INF("SYSTEM ACTUATING");
                pyroActuate(IGNITION_CHANNEL, 1);
                k_msleep(150);
                servoRotate(90);
                k_msleep(5000);
                systemState = SYS_DEACTUATE;
                break;

            case SYS_DEACTUATE:
                LOG_INF("SYSTEM DEACTUATING");
                pyroActuate(IGNITION_CHANNEL, 0);
                servoRotate(0);
                for (int i = 0; i < NUM_PYROS; i++)
                    pyroActuate(i, 0);
                systemState = SYS_READY;
                break;

            default:
                systemState = SYS_IDLE;
                break;
        }

            handleServo();
            for (int i = 0; i < NUM_PYROS; i++)
                handlePyro(i);

            static bool led_on = false;
            gpio_pin_set_dt(&led, led_on);
            led_on = !led_on;

            k_msleep(50);
        }

}

int initializePins() {

    int ret;

    if (!gpio_is_ready_dt(&led))
        return 0;
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
        return 0;

    if (!pwm_is_ready_dt(&pwm_servo)) {
        LOG_ERR("PWM NOT READY");
        return 0;
    }

    ret = gpio_pin_configure_dt(&pyro0_sense, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("PYRO 0 SENSE NOT READY");
        return 0;
    }

    for (int i = 0; i < NUM_PYROS; i++) {
        ret = gpio_pin_configure_dt(&pyroPins[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("FAILED TO CONFIGURE PYRO PINS");
            return 0;
        }
    }

    return 1;

}

bool systemReady(void) {
    //Servo must have completed its zeroing
    if (servoState != SERVO_IDLE && servoState != SERVO_ZERO) {
        return false;
    }

    // All pyro channels must be ready (continuity detected)
    for (int i = 0; i < NUM_PYROS; i++) {
        if (pyroState[i] != PYRO_READY) {
            return false;
        }
    }

    return true;
}


void handleServo(void) {
    switch (servoState) {
    case SERVO_ZERO:
        servoZero();
        servoState = SERVO_IDLE;
        break;

    case SERVO_ROTATE:
        servoRotate(90);
        servoState = SERVO_IDLE;
        break;

    default:
        break;
    }
}

void handlePyro(int i) {
    switch (pyroState[i]) {
    case PYRO_OFF:
        pyroActuate(i, 0);
        break;

    case PYRO_SENSE:
        if (pyroSense(i) == 0) {
            pyroState[i] = PYRO_READY;
            LOGG_INF("PYRO CONTINUITY OK")
        }
        break;

    case PYRO_READY:
        break;

    case PYRO_ON:
        pyroActuate(i, 1);
        k_msleep(100);
        pyroActuate(i, 0);
        pyroState[i] = PYRO_READY;
        break;

    default:
        break;
    }
}


void pyroActuate(uint8_t index, uint8_t state) {
    if (index >= NUM_PYROS) return;
    if (state != 1 && state != 0) return;
    gpio_pin_set_dt(&pyroPins[index], state);
}

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
    pwm_set_dt(&pwm_servo, SERVO_PERIOD, (uint32_t)SERVO_PERIOD * duty);

}
