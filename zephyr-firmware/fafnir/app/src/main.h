#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_PYROS 3

// extern GPIO_TypeDef* const pyroPort;
// extern const uint16_t pyroPins[NUM_PYROS];

// extern GPIO_TypeDef* const pyroDetectPort;
// extern const uint16_t pyroDetectPins[NUM_PYROS];

/* Motor functions */
// void setPWM(TIM_HandleTypeDef *timer_handle, uint32_t timer_channel, float duty); // pwm_set_dt
void servoZero(void);
void servoRotate(float angle);

/* Pyro channel functions */
void pyroActuate(uint8_t index, uint8_t state);
uint8_t pyroDetect(uint8_t index);


/* State machine functions  */
uint8_t systemIdle(void);
uint8_t systemReady(void);
void handleServo(void);
void handlePyro(int i);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
