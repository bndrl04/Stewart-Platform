#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

#define PCA9685_DEFAULT_ADDR 0x40

#ifdef __cplusplus
extern "C" {
#endif

bool pca9685_init(i2c_inst_t *i2c, uint8_t addr);
bool pca9685_set_pwm_freq(float freq_hz);

// Set raw 12-bit PWM (0..4095 on, 0..4095 off) on a channel (0..15).
bool pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);

// Set servo pulse width in microseconds (typical 500..2500).
bool pca9685_set_servo_us(uint8_t channel, uint16_t pulse_us);

// Set servo angle in degrees [-90, +90], mapped linearly to 500..2500 us.
bool pca9685_set_servo_angle(uint8_t channel, float angle_deg);

// Disable PWM output on a channel (puts it in low state).
bool pca9685_off(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif
