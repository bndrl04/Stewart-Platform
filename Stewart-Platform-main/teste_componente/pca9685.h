/**
 * pca9685.h
 * Driver PCA9685 (16 canale PWM, 12 biti) pentru Raspberry Pi Pico.
 * Pentru proiectul: platforma Stewart cu 6x MG996R.
 */

#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// Adresa I2C implicita a modulului PCA9685 (toti jumperii A0..A5 in 0)
#define PCA9685_DEFAULT_ADDR 0x40

// Structura de configurare a unei instante PCA9685
typedef struct {
    i2c_inst_t *i2c;     // i2c0 sau i2c1
    uint8_t address;     // adresa I2C (default 0x40)
} pca9685_t;


/**
 * Initializeaza PCA9685.
 * Returneaza true daca dispozitivul raspunde, false altfel.
 *  - i2c:     pointer la i2c0 sau i2c1
 *  - addr:    adresa I2C (de obicei 0x40)
 *  - freq_hz: frecventa PWM globala (50 Hz pentru servo-uri)
 */
bool pca9685_init(pca9685_t *dev, i2c_inst_t *i2c, uint8_t addr, uint16_t freq_hz);

/**
 * Seteaza frecventa PWM globala (3..1526 Hz).
 * Pentru servo-uri folosim 50 Hz.
 */
void pca9685_set_freq(pca9685_t *dev, uint16_t freq_hz);

/**
 * Seteaza pulsul brut pe un canal (0..15).
 *  - on:  0..4095, momentul cand semnalul URCA in ciclu
 *  - off: 0..4095, momentul cand semnalul COBOARA
 * Pentru servo: on=0, off=ticks corespunzatori latimii dorite.
 */
void pca9685_set_pwm(pca9685_t *dev, uint8_t channel, uint16_t on, uint16_t off);

/**
 * Trimite un puls de durata pulse_us microsecunde pe canalul respectiv.
 * Functioneaza corect doar daca frecventa e setata la 50 Hz.
 * MG996R: 500us = 0°, 1500us = 90°, 2500us = 180°
 */
void pca9685_set_servo_us(pca9685_t *dev, uint8_t channel, uint16_t pulse_us);

/**
 * Seteaza unghiul unui servo (0..180°).
 *  - pulse_min_us: pulsul pentru 0°   (typical 500 us, ajustabil per servo)
 *  - pulse_max_us: pulsul pentru 180° (typical 2500 us, ajustabil per servo)
 */
void pca9685_set_servo_angle(pca9685_t *dev, uint8_t channel, float angle_deg,
                              uint16_t pulse_min_us, uint16_t pulse_max_us);

/**
 * Opreste semnalul pe toate canalele (servo-urile nu mai primesc comenzi).
 */
void pca9685_all_off(pca9685_t *dev);

#endif // PCA9685_H
