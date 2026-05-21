/**
 * pca9685.c
 * Implementare driver PCA9685 pentru Raspberry Pi Pico.
 */

#include "pca9685.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>

// === Registri PCA9685 ===
#define REG_MODE1        0x00
#define REG_MODE2        0x01
#define REG_PRESCALE     0xFE
#define REG_LED0_ON_L    0x06   // primul registru de canal; urmatoarele vin +4

// === Biti MODE1 ===
#define MODE1_SLEEP      0x10
#define MODE1_RESTART    0x80
#define MODE1_AI         0x20   // Auto-Increment
#define MODE1_ALLCALL    0x01

// Frecventa oscilatorului intern
#define OSC_CLOCK        25000000UL


// === Functii utilitare interne ===

static int write_reg(pca9685_t *dev, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_write_blocking(dev->i2c, dev->address, buf, 2, false);
}

static int read_reg(pca9685_t *dev, uint8_t reg, uint8_t *value) {
    int r = i2c_write_blocking(dev->i2c, dev->address, &reg, 1, true);
    if (r < 0) return r;
    return i2c_read_blocking(dev->i2c, dev->address, value, 1, false);
}


// === API public ===

bool pca9685_init(pca9685_t *dev, i2c_inst_t *i2c, uint8_t addr, uint16_t freq_hz) {
    dev->i2c = i2c;
    dev->address = addr;

    // Verificam prezenta dispozitivului prin scriere goala (probe)
    uint8_t dummy;
    int r = i2c_read_blocking(i2c, addr, &dummy, 1, false);
    if (r < 0) {
        printf("PCA9685: nu raspunde la adresa 0x%02X\n", addr);
        return false;
    }

    // MODE1: trezim dispozitivul si activam auto-increment
    write_reg(dev, REG_MODE1, MODE1_AI);
    sleep_ms(5);

    pca9685_set_freq(dev, freq_hz);
    return true;
}


void pca9685_set_freq(pca9685_t *dev, uint16_t freq_hz) {
    // Calculam prescaler: prescale = round(osc/(4096 * freq)) - 1
    float prescaleval = ((float)OSC_CLOCK / (4096.0f * (float)freq_hz)) - 1.0f;
    int prescale = (int)(prescaleval + 0.5f);
    if (prescale < 3)   prescale = 3;
    if (prescale > 255) prescale = 255;

    // Pentru a schimba PRESCALE trebuie SLEEP activ
    uint8_t old_mode;
    read_reg(dev, REG_MODE1, &old_mode);
    uint8_t sleep_mode = (old_mode & 0x7F) | MODE1_SLEEP;
    write_reg(dev, REG_MODE1, sleep_mode);
    write_reg(dev, REG_PRESCALE, (uint8_t)prescale);
    write_reg(dev, REG_MODE1, old_mode);
    sleep_ms(5);
    // Restart cu auto-increment activ
    write_reg(dev, REG_MODE1, old_mode | MODE1_RESTART | MODE1_AI);
}


void pca9685_set_pwm(pca9685_t *dev, uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) return;
    if (on  > 4095)   on  = 4095;
    if (off > 4095)   off = 4095;

    uint8_t base = REG_LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        base,
        (uint8_t)(on  & 0xFF),
        (uint8_t)((on  >> 8) & 0x0F),
        (uint8_t)(off & 0xFF),
        (uint8_t)((off >> 8) & 0x0F),
    };
    i2c_write_blocking(dev->i2c, dev->address, buf, 5, false);
}


void pca9685_set_servo_us(pca9685_t *dev, uint8_t channel, uint16_t pulse_us) {
    // La 50 Hz: perioada = 20 ms = 20000 us, 4096 ticks = perioada
    // ticks = pulse_us * 4096 / 20000
    uint32_t ticks = ((uint32_t)pulse_us * 4096UL) / 20000UL;
    if (ticks > 4095) ticks = 4095;
    pca9685_set_pwm(dev, channel, 0, (uint16_t)ticks);
}


void pca9685_set_servo_angle(pca9685_t *dev, uint8_t channel, float angle_deg,
                              uint16_t pulse_min_us, uint16_t pulse_max_us) {
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    float pulse = (float)pulse_min_us +
                  ((float)pulse_max_us - (float)pulse_min_us) * angle_deg / 180.0f;
    pca9685_set_servo_us(dev, channel, (uint16_t)(pulse + 0.5f));
}


void pca9685_all_off(pca9685_t *dev) {
    for (uint8_t ch = 0; ch < 16; ch++) {
        pca9685_set_pwm(dev, ch, 0, 0);
    }
}
