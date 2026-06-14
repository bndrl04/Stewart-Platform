#include "pca9685.h"
#include "pico/stdlib.h"
#include <math.h>

// PCA9685 register addresses
#define REG_MODE1      0x00
#define REG_MODE2      0x01
#define REG_LED0_ON_L  0x06
#define REG_PRESCALE   0xFE

// MODE1 bits
#define MODE1_RESTART  0x80
#define MODE1_SLEEP    0x10
#define MODE1_AI       0x20  // auto-increment
#define MODE1_ALLCALL  0x01

// MODE2 bits
#define MODE2_OUTDRV   0x04  // totem-pole output

#define OSC_CLOCK_HZ   25000000.0f
#define PWM_RESOLUTION 4096

static i2c_inst_t *s_i2c = NULL;
static uint8_t s_addr = 0;

static bool write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    int n = i2c_write_blocking(s_i2c, s_addr, buf, 2, false);
    return n == 2;
}

static bool read_reg(uint8_t reg, uint8_t *val) {
    if (i2c_write_blocking(s_i2c, s_addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(s_i2c, s_addr, val, 1, false) == 1;
}

bool pca9685_init(i2c_inst_t *i2c, uint8_t addr) {
    s_i2c = i2c;
    s_addr = addr;

    // Reset MODE1: clear SLEEP, enable auto-increment.
    if (!write_reg(REG_MODE1, MODE1_AI)) return false;
    sleep_ms(1);
    // Totem-pole drive for the LED outputs (servos like a clean push-pull).
    if (!write_reg(REG_MODE2, MODE2_OUTDRV)) return false;
    sleep_ms(1);
    return true;
}

bool pca9685_set_pwm_freq(float freq_hz) {
    if (s_i2c == NULL) return false;
    if (freq_hz < 24.0f) freq_hz = 24.0f;
    if (freq_hz > 1526.0f) freq_hz = 1526.0f;

    // prescale = round(osc / (4096 * freq)) - 1
    float prescale_f = (OSC_CLOCK_HZ / ((float)PWM_RESOLUTION * freq_hz)) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

    uint8_t old_mode;
    if (!read_reg(REG_MODE1, &old_mode)) return false;

    // Prescale only writable in sleep mode.
    uint8_t sleep_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP;
    if (!write_reg(REG_MODE1, sleep_mode)) return false;
    if (!write_reg(REG_PRESCALE, prescale)) return false;
    if (!write_reg(REG_MODE1, old_mode)) return false;
    sleep_ms(1);
    // Restart so new prescale takes effect.
    if (!write_reg(REG_MODE1, old_mode | MODE1_RESTART | MODE1_AI)) return false;
    return true;
}

bool pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    if (s_i2c == NULL || channel > 15) return false;
    uint8_t buf[5] = {
        (uint8_t)(REG_LED0_ON_L + 4 * channel),
        (uint8_t)(on & 0xFF),
        (uint8_t)((on >> 8) & 0x0F),
        (uint8_t)(off & 0xFF),
        (uint8_t)((off >> 8) & 0x0F),
    };
    return i2c_write_blocking(s_i2c, s_addr, buf, 5, false) == 5;
}

bool pca9685_set_servo_us(uint8_t channel, uint16_t pulse_us) {
    // Assuming 50Hz: period = 20000 us, 4096 ticks per period.
    // ticks = pulse_us * 4096 / 20000 = pulse_us * 0.2048
    uint16_t off = (uint16_t)(((uint32_t)pulse_us * PWM_RESOLUTION) / 20000);
    if (off >= PWM_RESOLUTION) off = PWM_RESOLUTION - 1;
    return pca9685_set_pwm(channel, 0, off);
}

bool pca9685_set_servo_angle(uint8_t channel, float angle_deg) {
    if (angle_deg < -90.0f) angle_deg = -90.0f;
    if (angle_deg > 90.0f)  angle_deg =  90.0f;
    // -90 -> 500us, 0 -> 1500us, +90 -> 2500us
    float pulse = 1500.0f + angle_deg * (1000.0f / 90.0f);
    return pca9685_set_servo_us(channel, (uint16_t)(pulse + 0.5f));
}

bool pca9685_off(uint8_t channel) {
    // bit 4 of the OFF high byte forces output low (LEDn_OFF "full off")
    if (s_i2c == NULL || channel > 15) return false;
    uint8_t buf[5] = {
        (uint8_t)(REG_LED0_ON_L + 4 * channel),
        0, 0, 0, 0x10
    };
    return i2c_write_blocking(s_i2c, s_addr, buf, 5, false) == 5;
}
