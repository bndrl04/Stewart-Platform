/**
 * test_2_servo_single.c
 * =====================
 * Misca un singur servo pe canalul 0 al PCA9685.
 *
 * CABLAJ NECESAR (in plus fata de test 1):
 *   - Sursa WELL CONECTATA si PORNITA
 *   - WELL +V -> bloc verde V+ pe PCA9685
 *   - WELL -V -> bloc verde GND pe PCA9685
 *   - PUNTE GND intre PCA9685 si Pico (critic!)
 *   - Servo MG996R pe CANALUL 0:
 *       maro/negru   -> jos (GND)
 *       rosu         -> mijloc (V+)
 *       portocaliu   -> sus (PWM/signal)
 *
 * CE FACE TESTUL:
 *   Misca servo-ul ciclic intre 0°, 45°, 90°, 135°, 180°
 *   cu pauza intre pozitii pentru a putea observa.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pca9685.h"

#define I2C_PORT    i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_FREQ    400000

#define CANAL_SERVO 0   // canalul PCA9685 unde e conectat servo-ul

// CALIBRARE MG996R - ajusteaza daca servo-ul nu ajunge la 0° sau 180°
#define PULSE_MIN_US 500    // pulsul pentru 0°
#define PULSE_MAX_US 2500   // pulsul pentru 180°

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n==================================================\n");
    printf("TEST 2: Servo pe canalul %d\n", CANAL_SERVO);
    printf("==================================================\n");

    // I2C init
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // PCA9685 init (50 Hz pentru servo-uri)
    pca9685_t pca;
    if (!pca9685_init(&pca, I2C_PORT, PCA9685_DEFAULT_ADDR, 50)) {
        printf("EROARE: PCA9685 nu raspunde!\n");
        printf("Ruleaza intai test_1_i2c_scan pentru diagnostic.\n");
        while (true) sleep_ms(1000);
    }

    printf("PCA9685 OK. Frecventa PWM: 50 Hz\n");
    printf("Calibrare: 0° = %d us, 180° = %d us\n", PULSE_MIN_US, PULSE_MAX_US);
    printf("\nIncepem miscarea ciclica...\n\n");

    // Secventa de unghiuri
    const float unghiuri[] = {0, 45, 90, 135, 180, 135, 90, 45};
    const int n = sizeof(unghiuri) / sizeof(unghiuri[0]);

    while (true) {
        for (int i = 0; i < n; i++) {
            printf("  -> %.0f°\n", unghiuri[i]);
            pca9685_set_servo_angle(&pca, CANAL_SERVO, unghiuri[i],
                                     PULSE_MIN_US, PULSE_MAX_US);
            sleep_ms(800);
        }
    }

    return 0;
}
