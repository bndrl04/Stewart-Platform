/**
 * test_3_servo_all.c
 * ==================
 * Testeaza toate cele 6 servo-uri ale platformei Stewart.
 * Misca fiecare canal (0..5) pe rand pentru a verifica cablajul.
 *
 * CABLAJ:
 *   Identic cu test 2, dar cu 6 servo-uri conectate pe canalele 0..5.
 *
 * COMPORTAMENT:
 *   1. Aduce toate servo-urile la 90° (pozitie neutra)
 *   2. Pentru fiecare canal 0..5:
 *        90° -> 0° -> 90° -> 180° -> 90°
 *   3. La final, toate raman la 90°
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pca9685.h"

#define I2C_PORT    i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_FREQ    400000

#define NUM_SERVO    6
#define PULSE_MIN_US 500
#define PULSE_MAX_US 2500

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n==================================================\n");
    printf("TEST 3: Toate cele %d servo-uri\n", NUM_SERVO);
    printf("==================================================\n");

    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    pca9685_t pca;
    if (!pca9685_init(&pca, I2C_PORT, PCA9685_DEFAULT_ADDR, 50)) {
        printf("EROARE: PCA9685 nu raspunde!\n");
        while (true) sleep_ms(1000);
    }
    printf("PCA9685 OK\n\n");

   
    printf("Initializare: toate la 90°...\n");
    for (int ch = 0; ch < NUM_SERVO; ch++) {
        pca9685_set_servo_angle(&pca, ch, 90.0f, PULSE_MIN_US, PULSE_MAX_US);
    }
    sleep_ms(1500);

    printf("\nIncepem testul individual.\n\n");

    const float secventa[] = {90, 0, 90, 180, 90};
    const int n_pasi = sizeof(secventa) / sizeof(secventa[0]);

    while (true) {
        for (int canal = 0; canal < NUM_SERVO; canal++) {
            printf("--- Servo canal %d ---\n", canal);
            for (int p = 0; p < n_pasi; p++) {
                printf("   %.0f°\n", secventa[p]);
                pca9685_set_servo_angle(&pca, canal, secventa[p],
                                         PULSE_MIN_US, PULSE_MAX_US);
                sleep_ms(700);
            }
            sleep_ms(1000);
        }

        printf("\n=== Ciclu complet. Reluam in 3 secunde... ===\n\n");
        sleep_ms(3000);
    }

    return 0;
}
