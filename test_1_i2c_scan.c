/**
 * test_1_i2c_scan.c
 * =================
 * Scaneaza magistrala I2C0 si afiseaza dispozitivele gasite.
 * Confirma ca Pico vede PCA9685 (adresa 0x40).
 *
 * CABLAJ NECESAR:
 *   Pico GP4 (pin 6)  -> PCA9685 SDA
 *   Pico GP5 (pin 7)  -> PCA9685 SCL
 *   Pico 3V3 (pin 36) -> PCA9685 VCC
 *   Pico GND (pin 38) -> PCA9685 GND
 *   (sursa WELL nu e necesara pentru acest test)
 *
 * REZULTAT ASTEPTAT in serial monitor:
 *   Dispozitiv I2C gasit la adresa 0x40 (PCA9685)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT    i2c0
#define I2C_SDA_PIN 4    // GP4
#define I2C_SCL_PIN 5    // GP5
#define I2C_FREQ    400000  // 400 kHz

int main() {
    stdio_init_all();

    // Asteptam un pic pentru a se conecta serial-ul USB
    sleep_ms(8000);

    printf("\n");
    printf("==================================================\n");
    printf("TEST 1: Scan I2C\n");
    printf("==================================================\n");

    // Initializare I2C0
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("Scanare magistrala I2C0 (SDA=GP%d, SCL=GP%d)...\n\n",
           I2C_SDA_PIN, I2C_SCL_PIN);

    int n_devices = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t rxdata;
        int ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
        if (ret >= 0) {
            printf("  Dispozitiv gasit la adresa 0x%02X", addr);
            if (addr == 0x40) {
                printf(" (PCA9685 - corect!)");
            }
            printf("\n");
            n_devices++;
        }
    }

    printf("\n");
    if (n_devices == 0) {
        printf("!!! NICIUN DISPOZITIV GASIT !!!\n");
        printf("Verifica:\n");
        printf("  - SDA pe GP4 (pin 6)\n");
        printf("  - SCL pe GP5 (pin 7)\n");
        printf("  - VCC pe 3V3 (pin 36)\n");
        printf("  - GND comun\n");
    } else {
        printf("Total: %d dispozitiv(e) gasit(e).\n", n_devices);
    }

    // Buclam infinit pentru a tine activ serial-ul
    while (true) {
        sleep_ms(5000);
        printf("(Test 1 terminat. Apasa BOOTSEL pentru a flash-ui alt test.)\n");
    }

    return 0;
}
