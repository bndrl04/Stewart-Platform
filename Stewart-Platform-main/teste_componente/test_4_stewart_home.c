/**
 * test_4_stewart_home.c
 * =====================
 * Aduce platforma Stewart in pozitia HOME (toate bratele orizontale, paralele cu baza).
 *
 * CONCEPT:
 *   Fiecare servo are un OFFSET propriu fata de 90°, pentru ca:
 *     - Manivela nu se monteaza identic pe ax (axul are 25 dinti = pasi de 14.4°)
 *     - Servo-urile au mici variatii de calibrare din fabrica
 *
 * WORKFLOW DE CALIBRARE:
 *   1. Ruleaza prima data cu offseturi 0. Observa care brate nu sunt orizontale.
 *   2. Ajusteaza OFFSET_DEG[i]:
 *        - Brat prea sus  -> offset negativ (ex: -5)
 *        - Brat prea jos  -> offset pozitiv (ex: +7)
 *   3. Recompileaza si reflash-uieste. Repeta pana toate aliniate.
 *   4. NOTEAZA-TI offseturile finale - vor fi necesare pentru cinematica Stewart!
 *
 * SIGURANTA:
 *   Miscarea spre HOME e LENTA (rampa interpolata) - nu socheaza platforma.
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

// OFFSETURI DE CALIBRARE per canal (in grade, fata de 90°)
// === EDITEAZA AICI dupa observatia fizica ===
static const float OFFSET_DEG[NUM_SERVO] = {
    0.0f,   // canal 0
    0.0f,   // canal 1
    0.0f,   // canal 2
    0.0f,   // canal 3
    0.0f,   // canal 4
    0.0f,   // canal 5
};

// Limite de siguranta - nu permitem servo-uri sa iasa din interval rezonabil
#define ANGLE_MIN 30.0f
#define ANGLE_MAX 150.0f

// Stare interna - ultima pozitie comandata pe fiecare canal
static float pozitie_curenta[NUM_SERVO];


/**
 * Misca un servo lent de la pozitie_curenta la unghi_tinta.
 * Folosit doar pentru primul move catre 90°.
 */
static void misca_lent(pca9685_t *pca, uint8_t canal, float unghi_tinta,
                        int ms_per_grad) {
    float start = pozitie_curenta[canal];
    float delta = unghi_tinta - start;
    int n_pasi = (int)(delta < 0 ? -delta : delta);
    if (n_pasi < 1) n_pasi = 1;

    for (int i = 1; i <= n_pasi; i++) {
        float ung = start + delta * ((float)i / (float)n_pasi);
        pca9685_set_servo_angle(pca, canal, ung, PULSE_MIN_US, PULSE_MAX_US);
        sleep_ms(ms_per_grad);
    }
    pozitie_curenta[canal] = unghi_tinta;
}


/**
 * Misca toate servo-urile SIMULTAN la pozitiile target,
 * cu rampa interpolata (toate ajung in acelasi timp).
 */
static void misca_toate_simultan(pca9685_t *pca, const float *target,
                                  int ms_per_grad) {
    float start[NUM_SERVO];
    float delta_max = 0.0f;

    for (int i = 0; i < NUM_SERVO; i++) {
        start[i] = pozitie_curenta[i];
        float d = target[i] - start[i];
        if (d < 0) d = -d;
        if (d > delta_max) delta_max = d;
    }

    int n_pasi = (int)delta_max;
    if (n_pasi < 1) n_pasi = 1;

    for (int step = 1; step <= n_pasi; step++) {
        float progres = (float)step / (float)n_pasi;
        for (int i = 0; i < NUM_SERVO; i++) {
            float ung = start[i] + (target[i] - start[i]) * progres;
            pca9685_set_servo_angle(pca, i, ung, PULSE_MIN_US, PULSE_MAX_US);
        }
        sleep_ms(ms_per_grad);
    }

    for (int i = 0; i < NUM_SERVO; i++) {
        pozitie_curenta[i] = target[i];
    }
}


int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n==================================================\n");
    printf("TEST 4: Pozitionare HOME platforma Stewart\n");
    printf("==================================================\n");

    // I2C init
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // PCA9685 init
    pca9685_t pca;
    if (!pca9685_init(&pca, I2C_PORT, PCA9685_DEFAULT_ADDR, 50)) {
        printf("EROARE: PCA9685 nu raspunde!\n");
        while (true) sleep_ms(1000);
    }
    printf("PCA9685 OK\n\n");

    // PAS 1: aducem toate servo-urile la 90° brusc (presupunem pornite de la 90°)
    printf("Pas 1: Initializare la 90° (fara offset)...\n");
    for (int ch = 0; ch < NUM_SERVO; ch++) {
        pca9685_set_servo_angle(&pca, ch, 90.0f, PULSE_MIN_US, PULSE_MAX_US);
        pozitie_curenta[ch] = 90.0f;
    }
    sleep_ms(1500);

    // PAS 2: calculam tintele HOME (90 + offset) cu verificare limite
    float target_home[NUM_SERVO];
    for (int i = 0; i < NUM_SERVO; i++) {
        target_home[i] = 90.0f + OFFSET_DEG[i];
        if (target_home[i] < ANGLE_MIN) {
            printf("AVERTISMENT: canal %d tinta %.1f° sub limita, fixez la %.1f°\n",
                   i, target_home[i], ANGLE_MIN);
            target_home[i] = ANGLE_MIN;
        }
        if (target_home[i] > ANGLE_MAX) {
            printf("AVERTISMENT: canal %d tinta %.1f° peste limita, fixez la %.1f°\n",
                   i, target_home[i], ANGLE_MAX);
            target_home[i] = ANGLE_MAX;
        }
    }

    // PAS 3: miscare lenta sincronizata catre HOME
    printf("\nPas 2: Aplicare offseturi catre HOME...\n");
    misca_toate_simultan(&pca, target_home, 20);

    printf("\nPozitia HOME atinsa:\n");
    for (int i = 0; i < NUM_SERVO; i++) {
        printf("  Canal %d: %6.1f°  (offset %+.1f)\n",
               i, target_home[i], OFFSET_DEG[i]);
    }

    printf("\n==================================================\n");
    printf("URMATORII PASI:\n");
    printf("1. Observa platforma. Este orizontala?\n");
    printf("2. Daca un brat e mai sus/mai jos, ajusteaza OFFSET_DEG[]\n");
    printf("   in cod, recompileaza si reruleaza.\n");
    printf("3. Cand toate bratele sunt aliniate, NOTEAZA offseturile!\n");
    printf("==================================================\n");

    // Mentinem pozitia
    while (true) {
        sleep_ms(5000);
    }

    return 0;
}
