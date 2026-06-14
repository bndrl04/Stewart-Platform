#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include "pca9685.h"
#include "stewart_kinematics.h"

// ---- Hardware config ----
#define I2C_PORT     i2c0
#define I2C_SDA_PIN  4
#define I2C_SCL_PIN  5
#define I2C_FREQ_HZ  (400 * 1000)

#define PCA9685_ADDR PCA9685_DEFAULT_ADDR
#define SERVO_PWM_HZ 50.0f

// ---- Per-servo calibration ----
// Mechanical offset applied AFTER subtracting the HOME crank angle. Tweak each
// value after physical calibration (palonier mount tolerance). Positive value
// rotates the crank further CCW/up at the same logical command.
static const float servo_home_offset_deg[STEWART_NUM_LEGS] = {
    0.0f, 0.0f, 0.0f, +15.0f, 0.0f, 0.0f
};

// Physical wiring sign per servo. Inverts the PWM pulse direction to match how
// the servo is physically mounted (i.e. which way a positive crank delta makes
// the rod end go). In the paired/mirrored config, the mirrored servos in each
// pair end up with opposite physical sign relative to the standard ones.
// Calibrate empirically: after `home`, if `pos 0 0 H0+5` lowers a corner
// instead of raising it, flip the sign for the offending servo.
static const float servo_phys_sign[STEWART_NUM_LEGS] = {
    +1.0f, -1.0f, +1.0f, -1.0f, +1.0f, -1.0f
};

// PCA9685 channel assignment for legs 0..5.
static const uint8_t servo_channel[STEWART_NUM_LEGS] = { 0, 1, 2, 3, 4, 5 };

// ---- Motion state ----
typedef struct {
    float x, y, z, roll, pitch, yaw;
} pose_t;

static pose_t pose_current = { 0, 0, STEWART_H0, 0, 0, 0 };
static pose_t pose_target  = { 0, 0, STEWART_H0, 0, 0, 0 };
static volatile bool motion_active = false;

// Linear interpolation step per 10 ms tick. Tuned for responsive Xbox control.
#define STEP_POS_MM_PER_TICK   0.30f   // 30 mm/s
#define STEP_ANG_DEG_PER_TICK  0.60f   // 60 deg/s

// Stream mode: when on, suppresses character echo, "OK" replies and the prompt
// so a host script can push high-rate `rt` commands without log clutter.
static volatile bool stream_mode = false;

// Safety limits for streaming control (conservative). Used by `rt` command only.
#define RT_LIMIT_TRANS_MM       10.0f       // |x|, |y| <= this
#define RT_Z_MIN_MM             (STEWART_H0 - 10.0f)
#define RT_Z_MAX_MM             (STEWART_H0 + 15.0f)
#define RT_LIMIT_ROLL_PITCH_DEG 10.0f
#define RT_LIMIT_YAW_DEG        15.0f

// HOME crank angles, populated at boot from IK.
static float alpha_home[STEWART_NUM_LEGS] = {0};

// Servo override: when non-negative, that servo is being driven directly via
// "servo N angle" and is excluded from IK updates.
static int8_t servo_override_active[STEWART_NUM_LEGS] = { 0, 0, 0, 0, 0, 0 };

// Demo mode flag (cycles through poses).
static bool demo_running = false;
static uint32_t demo_step = 0;
static uint32_t demo_next_ms = 0;

// ---- Helpers ----

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void apply_pose(const pose_t *p) {
    float crank[STEWART_NUM_LEGS];
    unsigned int unreachable = 0;
    int rc = stewart_ik(p->x, p->y, p->z, p->roll, p->pitch, p->yaw,
                        crank, &unreachable);
    if (rc & STEWART_ERR_UNREACHABLE) {
        printf("WARN: unreachable, mask=0x%02x (legs clamped)\n", unreachable);
    }
    for (int i = 0; i < STEWART_NUM_LEGS; ++i) {
        if (servo_override_active[i]) continue;
        float delta = crank[i] - alpha_home[i];
        float cmd = servo_phys_sign[i] * delta + servo_home_offset_deg[i];
        cmd = clampf(cmd, -90.0f, 90.0f);
        pca9685_set_servo_angle(servo_channel[i], cmd);
    }
}

static bool step_toward(float *cur, float target, float step) {
    float diff = target - *cur;
    if (fabsf(diff) <= step) {
        *cur = target;
        return true;
    }
    *cur += (diff > 0 ? step : -step);
    return false;
}

// 100 Hz motion timer.
static bool motion_tick(repeating_timer_t *t) {
    (void)t;
    if (!motion_active) return true;
    bool d1 = step_toward(&pose_current.x,     pose_target.x,     STEP_POS_MM_PER_TICK);
    bool d2 = step_toward(&pose_current.y,     pose_target.y,     STEP_POS_MM_PER_TICK);
    bool d3 = step_toward(&pose_current.z,     pose_target.z,     STEP_POS_MM_PER_TICK);
    bool d4 = step_toward(&pose_current.roll,  pose_target.roll,  STEP_ANG_DEG_PER_TICK);
    bool d5 = step_toward(&pose_current.pitch, pose_target.pitch, STEP_ANG_DEG_PER_TICK);
    bool d6 = step_toward(&pose_current.yaw,   pose_target.yaw,   STEP_ANG_DEG_PER_TICK);
    apply_pose(&pose_current);
    if (d1 && d2 && d3 && d4 && d5 && d6) motion_active = false;
    return true;
}

// Returns true if the pose was accepted, false if rejected (unreachable target).
// When rejected, neither pose_target nor pose_current changes; the current
// motion (if any) continues to its existing target.
static bool set_target_pose(float x, float y, float z,
                            float roll, float pitch, float yaw) {
    float dummy[STEWART_NUM_LEGS];
    unsigned int unreachable = 0;
    int rc = stewart_ik(x, y, z, roll, pitch, yaw, dummy, &unreachable);
    if (rc & STEWART_ERR_UNREACHABLE) {
        printf("REJECT: target outside workspace, leg mask=0x%02x\n", unreachable);
        return false;
    }
    pose_target.x     = x;
    pose_target.y     = y;
    pose_target.z     = z;
    pose_target.roll  = roll;
    pose_target.pitch = pitch;
    pose_target.yaw   = yaw;
    motion_active = true;
    return true;
}

// ---- Demo sequence ----

typedef struct {
    pose_t pose;
    uint32_t hold_ms;
} demo_step_t;

static const demo_step_t demo_seq[] = {
    {{   0,   0, STEWART_H0,        0,    0,    0 }, 1000 },
    {{   0,   0, STEWART_H0 + 15,   0,    0,    0 }, 1000 },
    {{   0,   0, STEWART_H0 - 15,   0,    0,    0 }, 1000 },
    {{   0,   0, STEWART_H0,       10,    0,    0 }, 1000 },
    {{   0,   0, STEWART_H0,      -10,    0,    0 }, 1000 },
    {{   0,   0, STEWART_H0,        0,   10,    0 }, 1000 },
    {{   0,   0, STEWART_H0,        0,  -10,    0 }, 1000 },
    {{   0,   0, STEWART_H0,        0,    0,   15 }, 1000 },
    {{   0,   0, STEWART_H0,        0,    0,  -15 }, 1000 },
    {{  10,   0, STEWART_H0,        0,    0,    0 }, 1000 },
    {{   0,  10, STEWART_H0,        0,    0,    0 }, 1000 },
    {{   0,   0, STEWART_H0,        0,    0,    0 }, 1000 },
};
#define DEMO_LEN (sizeof(demo_seq) / sizeof(demo_seq[0]))

static void demo_advance(void) {
    if (!demo_running) return;
    if (!motion_active && (int32_t)(to_ms_since_boot(get_absolute_time()) - demo_next_ms) >= 0) {
        const demo_step_t *s = &demo_seq[demo_step];
        set_target_pose(s->pose.x, s->pose.y, s->pose.z,
                        s->pose.roll, s->pose.pitch, s->pose.yaw);
        demo_next_ms = to_ms_since_boot(get_absolute_time()) + s->hold_ms;
        demo_step = (demo_step + 1) % DEMO_LEN;
    }
}

// ---- Command parser ----

static void print_help(void) {
    printf(
        "Commands:\n"
        "  home              -> move to HOME (z=%.1f)\n"
        "  pos x y z         -> set translation, keep rotation\n"
        "  rot rx ry rz      -> set rotation (roll pitch yaw, deg), keep translation\n"
        "  pose x y z rx ry rz -> set full pose\n"
        "  servo N angle     -> drive servo N (0..5) to angle directly (-90..90)\n"
        "  release           -> release all servo overrides (resume IK)\n"
        "  demo              -> start demo sequence\n"
        "  stop              -> stop motion / demo\n"
        "  status            -> print current and target pose\n"
        "  stream on|off     -> toggle silent mode (no echo/prompt) for host streaming\n"
        "  rt x y z rx ry rz -> silent realtime pose update, clamped to safe limits\n"
        "  help              -> this list\n",
        STEWART_H0);
}

static void cmd_home(void) {
    demo_running = false;
    for (int i = 0; i < STEWART_NUM_LEGS; ++i) servo_override_active[i] = 0;
    set_target_pose(0, 0, STEWART_H0, 0, 0, 0);
    printf("OK home\n");
}

static void cmd_status(void) {
    printf("cur:  x=%.2f y=%.2f z=%.2f rx=%.2f ry=%.2f rz=%.2f\n",
           pose_current.x, pose_current.y, pose_current.z,
           pose_current.roll, pose_current.pitch, pose_current.yaw);
    printf("tgt:  x=%.2f y=%.2f z=%.2f rx=%.2f ry=%.2f rz=%.2f\n",
           pose_target.x, pose_target.y, pose_target.z,
           pose_target.roll, pose_target.pitch, pose_target.yaw);
    printf("motion=%d demo=%d\n", motion_active ? 1 : 0, demo_running ? 1 : 0);
}

static void handle_command(char *line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') return;

    char *tok = strtok(line, " \t\r\n");
    if (!tok) return;

    if (strcmp(tok, "help") == 0) {
        print_help();
    } else if (strcmp(tok, "home") == 0) {
        cmd_home();
    } else if (strcmp(tok, "status") == 0) {
        cmd_status();
    } else if (strcmp(tok, "stop") == 0) {
        motion_active = false;
        demo_running = false;
        pose_target = pose_current;
        printf("OK stop\n");
    } else if (strcmp(tok, "demo") == 0) {
        demo_running = true;
        demo_step = 0;
        demo_next_ms = to_ms_since_boot(get_absolute_time());
        for (int i = 0; i < STEWART_NUM_LEGS; ++i) servo_override_active[i] = 0;
        printf("OK demo\n");
    } else if (strcmp(tok, "release") == 0) {
        for (int i = 0; i < STEWART_NUM_LEGS; ++i) servo_override_active[i] = 0;
        set_target_pose(pose_current.x, pose_current.y, pose_current.z,
                        pose_current.roll, pose_current.pitch, pose_current.yaw);
        printf("OK release\n");
    } else if (strcmp(tok, "pos") == 0) {
        char *sx = strtok(NULL, " \t\r\n");
        char *sy = strtok(NULL, " \t\r\n");
        char *sz = strtok(NULL, " \t\r\n");
        if (!sx || !sy || !sz) { printf("ERR pos needs 3 args\n"); return; }
        if (set_target_pose(strtof(sx, NULL), strtof(sy, NULL), strtof(sz, NULL),
                            pose_target.roll, pose_target.pitch, pose_target.yaw)) {
            printf("OK pos\n");
        }
    } else if (strcmp(tok, "rot") == 0) {
        char *sr = strtok(NULL, " \t\r\n");
        char *sp = strtok(NULL, " \t\r\n");
        char *sy = strtok(NULL, " \t\r\n");
        if (!sr || !sp || !sy) { printf("ERR rot needs 3 args\n"); return; }
        if (set_target_pose(pose_target.x, pose_target.y, pose_target.z,
                            strtof(sr, NULL), strtof(sp, NULL), strtof(sy, NULL))) {
            printf("OK rot\n");
        }
    } else if (strcmp(tok, "pose") == 0) {
        char *args[6];
        for (int i = 0; i < 6; ++i) {
            args[i] = strtok(NULL, " \t\r\n");
            if (!args[i]) { printf("ERR pose needs 6 args\n"); return; }
        }
        if (set_target_pose(strtof(args[0], NULL), strtof(args[1], NULL), strtof(args[2], NULL),
                            strtof(args[3], NULL), strtof(args[4], NULL), strtof(args[5], NULL))) {
            printf("OK pose\n");
        }
    } else if (strcmp(tok, "rt") == 0) {
        // Realtime pose update: silent, clamped, used by host streaming scripts.
        char *args[6];
        for (int i = 0; i < 6; ++i) {
            args[i] = strtok(NULL, " \t\r\n");
            if (!args[i]) return;  // silent fail in stream
        }
        float x  = clampf(strtof(args[0], NULL), -RT_LIMIT_TRANS_MM,      RT_LIMIT_TRANS_MM);
        float y  = clampf(strtof(args[1], NULL), -RT_LIMIT_TRANS_MM,      RT_LIMIT_TRANS_MM);
        float z  = clampf(strtof(args[2], NULL),  RT_Z_MIN_MM,            RT_Z_MAX_MM);
        float rr = clampf(strtof(args[3], NULL), -RT_LIMIT_ROLL_PITCH_DEG, RT_LIMIT_ROLL_PITCH_DEG);
        float rp = clampf(strtof(args[4], NULL), -RT_LIMIT_ROLL_PITCH_DEG, RT_LIMIT_ROLL_PITCH_DEG);
        float ry = clampf(strtof(args[5], NULL), -RT_LIMIT_YAW_DEG,        RT_LIMIT_YAW_DEG);
        // Clear any leftover servo overrides so all 6 follow the IK in streaming.
        for (int i = 0; i < STEWART_NUM_LEGS; ++i) servo_override_active[i] = 0;
        set_target_pose(x, y, z, rr, rp, ry);
        // no OK reply: keep upstream traffic minimal
    } else if (strcmp(tok, "stream") == 0) {
        char *arg = strtok(NULL, " \t\r\n");
        if (arg && strcmp(arg, "on") == 0) {
            stream_mode = true;
            printf("OK stream on\n");
        } else if (arg && strcmp(arg, "off") == 0) {
            stream_mode = false;
            printf("OK stream off\n");
        } else {
            printf("ERR stream needs 'on' or 'off'\n");
        }
    } else if (strcmp(tok, "servo") == 0) {
        char *sn = strtok(NULL, " \t\r\n");
        char *sa = strtok(NULL, " \t\r\n");
        if (!sn || !sa) { printf("ERR servo needs 2 args\n"); return; }
        int n = atoi(sn);
        float a = strtof(sa, NULL);
        if (n < 0 || n >= STEWART_NUM_LEGS) { printf("ERR servo idx 0..5\n"); return; }
        servo_override_active[n] = 1;
        pca9685_set_servo_angle(servo_channel[n], clampf(servo_phys_sign[n] * a, -90.0f, 90.0f));
        printf("OK servo %d %.2f\n", n, a);
    } else {
        printf("ERR unknown '%s' (try 'help')\n", tok);
    }
}

// ---- Main ----

int main(void) {
    stdio_init_all();

    // Wait briefly for USB CDC to come up so the banner is visible.
    for (int i = 0; i < 30 && !stdio_usb_connected(); ++i) sleep_ms(100);

    i2c_init(I2C_PORT, I2C_FREQ_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("Stewart Platform 6-RSS (XBox control) booting...\n");

    if (!pca9685_init(I2C_PORT, PCA9685_ADDR)) {
        printf("FATAL: PCA9685 init failed at 0x%02x\n", PCA9685_ADDR);
        while (1) tight_loop_contents();
    }
    if (!pca9685_set_pwm_freq(SERVO_PWM_HZ)) {
        printf("FATAL: PCA9685 freq set failed\n");
        while (1) tight_loop_contents();
    }

    int rc = stewart_home_angles(alpha_home);
    if (rc != STEWART_OK) {
        printf("WARN: HOME IK status=0x%02x (geometry off?)\n", rc);
    }
    printf("HOME crank angles (deg): ");
    for (int i = 0; i < STEWART_NUM_LEGS; ++i) printf("%.2f ", alpha_home[i]);
    printf("\n");

    // Park at HOME (no smooth ramp yet, no current pose known).
    apply_pose(&pose_current);

    repeating_timer_t timer;
    add_repeating_timer_ms(-10, motion_tick, NULL, &timer);

    print_help();
    printf("> ");
    fflush(stdout);

    char line[128];
    size_t len = 0;
    while (true) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) {
            demo_advance();
            sleep_ms(2);
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                line[len] = '\0';
                if (!stream_mode) putchar('\n');
                handle_command(line);
                len = 0;
                if (!stream_mode) {
                    printf("> ");
                    fflush(stdout);
                }
            }
        } else if (c == 0x08 || c == 0x7F) {
            if (len > 0) {
                --len;
                if (!stream_mode) { printf("\b \b"); fflush(stdout); }
            }
        } else if (len < sizeof(line) - 1 && c >= 0x20 && c < 0x7F) {
            line[len++] = (char)c;
            if (!stream_mode) { putchar(c); fflush(stdout); }
        }
    }
}
