// Cubli-style reaction-wheel balancer — ESP32-C3 + BNO085 + Nidec 24H BLDC
// ----------------------------------------------------------------------------
//
// Control approach: SIGNED output with direction reversal at zero.
//
//   u = K_P * (pitch - setpoint) + K_D * pitch_rate     (signed, Hz)
//
//   |u| < DEADBAND     -> stay at FREQ_MIN in current direction (no flip)
//   sign(u) == dir     -> slew commanded freq toward |u|, clamped
//   sign(u) != dir     -> slew freq down to FREQ_MIN, dwell so the rotor can
//                         coast (no BRAKE wired), toggle F/R, then ramp up
//                         to |u| in the new direction
//
// Architecture:
//
//   imu_task     : owns SH-2, decodes rotation vector + calibrated gyro
//                  inside the event handler (so no events are lost when
//                  multiple reports arrive in one service call)
//
//   slew_task    : 10 ms tick. State machine that tracks (current_dir,
//                  current_freq) and rate-limits frequency changes to
//                  SLEW_HZ_PER_SEC. Owns the direction-flip sequence.
//
//   control_task : 10 ms tick (100 Hz). In BALANCE mode reads pitch +
//                  pitch_rate, computes signed u, writes desired_signed.
//                  Runs safety clamps (tilt magnitude, IMU staleness).
//
//   app_main     : sets up LEDC/F/R/LED, spawns tasks, runs UART:
//                     0          -> motor off (ramp down, then stop)
//                     <number>   -> MANUAL forward at that freq
//                     -<number>  -> MANUAL reverse at that freq
//                     b          -> arm BALANCE mode
//                     s          -> print status
//
// Tuning: K_P, K_D, DEADBAND_HZ, and the direction-flip dwell are the
// knobs that matter. If the motor faults during flips, raise FLIP_DWELL
// or lower DIR_FLIP_FREQ.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "driver/gpio.h"
#include "led_strip.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

// ===== Pin map =============================================================
#define PWM_PIN              GPIO_NUM_1
#define FR_PIN               GPIO_NUM_0    // Nidec F/R; HIGH = reverse
#define LED_PIN              8

// F/R pin level for "forward" direction. If on bench the directions are
// inverted (corrective torque points the wrong way for positive pitch), flip
// this from 0 to 1.
#define FR_FORWARD_LEVEL     0

// ===== Motor frequency limits =============================================
#define FREQ_MIN             500
#define FREQ_MAX             26000
#define SLEW_HZ_PER_SEC      50000         // max rate of frequency change.
                                           // Lower if motor loses sync.
#define SLEW_TICK_MS         10            // slew task period (= 1 tick at
                                           // CONFIG_FREERTOS_HZ=100)

// ===== Direction flip behavior ===========================================
#define DEADBAND_HZ          300           // |u| below this -> idle, no flip
#define DIR_FLIP_FREQ        700           // F/R toggled only when current
                                           // commanded freq is at or below this
#define FLIP_DWELL_TICKS     2             // wait this many ticks (20 ms) at
                                           // low freq before toggling F/R, so
                                           // the rotor has time to coast down

// ===== Control gains =====================================================
#define CTRL_LOOP_MS         10            // 100 Hz
#define CTRL_K_P             400.0f        // Hz per degree of tilt
#define CTRL_K_D             50.0f         // Hz per (deg/sec) of pitch rate
#define CTRL_PITCH_SETPOINT  0.0f          // degrees; calibrate to upright

// ===== Safety =============================================================
#define CTRL_TILT_MAX_DEG    30.0f
#define CTRL_FRESH_US        30000         // 30 ms

// ===== LEDC ===============================================================
#define LEDC_DUTY_FIXED      500           // fixed ~50% on 10-bit

// ===========================================================================
// Shared state (single core C3; portMUX is the canonical lock)
// ===========================================================================
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

// IMU state — written by event handler, read by control_task
static volatile float    s_pitch_deg        = 0.0f;
static volatile float    s_pitch_rate_dps   = 0.0f;
static volatile uint32_t s_last_imu_us      = 0;
static volatile bool     s_imu_ready        = false;

// Motor state
typedef enum { MODE_OFF = 0, MODE_MANUAL, MODE_BALANCE } mode_t;
typedef enum { DIR_FORWARD = 0, DIR_REVERSE = 1 }        dir_t;

static volatile mode_t s_mode            = MODE_OFF;
static volatile float  s_desired_signed  = 0.0f;     // signed Hz from control
static volatile int    s_current_freq    = FREQ_MIN; // unsigned magnitude
static volatile dir_t  s_current_dir     = DIR_FORWARD;
static volatile bool   s_motor_armed     = false;

// ===========================================================================
// LED helpers
// ===========================================================================
static led_strip_handle_t strip;
static void led_set(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(strip, 0, r, g, b);
    led_strip_refresh(strip);
}
static void led_red(void)    { led_set(10, 0, 0); }
static void led_green(void)  { led_set(0, 10, 0); }
static void led_blue(void)   { led_set(0, 0, 10); }

// ===========================================================================
// Helpers
// ===========================================================================
static int clamp_freq(int hz) {
    if (hz < FREQ_MIN) return FREQ_MIN;
    if (hz > FREQ_MAX) return FREQ_MAX;
    return hz;
}

static int fr_level_for(dir_t d) {
    return (d == DIR_FORWARD) ? FR_FORWARD_LEVEL : (FR_FORWARD_LEVEL ^ 1);
}

static void set_desired_signed(float u_hz) {
    portENTER_CRITICAL(&state_mux);
    s_desired_signed = u_hz;
    portEXIT_CRITICAL(&state_mux);
}

// ===========================================================================
// Motor low-level
// ===========================================================================
static void motor_hw_start(void) {
    ledc_channel_config_t channel = {
        .gpio_num = PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = LEDC_DUTY_FIXED,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, FREQ_MIN);
    portENTER_CRITICAL(&state_mux);
    s_current_freq   = FREQ_MIN;
    s_desired_signed = 0.0f;
    portEXIT_CRITICAL(&state_mux);
    s_motor_armed = true;
}

static void motor_hw_stop(void) {
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    s_motor_armed = false;
}

// Non-blocking slew + direction state machine.
// Runs every SLEW_TICK_MS. Owns ledc_set_freq and the F/R pin.
static void slew_task(void *arg) {
    const int step = (SLEW_HZ_PER_SEC * SLEW_TICK_MS) / 1000;
    int dwell = 0;

    while (1) {
        float    u;
        dir_t    cur_dir;
        int      cur_freq;
        mode_t   mode;
        bool     armed;
        portENTER_CRITICAL(&state_mux);
        u        = s_desired_signed;
        cur_dir  = s_current_dir;
        cur_freq = s_current_freq;
        mode     = s_mode;
        armed    = s_motor_armed;
        portEXIT_CRITICAL(&state_mux);

        if (!armed) {
            vTaskDelay(pdMS_TO_TICKS(SLEW_TICK_MS));
            continue;
        }

        // ----- decide desired direction + magnitude from signed u -----
        dir_t want_dir;
        int   want_mag;
        if (fabsf(u) < DEADBAND_HZ) {
            // Small command: hold current direction, idle at FREQ_MIN.
            // No flip while we're inside the deadband.
            want_dir = cur_dir;
            want_mag = FREQ_MIN;
        } else {
            want_dir = (u > 0.0f) ? DIR_FORWARD : DIR_REVERSE;
            want_mag = clamp_freq((int)fabsf(u));
        }

        // ----- pick slew target this tick -----
        int target;
        if (want_dir != cur_dir) {
            // Direction reversal pending. First ramp down to flip-safe freq.
            target = FREQ_MIN;
            if (cur_freq <= DIR_FLIP_FREQ) {
                dwell++;
                if (dwell >= FLIP_DWELL_TICKS) {
                    // Rotor has had time to coast — flip F/R now.
                    gpio_set_level(FR_PIN, fr_level_for(want_dir));
                    portENTER_CRITICAL(&state_mux);
                    s_current_dir = want_dir;
                    portEXIT_CRITICAL(&state_mux);
                    cur_dir = want_dir;
                    dwell = 0;
                    // Next iteration will start ramping up to want_mag.
                }
            } else {
                dwell = 0;
            }
        } else {
            target = want_mag;
            dwell = 0;
        }

        // ----- slew current_freq toward target -----
        int new_freq = cur_freq;
        if (target > cur_freq) {
            new_freq = cur_freq + step;
            if (new_freq > target) new_freq = target;
        } else if (target < cur_freq) {
            new_freq = cur_freq - step;
            if (new_freq < target) new_freq = target;
        }
        if (new_freq != cur_freq) {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, new_freq);
            portENTER_CRITICAL(&state_mux);
            s_current_freq = new_freq;
            portEXIT_CRITICAL(&state_mux);
            cur_freq = new_freq;
        }

        // OFF mode: once we've slewed all the way down, kill LEDC.
        if (mode == MODE_OFF && cur_freq <= FREQ_MIN) {
            motor_hw_stop();
        }

        vTaskDelay(pdMS_TO_TICKS(SLEW_TICK_MS));
    }
}

// ===========================================================================
// IMU
// ===========================================================================
extern sh2_Hal_t *bno085_get_hal(void);

static void quat_to_euler(float r, float i, float j, float k,
                          float *roll, float *pitch, float *yaw);

// Decode + state write happen inside the SH-2 handler (runs in imu_task's
// context via sh2_service). This way we never lose a sample when rotation
// vector and gyro events arrive in the same service call.
static void imu_event_handler(void *cookie, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t v;
    sh2_decodeSensorEvent(&v, event);
    uint32_t now = (uint32_t)esp_timer_get_time();

    if (v.sensorId == SH2_ROTATION_VECTOR) {
        float roll, pitch, yaw;
        quat_to_euler(
            v.un.rotationVector.real,
            v.un.rotationVector.i,
            v.un.rotationVector.j,
            v.un.rotationVector.k,
            &roll, &pitch, &yaw);
        portENTER_CRITICAL(&state_mux);
        s_pitch_deg   = pitch;
        s_last_imu_us = now;
        s_imu_ready   = true;
        portEXIT_CRITICAL(&state_mux);
    } else if (v.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        float rate_dps = v.un.gyroscope.y * (180.0f / (float)M_PI);
        portENTER_CRITICAL(&state_mux);
        s_pitch_rate_dps = rate_dps;
        portEXIT_CRITICAL(&state_mux);
    }
}

static void imu_async_handler(void *cookie, sh2_AsyncEvent_t *event) {
    (void)cookie; (void)event;
}

static void quat_to_euler(float r, float i, float j, float k,
                          float *roll, float *pitch, float *yaw) {
    *roll  = atan2f(2*(r*i + j*k), 1 - 2*(i*i + j*j)) * (180.0f / (float)M_PI);
    float sinp = 2*(r*j - k*i);
    *pitch = (fabsf(sinp) >= 1) ? copysignf(90.0f, sinp)
                                : asinf(sinp) * (180.0f / (float)M_PI);
    *yaw   = atan2f(2*(r*k + i*j), 1 - 2*(j*j + k*k)) * (180.0f / (float)M_PI);
}

static void bno085_init(void) {
    printf("BNO085 init starting...\n");
    sh2_Hal_t *hal = bno085_get_hal();
    int rc = sh2_open(hal, imu_async_handler, NULL);
    if (rc != SH2_OK) { printf("sh2_open failed: %d\n", rc); return; }
    sh2_setSensorCallback(imu_event_handler, NULL);

    sh2_SensorConfig_t cfg = { .reportInterval_us = 10000 };  // 100 Hz
    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &cfg);
    if (rc != SH2_OK) printf("enable rotation vector failed: %d\n", rc);
    rc = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
    if (rc != SH2_OK) printf("enable gyro failed: %d\n", rc);

    printf("BNO085 ready\n");
}

// imu_task owns SH-2 exclusively. Nothing else may call sh2_service().
static void imu_task(void *arg) {
    bno085_init();
    while (1) {
        sh2_service();
        vTaskDelay(1);   // 1 tick = 10 ms at FREERTOS_HZ=100
    }
}

// ===========================================================================
// Control task — the PD law producing signed Hz
// ===========================================================================
static void control_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    int print_div = 0;
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_LOOP_MS));

        float    pitch, pitch_rate;
        uint32_t last_us;
        bool     ready;
        mode_t   mode;
        portENTER_CRITICAL(&state_mux);
        pitch      = s_pitch_deg;
        pitch_rate = s_pitch_rate_dps;
        last_us    = s_last_imu_us;
        ready      = s_imu_ready;
        mode       = s_mode;
        portEXIT_CRITICAL(&state_mux);

        if (mode != MODE_BALANCE) continue;

        uint32_t now    = (uint32_t)esp_timer_get_time();
        uint32_t age_us = now - last_us;

        // ----- safety clamps -----
        bool safe_fault = false;
        if (!ready || age_us > CTRL_FRESH_US) {
            safe_fault = true;
            printf("[SAFE] IMU stale (%lu us)\n", (unsigned long)age_us);
        } else if (fabsf(pitch - CTRL_PITCH_SETPOINT) > CTRL_TILT_MAX_DEG) {
            safe_fault = true;
            printf("[SAFE] Tilt %.1f deg exceeds limit\n", (double)pitch);
        }
        if (safe_fault) {
            portENTER_CRITICAL(&state_mux);
            s_mode = MODE_OFF;
            portEXIT_CRITICAL(&state_mux);
            set_desired_signed(0.0f);
            led_red();
            continue;
        }

        // ----- PD law (signed) -----
        float err = pitch - CTRL_PITCH_SETPOINT;
        float u   = CTRL_K_P * err + CTRL_K_D * pitch_rate;
        set_desired_signed(u);

        // Sparse telemetry every 100 ms
        if (++print_div >= 10) {
            print_div = 0;
            const char *dname = (s_current_dir == DIR_FORWARD) ? "FWD" : "REV";
            printf("BAL pitch=%6.2f rate=%7.2f u=%8.1f dir=%s cur=%5d\n",
                   (double)pitch, (double)pitch_rate, (double)u,
                   dname, s_current_freq);
        }
    }
}

// ===========================================================================
// Mode transitions
// ===========================================================================
static void enter_off(void) {
    portENTER_CRITICAL(&state_mux);
    s_mode = MODE_OFF;
    portEXIT_CRITICAL(&state_mux);
    set_desired_signed(0.0f);   // slew_task ramps to FREQ_MIN, then stops LEDC
    led_red();
    printf("[MODE] OFF (ramping down)\n");
}

// Positive freq = FORWARD, negative freq = REVERSE. Useful for testing
// the F/R wiring without entering BALANCE mode.
static void enter_manual(int signed_freq) {
    if (!s_motor_armed) motor_hw_start();
    portENTER_CRITICAL(&state_mux);
    s_mode = MODE_MANUAL;
    portEXIT_CRITICAL(&state_mux);
    set_desired_signed((float)signed_freq);
    led_blue();
    printf("[MODE] MANUAL target=%d (%s)\n",
           abs(signed_freq), signed_freq >= 0 ? "FWD" : "REV");
}

static void enter_balance(void) {
    if (!s_imu_ready) {
        printf("[MODE] cannot arm BALANCE - no IMU sample yet\n");
        return;
    }
    if (!s_motor_armed) motor_hw_start();
    portENTER_CRITICAL(&state_mux);
    s_mode = MODE_BALANCE;
    portEXIT_CRITICAL(&state_mux);
    set_desired_signed(0.0f);
    led_green();
    printf("[MODE] BALANCE armed (Kp=%.1f Kd=%.1f deadband=%d)\n",
           (double)CTRL_K_P, (double)CTRL_K_D, DEADBAND_HZ);
}

static void print_status(void) {
    const char *mname = (s_mode == MODE_OFF) ? "OFF"
                      : (s_mode == MODE_MANUAL) ? "MANUAL" : "BALANCE";
    const char *dname = (s_current_dir == DIR_FORWARD) ? "FWD" : "REV";
    printf("[STATUS] mode=%s armed=%d dir=%s cur=%d u=%.1f pitch=%.2f rate=%.2f\n",
           mname, (int)s_motor_armed, dname, s_current_freq,
           (double)s_desired_signed, (double)s_pitch_deg,
           (double)s_pitch_rate_dps);
}

// ===========================================================================
// app_main
// ===========================================================================
void app_main(void)
{
    // --- LED strip ---
    led_strip_config_t led_cfg = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = { .resolution_hz = 10 * 1000 * 1000 };
    led_strip_new_rmt_device(&led_cfg, &rmt_cfg, &strip);
    led_red();

    // --- F/R pin: forward at boot ---
    gpio_config_t fr_io = {
        .pin_bit_mask = (1ULL << FR_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&fr_io);
    gpio_set_level(FR_PIN, FR_FORWARD_LEVEL);
    s_current_dir = DIR_FORWARD;

    // --- LEDC timer (channel attached on motor_hw_start) ---
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = FREQ_MIN,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    // --- Tasks ---
    xTaskCreate(imu_task,     "imu",     4096, NULL, 5, NULL);
    xTaskCreate(slew_task,    "slew",    2048, NULL, 6, NULL);
    xTaskCreate(control_task, "control", 4096, NULL, 4, NULL);

    printf("Cubli balancer ready.\n");
    printf("  <number>  MANUAL freq, positive=forward, negative=reverse\n");
    printf("  0         motor OFF\n");
    printf("  b         arm BALANCE mode\n");
    printf("  s         print status\n");

    // --- UART command loop ---
    char buf[32];
    int  pos = 0;
    while (1) {
        int c = getchar();
        if (c == EOF || c < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (c == '\r' || c == '\n') {
            if (pos == 0) continue;
            buf[pos] = '\0';
            pos = 0;
            printf("\n");

            if (buf[0] == 'b' || buf[0] == 'B') {
                enter_balance();
            } else if (buf[0] == 's' || buf[0] == 'S') {
                print_status();
            } else if (buf[0] == '-' || isdigit((unsigned char)buf[0])) {
                int freq = atoi(buf);
                if (freq == 0) {
                    enter_off();
                } else {
                    int mag = abs(freq);
                    if (mag >= FREQ_MIN && mag <= FREQ_MAX) {
                        enter_manual(freq);
                    } else {
                        printf("Out of range. 0 or +/-%d..%d.\n",
                               FREQ_MIN, FREQ_MAX);
                    }
                }
            } else {
                printf("Unknown command: %s\n", buf);
            }
        } else if (pos < (int)sizeof(buf) - 1) {
            printf("%c", (char)c);
            buf[pos++] = (char)c;
        }
    }
}