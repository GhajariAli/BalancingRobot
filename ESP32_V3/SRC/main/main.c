// Cubli-style reaction-wheel balancer — ESP32-C3 + BNO085 + Nidec 24H BLDC
// ----------------------------------------------------------------------------
//
// Control approach: DIRECTION FLIPPING with BRAKE-ASSISTED transitions.
//
//   desired_accel = K_P * pitch_err + K_D * pitch_rate - K_OMEGA * wheel_signed
//   desired_signed += desired_accel * dt                      (signed Hz)
//
//   sign(desired_signed) selects motor direction via F/R.
//   |desired_signed| sets the magnitude commanded on the LEDC clock input.
//
//   When sign(desired_signed) differs from current motor direction, the slew
//   task engages BRAKE to drop the rotor fast, dwells briefly so the rotor
//   actually stops, toggles F/R, releases BRAKE, then ramps up in the new
//   direction. With BRAKE wired, this transition is short instead of the
//   ~450 ms coast-down we had before.
//
//   K_OMEGA bleeds signed wheel speed toward zero so the wheel doesn't
//   accumulate unbounded momentum.
//
// UART:
//   0          motor OFF
//   <number>   MANUAL: positive = forward, negative = reverse
//   b          arm BALANCE mode
//   s          status / g  gains+knobs
//   p,d,w,c    K_P, K_D, K_OMEGA, pitch setpoint
//   r,z,f      slew rate, deadband, flip dwell
//   x <0|1>    manual brake override (for hardware testing)
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
#define FR_PIN               GPIO_NUM_0    // Nidec F/R
#define BRAKE_PIN            GPIO_NUM_10   // Nidec BRAKE (active LOW typical)
#define LED_PIN              8

// F/R level for "forward". If on bench positive desired pushes the body the
// wrong way, flip this (or invert the sign of K_P).
#define FR_FORWARD_LEVEL     0

// BRAKE polarity. If `x1` does nothing (or vice versa), flip these.
#define BRAKE_ENGAGED_LEVEL  0
#define BRAKE_RELEASED_LEVEL 1

// ===== Motor frequency limits =============================================
#define FREQ_MIN             500
#define FREQ_MAX             26000
#define SLEW_TICK_MS         10            // = 1 tick at FREERTOS_HZ=100

// Live-tunable motor knobs (UART: r, z, f, m, a). Power-on defaults.
#define SLEW_HZ_PER_SEC_DEFAULT  20000     // max rate of frequency change
#define DEADBAND_HZ_DEFAULT      200       // |desired| below this -> idle/no flip
#define FLIP_DWELL_TICKS_DEFAULT 3         // ticks at low freq before F/R toggle
                                           // (~30 ms with brake engaged)
#define DIR_FLIP_FREQ            FREQ_MIN  // brake until this freq, then flip F/R

// Anti-windup: integrator can lead the motor's actual speed by at most this
// many Hz. Prevents the integrator from racing ahead during slewing.
#define ANTIWINDUP_LEAD_HZ_DEFAULT  5000

// Direction-flip debounce: refuse a new F/R flip within this many ms of the
// last one. Protects the Nidec ESC from rapid reversals that can latch a
// fault. Set 0 to disable.
#define FLIP_MIN_INTERVAL_MS_DEFAULT 150

// ===== Control gains =====================================================
// Acceleration-based law:
//   desired_accel = K_P * pitch_err + K_D * pitch_rate - K_OMEGA * wheel_signed
//   desired_signed += desired_accel * dt
// Gains in Hz/sec units.
#define CTRL_LOOP_MS             10        // 100 Hz
#define CTRL_K_P_DEFAULT         2000.0f   // (Hz/s) per degree of tilt
#define CTRL_K_D_DEFAULT         200.0f    // (Hz/s) per (deg/s) of pitch rate
#define CTRL_K_OMEGA_DEFAULT     0.3f      // 1/s; wheel-speed decay rate
#define CTRL_SETPOINT_DEFAULT    0.0f      // degrees; calibrate to upright

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
static volatile float  s_desired         = 0.0f;     // signed Hz (controller out)
static volatile int    s_current_freq    = FREQ_MIN; // unsigned magnitude
static volatile dir_t  s_current_dir     = DIR_FORWARD;
static volatile bool   s_motor_armed     = false;
static volatile bool   s_brake_active    = false;

// Live-tunable gains
static volatile float  g_kp        = CTRL_K_P_DEFAULT;
static volatile float  g_kd        = CTRL_K_D_DEFAULT;
static volatile float  g_komega    = CTRL_K_OMEGA_DEFAULT;
static volatile float  g_setpoint  = CTRL_SETPOINT_DEFAULT;

// Live-tunable motor knobs
static volatile int    g_slew_hz           = SLEW_HZ_PER_SEC_DEFAULT;
static volatile int    g_deadband_hz       = DEADBAND_HZ_DEFAULT;
static volatile int    g_flip_dwell        = FLIP_DWELL_TICKS_DEFAULT;
static volatile int    g_antiwindup_hz     = ANTIWINDUP_LEAD_HZ_DEFAULT;
static volatile int    g_flip_min_interval_ms = FLIP_MIN_INTERVAL_MS_DEFAULT;
static volatile uint32_t s_last_flip_us    = 0;

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

static void set_desired(float v) {
    portENTER_CRITICAL(&state_mux);
    s_desired = v;
    portEXIT_CRITICAL(&state_mux);
}

// Idempotent — safe to call every tick.
static void brake_set(bool engaged) {
    if (engaged == s_brake_active) return;
    s_brake_active = engaged;
    gpio_set_level(BRAKE_PIN,
                   engaged ? BRAKE_ENGAGED_LEVEL : BRAKE_RELEASED_LEVEL);
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
    s_current_freq = FREQ_MIN;
    s_desired      = 0.0f;
    portEXIT_CRITICAL(&state_mux);
    s_motor_armed = true;
}

static void motor_hw_stop(void) {
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    s_motor_armed = false;
}

// Slew task: state machine that owns ledc_set_freq, the F/R pin, and BRAKE
// during direction-change transitions. Per tick:
//   1) Decide desired direction + magnitude from signed s_desired.
//   2) If direction matches current_dir, slew toward magnitude.
//   3) Else: brake on, ramp current_freq down to FREQ_MIN, dwell, flip F/R,
//      brake off, then start ramping toward new magnitude.
static void slew_task(void *arg) {
    int dwell = 0;
    bool in_flip = false;

    while (1) {
        float    desired;
        dir_t    cur_dir;
        int      cur_freq;
        mode_t   mode;
        bool     armed;
        portENTER_CRITICAL(&state_mux);
        desired  = s_desired;
        cur_dir  = s_current_dir;
        cur_freq = s_current_freq;
        mode     = s_mode;
        armed    = s_motor_armed;
        portEXIT_CRITICAL(&state_mux);

        if (!armed) {
            in_flip = false;
            dwell   = 0;
            vTaskDelay(pdMS_TO_TICKS(SLEW_TICK_MS));
            continue;
        }

        // Recompute step each tick — slew rate is live-tunable.
        int step = (g_slew_hz * SLEW_TICK_MS) / 1000;
        if (step < 1) step = 1;

        // ----- desired direction + magnitude from signed command -----
        dir_t want_dir;
        int   want_mag;
        if (fabsf(desired) < (float)g_deadband_hz) {
            // Small command: stay in current direction at FREQ_MIN. No flip.
            want_dir = cur_dir;
            want_mag = FREQ_MIN;
        } else {
            want_dir = (desired > 0.0f) ? DIR_FORWARD : DIR_REVERSE;
            want_mag = clamp_freq((int)fabsf(desired));
        }

        // ----- target this tick + brake state -----
        // Flip debounce: if we just completed a flip recently, refuse a new
        // one. Protects the Nidec ESC from latching a fault from too-rapid
        // direction reversals.
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        bool flip_allowed = !s_last_flip_us ||
            ((now_us - s_last_flip_us) >= (uint32_t)g_flip_min_interval_ms * 1000);

        int target;
        if ((want_dir != cur_dir && flip_allowed) || in_flip) {
            // Direction-flip sequence.
            in_flip = true;
            target  = FREQ_MIN;
            brake_set(true);   // active brake — rotor drops fast
            if (cur_freq <= DIR_FLIP_FREQ) {
                dwell++;
                if (dwell >= g_flip_dwell) {
                    // Rotor is stopped (or close). Flip F/R, release brake,
                    // exit flip state. Next tick begins ramp-up.
                    gpio_set_level(FR_PIN, fr_level_for(want_dir));
                    portENTER_CRITICAL(&state_mux);
                    s_current_dir = want_dir;
                    portEXIT_CRITICAL(&state_mux);
                    cur_dir = want_dir;
                    brake_set(false);
                    in_flip = false;
                    dwell   = 0;
                    s_last_flip_us = now_us;   // arm the debounce window
                }
            } else {
                dwell = 0;
            }
        } else if (want_dir != cur_dir) {
            // Flip wanted but debounced. Hold at FREQ_MIN in current dir
            // (no brake — let motor coast). When debounce window expires,
            // a flip will be allowed on a subsequent tick.
            target = FREQ_MIN;
            brake_set(false);
        } else {
            target = want_mag;
            brake_set(false);
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

        if (mode == MODE_OFF && cur_freq <= FREQ_MIN && !in_flip) {
            brake_set(false);
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
        // Kept for completeness; control_task computes its own rate from
        // numerical derivative of pitch (more robust to mount orientation).
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

static void imu_task(void *arg) {
    bno085_init();
    while (1) {
        sh2_service();
        vTaskDelay(1);   // 1 tick = 10 ms at FREERTOS_HZ=100
    }
}

// ===========================================================================
// Control task — acceleration-based PD with wheel-speed feedback
// ===========================================================================
static void control_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    int   print_div = 0;
    float prev_pitch = 0.0f;
    float rate_filt  = 0.0f;
    bool  rate_init  = false;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_LOOP_MS));

        float    pitch;
        uint32_t last_us;
        bool     ready;
        mode_t   mode;
        int      cur_freq;
        dir_t    cur_dir;
        portENTER_CRITICAL(&state_mux);
        pitch    = s_pitch_deg;
        last_us  = s_last_imu_us;
        ready    = s_imu_ready;
        mode     = s_mode;
        cur_freq = s_current_freq;
        cur_dir  = s_current_dir;
        portEXIT_CRITICAL(&state_mux);

        // ----- pitch rate via numerical derivative + 1st-order lowpass -----
        const float dt_ctrl = CTRL_LOOP_MS / 1000.0f;
        const float alpha   = 0.4f;
        float pitch_rate = 0.0f;
        if (ready) {
            if (!rate_init) {
                prev_pitch = pitch;
                rate_filt  = 0.0f;
                rate_init  = true;
            } else {
                float rate_raw = (pitch - prev_pitch) / dt_ctrl;
                rate_filt  = alpha * rate_raw + (1.0f - alpha) * rate_filt;
                pitch_rate = rate_filt;
                prev_pitch = pitch;
            }
        } else {
            rate_init = false;
        }

        if (mode != MODE_BALANCE) continue;

        uint32_t now    = (uint32_t)esp_timer_get_time();
        uint32_t age_us = now - last_us;

        // ----- safety clamps -----
        bool safe_fault = false;
        if (!ready || age_us > CTRL_FRESH_US) {
            safe_fault = true;
            printf("[SAFE] IMU stale (%lu us)\n", (unsigned long)age_us);
        } else if (fabsf(pitch - g_setpoint) > CTRL_TILT_MAX_DEG) {
            safe_fault = true;
            printf("[SAFE] Tilt %.1f deg exceeds limit\n", (double)pitch);
        }
        if (safe_fault) {
            portENTER_CRITICAL(&state_mux);
            s_mode = MODE_OFF;
            portEXIT_CRITICAL(&state_mux);
            set_desired(0.0f);
            brake_set(false);
            led_red();
            continue;
        }

        // ----- Acceleration law on signed wheel speed -----
        float wheel_signed = (float)cur_freq *
                             ((cur_dir == DIR_FORWARD) ? 1.0f : -1.0f);

        float err   = pitch - g_setpoint;
        float accel = g_kp * err
                    + g_kd * pitch_rate
                    - g_komega * wheel_signed;

        float new_desired = s_desired + accel * dt_ctrl;

        // Anti-windup: don't let the integrator lead the motor's actual
        // (signed) speed by more than g_antiwindup_hz. This is what keeps
        // the integrator from racing into saturation while the motor is
        // still slewing or transitioning direction.
        float aw = (float)g_antiwindup_hz;
        float upper = wheel_signed + aw;
        float lower = wheel_signed - aw;
        if (new_desired > upper) new_desired = upper;
        if (new_desired < lower) new_desired = lower;

        // Absolute clamp to motor range
        if (new_desired >  (float)FREQ_MAX) new_desired =  (float)FREQ_MAX;
        if (new_desired < -(float)FREQ_MAX) new_desired = -(float)FREQ_MAX;
        set_desired(new_desired);

        if (++print_div >= 10) {
            print_div = 0;
            const char *dname = (cur_dir == DIR_FORWARD) ? "FWD" : "REV";
            printf("BAL pitch=%6.2f rate=%7.2f accel=%8.1f des=%+7.1f dir=%s cur=%5d brk=%c\n",
                   (double)pitch, (double)pitch_rate, (double)accel,
                   (double)new_desired, dname, cur_freq,
                   s_brake_active ? '1' : '0');
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
    set_desired(0.0f);
    brake_set(false);
    led_red();
    printf("[MODE] OFF (ramping down)\n");
}

// MANUAL: positive = forward, negative = reverse. Useful for hardware tests.
static void enter_manual(int signed_freq) {
    int mag = abs(signed_freq);
    if (mag < FREQ_MIN || mag > FREQ_MAX) {
        printf("Out of range. +/-%d..%d.\n", FREQ_MIN, FREQ_MAX);
        return;
    }
    if (!s_motor_armed) motor_hw_start();
    portENTER_CRITICAL(&state_mux);
    s_mode = MODE_MANUAL;
    portEXIT_CRITICAL(&state_mux);
    set_desired((float)signed_freq);
    led_blue();
    printf("[MODE] MANUAL target=%d (%s)\n",
           mag, signed_freq >= 0 ? "FWD" : "REV");
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
    set_desired(0.0f);
    brake_set(false);
    led_green();
    printf("[MODE] BALANCE armed (Kp=%.1f Kd=%.1f Kw=%.3f setpoint=%.2f)\n",
           (double)g_kp, (double)g_kd, (double)g_komega, (double)g_setpoint);
}

static void print_gains(void) {
    printf("[GAINS] Kp=%.2f Kd=%.2f Kw=%.4f setpoint=%.3f deg\n",
           (double)g_kp, (double)g_kd, (double)g_komega, (double)g_setpoint);
    printf("[MOTOR] slew=%d Hz/s  deadband=%d Hz  flip_dwell=%d ticks (%d ms)\n",
           g_slew_hz, g_deadband_hz, g_flip_dwell, g_flip_dwell * SLEW_TICK_MS);
    printf("[SAFE]  antiwindup=%d Hz  flip_interval=%d ms\n",
           g_antiwindup_hz, g_flip_min_interval_ms);
}

static void print_status(void) {
    const char *mname = (s_mode == MODE_OFF) ? "OFF"
                      : (s_mode == MODE_MANUAL) ? "MANUAL" : "BALANCE";
    const char *dname = (s_current_dir == DIR_FORWARD) ? "FWD" : "REV";
    printf("[STATUS] mode=%s armed=%d dir=%s cur=%d des=%+.1f brk=%d pitch=%.2f\n",
           mname, (int)s_motor_armed, dname, s_current_freq,
           (double)s_desired, (int)s_brake_active, (double)s_pitch_deg);
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

    // --- F/R pin ---
    gpio_config_t fr_io = {
        .pin_bit_mask = (1ULL << FR_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&fr_io);
    gpio_set_level(FR_PIN, FR_FORWARD_LEVEL);
    s_current_dir = DIR_FORWARD;

    // --- BRAKE pin: released at boot ---
    gpio_config_t brake_io = {
        .pin_bit_mask = (1ULL << BRAKE_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&brake_io);
    gpio_set_level(BRAKE_PIN, BRAKE_RELEASED_LEVEL);
    s_brake_active = false;

    // --- LEDC timer ---
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

    printf("Cubli balancer ready (direction-flip + brake).\n");
    printf("  <number>  MANUAL freq, +ve=FWD, -ve=REV (e.g. 1500, -1500)\n");
    printf("  0         motor OFF\n");
    printf("  b         arm BALANCE mode\n");
    printf("  s         print status\n");
    printf("  g         print gains and motor knobs\n");
    printf("  p <num>   set K_P     in (Hz/s)/deg     (e.g. p 2000)\n");
    printf("  d <num>   set K_D     in (Hz/s)/(deg/s) (e.g. d 200)\n");
    printf("  w <num>   set K_OMEGA in 1/s            (e.g. w 0.3)\n");
    printf("  c <num>   set pitch setpoint in deg     (e.g. c 1.7)\n");
    printf("  r <num>   set slew rate in Hz/sec       (e.g. r 20000)\n");
    printf("  z <num>   set deadband in Hz            (e.g. z 200)\n");
    printf("  f <num>   set flip dwell in ticks       (e.g. f 3)\n");
    printf("  x <0|1>   manual brake (1=engage, 0=release)\n");
    printf("  a <num>   anti-windup lead in Hz         (e.g. a 5000)\n");
    printf("  m <num>   min ms between F/R flips       (e.g. m 150)\n");
    print_gains();

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
            } else if (buf[0] == 'g' || buf[0] == 'G') {
                print_gains();
            } else if (buf[0] == 'p' || buf[0] == 'P') {
                float v;
                if (sscanf(buf + 1, "%f", &v) == 1 && v >= 0.0f && v < 50000.0f) {
                    g_kp = v;
                    printf("K_P = %.2f\n", (double)v);
                } else {
                    printf("Usage: p <number>  (0 .. 50000)\n");
                }
            } else if (buf[0] == 'd' || buf[0] == 'D') {
                float v;
                if (sscanf(buf + 1, "%f", &v) == 1 && v >= 0.0f && v < 50000.0f) {
                    g_kd = v;
                    printf("K_D = %.2f\n", (double)v);
                } else {
                    printf("Usage: d <number>  (0 .. 50000)\n");
                }
            } else if (buf[0] == 'c' || buf[0] == 'C') {
                float v;
                if (sscanf(buf + 1, "%f", &v) == 1 && fabsf(v) <= 45.0f) {
                    g_setpoint = v;
                    printf("setpoint = %.3f deg\n", (double)v);
                } else {
                    printf("Usage: c <number>  (-45 .. 45 degrees)\n");
                }
            } else if (buf[0] == 'w' || buf[0] == 'W') {
                float v;
                if (sscanf(buf + 1, "%f", &v) == 1 && v >= 0.0f && v < 100.0f) {
                    g_komega = v;
                    printf("K_OMEGA = %.4f\n", (double)v);
                } else {
                    printf("Usage: w <number>  (0 .. 100)\n");
                }
            } else if (buf[0] == 'r' || buf[0] == 'R') {
                int v;
                if (sscanf(buf + 1, "%d", &v) == 1 && v >= 1000 && v <= 200000) {
                    g_slew_hz = v;
                    printf("slew = %d Hz/s\n", v);
                } else {
                    printf("Usage: r <number>  (1000 .. 200000 Hz/sec)\n");
                }
            } else if (buf[0] == 'z' || buf[0] == 'Z') {
                int v;
                if (sscanf(buf + 1, "%d", &v) == 1 && v >= 0 && v <= 5000) {
                    g_deadband_hz = v;
                    printf("deadband = %d Hz\n", v);
                } else {
                    printf("Usage: z <number>  (0 .. 5000 Hz)\n");
                }
            } else if (buf[0] == 'f' || buf[0] == 'F') {
                int v;
                if (sscanf(buf + 1, "%d", &v) == 1 && v >= 0 && v <= 100) {
                    g_flip_dwell = v;
                    printf("flip dwell = %d ticks (%d ms)\n", v, v * SLEW_TICK_MS);
                } else {
                    printf("Usage: f <number>  (0 .. 100 ticks)\n");
                }
            } else if (buf[0] == 'x' || buf[0] == 'X') {
                int v;
                if (sscanf(buf + 1, "%d", &v) == 1 && (v == 0 || v == 1)) {
                    brake_set(v != 0);
                    printf("brake = %s (manual override)\n",
                           v ? "ENGAGED" : "RELEASED");
                } else {
                    printf("Usage: x <0|1>\n");
                }
            } else if (buf[0] == 'a' || buf[0] == 'A') {
                int v;
                if (sscanf(buf + 1, "%d", &v) == 1 && v >= 100 && v <= 50000) {
                    g_antiwindup_hz = v;
                    printf("anti-windup = %d Hz\n", v);
                } else {
                    printf("Usage: a <number>  (100 .. 50000 Hz)\n");
                }
            } else if (buf[0] == 'm' || buf[0] == 'M') {
                int v;
                if (sscanf(buf + 1, "%d", &v) == 1 && v >= 0 && v <= 2000) {
                    g_flip_min_interval_ms = v;
                    printf("flip min interval = %d ms\n", v);
                } else {
                    printf("Usage: m <number>  (0 .. 2000 ms; 0 disables)\n");
                }
            } else if (buf[0] == '-' || isdigit((unsigned char)buf[0])) {
                int freq = atoi(buf);
                if (freq == 0) {
                    enter_off();
                } else {
                    enter_manual(freq);
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
