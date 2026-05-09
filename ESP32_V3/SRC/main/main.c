#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/gpio.h"
#include "led_strip.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#define PWM_PIN GPIO_NUM_1        // moved from GPIO_NUM_2
#define RAMP_RATE_HZ_PER_SEC 20000

static led_strip_handle_t strip;

// --- IMU globals ---
extern sh2_Hal_t *bno085_get_hal(void);
static sh2_SensorValue_t imu_event;
static volatile bool imu_event_ready = false;

static void set_led_red(void) {
    led_strip_set_pixel(strip, 0, 10, 0, 0);
    led_strip_refresh(strip);
}

static void set_led_green(void) {
    led_strip_set_pixel(strip, 0, 0, 10, 0);
    led_strip_refresh(strip);
}

void set_freq_ramped(int from_freq, int to_freq)
{
    if (from_freq == to_freq) return;

    const int ms_per_tick = 1000 / CONFIG_FREERTOS_HZ;
    const int hz_per_tick = (RAMP_RATE_HZ_PER_SEC * ms_per_tick) / 1000;
    const int step = (hz_per_tick < 1) ? 1 : hz_per_tick;
    const int dir = (from_freq < to_freq) ? 1 : -1;

    int f = from_freq;
    while (1) {
        f += dir * step;
        if (dir > 0 && f >= to_freq) f = to_freq;
        if (dir < 0 && f <= to_freq) f = to_freq;
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, f);
        if (f == to_freq) break;
        vTaskDelay(1);
    }
}

// --- IMU callback & helpers ---

static void imu_event_handler(void *cookie, sh2_SensorEvent_t *event) {
    sh2_decodeSensorEvent(&imu_event, event);
    imu_event_ready = true;
}

static void quat_to_euler(float r, float i, float j, float k,
                           float *roll, float *pitch, float *yaw) {
    *roll  = atan2f(2*(r*i + j*k), 1 - 2*(i*i + j*j)) * (180.0f / M_PI);
    float sinp = 2*(r*j - k*i);
    *pitch = (fabsf(sinp) >= 1) ? copysignf(90.0f, sinp) : asinf(sinp) * (180.0f / M_PI);
    *yaw   = atan2f(2*(r*k + i*j), 1 - 2*(j*j + k*k)) * (180.0f / M_PI);
}

static void imu_async_handler(void *cookie, sh2_AsyncEvent_t *event) {
    // we don't need async events for now
}

static void bno085_init(void) {
    printf("BNO085 init starting...\n");
    sh2_Hal_t *hal = bno085_get_hal();

    printf("Opening HAL...\n");
    int rc = sh2_open(hal, imu_async_handler, NULL);
    printf("sh2_open returned: %d\n", rc);
    if (rc != SH2_OK) {
        printf("BNO085 open failed: %d\n", rc);
        return;
    }

    printf("Setting sensor callback...\n");
    sh2_setSensorCallback(imu_event_handler, NULL);

    printf("Enabling rotation vector...\n");
    sh2_SensorConfig_t cfg = { .reportInterval_us = 10000 };
    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &cfg);
    printf("sh2_setSensorConfig returned: %d\n", rc);
    if (rc != SH2_OK) {
        printf("BNO085 sensor enable failed: %d\n", rc);
        return;
    }

    printf("BNO085 ready\n");
}

static void imu_task(void *arg) {
    bno085_init();
    while (1) {
        sh2_service();
        if (imu_event_ready) {
            imu_event_ready = false;
            if (imu_event.sensorId == SH2_ROTATION_VECTOR) {
                float roll, pitch, yaw;
                quat_to_euler(
                    imu_event.un.rotationVector.real,
                    imu_event.un.rotationVector.i,
                    imu_event.un.rotationVector.j,
                    imu_event.un.rotationVector.k,
                    &roll, &pitch, &yaw
                );
                printf("Roll: %6.1f  Pitch: %6.1f  Yaw: %6.1f\n", roll, pitch, yaw);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    // LED strip setup
    led_strip_config_t config = {
        .strip_gpio_num = 8,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_config = { .resolution_hz = 10 * 1000 * 1000 };
    led_strip_new_rmt_device(&config, &rmt_config, &strip);
    set_led_red();

    // LEDC timer setup
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 500,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    // LEDC channel setup
    ledc_channel_config_t channel = {
        .gpio_num = PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 500,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 1, NULL);

    bool motor_on = false;
    int current_freq = 500;

    printf("Enter frequency (0 = off, 500-26000 Hz) and press Enter:\n");
    printf("Ramp rate: %d Hz/sec\n", RAMP_RATE_HZ_PER_SEC);

    char buf[32];
    int pos = 0;

    while (1) {
        // --- Service IMU ---
        sh2_service();

        if (imu_event_ready) {
            imu_event_ready = false;
            if (imu_event.sensorId == SH2_ROTATION_VECTOR) {
                float roll, pitch, yaw;
                quat_to_euler(
                    imu_event.un.rotationVector.real,
                    imu_event.un.rotationVector.i,
                    imu_event.un.rotationVector.j,
                    imu_event.un.rotationVector.k,
                    &roll, &pitch, &yaw
                );
                printf("Roll: %6.1f  Pitch: %6.1f  Yaw: %6.1f\n", roll, pitch, yaw);
            }
        }

        // --- Handle serial input ---
        int c = getchar();
        if (c == EOF || c < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        printf("%c", (char)c);

        if (c == '\r' || c == '\n') {
            printf("\n");
            if (pos > 0) {
                buf[pos] = '\0';
                pos = 0;

                int freq = atoi(buf);

                if (freq == 0) {
                    if (motor_on) {
                        printf("Ramping down to stop...\n");
                        set_freq_ramped(current_freq, 500);
                        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                        motor_on = false;
                        current_freq = 500;
                        set_led_red();
                    }
                    printf("Motor off\n");

                } else if (freq >= 500 && freq <= 26000) {
                    if (!motor_on) {
                        ledc_channel_config(&channel);
                        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 500);
                        motor_on = true;
                        current_freq = 500;
                        set_led_green();
                    }
                    printf("Ramping %s from %d to %d Hz at %d Hz/sec...\n",
                           freq > current_freq ? "up" : "down",
                           current_freq, freq, RAMP_RATE_HZ_PER_SEC);
                    set_freq_ramped(current_freq, freq);
                    current_freq = freq;
                    printf("Done. Running at %d Hz\n", current_freq);

                } else {
                    printf("Out of range. Enter 0 or 500-26000.\n");
                }
            }
        } else if (pos < (int)sizeof(buf) - 1) {
            buf[pos++] = (char)c;
        }
    }
}