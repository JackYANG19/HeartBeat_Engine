#include <math.h>
#include "driver/ledc.h"
#include "led.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_10_BIT  
#define LEDC_FREQUENCY      5000              // 5 kHz PWM

#define LEDC_CHANNEL_RED    LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN  LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE   LEDC_CHANNEL_2

#define LEDC_MAX_DUTY    1023
#define MAX_BRIGHTNESS   32  // adjust overall brightness (range: 0–1023)
#define R_GAIN           1.0
#define G_GAIN           0.9
#define B_GAIN           0.7

#define COMMON_ANODE_DUTY(duty) (LEDC_MAX_DUTY - (duty))  // invert for common-anode


void led_pwm_init(void)
{
    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure PWM channels for RGB
    ledc_channel_config_t channels[3] = {
        {
            .channel    = LEDC_CHANNEL_RED,
            .duty       = LEDC_MAX_DUTY,
            .gpio_num   = RED_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_GREEN,
            .duty       = LEDC_MAX_DUTY,
            .gpio_num   = GREEN_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_BLUE,
            .duty       = LEDC_MAX_DUTY,
            .gpio_num   = BLUE_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        }
    };

    for (int i = 0; i < 3; i++) {
        ledc_channel_config(&channels[i]);
    }
}


void rainbow_cycle(void)
{
    for (int i = 0; i < 360; i++)  // 1° hue steps = smooth color transition
    {
        // 🌟 Brightness envelope: sine wave (0 → MAX → 0)
        float brightness = (sinf(i * M_PI / 180.0f) + 1.0f) / 2.0f;  // range: 0.0–1.0
        float scaled_brightness = brightness * MAX_BRIGHTNESS;
        if (scaled_brightness < 8) scaled_brightness = 8;

        // 🌈 HSV to RGB (hue-based color blending)
        float hue = i % 360;
        float r = 0, g = 0, b = 0;

        int h_i = (int)(hue / 60.0f) % 6;
        float f = (hue / 60.0f) - h_i;
        float q = 1.0f - f;

        switch (h_i) {
            case 0: r = 1; g = f; b = 0; break;
            case 1: r = q; g = 1; b = 0; break;
            case 2: r = 0; g = 1; b = f; break;
            case 3: r = 0; g = q; b = 1; break;
            case 4: r = f; g = 0; b = 1; break;
            case 5: r = 1; g = 0; b = q; break;
        }

        // Apply brightness and gain, then invert for common-anode
        uint32_t duty_r = COMMON_ANODE_DUTY((uint32_t)(scaled_brightness * r * R_GAIN));
        uint32_t duty_g = COMMON_ANODE_DUTY((uint32_t)(scaled_brightness * g * G_GAIN));
        uint32_t duty_b = COMMON_ANODE_DUTY((uint32_t)(scaled_brightness * b * B_GAIN));

        // Update LEDC duties
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_RED,   duty_r);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_GREEN, duty_g);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_BLUE,  duty_b);

        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_RED);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_GREEN);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_BLUE);

        vTaskDelay(pdMS_TO_TICKS(10));  // controls transition speed
    }
}

void led_turn_red_on(void)
{
    uint32_t brightness = 12;  // brightness

    // Red ON (low), Green & Blue OFF (high)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_RED,   COMMON_ANODE_DUTY(brightness));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_GREEN, COMMON_ANODE_DUTY(0));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_BLUE,  COMMON_ANODE_DUTY(0));

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_RED);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_GREEN);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_BLUE);
}