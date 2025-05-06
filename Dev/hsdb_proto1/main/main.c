#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led.h"
#include "imu.h"
#include "i2c_utils.h"
#include "haptic.h"

#define I2C_PORT I2C_NUM_0
#define SDA_PIN  GPIO_NUM_4
#define SCL_PIN  GPIO_NUM_5
#define I2C_FREQ 100000

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    // Choose only one at a time

    // Mode 1: GPIO on/off
    // led_gpio_init();
    // while (1) led_cycle_gpio();

    // Mode 2: PWM rainbow
    // led_pwm_init();
    // while (1) rainbow_cycle();
    // led_turn_red_on();    // turns on red LED fully
    i2c_master_init(I2C_PORT, SDA_PIN, SCL_PIN, I2C_FREQ);
    i2c_scan(I2C_PORT);

    // imu_init();
    haptic_init();

    // Example: play effect #? every second
    while (1) {
        // haptic_play_effect(1); //Strong Click – 100%
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // haptic_play_effect(4); //Sharp Click – 100%
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // haptic_play_effect(7); //Soft Bump – 100%
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // haptic_play_effect(10); //Double Click – 100%
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // haptic_play_effect(14); //Buzz – 100%
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // haptic_play_effect(3); //Strong Click – 30%
        // vTaskDelay(pdMS_TO_TICKS(1000));
        haptic_play_effect(6); //Sharp Click – 30%
        vTaskDelay(pdMS_TO_TICKS(1000));
        // haptic_play_effect(9); //Soft Bump – 30%
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // haptic_play_effect(11); //Double Click – 60%
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // haptic_play_effect(13); //Buzz – 60%
        // vTaskDelay(pdMS_TO_TICKS(1000));

        // haptic_play_custom_weak_click();
        // vTaskDelay(pdMS_TO_TICKS(1000));

    }
    
}