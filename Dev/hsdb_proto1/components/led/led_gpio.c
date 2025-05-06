#include "led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void led_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RED_GPIO) | (1ULL << GREEN_GPIO) | (1ULL << BLUE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void led_cycle_gpio(void) {
    gpio_set_level(RED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(RED_GPIO, 0);

    gpio_set_level(GREEN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(GREEN_GPIO, 0);

    gpio_set_level(BLUE_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(BLUE_GPIO, 0);
}
