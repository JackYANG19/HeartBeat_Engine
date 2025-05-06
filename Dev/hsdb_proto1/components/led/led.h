#ifndef LED_H
#define LED_H
#define RED_GPIO    3
#define GREEN_GPIO 10
#define BLUE_GPIO   0

// GPIO-based control
void led_gpio_init(void);
void led_cycle_gpio(void);

// PWM-based control
void led_pwm_init(void);
void rainbow_cycle(void);
void led_turn_red_on(void);

#endif