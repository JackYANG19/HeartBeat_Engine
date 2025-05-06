idf_component_register(SRCS "led_gpio.c" "led_pwm.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver)
