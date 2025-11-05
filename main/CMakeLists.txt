idf_component_register(SRCS lvgl9.c esp_logo.c esp_text.c lvgl_demo_ui.c
                    INCLUDE_DIRS .
                    PRIV_REQUIRES 
                        lvgl
                        driver
                        esp_lcd
                        esp_timer
                        esp_driver_gpio
                        esp_driver_i2c
)
