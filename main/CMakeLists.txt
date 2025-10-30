idf_component_register(SRCS lvgl9.c esp_logo.c esp_text.c lvgl_demo_ui.c
                    INCLUDE_DIRS .
                    REQUIRES lvgl esp_lcd driver esp_timer)
