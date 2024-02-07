# esp32-8048S043-lvgl9

Sunton ESP32-S3 800x480 Capacitive touch display


Basic example using esp-idf 5.1, with [esp_lcd_touch_gt911](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911) and [lvgl 9.x](https://components.espressif.com/components/lvgl/lvgl) from the component registry.


Using an adapted version of [lvgl_demo_ui.c](https://github.com/espressif/esp-idf/blob/master/examples/peripherals/lcd/i80_controller/main/lvgl_demo_ui.c) and processing the original [images](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/lcd/i80_controller/main/images) with [LVGLImage.py](https://github.com/lvgl/lvgl/blob/master/scripts/LVGLImage.py) because of changes in lvgl.

```
idf.py build flash monitor
```

The lv_conf.h enables the lvgl performance monitor, shown in the bottom-right corner.
