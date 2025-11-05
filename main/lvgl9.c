#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_lcd_touch_gt911.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>

#include "sdkconfig.h"
#include "lvgl.h"
#include "hardware.h"

#define TAG "lvgl9"

extern void example_lvgl_demo_ui(lv_obj_t *scr);

/*
* touch related
*/

static i2c_master_bus_handle_t i2c_bus = NULL;

esp_err_t gt911_init_i2c(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = TOUCH_PIN_SCL,
        .sda_io_num = TOUCH_PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_LOGI(TAG, "Initializing I2C");
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));
    return ESP_OK;
}

uint16_t gt911_map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) { return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

void gt911_process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    *x = gt911_map(*x, TOUCH_H_RES_MIN, TOUCH_H_RES_MAX, 0, LCD_H_RES);
    *y = gt911_map(*y, TOUCH_V_RES_MIN, TOUCH_V_RES_MAX, 0, LCD_V_RES);

    ESP_LOGI(TAG, "Touch X: %d Y: %d\n", *x, *y);
}

void gt911_touch_init(esp_lcd_touch_handle_t *tp)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    const esp_lcd_panel_io_i2c_config_t tp_io_config = { 
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
        .scl_speed_hz = TOUCH_FREQ_HZ,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 16,
        .lcd_param_bits = 0,
        .flags = {
            .dc_low_on_data = 0,
            .disable_control_phase = 1,
        } };

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TOUCH_PIN_RESET,
        .int_gpio_num = TOUCH_PIN_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .process_coordinates = gt911_process_coordinates,
        .interrupt_callback = NULL
        };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, tp));
}

static void gt911_touchpad_read(lv_indev_t *indev_drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)lv_indev_get_user_data(indev_drv);
    assert(tp);

    uint16_t touchpad_x;
    uint16_t touchpad_y;
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_read_data(tp);

    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, &touchpad_x, &touchpad_y, NULL, &touchpad_cnt, 1);
    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x;
        data->point.y = touchpad_y;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}


/*
* LVGL related functions
*/
static void lcd_lvgl_flush_cb(lv_display_t *drv, const lv_area_t *area, unsigned char *color_map)
{
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(drv);

  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;

  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

  lv_disp_flush_ready(drv);
}

static void tick(void *arg)
{ 
    lv_tick_inc(2); 
}

static void touch_event(lv_event_t *e)
{
uint8_t n = rand()%0xff;

  if (lv_event_get_code(e) == LV_EVENT_CLICKED)
  {
    lv_obj_t *label = (lv_obj_t*)lv_event_get_user_data(e);
    lv_obj_set_style_text_color(label, lv_color_make(n, n, n), 0);
  }
}

void lcd_init(void *)
{
  static lv_display_t *disp;
  static lv_indev_t *indev_drv_tp;
  static esp_lcd_touch_handle_t tp;
  static esp_lcd_panel_handle_t panel_handle = NULL;

  gpio_config_t bk_gpio_config = {.pin_bit_mask = 1ULL << LCD_PIN_BK_LIGHT,
                                  .mode = GPIO_MODE_OUTPUT,
                                  .pull_up_en = GPIO_PULLUP_DISABLE,
                                  .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                  .intr_type = GPIO_INTR_DISABLE};

  esp_lcd_rgb_panel_config_t panel_config = {
      .clk_src = LCD_CLK_SRC_DEFAULT,
      .timings = {.pclk_hz = LCD_PIXEL_CLOCK_HZ,
                  .h_res = LCD_H_RES,
                  .v_res = LCD_V_RES,

                  .hsync_pulse_width = 4,
                  .hsync_back_porch = 8,
                  .hsync_front_porch = 8,
                  .vsync_pulse_width = 4,
                  .vsync_back_porch = 8,
                  .vsync_front_porch = 8,
                  .flags = {.hsync_idle_low = false,
                            .vsync_idle_low = false,
                            .de_idle_high = false,
                            .pclk_active_neg = true,
                            .pclk_idle_high = false}},
      .data_width = 16,
      .bits_per_pixel = 0,
      .num_fbs = 2,
      .bounce_buffer_size_px = 0,
      .sram_trans_align = 0,
      .psram_trans_align = 64,

      .hsync_gpio_num = LCD_PIN_HSYNC,
      .vsync_gpio_num = LCD_PIN_VSYNC,
      .de_gpio_num    = LCD_PIN_DE,
      .pclk_gpio_num  = LCD_PIN_PCLK,
      .disp_gpio_num  = LCD_PIN_DISP_EN,
      .data_gpio_nums = {LCD_PIN_DATA0, LCD_PIN_DATA1, LCD_PIN_DATA2,
                         LCD_PIN_DATA3, LCD_PIN_DATA4, LCD_PIN_DATA5,
                         LCD_PIN_DATA6, LCD_PIN_DATA7, LCD_PIN_DATA8,
                         LCD_PIN_DATA9, LCD_PIN_DATA10, LCD_PIN_DATA11,
                         LCD_PIN_DATA12, LCD_PIN_DATA13, LCD_PIN_DATA14,
                         LCD_PIN_DATA15},
      .flags = {.disp_active_low = 0,
                .refresh_on_demand = 0,
                .fb_in_psram = true,
                .double_fb = true,
                .no_fb = 0,
                .bb_invalidate_cache = 0}};


  //
  // Init/setup lcd hardware and lvgl
  //

  ESP_LOGI(TAG, "Turning LCD backlight off");
  ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

  ESP_LOGI(TAG, "Initializing LCD panel");
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  ESP_LOGI(TAG, "Initialize LVGL library");
  lv_init();

  ESP_LOGI(TAG, "Allocating LVGL buffers from PSRAM");
  void *buf1 = heap_caps_malloc(LCD_H_RES * LCD_V_RES / 10, MALLOC_CAP_SPIRAM);
  assert(buf1);
  void *buf2 = heap_caps_malloc(LCD_H_RES * LCD_V_RES / 10, MALLOC_CAP_SPIRAM);
  assert(buf2);

  ESP_LOGI(TAG, "Register buffers and display callback with LVGL");
  disp = lv_display_create(LCD_H_RES, LCD_V_RES);
  lv_display_set_buffers(disp, buf1, buf2, LCD_H_RES * LCD_V_RES / 10, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_user_data(disp, panel_handle);
  lv_display_set_flush_cb(disp, lcd_lvgl_flush_cb);


  //
  // GT911 init and register as input device with lvgl:
  //
  ESP_ERROR_CHECK(gt911_init_i2c());
  gt911_touch_init(&tp);

  indev_drv_tp = lv_indev_create();
  lv_indev_set_type(indev_drv_tp, LV_INDEV_TYPE_POINTER);
  lv_indev_set_user_data(indev_drv_tp, tp);
  lv_indev_set_read_cb(indev_drv_tp, gt911_touchpad_read);
  lv_indev_enable(indev_drv_tp, true);
  lv_indev_set_display(indev_drv_tp, disp);


  //
  // start lvgl
  //
  ESP_LOGI(TAG, "Creating timer for lvgl-ticks");
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &tick,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "lvgl_tick",
      .skip_unhandled_events = true};

  esp_timer_handle_t lvgl_tick_timer = NULL;

  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2000));

  ESP_LOGI(TAG, "Turning on LCD backlight");
  gpio_set_level(LCD_PIN_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

  ESP_LOGI(TAG, "Printing to lvgl-label");
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_color(label, lv_color_make(0xff, 0x00, 0x00), 0);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
  lv_label_set_text_static(label, "LVLG9 test");

  lv_obj_add_event_cb(lv_scr_act(), touch_event, LV_EVENT_CLICKED, label);


  uint8_t n = 0;
  char buf[32];

  example_lvgl_demo_ui(lv_scr_act());

  ESP_LOGI(TAG, "Starting lvgl update loop");
  while (1)
  {
    sprintf(buf, "LVGL9: %04d", n++);
    lv_label_set_text_static(label, buf);

    vTaskDelay(20 / portTICK_PERIOD_MS);
    lv_timer_handler();
  }
}

void app_main(void) {

  // start LVGL related calls on core #1.
  xTaskCreatePinnedToCore(lcd_init, "lcd_init", 8192, NULL, 1, NULL, 1);

  vTaskDelay(portMAX_DELAY);
}
