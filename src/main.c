/**
 * @file main_simple.c
 * @brief Simple LVGL demo for Air101-LCD based on successful ESP-IDF examples
 * 
 * This implementation follows the most successful patterns found in working
 * ESP-IDF + LVGL projects, focusing on simplicity and reliability.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// LVGL includes
#include "lvgl.h"

static const char* TAG = "LVGL_DEMO";

// Hardware pin definitions
#define TFT_SCLK 2   
#define TFT_MOSI 3   
#define TFT_RST 10   
#define TFT_DC 6     
#define TFT_CS 7     

// Joystick pin definitions (Air101-LCD digital buttons)
#define BUTTON_LEFT 8      // Left button
#define BUTTON_UP 9        // Up button  
#define BUTTON_CENTER 4    // Center button (press)
#define BUTTON_DOWN 5      // Down button
#define BUTTON_RIGHT 13    // Right button

// Display dimensions
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 160

// SPI and display handles
static spi_device_handle_t spi_device;
static lv_display_t *display;

// LVGL display buffer
static lv_color_t *disp_buf;

// LVGL input device
static lv_indev_t *indev;

// Joystick state
static bool joystick_pressed = false;
static int16_t joystick_x = 0;
static int16_t joystick_y = 0;

// Forward declarations
void spi_init(void);
void display_init(void);
void lvgl_flush_cb(lv_display_t *disp_drv, const lv_area_t *area, uint8_t *px_map);
void create_demo_widgets(void);
static void lvgl_tick_task(void *arg);
static void lvgl_tick_callback(void *arg);
void joystick_init(void);
void joystick_read(lv_indev_t *indev_drv, lv_indev_data_t *data);
static void joystick_task(void *arg);
static void button_click_handler(lv_event_t *e);

/**
 * @brief Initialize SPI for display communication
 */
void spi_init(void) {
    ESP_LOGI(TAG, "Initializing SPI");

    // Configure control pins
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << TFT_RST) | (1ULL << TFT_DC),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_cfg);

    // SPI bus configuration
    spi_bus_config_t bus_cfg = {
        .miso_io_num = -1,
        .mosi_io_num = TFT_MOSI,
        .sclk_io_num = TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // SPI device configuration
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_20M,
        .mode = 0,
        .spics_io_num = TFT_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device));
    ESP_LOGI(TAG, "SPI initialized successfully");
}

/**
 * @brief Send command to display
 */
void send_cmd(uint8_t cmd) {
    gpio_set_level(TFT_DC, 0); // Command mode
    spi_transaction_t trans = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = cmd;
    spi_device_polling_transmit(spi_device, &trans);
}

/**
 * @brief Send data to display
 */
void send_data(uint8_t data) {
    gpio_set_level(TFT_DC, 1); // Data mode
    spi_transaction_t trans = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = data;
    spi_device_polling_transmit(spi_device, &trans);
}

/**
 * @brief Initialize ST7735 display
 */
void display_init(void) {
    ESP_LOGI(TAG, "Initializing ST7735 display");

    // Reset display
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Basic ST7735 initialization sequence
    send_cmd(0x01); // Software reset
    vTaskDelay(pdMS_TO_TICKS(150));
    
    send_cmd(0x11); // Sleep out
    vTaskDelay(pdMS_TO_TICKS(500));
    
    send_cmd(0x3A); // Color mode
    send_data(0x05); // 16-bit color
    
    send_cmd(0x36); // Memory access control
    send_data(0xC8); // Row/col addr, bottom to top refresh
    
    send_cmd(0x29); // Display on
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Display initialized successfully");
}

/**
 * @brief Set display area for drawing
 */
void set_addr_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    send_cmd(0x2A); // Column address set
    send_data(0x00);
    send_data(x1 + 2); // Start col (with offset)
    send_data(0x00);
    send_data(x2 + 2); // End col

    send_cmd(0x2B); // Row address set  
    send_data(0x00);
    send_data(y1 + 1); // Start row (with offset)
    send_data(0x00);
    send_data(y2 + 1); // End row

    send_cmd(0x2C); // Memory write
}

/**
 * @brief LVGL display flush callback
 */
void lvgl_flush_cb(lv_display_t *disp_drv, const lv_area_t *area, uint8_t *px_map) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    
    set_addr_window(area->x1, area->y1, area->x2, area->y2);
    
    gpio_set_level(TFT_DC, 1); // Data mode
    
    // Send pixel data
    uint16_t *color_p = (uint16_t *)px_map;
    for (uint32_t i = 0; i < w * h; i++) {
        uint16_t color = color_p[i];
        // Send high byte then low byte
        spi_transaction_t trans = {
            .length = 16,
            .flags = SPI_TRANS_USE_TXDATA,
        };
        trans.tx_data[0] = color >> 8;
        trans.tx_data[1] = color & 0xFF;
        spi_device_polling_transmit(spi_device, &trans);
    }
    
    lv_display_flush_ready(disp_drv);
}

/**
 * @brief Create demo widgets
 */
void create_demo_widgets(void) {
    ESP_LOGI(TAG, "Creating demo widgets");
    
    // Get active screen
    lv_obj_t *scr = lv_screen_active();
    
    // Set background color
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x003a57), LV_PART_MAIN);
    
    // Create title label
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Air101-LCD\nLVGL Demo");
    lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // Create a simple button
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 70, 40);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Press Me");
    lv_obj_center(btn_label);
    
    // Add click event handler
    lv_obj_add_event_cb(btn, button_click_handler, LV_EVENT_CLICKED, NULL);
    
    // Create a progress bar
    lv_obj_t *bar = lv_bar_create(scr);
    lv_obj_set_size(bar, 70, 10);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_bar_set_value(bar, 70, LV_ANIM_OFF);
    
    // Create status text
    lv_obj_t *status = lv_label_create(scr);
    lv_label_set_text(status, "System Ready");
    lv_obj_set_style_text_color(status, lv_color_hex(0x00ff00), LV_PART_MAIN);
    lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    ESP_LOGI(TAG, "Demo widgets created");
}

/**
 * @brief Button click event handler
 */
static void button_click_handler(lv_event_t *e) {
    static int click_count = 0;
    click_count++;
    
    ESP_LOGI(TAG, "🎉 Button clicked! Count: %d", click_count);
    
    // Get references to widgets (simple approach for demo)
    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_t *scr = lv_screen_active();
    
    // Change button text based on click count
    lv_obj_t *btn_label = lv_obj_get_child(btn, 0);
    switch (click_count % 4) {
        case 1:
            lv_label_set_text(btn_label, "Clicked!");
            lv_obj_set_style_bg_color(scr, lv_color_hex(0x4a0e4e), LV_PART_MAIN); // Purple
            break;
        case 2:
            lv_label_set_text(btn_label, "Again!");
            lv_obj_set_style_bg_color(scr, lv_color_hex(0x0e4b4a), LV_PART_MAIN); // Teal
            break;
        case 3:
            lv_label_set_text(btn_label, "More!");
            lv_obj_set_style_bg_color(scr, lv_color_hex(0x4a2c0e), LV_PART_MAIN); // Brown
            break;
        case 0:
            lv_label_set_text(btn_label, "Press Me");
            lv_obj_set_style_bg_color(scr, lv_color_hex(0x003a57), LV_PART_MAIN); // Original blue
            break;
    }
    
    // Update progress bar
    lv_obj_t *bar = NULL;
    // Find the progress bar (it's the 4th child: title, button, bar, status)
    for(int i = 0; i < lv_obj_get_child_count(scr); i++) {
        lv_obj_t *child = lv_obj_get_child(scr, i);
        if(lv_obj_check_type(child, &lv_bar_class)) {
            bar = child;
            break;
        }
    }
    
    if(bar) {
        int new_value = (click_count * 20) % 101; // Cycle 0-100
        lv_bar_set_value(bar, new_value, LV_ANIM_ON);
        ESP_LOGI(TAG, "Progress updated to %d%%", new_value);
    }
}

/**
 * @brief Initialize joystick (digital GPIO buttons)
 */
void joystick_init(void) {
    ESP_LOGI(TAG, "Initializing joystick buttons");
    
    // Configure all joystick buttons as input with pull-up
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_LEFT) | (1ULL << BUTTON_UP) | 
                       (1ULL << BUTTON_CENTER) | (1ULL << BUTTON_DOWN) | 
                       (1ULL << BUTTON_RIGHT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);
    
    ESP_LOGI(TAG, "Joystick buttons initialized (L:%d U:%d C:%d D:%d R:%d)", 
             BUTTON_LEFT, BUTTON_UP, BUTTON_CENTER, BUTTON_DOWN, BUTTON_RIGHT);
}

/**
 * @brief Read joystick state and provide to LVGL
 */
void joystick_read(lv_indev_t *indev_drv, lv_indev_data_t *data) {
    // Read digital button states (active LOW with pull-up)
    bool left_pressed = !gpio_get_level(BUTTON_LEFT);
    bool up_pressed = !gpio_get_level(BUTTON_UP);
    bool center_pressed = !gpio_get_level(BUTTON_CENTER);
    bool down_pressed = !gpio_get_level(BUTTON_DOWN);
    bool right_pressed = !gpio_get_level(BUTTON_RIGHT);
    
    // Static cursor position
    static int16_t cursor_x = 50; // Center of 100px screen
    static int16_t cursor_y = 80; // Center of 160px screen
    static uint32_t last_move_time = 0;
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool can_move = (current_time - last_move_time) > 100; // 100ms movement delay
    
    // Update cursor position based on button presses
    if (can_move) {
        bool moved = false;
        
        if (left_pressed) {
            cursor_x -= 4;
            if (cursor_x < 0) cursor_x = 0;
            moved = true;
        }
        else if (right_pressed) {
            cursor_x += 4;
            if (cursor_x >= DISPLAY_WIDTH) cursor_x = DISPLAY_WIDTH - 1;
            moved = true;
        }
        
        if (up_pressed) {
            cursor_y -= 4;
            if (cursor_y < 0) cursor_y = 0;
            moved = true;
        }
        else if (down_pressed) {
            cursor_y += 4;
            if (cursor_y >= DISPLAY_HEIGHT) cursor_y = DISPLAY_HEIGHT - 1;
            moved = true;
        }
        
        if (moved) {
            last_move_time = current_time;
        }
    }
    
    // Update LVGL input data
    data->point.x = cursor_x;
    data->point.y = cursor_y;
    data->state = center_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    
    // Store for debugging
    joystick_x = cursor_x;
    joystick_y = cursor_y;
    joystick_pressed = center_pressed;
}

/**
 * @brief Joystick monitoring task for debug output
 */
static void joystick_task(void *arg) {
    while (1) {
        // Read button states for debugging
        bool left_pressed = !gpio_get_level(BUTTON_LEFT);
        bool up_pressed = !gpio_get_level(BUTTON_UP);
        bool center_pressed = !gpio_get_level(BUTTON_CENTER);
        bool down_pressed = !gpio_get_level(BUTTON_DOWN);
        bool right_pressed = !gpio_get_level(BUTTON_RIGHT);
        
        // Log button states if any button is pressed
        if (left_pressed || up_pressed || center_pressed || down_pressed || right_pressed) {
            ESP_LOGI(TAG, "Buttons: L:%d U:%d C:%d D:%d R:%d | Cursor: X=%d Y=%d", 
                     left_pressed, up_pressed, center_pressed, down_pressed, right_pressed,
                     joystick_x, joystick_y);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // Check every 200ms
    }
}

/**
 * @brief LVGL tick callback - increments LVGL internal tick counter
 */
static void lvgl_tick_callback(void *arg) {
    // Called every 1ms by esp_timer
    lv_tick_inc(1);
}

/**
 * @brief LVGL task to handle GUI updates
 */
static void lvgl_tick_task(void *arg) {
    while (1) {
        // Let higher priority tasks run first
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // Handle LVGL timers with timeout to prevent blocking
        uint32_t task_delay_ms = lv_timer_handler();
        
        // Yield CPU to other tasks
        if (task_delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

/**
 * @brief Main application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "Starting Air101-LCD LVGL Demo");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    // Initialize hardware
    spi_init();
    display_init();
    joystick_init();

    // Initialize LVGL
    ESP_LOGI(TAG, "Initializing LVGL");
    lv_init();
    
    // Create an ESP timer for LVGL tick
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_callback,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 1000)); // 1ms (1000us) period

    // Allocate display buffer
    size_t buf_size = DISPLAY_WIDTH * 40; // 40 lines buffer
    disp_buf = heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!disp_buf) {
        ESP_LOGE(TAG, "Failed to allocate display buffer");
        return;
    }
    ESP_LOGI(TAG, "Display buffer allocated: %zu bytes", buf_size * sizeof(lv_color_t));

    // Create display object
    display = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    if (!display) {
        ESP_LOGE(TAG, "Failed to create display");
        return;
    }

    // Configure display
    lv_display_set_buffers(display, disp_buf, NULL, buf_size * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    // Create and configure input device (joystick)
    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, joystick_read);

    ESP_LOGI(TAG, "LVGL display and input configured");

    // Create demo UI
    create_demo_widgets();

    // Start LVGL task with lower priority to allow other tasks to run
    xTaskCreate(lvgl_tick_task, "lvgl_tick", 4096, NULL, 2, NULL);
    
    // Start joystick monitoring task
    xTaskCreate(joystick_task, "joystick", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "LVGL demo started successfully!");

    // Main loop - monitor system
    while (1) {
        ESP_LOGI(TAG, "System running - Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds
    }
}
