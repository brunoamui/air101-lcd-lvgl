/**
 * @file main_lvgl.c
 * @brief Main application with LVGL integration for Air101-LCD
 * 
 * This application demonstrates LVGL graphics library integration with
 * the Air101-LCD (ST7735) display and joystick input on ESP32-C3.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

// LVGL includes
#include "lvgl.h"

// Custom driver includes
#include "st7735_lvgl_driver.h"
#include "joystick_lvgl_driver.h"

static const char* TAG = "Air101-LVGL";

// LVGL objects
static lv_display_t *display;
static lv_indev_t *joystick;

// Task handles
static TaskHandle_t lvgl_task_handle = NULL;

/**
 * @brief LVGL timer task
 * 
 * This task calls lv_timer_handler() periodically to process LVGL timers,
 * animations, and other periodic tasks. It runs every 5ms as recommended
 * by LVGL documentation.
 */
static void lvgl_timer_task(void *param) {
    ESP_LOGI(TAG, "LVGL timer task started");
    
    while (1) {
        // Process LVGL timers and animations
        uint32_t task_delay_ms = lv_timer_handler();
        
        // Ensure minimum delay
        if (task_delay_ms > 100) {
            task_delay_ms = 100;
        } else if (task_delay_ms < 5) {
            task_delay_ms = 5;
        }
        
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
    
    vTaskDelete(NULL);
}

/**
 * @brief Create a simple LVGL demo UI
 * 
 * Creates a basic user interface with labels, buttons, and other widgets
 * to demonstrate LVGL functionality.
 */
static void create_demo_ui(void) {
    ESP_LOGI(TAG, "Creating demo UI");
    
    // Get the active screen
    lv_obj_t *scr = lv_screen_active();
    
    // Set background color
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x003a57), LV_PART_MAIN);
    
    // Create a title label
    lv_obj_t *title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "Air101-LCD\nLVGL Demo");
    lv_obj_set_style_text_color(title_label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 10);
    
    // Create a container for buttons
    lv_obj_t *button_cont = lv_obj_create(scr);
    lv_obj_set_size(button_cont, 110, 100);
    lv_obj_align(button_cont, LV_ALIGN_CENTER, 0, 10);
    lv_obj_set_style_bg_color(button_cont, lv_color_hex(0x444444), LV_PART_MAIN);
    lv_obj_set_style_border_color(button_cont, lv_color_hex(0x666666), LV_PART_MAIN);
    lv_obj_set_style_border_width(button_cont, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(button_cont, 8, LV_PART_MAIN);
    
    // Create buttons
    lv_obj_t *btn1 = lv_btn_create(button_cont);
    lv_obj_set_size(btn1, 90, 30);
    lv_obj_align(btn1, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_t *btn1_label = lv_label_create(btn1);
    lv_label_set_text(btn1_label, "Button 1");
    lv_obj_center(btn1_label);
    
    lv_obj_t *btn2 = lv_btn_create(button_cont);
    lv_obj_set_size(btn2, 90, 30);
    lv_obj_align(btn2, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t *btn2_label = lv_label_create(btn2);
    lv_label_set_text(btn2_label, "Button 2");
    lv_obj_center(btn2_label);
    
    // Create status label
    lv_obj_t *status_label = lv_label_create(scr);
    lv_label_set_text(status_label, "Use joystick to navigate");
    lv_obj_set_style_text_color(status_label, lv_color_hex(0xcccccc), LV_PART_MAIN);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    // Create a simple arc widget
    lv_obj_t *arc = lv_arc_create(scr);
    lv_obj_set_size(arc, 60, 60);
    lv_obj_align(arc, LV_ALIGN_BOTTOM_LEFT, 10, -40);
    lv_arc_set_range(arc, 0, 100);
    lv_arc_set_value(arc, 75);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x00ff00), LV_PART_INDICATOR);
    
    // Create a simple bar widget
    lv_obj_t *bar = lv_bar_create(scr);
    lv_obj_set_size(bar, 80, 15);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_RIGHT, -10, -50);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, 60, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xff6600), LV_PART_INDICATOR);
    
    ESP_LOGI(TAG, "Demo UI created successfully");
}

/**
 * @brief Initialize LVGL with custom drivers
 * 
 * Sets up LVGL with the ST7735 display driver and joystick input driver.
 */
static esp_err_t init_lvgl(void) {
    ESP_LOGI(TAG, "Initializing LVGL");
    
    // Initialize LVGL
    lv_init();
    
    // Initialize ST7735 hardware
    esp_err_t ret = st7735_init_hardware();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ST7735 hardware");
        return ret;
    }
    
    // Initialize LVGL display
    display = st7735_lvgl_init();
    if (!display) {
        ESP_LOGE(TAG, "Failed to initialize LVGL display");
        return ESP_FAIL;
    }
    
    // Initialize joystick hardware
    ret = joystick_init_hardware();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize joystick hardware");
        return ret;
    }
    
    // Initialize LVGL input device
    joystick = joystick_lvgl_init();
    if (!joystick) {
        ESP_LOGE(TAG, "Failed to initialize LVGL input device");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "LVGL initialization complete");
    return ESP_OK;
}

/**
 * @brief Button event callback
 * 
 * Handles button press events and provides feedback.
 */
static void btn_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    
    if (code == LV_EVENT_CLICKED) {
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        const char * btn_text = lv_label_get_text(label);
        ESP_LOGI(TAG, "%s clicked!", btn_text);
        
        // Provide visual feedback
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00aa00), LV_PART_MAIN);
        
        // Reset color after 200ms
        // Note: For now, just keep the button green (TODO: implement timer callback)
        // lv_timer_t * timer = lv_timer_create(timer_callback, 200, btn);
    }
}

/**
 * @brief Enhanced demo UI with interactivity
 * 
 * Creates an enhanced UI with interactive elements.
 */
static void create_enhanced_ui(void) {
    ESP_LOGI(TAG, "Creating enhanced interactive UI");
    
    // Get the active screen
    lv_obj_t *scr = lv_screen_active();
    
    // Clear the screen first
    lv_obj_clean(scr);
    
    // Set background gradient
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x001122), LV_PART_MAIN);
    
    // Create main container
    lv_obj_t *main_cont = lv_obj_create(scr);
    lv_obj_set_size(main_cont, 120, 150);
    lv_obj_center(main_cont);
    lv_obj_set_style_bg_color(main_cont, lv_color_hex(0x223344), LV_PART_MAIN);
    lv_obj_set_style_border_color(main_cont, lv_color_hex(0x445566), LV_PART_MAIN);
    lv_obj_set_style_border_width(main_cont, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(main_cont, 10, LV_PART_MAIN);
    
    // Title
    lv_obj_t *title = lv_label_create(main_cont);
    lv_label_set_text(title, "LVGL Demo");
    lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
    
    // Interactive buttons with callbacks
    lv_obj_t *btn1 = lv_btn_create(main_cont);
    lv_obj_set_size(btn1, 80, 25);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -20);
    lv_obj_t *btn1_label = lv_label_create(btn1);
    lv_label_set_text(btn1_label, "Demo 1");
    lv_obj_center(btn1_label);
    lv_obj_add_event_cb(btn1, btn_event_cb, LV_EVENT_ALL, NULL);
    
    lv_obj_t *btn2 = lv_btn_create(main_cont);
    lv_obj_set_size(btn2, 80, 25);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 10);
    lv_obj_t *btn2_label = lv_label_create(btn2);
    lv_label_set_text(btn2_label, "Demo 2");
    lv_obj_center(btn2_label);
    lv_obj_add_event_cb(btn2, btn_event_cb, LV_EVENT_ALL, NULL);
    
    // Status indicator
    lv_obj_t *led = lv_led_create(main_cont);
    lv_obj_set_size(led, 15, 15);
    lv_obj_align(led, LV_ALIGN_TOP_RIGHT, -10, 8);
    lv_led_set_color(led, lv_color_hex(0x00ff00));
    lv_led_on(led);
    
    // Progress bar
    lv_obj_t *bar = lv_bar_create(main_cont);
    lv_obj_set_size(bar, 90, 8);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, 85, LV_ANIM_ON);
    
    // Instructions
    lv_obj_t *instr = lv_label_create(scr);
    lv_label_set_text(instr, "Navigate: Joystick\\nSelect: Center");
    lv_obj_set_style_text_color(instr, lv_color_hex(0xaaaaaa), LV_PART_MAIN);
    lv_obj_set_style_text_align(instr, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(instr, LV_ALIGN_BOTTOM_MID, 0, -5);
    
    ESP_LOGI(TAG, "Enhanced UI created successfully");
}

/**
 * @brief Application main entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "Air101-LCD LVGL Demo Starting");
    
    // Log system information
    ESP_LOGI(TAG, "Free heap size: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // Initialize LVGL and drivers
    esp_err_t ret = init_lvgl();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LVGL: %d", ret);
        return;
    }
    
    // Create the user interface
    create_demo_ui();
    
    // Wait a bit, then switch to enhanced UI
    vTaskDelay(pdMS_TO_TICKS(3000));
    create_enhanced_ui();
    
    // Create LVGL timer task
    xTaskCreatePinnedToCore(
        lvgl_timer_task,
        "LVGL_Timer",
        4096,
        NULL,
        5, // High priority
        &lvgl_task_handle,
        1  // Pin to core 1
    );
    
    if (lvgl_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create LVGL timer task");
        return;
    }
    
    ESP_LOGI(TAG, "LVGL demo application started successfully");
    ESP_LOGI(TAG, "Use the joystick to navigate the interface");
    
    // Main application loop - monitor system health
    uint32_t loop_count = 0;
    while (1) {
        // Log system stats every 30 seconds
        if (loop_count % 300 == 0) {
            ESP_LOGI(TAG, "System stats - Free heap: %d bytes", 
                    esp_get_free_heap_size());
        }
        
        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay
    }
}
