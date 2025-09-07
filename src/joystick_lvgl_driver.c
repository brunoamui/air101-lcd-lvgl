/**
 * @file joystick_lvgl_driver.c
 * @brief Joystick input driver implementation for LVGL
 */

#include "joystick_lvgl_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "JOYSTICK_LVGL";

// Button debouncing variables
static uint32_t last_button_time = 0;
static lv_indev_t *joystick_indev;

// Current joystick state
typedef struct {
    bool left;
    bool right;
    bool up;
    bool down;
    bool center;
    bool any_pressed;
    uint32_t last_press_time;
} joystick_state_t;

static joystick_state_t joystick_state = {0};

esp_err_t joystick_init_hardware(void) {
    // Configure joystick input pins with pullup
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_LEFT) | (1ULL << BUTTON_UP) | 
                       (1ULL << BUTTON_CENTER) | (1ULL << BUTTON_DOWN) | 
                       (1ULL << BUTTON_RIGHT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure joystick GPIO pins");
        return ret;
    }
    
    ESP_LOGI(TAG, "Joystick hardware initialization complete");
    return ESP_OK;
}

void joystick_read_buttons(bool *left, bool *right, bool *up, bool *down, bool *center) {
    // Read GPIO levels (buttons are active LOW due to pullup resistors)
    *left = (gpio_get_level(BUTTON_LEFT) == 0);
    *right = (gpio_get_level(BUTTON_RIGHT) == 0);
    *up = (gpio_get_level(BUTTON_UP) == 0);
    *down = (gpio_get_level(BUTTON_DOWN) == 0);
    *center = (gpio_get_level(BUTTON_CENTER) == 0);
}

void joystick_lvgl_read(lv_indev_t * indev_drv, lv_indev_data_t * data) {
    // Get current time for debouncing
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool can_press_button = (current_time - last_button_time) > JOYSTICK_DEBOUNCE_MS;
    
    // Read current button states
    bool left, right, up, down, center;
    joystick_read_buttons(&left, &right, &up, &down, &center);
    
    // Check if any button is currently pressed
    bool any_pressed = left || right || up || down || center;
    
    // Update joystick state
    joystick_state.left = left;
    joystick_state.right = right;
    joystick_state.up = up;
    joystick_state.down = down;
    joystick_state.center = center;
    joystick_state.any_pressed = any_pressed;
    
    // Set default state
    data->state = LV_INDEV_STATE_RELEASED;
    data->continue_reading = false;
    
    // Handle button presses with debouncing
    if (can_press_button && any_pressed) {
        data->state = LV_INDEV_STATE_PRESSED;
        last_button_time = current_time;
        joystick_state.last_press_time = current_time;
        
        // Map joystick directions to LVGL key codes
        if (left) {
            data->key = LV_KEY_LEFT;
            ESP_LOGI(TAG, "Left pressed");
        } else if (right) {
            data->key = LV_KEY_RIGHT;
            ESP_LOGI(TAG, "Right pressed");
        } else if (up) {
            data->key = LV_KEY_UP;
            ESP_LOGI(TAG, "Up pressed");
        } else if (down) {
            data->key = LV_KEY_DOWN;
            ESP_LOGI(TAG, "Down pressed");
        } else if (center) {
            data->key = LV_KEY_ENTER;
            ESP_LOGI(TAG, "Center pressed");
        }
    } else if (any_pressed) {
        // Button still pressed but within debounce period
        data->state = LV_INDEV_STATE_PRESSED;
        
        // Keep the same key as the last press
        if (left) {
            data->key = LV_KEY_LEFT;
        } else if (right) {
            data->key = LV_KEY_RIGHT;
        } else if (up) {
            data->key = LV_KEY_UP;
        } else if (down) {
            data->key = LV_KEY_DOWN;
        } else if (center) {
            data->key = LV_KEY_ENTER;
        }
    }
    
    // Debug: Log button states periodically
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter >= 1000) { // About every 10 seconds at 10ms polling
        ESP_LOGI(TAG, "Joystick state - L:%d R:%d U:%d D:%d C:%d", 
                left, right, up, down, center);
        debug_counter = 0;
    }
}

lv_indev_t* joystick_lvgl_init(void) {
    // Create input device
    joystick_indev = lv_indev_create();
    if (!joystick_indev) {
        ESP_LOGE(TAG, "Failed to create LVGL input device");
        return NULL;
    }
    
    // Configure input device
    lv_indev_set_type(joystick_indev, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(joystick_indev, joystick_lvgl_read);
    
    ESP_LOGI(TAG, "LVGL joystick input device initialized");
    
    return joystick_indev;
}
