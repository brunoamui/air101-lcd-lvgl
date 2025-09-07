/**
 * @file joystick_lvgl_driver.h
 * @brief Joystick input driver for LVGL integration
 * 
 * This header provides the interface for integrating the joystick
 * with LVGL input device system for UI navigation.
 */

#ifndef JOYSTICK_LVGL_DRIVER_H
#define JOYSTICK_LVGL_DRIVER_H

#include "lvgl.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Joystick pin definitions (matching original project)
#define BUTTON_LEFT 8     // Left
#define BUTTON_UP 9       // Up
#define BUTTON_CENTER 4   // Center
#define BUTTON_DOWN 5     // Down
#define BUTTON_RIGHT 13   // Right

// Button debouncing settings
#define JOYSTICK_DEBOUNCE_MS 50

/**
 * @brief Initialize joystick hardware
 * @return esp_err_t ESP_OK on success
 */
esp_err_t joystick_init_hardware(void);

/**
 * @brief Initialize LVGL input device for joystick
 * @return lv_indev_t* Pointer to the created input device
 */
lv_indev_t* joystick_lvgl_init(void);

/**
 * @brief LVGL input device read callback for joystick
 * 
 * This function is called by LVGL to read the current state of the joystick.
 * It maps joystick buttons to LVGL input events.
 */
void joystick_lvgl_read(lv_indev_t * indev_drv, lv_indev_data_t * data);

/**
 * @brief Read raw joystick button states
 * @param left Pointer to store left button state
 * @param right Pointer to store right button state
 * @param up Pointer to store up button state
 * @param down Pointer to store down button state
 * @param center Pointer to store center button state
 */
void joystick_read_buttons(bool *left, bool *right, bool *up, bool *down, bool *center);

#ifdef __cplusplus
}
#endif

#endif // JOYSTICK_LVGL_DRIVER_H
