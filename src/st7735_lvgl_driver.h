/**
 * @file st7735_lvgl_driver.h
 * @brief ST7735 display driver adapter for LVGL
 * 
 * This header provides the interface for integrating the ST7735 display
 * with LVGL graphics library using ESP-IDF native SPI driver.
 */

#ifndef ST7735_LVGL_DRIVER_H
#define ST7735_LVGL_DRIVER_H

#include "lvgl.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin definitions (matching original project)
#define TFT_SCLK 2   // Pin_SCK
#define TFT_MOSI 3   // Pin_SDA
#define TFT_RST 10   // Pin_Res  
#define TFT_DC 6     // Pin_DC
#define TFT_CS 7     // Pin_CS

// Screen dimensions
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 160
#define DISPLAY_BUFFER_SIZE (DISPLAY_WIDTH * DISPLAY_HEIGHT / 10) // 1/10 screen buffer

// ST7735 Commands
#define ST7735_NOP       0x00
#define ST7735_SWRESET   0x01
#define ST7735_SLPIN     0x10
#define ST7735_SLPOUT    0x11
#define ST7735_PTLON     0x12
#define ST7735_NORON     0x13
#define ST7735_INVOFF    0x20
#define ST7735_INVON     0x21
#define ST7735_DISPOFF   0x28
#define ST7735_DISPON    0x29
#define ST7735_CASET     0x2A
#define ST7735_RASET     0x2B
#define ST7735_RAMWR     0x2C
#define ST7735_PTLAR     0x30
#define ST7735_COLMOD    0x3A
#define ST7735_MADCTL    0x36
#define ST7735_FRMCTR1   0xB1
#define ST7735_FRMCTR2   0xB2
#define ST7735_FRMCTR3   0xB3
#define ST7735_INVCTR    0xB4
#define ST7735_DISSET5   0xB6
#define ST7735_PWCTR1    0xC0
#define ST7735_PWCTR2    0xC1
#define ST7735_PWCTR3    0xC2
#define ST7735_PWCTR4    0xC3
#define ST7735_PWCTR5    0xC4
#define ST7735_VMCTR1    0xC5
#define ST7735_PWCTR6    0xFC
#define ST7735_GMCTRP1   0xE0
#define ST7735_GMCTRN1   0xE1

/**
 * @brief Initialize ST7735 hardware for LVGL use
 * @return esp_err_t ESP_OK on success
 */
esp_err_t st7735_init_hardware(void);

/**
 * @brief Initialize LVGL display driver
 * @return lv_display_t* Pointer to the created display object
 */
lv_display_t* st7735_lvgl_init(void);

/**
 * @brief LVGL display flush callback
 * 
 * This function is called by LVGL when it needs to flush the display buffer
 * to the actual display hardware.
 */
void st7735_lvgl_flush(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * px_map);

/**
 * @brief Send command to ST7735
 * @param cmd Command byte to send
 */
void st7735_send_command(uint8_t cmd);

/**
 * @brief Send data to ST7735
 * @param data Data byte to send
 */
void st7735_send_data(uint8_t data);

/**
 * @brief Send 16-bit data to ST7735 (for colors)
 * @param data 16-bit data to send
 */
void st7735_send_data_16(uint16_t data);

/**
 * @brief Set address window for drawing
 * @param x0 Start X coordinate
 * @param y0 Start Y coordinate
 * @param x1 End X coordinate
 * @param y1 End Y coordinate
 */
void st7735_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

#ifdef __cplusplus
}
#endif

#endif // ST7735_LVGL_DRIVER_H
