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
#include "lv_demos.h"

// Custom driver includes
#include "st7735_lvgl_driver.h"
#include "joystick_lvgl_driver.h"

static const char* TAG = "Air101-LVGL";

// Pin definitions (same as Arduino version)
#define TFT_SCLK 2   // Pin_SCK
#define TFT_MOSI 3   // Pin_SDA
#define TFT_RST 10   // Pin_Res  
#define TFT_DC 6     // Pin_DC
#define TFT_CS 7     // Pin_CS

// Screen dimensions
#define SCREEN_WIDTH 100
#define SCREEN_HEIGHT 160

// Joystick pin definitions
#define BUTTON_LEFT 8     // Left
#define BUTTON_UP 9       // Up
#define BUTTON_CENTER 4   // Center
#define BUTTON_DOWN 5     // Down
#define BUTTON_RIGHT 13   // Right

// ST7735 Commands
#define ST7735_NOP       0x00
#define ST7735_SWRESET   0x01
#define ST7735_RDDID     0x04
#define ST7735_RDDST     0x09
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
#define ST7735_RAMRD     0x2E
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
#define ST7735_RDID1     0xDA
#define ST7735_RDID2     0xDB
#define ST7735_RDID3     0xDC
#define ST7735_RDID4     0xDD
#define ST7735_PWCTR6    0xFC
#define ST7735_GMCTRP1   0xE0
#define ST7735_GMCTRN1   0xE1

// Color definitions (RGB565)
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF
#define ST7735_ORANGE  0xFD20

// SPI device handle
static spi_device_handle_t spi_device;

// Game variables
static int player_x = 50;
static int player_y = 80;
static int step = 1;
static int goal_w = 15;
static int goal_h = 13;
static int goal[2] = {60, 100};

// Button debouncing variables
static uint32_t last_button_time = 0;
static const uint32_t button_debounce_ms = 50; // 50ms debounce for responsiveness

// Function prototypes
void spi_send_command(uint8_t cmd);
void spi_send_data(uint8_t data);
void spi_send_data_16(uint16_t data);
void st7735_init(void);
void set_addr_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void fill_screen(uint16_t color);
void fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void draw_pixel(int16_t x, int16_t y, uint16_t color);
void init_gpio(void);
void display_task(void *arg);
void snake_game(void);

void spi_send_command(uint8_t cmd) {
    gpio_set_level(TFT_DC, 0); // Command mode
    
    spi_transaction_t trans = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = cmd;
    spi_device_polling_transmit(spi_device, &trans);
}

void spi_send_data(uint8_t data) {
    gpio_set_level(TFT_DC, 1); // Data mode
    
    spi_transaction_t trans = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = data;
    spi_device_polling_transmit(spi_device, &trans);
}

void spi_send_data_16(uint16_t data) {
    gpio_set_level(TFT_DC, 1); // Data mode
    
    spi_transaction_t trans = {
        .length = 16,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = data >> 8;
    trans.tx_data[1] = data & 0xFF;
    spi_device_polling_transmit(spi_device, &trans);
}

void set_addr_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    spi_send_command(ST7735_CASET); // Column address set
    spi_send_data(0x00);
    spi_send_data(x0 + 2);   // XSTART (offset for ST7735)
    spi_send_data(0x00);
    spi_send_data(x1 + 2);   // XEND

    spi_send_command(ST7735_RASET); // Row address set
    spi_send_data(0x00);
    spi_send_data(y0 + 1);   // YSTART (offset for ST7735)
    spi_send_data(0x00);
    spi_send_data(y1 + 1);   // YEND

    spi_send_command(ST7735_RAMWR); // Write to RAM
}

void fill_screen(uint16_t color) {
    fill_rect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, color);
}

void fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) return;
    if (x + w - 1 >= SCREEN_WIDTH) w = SCREEN_WIDTH - x;
    if (y + h - 1 >= SCREEN_HEIGHT) h = SCREEN_HEIGHT - y;

    set_addr_window(x, y, x + w - 1, y + h - 1);
    
    for (int i = 0; i < w * h; i++) {
        spi_send_data_16(color);
    }
}

void draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || y < 0 || x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) return;
    
    set_addr_window(x, y, x, y);
    spi_send_data_16(color);
}

void fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    for (int16_t y = -r; y <= r; y++) {
        for (int16_t x = -r; x <= r; x++) {
            if (x * x + y * y <= r * r) {
                draw_pixel(x0 + x, y0 + y, color);
            }
        }
    }
}

void st7735_init(void) {
    // Reset display
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Starting ST7735 initialization");
    
    // Software reset
    spi_send_command(ST7735_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Sleep out
    spi_send_command(ST7735_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Frame rate control - normal mode
    spi_send_command(ST7735_FRMCTR1);
    spi_send_data(0x01); spi_send_data(0x2C); spi_send_data(0x2D);
    
    // Frame rate control - idle mode
    spi_send_command(ST7735_FRMCTR2);
    spi_send_data(0x01); spi_send_data(0x2C); spi_send_data(0x2D);
    
    // Frame rate control - partial mode
    spi_send_command(ST7735_FRMCTR3);
    spi_send_data(0x01); spi_send_data(0x2C); spi_send_data(0x2D);
    spi_send_data(0x01); spi_send_data(0x2C); spi_send_data(0x2D);
    
    // Display inversion control
    spi_send_command(ST7735_INVCTR);
    spi_send_data(0x07);
    
    // Power control
    spi_send_command(ST7735_PWCTR1);
    spi_send_data(0xA2);
    spi_send_data(0x02);      // -4.6V
    spi_send_data(0x84);      // AUTO mode
    
    spi_send_command(ST7735_PWCTR2);
    spi_send_data(0xC5);      // VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    
    spi_send_command(ST7735_PWCTR3);
    spi_send_data(0x0A);      // Opamp current small
    spi_send_data(0x00);      // Boost frequency
    
    spi_send_command(ST7735_PWCTR4);
    spi_send_data(0x8A);      // BCLK/2, Opamp current small & Medium low
    spi_send_data(0x2A);
    
    spi_send_command(ST7735_PWCTR5);
    spi_send_data(0x8A); spi_send_data(0xEE);
    
    spi_send_command(ST7735_VMCTR1);
    spi_send_data(0x0E);
    
    spi_send_command(ST7735_INVOFF);    // Don't invert display
    
    spi_send_command(ST7735_MADCTL);
    spi_send_data(0xC8);      // Row addr/col addr, bottom to top refresh
    
    // Color mode 16-bit
    spi_send_command(ST7735_COLMOD);
    spi_send_data(0x05);
    
    // Gamma correction
    spi_send_command(ST7735_GMCTRP1);
    spi_send_data(0x02); spi_send_data(0x1c); spi_send_data(0x07); spi_send_data(0x12);
    spi_send_data(0x37); spi_send_data(0x32); spi_send_data(0x29); spi_send_data(0x2d);
    spi_send_data(0x29); spi_send_data(0x25); spi_send_data(0x2B); spi_send_data(0x39);
    spi_send_data(0x00); spi_send_data(0x01); spi_send_data(0x03); spi_send_data(0x10);
    
    spi_send_command(ST7735_GMCTRN1);
    spi_send_data(0x03); spi_send_data(0x1d); spi_send_data(0x07); spi_send_data(0x06);
    spi_send_data(0x2E); spi_send_data(0x2C); spi_send_data(0x29); spi_send_data(0x2D);
    spi_send_data(0x2E); spi_send_data(0x2E); spi_send_data(0x37); spi_send_data(0x3F);
    spi_send_data(0x00); spi_send_data(0x00); spi_send_data(0x02); spi_send_data(0x10);
    
    spi_send_command(ST7735_NORON);     // Normal display on
    vTaskDelay(pdMS_TO_TICKS(10));
    
    spi_send_command(ST7735_DISPON);    // Display on
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "ST7735 initialization complete");
}

void init_gpio(void) {
    // Configure control pins
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << TFT_RST) | (1ULL << TFT_DC),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_cfg);
    
    // Configure joystick input pins with pullup
    gpio_cfg.pin_bit_mask = (1ULL << BUTTON_LEFT) | (1ULL << BUTTON_UP) | 
                           (1ULL << BUTTON_CENTER) | (1ULL << BUTTON_DOWN) | 
                           (1ULL << BUTTON_RIGHT);
    gpio_cfg.mode = GPIO_MODE_INPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gpio_cfg);
    
    ESP_LOGI(TAG, "GPIO configuration complete");
}

void spi_init(void) {
    // SPI bus configuration
    spi_bus_config_t bus_cfg = {
        .miso_io_num = -1,  // Not used
        .mosi_io_num = TFT_MOSI,
        .sclk_io_num = TFT_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    
    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    
    // SPI device configuration
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        .mode = 0,                      // SPI mode 0
        .spics_io_num = TFT_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    
    // Add device to SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device));
    
    ESP_LOGI(TAG, "SPI initialization complete");
}

void snake_game(void) {
    // Read joystick values
    int leftValue = gpio_get_level(BUTTON_LEFT);
    int upValue = gpio_get_level(BUTTON_UP);
    int centerValue = gpio_get_level(BUTTON_CENTER);
    int downValue = gpio_get_level(BUTTON_DOWN);
    int rightValue = gpio_get_level(BUTTON_RIGHT);

    // Get current time for debouncing
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool can_press_button = (current_time - last_button_time) > button_debounce_ms;

    // Debug: Log button states every 200 calls (about once per 2 seconds at 10ms)
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter >= 200) {
        ESP_LOGI(TAG, "Button states - L:%d U:%d C:%d D:%d R:%d", 
                leftValue, upValue, centerValue, downValue, rightValue);
        debug_counter = 0;
    }

    // Clear previous position
    fill_circle(player_x, player_y, 4, ST7735_BLACK);

    // Movement logic with debouncing (buttons are active LOW due to pullup)
    if (can_press_button) {
        bool button_pressed = false;
        
        if (leftValue == 0) {
            player_x = player_x - step;
            ESP_LOGI(TAG, "Left pressed");
            button_pressed = true;
        }
        else if (rightValue == 0) {
            player_x = player_x + step;
            ESP_LOGI(TAG, "Right pressed");
            button_pressed = true;
        }
        else if (upValue == 0) {
            player_y = player_y - step;
            ESP_LOGI(TAG, "Up pressed");
            button_pressed = true;
        }
        else if (downValue == 0) {
            player_y = player_y + step;
            ESP_LOGI(TAG, "Down pressed");
            button_pressed = true;
        }
        else if (centerValue == 0) {
            ESP_LOGI(TAG, "Center pressed - reset screen");
            fill_screen(ST7735_BLACK);
            fill_rect(goal[0], goal[1], goal_w, goal_h, ST7735_ORANGE);
            button_pressed = true;
        }
        
        if (button_pressed) {
            last_button_time = current_time;
        }
    }

    // Wrap around screen edges
    if (player_x < 0) {
        player_x = SCREEN_WIDTH - 1;
    }
    if (player_x > SCREEN_WIDTH - 1) {
        player_x = 0;
    }
    if (player_y < 0) {
        player_y = SCREEN_HEIGHT - 1;
    }
    if (player_y > SCREEN_HEIGHT - 1) {
        player_y = 0;
    }

    // Check if goal is reached
    if (player_x >= goal[0] && player_x <= (goal[0] + goal_w) && 
        player_y >= goal[1] && player_y <= (goal[1] + goal_h)) {
        ESP_LOGI(TAG, "Goal reached!");
        fill_screen(ST7735_BLACK);
        goal[0] = esp_random() % (80 - 20) + 20;  // Random between 20-80
        goal[1] = esp_random() % (150 - 20) + 20; // Random between 20-150
        fill_rect(goal[0], goal[1], goal_w, goal_h, ST7735_ORANGE);
    }

    // Draw player dot
    fill_circle(player_x, player_y, 4, ST7735_BLUE);
}

void display_task(void *arg) {
    static int test_phase = 0;
    uint32_t last_update = 0;
    
    ESP_LOGI(TAG, "Display task started");
    
    // Initial test pattern
    fill_screen(ST7735_BLACK);
    
    // Draw test pattern
    fill_rect(10, 10, 30, 20, ST7735_RED);
    fill_rect(50, 10, 30, 20, ST7735_GREEN);
    fill_rect(10, 40, 30, 20, ST7735_BLUE);
    fill_rect(50, 40, 30, 20, ST7735_YELLOW);
    
    ESP_LOGI(TAG, "Test pattern drawn - setup complete");
    
    while (1) {
        if (test_phase < 4) {
            // Phase cycling mode - use 3-second timer
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            if (current_time - last_update > 3000) {
                last_update = current_time;
                
                switch (test_phase) {
                    case 0:
                        ESP_LOGI(TAG, "Phase 0: Red screen");
                        fill_screen(ST7735_RED);
                        break;
                    case 1:
                        ESP_LOGI(TAG, "Phase 1: Green screen");
                        fill_screen(ST7735_GREEN);
                        break;
                    case 2:
                        ESP_LOGI(TAG, "Phase 2: Blue screen");
                        fill_screen(ST7735_BLUE);
                        break;
                    case 3:
                        ESP_LOGI(TAG, "Phase 3: Starting snake game");
                        fill_screen(ST7735_BLACK);
                        fill_rect(goal[0], goal[1], goal_w, goal_h, ST7735_ORANGE);
                        test_phase = 4; // Move to snake game mode
                        break;
                }
                
                if (test_phase < 4) {
                    test_phase++;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
        } else {
            // Snake game mode - no timers, just run continuously
            snake_game();
            vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay for very responsive buttons
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Air101-LCD Test - ESP-IDF Version Starting");
    
    // Initialize GPIO pins
    init_gpio();
    
    // Initialize SPI
    spi_init();
    
    // Initialize display
    st7735_init();
    
    ESP_LOGI(TAG, "Hardware initialization complete");
    
    // Create display task (replaces Arduino loop)
    xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Display task created - system ready");
}
