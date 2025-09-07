/**
 * @file st7735_lvgl_driver.c
 * @brief ST7735 display driver implementation for LVGL
 */

#include "st7735_lvgl_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ST7735_LVGL";

// SPI device handle
static spi_device_handle_t spi_device;

// LVGL display buffers (double buffering for smooth animation)
static lv_color_t *disp_draw_buf1;
static lv_color_t *disp_draw_buf2;
static lv_display_t *display;

void st7735_send_command(uint8_t cmd) {
    gpio_set_level(TFT_DC, 0); // Command mode
    
    spi_transaction_t trans = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = cmd;
    spi_device_polling_transmit(spi_device, &trans);
}

void st7735_send_data(uint8_t data) {
    gpio_set_level(TFT_DC, 1); // Data mode
    
    spi_transaction_t trans = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = data;
    spi_device_polling_transmit(spi_device, &trans);
}

void st7735_send_data_16(uint16_t data) {
    gpio_set_level(TFT_DC, 1); // Data mode
    
    spi_transaction_t trans = {
        .length = 16,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = data >> 8;
    trans.tx_data[1] = data & 0xFF;
    spi_device_polling_transmit(spi_device, &trans);
}

void st7735_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Column address set
    st7735_send_command(ST7735_CASET);
    st7735_send_data(0x00);
    st7735_send_data(x0 + 2);   // XSTART (offset for ST7735)
    st7735_send_data(0x00);
    st7735_send_data(x1 + 2);   // XEND

    // Row address set
    st7735_send_command(ST7735_RASET);
    st7735_send_data(0x00);
    st7735_send_data(y0 + 1);   // YSTART (offset for ST7735)
    st7735_send_data(0x00);
    st7735_send_data(y1 + 1);   // YEND

    // Write to RAM
    st7735_send_command(ST7735_RAMWR);
}

static void st7735_hardware_init(void) {
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
        .clock_speed_hz = SPI_MASTER_FREQ_20M, // Increased speed for better performance
        .mode = 0,                      // SPI mode 0
        .spics_io_num = TFT_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    
    // Add device to SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device));
    
    ESP_LOGI(TAG, \"SPI initialization complete\");
}

static void st7735_display_init(void) {
    // Reset display
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, \"Starting ST7735 initialization\");
    
    // Software reset
    st7735_send_command(ST7735_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Sleep out
    st7735_send_command(ST7735_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Frame rate control - normal mode
    st7735_send_command(ST7735_FRMCTR1);
    st7735_send_data(0x01); st7735_send_data(0x2C); st7735_send_data(0x2D);
    
    // Frame rate control - idle mode
    st7735_send_command(ST7735_FRMCTR2);
    st7735_send_data(0x01); st7735_send_data(0x2C); st7735_send_data(0x2D);
    
    // Frame rate control - partial mode
    st7735_send_command(ST7735_FRMCTR3);
    st7735_send_data(0x01); st7735_send_data(0x2C); st7735_send_data(0x2D);
    st7735_send_data(0x01); st7735_send_data(0x2C); st7735_send_data(0x2D);
    
    // Display inversion control
    st7735_send_command(ST7735_INVCTR);
    st7735_send_data(0x07);
    
    // Power control
    st7735_send_command(ST7735_PWCTR1);
    st7735_send_data(0xA2);
    st7735_send_data(0x02);      // -4.6V
    st7735_send_data(0x84);      // AUTO mode
    
    st7735_send_command(ST7735_PWCTR2);
    st7735_send_data(0xC5);      // VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    
    st7735_send_command(ST7735_PWCTR3);
    st7735_send_data(0x0A);      // Opamp current small
    st7735_send_data(0x00);      // Boost frequency
    
    st7735_send_command(ST7735_PWCTR4);
    st7735_send_data(0x8A);      // BCLK/2, Opamp current small & Medium low
    st7735_send_data(0x2A);
    
    st7735_send_command(ST7735_PWCTR5);
    st7735_send_data(0x8A); st7735_send_data(0xEE);
    
    st7735_send_command(ST7735_VMCTR1);
    st7735_send_data(0x0E);
    
    st7735_send_command(ST7735_INVOFF);    // Don't invert display
    
    st7735_send_command(ST7735_MADCTL);
    st7735_send_data(0xC8);      // Row addr/col addr, bottom to top refresh
    
    // Color mode 16-bit
    st7735_send_command(ST7735_COLMOD);
    st7735_send_data(0x05);
    
    // Gamma correction
    st7735_send_command(ST7735_GMCTRP1);
    st7735_send_data(0x02); st7735_send_data(0x1c); st7735_send_data(0x07); st7735_send_data(0x12);
    st7735_send_data(0x37); st7735_send_data(0x32); st7735_send_data(0x29); st7735_send_data(0x2d);
    st7735_send_data(0x29); st7735_send_data(0x25); st7735_send_data(0x2B); st7735_send_data(0x39);
    st7735_send_data(0x00); st7735_send_data(0x01); st7735_send_data(0x03); st7735_send_data(0x10);
    
    st7735_send_command(ST7735_GMCTRN1);
    st7735_send_data(0x03); st7735_send_data(0x1d); st7735_send_data(0x07); st7735_send_data(0x06);
    st7735_send_data(0x2E); st7735_send_data(0x2C); st7735_send_data(0x29); st7735_send_data(0x2D);
    st7735_send_data(0x2E); st7735_send_data(0x2E); st7735_send_data(0x37); st7735_send_data(0x3F);
    st7735_send_data(0x00); st7735_send_data(0x00); st7735_send_data(0x02); st7735_send_data(0x10);
    
    st7735_send_command(ST7735_NORON);     // Normal display on
    vTaskDelay(pdMS_TO_TICKS(10));
    
    st7735_send_command(ST7735_DISPON);    // Display on
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, \"ST7735 initialization complete\");
}

void st7735_lvgl_flush(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * px_map) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    
    // Set address window
    st7735_set_addr_window(area->x1, area->y1, area->x2, area->y2);
    
    // Send pixel data
    gpio_set_level(TFT_DC, 1); // Data mode
    
    // For RGB565 format, we can send data directly
    // LVGL uses native color format (RGB565 for 16-bit)
    uint16_t *color_p = (uint16_t *)px_map;
    
    // Send color data efficiently
    for (uint32_t i = 0; i < w * h; i++) {
        uint16_t color = color_p[i];
        // Swap bytes for ST7735 if needed (depends on LVGL configuration)
        st7735_send_data_16(color);
    }
    
    // Inform LVGL that flushing is done
    lv_display_flush_ready(disp_drv);
}

esp_err_t st7735_init_hardware(void) {
    esp_err_t ret = ESP_OK;
    
    st7735_hardware_init();
    st7735_display_init();
    
    ESP_LOGI(TAG, "ST7735 hardware initialization completed successfully");
    return ESP_OK;
}

lv_display_t* st7735_lvgl_init(void) {
    // Allocate draw buffers
    disp_draw_buf1 = heap_caps_malloc(DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    disp_draw_buf2 = heap_caps_malloc(DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    
    if (!disp_draw_buf1 || !disp_draw_buf2) {
        ESP_LOGE(TAG, \"Failed to allocate display buffers\");
        return NULL;
    }
    
    ESP_LOGI(TAG, \"Display buffers allocated: %d bytes each\", DISPLAY_BUFFER_SIZE * sizeof(lv_color_t));
    
    // Create display
    display = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    if (!display) {
        ESP_LOGE(TAG, \"Failed to create LVGL display\");
        return NULL;
    }
    
    // Set display buffer
    lv_display_set_buffers(display, disp_draw_buf1, disp_draw_buf2, 
                          DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    
    // Set flush callback
    lv_display_set_flush_cb(display, st7735_lvgl_flush);
    
    ESP_LOGI(TAG, \"LVGL display initialized: %dx%d\", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    
    return display;
}
