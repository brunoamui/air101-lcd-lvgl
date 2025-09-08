/**
 * @file ads1118_driver.c
 * @brief ADS1118 16-bit ADC Driver Implementation for ESP32-C3
 * 
 * Implementation of Texas Instruments ADS1118 16-bit ADC with SPI interface.
 * Optimized for dual-channel current monitoring applications with ACS712 sensors.
 * 
 * @author Air101-LCD-LVGL Project  
 * @date 2025-01-07
 */

#include "ads1118_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ADS1118";

// ADS1118 Configuration Register Default Values
#define ADS1118_CONFIG_DEFAULT      0x858B  // Default config register value
#define ADS1118_CONFIG_OS_START     (1 << ADS1118_REG_CONFIG_OS_SHIFT)
#define ADS1118_CONFIG_VALID        (1 << ADS1118_REG_CONFIG_VALID_SHIFT)
#define ADS1118_CONFIG_NOP          (0x01 << ADS1118_REG_CONFIG_NOP_SHIFT)

// ADS1118 Voltage Reference and Resolution
#define ADS1118_VREF_MV            2048     // Internal reference voltage in mV
#define ADS1118_RESOLUTION_BITS    16       // ADC resolution
#define ADS1118_MAX_COUNTS         32768    // 2^15 for signed 16-bit

// ADS1118 PGA Full Scale Ranges in mV
static const uint16_t ads1118_pga_ranges_mv[] = {
    6144, // ADS1118_PGA_6_144V
    4096, // ADS1118_PGA_4_096V  
    2048, // ADS1118_PGA_2_048V
    1024, // ADS1118_PGA_1_024V
    512,  // ADS1118_PGA_0_512V
    256   // ADS1118_PGA_0_256V
};

/**
 * @brief Build configuration register value
 * @param config Pointer to configuration structure
 * @param mux Multiplexer setting to use
 * @return 16-bit configuration register value
 */
static uint16_t ads1118_build_config_reg(const ads1118_config_t *config, ads1118_mux_t mux) {
    uint16_t config_reg = 0;
    
    // Start single conversion
    config_reg |= ADS1118_CONFIG_OS_START;
    
    // Input multiplexer
    config_reg |= (mux << ADS1118_REG_CONFIG_MUX_SHIFT);
    
    // Programmable gain amplifier
    config_reg |= (config->pga << ADS1118_REG_CONFIG_PGA_SHIFT);
    
    // Operating mode
    config_reg |= (config->mode << ADS1118_REG_CONFIG_MODE_SHIFT);
    
    // Data rate
    config_reg |= (config->data_rate << ADS1118_REG_CONFIG_DR_SHIFT);
    
    // Temperature sensor mode
    if (config->temperature_mode) {
        config_reg |= (1 << ADS1118_REG_CONFIG_TS_MODE_SHIFT);
    }
    
    // Pull-up enable
    if (config->pullup_enable) {
        config_reg |= (1 << ADS1118_REG_CONFIG_PULL_UP_EN_SHIFT);
    }
    
    // No operation (required as per datasheet)
    config_reg |= ADS1118_CONFIG_NOP;
    
    // Valid data bit
    config_reg |= ADS1118_CONFIG_VALID;
    
    return config_reg;
}

/**
 * @brief Calculate voltage per bit for current PGA setting
 * @param pga PGA setting
 * @return Voltage per bit in volts
 */
static float ads1118_calculate_voltage_per_bit(ads1118_pga_t pga) {
    uint16_t range_mv = ads1118_pga_ranges_mv[pga];
    return (float)range_mv / 1000.0f / (float)ADS1118_MAX_COUNTS;
}

/**
 * @brief Perform SPI transaction with ADS1118
 * @param handle Pointer to ADS1118 handle
 * @param tx_data Data to transmit (16-bit config)
 * @param rx_data Pointer to receive data buffer (16-bit ADC result)
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t ads1118_spi_transaction(ads1118_handle_t *handle, uint16_t tx_data, uint16_t *rx_data) {
    spi_transaction_t trans = {
        .length = 16,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .tx_data = {(tx_data >> 8) & 0xFF, tx_data & 0xFF},
    };
    
    esp_err_t ret = spi_device_polling_transmit(handle->spi_device, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (rx_data) {
        *rx_data = (trans.rx_data[0] << 8) | trans.rx_data[1];
    }
    
    return ESP_OK;
}

esp_err_t ads1118_init(ads1118_handle_t *handle, const ads1118_config_t *config) {
    if (!handle || !config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing ADS1118 ADC on CS pin %d", ADS1118_CS_PIN);
    
    // Configure CS pin
    gpio_config_t cs_config = {
        .pin_bit_mask = (1ULL << ADS1118_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&cs_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS pin: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(ADS1118_CS_PIN, 1); // CS idle high
    
    // SPI device configuration
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = ADS1118_SPI_CLOCK_HZ,
        .mode = 1, // SPI mode 1 (CPOL=0, CPHA=1) as per ADS1118 datasheet
        .spics_io_num = ADS1118_CS_PIN,
        .queue_size = 1,
        .flags = 0,
    };
    
    ret = spi_bus_add_device(ADS1118_SPI_HOST, &dev_config, &handle->spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Store configuration
    handle->config = *config;
    handle->cs_pin = ADS1118_CS_PIN;
    handle->voltage_per_bit = ads1118_calculate_voltage_per_bit(config->pga);
    
    ESP_LOGI(TAG, "ADS1118 initialized successfully");
    ESP_LOGI(TAG, "PGA: ±%dmV, Data Rate: %d, Mode: %s", 
             ads1118_pga_ranges_mv[config->pga],
             config->data_rate,
             (config->mode == ADS1118_MODE_CONTINUOUS) ? "Continuous" : "Single-shot");
    
    return ESP_OK;
}

esp_err_t ads1118_deinit(ads1118_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->spi_device) {
        esp_err_t ret = spi_bus_remove_device(handle->spi_device);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
            return ret;
        }
        handle->spi_device = NULL;
    }
    
    ESP_LOGI(TAG, "ADS1118 deinitialized");
    return ESP_OK;
}

esp_err_t ads1118_read_raw(ads1118_handle_t *handle, ads1118_mux_t mux, int16_t *raw_value) {
    if (!handle || !raw_value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Build configuration register
    uint16_t config_reg = ads1118_build_config_reg(&handle->config, mux);
    
    // First transaction: Send configuration, receive previous conversion result
    uint16_t dummy_result;
    esp_err_t ret = ads1118_spi_transaction(handle, config_reg, &dummy_result);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for conversion to complete
    // For single-shot mode, wait based on data rate
    const uint16_t conversion_delays_ms[] = {
        125, 63, 32, 16, 8, 4, 3, 2  // Delays for each data rate (8SPS to 860SPS)
    };
    uint16_t delay_ms = conversion_delays_ms[handle->config.data_rate];
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // Second transaction: Send configuration again, receive actual conversion result
    uint16_t adc_result;
    ret = ads1118_spi_transaction(handle, config_reg, &adc_result);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *raw_value = (int16_t)adc_result;
    
    ESP_LOGD(TAG, "Raw ADC value: 0x%04X (%d)", adc_result, *raw_value);
    
    return ESP_OK;
}

esp_err_t ads1118_read_voltage(ads1118_handle_t *handle, ads1118_mux_t mux, float *voltage_v) {
    if (!handle || !voltage_v) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int16_t raw_value;
    esp_err_t ret = ads1118_read_raw(handle, mux, &raw_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw value to voltage
    *voltage_v = (float)raw_value * handle->voltage_per_bit;
    
    ESP_LOGD(TAG, "Voltage: %.4fV (raw: %d)", *voltage_v, raw_value);
    
    return ESP_OK;
}

esp_err_t ads1118_read_current(ads1118_handle_t *handle, ads1118_mux_t mux, 
                               const current_sensor_config_t *sensor_config, 
                               float *current_a) {
    if (!handle || !sensor_config || !current_a) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float voltage_v;
    esp_err_t ret = ads1118_read_voltage(handle, mux, &voltage_v);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert voltage to current using sensor characteristics
    // Current = (Voltage - Zero_Current_Voltage) / Sensitivity
    *current_a = (voltage_v - sensor_config->zero_current_voltage) / sensor_config->sensitivity_v_per_a;
    
    // Clamp to sensor range
    if (*current_a > sensor_config->max_current_a) {
        *current_a = sensor_config->max_current_a;
    } else if (*current_a < -sensor_config->max_current_a) {
        *current_a = -sensor_config->max_current_a;
    }
    
    ESP_LOGD(TAG, "Current: %.3fA (voltage: %.4fV)", *current_a, voltage_v);
    
    return ESP_OK;
}

esp_err_t ads1118_read_temperature(ads1118_handle_t *handle, float *temperature_c) {
    if (!handle || !temperature_c) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create temporary config for temperature reading
    ads1118_config_t temp_config = handle->config;
    temp_config.temperature_mode = true;
    
    // Build temperature mode configuration
    uint16_t config_reg = ads1118_build_config_reg(&temp_config, ADS1118_MUX_A0_A1);
    
    // First transaction: Send temperature config
    uint16_t dummy_result;
    esp_err_t ret = ads1118_spi_transaction(handle, config_reg, &dummy_result);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for temperature conversion (typically faster than ADC)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Second transaction: Read temperature result
    uint16_t temp_result;
    ret = ads1118_spi_transaction(handle, config_reg, &temp_result);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw temperature to Celsius
    // Temperature = raw_value >> 2 * 0.03125°C per LSB
    int16_t temp_raw = (int16_t)temp_result >> 2; // Temperature data is in upper 14 bits
    *temperature_c = (float)temp_raw * 0.03125f;
    
    ESP_LOGD(TAG, "Temperature: %.2f°C (raw: 0x%04X)", *temperature_c, temp_result);
    
    return ESP_OK;
}

esp_err_t ads1118_set_config(ads1118_handle_t *handle, const ads1118_config_t *config) {
    if (!handle || !config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    handle->config = *config;
    handle->voltage_per_bit = ads1118_calculate_voltage_per_bit(config->pga);
    
    ESP_LOGI(TAG, "Configuration updated");
    
    return ESP_OK;
}

void ads1118_get_default_config(ads1118_config_t *config) {
    if (!config) {
        return;
    }
    
    config->mux = ADS1118_MUX_A0_A1;
    config->pga = ADS1118_PGA_4_096V;        // ±4.096V range good for ACS712
    config->mode = ADS1118_MODE_SINGLE_SHOT;  // Power-saving single-shot mode
    config->data_rate = ADS1118_DR_128SPS;    // Good balance of speed vs noise
    config->temperature_mode = false;         // ADC mode by default
    config->pullup_enable = false;            // No internal pullup needed
}

void ads1118_get_acs712_20a_config(current_sensor_config_t *sensor_config) {
    if (!sensor_config) {
        return;
    }
    
    // ACS712-20A specifications
    sensor_config->sensitivity_v_per_a = 0.1f;   // 100mV/A
    sensor_config->zero_current_voltage = 2.5f;  // 2.5V at 0A (with 5V supply)
    sensor_config->max_current_a = 20.0f;        // ±20A maximum
}
