/**
 * @file ads1118_driver.h
 * @brief ADS1118 16-bit ADC Driver for ESP32-C3
 * 
 * Driver for Texas Instruments ADS1118 16-bit ADC with SPI interface.
 * Optimized for dual-channel current monitoring applications.
 * 
 * @author Air101-LCD-LVGL Project
 * @date 2025-01-07
 */

#ifndef ADS1118_DRIVER_H
#define ADS1118_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ADS1118 Hardware Configuration
#define ADS1118_SPI_HOST        SPI2_HOST
#define ADS1118_SPI_CLOCK_HZ    4000000    // 4MHz SPI clock
#define ADS1118_CS_PIN          GPIO_NUM_1 // Chip Select pin

// ADS1118 Register Bit Definitions
#define ADS1118_REG_CONFIG_OS_SHIFT     15  // Operational status/single-shot conversion start
#define ADS1118_REG_CONFIG_MUX_SHIFT    12  // Input multiplexer configuration
#define ADS1118_REG_CONFIG_PGA_SHIFT    9   // Programmable gain amplifier configuration
#define ADS1118_REG_CONFIG_MODE_SHIFT   8   // Device operating mode
#define ADS1118_REG_CONFIG_DR_SHIFT     5   // Data rate
#define ADS1118_REG_CONFIG_TS_MODE_SHIFT 4  // Temperature sensor mode
#define ADS1118_REG_CONFIG_PULL_UP_EN_SHIFT 3 // Pull-up enable
#define ADS1118_REG_CONFIG_NOP_SHIFT    1   // No operation
#define ADS1118_REG_CONFIG_VALID_SHIFT  0   // Valid data

// ADS1118 Multiplexer Configuration
typedef enum {
    ADS1118_MUX_A0_A1 = 0x00, // AINP = AIN0, AINN = AIN1 (default)
    ADS1118_MUX_A0_A3 = 0x01, // AINP = AIN0, AINN = AIN3
    ADS1118_MUX_A1_A3 = 0x02, // AINP = AIN1, AINN = AIN3
    ADS1118_MUX_A2_A3 = 0x03, // AINP = AIN2, AINN = AIN3
    ADS1118_MUX_A0_GND = 0x04, // AINP = AIN0, AINN = GND
    ADS1118_MUX_A1_GND = 0x05, // AINP = AIN1, AINN = GND
    ADS1118_MUX_A2_GND = 0x06, // AINP = AIN2, AINN = GND
    ADS1118_MUX_A3_GND = 0x07  // AINP = AIN3, AINN = GND
} ads1118_mux_t;

// ADS1118 Programmable Gain Amplifier Configuration
typedef enum {
    ADS1118_PGA_6_144V = 0x00, // ±6.144V (default)
    ADS1118_PGA_4_096V = 0x01, // ±4.096V
    ADS1118_PGA_2_048V = 0x02, // ±2.048V
    ADS1118_PGA_1_024V = 0x03, // ±1.024V
    ADS1118_PGA_0_512V = 0x04, // ±0.512V
    ADS1118_PGA_0_256V = 0x05  // ±0.256V
} ads1118_pga_t;

// ADS1118 Data Rate Configuration
typedef enum {
    ADS1118_DR_8SPS = 0x00,   // 8 samples per second
    ADS1118_DR_16SPS = 0x01,  // 16 samples per second
    ADS1118_DR_32SPS = 0x02,  // 32 samples per second
    ADS1118_DR_64SPS = 0x03,  // 64 samples per second
    ADS1118_DR_128SPS = 0x04, // 128 samples per second (default)
    ADS1118_DR_250SPS = 0x05, // 250 samples per second
    ADS1118_DR_475SPS = 0x06, // 475 samples per second
    ADS1118_DR_860SPS = 0x07  // 860 samples per second
} ads1118_data_rate_t;

// ADS1118 Operating Mode
typedef enum {
    ADS1118_MODE_CONTINUOUS = 0x00, // Continuous conversion mode
    ADS1118_MODE_SINGLE_SHOT = 0x01 // Power-down single-shot mode (default)
} ads1118_mode_t;

// ADS1118 Configuration Structure
typedef struct {
    ads1118_mux_t mux;                // Input multiplexer configuration
    ads1118_pga_t pga;                // Programmable gain amplifier
    ads1118_mode_t mode;              // Operating mode
    ads1118_data_rate_t data_rate;    // Data rate
    bool temperature_mode;            // Temperature sensor mode
    bool pullup_enable;               // Internal pullup enable on DOUT
} ads1118_config_t;

// ADS1118 Device Handle
typedef struct {
    spi_device_handle_t spi_device;   // SPI device handle
    ads1118_config_t config;          // Current configuration
    gpio_num_t cs_pin;                // Chip select pin
    float voltage_per_bit;            // Voltage per bit for current PGA setting
} ads1118_handle_t;

// Current Sensor Configuration for ACS712-20A
typedef struct {
    float sensitivity_v_per_a;        // Sensor sensitivity (V/A) - 0.1V/A for ACS712-20A
    float zero_current_voltage;       // Voltage at zero current - 2.5V for ACS712
    float max_current_a;              // Maximum current rating - 20A for ACS712-20A
} current_sensor_config_t;

/**
 * @brief Initialize ADS1118 ADC
 * @param handle Pointer to ADS1118 handle structure
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ads1118_init(ads1118_handle_t *handle, const ads1118_config_t *config);

/**
 * @brief Deinitialize ADS1118 ADC
 * @param handle Pointer to ADS1118 handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ads1118_deinit(ads1118_handle_t *handle);

/**
 * @brief Read raw ADC value from specified channel
 * @param handle Pointer to ADS1118 handle structure
 * @param mux Channel multiplexer setting
 * @param raw_value Pointer to store raw 16-bit ADC value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ads1118_read_raw(ads1118_handle_t *handle, ads1118_mux_t mux, int16_t *raw_value);

/**
 * @brief Read voltage from specified channel
 * @param handle Pointer to ADS1118 handle structure
 * @param mux Channel multiplexer setting
 * @param voltage_v Pointer to store voltage value in volts
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ads1118_read_voltage(ads1118_handle_t *handle, ads1118_mux_t mux, float *voltage_v);

/**
 * @brief Read current from ACS712 sensor connected to specified channel
 * @param handle Pointer to ADS1118 handle structure
 * @param mux Channel multiplexer setting
 * @param sensor_config Pointer to current sensor configuration
 * @param current_a Pointer to store current value in amperes
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ads1118_read_current(ads1118_handle_t *handle, ads1118_mux_t mux, 
                               const current_sensor_config_t *sensor_config, 
                               float *current_a);

/**
 * @brief Read temperature from internal temperature sensor
 * @param handle Pointer to ADS1118 handle structure
 * @param temperature_c Pointer to store temperature value in Celsius
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ads1118_read_temperature(ads1118_handle_t *handle, float *temperature_c);

/**
 * @brief Set ADS1118 configuration
 * @param handle Pointer to ADS1118 handle structure
 * @param config Pointer to new configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ads1118_set_config(ads1118_handle_t *handle, const ads1118_config_t *config);

/**
 * @brief Get default configuration for current monitoring
 * @param config Pointer to configuration structure to fill
 */
void ads1118_get_default_config(ads1118_config_t *config);

/**
 * @brief Get default ACS712-20A current sensor configuration
 * @param sensor_config Pointer to sensor configuration structure to fill
 */
void ads1118_get_acs712_20a_config(current_sensor_config_t *sensor_config);

#ifdef __cplusplus
}
#endif

#endif // ADS1118_DRIVER_H
