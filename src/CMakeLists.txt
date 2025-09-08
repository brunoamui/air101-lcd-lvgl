# Main component for Air101-LCD LVGL demo
# Simple direct approach

idf_component_register(
    SRCS "ads1118_driver.c" "main.c"
    INCLUDE_DIRS "." "../include"
    REQUIRES 
        driver 
        esp_timer 
        freertos 
        esp_common
        spi_flash
        heap
)
