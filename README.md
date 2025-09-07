# ESP32-C3 Air101-LCD LVGL Demo

A complete ESP-IDF implementation of an LVGL-based user interface running on ESP32-C3 with Air101-LCD display. This professional embedded development project demonstrates custom ST7735 driver integration, joystick controls, and interactive UI widgets.

## 🎯 Features

- **LVGL Integration**: Modern embedded GUI with widgets and animations
- **Native ESP-IDF**: Professional embedded framework implementation
- **ST7735 Display Driver**: Custom SPI display driver optimized for 80x160 LCD
- **Interactive Controls**: 5-way joystick navigation with cursor positioning
- **Color Display**: 16-bit RGB565 color support with proper color mapping
- **FreeRTOS Integration**: Multi-task architecture with proper timing
- **Hardware Optimization**: Direct GPIO and SPI control for maximum performance

## 🔧 Hardware Setup

### Components
- **ESP32-C3 DevKit M-1** - Main microcontroller board
- **Air101-LCD Display Module** (ST7735 controller, 80x160 pixels)
- **5-way Joystick Module** (Left, Right, Up, Down, Center)

### ESP32-C3 Board Pinout

The ESP32-C3 board provides the following GPIO pins and power connections:

#### Power Pins
- **3.3V** (Pin 13, 26, 31) - 3.3V regulated output
- **5V** (Pin 16, 32) - 5V input/output (when powered via USB)
- **GND** (Pin 1, 17, 25) - Ground connections

#### Digital GPIO Pins
- **GPIO0-GPIO10** - General purpose I/O pins
- **GPIO18, GPIO19** - Additional GPIO pins
- **GPIO20, GPIO21** - SPI pins (can also be used as GPIO)

#### Special Function Pins
- **ADC0/GPIO0** (Pin 02) - Analog input
- **ADC1/GPIO1** (Pin 03) - Analog input  
- **SPI_CLK/GPIO20** (Pin 23) - Hardware SPI clock
- **SPI_MOSI/GPIO21** (Pin 24) - Hardware SPI MOSI
- **SPI_MISO/GPIO19** (Pin 22) - Hardware SPI MISO
- **SPI_CS/GPIO10** (Pin 21) - Hardware SPI chip select
- **I2C_SCL/GPIO5** (Pin 27) - I2C clock
- **I2C_SDA/GPIO4** (Pin 28) - I2C data
- **UART_TX/GPIO21** - Serial transmit
- **UART_RX/GPIO20** - Serial receive

### Air101-LCD Display Module Pinout

The Air101-LCD module includes both the ST7735 display and 5-way joystick:

#### Display Connections (ST7735)
- **GND** (Pin 1) - Ground
- **3.3V** (Pin 2) - Power supply
- **LKEY** (Pin 3) - Backlight control (not used)
- **UPKEY** (Pin 4) - Up button
- **CENTER** (Pin 5) - Center button  
- **DWKEY** (Pin 6) - Down button
- **3.3V** (Pin 7) - Power supply
- **GND** (Pin 8) - Ground
- **BL** (Pin 9) - Backlight positive
- **RS** (Pin 10) - Reset
- **DC** (Pin 11) - Data/Command
- **RES** (Pin 12) - Reset (duplicate)
- **SDA** (Pin 13) - SPI MOSI (Data)
- **SCL** (Pin 14) - SPI Clock
- **3.3V** (Pin 15) - Power supply
- **GND** (Pin 16) - Ground
- **RKEY** (Pin 23) - Right button

### Wiring Connections

| Function | ESP32-C3 Pin | Air101-LCD Pin | Wire Color Suggestion |
|----------|--------------|----------------|----------------------|
| **Power Connections** |
| VCC | 3.3V (Pin 13) | 3.3V (Pin 2) | Red |
| Ground | GND (Pin 1) | GND (Pin 1) | Black |
| **Display SPI** |
| SPI Clock | GPIO 2 | SCL (Pin 14) | Yellow |
| SPI Data | GPIO 3 | SDA (Pin 13) | Blue |
| Chip Select | GPIO 7 | CS (connect to 3.3V) | Green |
| Data/Command | GPIO 6 | DC (Pin 11) | Purple |
| Reset | GPIO 10 | RS (Pin 10) | Orange |
| **Joystick Controls** |
| Left Button | GPIO 8 | LKEY (Pin 3) | Gray |
| Up Button | GPIO 9 | UPKEY (Pin 4) | White |
| Center Button | GPIO 4 | CENTER (Pin 5) | Brown |
| Down Button | GPIO 5 | DWKEY (Pin 6) | Pink |
| Right Button | GPIO 13 | RKEY (Pin 23) | Cyan |

### Assembly Notes

1. **Power Supply**: The Air101-LCD module operates at 3.3V. Connect multiple 3.3V and GND pins for stable power distribution.

2. **SPI Configuration**: This project uses software SPI implementation for maximum control. The CS (Chip Select) pin can be tied to 3.3V since we're the only SPI device.

3. **Button Configuration**: All buttons are configured with internal pull-up resistors. Buttons read LOW when pressed.

4. **Display Orientation**: The 80x160 pixel display is oriented in portrait mode with proper offset configuration for the ST7735 controller.

5. **Wire Management**: Use short, direct connections to minimize noise and ensure reliable communication at 20MHz SPI clock speed.

## 🚀 Quick Start

### Prerequisites
- [PlatformIO](https://platformio.org/)
- ESP32-C3 hardware setup as described above

### Build & Flash
```bash
# Clone the repository
git clone https://github.com/brunoamui/air101-lcd-lvgl.git
cd air101-lcd-lvgl

# Initialize ESP-IDF environment (if using ESP-IDF directly)
idf.py build

# Flash to device
idf.py -p /dev/cu.usbmodem1412401 flash

# Monitor serial output
idf.py -p /dev/cu.usbmodem1412401 monitor
```

**Alternative with PlatformIO:**
```bash
# If using PlatformIO instead
pio run
pio run --target upload
pio device monitor --baud 115200
```

## 🎮 Demo Controls

- **Directional Pad**: Navigate UI cursor around the screen
- **Center Button**: Click/press UI elements (buttons, widgets)
- **Interactive Elements**: 
  - "Press Me" button - cycles through different background colors
  - Progress bar - updates with each button click
  - Cursor position display - shows real-time X,Y coordinates
- **Visual Feedback**: Screen background changes color with each interaction

## 🏗️ Architecture

### Project Structure
```
├── src/
│   └── main.c          # Main ESP-IDF application
├── platformio.ini      # PlatformIO configuration
├── .gitignore         # Git exclusions
└── README.md          # This file
```

### Key Components

#### LVGL Integration
- **Widget System**: Labels, buttons, progress bars, and interactive elements
- **Event Handling**: Touch/click event processing and callbacks
- **Theme Support**: Default LVGL themes with custom color schemes
- **Animation Engine**: Smooth transitions and visual feedback

#### Display System
- **ST7735 Driver**: Custom SPI implementation optimized for 80x160 displays
- **Color Management**: Proper RGB565 color format with byte-swapping control
- **Buffer Management**: Efficient partial screen updates
- **Flush Optimization**: Direct SPI transfer without intermediate buffers

#### Input System
- **Joystick Navigation**: 5-way directional control with cursor positioning
- **Button Debouncing**: Hardware debouncing with 100ms movement delay
- **LVGL Input Device**: Integrated pointer device for UI interaction
- **Real-time Feedback**: Live cursor coordinate display

## 📊 Performance Metrics

- **Display Resolution**: 80x160 pixels (12,800 total pixels)
- **Color Depth**: 16-bit RGB565 (65,536 colors)
- **SPI Clock Speed**: 20MHz for fast display updates
- **LVGL Task**: 5ms update cycle with dynamic timing
- **Input Response**: 100ms cursor movement with smooth navigation
- **Memory Efficiency**: Optimized buffer usage for small display size

## 🔄 Development Journey

This project demonstrates professional embedded GUI development:

1. **ESP-IDF Foundation**: Built on native ESP-IDF framework for reliability
2. **LVGL Integration**: Modern embedded graphics library with component manager
3. **Custom Driver Development**: Tailored ST7735 driver for optimal performance
4. **Color System Debugging**: Resolved RGB565 byte-swapping issues for correct colors
5. **Professional Structure**: Multi-task FreeRTOS architecture with proper resource management

## 🛠️ Technical Details

### SPI Configuration
- **Bus**: SPI2_HOST with DMA enabled
- **Speed**: 20MHz clock frequency for fast display updates
- **Mode**: SPI Mode 0 (CPOL=0, CPHA=0) 
- **Transfer Size**: Up to 4096 bytes with DMA
- **Color Format**: RGB565 with byte-swapping disabled for correct colors

### GPIO Configuration
- **Display Control**: Output pins for RST and DC
- **Button Inputs**: Pull-up enabled with interrupt capability
- **Proper Initialization**: Hardware abstraction layer configuration

### LVGL Integration
- **LVGL Task**: GUI updates with adaptive timing (5-10ms cycles)
- **Input Task**: Joystick monitoring and coordinate updates  
- **Timer System**: ESP timer for precise 1ms LVGL tick increments
- **Memory Management**: DMA-capable buffer allocation for display data
- **Task Priorities**: Balanced priority system for smooth UI performance

## 🐛 Troubleshooting

### Common Issues
1. **Display not showing**: Check SPI connections and power supply
2. **Buttons not responding**: Verify GPIO connections and pull-ups
3. **Build errors**: Ensure ESP-IDF framework is properly installed

### Serial Debug Output
The system provides comprehensive logging:
- Hardware initialization status
- Button press events with coordinates
- UI widget interactions and callbacks
- System performance metrics
- Memory usage and heap monitoring

## 📝 License

This project is open source and available under the MIT License.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## 🙏 Acknowledgments

- Based on the working blog post implementation for Air101-LCD
- Uses ESP-IDF framework by Espressif Systems
- Built with PlatformIO for cross-platform development
