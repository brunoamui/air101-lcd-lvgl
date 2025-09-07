# ESP32-C3 Air101-LCD Snake Game

A complete ESP-IDF implementation of a snake game running on ESP32-C3 with Air101-LCD display, converted from Arduino framework to native ESP-IDF for professional embedded development.

## 🎯 Features

- **Native ESP-IDF**: Professional embedded framework implementation
- **ST7735 Display Driver**: Custom SPI display driver with DMA support
- **Ultra-responsive Controls**: 60ms button response with debouncing
- **Snake Game**: Complete game with collision detection and goal system
- **FreeRTOS Integration**: Proper task-based architecture
- **Hardware Optimization**: Direct GPIO and SPI control for maximum performance

## 🔧 Hardware Setup

### Components
- **ESP32-C3 DevKit M-1**
- **Air101-LCD Display** (ST7735 controller)
- **5-way Joystick** (Left, Right, Up, Down, Center)

### Pin Configuration

| Function | ESP32-C3 GPIO | Description |
|----------|---------------|-------------|
| SPI SCLK | GPIO 2 | Display Clock |
| SPI MOSI | GPIO 3 | Display Data |
| TFT RST  | GPIO 10 | Display Reset |
| TFT DC   | GPIO 6 | Data/Command |
| TFT CS   | GPIO 7 | Chip Select |
| BTN LEFT | GPIO 8 | Joystick Left |
| BTN UP   | GPIO 9 | Joystick Up |
| BTN CENTER | GPIO 4 | Joystick Center |
| BTN DOWN | GPIO 5 | Joystick Down |
| BTN RIGHT | GPIO 13 | Joystick Right |

## 🚀 Quick Start

### Prerequisites
- [PlatformIO](https://platformio.org/)
- ESP32-C3 hardware setup as described above

### Build & Flash
```bash
# Clone the repository
git clone <your-repo-url>
cd air101-lcd-test

# Build the project
pio run

# Flash to device
pio run --target upload --upload-port /dev/cu.usbmodem1412401

# Monitor serial output
pio device monitor --port /dev/cu.usbmodem1412401 --baud 115200
```

## 🎮 Game Controls

- **Directional Pad**: Move the blue player dot
- **Center Button**: Reset screen and generate new goal
- **Objective**: Navigate to the orange goal rectangle
- **Screen Wrapping**: Player wraps around screen edges

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

#### Display System
- **ST7735 Driver**: Native SPI implementation with proper initialization sequence
- **Graphics Functions**: Pixel, rectangle, and circle drawing primitives
- **Color Support**: 16-bit RGB565 color space
- **DMA Integration**: Hardware-accelerated SPI transfers

#### Input System
- **GPIO Configuration**: Pull-up enabled inputs with proper debouncing
- **50ms Debounce**: Prevents double-presses and contact bounce
- **10ms Polling**: Ultra-responsive button detection
- **Real-time Processing**: FreeRTOS task-based input handling

#### Game Logic
- **Phase System**: Demo phases followed by continuous gameplay
- **Collision Detection**: Goal detection with pixel-perfect accuracy
- **Random Generation**: Cryptographically secure random goal placement
- **Boundary Handling**: Smooth screen wrapping

## 📊 Performance Metrics

- **Button Response Time**: ~60ms (50ms debounce + 10ms polling)
- **Display Refresh**: Real-time with hardware SPI DMA
- **Memory Usage**: 2.7% RAM (8,976 bytes of 327,680 bytes)
- **Flash Usage**: 20.7% Flash (217,414 bytes of 1,048,576 bytes)
- **Task Performance**: 100Hz game loop (10ms per cycle)

## 🔄 Development Journey

This project demonstrates a complete conversion from Arduino framework to ESP-IDF:

1. **Arduino Version**: Started with Adafruit libraries and Arduino IDE-style code
2. **ESP-IDF Conversion**: Migrated to native ESP-IDF with custom drivers
3. **Performance Optimization**: Implemented proper debouncing and timing
4. **Professional Structure**: FreeRTOS tasks, proper error handling, logging

## 🛠️ Technical Details

### SPI Configuration
- **Bus**: SPI2_HOST with DMA enabled
- **Speed**: 10MHz clock frequency
- **Mode**: SPI Mode 0 (CPOL=0, CPHA=0)
- **Transfer Size**: Up to 4096 bytes with DMA

### GPIO Configuration
- **Display Control**: Output pins for RST and DC
- **Button Inputs**: Pull-up enabled with interrupt capability
- **Proper Initialization**: Hardware abstraction layer configuration

### FreeRTOS Integration
- **Main Task**: Display and game logic at 100Hz
- **Logging System**: ESP-IDF logging with timestamps
- **Memory Management**: Proper task stack allocation (4096 bytes)
- **Watchdog Handling**: Prevents system resets with proper delays

## 🐛 Troubleshooting

### Common Issues
1. **Display not showing**: Check SPI connections and power supply
2. **Buttons not responding**: Verify GPIO connections and pull-ups
3. **Build errors**: Ensure ESP-IDF framework is properly installed

### Serial Debug Output
The system provides comprehensive logging:
- Phase transitions
- Button press events
- Goal achievements
- System initialization status

## 📝 License

This project is open source and available under the MIT License.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## 🙏 Acknowledgments

- Based on the working blog post implementation for Air101-LCD
- Uses ESP-IDF framework by Espressif Systems
- Built with PlatformIO for cross-platform development
