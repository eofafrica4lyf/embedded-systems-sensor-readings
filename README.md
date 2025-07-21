# STM32G071 UART Command Console with LoRa (SX126x) and BME68x Sensor

## Overview
This project is a robust, ready-to-use firmware for the STM32G071 microcontroller, providing a UART command-line interface to:
- Read temperature, pressure, and humidity from a Bosch BME68x sensor (I²C)
- Transmit LoRa messages using an SX126x LoRa HAT (SPI)
- Perform simple arithmetic and system tests

---

## Features
- **UART Command Console**: Interact with the device using a serial terminal (e.g., PuTTY, Tera Term)
- **BME68x Sensor Support**: Read environmental data on demand
- **LoRa SX126x Support**: Send and scan LoRa messages at 868 MHz
- **Non-blocking LoRa Scan**: Start/stop continuous LoRa transmissions without freezing the console
- **Robust Error Handling**: Detects missing hardware, busy states, and sensor errors

---

## Hardware Requirements
- **STM32G071 Nucleo or custom board**
- **SX126x LoRa HAT/module** (SPI interface, e.g., EBYTE E22/E220, or similar)
- **Bosch BME68x sensor** (I²C interface)
- **Wiring**:
  - **BME68x**: Connect to STM32 I²C1 (SCL/SDA, 3.3/5.0V, GND)
  - **LoRa HAT**:
    - SPI: SCK, MISO, MOSI, NSS (CS)
    - Control: RESET, BUSY
    - Power: 3.3/5.0V, GND
  - **UART2**: For user console (connect to PC via USB-UART adapter)

### Example Pin Mapping
| STM32 Pin | BME68x | LoRa HAT (SX126x) | UART2 (Console) |
|-----------|--------|-------------------|-----------------|
| PB8/PB9   | SCL/SDA|                   |                 |
| PA5/PA6/PA7|       | SCK/MISO/MOSI     |                 |
| PB0       |        | NSS (CS)          |                 |
| PB5       |        | RESET             |                 |
| PB11      |        | BUSY              |                 |
| PA2/PA3   |        |                   | TX/RX           |
| 3.3V/GND  | VCC/GND| VCC/GND           | GND             |
If 3.3V doesn't work for you, try 5.0V
---

## Software Setup
1. **Clone or Download this Repository**
2. **Open in STM32CubeIDE** (or compatible tool)
3. **Check/Adjust Pin Assignments** in `main.c` and CubeMX `.ioc` file to match your hardware
4. **Build the Project**
5. **Flash the Firmware** to your STM32G071 board
6. **Connect Serial Terminal** to UART2 (115200 baud, 8N1)

---

## Usage: UART Command Console
After reset, you’ll see:
```
[STM32] UART+BME68x Ready
> 
```
Type commands and press Enter. Supported commands:

| Command         | Description                                 | Example Output                  |
|-----------------|---------------------------------------------|---------------------------------|
| `start`         | Show available commands                     | List of commands                |
| `temp`          | Read temperature from BME68x                | `[STM32] Temperature: 23.45 °C` |
| `pressure`      | Read pressure from BME68x                   | `[STM32] Pressure: 1013.25 hPa` |
| `humidity`      | Read humidity from BME68x                   | `[STM32] Humidity: 45.67 %`     |
| `add x y`       | Add two numbers (float)                     | `[STM32] 5.5 + 3.2 = 8.7`       |
| `lora tx`       | Send a LoRa message ("Hello from STM32 LoRa!") | LoRa TX status messages         |

---

## LoRa Operation
- **lora tx**: Sends a single LoRa message at 868 MHz. Check your receiver for the message.
- **Hardware detection**: The firmware checks for LoRa hardware before each transmission and reports errors if not detected.

---

## Troubleshooting
- **No output on serial terminal**: Check UART2 wiring and baud rate.
- **Sensor read failed**: Check BME68x wiring and power. Switch Vcc to 5.0V if 3.3V doesn't work.
- **LoRa module not detected**: Check SPI wiring (SCK, MISO, MOSI, NSS), RESET, BUSY, and power. Ensure LoRa HAT is powered and not held in reset.
- **LoRa TX always "succeeds" even if module is missing**: The firmware checks for hardware presence, but SPI cannot always detect a missing device. Double-check wiring and use a receiver to confirm transmission.
- **Compile errors**: Ensure all STM32 HAL headers are included and CubeMX-generated files are present.

---

## Support
For questions, improvements, or bug reports, please open an issue or contact the project maintainer.

---

**Enjoy rapid prototyping with STM32, LoRa, and BME68x!** 