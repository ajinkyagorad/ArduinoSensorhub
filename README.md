# 📚 Sensor Hub Project
![working](https://github.com/ajinkyagorad/ArduinoSensorhub/blob/ebeee03ea8cb49a070fc15688e30c65e97f7ba0e/setup.jpg)


## 🎯 Overview
This project is a comprehensive sensor hub using the Xiao ESP32S3 microcontroller. It integrates various sensors, an 8x8 WS2812 LED matrix, and a high-resolution 1.9-inch ST7789 TFT display.

## 🔧 Components Used
- **Xiao ESP32S3** 🖥️
- **8x8 WS2812 LED Matrix** 💡
- **ST7789 1.9-inch TFT Display** 🖼️
- **VL53L0X Distance Sensor** 📏
- **MPU6050 Accelerometer** 🧭
- **BMP280 Temperature & Pressure Sensor** 🌡️
- **ENS160 eCO2 & TVOC Sensor** 🏭
- **AHT21 Temperature & Humidity Sensor** 💧
- **GPS Module** 📡

## 🛠️ Setup
1. **Connect all components** as per the provided wiring diagram.
2. **Upload the code** to your Xiao ESP32S3 using the Arduino IDE.
3. **Install the required libraries**:
   - `Adafruit_GFX`
   - `Adafruit_ST7789`
   - `Adafruit_NeoPixel`
   - `VL53L0X`
   - `MPU6050`
   - `Adafruit_BMP280`
   - `DFRobot_ENS160`
   - `Adafruit_AHTX0`

## 🚀 Functionality
- The hub reads and displays data from all connected sensors.
- Sensor statuses are shown on the 8x8 LED matrix:
  - 🟢 Green: Working
  - 🔴 Red: Failed
  - 🔵 Blue: Initializing
- Data output is shown on both the TFT display and Serial Monitor.

## 📷 Display Content
- Distance (VL53L0X)
- Acceleration (MPU6050)
- Temperature & Pressure (BMP280)
- eCO2 & TVOC (ENS160)
- Temperature & Humidity (AHT21)
- GPS Data

## 🤖 Notes
- Ensure proper power supply for the WS2812 LED matrix.
- Adjust the brightness if needed.
- Verify the GPS module's baud rate for compatibility.

Enjoy your sensor hub! 🌟

