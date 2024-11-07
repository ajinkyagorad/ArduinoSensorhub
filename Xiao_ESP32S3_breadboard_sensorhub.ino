#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h> // Library for AHT21

#define TFT_CS   1
#define TFT_RST  3
#define TFT_DC   2
#define LED_PIN  D0
#define NUMPIXELS 64

// Initialize display and NeoPixel
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Initialize sensors
VL53L0X vl53;
MPU6050 mpu;
Adafruit_BMP280 bmp;
DFRobot_ENS160_I2C ens160(&Wire, 0x53); // Use I2C for ENS160
Adafruit_AHTX0 aht; // Initialize AHT21 sensor

// Use HardwareSerial for GPS on ESP32
#define gpsSerial Serial1

void scanI2CDevices() {
  Serial.println("Scanning I2C devices...");
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("!");
      count++;
    }
  }
  if (count == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.println("I2C scan complete");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Scan for I2C devices
  scanI2CDevices();

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(10); // Set brightness to a lower value
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Indicate Xiao is on
  pixels.show();

  // Initialize TFT
  tft.init(170, 320); // Initialize with width and height
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  pixels.setPixelColor(2, pixels.Color(0, 255, 0)); // Indicate TFT initialized
  pixels.show();
  Serial.println("TFT initialized");

  // Initialize VL53L0X
  if (vl53.init()) {
    pixels.setPixelColor(3, pixels.Color(0, 255, 0)); // VL53L0X works
    Serial.println("VL53L0X initialized");
  } else {
    pixels.setPixelColor(3, pixels.Color(255, 0, 0)); // VL53L0X failed
    Serial.println("VL53L0X initialization failed");
  }
  pixels.show();

  // Initialize MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    pixels.setPixelColor(4, pixels.Color(0, 255, 0)); // MPU6050 works
    Serial.println("MPU6050 initialized");
  } else {
    pixels.setPixelColor(4, pixels.Color(255, 0, 0)); // MPU6050 failed
    Serial.println("MPU6050 initialization failed");
  }
  pixels.show();

  // Initialize BMP280
  if (bmp.begin(0x76)) {
    pixels.setPixelColor(5, pixels.Color(0, 255, 0)); // BMP280 works
    Serial.println("BMP280 initialized at 0x76");
  } else if (bmp.begin(0x77)) {
    pixels.setPixelColor(5, pixels.Color(0, 255, 0)); // BMP280 works
    Serial.println("BMP280 initialized at 0x77");
  } else {
    pixels.setPixelColor(5, pixels.Color(255, 0, 0)); // BMP280 failed
    Serial.println("BMP280 initialization failed");
  }
  pixels.show();

  // Initialize ENS160
  while (ens160.begin() != NO_ERR) {
    Serial.println("Communication with ENS160 failed, retrying...");
    pixels.setPixelColor(6, pixels.Color(255, 0, 0)); // ENS160 failed
    pixels.show();
    delay(3000);
  }
  Serial.println("ENS160 initialized successfully!");
  pixels.setPixelColor(6, pixels.Color(0, 255, 0)); // ENS160 works
  pixels.show();

  // Initialize AHT21
  if (aht.begin()) {
    pixels.setPixelColor(7, pixels.Color(0, 255, 0)); // AHT21 works
    Serial.println("AHT21 initialized successfully!");
  } else {
    pixels.setPixelColor(7, pixels.Color(255, 0, 0)); // AHT21 failed
    Serial.println("AHT21 initialization failed");
  }
  pixels.show();

  // Initialize GPS
  gpsSerial.begin(9600);
  pixels.setPixelColor(8, pixels.Color(0, 255, 0)); // GPS assumed initialized
  Serial.println("GPS initialized");
  pixels.show();
}

void loop() {
  // Clear the display for new data
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);

  // Read VL53L0X distance
  tft.print("VL53L0X Distance: ");
  int vl53Distance = vl53.readRangeSingleMillimeters();
  tft.println(vl53Distance);
  Serial.print("VL53L0X Distance: ");
  Serial.println(vl53Distance);

  // Read MPU6050 data
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  tft.print("MPU6050 Accel X: ");
  tft.println(ax);
  tft.print("MPU6050 Accel Y: ");
  tft.println(ay);
  tft.print("MPU6050 Accel Z: ");
  tft.println(az);
  Serial.print("MPU6050 Accel X: ");
  Serial.println(ax);
  Serial.print("MPU6050 Accel Y: ");
  Serial.println(ay);
  Serial.print("MPU6050 Accel Z: ");
  Serial.println(az);

  // Read BMP280 data
  float bmpTemp = bmp.readTemperature();
  float bmpPressure = bmp.readPressure();
  tft.print("BMP280 Temp: ");
  tft.println(bmpTemp);
  tft.print("BMP280 Pressure: ");
  tft.println(bmpPressure);
  Serial.print("BMP280 Temp: ");
  Serial.println(bmpTemp);
  Serial.print("BMP280 Pressure: ");
  Serial.println(bmpPressure);

  // Read ENS160 data
  uint16_t eco2 = ens160.getECO2();
  uint16_t tvoc = ens160.getTVOC();
  tft.print("ENS160 eCO2: ");
  tft.println(eco2);
  tft.print("ENS160 TVOC: ");
  tft.println(tvoc);
  Serial.print("ENS160 eCO2: ");
  Serial.println(eco2);
  Serial.print("ENS160 TVOC: ");
  Serial.println(tvoc);

  // Read AHT21 data
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  tft.print("AHT21 Temp: ");
  tft.println(temp.temperature);
  tft.print("AHT21 Humidity: ");
  tft.println(humidity.relative_humidity);
  Serial.print("AHT21 Temp: ");
  Serial.println(temp.temperature);
  Serial.print("AHT21 Humidity: ");
  Serial.println(humidity.relative_humidity);

  // Read GPS data
  if (gpsSerial.available() > 0) {
    String gpsData = gpsSerial.readStringUntil('\n');
    tft.print("GPS Data: ");
    tft.println(gpsData);
    Serial.print("GPS Data: ");
    Serial.println(gpsData);
  } else {
    tft.println("GPS Data: No data available");
    Serial.println("No GPS data available");
  }

  // Update LED indicator
  pixels.show();

  // Delay before next reading
  delay(200);
}
