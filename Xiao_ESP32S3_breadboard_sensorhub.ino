#include <Wire.h>
#include <Adafruit_GFX.h>           // Core graphics library
#include <Adafruit_ST7789.h>        // Hardware-specific library for ST7789
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>     // Library for NeoMatrix
#include <VL53L0X.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <string.h>
// ----------------------------- Pin Definitions -----------------------------
// TFT Display Pins
#define TFT_CS     5    // GPIO5 (D1 on some boards)
#define TFT_RST    16   // GPIO16 (D3 on some boards)
#define TFT_DC     17   // GPIO17 (D2 on some boards)

// NeoPixel Ring Pins
#define NEO_PIXEL_PIN 4    // GPIO4 (D0 on some boards)
#define NUMPIXELS 64       // Number of NeoPixels

// NeoMatrix Pins
#define NEOMATRIX_PIN 12   // GPIO12 (You can change this if needed)
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8

// GPS Module Pins (Assuming connected to Serial1)
#define GPS_RX_PIN  27    // GPIO27 (RX)
#define GPS_TX_PIN  26    // GPIO26 (TX)

// ----------------------------- Wi-Fi Credentials -----------------------------
const char* ssid = "Triton9";
const char* password = "88888888";

// ----------------------------- Web Server Setup -----------------------------
WebServer server(80);

// ----------------------------- NTP Client Setup -----------------------------
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC Time, update every 60 seconds

// ----------------------------- Initialize Display and LEDs -----------------------------
// Initialize TFT Display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Initialize NeoPixel Ring
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Initialize NeoMatrix
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(MATRIX_WIDTH, MATRIX_HEIGHT, NEOMATRIX_PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
  NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB            + NEO_KHZ800);

// ----------------------------- Initialize Sensors -----------------------------
VL53L0X vl53;
MPU6050 mpu;
Adafruit_BMP280 bmp;
DFRobot_ENS160_I2C ens160(&Wire, 0x53); // ENS160 I2C address
Adafruit_AHTX0 aht;

// Initialize GPS Serial
HardwareSerial gpsSerial(1); // Using Serial1
// Assign RX and TX pins if necessary (depends on your ESP32 board)
// For some ESP32 boards, you might need to specify the RX and TX pins
// gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

// ----------------------------- Buffer Sizes -----------------------------
#define BUFFER_SIZE 100

// Data Structures for Sensor Data
struct VL53Data {
  unsigned long timestamp; // Epoch time
  int distance_mm;
  unsigned long readTime_us;
} vl53DataBuffer[BUFFER_SIZE];
int vl53Index = 0;

struct MPU6050Data {
  unsigned long timestamp;
  int16_t ax, ay, az;
  unsigned long readTime_us;
} mpuDataBuffer[BUFFER_SIZE];
int mpuIndex = 0;

struct BMP280Data {
  unsigned long timestamp;
  float temperature_C;
  float pressure_Pa;
  unsigned long readTime_us;
} bmpDataBuffer[BUFFER_SIZE];
int bmpIndex = 0;

struct ENS160Data {
  unsigned long timestamp;
  uint16_t eco2_ppm;
  uint16_t tvoc_ppb;
  unsigned long readTime_us;
} ens160DataBuffer[BUFFER_SIZE];
int ens160Index = 0;

struct AHT21Data {
  unsigned long timestamp;
  float temperature_C;
  float humidity_percent;
  unsigned long readTime_us;
} aht21DataBuffer[BUFFER_SIZE];
int aht21Index = 0;

// ----------------------------- Timer Variables -----------------------------
unsigned long previousSensorRead = 0;
unsigned long previousDisplayUpdate = 0;
unsigned long previousTimeUpdate = 0;

// Intervals in milliseconds
const unsigned long SENSOR_READ_INTERVAL = 1000;     // 1 second
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000;  // 1 second
const unsigned long TIME_UPDATE_INTERVAL = 60000;    // 60 seconds

// ----------------------------- Function Prototypes -----------------------------
void scanI2CDevices();
void addVL53Data(int distance, unsigned long readTime);
void addMPU6050Data(int16_t ax, int16_t ay, int16_t az, unsigned long readTime);
void addBMP280Data(float temp, float pressure, unsigned long readTime);
void addENS160Data(uint16_t eco2, uint16_t tvoc, unsigned long readTime);
void addAHT21Data(float temp, float humidity, unsigned long readTime);
String getJSONData();
void handleRoot();
void displayHeader(const char* header, uint16_t color);
void displayValue(const char* label, String value, uint16_t color, int y);

// ----------------------------- Setup Function -----------------------------
void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  Wire.begin();

  // Initialize GPS Serial (Assuming GPS is connected to Serial1)
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize TFT Display
  tft.init(240, 240); // Initialize with width and height (adjust as per your TFT)
  tft.setRotation(2); // Rotate display as needed
  tft.fillScreen(ST77XX_BLACK);
  displayHeader("Initializing...", ST77XX_GREEN);
  delay(1000); // Brief delay to show the header

  // Initialize NeoPixel Ring
  pixels.begin();
  pixels.setBrightness(10); // Set brightness to a lower value
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Indicate Device is On
  pixels.show();
  displayValue("NeoPixel", "Initialized", ST77XX_GREEN, 40);
  delay(500);

  // Initialize NeoMatrix
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(40); // Set brightness (0-255)
  matrix.fillScreen(0);
  matrix.show();
  displayValue("NeoMatrix", "Initialized", ST77XX_GREEN, 60);
  delay(500);

  // Initialize VL53L0X Sensor
  tft.fillRect(0, 80, tft.width(), 20, ST77XX_BLACK);
  tft.setCursor(0, 80);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.println("VL53L0X Init...");
  if (vl53.init()) {
    pixels.setPixelColor(3, pixels.Color(0, 255, 0)); // VL53L0X works
    tft.setCursor(0, 80);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.println("VL53L0X Init: Success");
    Serial.println("VL53L0X initialized");
  } else {
    pixels.setPixelColor(3, pixels.Color(255, 0, 0)); // VL53L0X failed
    tft.setCursor(0, 80);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.println("VL53L0X Init: Failed");
    Serial.println("VL53L0X initialization failed");
  }
  pixels.show();
  delay(500);

  // Initialize MPU6050 Sensor
  tft.fillRect(0, 100, tft.width(), 20, ST77XX_BLACK);
  tft.setCursor(0, 100);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("MPU6050 Init...");
  mpu.initialize();
  if (mpu.testConnection()) {
    pixels.setPixelColor(4, pixels.Color(0, 255, 0)); // MPU6050 works
    tft.setCursor(0, 100);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.println("MPU6050 Init: Success");
    Serial.println("MPU6050 initialized");
  } else {
    pixels.setPixelColor(4, pixels.Color(255, 0, 0)); // MPU6050 failed
    tft.setCursor(0, 100);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.println("MPU6050 Init: Failed");
    Serial.println("MPU6050 initialization failed");
  }
  pixels.show();
  delay(500);

  // Initialize BMP280 Sensor
  tft.fillRect(0, 120, tft.width(), 20, ST77XX_BLACK);
  tft.setCursor(0, 120);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("BMP280 Init...");
  if (bmp.begin(0x76)) {
    pixels.setPixelColor(5, pixels.Color(0, 255, 0)); // BMP280 works
    tft.setCursor(0, 120);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.println("BMP280 Init: Success @0x76");
    Serial.println("BMP280 initialized at 0x76");
  } else if (bmp.begin(0x77)) {
    pixels.setPixelColor(5, pixels.Color(0, 255, 0)); // BMP280 works
    tft.setCursor(0, 120);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.println("BMP280 Init: Success @0x77");
    Serial.println("BMP280 initialized at 0x77");
  } else {
    pixels.setPixelColor(5, pixels.Color(255, 0, 0)); // BMP280 failed
    tft.setCursor(0, 120);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.println("BMP280 Init: Failed");
    Serial.println("BMP280 initialization failed");
  }
  pixels.show();
  delay(500);

  // Initialize ENS160 Sensor
  tft.fillRect(0, 140, tft.width(), 20, ST77XX_BLACK);
  tft.setCursor(0, 140);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("ENS160 Init...");
  while (ens160.begin() != NO_ERR) {
    Serial.println("Communication with ENS160 failed, retrying...");
    pixels.setPixelColor(6, pixels.Color(255, 0, 0)); // ENS160 failed
    tft.setCursor(0, 140);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.println("ENS160 Init: Failed");
    pixels.show();
    delay(3000);
    tft.setCursor(0, 140);
    tft.print("ENS160 Init: Retry");
  }
  pixels.setPixelColor(6, pixels.Color(0, 255, 0)); // ENS160 works
  tft.setCursor(0, 140);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("ENS160 Init: Success");
  Serial.println("ENS160 initialized successfully!");
  pixels.show();
  delay(500);

  // Initialize AHT21 Sensor
  tft.fillRect(0, 160, tft.width(), 20, ST77XX_BLACK);
  tft.setCursor(0, 160);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("AHT21 Init...");
  if (aht.begin()) {
    pixels.setPixelColor(7, pixels.Color(0, 255, 0)); // AHT21 works
    tft.setCursor(0, 160);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.println("AHT21 Init: Success");
    Serial.println("AHT21 initialized successfully!");
  } else {
    pixels.setPixelColor(7, pixels.Color(255, 0, 0)); // AHT21 failed
    tft.setCursor(0, 160);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.println("AHT21 Init: Failed");
    Serial.println("AHT21 initialization failed");
  }
  pixels.show();
  delay(500);

  // Connect to Wi-Fi
  tft.fillRect(0, 180, tft.width(), 20, ST77XX_BLACK);
  tft.setCursor(0, 180);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  pixels.setPixelColor(1, pixels.Color(255, 255, 0)); // Indicate Wi-Fi Connecting
  pixels.show();
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");
  pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // Indicate Wi-Fi connected
  pixels.show();
  tft.setCursor(0, 180);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("Wi-Fi Connected");
  delay(500);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize NTP Client
  tft.fillRect(0, 200, tft.width(), 20, ST77XX_BLACK);
  tft.setCursor(0, 200);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("Syncing Time...");
  timeClient.begin();
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  Serial.println("Time synchronized via NTP");
  tft.setCursor(0, 200);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.println("Time Synced");
  delay(500);

  // Initialize Web Server Routes
  server.on("/", handleRoot);
  server.on("/data", HTTP_GET, [](){
    String json = getJSONData();
    server.send(200, "application/json", json);
  });

  // Start Web Server
  server.begin();
  Serial.println("HTTP server started");
}

// ----------------------------- Function Implementations -----------------------------

// Function to Scan I2C Devices
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

// Functions to Add Data to Buffers
void addVL53Data(int distance, unsigned long readTime) {
  vl53DataBuffer[vl53Index].timestamp = timeClient.getEpochTime();
  vl53DataBuffer[vl53Index].distance_mm = distance;
  vl53DataBuffer[vl53Index].readTime_us = readTime;
  vl53Index = (vl53Index + 1) % BUFFER_SIZE;
}

void addMPU6050Data(int16_t ax, int16_t ay, int16_t az, unsigned long readTime) {
  mpuDataBuffer[mpuIndex].timestamp = timeClient.getEpochTime();
  mpuDataBuffer[mpuIndex].ax = ax;
  mpuDataBuffer[mpuIndex].ay = ay;
  mpuDataBuffer[mpuIndex].az = az;
  mpuDataBuffer[mpuIndex].readTime_us = readTime;
  mpuIndex = (mpuIndex + 1) % BUFFER_SIZE;
}

void addBMP280Data(float temp, float pressure, unsigned long readTime) {
  bmpDataBuffer[bmpIndex].timestamp = timeClient.getEpochTime();
  bmpDataBuffer[bmpIndex].temperature_C = temp;
  bmpDataBuffer[bmpIndex].pressure_Pa = pressure;
  bmpDataBuffer[bmpIndex].readTime_us = readTime;
  bmpIndex = (bmpIndex + 1) % BUFFER_SIZE;
}

void addENS160Data(uint16_t eco2, uint16_t tvoc, unsigned long readTime) {
  ens160DataBuffer[ens160Index].timestamp = timeClient.getEpochTime();
  ens160DataBuffer[ens160Index].eco2_ppm = eco2;
  ens160DataBuffer[ens160Index].tvoc_ppb = tvoc;
  ens160DataBuffer[ens160Index].readTime_us = readTime;
  ens160Index = (ens160Index + 1) % BUFFER_SIZE;
}

void addAHT21Data(float temp, float humidity, unsigned long readTime) {
  aht21DataBuffer[aht21Index].timestamp = timeClient.getEpochTime();
  aht21DataBuffer[aht21Index].temperature_C = temp;
  aht21DataBuffer[aht21Index].humidity_percent = humidity;
  aht21DataBuffer[aht21Index].readTime_us = readTime;
  aht21Index = (aht21Index + 1) % BUFFER_SIZE;
}

// Function to Generate JSON Data
String getJSONData() {
  String json = "{";

  // Helper lambda to append data for each buffer
  auto appendVL53 = [&json]() {
    json += "\"VL53L0X_Distance_mm\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (vl53Index + i) % BUFFER_SIZE;
      if(vl53DataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(vl53DataBuffer[idx].timestamp) + ",\"value\":" + String(vl53DataBuffer[idx].distance_mm) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";
  };

  auto appendMPU6050 = [&json]() {
    json += "\"MPU6050_Ax\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (mpuIndex + i) % BUFFER_SIZE;
      if(mpuDataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(mpuDataBuffer[idx].timestamp) + ",\"value\":" + String(mpuDataBuffer[idx].ax) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";

    json += "\"MPU6050_Ay\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (mpuIndex + i) % BUFFER_SIZE;
      if(mpuDataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(mpuDataBuffer[idx].timestamp) + ",\"value\":" + String(mpuDataBuffer[idx].ay) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";

    json += "\"MPU6050_Az\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (mpuIndex + i) % BUFFER_SIZE;
      if(mpuDataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(mpuDataBuffer[idx].timestamp) + ",\"value\":" + String(mpuDataBuffer[idx].az) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";
  };

  auto appendBMP280 = [&json]() {
    json += "\"BMP280_Temp_C\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (bmpIndex + i) % BUFFER_SIZE;
      if(bmpDataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(bmpDataBuffer[idx].timestamp) + ",\"value\":" + String(bmpDataBuffer[idx].temperature_C) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";

    json += "\"BMP280_Press_Pa\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (bmpIndex + i) % BUFFER_SIZE;
      if(bmpDataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(bmpDataBuffer[idx].timestamp) + ",\"value\":" + String(bmpDataBuffer[idx].pressure_Pa) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";
  };

  auto appendENS160 = [&json]() {
    json += "\"ENS160_eCO2_ppm\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (ens160Index + i) % BUFFER_SIZE;
      if(ens160DataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(ens160DataBuffer[idx].timestamp) + ",\"value\":" + String(ens160DataBuffer[idx].eco2_ppm) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";

    json += "\"ENS160_TVOC_ppb\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (ens160Index + i) % BUFFER_SIZE;
      if(ens160DataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(ens160DataBuffer[idx].timestamp) + ",\"value\":" + String(ens160DataBuffer[idx].tvoc_ppb) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";
  };

  auto appendAHT21 = [&json]() {
    json += "\"AHT21_Temp_C\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (aht21Index + i) % BUFFER_SIZE;
      if(aht21DataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(aht21DataBuffer[idx].timestamp) + ",\"value\":" + String(aht21DataBuffer[idx].temperature_C) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";

    json += "\"AHT21_Humidity_percent\":[";
    for(int i=0; i<BUFFER_SIZE; i++) {
      int idx = (aht21Index + i) % BUFFER_SIZE;
      if(aht21DataBuffer[idx].timestamp == 0) continue;
      json += "{\"timestamp\":" + String(aht21DataBuffer[idx].timestamp) + ",\"value\":" + String(aht21DataBuffer[idx].humidity_percent) + "},";
    }
    if(json.endsWith(",")) json.remove(json.length()-1);
    json += "],";
  };

  // Append all sensor data
  appendVL53();
  appendMPU6050();
  appendBMP280();
  appendENS160();
  appendAHT21();

  // Remove trailing comma if present
  if(json.endsWith(",")) json.remove(json.length()-1);
  json += "}";

  return json;
}

// Function to Handle Root Web Page
void handleRoot() {
  String html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "  <title>Sensor Dashboard</title>";
  html += "  <meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "  <style>";
  html += "    body { font-family: Arial, sans-serif; background-color: #f4f4f4; margin: 0; padding: 20px; }";
  html += "    h1 { text-align: center; color: #333; }";
  html += "    .sensor { background: #fff; padding: 15px; margin-bottom: 20px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }";
  html += "    .sensor h2 { color: #555; }";
  html += "    svg { width: 100%; height: 200px; border: 1px solid #ddd; background-color: #fafafa; }";
  html += "    .axis { stroke: #ccc; }";
  html += "    .line { fill: none; stroke-width: 2px; }";
  html += "    .line1 { stroke: #ff0000; }"; // Red for TVOC
  html += "    .line2 { stroke: #0000ff; }"; // Blue for eCO2
  html += "  </style>";
  html += "</head>";
  html += "<body>";
  html += "  <h1>Sensor Dashboard</h1>";

  // Sensor Sections
  html += "  <div class='sensor' id='vl53'>";
  html += "    <h2>VL53L0X Distance (mm)</h2>";
  html += "    <svg id='svg_vl53'></svg>";
  html += "  </div>";

  html += "  <div class='sensor' id='mpu6050'>";
  html += "    <h2>MPU6050 Accelerometer (ax, ay, az)</h2>";
  html += "    <svg id='svg_mpu6050'></svg>";
  html += "  </div>";

  html += "  <div class='sensor' id='bmp280'>";
  html += "    <h2>BMP280 Temperature (°C) & Pressure (Pa)</h2>";
  html += "    <svg id='svg_bmp280_temp'></svg>";
  html += "    <svg id='svg_bmp280_press'></svg>";
  html += "  </div>";

  html += "  <div class='sensor' id='ens160'>";
  html += "    <h2>ENS160 eCO2 (ppm) & TVOC (ppb)</h2>";
  html += "    <svg id='svg_ens160_eco2'></svg>";
  html += "    <svg id='svg_ens160_tvoc'></svg>";
  html += "  </div>";

  html += "  <div class='sensor' id='aht21'>";
  html += "    <h2>AHT21 Temperature (°C) & Humidity (%)</h2>";
  html += "    <svg id='svg_aht21_temp'></svg>";
  html += "    <svg id='svg_aht21_humid'></svg>";
  html += "  </div>";

  // JavaScript for Fetching Data and Plotting
  html += "  <script>";
  html += "    async function fetchData() {";
  html += "      try {";
  html += "        const response = await fetch('/data');";
  html += "        const data = await response.json();";
  html += "        console.log('Fetched Data:', data);"; // Debugging
  html += "        plotVL53(data.VL53L0X_Distance_mm);";
  html += "        plotMPU6050(data.MPU6050_Ax, data.MPU6050_Ay, data.MPU6050_Az);";
  html += "        plotBMP280Temp(data.BMP280_Temp_C);";
  html += "        plotBMP280Press(data.BMP280_Press_Pa);";
  html += "        plotENS160Eco2(data.ENS160_eCO2_ppm);";
  html += "        plotENS160Tvoc(data.ENS160_TVOC_ppb);";
  html += "        plotAHT21Temp(data.AHT21_Temp_C);";
  html += "        plotAHT21Humid(data.AHT21_Humidity_percent);";
  html += "      } catch(err) {";
  html += "        console.error('Error fetching data:', err);";
  html += "      }";
  html += "    }";
  html += "    setInterval(fetchData, 5000); // Fetch data every 5 seconds";
  html += "    window.onload = fetchData;";

  // Helper function to create SVG paths
  html += "    function createPath(data, color) {";
  html += "      const width = 800;"; // Adjust as needed
  html += "      const height = 200;";
  html += "      const padding = 40;";
  html += "      let path = '';";
  html += "      const maxTime = data[data.length -1].timestamp;";
  html += "      const minTime = data[0].timestamp;";
  html += "      const values = data.map(d => d.value);";
  html += "      const minValue = Math.min(...values);";
  html += "      const maxValue = Math.max(...values);";
  html += "      const timeRange = maxTime - minTime;";
  html += "      if(timeRange === 0) { return ''; }"; // Prevent division by zero
  html += "      data.forEach((point, index) => {";
  html += "        const x = padding + ((point.timestamp - minTime) / timeRange) * (width - 2 * padding);";
  html += "        const y = height - padding - ((point.value - minValue) / (maxValue - minValue)) * (height - 2 * padding);";
  html += "        if(index === 0) {";
  html += "          path += 'M ' + x + ' ' + y + ' ';";
  html += "        } else {";
  html += "          path += 'L ' + x + ' ' + y + ' ';";
  html += "        }";
  html += "      });";
  html += "      return '<path d=\"' + path + '\" stroke=\"' + color + '\" class=\"line\" />';";
  html += "    }";

  // Plot Functions
  html += "    function plotVL53(data) {";
  html += "      const svg = document.getElementById('svg_vl53');";
  html += "      if(data.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const path = createPath(data, '#ff0000');"; // Red color
  html += "      svg.innerHTML = path;";
  html += "    }";

  html += "    function plotMPU6050(ax, ay, az) {";
  html += "      const svg = document.getElementById('svg_mpu6050');";
  html += "      if(ax.length === 0 || ay.length === 0 || az.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const width = 800;";
  html += "      const height = 200;";
  html += "      const padding = 40;";
  html += "      const maxTime = Math.max(ax[ax.length -1].timestamp, ay[ay.length -1].timestamp, az[az.length -1].timestamp);";
  html += "      const minTime = Math.min(ax[0].timestamp, ay[0].timestamp, az[0].timestamp);";
  html += "      const timeRange = maxTime - minTime;";
  html += "      if(timeRange === 0) { svg.innerHTML = ''; return; }";
  html += "      const allValues = ax.concat(ay, az).map(d => d.value);";
  html += "      const minValue = Math.min(...allValues);";
  html += "      const maxValue = Math.max(...allValues);";
  html += "      let pathAx = 'M ';";
  html += "      ax.forEach((point, index) => {";
  html += "        const x = padding + ((point.timestamp - minTime) / timeRange) * (width - 2 * padding);";
  html += "        const y = height - padding - ((point.value - minValue) / (maxValue - minValue)) * (height - 2 * padding);";
  html += "        if(index === 0) { pathAx += x + ' ' + y + ' '; }";
  html += "        else { pathAx += 'L ' + x + ' ' + y + ' '; }";
  html += "      });";
  html += "      let pathAy = 'M ';";
  html += "      ay.forEach((point, index) => {";
  html += "        const x = padding + ((point.timestamp - minTime) / timeRange) * (width - 2 * padding);";
  html += "        const y = height - padding - ((point.value - minValue) / (maxValue - minValue)) * (height - 2 * padding);";
  html += "        if(index === 0) { pathAy += x + ' ' + y + ' '; }";
  html += "        else { pathAy += 'L ' + x + ' ' + y + ' '; }";
  html += "      });";
  html += "      let pathAz = 'M ';";
  html += "      az.forEach((point, index) => {";
  html += "        const x = padding + ((point.timestamp - minTime) / timeRange) * (width - 2 * padding);";
  html += "        const y = height - padding - ((point.value - minValue) / (maxValue - minValue)) * (height - 2 * padding);";
  html += "        if(index === 0) { pathAz += x + ' ' + y + ' '; }";
  html += "        else { pathAz += 'L ' + x + ' ' + y + ' '; }";
  html += "      });";
  html += "      svg.innerHTML = `";
  html += "        <path d=\"${pathAx}\" stroke=\"#ff0000\" class=\"line line1\" />"; // Red for Ax
  html += "        <path d=\"${pathAy}\" stroke=\"#00ff00\" class=\"line line2\" />"; // Green for Ay
  html += "        <path d=\"${pathAz}\" stroke=\"#0000ff\" class=\"line line3\" />`;"; // Blue for Az
  html += "    }";

  html += "    function plotBMP280Temp(data) {";
  html += "      const svg = document.getElementById('svg_bmp280_temp');";
  html += "      if(data.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const path = createPath(data, '#ffa500');"; // Orange color
  html += "      svg.innerHTML = path;";
  html += "    }";

  html += "    function plotBMP280Press(data) {";
  html += "      const svg = document.getElementById('svg_bmp280_press');";
  html += "      if(data.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const path = createPath(data, '#800080');"; // Purple color
  html += "      svg.innerHTML = path;";
  html += "    }";

  html += "    function plotENS160Eco2(data) {";
  html += "      const svg = document.getElementById('svg_ens160_eco2');";
  html += "      if(data.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const path = createPath(data, '#0000ff');"; // Blue color for eCO2
  html += "      svg.innerHTML = path;";
  html += "    }";

  html += "    function plotENS160Tvoc(data) {";
  html += "      const svg = document.getElementById('svg_ens160_tvoc');";
  html += "      if(data.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const path = createPath(data, '#ff0000');"; // Red color for TVOC
  html += "      svg.innerHTML = path;";
  html += "    }";

  html += "    function plotAHT21Temp(data) {";
  html += "      const svg = document.getElementById('svg_aht21_temp');";
  html += "      if(data.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const path = createPath(data, '#008000');"; // Green color
  html += "      svg.innerHTML = path;";
  html += "    }";

  html += "    function plotAHT21Humid(data) {";
  html += "      const svg = document.getElementById('svg_aht21_humid');";
  html += "      if(data.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const path = createPath(data, '#000080');"; // Navy color
  html += "      svg.innerHTML = path;";
  html += "    }";

  html += "  </script>";
  html += "</body>";
  html += "</html>";

  server.send(200, "text/html", html);
}

// Function to Display Header on TFT
void displayHeader(const char* header, uint16_t color) {
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println(header);
  tft.drawLine(0, 20, tft.width(), 20, color); // Separator line
}

// Function to Display Value on TFT
// Function to Display Value on TFT
void displayValue(const char* label, const char* value, uint16_t color, int y) {
    tft.setTextColor(color, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, y);
    tft.print(label);
    tft.print(": ");
    tft.print(value);

    // Calculate approximate text width
    int textWidth = strlen(label) * 6 + strlen(value) * 6 + 10; // Adjust based on font size

    // Clear the rest of the line to prevent ghosting
    if(textWidth < tft.width()) {
        tft.fillRect(textWidth, y, tft.width() - textWidth, 8, ST77XX_BLACK);
    }
}
// ----------------------------- Loop Function -----------------------------
void loop() {
  server.handleClient();
  yield(); // Yield to allow background tasks

  unsigned long currentMillis = millis();

  // Sensor Reading Interval
  if (currentMillis - previousSensorRead >= SENSOR_READ_INTERVAL) {
    previousSensorRead = currentMillis;

    // Read VL53L0X Sensor
    unsigned long vl53StartTime = micros();
    int vl53Distance = vl53.readRangeSingleMillimeters();
    unsigned long vl53ReadTime = micros() - vl53StartTime;
    addVL53Data(vl53Distance > 0 ? vl53Distance : -1, vl53ReadTime);
    Serial.print("VL53L0X Distance: ");
    Serial.println(vl53Distance > 0 ? vl53Distance : -1);

    // Read MPU6050 Sensor
    unsigned long mpuStartTime = micros();
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    unsigned long mpuReadTime = micros() - mpuStartTime;
    addMPU6050Data(ax, ay, az, mpuReadTime);
    Serial.print("MPU6050 Accel: (");
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.println(az);

    // Read BMP280 Sensor
    unsigned long bmpStartTime = micros();
    float bmpTemp = bmp.readTemperature();
    float bmpPressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa for readability
    unsigned long bmpReadTime = micros() - bmpStartTime;
    addBMP280Data(bmpTemp, bmpPressure, bmpReadTime);
    Serial.print("BMP280 Temp: ");
    Serial.println(bmpTemp);
    Serial.print("BMP280 Pressure: ");
    Serial.println(bmpPressure);

    // Read ENS160 Sensor
    unsigned long ens160StartTime = micros();
    uint16_t eco2 = ens160.getECO2();
    uint16_t tvoc = ens160.getTVOC();
    unsigned long ens160ReadTime = micros() - ens160StartTime;
    addENS160Data(eco2, tvoc, ens160ReadTime);
    Serial.print("ENS160 eCO2: ");
    Serial.println(eco2);
    Serial.print("ENS160 TVOC: ");
    Serial.println(tvoc);

    // Read AHT21 Sensor
    unsigned long ahtStartTime = micros();
    sensors_event_t humidity, tempEvent;
    aht.getEvent(&humidity, &tempEvent);
    unsigned long ahtReadTime = micros() - ahtStartTime;
    if (!isnan(tempEvent.temperature)) {
      addAHT21Data(tempEvent.temperature, humidity.relative_humidity, ahtReadTime);
      Serial.print("AHT21 Temp: ");
      Serial.println(tempEvent.temperature);
      Serial.print("AHT21 Humidity: ");
      Serial.println(humidity.relative_humidity);
    } else {
      addAHT21Data(-1, -1, ahtReadTime);
      Serial.print("AHT21 Temp: ");
      Serial.println(-1);
      Serial.print("AHT21 Humidity: ");
      Serial.println(-1);
    }

    // Read GPS Data
    unsigned long gpsStartTime = micros();
    char gpsData[100] = "NA"; // Fixed-size buffer to store GPS data
    if (gpsSerial.available()) {
      gpsSerial.readBytesUntil('\n', gpsData, sizeof(gpsData) - 1);
      gpsData[sizeof(gpsData) - 1] = '\0'; // Ensure null-termination
      Serial.print("GPS Data: ");
      Serial.println(gpsData);
    } else {
      Serial.println("GPS Data: NA");
    }
    unsigned long gpsReadTime = micros() - gpsStartTime;

    // Update NeoMatrix Based on ENS160 Values
    // Mapping:
    // - TVOC: 150 ppb (min) to 1200 ppb (max) -> Red channel brightness
    // - eCO2: 600 ppm (min) to 1200 ppm (max) -> Blue channel brightness

    // Clamp the values within the specified ranges
    uint16_t clampedTVOC = constrain(tvoc, 150, 1200);
    uint16_t clampedCO2 = constrain(eco2, 600, 1200);

    // Map the values to brightness (0-255)
    uint8_t mappedRed = map(clampedTVOC, 150, 1200, 0, 255);
    uint8_t mappedBlue = map(clampedCO2, 600, 1200, 0, 255);

    // Update the center 4x4 LEDs
    for(int y = 2; y < 6; y++) { // Rows 2 to 5
      for(int x = 2; x < 6; x++) { // Columns 2 to 5
        matrix.drawPixel(x, y, matrix.Color(mappedRed, 0, mappedBlue));
      }
    }
    matrix.show();

    // Clear the center 4x4 LEDs before the next update to prevent accumulation
    for(int y = 2; y < 6; y++) { // Rows 2 to 5
      for(int x = 2; x < 6; x++) { // Columns 2 to 5
        matrix.drawPixel(x, y, matrix.Color(0, 0, 0));
      }
    }
  }

  // Display Update Interval
  if (currentMillis - previousDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    previousDisplayUpdate = currentMillis;

    // Display Header
    displayHeader("Sensor Dashboard", ST77XX_CYAN);

    int currentY = 40; // Starting Y position after header

    // Display VL53L0X Data
    if(vl53Index > 0) {
      VL53Data latestVL53 = vl53DataBuffer[(vl53Index - 1 + BUFFER_SIZE) % BUFFER_SIZE];
      
      char distanceStr[20];
      snprintf(distanceStr, sizeof(distanceStr), "%d mm", (latestVL53.distance_mm > 0) ? latestVL53.distance_mm : -1);
      displayValue("VL53L0X", distanceStr, ST77XX_GREEN, currentY);
      currentY += 20;

      char readTimeStr[20];
      snprintf(readTimeStr, sizeof(readTimeStr), "%lu µs", latestVL53.readTime_us);
      displayValue("Read Time", readTimeStr, ST77XX_GREEN, currentY);
      currentY += 20;
    }

    // Display MPU6050 Data
    if(mpuIndex > 0) {
      MPU6050Data latestMPU = mpuDataBuffer[(mpuIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE];
      
      char accelStr[30];
      snprintf(accelStr, sizeof(accelStr), "(%d, %d, %d)", latestMPU.ax, latestMPU.ay, latestMPU.az);
      displayValue("MPU6050", accelStr, ST77XX_CYAN, currentY);
      currentY += 20;

      char readTimeStr[20];
      snprintf(readTimeStr, sizeof(readTimeStr), "%lu µs", latestMPU.readTime_us);
      displayValue("Read Time", readTimeStr, ST77XX_CYAN, currentY);
      currentY += 20;
    }

    // Display BMP280 Data
    if(bmpIndex > 0) {
      BMP280Data latestBMP = bmpDataBuffer[(bmpIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE];
      
      char tempStr[20];
      snprintf(tempStr, sizeof(tempStr), "%.2f °C", latestBMP.temperature_C);
      displayValue("BMP280 Temp", tempStr, ST77XX_YELLOW, currentY);
      currentY += 20;

      char pressStr[20];
      snprintf(pressStr, sizeof(pressStr), "%.2f hPa", latestBMP.pressure_Pa);
      displayValue("BMP280 Press", pressStr, ST77XX_YELLOW, currentY);
      currentY += 20;

      char readTimeStr[20];
      snprintf(readTimeStr, sizeof(readTimeStr), "%lu µs", latestBMP.readTime_us);
      displayValue("Read Time", readTimeStr, ST77XX_YELLOW, currentY);
      currentY += 20;
    }

    // Display ENS160 Data
    if(ens160Index > 0) {
      ENS160Data latestENS = ens160DataBuffer[(ens160Index - 1 + BUFFER_SIZE) % BUFFER_SIZE];
      
      char eco2Str[20];
      snprintf(eco2Str, sizeof(eco2Str), "%d ppm", latestENS.eco2_ppm);
      displayValue("ENS160 eCO2", eco2Str, ST77XX_MAGENTA, currentY);
      currentY += 20;

      char tvocStr[20];
      snprintf(tvocStr, sizeof(tvocStr), "%d ppb", latestENS.tvoc_ppb);
      displayValue("ENS160 TVOC", tvocStr, ST77XX_MAGENTA, currentY);
      currentY += 20;

      char readTimeStr[20];
      snprintf(readTimeStr, sizeof(readTimeStr), "%lu µs", latestENS.readTime_us);
      displayValue("Read Time", readTimeStr, ST77XX_MAGENTA, currentY);
      currentY += 20;
    }

    // Display AHT21 Data
    if(aht21Index > 0) {
      AHT21Data latestAHT = aht21DataBuffer[(aht21Index - 1 + BUFFER_SIZE) % BUFFER_SIZE];
      
      if(latestAHT.temperature_C != -1) {
        char tempStr[20];
        snprintf(tempStr, sizeof(tempStr), "%.2f °C", latestAHT.temperature_C);
        displayValue("AHT21 Temp", tempStr, ST77XX_BLUE, currentY);
        currentY += 20;

        char humidStr[20];
        snprintf(humidStr, sizeof(humidStr), "%.2f %%", latestAHT.humidity_percent);
        displayValue("AHT21 Humid", humidStr, ST77XX_BLUE, currentY);
      } else {
        displayValue("AHT21 Temp", "N/A", ST77XX_BLUE, currentY);
        currentY += 20;
        displayValue("AHT21 Humid", "N/A", ST77XX_BLUE, currentY);
      }
      currentY += 20;

      char readTimeStr[20];
      snprintf(readTimeStr, sizeof(readTimeStr), "%lu µs", latestAHT.readTime_us);
      displayValue("Read Time", readTimeStr, ST77XX_BLUE, currentY);
      currentY += 20;
    }

    // Update NeoPixel Status
    pixels.show();
  }

  // Time Update Interval
  if (currentMillis - previousTimeUpdate >= TIME_UPDATE_INTERVAL) {
    previousTimeUpdate = currentMillis;
    if(timeClient.update()) {
      Serial.println("Time updated via NTP");
    } else {
      timeClient.forceUpdate();
      Serial.println("Forced Time update via NTP");
    }
  }

  // Continue Loop Without Delay
}

