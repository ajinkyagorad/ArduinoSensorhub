#include <Wire.h>
#include <Adafruit_GFX.h>           // Core graphics library
#include <Adafruit_ST7789.h>        // Hardware-specific library for ST7789
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include <Adafruit_BMP085.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Pin Definitions
#define TFT_CS   D1
#define TFT_RST  D3
#define TFT_DC   D2
#define LED_PIN  D0
#define NUMPIXELS 64

// Wi-Fi Credentials
const char* ssid = "Triton9";
const char* password = "88888888";

// Web Server Port
WebServer server(80);

// NTP Client Setup
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // UTC Time, update every 60 seconds

// Initialize Display and NeoPixel
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Initialize Sensors
VL53L0X vl53;
MPU6050 mpu;
Adafruit_BMP085 bmp;
DFRobot_ENS160_I2C ens160(&Wire, 0x53); // ENS160 I2C address
Adafruit_AHTX0 aht;



// Helper function to map values to RGB color
struct RGB {
  int r, g, b;
};
// Initialize GPS (Assuming GPS is connected to Serial1)
#define gpsSerial Serial1

// Buffer Sizes
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
  int16_t gx, gy, gz; // Add gyroscope data
  float temp; // Add internal temperature
  unsigned long readTime_us;
} mpuDataBuffer[BUFFER_SIZE];

int mpuIndex = 0;

struct BMP180Data {
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



// Global sensor variables
int vl53Distance;
int16_t ax, ay, az;
float bmpTemp, bmpPressure;
uint16_t eco2, tvoc;
sensors_event_t humidity, temp;
String gpsData;



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

// Function to Add Data to VL53 Buffer
void addVL53Data(int distance, unsigned long readTime) {
  vl53DataBuffer[vl53Index].timestamp = timeClient.getEpochTime();
  vl53DataBuffer[vl53Index].distance_mm = distance;
  vl53DataBuffer[vl53Index].readTime_us = readTime;
  vl53Index = (vl53Index + 1) % BUFFER_SIZE;
}

// Function to Add Data to MPU6050 Buffer
void addMPU6050Data(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float temp, unsigned long readTime) {
  mpuDataBuffer[mpuIndex].timestamp = timeClient.getEpochTime();
  mpuDataBuffer[mpuIndex].ax = ax;
  mpuDataBuffer[mpuIndex].ay = ay;
  mpuDataBuffer[mpuIndex].az = az;
  mpuDataBuffer[mpuIndex].gx = gx;
  mpuDataBuffer[mpuIndex].gy = gy;
  mpuDataBuffer[mpuIndex].gz = gz;
  mpuDataBuffer[mpuIndex].temp = temp;
  mpuDataBuffer[mpuIndex].readTime_us = readTime;
  mpuIndex = (mpuIndex + 1) % BUFFER_SIZE;
}


// Function to Add Data to BMP280 Buffer
void addBMP280Data(float temp, float pressure, unsigned long readTime) {
  bmpDataBuffer[bmpIndex].timestamp = timeClient.getEpochTime();
  bmpDataBuffer[bmpIndex].temperature_C = temp;
  bmpDataBuffer[bmpIndex].pressure_Pa = pressure;
  bmpDataBuffer[bmpIndex].readTime_us = readTime;
  bmpIndex = (bmpIndex + 1) % BUFFER_SIZE;
}

// Function to Add Data to ENS160 Buffer
void addENS160Data(uint16_t eco2, uint16_t tvoc, unsigned long readTime) {
  ens160DataBuffer[ens160Index].timestamp = timeClient.getEpochTime();
  ens160DataBuffer[ens160Index].eco2_ppm = eco2;
  ens160DataBuffer[ens160Index].tvoc_ppb = tvoc;
  ens160DataBuffer[ens160Index].readTime_us = readTime;
  ens160Index = (ens160Index + 1) % BUFFER_SIZE;
}

// Function to Add Data to AHT21 Buffer
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
  html += "    .line1 { stroke: #ff0000; }"; // e.g., Red for X-axis
  html += "    .line2 { stroke: #00ff00; }"; // e.g., Green for Y-axis
  html += "    .line3 { stroke: #0000ff; }"; // e.g., Blue for Z-axis
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
  html += "      const times = data.map(d => d.timestamp);";
  html += "      const values = data.map(d => d.value);";
  html += "      const minValue = Math.min(...values);";
  html += "      const maxValue = Math.max(...values);";
  html += "      data.forEach((point, index) => {";
  html += "        const x = padding + ((point.timestamp - minTime) / (maxTime - minTime)) * (width - 2 * padding);";
  html += "        const y = height - padding - ((point.value - minValue) / (maxValue - minValue)) * (height - 2 * padding);";
  html += "        if(index === 0) {";
  html += "          path += `M ${x} ${y} `;";
  html += "        } else {";
  html += "          path += `L ${x} ${y} `;";
  html += "        }";
  html += "      });";
  html += "      return `<path d='${path}' stroke='${color}' class='line' />`;";
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
  html += "      if(ax.length === 0) { svg.innerHTML = ''; return; }";
  html += "      let path = '';";
  html += "      const width = 800;";
  html += "      const height = 200;";
  html += "      const padding = 40;";
  html += "      const maxTime = ax[ax.length -1].timestamp;";
  html += "      const minTime = ax[0].timestamp;";
  html += "      const axValues = ax.map(d => d.value);";
  html += "      const ayValues = ay.map(d => d.value);";
  html += "      const azValues = az.map(d => d.value);";
  html += "      const minValue = Math.min(...axValues, ...ayValues, ...azValues);";
  html += "      const maxValue = Math.max(...axValues, ...ayValues, ...azValues);";
  html += "      // Ax";
  html += "      let pathAx = 'M ';";
  html += "      ax.forEach((point, index) => {";
  html += "        const x = padding + ((point.timestamp - minTime) / (maxTime - minTime)) * (width - 2 * padding);";
  html += "        const y = height - padding - ((point.value - minValue) / (maxValue - minValue)) * (height - 2 * padding);";
  html += "        if(index === 0) { pathAx += `${x} ${y} `; }";
  html += "        else { pathAx += `L ${x} ${y} `; }";
  html += "      });";
  html += "      // Ay";
  html += "      let pathAy = 'M ';";
  html += "      ay.forEach((point, index) => {";
  html += "        const x = padding + ((point.timestamp - minTime) / (maxTime - minTime)) * (width - 2 * padding);";
  html += "        const y = height - padding - ((point.value - minValue) / (maxValue - minValue)) * (height - 2 * padding);";
  html += "        if(index === 0) { pathAy += `${x} ${y} `; }";
  html += "        else { pathAy += `L ${x} ${y} `; }";
  html += "      });";
  html += "      // Az";
  html += "      let pathAz = 'M ';";
  html += "      az.forEach((point, index) => {";
  html += "        const x = padding + ((point.timestamp - minTime) / (maxTime - minTime)) * (width - 2 * padding);";
  html += "        const y = height - padding - ((point.value - minValue) / (maxValue - minValue)) * (height - 2 * padding);";
  html += "        if(index === 0) { pathAz += `${x} ${y} `; }";
  html += "        else { pathAz += `L ${x} ${y} `; }";
  html += "      });";
  html += "      svg.innerHTML = `<path d='${pathAx}' stroke='#ff0000' class='line line1' /><path d='${pathAy}' stroke='#00ff00' class='line line2' /><path d='${pathAz}' stroke='#0000ff' class='line line3' />`;";
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
  html += "      const path = createPath(data, '#00ffff');"; // Cyan color
  html += "      svg.innerHTML = path;";
  html += "    }";

  html += "    function plotENS160Tvoc(data) {";
  html += "      const svg = document.getElementById('svg_ens160_tvoc');";
  html += "      if(data.length === 0) { svg.innerHTML = ''; return; }";
  html += "      const path = createPath(data, '#ff00ff');"; // Magenta color
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
// Setup Function
void setup() {
  // Initialize TFT Display
  tft.init(170, 320); // Initialize with width and height
  tft.setRotation(2); // Rotate display anticlockwise by 90 degrees
  tft.fillScreen(ST77XX_BLACK);
  pixels.setPixelColor(2, pixels.Color(0, 255, 0)); // Indicate TFT initialized
  pixels.show();
  Serial.println("TFT initialized");
  tft.println("TFT initialized");

  // Initialize Serial Communication
  Serial.begin(115200);
  Wire.begin();

  // Initialize GPS Serial (Assuming GPS is connected to Serial1)
  gpsSerial.begin(9600);

  // Scan for I2C Devices
  scanI2CDevices();

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(10); // Set brightness to a lower value
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Indicate Device is On
  pixels.show();

  // Initialize VL53L0X Sensor
  if (vl53.init()) {
    pixels.setPixelColor(3, pixels.Color(0, 255, 0)); // VL53L0X works
    Serial.println("VL53L0X initialized");
    tft.println("VL53L0X initialized");
  } else {
    pixels.setPixelColor(3, pixels.Color(255, 0, 0)); // VL53L0X failed
    Serial.println("VL53L0X initialization failed");
    tft.println("VL53L0X initialization failed");
  }
  pixels.show();

  // Initialize MPU6050 Sensor
  mpu.initialize();
  if (mpu.testConnection()) {
    pixels.setPixelColor(4, pixels.Color(0, 255, 0)); // MPU6050 works
    Serial.println("MPU6050 initialized");
    tft.println("MPU6050 initialized");
  } else {
    pixels.setPixelColor(4, pixels.Color(255, 0, 0)); // MPU6050 failed
    Serial.println("MPU6050 initialization failed");
    tft.println("MPU6050 initialization failed");
  }
  pixels.show();

  // Initialize BMP180 Sensor
  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed.");
  } else {
    Serial.println("BMP180 initialized successfully.");
  }

  pixels.show();

  // Initialize ENS160 Sensor
  while (ens160.begin() != NO_ERR) {
    Serial.println("Communication with ENS160 failed, retrying...");
    tft.println("ENS160 failed, retrying...");
    pixels.setPixelColor(6, pixels.Color(255, 0, 0)); // ENS160 failed
    pixels.show();
    delay(3000);
  }
  Serial.println("ENS160 initialized successfully!");
  tft.println("ENS160 initialized successfully!");
  pixels.setPixelColor(6, pixels.Color(0, 255, 0)); // ENS160 works
  pixels.show();

  // Initialize AHT21 Sensor
  if (aht.begin()) {
    pixels.setPixelColor(7, pixels.Color(0, 255, 0)); // AHT21 works
    Serial.println("AHT21 initialized successfully!");
    tft.println("AHT21 initialized successfully!");
  } else {
    pixels.setPixelColor(7, pixels.Color(255, 0, 0)); // AHT21 failed
    Serial.println("AHT21 initialization failed");
    tft.println("AHT21 initialization failed");
  }
  pixels.show();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  pixels.setPixelColor(1, pixels.Color(255, 255, 0)); // Indicate Wi-Fi Connecting
  pixels.show();
  Serial.print("Connecting to Wi-Fi");
  tft.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tft.print(".");
  }
  Serial.println("\nWi-Fi connected");
  tft.println("\nWi-Fi connected");
  pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // Indicate Wi-Fi connected
  pixels.show();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  tft.print("IP address: ");
  tft.println(WiFi.localIP());

  // Initialize NTP Client
  timeClient.begin();
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  Serial.println("Time synchronized via NTP");
  tft.println("Time synchronized via NTP");

  // Initialize Web Server Routes
  server.on("/", handleRoot);
  server.on("/data", HTTP_GET, [](){
    String json = getJSONData();
    server.send(200, "application/json", json);
  });

  // Start Web Server
  server.begin();
  Serial.println("HTTP server started");
  tft.println("HTTP server started");
}

// Function to Clear Specific Line on TFT
void clearLine(int y) {
  tft.fillRect(0, y, tft.width(), 20, ST77XX_BLACK); // Assuming 20 pixels height per line
}

// Function to Display Header on TFT
void displayHeader(const char* header, uint16_t color) {
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setTextSize(2);
  int y = 0;
  clearLine(y);
  clearLine(y + 10);
  tft.setCursor(0, y);
  tft.println(header);
  tft.drawLine(0, y + 30, tft.width(), y + 30, color); // Adjust as needed
}

// Function to Display Value on TFT
void displayValue(const char* label, String value, uint16_t color, int y) {
  clearLine(y);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, y);
  tft.print(label);
  tft.print(": ");
  tft.print(value);

  // Append spaces to clear remaining part of the line
  String displayStr = label + String(": ") + value;
  while (displayStr.length() < 20) { // Adjust based on display width
    displayStr += " ";
  }
  tft.setCursor(0, y);
  tft.print(displayStr.substring(0, 20));
}


RGB mapSensorValueToColor(float value, float minRange, float maxRange, RGB minColor, RGB maxColor) {
  float normalized = (value - minRange) / (maxRange - minRange);
  if (normalized < 0) normalized = 0;
  if (normalized > 1) normalized = 1;

  RGB color;
  color.r = minColor.r + (maxColor.r - minColor.r) * normalized;
  color.g = minColor.g + (maxColor.g - minColor.g) * normalized;
  color.b = minColor.b + (maxColor.b - minColor.b) * normalized;

  return color;
}

// Display RGB colors based on sensor values on the NeoPixel matrix
void displaySensorColors() {
  RGB color;

  // Example for VL53L0X (Distance)
  color = mapSensorValueToColor(vl53Distance, 0, 2000, {0, 0, 255}, {255, 0, 0}); // Blue to Red
  pixels.setPixelColor(20, pixels.Color(color.r, color.g, color.b));

  // Separate display for MPU6050 (Acceleration components)
  color = mapSensorValueToColor(ax, -16000, 16000, {0, 0, 255}, {255, 0, 0}); // Blue to Red for ax
  pixels.setPixelColor(21, pixels.Color(color.r, color.g, color.b));

  color = mapSensorValueToColor(ay, -16000, 16000, {0, 255, 0}, {255, 255, 0}); // Green to Yellow for ay
  pixels.setPixelColor(22, pixels.Color(color.r, color.g, color.b));

  color = mapSensorValueToColor(az, -16000, 16000, {0, 255, 255}, {255, 0, 255}); // Cyan to Magenta for az
  pixels.setPixelColor(23, pixels.Color(color.r, color.g, color.b));

  // Example for BMP 180/280 (Temperature)
  color = mapSensorValueToColor(bmpTemp, 15, 35, {0, 0, 255}, {255, 165, 0}); // Temperature color mapping
  pixels.setPixelColor(24, pixels.Color(color.r, color.g, color.b));

  // Example for ENS160 (eCO2)
  color = mapSensorValueToColor(eco2, 400, 2000, {0, 255, 0}, {255, 0, 0}); // Green (healthy) to Red (unhealthy)
  pixels.setPixelColor(25, pixels.Color(color.r, color.g, color.b));

  // Example for ENS160 (TVOC)
  color = mapSensorValueToColor(tvoc, 0, 1000, {0, 255, 255}, {255, 0, 255}); // Cyan (good) to Magenta (poor)
  pixels.setPixelColor(26, pixels.Color(color.r, color.g, color.b));

  // Example for AHT21 (Humidity)
  color = mapSensorValueToColor(humidity.relative_humidity, 30, 70, {255, 255, 255}, {0, 0, 255}); // White to Blue
  pixels.setPixelColor(27, pixels.Color(color.r, color.g, color.b));

  // Example for GPS Data Status (if available)
  if (gpsData != "NA") {
    color = {0, 255, 0}; // Green if GPS data is present
  } else {
    color = {255, 0, 0}; // Red if no GPS data
  }
  pixels.setPixelColor(28, pixels.Color(color.r, color.g, color.b));

  // Show the colors on the NeoPixel matrix
  pixels.show();
}

// Loop Function

// Loop Function
void loop() {
  server.handleClient();

  // Display Header
  displayHeader("Sensor Dashboard", ST77XX_CYAN);

  int currentY = 40; // Starting Y position after header

  // Read VL53L0X Sensor
  unsigned long vl53StartTime = micros();
  vl53Distance = vl53.readRangeSingleMillimeters();
  unsigned long vl53ReadTime = micros() - vl53StartTime;
  addVL53Data(vl53Distance > 0 ? vl53Distance : -1, vl53ReadTime);
  displayValue("VL53L0X", String(vl53Distance > 0 ? vl53Distance : -1) + " mm", ST77XX_GREEN, currentY);
  currentY += 20;
  displayValue("Read Time", String(vl53ReadTime) + " µs", ST77XX_GREEN, currentY);
  currentY += 20;
  Serial.print("VL53L0X Distance: ");
  Serial.println(vl53Distance > 0 ? vl53Distance : -1);

  // Read MPU6050 Sensor (including gyroscope and temperature)
  unsigned long mpuStartTime = micros();
  mpu.getAcceleration(&ax, &ay, &az);
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float mpuTemp = mpu.getTemperature() / 340.00 + 36.53; // Convert raw temp data
  unsigned long mpuReadTime = micros() - mpuStartTime;
  addMPU6050Data(ax, ay, az, gx, gy, gz, mpuTemp, mpuReadTime);
  displayValue("MPU6050 Accel", "(" + String(ax) + ", " + String(ay) + ", " + String(az) + ")", ST77XX_CYAN, currentY);
  currentY += 20;
  displayValue("MPU6050 Gyro", "(" + String(gx) + ", " + String(gy) + ", " + String(gz) + ")", ST77XX_CYAN, currentY);
  currentY += 20;
  displayValue("MPU6050 Temp", String(mpuTemp) + " °C", ST77XX_CYAN, currentY);
  currentY += 20;
  displayValue("Read Time", String(mpuReadTime) + " µs", ST77XX_CYAN, currentY);
  currentY += 20;
  Serial.print("MPU6050 Gyro: (");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.println(gz);
  Serial.print("MPU6050 Temp: ");
  Serial.println(mpuTemp);


  // Read BMP280 Sensor
  unsigned long bmpStartTime = micros();
  bmpTemp = bmp.readTemperature();
  bmpPressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa for readability
  unsigned long bmpReadTime = micros() - bmpStartTime;
  addBMP280Data(bmpTemp, bmpPressure, bmpReadTime);
  displayValue("BMP280 Temp", String(bmpTemp) + " °C", ST77XX_YELLOW, currentY);
  currentY += 20;
  displayValue("BMP280 Press", String(bmpPressure) + " hPa", ST77XX_YELLOW, currentY);
  currentY += 20;
  displayValue("Read Time", String(bmpReadTime) + " µs", ST77XX_YELLOW, currentY);
  currentY += 20;
  Serial.print("BMP280 Temp: ");
  Serial.println(bmpTemp);
  Serial.print("BMP280 Pressure: ");
  Serial.println(bmpPressure);

  // Read ENS160 Sensor
  unsigned long ens160StartTime = micros();
  eco2 = ens160.getECO2();
  tvoc = ens160.getTVOC();
  unsigned long ens160ReadTime = micros() - ens160StartTime;
  addENS160Data(eco2, tvoc, ens160ReadTime);
  displayValue("ENS160 eCO2", String(eco2) + " ppm", ST77XX_MAGENTA, currentY);
  currentY += 20;
  displayValue("ENS160 TVOC", String(tvoc) + " ppb", ST77XX_MAGENTA, currentY);
  currentY += 20;
  displayValue("Read Time", String(ens160ReadTime) + " µs", ST77XX_MAGENTA, currentY);
  currentY += 20;
  Serial.print("ENS160 eCO2: ");
  Serial.println(eco2);
  Serial.print("ENS160 TVOC: ");
  Serial.println(tvoc);

  // Read AHT21 Sensor
  unsigned long ahtStartTime = micros();
  aht.getEvent(&humidity, &temp);
  unsigned long ahtReadTime = micros() - ahtStartTime;
  if (!isnan(temp.temperature)) {
    addAHT21Data(temp.temperature, humidity.relative_humidity, ahtReadTime);
    displayValue("AHT21 Temp", String(temp.temperature) + " °C", ST77XX_BLUE, currentY);
    currentY += 20;
    displayValue("AHT21 Humid", String(humidity.relative_humidity) + " %", ST77XX_BLUE, currentY);
  } else {
    addAHT21Data(-1, -1, ahtReadTime);
    displayValue("AHT21 Temp", "N/A", ST77XX_BLUE, currentY);
    currentY += 20;
    displayValue("AHT21 Humid", "N/A", ST77XX_BLUE, currentY);
  }
  currentY += 20;
  displayValue("Read Time", String(ahtReadTime) + " µs", ST77XX_BLUE, currentY);
  currentY += 20;
  Serial.print("AHT21 Temp: ");
  Serial.println(!isnan(temp.temperature) ? temp.temperature : -1);
  Serial.print("AHT21 Humidity: ");
  Serial.println(!isnan(humidity.relative_humidity) ? humidity.relative_humidity : -1);

  // Read GPS Data
  unsigned long gpsStartTime = micros();
  gpsData = "NA"; // Declare as global at the top
  if (gpsSerial.available()) {
    gpsData = gpsSerial.readStringUntil('\n');
    gpsData.trim(); // Remove any trailing newline or carriage return
    Serial.print("GPS Data: ");
    Serial.println(gpsData);
  } else {
    Serial.println("GPS Data: NA");
  }
  unsigned long gpsReadTime = micros() - gpsStartTime;
  displayValue("GPS Data", gpsData, ST77XX_WHITE, currentY);
  currentY += 20;
  displayValue("Read Time", String(gpsReadTime) + " µs", ST77XX_WHITE, currentY);
  currentY += 20;

  // Update NeoPixel Status
  displaySensorColors();

  // Small Delay to Prevent Overloading
  //delay(1000);
}
