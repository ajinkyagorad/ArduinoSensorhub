#include <Wire.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TFT_CS   D1
#define TFT_RST  D3
#define TFT_DC   D2

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
HardwareSerial gpsSerial(1); // Use UART1 for GPS

// Create mutex for I2C bus and data access
SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t data_mutex;

// Shared sensor data
int vl53Distance = 0;
int16_t ax = 0, ay = 0, az = 0;
float bmpTemp = 0, bmpPressure = 0;
uint16_t eco2 = 0, tvoc = 0;
float ahtTemp = 0, ahtHumidity = 0;
String gpsData = "";

// Data buffers for plots
#define DATA_BUFFER_SIZE 170 // Width of display for plots
float accelXData[DATA_BUFFER_SIZE] = {0};
float accelYData[DATA_BUFFER_SIZE] = {0};
float accelZData[DATA_BUFFER_SIZE] = {0};
int dataIndex = 0; // Index for data buffers

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Create mutexes
  i2c_mutex = xSemaphoreCreateMutex();
  data_mutex = xSemaphoreCreateMutex();

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(10); // Set brightness to a lower value
  pixels.clear();
  pixels.show();

  // Initialize TFT
  tft.init(170, 320); // Initialize with width and height
  tft.setRotation(2); // Rotate display anticlockwise by 90 degrees
  tft.fillScreen(ST77XX_BLACK);
  Serial.println("TFT initialized");

  // Initialize VL53L0X
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    if (vl53.init()) {
      Serial.println("VL53L0X initialized");
    } else {
      Serial.println("VL53L0X initialization failed");
    }
    xSemaphoreGive(i2c_mutex);
  }

  // Initialize MPU6050
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    mpu.initialize();
    if (mpu.testConnection()) {
      Serial.println("MPU6050 initialized");
    } else {
      Serial.println("MPU6050 initialization failed");
    }
    xSemaphoreGive(i2c_mutex);
  }

  // Initialize BMP280
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    if (bmp.begin(0x76)) {
      Serial.println("BMP280 initialized at 0x76");
    } else if (bmp.begin(0x77)) {
      Serial.println("BMP280 initialized at 0x77");
    } else {
      Serial.println("BMP280 initialization failed");
    }
    xSemaphoreGive(i2c_mutex);
  }

  // Initialize ENS160
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    while (ens160.begin() != NO_ERR) {
      Serial.println("Communication with ENS160 failed, retrying...");
      delay(3000);
    }
    Serial.println("ENS160 initialized successfully!");
    xSemaphoreGive(i2c_mutex);
  }

  // Initialize AHT21
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    if (aht.begin()) {
      Serial.println("AHT21 initialized successfully!");
    } else {
      Serial.println("AHT21 initialization failed");
    }
    xSemaphoreGive(i2c_mutex);
  }

  // Initialize GPS
  gpsSerial.begin(9600);
  Serial.println("GPS initialized");

  // Create tasks
  xTaskCreate(vl53Task, "VL53L0X Task", 2048, NULL, 1, NULL);
  xTaskCreate(mpuTask, "MPU6050 Task", 2048, NULL, 1, NULL);
  xTaskCreate(bmpTask, "BMP280 Task", 2048, NULL, 1, NULL);
  xTaskCreate(ensTask, "ENS160 Task", 2048, NULL, 1, NULL);
  xTaskCreate(ahtTask, "AHT21 Task", 2048, NULL, 1, NULL);
  xTaskCreate(gpsTask, "GPS Task", 4096, NULL, 1, NULL);
  xTaskCreate(displayTask, "Display Task", 8192, NULL, 1, NULL);
}

void loop() {
  // Empty loop since tasks handle everything
}

// Task to read VL53L0X sensor
void vl53Task(void *pvParameters) {
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    if (!vl53.init()) {
      Serial.println("Failed to initialize VL53L0X");
      xSemaphoreGive(i2c_mutex);
      vTaskDelete(NULL);
    }
    xSemaphoreGive(i2c_mutex);
  }
  while (1) {
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
      vl53Distance = vl53.readRangeSingleMillimeters();
      xSemaphoreGive(i2c_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task to read MPU6050 sensor
void mpuTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
      mpu.getAcceleration(&ax, &ay, &az);
      xSemaphoreGive(i2c_mutex);
    }
    if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
      accelXData[dataIndex] = ax;
      accelYData[dataIndex] = ay;
      accelZData[dataIndex] = az;
      dataIndex = (dataIndex + 1) % DATA_BUFFER_SIZE;
      xSemaphoreGive(data_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task to read BMP280 sensor
void bmpTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
      bmpTemp = bmp.readTemperature();
      bmpPressure = bmp.readPressure();
      xSemaphoreGive(i2c_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task to read ENS160 sensor
void ensTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
      ens160.setPWRMode(ENS160_STANDARD_MODE);
      eco2 = ens160.getECO2();
      tvoc = ens160.getTVOC();
      xSemaphoreGive(i2c_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task to read AHT21 sensor
void ahtTask(void *pvParameters) {
  sensors_event_t humidityEvent, tempEvent;
  while (1) {
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
      aht.getEvent(&humidityEvent, &tempEvent);
      ahtTemp = tempEvent.temperature;
      ahtHumidity = humidityEvent.relative_humidity;
      xSemaphoreGive(i2c_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task to read GPS data
void gpsTask(void *pvParameters) {
  while (1) {
    if (gpsSerial.available()) {
      String tempData = gpsSerial.readStringUntil('\n');
      if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
        gpsData = tempData;
        xSemaphoreGive(data_mutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Display task
void displayTask(void *pvParameters) {
  int prevIndex = dataIndex;
  while (1) {
    if (prevIndex != dataIndex) {
      prevIndex = dataIndex;

      tft.startWrite();

      // Accelerometer Plot
      tft.fillRect(0, 0, 170, 80, ST77XX_BLACK);
      tft.drawRect(0, 0, 170, 80, ST77XX_WHITE);
      tft.setCursor(5, 5);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setTextSize(1);
      tft.print("Accelerometer (X,Y,Z)");

      if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
        for (int i = 1; i < DATA_BUFFER_SIZE; i++) {
          int x1 = i - 1;
          int x2 = i;
          int idx1 = (dataIndex + i - 1) % DATA_BUFFER_SIZE;
          int idx2 = (dataIndex + i) % DATA_BUFFER_SIZE;

          int y1_x = 40 - (accelXData[idx1] / 32768.0) * 40;
          int y2_x = 40 - (accelXData[idx2] / 32768.0) * 40;
          int y1_y = 40 - (accelYData[idx1] / 32768.0) * 40;
          int y2_y = 40 - (accelYData[idx2] / 32768.0) * 40;
          int y1_z = 40 - (accelZData[idx1] / 32768.0) * 40;
          int y2_z = 40 - (accelZData[idx2] / 32768.0) * 40;

          tft.drawLine(x1, y1_x, x2, y2_x, ST77XX_RED);
          tft.drawLine(x1, y1_y, x2, y2_y, ST77XX_GREEN);
          tft.drawLine(x1, y1_z, x2, y2_z, ST77XX_BLUE);
        }
        // Current values
        tft.setCursor(5, 65);
        tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        tft.printf("X: %d  ", ax);
        tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        tft.printf("Y: %d  ", ay);
        tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
        tft.printf("Z: %d", az);
        xSemaphoreGive(data_mutex);
      }

      // BMP280 Data
      tft.fillRect(0, 85, 170, 40, ST77XX_BLACK);
      tft.drawRect(0, 85, 170, 40, ST77XX_WHITE);
      tft.setCursor(5, 90);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.print("BMP280 Temp & Pressure");
      tft.setCursor(5, 105);
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.printf("Temp: %.2f C", bmpTemp);
      tft.setCursor(5, 120);
      tft.printf("Pressure: %.2f hPa", bmpPressure / 100.0);

      // ENS160 Data
      tft.fillRect(0, 130, 170, 40, ST77XX_BLACK);
      tft.drawRect(0, 130, 170, 40, ST77XX_WHITE);
      tft.setCursor(5, 135);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.print("ENS160 Air Quality");
      tft.setCursor(5, 150);
      tft.printf("eCO2: %d ppm", eco2);
      tft.setCursor(5, 165);
      tft.printf("TVOC: %d ppb", tvoc);

      // AHT21 Data
      tft.fillRect(0, 175, 170, 40, ST77XX_BLACK);
      tft.drawRect(0, 175, 170, 40, ST77XX_WHITE);
      tft.setCursor(5, 180);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.print("AHT21 Temp & Humidity");
      tft.setCursor(5, 195);
      tft.printf("Temp: %.2f C", ahtTemp);
      tft.setCursor(5, 210);
      tft.printf("Humidity: %.2f %%", ahtHumidity);

      // GPS Data
      tft.fillRect(0, 220, 170, 60, ST77XX_BLACK);
      tft.drawRect(0, 220, 170, 60, ST77XX_WHITE);
      tft.setCursor(5, 225);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.print("GPS Coordinates");
      if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
        tft.setCursor(5, 240);
        tft.print(gpsData);
        xSemaphoreGive(data_mutex);
      }

      tft.endWrite();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
