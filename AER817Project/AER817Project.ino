#include "esp_camera.h"
#include <WiFi.h>

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_XIAO_ESP32S3  // Has PSRAM
#include "camera_pins.h"

// ===========================
// WiFi credentials
// ===========================
const char *ssid = "A Pixel 8";
const char *password = "Arjun0701";

// LIDAR configuration
HardwareSerial lidarSerial(1);  // Use UART1 for LIDAR communication
#define LIDAR_RX_PIN 43         // LIDAR TX connects to ESP32 RX
#define LIDAR_TX_PIN 44         // LIDAR RX connects to ESP32 TX


String lidarData = "No Data";  // Shared LIDAR data for the web server
String lidarStrength = "No Data";
String lidarTemperature = "No Data";

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.println();

  // Initialize LIDAR Serial Communication
  lidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.println("LIDAR initialized...");

  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    if (err == ESP_ERR_NO_MEM) {
      Serial.println("Not enough memory to allocate buffers.");
    } else if (err == ESP_ERR_INVALID_ARG) {
      Serial.println("Invalid configuration arguments.");
    } else if (err == ESP_ERR_INVALID_STATE) {
      Serial.println("Camera already initialized.");
    } else {
      Serial.println("Unknown error.");
    }
    while (true) delay(1);  // Halt execution
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start Camera Server
  startCameraServer();
  Serial.println("Camera Web Server started.");
}

void loop() {
  if (lidarSerial.available() >= 9) {
    uint8_t buf[9];
    lidarSerial.readBytes(buf, 9);

    // Validate the packet header
    if (buf[0] == 0x59 && buf[1] == 0x59) {
      uint16_t distance = buf[2] + (buf[3] << 8);       // Combine two bytes for distance
      uint16_t strength = buf[4] + (buf[5] << 8);       // Combine two bytes for strength
      int16_t rawTemperature = buf[6] + (buf[7] << 8);  // Combine two bytes for temperature

      // Correct the temperature as per the sensor datasheet
      float temperature = (rawTemperature / 8.0) - 256.0;

      // Print parsed values
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" cm, Strength: ");
      Serial.print(strength);
      Serial.print(", Temperature: ");
      Serial.println(temperature);

      // Update shared variable for the web server
      lidarData = String(distance) + " cm";
      lidarStrength = String(strength);
      lidarTemperature = String(temperature, 2) + " °C";  // Limit temperature to 2 decimal places
    } else {
      Serial.println("Invalid LIDAR data packet.");
    }
  }
}
