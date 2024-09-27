#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

const char* ssid = "gladys";
const char* password = "gladysss";

String url =  "http://192.168.249.173:8080/data/create";

HTTPClient http;
Adafruit_MPU6050 mpu;

int samplingRateHz = 20;   
int numSamples = 200;      
int samplingIntervalMs = 1000 / samplingRateHz;

// Conectar a WiFi
void runWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado a WiFi");
  Serial.println(WiFi.localIP());
}

// Enviar datos a través de HTTP POST
void sendDataToServer(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
  StaticJsonDocument<256> jsonDoc;

  jsonDoc["xac"] = accX;     // Aceleración X
  jsonDoc["yac"] = accY;     // Aceleración Y
  jsonDoc["zac"] = accZ;     // Aceleración Z
  jsonDoc["xgi"] = gyroX;  // Giroscopio X
  jsonDoc["ygi"] = gyroY;  // Giroscopio Y
  jsonDoc["zgi"] = gyroZ;  // Giroscopio Z

  String jsonData;
  serializeJson(jsonDoc, jsonData);

  Serial.print("JSON data: ");
  Serial.println(jsonData);


  http.begin(url.c_str());
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(jsonData);
  Serial.print("Respuesta del servidor: ");
  Serial.println(httpResponseCode);
  http.end();
}

// Leer datos de todos los sensores y enviar en formato JSON
void readAndSendAllSensors() {
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sendDataToServer(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);
    
    delay(samplingIntervalMs);
  }
}

// Inicializar la ESP32
void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Error al inicializar el MPU6050");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 inicializado correctamente");

  runWiFi();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

// Loop principal del programa
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexión WiFi perdida. Intentando reconectar...");
    runWiFi();
  }
}

// Detectar eventos del puerto serie
void serialEvent() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data == "m") {
      readAndSendAllSensors();
    }
  }
}