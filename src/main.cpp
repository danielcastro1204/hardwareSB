#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PubSubClient.h>

const char* ssid = "SILVIA_ESCOBAR";
const char* password = "13303110";

String url = "http://192.168.220.173:8080/data/create";

const char* mqttServer = "broker.hivemq.com";  // Servidor MQTT
const int mqttPort = 1883;                     // Puerto MQTT
const char* clientName = "ESP32ClientIcesi";   // Nombre del cliente MQTT
const char* topic = "icesitel";       // Topic para recibir comandos

HTTPClient http;
Adafruit_MPU6050 mpu;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

int samplingRateHz = 20;   
int numSamples = 200;      
int samplingIntervalMs = 1000 / samplingRateHz;

StaticJsonDocument<1024> allMeasurements;
JsonArray measurementsArray = allMeasurements.createNestedArray("measurements");

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

// Función para agregar una medición al arreglo de mediciones
void addMeasurement(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {
  StaticJsonDocument<256> jsonDoc;

  jsonDoc["xac"] = accX;     // Aceleración X
  jsonDoc["yac"] = accY;     // Aceleración Y
  jsonDoc["zac"] = accZ;     // Aceleración Z
  jsonDoc["xgi"] = gyroX;    // Giroscopio X
  jsonDoc["ygi"] = gyroY;    // Giroscopio Y
  jsonDoc["zgi"] = gyroZ;    // Giroscopio Z

  measurementsArray.add(jsonDoc);
}

// Función para enviar todas las mediciones acumuladas a través de HTTP POST
void sendAllDataToServer() {
  String jsonData;
  serializeJson(allMeasurements, jsonData);

  Serial.print("JSON data: ");
  Serial.println(jsonData);

  http.begin(url.c_str());
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(jsonData);
  Serial.print("Respuesta del servidor: ");
  Serial.println(httpResponseCode);
  http.end();
}

// Leer datos de todos los sensores, almacenarlos y enviar en formato JSON al final
void readAndSendAllSensors() {
  measurementsArray.clear();
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    addMeasurement(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);
    
    delay(samplingIntervalMs);
  }
  
  sendAllDataToServer();
}

// Callback que se activa cuando llega un mensaje MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el topic ");
  Serial.print(topic);
  Serial.print(": ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  if (message == "m") {
    Serial.println("Iniciando medición de todos los sensores");
    readAndSendAllSensors();
  } else {
    Serial.println("Comando no reconocido");
  }
}

// Conectar al broker MQTT
void connectToMQTTBroker() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
  
  while (!mqttClient.connected()) {
    Serial.println("Intentando conectar al servidor MQTT...");
    if (mqttClient.connect(clientName)) {
      Serial.println("Conectado al servidor MQTT!");
      mqttClient.subscribe(topic); 
      Serial.println("Suscrito al topic icesitel");
    } else {
      Serial.print("Error al conectar: ");
      Serial.println(mqttClient.state());
      delay(5000);
    }
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
  connectToMQTTBroker();  

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
  mqttClient.loop(); 
}