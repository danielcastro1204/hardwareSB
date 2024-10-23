#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PubSubClient.h>

const char* ssid = "gladys";
const char* password = "gladysss";

String url = "http://192.168.151.173:8080/measure";

const char* mqttServer = "broker.emqx.io";             // Servidor MQTT
const int mqttPort = 1883;                             // Puerto MQTT
const char* clientName = "ESP32ClientIcesiA00123456";  // Nombre del cliente MQTT
const char* topic = "icesitel";                        // Topic para recibir comandos

HTTPClient http;
Adafruit_MPU6050 mpu;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

int samplingRateHz = 20;                               // Frecuencia de muestreo
int numSamples = 200;                                  // Número de muestras a recolectar
int samplingIntervalMs = 1000 / samplingRateHz;

StaticJsonDocument<4096> allMeasurements;
JsonObject readings = allMeasurements.createNestedObject("readings");
JsonArray accelerometer = readings.createNestedArray("accelerometer");
JsonArray gyroscope = readings.createNestedArray("gyroscope");

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

// Agregar una medición a los arrays de JSON
void addMeasurement(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float timestamp) {
  JsonObject accMeasurement = accelerometer.createNestedObject();
  accMeasurement["x"] = accX;
  accMeasurement["y"] = accY;
  accMeasurement["z"] = accZ;
  accMeasurement["timestamp"] = timestamp;

  JsonObject gyroMeasurement = gyroscope.createNestedObject();
  gyroMeasurement["x"] = gyroX;
  gyroMeasurement["y"] = gyroY;
  gyroMeasurement["z"] = gyroZ;
  gyroMeasurement["timestamp"] = timestamp;
}

// Función para enviar los datos por HTTP POST
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

// Leer y enviar datos de los sensores
void readAndSendAllSensors() {
  accelerometer.clear();
  gyroscope.clear();

  unsigned long startTime = millis();

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float currentTime = (millis() - startTime) / 1000.0;
    addMeasurement(a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, currentTime);

    delay(samplingIntervalMs);
  }

  sendAllDataToServer();

  mqttClient.publish(topic, "E");
  Serial.println("Mensaje 'E' enviado.");
}

// Callback para mensajes MQTT
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
    Serial.println("Iniciando medición de los sensores");
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

// Configuración inicial
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

  Serial.println("Envía 'm' por MQTT para iniciar la medición.");
}

// Loop principal
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexión WiFi perdida. Intentando reconectar...");
    runWiFi();
  }
  mqttClient.loop(); 
}