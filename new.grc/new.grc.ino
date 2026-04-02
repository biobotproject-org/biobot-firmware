#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "bmv080.h"
#include <BMV080.h>  // Ensure this is the BMV080 driver with convenience getters

// Debug mode - set to true for 5 minute intervals, false for 1 hour intervals
#define DEBUG_MODE true

// Authentication token - set your token here
const char* authToken = "your-auth-token-here";

// Hardware Serial for Notecard
HardwareSerial noteSerial(1);
const int RX_PIN = 3;
const int TX_PIN = 4;
const int BAUD_RATE = 9600;

// API configuration
const char* devicesUrl = "https://dev.jefftheme.dev/devices";
const char* sensorDataUrl = "https://dev.jefftheme.dev/sensordata";

const char* deviceId = "device-123";
const char* deviceName = "Soil Sensor Unit 1";
const char* deviceType = "soil-monitor";
const char* deviceStatus = "active";

const int pressureSensorPin = A6;
int sensorValue = 0;

// BME680 sensor
Adafruit_BME680 bme680;

// BMV080 sensor
BMV080 bmv080;

// Dummy placeholders for air quality & VOC index
float airQuality = 0;
float vocIndex = 0;

unsigned long lastStatusUpdate = 0;
unsigned long lastSensorDataSend = 0;
const unsigned long statusUpdateInterval = 60000;
const unsigned long sensorReadInterval = DEBUG_MODE ? 300000 : 3600000;
unsigned long lastSensorRead = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(pressureSensorPin, INPUT);
  Wire.begin();

  // Init BME680
  if (!bme680.begin()) {
    Serial.println("BME680 not detected");
  } else {
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);
  }

  // Init BMV080
  if (!bmv080.begin()) {
    Serial.println("BMV080 not detected! Check wiring.");
  } else {
    Serial.println("BMV080 sensor initialized!");
  }

  noteSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(500);
}

void loop() {
  if (noteSerial.available()) Serial.write(noteSerial.read());

  if (millis() - lastSensorRead >= 500) {
    sensorValue = analogRead(pressureSensorPin);
    lastSensorRead = millis();
  }

  if (millis() - lastStatusUpdate >= statusUpdateInterval) {
    sendStatusUpdate();
    lastStatusUpdate = millis();
  }

  if (millis() - lastSensorDataSend >= sensorReadInterval) {
    sendSensorData();
    lastSensorDataSend = millis();
  }
}

void sendRequest(const char *jsonStr) {
  noteSerial.print(jsonStr);
  noteSerial.println();
  noteSerial.flush();
}

void sendSensorData() {
  // BME680 reading
  if (!bme680.performReading()) {
    Serial.println("BME680 read failed");
  }

  // BMV080 particulate readings
  float pm1 = bmv080.readPM1_0();
  float pm25 = bmv080.readPM2_5();
  float pm10 = bmv080.readPM10();

  // Use dummy placeholders for IAQ and CO2 (not supported in BMV080 driver)
  float iaq = 0;
  float co2 = 0;

  StaticJsonDocument<1024> doc;
  doc["req"] = "note.add";
  doc["file"] = "data.qo";
  doc["sync"] = true;

  JsonObject body = doc.createNestedObject("body");
  body["request_type"] = "sensor_data";
  body["url"] = sensorDataUrl;
  body["auth_token"] = authToken;
  body["deviceId"] = deviceId;
  body["timestamp"] = millis();

  JsonArray readings = body.createNestedArray("readings");

  JsonObject soilReading = readings.createNestedObject();
  soilReading["readingType"] = "soil_pressure";
  soilReading["value"] = sensorValue;
  soilReading["unit"] = "raw";

  JsonObject tempReading = readings.createNestedObject();
  tempReading["readingType"] = "temperature";
  tempReading["value"] = bme680.temperature;
  tempReading["unit"] = "celsius";

  JsonObject humidityReading = readings.createNestedObject();
  humidityReading["readingType"] = "humidity";
  humidityReading["value"] = bme680.humidity;
  humidityReading["unit"] = "percent";

  JsonObject airQReading = readings.createNestedObject();
  airQReading["readingType"] = "air_quality";
  airQReading["value"] = airQuality;
  airQReading["unit"] = "index";

  JsonObject pm1Reading = readings.createNestedObject();
  pm1Reading["readingType"] = "pm1_0";
  pm1Reading["value"] = pm1;
  pm1Reading["unit"] = "ug/m3";

  JsonObject pm25Reading = readings.createNestedObject();
  pm25Reading["readingType"] = "pm2_5";
  pm25Reading["value"] = pm25;
  pm25Reading["unit"] = "ug/m3";

  JsonObject pm10Reading = readings.createNestedObject();
  pm10Reading["readingType"] = "pm10";
  pm10Reading["value"] = pm10;
  pm10Reading["unit"] = "ug/m3";

  JsonObject iaqReading = readings.createNestedObject();
  iaqReading["readingType"] = "iaq";
  iaqReading["value"] = iaq;
  iaqReading["unit"] = "index";

  JsonObject co2Reading = readings.createNestedObject();
  co2Reading["readingType"] = "co2_estimate";
  co2Reading["value"] = co2;
  co2Reading["unit"] = "ppm";

  JsonObject vocReading = readings.createNestedObject();
  vocReading["readingType"] = "voc_index";
  vocReading["value"] = vocIndex;
  vocReading["unit"] = "index";

  String jsonStr;
  serializeJson(doc, jsonStr);
  sendRequest(jsonStr.c_str());
}

void sendStatusUpdate() {
  StaticJsonDocument<512> doc;
  doc["req"] = "note.add";
  doc["file"] = "status.qo";
  doc["sync"] = true;

  JsonObject body = doc.createNestedObject("body");
  body["request_type"] = "status_update";
  body["url"] = String(devicesUrl) + "/" + deviceId + "/status";
  body["auth_token"] = authToken;
  body["deviceId"] = deviceId;
  body["status"] = "active";
  body["timestamp"] = millis();

  String jsonStr;
  serializeJson(doc, jsonStr);
  sendRequest(jsonStr.c_str());
}
