#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "SparkFun_BMV080_Arduino_Library.h"
#include <Wire.h>
#include <Notecard.h>
#include <time.h>

#define PRODUCT_UID "com.gmail.joshkroker:hsd_dev"
static constexpr char AUTH_TOKEN[] = "sk_8a6b6d4fb38feaa0b5b225d8306b36049c679cb89c15bbab1f3bdc0163ef6dd4";
static constexpr char DEVICE_ID[] = "sensor-001";
static constexpr char DEVICE_NAME[] = "Air Quality Monitor 1";
static constexpr char DEVICE_TYPE[] = "air-quality-monitor";
static constexpr char DEVICES_URL[] = "https://dev.jefftheme.dev/devices";

#define I2C_SDA A4
#define I2C_SCL A5
#define BME_I2C_ADDR 0x77  // Try 0x76 if 0x77 doesn't work

const uint32_t SERIAL_SPEED = 115200;
const uint8_t BMV080_ADDR = 0x57;
const uint32_t SENSOR_READ_INTERVAL = 1000;      // 12 seconds
const uint32_t DATA_SEND_INTERVAL = 30000;       // 10 minutes 
const uint8_t MAX_READINGS_BUFFER = 20;           // 10 min / 12 sec = 50 readings

Adafruit_BME680 bme;  // Using Adafruit library instead
SparkFunBMV080 bmv080;
Notecard notecard;

volatile bool i2cBusy = false;
volatile bool dataBusy = false;

struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float gasResistance;
  float altitude;
  float pm1;
  float pm25;
  float pm10;
  bool bmv080Valid;
  bool bmv080Obstructed;
  unsigned long timestamp;
  char timestampISO[30];  
};

SensorData readingsBuffer[MAX_READINGS_BUFFER];
uint8_t bufferIndex = 0;
uint8_t bufferCount = 0;
volatile bool hasDataToSend = false;

float altitude(const float press, const float seaLevel = 1013.25) {
  return 44330.0 * (1.0 - pow(press / seaLevel, 0.1903));
}

bool takeBusyFlag(volatile bool* flag, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (*flag) {
    if (millis() - start > timeoutMs) {
      return false;
    }
    delay(10);
  }
  *flag = true;
  return true;
}

void releaseBusyFlag(volatile bool* flag) {
  *flag = false;
}

// Get ISO 8601 timestamp from Notecard - assumes i2cBusy is already held by caller
bool getTimestampFromNotecard_Internal(char* isoTimestamp, size_t bufferSize) {
  Serial.println("[TIME] Requesting timestamp...");

  J *req = notecard.newRequest("card.time");
  if (!req) {
    Serial.println("[TIME] ERROR: Failed to create request");
    return false;
  }

  J *rsp = notecard.requestAndResponse(req);
  if (!rsp) {
    Serial.println("[TIME] ERROR: No response from Notecard");
    return false;
  }

  // Get the time as Unix timestamp
  double unixTime = JGetNumber(rsp, "time");
  Serial.printf("[TIME] Received Unix time: %.0f\n", unixTime);
  
  notecard.deleteResponse(rsp);

  if (unixTime == 0) {
    Serial.println("[TIME] ERROR: Unix time is 0");
    return false;
  }

  // Convert Unix timestamp to ISO 8601 format
  time_t rawTime = (time_t)unixTime;
  struct tm* timeinfo = gmtime(&rawTime);
  strftime(isoTimestamp, bufferSize, "%Y-%m-%dT%H:%M:%SZ", timeinfo);
  
  Serial.printf("[TIME] Generated ISO timestamp: %s\n", isoTimestamp);
  return true;
}

class NotecardManager {
public:
  void begin() {
    Serial.println("[NC] Initializing Notecard...");
    
    if (!takeBusyFlag(&i2cBusy, 5000)) {
      Serial.println("[NC] ERROR: I2C busy timeout");
      return;
    }

    notecard.begin();
    notecard.setDebugOutputStream(Serial);

    J *ver = notecard.newRequest("card.version");
    if (ver) {
      J *rsp = notecard.requestAndResponse(ver);
      if (rsp) {
        Serial.println("[NC] Notecard connected");
        notecard.deleteResponse(rsp);
      }
    }

    J *req = notecard.newRequest("hub.set");
    if (req) {
      JAddStringToObject(req, "product", PRODUCT_UID);
      JAddStringToObject(req, "mode", "continuous");
      notecard.sendRequest(req);
    }

    releaseBusyFlag(&i2cBusy);
    Serial.println("[NC] Ready");
  }

  bool registerDevice() {
    Serial.println("[NC] Registering device...");
    
    if (!takeBusyFlag(&i2cBusy, 10000)) {
      Serial.println("[NC] ERROR: I2C busy");
      return false;
    }

    J *req = notecard.newRequest("note.add");
    if (!req) {
      releaseBusyFlag(&i2cBusy);
      Serial.println("[NC] ERROR: Request creation failed");
      return false;
    }

    JAddStringToObject(req, "file", "device.qo");
    JAddBoolToObject(req, "sync", true);

    J *body = JCreateObject();
    if (!body) {
      releaseBusyFlag(&i2cBusy);
      Serial.println("[NC] ERROR: Body creation failed");
      return false;
    }

    JAddStringToObject(body, "request_type", "create_device");
    JAddStringToObject(body, "url", DEVICES_URL);
    JAddStringToObject(body, "auth_token", AUTH_TOKEN);
    JAddStringToObject(body, "deviceId", DEVICE_ID);
    JAddStringToObject(body, "name", DEVICE_NAME);
    JAddStringToObject(body, "type", DEVICE_TYPE);
    JAddStringToObject(body, "status", "active");
    JAddNumberToObject(body, "timestamp", millis());
    JAddItemToObject(req, "body", body);

    bool ok = notecard.sendRequest(req);
    releaseBusyFlag(&i2cBusy);
    
    Serial.printf("[NC] Registration %s\n", ok ? "OK" : "FAILED");
    return ok;
  }

  bool sendBatchedData(SensorData* readings, uint8_t count) {
    Serial.printf("[NC] Sending %d batched readings...\n", count);
    
    if (!takeBusyFlag(&i2cBusy, 10000)) {
      Serial.println("[NC] ERROR: I2C busy timeout during send");
      return false;
    }

    J *req = notecard.newRequest("note.add");
    if (!req) {
      Serial.println("[NC] ERROR: Failed to create note.add request");
      releaseBusyFlag(&i2cBusy);
      return false;
    }

    JAddStringToObject(req, "file", "data.qo");
    JAddBoolToObject(req, "sync", true);

    J *body = JCreateObject();
    if (!body) {
      Serial.println("[NC] ERROR: Failed to create body object");
      releaseBusyFlag(&i2cBusy);
      return false;
    }

    J *requests = JCreateArray();
    if (!requests) {
      Serial.println("[NC] ERROR: Failed to create requests array");
      releaseBusyFlag(&i2cBusy);
      return false;
    }

    Serial.printf("[NC] Building payload for %d readings...\n", count);
    
    for (uint8_t i = 0; i < count; i++) {
      SensorData &data = readings[i];
      
      J *requestObj = JCreateObject();
      if (!requestObj) {
        Serial.printf("[NC] WARNING: Failed to create request object %d\n", i);
        continue;
      }

      JAddStringToObject(requestObj, "deviceId", DEVICE_ID);
      
      J *readingsArray = JCreateArray();
      if (!readingsArray) {
        Serial.printf("[NC] WARNING: Failed to create readings array %d\n", i);
        continue;
      }

      // Temperature reading
      J *tempReading = JCreateObject();
      JAddStringToObject(tempReading, "readingType", "temperature");
      JAddStringToObject(tempReading, "unit", "°C");
      JAddNumberToObject(tempReading, "value", data.temperature);
      JAddStringToObject(tempReading, "timestamp", data.timestampISO);
      JAddItemToArray(readingsArray, tempReading);

      // Humidity reading
      J *humidityReading = JCreateObject();
      JAddStringToObject(humidityReading, "readingType", "humidity");
      JAddStringToObject(humidityReading, "unit", "%");
      JAddNumberToObject(humidityReading, "value", data.humidity);
      JAddStringToObject(humidityReading, "timestamp", data.timestampISO);
      JAddItemToArray(readingsArray, humidityReading);

      // Pressure reading
      J *pressureReading = JCreateObject();
      JAddStringToObject(pressureReading, "readingType", "pressure");
      JAddStringToObject(pressureReading, "unit", "hPa");
      JAddNumberToObject(pressureReading, "value", data.pressure);
      JAddStringToObject(pressureReading, "timestamp", data.timestampISO);
      JAddItemToArray(readingsArray, pressureReading);

      // Gas resistance reading
      J *gasReading = JCreateObject();
      JAddStringToObject(gasReading, "readingType", "gasResistance");
      JAddStringToObject(gasReading, "unit", "Ω");
      JAddNumberToObject(gasReading, "value", data.gasResistance);
      JAddStringToObject(gasReading, "timestamp", data.timestampISO);
      JAddItemToArray(readingsArray, gasReading);

      // Altitude reading
      J *altitudeReading = JCreateObject();
      JAddStringToObject(altitudeReading, "readingType", "altitude");
      JAddStringToObject(altitudeReading, "unit", "m");
      JAddNumberToObject(altitudeReading, "value", data.altitude);
      JAddStringToObject(altitudeReading, "timestamp", data.timestampISO);
      JAddItemToArray(readingsArray, altitudeReading);

      // PM1.0 reading
      J *pm1Reading = JCreateObject();
      JAddStringToObject(pm1Reading, "readingType", "pm1");
      JAddStringToObject(pm1Reading, "unit", "μg/m³");
      JAddNumberToObject(pm1Reading, "value", data.pm1);
      JAddStringToObject(pm1Reading, "timestamp", data.timestampISO);
      JAddItemToArray(readingsArray, pm1Reading);

      // PM2.5 reading
      J *pm25Reading = JCreateObject();
      JAddStringToObject(pm25Reading, "readingType", "pm25");
      JAddStringToObject(pm25Reading, "unit", "μg/m³");
      JAddNumberToObject(pm25Reading, "value", data.pm25);
      JAddStringToObject(pm25Reading, "timestamp", data.timestampISO);
      JAddItemToArray(readingsArray, pm25Reading);

      // PM10 reading
      J *pm10Reading = JCreateObject();
      JAddStringToObject(pm10Reading, "readingType", "pm10");
      JAddStringToObject(pm10Reading, "unit", "μg/m³");
      JAddNumberToObject(pm10Reading, "value", data.pm10);
      JAddStringToObject(pm10Reading, "timestamp", data.timestampISO);
      JAddItemToArray(readingsArray, pm10Reading);

      // Obstructed status (only if sensor was valid)
      if (data.bmv080Valid) {
        J *obstructedReading = JCreateObject();
        JAddStringToObject(obstructedReading, "readingType", "obstructed");
        JAddStringToObject(obstructedReading, "unit", "bool");
        JAddNumberToObject(obstructedReading, "value", data.bmv080Obstructed ? 1 : 0);
        JAddStringToObject(obstructedReading, "timestamp", data.timestampISO);
        JAddItemToArray(readingsArray, obstructedReading);
      }

      JAddItemToObject(requestObj, "readings", readingsArray);
      
      JAddItemToArray(requests, requestObj);
    }

    JAddItemToObject(body, "requests", requests);
    JAddItemToObject(req, "body", body);
    
    Serial.println("[NC] Sending request to Notecard...");
    bool ok = notecard.sendRequest(req);
    
    if (ok) {
      Serial.printf("[NC] ✓ Successfully sent %d readings\n", count);
    } else {
      Serial.printf("[NC] ✗ Failed to send %d readings\n", count);
    }
    
    releaseBusyFlag(&i2cBusy);
    return ok;
  }
};

NotecardManager notecardManager;

unsigned long lastSensorRead = 0;
unsigned long lastDataSend = 0;
unsigned long lastRegister = 0;
bool deviceRegistered = false;
int sensorReadCount = 0;
bool bme680GasReady = false;

void readSensors() {
  if (millis() - lastSensorRead < SENSOR_READ_INTERVAL) {
    return;
  }
  
  Serial.println("[SENS] Reading sensors...");
  
  // Take the I2C busy flag ONCE for all I2C operations
  if (!takeBusyFlag(&i2cBusy, 5000)) {
    Serial.println("[SENS] I2C busy, skipping");
    lastSensorRead = millis();
    return;
  }

  // Get timestamp from Notecard (while holding i2cBusy)
  char isoTimestamp[30];
  bool hasValidTimestamp = getTimestampFromNotecard_Internal(isoTimestamp, sizeof(isoTimestamp));
  
  if (!hasValidTimestamp) {
    Serial.println("[SENS] Failed to get timestamp, skipping reading");
    releaseBusyFlag(&i2cBusy);
    lastSensorRead = millis();
    return;
  }

  // Read BME680/688 sensor (while still holding i2cBusy)
  // Use performReading() which is blocking but simpler and more reliable
  if (!bme.performReading()) {
    Serial.println("[SENS] BME680/688 read failed");
    releaseBusyFlag(&i2cBusy);
    lastSensorRead = millis();
    return;
  }

  float temp = bme.temperature;
  float humidity = bme.humidity;
  float pressure = bme.pressure / 100.0;  // Convert Pa to hPa
  float gas = bme.gas_resistance / 1000.0;  // Convert to kOhms

  // Validate gas resistance reading
  bool gasValid = (gas >= 1.0 && gas <= 500.0);  // kOhms range
  if (!gasValid) {
    Serial.printf("[SENS] WARNING: Gas resistance out of range: %.1f kΩ\n", gas);
  }
  
  // Mark gas sensor as ready after first few readings
  if (sensorReadCount >= 2 && gasValid) {
    bme680GasReady = true;
  }

  // Read BMV080 sensor (while still holding i2cBusy)
  bool bmv080Valid = bmv080.readSensor();
  float pm1 = 0, pm25 = 0, pm10 = 0;
  bool obstructed = false;
  
  if (bmv080Valid) {
    pm1 = bmv080.PM1();
    pm25 = bmv080.PM25();
    pm10 = bmv080.PM10();
    obstructed = bmv080.isObstructed();
    Serial.printf("[SENS] BMV080 Valid - PM1:%.1f PM2.5:%.1f PM10:%.1f Obstructed:%s\n", 
                 pm1, pm25, pm10, obstructed ? "YES" : "NO");
  } else {
    Serial.println("[SENS] BMV080 Read FAILED - PM values will be 0");
  }
  
  // Done with I2C, release the flag
  releaseBusyFlag(&i2cBusy);

  // Now handle data buffer with dataBusy flag
  if (!takeBusyFlag(&dataBusy, 1000)) {
    Serial.println("[SENS] Data busy, skipping");
    lastSensorRead = millis();
    return;
  }

  // Skip first 2 readings to allow sensors to stabilize, then start buffering
  if (sensorReadCount >= 2) {  
    readingsBuffer[bufferIndex].temperature = temp;
    readingsBuffer[bufferIndex].humidity = humidity;
    readingsBuffer[bufferIndex].pressure = pressure;
    readingsBuffer[bufferIndex].gasResistance = gas;
    readingsBuffer[bufferIndex].altitude = altitude(pressure);
    readingsBuffer[bufferIndex].bmv080Valid = bmv080Valid;
    readingsBuffer[bufferIndex].pm1 = pm1;
    readingsBuffer[bufferIndex].pm25 = pm25;
    readingsBuffer[bufferIndex].pm10 = pm10;
    readingsBuffer[bufferIndex].bmv080Obstructed = obstructed;
    readingsBuffer[bufferIndex].timestamp = millis();
    strncpy(readingsBuffer[bufferIndex].timestampISO, isoTimestamp, sizeof(readingsBuffer[bufferIndex].timestampISO) - 1);
    
    bufferIndex = (bufferIndex + 1) % MAX_READINGS_BUFFER;
    if (bufferCount < MAX_READINGS_BUFFER) {
      bufferCount++;
    }
    
    hasDataToSend = true;
    
    Serial.printf("[SENS] T:%.1f H:%.1f P:%.1f Gas:%.1f PM2.5:%.1f @ %s [Buffer: %d/%d] %s\n", 
                 readingsBuffer[(bufferIndex - 1 + MAX_READINGS_BUFFER) % MAX_READINGS_BUFFER].temperature,
                 readingsBuffer[(bufferIndex - 1 + MAX_READINGS_BUFFER) % MAX_READINGS_BUFFER].humidity,
                 readingsBuffer[(bufferIndex - 1 + MAX_READINGS_BUFFER) % MAX_READINGS_BUFFER].pressure,
                 readingsBuffer[(bufferIndex - 1 + MAX_READINGS_BUFFER) % MAX_READINGS_BUFFER].gasResistance,
                 readingsBuffer[(bufferIndex - 1 + MAX_READINGS_BUFFER) % MAX_READINGS_BUFFER].pm25,
                 isoTimestamp,
                 bufferCount, MAX_READINGS_BUFFER,
                 bme680GasReady ? "" : "[GAS WARMING UP]");
  } else {
    Serial.printf("[SENS] Warmup reading %d/2 - T:%.1f H:%.1f P:%.1f Gas:%.1f kΩ\n",
                 sensorReadCount + 1,
                 temp, humidity, pressure, gas);
  }

  sensorReadCount++;
  releaseBusyFlag(&dataBusy);
  lastSensorRead = millis();
}

void sendData() {
  if (!hasDataToSend || bufferCount == 0 || millis() - lastDataSend < DATA_SEND_INTERVAL) {
    return;
  }
  
  Serial.printf("[SEND] Preparing to send %d readings...\n", bufferCount);
  
  if (!takeBusyFlag(&dataBusy, 1000)) {
    Serial.println("[SEND] Data busy");
    return;
  }
  
  SensorData sendBuffer[MAX_READINGS_BUFFER];
  uint8_t sendCount = bufferCount;
  
  uint8_t startIdx = (bufferIndex - bufferCount + MAX_READINGS_BUFFER) % MAX_READINGS_BUFFER;
  for (uint8_t i = 0; i < sendCount; i++) {
    sendBuffer[i] = readingsBuffer[(startIdx + i) % MAX_READINGS_BUFFER];
  }
  
  bufferCount = 0;
  bufferIndex = 0;
  hasDataToSend = false;
  
  releaseBusyFlag(&dataBusy);
  
  notecardManager.sendBatchedData(sendBuffer, sendCount);
  lastDataSend = millis();
}

void registerDevice() {
  if (deviceRegistered || millis() - lastRegister < 5000) {
    return;
  }
  
  Serial.println("[REG] Attempting registration...");
  deviceRegistered = notecardManager.registerDevice();
  lastRegister = millis();
  
  if (!deviceRegistered) {
    Serial.println("[REG] Will retry in 5s");
  }
}

void setup() {
  Serial.begin(SERIAL_SPEED);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  Air Quality Monitor - Batched Mode");
  Serial.println("  BME680/688 Compatible Version");
  Serial.println("========================================");
  Serial.printf("Read interval: %d sec\n", SENSOR_READ_INTERVAL / 1000);
  Serial.printf("Send interval: %d min\n", DATA_SEND_INTERVAL / 60000);
  Serial.printf("Max readings per batch: %d\n", MAX_READINGS_BUFFER);
  Serial.printf("Monthly requests: ~%d / 5000\n", (30 * 24 * 60 * 60 * 1000) / DATA_SEND_INTERVAL);
  Serial.printf("Free heap: %d bytes\n\n", ESP.getFreeHeap());

  Serial.println("[SETUP] Init I2C...");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(100);

  Serial.println("[SETUP] Init BME680/688...");
  bool bmeReady = false;
  
  // Try both I2C addresses
  if (bme.begin(BME_I2C_ADDR, &Wire)) {
    bmeReady = true;
    Serial.printf("[SETUP] BME680/688 found at 0x%02X\n", BME_I2C_ADDR);
  } else if (bme.begin(0x76, &Wire)) {
    bmeReady = true;
    Serial.println("[SETUP] BME680/688 found at 0x76");
  }

  if (bmeReady) {
    // Set oversampling for better accuracy
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    
    // Configure gas sensor - 320°C for 150ms per reading
    // The Adafruit library will handle triggering properly
    bme.setGasHeater(320, 150);
    
    Serial.println("[SETUP] BME680/688 configured successfully");
    Serial.println("[SETUP] Taking initial readings to warm up gas sensor...");
    
    // Take a few readings to warm up the sensor
    for (int i = 0; i < 5; i++) {
      if (bme.performReading()) {
        Serial.printf("[SETUP] Warmup %d/5 - Gas: %.1f kΩ\n", i + 1, bme.gas_resistance / 1000.0);
      } else {
        Serial.printf("[SETUP] Warmup %d/5 - Read failed\n", i + 1);
      }
      delay(2000);
    }
    Serial.println("[SETUP] Gas sensor warmup complete");
  } else {
    Serial.println("[SETUP] BME680/688 FAILED - check wiring and I2C address");
  }

  Serial.println("[SETUP] Init BMV080...");
  if (!bmv080.begin(BMV080_ADDR, Wire)) {
    Serial.println("[SETUP] BMV080 NOT FOUND!");
    while (1) {
      delay(5000);
      Serial.println("[SETUP] HALTED - No BMV080");
    }
  }
  bmv080.init();
  bmv080.setMode(SF_BMV080_MODE_CONTINUOUS);
  Serial.println("[SETUP] BMV080 OK");

  Serial.println("[SETUP] Init Notecard...");
  notecardManager.begin();

  Serial.println("\n========================================");
  Serial.println("  Setup Complete");
  Serial.println("========================================\n");
}

void loop() {
  static unsigned long lastStatus = 0;
  
  registerDevice();
  readSensors();
  sendData();
  
  if (millis() - lastStatus > 30000) {
    Serial.printf("\n[STATUS] Uptime: %lu s, Heap: %d, Reg: %s, Buffer: %d/%d, Gas: %s\n",
                 millis() / 1000, ESP.getFreeHeap(),
                 deviceRegistered ? "YES" : "NO",
                 bufferCount, MAX_READINGS_BUFFER,
                 bme680GasReady ? "READY" : "WARMING");
    Serial.printf("[STATUS] Next send in: %lu sec\n\n",
                 (DATA_SEND_INTERVAL - (millis() - lastDataSend)) / 1000);
    lastStatus = millis();
  }
  
  delay(100);
}