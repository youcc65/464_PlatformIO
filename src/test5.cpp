// 测试手表卡


#include <Arduino.h> 
#include <PulseSensorPlayground.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_GPS.h> 
#include <vector>
#include "secrets.h"
#include <esp_system.h> 

// NTP configuration
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -4 * 3600, 1000);

// Stepcount
static uint32_t stepCount = 0; 
static uint32_t stepCountOffset = 0;

// GPS
Adafruit_GPS GPS(&Wire);
#define GPSECHO false
uint32_t gpsTimer;
bool gpsAvailable = false;

//Firebase sending time!!!!!!!
unsigned long lastFirebaseUpdate = 0;
const unsigned long uploadInterval = 60000;  
const int maxDataPoints = 12;               
int dataPointsCount = 0;

// Display configuration
#define TFT_CS    25
#define TFT_RST   27
#define TFT_DC    26
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool pulseAvailable = false; 
bool bmpAvailable   = false;
bool bnoAvailable   = false;

// Pulse Sensor configuration
const int PulseWire = 35;
const int LED13 = 13;
int Threshold = 550;
PulseSensorPlayground pulseSensor;

// BMP388 configuration
#define BMP388_ADDRESS 0x77
Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25)

// BNO08x configuration
#define BNO08X_ADDRESS 0x4A
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Button configuration
const int buttonPin = 2;
int lastButtonState = HIGH;
enum DisplayMode { CLOCK, HEART_RATE, TEMP_PRESSURE, ALTITUDE, MOTION };
DisplayMode currentMode = CLOCK;

// Clock variables
unsigned long previousMillis = 0;
const long interval = 1000;
int seconds = 0, minutes = 0, hours = 0;
bool initialClockDraw = true;

// ============= For long press detection =============
static unsigned long buttonPressStartTime = 0;   // record the time after press
static const unsigned long longPressDuration = 2000; // treat as long press if more than two seconds
static const unsigned long restartPressDuration = 8000; // 8 s 重启

// ============= If is recording =============
bool isRecording = false;

// ============= Define a structure to record sensor data =============
struct SensorData {
  unsigned long timeStamp; 
  int heartRate;
  float temperature;
  float pressure;
  float altitude;
  uint32_t steps;
};
// use std::vector to record data
std::vector<SensorData> recordedData;

// ============= Define a structure to record GPS data ==============
struct GPSData {
    unsigned long timeStamp;
    float latitude;
    float longitude;
    float speed;
    float angle;
    float altitude;
    int satellites;
  };
  std::vector<GPSData> recordedGPSData;

// ============= recordedData time interval =============
unsigned long lastRecordTime = 0;
const unsigned long recordInterval = 10000; // record Every 10 seconds

// Wi‑Fi & NTP
void connectToWiFi();
extern NTPClient timeClient;  
// Firebase
void sendDataToFirebase(const String& path, const String& jsonData);
void uploadSensorDataToFirebase();
void uploadRecordedDataToFirebase();
void uploadRecordedGPSDataToFirebase();
void handleButton();
void updateClock();
void drawDigitalClock();
void updateSensors();
void recordCurrentData();
void updateDisplay();
void displayHeartRate();
void displayTempPressure();
void displayAltitude();
void displayMotion();
void drawHeader(const char* title);
void displayValue(const char* label, String value, int yPos, uint16_t color);
void drawRecordingIndicator(bool isOn);
void configureBMP388();
void configureBNO08x();






//==========================================void setup=====================================

void setup() {
  Serial.begin(115200);
  
  // Initialize display
  tft.init(240, 240);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  
  // Initialize pulse sensor
  pulseSensor.analogInput(PulseWire);
  pulseSensor.blinkOnPulse(LED13);
  pulseSensor.setThreshold(Threshold);
  if (!pulseSensor.begin()) {
    Serial.println("Pulse sensor NOT connected!");
    pulseAvailable = false;
  } else {
    pulseAvailable = true;
  }

  // Initialize I2C sensors
  Wire.begin(21, 22);

  //  BMP388
  if (!bmp.begin_I2C(BMP388_ADDRESS)) {
    Serial.println("BMP388 NOT connected!");
    bmpAvailable = false;
  } else {
    bmpAvailable = true;
    configureBMP388();
  }

  //  BNO08x
  if (!bno08x.begin_I2C(BNO08X_ADDRESS)) {
    Serial.println("BNO08x NOT connected!");
    bnoAvailable = false;
  } else {
    bnoAvailable = true;
    configureBNO08x();
  }

  // GPS
  gpsAvailable=GPS.begin(0x10);
  if (gpsAvailable) {
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    GPS.println(PMTK_Q_RELEASE);
  }

  // Configure button
  pinMode(buttonPin, INPUT_PULLUP);

  // Connect to Wi-Fi
  connectToWiFi();

  // Initialize NTP client
  timeClient.begin();
  timeClient.update();
}



//=======================================void loop===========================================
void loop() {
  bool ntpSuccess = false;

  if (WiFi.status() == WL_CONNECTED) {
    ntpSuccess = timeClient.update();  
  }

  if (ntpSuccess) {
    // use NTP time
    hours   = timeClient.getHours();
    minutes = timeClient.getMinutes();
    seconds = timeClient.getSeconds();
  } else {
    
    updateClock();
  }

  // GPS 
  //char c = GPS.read();
  //if (GPS.newNMEAreceived()) {
    //if (GPS.parse(GPS.lastNMEA())) {
     // gpsAvailable = true;
    //}
  //}

  handleButton();
  updateSensors();
  updateDisplay();

// Firebase senting time !!!!!!
if (millis() - lastFirebaseUpdate >= uploadInterval) {
  lastFirebaseUpdate = millis();
  uploadSensorDataToFirebase();
  if (gpsAvailable && !recordedGPSData.empty()) {
    uploadRecordedGPSDataToFirebase();
  }
}

  if (dataPointsCount < maxDataPoints) {
    
    dataPointsCount++;
  }

  // ============= If recording, get data =============
  if (isRecording) {
    if (millis() - lastRecordTime >= recordInterval) {
      lastRecordTime = millis();
      recordCurrentData();  // **必须**调用，才能往 recordedData/recordedGPSData 里 push
    }
    static unsigned long lastGPS = 0;
    if (millis() - lastGPS >= 1000) {
      lastGPS = millis();
      char c;
      while ((c = GPS.read())) {}
      if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA())) {
        gpsAvailable = true;
      }
    }
  }

  delay(100);
}

//-----------------------------------------------------
// Wi-Fi
//-----------------------------------------------------
void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
}

//-----------------------------------------------------
// Firebase
//-----------------------------------------------------
void sendDataToFirebase(const String& path, const String& jsonData) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String url = "https://" + String(firebaseHost) + "/" + path + ".json?auth=" + String(firebaseSecret);

    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Data sent to Firebase!");
      Serial.println("Response: " + response);
    } else {
      Serial.print("Error sending data. HTTP code: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("Wi-Fi disconnected.");
  }
}

void uploadSensorDataToFirebase() {
  StaticJsonDocument<200> jsonDoc;

  
  int bpm = pulseAvailable ? pulseSensor.getBeatsPerMinute() : 0;
  
  float temperature = 0.0;
  float pressure    = 0.0;
  float altitude    = 0.0;
  if (bmpAvailable) {
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0;
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }

  float motionX = 0.0, motionY = 0.0, motionZ = 0.0;
  if (bnoAvailable) {
    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      motionX = sensorValue.un.accelerometer.x;
      motionY = sensorValue.un.accelerometer.y;
      motionZ = sensorValue.un.accelerometer.z;
    }
    
  }

  // JSON
  jsonDoc["heart_rate"] = bpm;
  jsonDoc["temperature"] = temperature;
  jsonDoc["pressure"] = pressure;
  jsonDoc["altitude"] = altitude;
  jsonDoc["step_count"] = stepCount;

  String jsonString;
  serializeJson(jsonDoc, jsonString);

  sendDataToFirebase("users/rlrW9nxLkjcFQCWRJJSNI4DXn5x1/sensorData", jsonString);
}


void handleButton() {
  int buttonState = digitalRead(buttonPin);

  // detect pressing button
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressStartTime = millis();
  } 
  else if (buttonState == HIGH && lastButtonState == LOW) {
    unsigned long pressDuration = millis() - buttonPressStartTime;

    if (pressDuration < longPressDuration) {
      // 短按：切换显示模式
      currentMode = static_cast<DisplayMode>((currentMode + 1) % 5);
      tft.fillScreen(ST77XX_BLACK);
      initialClockDraw = true;
    }
    else if (pressDuration < 10000) {
      // 长按 ≥2秒 且 <10秒：录制开关
      isRecording = !isRecording;
      if (isRecording) {
        recordedData.clear();
        recordedGPSData.clear();
        Serial.println("=== Start Recording Data ===");
        lastRecordTime = millis();
        stepCountOffset = stepCount;
        drawRecordingIndicator(true);
      } else {
        Serial.println("=== Stop Recording Data, uploading... ===");
        uploadRecordedDataToFirebase();
        if (gpsAvailable && !recordedGPSData.empty()) {
          uploadRecordedGPSDataToFirebase();
        }
        drawRecordingIndicator(false);
      }
    }
    else {
      // 超长按 ≥10秒：软件重启
      Serial.println("Long press ≥10s detected, restarting ESP32...");
      ESP.restart();
    }
  }

  lastButtonState = buttonState;
}


void updateSensors() {
  //static unsigned long lastFirebaseUpdate = 0;
  static unsigned long lastBMPCheck = 0;
  

  unsigned long now = millis();
  if (now - lastBMPCheck >= 1000) {
    lastBMPCheck = now;
    if (bmpAvailable) {
      // If available, test if performReading() successful
      bool readOk = bmp.performReading();
      if (!readOk) {
        // 
        bmpAvailable = false;
        Serial.println("BMP388 disconnected!");
      }
    } else {
      
      Serial.println("Trying to reconnect BMP388...");
      if (bmp.begin_I2C(BMP388_ADDRESS)) {
        
        configureBMP388();
        bmpAvailable = true;
        Serial.println("BMP388 reconnected!");
      } else {
        
        Serial.println("BMP388 still not connected...");
      }
    }
  }  
  
  


  switch (currentMode) {
    case HEART_RATE:
      
      if (pulseAvailable) {
        pulseSensor.getBeatsPerMinute();
      }
      break;
    case TEMP_PRESSURE:
    case ALTITUDE:
      if (bmpAvailable) {
        bmp.performReading();
      }
      break;
    case MOTION:
      if (bnoAvailable) {
        
        if (bno08x.getSensorEvent(&sensorValue)) {
          if (sensorValue.sensorId == SH2_STEP_COUNTER) {
            stepCount = sensorValue.un.stepCounter.steps;
          }
        }
      }
      break;
    default:
      break;
  }
}

//-----------------------------------------------------
// Display
//-----------------------------------------------------
void updateDisplay() {
  switch (currentMode) {
    case CLOCK:
      drawDigitalClock();
      break;
    case HEART_RATE:
      displayHeartRate();
      break;
    case TEMP_PRESSURE:
      displayTempPressure();
      break;
    case ALTITUDE:
      displayAltitude();
      break;
    case MOTION:
      displayMotion();
      break;
  }
  
  drawRecordingIndicator(isRecording);
}


void drawDigitalClock() {
  static int lastSecond = -1;
  
  if (initialClockDraw) {
    tft.fillScreen(ST77XX_BLACK);
    drawHeader("Digital Clock");
    initialClockDraw = false;
  }

  if (seconds != lastSecond) {
    lastSecond = seconds;
    
    tft.setTextSize(4);
    tft.setTextColor(ST77XX_WHITE);
    
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", hours, minutes, seconds);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);
    int xPos = (240 - w) / 2;
    int yPos = (240 - h) / 2;
    
    
    tft.fillRect(xPos - 5, yPos - 5, w + 10, h + 10, ST77XX_BLACK);
    
    tft.setCursor(xPos, yPos);
    tft.print(timeStr);

    tft.setTextSize(1);
    tft.setCursor(80, 220);
    tft.print("HH:MM:SS 24H");
  }
}


void updateClock() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    if (++seconds >= 60) {
      seconds = 0;
      minutes++;
      if (++minutes >= 60) {
        minutes = 0;
        hours = (hours + 1) % 24; 
      }
    }
  }
}


void displayHeartRate() {
  drawHeader("Heart Rate");
  
  
  if (!pulseAvailable) {
    displayValue("PulseSensor", "Not Connected", 100, ST77XX_RED);
    return;
  }

  int bpm = pulseSensor.getBeatsPerMinute();

  tft.setCursor(20, 80);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Status: ");
  
  if (pulseSensor.sawStartOfBeat()) {
    if (bpm >= 55 && bpm <= 100) {
      tft.setTextColor(ST77XX_GREEN);
      tft.print("Detect");
      displayValue("BPM:", String(bpm), 120, ST77XX_GREEN);
    } else {
      tft.setTextColor(ST77XX_YELLOW);
      tft.print("...");
    }
  } else {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Detecting...");
  }
}


void displayTempPressure() {
  drawHeader("Environment");
  
  if (!bmpAvailable) {
    displayValue("BMP388", "Not Connected", 80, ST77XX_RED);
  } else {
    displayValue("Temp:", String(bmp.temperature, 1) + " C", 60, ST77XX_YELLOW);
    displayValue("Press:", String(bmp.pressure / 100.0, 1) + " hPa", 120, ST77XX_CYAN);
  }
}


void displayAltitude() {
  drawHeader("Altitude");
  
  if (!bmpAvailable) {
    displayValue("BMP388", "Not Connected", 90, ST77XX_RED);
  } else {
    float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    displayValue("Height:", String(alt, 1) + " m", 90, ST77XX_GREEN);
  }
}



void displayMotion() {
  drawHeader("Step Counter");

  if (!bnoAvailable) {
    displayValue("BNO08x", "Not Connected", 120, ST77XX_RED);
    return;
  }

  uint32_t displaySteps = isRecording
      ? (stepCount - stepCountOffset)
      : stepCount;
  
  displayValue("Steps:", String(displaySteps), 100, ST77XX_GREEN);
}


void drawHeader(const char* title) {
  tft.fillRect(0, 0, 240, 40, ST77XX_BLUE);
  tft.setCursor(10, 10);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(title);
}

void displayValue(const char* label, String value, int yPos, uint16_t color) {
  tft.fillRect(0, yPos - 10, 240, 40, ST77XX_BLACK);
  tft.setCursor(20, yPos - 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(label);
  
  tft.setCursor(40, yPos + 10);
  tft.setTextColor(color);
  tft.print(value);
}


void configureBMP388() {
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void configureBNO08x() {
  bno08x.enableReport(SH2_STEP_COUNTER);
  //bno08x.enableReport(SH2_ACCELEROMETER);
  //bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  //bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);
}


// =============  =============
void recordCurrentData() {
  SensorData data;
  data.timeStamp = timeClient.getEpochTime(); 
  
  data.heartRate = pulseAvailable ? pulseSensor.getBeatsPerMinute() : 0;

  if (bmpAvailable) {
    data.temperature = bmp.temperature;
    data.pressure    = bmp.pressure / 100.0;
    data.altitude    = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  } else {
    data.temperature = 0;
    data.pressure    = 0;
    data.altitude    = 0;
  }
  data.steps = isRecording
    ? (stepCount - stepCountOffset)
    : stepCount;

  recordedData.push_back(data);

  //GPS
  GPSData gd;
  if (gpsAvailable) {
  gd.timeStamp  = timeClient.getEpochTime();
  gd.latitude   = GPS.latitude;
  gd.longitude  = GPS.longitude;
  gd.speed      = GPS.speed;
  gd.angle      = GPS.angle;
  gd.altitude   = GPS.altitude;
  gd.satellites = GPS.satellites;
  recordedGPSData.push_back(gd);
  }

  Serial.println("Recorded one sensor data entry" 
    + String(gpsAvailable ? " + GPS" : ""));
}


// ============= upload recordedData to Firebase =============
void uploadRecordedDataToFirebase() {
  // 这里要注意 JSON 内存大小可能不够，
  // 如果记录时间过长、数据量大，需要改用动态分配或分批上传
  // 这里先简单示例
  StaticJsonDocument<4096> doc; // 如果数据量很大，需要更大或改成动态的

  // 给一个 flag 标识这是一次性记录的数据
  doc["flag"] = "recorded_data";

  JsonArray records = doc.createNestedArray("records");
  for (auto &rd : recordedData) {
    JsonObject obj = records.createNestedObject();
    obj["timeStamp"]   = rd.timeStamp;
    obj["heart_rate"]  = rd.heartRate;
    obj["temperature"] = rd.temperature;
    obj["pressure"]    = rd.pressure;
    obj["altitude"]    = rd.altitude;
    obj["step_count"]  = rd.steps;
  }

  String jsonOutput;
  serializeJson(doc, jsonOutput);
  sendDataToFirebase("users/rlrW9nxLkjcFQCWRJJSNI4DXn5x1/recordedData", jsonOutput);
  Serial.println("Recorded data uploaded to Firebase!");
}


//============== upload recordedGPSData to Firebase =============
void uploadRecordedGPSDataToFirebase() {
    if (!gpsAvailable || recordedGPSData.empty()) return;
    
    StaticJsonDocument<4096> doc;
    doc["flag"] = "GPS_recorded_data";
    auto arr = doc.createNestedArray("records");
    for (auto &gd : recordedGPSData) {
      auto o = arr.createNestedObject();
      o["timeStamp"]  = gd.timeStamp;
      o["latitude"]   = gd.latitude;
      o["longitude"]  = gd.longitude;
      o["speed"]      = gd.speed;
      o["angle"]      = gd.angle;
      o["altitude"]   = gd.altitude;
      o["satellites"] = gd.satellites;
    }
    String jsonOutput; 
    serializeJson(doc, jsonOutput);
    sendDataToFirebase("users/rlrW9nxLkjcFQCWRJJSNI4DXn5x1/GPSdata", jsonOutput);
    Serial.println("Recorded GPS data uploaded to Firebase!");
  }





void drawRecordingIndicator(bool isOn) {
  
  int x = 230;
  int y = 10;
  
  int r = 5;

  if (isOn) {
    
    tft.fillCircle(x, y, r, ST77XX_RED);
  } else {
    
    tft.fillCircle(x, y, r, ST77XX_BLACK);
  }
}

  