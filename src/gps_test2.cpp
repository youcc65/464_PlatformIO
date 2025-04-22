#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_GPS.h>

// GPS 串口映射到 UART2
#define GPS_RX_PIN 16  // GPS TXD → ESP32 RX2
#define GPS_TX_PIN 17  // GPS RXD ← ESP32 TX2

HardwareSerial GPSSerial(1);
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false  // 打开为 true 时会把原始 NMEA 串口直接打印到 Serial

uint32_t timer;

void setup() {
  // 1) 初始化调试串口
  Serial.begin(115200);
  while (!Serial) {}  // 等待串口就绪
  Serial.println(F("Adafruit UART GPS test start"));

  // 2) 初始化 GPS 所在的硬件串口
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // 3) 告诉库去设置 GPS 芯片波特率（内部会调用 GPSSerial.begin）
  GPS.begin(9600);

  // 4) 配置 NMEA 输出类型和更新频率
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // 输出 RMC + GGA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);      // 1Hz 更新
  GPS.sendCommand(PGCMD_ANTENNA);                 // 天线状态查询

  delay(1000);
  // 5) 请求固件版本信息（可选）
  GPSSerial.println(PMTK_Q_RELEASE);

  // 6) 设定打印定时器
  timer = millis();
}

void loop() {
  // —— 持续读取 GPS 串口 ——  
  while (GPSSerial.available()) {
    char c = GPS.read();
    if (GPSECHO && c) Serial.write(c);
  }

  // —— 如果收到了完整 NMEA，则解析 ——  
  if (GPS.newNMEAreceived()) {
    // newNMEAreceived() 会被 reset 为 false
    if (!GPS.parse(GPS.lastNMEA())) {
      // 解析失败，直接忽略
    }
  }

  // —— 每两秒打印一次当前信息 ——  
  if (millis() - timer >= 2000) {
    timer = millis();

    Serial.print(F("Time: "));
    if (GPS.hour   < 10) Serial.print('0');
    Serial.print(GPS.hour); Serial.print(':');
    if (GPS.minute < 10) Serial.print('0');
    Serial.print(GPS.minute); Serial.print(':');
    if (GPS.seconds< 10) Serial.print('0');
    Serial.print(GPS.seconds); Serial.print('.');
    if (GPS.milliseconds < 10)       Serial.print("00");
    else if (GPS.milliseconds < 100) Serial.print('0');
    Serial.println(GPS.milliseconds);

    Serial.print(F("Date: "));
    Serial.print(GPS.day);   Serial.print('/');
    Serial.print(GPS.month); Serial.print("/20");
    Serial.println(GPS.year);

    Serial.print(F("Fix: "));
    Serial.print((int)GPS.fix);
    Serial.print(F("  Quality: "));
    Serial.println((int)GPS.fixquality);

    if (GPS.fix) {
      Serial.print(F("Location: "));
      Serial.print(GPS.latitude, 4);  Serial.print(GPS.lat);
      Serial.print(F(", "));
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print(F("Speed (knots): "));
      Serial.println(GPS.speed, 2);

      Serial.print(F("Angle: "));
      Serial.println(GPS.angle, 2);

      Serial.print(F("Altitude: "));
      Serial.println(GPS.altitude, 2);

      Serial.print(F("Satellites: "));
      Serial.println((int)GPS.satellites);

      Serial.print(F("Antenna status: "));
      Serial.println((int)GPS.antenna);
    }
  }

  // —— 你其他的逻辑（按键、传感器、Firebase 等）也可以紧接在这里，无需再 delay ——  
}


