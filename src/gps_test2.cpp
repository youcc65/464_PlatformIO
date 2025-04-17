#include <Wire.h>
#include <Adafruit_GPS.h>

// 使用硬件 I2C
Adafruit_GPS GPS(&Wire);

#define GPSECHO false  // 打开可以看到原始 NMEA

uint32_t timer;

void setup() {
  // 1) I2C 初始化
  Wire.begin();

  // 2) 串口初始化
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println(F("Adafruit I2C GPS test start"));

  // 3) GPS 模块 I2C 地址设为 0x10
  GPS.begin(0x10);

  // 4) 配置 NMEA 输出和更新率
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5) 查询天线状态
  GPS.sendCommand(PGCMD_ANTENNA);
  // 6) 等待模块准备并打印固件版本
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);

  // 7) 初始化定时器
  timer = millis();
}

void loop() {
  // —— 第一部分：不断读取字节并送入解析器 —— 
  char c = GPS.read();           // 从 I2C 读取一个字节
  if (GPSECHO && c) Serial.print(c);

  // 如果收到完整句子，尝试解析
  if (GPS.newNMEAreceived()) {
    // 打印原始 NMEA 调试信息（可选）
    // Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA())) {
      // 解析失败就丢掉，下一次再试
      return;
    }
  }

  // —— 第二部分：每隔 2 秒打印一次状态 —— 
  if (millis() - timer >= 2000) {
    timer = millis();

    // 时间
    Serial.print(F("Time: "));
    if (GPS.hour   < 10) Serial.print('0');
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) Serial.print('0');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds< 10) Serial.print('0');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10)       Serial.print("00");
    else if (GPS.milliseconds < 100) Serial.print('0');
    Serial.println(GPS.milliseconds);

    // 日期
    Serial.print(F("Date: "));
    Serial.print(GPS.day,   DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print('/');
    Serial.println(2000 + GPS.year, DEC);

    // 定位状态
    Serial.print(F("Fix: "));
    Serial.print((int)GPS.fix);
    Serial.print(F("  quality: "));
    Serial.println((int)GPS.fixquality);

    // 位置、速度、角度、高度、卫星数
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
  }
}
