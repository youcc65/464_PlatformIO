#include <Arduino.h>
#include <Adafruit_GPS.h>

// ESP32 上我们用 Serial2（TX=17, RX=16）来和 GPS 模块通信
#define GPSSerial Serial2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

Adafruit_GPS GPS(&GPSSerial);

void setup() {
  // 用串口 115200 打开 PC 端监视器
  Serial.begin(115200);

  // 启动 ESP32 的第二串口，与 GPS 硬件连线：TX->RX, RX->TX
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // 初始化 GPS
  GPS.begin(9600);

  // 配置只输出 RMC + GGA 两种 NMEA 语句（可选）
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // 配置更新速率：1Hz 或 10Hz（10Hz 可改为 PMTK_SET_NMEA_UPDATE_10HZ）
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(1000);
  Serial.println("🚀 GPS 初始化完成，开始接收数据...");
}

void loop() {
  // 读取并打印所有原始 NMEA 数据（方便调试）
  while (GPSSerial.available()) {
    Serial.write(GPSSerial.read());
  }

  // 当接收到完整一条 NMEA 句时，尝试解析
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      // 解析失败，直接返回
      return;
    }
    
    // 如果已经有定位 fix，就把关键信息打印出来
    if (GPS.fix) {
      Serial.print("📍 位置: ");
      Serial.print(GPS.latitude, 6); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 6); Serial.println(GPS.lon);

      Serial.print("⏱ 时间 UTC: ");
      Serial.print(GPS.hour); Serial.print(':');
      Serial.print(GPS.minute); Serial.print(':');
      Serial.print(GPS.seconds); Serial.print('.');
      Serial.println(GPS.milliseconds);

      Serial.print("🔭 卫星数: ");
      Serial.println(GPS.satellites);

      Serial.println();
    } else {
      Serial.println("❌ 尚未获得定位 fix");
    }
  }
}
