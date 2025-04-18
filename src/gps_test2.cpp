#include <Wire.h>
#include <Adafruit_GPS.h>

// Use Hardware I2C
Adafruit_GPS GPS(&Wire);

#define GPSECHO false  // Open to see the original NMEA

uint32_t timer;

void setup() {
  // 1) I2C initialize
  Wire.begin();

  // 2) Serial Monitor initialize
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println(F("Adafruit I2C GPS test start"));

  // 3) Address of GPS module I2C as 0x10
  GPS.begin(0x10);

  // 4) Configure the NMEA output and update rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5) antenna status
  GPS.sendCommand(PGCMD_ANTENNA);
  
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);

  // 7) Initialize timer
  timer = millis();
}

void loop() {
  // read bytes and send them to the parser
  char c = GPS.read();           // Read a byte from I2C
  if (GPSECHO && c) Serial.print(c);

  // If a complete sentence is received, try to parse it
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }

  // Print the status every 2 seconds
  if (millis() - timer >= 2000) {
    timer = millis();

    // Time
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

    // Date
    Serial.print(F("Date: "));
    Serial.print(GPS.day,   DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print('/');
    Serial.println(2000 + GPS.year, DEC);

    // Positioning status
    Serial.print(F("Fix: "));
    Serial.print((int)GPS.fix);
    Serial.print(F("  quality: "));
    Serial.println((int)GPS.fixquality);

    // Position, speed, Angle, altitude, number of satellites
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
