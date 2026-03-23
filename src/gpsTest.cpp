/**
 * gpsTest.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Raw GPS pass-through test for the LC86L-LoRa-E5.
 *
 * Bridges the LC86L GPS module (LPUART) to the USB Serial port so you can
 * watch raw NMEA sentences in any terminal (PlatformIO monitor, PuTTY, etc.)
 * and send PMTK commands back to the module.
 *
 * BUILD & FLASH:
 *   pio run -e lora_e5_gps_test -t upload
 *   pio device monitor -b 115200
 *
 * EXPECTED OUTPUT (once the module has a sky view):
 *   $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,...
 *   $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,...
 *   ...
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <definitions.h>

HardwareSerial LPUART(GPS_RX, GPS_TX); // LPUART mapped to GPS RX/TX pins

void setup() {
    // USB Serial — debug output
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(500);

    // GPS enable pin
    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW);
    delay(100);

    // Power up the GPS module
    Serial.println(F("GPS Test — powering up LC86L..."));
    digitalWrite(GPS_EN, HIGH);
    delay(500); // Let module settle

    // Open LPUART to the GPS module
    LPUART.begin(9600);

    Serial.println(F("Bridge active: GPS NMEA -> USB Serial"));
    Serial.println(F("Type PMTK commands here to send to the GPS module."));
    Serial.println(F("──────────────────────────────────────────────────"));
}

void loop() {
    // Forward every NMEA byte from GPS to USB Serial
    while (LPUART.available()) {
        Serial.write(LPUART.read());
    }

    // Forward any command typed in the terminal back to the GPS module
    while (Serial.available()) {
        LPUART.write(Serial.read());
    }
}
