// #include <Arduino.h>
// #include <HardwareSerial.h>
// #include <definitions.h>

// // Create LPUART instance
// HardwareSerial LPUART(GPS_RX, GPS_TX); // RX, TX pins

// void setup() {
//   // Initialize main serial for debugging
//   Serial.begin(115200);
  
//   // Initialize LPUART
//   LPUART.begin(9600); // Common baud rate, adjust as needed
  
//   // Optional: Set specific LPUART parameters
//   // LPUART.begin(baudrate, config)
//   // Example: LPUART.begin(9600, SERIAL_8N1);
  
//   Serial.println("LPUART initialized on PC0(TX), PC1(RX)");

//   pinMode(GPS_EN, OUTPUT);
//   digitalWrite(GPS_EN, HIGH); // Ensure GPS is disabled initially
// }

// void loop() {
//   // Send data via LPUART
//   if(LPUART.available()){
//     Serial.write(LPUART.read());
//   }
//   if(Serial.available()){
//     LPUART.write(Serial.read());
//   }
// }