// #include <Arduino.h>
// #include <HardwareSerial.h>
// #include <definitions.h>

// // Create LPUART instance
// HardwareSerial LPUART(GPS_RX, GPS_TX); // RX, TX pins

// void setup() {
//   // Initialize main serial for debugging

//    // Ensure GPS is disabled initially

//   Serial.begin(115200);
//   Serial.println("Starting LPUART on PC0(TX), PC1(RX)");

//   delay(100); // Small delay to ensure Serial is ready
//   Serial.println("LPUART initialized on PC0(TX), PC1(RX)");
//   delay(100);
//   Serial.println("Enabling GPS Pin");
//   delay(100);
//   pinMode(GPS_EN, OUTPUT);
//   Serial.println("Switchibng GPS Pin");
//   delay(100);
//   digitalWrite(GPS_EN, HIGH);
//   // Initialize LPUART
//   LPUART.begin(9600); // Common baud rate, adjust as needed
//   // Optional: Set specific LPUART parameters
//   // LPUART.begin(baudrate, config)
//   // Example: LPUART.begin(9600, SERIAL_8N1);
  
  
//    // Wait for a second before starting communication

  

//   Serial.println("GPS PIN EN");
//   delay(1000); //
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