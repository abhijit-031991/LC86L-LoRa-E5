// #include <Arduino.h>
// #include <definitions.h>
// /*
//   Example sketch: Test_MPR121_LoRaE5.ino
//   Demonstrates enabling electrodes 0–3 and printing their touch state.
// */

// #include <Wire.h>
// #include "MPR121_LoRaE5.h"

// MPR121_LoRaE5 mpr;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) delay(10);

//   Serial.println("MPR121 Test: Enable electrodes 0-3");

//   if (!mpr.begin()) {
//     Serial.println("MPR121 not found, check wiring!");
//     while (1) delay(10);
//   }

//   // Put device into stop mode before config
//   mpr.stopMode();

//   // Set thresholds (global defaults)
//   mpr.setThresholds(12, 6);

//   mpr.setSampleIntervalMs(128); // Set sample interval to 128ms

//   mpr.setCDC(0x10); // Set CDC to a reasonable default
  
//   // Enable only electrodes 0–3
//   // The library’s runMode() expects a count, so pass 4 to enable 0–3
//   mpr.runMode(4);

//   Serial.println("Electrodes 0–3 enabled.");
// }

// void loop() {
//   uint16_t status = mpr.touched();

//   for (uint8_t i = 0; i < 4; i++) {
//     if (mpr.isTouched(i)) {
//       Serial.print("Electrode ");
//       Serial.print(i);
//       Serial.println(" is touched");
//     }else{
//       Serial.print("Electrode ");
//       Serial.print(i);
//       Serial.println(" is not touched");
//     }
//   }

//   delay(10000);
// }