#include <Arduino.h>
#include <definitions.h>
#include <SPI.h>
#include <RadioLib.h>

// Instantiate the built-in radio (STM32WLE5JC with integrated SX126x)
STM32WLx radio = new STM32WLx_Module();

void setup() {
  Serial.begin(115200);

  // Configure RF switch table (maps internal PA modes to DIO pins)
  // Even though no external switch chip is used, the LoRa-E5 has two PA modes (LP/HP) controlled by DIO2.
  static const uint32_t rfswitch_pins[] = {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
  static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
    {STM32WLx::MODE_RX,    {HIGH, LOW}},
    {STM32WLx::MODE_TX_LP, {HIGH, HIGH}},
    {STM32WLx::MODE_TX_HP, {LOW,  HIGH}},
    END_OF_MODE_TABLE,
  };
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);  // see example table below:contentReference[oaicite:1]{index=1} 

  // Initialize LoRa (868 MHz, 125 kHz, SF7, CR=4/5, sync=0x12, power=15 dBm, preamble=8)
  // Use tcxoVoltage=0.0 since LoRa-E5 uses the internal XTAL (no external TCXO):contentReference[oaicite:2]{index=2}.
  int state = radio.begin(867.0, 125.0, 12, 5, 0x12, 15, 8, 1.7, false);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Init failed, code "); Serial.println(state);
    while (true);
  }
  Serial.println("Radio initialized");
}

void loop() {
  // Transmit a test message
  data d;
  d.datetime = millis();  
  d.locktime = 0;
  d.lat = 0.0;  
  d.lng = 0.0;
  d.hdop = 0;

  int state = radio.transmit((uint8_t*)&d, sizeof(d));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Message sent!");
  } else {
    Serial.print("Transmit failed, code "); Serial.println(state);
  }
  delay(10000);
}
