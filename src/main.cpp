#include <Arduino.h>
#include <codes.h>
#include <definitions.h>
#include <TinyGPS++.h>
#include <LSM6DSL.h>
#include <MPR121_LoRaE5.h>
#include <STM32Stop1.h>
#include <STM32RTC.h>
#include <RadioLib.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <LoRaE5_SPIFlash.h>
#include <STM32LowPower.h>
#include <TimeLib.h>
#include <time.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
LSM6DSL imu;
MPR121_LoRaE5 mpr;
STM32Stop1 stop1;
STM32RTC& rtc = STM32RTC::getInstance();
STM32WLx radio = new STM32WLx_Module();
LoRaE5_SPIFlash flash(FSS_PIN, &SPI, 8000000); // SPI pins for LoRa E5 flash
HardwareSerial LPUART(GPS_RX, GPS_TX); // RX, TX for GPS

static const uint32_t rfswitch_pins[] = { PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
static const Module::RfSwitchMode_t rfswitch_table[] = {
  { STM32WLx::MODE_IDLE,  {LOW, LOW} },
  { STM32WLx::MODE_RX,    {HIGH, LOW} },
  { STM32WLx::MODE_TX_HP, {LOW, HIGH} },
  // { STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH} },
  END_OF_MODE_TABLE,
};

///// Variables //////

// GPS Variables
unsigned long gpsCounter = 0;
unsigned long gpsCounterTarget;

// Ping Variables
unsigned long pingCounter = 0;
unsigned long pingCounterTarget;

// Capacitive Touch Variables
unsigned long capCounter = 0;
unsigned long capCounterTarget = 10; // Check capacitive touch every 10 seconds

//Flash Addresses
unsigned long writeAdd = 0; // Last written to flash address
unsigned long readAdd = 0; // Last read from flash address

// Setting Variables
int gpsFrequency = 3;   // in minutes  
int gpsTimeout = 120; // in seconds
int gpsHdop = 5; // HDOP value
int radioFrequency = 1;  // in minutes
int capFrequency = 30; // in seconds

// Flags
bool packetReceived = false;
bool scheduled = false;
bool submerged = false;

// Data Variables
float lat;
float lng;

int state = RADIOLIB_ERR_NONE;

///////////////////////////////////////////////

void checkElectrodes() {
    bool e1 = mpr.isTouched(0);
    bool e2 = mpr.isTouched(1);
    bool e3 = mpr.isTouched(2);
    bool e4 = mpr.isTouched(3);

    // Print status for each electrode
    for (uint8_t i = 0; i < 4; i++) {
        if (mpr.isTouched(i)) {
            Serial.print("Electrode "); Serial.print(i);
            Serial.println(" is touched");
        } else {
            Serial.print("Electrode "); Serial.print(i);
            Serial.println(" is not touched");
        }
    }

    // Decide if submerged
    if (e1 && e2 && e3 && e4) {
        Serial.println("Device is SUBMERGED!");
        submerged = true;
    } else {
        Serial.println("Device is NOT submerged.");
        submerged = false;
    }
}

void isr(void* data) {
    // This ISR will be called when a touch event occurs
    gpsCounter = gpsCounter + 1;
    pingCounter = pingCounter + 1;
    capCounter = capCounter + 1;
}

void getlocation(bool setup){
    Serial.println("Getting GPS location...");
    data dat;
    unsigned long timeout;
    if (setup){
        timeout = 30*60*1000; // 30 minutes in milliseconds
    }else{
        timeout = gpsTimeout * 1000; // Convert seconds to milliseconds
    }

    Serial.println(timeout);
    LPUART.begin(9600);
    digitalWrite(GPS_EN, HIGH);
    unsigned long start = millis();
    bool goodFixFound = false;
    
    while (millis() - start < timeout && !goodFixFound) {
        while (LPUART.available() > 0) {
            if(gps.encode(LPUART.read())){
                if (gps.location.isValid()) {
                    Serial.print("Location Age: ");
                    Serial.println(gps.location.age());
                    Serial.print("Satellites: ");
                    Serial.println(gps.satellites.value());
                    Serial.print("HDOP: ");
                    Serial.println(gps.hdop.hdop());
                    
                    // Check if we have a good fix
                    if (gps.hdop.hdop() < (double)gpsHdop && 
                        gps.location.age() < 1000 && 
                        gps.time.age() < 1000 && 
                        millis() - start > 3000) {
                        goodFixFound = true;
                        Serial.println("Good GPS fix acquired!");
                        break; // Break out of inner loop
                    }
                }
            }else{
              // Serial.println("Acquiring... ");
            }
        }
        // Small delay to prevent overwhelming the processor
        delay(10);
    }
    
    digitalWrite(GPS_EN, LOW);
    LPUART.end();

    if (goodFixFound) {
        Serial.println("GPS location acquired successfully");
    } else {
        Serial.println("GPS timeout - no good fix found");
    }
    
    if(!setup){
      dat.id = tag;
      dat.lat = gps.location.isValid() ? gps.location.lat() : 0;
      dat.lng = gps.location.isValid() ? gps.location.lng() : 0;
      dat.hdop = gps.hdop.hdop();
      dat.locktime = (millis() - start) / 1000; // Lock
      // Set timestamp
      if (gps.time.isValid() && gps.date.isValid()) {
          setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 
                gps.date.day(), gps.date.month(), gps.date.year());
          dat.datetime = (uint32_t)now();
      } else {
          dat.datetime = rtc.getEpoch(); // Use system time if GPS time invalid
      }
    }
    flash.powerUp();
    if (flash.writeStruct(writeAdd, dat)) {
      Serial.println("Data written to flash successfully.");
      writeAdd += sizeof(dat); // Move to the next address
    } else {
      Serial.println("Failed to write data to flash.");
    }
    flash.powerDown();
    Serial.print("Stored GPS data:");
    writeAdd = writeAdd + sizeof(dat);
    Serial.print("Total records stored: "); Serial.println(writeAdd / sizeof(dat));
    Serial.print("Write Address: "); Serial.println(writeAdd);
    Serial.flush();   
}

void gpsCalibration(){
  reqPing reqPacket;
  reqPacket.tag = tag;
  reqPacket.request = INIT_BEGIN;
  state = radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
  delay(3000);
  // getlocation(true); // Get initial GPS location with extended timeout
  reqPacket.request = INIT_END;
  state = radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
  delay(3000);
  radio.sleep(); // Put radio to sleep to save power
}

void readSend(unsigned long address){
    data dat;
    flash.powerUp();
    if (flash.readStruct(address, dat)) {
        Serial.println("Data read from flash successfully.");
        Serial.println("Stored GPS data:");
        Serial.print("Datetime: "); Serial.println(dat.datetime);
        Serial.print("Lat: "); Serial.println(dat.lat, 6);
        Serial.print("Lng: "); Serial.println(dat.lng, 6);
        Serial.print("Lock time: "); Serial.println(dat.locktime);
        Serial.print("HDOP: "); Serial.println(dat.hdop);
        
        // Send data via LoRa
        state = radio.transmit((uint8_t*)&dat, sizeof(dat));
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("Data sent via LoRa successfully.");
        } else {
            Serial.print("Failed to send data via LoRa. Error code: ");
            Serial.println(state);
        }
    }

    flash.powerDown();

}

void ping(){
    reqPing ping;
    ping.tag = tag;
    ping.request = SIMPLE_PING; // Ping request

    Serial.println("Sending ping...");

    state = radio.transmit((uint8_t*)&ping, sizeof(ping));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Ping sent successfully.");
    } else {
        Serial.print("Failed to send ping. Error code: ");
        Serial.println(state);
    }

    radio.sleep(); // Put radio to sleep to save power
}

void receive(unsigned long timeout){

    uint8_t buffer[28];
    size_t length = sizeof(buffer);
    reqPing reqPacket;
    setttings settingsPacket;
    longPing pingLong;

    unsigned long start = millis();    
    while (millis() - start < timeout)
    {
      state = radio.receive(buffer, length);
      if (state == RADIOLIB_ERR_NONE) {
        Serial.print(F("[LoRa] Bytes received: "));
        Serial.println(length);
      } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        Serial.println(F("[LoRa] Receive timeout!"));
      } else {
        Serial.print(F("[LoRa] Receive failed, code "));
        Serial.println(state);
      }
    }
    
    if (length == sizeof(reqPacket))
    {
      memcpy(&reqPacket, buffer, sizeof(reqPacket));
      if (reqPacket.tag == tag && reqPacket.request == SIMPLE_PING_ACK) // Check if ping request is for this device
      {
        pingLong.ta = tag;
        pingLong.cnt = writeAdd / sizeof(data); // Number of stored records
        pingLong.la = lat;
        pingLong.ln = lng;
        pingLong.devtyp = devType; // Device type
        pingLong.mortality = false; // Mortality flag

        state = radio.transmit((uint8_t*)&pingLong, sizeof(pingLong));
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("Ping response sent successfully.");
        } else {
            Serial.print("Failed to send ping response. Error code: ");
            Serial.println(state);
        } 
      }
      
    }
    if (length == sizeof(settingsPacket))  // Check if setting is incoming and assimilate
    {
      memcpy(&settingsPacket, buffer, sizeof(settingsPacket));
      if (settingsPacket.tag == tag)
      {
        gpsFrequency = settingsPacket.gpsFrq;
        gpsTimeout = settingsPacket.gpsTout;
        gpsHdop = settingsPacket.hdop;
        radioFrequency = settingsPacket.radioFrq;
        scheduled = settingsPacket.scheduled;

        reqPacket.tag = tag;
        reqPacket.request = SETTINGS_UPDATED; // Acknowledge settings update
        state = radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("Settings update acknowledged.");
        } else {
            Serial.print("Failed to acknowledge settings update. Error code: ");
            Serial.println(state);
        }
      }
      
    }

    radio.sleep(); // Put radio to sleep to save power
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup...");
    pinMode(GPS_EN, OUTPUT);
    while (!Serial) {
        delay(10); // Wait for serial port to connect
        Serial.println("Waiting for Serial connection...");
    }
    Serial.flush();
    // pinMode(CS_INT_PIN, INPUT_PULLUP); // Set interrupt pin for MPR121
    delay(10000);             
    
    Serial.println("Setup started.");

    // // Initialize RTC
    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin(STM32RTC::HOUR_24); // Initialize RTC in 24-hour format

    // Initialize radio
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    state = radio.begin(867.0, 125.0, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 1.6, false);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.print("Radio init failed: ");
      Serial.println(state);
      while (true);
    }else {
      Serial.println("Radio initialized successfully.");
    }

    state = radio.transmit("Hello, world!");
    if (state != RADIOLIB_ERR_NONE) {
      Serial.print("Radio transmit failed: ");
      Serial.println(state);
    } else {
      Serial.println("Radio transmit successful.");
    }

    radio.sleep(); // Put radio to sleep to save power

    if (flash.begin()) {
      Serial.println("SPI Flash initialized successfully.");
      flash.printChipInfo();
    } else {
      Serial.println("Failed to initialize SPI Flash.");
    }

    flash.powerDown(); // Power down flash to save power
    delay(50);

    if (!mpr.begin()) {
        Serial.println("MPR121 not found, check wiring!");
        while (1) delay(10);
    }

    // Put device into stop mode before config
    mpr.stopMode();
    // Set thresholds (global defaults)
    mpr.setThresholds(12, 6);
    mpr.setSampleIntervalMs(128); // Set sample interval to 128ms
    mpr.setCDC(0x10); // Set CDC to a reasonable default
    // Enable only electrodes 0–3
    // The library’s runMode() expects a count, so pass 4 to enable 0–3
    mpr.runMode(4);
    Serial.println("Electrodes 0–3 enabled.");

    gpsCalibration(); // Get initial GPS location with extended timeout

    ping(); // Send initial ping to indicate device is online

    gpsCounterTarget = gpsFrequency * 60; // Convert minutes to seconds
    pingCounterTarget = radioFrequency * 60; // Convert minutes to seconds
    capCounterTarget = capFrequency; // In seconds

    pinMode(CS_INT_PIN, INPUT_ANALOG); // Set interrupt pin for MPR121
    pinMode(SDA_PIN, INPUT_ANALOG); // Set SDA pin for I2C
    pinMode(SCL_PIN, INPUT_ANALOG); // Set SCL pin for I2C
    pinMode(GPS_RX, INPUT_ANALOG); // Set GPS RX pin
    pinMode(GPS_TX, INPUT_ANALOG); // Set GPS TX pin

    Wire.end(); // End I2C to save power
    SPI.end(); // End SPI to save power
    Serial.println(digitalRead(CS_INT_PIN) ? "CS_INT_PIN is HIGH" : "CS_INT_PIN is LOW");
    Serial.println("Setup complete.");
    Serial.flush();
    Serial.end(); // End Serial to save power
    rtc.attachSecondsInterrupt(isr);
    LowPower.deepSleep();
}

void loop() {

    if (capCounter >= capCounterTarget)
    {
        Serial.begin(115200);
        Wire.begin(); // Reinitialize I2C
        Serial.println("Woke up from deep sleep.");
        checkElectrodes();        
        capCounter = 0;
        Serial.flush();
        Serial.end();
        Wire.end(); // End I2C to save power
    }    

    if (gpsCounter >= gpsCounterTarget && !submerged)
    {
        Serial.begin(115200);
        Serial.println("Woke up from deep sleep.");
        getlocation(false);
        gpsCounter = 0;
        Serial.flush();
        Serial.end();
    }
    if (pingCounter >= pingCounterTarget && !submerged)
    {
        Serial.begin(115200);
        Serial.println("Woke up from deep sleep.");
        SPI.begin(); // Reinitialize SPI
        ping();
        receive(5000); // Listen for 5 seconds for any incoming messages
        pingCounter = 0;
        Serial.flush();
        Serial.end();
        SPI.end(); // End SPI to save power
    }
    
    LowPower.deepSleep();        
}