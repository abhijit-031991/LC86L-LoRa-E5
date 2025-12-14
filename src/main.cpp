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
int startHour = 0; // Start hour for scheduled mode
int endHour = 23; // End hour for scheduled mode

// Flags
bool packetReceived = false;
bool scheduled = false;
bool submerged = false;

// Data Variables
float lat;
float lng;
time_t currentTime;
unsigned int count = 0;

int state = RADIOLIB_ERR_NONE;

///////////////////////////////////////////////
void calculateTargets() {
    gpsCounterTarget   = gpsFrequency  * 60;   // Convert minutes to seconds
    pingCounterTarget = radioFrequency * 60; // Convert minutes to seconds

    char buffer[100]; // make sure it's big enough

    // Format the results into a string
    sprintf(buffer,
            "GPS Target: %ld | Radio Target: %ld",
            gpsCounterTarget, pingCounterTarget);

    // Print the string to Serial
    Serial.println(buffer);
}

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

void getlocation(bool setup,bool &GF){
    Serial.println("Getting GPS location...");
    data dat;
    float startHdop = 10.00;
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
        // Serial.println("Waiting for GPS fix...");
        while (LPUART.available() > 0) {
            if(gps.encode(LPUART.read())){
                if (gps.location.isValid()) {
                    Serial.print("Location Age: ");
                    Serial.println(gps.location.age());
                    Serial.print("Satellites: ");
                    Serial.println(gps.satellites.value());
                    Serial.print("HDOP: ");
                    Serial.println(gps.hdop.hdop());
                    if (gps.hdop.hdop() != 0.00) {
                        startHdop = gps.hdop.hdop();
                    }                    
                    Serial.print("Latitude: ");
                    Serial.println(gps.location.lat(), 6);
                    Serial.print("Longitude: ");
                    Serial.println(gps.location.lng(), 6);
                    
                    // Check if we have a good fix
                    if (startHdop < (double)gpsHdop && 
                        gps.location.age() < 1000 && 
                        gps.time.age() < 1000 && 
                        millis() - start > 3000) {
                        goodFixFound = true;
                        Serial.println("Good GPS fix acquired!");
                        break; // Break out of inner loop
                    }
                }
            }else{
            //   Serial.println("Acquiring... ");
            }
        }
        // Small delay to prevent overwhelming the processor
        delay(10);
    }
    
    digitalWrite(GPS_EN, LOW);
    LPUART.end();

    if (goodFixFound) {
        Serial.println("GPS location acquired successfully");
        GF = true;
        lat = gps.location.lat();
        lng = gps.location.lng();
    } else {
        Serial.println("GPS timeout - no good fix found");
        GF = false;
    } 
    if (setup && goodFixFound)
    {
        setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day (), gps.date.month (),gps.date.year());
        currentTime = now();
        Serial.println(currentTime);
        rtc.setEpoch(currentTime);
        Serial.print("RTC Time Set: ");Serial.println(rtc.getEpoch());
    }    
    
    if(!setup){

        dat.x = 0.00;
        dat.y = 0.00;
        dat.z = 0.00;

        Serial.print(dat.x); Serial.print(", ");
        Serial.print(dat.y); Serial.print(", ");
        Serial.println(dat.z);

        count = count + 1;
        dat.id = tag;
        dat.lat = gps.location.isValid() ? gps.location.lat() : 0;
        dat.lng = gps.location.isValid() ? gps.location.lng() : 0;
        dat.hdop = gps.hdop.hdop();
        dat.locktime = (millis() - start) / 1000; // Lock
        dat.count = count;
        // Set timestamp
        if (gps.time.isValid() && gps.date.isValid()) {
            setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 
                    gps.date.day(), gps.date.month(), gps.date.year());
            dat.datetime = (uint32_t)now();
        } else {
            dat.datetime = rtc.getEpoch(); // Use system time if GPS time invalid
        }
        // === FIXED FLASH WRITE SECTION ===
        Serial.println(F("Writing to flash..."));
        
        flash.powerUp();
        delay(10); // Give flash time to wake up
        
        // Check if we need to erase a new sector
        // Erase at sector boundaries (typically every 4096 bytes)
        uint32_t sectorSize = flash.getSectorSize();
        if (writeAdd % sectorSize == 0) {
            Serial.print(F("Erasing sector at address: 0x"));
            Serial.println(writeAdd, HEX);
            
            uint8_t eraseResult = flash.eraseSector(writeAdd);
            if (eraseResult != FLASH_OK) {
                Serial.print(F("Sector erase FAILED! Error code: "));
                Serial.println(eraseResult);
                flash.powerDown();
                return;
            }
            Serial.println(F("Sector erased successfully"));
        }
        
        // Write the struct
        Serial.print(F("Writing struct at address: 0x"));
        Serial.println(writeAdd, HEX);
        Serial.print(F("Struct size: "));
        Serial.println(sizeof(dat));
        
        uint8_t writeResult = flash.writeStruct(writeAdd, dat, true); // Enable verify
        
        if (writeResult == FLASH_OK) {
            Serial.println(F("Data written to flash successfully."));
            
            // Verify by reading back
            data verification;
            uint8_t readResult = flash.readStruct(writeAdd, verification);
            if (readResult == FLASH_OK) {
                Serial.println(F("Verification read successful"));
                Serial.print(F("Verify lat: ")); Serial.println(verification.lat, 8);
                Serial.print(F("Verify lng: ")); Serial.println(verification.lng, 8);
                Serial.print(F("Verify time: ")); Serial.println(verification.datetime);
                Serial.print(F("Verify count: ")); Serial.println(verification.count);
                
                // Only increment address if write was truly successful
                writeAdd += sizeof(dat);
            } else {
                Serial.print(F("Verification read FAILED! Error: "));
                Serial.println(readResult);
            }
        } else {
            Serial.print(F("Failed to write data to flash! Error code: "));
            Serial.println(writeResult);
            
            // Print error code meanings
            switch(writeResult) {
                case FLASH_ERROR_TIMEOUT:
                    Serial.println(F("  -> TIMEOUT"));
                    break;
                case FLASH_ERROR_WRITE_PROTECTED:
                    Serial.println(F("  -> WRITE PROTECTED"));
                    break;
                case FLASH_ERROR_INVALID_ADDRESS:
                    Serial.println(F("  -> INVALID ADDRESS"));
                    break;
                case FLASH_ERROR_NOT_ERASED:
                    Serial.println(F("  -> NOT ERASED"));
                    break;
                case FLASH_ERROR_VERIFY_FAILED:
                    Serial.println(F("  -> VERIFY FAILED"));
                    break;
                case FLASH_ERROR_NO_RESPONSE:
                    Serial.println(F("  -> NO RESPONSE"));
                    break;
                default:
                    Serial.println(F("  -> UNKNOWN ERROR"));
                    break;
            }
        }
        
        flash.powerDown();
        
        Serial.print("Total records stored: "); 
        Serial.println(writeAdd / sizeof(dat));
        Serial.print("Write Address: 0x"); 
        Serial.println(writeAdd, HEX);
        Serial.flush();
    }
}

// IMPROVED locationReadSend function:
void readSend(unsigned long startAddress) {
    data dat;
    flash.powerUp();
    delay(10);

    uint8_t res = flash.readStruct(startAddress, dat);
    Serial.print(F("Reading from address 0x"));
    Serial.print(startAddress, HEX);
    Serial.print(F(" => "));
    
    if (res != FLASH_OK) {
        Serial.print(F("FAIL (code "));
        Serial.print(res);
        Serial.println(F(")"));
        flash.powerDown();
        return;
    }
    Serial.println(F("OK"));

    // Debug values
    Serial.print(F("datetime: ")); Serial.println(dat.datetime);
    Serial.print(F("lat: ")); Serial.println(dat.lat, 6);
    Serial.print(F("lng: ")); Serial.println(dat.lng, 6);
    Serial.print(F("Count: ")); Serial.println(dat.count);
    Serial.print(F("hdop: ")); Serial.println(dat.hdop, 2);
    Serial.print(F("locktime: ")); Serial.println(dat.locktime);    
    Serial.print(F("id: ")); Serial.println(dat.id);
    Serial.flush();

    flash.powerDown();

    // Wake radio before transmit
    radio.standby();
    state = radio.transmit((uint8_t*)&dat, sizeof(dat));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Data sent successfully.");
    } else {
        Serial.print("Failed to send data. Error code: ");
        Serial.println(state);
    }
    radio.sleep();
}

void ping(byte requestCode, bool simplePing){
  // Wake radio before transmit
  radio.standby();
  
  if (simplePing) {
      reqPing ping;
      ping.tag = tag;
      ping.request = requestCode; // SIMPLE_PING or REQUEST_SETTINGS

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

  if (!simplePing)
  {
    longPing pingLong;
    pingLong.ta = tag;
    pingLong.cnt = writeAdd / sizeof(data); // Number of stored records
    pingLong.la = lat;
    pingLong.ln = lng;
    pingLong.devtyp = devType; // Device type
    pingLong.mortality = submerged; // Mortality flag
    Serial.println("Sending detailed ping...");
    state = radio.transmit((uint8_t*)&pingLong, sizeof(pingLong));
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Detailed ping sent successfully.");
    } else {
        Serial.print("Failed to send detailed ping. Error code: ");
        Serial.println(state);
    }
    radio.sleep();
  }
}

void receive(unsigned long timeout){
    // Wake radio and start receiving
    radio.standby();
    radio.startReceive();

    uint8_t buffer[28];
    size_t length = sizeof(buffer);
    reqPing reqPacket;
    setttings settingsPacket;
    longPing pingLong;
    size_t packetLength;

    unsigned long start = millis();    
    while (millis() - start < timeout)
    {
      state = radio.receive(buffer, length);
      
      if (state == RADIOLIB_ERR_NONE) {
        packetLength = radio.getPacketLength();
        Serial.print(F("[LoRa] Bytes received: "));
        Serial.println(packetLength);
        
        if (packetLength == sizeof(reqPacket))
        {
          memcpy(&reqPacket, buffer, sizeof(reqPacket));
          if (reqPacket.tag == tag && reqPacket.request == SIMPLE_PING_ACK) // Check if ping request is for this device
          {
            radio.standby(); // Ensure radio is ready to transmit
            pingLong.ta = tag;
            pingLong.cnt = writeAdd / sizeof(data); // Number of stored records
            pingLong.la = lat;
            pingLong.ln = lng;
            pingLong.devtyp = devType; // Device type
            pingLong.mortality = submerged; // Use actual submerged state

            state = radio.transmit((uint8_t*)&pingLong, sizeof(pingLong));
            if (state == RADIOLIB_ERR_NONE) {
                Serial.println("Ping response sent successfully.");
            } else {
                Serial.print("Failed to send ping response. Error code: ");
                Serial.println(state);
            }
            radio.startReceive(); // Go back to RX mode
          }

          if (reqPacket.tag == tag && reqPacket.request == DATA_DOWNLOAD_ALL) // Check if ping request is for this device
          {
              radio.standby(); // Ensure radio is ready to transmit
              Serial.println("Data download request received.");
              reqPacket.request = DATA_DOWNLOAD_BEGIN;
              state = radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
              unsigned long tempReadAdd = 0; // Start from the beginning of flash
              while (tempReadAdd < writeAdd) {
                  readSend(tempReadAdd);
                  tempReadAdd = tempReadAdd + sizeof(data);
                  delay(100); // Small delay to ensure the receiver can keep up
              }
              reqPacket.request = DATA_DOWNLOAD_END;
              state = radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
              Serial.println("Data download completed.");
              break; // Exit after download
          }

          if (reqPacket.tag == tag && reqPacket.request == DATA_DOWNLOAD_NEW) // Check if ping request is for this device
          {
              radio.standby(); // Ensure radio is ready to transmit
              Serial.println("New data download request received.");
              reqPacket.request = DATA_DOWNLOAD_BEGIN;
              state = radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
              unsigned long tempReadAdd = readAdd; // Start from the last read address
              while (tempReadAdd < writeAdd) {
                  readSend(tempReadAdd);
                  tempReadAdd += sizeof(data);
                  delay(100); // Small delay to ensure the receiver can keep up
              }
              readAdd = writeAdd; // Update last read address to current write address
              reqPacket.request = DATA_DOWNLOAD_END;
              state = radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
              Serial.println("New data download completed.");
              break; // Exit after download
          }      
        }
        if (packetLength == sizeof(settingsPacket))  // Check if setting is incoming and assimilate
        {
          memcpy(&settingsPacket, buffer, sizeof(settingsPacket));
          if (settingsPacket.tag == tag)
          {
            gpsFrequency = settingsPacket.gpsFrq;
            gpsTimeout = settingsPacket.gpsTout;
            gpsHdop = settingsPacket.hdop;
            radioFrequency = settingsPacket.radioFrq;
            scheduled = settingsPacket.scheduled;
            startHour = settingsPacket.startHour; // Update start hour
            endHour = settingsPacket.endHour;     // Update end hour
            
            calculateTargets(); // Recalculate targets with new settings

            radio.standby(); // Ensure radio is ready to transmit
            reqPacket.tag = tag;
            reqPacket.request = SETTINGS_UPDATED; // Acknowledge settings update
            state = radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
            if (state == RADIOLIB_ERR_NONE) {
                Serial.println("Settings update acknowledged.");
            } else {
                Serial.print("Failed to acknowledge settings update. Error code: ");
                Serial.println(state);
            }
            radio.startReceive(); // Go back to RX mode
          }      
        }
        
      } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        // Normal timeout, continue waiting
      } else {
        Serial.print(F("[LoRa] Receive failed, code "));
        Serial.println(state);
      }
      
      delay(10); // Small delay to prevent tight loop
    }

    radio.sleep(); // Put radio to sleep to save power
}

void radioCodeChecker(int st){
  if (st == RADIOLIB_ERR_NONE) {
      Serial.println("Radio operation successful.");
  } else {
      Serial.print("Radio operation failed. Error code: ");
      Serial.println(state);
  }
}

void deviceCalibration(bool mprSt){
    bool fixStatus = false;
    Serial.println("Starting device calibration...");
    
    // Wake radio before ping
    radio.standby();
    ping(CALIBRATION_BEGIN, true);    // Indicate calibration start
    
    flash.powerUp();
    delay(10); // Ensure flash is ready
    if(flash.getJEDECID() != 0){
      Serial.println("Flash memory detected successfully.");
      FlashChipInfo chipInfo = flash.getChipInfo();
      Serial.print("Manufacturer: "); Serial.println(chipInfo.manufacturer);
      Serial.print("Model: "); Serial.println(chipInfo.model);
      Serial.print("Capacity: "); Serial.print(chipInfo.capacity / 1024); Serial.println(" KB");
      Serial.print("Page Size: "); Serial.print(chipInfo.pageSize); Serial.println(" bytes");
      Serial.print("Sector Size: "); Serial.print(chipInfo.sectorSize); Serial.println(" bytes");
      Serial.print("JEDEC ID: 0x"); Serial.println(chipInfo.jedecId, HEX);
      ping(FLASH_SUCCESS, true); // Indicate flash success
    } else {
      Serial.println("Failed to detect flash memory.");
      ping(FLASH_ERROR, true); // Indicate flash error
    }
    flash.powerDown();

    if (mprSt){
      Serial.println("MPR121 detected successfully.");
      ping(CAPACITANCE_OK, true); // Indicate capacitance sensor success
    } else {
      Serial.println("Failed to detect MPR121.");
      ping(CAPACITANCE_ERROR, true); // Indicate capacitance sensor error
    }
    
    // getlocation(true, fixStatus); // Get initial GPS location with extended timeout
    if (fixStatus){
      ping(GPS_SUCCESS, true); // Indicate GPS success
    } else {
      ping(GPS_ERROR, true); // Indicate GPS error
    }
    ping(CALIBRATION_END, true); // Indicate calibration end
    
    Serial.println("Device calibration completed.");

    // Wait for settings packet
    uint8_t buffer[32];
    size_t length = sizeof(buffer);
    setttings setPacket;
    reqPing reqPacket;
    int packetLength;

    Serial.println("Waiting for input");
    
    // Start receiving mode
    radio.standby();
    radio.startReceive();

    unsigned long start = millis();    
    while (millis() - start < 300000)
    {
        state = radio.receive(buffer, length);
        if (state == RADIOLIB_ERR_NONE) {
            packetLength = radio.getPacketLength();
            Serial.print(F("[LoRa] Bytes received: "));
            Serial.println(packetLength);

            if (packetLength == sizeof(setPacket))
            {
                memcpy(&setPacket, buffer, sizeof(setPacket));
                if (setPacket.tag == tag)
                {
                    gpsFrequency = setPacket.gpsFrq;
                    gpsTimeout = setPacket.gpsTout;
                    gpsHdop = setPacket.hdop;
                    radioFrequency = setPacket.radioFrq;
                    scheduled = setPacket.scheduled;
                    startHour = setPacket.startHour;
                    endHour = setPacket.endHour;
                    
                    // Use GPS time if available
                    if (gps.time.isValid() && gps.date.isValid()) {
                        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 
                                gps.date.day(), gps.date.month(), gps.date.year());
                        rtc.setEpoch((uint32_t)now());
                    }
                    
                    calculateTargets();
                    
                    radio.standby(); // Ensure radio is ready to transmit
                    ping(SETTINGS_UPDATED, true); // Acknowledge settings update
                    radio.startReceive(); // Continue receiving
                    delay(100);
                }
            } 
            if (packetLength == sizeof(reqPacket))
            {
                memcpy(&reqPacket, buffer, sizeof(reqPacket));
                Serial.println(reqPacket.tag);
                Serial.println(reqPacket.request);
                if (reqPacket.tag == tag && reqPacket.request == FINISH_CALIBRATION)
                {
                    break;
                }
            }
        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // Normal timeout, continue
        } else {
            Serial.print(F("[LoRa] Receive failed, code "));
            Serial.println(state);
        }
        
        delay(10); // Prevent tight loop
    }
    
    radio.sleep(); // Put radio to sleep after calibration
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup...");
    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW); // Ensure GPS starts off
    
    while (!Serial) {
        delay(10); // Wait for serial port to connect
        Serial.println("Waiting for Serial connection...");
    }
    Serial.flush();
    
    delay(2000); // Reduced from 10000 - no need for 10 seconds            
    
    Serial.println("Setup started.");

    // Initialize RTC
    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin(STM32RTC::HOUR_24); // Initialize RTC in 24-hour format

    // Initialize radio with retry logic
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    int retries = 3;
    while (retries > 0) {
        state = radio.begin(867.0, 125.0, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 1.6, false);
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("Radio initialized successfully.");
            break;
        }
        Serial.print("Radio init failed: ");
        Serial.println(state);
        retries--;
        delay(1000);
    }
    
    if (retries == 0) {
        Serial.println("Radio initialization failed after retries!");
        while (true); // Halt - radio is critical
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

    bool mprStatus = mpr.begin();

    if (!mprStatus) {
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
    // The library's runMode() expects a count, so pass 4 to enable 0–3
    mpr.runMode(4);
    Serial.println("Electrodes 0–3 enabled.");

    deviceCalibration(mprStatus); // Calibrate device and report status

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
        bool fix = false;
        Serial.begin(115200);
        Serial.println("Woke up from deep sleep.");
        getlocation(false, fix);    // Get GPS location with standard timeout
        gpsCounter = 0;
        Serial.flush();
        Serial.end();
    }
    
    if (pingCounter >= pingCounterTarget && !submerged)
    {
        Serial.begin(115200);
        Serial.println("Woke up from deep sleep.");
        SPI.begin(); // Reinitialize SPI
        
        // Wake up and reconfigure radio
        radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
        radio.standby();
        
        ping(SIMPLE_PING, true);  // Simple ping
        receive(5000); // Listen for 5 seconds for any incoming messages
        
        pingCounter = 0;
        Serial.flush();
        Serial.end();
        SPI.end(); // End SPI to save power
    }
    
    LowPower.deepSleep();        
}