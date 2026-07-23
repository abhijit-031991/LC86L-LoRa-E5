/*
 * testing.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Peripheral self-test for the LC86L-LoRa-E5 board.
 * Covers all hardware present on both the terrestrial and aquatic variants.
 *
 * Flash:  pio run -e testing -t upload
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * Test sequence
 * ─────────────────────────────────────────────────────────────────────────────
 *  1. Radio       — radio.begin() + transmit one test ping
 *  2. I2C scan    — scan 0x01–0x7E, list all responding devices
 *  3. LSM6DSL     — WHO_AM_I check + 5 accelerometer samples
 *  4. MPR121      — begin() + electrode touch mask (aquatic sensor)
 *  5. RTC         — set a known epoch, wait 2 s, verify it incremented
 *  6. SPI Flash   — begin() + JEDEC ID + byte write/read/verify
 *  7. GPS (NMEA)  — enable GPS_EN, stream raw NMEA to Serial for 20 s
 *
 * Each test prints PASS or FAIL with a short reason.
 * A final summary line lists the overall pass/fail count.
 * The MCU halts in loop() after setup() completes — no sleep, no repeat.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <RadioLib.h>
#include <STM32RTC.h>
#include <LSM6DSL.h>
#include <LoRaE5_SPIFlash.h>
#include <MPR121_LoRaE5.h>
#include <codes.h>
#include <definitions.h>

// ─────────────────────────────────────────────────────────────────────────────
// RF switch table
// ─────────────────────────────────────────────────────────────────────────────

static const uint32_t rfswitch_pins[] = {
    PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    { STM32WLx::MODE_IDLE,  { LOW,  LOW  } },
    { STM32WLx::MODE_RX,    { HIGH, LOW  } },
    { STM32WLx::MODE_TX_HP, { LOW,  HIGH } },
    END_OF_MODE_TABLE,
};

// ─────────────────────────────────────────────────────────────────────────────
// Object instances  (same declarations as production firmware)
// ─────────────────────────────────────────────────────────────────────────────

STM32WLx          radio = new STM32WLx_Module();
STM32RTC&         rtc   = STM32RTC::getInstance();
LSM6DSL           imu;                               // I2C, address 0x6B
MPR121_LoRaE5     mpr;                               // I2C, address 0x5A
LoRaE5_SPIFlash   flash(FSS_PIN, &SPI, 8000000);     // SPI2, CS = PB9
HardwareSerial    LPUART(GPS_RX, GPS_TX);            // LPUART1 on PC0/PC1
TinyGPSPlus       gps;                               // parser (unused here; included for completeness)

// ─────────────────────────────────────────────────────────────────────────────
// Test bookkeeping
// ─────────────────────────────────────────────────────────────────────────────

static uint8_t passCount = 0;
static uint8_t failCount = 0;

static void pass(const __FlashStringHelper* label) {
    Serial.print(F("  [PASS] ")); Serial.println(label);
    passCount++;
}

static void fail(const __FlashStringHelper* label, const __FlashStringHelper* reason) {
    Serial.print(F("  [FAIL] ")); Serial.print(label);
    Serial.print(F(" — ")); Serial.println(reason);
    failCount++;
}

static void section(const __FlashStringHelper* title) {
    Serial.println();
    Serial.print(F("━━━ ")); Serial.print(title); Serial.println(F(" ━━━"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Radio test
// ─────────────────────────────────────────────────────────────────────────────

static void testRadio() {
    section(F("Radio (STM32WLx sub-GHz)"));

    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    int state = radio.begin(867.0, 125.0, 12, 5,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                            22, 8, 1.6, false);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("  radio.begin() code: ")); Serial.println(state);
        fail(F("radio.begin()"), F("non-zero return code"));
        return;
    }
    pass(F("radio.begin()"));

    reqPing pkt;
    pkt.tag     = tag;
    pkt.request = CALIBRATION_BEGIN;
    radio.standby();
    state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
    radio.sleep();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("  transmit() code: ")); Serial.println(state);
        fail(F("radio.transmit()"), F("non-zero return code"));
    } else {
        pass(F("radio.transmit() test ping"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. I2C scan
// ─────────────────────────────────────────────────────────────────────────────

static void testI2CScan() {
    section(F("I2C bus scan (PA15=SDA, PB15=SCL)"));

    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.begin();

    uint8_t found = 0;
    for (uint8_t addr = 0x01; addr <= 0x7E; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  Device at 0x"));
            if (addr < 0x10) Serial.print('0');
            Serial.print(addr, HEX);
            // Label known addresses
            if (addr == 0x6B || addr == 0x6A) Serial.print(F("  ← LSM6DSL"));
            if (addr == 0x5A)                 Serial.print(F("  ← MPR121"));
            Serial.println();
            found++;
        }
    }

    if (found == 0) {
        fail(F("I2C scan"), F("no devices found"));
    } else {
        Serial.print(F("  Total devices found: ")); Serial.println(found);
        pass(F("I2C scan"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. LSM6DSL (IMU) test
// ─────────────────────────────────────────────────────────────────────────────

static void testLSM6DSL() {
    section(F("LSM6DSL accelerometer / gyroscope"));

    if (!imu.begin()) {
        fail(F("imu.begin()"), F("device not found on I2C"));
        return;
    }

    uint8_t whoami = imu.whoAmI();
    Serial.print(F("  WHO_AM_I = 0x")); Serial.println(whoami, HEX);
    if (whoami != 0x6A && whoami != 0x6B) {
        fail(F("WHO_AM_I"), F("unexpected value (expected 0x6A or 0x6B)"));
        return;
    }
    pass(F("WHO_AM_I"));

    imu.setAccelConfig(LSM6DSL_XL_ODR_104Hz, LSM6DSL_XL_FS_2g);
    imu.enableGyroscope(LSM6DSL_G_ODR_104Hz);
    delay(20);  // let ODR settle

    Serial.println(F("  Sample   Ax(g)    Ay(g)    Az(g)    Gx(dps)  Gy(dps)  Gz(dps)"));
    for (uint8_t i = 0; i < 5; i++) {
        float ax, ay, az, gx, gy, gz;
        imu.readAccelerometer_g(ax, ay, az, LSM6DSL_XL_FS_2g);
        imu.readGyroscope_dps(gx, gy, gz, LSM6DSL_G_FS_245dps);

        Serial.print(F("  ")); Serial.print(i + 1);
        Serial.print(F("        "));
        Serial.print(ax, 3); Serial.print(F("  "));
        Serial.print(ay, 3); Serial.print(F("  "));
        Serial.print(az, 3); Serial.print(F("  "));
        Serial.print(gx, 2); Serial.print(F("  "));
        Serial.print(gy, 2); Serial.print(F("  "));
        Serial.println(gz, 2);
        delay(50);
    }

    // Sanity: magnitude should be close to 1 g when stationary
    float ax, ay, az;
    imu.readAccelerometer_g(ax, ay, az, LSM6DSL_XL_FS_2g);
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    Serial.print(F("  Magnitude: ")); Serial.print(mag, 3); Serial.println(F(" g (expect ~1.0)"));

    if (mag < 0.7f || mag > 1.3f) {
        fail(F("accel magnitude"), F("outside 0.7–1.3 g range — check orientation or hardware"));
    } else {
        pass(F("LSM6DSL accel + gyro"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. MPR121 (capacitive electrodes) test
// ─────────────────────────────────────────────────────────────────────────────

static void testMPR121() {
    section(F("MPR121 capacitive touch (aquatic electrodes)"));

    // Wire already initialised in testI2CScan
    bool ok = mpr.begin(MPR121_LoRaE5::DEFAULT_ADDR, Wire);
    if (!ok) {
        fail(F("mpr.begin()"), F("device not found at 0x5A"));
        return;
    }
    pass(F("mpr.begin()"));

    mpr.stopMode();
    mpr.setThresholds(12, 6);
    mpr.setSampleIntervalMs(128);
    mpr.setCDC(0x10);
    mpr.runMode(4);   // electrodes 0–3 active
    delay(50);

    uint16_t mask = mpr.handleIRQ_and_getTouchMask();
    Serial.print(F("  Touch mask (raw 12-bit): 0x"));
    Serial.println(mask, HEX);

    Serial.print(F("  Electrode state: "));
    for (uint8_t i = 0; i < MAX_ELECTRODES; i++) {
        Serial.print(F("E")); Serial.print(i);
        Serial.print((mask >> i) & 1 ? F(":WET ") : F(":DRY "));
    }
    Serial.println();

    pass(F("MPR121 electrode read"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. RTC test
// ─────────────────────────────────────────────────────────────────────────────

static void testRTC() {
    section(F("RTC (LSE clock)"));

    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin(STM32RTC::HOUR_24);

    // Set a known epoch and verify it advances
    const time_t testEpoch = 1700000000UL;  // 2023-11-14 22:13:20 UTC
    rtc.setEpoch(testEpoch);
    delay(2100);   // wait slightly more than 2 s

    time_t readback = rtc.getEpoch();
    time_t delta    = readback - testEpoch;

    Serial.print(F("  Set epoch  : ")); Serial.println((uint32_t)testEpoch);
    Serial.print(F("  Read epoch : ")); Serial.println((uint32_t)readback);
    Serial.print(F("  Delta      : ")); Serial.print((int32_t)delta); Serial.println(F(" s (expect 2)"));

    if (delta < 1 || delta > 4) {
        fail(F("RTC tick"), F("epoch did not advance by ~2 s — check LSE crystal"));
    } else {
        pass(F("RTC tick and readback"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. SPI Flash test
// ─────────────────────────────────────────────────────────────────────────────

static void testSPIFlash() {
    section(F("SPI Flash (W25Qxx on SPI2, CS=PB9)"));

    SPI.setMOSI(MOSI_PIN);
    SPI.setMISO(MISO_PIN);
    SPI.setSCLK(SCK_PIN);

    if (!flash.begin()) {
        fail(F("flash.begin()"), F("no response — check SPI wiring and CS pin"));
        return;
    }
    pass(F("flash.begin()"));

    uint32_t jedec = flash.getJEDECID();
    Serial.print(F("  JEDEC ID     : 0x")); Serial.println(jedec, HEX);
    Serial.print(F("  Manufacturer : ")); Serial.println(flash.getManufacturer());
    Serial.print(F("  Model        : ")); Serial.println(flash.getModel());
    Serial.print(F("  Capacity     : ")); Serial.print(flash.getCapacity() / 1024); Serial.println(F(" KB"));

    if (jedec == 0x000000 || jedec == 0xFFFFFF) {
        fail(F("JEDEC ID"), F("all zeros or all ones — flash not responding"));
        return;
    }
    pass(F("JEDEC ID valid"));

    // Write/read/verify a single byte at address 0 (sector must be blank or erased)
    const uint32_t TEST_ADDR = 0x000000;
    const uint8_t  TEST_VAL  = 0xA5;

    flash.eraseSector(TEST_ADDR);
    flash.writeByte(TEST_ADDR, TEST_VAL, false);   // write without internal verify
    uint8_t readback = flash.readByte(TEST_ADDR);

    Serial.print(F("  Write 0x")); Serial.print(TEST_VAL, HEX);
    Serial.print(F("  →  Read 0x")); Serial.println(readback, HEX);

    if (readback != TEST_VAL) {
        fail(F("flash write/read"), F("readback mismatch"));
    } else {
        pass(F("flash erase / write / read"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. GPS (LC86L on LPUART1) — stream raw NMEA for 20 s
// ─────────────────────────────────────────────────────────────────────────────

static void testGPS() {
    section(F("GPS LC86L (LPUART1 PC0/PC1, EN=PB5)  —  20 s NMEA stream"));

    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, HIGH);
    delay(200);   // let module power up

    LPUART.begin(9600);
    delay(500);

    Serial.println(F("  ── Raw NMEA output ──────────────────────────────────────"));

    uint32_t start       = millis();
    uint32_t byteCount   = 0;
    uint32_t sentenceCount = 0;

    while (millis() - start < 20000UL) {
        while (LPUART.available()) {
            char c = (char)LPUART.read();
            Serial.print(c);
            byteCount++;
            if (c == '\n') sentenceCount++;
        }
    }

    Serial.println(F("  ── End of NMEA stream ───────────────────────────────────"));
    Serial.print(F("  Bytes received   : ")); Serial.println(byteCount);
    Serial.print(F("  Sentences counted: ")); Serial.println(sentenceCount);

    LPUART.end();
    digitalWrite(GPS_EN, LOW);   // power down GPS

    if (byteCount == 0) {
        fail(F("GPS NMEA"), F("no bytes received — check wiring and GPS_EN"));
    } else if (sentenceCount == 0) {
        fail(F("GPS NMEA"), F("bytes received but no newlines — possible baud mismatch"));
    } else {
        pass(F("GPS NMEA stream received"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup — run all tests once
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════════╗"));
    Serial.println(F("║   LC86L-LoRa-E5  Peripheral Self-Test           ║"));
    Serial.println(F("╚══════════════════════════════════════════════════╝"));
    Serial.print(F("Device tag: ")); Serial.println(tag);

    testRadio();
    testI2CScan();
    testLSM6DSL();
    testMPR121();
    testRTC();
    testSPIFlash();
    testGPS();

    // ── Summary ───────────────────────────────────────────────────────────────
    Serial.println();
    Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
    Serial.print(F("RESULT:  "));
    Serial.print(passCount); Serial.print(F(" passed   "));
    Serial.print(failCount); Serial.println(F(" failed"));
    if (failCount == 0) {
        Serial.println(F("All tests passed."));
    } else {
        Serial.println(F("One or more tests failed — review output above."));
    }
    Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop — halt; tests are one-shot in setup()
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    // nothing — MCU idles here after test run
}
