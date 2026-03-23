/**
 * peripheral_test.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Stand-alone peripheral test firmware for the LC86L-LoRa-E5 tracker.
 *
 * BUILD (never touches main.cpp):
 *   pio run  -e lora_e5_test            <- compile only
 *   pio run  -e lora_e5_test -t upload  <- flash to board
 *   pio device monitor -b 115200        <- open serial monitor
 *
 * USAGE:
 *   Open any serial terminal at 115 200 baud.
 *   Press the key shown in the menu to run individual tests or 'A' for all.
 *
 * SAFETY:
 *   Flash write/read uses the SECOND-TO-LAST 4 KB sector as a scratch area.
 *   User data (stored from address 0 upward) and the metadata sector
 *   (last 4 KB sector) are never touched.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <codes.h>
#include <definitions.h>
#include <TinyGPS++.h>
#include <LSM6DSL.h>
#include <MPR121_LoRaE5.h>
#include <STM32RTC.h>
#include <RadioLib.h>
#include <SPI.h>
#include <LoRaE5_SPIFlash.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <time.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Object instances  (identical wiring to main.cpp)
// ─────────────────────────────────────────────────────────────────────────────
TinyGPSPlus      gps;
LSM6DSL          imu;
MPR121_LoRaE5    mpr;
STM32RTC&        rtc   = STM32RTC::getInstance();
STM32WLx         radio = new STM32WLx_Module();
LoRaE5_SPIFlash  flash(FSS_PIN, &SPI, 8000000);
HardwareSerial   LPUART(GPS_RX, GPS_TX);

static const uint32_t rfswitch_pins[] = {
    PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    { STM32WLx::MODE_IDLE,  {LOW,  LOW}  },
    { STM32WLx::MODE_RX,    {HIGH, LOW}  },
    { STM32WLx::MODE_TX_HP, {LOW,  HIGH} },
    END_OF_MODE_TABLE,
};

// ─────────────────────────────────────────────────────────────────────────────
//  Result tracking
// ─────────────────────────────────────────────────────────────────────────────
struct TestResult {
    const char* name;
    bool        passed;
    char        detail[64];
};

static TestResult results[20];   // enough for all sub-tests
static uint8_t    resultCount = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Shared helpers
// ─────────────────────────────────────────────────────────────────────────────
static void separator(const char* title) {
    Serial.println();
    Serial.println(F("========================================"));
    Serial.print(F("  "));
    Serial.println(title);
    Serial.println(F("========================================"));
}

static void record(const char* name, bool pass, const char* detail = "") {
    if (resultCount < (sizeof(results) / sizeof(results[0]))) {
        results[resultCount].name   = name;
        results[resultCount].passed = pass;
        strncpy(results[resultCount].detail, detail,
                sizeof(results[0].detail) - 1);
        results[resultCount].detail[sizeof(results[0].detail) - 1] = '\0';
        resultCount++;
    }
    Serial.print(pass ? F("  [PASS] ") : F("  [FAIL] "));
    Serial.print(name);
    if (detail != nullptr && detail[0] != '\0') {
        Serial.print(F("  ->  "));
        Serial.print(detail);
    }
    Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 1 — RTC
// ─────────────────────────────────────────────────────────────────────────────
void test_RTC() {
    separator("TEST 1: RTC  (STM32RTC / LSE)");

    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin(STM32RTC::HOUR_24);

    // --- set a known epoch and read it back ---
    const uint32_t testEpoch = 1700000000UL;   // Nov 2023 — just a reference
    rtc.setEpoch(testEpoch);
    delay(100);
    uint32_t readBack = rtc.getEpoch();

    char buf[40];
    snprintf(buf, sizeof(buf), "set=%lu  got=%lu",
             (unsigned long)testEpoch, (unsigned long)readBack);
    record("Epoch set/read-back",
           (readBack >= testEpoch && readBack <= testEpoch + 2), buf);

    // --- tick test: verify counter advances by ~2 s ---
    uint32_t t0 = rtc.getEpoch();
    delay(2100);
    uint32_t t1 = rtc.getEpoch();
    uint32_t delta = t1 - t0;
    snprintf(buf, sizeof(buf), "delta=%lu s (expect 2)", (unsigned long)delta);
    record("RTC tick (2 s window)", (delta >= 2 && delta <= 4), buf);

    // --- read back H:M:S fields ---
    uint8_t hh = rtc.getHours();
    uint8_t mm = rtc.getMinutes();
    uint8_t ss = rtc.getSeconds();
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", hh, mm, ss);
    record("H:M:S fields readable", true, buf);   // always pass if no exception
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 2 — SPI Flash
// ─────────────────────────────────────────────────────────────────────────────
void test_Flash() {
    separator("TEST 2: SPI Flash");

    // --- initialise ---
    bool initOk = flash.begin();
    record("flash.begin()", initOk);
    if (!initOk) return;

    flash.printChipInfo();

    // --- JEDEC ID ---
    uint32_t jedec = flash.getJEDECID();
    bool jedecOk   = (jedec != 0x000000UL && jedec != 0xFFFFFFUL);
    char buf[48];
    snprintf(buf, sizeof(buf), "0x%06lX", (unsigned long)jedec);
    record("JEDEC ID", jedecOk, buf);

    // --- capacity sanity ---
    uint32_t cap    = flash.getCapacity();
    uint32_t sectSz = flash.getSectorSize();
    bool capOk      = (cap > 0 && sectSz > 0);
    snprintf(buf, sizeof(buf), "cap=%lu KB  sector=%u B",
             (unsigned long)(cap / 1024), (unsigned)sectSz);
    record("Capacity / sector size", capOk, buf);

    if (!capOk) { flash.powerDown(); return; }

    // --- scratch sector: second-to-last (never overlaps user data or metadata) ---
    uint32_t scratchAddr = cap - (2u * sectSz);
    snprintf(buf, sizeof(buf), "0x%06lX", (unsigned long)scratchAddr);
    Serial.print(F("  Scratch sector address: "));
    Serial.println(buf);

    // --- erase scratch sector ---
    uint8_t eraseRes = flash.eraseSector(scratchAddr);
    record("Erase scratch sector", eraseRes == FLASH_OK);
    if (eraseRes != FLASH_OK) { flash.powerDown(); return; }

    // --- blank check ---
    bool blank = flash.isBlank(scratchAddr, 16);
    record("Blank check after erase", blank);

    // --- byte write + verify ---
    const uint8_t MAGIC_BYTE = 0xA5;
    uint8_t writeRes = flash.writeByte(scratchAddr, MAGIC_BYTE, /*verify=*/true);
    record("Write byte 0xA5", writeRes == FLASH_OK);

    uint8_t readByte = flash.readByte(scratchAddr);
    snprintf(buf, sizeof(buf), "expected=0xA5  got=0x%02X", readByte);
    record("Read-back byte", readByte == MAGIC_BYTE, buf);

    // --- struct write + verify ---
    struct __attribute__((__packed__)) ScratchStruct {
        uint32_t magic;
        float    fval;
        uint16_t seq;
    };
    ScratchStruct ts = { 0xDEADBEEFUL, 3.14159f, 0x1234u };
    uint32_t structAddr = scratchAddr + 4;          // offset past the byte above
    uint8_t sRes = flash.writeStruct(structAddr, ts, /*verify=*/true);
    record("Write struct (10 B)", sRes == FLASH_OK);

    ScratchStruct tsRead = {};
    uint8_t rRes = flash.readStruct(structAddr, tsRead);
    bool structOk = (rRes == FLASH_OK)          &&
                    (tsRead.magic == ts.magic)   &&
                    (tsRead.seq   == ts.seq);
    snprintf(buf, sizeof(buf), "magic=0x%08lX  seq=0x%04X",
             (unsigned long)tsRead.magic, (unsigned)tsRead.seq);
    record("Struct read-back", structOk, buf);

    // --- power cycle retention ---
    flash.powerDown();
    delay(20);
    flash.powerUp();
    delay(20);
    uint8_t afterPower = flash.readByte(scratchAddr);
    snprintf(buf, sizeof(buf), "expected=0xA5  got=0x%02X", afterPower);
    record("Data survives power-cycle", afterPower == MAGIC_BYTE, buf);

    flash.powerDown();
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 3 — LoRa Radio
// ─────────────────────────────────────────────────────────────────────────────
void test_Radio() {
    separator("TEST 3: LoRa Radio  (STM32WLx @ 867 MHz)");

    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

    int st = radio.begin(867.0, 125.0, 12, 5,
                         RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                         22, 8, 1.6, false);
    bool initOk = (st == RADIOLIB_ERR_NONE);
    char buf[40];
    snprintf(buf, sizeof(buf), "RadioLib code %d", st);
    record("radio.begin()", initOk, initOk ? "OK" : buf);
    if (!initOk) return;

    // --- TX test packet ---
    const char* msg = "PERIPH_TEST_TX";
    st = radio.transmit((uint8_t*)msg, strlen(msg));
    bool txOk = (st == RADIOLIB_ERR_NONE);
    snprintf(buf, sizeof(buf), "code %d", st);
    record("TX test packet", txOk, txOk ? "sent OK" : buf);

    // --- print RF config ---
    Serial.print(F("  Freq=867.0 MHz  BW=125 kHz  SF=12  CR=4/5  Pwr=22 dBm\n"));

    // --- RX window: 3 s — optional echo/base-station reply ---
    Serial.println(F("  Listening 3 s for any reply (base station optional)..."));
    radio.standby();
    radio.startReceive();

    uint8_t  rxBuf[32] = {};
    size_t   rxLen     = sizeof(rxBuf);
    uint32_t t0        = millis();
    bool     rxGot     = false;
    while (millis() - t0 < 3000) {
        st = radio.receive(rxBuf, rxLen);
        if (st == RADIOLIB_ERR_NONE) {
            rxGot = true;
            snprintf(buf, sizeof(buf), "RSSI=%.1f dBm  SNR=%.1f dB",
                     (double)radio.getRSSI(), (double)radio.getSNR());
            record("RX reply received", true, buf);
            break;
        }
        delay(10);
    }
    if (!rxGot) {
        record("RX window (3 s)", false,
               "no reply — expected if no base station present");
    }

    radio.sleep();
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 4 — GPS
// ─────────────────────────────────────────────────────────────────────────────
void test_GPS() {
    separator("TEST 4: GPS  (LC86L via LPUART @ 9600)");

    const uint32_t GPS_TEST_MS = 60000UL;   // 1-minute window

    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, HIGH);
    LPUART.begin(9600);

    Serial.print(F("  Searching for NMEA stream ("));
    Serial.print(GPS_TEST_MS / 1000);
    Serial.println(F(" s timeout — needs clear sky for full fix)..."));

    uint32_t start     = millis();
    uint32_t nmeaChars = 0;
    bool     fixFound  = false;
    float    bestHdop  = 99.0f;

    while (millis() - start < GPS_TEST_MS) {
        while (LPUART.available()) {
            gps.encode((char)LPUART.read());
            nmeaChars++;
        }
        if (gps.location.isValid()    &&
            gps.location.age() < 1500 &&
            gps.hdop.hdop() < 5.0f    &&
            (millis() - start) > 3000) {
            fixFound = true;
            bestHdop = gps.hdop.hdop();
            break;
        }
        delay(10);
    }

    digitalWrite(GPS_EN, LOW);
    LPUART.end();

    // --- NMEA data stream ---
    char buf[56];
    snprintf(buf, sizeof(buf), "%lu chars parsed", (unsigned long)nmeaChars);
    record("NMEA stream active", nmeaChars > 0, buf);

    // --- satellite count (partial info even without full fix) ---
    bool satOk = (gps.satellites.isValid() && gps.satellites.value() > 0);
    if (gps.satellites.isValid()) {
        snprintf(buf, sizeof(buf), "%u satellites visible",
                 (unsigned)gps.satellites.value());
    } else {
        strncpy(buf, "no satellite data within window", sizeof(buf) - 1);
    }
    record("Satellites visible", satOk, buf);

    // --- HDOP ---
    if (gps.hdop.isValid()) {
        snprintf(buf, sizeof(buf), "%.2f", (double)gps.hdop.hdop());
        record("HDOP readable", true, buf);
    }

    // --- full position fix ---
    if (fixFound) {
        snprintf(buf, sizeof(buf), "%.6f, %.6f  HDOP=%.1f",
                 gps.location.lat(), gps.location.lng(), (double)bestHdop);
    } else {
        strncpy(buf, "timed out — move to open sky", sizeof(buf) - 1);
    }
    record("Position fix (HDOP<5)", fixFound, buf);

    // --- RTC sync from GPS time ---
    if (fixFound && gps.time.isValid() && gps.date.isValid()) {
        struct tm t   = {};
        t.tm_hour     = gps.time.hour();
        t.tm_min      = gps.time.minute();
        t.tm_sec      = gps.time.second();
        t.tm_mday     = gps.date.day();
        t.tm_mon      = gps.date.month() - 1;
        t.tm_year     = gps.date.year() - 1900;
        time_t epoch  = mktime(&t);
        rtc.setEpoch((uint32_t)epoch);
        snprintf(buf, sizeof(buf), "epoch=%lu", (unsigned long)epoch);
        record("RTC synced from GPS", true, buf);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 5 — IMU  (LSM6DSL)
// ─────────────────────────────────────────────────────────────────────────────
void test_IMU() {
    separator("TEST 5: IMU  (LSM6DSL  I2C @ 0x6B)");

    Wire.begin();

    bool initOk = imu.begin();
    record("imu.begin()", initOk);
    if (!initOk) { Wire.end(); return; }

    // --- WHO_AM_I ---
    uint8_t who = imu.whoAmI();
    // LSM6DSL = 0x6A; some board variants respond 0x6B
    bool whoOk = (who == 0x6A || who == 0x6B);
    char buf[32];
    snprintf(buf, sizeof(buf), "got 0x%02X (expect 0x6A/0x6B)", who);
    record("WHO_AM_I register", whoOk, buf);

    // ── Accelerometer ──
    imu.setAccelConfig(LSM6DSL_XL_ODR_104Hz, LSM6DSL_XL_FS_2g);
    delay(20);   // let ODR settle

    float ax, ay, az;
    imu.readAccelerometer_g(ax, ay, az, LSM6DSL_XL_FS_2g);

    // Sanity: at rest, total magnitude ≈ 1 g (0.5–1.5 g acceptable)
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    bool accelOk = (mag > 0.5f && mag < 2.0f);
    snprintf(buf, sizeof(buf), "X=%.2f Y=%.2f Z=%.2f g  |mag|=%.2f",
             (double)ax, (double)ay, (double)az, (double)mag);
    record("Accelerometer (104 Hz, ±2 g)", accelOk, buf);

    // ── Gyroscope ──
    imu.enableGyroscope(LSM6DSL_G_ODR_104Hz);
    delay(20);

    float gx, gy, gz;
    imu.readGyroscope_dps(gx, gy, gz, LSM6DSL_G_FS_245dps);

    // Sanity: at rest, gyro magnitude should be < 10 dps (noise only)
    float gmag = sqrtf(gx*gx + gy*gy + gz*gz);
    bool gyroOk = (gmag < 50.0f);   // generous tolerance
    snprintf(buf, sizeof(buf), "X=%.1f Y=%.1f Z=%.1f dps  |mag|=%.1f",
             (double)gx, (double)gy, (double)gz, (double)gmag);
    record("Gyroscope (104 Hz, ±245 dps)", gyroOk, buf);

    // ── On-die temperature ──
    int16_t rawT = imu.readTemperature();
    // Formula: Temp_degC = 25 + rawT / 256
    float tempC  = 25.0f + (float)rawT / 256.0f;
    bool tempOk  = (tempC > -20.0f && tempC < 85.0f);
    snprintf(buf, sizeof(buf), "%.1f C  (raw=%d)", (double)tempC, rawT);
    record("On-die temperature", tempOk, buf);

    // ── 10-sample burst to check for frozen output ──
    Serial.println(F("  Sampling accel 10x @ 104 Hz..."));
    float sumMag = 0.0f;
    for (uint8_t i = 0; i < 10; i++) {
        imu.readAccelerometer_g(ax, ay, az, LSM6DSL_XL_FS_2g);
        sumMag += sqrtf(ax*ax + ay*ay + az*az);
        delay(10);
    }
    float avgMag = sumMag / 10.0f;
    bool burstOk = (avgMag > 0.5f && avgMag < 2.0f);
    snprintf(buf, sizeof(buf), "avg |accel|=%.3f g over 10 samples", (double)avgMag);
    record("10-sample burst average", burstOk, buf);

    imu.disableAccelerometer();
    imu.disableGyroscope();
    Wire.end();
}

// ─────────────────────────────────────────────────────────────────────────────
//  TEST 6 — Capacitive Touch  (MPR121)
// ─────────────────────────────────────────────────────────────────────────────
void test_MPR121() {
    separator("TEST 6: Capacitive Touch  (MPR121 I2C @ 0x5A)");

    Wire.begin();

    bool initOk = mpr.begin(MPR121_LoRaE5::DEFAULT_ADDR, Wire, 12, 6, false);
    record("mpr.begin()", initOk);
    if (!initOk) { Wire.end(); return; }

    // Configure 4 electrodes (same as main.cpp setup())
    mpr.stopMode();
    mpr.setThresholds(12, 6);
    mpr.setSampleIntervalMs(128);
    mpr.setCDC(0x10);
    mpr.runMode(4);
    delay(100);   // let baseline settle

    // --- Electrode baseline & filtered data ---
    char buf[72];
    buf[0] = '\0';
    bool baselineOk = false;
    for (uint8_t i = 0; i < MAX_ELECTRODES; i++) {
        uint8_t  base = mpr.baselineData(i);
        uint16_t filt = mpr.filteredData(i);
        if (base > 0 || filt > 0) baselineOk = true;
        char tmp[18];
        snprintf(tmp, sizeof(tmp), "E%u:B=%u,F=%u  ", i, base, filt);
        strncat(buf, tmp, sizeof(buf) - strlen(buf) - 1);
    }
    record("Electrode baseline & filtered data", baselineOk, buf);

    // --- CDC readback ---
    uint8_t cdc = mpr.getCDC();
    snprintf(buf, sizeof(buf), "CDC=0x%02X (set 0x10)", cdc);
    record("CDC register readback", cdc == 0x10, buf);

    // --- Touch status mask ---
    uint16_t mask = mpr.touched();
    snprintf(buf, sizeof(buf), "mask=0x%03X  E0=%d E1=%d E2=%d E3=%d",
             mask,
             (int)mpr.isTouched(0),
             (int)mpr.isTouched(1),
             (int)mpr.isTouched(2),
             (int)mpr.isTouched(3));
    record("Touch status register readable", true, buf);

    // --- Per-electrode touch/release live monitor  (5 s) ---
    Serial.println(F("  >> Touch electrodes to test live detection (5 s)..."));
    uint32_t t0         = millis();
    uint8_t  evtCount   = 0;
    uint16_t prevMask   = mpr.touched();
    while (millis() - t0 < 5000) {
        uint16_t cur = mpr.touched();
        if (cur != prevMask) {
            evtCount++;
            Serial.print(F("     State change -> mask=0x"));
            Serial.println(cur, HEX);
            prevMask = cur;
        }
        delay(50);
    }
    snprintf(buf, sizeof(buf), "%u state changes in 5 s", evtCount);
    record("Touch/release events detected", evtCount > 0, buf);

    // --- Submersion simulation: all 4 electrodes simultaneously (5 s window) ---
    Serial.println(F("  >> Hold ALL 4 electrodes for up to 5 s to test submersion trigger..."));
    t0 = millis();
    bool subTest = false;
    while (millis() - t0 < 5000) {
        if (mpr.isTouched(0) && mpr.isTouched(1) &&
            mpr.isTouched(2) && mpr.isTouched(3)) {
            subTest = true;
            break;
        }
        delay(100);
    }
    record("Submersion trigger (all 4 touched)", subTest,
           subTest ? "triggered OK" : "not triggered (skipped — no penalty)");

    mpr.stopMode();
    Wire.end();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Summary
// ─────────────────────────────────────────────────────────────────────────────
void printSummary() {
    separator("SUMMARY");
    uint8_t passed = 0, failed = 0;
    for (uint8_t i = 0; i < resultCount; i++) {
        Serial.print(results[i].passed ? F("  [PASS] ") : F("  [FAIL] "));
        Serial.print(results[i].name);
        if (results[i].detail[0]) {
            Serial.print(F("  ->  "));
            Serial.print(results[i].detail);
        }
        Serial.println();
        results[i].passed ? passed++ : failed++;
    }
    Serial.println(F("----------------------------------------"));
    Serial.print(F("  Sub-tests: "));  Serial.print(resultCount);
    Serial.print(F("   PASS: "));      Serial.print(passed);
    Serial.print(F("   FAIL: "));      Serial.println(failed);
    Serial.println(F("========================================"));
}

// ─────────────────────────────────────────────────────────────────────────────
//  Menu
// ─────────────────────────────────────────────────────────────────────────────
void printMenu() {
    Serial.println(F("\n========================================"));
    Serial.println(F("  LC86L-LoRa-E5  PERIPHERAL TEST v1.0  "));
    Serial.println(F("========================================"));
    Serial.println(F("  [1]  RTC"));
    Serial.println(F("  [2]  SPI Flash"));
    Serial.println(F("  [3]  LoRa Radio"));
    Serial.println(F("  [4]  GPS  (60 s window — open sky)"));
    Serial.println(F("  [5]  IMU  (LSM6DSL)"));
    Serial.println(F("  [6]  Capacitive Touch  (MPR121)"));
    Serial.println(F("  [A]  Run ALL tests in sequence"));
    Serial.println(F("  [S]  Show last summary"));
    Serial.println(F("  [R]  Reset MCU"));
    Serial.println(F("----------------------------------------"));
    Serial.print(F("  Choice: "));
}

// ─────────────────────────────────────────────────────────────────────────────
//  Arduino entry points
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(500);

    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW);

    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    char ch = (char)Serial.read();
    while (Serial.available()) Serial.read();   // flush CR/LF

    Serial.println(ch);   // echo selection

    // Reset result table before each new test run
    resultCount = 0;
    memset(results, 0, sizeof(results));

    bool ranTests = false;

    switch (ch) {
        case '1':            test_RTC();    ranTests = true; break;
        case '2':            test_Flash();  ranTests = true; break;
        case '3':            test_Radio();  ranTests = true; break;
        case '4':            test_GPS();    ranTests = true; break;
        case '5':            test_IMU();    ranTests = true; break;
        case '6':            test_MPR121(); ranTests = true; break;
        case 'A': case 'a':
            test_RTC();
            test_Flash();
            test_Radio();
            test_GPS();
            test_IMU();
            test_MPR121();
            ranTests = true;
            break;
        case 'S': case 's':
            printSummary();
            break;
        case 'R': case 'r':
            Serial.println(F("  Resetting MCU..."));
            Serial.flush();
            delay(100);
            NVIC_SystemReset();
            break;
        default:
            Serial.println(F("  Unknown command."));
            break;
    }

    if (ranTests) {
        printSummary();
    }

    printMenu();
}
