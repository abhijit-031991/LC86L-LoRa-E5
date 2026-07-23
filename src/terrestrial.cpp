/**
 * terrestrial.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Firmware for the LC86L-LoRa-E5 — Terrestrial Variant
 *
 * Hardware:
 *   STM32WLE5JC   Internal sub-GHz LoRa radio
 *   LC86L         GPS module on LPUART (PC0/PC1), enable pin PB5
 *   LSM6DSL       IMU (accelerometer) on I2C (SDA=PA15, SCL=PB15)
 *   W25Qxx        SPI NOR Flash on SPI (CS=PB9)
 *   STM32 RTC     LSE-clocked, 1-second interrupt
 *
 * Radio config:  867 MHz | SF12 | BW 125 kHz | CR 4/5 | 22 dBm
 *
 * Flash:  pio run -e terrestrial -t upload
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * FEATURE STATUS
 * ─────────────────────────────────────────────────────────────────────────────
 *  [DONE]  LoRa simple ping + long ping (tag, record count, lat/lng, devtype)
 *  [DONE]  Settings update over LoRa (GPS freq, timeout, HDOP, radio freq, schedule)
 *  [DONE]  GPS fix acquisition with configurable timeout and HDOP threshold
 *  [DONE]  RTC synchronised from GPS time on first fix
 *  [DONE]  Sequential flash data logging (lat, lng, hdop, xyz, timestamp, locktime)
 *  [DONE]  Wear-leveled flash metadata — writeAdd/readAdd survive power cycles
 *  [DONE]  Full data download over LoRa (DATA_DOWNLOAD_ALL)
 *  [DONE]  New-records-only download over LoRa (DATA_DOWNLOAD_NEW)
 *  [DONE]  LSM6DSL accelerometer reading stored in each data record
 *  [DONE]  Device calibration mode on boot (flash + IMU + GPS checks)
 *  [DONE]  Deep sleep between RTC ticks (STM32 Stop1 mode)
 *  [DONE]  Scheduled transmission window — when scheduled=true, radio pings
 *          are only transmitted within the startHour–endHour window. GPS
 *          acquisition is unaffected. Overnight windows (e.g. 22–06) are
 *          handled correctly. The counter always resets so the interval
 *          stays consistent when the window re-opens.
 *  [TODO]  Flash full notification — currently returns silently when the data
 *          area is full. Transmit MEMORY_FULL to the base station instead.
 *  [DONE]  Movement-based mortality detection via AccelMetrics library.
 *          IMU sampled every IMU_SAMPLE_INTERVAL seconds into a circular
 *          buffer. isMortality() checks sustained inactivity over the window
 *          and drives the mortality flag in every long ping.
 *  [TODO]  IMU motion wake — configure ACC_INT1 (PB4) as a wakeup source so
 *          the device can log an event immediately on significant movement
 *          without waiting for the next GPS interval.
 *  [TODO]  GPS warm-start optimisation — save almanac/ephemeris to flash and
 *          send PMTK hot-start command to reduce time-to-first-fix.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <TimeLib.h>
#include <time.h>
#include <Wire.h>
#include <LSM6DSL.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <LoRaE5_SPIFlash.h>
#include <STM32Stop1.h>
#include <STM32RTC.h>
#include <RadioLib.h>
#include <STM32LowPower.h>
#include <codes.h>
#include <definitions.h>
#include <AccelMetrics.h>

// ─────────────────────────────────────────────────────────────────────────────
// Object instances
// ─────────────────────────────────────────────────────────────────────────────

TinyGPSPlus      gps;
HardwareSerial   LPUART(GPS_RX, GPS_TX);   // LPUART mapped to GPS pins
LSM6DSL          imu;                      // I2C address 0x6B (default)
STM32Stop1       stop1;
STM32RTC&        rtc   = STM32RTC::getInstance();
STM32WLx         radio = new STM32WLx_Module();
LoRaE5_SPIFlash  flash(FSS_PIN, &SPI, 8000000);

// ─────────────────────────────────────────────────────────────────────────────
// RF switch table (PA4 = RX enable, PA5 = TX HP enable)
// ─────────────────────────────────────────────────────────────────────────────

static const uint32_t rfswitch_pins[] = {
    PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};
static const Module::RfSwitchMode_t rfswitch_table[] = {
    { STM32WLx::MODE_IDLE,  {LOW,  LOW } },
    { STM32WLx::MODE_RX,    {HIGH, LOW } },
    { STM32WLx::MODE_TX_HP, {LOW,  HIGH} },
    END_OF_MODE_TABLE,
};

// ─────────────────────────────────────────────────────────────────────────────
// Flash metadata — wear-leveled storage of writeAdd/readAdd in the last sector
// ─────────────────────────────────────────────────────────────────────────────

#define FLASH_META_MAGIC  0xA5A5A5A5UL

struct __attribute__((__packed__)) FlashMeta {
    uint32_t magic;     // FLASH_META_MAGIC when slot is valid
    uint32_t writeAdd;  // Saved write pointer
    uint32_t readAdd;   // Saved read pointer
};

// Derived from the struct so it stays correct if fields are ever added
#define FLASH_META_SIZE   sizeof(FlashMeta)

// ─────────────────────────────────────────────────────────────────────────────
// IMU sampling — AccelMetrics configuration
// ─────────────────────────────────────────────────────────────────────────────
//
// Sampling strategy: burst-average at GPS acquisition time.
//   One sample per GPS fix is added to the AccelMetrics buffer.
//   Each sample is the mean of IMU_BURST_SAMPLES rapid readings taken
//   immediately after the GPS fix (or timeout), costing ~1 second of I2C
//   activity per GPS interval instead of continuous background sampling.
//
// IMU_BURST_SAMPLES    Number of readings to average per GPS event.
//                      At 52 Hz ODR with 20 ms between reads:
//                      50 samples ≈ 1 second of data per fix.
//
// IMU_BUFFER_SIZE      GPS-interval snapshots held in the circular buffer.
//                      At the default 3-minute GPS interval, 48 snapshots
//                      cover ~2.4 hours — the evidence window for
//                      isMortality().
//                      Memory cost: 3 × 48 × 4 = 576 bytes.

#define IMU_BURST_SAMPLES  50
#define IMU_BUFFER_SIZE    48   // ~2.4-hour mortality window at 3-min GPS

float        imuBufX[IMU_BUFFER_SIZE];
float        imuBufY[IMU_BUFFER_SIZE];
float        imuBufZ[IMU_BUFFER_SIZE];
AccelMetrics accel(imuBufX, imuBufY, imuBufZ, IMU_BUFFER_SIZE);

// Forward declarations for metadata helpers
bool loadFlashMetadata();
void saveFlashMetadata();

// ─────────────────────────────────────────────────────────────────────────────
// Runtime variables
// ─────────────────────────────────────────────────────────────────────────────

// RTC-driven software timers (incremented every second in isr())
volatile unsigned long gpsCounter  = 0;
volatile unsigned long pingCounter = 0;

// Timer targets (recalculated whenever settings change)
unsigned long gpsCounterTarget;
unsigned long pingCounterTarget;

// Flash address pointers (restored from flash metadata on boot)
unsigned long writeAdd = 0;
unsigned long readAdd  = 0;
bool          flashOk  = false;  // set true only when flash.begin() succeeds
bool          imuReady = false;  // set true only when imu.begin() succeeds

// Configurable settings (updated over LoRa)
int gpsFrequency  = 60;   // minutes between GPS acquisitions (1 hour default)
int gpsTimeout    = 120;  // seconds to wait for a good fix
int gpsHdop       = 5;    // HDOP threshold — lower is better
int radioFrequency = 1;   // minutes between radio pings
int startHour     = 0;    // scheduled mode window start (0–23)
int endHour       = 23;   // scheduled mode window end   (0–23)

// Flags and state
bool scheduled = false;   // true = only ping within startHour–endHour
float lat  = 0.0f;        // last known latitude
float lng  = 0.0f;        // last known longitude
unsigned int count  = 0;  // total GPS records acquired this session
unsigned int pingId = 0;  // increments with each acknowledged ping

int state = RADIOLIB_ERR_NONE;  // last RadioLib status code

// ─────────────────────────────────────────────────────────────────────────────
// isInScheduledWindow — returns true if the current RTC hour falls inside the
// configured transmission window.
//
// Two cases:
//   Normal    startHour <= endHour  e.g. 08–18  →  active during the day
//   Overnight startHour >  endHour  e.g. 22–06  →  active across midnight
//
// Called only when scheduled=true; ignored in main mode.
// ─────────────────────────────────────────────────────────────────────────────

bool isInScheduledWindow() {
    uint8_t hour = rtc.getHours();
    if (startHour <= endHour) {
        return hour >= startHour && hour <= endHour;
    } else {
        // Overnight window — active from startHour through midnight and up to endHour
        return hour >= startHour || hour <= endHour;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// RTC 1-second interrupt service routine
// Increments all software counters; keep this as short as possible.
// ─────────────────────────────────────────────────────────────────────────────

void isr(void* /*data*/) {
    gpsCounter++;
    pingCounter++;
}

// ─────────────────────────────────────────────────────────────────────────────
// Recalculate timer targets after a settings update
// ─────────────────────────────────────────────────────────────────────────────

void calculateTargets() {
    gpsCounterTarget  = (unsigned long)gpsFrequency  * 60UL;
    pingCounterTarget = (unsigned long)radioFrequency * 60UL;
    Serial.print(F("GPS target: "));   Serial.print(gpsCounterTarget);
    Serial.print(F("s | Ping target: ")); Serial.print(pingCounterTarget);
    Serial.println(F("s"));
}

// ─────────────────────────────────────────────────────────────────────────────
// readImu — enables the accelerometer, takes `samples` readings, returns the
// mean as x/y/z in g (full scale ±2 g), then disables both sensors.
//
// samples = 1   single snapshot (fast, ~50 ms total)
// samples = 50  burst average   (~1 s total at 52 Hz ODR)
//
// The burst-average mode is used at GPS acquisition time: 50 readings at
// 20 ms intervals reduce quantization noise and give a representative mean
// of the device's state during that GPS interval.  The averaged value is
// stored in the data record AND fed to AccelMetrics for mortality detection.
//
// imu.begin() is called ONCE in setup().  Calling it here every read would
// re-enable the gyroscope at 104 Hz and leave it running during deep sleep,
// wasting ~5 mA continuously.  We only enable the accelerometer for the
// duration of each read, then disable both sensors.
// ─────────────────────────────────────────────────────────────────────────────

bool readImu(float &x, float &y, float &z, uint8_t samples = 1) {
    if (!imuReady) return false;
    Wire.begin();
    imu.enableAccelerometer(LSM6DSL_XL_ODR_52Hz);
    delay(30);   // settling time — one ODR period at 52 Hz ≈ 19 ms

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    for (uint8_t i = 0; i < samples; i++) {
        float sx, sy, sz;
        imu.readAccelerometer_g(sx, sy, sz, LSM6DSL_XL_FS_2g);
        sumX += sx; sumY += sy; sumZ += sz;
        if (i < samples - 1) delay(20);  // wait one ODR period before next read
    }

    imu.disableAccelerometer();
    imu.disableGyroscope();   // imu.begin() enables gyro at 104 Hz — keep it off
    Wire.end();

    x = sumX / (float)samples;
    y = sumY / (float)samples;
    z = sumZ / (float)samples;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// getlocation — acquires a GPS fix, reads the IMU, and writes a data record
//               to flash.
//
// Parameters:
//   setup  true  → first-boot call; uses a 30-minute timeout and sets the RTC
//          false → normal periodic call; uses gpsTimeout setting
//   GF     output flag — set true if a good fix was obtained
// ─────────────────────────────────────────────────────────────────────────────

void getlocation(bool setup, bool &GF) {
    Serial.println(F("Acquiring GPS fix..."));

    unsigned long timeout = setup ? 30UL * 60UL * 1000UL
                                  : (unsigned long)gpsTimeout * 1000UL;

    // Power up GPS module and open LPUART
    digitalWrite(GPS_EN, HIGH);
    LPUART.begin(9600);

    unsigned long start        = millis();
    unsigned long fixMillis    = 0;      // elapsed ms at the moment of good fix
    bool          goodFix      = false;
    float         currentHdop  = 99.0f;  // updated only when a fresh non-zero
                                         // HDOP arrives; avoids accepting a
                                         // cached value left over from a prior
                                         // session (TinyGPS++ library quirk)
    unsigned long nmeaBytes    = 0;      // total bytes received; used to detect
                                         // a non-responsive GPS module

    while (millis() - start < timeout && !goodFix) {
        while (LPUART.available()) {
            nmeaBytes++;
            if (gps.encode(LPUART.read()) && gps.location.isValid()) {
                if (gps.hdop.hdop() > 0.0f)
                    currentHdop = gps.hdop.hdop();

                // Accept fix only after 3 s warm-up, with fresh data and
                // HDOP below threshold
                if (currentHdop < (float)gpsHdop &&
                    gps.location.age() < 1000 &&
                    gps.time.age()     < 1000 &&
                    millis() - start   > 3000) {
                    goodFix   = true;
                    fixMillis = millis() - start;  // capture at moment of fix
                    Serial.println(F("Good fix acquired"));
                    break;
                }
            }
        }
        delay(10);
    }

    // Power down GPS
    digitalWrite(GPS_EN, LOW);
    LPUART.end();

    GF = goodFix;

    // ── IMU burst average ─────────────────────────────────────────────────────
    // Taken here regardless of GPS fix result so the AccelMetrics buffer
    // keeps accumulating for mortality detection even on timeout intervals.
    // Skipped on the setup call (setup=true) which only sets the RTC.
    float imuX = 0.0f, imuY = 0.0f, imuZ = 0.0f;
    bool  imuOk = false;
    if (!setup) {
        imuOk = readImu(imuX, imuY, imuZ, IMU_BURST_SAMPLES);
        if (imuOk) {
            accel.addSample(imuX, imuY, imuZ);
        }
    }

    if (!goodFix) {
        if (nmeaBytes == 0) {
            Serial.println(F("GPS timeout — no NMEA bytes received (module not responding?)"));
        } else {
            Serial.print(F("GPS timeout — no good fix ("));
            Serial.print(nmeaBytes);
            Serial.println(F(" bytes received)"));
        }
        return;
    }

    // Update global position (used by long ping between GPS acquisitions)
    lat = gps.location.lat();
    lng = gps.location.lng();
    Serial.print(F("Lat: ")); Serial.print(lat, 6);
    Serial.print(F("  Lng: ")); Serial.println(lng, 6);

    // On the setup call, synchronise the RTC from GPS time
    if (setup && gps.time.isValid() && gps.date.isValid()) {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
                gps.date.day(),  gps.date.month(),  gps.date.year());
        rtc.setEpoch((uint32_t)now());
        Serial.print(F("RTC set to epoch: ")); Serial.println(rtc.getEpoch());
        return;  // setup call — no flash write needed
    }

    // ── Normal call: read IMU, build data struct, write to flash ─────────────

    data dat;
    dat.id       = tag;
    dat.lat      = gps.location.lat();   // read directly — fix is confirmed valid
    dat.lng      = gps.location.lng();
    dat.hdop     = currentHdop;          // use the validated local value
    dat.locktime = (uint16_t)min(fixMillis / 1000UL, 65535UL);  // time-to-fix
    dat.count    = ++count;

    // Timestamp: prefer GPS time, fall back to RTC
    if (gps.time.isValid() && gps.date.isValid()) {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
                gps.date.day(),  gps.date.month(),  gps.date.year());
        dat.datetime = (uint32_t)now();
    } else {
        dat.datetime = rtc.getEpoch();
    }

    // IMU burst average was already taken above and fed to AccelMetrics.
    // Reuse those values here so we don't sample again.
    dat.x = imuOk ? imuX : 0.0f;
    dat.y = imuOk ? imuY : 0.0f;
    dat.z = imuOk ? imuZ : 0.0f;
    Serial.print(F("IMU avg (g): "));
    Serial.print(dat.x, 3); Serial.print(F(", "));
    Serial.print(dat.y, 3); Serial.print(F(", "));
    Serial.println(dat.z, 3);

    // ── Flash write ──────────────────────────────────────────────────────────

    if (!flashOk) {
        Serial.println(F("Flash not initialised — record not written"));
        return;
    }

    flash.powerUp();
    delay(10);

    // Guard: keep the last sector reserved for wear-leveled metadata
    uint32_t sectorSize = flash.getSectorSize();
    uint32_t metaSector = flash.getCapacity() - sectorSize;
    if (writeAdd + sizeof(dat) > metaSector) {
        Serial.println(F("Flash data area full — record not written"));
        flash.powerDown();
        return;
    }

    // Erase sector boundary(ies) crossed by this write.
    // Both results are checked — if an erase fails we abort rather than
    // writing into an un-erased sector and corrupting data.
    uint32_t writeEnd = writeAdd + sizeof(dat) - 1;
    if (writeAdd % sectorSize == 0) {
        if (flash.eraseSector(writeAdd) != FLASH_OK) {
            Serial.println(F("Sector erase failed — record not written"));
            flash.powerDown();
            return;
        }
    }
    if ((writeEnd / sectorSize) > (writeAdd / sectorSize)) {
        uint32_t nextSector = ((writeAdd / sectorSize) + 1) * sectorSize;
        if (flash.eraseSector(nextSector) != FLASH_OK) {
            Serial.println(F("Next sector erase failed — record not written"));
            flash.powerDown();
            return;
        }
    }

    uint8_t result = flash.writeStruct(writeAdd, dat, /*verify=*/true);
    if (result == FLASH_OK) {
        writeAdd += sizeof(dat);
        saveFlashMetadata();
        pingId = 0;   // reset ping counter after new data written
        Serial.print(F("Flash write OK — total records: "));
        Serial.println(writeAdd / sizeof(dat));
    } else {
        Serial.print(F("Flash write failed, code: ")); Serial.println(result);
    }

    flash.powerDown();
}

// ─────────────────────────────────────────────────────────────────────────────
// readSend — reads one data record from flash and transmits it over LoRa.
// Used during data download sequences.
// ─────────────────────────────────────────────────────────────────────────────

void readSend(unsigned long address) {
    data dat;

    flash.powerUp();
    delay(10);
    uint8_t res = flash.readStruct(address, dat);
    flash.powerDown();

    if (res != FLASH_OK) {
        Serial.print(F("Flash read failed at 0x")); Serial.println(address, HEX);
        return;
    }

    radio.standby();
    state = radio.transmit((uint8_t*)&dat, sizeof(dat));
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("TX failed, code: ")); Serial.println(state);
    }
    radio.sleep();
}

// ─────────────────────────────────────────────────────────────────────────────
// loadFlashMetadata — scans the wear-leveled metadata sector and restores the
// most recently saved writeAdd/readAdd.  Call once on boot.
// ─────────────────────────────────────────────────────────────────────────────

bool loadFlashMetadata() {
    uint32_t sector = flash.getCapacity() - flash.getSectorSize();
    uint32_t slots  = flash.getSectorSize() / FLASH_META_SIZE;
    FlashMeta meta;
    bool found = false;

    for (uint32_t i = 0; i < slots; i++) {
        if (flash.readStruct(sector + i * FLASH_META_SIZE, meta) != FLASH_OK)
            continue;
        if (meta.magic != FLASH_META_MAGIC)
            continue;
        writeAdd = meta.writeAdd;
        readAdd  = meta.readAdd;
        found    = true;  // keep scanning; last valid slot wins
    }

    if (found) {
        if (readAdd > writeAdd) readAdd = 0;  // guard against corrupted pointer
        Serial.print(F("Metadata restored: writeAdd=0x")); Serial.print(writeAdd, HEX);
        Serial.print(F(" readAdd=0x"));        Serial.println(readAdd,  HEX);
    } else {
        writeAdd = readAdd = 0;
        Serial.println(F("No metadata found — starting from address 0"));
    }
    return found;
}

// ─────────────────────────────────────────────────────────────────────────────
// saveFlashMetadata — writes writeAdd/readAdd to the next free slot in the
// metadata sector.  When all slots are consumed the sector is erased and
// writing wraps back to slot 0 (341 writes per erase cycle).
// ─────────────────────────────────────────────────────────────────────────────

void saveFlashMetadata() {
    uint32_t sector   = flash.getCapacity() - flash.getSectorSize();
    uint32_t slots    = flash.getSectorSize() / FLASH_META_SIZE;
    int32_t  freeSlot = -1;
    FlashMeta meta;

    for (uint32_t i = 0; i < slots; i++) {
        flash.readStruct(sector + i * FLASH_META_SIZE, meta);
        if (meta.magic == 0xFFFFFFFFUL) {   // erased flash reads as 0xFF
            freeSlot = (int32_t)i;
            break;
        }
    }

    if (freeSlot < 0) {
        flash.eraseSector(sector);  // all slots used — wrap around
        freeSlot = 0;
    }

    FlashMeta newMeta = { FLASH_META_MAGIC, (uint32_t)writeAdd, (uint32_t)readAdd };
    if (flash.writeStruct(sector + (uint32_t)freeSlot * FLASH_META_SIZE,
                          newMeta, /*verify=*/true) != FLASH_OK) {
        Serial.println(F("Metadata save failed"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ping — transmits a ping packet over LoRa.
//
//   simplePing = true  → reqPing struct (tag + request code); used for
//                        SIMPLE_PING, CALIBRATION_BEGIN/END, status codes
//   simplePing = false → longPing struct with full device status
// ─────────────────────────────────────────────────────────────────────────────

void ping(byte requestCode, bool simplePing) {
    radio.standby();

    if (simplePing) {
        reqPing pkt;
        pkt.tag     = tag;
        pkt.request = requestCode;
        state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
    } else {
        longPing pkt;
        pkt.ta        = tag;
        pkt.cnt       = (uint16_t)(writeAdd / sizeof(data));  // stored record count
        pkt.pid       = pingId;
        pkt.la        = lat;
        pkt.ln        = lng;
        pkt.devtyp    = devType;
        pkt.mortality = accel.isMortality();  // true when sustained inactivity detected
        state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
    }

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Ping TX failed, code: ")); Serial.println(state);
    }
    radio.sleep();
}

// ─────────────────────────────────────────────────────────────────────────────
// receive — listens for incoming LoRa packets for up to `timeout` ms.
//
// Handles:
//   SIMPLE_PING_ACK   → respond with longPing
//   DATA_DOWNLOAD_ALL → stream all flash records
//   DATA_DOWNLOAD_NEW → stream records since last download
//   setttings packet  → apply settings, ACK with SETTINGS_UPDATED
// ─────────────────────────────────────────────────────────────────────────────

void receive(unsigned long timeout) {
    uint8_t  buffer[32];
    size_t   length = sizeof(buffer);
    reqPing  reqPacket;
    setttings settingsPacket;
    size_t   packetLength;

    radio.standby();
    radio.startReceive();

    unsigned long start = millis();
    while (millis() - start < timeout) {
        state = radio.receive(buffer, length);

        if (state == RADIOLIB_ERR_NONE) {
            packetLength = radio.getPacketLength();

            // ── Simple request packet ─────────────────────────────────────────
            if (packetLength == sizeof(reqPacket)) {
                memcpy(&reqPacket, buffer, sizeof(reqPacket));

                if (reqPacket.tag != tag) {
                    radio.startReceive();
                    continue;
                }

                if (reqPacket.request == SIMPLE_PING_ACK) {
                    // Base station acknowledged our ping — send full status
                    ping(0, /*simplePing=*/false);
                    pingId++;
                    radio.standby();
                    radio.startReceive();
                }

                if (reqPacket.request == DATA_DOWNLOAD_ALL) {
                    // Stream every stored record from the beginning.
                    // readSend() manages flash power per record — no outer
                    // powerUp/powerDown needed here.
                    radio.standby();
                    reqPacket.request = DATA_DOWNLOAD_BEGIN;
                    radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));

                    unsigned long addr = 0;
                    while (addr < writeAdd) {
                        readSend(addr);
                        addr += sizeof(data);
                        delay(100);
                    }

                    reqPacket.request = DATA_DOWNLOAD_END;
                    radio.standby();
                    radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
                    radio.sleep();
                    Serial.println(F("Full download complete"));
                    break;
                }

                if (reqPacket.request == DATA_DOWNLOAD_NEW) {
                    // Stream only records written since the last download.
                    // readSend() manages flash power per record.
                    // saveFlashMetadata() needs flash powered up separately
                    // since readSend() leaves the flash in power-down state.
                    radio.standby();
                    reqPacket.request = DATA_DOWNLOAD_BEGIN;
                    radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));

                    unsigned long addr = readAdd;
                    while (addr < writeAdd) {
                        readSend(addr);
                        addr += sizeof(data);
                        delay(100);
                    }
                    readAdd = writeAdd;  // advance read pointer

                    flash.powerUp();
                    delay(10);
                    saveFlashMetadata();
                    flash.powerDown();

                    reqPacket.request = DATA_DOWNLOAD_END;
                    radio.standby();
                    radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
                    radio.sleep();
                    Serial.println(F("New-records download complete"));
                    break;
                }
            }

            // ── Settings packet ───────────────────────────────────────────────
            if (packetLength == sizeof(settingsPacket)) {
                memcpy(&settingsPacket, buffer, sizeof(settingsPacket));

                if (settingsPacket.tag == tag) {
                    if (settingsPacket.gpsFrq  >= 1  && settingsPacket.gpsFrq  <= 1440) gpsFrequency  = settingsPacket.gpsFrq;
                    if (settingsPacket.gpsTout >= 10 && settingsPacket.gpsTout <= 3600) gpsTimeout    = settingsPacket.gpsTout;
                    if (settingsPacket.hdop    >= 1  && settingsPacket.hdop    <= 50  ) gpsHdop       = settingsPacket.hdop;
                    if (settingsPacket.radioFrq>= 1  && settingsPacket.radioFrq<= 1440) radioFrequency = settingsPacket.radioFrq;
                    if (settingsPacket.startHour>= 0 && settingsPacket.startHour<= 23 ) startHour     = settingsPacket.startHour;
                    if (settingsPacket.endHour  >= 0 && settingsPacket.endHour  <= 23 ) endHour       = settingsPacket.endHour;
                    scheduled = settingsPacket.scheduled;
                    calculateTargets();

                    // Acknowledge
                    radio.standby();
                    reqPacket.tag     = tag;
                    reqPacket.request = SETTINGS_UPDATED;
                    radio.transmit((uint8_t*)&reqPacket, sizeof(reqPacket));
                    radio.startReceive();
                    Serial.println(F("Settings updated and ACKed"));
                }
            }

        } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
            Serial.print(F("RX error, code: ")); Serial.println(state);
        }

        delay(10);
    }

    radio.sleep();
}

// ─────────────────────────────────────────────────────────────────────────────
// deviceCalibration — runs on every boot to verify hardware and push status
// codes to the base station.  Waits up to 5 minutes for initial settings.
//
// Sequence:
//   1. CALIBRATION_BEGIN
//   2. Flash JEDEC check           → FLASH_SUCCESS / FLASH_ERROR
//   3. IMU WHO_AM_I check          → ACCELEROMETER_OK / ACCELEROMETER_ERROR
//   4. GPS fix (30-min timeout)    → GPS_SUCCESS / GPS_ERROR  (sets RTC)
//   5. CALIBRATION_END
//   6. RX loop — accept settings and/or FINISH_CALIBRATION
// ─────────────────────────────────────────────────────────────────────────────

void deviceCalibration() {
    Serial.println(F("Starting device calibration"));
    bool fixStatus = false;

    ping(CALIBRATION_BEGIN, true);

    // ── Flash check ───────────────────────────────────────────────────────────
    flash.powerUp();
    delay(10);
    if (flash.getJEDECID() != 0) {
        Serial.println(F("Flash OK"));
        ping(FLASH_SUCCESS, true);
    } else {
        Serial.println(F("Flash not detected"));
        ping(FLASH_ERROR, true);
    }
    flash.powerDown();

    // ── IMU check ─────────────────────────────────────────────────────────────
    // imuReady was set during setup() — no need to call begin() again here.
    if (imuReady) {
        Serial.println(F("IMU OK"));
        ping(ACCELEROMETER_OK, true);
    } else {
        Serial.println(F("IMU not detected"));
        ping(ACCELEROMETER_ERROR, true);
    }

    // ── GPS check (30-min timeout, also sets RTC) ─────────────────────────────
    getlocation(/*setup=*/true, fixStatus);
    ping(fixStatus ? GPS_SUCCESS : GPS_ERROR, true);

    ping(CALIBRATION_END, true);
    Serial.println(F("Calibration checks done — waiting for settings"));

    // ── Wait for initial settings and/or FINISH_CALIBRATION ──────────────────
    uint8_t   buffer[32];
    size_t    length = sizeof(buffer);
    setttings setPacket;
    reqPing   reqPacket;

    radio.standby();
    radio.startReceive();

    unsigned long start = millis();
    while (millis() - start < 300000UL) {  // 5-minute window
        state = radio.receive(buffer, length);

        if (state == RADIOLIB_ERR_NONE) {
            size_t pktLen = radio.getPacketLength();

            if (pktLen == sizeof(setPacket)) {
                memcpy(&setPacket, buffer, sizeof(setPacket));
                if (setPacket.tag == tag) {
                    if (setPacket.gpsFrq  >= 1  && setPacket.gpsFrq  <= 1440) gpsFrequency   = setPacket.gpsFrq;
                    if (setPacket.gpsTout >= 10 && setPacket.gpsTout <= 3600) gpsTimeout     = setPacket.gpsTout;
                    if (setPacket.hdop    >= 1  && setPacket.hdop    <= 50  ) gpsHdop        = setPacket.hdop;
                    if (setPacket.radioFrq>= 1  && setPacket.radioFrq<= 1440) radioFrequency = setPacket.radioFrq;
                    if (setPacket.startHour>= 0 && setPacket.startHour<= 23 ) startHour      = setPacket.startHour;
                    if (setPacket.endHour  >= 0 && setPacket.endHour  <= 23 ) endHour        = setPacket.endHour;
                    scheduled = setPacket.scheduled;
                    calculateTargets();

                    radio.standby();
                    ping(SETTINGS_UPDATED, true);
                    radio.standby();
                    radio.startReceive();
                    Serial.println(F("Initial settings received and applied"));
                }
            }

            if (pktLen == sizeof(reqPacket)) {
                memcpy(&reqPacket, buffer, sizeof(reqPacket));
                if (reqPacket.tag == tag && reqPacket.request == FINISH_CALIBRATION) {
                    Serial.println(F("Calibration finished by base station"));
                    break;
                }
            }
        } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
            Serial.print(F("RX error, code: ")); Serial.println(state);
        }

        delay(10);
    }

    radio.sleep();
    Serial.println(F("Calibration complete"));
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);
    Serial.println(F("Terrestrial firmware starting..."));

    // GPS enable pin — keep GPS off until needed
    pinMode(GPS_EN, OUTPUT);
    digitalWrite(GPS_EN, LOW);

    // ── RTC ──────────────────────────────────────────────────────────────────
    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin(STM32RTC::HOUR_24);

    // ── Radio ─────────────────────────────────────────────────────────────────
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    for (int attempt = 0; attempt < 3; attempt++) {
        state = radio.begin(867.0, 125.0, 12, 5,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 1.6, false);
        if (state == RADIOLIB_ERR_NONE) break;
        Serial.print(F("Radio init failed, retrying... code: ")); Serial.println(state);
        delay(1000);
    }
    if (state != RADIOLIB_ERR_NONE) {
        Serial.println(F("Radio init failed after 3 attempts — resetting"));
        Serial.flush();
        NVIC_SystemReset();
    }
    Serial.println(F("Radio initialised"));
    radio.sleep();

    // ── IMU ──────────────────────────────────────────────────────────────────
    // Initialise once here. readImu() and the loop sampling block rely on
    // imuReady and skip gracefully if the sensor is absent.
    Wire.begin();
    imuReady = imu.begin();
    if (imuReady) {
        imu.disableGyroscope();   // begin() enables gyro at 104 Hz — keep it off
        Serial.println(F("IMU initialised"));
    } else {
        Serial.println(F("IMU init failed"));
    }
    Wire.end();

    // ── Flash ─────────────────────────────────────────────────────────────────
    flashOk = flash.begin();
    if (flashOk) {
        Serial.println(F("Flash initialised"));
        flash.powerUp();
        delay(10);
        loadFlashMetadata();
        flash.powerDown();
    } else {
        Serial.println(F("Flash init failed — data will not be logged"));
    }

    // ── Calibration (runs hardware checks and waits for first settings) ───────
    deviceCalibration();

    // ── Set timer targets from (possibly updated) settings ────────────────────
    calculateTargets();

    // ── Prepare pins for low-power sleep (reduce leakage) ────────────────────
    pinMode(GPS_RX,  INPUT_ANALOG);
    pinMode(GPS_TX,  INPUT_ANALOG);
    pinMode(SDA_PIN, INPUT_ANALOG);
    pinMode(SCL_PIN, INPUT_ANALOG);
    SPI.end();
    Wire.end();

    Serial.println(F("Setup complete — entering sleep"));
    Serial.flush();
    Serial.end();

    // Attach RTC 1-second interrupt and enter initial deep sleep
    rtc.attachSecondsInterrupt(isr);
    LowPower.deepSleep();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop — executes after every RTC wakeup.
//
// On each wakeup we check which counter(s) have reached their target and
// perform the corresponding action, then return to deep sleep.
// ─────────────────────────────────────────────────────────────────────────────

void loop() {

    // ── GPS acquisition ───────────────────────────────────────────────────────
    // The counter is reset after getlocation() returns, so the next
    // acquisition fires gpsCounterTarget seconds after the previous one
    // finishes (not from when it started). At the default 3-minute interval
    // with a 120-second timeout worst case, fixes can be up to 5 minutes
    // apart under poor signal. This is intentional — back-to-back
    // acquisitions are avoided.
    if (gpsCounter >= gpsCounterTarget) {
        bool fix = false;
        Serial.begin(115200);

        getlocation(/*setup=*/false, fix);

        noInterrupts();
        gpsCounter = 0;
        interrupts();

        Serial.flush();
        Serial.end();
    }

    // ── Radio ping ────────────────────────────────────────────────────────────
    // In main mode (scheduled=false): always ping when the counter fires.
    // In scheduled mode (scheduled=true): only ping within the configured
    // window. The counter is always reset so the interval stays consistent
    // and pings don't bunch up when the window re-opens.
    if (pingCounter >= pingCounterTarget) {
        noInterrupts();
        pingCounter = 0;
        interrupts();

        if (!scheduled || isInScheduledWindow()) {
            Serial.begin(115200);

            SPI.begin();   // SPI needed if a data download is requested
            radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
            radio.standby();

            ping(SIMPLE_PING, true);   // send simple ping
            receive(5000);             // listen 5 s for ACK / commands

            SPI.end();
            Serial.flush();
            Serial.end();
        }
    }

    LowPower.deepSleep();
}
