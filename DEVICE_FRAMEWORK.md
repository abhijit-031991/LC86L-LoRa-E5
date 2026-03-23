# LC86L-LoRa-E5 Device Framework
> Firmware version 1.0 — STM32WL embedded tracker

---

## 1. Overview

The LC86L-LoRa-E5 is a low-power wildlife/asset tracking device. It periodically acquires GPS
fixes, logs them to on-board SPI Flash, and relays them to a base station over LoRa. A
capacitive-touch sensor array (MPR121) detects submersion. The device spends the vast majority
of its life in deep sleep, waking only on a 1-second RTC tick.

---

## 2. Hardware Components

```
+--------------------------------------------------+
|               STM32WL (MCU + LoRa)               |
|                                                  |
|  LPUART ──► LC86L GPS module  (PC0/PC1, PB5 EN) |
|  I2C    ──► LSM6DSL IMU        (PA15 SDA, PB15) |
|  I2C    ──► MPR121 Cap-touch   (PA0 INT)         |
|  SPI    ──► SPI Flash          (PB9 CS, 8 MHz)   |
|  RFSwitch──► PA4/PA5 (RX/TX_HP)                 |
+--------------------------------------------------+
```

| Component     | Interface | Purpose                                  |
|---------------|-----------|------------------------------------------|
| LC86L GPS     | LPUART    | Position + time acquisition              |
| LSM6DSL IMU   | I2C       | Accelerometer / gyroscope (reserved)     |
| MPR121        | I2C       | 4-electrode submersion detection         |
| SPI Flash     | SPI       | Non-volatile GPS record storage          |
| STM32WLx LoRa | Internal  | Bidirectional RF comms at 867 MHz        |
| STM32 RTC     | LSE clock | 1-second tick that drives all timers     |

---

## 3. Firmware Architecture

```
setup()
  │
  ├─ Serial / GPIO init
  ├─ RTC init (LSE, 24h)
  ├─ Radio init + test TX  ──[fail]──► NVIC_SystemReset()
  ├─ SPI Flash init
  │     └─ loadFlashMetadata()   ← restore writeAdd / readAdd
  ├─ MPR121 init           ──[fail]──► NVIC_SystemReset()
  ├─ deviceCalibration()
  │     ├─ ping CALIBRATION_BEGIN
  │     ├─ flash diagnostics
  │     ├─ MPR121 check
  │     ├─ getlocation(setup=true)  ← extended 30-min GPS + RTC sync
  │     ├─ ping GPS result
  │     └─ wait for FINISH_CALIBRATION + settings packet (5-min window)
  ├─ calculateTargets()    ← set gpsCounterTarget, pingCounterTarget
  ├─ Power down peripherals (Wire, SPI, Serial)
  ├─ RTC.attachSecondsInterrupt(isr)
  └─ LowPower.deepSleep()
         │
         ▼
loop()  ◄────────────────────────── woken by RTC every second
  │
  ├─ [capCounter >= 10s]  → checkElectrodes()    → capCounter = 0
  ├─ [gpsCounter >= gpsFrequency×60s]  → getlocation()  → gpsCounter = 0
  ├─ [pingCounter >= radioFrequency×60s] → ping() + receive() → pingCounter = 0
  └─ LowPower.deepSleep()
```

---

## 4. Sleep / Wake Cycle

Every second the RTC fires `isr()`, which atomically increments three software counters:

```
 gpsCounter   → triggers GPS acquisition when >= gpsFrequency  (minutes × 60)
 pingCounter  → triggers LoRa ping        when >= radioFrequency (minutes × 60)
 capCounter   → triggers submersion check when >= 10 (seconds)
```

The device is asleep between ticks. Peripherals are powered off between uses:

```
  SLEEP  ──RTC tick──►  isr() increments counters
                         │
                         └──► loop() checks thresholds
                                   ├─ powers up needed peripheral
                                   ├─ does work
                                   ├─ powers peripheral back down
                                   └─ returns to deepSleep()
```

> If `submerged == true`, GPS and radio tasks are skipped. Only the capacitive check
> keeps running so the device can detect when it surfaces again.

---

## 5. GPS Acquisition Flow (`getlocation()`)

```
getlocation(setup, &GF)
│
├─ Power GPS on (GPS_EN HIGH), open LPUART @ 9600
├─ Timeout = 30 min (setup) or gpsTimeout seconds (normal)
│
├─ Parse NMEA stream via TinyGPS++
│     Wait for: location valid, age < 1s, HDOP < gpsHdop threshold, >3s elapsed
│
├─ GPS_EN LOW, LPUART close
│
├─[No fix] → GF = false, return
│
├─[Setup + fix] → setTime() + rtc.setEpoch()   ← sync RTC to GPS
│
└─[Normal + fix]
      ├─ Build `data` struct:
      │     datetime  = GPS time or RTC fallback
      │     lat / lng = GPS coords
      │     hdop      = GPS HDOP
      │     locktime  = elapsed ms (capped to uint16_t max)
      │     count     = incremental record number
      │     id        = device tag
      │
      ├─ Flash write sequence:
      │     flash.powerUp()
      │     Capacity guard: writeAdd + sizeof(dat) <= metaSector?
      │     Sector boundary check (Case 1): erase if writeAdd % sectorSize == 0
      │     Cross-sector check (Case 2): erase next sector if write spans boundary
      │     flash.writeStruct(writeAdd, dat, verify=true)
      │     Verify read-back
      │     writeAdd += sizeof(dat)
      │     saveFlashMetadata()        ← persist pointers to NVM
      │     flash.powerDown()
      │
      └─ pingId = 0   ← reset after new data session
```

---

## 6. LoRa Ping / Receive Flow

### 6a. Outbound ping

```
ping(requestCode, simplePing)
│
├─[simplePing = true]
│     TX: reqPing { tag, requestCode }
│         codes: SIMPLE_PING, REQUEST_SETTINGS, CALIBRATION_BEGIN/END, …
│
└─[simplePing = false]
      TX: longPing { tag, recordCount, pingId, lat, lng, devType, mortality }
```

### 6b. Receive window (5-second listen after each ping)

```
receive(5000ms)
│
├─ radio.startReceive()
│
├─ On packet received:
│     ├─[sizeof == reqPing]
│     │     ├─[SIMPLE_PING_ACK]  → TX longPing (full status)
│     │     ├─[DATA_DOWNLOAD_ALL] → TX DATA_DOWNLOAD_BEGIN
│     │     │                       loop: readSend(0 → writeAdd)
│     │     │                       TX DATA_DOWNLOAD_END
│     │     └─[DATA_DOWNLOAD_NEW] → TX DATA_DOWNLOAD_BEGIN
│     │                             loop: readSend(readAdd → writeAdd)
│     │                             readAdd = writeAdd
│     │                             TX DATA_DOWNLOAD_END
│     │
│     └─[sizeof == setttings]
│           Validate all fields (range-checked)
│           Apply: gpsFrequency, gpsTimeout, gpsHdop, radioFrequency,
│                  scheduled, startHour, endHour
│           calculateTargets()
│           TX SETTINGS_UPDATED ACK
│
└─ radio.sleep()
```

---

## 7. Submersion Detection Flow

```
checkElectrodes()   (runs every 10 seconds)
│
├─ Read electrodes 0–3 via MPR121
├─ [all 4 touched] → submerged = true   ("SUBMERGED")
└─ [any not touched] → submerged = false ("NOT submerged")

Effect on main loop:
  submerged == true  → GPS and radio tasks SUSPENDED
  submerged == false → normal operation resumes
```

---

## 8. Flash Storage Layout

```
 Address 0x000000
 ┌──────────────────────────────────┐
 │  data records                    │  36 bytes each (packed struct)
 │  [record 0][record 1]...[record N]│  written sequentially
 │                                  │
 │  writeAdd advances here ──►      │
 │                                  │
 ├──────────────────────────────────┤  ← metaSector = capacity - 4096
 │  Metadata sector (4 KB)          │
 │  341 wear-leveled FlashMeta slots│  12 bytes each
 │  { magic, writeAdd, readAdd }    │  written on every successful record
 └──────────────────────────────────┘
  Address (capacity)
```

**Metadata persistence:**
- On boot: `loadFlashMetadata()` scans all 341 slots, restores the last valid `writeAdd`/`readAdd`
- After every write: `saveFlashMetadata()` appends to the next free slot (wraps + erases when full)
- Protects against data loss across power cycles and deep-sleep resets

**Sector erase strategy:**
- Case 1: write address falls on a 4 KB boundary → erase that sector before writing
- Case 2: write spans into the next sector → also erase the next sector
- This prevents `FLASH_ERROR_VERIFY_FAILED` at sector crossing boundaries

---

## 9. Calibration Flow (first boot)

```
deviceCalibration()
│
├─ ping(CALIBRATION_BEGIN)
├─ Flash check   → ping FLASH_SUCCESS / FLASH_ERROR
├─ MPR121 check  → ping CAPACITANCE_OK / CAPACITANCE_ERROR
├─ getlocation(setup=true)  → ping GPS_SUCCESS / GPS_ERROR  (RTC synced here)
├─ ping(CALIBRATION_END)
│
└─ Listen up to 5 minutes for:
      [setttings packet]  → apply validated settings, ACK with SETTINGS_UPDATED
      [FINISH_CALIBRATION] → exit calibration window
```

After calibration the device enters the normal sleep/wake cycle.

---

## 10. Data Record Structure

```cpp
struct data {                   // 36 bytes, packed
    uint32_t datetime;          // Unix timestamp (GPS or RTC)
    uint16_t locktime;          // Seconds to first GPS fix (0–65535)
    float    lat;               // Latitude  (degrees)
    float    lng;               // Longitude (degrees)
    float    hdop;              // GPS HDOP quality indicator
    float    x;                 // Accelerometer X (reserved, currently 0)
    float    y;                 // Accelerometer Y (reserved, currently 0)
    float    z;                 // Accelerometer Z (reserved, currently 0)
    unsigned int count;         // Sequential record counter
    uint16_t id;                // Device tag ID (22222)
};
```

---

## 11. Settings Parameters

| Parameter      | Default | Valid Range      | Effect                              |
|----------------|---------|------------------|-------------------------------------|
| gpsFrequency   | 3 min   | 1 – 1440 min     | How often GPS runs                  |
| gpsTimeout     | 120 s   | 10 – 3600 s      | Max time to wait for a GPS fix      |
| gpsHdop        | 5       | 1 – 50           | Minimum fix quality (lower = better)|
| radioFrequency | 1 min   | 1 – 1440 min     | How often a ping is sent            |
| scheduled      | false   | bool             | Enable time-window operation        |
| startHour      | 0       | 0 – 23           | Active window start (scheduled mode)|
| endHour        | 23      | 0 – 23           | Active window end   (scheduled mode)|

Settings are applied only if all values pass range validation. Out-of-range fields are silently ignored and the current value is kept.

---

## 12. LoRa Radio Configuration

| Parameter     | Value                              |
|---------------|------------------------------------|
| Frequency     | 867.0 MHz                          |
| Bandwidth     | 125.0 kHz                          |
| Spreading F.  | SF12                               |
| Coding Rate   | 4/5                                |
| Sync Word     | RADIOLIB_SX126X_SYNC_WORD_PRIVATE  |
| Output power  | 22 dBm (TX_HP path via PA5)        |
| Preamble      | 8 symbols                          |
| TCXO voltage  | 1.6 V                              |

---

## 13. Error Recovery

| Fault                     | Behaviour                                     |
|---------------------------|-----------------------------------------------|
| Radio init fails (3 retries) | `NVIC_SystemReset()` — device reboots       |
| MPR121 not found          | `NVIC_SystemReset()` — device reboots         |
| GPS timeout               | Record stored with lat/lng = 0, GF = false    |
| Flash write fails         | Error logged to Serial, writeAdd not advanced |
| Flash full                | Write skipped, Serial warning, device continues |
| Metadata save fails       | Serial warning, operation continues           |
| Malformed LoRa packet     | Ignored (size check + tag check)              |
| Invalid settings packet   | Out-of-range fields silently rejected         |
