/*
 * radio-listener.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Receive-only radio monitor for the LC86L-LoRa-E5.
 * Listens on the same channel as the production firmware and prints every
 * incoming packet to Serial, decoded where the length matches a known struct.
 *
 * Flash:  pio run -e radio-listener -t upload
 *
 * Radio config (must match transmitting devices exactly):
 *   867.0 MHz | SF12 | BW 125 kHz | CR 4/5 | Sync 0x12 (private)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * Decoded packet types  (matched by payload length)
 * ─────────────────────────────────────────────────────────────────────────────
 *   3 bytes   reqPing    tag, request code + human-readable code name
 *  16 bytes   longPing   tag, count, packet ID, lat, lng, devType, mortality
 *  36 bytes   data       datetime, locktime, lat, lng, hdop, x, y, z, count, id
 *   other     raw hex dump
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <RadioLib.h>
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
// Radio object
// ─────────────────────────────────────────────────────────────────────────────

STM32WLx radio = new STM32WLx_Module();

// ─────────────────────────────────────────────────────────────────────────────
// RX interrupt flag
// ─────────────────────────────────────────────────────────────────────────────

static volatile bool rxFlag = false;
static void onRx() { rxFlag = true; }

// ─────────────────────────────────────────────────────────────────────────────
// Request code name lookup
// ─────────────────────────────────────────────────────────────────────────────

static const char* requestName(byte code) {
    switch (code) {
        case CALIBRATION_BEGIN:       return "CALIBRATION_BEGIN";
        case CALIBRATION_END:         return "CALIBRATION_END";
        case FINISH_CALIBRATION:      return "FINISH_CALIBRATION";
        case GPS_CALIBRATION:         return "GPS_CALIBRATION";
        case GPS_CALIBRATION_END:     return "GPS_CALIBRATION_END";
        case GPS_ERROR:               return "GPS_ERROR";
        case GPS_SUCCESS:             return "GPS_SUCCESS";
        case FLASH_DIAGNOSTICS:       return "FLASH_DIAGNOSTICS";
        case FLASH_DIAGNOSTICS_END:   return "FLASH_DIAGNOSTICS_END";
        case FLASH_ERROR:             return "FLASH_ERROR";
        case FLASH_SUCCESS:           return "FLASH_SUCCESS";
        case SIMPLE_PING:             return "SIMPLE_PING";
        case SIMPLE_PING_ACK:         return "SIMPLE_PING_ACK";
        case REQUEST_SETTINGS:        return "REQUEST_SETTINGS";
        case SETTINGS_UPDATED:        return "SETTINGS_UPDATED";
        case DATA_DOWNLOAD_NEW:       return "DATA_DOWNLOAD_NEW";
        case DATA_DOWNLOAD_ALL:       return "DATA_DOWNLOAD_ALL";
        case DATA_DOWNLOAD_END:       return "DATA_DOWNLOAD_END";
        case DATA_DOWNLOAD_BEGIN:     return "DATA_DOWNLOAD_BEGIN";
        case MEMORY_FULL:             return "MEMORY_FULL";
        case MEMORY_ERROR:            return "MEMORY_ERROR";
        case MEMORY_CLEAR:            return "MEMORY_CLEAR";
        case MEMORY_CLEARED:          return "MEMORY_CLEARED";
        case ACCELEROMETER_OK:        return "ACCELEROMETER_OK";
        case ACCELEROMETER_ERROR:     return "ACCELEROMETER_ERROR";
        case CAPACITANCE_OK:          return "CAPACITANCE_OK";
        case CAPACITANCE_ERROR:       return "CAPACITANCE_ERROR";
        case SURFACED_OK:             return "SURFACED_OK";
        case ARGOS_OK:                return "ARGOS_OK";
        case ARGOS_TEST:              return "ARGOS_TEST";
        case ARGOS_ERROR:             return "ARGOS_ERROR";
        default:                      return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Packet decoders
// ─────────────────────────────────────────────────────────────────────────────

static void decodeReqPing(const uint8_t* buf) {
    reqPing pkt;
    memcpy(&pkt, buf, sizeof(pkt));
    Serial.println(F("  Type    : reqPing"));
    Serial.print(F("  Tag     : ")); Serial.println(pkt.tag);
    Serial.print(F("  Request : "));
    Serial.print(pkt.request);
    Serial.print(F(" ("));
    Serial.print(requestName(pkt.request));
    Serial.println(F(")"));
}

static void decodeLongPing(const uint8_t* buf) {
    longPing pkt;
    memcpy(&pkt, buf, sizeof(pkt));
    Serial.println(F("  Type      : longPing"));
    Serial.print(F("  Tag       : ")); Serial.println(pkt.ta);
    Serial.print(F("  Count     : ")); Serial.println(pkt.cnt);
    Serial.print(F("  Packet ID : ")); Serial.println(pkt.pid);
    Serial.print(F("  Latitude  : ")); Serial.println(pkt.la, 6);
    Serial.print(F("  Longitude : ")); Serial.println(pkt.ln, 6);
    Serial.print(F("  Dev type  : ")); Serial.println(pkt.devtyp);
    Serial.print(F("  Mortality : ")); Serial.println(pkt.mortality ? F("YES") : F("no"));
}

static void decodeData(const uint8_t* buf) {
    data pkt;
    memcpy(&pkt, buf, sizeof(pkt));
    Serial.println(F("  Type      : data"));
    Serial.print(F("  Datetime  : ")); Serial.println(pkt.datetime);
    Serial.print(F("  Lock time : ")); Serial.print(pkt.locktime); Serial.println(F(" s"));
    Serial.print(F("  Latitude  : ")); Serial.println(pkt.lat, 6);
    Serial.print(F("  Longitude : ")); Serial.println(pkt.lng, 6);
    Serial.print(F("  HDOP      : ")); Serial.println(pkt.hdop, 2);
    Serial.print(F("  Accel X   : ")); Serial.println(pkt.x, 4);
    Serial.print(F("  Accel Y   : ")); Serial.println(pkt.y, 4);
    Serial.print(F("  Accel Z   : ")); Serial.println(pkt.z, 4);
    Serial.print(F("  Count     : ")); Serial.println(pkt.count);
    Serial.print(F("  Packet ID : ")); Serial.println(pkt.id);
}

static void hexDump(const uint8_t* buf, size_t len) {
    Serial.println(F("  Type      : unknown — raw hex"));
    Serial.print(F("  Bytes     : "));
    for (size_t i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX);
        Serial.print(' ');
        if ((i + 1) % 16 == 0 && i + 1 < len) {
            Serial.println();
            Serial.print(F("              "));
        }
    }
    Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("┌─────────────────────────────────────────┐"));
    Serial.println(F("│  LoRa-E5 Radio Listener                 │"));
    Serial.println(F("│  867 MHz | SF12 | BW125 | CR4/5         │"));
    Serial.println(F("└─────────────────────────────────────────┘"));

    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    int state = radio.begin(867.0, 125.0, 12, 5,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                            22, 8, 1.6, false);

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Radio init failed, code: ")); Serial.println(state);
        Serial.flush();
        NVIC_SystemReset();
    }
    Serial.println(F("Radio OK — listening...\n"));

    radio.setDio1Action(onRx);
    radio.startReceive();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
    if (!rxFlag) return;
    rxFlag = false;

    size_t len = radio.getPacketLength();
    uint8_t buf[256];
    int state = radio.readData(buf, len);

    // Restart RX immediately so no packets are missed while we print
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("readData() error: ")); Serial.println(state);
        return;
    }

    // ── Header ────────────────────────────────────────────────────────────────
    Serial.println(F("──────────────────────────────────────────────────"));
    Serial.print(F("  Length    : ")); Serial.print(len); Serial.println(F(" bytes"));
    Serial.print(F("  RSSI      : ")); Serial.print(radio.getRSSI()); Serial.println(F(" dBm"));
    Serial.print(F("  SNR       : ")); Serial.print(radio.getSNR());  Serial.println(F(" dB"));

    // ── Decode by length ──────────────────────────────────────────────────────
    if      (len == sizeof(reqPing))  decodeReqPing(buf);
    else if (len == sizeof(longPing)) decodeLongPing(buf);
    else if (len == sizeof(data))     decodeData(buf);
    else                              hexDump(buf, len);
}
