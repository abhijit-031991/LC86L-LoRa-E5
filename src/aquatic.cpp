/*
 * aquatic.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Firmware for the LC86L-LoRa-E5 — Aquatic Variant
 *
 * Hardware:
 *   STM32WLE5JC   Internal sub-GHz LoRa radio
 *   MPR121        Capacitive touch sensor on I2C (SDA=PA15, SCL=PB15)
 *                 Electrodes 0–3 connected; ~IRQ line on PB4
 *
 * Radio config:  867 MHz | SF12 | BW 125 kHz | CR 4/5 | 22 dBm
 *
 * Flash:  pio run -e aquatic -t upload
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * Surfacing logic
 * ─────────────────────────────────────────────────────────────────────────────
 *   All four electrodes wet  →  SUBMERGED  (no action)
 *   Any electrode dry        →  SURFACED   →  transmit SURFACED_OK × 3
 *
 * Wake sources (STOP1):
 *   1. MPR121 ~IRQ on PB4 (EXTI, FALLING) — electrode state changed
 *   2. RTC 1-second tick — increments a watchdog counter; after 60 minutes
 *      with no MPR121 activity the sensor is soft-reset and reconfigured
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * Boot sequence
 * ─────────────────────────────────────────────────────────────────────────────
 *   1. Initialise RTC (LSE clock — drives the 1-second watchdog tick)
 *   2. Initialise radio (same parameters as terrestrial variant)
 *   3. Transmit "Hello, world!" handshake
 *   4. Initialise MPR121 — hard-reset on failure
 *   5. LoRa calibration report:  CALIBRATION_BEGIN → CAPACITANCE_OK/ERROR
 *                                 → CALIBRATION_END
 *      (no settings window — aquatic device needs no user input)
 *   6. Sample initial surface state (no ping sent)
 *   7. Attach EXTI on PB4; attach RTC 1-second interrupt
 *   8. Power down I2C bus; enter STOP1 sleep
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPR121_LoRaE5.h>
#include <STM32Stop1.h>
#include <STM32RTC.h>
#include <RadioLib.h>
#include <codes.h>
#include <definitions.h>

// ─────────────────────────────────────────────────────────────────────────────
// Object instances
// ─────────────────────────────────────────────────────────────────────────────

MPR121_LoRaE5 mpr;
STM32Stop1    stop1;
STM32RTC&     rtc   = STM32RTC::getInstance();
STM32WLx      radio = new STM32WLx_Module();

// ─────────────────────────────────────────────────────────────────────────────
// RF switch table — identical to the terrestrial variant
// PA4 = RX enable, PA5 = TX HP enable
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
// State
// ─────────────────────────────────────────────────────────────────────────────

volatile bool          mprInterruptFired = false;
volatile unsigned long watchdogCounter   = 0;   // seconds since last MPR121 event
bool                   submerged         = false;
int                    state             = RADIOLIB_ERR_NONE;

#define WATCHDOG_INTERVAL_S  3600UL   // 60 minutes

// ─────────────────────────────────────────────────────────────────────────────
// ISRs
// ─────────────────────────────────────────────────────────────────────────────

void mprISR() {
    mprInterruptFired = true;
}

// RTC fires every second — used only for the 60-minute watchdog counter
void rtcISR(void* /*unused*/) {
    watchdogCounter++;
}

// ─────────────────────────────────────────────────────────────────────────────
// configureMPR121 — apply the standard electrode configuration.
// Used at boot and after a watchdog soft-reset.
// Caller must ensure Wire is initialised before calling.
// ─────────────────────────────────────────────────────────────────────────────

void configureMPR121() {
    mpr.stopMode();
    mpr.setThresholds(12, 6);
    mpr.setSampleIntervalMs(128);
    mpr.setCDC(0x10);
    mpr.runMode(4);  // enable electrodes 0–3
}

// ─────────────────────────────────────────────────────────────────────────────
// readSurfaceState — reads touch-status registers (clears ~IRQ) and returns
// true when surfaced (any electrode dry).
// Caller must ensure Wire is initialised before calling.
// ─────────────────────────────────────────────────────────────────────────────

bool readSurfaceState() {
    uint16_t mask = mpr.handleIRQ_and_getTouchMask();
    bool e0 = (mask >> 0) & 1;
    bool e1 = (mask >> 1) & 1;
    bool e2 = (mask >> 2) & 1;
    bool e3 = (mask >> 3) & 1;
    return !(e0 && e1 && e2 && e3);  // all wet → submerged; any dry → surfaced
}

// ─────────────────────────────────────────────────────────────────────────────
// sendPing — transmit a simple 3-byte reqPing packet over LoRa.
// Restores the RF switch table before transmitting (required after STOP1).
// ─────────────────────────────────────────────────────────────────────────────

void sendPing(byte requestCode) {
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    radio.standby();
    reqPing pkt;
    pkt.tag     = tag;
    pkt.request = requestCode;
    state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
    radio.sleep();
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);
    Serial.print("Device ID : "); Serial.println(tag);
    Serial.println(F("Aquatic firmware starting..."));

    // ── RTC (LSE clock — drives the 1-second watchdog tick) ──────────────────
    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin(STM32RTC::HOUR_24);

    // ── Radio (exact same parameters as the terrestrial variant) ─────────────
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

    state = radio.transmit("Hello, world!");
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("Radio TX test failed, code: ")); Serial.println(state);
    } else {
        Serial.println(F("Radio TX test OK"));
    }
    radio.sleep();

    // ── MPR121 ───────────────────────────────────────────────────────────────
    Wire.begin();
    bool mprStatus = mpr.begin();
    if (!mprStatus) {
        Serial.println(F("MPR121 not found — resetting"));
        Serial.flush();
        NVIC_SystemReset();
    }
    configureMPR121();
    Serial.println(F("MPR121 initialised, electrodes 0-3 enabled"));

    // ── LoRa calibration report (no user-input wait) ──────────────────────────
    sendPing(CALIBRATION_BEGIN);
    sendPing(mprStatus ? CAPACITANCE_OK : CAPACITANCE_ERROR);
    sendPing(CALIBRATION_END);

    // ── Sample initial surface state (baseline — no ping sent at boot) ────────
    bool surfaced = readSurfaceState();
    submerged = !surfaced;
    Serial.print(F("Initial state: "));
    Serial.println(submerged ? F("SUBMERGED") : F("SURFACED"));

    // ── Attach EXTI on PB4 (MPR121 ~IRQ, active LOW) ─────────────────────────
    pinMode(ACC_INT1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ACC_INT1_PIN), mprISR, FALLING);

    // ── Attach RTC 1-second interrupt for the 60-minute watchdog ─────────────
    rtc.attachSecondsInterrupt(rtcISR);

    // ── Power down I2C bus to reduce leakage current ──────────────────────────
    Wire.end();
    pinMode(SDA_PIN, INPUT_ANALOG);
    pinMode(SCL_PIN, INPUT_ANALOG);

    Serial.println(F("Setup complete — entering STOP1 sleep"));
    Serial.flush();
    Serial.end();

    stop1.sleep();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop — entered on every STOP1 wakeup.
//
// Two wake paths:
//   mprInterruptFired  MPR121 ~IRQ fired (electrode state changed)
//   watchdogCounter    RTC 1-second tick; fires watchdog at 3600 s
// ─────────────────────────────────────────────────────────────────────────────

void loop() {

    // ── MPR121 interrupt ──────────────────────────────────────────────────────
    if (mprInterruptFired) {
        mprInterruptFired = false;

        // Reset watchdog — sensor is responding, no reset needed
        noInterrupts();
        watchdogCounter = 0;
        interrupts();

        Wire.begin();
        bool surfaced = readSurfaceState();

        if (surfaced && submerged) {
            // Transition: submerged → surfaced — send three successive pings
            sendPing(SURFACED_OK);
            sendPing(SURFACED_OK);
            sendPing(SURFACED_OK);
        }
        submerged = !surfaced;

        Wire.end();
        pinMode(SDA_PIN, INPUT_ANALOG);
        pinMode(SCL_PIN, INPUT_ANALOG);
    }

    // ── 60-minute watchdog ────────────────────────────────────────────────────
    if (watchdogCounter >= WATCHDOG_INTERVAL_S) {
        noInterrupts();
        watchdogCounter = 0;
        interrupts();

        Serial.begin(115200);
        Serial.println(F("Watchdog: no MPR121 activity for 60 min — soft resetting sensor"));

        // Soft-reset and reconfigure MPR121 to recover from any sensor lockup
        Wire.begin();
        mpr.softReset();
        delay(10);
        configureMPR121();
        // Read status once to clear any IRQ line left asserted after the reset
        submerged = !readSurfaceState();
        Serial.print(F("MPR121 reset complete. State: "));
        Serial.println(submerged ? F("SUBMERGED") : F("SURFACED"));
        Wire.end();
        pinMode(SDA_PIN, INPUT_ANALOG);
        pinMode(SCL_PIN, INPUT_ANALOG);

        Serial.flush();
        Serial.end();
    }

    stop1.sleep();
}
