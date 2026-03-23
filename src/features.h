/**
 * features.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Hardware feature flags for the LC86L-LoRa-E5 firmware.
 *
 * The active variant is selected by a -DVARIANT_xxx build flag set in
 * platformio.ini. Never edit this file to switch variants — change the
 * PlatformIO environment instead.
 *
 *  pio run -e lora_e5       -t upload   <- Full hardware (all peripherals)
 *  pio run -e lora_e5_v1    -t upload   <- Variant 1: No MPR121
 *  pio run -e lora_e5_v2    -t upload   <- Variant 2: Radio + MPR121 only
 *  pio run -e lora_e5_v3    -t upload   <- Variant 3: GPS + Flash only (no IMU, no MPR121)
 *  pio run -e lora_e5_test  -t upload   <- Peripheral test firmware
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * Feature flags (defined below, DO NOT define manually):
 *
 *   FEATURE_GPS      – LC86L GPS module  (LPUART, GPS_EN pin)
 *   FEATURE_FLASH    – SPI NOR Flash     (data logging, metadata)
 *   FEATURE_IMU      – LSM6DSL IMU       (accelerometer / gyroscope)
 *   FEATURE_MPR121   – MPR121            (capacitive touch / submersion)
 *
 * Radio (STM32WLx) and RTC are always compiled — they are the core system.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef FEATURES_H
#define FEATURES_H

// ── Default to full hardware if no variant flag is passed ───────────────────
#if !defined(VARIANT_FULL) && !defined(VARIANT_V1) && \
    !defined(VARIANT_V2)   && !defined(VARIANT_V3)
    #define VARIANT_FULL
#endif

// ── VARIANT_FULL  :  All peripherals ────────────────────────────────────────
#ifdef VARIANT_FULL
    #define FEATURE_GPS
    #define FEATURE_FLASH
    #define FEATURE_IMU
    #define FEATURE_MPR121
#endif

// ── VARIANT_V1  :  No MPR121 (no capacitive touch / submersion detection) ───
#ifdef VARIANT_V1
    #define FEATURE_GPS
    #define FEATURE_FLASH
    #define FEATURE_IMU
    // FEATURE_MPR121  intentionally omitted
#endif

// ── VARIANT_V2  :  Radio + MPR121 only (no GPS, no Flash, no IMU) ───────────
#ifdef VARIANT_V2
    #define FEATURE_MPR121
    // FEATURE_GPS, FEATURE_FLASH, FEATURE_IMU  intentionally omitted
#endif

// ── VARIANT_V3  :  GPS + Flash only (no IMU, no MPR121) ─────────────────────
#ifdef VARIANT_V3
    #define FEATURE_GPS
    #define FEATURE_FLASH
    // FEATURE_IMU, FEATURE_MPR121  intentionally omitted
#endif

#endif // FEATURES_H
