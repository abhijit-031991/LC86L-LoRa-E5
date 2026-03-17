# LoRaE5_SPIFlash — Library Fixes & Implementation Migration Guide

This document covers every change made to the `LoRaE5_SPIFlash` library and every
change required in the application firmware (`main.cpp` or equivalent) when porting
to another device that stores a different struct to SPI flash.

---

## Part 1 — Library Changes (`lib/LoRaE5_SPIFlash/`)

The library files are **drop-in replacements**. Copy both files into the new project
`lib/LoRaE5_SPIFlash/` folder. No edits are required to the library itself.

### What was fixed and why

---

#### FIX 1 — Erase operations now return `FLASH_ERROR_TIMEOUT`
**Functions:** `eraseSector()`, `eraseBlock32K()`, `eraseBlock64K()`, `eraseChip()`

**Before:** `waitForReady()` was called but its return value was silently discarded.
If the flash chip stalled (e.g. after a power glitch), the function returned `FLASH_OK`
even though the sector was never actually erased. Any write that followed would
fail with a verify error.

**After:** All four erase functions check the return value of `waitForReady()` and
return `FLASH_ERROR_TIMEOUT` if it expires.

```cpp
// After the SPI erase command:
if (!waitForReady(10000)) return FLASH_ERROR_TIMEOUT;
return FLASH_OK;
```

---

#### FIX 2 — `writePage()` rejects page-boundary-crossing writes
**Function:** `writePage()`

**Before:** The W25Q64 `PAGE_PROGRAM` command silently wraps its write pointer when
it reaches the end of a 256-byte page. Bytes that overflowed would wrap back to the
start of the same page, silently corrupting up to 255 bytes of previously written data.

**After:** `writePage()` calculates the offset within the page and rejects the call if
`pageOffset + length > pageSize`. `writeArray()` always splits correctly before calling
`writePage()`, so this guard only fires if `writePage()` is called directly with bad args.

```cpp
uint32_t pageOffset = address % _chipInfo.pageSize;
if ((uint32_t)pageOffset + (uint32_t)length > (uint32_t)_chipInfo.pageSize) {
    return FLASH_ERROR_INVALID_ADDRESS;
}
```

---

#### FIX 3 — `isValidRange()` is overflow-safe
**Function:** `isValidRange()`

**Before:** The check `address + length <= capacity` overflows to near-zero for very
large values of `length`, allowing any address to pass validation.

**After:** Rewritten as `length <= (capacity - address)`, which uses only subtraction
and never overflows for valid `uint32_t` inputs.

```cpp
return (address < _chipInfo.capacity) &&
       (length <= (_chipInfo.capacity - address));
```

---

#### FIX 4 — `setChipInfo()` guards against undefined behaviour on unknown chips
**Function:** `setChipInfo()`

**Before:** Capacity for unknown chips was inferred with `capacity = (uint32_t)1 << capacityCode`.
A corrupt or noisy bus can produce a `capacityCode` of `0xFF`. Left-shifting a 32-bit
integer by 32 or more bits is undefined behaviour in C++.

**After:** The shift is only performed when `capacityCode` is in the range 1..31.
All other values leave `capacity = 0`, which `identifyChip()` then rejects.

```cpp
if (capacityCode >= 1 && capacityCode <= 31) {
    capacity = (uint32_t)1 << capacityCode;
}
```

---

#### FIX 5 — `identifyChip()` validates each JEDEC byte individually
**Function:** `identifyChip()`

**Before:** Only an all-zeros or all-ones 24-bit JEDEC ID was rejected. A partial-clock
response can produce a value like `0xEF00FF` that passes those guards but is clearly invalid.

**After:** The manufacturer byte and memory-type byte are each checked independently.
Either being `0x00` or `0xFF` causes the function to return `false`.

```cpp
uint8_t mfr     = (jedecId >> 16) & 0xFF;
uint8_t memType = (jedecId >>  8) & 0xFF;
if (mfr == 0x00 || mfr == 0xFF)         return false;
if (memType == 0x00 || memType == 0xFF)  return false;
```

---

#### FIX 6 — `powerUp()` waits for tRES1 after wakeup
**Function:** `powerUp()`

**Before:** The Release-Power-Down command was issued with no delay afterwards. The
W25Q64 datasheet specifies a minimum tRES1 of 3 us between CS de-assertion and the
first command. Violating this causes the first SPI transaction to be mis-clocked.

**After:** A 5 us delay (safely above the 3 us minimum) is added after the transaction.

```cpp
void LoRaE5_SPIFlash::powerUp() {
    beginTransaction();
    transfer(FLASH_CMD_RELEASE_POWER_DOWN);
    endTransaction();
    delayMicroseconds(5);   // tRES1 >= 3 us (W25Q64 datasheet)
}
```

---

#### FIX 7 — `readUint16/32` and `writeUint16/32` use little-endian byte order
**Functions:** `readUint16()`, `writeUint16()`, `readUint32()`, `writeUint32()`

**Before:** These helpers stored the most-significant byte first (big-endian). The
STM32 (Cortex-M4) is a little-endian processor. `writeStruct` / `readStruct` use
`reinterpret_cast` which preserves native byte order. This meant a `uint32_t` field
written by `writeStruct` could not be correctly read back by `readUint32` — every value
would be byte-reversed.

**After:** All four helpers now store LSB first, matching native STM32 byte order.

```cpp
// readUint32 — LSB first
value = (uint32_t)buffer[0]         |
        ((uint32_t)buffer[1] <<  8) |
        ((uint32_t)buffer[2] << 16) |
        ((uint32_t)buffer[3] << 24);

// writeUint32 — LSB first
uint8_t buffer[4] = {
    (uint8_t)(value & 0xFF),
    (uint8_t)(value >>  8),
    (uint8_t)(value >> 16),
    (uint8_t)(value >> 24)
};
```

> **Note:** This fix only matters if your application calls `readUint32()` or
> `writeUint32()` directly. If you exclusively use `writeStruct()` / `readStruct()`,
> the fix has no observable effect but prevents a latent bug if scalar helpers are added later.


---

#### FIX 8 — `verify()` uses 64-byte chunk reads
**Function:** `verify()`

**Before:** Verification read one byte at a time via `readByte()`, which opens and
closes a full SPI transaction for every single byte. Verifying a 36-byte struct cost
36 complete SPI transactions.

**After:** Reads up to 64 bytes per SPI transaction using `readArray()` and `memcmp`.
This reduces SPI overhead by up to 64x and makes verification fast enough to leave
enabled in production.

---

#### FIX 9 — `isBlank()` uses 64-byte chunk reads
**Function:** `isBlank()`

Same root cause and fix as FIX 8. Previously read one byte at a time; now reads in
64-byte blocks.

---

#### FIX 10 — `readString()` uses 64-byte chunk reads
**Function:** `readString()`

Same root cause and fix as FIX 8. Previously read one byte at a time; now reads in
64-byte blocks and scans for the null terminator within each chunk.

---

#### FIX 11 — `writeString()` uses a fixed stack buffer instead of heap allocation
**Function:** `writeString()`

**Before:** `new uint8_t[length]` was called on the heap every time a string was
written. On embedded MCUs, repeated heap allocation and deallocation fragments the
heap, can fail silently, and is non-deterministic in timing.

**After:** A fixed 256-byte stack buffer is used (255 bytes of content + 1 null
terminator). Strings longer than 255 characters are silently truncated. Stack
allocation is deterministic and has zero fragmentation risk.

```cpp
const uint16_t MAX_LEN = 255;
uint8_t buffer[MAX_LEN + 1];
// fill buffer from str, append null terminator
return writeArray(address, buffer, strLen + 1, verify);
```

---

#### FIX 12 — `clearProtectionBits()` writes SR1 only
**Function:** `clearProtectionBits()`

**Before:** Two bytes were sent to the `WRITE_STATUS_REG (0x01)` command. On Winbond
W25Q chips the second byte targets SR2. Sending `0x00` to SR2 clears the QE
(Quad Enable) bit, disabling QSPI mode on chips configured for it.

**After:** Only one byte (`0x00`) is sent, clearing only the BP protection bits in SR1.

```cpp
beginTransaction();
transfer(FLASH_CMD_WRITE_STATUS_REG);
transfer(0x00);   // SR1 only — do not touch SR2
endTransaction();
```

---

#### FIX 13 — `recover()` properly reinitialises the driver after recovery
**Function:** `recover()`

**Before:** After performing the hardware recovery sequence, `initialized` was left
`false` and `chipInfo` was not repopulated. Every subsequent API call returned
`FLASH_ERROR_NO_RESPONSE`, making `recover()` completely non-functional.

**After:** `initialized` is reset to `false` at entry, `identifyChip()` is called to
repopulate `chipInfo`, and `initialized` is set to `true` only on success.

```cpp
bool LoRaE5_SPIFlash::recover() {
    _initialized = false;
    forceReleaseFromPowerDown();
    softwareReset();
    if (!waitForReady(100))  return false;
    clearProtectionBits();
    if (!identifyChip())     return false;
    _initialized = true;
    return true;
}
```

---

## Part 2 — Application Firmware Changes

These are the changes required in your own `main.cpp` (or equivalent). They must be
replicated for every project that writes a data struct to SPI flash. Wherever this
guide says `YourStruct` or `yourDat`, substitute the name of your own data
struct and instance variable.

---

### Change A — Root-cause fix: dual-sector erase before every write

**Why this matters (the 113/114 failure):**

With a 36-byte struct and 4096-byte sectors, `lcm(36, 4096) / 36 = 1024`. Records only
fall exactly on a sector boundary every 1024 records. For every other boundary
crossing, the write starts in one sector and ends in the next.

The old code only erased when `writeAdd % sectorSize == 0` (write starts exactly on a
boundary). On all other boundary crossings, the next sector was never erased, so
`PAGE_PROGRAM` wrote into unerased cells and `FLASH_ERROR_VERIFY_FAILED` was returned at
record 113 on every device.

**The same failure point exists for any struct size that is not a power of 2:**

| Struct size | First failure at record |
|---|---|
| 36 bytes | 113 |
| 40 bytes | 102 |
| 48 bytes | 85 |
| 52 bytes | 78 |

**Fix — replace the old single boundary check with these two cases:**

```cpp
// Place this block immediately before flash.writeStruct()

uint32_t sectorSize   = flash.getSectorSize();
uint32_t writeEndAddr = writeAdd + sizeof(yourDat) - 1;

// Case 1: write starts exactly on a sector boundary — erase this sector
if (writeAdd % sectorSize == 0) {
    uint8_t eraseResult = flash.eraseSector(writeAdd);
    if (eraseResult != FLASH_OK) {
        flash.powerDown();
        return;
    }
}

// Case 2: write crosses into the NEXT sector — erase that sector too
if ((writeEndAddr / sectorSize) > (writeAdd / sectorSize)) {
    uint32_t nextSector = ((writeAdd / sectorSize) + 1) * sectorSize;
    uint8_t eraseResult = flash.eraseSector(nextSector);
    if (eraseResult != FLASH_OK) {
        flash.powerDown();
        return;
    }
}

// Now safe to write
flash.writeStruct(writeAdd, yourDat, true);
```

> Both cases use separate `if` statements (not `if / else`) so that both fire
> simultaneously if the struct is large enough to span two boundaries in one write.

---

### Change B — Use `sizeof(YourStruct)` everywhere, never hardcode a byte count

If the struct layout changes, a hardcoded stride silently desynchronises `writeAdd`
from the actual byte footprint on flash.

```cpp
// Wrong
writeAdd = writeAdd + 36;
readAdd  = readAdd  + 36;

// Correct
writeAdd += sizeof(yourDat);
readAdd  += sizeof(yourDat);
```

Ensure your struct uses `__attribute__((__packed__))` so the compiler does not insert padding:

```cpp
struct __attribute__((__packed__)) YourStruct {
    uint32_t datetime;
    // ... your fields ...
};
```

---

### Change C — Reserve the last flash sector for metadata (capacity guard)

Add this check immediately after `flash.powerUp()` and before the erase logic:

```cpp
uint32_t metaSector = flash.getCapacity() - flash.getSectorSize();
if (writeAdd + sizeof(yourDat) > metaSector) {
    Serial.println(F("Flash data area full"));
    flash.powerDown();
    return;
}
```

For a W25Q64 (8 MB), `metaSector = 0x7FF000`.
For a W25Q32 (4 MB), `metaSector = 0x3FF000`.
The value is computed at runtime from `getCapacity()` so it adapts automatically to
whichever flash chip is fitted.


### Change D — NVM persistence for `writeAdd` and `readAdd`

Without this, the device resets `writeAdd` to `0x0` on every power cycle and
overwrites all previously stored records from the beginning.

#### D.1 — Add the metadata struct and constants near the top of your source file

Place this immediately after your `writeAdd` / `readAdd` variable declarations:

```cpp
uint32_t writeAdd = 0;
uint32_t readAdd  = 0;

// Wear-leveled metadata persistence in the last flash sector
#define FLASH_META_MAGIC  0xA5A5A5A5UL
#define FLASH_META_SIZE   12u

struct __attribute__((__packed__)) FlashMeta {
    uint32_t magic;     // Set to FLASH_META_MAGIC when slot is valid
    uint32_t writeAdd;  // Saved write pointer
    uint32_t readAdd;   // Saved read pointer
};

// Forward declarations (functions defined before setup(), called from write task)
bool loadFlashMetadata();
void saveFlashMetadata();
```

#### D.2 — Add the two helper functions before `setup()`

```cpp
// Scan the metadata sector for the last valid entry and restore writeAdd/readAdd.
// Returns true if valid metadata was found, false if starting fresh from address 0.
bool loadFlashMetadata() {
    uint32_t sector = flash.getCapacity() - flash.getSectorSize();
    uint32_t slots  = flash.getSectorSize() / FLASH_META_SIZE;
    FlashMeta meta;
    bool found = false;
    for (uint32_t i = 0; i < slots; i++) {
        uint8_t result = flash.readStruct(sector + i * FLASH_META_SIZE, meta);
        if (result != FLASH_OK)              continue;
        if (meta.magic != FLASH_META_MAGIC)  continue;
        writeAdd = meta.writeAdd;
        readAdd  = meta.readAdd;
        if (readAdd > writeAdd) { readAdd = 0; }  // guard: corrupted metadata
        found = true;
    }
    if (found) {
        Serial.print(F("Metadata restored: writeAdd=0x"));
        Serial.print(writeAdd, HEX);
        Serial.print(F(", readAdd=0x"));
        Serial.println(readAdd, HEX);
    } else {
        Serial.println(F("No metadata - starting from address 0"));
        writeAdd = 0;
        readAdd  = 0;
    }
    return found;
}

// Persist writeAdd and readAdd to the next free slot in the metadata sector.
// When all 341 slots are used the sector is erased and writing restarts from slot 0.
void saveFlashMetadata() {
    uint32_t sector = flash.getCapacity() - flash.getSectorSize();
    uint32_t slots  = flash.getSectorSize() / FLASH_META_SIZE;
    FlashMeta meta;
    int32_t freeSlot = -1;
    for (uint32_t i = 0; i < slots; i++) {
        flash.readStruct(sector + i * FLASH_META_SIZE, meta);
        if (meta.magic == 0xFFFFFFFF) {   // erased flash reads as 0xFF
            freeSlot = (int32_t)i;
            break;
        }
    }
    if (freeSlot < 0) {
        flash.eraseSector(sector);        // all slots used — wrap around
        freeSlot = 0;
    }
    FlashMeta newMeta = { FLASH_META_MAGIC, writeAdd, readAdd };
    uint8_t result = flash.writeStruct(
        sector + (uint32_t)freeSlot * FLASH_META_SIZE, newMeta, true);
    if (result != FLASH_OK) {
        Serial.print(F("Metadata save failed: "));
        Serial.println(result);
    }
}
```

#### D.3 — Call `saveFlashMetadata()` after every successful write

Call it immediately after `writeAdd += sizeof(yourDat)`, while flash is still powered:

```cpp
writeAdd += sizeof(yourDat);
saveFlashMetadata();    // persist before powering flash down
flash.powerDown();
```

#### D.4 — Call `loadFlashMetadata()` in `setup()` after `flash.begin()` succeeds

```cpp
if (flash.begin()) {
    flash.printChipInfo();
    loadFlashMetadata();    // flash is still powered here
} else {
    Serial.println("Failed to initialize SPI Flash.");
}
flash.powerDown();
```

#### Wear-level analysis

| Parameter | Value |
|---|---|
| Metadata sector size | 4096 bytes |
| Slot size | 12 bytes |
| Slots per sector erase | 341 |
| GPS fix interval | 10 min |
| Saves per day | 144 |
| Sector erases per year | ~154 |
| W25Q64 rated erase cycles | 100,000 |
| Years until metadata sector wears out | ~650 years |

Even at a 1-minute fix interval the metadata sector lasts ~65 years.

---

## Part 3 — Quick Checklist for Porting to a New Device

- [ ] Copy `LoRaE5_SPIFlash.cpp` and `LoRaE5_SPIFlash.h` into `lib/LoRaE5_SPIFlash/`
- [ ] Confirm your data struct has `__attribute__((__packed__))`
- [ ] Verify `sizeof(YourStruct)` at runtime matches your expected byte count
- [ ] Replace the old single-case erase with the two-case block from **Change A**
- [ ] Replace every hardcoded byte stride with `sizeof(yourDat)` (**Change B**)
- [ ] Add the capacity guard from **Change C** before the erase logic
- [ ] Add the `FlashMeta` struct, defines, and forward declarations from **D.1**
- [ ] Add `loadFlashMetadata()` and `saveFlashMetadata()` before `setup()` (**D.2**)
- [ ] Call `saveFlashMetadata()` after every successful `writeAdd` increment (**D.3**)
- [ ] Call `loadFlashMetadata()` inside `flash.begin()` success block in `setup()` (**D.4**)
- [ ] The GSM/MQTT upload loop needs no changes — `readAdd` is restored automatically on boot

---

## Part 4 — Error Code Reference

| Code | Value | Meaning |
|---|---|---|
| `FLASH_OK` | `0x00` | Operation succeeded |
| `FLASH_ERROR_TIMEOUT` | `0x01` | `waitForReady()` expired — erase or write stalled |
| `FLASH_ERROR_WRITE_PROTECTED` | `0x02` | WEL bit not set after `writeEnable()` |
| `FLASH_ERROR_INVALID_ADDRESS` | `0x03` | Address out of range or page-boundary crossing |
| `FLASH_ERROR_NOT_ERASED` | `0x04` | Target location was not blank before write |
| `FLASH_ERROR_VERIFY_FAILED` | `0x05` | Read-back did not match written data |
| `FLASH_ERROR_NO_RESPONSE` | `0x06` | `initialized` is false — call `begin()` first |
| `FLASH_ERROR_UNKNOWN_CHIP` | `0x07` | JEDEC ID not recognised and capacity is 0 |

The most common field error was `FLASH_ERROR_VERIFY_FAILED (0x05)` at record 113,
caused by the unerased sector described in Change A. After applying Change A this
error should not appear in normal operation.

---

*Applies to LoRaE5_SPIFlash with all 13 fixes applied and main.cpp with NVM
persistence and dual-sector erase fix. Build verified clean on PlatformIO
ststm32 / lora_e5_mini / framework-arduinoststm32.*
