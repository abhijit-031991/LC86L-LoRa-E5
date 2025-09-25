/**
 * LoRaE5_SPIFlash.cpp
 * 
 * Custom SPI Flash Memory Library Implementation for LoRa-E5 (STM32WLE5JC)
 * 
 * Author: Custom implementation for LoRa-E5
 * License: MIT
 */

#include "LoRaE5_SPIFlash.h"

// Known flash chip database
static const FlashChipInfo knownChips[] = {
    // Winbond W25Q series
    {0xEF4016, 4194304,   256, 4096, 65536, "Winbond", "W25Q32"},
    {0xEF4017, 8388608,   256, 4096, 65536, "Winbond", "W25Q64"},
    {0xEF4018, 16777216,  256, 4096, 65536, "Winbond", "W25Q128"},
    {0xEF4019, 33554432,  256, 4096, 65536, "Winbond", "W25Q256"},
    
    // Macronix MX25L series
    {0xC22016, 4194304,   256, 4096, 65536, "Macronix", "MX25L32"},
    {0xC22017, 8388608,   256, 4096, 65536, "Macronix", "MX25L64"},
    {0xC22018, 16777216,  256, 4096, 65536, "Macronix", "MX25L128"},
    
    // Microchip SST25 series
    {0xBF2541, 2097152,   256, 4096, 65536, "Microchip", "SST25VF016B"},
    {0xBF254A, 4194304,   256, 4096, 65536, "Microchip", "SST25VF032B"},
    
    // Cypress S25FL series
    {0x012018, 16777216,  256, 4096, 65536, "Cypress", "S25FL128S"},
    {0x012019, 33554432,  256, 4096, 65536, "Cypress", "S25FL256S"},
    
    // Generic fallback
    {0x000000, 0, 256, 4096, 65536, "Unknown", "Generic"}
};

LoRaE5_SPIFlash::LoRaE5_SPIFlash(uint8_t csPin, SPIClass* spiInstance, uint32_t spiFrequency) 
    : _spi(spiInstance), _csPin(csPin), _spiFrequency(spiFrequency), _initialized(false) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    
    // Initialize chip info to defaults
    _chipInfo = {0, 0, 256, 4096, 65536, "Unknown", "Unknown"};
}

bool LoRaE5_SPIFlash::begin() {
    if (_spi == nullptr) return false;
    
    _spi->begin();
    
    // Power up the chip if it was in power-down mode
    powerUp();
    delay(10);
    
    // Try to identify the chip
    if (!identifyChip()) {
        return false;
    }
    
    _initialized = true;
    return true;
}

void LoRaE5_SPIFlash::end() {
    if (_initialized) {
        powerDown();
        _spi->end();
        _initialized = false;
    }
}

void LoRaE5_SPIFlash::beginTransaction() {
    _spi->beginTransaction(SPISettings(_spiFrequency, MSBFIRST, SPI_MODE0));
    digitalWrite(_csPin, LOW);
}

void LoRaE5_SPIFlash::endTransaction() {
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}

uint8_t LoRaE5_SPIFlash::verify(uint32_t address, const uint8_t* buffer, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        uint8_t readData = readByte(address + i);
        if (readData != buffer[i]) {
            return FLASH_ERROR_VERIFY_FAILED;
        }
    }
    return FLASH_OK;
}

void LoRaE5_SPIFlash::setSPIFrequency(uint32_t frequency) {
    _spiFrequency = frequency;
}

void LoRaE5_SPIFlash::printChipInfo() {
    Serial.println("=== Flash Chip Information ===");
    Serial.print("Manufacturer: ");
    Serial.println(_chipInfo.manufacturer);
    Serial.print("Model: ");
    Serial.println(_chipInfo.model);
    Serial.print("JEDEC ID: 0x");
    Serial.println(_chipInfo.jedecId, HEX);
    Serial.print("Capacity: ");
    Serial.print(_chipInfo.capacity);
    Serial.println(" bytes");
    Serial.print("Page Size: ");
    Serial.print(_chipInfo.pageSize);
    Serial.println(" bytes");
    Serial.print("Sector Size: ");
    Serial.print(_chipInfo.sectorSize);
    Serial.println(" bytes");
    Serial.print("Block Size: ");
    Serial.print(_chipInfo.blockSize);
    Serial.println(" bytes");
    Serial.println("===============================");
}

void LoRaE5_SPIFlash::runDiagnostics() {
    Serial.println("=== Flash Memory Diagnostics ===");
    
    // Test 1: Basic communication
    Serial.print("1. Communication Test: ");
    uint32_t jedecId = readJEDECID();
    if (jedecId != 0x000000 && jedecId != 0xFFFFFF) {
        Serial.println("PASS");
        Serial.print("   JEDEC ID: 0x");
        Serial.println(jedecId, HEX);
    } else {
        Serial.println("FAIL - No response from chip");
        return;
    }
    
    // Test 2: Status register read
    Serial.print("2. Status Register Test: ");
    uint8_t status = readStatusRegister();
    Serial.println("PASS");
    Serial.print("   Status: 0x");
    Serial.println(status, HEX);
    Serial.print("   Busy: ");
    Serial.println(isBusy() ? "Yes" : "No");
    Serial.print("   Write Enabled: ");
    Serial.println(isWriteEnabled() ? "Yes" : "No");
    
    // Test 3: Write/Read test
    Serial.print("3. Write/Read Test: ");
    uint32_t testAddr = 0x1000;
    uint8_t testData = 0xAA;
    
    // Erase sector first
    if (eraseSector(testAddr) == FLASH_OK) {
        if (writeByte(testAddr, testData) == FLASH_OK) {
            uint8_t readData = readByte(testAddr);
            if (readData == testData) {
                Serial.println("PASS");
            } else {
                Serial.print("FAIL - Read verification (expected: 0x");
                Serial.print(testData, HEX);
                Serial.print(", got: 0x");
                Serial.print(readData, HEX);
                Serial.println(")");
            }
        } else {
            Serial.println("FAIL - Write operation");
        }
    } else {
        Serial.println("FAIL - Sector erase");
    }
    
    // Test 4: Array operations
    Serial.print("4. Array Operations Test: ");
    uint8_t testArray[16];
    uint8_t readArray[16];
    for (int i = 0; i < 16; i++) {
        testArray[i] = i * 16 + i;
    }
    
    if (writeArray(testAddr + 0x100, testArray, 16) == FLASH_OK) {
        if (this->readArray(testAddr + 0x100, readArray, 16) == FLASH_OK) {
            bool match = true;
            for (int i = 0; i < 16; i++) {
                if (testArray[i] != readArray[i]) {
                    match = false;
                    break;
                }
            }
            if (match) {
                Serial.println("PASS");
            } else {
                Serial.println("FAIL - Array data mismatch");
            }
        } else {
            Serial.println("FAIL - Array read");
        }
    } else {
        Serial.println("FAIL - Array write");
    }
    
    // Test 5: String operations
    Serial.print("5. String Operations Test: ");
    String testStr = "LoRa-E5 Flash";
    String readStr;
    
    if (writeString(testAddr + 0x200, testStr) == FLASH_OK) {
        if (readString(testAddr + 0x200, readStr) == FLASH_OK) {
            if (testStr == readStr) {
                Serial.println("PASS");
            } else {
                Serial.print("FAIL - String mismatch (expected: '");
                Serial.print(testStr);
                Serial.print("', got: '");
                Serial.print(readStr);
                Serial.println("')");
            }
        } else {
            Serial.println("FAIL - String read");
        }
    } else {
        Serial.println("FAIL - String write");
    }
    
    Serial.println("=== Diagnostics Complete ===");
}

uint8_t LoRaE5_SPIFlash::transfer(uint8_t data) {
    return _spi->transfer(data);
}

void LoRaE5_SPIFlash::transfer(uint8_t* buffer, size_t length) {
    _spi->transfer(buffer, length);
}

void LoRaE5_SPIFlash::writeEnable() {
    beginTransaction();
    transfer(FLASH_CMD_WRITE_ENABLE);
    endTransaction();
}

void LoRaE5_SPIFlash::writeDisable() {
    beginTransaction();
    transfer(FLASH_CMD_WRITE_DISABLE);
    endTransaction();
}

uint8_t LoRaE5_SPIFlash::readStatusRegister() {
    beginTransaction();
    transfer(FLASH_CMD_READ_STATUS_REG);
    uint8_t status = transfer(0x00);
    endTransaction();
    return status;
}

void LoRaE5_SPIFlash::waitForReady(uint32_t timeoutMs) {
    uint32_t startTime = millis();
    while (isBusy()) {
        if (millis() - startTime > timeoutMs) {
            break; // Timeout
        }
        delay(1);
    }
}

bool LoRaE5_SPIFlash::isWriteEnabled() {
    return (readStatusRegister() & FLASH_STATUS_WEL) != 0;
}

bool LoRaE5_SPIFlash::isBusy() {
    return (readStatusRegister() & FLASH_STATUS_BUSY) != 0;
}

uint32_t LoRaE5_SPIFlash::readJEDECID() {
    beginTransaction();
    transfer(FLASH_CMD_JEDEC_ID);
    uint32_t id = 0;
    id |= (uint32_t)transfer(0x00) << 16;  // Manufacturer ID
    id |= (uint32_t)transfer(0x00) << 8;   // Memory Type
    id |= (uint32_t)transfer(0x00);        // Capacity
    endTransaction();
    return id;
}

bool LoRaE5_SPIFlash::identifyChip() {
    uint32_t jedecId = readJEDECID();
    
    if (jedecId == 0x000000 || jedecId == 0xFFFFFF) {
        return false; // No chip detected or communication error
    }
    
    setChipInfo(jedecId);
    return true;
}

void LoRaE5_SPIFlash::setChipInfo(uint32_t jedecId) {
    // Search for known chip
    for (size_t i = 0; i < sizeof(knownChips) / sizeof(knownChips[0]) - 1; i++) {
        if (knownChips[i].jedecId == jedecId) {
            _chipInfo = knownChips[i];
            return;
        }
    }
    
    // Unknown chip - try to determine capacity from JEDEC ID
    uint8_t capacityCode = jedecId & 0xFF;
    uint32_t capacity = 1 << capacityCode; // 2^capacity_code bytes
    
    _chipInfo.jedecId = jedecId;
    _chipInfo.capacity = capacity;
    _chipInfo.pageSize = 256;      // Standard page size
    _chipInfo.sectorSize = 4096;   // Standard sector size
    _chipInfo.blockSize = 65536;   // Standard block size
    _chipInfo.manufacturer = "Unknown";
    _chipInfo.model = "Generic";
}

bool LoRaE5_SPIFlash::isValidAddress(uint32_t address) {
    return address < _chipInfo.capacity;
}

bool LoRaE5_SPIFlash::isValidRange(uint32_t address, uint32_t length) {
    return (address < _chipInfo.capacity) && 
           (address + length <= _chipInfo.capacity);
}

void LoRaE5_SPIFlash::powerDown() {
    beginTransaction();
    transfer(FLASH_CMD_POWER_DOWN);
    endTransaction();
}

void LoRaE5_SPIFlash::powerUp() {
    beginTransaction();
    transfer(FLASH_CMD_RELEASE_POWER_DOWN);
    endTransaction();
}

uint8_t LoRaE5_SPIFlash::eraseSector(uint32_t address) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidAddress(address)) return FLASH_ERROR_INVALID_ADDRESS;
    
    // Align address to sector boundary
    address = (address / _chipInfo.sectorSize) * _chipInfo.sectorSize;
    
    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;
    
    beginTransaction();
    transfer(FLASH_CMD_SECTOR_ERASE);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    endTransaction();
    
    waitForReady(10000); // Sector erase can take up to 400ms typically
    
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::eraseBlock32K(uint32_t address) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidAddress(address)) return FLASH_ERROR_INVALID_ADDRESS;
    
    address = (address / 32768) * 32768; // Align to 32K boundary
    
    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;
    
    beginTransaction();
    transfer(FLASH_CMD_BLOCK_ERASE_32K);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    endTransaction();
    
    waitForReady(30000); // Block erase can take up to 2s
    
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::eraseBlock64K(uint32_t address) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidAddress(address)) return FLASH_ERROR_INVALID_ADDRESS;
    
    address = (address / _chipInfo.blockSize) * _chipInfo.blockSize;
    
    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;
    
    beginTransaction();
    transfer(FLASH_CMD_BLOCK_ERASE_64K);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    endTransaction();
    
    waitForReady(30000); // Block erase can take up to 2s
    
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::eraseChip() {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    
    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;
    
    beginTransaction();
    transfer(FLASH_CMD_CHIP_ERASE);
    endTransaction();
    
    waitForReady(120000); // Chip erase can take up to 100s for large chips
    
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::readByteInternal(uint32_t address) {
    beginTransaction();
    transfer(FLASH_CMD_READ_DATA);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    uint8_t data = transfer(0x00);
    endTransaction();
    return data;
}

uint8_t LoRaE5_SPIFlash::readByte(uint32_t address) {
    if (!_initialized) return 0xFF;
    if (!isValidAddress(address)) return 0xFF;
    
    return readByteInternal(address);
}

uint8_t LoRaE5_SPIFlash::writeByteInternal(uint32_t address, uint8_t data, bool verify) {
    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;
    
    beginTransaction();
    transfer(FLASH_CMD_PAGE_PROGRAM);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    transfer(data);
    endTransaction();
    
    waitForReady(5000);
    
    if (verify) {
        uint8_t readBack = readByteInternal(address);
        if (readBack != data) {
            return FLASH_ERROR_VERIFY_FAILED;
        }
    }
    
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::writeByte(uint32_t address, uint8_t data, bool verify) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidAddress(address)) return FLASH_ERROR_INVALID_ADDRESS;
    
    return writeByteInternal(address, data, verify);
}

uint8_t LoRaE5_SPIFlash::readArray(uint32_t address, uint8_t* buffer, uint32_t length) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, length)) return FLASH_ERROR_INVALID_ADDRESS;
    if (buffer == nullptr) return FLASH_ERROR_INVALID_ADDRESS;
    
    beginTransaction();
    transfer(FLASH_CMD_READ_DATA);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    
    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = transfer(0x00);
    }
    
    endTransaction();
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::writeArray(uint32_t address, const uint8_t* buffer, uint32_t length, bool verify) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, length)) return FLASH_ERROR_INVALID_ADDRESS;
    if (buffer == nullptr) return FLASH_ERROR_INVALID_ADDRESS;
    
    uint32_t bytesWritten = 0;
    
    while (bytesWritten < length) {
        // Calculate how many bytes we can write in this page
        uint32_t currentAddress = address + bytesWritten;
        uint32_t pageOffset = currentAddress % _chipInfo.pageSize;
        uint32_t bytesThisPage = min(length - bytesWritten, _chipInfo.pageSize - pageOffset);
        
        // Write page
        uint8_t result = writePage(currentAddress, buffer + bytesWritten, bytesThisPage, verify);
        if (result != FLASH_OK) {
            return result;
        }
        
        bytesWritten += bytesThisPage;
    }
    
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::writePage(uint32_t address, const uint8_t* buffer, uint16_t length, bool verify) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (length > _chipInfo.pageSize) return FLASH_ERROR_INVALID_ADDRESS;
    
    writeEnable();
    if (!isWriteEnabled()) return FLASH_ERROR_WRITE_PROTECTED;
    
    beginTransaction();
    transfer(FLASH_CMD_PAGE_PROGRAM);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    
    for (uint16_t i = 0; i < length; i++) {
        transfer(buffer[i]);
    }
    
    endTransaction();
    
    waitForReady(5000);
    
    if (verify) {
        return this->verify(address, buffer, length);
    }
    
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::fastRead(uint32_t address, uint8_t* buffer, uint32_t length) {
    if (!_initialized) return FLASH_ERROR_NO_RESPONSE;
    if (!isValidRange(address, length)) return FLASH_ERROR_INVALID_ADDRESS;
    if (buffer == nullptr) return FLASH_ERROR_INVALID_ADDRESS;
    
    beginTransaction();
    transfer(FLASH_CMD_FAST_READ);
    transfer((address >> 16) & 0xFF);
    transfer((address >> 8) & 0xFF);
    transfer(address & 0xFF);
    transfer(0x00); // Dummy byte for fast read
    
    for (uint32_t i = 0; i < length; i++) {
        buffer[i] = transfer(0x00);
    }
    
    endTransaction();
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::readUint16(uint32_t address, uint16_t& value) {
    uint8_t buffer[2];
    uint8_t result = readArray(address, buffer, 2);
    if (result == FLASH_OK) {
        value = (buffer[0] << 8) | buffer[1];
    }
    return result;
}

uint8_t LoRaE5_SPIFlash::writeUint16(uint32_t address, uint16_t value, bool verify) {
    uint8_t buffer[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return writeArray(address, buffer, 2, verify);
}

uint8_t LoRaE5_SPIFlash::readUint32(uint32_t address, uint32_t& value) {
    uint8_t buffer[4];
    uint8_t result = readArray(address, buffer, 4);
    if (result == FLASH_OK) {
        value = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | 
                ((uint32_t)buffer[2] << 8) | buffer[3];
    }
    return result;
}

uint8_t LoRaE5_SPIFlash::writeUint32(uint32_t address, uint32_t value, bool verify) {
    uint8_t buffer[4] = {
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };
    return writeArray(address, buffer, 4, verify);
}

uint8_t LoRaE5_SPIFlash::readFloat(uint32_t address, float& value) {
    return readArray(address, reinterpret_cast<uint8_t*>(&value), sizeof(float));
}

uint8_t LoRaE5_SPIFlash::writeFloat(uint32_t address, float value, bool verify) {
    return writeArray(address, reinterpret_cast<const uint8_t*>(&value), sizeof(float), verify);
}

uint8_t LoRaE5_SPIFlash::readString(uint32_t address, String& str, uint16_t maxLength) {
    str = "";
    
    for (uint16_t i = 0; i < maxLength; i++) {
        uint8_t c = readByte(address + i);
        if (c == 0) break; // Null terminator
        str += (char)c;
    }
    
    return FLASH_OK;
}

uint8_t LoRaE5_SPIFlash::writeString(uint32_t address, const String& str, bool verify) {
    uint32_t length = str.length() + 1; // Include null terminator
    uint8_t* buffer = new uint8_t[length];
    
    for (uint32_t i = 0; i < str.length(); i++) {
        buffer[i] = str.charAt(i);
    }
    buffer[str.length()] = 0; // Null terminator
    
    uint8_t result = writeArray(address, buffer, length, verify);
    
    delete[] buffer;
    return result;
}

bool LoRaE5_SPIFlash::isBlank(uint32_t address, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        if (readByte(address + i) != 0xFF) {
            return false;
        }
    }
    return true;
}

