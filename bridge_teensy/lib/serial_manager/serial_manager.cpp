#include <Arduino.h>
#include "serial_manager.h"

// Reads a command frame from the serial stream.
// Returns an error code if anything goes wrong (timeout, bad header, CRC mismatch, etc.)
CommandError SerialManager::readCommand(uint8_t &type, uint8_t *payload, uint8_t &length, uint8_t &checksum) {

    // Helper lambda to wait for a byte with timeout
    auto waitForByte = [&](uint8_t &out) -> bool {
        unsigned long start = millis();
        while (commStream.available() < 1) {
            // If timeout expires, set error and return false
            if (millis() - start > timeoutMs) {
                lastError = CommandError::Timeout;
                return false;
            } 
        }
        out = commStream.read();
        return true;
    };

    // If no data is available, return an empty buffer error
    if (commStream.available() < 1)
    { 
        lastError = CommandError::EmptyBuffer;
        return lastError;
    }

    // Search for the header byte (0xAA) to start a new frame
    uint8_t byte;
    uint16_t headerWaitCount = 0;
    do {
        if (!waitForByte(byte)) return lastError; // Timeout while waiting for header
        headerWaitCount++;
        if (headerWaitCount > 1000) { // Prevent infinite loop if header never arrives
            lastError = CommandError::BadHeader;
            return lastError;
        }
    } while (byte != HEADER_BYTE);


    // Read the command type byte
    if (!waitForByte(type)) return lastError;

    // Read the payload length byte
    if (!waitForByte(length)) return lastError;
    // Check if the payload length is within allowed bounds
    if (length > maxPayloadSize) {
        lastError = CommandError::PayloadTooLarge;
        return lastError;
    }

    // Read the payload bytes
    for (uint8_t i = 0; i < length; i++) {
        if (!waitForByte(payload[i])) return lastError;
    }

    // Read the checksum byte
    if (!waitForByte(checksum)) return lastError;

    // Prepare buffer for CRC calculation
    uint8_t crc_buf[3 + maxPayloadSize];
    crc_buf[0] = HEADER_BYTE;
    crc_buf[1] = type;
    crc_buf[2] = length;
    for (uint8_t i = 0; i < length; i++) {
        crc_buf[3 + i] = payload[i];
    }

    // Calculate CRC-8 and compare with received checksum
    uint8_t calc_crc = crc8(crc_buf, 3 + length);
    if (calc_crc != checksum) {
        lastError = CommandError::CRCMismatch;
        return lastError;
    }

    // If everything is OK, set error to None
    lastError = CommandError::None;
    return lastError;
}


// CRC-8 calculation for a buffer
uint8_t SerialManager::crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}


// Sends a command frame over the serial stream
void SerialManager::sendCommand(uint8_t type, const uint8_t *payload, uint8_t length) {

    // Prepare buffer for CRC calculation: [Header][Type][Length][Payload...]
    uint8_t crc_buf[3 + length];
    crc_buf[0] = HEADER_BYTE;
    crc_buf[1] = type;
    crc_buf[2] = length;
    for (uint8_t i = 0; i < length; i++) {
        crc_buf[3 + i] = payload[i];
    }

    // Calculate CRC-8 checksum
    uint8_t checksum = crc8(crc_buf, 3 + length);

    // Write header, type, length to the stream
    commStream.write(HEADER_BYTE);
    commStream.write(type);
    commStream.write(length);

    // Write payload bytes to the stream
    for (uint8_t i = 0; i < length; i++) {
        commStream.write(payload[i]);
    }

    // Write checksum byte to the stream
    commStream.write(checksum);
}