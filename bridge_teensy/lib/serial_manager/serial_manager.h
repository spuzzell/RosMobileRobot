#pragma once
#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include <Arduino.h>

/**
 * @brief Error codes for command reading and communication.
 */
enum class CommandError : uint8_t {
    None = 0,         /**< No error */
    EmptyBuffer,      /**< No data available in buffer */
    Timeout,          /**< Timeout while waiting for data */
    BadHeader,        /**< Invalid header byte received */
    PayloadTooLarge,  /**< Payload size exceeds maximum allowed */
    CRCMismatch,      /**< CRC-8 checksum mismatch */
    StreamError       /**< General stream error */
};


/**
 * @brief Manages serial communication of commands between microcontrollers and host.
 *
 * This class provides methods for sending and receiving framed commands
 * with CRC-8 validation over a Stream interface. It is designed for robust
 * communication in embedded robotics applications.
 */
class SerialManager
{
    public:
    /**
     * @brief Construct a new SerialManager object.
     * 
     * @param inputStream Reference to the Stream object for communication.
     * @param timeoutMs Timeout in milliseconds for reading operations.
     * @param maxPayloadSize Maximum allowed payload size in bytes.
     */
    SerialManager(Stream &inputStream, unsigned long timeoutMs = 100, uint8_t maxPayloadSize = 32)
        : commStream(inputStream), timeoutMs(timeoutMs), maxPayloadSize(maxPayloadSize), lastError(CommandError::None) {}

    /**
     * @brief Set the timeout for reading operations.
     * 
     * @param ms Timeout in milliseconds.
     */
    void setTimeout(unsigned long ms) { timeoutMs = ms; }

    /**
     * @brief Set the maximum allowed payload size.
     * 
     * @param size Maximum payload size in bytes.
     */
    void setMaxPayloadSize(uint8_t size) { maxPayloadSize = size; }

    /**
     * @brief Read a command from the input stream.
     * 
     * @param type Output: Command type byte.
     * @param payload Output: Buffer to store payload bytes.
     * @param length Output: Number of payload bytes.
     * @param checksum Output: Received checksum byte.
     * @return CommandError Error code indicating result of the operation.
     */
    CommandError readCommand(uint8_t &type, uint8_t *payload, uint8_t &length, uint8_t &checksum);

    /**
     * @brief Send a command over the output stream.
     * 
     * @param type Command type byte.
     * @param payload Pointer to payload bytes (can be nullptr if length is 0).
     * @param length Number of payload bytes.
     */
    void sendCommand(uint8_t type, const uint8_t *payload, uint8_t length);

    /**
     * @brief Get the last error code.
     * 
     * @return CommandError Last error encountered.
     */
    CommandError getLastError() const { return lastError; }


    private:
    Stream &commStream;           /**< Reference to the communication stream */
    unsigned long timeoutMs;      /**< Timeout for reading operations (ms) */
    uint8_t maxPayloadSize;       /**< Maximum allowed payload size (bytes) */
    CommandError lastError;       /**< Last error encountered */
    static constexpr uint8_t HEADER_BYTE = 0xAA;

    /**
     * @brief Calculate CRC-8 checksum for a data buffer.
     * 
     * @param data Pointer to data buffer.
     * @param len Number of bytes in buffer.
     * @return uint8_t CRC-8 value.
     */
    uint8_t crc8(const uint8_t *data, uint8_t len);

};

#endif //SERIAL_MANAGER_H