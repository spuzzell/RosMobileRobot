#pragma once
#ifndef COMMANDS_H
#define COMMANDS_H

//##################################################################################################
//                                             COMMANDS
//##################################################################################################

//List of defined Command type byte for motors
#define CMD_ENCODER     0x04     // Request encoder data: no payload


//List of defined Command type byte for system and communication control
#define CMD_PING        0x07     // Heartbeat request: payload = uint8 sequence_id
#define CMD_PONG        0x08     // Heartbeat response: payload = uint8 sequence_id
#define CMD_TIME        0x09     // Request system time: no payload
#define CMD_TIME_RESP   0x0A    // System time response: payload = uint32 timestamp_ms

//List of defined Command type byte for general system, status, and error reporting
#define CMD_INFO        0x0B  // Informational message, payload: uint8 info_code, optional data
#define CMD_WARN        0x0C  // Warning message, payload: uint8 warn_code, optional data
#define CMD_ERROR       0x0D  // Error message, payload: uint8 error_code, optional data
#define CMD_BOOT        0x0E  // Board booted, payload: uint32 timestamp_ms
#define CMD_READY       0x0F  // System ready, payload: uint8 status_code
#define CMD_NACK        0x14  // Negative acknowledgment: payload = uint8 error_code


//##################################################################################################
//                                             PAYLOADS
//##################################################################################################

// Predefined info codes
#define INFO_USB_SERIAL_READY     0x01  // USB serial initialized
#define INFO_UART2_READY          0x02  // UART2 (Serial2) initialized
#define INFO_MOTOR_READY          0x03  // UART2 (Serial2) initialized
#define INFO_ENCODER_READY        0x04  // UART2 (Serial2) initialized
#define INFO_BOARD_BOOTED         0x05  // Board booted
#define INFO_SYSTEM_READY         0x06  // System ready
#define INFO_VERSION              0x07  // Firmware version info

// Predefined warning codes
#define WARN_UART2_UNREACHABLE    0x01  // UART2 not available

// Predefined error codes
#define ERROR_CRC_MISMATCH        0x01  // CRC-8 checksum mismatch
#define ERROR_PAYLOAD_TOO_LARGE   0x02  // Payload size exceeds maximum
#define ERROR_BAD_HEADER          0x03  // Invalid header byte
#define ERROR_TIMEOUT             0x04  // Timeout waiting for data
#define ERROR_STREAM_FAILURE      0x05  // Stream error
#define ERROR_UNKNOWN_COMMAND     0x06  // Unknown command type


#endif //COMMANDS_H