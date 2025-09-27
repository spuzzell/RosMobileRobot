// ...existing code...
#include <Arduino.h>
#include <Encoder.h>
#include "serial_manager.h"


// ====== Commands ======
// CMD_ENCODER: host requests the current encoder ticks (no payload sent by host)
// CMD_STATUS:  device responds with encoder ticks (payload contains 4x int32 + timestamp)
#define CMD_ENCODER     0x04     // Request encoder data: no payload
#define CMD_STATUS      0x05     // Encoder response: payload = int32 ticks per wheel

// ====== Serial configuration and managers ======
constexpr unsigned long TIMEOUT_MS = 100;          // read timeout for commands (ms)
constexpr uint8_t MAX_PAYLOAD_SIZE = 32;           // maximum payload buffer size
constexpr unsigned long SERIAL_BAUD = 115200;      // UART baud rate to ESP32
SerialManager ESP32_serialManager2(Serial2, TIMEOUT_MS, MAX_PAYLOAD_SIZE);

// ====== Encoder objects ======
// Pins configured for each quadrature encoder (A, B)
Encoder motorFL(2, 3);
Encoder motorRL(4, 5);
Encoder motorFR(9, 10);
Encoder motorRR(11, 12);

void setup() {
    // Initialize Serial2 for communication with the ESP32 (or host)
    Serial2.begin(SERIAL_BAUD);
    // Wait for the serial port to be ready (useful for USB-serial devices)
    while (!Serial2) {}
}

void loop() {
    // Buffers and variables for incoming command parsing
    uint8_t commandType;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint8_t payloadLength;
    uint8_t receivedChecksum;

    // Attempt to read a command from the serial manager
    CommandError error = ESP32_serialManager2.readCommand(commandType, payload, payloadLength, receivedChecksum);

    if (error == CommandError::None) {
        // Process valid command
        switch(commandType){
            case CMD_ENCODER: {
                // Read current tick counts from each encoder (signed 32-bit)
                int32_t ticksFL = (int32_t)motorFL.read();
                int32_t ticksFR = (int32_t)motorFR.read();
                int32_t ticksRL = (int32_t)motorRL.read();
                int32_t ticksRR = (int32_t)motorRR.read();

                // Prepare outgoing payload:
                // bytes 0-3:  ticksFL (little-endian)
                // bytes 4-7:  ticksFR
                // bytes 8-11: ticksRL
                // bytes 12-15: ticksRR
                // bytes 16-19: timestamp (millis() truncated to 32-bit)
                uint8_t outPayload[20];

                // Helper lambda to pack a 32-bit signed integer into 4 bytes (little-endian)
                auto pack32 = [&](int32_t v, uint8_t *buf) {
                    buf[0] = (uint8_t)(v & 0xFF);
                    buf[1] = (uint8_t)((v >> 8) & 0xFF);
                    buf[2] = (uint8_t)((v >> 16) & 0xFF);
                    buf[3] = (uint8_t)((v >> 24) & 0xFF);
                };

                pack32(ticksFL, &outPayload[0]);
                pack32(ticksFR, &outPayload[4]);
                pack32(ticksRL, &outPayload[8]);
                pack32(ticksRR, &outPayload[12]);

                // Attach a 32-bit timestamp to help correlate readings on the host
                uint32_t ts = (uint32_t)millis();
                pack32((int32_t)ts, &outPayload[16]);

                // Send the status response back over Serial2
                ESP32_serialManager2.sendCommand(CMD_STATUS, outPayload, 20);
                break;
            }

            default:
                // Unknown/unhandled command: ignore
                break;

        };
    }
}