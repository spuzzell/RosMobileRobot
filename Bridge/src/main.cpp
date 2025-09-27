#include <Arduino.h>
#include "serial_manager.h"
#include "commands.h"
#include "pins.h"
#include "motors.h"



 // Top-level comment:
// This sketch runs on an ESP (Serial and Serial2). It requests encoder
// values from a Teensy over Serial2, runs a PI speed controller to convert
// desired tick/s setpoints into PWM commands, and forwards encoder/status
// to the PI (host) over Serial. It also accepts motor tick setpoints from
// the PI and enforces a timeout-based safety brake.
 
// ===============================================
// =============== Motor instances ===============
// ===============================================
Motor motorFL(0, PWM_LEFT_FRONT,  IN1_LEFT_FRONT,  IN2_LEFT_FRONT);   // Front Left
Motor motorFR(1, PWM_RIGHT_FRONT, IN1_RIGHT_FRONT, IN2_RIGHT_FRONT);  // Front Right
Motor motorRL(2, PWM_LEFT_REAR,   IN1_LEFT_REAR,   IN2_LEFT_REAR);    // Rear Left
Motor motorRR(3, PWM_RIGHT_REAR,  IN1_RIGHT_REAR,  IN2_RIGHT_REAR);   // Rear Right

// MotorDriver orchestrates the four motors. Pass pointers to the motor instances.
//MotorDriver driver(&motorFL, &motorFR, &motorRL, &motorRR);
int32_t pwmFL = 0, pwmFR = 0, pwmRL = 0, pwmRR = 0; // current PWM outputs for each motor
unsigned long lastMotorCommand = 0;                 // timestamp of last CMD_MOTOR_TICK received



// ===============================================
// ====== Serial configuration and managers ======
// ===============================================
// Timeouts and buffer sizes for serial communication
constexpr unsigned long TIMEOUT_MS_ESP = 50;       // timeout for host (ESP) reads
constexpr unsigned long TIMEOUT_MS_TEENY = 5;      // timeout for Teensy reads (fast)
constexpr uint8_t MAX_PAYLOAD_SIZE = 32;
constexpr int RX2_PIN = RX2_PIN_ESP_TO_TEENSY;
constexpr int TX2_PIN = TX2_PIN_ESP_TO_TEENSY;
constexpr unsigned long SERIAL_BAUD = 115200;

// SerialManager instances:
// - PI_serialManager communicates with the host/PI via Serial
// - TEENSY_serialManager2 communicates with the Teensy via Serial2
SerialManager PI_serialManager(Serial, TIMEOUT_MS_ESP, MAX_PAYLOAD_SIZE);
SerialManager TEENSY_serialManager2(Serial2, TIMEOUT_MS_TEENY, MAX_PAYLOAD_SIZE);

// ===============================================
// ============ Encoder configuration ============
// ===============================================
constexpr uint32_t ENCODER_PERIOD_MS = 10; // how often to request encoder data
uint32_t lastEncoderRequestMs = 0;
uint32_t lastSuccesfulEncoderRequestMs = 0;
uint32_t t0 = 0; // previous timestamp used to compute dt
uint32_t t1 = 0; // current timestamp used to compute dt

// encoder values (scaled/divided per the original code)
int32_t encFL=0, encFR=0, encRL=0,  encRR=0;
int32_t prev_encFL=0, prev_encFR=0, prev_encRL=0,  prev_encRR=0;

// desired setpoints (ticks per second) and measured ticks/sec
int16_t tickFL_perSec = 0, tickFR_perSec = 0, tickRL_perSec = 0, tickRR_perSec = 0;
int16_t measTickFL_perSec = 0, measTickFR_perSec = 0, measTickRL_perSec = 0, measTickRR_perSec = 0;
int16_t prevMeasFL = 0, prevMeasFR = 0, prevMeasRL = 0, prevMeasRR = 0;
bool gotValidMeas = true;       // set to false if measurement validation fails (unused here)
bool measBaselineSet = false;   // control flag used in the measurement initialization
const int16_t MAX_MEAS_DELTA = 1000; // not currently used but kept for validation logic


//=========== LOGGING ===========
uint32_t t_start = 0; // optional test start timestamp (unused in code)




void setup() {
    // initialize USB serial to host/PI
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {} // wait for Serial to be ready (useful for native USB MCUs)

     // initialize second UART to Teensy with explicit RX/TX pins
    Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
    lastMotorCommand=millis(); // initialize watchdog timer

}

void loop() {

    // Local buffers for incoming/outgoing serial commands
    uint8_t commandType;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint8_t payloadLength;
    uint8_t receivedChecksum;

    uint8_t commandType_Teensy;
    uint8_t payload_Teensy[MAX_PAYLOAD_SIZE];
    uint8_t payloadLength_Teensy;
    uint8_t receivedChecksum_Teensy;

    CommandError error = CommandError::Timeout;
    CommandError errorT = CommandError::Timeout;

    // Check for incoming command from PI (host)
    if (Serial.available()) {
        error = PI_serialManager.readCommand(commandType, payload, payloadLength, receivedChecksum);
    }

    // Periodically request encoder/status from the Teensy
    if ((millis() - lastSuccesfulEncoderRequestMs) >= ENCODER_PERIOD_MS) {
        // send a CMD_ENCODER request with no payload (adjust if your protocol expects something else)
        TEENSY_serialManager2.sendCommand(CMD_ENCODER, nullptr, 0);

        lastEncoderRequestMs = millis();

        // small delay to let Teensy respond; the SerialManager read uses its own timeout too
        delay(2);

        if (Serial2.available()) {
            errorT = TEENSY_serialManager2.readCommand(commandType_Teensy, payload_Teensy, payloadLength_Teensy, receivedChecksum_Teensy);
        }


        if (errorT == CommandError::None) {
            // Successful read from Teensy
            lastSuccesfulEncoderRequestMs = lastEncoderRequestMs;


            switch(commandType_Teensy){
                case CMD_STATUS:
                {
                    // helper to read little-endian int32 from payload_Teensy
                    auto readInt32LE = [&](uint8_t idx)->int32_t{
                        uint32_t b0 = payload_Teensy[idx];
                        uint32_t b1 = payload_Teensy[idx+1];
                        uint32_t b2 = payload_Teensy[idx+2];
                        uint32_t b3 = payload_Teensy[idx+3];
                        uint32_t u = (b0) | (b1 << 8) | (b2 << 16) | (b3 << 24);
                        return (int32_t)u;
                    };

                    // Expect at least 16 bytes (4 x int32 encoders)
                    if (payloadLength_Teensy >= 16) {
                        // shift previous samples
                        t0 = t1;
                        prev_encFL = encFL;
                        prev_encFR = encFR;
                        prev_encRL = encRL;
                        prev_encRR = encRR;

                        // read raw encoder counts and apply a divide-by-4 (specific to hardware/firmware)
                        t1 = lastEncoderRequestMs;
                        encFL = (readInt32LE(0))/4;
                        encFR = (readInt32LE(4))/4;
                        encRL = (readInt32LE(8))/4;
                        encRR = (readInt32LE(12))/4;

                        // Compute measured ticks/second from encoder deltas
                        uint32_t dt_ms = t1 - t0;            // handles millis() wrap by unsigned math
                        if (dt_ms > 0 && t0 != 0) {          // skip first sample
                            int32_t dFL = encFL - prev_encFL;
                            int32_t dFR = encFR - prev_encFR;
                            int32_t dRL = encRL - prev_encRL;
                            int32_t dRR = encRR - prev_encRR;

                            // convert delta over dt to ticks/sec (clamp to int16 range)
                            auto toPerSec = [&](int32_t d) -> int16_t {
                                int32_t v = (int32_t)((int64_t)d * 1000 / (int32_t)dt_ms);
                                if (v > 32767)  v = 32767;
                                if (v < -32768) v = -32768;
                                return (int16_t)v;
                            };

                            if (!measBaselineSet) {
                                // on first valid measurement, set measured values
                                measTickFL_perSec = toPerSec(dFL);
                                measTickFR_perSec = toPerSec(dFR);
                                measTickRL_perSec = toPerSec(dRL);
                                measTickRR_perSec = toPerSec(dRR);
                            }else{
                                // if baseline flag set, clear it (legacy behavior)
                                measBaselineSet=false;
                            }

                        } else {
                            // no valid dt yet -> zero measurements
                            measTickFL_perSec = measTickFR_perSec = measTickRL_perSec = measTickRR_perSec = 0;
                        }
                        // store last measured values (for debugging or smoothing if needed)
                        prevMeasFL = measTickFL_perSec;
                        prevMeasFR = measTickFR_perSec;
                        prevMeasRL = measTickRL_perSec;
                        prevMeasRR = measTickRR_perSec;
                        
                    }


                    if (gotValidMeas){
                        Serial.printf("PI RAN\n");
                         // PI speed controller (ticks/sec -> PWM). Tune Kp, Ki and PWM_MAX as needed.
                        static float iFL = 0, iFR = 0, iRL = 0, iRR = 0;
                        static float prev_uFL = 0, prev_uFR = 0, prev_uRL = 0, prev_uRR = 0;
                        const float Kp = 0.6f;   // proportional gain
                        const float Ki = 0.005f;   // integral gain (per second)
                        const int PWM_MAX = 255;  // adjust if your driver uses a different PWM range
                        const float MAX_PWM_CHANGE_PER_SEC = 200.0f;

                        // sample time (s)
                        float dt = (t0 != 0 && t1 >= t0)
                                   ? (float)(t1 - t0) / 1000.0f
                                   : (float)ENCODER_PERIOD_MS / 1000.0f;
                        

                        // PI control step that includes:
                        // - proportional + integral action
                        // - anti-windup (conditional integration)
                        // - deadband
                        // - output saturation
                        // - slew rate limiting
                        auto stepPI = [&](int16_t sp, int16_t meas, float &I, float &prev_u)->int32_t {
                            float e = (float)sp - (float)meas;

                            // propose integral update
                            float I_new = I + Ki * e * dt;

                            // unsaturated control
                            float u_unsat = Kp * e + I_new;

                            // saturate
                            float u = u_unsat;
                            if (u > PWM_MAX) u = (float)PWM_MAX;
                            if (u < -PWM_MAX) u = -(float)PWM_MAX;

                            // apply deadband BEFORE slew limit so the limiter constrains the final command
                            const int DB = 5;
                            if (u > -DB && u < DB) u = 0.0f;

                            // slew limiting
                            float max_delta = MAX_PWM_CHANGE_PER_SEC * dt;
                            float delta = u - prev_u;
                            bool slew_limited = false;
                            if (delta > max_delta) {
                                u = prev_u + max_delta;
                                slew_limited = true;
                            } else if (delta < -max_delta) {
                                u = prev_u - max_delta;
                                slew_limited = true;
                            }

                            // conditional integration (anti-windup)
                            bool within = (u_unsat <= PWM_MAX && u_unsat >= -PWM_MAX);
                            bool relieving = (u_unsat > PWM_MAX && e < 0) || (u_unsat < -PWM_MAX && e > 0);
                            if (within || relieving) {
                                const float I_LIMIT = 10000.0f;
                                if (I_new > I_LIMIT) I = I_LIMIT;
                                else if (I_new < -I_LIMIT) I = -I_LIMIT;
                                else I = I_new;
                            }

                            prev_u = u;

                            return (int32_t)u;
                        };

                        // compute PWM commands for each wheel
                        pwmFL = stepPI(tickFL_perSec, measTickFL_perSec, iFL, prev_uFL);
                        pwmFR = stepPI(tickFR_perSec, measTickFR_perSec, iFR, prev_uFR);
                        pwmRL = stepPI(tickRL_perSec, measTickRL_perSec, iRL, prev_uRL);
                        pwmRR = stepPI(tickRR_perSec, measTickRR_perSec, iRR, prev_uRR);

                        // store prev_u again (redundant given stepPI sets it, but kept for clarity)
                        prev_uFL = (float)pwmFL;
                        prev_uFR = (float)pwmFR;
                        prev_uRL = (float)pwmRL;
                        prev_uRR = (float)pwmRR;

                        // reset integrator when setpoint is zero to avoid windup
                        if (tickFL_perSec == 0) iFL = 0;
                        if (tickFR_perSec == 0) iFR = 0;
                        if (tickRL_perSec == 0) iRL = 0;
                        if (tickRR_perSec == 0) iRR = 0;
                    
                    }

                    // Forward the Teensy's status payload to the PI and append a timestamp.
                    // This preserves the original payload and appends 4 bytes of timestamp
                    // if there is room in the MAX_PAYLOAD_SIZE buffer.
                    {
                        uint8_t combinedPayload[MAX_PAYLOAD_SIZE];
                        uint8_t combinedLength = payloadLength_Teensy;
                        if (combinedLength > MAX_PAYLOAD_SIZE) combinedLength = MAX_PAYLOAD_SIZE; // safety
                        // copy original payload (up to available space)
                        for (uint8_t i = 0; i < combinedLength; ++i) combinedPayload[i] = payload_Teensy[i];

                        // attempt to append timestamp (4 bytes)
                        if ((uint8_t)(payloadLength_Teensy + 4) <= MAX_PAYLOAD_SIZE) {
                            uint32_t ts = lastEncoderRequestMs;
                            combinedPayload[combinedLength + 0] = (uint8_t)(ts & 0xFF);
                            combinedPayload[combinedLength + 1] = (uint8_t)((ts >> 8) & 0xFF);
                            combinedPayload[combinedLength + 2] = (uint8_t)((ts >> 16) & 0xFF);
                            combinedPayload[combinedLength + 3] = (uint8_t)((ts >> 24) & 0xFF);
                            combinedLength += 4;
                        } 

                        PI_serialManager.sendCommand(CMD_STATUS, combinedPayload, combinedLength);
                    }
                    
                    break;
                }

                default:
                    // ignore other Teensy message types for now
                    break;    
            }
        }
    }

    // Process commands received from the PI (host)
    if (error == CommandError::None) {
        switch(commandType){
            case CMD_MOTOR_TICK: {

                lastMotorCommand=millis(); // update watchdog timestamp
                // helper to read signed int16 little-endian
                auto readInt16LE_raw = [&](uint8_t idx)->int16_t {
                        uint16_t lo = payload[idx];
                        uint16_t hi = payload[idx + 1];
                        return (int16_t)(lo | (hi << 8));
                };
                
                if(payloadLength!=8){break;} // expect exactly four int16 values (4*2=8)
                 // set desired wheel tick/s setpoints (these drive the PI controller above)
                tickFL_perSec = readInt16LE_raw(0);
                tickFR_perSec = readInt16LE_raw(2);
                tickRL_perSec = readInt16LE_raw(4);
                tickRR_perSec = readInt16LE_raw(6);
 
                break;
            }

            default:
                // ignore unknown host commands
                break;

        };
    }


    // Safety timeout: if we haven't received a motor command in 2 seconds,
    // stop all motors and clear setpoints to avoid runaway.
    if((millis() - lastMotorCommand) > 2000){
        pwmFL = 0;
        pwmFR = 0;
        pwmRL = 0;
        pwmRR = 0;
        tickFL_perSec = 0;
        tickFR_perSec = 0;
        tickRL_perSec = 0;
        tickRR_perSec = 0;
        motorFL.brake();
        motorFR.brake();
        motorRL.brake();
        motorRR.brake();
    }
    else{
        // otherwise, apply computed PWM values to the motor drivers
        motorFL.setSpeed(pwmFL);
        motorFR.setSpeed(pwmFR);
        motorRL.setSpeed(pwmRL);
        motorRR.setSpeed(pwmRR);
    }
}


