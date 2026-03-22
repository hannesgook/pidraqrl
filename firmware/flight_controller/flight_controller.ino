// Copyright (c) 2025 Hannes Göök
// MIT License - PidraQRL Project
// https://github.com/hannesgook/pqrl2

#include <NimBLEDevice.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <math.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include "driver/mcpwm.h"
#include <SPI.h>
#include <LoRa.h> // https://github.com/sandeepmistry/arduino-LoRa

// =============================================================================
// --- DUAL RECEIVER MODE ------------------------------------------------------
//
//   Both BLE and LoRa are always active simultaneously.
//   BLE TX (IMU telemetry) is always on.
//
//   PRIORITY RULE:
//     The RL script sends a 'C' (claim) packet over BLE:
//       C 0x01  →  BLE claims control, LoRa packets are IGNORED
//       C 0x00  →  BLE releases control, LoRa packets are APPLIED
//
//     While BLE is claimed, the heartbeat keeps sending S packets with
//     RL commands. While released, the heartbeat still sends safe-defaults
//     (keeping BLE telemetry alive) but LoRa drives the drone.
//
//     A timeout (BLE_CLAIM_TIMEOUT_MS) auto-releases if the RL script
//     crashes without sending a release - safety net.
//
// =============================================================================

// BLE claim timeout
// If the RL script crashes mid-training without sending C 0x00, auto-release after this many ms so LoRa can take over again.
#define BLE_CLAIM_TIMEOUT_MS 2000UL

// Pin assignments
static const int ESC_PIN_FL = 25;
static const int ESC_PIN_FR = 26;
static const int ESC_PIN_BL = 27;
static const int ESC_PIN_BR = 32;

// LoRa pin assignments
#define LORA_NSS_PIN 5
#define LORA_RST_PIN 14
#define LORA_DIO0_PIN 2
#define LORA_FREQUENCY 433E6

// BLE config
#define DEVICE_NAME "ESP32-FC"

#define SERVICE_UUID "fc000000-0000-0000-0000-000000000001"
#define TX_CHARACTERISTIC "fc000000-0000-0000-0000-000000000002"
#define RX_CHARACTERISTIC "fc000000-0000-0000-0000-000000000003"

#define BLE_MTU 517
#define CONN_INTERVAL_MIN 6
#define CONN_INTERVAL_MAX 6
#define CONN_LATENCY 0
#define SUPERVISION_TIMEOUT 100

// Timing
#define IMU_HZ 250
#define MOTOR_HZ 50
#define BLE_SEND_HZ 100

#define IMU_PERIOD_US (1000000UL / IMU_HZ)
#define MOTOR_PERIOD_MS (1000UL    / MOTOR_HZ)
#define BLE_PERIOD_US (1000000UL / BLE_SEND_HZ)

// IMU
#define MPU6050_ADDR 0x68
#define MPU_GYRO_FS_SEL 0x10
#define MPU_GYRO_SCALE_LSB 32.8f

// Flight limits
static const float STICK_MAX_ANGLE_DEG = 30.0f;
static const float STICK_MAX_YAW_RATE_DPS = 120.0f;
static const int   IDLE_CUTOFF_US = 1050;
static const int   MAX_THROTTLE = 2000;

// Failsafe
// If no valid packet from the ACTIVE source within this window, motors idle.
#define FAILSAFE_MS 1000UL

// Ring buffers (different for BLE and LoRa)
#define RX_RING_SIZE 512

static uint8_t ble_ring[RX_RING_SIZE];
static volatile uint16_t ble_ring_head = 0;
static volatile uint16_t ble_ring_tail = 0;
static portMUX_TYPE bleBufMux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t lora_ring[RX_RING_SIZE];
static volatile uint16_t lora_ring_head = 0;
static volatile uint16_t lora_ring_tail = 0;
static portMUX_TYPE loraBufMux = portMUX_INITIALIZER_UNLOCKED;

// motors
static volatile int motorFL = 1000;
static volatile int motorFR = 1000;
static volatile int motorBL = 1000;
static volatile int motorBR = 1000;
static portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

Madgwick filter;

// BLE
static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pTxCharacteristic = nullptr;
static NimBLECharacteristic* pRxCharacteristic = nullptr;
static volatile bool deviceConnected = false;
static volatile bool oldDeviceConnected = false;

// BLE claiming
// Set by 'C' packet: true = BLE controls drone, false = LoRa controls drone.
// Protected by claimMux for atomic read/write from multiple tasks.
static volatile bool bleClaimed = false;
static volatile uint32_t bleClaimTimeMs = 0; // millis() when last C 0x01 received
static portMUX_TYPE claimMux = portMUX_INITIALIZER_UNLOCKED;

// rx flags / failsafe
static volatile bool rxNewPacket = false;
static volatile uint32_t rxLastGoodPacketMs = 0;

// hz tracking
#define STATS_INTERVAL_MS 500UL
static volatile uint32_t hzImuCount = 0;
static volatile uint32_t hzMotorCount = 0;
static volatile uint32_t hzBleTxCount = 0;
static volatile uint32_t hzLoraRxCount = 0;
static volatile uint32_t hzBleRxGoodCount = 0;
static volatile uint32_t hzLoraRxGoodCount= 0;
static volatile uint32_t hzLoraDropCount = 0;

static portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE stickMux = portMUX_INITIALIZER_UNLOCKED;

// imu
static volatile float rollDeg = 0.0f;
static volatile float pitchDeg = 0.0f;
static volatile float yawDeg = 0.0f;
static volatile float gx_dps = 0.0f;
static volatile float gy_dps = 0.0f;
static volatile float gz_dps = 0.0f;
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;

static volatile float rollOffset = 0.0f;
static volatile float pitchOffset = 0.0f;
static volatile float yawOffset = 0.0f;

static volatile float targetRollDegTelemetry = 0.0f;

static volatile float stickRoll = 0.0f;
static volatile float stickPitch = 0.0f;
static volatile float stickYaw = 0.0f;
static volatile int cmdThrottleUs = 1000;

static volatile float angleKpVal = 10.0f;

// arming
static bool  armed = false;
static float rollArmRef = 0.0f;
static float pitchArmRef = 0.0f;
static float yawArmRef = 0.0f;

static volatile uint32_t imuCount = 0;
static volatile uint32_t bleCount = 0;
static volatile uint32_t motorCount = 0;

// timing / tasks
static TaskHandle_t imuTaskHandle = nullptr;
static hw_timer_t*  imuHwTimer = nullptr;

volatile uint32_t idleCount0 = 0;
volatile uint32_t idleCount1 = 0;

// rx
static uint32_t rxBlePacketCount = 0;
static uint32_t rxLoraPacketCount = 0;
static uint32_t rxBadChkCount = 0;
static uint32_t rxShortCount = 0;
static int   rx_throttleUs = 1000;
static float rx_roll = 0, rx_pitch = 0, rx_yaw = 0;
static float rx_P = 0, rx_I = 0, rx_D = 0;
static float rx_P2 = 0, rx_I2 = 0, rx_D2 = 0;
static float rx_aKp = 0;
static volatile char rxActiveSource = '-';

// Biquad Low-Pass Filter
#define ACCEL_LPF_CUTOFF_HZ 5.0f
#define GYRO_LPF_CUTOFF_HZ 10.0f

struct BiquadLPF {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2;
    float y1, y2;

    void begin(float fc, float fs) {
        float omega = 2.0f * M_PI * fc / fs;
        float cosW = cosf(omega);
        float sinW = sinf(omega);
        float alpha = sinW / (2.0f * 0.7071f);
        float a0 = 1.0f + alpha;
        b0 = ((1.0f - cosW) / 2.0f) / a0;
        b1 =  (1.0f - cosW)         / a0;
        b2 = b0;
        a1 = (-2.0f * cosW)         / a0;
        a2 = (1.0f - alpha)         / a0;
        x1 = x2 = y1 = y2 = 0.0f;
    }

    inline float update(float x0) {
        float y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
        return y0;
    }
};

static BiquadLPF lpfAX, lpfAY, lpfAZ;
static BiquadLPF lpfGX, lpfGY, lpfGZ;

// MCPWM ESC driver
struct EscChannel {
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_operator_t op;
};

static const EscChannel ESC_FL = { MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A };
static const EscChannel ESC_FR = { MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B };
static const EscChannel ESC_BL = { MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A };
static const EscChannel ESC_BR = { MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B };

static inline uint16_t clampEscUs(uint16_t us) {
    if (us < 1000) return 1000;
    if (us > 2000) return 2000;
    return us;
}

void escWriteUs(const EscChannel& esc, uint16_t us) {
    us = clampEscUs(us);
    mcpwm_set_duty_in_us(esc.unit, esc.timer, esc.op, us);
}

void setupMotors() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ESC_PIN_FL);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, ESC_PIN_FR);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, ESC_PIN_BL);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, ESC_PIN_BR);

    mcpwm_config_t cfg = {};
    cfg.frequency = 50;
    cfg.cmpr_a = 0.0f;
    cfg.cmpr_b = 0.0f;
    cfg.counter_mode = MCPWM_UP_COUNTER;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cfg);

    escWriteUs(ESC_FL, 1000);
    escWriteUs(ESC_FR, 1000);
    escWriteUs(ESC_BL, 1000);
    escWriteUs(ESC_BR, 1000);

    delay(3000);
}

float constrainf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

int clampUs(int us) {
    if (us < 1000) return 1000;
    if (us > MAX_THROTTLE) return MAX_THROTTLE;
    return us;
}

void setMotorTargets(int fl, int fr, int bl, int br) {
    portENTER_CRITICAL(&motorMux);
    motorFL = clampUs(fl);
    motorFR = clampUs(fr);
    motorBL = clampUs(bl);
    motorBR = clampUs(br);
    portEXIT_CRITICAL(&motorMux);
}

static inline void write_f32_le(uint8_t* p, float f) {
    union { float f; uint8_t b[4]; } u;
    u.f = f;
    p[0] = u.b[0]; p[1] = u.b[1]; p[2] = u.b[2]; p[3] = u.b[3];
}

static inline float unmap8(uint8_t v, float outMin, float outMax) {
    return outMin + (v / 255.0f) * (outMax - outMin);
}

static bool isBleInControl() {
    bool claimed;
    uint32_t claimTime;
    portENTER_CRITICAL(&claimMux);
    claimed = bleClaimed;
    claimTime = bleClaimTimeMs;
    portEXIT_CRITICAL(&claimMux);

    if (!claimed) return false;

    // auto-release if RL script crashed without sending C 0x00
    if ((millis() - claimTime) > BLE_CLAIM_TIMEOUT_MS) {
        portENTER_CRITICAL(&claimMux);
        bleClaimed = false;
        portEXIT_CRITICAL(&claimMux);
        Serial.println("[CLAIM] BLE claim timed out - auto-released to LoRa");
        return false;
    }
    return true;
}

// pid
typedef struct {
    float kp, ki, kd;
    float iTerm, lastRate;
    float outMin, outMax;
} PID_t;

float pidRate(PID_t& pid, float targetDps, float measuredDps, float dt);
float pidRate(PID_t& pid, float targetDps, float measuredDps, float dt) {
    float error = targetDps - measuredDps;
    if (pid.ki > 0.0001f) {
        pid.iTerm += pid.ki * error * dt;
        pid.iTerm = constrainf(pid.iTerm, pid.outMin, pid.outMax);
    }
    float dTerm = 0.0f;
    if (pid.kd > 0.0001f && dt > 0.0f) {
        dTerm = -pid.kd * (measuredDps - pid.lastRate) / dt;
    }
    pid.lastRate = measuredDps;
    return constrainf(pid.kp * error + pid.iTerm + dTerm, pid.outMin, pid.outMax);
}

PID_t rollRatePID = { 0.93f, 0.0f, 0.0f, 0.0f, 0.0f, -250.0f, 250.0f };
PID_t pitchRatePID = { 0.93f, 0.0f, 0.0f, 0.0f, 0.0f, -250.0f, 250.0f };
PID_t yawRatePID = { 0.0f,  0.0f, 0.0f, 0.0f, 0.0f, -250.0f, 250.0f };

// MPU-6050
void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

void mpuInit() {
    mpuWrite(0x6B, 0x00);
    delay(100);
    mpuWrite(0x1A, 0x06);
    mpuWrite(0x19, 0x03);
    mpuWrite(0x1B, MPU_GYRO_FS_SEL);
    mpuWrite(0x1C, 0x00);
    delay(100);
}

bool mpuReadRaw(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU6050_ADDR, 14, true) != 14) return false;
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
    return true;
}

void mpuCalibrateGyro() {
    Serial.println("[IMU] Calibrating gyro...");
    for (int i = 0; i < 50; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpuReadRaw(ax, ay, az, gx, gy, gz);
        delay(4);
    }
    long sumX = 0, sumY = 0, sumZ = 0;
    int good = 0;
    for (int i = 0; i < 2000; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
            sumX += gx; sumY += gy; sumZ += gz;
            good++;
        }
        delay(4);
    }
    if (good > 0) {
        gyroBiasX = (float)sumX / good;
        gyroBiasY = (float)sumY / good;
        gyroBiasZ = (float)sumZ / good;
    }
    Serial.printf("[IMU] Gyro cal done (%d/2000 good): %.2f %.2f %.2f\n", good, gyroBiasX, gyroBiasY, gyroBiasZ);
}

static void ring_push(uint8_t* ring, volatile uint16_t& head, volatile uint16_t& tail, portMUX_TYPE& mux, const uint8_t* data, uint16_t len) {
    portENTER_CRITICAL(&mux);
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (head + 1) % RX_RING_SIZE;
        if (next == tail) break;
        ring[head] = data[i];
        head = next;
    }
    portEXIT_CRITICAL(&mux);
}

static uint16_t ring_pop(uint8_t* ring, volatile uint16_t& head, volatile uint16_t& tail, portMUX_TYPE& mux, uint8_t* out, uint16_t max_len) {
    uint16_t count = 0;
    portENTER_CRITICAL(&mux);
    while (tail != head && count < max_len) {
        out[count++] = ring[tail];
        tail = (tail + 1) % RX_RING_SIZE;
    }
    portEXIT_CRITICAL(&mux);
    return count;
}

static void ble_ring_push(const uint8_t* data, uint16_t len) {
    ring_push(ble_ring, ble_ring_head, ble_ring_tail, bleBufMux, data, len);
}
static uint16_t ble_ring_pop(uint8_t* out, uint16_t max_len) {
    return ring_pop(ble_ring, ble_ring_head, ble_ring_tail, bleBufMux, out, max_len);
}
static void lora_ring_push(const uint8_t* data, uint16_t len) {
    ring_push(lora_ring, lora_ring_head, lora_ring_tail, loraBufMux, data, len);
}
static uint16_t lora_ring_pop(uint8_t* out, uint16_t max_len) {
    return ring_pop(lora_ring, lora_ring_head, lora_ring_tail, loraBufMux, out, max_len);
}

static void applyParsedSPacket(char source, int newThrottleUs, float newStickRoll, float newStickPitch, float newStickYaw, float newP, float newI, float newD, float newP2, float newI2, float newD2, float newAngleKp)
{
    portENTER_CRITICAL(&stickMux);
    cmdThrottleUs = newThrottleUs;
    stickRoll = newStickRoll;
    stickPitch = newStickPitch;
    stickYaw = newStickYaw;
    targetRollDegTelemetry = newStickRoll * STICK_MAX_ANGLE_DEG;
    portEXIT_CRITICAL(&stickMux);

    rollRatePID.kp = newP;  rollRatePID.ki = newI;  rollRatePID.kd = newD;
    pitchRatePID.kp = newP;  pitchRatePID.ki = newI;  pitchRatePID.kd = newD;
    yawRatePID.kp = newP2; yawRatePID.ki = newI2; yawRatePID.kd = newD2;
    angleKpVal = newAngleKp;

    rx_throttleUs = newThrottleUs;
    rx_roll = newStickRoll;  rx_pitch = newStickPitch; rx_yaw = newStickYaw;
    rx_P = newP;  rx_I = newI;  rx_D = newD;
    rx_P2 = newP2; rx_I2 = newI2; rx_D2 = newD2;
    rx_aKp = newAngleKp;

    rxNewPacket = true;
    rxLastGoodPacketMs = millis();
    rxActiveSource = source;

    if (source == 'B') {
        rxBlePacketCount++;
        hzBleRxGoodCount++;
    } else {
        rxLoraPacketCount++;
        hzLoraRxGoodCount++;
    }
}

// true if a valid packet was parsed and applied
bool processRxFrom(char source) {
    uint8_t buf[68];
    uint16_t len;

    if (source == 'B') {
        len = ble_ring_pop(buf, sizeof(buf));
    } else {
        len = lora_ring_pop(buf, sizeof(buf));
    }
    if (len == 0) return false;

    uint8_t* payload = buf;
    uint16_t payloadLen = len;
    if (len >= 38 && buf[0] != 'S' && buf[0] != 'C' && buf[4] == 'S') {
        payload = buf + 4;
        payloadLen = len - 4;
    }

    // 'C' packet (BLE claim/release, 2 bytes)
    // Only from BLE source, LoRa should never send 'C'.
    if (payloadLen >= 2 && payload[0] == 'C' && source == 'B') {
        bool claim = (payload[1] != 0);
        portENTER_CRITICAL(&claimMux);
        bleClaimed = claim;
        bleClaimTimeMs = millis();
        portEXIT_CRITICAL(&claimMux);
        Serial.printf("[CLAIM] BLE %s control\n", claim ? "CLAIMED" : "RELEASED");
        return true;
    }

    // BLE S/R gate
    // If this packet came from BLE but BLE has NOT claimed control, discard S/R packets so they don't compete with LoRa.
    if (source == 'B' && !isBleInControl()) {
        return false;
    }

    // 'R' packet (BLE RL command, 17 bytes)
    if (payloadLen >= 17 && payload[0] == 'R') {
        float newStickRoll, newAngleKp, newRateKp;
        float newThrottleUsF;
        memcpy(&newStickRoll, payload + 1,  4);
        memcpy(&newAngleKp, payload + 5,  4);
        memcpy(&newRateKp, payload + 9,  4);
        memcpy(&newThrottleUsF, payload + 13, 4);

        int newThrottleUs = (int)constrainf(newThrottleUsF, 1000.0f, 2000.0f);
        newStickRoll = constrainf(newStickRoll, -1.0f, 1.0f);
        newAngleKp = constrainf(newAngleKp, 2.0f, 20.0f);
        newRateKp = constrainf(newRateKp, 0.3f, 2.0f);

        applyParsedSPacket(source, newThrottleUs, newStickRoll, 0.0f, 0.0f, newRateKp, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, newAngleKp);
        return true;
    }

    // 'S' packet (34 bytes + checksum)
    {
        if (payloadLen < 34 || payload[0] != 'S') {
            rxShortCount++;
            return false;
        }
        uint8_t chk = 0;
        for (int i = 1; i < 33; i++) chk ^= payload[i];
        if (chk != payload[33]) {
            rxBadChkCount++;
            return false;
        }

        int newThrottleUs = (int)unmap8(payload[1], 1000.0f, 2000.0f);
        float newStickRoll = unmap8(payload[2], -1.0f, 1.0f);
        float newStickPitch = unmap8(payload[3], -1.0f, 1.0f);
        float newStickYaw = unmap8(payload[4], -1.0f, 1.0f);

        float newP, newI, newD, newP2, newI2, newD2, newAngleKp;
        memcpy(&newP, payload +  5, 4);
        memcpy(&newI, payload +  9, 4);
        memcpy(&newD, payload + 13, 4);
        memcpy(&newP2, payload + 17, 4);
        memcpy(&newI2, payload + 21, 4);
        memcpy(&newD2, payload + 25, 4);
        memcpy(&newAngleKp, payload + 29, 4);

        newThrottleUs = (int)constrainf((float)newThrottleUs, 1000.0f, 2000.0f);
        newStickRoll = constrainf(newStickRoll,  -1.0f, 1.0f);
        newStickPitch = constrainf(newStickPitch, -1.0f, 1.0f);
        newStickYaw = constrainf(newStickYaw,   -1.0f, 1.0f);
        newP = constrainf(newP, 0.0f, 10.0f);
        newI = constrainf(newI, 0.0f, 5.0f);
        newD = constrainf(newD, 0.0f, 1.0f);
        newP2 = constrainf(newP2, 0.0f, 10.0f);
        newI2 = constrainf(newI2, 0.0f, 5.0f);
        newD2 = constrainf(newD2, 0.0f, 1.0f);
        newAngleKp = constrainf(newAngleKp, 2.0f, 20.0f);

        applyParsedSPacket(source, newThrottleUs, newStickRoll, newStickPitch, newStickYaw, newP, newI, newD, newP2, newI2, newD2, newAngleKp);
        return true;
    }
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override {
        deviceConnected = true;
        s->updateConnParams(connInfo.getAddress(), CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, CONN_LATENCY, SUPERVISION_TIMEOUT);
        Serial.println("[BLE] Connected");
    }
    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
        deviceConnected = false;
        // Auto-release BLE claim on disconnect so LoRa takes over immediately
        portENTER_CRITICAL(&claimMux);
        bleClaimed = false;
        portEXIT_CRITICAL(&claimMux);
        Serial.println("[BLE] Disconnected - BLE claim auto-released");
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        const uint8_t* data = pChar->getValue().data();
        size_t len = pChar->getValue().length();
        if (data && len > 0) {
            ble_ring_push(data, (uint16_t)len);
        }
    }
};

bool ble_send(const uint8_t* data, uint16_t len) {
    if (!deviceConnected || len == 0) return false;
    pTxCharacteristic->setValue(data, len);
    pTxCharacteristic->notify();
    return true;
}

void setupBLE() {
    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setMTU(BLE_MTU);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        TX_CHARACTERISTIC, NIMBLE_PROPERTY::NOTIFY);

    pRxCharacteristic = pService->createCharacteristic(
        RX_CHARACTERISTIC,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    pService->start();

    NimBLEAdvertisementData advData;
    advData.setName(DEVICE_NAME);
    advData.addServiceUUID(SERVICE_UUID);

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setAdvertisementData(advData);
    pAdv->start();

    Serial.println("[BLE] Advertising - TX + RX active");
}

void sendIMUPacket() {
    float lr, lp, ly, lgx, lgy, lgz, lTargetRoll;
    portENTER_CRITICAL(&imuMux);
    lr = rollDeg  - rollOffset;
    lp = pitchDeg - pitchOffset;
    ly = yawDeg   - yawOffset;
    lgx = gx_dps; lgy = gy_dps; lgz = gz_dps;
    portEXIT_CRITICAL(&imuMux);

    portENTER_CRITICAL(&stickMux);
    lTargetRoll = targetRollDegTelemetry;
    portEXIT_CRITICAL(&stickMux);

    uint32_t ts = micros();

    uint8_t pkt[33];
    pkt[0] = 'I';
    write_f32_le(pkt +  1, lr);
    write_f32_le(pkt +  5, lp);
    write_f32_le(pkt +  9, ly);
    write_f32_le(pkt + 13, lgx);
    write_f32_le(pkt + 17, lgy);
    write_f32_le(pkt + 21, lgz);
    write_f32_le(pkt + 25, lTargetRoll);
    pkt[29] = (ts >>  0) & 0xFF;
    pkt[30] = (ts >>  8) & 0xFF;
    pkt[31] = (ts >> 16) & 0xFF;
    pkt[32] = (ts >> 24) & 0xFF;

    if (ble_send(pkt, sizeof(pkt))) {
        portENTER_CRITICAL(&imuMux);
        bleCount++;
        portEXIT_CRITICAL(&imuMux);
        hzBleTxCount++;
    }
}

void setupLoRa() {
    LoRa.setPins(LORA_NSS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("[LORA] FATAL: module not found - check wiring/pins!");
        while (true) { delay(1000); }
    }
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.enableCrc();
    Serial.println("[LORA] Ready - RX @ 433 MHz");
}

void loraRxTask(void* parameter) {
    while (true) {
        int packetSize = LoRa.parsePacket();
        if (packetSize > 0) {
            uint8_t pkt[68];
            uint16_t idx = 0;
            while (LoRa.available() && idx < sizeof(pkt)) {
                pkt[idx++] = (uint8_t)LoRa.read();
            }
            if (idx > 0) {
                lora_ring_push(pkt, idx);
                hzLoraRxCount++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// drain LoRa ring buffer
// Only applies commands when BLE has NOT claimed control.
void loraDrainTask(void* parameter) {
    while (true) {
        if (isBleInControl()) {
            // BLE has claimed control, drain and DISCARD LoRa packets
            uint8_t discard[68];
            uint16_t discarded = lora_ring_pop(discard, sizeof(discard));
            if (discarded > 0) {
                hzLoraDropCount++;
            }
        } else {
            // BLE released
            processRxFrom('L');
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ble drain always drains BLE ring buffer.
// 'C' packets are handled inside processRxFrom.
// 'S' packets are only applied to flight state when BLE has claimed control
// otherwise they are parsed but NOT applied (telemetry stays alive).
void bleDrainTask(void* parameter) {
    while (true) {
        // Always drain, 'C' packets must always be processed.
        // 'S'/'R' packets from BLE are always applied because if BLE is sending them, it has claimed control (the RL script sends C 0x01 before starting and C 0x00 after stopping).
        processRxFrom('B');
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void IRAM_ATTR imuHwTimerISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(imuTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void imuTask(void* parameter) {
    static float lastGx = 0.0f, lastGy = 0.0f, lastGz = 0.0f;
    static const float DT = 1.0f / (float)IMU_HZ;
    static float yawHeldDeg = 0.0f;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int16_t axr, ayr, azr, gxr, gyr, gzr;
        if (mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr)) {
            float ax =  axr / 16384.0f;
            float ay =  ayr / 16384.0f;
            float az =  azr / 16384.0f;
            float gxRaw = (gxr - gyroBiasX) / MPU_GYRO_SCALE_LSB;
            float gyRaw = (gyr - gyroBiasY) / MPU_GYRO_SCALE_LSB;
            float gzRaw = (gzr - gyroBiasZ) / MPU_GYRO_SCALE_LSB;

            ax = lpfAX.update(ax);
            ay = lpfAY.update(ay);
            az = lpfAZ.update(az);
            float gx_f = lpfGX.update(gxRaw);
            float gy_f = lpfGY.update(gyRaw);
            float gz_f = lpfGZ.update(gzRaw);

            lastGx = gx_f; lastGy = gy_f; lastGz = gz_f;
            filter.updateIMU(gx_f, gy_f, gz_f, ax, ay, az);
        } else {
            filter.updateIMU(lastGx, lastGy, lastGz, 0, 0, 1);
        }

        float iRoll = -filter.getRoll();
        float iPitch =  filter.getPitch();
        float iYaw = -filter.getYaw();
        float iGx = -lastGx;
        float iGy =  lastGy;
        float iGz =  lastGz;

        portENTER_CRITICAL(&imuMux);
        rollDeg = iRoll; pitchDeg = iPitch; yawDeg = iYaw;
        gx_dps = iGx; gy_dps = iGy; gz_dps = iGz;
        imuCount++;
        hzImuCount++;
        portEXIT_CRITICAL(&imuMux);

        // global failsafe
        if ((millis() - rxLastGoodPacketMs) > FAILSAFE_MS) {
            setMotorTargets(1000, 1000, 1000, 1000);
            continue;
        }

        float sRoll, sPitch, sYaw;
        int   throttleUs;
        portENTER_CRITICAL(&stickMux);
        sRoll = stickRoll;
        sPitch = stickPitch;
        sYaw = stickYaw;
        throttleUs = cmdThrottleUs;
        portEXIT_CRITICAL(&stickMux);

        if (!armed) {
            armed = true;
            rollArmRef = iRoll; pitchArmRef = iPitch; yawArmRef = iYaw;
        }

        if (throttleUs >= IDLE_CUTOFF_US) {
            float targetRollDeg =  sRoll  * STICK_MAX_ANGLE_DEG;
            float targetPitchDeg = -sPitch * STICK_MAX_ANGLE_DEG;

            float targetRollRateDps = angleKpVal * (targetRollDeg  - (iRoll  - rollArmRef));
            float targetPitchRateDps = angleKpVal * (targetPitchDeg - (iPitch - pitchArmRef));
            float yawStickRateDps = sYaw * STICK_MAX_YAW_RATE_DPS;
            yawHeldDeg            += (iGz - yawStickRateDps) * DT;  // drift from intended heading
            float targetYawRateDps = yawStickRateDps + angleKpVal * (0.0f - yawHeldDeg);

            float rollOut = pidRate(rollRatePID,  targetRollRateDps,  iGx, DT);
            float pitchOut = pidRate(pitchRatePID, targetPitchRateDps, iGy, DT);
            float yawOut = pidRate(yawRatePID,   targetYawRateDps,   iGz, DT);

            int fl = throttleUs + (int)pitchOut + (int)rollOut + (int)yawOut;
            int fr = throttleUs + (int)pitchOut - (int)rollOut - (int)yawOut;
            int bl = throttleUs - (int)pitchOut + (int)rollOut - (int)yawOut;
            int br = throttleUs - (int)pitchOut - (int)rollOut + (int)yawOut;

            if (fabsf(iRoll - rollArmRef) > 45.0f) {
                setMotorTargets(1000, 1000, 1000, 1000);
                continue;
            }

            setMotorTargets(fl, fr, bl, br);
        } else {
            setMotorTargets(1000, 1000, 1000, 1000);
        }
    }
}

void motorTask(void* parameter) {
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        int fl, fr, bl, br;
        portENTER_CRITICAL(&motorMux);
        fl = motorFL; fr = motorFR; bl = motorBL; br = motorBR;
        portEXIT_CRITICAL(&motorMux);

        escWriteUs(ESC_FL, (uint16_t)fl);
        escWriteUs(ESC_FR, (uint16_t)fr);
        escWriteUs(ESC_BL, (uint16_t)bl);
        escWriteUs(ESC_BR, (uint16_t)br);

        motorCount++;
        hzMotorCount++;
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_PERIOD_MS));
    }
}

void bleTxTask(void* parameter) {
    uint32_t lastSend = micros();

    while (true) {
        if (!deviceConnected && oldDeviceConnected) {
            vTaskDelay(pdMS_TO_TICKS(50));
            NimBLEDevice::getAdvertising()->start();
            oldDeviceConnected = false;
        }
        if (deviceConnected && !oldDeviceConnected) {
            oldDeviceConnected = true;
        }

        uint32_t now = micros();
        if (deviceConnected && (now - lastSend) >= BLE_PERIOD_US) {
            lastSend += BLE_PERIOD_US;
            if ((now - lastSend) > BLE_PERIOD_US * 4) lastSend = now;
            sendIMUPacket();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void idleTask0(void* param) { while (true) { idleCount0++; taskYIELD(); } }
void idleTask1(void* param) { while (true) { idleCount1++; taskYIELD(); } }

void setup() {
    Serial.begin(115200);
    esp_task_wdt_deinit();

    Serial.println("\n[SYS] ESP32 Flight Controller starting...");
    Serial.printf("[SYS] IMU=%d Hz  MOTOR=%d Hz  BLE_TX=%d Hz\n", IMU_HZ, MOTOR_HZ, BLE_SEND_HZ);
    Serial.println("[SYS] RX mode: DUAL (BLE claim/release + LoRa fallback)");
    Serial.printf("[SYS] BLE claim timeout: %lu ms\n", (unsigned long)BLE_CLAIM_TIMEOUT_MS);

    setupMotors();

    Wire.begin();
    Wire.setClock(400000);
    mpuInit();
    delay(100);
    mpuCalibrateGyro();

    lpfAX.begin(ACCEL_LPF_CUTOFF_HZ, IMU_HZ);
    lpfAY.begin(ACCEL_LPF_CUTOFF_HZ, IMU_HZ);
    lpfAZ.begin(ACCEL_LPF_CUTOFF_HZ, IMU_HZ);
    lpfGX.begin(GYRO_LPF_CUTOFF_HZ,  IMU_HZ);
    lpfGY.begin(GYRO_LPF_CUTOFF_HZ,  IMU_HZ);
    lpfGZ.begin(GYRO_LPF_CUTOFF_HZ,  IMU_HZ);
    Serial.printf("[LPF] HW DLPF: ~21 Hz | SW Accel: %.0f Hz | SW Gyro: %.0f Hz\n", (float)ACCEL_LPF_CUTOFF_HZ, (float)GYRO_LPF_CUTOFF_HZ);

    filter.begin(IMU_HZ);

    for (int i = 0; i < 500; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
            float fax = lpfAX.update(ax / 16384.0f);
            float fay = lpfAY.update(ay / 16384.0f);
            float faz = lpfAZ.update(az / 16384.0f);
            float fgx = lpfGX.update((gx - gyroBiasX) / MPU_GYRO_SCALE_LSB);
            float fgy = lpfGY.update((gy - gyroBiasY) / MPU_GYRO_SCALE_LSB);
            float fgz = lpfGZ.update((gz - gyroBiasZ) / MPU_GYRO_SCALE_LSB);
            filter.updateIMU(fgx, fgy, fgz, fax, fay, faz);
        }
        delay(4);
    }
    Serial.println("[IMU] Warm-up done");

    rollOffset = -filter.getRoll();
    pitchOffset =  filter.getPitch();
    yawOffset = -filter.getYaw();
    Serial.printf("[CAL] Zero offsets: R=%.2f P=%.2f Y=%.2f\n", rollOffset, pitchOffset, yawOffset);

    setupBLE();
    setupLoRa();

    xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, nullptr, configMAX_PRIORITIES - 1, &imuTaskHandle, 0);

    imuHwTimer = timerBegin(1000000);
    timerAttachInterrupt(imuHwTimer, imuHwTimerISR);
    timerAlarm(imuHwTimer, IMU_PERIOD_US, true, 0);
    Serial.printf("[TIMER] IMU hw timer: %lu µs period\n", (unsigned long)IMU_PERIOD_US);

    xTaskCreatePinnedToCore(motorTask, "motorTask", 2048, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(bleTxTask, "bleTxTask", 4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(bleDrainTask, "bleDrain", 2048, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(loraRxTask, "loraRxTask", 4096, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(loraDrainTask,"loraDrain", 2048, nullptr, 2, nullptr, 1);

    xTaskCreatePinnedToCore(idleTask0, "idle0", 1024, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(idleTask1, "idle1", 1024, nullptr, 1, nullptr, 1);

    Serial.println("[SYS] Setup complete - BLE + LoRa dual mode");
}

void loop() {
    static uint32_t lastStatMs = 0;
    uint32_t nowMs = millis();

    if ((nowMs - lastStatMs) >= STATS_INTERVAL_MS) {
        float dt = (nowMs - lastStatMs) / 1000.0f;
        lastStatMs = nowMs;

        uint32_t snapImu = hzImuCount; hzImuCount = 0;
        uint32_t snapMotor = hzMotorCount; hzMotorCount = 0;
        uint32_t snapBleTx = hzBleTxCount; hzBleTxCount = 0;
        uint32_t snapLoraRx = hzLoraRxCount; hzLoraRxCount = 0;
        uint32_t snapBleGood = hzBleRxGoodCount; hzBleRxGoodCount = 0;
        uint32_t snapLoraGood = hzLoraRxGoodCount; hzLoraRxGoodCount = 0;
        uint32_t snapLoraDrop = hzLoraDropCount; hzLoraDropCount = 0;

        float imuHz = snapImu / dt;
        float motorHz = snapMotor / dt;
        float bleTxHz = snapBleTx / dt;
        float loraRxHz = snapLoraRx / dt;
        float bleGoodHz = snapBleGood / dt;
        float loraGoodHz = snapLoraGood / dt;
        float loraDropHz = snapLoraDrop / dt;

        float lr, lp, ly;
        portENTER_CRITICAL(&imuMux);
        lr = rollDeg  - rollOffset;
        lp = pitchDeg - pitchOffset;
        ly = yawDeg   - yawOffset;
        portEXIT_CRITICAL(&imuMux);

        int mFL, mFR, mBL, mBR;
        portENTER_CRITICAL(&motorMux);
        mFL = motorFL; mFR = motorFR; mBL = motorBL; mBR = motorBR;
        portEXIT_CRITICAL(&motorMux);

        uint32_t msSinceAny = millis() - rxLastGoodPacketMs;
        bool failsafeActive = (msSinceAny > FAILSAFE_MS);
        bool bleInCtrl = isBleInControl();

        Serial.println("-----------------------------------------------------");
        Serial.printf("[HZ]  IMU=%.0f  Motor=%.0f  BLE-TX=%.0f  "
                      "LoRa-Radio=%.0f\n",
                      imuHz, motorHz, bleTxHz, loraRxHz);
        Serial.printf("[HZ]  BLE-RX-Good=%.0f  LoRa-Applied=%.0f  LoRa-Dropped=%.0f\n",
                      bleGoodHz, loraGoodHz, loraDropHz);
        Serial.printf("[RX]  ble_ok=%-4u  lora_ok=%-4u  bad_chk=%-4u  short=%-4u  "
                      "ACTIVE=%c\n",
                      rxBlePacketCount, rxLoraPacketCount,
                      rxBadChkCount, rxShortCount, (char)rxActiveSource);
        Serial.printf("[CTL] BLE_claimed=%s  last_any=%lums ago  FAILSAFE=%s\n",
                      bleInCtrl ? "YES" : "no",
                      (unsigned long)msSinceAny,
                      failsafeActive ? "ACTIVE" : "ok");
        Serial.printf("[CMD] thr=%4d µs  roll=%+.3f  pitch=%+.3f  yaw=%+.3f\n",
                      rx_throttleUs, rx_roll, rx_pitch, rx_yaw);
        Serial.printf("[PID] roll/pitch  P=%.4f  I=%.4f  D=%.4f\n",
                      rx_P, rx_I, rx_D);
        Serial.printf("[PID] yaw         P=%.4f  I=%.4f  D=%.4f  angleKP=%.3f\n",
                      rx_P2, rx_I2, rx_D2, rx_aKp);
        Serial.printf("[IMU] R=%+6.2f°  P=%+6.2f°  Y=%+6.2f°\n", lr, lp, ly);
        Serial.printf("[MTR] FL=%4d  FR=%4d  BL=%4d  BR=%4d µs\n",
                      mFL, mFR, mBL, mBR);
        Serial.printf("[SYS] armed=%d  ble_conn=%d\n",
                      (int)armed, (int)deviceConnected);

        rxBlePacketCount = 0;
        rxLoraPacketCount = 0;
        rxBadChkCount = 0;
        rxShortCount = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}
