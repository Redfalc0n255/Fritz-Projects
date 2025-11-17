// Includes =======================================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <esp_now.h>

// Pin Maps & Hardware Constants ==================================
constexpr int A_PWMA  = 27;
constexpr int A_AIN1  = 26;
constexpr int A_AIN2  = 25;
constexpr int A_PWMB  = 33;
constexpr int A_BIN1  = 32;
constexpr int A_BIN2  = 14;
constexpr int B_PWMA  = 15;
constexpr int B_AIN1  = 21;
constexpr int B_AIN2  = 5;
constexpr int B_PWMB  = 17;
constexpr int B_BIN1  = 4;
constexpr int B_BIN2  = 16;

constexpr int BUZZER_PIN     = 12;
constexpr int BUZZER_CHANNEL = 6;
constexpr uint16_t BEEP_DURATION_MS = 140;
constexpr uint16_t BEEP_BASE_FREQ_HZ = 700;
constexpr uint16_t BEEP_STEP_FREQ_HZ = 90;

constexpr i2s_port_t I2S_PORT    = I2S_NUM_0;
constexpr int I2S_BCLK_PIN       = 22;
constexpr int I2S_LRCLK_PIN      = 23;
constexpr int I2S_DIN_PIN        = 18;

constexpr int SERVO_PAN_PIN   = 13;
constexpr int SERVO_MIN_DEG   = 0;
constexpr int SERVO_MAX_DEG   = 180;
constexpr int SERVO_CENTER_DEG = 90;
constexpr int SERVO_MIN_US    = 400;   // calibrated safe min
constexpr int SERVO_CENTER_US = 1500;  // calibrated center
constexpr int SERVO_MAX_US    = 2600;  // calibrated safe max

constexpr int TFMINI_RX_PIN = 2;
constexpr int TFMINI_TX_PIN = 19;
constexpr uint32_t TFMINI_BAUD = 115200;

constexpr uint8_t  SWEEP_START_DEG  = 0;
constexpr uint8_t  SWEEP_SPAN_DEG   = 180;
constexpr uint8_t  SWEEP_STEP_DEG   = 2;
constexpr uint16_t SERVO_SETTLE_MS  = 35;
constexpr uint16_t LIDAR_TIMEOUT_MS = 60;

constexpr uint8_t MAP_CELLS   = 21;
constexpr float   MAP_CELL_CM = 10.0f;
constexpr size_t  MAP_BUFFER_MAX = (MAP_CELLS * (MAP_CELLS + 1)) + 4;

// ESP-NOW Protocol ===============================================
enum PacketType : uint8_t { PKT_CONTROL = 0x01, PKT_MAP_CHUNK = 0x42 };
enum ActionMask : uint8_t { ACTION_BEEP = 1 << 0, ACTION_REQUEST_MAP = 1 << 1 };

struct ControlPacket {
  uint8_t msgType;
  char    driveCommand;
  uint8_t actionMask;
};

// Sweep Data Structures ==========================================
struct SweepSample {
  uint16_t distanceCm;
  uint16_t angleDeg;
};

struct SweepBatch {
  SweepSample samples[90];
  size_t count;
};

struct MotorPins {
  int pwm;
  int in1;
  int in2;
  bool inverted;
};

enum DriveCommand { CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT };

// Global State ===================================================
HardwareSerial SerialTF(2);
Servo scannerServo;

const MotorPins MOTOR_MAP[4] = {
  {A_PWMA, A_AIN1, A_AIN2, false},
  {A_PWMB, A_BIN1, A_BIN2, false},
  {B_PWMA, B_AIN1, B_AIN2, true},
  {B_PWMB, B_BIN1, B_BIN2, false}
};

uint8_t controllerMac[] = { 0x04, 0x83, 0x08, 0x57, 0xF8, 0x90 };
volatile bool pendingSweepRequest = false;
volatile ControlPacket latestPacket = { PKT_CONTROL, 'S', 0 };
volatile bool sweepInProgress = false;

// Motor Helpers ==================================================
// setupPins()
// Configures all motor control pins and ensures outputs start low.
void setupPins() {
  for (const MotorPins &pins : MOTOR_MAP) {
    pinMode(pins.in1, OUTPUT);
    pinMode(pins.in2, OUTPUT);
    analogWrite(pins.pwm, 0);
  }
}

// setMotor(pins, direction, duty)
// Drives a single motor forward/back/stop with the given PWM duty.
void setMotor(const MotorPins &pins, int direction, uint8_t duty = 255) {
  if (direction == 0) {
    digitalWrite(pins.in1, LOW);
    digitalWrite(pins.in2, LOW);
    analogWrite(pins.pwm, 0);
    return;
  }

  bool driveForward = (direction > 0) ^ pins.inverted;
  digitalWrite(pins.in1, driveForward ? HIGH : LOW);
  digitalWrite(pins.in2, driveForward ? LOW : HIGH);
  analogWrite(pins.pwm, duty);
}

// driveWheels(fl, fr, rr, rl, duty)
// Applies signed directions to each wheel simultaneously.
void driveWheels(int8_t fl, int8_t fr, int8_t rr, int8_t rl, uint8_t duty = 200) {
  setMotor(MOTOR_MAP[0], fl, duty);
  setMotor(MOTOR_MAP[1], fr, duty);
  setMotor(MOTOR_MAP[2], rr, duty);
  setMotor(MOTOR_MAP[3], rl, duty);
}

// stopAllMotors()
// Immediately brakes every wheel.
void stopAllMotors() {
  for (const MotorPins &pins : MOTOR_MAP) {
    setMotor(pins, 0);
  }
}

// charToCommand(cmdChar)
// Converts incoming ASCII commands into the DriveCommand enum.
DriveCommand charToCommand(char cmdChar) {
  switch (cmdChar) {
    case 'F': return CMD_FORWARD;
    case 'B': return CMD_BACKWARD;
    case 'L': return CMD_LEFT;
    case 'R': return CMD_RIGHT;
    default:  return CMD_STOP;
  }
}

// applyDriveCommand(cmd)
// Sends the selected tank-drive pattern to the wheel map.
void applyDriveCommand(DriveCommand cmd) {
  switch (cmd) {
    case CMD_FORWARD:  driveWheels(+1, +1, +1, +1); break;
    case CMD_BACKWARD: driveWheels(-1, -1, -1, -1); break;
    case CMD_LEFT:     driveWheels(-1, +1, +1, -1); break;
    case CMD_RIGHT:    driveWheels(+1, -1, -1, +1); break;
    case CMD_STOP:
    default:           stopAllMotors();             break;
  }
}

// TF-Mini + Servo Helpers ========================================
// initTfMini()
// Bootstraps the TF-mini UART connection.
void initTfMini() {
  SerialTF.begin(TFMINI_BAUD, SERIAL_8N1, TFMINI_RX_PIN, TFMINI_TX_PIN);
  SerialTF.setTimeout(20);
  delay(50);
  while (SerialTF.available()) {
    SerialTF.read();
  }
}

// readTfMiniWithTimeout(timeoutMs)
// Returns TF-mini distance in cm or 0xFFFF if no frame arrives in time.
uint16_t readTfMiniWithTimeout(uint16_t timeoutMs) {
  uint8_t frame[9];
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (!SerialTF.available()) continue;

    frame[0] = SerialTF.read();
    if (frame[0] != 0x59) continue;

    unsigned long headerWait = millis();
    while (!SerialTF.available()) {
      if (millis() - headerWait > timeoutMs) return 0xFFFF;
    }
    frame[1] = SerialTF.read();
    if (frame[1] != 0x59) continue;

    unsigned long dataWait = millis();
    while (SerialTF.available() < 7) {
      if (millis() - dataWait > timeoutMs) return 0xFFFF;
    }
    SerialTF.readBytes(frame + 2, 7);

    uint8_t checksum = 0;
    for (int i = 0; i < 8; ++i) {
      checksum += frame[i];
    }
    if (checksum != frame[8]) continue;

    uint16_t distanceCm = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
    return distanceCm;
  }

  return 0xFFFF;
}

// sampleSweeps()
// Centers the servo, sweeps right->left collecting samples, returns to center.
SweepBatch sampleSweeps() {
  SweepBatch batch{};
  const int spanUs = SERVO_MAX_US - SERVO_MIN_US;
  const int stepUs = max<int>(1, spanUs * SWEEP_STEP_DEG / SWEEP_SPAN_DEG);
  const size_t targetCount = min<size_t>(
      sizeof(batch.samples) / sizeof(batch.samples[0]),
      spanUs / stepUs + 1);

  // Center first
  scannerServo.writeMicroseconds(SERVO_CENTER_US);
  delay(SERVO_SETTLE_MS);

  // Sweep from right (max) to left (min)
  size_t idx = 0;
  for (int us = SERVO_MAX_US; us >= SERVO_MIN_US && idx < targetCount; us -= stepUs, ++idx) {
    scannerServo.writeMicroseconds(us);
    delay(SERVO_SETTLE_MS);

    // Map pulse to approximate angle 0-180
    float angleDeg = (float)(us - SERVO_MIN_US) * SWEEP_SPAN_DEG / (float)spanUs;
    batch.samples[idx] = {
      readTfMiniWithTimeout(LIDAR_TIMEOUT_MS),
      static_cast<uint16_t>(angleDeg)
    };
  }

  batch.count = idx;

  // Return to center
  scannerServo.writeMicroseconds(SERVO_CENTER_US);
  delay(SERVO_SETTLE_MS);

  return batch;
}

// buildMapAscii(sweeps, buffer, capacity)
// Rasterizes sweep samples into an ASCII grid; returns bytes written.
size_t buildMapAscii(const SweepBatch &sweeps, char *buffer, size_t capacity) {
  if (!buffer || capacity == 0) return 0;

  char grid[MAP_CELLS][MAP_CELLS];
  for (uint8_t r = 0; r < MAP_CELLS; ++r) {
    for (uint8_t c = 0; c < MAP_CELLS; ++c) {
      grid[r][c] = '.';
    }
  }

  const uint8_t center = MAP_CELLS / 2;
  grid[center][center] = 'o';

  for (size_t i = 0; i < sweeps.count; ++i) {
    const SweepSample &sample = sweeps.samples[i];
    if (sample.distanceCm == 0xFFFF) continue;

    float rCm = sample.distanceCm;
    float theta = radians(sample.angleDeg);
    float x = rCm * cosf(theta);
    float y = rCm * sinf(theta);

    int col = center + int(roundf(x / MAP_CELL_CM));
    int row = center - int(roundf(y / MAP_CELL_CM));
    if (row >= 0 && row < MAP_CELLS && col >= 0 && col < MAP_CELLS) {
      grid[row][col] = '#';
    }
  }

  size_t idx = 0;
  for (uint8_t r = 0; r < MAP_CELLS; ++r) {
    for (uint8_t c = 0; c < MAP_CELLS; ++c) {
      if (idx + 1 >= capacity) {
        buffer[capacity - 1] = '\0';
        return capacity - 1;
      }
      buffer[idx++] = grid[r][c];
    }
    if (idx + 1 >= capacity) {
      buffer[capacity - 1] = '\0';
      return capacity - 1;
    }
    buffer[idx++] = '\n';
  }

  buffer[idx] = '\0';
  return idx;
}

// sendMapChunks(asciiMap, length)
// Sends the ASCII map back to the controller in ESP-NOW sized frames.
void sendMapChunks(const char *asciiMap, size_t len) {
  if (!asciiMap || len == 0) return;

  const uint8_t payloadMax = 180;
  uint8_t totalChunks = (len + payloadMax - 1) / payloadMax;

  for (uint8_t idx = 0; idx < totalChunks; ++idx) {
    uint16_t sliceLen = min<uint16_t>(payloadMax, len - idx * payloadMax);
    uint8_t packet[5 + payloadMax];
    packet[0] = PKT_MAP_CHUNK;
    packet[1] = idx;
    packet[2] = totalChunks;
    packet[3] = sliceLen & 0xFF;
    packet[4] = sliceLen >> 8;
    memcpy(packet + 5, asciiMap + idx * payloadMax, sliceLen);
    esp_now_send(controllerMac, packet, 5 + sliceLen);
  }
}

// Motion Helpers =================================================
// turnRobot(direction, angleDeg, duty)
// Spins the robot in place in the requested direction for angleDeg degrees.
void turnRobot(const String &direction, float angleDeg, uint8_t duty = 200) {
  static constexpr float DEG_PER_MS_IN_PLACE = 0.9f;
  if (angleDeg <= 0) return;

  bool left = direction.equalsIgnoreCase("left");
  unsigned long runTime = static_cast<unsigned long>(angleDeg / DEG_PER_MS_IN_PLACE);
  int leftDir = left ? -1 : +1;
  int rightDir = -leftDir;

  driveWheels(leftDir, rightDir, rightDir, leftDir, duty);
  delay(runTime);
  stopAllMotors();
}

// move(direction, distanceCm, duty)
// Drives forward or backward for the requested linear distance.
void move(const String &direction, float distanceCm, uint8_t duty = 200) {
  static constexpr float CM_PER_MS_AT_DUTY = 0.12f;
  if (distanceCm <= 0) return;

  int dir = direction.equalsIgnoreCase("forward") ? +1 : -1;
  unsigned long runTime = static_cast<unsigned long>(distanceCm / CM_PER_MS_AT_DUTY);

  driveWheels(dir, dir, dir, dir, duty);
  delay(runTime);
  stopAllMotors();
}

// Buzzer Helpers ================================================
// initBuzzer()
// Prepares the buzzer pin for analogWrite-based PWM drive.
void initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(BUZZER_PIN, 0);
}

// beep(slot, durationMs, volumePct)
// Emits a blocking tone using software PWM (analogWrite + toggling).
void beep(uint8_t slot, uint16_t durationMs, uint8_t volumePct) {
  slot = constrain(slot, 1, 10);
  volumePct = constrain(volumePct, 0, 100);

  uint16_t freq = BEEP_BASE_FREQ_HZ + (slot - 1) * BEEP_STEP_FREQ_HZ;
  uint32_t periodUs = 1000000UL / max<uint16_t>(freq, 1);
  uint32_t halfPeriodUs = periodUs / 2;
  uint8_t duty = map(volumePct, 0, 100, 0, 255);

  unsigned long endTime = millis() + durationMs;
  while (millis() < endTime) {
    analogWrite(BUZZER_PIN, duty);
    delayMicroseconds(halfPeriodUs);
    analogWrite(BUZZER_PIN, 0);
    delayMicroseconds(halfPeriodUs);
  }
  analogWrite(BUZZER_PIN, 0);
}

// updateBuzzer()
// Placeholder for future non-blocking buzzer logic.
void updateBuzzer() {
  // Intentionally left blank (beep() is currently blocking).
}

// ESP-NOW Helpers ===============================================
// addControllerPeer()
// Registers the handheld controller as an ESP-NOW peer.
void addControllerPeer() {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, controllerMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

// onControllerPacket(info, data, len)
// Handles inbound controller packets and raises sweep requests.
void onControllerPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len != sizeof(ControlPacket)) return;
  const ControlPacket *packet = reinterpret_cast<const ControlPacket *>(data);
  if (packet->msgType != PKT_CONTROL) return;

  noInterrupts();
  latestPacket.msgType = packet->msgType;
  latestPacket.driveCommand = packet->driveCommand;
  latestPacket.actionMask = packet->actionMask;
  if (packet->actionMask & ACTION_REQUEST_MAP) {
    // Immediately stop and block drive updates until sweep completes.
    latestPacket.driveCommand = 'S';
    pendingSweepRequest = true;
    sweepInProgress = true;
    stopAllMotors();
  }
  interrupts();

  Serial.printf("RX: cmd=%c actions=0x%02X\n", packet->driveCommand, packet->actionMask);
}

// onEspNowSent(info, status)
// Logs failed transmissions for debugging purposes.
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS && info) {
    Serial.printf("ESP-NOW send to %02X:%02X:%02X:%02X:%02X:%02X failed (%d)\n",
                  info->des_addr[0], info->des_addr[1], info->des_addr[2],
                  info->des_addr[3], info->des_addr[4], info->des_addr[5], status);
  }
}

// serviceRemoteDrive()
// Applies the latest joystick command and handles queued actions.
void serviceRemoteDrive() {
  if (sweepInProgress) {
    return;
  }
  ControlPacket snapshot;
  noInterrupts();
  snapshot.msgType = latestPacket.msgType;
  snapshot.driveCommand = latestPacket.driveCommand;
  snapshot.actionMask = latestPacket.actionMask;
  interrupts();

  applyDriveCommand(charToCommand(snapshot.driveCommand));
  Serial.printf("APPLY: cmd=%c\n", snapshot.driveCommand);
  if (snapshot.actionMask & ACTION_BEEP) {
    beep(3, BEEP_DURATION_MS, 40);
    noInterrupts();
    latestPacket.actionMask &= static_cast<uint8_t>(~ACTION_BEEP);
    interrupts();
  }
}

// Arduino Lifecycle =============================================
// setup()
// Initializes hardware subsystems and ESP-NOW connectivity.
void setup() {
  Serial.begin(115200);

  setupPins();
  scannerServo.attach(SERVO_PAN_PIN);
  scannerServo.writeMicroseconds(SERVO_CENTER_US);
  delay(200);
  initTfMini();
  initBuzzer();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onControllerPacket);
  esp_now_register_send_cb(onEspNowSent);
  addControllerPeer();

  Serial.println("A4AutoMain ready.");
}

// loop()
// Main scheduler: processes remote input, sweeps, and housekeeping.
void loop() {
  serviceRemoteDrive();

  if (pendingSweepRequest) {
    sweepInProgress = true;
    stopAllMotors();
    SweepBatch sweep = sampleSweeps();
    char mapBuffer[MAP_BUFFER_MAX];
    size_t asciiLen = buildMapAscii(sweep, mapBuffer, sizeof(mapBuffer));

    Serial.println(F("---- sweep map ----"));
    Serial.print(mapBuffer);
    Serial.println(F("-------------------"));

    sendMapChunks(mapBuffer, asciiLen);
    pendingSweepRequest = false;
    sweepInProgress = false;
  }

  updateBuzzer();
  delay(10);
}
