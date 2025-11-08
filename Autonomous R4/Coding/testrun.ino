#include <Arduino.h>
#include <ESP32Servo.h>
#include <driver/i2s.h>
#include <math.h>
#include <limits.h>



// ===== PIN MAP (yours) =====
// Driver A
const int A_PWMA = 27, A_AIN1 = 26, A_AIN2 = 25;   // M1 = top-left
const int A_PWMB = 33, A_BIN1 = 32, A_BIN2 = 14;   // M2 = top-right
// Driver B
const int B_PWMA = 15, B_AIN1 = 21, B_AIN2 = 5;    // M3 = back-right (inverted)
const int B_PWMB = 17, B_BIN1 = 4,  B_BIN2 = 16;   // M4 = back-left

// STBY: set to a GPIO if wired; else tie STBY to 3.3 V on each TB6612 board
const int STBY_PIN = -1; // e.g., 23 if you wired it

// Servo + TFmini hardware
const int SERVO_PAN_PIN = 13;
const int SERVO_MIN_DEG = 20;
const int SERVO_MAX_DEG = 160;
const int SERVO_CENTER_DEG = 90;

const int TFMINI_RX_PIN = 2;   // TFmini TX (green) -> ESP32 RX
const int TFMINI_TX_PIN = 19;  // TFmini RX (white) <- ESP32 TX
const uint32_t TFMINI_BAUD = 115200;
HardwareSerial SerialTF(2);

// MAX98357A I2S pins (adjust if you wired differently)
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int I2S_BCLK_PIN = 22;
const int I2S_LRCLK_PIN = 23;
const int I2S_DIN_PIN = 18;

// Servo + speaker state
Servo lidarServo;
bool servoTimersConfigured = false;
bool servoAttached = false;
bool speakerInitialized = false;
bool tfMiniInitialized = false;

struct ScanSample {
  int angleDeg = SERVO_CENTER_DEG;
  uint16_t distanceCm = 0;
  uint16_t strength = 0;
  bool valid = false;
};

const int SCAN_STEP_DEG = 12;
const size_t MAX_SCAN_SAMPLES = ((SERVO_MAX_DEG - SERVO_MIN_DEG) / SCAN_STEP_DEG) + 1;
const uint16_t MIN_CLEARANCE_CM = 40;
const uint16_t IDEAL_CLEARANCE_CM = 200;
const uint16_t MAX_DISTANCE_CM = 600;
const int CLEARANCE_PENALTY_PER_DEG = 3;
const int HEADING_BLOCKED = 1000;

const int SLOW_DRIVE_POWER = 230;
const int PIVOT_POWER = 255;

ScanSample scanBuffer[MAX_SCAN_SAMPLES];

// Invert only M3
bool invertDir[5] = {false, false, false, false, true}; // index 1..4

// -------- Low-level helpers --------
static inline void getMotorPins(int m, int &in1, int &in2, int &pwm) {
  switch (m) {
    case 1: in1=A_AIN1; in2=A_AIN2; pwm=A_PWMA; break;
    case 2: in1=A_BIN1; in2=A_BIN2; pwm=A_PWMB; break;
    case 3: in1=B_AIN1; in2=B_AIN2; pwm=B_PWMA; break;
    case 4: in1=B_BIN1; in2=B_BIN2; pwm=B_PWMB; break;
    default: in1=in2=pwm=-1; break;
  }
}

static inline void motorForward(int m) {
  int in1,in2,pwm; getMotorPins(m,in1,in2,pwm); if (pwm<0) return;
  bool inv = invertDir[m];
  digitalWrite(in1, inv ? LOW  : HIGH);
  digitalWrite(in2, inv ? HIGH : LOW);
  digitalWrite(pwm, HIGH); // full speed (no PWM)
}
static inline void motorBackward(int m) {
  int in1,in2,pwm; getMotorPins(m,in1,in2,pwm); if (pwm<0) return;
  bool inv = invertDir[m];
  digitalWrite(in1, inv ? HIGH : LOW);
  digitalWrite(in2, inv ? LOW  : HIGH);
  digitalWrite(pwm, HIGH);
}
static inline void motorStop(int m) {
  int in1,in2,pwm; getMotorPins(m,in1,in2,pwm); if (pwm<0) return;
  digitalWrite(pwm, LOW); digitalWrite(in1, LOW); digitalWrite(in2, LOW);
}

static inline void allStop() { for (int m=1;m<=4;++m) motorStop(m); }

// Drive an individual motor with signed power (PWM magnitude + direction).
static inline void setMotorPower(int m, int power) {
  int in1, in2, pwm;
  getMotorPins(m, in1, in2, pwm);
  if (pwm < 0) return;

  bool forward = power >= 0;
  int duty = constrain(abs(power), 0, 255);
  bool inv = invertDir[m];
  bool finalForward = inv ? !forward : forward;

  digitalWrite(in1, finalForward ? HIGH : LOW);
  digitalWrite(in2, finalForward ? LOW : HIGH);
  analogWrite(pwm, duty);
}

// Apply the same signed power to both wheels on each side.
void driveTank(int leftPower, int rightPower) {
  setMotorPower(1, leftPower);
  setMotorPower(4, leftPower);
  setMotorPower(2, rightPower);
  setMotorPower(3, rightPower);
}

// Pivot left/right in place for a fixed time using tank controls.
void pivotInPlace(int direction, uint16_t durationMs, int power = PIVOT_POWER) {
  if (power < 0) power = -power;
  int left = direction >= 0 ? power : -power;
  int right = direction >= 0 ? -power : power;
  driveTank(left, right);
  delay(durationMs);
  allStop();
}

// Move ahead slowly for a bounded duration, then brake all motors.
void creepForward(uint16_t durationMs, int power = SLOW_DRIVE_POWER) {
  driveTank(power, power);
  delay(durationMs);
  allStop();
}

void hardTurnLeft() {
  driveTank(-PIVOT_POWER, PIVOT_POWER);
  delay(800);
  allStop();
}

void hardTurnRight() {
  driveTank(PIVOT_POWER, -PIVOT_POWER);
  delay(1000);
  allStop();
}

// Lazily attach the lidar servo so the pin only goes active once.
void ensureServoAttached() {
  if (!servoTimersConfigured) {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servoTimersConfigured = true;
  }
  if (servoAttached) return;
  lidarServo.setPeriodHertz(50);       // standard servo refresh
  lidarServo.attach(SERVO_PAN_PIN, 500, 2400); // microsecond range
  lidarServo.write(SERVO_CENTER_DEG);
  servoAttached = true;
}

// Configure the MAX98357A I2S peripheral when first needed.
void ensureSpeakerInitialized() {
  if (speakerInitialized) return;
  i2s_config_t cfg = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 22050,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRCLK_PIN,
    .data_out_num = I2S_DIN_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  i2s_set_pin(I2S_PORT, &pins);
  speakerInitialized = true;
}

// Play a simple sine tone over I2S for cueing test phases.
void playTone(float freqHz, uint16_t durationMs, float volume = 0.35f) {
  ensureSpeakerInitialized();
  const float sampleRate = 22050.0f;
  const size_t chunkSamples = 256;
  int16_t buffer[chunkSamples * 2];
  size_t totalSamples = static_cast<size_t>(sampleRate * durationMs / 1000.0f);
  size_t produced = 0;
  float phase = 0.0f;
  const float phaseStep = 2.0f * PI * freqHz / sampleRate;
  while (produced < totalSamples) {
    size_t thisChunk = min(chunkSamples, totalSamples - produced);
    for (size_t i = 0; i < thisChunk; ++i) {
      float sample = sinf(phase) * volume;
      int16_t value = static_cast<int16_t>(sample * 32767);
      buffer[i * 2] = value;
      buffer[i * 2 + 1] = value;
      phase += phaseStep;
      if (phase > 2.0f * PI) phase -= 2.0f * PI;
    }
    size_t bytesToWrite = thisChunk * sizeof(int16_t) * 2;
    size_t bytesWritten = 0;
    i2s_write(I2S_PORT, buffer, bytesToWrite, &bytesWritten, portMAX_DELAY);
    produced += thisChunk;
  }
  i2s_zero_dma_buffer(I2S_PORT);
}

struct ToneEvent {
  float freqHz;
  uint16_t durationMs;
  uint16_t gapMs;
};

void playPhrase(const ToneEvent *events, size_t count, float volume = 0.3f) {
  for (size_t i = 0; i < count; ++i) {
    playTone(events[i].freqHz, events[i].durationMs, volume);
    if (events[i].gapMs) delay(events[i].gapMs);
  }
}

void announcePhaseNumber(int phase) {
  for (int i = 0; i < phase; ++i) {
    playTone(550.0f + i * 40.0f, 180);
    delay(180);
  }
}

void cueDirection(bool isLeft) {
  float freq = isLeft ? 400.0f : 900.0f;
  playTone(freq, 140);
  delay(200);
}

void speakLeft() {
  static const ToneEvent phrase[] = {
    {520.0f, 140, 40},
    {420.0f, 160, 30},
    {300.0f, 120, 0}
  };
  playPhrase(phrase, sizeof(phrase)/sizeof(phrase[0]));
}

void speakRight() {
  static const ToneEvent phrase[] = {
    {680.0f, 130, 40},
    {880.0f, 130, 40},
    {640.0f, 160, 0}
  };
  playPhrase(phrase, sizeof(phrase)/sizeof(phrase[0]));
}

void speakForward() {
  static const ToneEvent phrase[] = {
    {450.0f, 110, 30},
    {520.0f, 110, 30},
    {600.0f, 150, 30},
    {520.0f, 130, 0}
  };
  playPhrase(phrase, sizeof(phrase)/sizeof(phrase[0]));
}

void beepScanStart() {
  playTone(760.0f, 180);
  delay(120);
}

void beepForwardCue() {
  playTone(1020.0f, 160);
  delay(80);
}

void beepHeadingBlocked() {
  for (int i = 0; i < 3; ++i) {
    playTone(320.0f, 120);
    delay(60);
  }
}

void beepObstacleAlert() {
  playTone(450.0f, 120);
  delay(60);
  playTone(320.0f, 140);
  delay(80);
}

// Start the TFmini UART on the requested pins (avoids Serial0 conflict).
void ensureTfminiInitialized() {
  if (tfMiniInitialized) return;
  SerialTF.begin(TFMINI_BAUD, SERIAL_8N1, TFMINI_RX_PIN, TFMINI_TX_PIN);
  SerialTF.setTimeout(20);
  tfMiniInitialized = true;
}

// Pull a single well-formed TFmini frame from the serial buffer.
bool readTfminiFrame(uint16_t &distanceCm, uint16_t &strength) {
  while (SerialTF.available() >= 9) {
    if (SerialTF.read() != 0x59) continue;
    if (!SerialTF.available()) break;
    uint8_t second = SerialTF.read();
    if (second != 0x59) continue;

    if (SerialTF.available() < 7) break;
    uint8_t frame[7];
    for (int i = 0; i < 7; ++i) {
      frame[i] = SerialTF.read();
    }

    uint8_t checksum = 0x59 + 0x59;
    for (int i = 0; i < 6; ++i) checksum += frame[i];
    if (checksum != frame[6]) continue;

    distanceCm = frame[0] | (frame[1] << 8);
    strength = frame[2] | (frame[3] << 8);
    return true;
  }
  return false;
}

// Keep attempting to read frames until success or timeout.
bool fetchTfminiMeasurement(uint16_t &distanceCm, uint16_t &strength, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (readTfminiFrame(distanceCm, strength)) return true;
  }
  return false;
}

// Move a servo between two angles with a dwell at each incremental step.
void sweepServo(Servo &servo, int fromDeg, int toDeg, int stepDeg, uint16_t dwellMs) {
  if (stepDeg <= 0) stepDeg = 1;
  if (fromDeg == toDeg) {
    servo.write(fromDeg);
    delay(dwellMs);
    return;
  }
  int direction = (fromDeg < toDeg) ? stepDeg : -stepDeg;
  for (int angle = fromDeg; angle != toDeg; angle += direction) {
    servo.write(angle);
    delay(dwellMs);
  }
  servo.write(toDeg);
  delay(dwellMs);
}

// Sweep the TFmini across the field and log every measurement.
size_t performScan(ScanSample *outSamples, size_t capacity, int stepDeg = SCAN_STEP_DEG, bool audioFeedback = false) {
  ensureServoAttached();
  ensureTfminiInitialized();
  if (stepDeg <= 0) stepDeg = 1;

  if (audioFeedback) beepScanStart();

  size_t count = 0;
  for (int angle = SERVO_MIN_DEG; angle <= SERVO_MAX_DEG && count < capacity; angle += stepDeg) {
    lidarServo.write(angle);
    delay(110);
    uint16_t distance = 0;
    uint16_t strength = 0;
    bool ok = fetchTfminiMeasurement(distance, strength, 120);
    outSamples[count].angleDeg = angle;
    uint16_t clamped = distance > MAX_DISTANCE_CM ? MAX_DISTANCE_CM : distance;
    outSamples[count].distanceCm = ok ? clamped : 0;
    outSamples[count].strength = strength;
    outSamples[count].valid = ok;
    if (ok) {
      Serial.printf("SCAN angle %3d -> %4u cm (strength %u)\n", angle, outSamples[count].distanceCm, strength);
      if (audioFeedback && outSamples[count].distanceCm < MIN_CLEARANCE_CM) {
        beepObstacleAlert();
      }
    } else {
      Serial.printf("SCAN angle %3d -> timeout\n", angle);
    }
    ++count;
  }
  lidarServo.write(SERVO_CENTER_DEG);
  delay(120);
  return count;
}

// Score each sample for clearance and pick the most promising heading.
int chooseHeadingOffset(const ScanSample *samples, size_t count) {
  int bestScore = INT_MIN;
  int bestOffset = HEADING_BLOCKED;
  for (size_t i = 0; i < count; ++i) {
    if (!samples[i].valid) continue;
    uint16_t distance = samples[i].distanceCm;
    if (distance < MIN_CLEARANCE_CM) continue;
    int offset = samples[i].angleDeg - SERVO_CENTER_DEG;
    int score = static_cast<int>(distance);
    score -= abs(offset) * CLEARANCE_PENALTY_PER_DEG;
    if (distance > IDEAL_CLEARANCE_CM) score += 20;
    if (score > bestScore) {
      bestScore = score;
      bestOffset = offset;
    }
  }
  return (bestScore == INT_MIN) ? HEADING_BLOCKED : bestOffset;
}

// Pivot just enough to face the desired heading.
void alignToHeading(int headingOffsetDeg) {
  int absOffset = abs(headingOffsetDeg);
  if (absOffset < 6) return;
  int repeats = constrain((absOffset + 19) / 20, 1, 3);
  for (int i = 0; i < repeats; ++i) {
    if (headingOffsetDeg < 0) {
      hardTurnLeft();
    } else {
      hardTurnRight();
    }
  }
}

// Full navigation cycle: scan, pick heading, align, creep forward.
void runAutonomyStep() {
  size_t samples = performScan(scanBuffer, MAX_SCAN_SAMPLES, SCAN_STEP_DEG, true);
  int heading = chooseHeadingOffset(scanBuffer, samples);
  if (heading == HEADING_BLOCKED) {
    Serial.println(F("[AUTO] No clear heading, pivoting to search..."));
    beepHeadingBlocked();
    pivotInPlace(1, 750, PIVOT_POWER);
    return;
  }
  Serial.printf("[AUTO] Best heading %+d deg\n", heading);
  if (heading < -6) {
    speakLeft();
  } else if (heading > 6) {
    speakRight();
  } else {
    speakForward();
  }
  alignToHeading(heading);
  beepForwardCue();
  creepForward(900, SLOW_DRIVE_POWER);
}

void testServo() {
  ensureServoAttached();
  Serial.println(F("[TEST] Servo sweep"));
  sweepServo(lidarServo, SERVO_CENTER_DEG, SERVO_MAX_DEG, 2, 15);
  sweepServo(lidarServo, SERVO_MAX_DEG, SERVO_MIN_DEG, 2, 15);
  sweepServo(lidarServo, SERVO_MIN_DEG, SERVO_CENTER_DEG, 2, 15);
}

void testSpeaker() {
  ensureSpeakerInitialized();
  Serial.println(F("[TEST] Speaker (MAX98357A)"));
  const size_t samplesPerChunk = 256;
  int16_t buffer[samplesPerChunk * 2];
  const float baseFreq = 440.0f;
  const float modFreq = 3.0f;
  const float sampleRate = 22050.0f;
  float phase = 0.0f;
  const int chunks = 80; // ~0.9 s of audio
  for (int chunk = 0; chunk < chunks; ++chunk) {
    for (size_t i = 0; i < samplesPerChunk; ++i) {
      float t = (chunk * samplesPerChunk + i) / sampleRate;
      float freq = baseFreq + 220.0f * sinf(2.0f * PI * modFreq * t);
      phase += 2.0f * PI * freq / sampleRate;
      if (phase > 2.0f * PI) phase -= 2.0f * PI;
      float sample = sinf(phase);
      float noise = (float)random(-2048, 2048) / 32768.0f;
      int16_t value = (int16_t)((sample * 0.8f + noise * 0.2f) * 16000);
      buffer[i * 2] = value;
      buffer[i * 2 + 1] = value;
    }
    size_t bytesWritten = 0;
    i2s_write(I2S_PORT, buffer, sizeof(buffer), &bytesWritten, portMAX_DELAY);
  }
  i2s_zero_dma_buffer(I2S_PORT);
}

static void runTurnCase(const char *label, int leftPower, int rightPower, uint16_t durationMs) {
  Serial.printf("[TURN] %s | L=%d R=%d for %ums\n", label, leftPower, rightPower, durationMs);
  driveTank(leftPower, rightPower);
  delay(durationMs);
  allStop();
}

void testTurning() {
  struct TurnTestCase {
    const char *label;
    int leftPower;
    int rightPower;
    uint16_t durationMs;
  };

  const TurnTestCase leftBaseline = {"LEFT baseline pivot", -255, 255, 700};
  const TurnTestCase rightCandidates[] = {
    {"RIGHT pivot 0.9s", 255, -255, 900},
    {"RIGHT pivot 1.0s", 255, -255, 1000},
    {"RIGHT pivot 1.1s", 255, -255, 1100},
    {"RIGHT pivot 1.2s", 255, -255, 1200},
    {"RIGHT pivot 1.3s", 255, -255, 1300}
  };

  Serial.println(F("[TEST] Turning calibration (stage 1 focus)"));

  // Baseline left test (phase 1)
  announcePhaseNumber(1);
  delay(1000);
  cueDirection(true);
  runTurnCase(leftBaseline.label, leftBaseline.leftPower, leftBaseline.rightPower, leftBaseline.durationMs);
  Serial.println(F("[TURN] Baseline left complete. Prepare for right variants."));
  delay(5000);

  // Sweep through right variants with audio cues 2..N+1
  for (size_t idx = 0; idx < sizeof(rightCandidates)/sizeof(rightCandidates[0]); ++idx) {
    announcePhaseNumber(static_cast<int>(idx) + 2);
    delay(1000);
    cueDirection(false);
    const TurnTestCase &tc = rightCandidates[idx];
    runTurnCase(tc.label, tc.leftPower, tc.rightPower, tc.durationMs);
    Serial.printf("[TURN] Right test %zu done\n", idx);
    delay(5000);
  }

  Serial.println(F("[TEST] Turning calibration complete."));
}

void testLidar() {
  Serial.println(F("[TEST] TFmini sweep + map dump"));
  size_t samples = performScan(scanBuffer, MAX_SCAN_SAMPLES, 8);
  for (size_t i = 0; i < samples; ++i) {
    if (scanBuffer[i].valid) {
      Serial.printf("Sample %2u | angle %3d deg -> %4u cm (strength %u)\n",
                    static_cast<unsigned>(i),
                    scanBuffer[i].angleDeg,
                    scanBuffer[i].distanceCm,
                    scanBuffer[i].strength);
    } else {
      Serial.printf("Sample %2u | angle %3d deg -> timeout\n",
                    static_cast<unsigned>(i),
                    scanBuffer[i].angleDeg);
    }
  }
}

// -------- Motions (1 = M1 TL, 2 = M2 TR, 3 = M3 BR, 4 = M4 BL) --------
static inline void forward()       { for (int m=1;m<=4;++m) motorForward(m); }
static inline void backward()      { for (int m=1;m<=4;++m) motorBackward(m); }
static inline void pivotLeft()     { motorBackward(1); motorBackward(4); motorForward(2); motorForward(3); }
static inline void pivotRight()    { motorForward(1);  motorForward(4);  motorBackward(2); motorBackward(3); }

// Run a motion for exactly 1s, then stop
void run1s(void (*motion)()) {
  motion();
  delay(1000);
  allStop();
}

// -------- Setup --------
void setupPins() {
  int pins[] = {
    A_AIN1, A_AIN2, A_BIN1, A_BIN2,
    B_AIN1, B_AIN2, B_BIN1, B_BIN2,
    A_PWMA, A_PWMB, B_PWMA, B_PWMB
  };
  for (int i=0;i<(int)(sizeof(pins)/sizeof(pins[0]));++i) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
  if (STBY_PIN >= 0) { pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, HIGH); }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== ElectroCar Autonomous Mode ==="));
  setupPins();
  ensureServoAttached();
  ensureTfminiInitialized();
}

// -------- Main loop --------
void loop() {
  runAutonomyStep();
  delay(200);
}
