#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "driver/i2s.h"

// -------- Pin mapping (from Fritz-A-Phone context) --------
constexpr int PIN_SD_CS   = 27;  // SPI CS for microSD
constexpr int PIN_I2S_BCLK  = 26; // Shared BCLK
constexpr int PIN_I2S_LRCLK = 25; // Shared LRCLK / WS
constexpr int PIN_I2S_DOUT  = 13; // To MAX98357A DIN
constexpr int PIN_I2S_DIN   = 33; // From INMP441 SD

// -------- UI pins (active-low buttons, active-high LEDs) --------
constexpr int PIN_BTN_GREEN  = 17; // TX2 silkscreen
constexpr int PIN_BTN_YELLOW = 32; // labeled "white" in context, physically yellow button
constexpr int PIN_BTN_RED    = 14;
constexpr int PIN_BTN_BLACK  = 16; // RX2 silkscreen

constexpr int PIN_LED_RED   = 4;
constexpr int PIN_LED_GREEN = 5;

// -------- Audio settings --------
constexpr uint32_t SAMPLE_RATE     = 16000;   // Hz
constexpr uint8_t  BITS_PER_SAMPLE = 16;      // store/play 16-bit PCM
constexpr uint8_t  CHANNELS        = 1;       // mono (left)
constexpr uint16_t RECORD_SECONDS  = 5;       // default record length
constexpr char     RECORD_PATH[]   = "/test_record.wav";

// -------- Simple DSP controls --------
// High‑pass: remove DC / rumble. Target fc ≈ 200–300 Hz at 16 kHz Fs.
constexpr float HPF_ALPHA = 0.90f;  // 0.89@300 Hz .. 0.93@200 Hz
// Low‑pass: soften hiss / sibilance. Target fc ≈ 3–4 kHz at 16 kHz Fs.
constexpr float LPF_ALPHA = 0.58f;  // ~3.5 kHz corner
// Noise gate: below this absolute sample value, force silence.
constexpr int16_t GATE_THRESHOLD = 300; // speech typically 2000–8000 on INMP441
// Optional output gain (applied after filters). 1.0 = unity.
constexpr float OUT_GAIN = 1.0f;

// -------- Internal helpers --------
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

bool sdReady = false;
enum class ActionState { Idle, Recording, Playing };
ActionState actionState = ActionState::Idle;

void printHelp();
bool initSD();
bool sdSelfTest();
bool ensureSDReady();
bool startI2SMic();
bool startI2SSpeaker();
void stopI2S();
bool recordToWav(uint16_t seconds, const char *path);
bool playWav(const char *path);
void playTone(uint16_t freq, uint16_t ms);
void setupButtonsAndLeds();
void pollButtons();
void setLeds(bool greenOn, bool redOn);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println();
  Serial.println("Fritz-A-Phone bring-up tester");
  Serial.println("-------------------------------");
  printHelp();

  setupButtonsAndLeds();

  sdReady = initSD();
  if (sdReady) {
    sdSelfTest();
  }
}

void loop() {
  pollButtons();
  delay(10);
}

// -------------------------------------------------------------------
// Utility + diagnostics
// -------------------------------------------------------------------

void printHelp() {
  Serial.println("Button test mode:");
  Serial.println("  Green  -> lights green LED");
  Serial.println("  Red    -> lights red LED and records 5s to SD");
  Serial.println("  Black  -> plays last recording from SD");
  Serial.println("  Yellow -> reserved (no action yet)");
}

bool initSD() {
  Serial.print("Mounting SD ... ");
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("failed");
    sdReady = false;
    return false;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("no card detected");
    sdReady = false;
    return false;
  }
  Serial.print("ok (");
  switch (cardType) {
    case CARD_MMC: Serial.print("MMC"); break;
    case CARD_SD: Serial.print("SDSC"); break;
    case CARD_SDHC: Serial.print("SDHC/SDXC"); break;
    default: Serial.print("Unknown"); break;
  }
  Serial.print(", ");
  Serial.print((uint32_t)SD.cardSize() / (1024 * 1024));
  Serial.println(" MB)");
  sdReady = true;
  return true;
}

bool sdSelfTest() {
  const char *path = "/sd_self_test.txt";
  Serial.print("SD self-test file ... ");
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.println("write open failed");
    return false;
  }
  f.println("Fritz-A-Phone SD self-test line");
  f.close();

  f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.println("read open failed");
    return false;
  }
  String line = f.readStringUntil('\n');
  f.close();
  if (line.length() > 0) {
    Serial.println("ok");
    return true;
  }
  Serial.println("empty read");
  return false;
}

bool ensureSDReady() {
  if (sdReady) return true;
  Serial.println("Mounting SD (lazy init)...");
  sdReady = initSD();
  if (sdReady) sdSelfTest();
  return sdReady;
}

// -------------------------------------------------------------------
// I2S setup helpers
// -------------------------------------------------------------------

void stopI2S() {
  i2s_driver_uninstall(I2S_PORT);
}

bool startI2SMic() {
  stopI2S();
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) {
    Serial.println("I2S driver (RX) install failed");
    return false;
  }

  i2s_pin_config_t pins = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_LRCLK,
    .data_out_num = I2S_PIN_NO_CHANGE, // not driving speaker during capture
    .data_in_num = PIN_I2S_DIN
  };
  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
    Serial.println("I2S pin set (RX) failed");
    stopI2S();
    return false;
  }
  // Microphone outputs MSB-justified 24-bit; shift down later.
  i2s_zero_dma_buffer(I2S_PORT);
  return true;
}

bool startI2SSpeaker() {
  stopI2S();
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) {
    Serial.println("I2S driver (TX) install failed");
    return false;
  }

  i2s_pin_config_t pins = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_LRCLK,
    .data_out_num = PIN_I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
    Serial.println("I2S pin set (TX) failed");
    stopI2S();
    return false;
  }
  i2s_zero_dma_buffer(I2S_PORT);
  return true;
}

// -------------------------------------------------------------------
// WAV helpers
// -------------------------------------------------------------------

void writeWavHeader(File &f, uint32_t dataBytes) {
  f.seek(0);
  const uint32_t byteRate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
  const uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);

  uint8_t hdr[44] = {
    'R','I','F','F',
    0,0,0,0, // chunk size
    'W','A','V','E',
    'f','m','t',' ',
    16,0,0,0,              // subchunk1 size
    1,0,                  // PCM
    CHANNELS,0,
    (uint8_t)(SAMPLE_RATE & 0xFF), (uint8_t)((SAMPLE_RATE >> 8) & 0xFF),
    (uint8_t)((SAMPLE_RATE >> 16) & 0xFF), (uint8_t)((SAMPLE_RATE >> 24) & 0xFF),
    (uint8_t)(byteRate & 0xFF), (uint8_t)((byteRate >> 8) & 0xFF),
    (uint8_t)((byteRate >> 16) & 0xFF), (uint8_t)((byteRate >> 24) & 0xFF),
    (uint8_t)(blockAlign & 0xFF), (uint8_t)((blockAlign >> 8) & 0xFF),
    BITS_PER_SAMPLE,0,
    'd','a','t','a',
    0,0,0,0 // data chunk size
  };

  uint32_t chunkSize = 36 + dataBytes;
  hdr[4]  = (uint8_t)(chunkSize & 0xFF);
  hdr[5]  = (uint8_t)((chunkSize >> 8) & 0xFF);
  hdr[6]  = (uint8_t)((chunkSize >> 16) & 0xFF);
  hdr[7]  = (uint8_t)((chunkSize >> 24) & 0xFF);
  hdr[40] = (uint8_t)(dataBytes & 0xFF);
  hdr[41] = (uint8_t)((dataBytes >> 8) & 0xFF);
  hdr[42] = (uint8_t)((dataBytes >> 16) & 0xFF);
  hdr[43] = (uint8_t)((dataBytes >> 24) & 0xFF);

  f.write(hdr, sizeof(hdr));
}

// -------------------------------------------------------------------
// Record + playback
// -------------------------------------------------------------------

bool recordToWav(uint16_t seconds, const char *path) {
  if (!startI2SMic()) return false;

  File wav = SD.open(path, FILE_WRITE);
  if (!wav) {
    Serial.println("Failed to open record file");
    stopI2S();
    return false;
  }

  writeWavHeader(wav, 0); // placeholder

  const size_t i2sBufSamples = 512;
  int32_t i2sBuf[i2sBufSamples];
  int16_t pcmBuf[i2sBufSamples];

  // Stateful filters
  static float hpfState = 0.0f; // high‑pass state
  static float lpfState = 0.0f; // low‑pass state

  auto processSample = [&](int32_t raw32) -> int16_t {
    // INMP441: 24‑bit left‑justified in 32‑bit word. Use >>11 (original scaling).
    int16_t s = (int16_t)(raw32 >> 11);

    // Noise gate
    if (abs(s) < GATE_THRESHOLD) s = 0;

    // One‑pole high‑pass (DC/rumble removal)
    // y[n] = HPF_ALPHA*(y[n-1] + x[n] - x[n-1])
    static int16_t prevIn = 0;
    float yHP = HPF_ALPHA * (hpfState + (float)s - (float)prevIn);
    hpfState = yHP;
    prevIn = s;

    // One‑pole low‑pass (de‑hiss / tame sibilance)
    float yLP = lpfState + LPF_ALPHA * (yHP - lpfState);
    lpfState = yLP;

    // Optional gain and clip
    int32_t out = (int32_t)(yLP * OUT_GAIN);
    if (out > INT16_MAX) out = INT16_MAX;
    if (out < INT16_MIN) out = INT16_MIN;
    return (int16_t)out;
  };

  const uint32_t totalSamplesTarget = (uint32_t)seconds * SAMPLE_RATE;
  uint32_t samplesWritten = 0;

  while (samplesWritten < totalSamplesTarget) {
    size_t bytesRead = 0;
    esp_err_t res = i2s_read(I2S_PORT, (void *)i2sBuf, sizeof(i2sBuf), &bytesRead, portMAX_DELAY);
    if (res != ESP_OK || bytesRead == 0) {
      Serial.println("I2S read error");
      wav.close();
      stopI2S();
      return false;
    }
    size_t samplesRead = bytesRead / sizeof(int32_t);
    for (size_t i = 0; i < samplesRead; ++i) {
      pcmBuf[i] = processSample(i2sBuf[i]);
    }
    size_t bytesToWrite = samplesRead * sizeof(int16_t);
    wav.write((uint8_t *)pcmBuf, bytesToWrite);
    samplesWritten += samplesRead;
  }

  uint32_t dataBytes = samplesWritten * sizeof(int16_t);
  writeWavHeader(wav, dataBytes);
  wav.close();
  stopI2S();

  Serial.print("Saved ");
  Serial.print(seconds);
  Serial.print("s to ");
  Serial.print(path);
  Serial.print(" (");
  Serial.print(dataBytes / 1024);
  Serial.println(" KB)");
  return true;
}

bool playWav(const char *path) {
  if (!SD.exists(path)) {
    Serial.println("File not found on SD.");
    return false;
  }
  File wav = SD.open(path, FILE_READ);
  if (!wav) {
    Serial.println("Open failed.");
    return false;
  }

  // Skip WAV header (44 bytes)
  if (!startI2SSpeaker()) {
    wav.close();
    return false;
  }
  wav.seek(44);

  // Reset filter state for playback so we don't carry record-time history.
  // (Playback currently writes raw file to speaker; we leave it flat.)

  const size_t bufBytes = 1024;
  uint8_t buf[bufBytes];
  while (true) {
    int n = wav.read(buf, bufBytes);
    if (n <= 0) break;
    size_t written = 0;
    while (written < (size_t)n) {
      size_t out = 0;
      if (i2s_write(I2S_PORT, buf + written, n - written, &out, portMAX_DELAY) != ESP_OK) {
        Serial.println("I2S write error");
        wav.close();
        stopI2S();
        return false;
      }
      written += out;
    }
  }
  wav.close();
  stopI2S();
  Serial.println("Playback done.");
  return true;
}

void playTone(uint16_t freq, uint16_t ms) {
  const float twoPiF = 2.0f * PI * freq;
  const uint16_t samples = (SAMPLE_RATE * ms) / 1000;
  for (uint16_t i = 0; i < samples; ++i) {
    float t = (float)i / SAMPLE_RATE;
    float s = sinf(twoPiF * t);
    int16_t sample = (int16_t)(s * 30000); // moderate volume
    size_t out;
    i2s_write(I2S_PORT, &sample, sizeof(sample), &out, portMAX_DELAY);
  }
}

// -------------------------------------------------------------------
// Button + LED helpers
// -------------------------------------------------------------------

void setupButtonsAndLeds() {
  pinMode(PIN_BTN_GREEN, INPUT_PULLUP);
  pinMode(PIN_BTN_YELLOW, INPUT_PULLUP);
  pinMode(PIN_BTN_RED, INPUT_PULLUP);
  pinMode(PIN_BTN_BLACK, INPUT_PULLUP);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  setLeds(false, false);
}

void setLeds(bool greenOn, bool redOn) {
  digitalWrite(PIN_LED_GREEN, greenOn ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,   redOn   ? HIGH : LOW);
}

void pollButtons() {
  static bool prevGreen = false;
  static bool prevYellow = false;
  static bool prevRed = false;
  static bool prevBlack = false;

  static uint32_t lastGreenMs = 0;
  static uint32_t lastYellowMs = 0;
  static uint32_t lastRedMs = 0;
  static uint32_t lastBlackMs = 0;

  constexpr uint32_t debounceMs = 60;
  const uint32_t now = millis();

  auto pressedEvent = [&](int pin, bool &prevState, uint32_t &lastChangeMs) -> bool {
    bool pressed = (digitalRead(pin) == LOW); // active-low
    if (pressed != prevState && (now - lastChangeMs) > debounceMs) {
      prevState = pressed;
      lastChangeMs = now;
      return pressed; // true only on new press
    }
    return false;
  };

  if (pressedEvent(PIN_BTN_GREEN, prevGreen, lastGreenMs)) {
    Serial.println("Green pressed -> green LED");
    setLeds(true, false);
  }

  if (pressedEvent(PIN_BTN_RED, prevRed, lastRedMs)) {
    if (actionState == ActionState::Idle) {
      actionState = ActionState::Recording;
      Serial.println("Red pressed -> record 5s to SD");
      setLeds(false, true);
      if (!ensureSDReady() || !recordToWav(RECORD_SECONDS, RECORD_PATH)) {
        Serial.println("Recording failed (SD or I2S)");
      } else {
        Serial.println("Recording complete.");
      }
      setLeds(false, false);
      actionState = ActionState::Idle;
    } else {
      Serial.println("Busy; red press ignored");
    }
  }

  if (pressedEvent(PIN_BTN_BLACK, prevBlack, lastBlackMs)) {
    if (actionState == ActionState::Idle) {
      actionState = ActionState::Playing;
      Serial.println("Black pressed -> play last recording");
      setLeds(false, false);
      if (!ensureSDReady() || !playWav(RECORD_PATH)) {
        Serial.println("Playback failed (missing file or I2S)");
      }
      actionState = ActionState::Idle;
    } else {
      Serial.println("Busy; black press ignored");
    }
  }

  if (pressedEvent(PIN_BTN_YELLOW, prevYellow, lastYellowMs)) {
    if (actionState == ActionState::Idle) {
      actionState = ActionState::Playing;
      Serial.println("Yellow pressed -> speaker test tones");
      setLeds(true, true);

      if (!startI2SSpeaker()) {
        Serial.println("Speaker test failed: I2S speaker init failed");
      } else {
        // A few distinct tones to help judge clarity / distortion
        playTone(440, 300);   // A4
        delay(80);
        playTone(880, 300);   // A5
        delay(80);
        playTone(1200, 300);  // brighter tone
        delay(80);

        // Short sweep upward
        for (int f = 300; f <= 2000; f += 100) {
          playTone(f, 35);
        }

        stopI2S();
        Serial.println("Speaker test complete.");
      }

      setLeds(false, false);
      actionState = ActionState::Idle;
    } else {
      Serial.println("Busy; yellow press ignored");
    }
  }

  // When idle, mirror held buttons on LEDs
  if (actionState == ActionState::Idle) {
    bool greenHeld = (digitalRead(PIN_BTN_GREEN) == LOW);
    bool redHeld = (digitalRead(PIN_BTN_RED) == LOW);
    setLeds(greenHeld, redHeld);
  }
}
