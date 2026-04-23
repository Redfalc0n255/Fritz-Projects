#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "driver/i2s.h"

constexpr int PIN_SD_CS = 27;
constexpr int PIN_I2S_BCLK = 26;
constexpr int PIN_I2S_LRCLK = 25;
constexpr int PIN_I2S_DOUT = 13;
constexpr int PIN_I2S_DIN = 33;

constexpr int PIN_BTN_B1 = 17;
constexpr int PIN_BTN_B2 = 16;
constexpr int PIN_BTN_B3 = 32;
constexpr int PIN_BTN_B4 = 14;

constexpr int PIN_LED_RED = 4;
constexpr int PIN_LED_GREEN = 5;

constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr uint32_t SAMPLE_RATE = 16000;
constexpr uint8_t BITS_PER_SAMPLE = 16;
constexpr uint8_t CHANNELS = 1;
constexpr size_t RECORD_CHUNK_SAMPLES = 512;
constexpr char RECORD_RAW_PATH[] = "/audio_tester_raw.pcm";
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;

enum class ActionState {
  Idle,
  Recording,
  Playing
};

struct ButtonState {
  int pin;
  bool stablePressed;
  bool lastRawPressed;
  unsigned long lastDebounceMs;
};

struct AudioTuningSettings {
  float inputGain = 1.0f;
  float outputGain = 1.0f;
  float hpfAlpha = 0.90f;
  float lpfAlpha = 0.58f;
  int gateThreshold = 300;
  int inputShift = 11;
  bool gateEnabled = true;
  bool hpfEnabled = true;
  bool lpfEnabled = true;
  bool liveStats = true;
};

struct RecorderState {
  bool active = false;
  File file;
  uint32_t dataBytes = 0;
  unsigned long startedMs = 0;
  float hpfState = 0.0f;
  float lpfState = 0.0f;
  int16_t prevIn = 0;
  int16_t sessionPeak = 0;
  unsigned long lastStatsMs = 0;
};

bool sdReady = false;
bool i2sInstalled = false;
ActionState actionState = ActionState::Idle;
AudioTuningSettings tuning;
RecorderState recorder;
ButtonState buttonB1;
ButtonState buttonB2;
ButtonState buttonB3;
ButtonState buttonB4;
String serialBuffer;
int currentScaleNoteIndex = -1;
constexpr uint16_t SCALE_NOTES[] = {415, 440, 466, 494, 523, 554};
constexpr int SCALE_NOTE_COUNT = sizeof(SCALE_NOTES) / sizeof(SCALE_NOTES[0]);

void printHelp();
void printSettings();
void printQuickCommands();
bool initSD();
bool ensureSDReady();
void setupButtonsAndLeds();
void setLeds(bool greenOn, bool redOn);
void stopI2S();
bool startI2SMic();
bool startI2SSpeaker();
void writeWavHeader(File& file, uint32_t dataBytes);
bool beginRecording();
bool continueRecording();
bool finishRecording();
bool cancelRecording();
bool playProcessedRecording();
void playToneWithGain(uint16_t freq, uint16_t ms, float gain);
void playScaleNote(int noteIndex);
void playRouteBeepMask(uint8_t mask);
void printScaleNotes();
void handleSerialInput();
void processSerialCommand(const String& command);
bool parseSetCommand(const String& command);
bool updateButton(ButtonState& button, bool& pressedEdge, bool& releasedEdge);
void pollButtons();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  setupButtonsAndLeds();
  printHelp();

  sdReady = initSD();
  printSettings();
}

void loop() {
  handleSerialInput();
  pollButtons();

  if (recorder.active) {
    if (!continueRecording()) {
      Serial.println("Recording failed. Cancelling current take.");
      cancelRecording();
      setLeds(false, false);
    }
  }

  delay(5);
}

void printHelp() {
  Serial.println();
  Serial.println("ESP32 Audio Tester");
  Serial.println("------------------");
  Serial.println("Buttons:");
  Serial.println("  B1 hold  -> record while held");
  Serial.println("  B2 click -> play last recording through current DSP settings");
  Serial.println("  B3 click -> print current settings");
  Serial.println("  B4 click -> cycle and play next scale note");
  Serial.println();
  Serial.println("Serial commands:");
  printQuickCommands();
}

void printQuickCommands() {
  Serial.println("  help");
  Serial.println("  show");
  Serial.println("  set input_gain <float>");
  Serial.println("  set output_gain <float>");
  Serial.println("  set hpf <float>");
  Serial.println("  set lpf <float>");
  Serial.println("  set gate <int>");
  Serial.println("  set shift <int>");
  Serial.println("  set gate_on <0|1>");
  Serial.println("  set hpf_on <0|1>");
  Serial.println("  set lpf_on <0|1>");
  Serial.println("  set stats <0|1>");
  Serial.println("  tone");
  Serial.println("  note <0-5>");
  Serial.println("  notenext");
  Serial.println("  listnotes");
  Serial.println("  routebeep <1-7>");
  Serial.println("  play");
  Serial.println("  delete");
  Serial.println();
}

void printSettings() {
  Serial.println("Current DSP settings:");
  Serial.print("  input_gain   = "); Serial.println(tuning.inputGain, 3);
  Serial.print("  output_gain  = "); Serial.println(tuning.outputGain, 3);
  Serial.print("  hpf_alpha    = "); Serial.println(tuning.hpfAlpha, 3);
  Serial.print("  lpf_alpha    = "); Serial.println(tuning.lpfAlpha, 3);
  Serial.print("  gate_thresh  = "); Serial.println(tuning.gateThreshold);
  Serial.print("  input_shift  = "); Serial.println(tuning.inputShift);
  Serial.print("  gate_enabled = "); Serial.println(tuning.gateEnabled ? "true" : "false");
  Serial.print("  hpf_enabled  = "); Serial.println(tuning.hpfEnabled ? "true" : "false");
  Serial.print("  lpf_enabled  = "); Serial.println(tuning.lpfEnabled ? "true" : "false");
  Serial.print("  live_stats   = "); Serial.println(tuning.liveStats ? "true" : "false");
  Serial.print("  current_note = "); Serial.println(currentScaleNoteIndex);
  Serial.println();
}

void printScaleNotes() {
  Serial.println("Scale notes:");
  for (int i = 0; i < SCALE_NOTE_COUNT; ++i) {
    Serial.print("  ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(SCALE_NOTES[i]);
    Serial.println(" Hz");
  }
  Serial.println();
}

bool initSD() {
  Serial.print("Mounting SD ... ");
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("failed");
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("no card");
    return false;
  }

  Serial.print("ok, size=");
  Serial.print((uint32_t)SD.cardSize() / (1024UL * 1024UL));
  Serial.println(" MB");
  return true;
}

bool ensureSDReady() {
  if (sdReady) return true;
  sdReady = initSD();
  return sdReady;
}

void setupButtonsAndLeds() {
  pinMode(PIN_BTN_B1, INPUT_PULLUP);
  pinMode(PIN_BTN_B2, INPUT_PULLUP);
  pinMode(PIN_BTN_B3, INPUT_PULLUP);
  pinMode(PIN_BTN_B4, INPUT_PULLUP);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  buttonB1 = {PIN_BTN_B1, false, false, 0};
  buttonB2 = {PIN_BTN_B2, false, false, 0};
  buttonB3 = {PIN_BTN_B3, false, false, 0};
  buttonB4 = {PIN_BTN_B4, false, false, 0};

  setLeds(false, false);
}

void setLeds(bool greenOn, bool redOn) {
  digitalWrite(PIN_LED_GREEN, greenOn ? HIGH : LOW);
  digitalWrite(PIN_LED_RED, redOn ? HIGH : LOW);
}

void stopI2S() {
  if (!i2sInstalled) return;
  i2s_driver_uninstall(I2S_PORT);
  i2sInstalled = false;
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
    .fixed_mclk = 0,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
  };

  if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) {
    Serial.println("I2S mic install failed");
    return false;
  }
  i2sInstalled = true;

  i2s_pin_config_t pins = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_LRCLK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_DIN
  };

  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
    Serial.println("I2S mic pin config failed");
    stopI2S();
    return false;
  }

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
    .fixed_mclk = 0,
    .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
  };

  if (i2s_driver_install(I2S_PORT, &cfg, 0, nullptr) != ESP_OK) {
    Serial.println("I2S speaker install failed");
    return false;
  }
  i2sInstalled = true;

  i2s_pin_config_t pins = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_LRCLK,
    .data_out_num = PIN_I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
    Serial.println("I2S speaker pin config failed");
    stopI2S();
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  return true;
}

void writeWavHeader(File& file, uint32_t dataBytes) {
  file.seek(0);
  const uint32_t byteRate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
  const uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);

  uint8_t hdr[44] = {
    'R','I','F','F',
    0,0,0,0,
    'W','A','V','E',
    'f','m','t',' ',
    16,0,0,0,
    1,0,
    CHANNELS,0,
    (uint8_t)(SAMPLE_RATE & 0xFF), (uint8_t)((SAMPLE_RATE >> 8) & 0xFF),
    (uint8_t)((SAMPLE_RATE >> 16) & 0xFF), (uint8_t)((SAMPLE_RATE >> 24) & 0xFF),
    (uint8_t)(byteRate & 0xFF), (uint8_t)((byteRate >> 8) & 0xFF),
    (uint8_t)((byteRate >> 16) & 0xFF), (uint8_t)((byteRate >> 24) & 0xFF),
    (uint8_t)(blockAlign & 0xFF), (uint8_t)((blockAlign >> 8) & 0xFF),
    BITS_PER_SAMPLE,0,
    'd','a','t','a',
    0,0,0,0
  };

  uint32_t chunkSize = 36 + dataBytes;
  hdr[4] = (uint8_t)(chunkSize & 0xFF);
  hdr[5] = (uint8_t)((chunkSize >> 8) & 0xFF);
  hdr[6] = (uint8_t)((chunkSize >> 16) & 0xFF);
  hdr[7] = (uint8_t)((chunkSize >> 24) & 0xFF);
  hdr[40] = (uint8_t)(dataBytes & 0xFF);
  hdr[41] = (uint8_t)((dataBytes >> 8) & 0xFF);
  hdr[42] = (uint8_t)((dataBytes >> 16) & 0xFF);
  hdr[43] = (uint8_t)((dataBytes >> 24) & 0xFF);

  file.write(hdr, sizeof(hdr));
}

bool beginRecording() {
  if (actionState != ActionState::Idle) return false;
  if (!ensureSDReady()) return false;
  if (!startI2SMic()) return false;

  if (SD.exists(RECORD_RAW_PATH)) {
    SD.remove(RECORD_RAW_PATH);
  }

  recorder.file = SD.open(RECORD_RAW_PATH, FILE_WRITE);
  if (!recorder.file) {
    Serial.println("Failed to open recording file");
    stopI2S();
    return false;
  }

  recorder.active = true;
  recorder.dataBytes = 0;
  recorder.startedMs = millis();
  recorder.hpfState = 0.0f;
  recorder.lpfState = 0.0f;
  recorder.prevIn = 0;
  recorder.sessionPeak = 0;
  recorder.lastStatsMs = 0;
  actionState = ActionState::Recording;
  setLeds(true, false);
  Serial.println("Recording started");
  return true;
}

bool continueRecording() {
  if (!recorder.active) return false;

  int32_t i2sBuf[RECORD_CHUNK_SAMPLES];
  size_t bytesRead = 0;

  esp_err_t res = i2s_read(I2S_PORT, (void*)i2sBuf, sizeof(i2sBuf), &bytesRead, portMAX_DELAY);
  if (res != ESP_OK || bytesRead == 0) {
    Serial.println("I2S read error during tester recording");
    return false;
  }

  size_t samplesRead = bytesRead / sizeof(int32_t);
  int16_t chunkPeak = 0;

  for (size_t i = 0; i < samplesRead; ++i) {
    int16_t rawPreview = (int16_t)(i2sBuf[i] >> 11);
    int16_t absSample = (int16_t)min(fabsf((float)rawPreview), 32767.0f);
    if (absSample > chunkPeak) chunkPeak = absSample;
    if (absSample > recorder.sessionPeak) recorder.sessionPeak = absSample;
  }

  size_t bytesToWrite = samplesRead * sizeof(int32_t);
  if (recorder.file.write((uint8_t*)i2sBuf, bytesToWrite) != bytesToWrite) {
    Serial.println("SD write error during tester recording");
    return false;
  }

  recorder.dataBytes += bytesToWrite;

  if (tuning.liveStats && millis() - recorder.lastStatsMs >= 500) {
    recorder.lastStatsMs = millis();
    Serial.print("Chunk peak=");
    Serial.print(chunkPeak);
    Serial.print(" sessionPeak=");
    Serial.print(recorder.sessionPeak);
    Serial.print(" bytes=");
    Serial.println(recorder.dataBytes);
  }

  return true;
}

bool finishRecording() {
  if (!recorder.active) return false;

  recorder.file.close();
  stopI2S();

  unsigned long durationMs = millis() - recorder.startedMs;
  recorder.active = false;
  actionState = ActionState::Idle;
  setLeds(false, false);

  Serial.print("Dry recording saved to ");
  Serial.print(RECORD_RAW_PATH);
  Serial.print(" durationMs=");
  Serial.print(durationMs);
  Serial.print(" bytes=");
  Serial.print(recorder.dataBytes);
  Serial.print(" sessionPeak=");
  Serial.println(recorder.sessionPeak);
  return true;
}

bool cancelRecording() {
  if (recorder.active) {
    recorder.file.close();
  }
  stopI2S();
  recorder.active = false;
  actionState = ActionState::Idle;
  if (SD.exists(RECORD_RAW_PATH)) {
    SD.remove(RECORD_RAW_PATH);
  }
  return true;
}

bool playProcessedRecording() {
  if (actionState != ActionState::Idle) {
    Serial.println("Busy. Playback ignored.");
    return false;
  }

  if (!SD.exists(RECORD_RAW_PATH)) {
    Serial.println("No recording found.");
    return false;
  }

  File rawFile = SD.open(RECORD_RAW_PATH, FILE_READ);
  if (!rawFile) {
    Serial.println("Failed to open recording.");
    return false;
  }

  if (!startI2SSpeaker()) {
    rawFile.close();
    return false;
  }

  actionState = ActionState::Playing;
  setLeds(false, true);

  const size_t sampleCount = RECORD_CHUNK_SAMPLES;
  int32_t rawBuf[sampleCount];
  int16_t sampleBuf[sampleCount];
  float hpfState = 0.0f;
  float lpfState = 0.0f;
  int16_t prevIn = 0;

  while (true) {
    int n = rawFile.read((uint8_t*)rawBuf, sizeof(rawBuf));
    if (n <= 0) break;

    size_t samples = n / sizeof(int32_t);
    for (size_t i = 0; i < samples; ++i) {
      float sample = (float)((int16_t)(rawBuf[i] >> tuning.inputShift));
      sample *= tuning.inputGain;

      int16_t absSample = (int16_t)min(fabsf(sample), 32767.0f);
      if (tuning.gateEnabled && tuning.gateThreshold > 0) {
        float gate = absSample / (float)tuning.gateThreshold;
        if (gate < 1.0f) {
          gate *= gate;
          sample *= gate;
        }
      }

      if (tuning.hpfEnabled) {
        float yHP = tuning.hpfAlpha * (hpfState + sample - (float)prevIn);
        hpfState = yHP;
        prevIn = (int16_t)sample;
        sample = yHP;
      } else {
        prevIn = (int16_t)sample;
      }

      if (tuning.lpfEnabled) {
        float yLP = lpfState + tuning.lpfAlpha * (sample - lpfState);
        lpfState = yLP;
        sample = yLP;
      }

      sample *= tuning.outputGain;
      if (sample > 32767.0f) sample = 32767.0f;
      if (sample < -32768.0f) sample = -32768.0f;
      sampleBuf[i] = (int16_t)sample;
    }

    size_t bytesWritten = 0;
    uint8_t* raw = (uint8_t*)sampleBuf;
    size_t bytesTotal = samples * sizeof(int16_t);
    while (bytesWritten < bytesTotal) {
      size_t out = 0;
      if (i2s_write(I2S_PORT, raw + bytesWritten, bytesTotal - bytesWritten, &out, portMAX_DELAY) != ESP_OK) {
        rawFile.close();
        stopI2S();
        actionState = ActionState::Idle;
        setLeds(false, false);
        Serial.println("Playback failed");
        return false;
      }
      bytesWritten += out;
    }
  }

  rawFile.close();
  stopI2S();
  actionState = ActionState::Idle;
  setLeds(false, false);
  Serial.println("Processed playback done");
  return true;
}

void playToneWithGain(uint16_t freq, uint16_t ms, float gain) {
  if (!startI2SSpeaker()) {
    Serial.println("Speaker init failed for beep test.");
    return;
  }

  setLeds(false, true);
  const float twoPiF = 2.0f * PI * freq;
  const uint16_t samples = (SAMPLE_RATE * ms) / 1000;

  for (uint16_t i = 0; i < samples; ++i) {
    float t = (float)i / SAMPLE_RATE;
    float s = sinf(twoPiF * t);
    float sampleValue = s * 26000.0f * gain;
    if (sampleValue > 32767.0f) sampleValue = 32767.0f;
    if (sampleValue < -32768.0f) sampleValue = -32768.0f;
    int16_t sample = (int16_t)sampleValue;
    size_t out = 0;
    i2s_write(I2S_PORT, &sample, sizeof(sample), &out, portMAX_DELAY);
  }

  stopI2S();
  setLeds(false, false);
}

void playScaleNote(int noteIndex) {
  currentScaleNoteIndex = (noteIndex % SCALE_NOTE_COUNT + SCALE_NOTE_COUNT) % SCALE_NOTE_COUNT;

  Serial.print("Playing scale note ");
  Serial.print(currentScaleNoteIndex);
  Serial.print(" -> ");
  Serial.print(SCALE_NOTES[currentScaleNoteIndex]);
  Serial.println(" Hz");

  playToneWithGain(SCALE_NOTES[currentScaleNoteIndex], 160, 1.0f);
}

void playRouteBeepMask(uint8_t mask) {
  static const uint16_t routeTones[3] = {392, 494, 587};

  Serial.print("Playing route beep mask ");
  Serial.println(mask);

  for (uint8_t i = 0; i < 3; ++i) {
    if ((mask & (1u << i)) == 0) continue;
    playToneWithGain(routeTones[i], 135, 1.0f);
    delay(60);
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      String command = serialBuffer;
      serialBuffer = "";
      command.trim();
      if (command.length() > 0) {
        processSerialCommand(command);
      }
      continue;
    }
    serialBuffer += ch;
  }
}

void processSerialCommand(const String& command) {
  String lower = command;
  lower.toLowerCase();

  if (lower == "help") {
    printHelp();
    return;
  }

  if (lower == "show") {
    printSettings();
    return;
  }

  if (lower == "listnotes") {
    printScaleNotes();
    return;
  }

  if (lower == "play") {
    playProcessedRecording();
    return;
  }

  if (lower == "delete") {
    if (SD.exists(RECORD_RAW_PATH)) {
      SD.remove(RECORD_RAW_PATH);
      Serial.println("Deleted last test recording.");
    } else {
      Serial.println("No recording to delete.");
    }
    return;
  }

  if (lower == "tone") {
    if (startI2SSpeaker()) {
      const uint16_t freqs[] = {330, 440, 660, 880};
      for (uint16_t freq : freqs) {
        const uint16_t samples = (SAMPLE_RATE * 160) / 1000;
        for (uint16_t i = 0; i < samples; ++i) {
          float t = (float)i / SAMPLE_RATE;
          float s = sinf(2.0f * PI * freq * t);
          float sampleValue = s * 24000.0f * tuning.outputGain;
          if (sampleValue > 32767.0f) sampleValue = 32767.0f;
          if (sampleValue < -32768.0f) sampleValue = -32768.0f;
          int16_t sample = (int16_t)sampleValue;
          size_t out = 0;
          i2s_write(I2S_PORT, &sample, sizeof(sample), &out, portMAX_DELAY);
        }
        delay(40);
      }
      stopI2S();
      Serial.println("Speaker tone test complete.");
    }
    return;
  }

  if (lower == "notenext") {
    playScaleNote(currentScaleNoteIndex + 1);
    return;
  }

  if (lower.startsWith("note ")) {
    int noteIndex = lower.substring(5).toInt();
    playScaleNote(noteIndex);
    return;
  }

  if (lower == "beepnext") {
    playScaleNote(currentScaleNoteIndex + 1);
    return;
  }

  if (lower.startsWith("beep ")) {
    int noteIndex = lower.substring(5).toInt();
    playScaleNote(noteIndex);
    return;
  }

  if (lower.startsWith("routebeep ")) {
    int mask = lower.substring(10).toInt();
    mask = constrain(mask, 1, 7);
    playRouteBeepMask((uint8_t)mask);
    return;
  }

  if (parseSetCommand(command)) {
    printSettings();
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(command);
  printQuickCommands();
}

bool parseSetCommand(const String& command) {
  if (!command.startsWith("set ")) return false;

  int firstSpace = command.indexOf(' ');
  int secondSpace = command.indexOf(' ', firstSpace + 1);
  if (secondSpace < 0) {
    Serial.println("Use: set <name> <value>");
    return true;
  }

  String key = command.substring(firstSpace + 1, secondSpace);
  String value = command.substring(secondSpace + 1);
  key.trim();
  value.trim();
  key.toLowerCase();

  if (key == "input_gain") {
    tuning.inputGain = value.toFloat();
  } else if (key == "output_gain") {
    tuning.outputGain = value.toFloat();
  } else if (key == "hpf") {
    tuning.hpfAlpha = constrain(value.toFloat(), 0.0f, 0.999f);
  } else if (key == "lpf") {
    tuning.lpfAlpha = constrain(value.toFloat(), 0.0f, 0.999f);
  } else if (key == "gate") {
    tuning.gateThreshold = (int)max(0L, value.toInt());
  } else if (key == "shift") {
    tuning.inputShift = constrain(value.toInt(), 0, 15);
  } else if (key == "gate_on") {
    tuning.gateEnabled = value.toInt() != 0;
  } else if (key == "hpf_on") {
    tuning.hpfEnabled = value.toInt() != 0;
  } else if (key == "lpf_on") {
    tuning.lpfEnabled = value.toInt() != 0;
  } else if (key == "stats") {
    tuning.liveStats = value.toInt() != 0;
  } else {
    Serial.print("Unknown set key: ");
    Serial.println(key);
  }

  return true;
}

bool updateButton(ButtonState& button, bool& pressedEdge, bool& releasedEdge) {
  pressedEdge = false;
  releasedEdge = false;

  bool rawPressed = (digitalRead(button.pin) == LOW);
  unsigned long now = millis();

  if (rawPressed != button.lastRawPressed) {
    button.lastRawPressed = rawPressed;
    button.lastDebounceMs = now;
  }

  if (now - button.lastDebounceMs < BUTTON_DEBOUNCE_MS) {
    return button.stablePressed;
  }

  if (rawPressed != button.stablePressed) {
    button.stablePressed = rawPressed;
    if (button.stablePressed) {
      pressedEdge = true;
    } else {
      releasedEdge = true;
    }
  }

  return button.stablePressed;
}

void pollButtons() {
  bool b1PressedEdge = false;
  bool b1ReleasedEdge = false;
  bool b2PressedEdge = false;
  bool b2ReleasedEdge = false;
  bool b3PressedEdge = false;
  bool b3ReleasedEdge = false;
  bool b4PressedEdge = false;
  bool b4ReleasedEdge = false;

  bool b1Held = updateButton(buttonB1, b1PressedEdge, b1ReleasedEdge);
  updateButton(buttonB2, b2PressedEdge, b2ReleasedEdge);
  updateButton(buttonB3, b3PressedEdge, b3ReleasedEdge);
  updateButton(buttonB4, b4PressedEdge, b4ReleasedEdge);

  if (b1PressedEdge && actionState == ActionState::Idle) {
    beginRecording();
  }

  if (b1ReleasedEdge && recorder.active) {
    finishRecording();
  }

  if (b2PressedEdge && actionState == ActionState::Idle) {
    playProcessedRecording();
  }

  if (b3PressedEdge) {
    printSettings();
  }

  if (b4PressedEdge) {
    playScaleNote(currentScaleNoteIndex + 1);
  }

  if (actionState == ActionState::Idle) {
    setLeds(b1Held, false);
  }
}
