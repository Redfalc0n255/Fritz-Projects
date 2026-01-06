#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>
#include <SPI.h>
#include <SD.h>

// -------------------------------
// PIN DEFINITIONS
// -------------------------------
#define LED_PIN         3
#define NUM_LEDS        16

#define GREEN_BUTTON_PIN 9     // Green button to GND, uses INPUT_PULLUP
#define RED_BUTTON_PIN   10    // Red button to GND, uses INPUT_PULLUP
#define BUTTON_ACTIVE_STATE LOW

#define I2S_BCLK_PIN    1
#define I2S_LRC_PIN     2
#define I2S_DOUT_PIN    0

// SD card SPI pins
#define SD_MISO         5
#define SD_MOSI         6
#define SD_SCK          4
#define SD_CS           7

// -------------------------------
// CONSTANTS
// -------------------------------
const uint8_t LED_BRIGHTNESS      = 40;
const uint32_t BUTTON_DEBOUNCE_MS = 50;
const uint32_t PRESS_BLINK_MS     = 500;
const uint32_t POWER_OFF_HOLD_MS  = 2000;
const size_t MAX_WAV_FILES        = 64;
const size_t I2S_CHUNK_BYTES      = 512;

// -------------------------------
// GLOBALS
// -------------------------------
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
SPIClass spiSD(FSPI);

struct WavInfo {
  uint32_t sampleRate;
  uint16_t bitsPerSample;
  uint16_t numChannels;
  uint32_t dataStart;
};

String wavFiles[MAX_WAV_FILES];
size_t wavCount = 0;
int currentTrack = -1;

bool powerOn = false;
bool isPlaying = false;
volatile bool nextTrackRequested = false;
volatile bool powerOffRequested = false;

bool lastGreenState = HIGH;
bool lastRedState = HIGH;
uint32_t lastGreenChangeMs = 0;
uint32_t lastRedChangeMs = 0;

bool blinkActive = false;
uint32_t blinkStartMs = 0;
uint32_t currentPlayingColor = 0;

// -------------------------------
// HELPERS: LED
// -------------------------------
void setLedOff() {
  strip.clear();
  strip.show();
}

//sets color to green
void setLedGreen() {
  strip.fill(strip.Color(0, 255, 0));
  strip.show();
}

//sets color to red
void setLedRed() {
  strip.fill(strip.Color(255, 0, 0));
  strip.show();
}

uint32_t pickRandomColor() {
  uint8_t r = random(0, 256);
  uint8_t g = random(0, 256);
  uint8_t b = random(0, 256);
  return strip.Color(r, g, b);
}

void applyStateLeds() {
  if (!powerOn) {
    setLedOff();
    return;
  }
  if (isPlaying) {
    strip.fill(currentPlayingColor);
    strip.show();
  } else {
    setLedGreen();
  }
}
//Calls blink
void triggerBlink() {
  blinkActive = true;
  blinkStartMs = millis();
}

//Blinks when called.
void updateBlink() {
  if (!blinkActive) return;
  uint32_t elapsed = millis() - blinkStartMs;
  if (elapsed >= PRESS_BLINK_MS) {
    blinkActive = false;
    applyStateLeds();
    return;
  }

  float phase = (float)elapsed / (float)PRESS_BLINK_MS;
  float brightness = phase < 0.5f ? phase * 2.0f : (1.0f - phase) * 2.0f;
  uint8_t level = (uint8_t)(brightness * 255);

  strip.fill(strip.Color(level, level, level));
  strip.show();
}

// -------------------------------
// WAV + I2S
// -------------------------------
//I2S is the communication playform that our speaker uses to translate code into speaker code. These are all the cariables that we can change to make the speaker 
//do different stuff, honestly dont really touch any of these, these are the only values that work for the wav files I am using.
void setupI2S(uint32_t sampleRate) {
  i2s_driver_uninstall(I2S_NUM_0);

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = (int)sampleRate,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 256,
      .use_apll = true,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0};

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK_PIN,
      .ws_io_num = I2S_LRC_PIN,
      .data_out_num = I2S_DOUT_PIN,
      .data_in_num = -1};

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

bool readWavHeader(File &f, WavInfo &info) {
  if (!f) return false;

  uint8_t header[44];
  if (f.read(header, 44) != 44) return false;

  if (header[0] != 'R' || header[1] != 'I' || header[2] != 'F' || header[3] != 'F') return false;
  if (header[8] != 'W' || header[9] != 'A' || header[10] != 'V' || header[11] != 'E') return false;

  info.numChannels = header[22] | (header[23] << 8);
  info.sampleRate = header[24] | (header[25] << 8) | (header[26] << 16) | (header[27] << 24);
  info.bitsPerSample = header[34] | (header[35] << 8);
  info.dataStart = 44;

  return info.bitsPerSample == 16;
}

// -------------------------------
// SD FILE HANDLING
// -------------------------------
void loadWavList() {
  wavCount = 0;
  currentTrack = -1;

  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println("SD root not available");
    return;
  }

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory() && wavCount < MAX_WAV_FILES) {
      String name = entry.name();
      String lower = name;
      lower.toLowerCase();
      if (lower.endsWith(".wav")) {
        wavFiles[wavCount++] = "/" + name;
      }
    }
    entry.close();
  }
  root.close();
  Serial.printf("Found %u wav files.\n", (unsigned)wavCount);
}

String nextTrackPath() {
  if (wavCount == 0) return "";
  currentTrack = (currentTrack + 1) % (int)wavCount;
  return wavFiles[currentTrack];
}

// -------------------------------
// BUTTON HANDLING
// -------------------------------
void handleButtons() {
  uint32_t now = millis();

  bool gState = digitalRead(GREEN_BUTTON_PIN);
  if (gState != lastGreenState && (now - lastGreenChangeMs) > BUTTON_DEBOUNCE_MS) {
    lastGreenChangeMs = now;
    lastGreenState = gState;
    if (gState == BUTTON_ACTIVE_STATE && powerOn) {
      nextTrackRequested = true;
      triggerBlink();
    }
  }

  bool rState = digitalRead(RED_BUTTON_PIN);
  if (rState != lastRedState && (now - lastRedChangeMs) > BUTTON_DEBOUNCE_MS) {
    lastRedChangeMs = now;
    lastRedState = rState;
    if (rState == BUTTON_ACTIVE_STATE) {
      if (powerOn) {
        powerOffRequested = true;
        setLedRed();  // immediate feedback while we wind down
      } else {
        powerOn = true;
        applyStateLeds();  // shows green
      }
    }
  }
}

// -------------------------------
// POWER OFF
// -------------------------------
void performPowerOff() {
  powerOffRequested = false;
  powerOn = false;
  isPlaying = false;

  setLedRed();
  delay(POWER_OFF_HOLD_MS);
  setLedOff();

  i2s_driver_uninstall(I2S_NUM_0);
}

// -------------------------------
// PLAYBACK
// -------------------------------
void playWavFromSD(const String &path) {
  if (!powerOn || path.isEmpty()) return;

  File soundFile = SD.open(path);
  if (!soundFile) {
    Serial.println("Failed to open WAV file.");
    return;
  }

  WavInfo info;
  if (!readWavHeader(soundFile, info)) {
    Serial.println("Unsupported WAV.");
    soundFile.close();
    return;
  }

  soundFile.seek(info.dataStart);
  setupI2S(info.sampleRate);

  uint8_t buffer[I2S_CHUNK_BYTES];
  isPlaying = true;
  currentPlayingColor = pickRandomColor();
  applyStateLeds();

  Serial.print("Playing ");
  Serial.println(path);
  Serial.println();
  Serial.printf("WAV: %lu Hz, %u-bit, %u ch, data@%lu\n",
              (unsigned long)info.sampleRate,
              info.bitsPerSample,
              info.numChannels,
              (unsigned long)info.dataStart);

  while (soundFile.available() && powerOn && !powerOffRequested) {
    handleButtons();
    updateBlink();

    if (nextTrackRequested || powerOffRequested) break;

    size_t toRead = min((size_t)soundFile.available(), I2S_CHUNK_BYTES);
    size_t bytesRead = soundFile.read(buffer, toRead);
    if (bytesRead == 0) break;

    size_t bytesWritten;
    i2s_write(I2S_NUM_0, (const char *)buffer, bytesRead, &bytesWritten, portMAX_DELAY);
  }

  soundFile.close();
  isPlaying = false;

  if (powerOffRequested) {
    performPowerOff();
  } else if (!nextTrackRequested) {
    applyStateLeds();  // back to idle green
  }
}

// -------------------------------
// SETUP / LOOP
// -------------------------------
//Runs one time when esp32-c3 is started. Starts Serial, then initializes pin values as inputs and outputs accordingly.
//Also initializes the i2s communication accordingly for the speaker.
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Code 1");

  pinMode(GREEN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RED_BUTTON_PIN, INPUT_PULLUP);

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  setLedOff();

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("SD card mount failed!");
  } else {
    loadWavList();
  }
  Serial.println("code here");
  setupI2S(22050);
  Serial.println("Ready. Red toggles power; green cycles tracks.");
}

//Main loop functionality. Firsts checks to see if buttons are being pressed, then updates functionality based on if the buttons are pressed,
//else code just keeps playing the current song, and current led color.
void loop() {
  handleButtons();
  updateBlink();

  if (powerOffRequested && !isPlaying) {
    performPowerOff();
    return;
  }

  if (!powerOn) {
    delay(5);
    return;
  }

  if (wavCount == 0) {
    // Try to recover if SD was inserted after boot
    loadWavList();
  }

  if (!isPlaying && nextTrackRequested) {
    nextTrackRequested = false;  // consume this press
    String path = nextTrackPath();
    if (path.length() > 0) {
      playWavFromSD(path);
    } else {
      Serial.println("No WAV files to play.");
    }
  }

  delay(2);
}
