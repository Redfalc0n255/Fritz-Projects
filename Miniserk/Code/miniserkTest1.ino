#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>

// -------------------------------
// PIN DEFINITIONS
#define LED_PIN        3       // WS2812B data pin
#define NUM_LEDS       16      // WS812B Ring LED number

#define I2S_BCLK_PIN   1       // MAX98357A BCLK
#define I2S_LRC_PIN    2       // MAX98357A LRC
#define I2S_DOUT_PIN   0       // MAX98357A DIN

// LED SETUP
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// I2S CONFIG
void setupI2S() {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 22050,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCLK_PIN,
      .ws_io_num = I2S_LRC_PIN,
      .data_out_num = I2S_DOUT_PIN,
      .data_in_num = -1  // not used
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// SIMPLE TEST SOUND (a beep)
void playBeep(int freq = 880, int durationMs = 300) {
  const int sampleRate = 22050;
  const float amplitude = 12000.0f;   // 16-bit max is 32767

  int totalSamples = (sampleRate * durationMs) / 1000;

  for (int i = 0; i < totalSamples; i++) {
    float theta = 2.0f * PI * freq * (float)i / (float)sampleRate;
    int16_t sample = (int16_t)(amplitude * sinf(theta));

    size_t bytesWritten;
    i2s_write(I2S_NUM_0, (const char *)&sample, sizeof(sample),
              &bytesWritten, portMAX_DELAY);
  }
}

// -------------------------------
// SETUP
void setup() {
  Serial.begin(115200);
  delay(500);

  // LED ring
  strip.begin();
  strip.setBrightness(40);  
  strip.fill(strip.Color(0, 0, 0));
  strip.show();

  // I2S audio
  setupI2S();

  Serial.println("Startup: playing test beep...");
  // One startup beep so you know audio path works
  playBeep(880, 300);
  delay(300);
  playBeep(660, 200);
}

// MAIN LOOP
void loop() {
  static uint16_t hue = 0;
  static unsigned long lastBeep = 0;

  // LED rainbow animation
  strip.fill(strip.ColorHSV(hue));
  strip.show();
  hue += 256;       // smaller = slower hue change
  delay(20);

  // Periodic beep every 5 seconds
  unsigned long now = millis();
  if (now - lastBeep > 5000) {
    Serial.println("Periodic beep...");
    playBeep(880, 200);
    lastBeep = now;
  }
}