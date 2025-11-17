#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// --------------------------- Hardware map ---------------------------
constexpr int JOYSTICK_VRX_PIN = 34;   // moved to ADC1 (VRX)
constexpr int JOYSTICK_VRY_PIN = 35;   // moved to ADC1 (VRY)
constexpr int BUTTON_BEEP_PIN  = 22;   // active HIGH
constexpr int BUTTON_SWEEP_PIN = 23;   // active HIGH

// --------------------------- ESP-NOW peers --------------------------
uint8_t robotMacAddress[] = { 0x04, 0x83, 0x08, 0x58, 0xF2, 0x08 };

// --------------------------- Packet model ---------------------------
enum PacketType : uint8_t {
  PKT_CONTROL   = 0x01,
  PKT_MAP_CHUNK = 0x42
};

enum ActionMask : uint8_t {
  ACTION_BEEP        = 1 << 0,
  ACTION_REQUEST_MAP = 1 << 1
};

struct ControlPacket {
  uint8_t msgType;      // always PKT_CONTROL
  char    driveCommand; // 'F','B','L','R','S'
  uint8_t actionMask;   // see ActionMask bits
};

constexpr uint16_t MAP_BUFFER_MAX = 640;
constexpr uint8_t  MAP_CHUNK_MAX  = 180;

struct MapAssembler {
  char     buffer[MAP_BUFFER_MAX];
  uint16_t length = 0;
  uint8_t  nextChunk = 0;
  uint8_t  totalChunks = 0;
  bool     active = false;
  bool     complete = false;

  void reset() {
    length = 0;
    nextChunk = 0;
    totalChunks = 0;
    active = false;
    complete = false;
    buffer[0] = '\0';
  }

  void begin(uint8_t total) {
    reset();
    totalChunks = total;
    active = true;
  }

  void append(const char *payload, uint16_t payloadLen, uint8_t chunkIdx) {
    if (!active || chunkIdx != nextChunk) return;
    if (length + payloadLen >= MAP_BUFFER_MAX) {
      reset();
      return;
    }
    memcpy(buffer + length, payload, payloadLen);
    length += payloadLen;
    buffer[length] = '\0';
    nextChunk++;
    if (nextChunk >= totalChunks) {
      active = false;
      complete = true;
    }
  }
};

// --------------------------- Controller state -----------------------
enum ControllerMode { MODE_SENDING, MODE_RECEIVING };

ControllerMode controllerMode = MODE_SENDING;

struct ButtonState {
  bool current = false;
  bool previous = false;
  uint8_t pin = 0;
  bool activeHigh = false;

  void init(uint8_t gpio, bool activeHighInput) {
    pin = gpio;
    activeHigh = activeHighInput;
  }
  void read() {
    previous = current;
    int raw = digitalRead(pin);
    current = activeHigh ? (raw == HIGH) : (raw == LOW);
  }
  bool pressed() const { return current && !previous; }
};

ButtonState buttonBeep, buttonSweep;

char currentDriveCommand = 'S';
uint8_t pendingActions = 0;

constexpr unsigned long MIN_SEND_INTERVAL_MS = 60;
unsigned long lastSendMs = 0;

MapAssembler mapRx;
unsigned long receiveModeStartedMs = 0;
constexpr unsigned long MAP_WAIT_TIMEOUT_MS = 5000;

volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;

// pickDriveCommand(rawX, rawY)
// Converts joystick ADC readings into a discrete drive command
// Returns one of 'F','B','L','R','S' for forward/back/turn/stop.
char pickDriveCommand(int rawX, int rawY) {
  constexpr int ADC_CENTER_X = 3035;  // from workingCodeForController
  constexpr int ADC_CENTER_Y = 2960;
  constexpr int DEADZONE = 200;

  int dx = rawX - ADC_CENTER_X;
  int dy = rawY - ADC_CENTER_Y;

  if (dx > DEADZONE)  return 'L';  // joystick left
  if (dx < -DEADZONE) return 'R';  // joystick right
  if (dy < -DEADZONE) return 'F';  // joystick forward
  if (dy > DEADZONE)  return 'B';  // joystick backward
  return 'S';
}

// enqueueAction(mask)
// Sets the specified bits in the pending action mask so the next
// control packet can ask the robot to perform those actions.
void enqueueAction(uint8_t mask) {
  pendingActions |= mask;
}

// sendControlPacket(forceSend)
// Sends the latest ControlPacket over ESP-NOW; forceSend bypasses the
// interval throttle. Returns true when the packet left successfully.
bool sendControlPacket(bool forceSend = false) {
  unsigned long now = millis();
  if (!forceSend && (now - lastSendMs) < MIN_SEND_INTERVAL_MS && pendingActions == 0 &&
      currentDriveCommand == 'S') {
    return true;  // nothing new to report
  }

  ControlPacket packet{
    .msgType = PKT_CONTROL,
    .driveCommand = currentDriveCommand,
    .actionMask = pendingActions
  };

  uint8_t actionsToSend = pendingActions;
  esp_err_t result = esp_now_send(robotMacAddress, reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  if (result == ESP_OK) {
    pendingActions = 0;
    lastSendMs = now;
    return true;
  }

  // send failed: restore the action bits so we can retry
  pendingActions |= actionsToSend;
  Serial.printf("ESP-NOW send failed (%d)\n", result);
  return false;
}

// enterReceiveMode()
// Switches the controller into map-receive mode and arms the assembler.
void enterReceiveMode() {
  controllerMode = MODE_RECEIVING;
  mapRx.reset();
  receiveModeStartedMs = millis();
  Serial.println("Controller: awaiting sweep map...");
}

// exitReceiveMode()
// Leaves map-receive mode, clearing any partial data and re-enabling control input.
void exitReceiveMode() {
  controllerMode = MODE_SENDING;
  mapRx.reset();
  Serial.println("Controller: back to joystick control.");
}

// handleMapChunk(data, len)
// Feeds an incoming PKT_MAP_CHUNK frame into the MapAssembler,
// ignoring malformed or out-of-order chunks.
void handleMapChunk(const uint8_t *data, int len) {
  if (len < 5) return;

  uint8_t chunkIdx   = data[1];
  uint8_t totalChunk = data[2];
  uint16_t payloadLen = data[3] | (static_cast<uint16_t>(data[4]) << 8);
  payloadLen = min<uint16_t>(payloadLen, len - 5);
  payloadLen = min<uint16_t>(payloadLen, MAP_CHUNK_MAX);

  if (chunkIdx == 0 || !mapRx.active) {
    mapRx.begin(totalChunk);
  }

  mapRx.append(reinterpret_cast<const char *>(data + 5), payloadLen, chunkIdx);
}

// onEspNowRecv(info, data, len)
// ESP-NOW receive callback; info->src_addr identifies the sender.
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len <= 0) {
    return;
  }

  uint8_t msgType = data[0];
  switch (msgType) {
    case PKT_MAP_CHUNK:
      handleMapChunk(data, len);
      receiveModeStartedMs = millis();  // keep timeout fresh while data arrives
      if (controllerMode != MODE_RECEIVING) {
        controllerMode = MODE_RECEIVING;
      }
      break;
    default:
      break;
  }
}

// onEspNowSent(info, status)
// ESP-NOW send callback; records the last link status for UI/debug.
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  lastSendStatus = status;
  if (!info) return;

  if (status != ESP_NOW_SEND_SUCCESS) {
    const uint8_t *mac = info->des_addr;   // destination MAC
    Serial.printf("Send to %02X:%02X:%02X:%02X:%02X:%02X failed (%d)\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], status);
  }
}
// setupPins()
// Configures joystick and button GPIO directions/pull-ups and
// arms the ButtonState trackers
void setupPins() {
  pinMode(JOYSTICK_VRX_PIN, INPUT);
  pinMode(JOYSTICK_VRY_PIN, INPUT);
  pinMode(BUTTON_BEEP_PIN, INPUT);
  pinMode(BUTTON_SWEEP_PIN, INPUT);

  buttonBeep.init(BUTTON_BEEP_PIN, true);
  buttonSweep.init(BUTTON_SWEEP_PIN, true);
}

// setup()
// Initializes serial, GPIO, ESP-NOW peer linkage, and controller state.
void setup() {
  Serial.begin(115200);
  setupPins();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, rebooting...");
    delay(2000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, robotMacAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add robot peer. Halting.");
    while (true) delay(1000);
  }

  Serial.println("A4AutoController ready (mode: sending).");
}

// serviceSendingMode()
// Reads joystick/buttons, updates drive command, queues actions,
// and transmits control packets while in MODE_SENDING.
void serviceSendingMode() {
  int rawX = analogRead(JOYSTICK_VRX_PIN);
  int rawY = analogRead(JOYSTICK_VRY_PIN);
  currentDriveCommand = pickDriveCommand(rawX, rawY);

  Serial.printf("TX cmd:%c rawX:%d rawY:%d\n", currentDriveCommand, rawX, rawY);

  buttonBeep.read();
  buttonSweep.read();

  if (buttonBeep.pressed()) {
    enqueueAction(ACTION_BEEP);
  }

  if (buttonSweep.pressed()) {
    enqueueAction(ACTION_REQUEST_MAP);
    currentDriveCommand = 'S';  // ensure the robot stops before sweeping
    if (sendControlPacket(true)) {
      enterReceiveMode();
      return;
    } else {
      Serial.println("Sweep request failed to send; will retry.");
    }
  }

  sendControlPacket();
}

// serviceReceiveMode()
// Monitors map assembly while waiting for a sweep result and exits
// back to MODE_SENDING when complete or on timeout.
void serviceReceiveMode() {
  if (mapRx.complete) {
    Serial.println("---- Sweep Map ----");
    Serial.println(mapRx.buffer);
    Serial.println("-------------------");
    exitReceiveMode();
    return;
  }

  if ((millis() - receiveModeStartedMs) > MAP_WAIT_TIMEOUT_MS) {
    Serial.println("Map reception timeout.");
    exitReceiveMode();
  }
}

// loop()
// Main scheduler that routes between sending/receiving modes.
void loop() {
  if (controllerMode == MODE_SENDING) {
    serviceSendingMode();
  } else {
    serviceReceiveMode();
  }

  delay(10);
}
