#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <mbedtls/base64.h>
#include <mbedtls/md.h>
#include <time.h>
#include "driver/i2s.h"

// ============================================================
// CONSTANT VARIABLES
// ============================================================

// ---------------- Device profile fallback ----------------
// Runtime identity is resolved from the ESP32 WiFi STA MAC.
// This constant is only used if the SD/cloud config cannot be loaded.
constexpr int DEVICE_PROFILE = 0;

// ---------------- Pin mapping ----------------
constexpr int PIN_SD_CS      = 27;
constexpr int PIN_I2S_BCLK   = 26;
constexpr int PIN_I2S_LRCLK  = 25;
constexpr int PIN_I2S_DOUT   = 13;
constexpr int PIN_I2S_DIN    = 33;

// Buttons
constexpr int PIN_BTN_B1_GREEN = 17;
constexpr int PIN_BTN_B2_BLACK = 16;
constexpr int PIN_BTN_B3_WHITE = 32;   // physically the old "yellow" button pin
constexpr int PIN_BTN_B4_RED   = 14;

// LEDs
constexpr int PIN_LED_RED    = 4;
constexpr int PIN_LED_GREEN  = 5;

// ---------------- Audio settings ----------------
constexpr i2s_port_t I2S_PORT        = I2S_NUM_0;
constexpr uint32_t SAMPLE_RATE       = 16000;
constexpr uint8_t  BITS_PER_SAMPLE   = 16;
constexpr uint8_t  CHANNELS          = 1;
constexpr uint32_t MAX_RECORD_MS     = 100000UL;   // 100 seconds
constexpr size_t   RECORD_CHUNK_SAMPLES = 512;

// ---------------- DSP settings ----------------
constexpr float HPF_ALPHA = 0.88f;
constexpr float LPF_ALPHA = 0.70f;
constexpr int16_t GATE_THRESHOLD = 140;
constexpr float RECORD_INPUT_GAIN = 1.05f;
constexpr uint8_t RECORD_INPUT_SHIFT = 11;
constexpr float VOLUME_LEVELS[5] = {0.20f, 0.40f, 0.60f, 0.80f, 1.00f};

// ---------------- Timing ----------------
constexpr uint32_t BUTTON_DEBOUNCE_MS      = 40;
constexpr uint32_t BUTTON_LONG_PRESS_MS    = 500;
constexpr uint32_t BUTTON_DOUBLE_CLICK_MS  = 400;
constexpr uint32_t SENDING_IDLE_TIMEOUT_MS = 60000;
constexpr uint32_t USER_ACTIVE_NETWORK_GRACE_MS = 15000;
constexpr uint32_t BACKGROUND_SERVICE_INTERVAL_MS = 50;
constexpr uint32_t LED_BLINK_INTERVAL_MS   = 450;
constexpr uint32_t DOWNLOAD_LED_INTERVAL_MS = 180;
constexpr uint32_t BUSY_LED_INTERVAL_MS    = 180;
constexpr uint32_t SEND_TO_PREFIX_RESET_MS = 5000;

// ---------------- File structure ----------------
constexpr const char* ROOT_DIR             = "/fritzphone";
constexpr const char* SYSTEM_DIR           = "/fritzphone/system";
constexpr const char* SYSTEM_AUDIO_DIR     = "/fritzphone/system/audio";
constexpr const char* SYSTEM_PROMPTS_DIR   = "/fritzphone/system/audio/prompts";
constexpr const char* SYSTEM_NAMES_DIR     = "/fritzphone/system/audio/names";
constexpr const char* SYSTEM_CONTACTS_DIR  = "/fritzphone/system/contacts";
constexpr const char* MESSAGE_QUEUE_DIR    = "/fritzphone/message_queue";
constexpr const char* SAVED_MESSAGES_DIR   = "/fritzphone/saved_messages";
constexpr const char* DEVICE_DATA_DIR      = "/fritzphone/device_data";
constexpr const char* TEMP_DIR             = "/fritzphone/temp";
constexpr const char* MOCK_SENT_DIR        = "/fritzphone/mock_sent";

constexpr const char* TEMP_OUTGOING_WAV    = "/fritzphone/temp/current_outgoing.wav";
constexpr const char* TEMP_OUTGOING_META   = "/fritzphone/temp/current_outgoing.txt";
constexpr const char* TEMP_MIC_SELF_TEST_WAV = "/fritzphone/temp/mic_self_test.wav";
constexpr const char* DEVICE_INFO_PATH     = "/fritzphone/device_data/device_info.txt";
constexpr const char* SETTINGS_PATH        = "/fritzphone/device_data/settings.txt";
constexpr const char* WIFI_CONFIG_PATH     = "/fritzphone/system/wifi.json";
constexpr const char* IOT_CONFIG_PATH      = "/fritzphone/system/iot.json";
constexpr const char* AZURE_SERVICES_PATH  = "/fritzphone/system/azure_services.json";
constexpr const char* PROMPT_INITIALIZING_FRITZ_PHONE = "/fritzphone/system/audio/prompts/initializing_fritz_phone.wav";
constexpr const char* PROMPT_SAPPHIRE_EDITION         = "/fritzphone/system/audio/prompts/sapphire_edition.wav";
constexpr const char* PROMPT_EMERALD_EDITION          = "/fritzphone/system/audio/prompts/emerald_edition.wav";
constexpr const char* PROMPT_SCARLET_EDITION          = "/fritzphone/system/audio/prompts/scarlet_edition.wav";
constexpr const char* PROMPT_SENDING_TO               = "/fritzphone/system/audio/prompts/sending_to.wav";
constexpr const char* PROMPT_MESSAGE_FROM             = "/fritzphone/system/audio/prompts/message_from.wav";
constexpr const char* PROMPT_NEW_MESSAGE_FROM         = "/fritzphone/system/audio/prompts/new_message_from.wav";
constexpr const char* PROMPT_MESSAGE_SENT_TO          = "/fritzphone/system/audio/prompts/message_sent_to.wav";
constexpr const char* PROMPT_NO_NEW_MESSAGES          = "/fritzphone/system/audio/prompts/no_new_messages.wav";
constexpr const char* PROMPT_SYSTEM_ERROR             = "/fritzphone/system/audio/prompts/system_error.wav";
constexpr const char* NAME_BEN                        = "/fritzphone/system/audio/names/ben.wav";
constexpr const char* NAME_BONNIE                     = "/fritzphone/system/audio/names/bonnie.wav";
constexpr const char* NAME_PETER                      = "/fritzphone/system/audio/names/pete.wav";
constexpr const char* NAME_SOPHIE                     = "/fritzphone/system/audio/names/sophie.wav";
constexpr const char* NAME_EVERYONE                   = "/fritzphone/system/audio/prompts/everyone.wav";

constexpr size_t MAX_SAVED_MESSAGES = 20;
constexpr size_t MAX_CONFIGURED_NETWORKS = 4;
constexpr size_t MAX_CLOUD_DEVICES = 4;

// ---------------- Cloud timing ----------------
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS     = 20000;
constexpr uint32_t WIFI_RETRY_DELAY_MS         = 10000;
constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS  = 30000;
constexpr uint32_t QUEUE_SANITIZE_INTERVAL_MS  = 5000;
constexpr uint32_t QUEUE_STATE_REFRESH_MS      = 1000;
constexpr uint32_t PENDING_POLL_INTERVAL_MS    = 15000;
constexpr uint32_t HEARTBEAT_INTERVAL_MS       = 60000;
constexpr uint32_t IOT_SAS_LIFETIME_SECONDS    = 3600;
constexpr uint32_t FUNCTION_AUTH_WINDOW_SEC    = 300;
constexpr uint32_t DOWNLOAD_HTTP_TIMEOUT_MS    = 45000;
constexpr uint32_t UPLOAD_HTTP_TIMEOUT_MS      = 90000;
constexpr uint32_t NTP_SYNC_TIMEOUT_MS         = 15000;
constexpr uint32_t TLS_HANDSHAKE_TIMEOUT_SEC   = 5;
constexpr int HTTPS_PORT                       = 443;

static const char* NTP_SERVERS[] = {
  "pool.ntp.org",
  "time.nist.gov"
};

// ============================================================
// CHANGING VARIABLES
// ============================================================

bool sdReady = false;
bool wifiConfigLoaded = false;
bool iotConfigLoaded = false;
bool azureServicesLoaded = false;
bool wifiConnected = false;
bool cloudIdentityResolved = false;
bool timeSynced = false;
bool mqttConnected = false;
bool i2sInstalled = false;
bool queueBlinkState = false;
bool busyBlinkState = false;
bool userActive = true;
bool online = false;
unsigned long lastBlinkMs = 0;
unsigned long lastBusyBlinkMs = 0;
unsigned long lastUserActivityMs = 0;
unsigned long lastBackgroundServiceMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastPollMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastTwinReportMs = 0;
unsigned long lastMqttConnectAttemptMs = 0;
unsigned long lastPresencePingMs = 0;
unsigned long lastQueueSanitizeMs = 0;
unsigned long lastQueueStateRefreshMs = 0;
unsigned long lastMicDebugMs = 0;
size_t cachedQueueDepth = 0;
uint32_t nextMessageSequence = 1;
int currentRecipientIndex = 0;
int currentVolumeIndex = 2; // 60% default
int activeDeviceProfile = DEVICE_PROFILE;
bool sendToPrefixArmed = true;
unsigned long lastRecipientPromptMs = 0;
bool b4PendingSingleClick = false;
unsigned long b4PendingClickMs = 0;
uint8_t activeRecipientMask = 0;
bool cloudDownloadActive = false;
bool cloudUploadActive = false;

String lastPlayedWavPath = "";
String lastPlayedMetaPath = "";
String currentTempRecipientId = "";
String currentTempRecipientName = "";
String currentTempRecipientNameWav = "";

// Recording state
bool recordingActive = false;
File recordingFile;
uint32_t recordingDataBytes = 0;
unsigned long recordingStartedMs = 0;
float recordHpfState = 0.0f;
float recordLpfState = 0.0f;
int16_t recordPrevIn = 0;

struct WifiNetworkConfig {
  String name;
  String ssid;
  String password;
  int priority;
};

struct CloudDeviceConfig {
  String ownerName;
  String edition;
  String logicalDeviceId;
  String iotDeviceId;
  String iotDeviceKey;
  String wifiStaMac;
  String wifiApMac;
};

struct PendingCloudMessage {
  String deliveryId;
  String messageId;
  String senderId;
  String senderName;
  String receiverId;
  String receiverName;
  String payloadPath;
  String downloadUrl;
  uint32_t messageLength;
  uint32_t durationMs;
  String timeSentUtc;
};

WifiNetworkConfig wifiNetworks[MAX_CONFIGURED_NETWORKS];
size_t wifiNetworkCount = 0;
CloudDeviceConfig cloudDevices[MAX_CLOUD_DEVICES];
size_t cloudDeviceCount = 0;
int activeCloudDeviceIndex = -1;

String functionBaseUrl = "https://fritzfonefa-h2b9d7cfa0d3frdn.westus2-01.azurewebsites.net/api";
String iotHubHostName = "";
String currentStaMac = "";
String currentApMac = "";
String currentIotDeviceId = "";
String currentIotDeviceKey = "";
String currentSasToken = "";
time_t currentSasExpiry = 0;
String cloudStatus = "booting";

WiFiClientSecure mqttTlsClient;
PubSubClient mqttClient(mqttTlsClient);

// ============================================================
// CLASSES / ENUMS
// ============================================================

enum class DeviceMode {
  Receiving,
  Sending
};

enum class ActionState {
  Idle,
  Recording,
  Playing,
  Sending
};

enum class ButtonEvent {
  None,
  Press,
  Release,
  Click,
  DoubleClick,
  LongPress
};

enum class ContactId {
  Ben,
  Pete,
  Bonnie,
  Sophie,
  All
};

struct Contact {
  ContactId id;
  const char* displayName;
  const char* deviceId;
  const char* spokenNamePath;
};

struct Button {
  int pin;
  bool stablePressed;
  bool lastRawPressed;
  bool longPressFired;
  bool waitingForDoubleClick;
  unsigned long lastDebounceMs;
  unsigned long pressedAtMs;
  unsigned long releasedAtMs;
  unsigned long lastClickMs;
};

// ============================================================
// FORWARD DECLARATIONS
// ============================================================

// ---------------- Device / contacts ----------------
const char* getThisDeviceId();
const char* getThisDeviceName();
const char* getThisEditionName();
size_t getRecipientCount();
const Contact& getRecipientAt(size_t index);
const Contact& getCurrentRecipient();
void cycleRecipient();
const Contact& getButtonRecipient(uint8_t buttonIndex);
uint8_t getHeldSendButtonMask();
bool buildRecipientSelectionStrings(uint8_t recipientMask, String& outIds, String& outNames);
void playContactName(const Contact& contact);
bool beginDirectMessageRecording();
bool finalizeDirectMessageRecording();
void servicePendingB4Click();

// ---------------- Utility / filesystem ----------------
void printBootHeader();
bool initSD();
bool ensureDir(const char* path);
bool ensureFilesystemLayout();
bool writeDeviceInfoIfMissing();
bool writeSettingsIfMissing();
bool fileExists(const char* path);
bool deleteFileIfExists(const char* path);
bool moveFile(const char* src, const char* dst);
uint32_t extractSequenceFromName(const String& name);
String buildMessageBaseName(uint32_t seq);
String buildQueuedWavPath(uint32_t seq);
String buildQueuedMetaPath(uint32_t seq);
String buildSavedWavPath(uint32_t seq);
String buildSavedMetaPath(uint32_t seq);
String buildMockSentWavPath(uint32_t seq);
String buildMockSentMetaPath(uint32_t seq);
uint32_t findHighestSequenceInDir(const char* dirPath);
void refreshNextMessageSequence();
size_t countWavsInDir(const char* dirPath);
bool findOldestWavInDir(const char* dirPath, String& outWavPath, String& outBaseName);
String replaceExtension(const String& path, const char* newExt);
String sanitizeMacAddress(const String& value);
int profileFromLogicalDeviceId(const String& logicalDeviceId);
String urlEncode(const String& value);
String bytesToBase64(const uint8_t* data, size_t len);
bool decodeBase64(const String& input, uint8_t* output, size_t outputSize, size_t& decodedLen);
String hmacSha256Base64(const String& base64Key, const String& message);
String sha256HexOfFile(const char* path);
time_t getUnixTimeNow();

// ---------------- Metadata ----------------
bool writeSimpleMetadata(const char* path,
                         const String& messageId,
                         const String& fromId,
                         const String& fromName,
                         const String& toId,
                         const String& toName,
                         uint32_t durationMs,
                         const String& status);
String readMetadataValue(const char* path, const char* key);
bool writeCloudMetadata(const char* path, const PendingCloudMessage& message, const String& status);

// ---------------- I2S / WAV / audio ----------------
void stopI2S();
bool startI2SMic();
bool startI2SSpeaker();
void writeWavHeader(File& f, uint32_t dataBytes);
bool beginRecordingToTemp();
bool continueRecordingToTemp();
bool finishRecordingToTemp();
bool cancelRecordingToTemp();
bool playWav(const char* path);
bool playWav(const String& path);
bool playWavScaled(const char* path, float volumeScale);
bool playWavScaled(const String& path, float volumeScale);
int16_t scaleSample(int16_t sample, float gain);
bool playCurrentTempRecording();
void playTone(uint16_t freq, uint16_t ms);
void playToneWithGain(uint16_t freq, uint16_t ms, float gain);
void playFeedbackToneOK();
void playFeedbackToneError();
void playFeedbackToneModeReceive();
void playFeedbackToneModeSend();
void playFeedbackToneVolume();
void playRecordStartBeep();
void playRecipientRoutingBeepSequence(uint8_t recipientMask);
void playMessageSentRecipientSequence(uint8_t recipientMask);
uint16_t getRecipientNotificationTone(const Contact& contact);
bool playOptionalPrompt(const char* path, float volumeScale = -1.0f);
bool playEditionPrompt();
void playStartupAnnouncement();
void playSendingToPrompt();
void playRecipientPrompt();
void playMessageSentToPrompt();
void playMessageFromPrompt(const char* senderNamePath);
void playNoNewMessagesPrompt();

// ---------------- Queue / archive / mock sending ----------------
bool hasQueuedMessages();
size_t refreshQueueDepth(bool force = false);
bool isValidWavFile(const String& wavPath);
bool isValidQueuedMessage(const String& wavPath, const String& metaPath);
void logFileHeaderBytes(const String& path, const char* label);
size_t pruneInvalidQueuedMessages(bool verbose = false);
bool playNextQueuedMessage();
bool replayLastPlayedMessage();
bool archiveMessage(const String& wavPath, const String& metaPath);
bool enforceSavedMessageLimit();
bool mockSendCurrentTempMessage();
bool cloudSendCurrentTempMessage();
bool enqueueDownloadedCloudMessage(const PendingCloudMessage& message, Stream& source);
bool acknowledgeDelivery(const String& deliveryId, const String& status);
bool recordFixedDurationToWav(uint16_t seconds, const char* path);

// ---------------- LEDs / modes / state ----------------
void setupButtonsAndLeds();
void setLeds(bool greenOn, bool redOn);
void flashTransferLeds(uint8_t pulses, uint16_t onMs, uint16_t offMs);
void updateLedState();
void setMode(DeviceMode mode);
void toggleMode();
void resetIdleTimer();
void updateInteractionState();
void updateIdleTimeout();
float getCurrentVolumeMultiplier();
void cycleVolumeLevel();
bool hasTempDraft();
bool discardTempDraft(bool playTone = false);
bool refreshTempDraftRecipientMetadata();

// ---------------- Buttons ----------------
void initButton(Button& btn, int pin);
ButtonEvent updateButton(Button& btn);
void pollButtons();
void handleButtonEventB1(ButtonEvent event);
void handleButtonEventB2(ButtonEvent event);
void handleButtonEventB3(ButtonEvent event);
void handleButtonEventB4(ButtonEvent event);

// ---------------- Setup / loop helpers ----------------
bool loadWifiConfig();
bool loadIotConfig();
bool loadAzureServicesConfig();
bool resolveDeviceIdentity();
bool connectToWiFiIfNeeded();
bool syncClockIfNeeded();
bool ensureMqttConnection();
void mqttCallback(char* topic, uint8_t* payload, unsigned int length);
bool publishPresence(const char* lifecycleState);
bool publishHeartbeat();
bool publishTwinReportedProperties();
bool pollCloudInboxIfDue();
bool downloadPendingMessages();
bool sendTempMessageToCloud();
bool sendDeviceHeartbeat(bool force = false);
bool shouldRunBackgroundNetworkTasks();
String buildFunctionAuthSignature(const String& method, const String& route, const String& timestampUtc);
void configureSecureClient(WiFiClientSecure& client);
void addFunctionAuthHeaders(HTTPClient& http, const String& method, const String& route);
void handleSerialDebugCommands();
void serviceBackgroundTasks();
void printFinalTodo();

// ============================================================
// GLOBAL STATE OBJECTS
// ============================================================

DeviceMode currentMode = DeviceMode::Receiving;
ActionState currentActionState = ActionState::Idle;

Button buttonB1;
Button buttonB2;
Button buttonB3;
Button buttonB4;

// ============================================================
// CONTACT TABLES
// ============================================================

static const Contact BEN_RECIPIENTS[] = {
  {ContactId::Bonnie, "Bonnie", "bonnie_emerald", NAME_BONNIE}, // B1 / green button slot
  {ContactId::Pete,   "Pete",   "pete_sapphire",  NAME_PETER},  // B2 / blue button slot
  {ContactId::Sophie, "Sophie", "sophie_scarlet", NAME_SOPHIE}  // B3 / red button slot
};

static const Contact PETE_RECIPIENTS[] = {
  {ContactId::Bonnie, "Bonnie", "bonnie_emerald", NAME_BONNIE}, // B1 / green button slot
  {ContactId::Ben,    "Ben",    "ben_slate",      NAME_BEN},    // B2 / grey button slot
  {ContactId::Sophie, "Sophie", "sophie_scarlet", NAME_SOPHIE}  // B3 / red button slot
};

static const Contact BONNIE_RECIPIENTS[] = {
  {ContactId::Pete,   "Pete",   "pete_sapphire",  NAME_PETER},  // B1 / blue button slot
  {ContactId::Ben,    "Ben",    "ben_slate",      NAME_BEN},    // B2 / grey button slot
  {ContactId::Sophie, "Sophie", "sophie_scarlet", NAME_SOPHIE}  // B3 / red button slot
};

static const Contact SOPHIE_RECIPIENTS[] = {
  {ContactId::Bonnie, "Bonnie", "bonnie_emerald", NAME_BONNIE}, // B1 / green button slot
  {ContactId::Pete,   "Pete",   "pete_sapphire",  NAME_PETER},  // B2 / blue button slot
  {ContactId::Ben,    "Ben",    "ben_slate",      NAME_BEN}     // B3 / grey button slot
};

// ============================================================
// DEVICE / CONTACT METHODS
// ============================================================

/**
 * Returns this device's permanent internal ID.
 */
const char* getThisDeviceId() {
  switch (activeDeviceProfile) {
    case 0: return "ben_slate";
    case 1: return "pete_sapphire";
    case 2: return "bonnie_emerald";
    case 3: return "sophie_scarlet";
    default: return "unknown_device";
  }
}

/**
 * Returns this device owner's display name.
 */
const char* getThisDeviceName() {
  switch (activeDeviceProfile) {
    case 0: return "Ben";
    case 1: return "Pete";
    case 2: return "Bonnie";
    case 3: return "Sophie";
    default: return "Unknown";
  }
}

/**
 * Returns this device edition name.
 */
const char* getThisEditionName() {
  switch (activeDeviceProfile) {
    case 0: return "Slate";
    case 1: return "Sapphire";
    case 2: return "Emerald";
    case 3: return "Scarlet";
    default: return "Unknown";
  }
}

/**
 * Returns the number of valid recipients for this device.
 */
size_t getRecipientCount() {
  return 3;
}

/**
 * Returns the recipient entry at a given index for this device.
 */
const Contact& getRecipientAt(size_t index) {
  switch (activeDeviceProfile) {
    case 0: return BEN_RECIPIENTS[index];
    case 1: return PETE_RECIPIENTS[index];
    case 2: return BONNIE_RECIPIENTS[index];
    case 3: return SOPHIE_RECIPIENTS[index];
    default: return BEN_RECIPIENTS[index];
  }
}

/**
 * Returns the currently selected recipient.
 */
const Contact& getCurrentRecipient() {
  return getRecipientAt(currentRecipientIndex);
}

/**
 * Returns the fixed contact mapped to one of the three direct-send buttons.
 */
const Contact& getButtonRecipient(uint8_t buttonIndex) {
  size_t recipientCount = getRecipientCount();
  if (buttonIndex >= recipientCount) {
    buttonIndex = 0;
  }
  return getRecipientAt(buttonIndex);
}

/**
 * Returns a bitmask of the three direct-send buttons currently held.
 */
uint8_t getHeldSendButtonMask() {
  uint8_t mask = 0;
  if (buttonB1.stablePressed) mask |= 0x01;
  if (buttonB2.stablePressed) mask |= 0x02;
  if (buttonB3.stablePressed) mask |= 0x04;
  return mask;
}

/**
 * Converts a recipient button mask into comma-separated logical IDs and display names.
 */
bool buildRecipientSelectionStrings(uint8_t recipientMask, String& outIds, String& outNames) {
  outIds = "";
  outNames = "";

  for (uint8_t i = 0; i < 3; ++i) {
    uint8_t bit = (1u << i);
    if ((recipientMask & bit) == 0) continue;

    const Contact& contact = getButtonRecipient(i);
    if (outIds.length() > 0) {
      outIds += ",";
      outNames += ",";
    }

    outIds += contact.deviceId;
    outNames += contact.displayName;
  }

  return outIds.length() > 0;
}

/**
 * Plays a contact name prompt directly, without the older "sending to" prefix flow.
 */
void playContactName(const Contact& contact) {
  Serial.print("Selected recipient button -> ");
  Serial.println(contact.displayName);

  if (SD.exists(contact.spokenNamePath)) {
    playWavScaled(contact.spokenNamePath, 0.85f);
  } else {
    playFeedbackToneOK();
  }
}

// ============================================================
// UTILITY / FILESYSTEM METHODS
// ============================================================

/**
 * Prints boot information to Serial.
 */
void printBootHeader() {
  Serial.println();
  Serial.println("Fritz Phone Azure build");
  Serial.println("-----------------------");
  Serial.print("Device: ");
  Serial.print(getThisDeviceName());
  Serial.print(" / ");
  Serial.println(getThisEditionName());
  Serial.print("STA MAC: ");
  Serial.println(currentStaMac);
  if (currentIotDeviceId.length() > 0) {
    Serial.print("IoT Device ID: ");
    Serial.println(currentIotDeviceId);
  }
  Serial.print("Function API: ");
  Serial.println(functionBaseUrl);
  Serial.println("Button map: B1/B2/B3 direct-send, B4 queue play/replay");
}

/**
 * Initializes the SD card and reports status.
 */
bool initSD() {
  Serial.print("Mounting SD ... ");
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("failed");
    sdReady = false;
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("no card");
    sdReady = false;
    return false;
  }

  Serial.print("ok, size=");
  Serial.print((uint32_t)SD.cardSize() / (1024UL * 1024UL));
  Serial.println(" MB");

  sdReady = true;
  return true;
}

/**
 * Creates a directory if it does not already exist.
 */
bool ensureDir(const char* path) {
  if (SD.exists(path)) return true;
  return SD.mkdir(path);
}

/**
 * Creates the complete Fritz Phone folder structure on the SD card.
 */
bool ensureFilesystemLayout() {
  if (!sdReady) return false;

  bool ok = true;
  ok &= ensureDir(ROOT_DIR);
  ok &= ensureDir(SYSTEM_DIR);
  ok &= ensureDir(SYSTEM_AUDIO_DIR);
  ok &= ensureDir(SYSTEM_PROMPTS_DIR);
  ok &= ensureDir(SYSTEM_NAMES_DIR);
  ok &= ensureDir(SYSTEM_CONTACTS_DIR);
  ok &= ensureDir(MESSAGE_QUEUE_DIR);
  ok &= ensureDir(SAVED_MESSAGES_DIR);
  ok &= ensureDir(DEVICE_DATA_DIR);
  ok &= ensureDir(TEMP_DIR);
  ok &= ensureDir(MOCK_SENT_DIR);

  if (!ok) {
    Serial.println("Failed to create one or more directories.");
    return false;
  }

  writeDeviceInfoIfMissing();
  writeSettingsIfMissing();
  return true;
}

/**
 * Writes device identity information to SD if not already present.
 */
bool writeDeviceInfoIfMissing() {
  if (SD.exists(DEVICE_INFO_PATH)) {
    SD.remove(DEVICE_INFO_PATH);
  }

  File f = SD.open(DEVICE_INFO_PATH, FILE_WRITE);
  if (!f) return false;

  f.print("DEVICE_ID=");
  f.println(getThisDeviceId());
  f.print("OWNER_NAME=");
  f.println(getThisDeviceName());
  f.print("EDITION=");
  f.println(getThisEditionName());
  f.close();
  return true;
}

/**
 * Writes default settings to SD if not already present.
 */
bool writeSettingsIfMissing() {
  if (SD.exists(SETTINGS_PATH)) return true;

  File f = SD.open(SETTINGS_PATH, FILE_WRITE);
  if (!f) return false;

  f.print("VOLUME_INDEX=");
  f.println(currentVolumeIndex);
  f.print("MAX_SAVED_MESSAGES=");
  f.println((int)MAX_SAVED_MESSAGES);
  f.close();
  return true;
}

/**
 * Returns true if a file exists on SD.
 */
bool fileExists(const char* path) {
  return sdReady && SD.exists(path);
}

/**
 * Deletes a file if it exists.
 */
bool deleteFileIfExists(const char* path) {
  if (!SD.exists(path)) return true;
  return SD.remove(path);
}

/**
 * Moves a file by copying its bytes then deleting the source.
 */
bool moveFile(const char* src, const char* dst) {
  if (!SD.exists(src)) return false;
  if (SD.exists(dst)) {
    SD.remove(dst);
  }

  File in = SD.open(src, FILE_READ);
  if (!in) return false;

  File out = SD.open(dst, FILE_WRITE);
  if (!out) {
    in.close();
    return false;
  }

  uint8_t buffer[1024];
  while (true) {
    int n = in.read(buffer, sizeof(buffer));
    if (n <= 0) break;
    if (out.write(buffer, n) != (size_t)n) {
      in.close();
      out.close();
      return false;
    }
  }

  in.close();
  out.close();

  if (!SD.remove(src)) return false;
  return true;
}

/**
 * Extracts a numeric sequence from filenames like msg_000123.wav.
 */
uint32_t extractSequenceFromName(const String& name) {
  int us = name.indexOf('_');
  int dot = name.lastIndexOf('.');
  if (us < 0 || dot < 0 || dot <= us) return 0;
  String num = name.substring(us + 1, dot);
  return (uint32_t)num.toInt();
}

/**
 * Builds a base message name without extension.
 */
String buildMessageBaseName(uint32_t seq) {
  char buf[32];
  snprintf(buf, sizeof(buf), "msg_%06lu", (unsigned long)seq);
  return String(buf);
}

/**
 * Builds queued WAV path.
 */
String buildQueuedWavPath(uint32_t seq) {
  return String(MESSAGE_QUEUE_DIR) + "/" + buildMessageBaseName(seq) + ".wav";
}

/**
 * Builds queued metadata path.
 */
String buildQueuedMetaPath(uint32_t seq) {
  return String(MESSAGE_QUEUE_DIR) + "/" + buildMessageBaseName(seq) + ".txt";
}

/**
 * Builds saved WAV path.
 */
String buildSavedWavPath(uint32_t seq) {
  return String(SAVED_MESSAGES_DIR) + "/" + buildMessageBaseName(seq) + ".wav";
}

/**
 * Builds saved metadata path.
 */
String buildSavedMetaPath(uint32_t seq) {
  return String(SAVED_MESSAGES_DIR) + "/" + buildMessageBaseName(seq) + ".txt";
}

/**
 * Builds mock-sent WAV path.
 */
String buildMockSentWavPath(uint32_t seq) {
  return String(MOCK_SENT_DIR) + "/" + buildMessageBaseName(seq) + ".wav";
}

/**
 * Builds mock-sent metadata path.
 */
String buildMockSentMetaPath(uint32_t seq) {
  return String(MOCK_SENT_DIR) + "/" + buildMessageBaseName(seq) + ".txt";
}

/**
 * Replaces a file extension with a new extension.
 */
String replaceExtension(const String& path, const char* newExt) {
  int dot = path.lastIndexOf('.');
  if (dot < 0) return path + newExt;
  return path.substring(0, dot) + newExt;
}

/**
 * Finds the highest message sequence in a directory.
 */
uint32_t findHighestSequenceInDir(const char* dirPath) {
  File dir = SD.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return 0;
  }

  uint32_t highest = 0;
  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (name.endsWith(".wav")) {
        uint32_t seq = extractSequenceFromName(name.substring(name.lastIndexOf('/') + 1));
        if (seq > highest) highest = seq;
      }
    }
    entry.close();
    entry = dir.openNextFile();
  }

  dir.close();
  return highest;
}

/**
 * Refreshes the next available sequence number based on SD contents.
 */
void refreshNextMessageSequence() {
  uint32_t a = findHighestSequenceInDir(MESSAGE_QUEUE_DIR);
  uint32_t b = findHighestSequenceInDir(SAVED_MESSAGES_DIR);
  uint32_t c = findHighestSequenceInDir(MOCK_SENT_DIR);
  uint32_t maxSeq = max(a, max(b, c));
  nextMessageSequence = maxSeq + 1;
}

/**
 * Counts WAV files in a directory.
 */
size_t countWavsInDir(const char* dirPath) {
  File dir = SD.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return 0;
  }

  size_t count = 0;
  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (name.endsWith(".wav")) count++;
    }
    entry.close();
    entry = dir.openNextFile();
  }

  dir.close();
  return count;
}

/**
 * Finds the oldest WAV file in a directory using lexical filename order.
 */
bool findOldestWavInDir(const char* dirPath, String& outWavPath, String& outBaseName) {
  File dir = SD.open(dirPath);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return false;
  }

  String bestName = "";
  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String fullName = entry.name();
      String shortName = fullName.substring(fullName.lastIndexOf('/') + 1);
      if (shortName.endsWith(".wav")) {
        if (bestName.length() == 0 || shortName < bestName) {
          bestName = shortName;
        }
      }
    }
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();

  if (bestName.length() == 0) return false;

  outWavPath = String(dirPath) + "/" + bestName;
  outBaseName = bestName.substring(0, bestName.lastIndexOf('.'));
  return true;
}

String sanitizeMacAddress(const String& value) {
  String clean = value;
  clean.trim();
  clean.toUpperCase();
  return clean;
}

int profileFromLogicalDeviceId(const String& logicalDeviceId) {
  if (logicalDeviceId == "ben_slate") return 0;
  if (logicalDeviceId == "pete_sapphire") return 1;
  if (logicalDeviceId == "bonnie_emerald") return 2;
  if (logicalDeviceId == "sophie_scarlet") return 3;
  return DEVICE_PROFILE;
}

String urlEncode(const String& value) {
  String encoded;
  static const char* hex = "0123456789ABCDEF";

  for (size_t i = 0; i < value.length(); ++i) {
    char c = value[i];
    bool safe =
      (c >= 'a' && c <= 'z') ||
      (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') ||
      c == '-' || c == '_' || c == '.' || c == '~';

    if (safe) {
      encoded += c;
    } else {
      encoded += '%';
      encoded += hex[(c >> 4) & 0x0F];
      encoded += hex[c & 0x0F];
    }
  }

  return encoded;
}

String bytesToBase64(const uint8_t* data, size_t len) {
  size_t outLen = 0;
  size_t needed = ((len + 2) / 3) * 4 + 1;
  uint8_t* out = (uint8_t*)malloc(needed);
  if (out == nullptr) return "";

  int rc = mbedtls_base64_encode(out, needed, &outLen, data, len);
  String result = "";
  if (rc == 0) {
    result = String((const char*)out);
  }

  free(out);
  return result;
}

bool decodeBase64(const String& input, uint8_t* output, size_t outputSize, size_t& decodedLen) {
  decodedLen = 0;
  return mbedtls_base64_decode(output, outputSize, &decodedLen, (const uint8_t*)input.c_str(), input.length()) == 0;
}

String hmacSha256Base64(const String& base64Key, const String& message) {
  uint8_t keyBytes[64] = {0};
  size_t keyLen = 0;
  if (!decodeBase64(base64Key, keyBytes, sizeof(keyBytes), keyLen)) {
    return "";
  }

  uint8_t digest[32] = {0};
  const mbedtls_md_info_t* mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (mdInfo == nullptr) return "";

  if (mbedtls_md_hmac(mdInfo,
                      keyBytes,
                      keyLen,
                      (const uint8_t*)message.c_str(),
                      message.length(),
                      digest) != 0) {
    return "";
  }

  return bytesToBase64(digest, sizeof(digest));
}

String sha256HexOfFile(const char* path) {
  if (!SD.exists(path)) return "";

  File file = SD.open(path, FILE_READ);
  if (!file) return "";

  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const mbedtls_md_info_t* mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (mdInfo == nullptr || mbedtls_md_setup(&ctx, mdInfo, 0) != 0 || mbedtls_md_starts(&ctx) != 0) {
    mbedtls_md_free(&ctx);
    file.close();
    return "";
  }

  uint8_t buffer[1024];
  while (file.available()) {
    int read = file.read(buffer, sizeof(buffer));
    if (read > 0) {
      mbedtls_md_update(&ctx, buffer, read);
    }
  }

  uint8_t digest[32] = {0};
  mbedtls_md_finish(&ctx, digest);
  mbedtls_md_free(&ctx);
  file.close();

  String hex;
  char part[3];
  for (size_t i = 0; i < sizeof(digest); ++i) {
    snprintf(part, sizeof(part), "%02x", digest[i]);
    hex += part;
  }

  return hex;
}

time_t getUnixTimeNow() {
  return time(nullptr);
}

bool loadWifiConfig() {
  wifiNetworkCount = 0;
  wifiConfigLoaded = false;

  if (!SD.exists(WIFI_CONFIG_PATH)) {
    Serial.println("Missing wifi.json");
    return false;
  }

  File file = SD.open(WIFI_CONFIG_PATH, FILE_READ);
  if (!file) return false;

  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    Serial.print("wifi.json parse failed: ");
    Serial.println(error.c_str());
    return false;
  }

  JsonArray networks = doc["networks"].as<JsonArray>();
  for (JsonObject network : networks) {
    if (wifiNetworkCount >= MAX_CONFIGURED_NETWORKS) break;
    wifiNetworks[wifiNetworkCount].name = String(network["name"] | "");
    wifiNetworks[wifiNetworkCount].ssid = String(network["ssid"] | "");
    wifiNetworks[wifiNetworkCount].password = String(network["password"] | "");
    wifiNetworks[wifiNetworkCount].priority = network["priority"] | 0;
    wifiNetworkCount++;
  }

  wifiConfigLoaded = wifiNetworkCount > 0;
  return wifiConfigLoaded;
}

bool loadIotConfig() {
  cloudDeviceCount = 0;
  iotConfigLoaded = false;

  if (!SD.exists(IOT_CONFIG_PATH)) {
    Serial.println("Missing iot.json");
    return false;
  }

  File file = SD.open(IOT_CONFIG_PATH, FILE_READ);
  if (!file) return false;

  DynamicJsonDocument doc(6144);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    Serial.print("iot.json parse failed: ");
    Serial.println(error.c_str());
    return false;
  }

  iotHubHostName = String(doc["iotHub"]["hostName"] | "");
  iotHubHostName.toLowerCase();

  JsonArray devices = doc["devices"].as<JsonArray>();
  for (JsonObject device : devices) {
    if (cloudDeviceCount >= MAX_CLOUD_DEVICES) break;
    cloudDevices[cloudDeviceCount].ownerName = String(device["ownerName"] | "");
    cloudDevices[cloudDeviceCount].edition = String(device["edition"] | "");
    cloudDevices[cloudDeviceCount].logicalDeviceId = String(device["logicalDeviceId"] | "");
    cloudDevices[cloudDeviceCount].iotDeviceId = String(device["iotDeviceId"] | "");
    cloudDevices[cloudDeviceCount].iotDeviceKey = String(device["iotDeviceKey"] | "");
    cloudDevices[cloudDeviceCount].wifiStaMac = sanitizeMacAddress(String(device["wifiStaMac"] | ""));
    cloudDevices[cloudDeviceCount].wifiApMac = sanitizeMacAddress(String(device["wifiApMac"] | ""));
    cloudDeviceCount++;
  }

  iotConfigLoaded = cloudDeviceCount > 0 && iotHubHostName.length() > 0;
  return iotConfigLoaded;
}

bool loadAzureServicesConfig() {
  azureServicesLoaded = false;
  if (!SD.exists(AZURE_SERVICES_PATH)) return false;

  File file = SD.open(AZURE_SERVICES_PATH, FILE_READ);
  if (!file) return false;

  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    Serial.print("azure_services.json parse failed: ");
    Serial.println(error.c_str());
    return false;
  }

  functionBaseUrl = String(doc["functionApp"]["defaultBaseUrl"] | functionBaseUrl.c_str());
  if (iotHubHostName.length() == 0) {
    iotHubHostName = String(doc["iotHub"]["hostName"] | "");
    iotHubHostName.toLowerCase();
  }

  azureServicesLoaded = true;
  return true;
}

bool resolveDeviceIdentity() {
  currentStaMac = sanitizeMacAddress(WiFi.macAddress());
  currentApMac = sanitizeMacAddress(WiFi.softAPmacAddress());

  activeCloudDeviceIndex = -1;
  for (size_t i = 0; i < cloudDeviceCount; ++i) {
    if (cloudDevices[i].wifiStaMac == currentStaMac) {
      activeCloudDeviceIndex = (int)i;
      break;
    }
  }

  if (activeCloudDeviceIndex < 0) {
    Serial.print("MAC not found in iot.json, using fallback profile. STA MAC=");
    Serial.println(currentStaMac);
    currentIotDeviceId = "";
    currentIotDeviceKey = "";
    activeDeviceProfile = DEVICE_PROFILE;
    cloudIdentityResolved = false;
    return false;
  }

  activeDeviceProfile = profileFromLogicalDeviceId(cloudDevices[activeCloudDeviceIndex].logicalDeviceId);
  currentIotDeviceId = cloudDevices[activeCloudDeviceIndex].iotDeviceId;
  currentIotDeviceKey = cloudDevices[activeCloudDeviceIndex].iotDeviceKey;
  cloudIdentityResolved = true;

  Serial.print("Resolved device identity from MAC: ");
  Serial.print(cloudDevices[activeCloudDeviceIndex].logicalDeviceId);
  Serial.print(" (");
  Serial.print(currentIotDeviceId);
  Serial.println(")");
  return true;
}

bool connectToWiFiIfNeeded() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return true;
  }

  if (!wifiConfigLoaded || wifiNetworkCount == 0) {
    wifiConnected = false;
    return false;
  }

  unsigned long now = millis();
  if (lastWifiAttemptMs != 0 && now - lastWifiAttemptMs < WIFI_RETRY_DELAY_MS) {
    wifiConnected = false;
    return false;
  }

  lastWifiAttemptMs = now;
  WiFi.mode(WIFI_STA);

  for (size_t i = 0; i < wifiNetworkCount; ++i) {
    Serial.print("Connecting to WiFi SSID: ");
    Serial.println(wifiNetworks[i].ssid);

    WiFi.begin(wifiNetworks[i].ssid.c_str(), wifiNetworks[i].password.c_str());
    unsigned long started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.print("WiFi connected, IP=");
      Serial.println(WiFi.localIP());
      return true;
    }
  }

  wifiConnected = false;
  return false;
}

bool syncClockIfNeeded() {
  if (timeSynced && getUnixTimeNow() > 1700000000) return true;
  if (!wifiConnected) return false;

  configTime(0, 0, NTP_SERVERS[0], NTP_SERVERS[1]);
  unsigned long started = millis();
  while (millis() - started < NTP_SYNC_TIMEOUT_MS) {
    time_t now = getUnixTimeNow();
    if (now > 1700000000) {
      timeSynced = true;
      Serial.print("Clock synced, unix=");
      Serial.println((unsigned long)now);
      return true;
    }
    delay(250);
  }

  return false;
}

String buildIotHubUsername() {
  return iotHubHostName + "/" + currentIotDeviceId + "/?api-version=2021-04-12";
}

String buildIotHubResourceUri() {
  return iotHubHostName + "/devices/" + currentIotDeviceId;
}

String buildIotHubSasToken(time_t expiry) {
  String resourceUri = buildIotHubResourceUri();
  String stringToSign = urlEncode(resourceUri) + "\n" + String((unsigned long)expiry);
  String signature = hmacSha256Base64(currentIotDeviceKey, stringToSign);
  if (signature.length() == 0) return "";

  return "SharedAccessSignature sr=" + urlEncode(resourceUri) +
         "&sig=" + urlEncode(signature) +
         "&se=" + String((unsigned long)expiry);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  Serial.print("MQTT topic: ");
  Serial.println(topic);
  if (length > 0) {
    Serial.print("MQTT payload: ");
    for (unsigned int i = 0; i < length; ++i) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }
}

bool publishPresence(const char* lifecycleState) {
  if (!mqttClient.connected()) return false;

  DynamicJsonDocument doc(512);
  doc["logicalDeviceId"] = getThisDeviceId();
  doc["ownerName"] = getThisDeviceName();
  doc["edition"] = getThisEditionName();
  doc["wifiStaMac"] = currentStaMac;
  doc["state"] = lifecycleState;
  doc["ipAddress"] = WiFi.localIP().toString();
  doc["freeHeap"] = ESP.getFreeHeap();

  String payload;
  serializeJson(doc, payload);

  String topic = "devices/" + currentIotDeviceId + "/messages/events/$.ct=application%2Fjson%3Bcharset%3Dutf-8";
  return mqttClient.publish(topic.c_str(), payload.c_str(), false);
}

bool publishHeartbeat() {
  return publishPresence("heartbeat");
}

bool publishTwinReportedProperties() {
  if (!mqttClient.connected()) return false;

  DynamicJsonDocument doc(512);
  doc["logicalDeviceId"] = getThisDeviceId();
  doc["ownerName"] = getThisDeviceName();
  doc["edition"] = getThisEditionName();
  doc["wifiStaMac"] = currentStaMac;
  doc["online"] = true;
  doc["queueDepth"] = countWavsInDir(MESSAGE_QUEUE_DIR);
  doc["ipAddress"] = WiFi.localIP().toString();
  doc["firmware"] = "fritz-phone-azure-v1";

  String payload;
  serializeJson(doc, payload);

  String topic = "$iothub/twin/PATCH/properties/reported/?$rid=" + String(millis());
  return mqttClient.publish(topic.c_str(), payload.c_str(), false);
}

bool ensureMqttConnection() {
  if (!cloudIdentityResolved || currentIotDeviceId.length() == 0 || currentIotDeviceKey.length() == 0) {
    mqttConnected = false;
    return false;
  }

  if (!wifiConnected || !syncClockIfNeeded()) {
    mqttConnected = false;
    return false;
  }

  unsigned long nowMs = millis();
  if (!mqttClient.connected() && lastMqttConnectAttemptMs != 0 && nowMs - lastMqttConnectAttemptMs < MQTT_RECONNECT_INTERVAL_MS) {
    mqttConnected = false;
    return false;
  }

  configureSecureClient(mqttTlsClient);
  mqttClient.setServer(iotHubHostName.c_str(), 8883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(90);

  time_t now = getUnixTimeNow();
  if (currentSasToken.length() == 0 || now + 300 >= currentSasExpiry) {
    currentSasExpiry = now + IOT_SAS_LIFETIME_SECONDS;
    currentSasToken = buildIotHubSasToken(currentSasExpiry);
  }

  if (mqttClient.connected()) {
    mqttConnected = true;
    return true;
  }

  String username = buildIotHubUsername();
  Serial.print("Connecting to IoT Hub as ");
  Serial.println(currentIotDeviceId);
  lastMqttConnectAttemptMs = nowMs;
  mqttTlsClient.stop();

  bool ok = mqttClient.connect(currentIotDeviceId.c_str(), username.c_str(), currentSasToken.c_str());
  mqttConnected = ok;

  if (!ok) {
    mqttClient.disconnect();
    mqttTlsClient.stop();
    Serial.print("IoT Hub MQTT connect failed, state=");
    Serial.println(mqttClient.state());
    Serial.print("Heap now=");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" minHeap=");
    Serial.println(ESP.getMinFreeHeap());
    return false;
  }

  mqttClient.subscribe("$iothub/twin/res/#");
  publishPresence("connected");
  publishTwinReportedProperties();
  lastHeartbeatMs = millis();
  lastTwinReportMs = millis();
  return true;
}

String buildFunctionAuthSignature(const String& method, const String& route, const String& timestampUtc) {
  String canonical = method + "\n" + route + "\n" + timestampUtc + "\n" + currentIotDeviceId;
  return hmacSha256Base64(currentIotDeviceKey, canonical);
}

void configureSecureClient(WiFiClientSecure& client) {
  client.setInsecure();
  client.setHandshakeTimeout(TLS_HANDSHAKE_TIMEOUT_SEC);
}

void addFunctionAuthHeaders(HTTPClient& http, const String& method, const String& route) {
  time_t now = getUnixTimeNow();
  String timestampUtc = String((unsigned long)now);
  String signature = buildFunctionAuthSignature(method, route, timestampUtc);

  http.addHeader("x-fritz-device-id", currentIotDeviceId);
  http.addHeader("x-fritz-logical-device-id", getThisDeviceId());
  http.addHeader("x-fritz-timestamp", timestampUtc);
  http.addHeader("x-fritz-signature", signature);
}

bool enqueueDownloadedCloudMessage(const PendingCloudMessage& message, Stream& source) {
  uint32_t seq = nextMessageSequence++;
  String wavPath = buildQueuedWavPath(seq);
  String metaPath = buildQueuedMetaPath(seq);

  deleteFileIfExists(wavPath.c_str());
  deleteFileIfExists(metaPath.c_str());

  File wavFile = SD.open(wavPath.c_str(), FILE_WRITE);
  if (!wavFile) return false;

  uint8_t buffer[1024];
  uint32_t remaining = message.messageLength;
  uint32_t totalWritten = 0;
  unsigned long lastReadMs = millis();
  unsigned long lastDownloadLedMs = 0;
  bool downloadLedOn = false;

  while (remaining > 0 && millis() - lastReadMs < DOWNLOAD_HTTP_TIMEOUT_MS) {
    unsigned long now = millis();
    if (lastDownloadLedMs == 0 || now - lastDownloadLedMs >= DOWNLOAD_LED_INTERVAL_MS) {
      lastDownloadLedMs = now;
      downloadLedOn = !downloadLedOn;
      setLeds(downloadLedOn, downloadLedOn);
    }

    size_t chunk = min((size_t)remaining, sizeof(buffer));
    int read = source.readBytes(buffer, chunk);
    if (read <= 0) {
      delay(10);
      continue;
    }

    lastReadMs = millis();
    if (wavFile.write(buffer, read) != (size_t)read) {
      wavFile.close();
      updateLedState();
      deleteFileIfExists(wavPath.c_str());
      deleteFileIfExists(metaPath.c_str());
      Serial.println("Queue download failed: SD write error.");
      return false;
    }
    totalWritten += read;
    remaining -= read;
  }

  wavFile.close();

  if (totalWritten != message.messageLength) {
    deleteFileIfExists(wavPath.c_str());
    deleteFileIfExists(metaPath.c_str());
    Serial.print("Queue download failed: expected bytes=");
    Serial.print(message.messageLength);
    Serial.print(" actual=");
    Serial.println(totalWritten);
    updateLedState();
    return false;
  }

  bool wrote = writeCloudMetadata(metaPath.c_str(), message, "downloaded");
  if (!wrote) {
    deleteFileIfExists(wavPath.c_str());
    deleteFileIfExists(metaPath.c_str());
    Serial.println("Queue download failed: metadata write error.");
    updateLedState();
    return false;
  }

  if (!isValidQueuedMessage(wavPath, metaPath)) {
    logFileHeaderBytes(wavPath, "downloaded queue wav");
    deleteFileIfExists(wavPath.c_str());
    deleteFileIfExists(metaPath.c_str());
    Serial.println("Queue download failed: queued WAV validation failed.");
    updateLedState();
    return false;
  }

  refreshQueueDepth(true);
  updateLedState();
  return true;
}

bool acknowledgeDelivery(const String& deliveryId, const String& status) {
  if (!wifiConnected) return false;

  String route = "/messages/ack";
  String url = functionBaseUrl + route + "?deliveryId=" + urlEncode(deliveryId);

  WiFiClientSecure client;
  configureSecureClient(client);
  HTTPClient http;
  if (!http.begin(client, url)) return false;

  addFunctionAuthHeaders(http, "POST", route);
  http.addHeader("Content-Type", "application/json");

  DynamicJsonDocument doc(256);
  doc["status"] = status;
  doc["deviceLogicalId"] = getThisDeviceId();
  String body;
  serializeJson(doc, body);

  int statusCode = http.POST(body);
  if (statusCode < 200 || statusCode >= 300) {
    Serial.print("Delivery ack failed, status=");
    Serial.println(statusCode);
  }
  http.end();
  return statusCode >= 200 && statusCode < 300;
}

bool downloadPendingMessages() {
  if (!wifiConnected) return false;

  String route = "/messages/pending";
  String query = "?recipientId=" + urlEncode(getThisDeviceId()) + "&limit=4";
  String url = functionBaseUrl + route + query;

  WiFiClientSecure client;
  configureSecureClient(client);
  HTTPClient http;
  if (!http.begin(client, url)) return false;

  addFunctionAuthHeaders(http, "GET", route);
  http.setTimeout(DOWNLOAD_HTTP_TIMEOUT_MS);

  int statusCode = http.GET();
  if (statusCode != HTTP_CODE_OK) {
    Serial.print("Pending poll failed, status=");
    Serial.println(statusCode);
    http.end();
    return false;
  }

  String response = http.getString();
  http.end();

  PendingCloudMessage pendingMessages[4];
  size_t pendingCount = 0;

  {
    DynamicJsonDocument doc(12288);
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
      Serial.print("Pending response parse failed: ");
      Serial.println(error.c_str());
      return false;
    }

    JsonArray deliveries = doc["deliveries"].as<JsonArray>();
    for (JsonObject delivery : deliveries) {
      if (pendingCount >= 4) break;

      PendingCloudMessage& message = pendingMessages[pendingCount];
      message.deliveryId = String(delivery["deliveryId"] | "");
      message.messageId = String(delivery["messageId"] | "");
      message.senderId = String(delivery["senderId"] | "");
      message.senderName = String(delivery["senderName"] | "");
      message.receiverId = String(delivery["receiverId"] | "");
      message.receiverName = String(delivery["receiverName"] | "");
      message.payloadPath = String(delivery["payloadPath"] | "");
      message.downloadUrl = String(delivery["downloadUrl"] | "");
      message.messageLength = delivery["messageLength"] | 0;
      message.durationMs = delivery["durationMs"] | 0;
      message.timeSentUtc = String(delivery["timeSentUtc"] | "");

      if (message.downloadUrl.length() == 0 || message.deliveryId.length() == 0 || message.messageLength == 0) {
        continue;
      }

      pendingCount++;
    }
  }

  response = String();

  for (size_t i = 0; i < pendingCount; ++i) {
    PendingCloudMessage& message = pendingMessages[i];
    bool saved = false;
    {
      cloudDownloadActive = true;
      updateLedState();
      WiFiClientSecure downloadClient;
      configureSecureClient(downloadClient);
      HTTPClient downloadHttp;
      if (!downloadHttp.begin(downloadClient, message.downloadUrl)) {
        cloudDownloadActive = false;
        updateLedState();
        continue;
      }

      downloadHttp.setTimeout(DOWNLOAD_HTTP_TIMEOUT_MS);
      int downloadStatus = downloadHttp.GET();
      if (downloadStatus != HTTP_CODE_OK) {
        downloadHttp.end();
        cloudDownloadActive = false;
        updateLedState();
        continue;
      }

      saved = enqueueDownloadedCloudMessage(message, *downloadHttp.getStreamPtr());
      downloadHttp.end();
      downloadClient.stop();
      cloudDownloadActive = false;
      updateLedState();
    }

    if (saved) {
      bool acked = acknowledgeDelivery(message.deliveryId, "Downloaded");
      Serial.print("Downloaded queued message from ");
      Serial.print(message.senderName);
      Serial.print(" ack=");
      Serial.print(acked ? "ok" : "failed");
      Serial.print(" heap=");
      Serial.println(ESP.getFreeHeap());
    }
  }

  return true;
}

bool pollCloudInboxIfDue() {
  if (!wifiConnected) return false;
  unsigned long now = millis();
  if (lastPollMs != 0 && now - lastPollMs < PENDING_POLL_INTERVAL_MS) {
    return false;
  }

  lastPollMs = now;
  return downloadPendingMessages();
}

bool sendDeviceHeartbeat(bool force) {
  if (!wifiConnected) return false;
  if (!syncClockIfNeeded()) return false;

  unsigned long nowMs = millis();
  if (!force && lastPresencePingMs != 0 && nowMs - lastPresencePingMs < HEARTBEAT_INTERVAL_MS) {
    return false;
  }

  String route = "/device/ping";
  String url = functionBaseUrl + route;

  WiFiClientSecure client;
  configureSecureClient(client);
  HTTPClient http;
  if (!http.begin(client, url)) return false;

  addFunctionAuthHeaders(http, "POST", route);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(10000);

  DynamicJsonDocument doc(512);
  doc["logicalDeviceId"] = getThisDeviceId();
  doc["ownerName"] = getThisDeviceName();
  doc["edition"] = getThisEditionName();
  doc["wifiStaMac"] = currentStaMac;
  doc["ipAddress"] = WiFi.localIP().toString();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["queueDepth"] = countWavsInDir(MESSAGE_QUEUE_DIR);
  doc["firmware"] = "fritz-phone-azure-v2";
  doc["mode"] = "DirectButtons";
  doc["actionState"] = (int)currentActionState;

  String body;
  serializeJson(doc, body);

  int statusCode = http.POST(body);
  http.end();

  if (statusCode >= 200 && statusCode < 300) {
    lastPresencePingMs = nowMs;
    return true;
  }

  Serial.print("Presence ping failed, status=");
  Serial.println(statusCode);
  return false;
}

bool shouldRunBackgroundNetworkTasks() {
  if (!wifiConnected) return false;
  if (!online) return false;
  if (recordingActive) return false;
  if (currentActionState != ActionState::Idle) return false;
  if (hasQueuedMessages()) return false;
  if (b4PendingSingleClick) return false;
  return true;
}

bool sendTempMessageToCloud() {
  if (!SD.exists(TEMP_OUTGOING_WAV) || !SD.exists(TEMP_OUTGOING_META)) {
    Serial.println("No temp message to send to cloud.");
    return false;
  }

  if (!wifiConnected) {
    Serial.println("Cannot send: WiFi not connected.");
    return false;
  }

  String messageId = readMetadataValue(TEMP_OUTGOING_META, "MESSAGE_ID");
  String recipientId = readMetadataValue(TEMP_OUTGOING_META, "TO_ID");
  String recipientName = readMetadataValue(TEMP_OUTGOING_META, "TO_NAME");
  String durationMs = readMetadataValue(TEMP_OUTGOING_META, "DURATION_MS");

  if (!isValidWavFile(TEMP_OUTGOING_WAV)) {
    Serial.println("Cloud send blocked: temp WAV validation failed.");
    logFileHeaderBytes(TEMP_OUTGOING_WAV, "temp outgoing wav");
    return false;
  }

  File wavFile = SD.open(TEMP_OUTGOING_WAV, FILE_READ);
  if (!wavFile) return false;

  String route = "/messages/send";
  String query = "?messageId=" + urlEncode(messageId) +
                 "&recipientId=" + urlEncode(recipientId) +
                 "&durationMs=" + urlEncode(durationMs);
  String url = functionBaseUrl + route + query;

  WiFiClientSecure client;
  configureSecureClient(client);
  HTTPClient http;
  if (!http.begin(client, url)) {
    wavFile.close();
    return false;
  }

  cloudUploadActive = true;
  lastBusyBlinkMs = millis();
  busyBlinkState = true;
  flashTransferLeds(2, 55, 45);
  updateLedState();

  addFunctionAuthHeaders(http, "POST", route);
  http.addHeader("Content-Type", "audio/wav");
  http.addHeader("x-fritz-recipient-name", recipientName);
  http.addHeader("x-fritz-message-length", String((unsigned long)wavFile.size()));
  http.setTimeout((UPLOAD_HTTP_TIMEOUT_MS > 65535u) ? 65535u : (uint16_t)UPLOAD_HTTP_TIMEOUT_MS);

  int statusCode = http.sendRequest("POST", &wavFile, wavFile.size());
  wavFile.close();

  String response = http.getString();
  http.end();
  cloudUploadActive = false;
  updateLedState();

  if (statusCode < 200 || statusCode >= 300) {
    Serial.print("Cloud send failed, status=");
    Serial.println(statusCode);
    Serial.println(response);
    return false;
  }

  DynamicJsonDocument doc(1024);
  DeserializationError parseError = deserializeJson(doc, response);
  if (!parseError) {
    Serial.print("Cloud send ok, deliveries=");
    Serial.println((int)(doc["deliveryCount"] | 0));
    Serial.print("Payload path: ");
    Serial.println(String(doc["payloadPath"] | ""));
  }

  return true;
}

bool cloudSendCurrentTempMessage() {
  uint8_t recipientMask = activeRecipientMask;
  ActionState previousState = currentActionState;
  currentActionState = ActionState::Sending;
  updateLedState();

  bool ok = sendTempMessageToCloud();
  if (!ok) {
    currentActionState = previousState;
    updateLedState();
    return false;
  }

  String messageId = readMetadataValue(TEMP_OUTGOING_META, "MESSAGE_ID");
  uint32_t seq = extractSequenceFromName(messageId + ".wav");
  if (seq == 0) seq = nextMessageSequence;

  String dstWav = buildMockSentWavPath(seq);
  String dstMeta = buildMockSentMetaPath(seq);
  bool okWav = moveFile(TEMP_OUTGOING_WAV, dstWav.c_str());
  bool okMeta = moveFile(TEMP_OUTGOING_META, dstMeta.c_str());

  if (okWav && okMeta) {
    nextMessageSequence = max(nextMessageSequence, seq + 1);
    currentActionState = previousState;
    updateLedState();
    playMessageSentRecipientSequence(recipientMask);
    return true;
  }

  currentActionState = previousState;
  updateLedState();
  playFeedbackToneError();
  return false;
}

// ============================================================
// METADATA METHODS
// ============================================================

/**
 * Writes a simple key=value metadata file for a message.
 */
bool writeSimpleMetadata(const char* path,
                         const String& messageId,
                         const String& fromId,
                         const String& fromName,
                         const String& toId,
                         const String& toName,
                         uint32_t durationMs,
                         const String& status) {
  if (SD.exists(path)) {
    SD.remove(path);
  }

  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;

  f.print("MESSAGE_ID=");
  f.println(messageId);
  f.print("FROM_ID=");
  f.println(fromId);
  f.print("FROM_NAME=");
  f.println(fromName);
  f.print("TO_ID=");
  f.println(toId);
  f.print("TO_NAME=");
  f.println(toName);
  f.print("DURATION_MS=");
  f.println(durationMs);
  f.print("STATUS=");
  f.println(status);
  f.close();
  return true;
}

/**
 * Reads a single metadata value by key from a key=value text file.
 */
String readMetadataValue(const char* path, const char* key) {
  File f = SD.open(path, FILE_READ);
  if (!f) return "";

  String wanted = String(key) + "=";
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith(wanted)) {
      f.close();
      return line.substring(wanted.length());
    }
  }

  f.close();
  return "";
}

bool writeCloudMetadata(const char* path, const PendingCloudMessage& message, const String& status) {
  if (SD.exists(path)) {
    SD.remove(path);
  }

  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;

  f.print("MESSAGE_ID=");
  f.println(message.messageId);
  f.print("DELIVERY_ID=");
  f.println(message.deliveryId);
  f.print("FROM_ID=");
  f.println(message.senderId);
  f.print("FROM_NAME=");
  f.println(message.senderName);
  f.print("TO_ID=");
  f.println(message.receiverId);
  f.print("TO_NAME=");
  f.println(message.receiverName);
  f.print("DURATION_MS=");
  f.println(message.durationMs);
  f.print("MESSAGE_LENGTH=");
  f.println(message.messageLength);
  f.print("TIME_SENT_UTC=");
  f.println(message.timeSentUtc);
  f.print("PAYLOAD_PATH=");
  f.println(message.payloadPath);
  f.print("STATUS=");
  f.println(status);
  f.close();
  return true;
}

// ============================================================
// I2S / WAV / AUDIO METHODS
// ============================================================

/**
 * Stops and uninstalls the current I2S driver.
 */
void stopI2S() {
  if (!i2sInstalled) return;
  i2s_driver_uninstall(I2S_PORT);
  i2sInstalled = false;
}

/**
 * Starts I2S in microphone receive mode.
 */
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
    Serial.println("I2S RX install failed");
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
    Serial.println("I2S RX pin config failed");
    stopI2S();
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  return true;
}

/**
 * Starts I2S in speaker transmit mode.
 */
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
    Serial.println("I2S TX install failed");
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
    Serial.println("I2S TX pin config failed");
    stopI2S();
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  return true;
}

/**
 * Writes a standard PCM WAV header to an open file.
 */
void writeWavHeader(File& f, uint32_t dataBytes) {
  f.seek(0);

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

/**
 * Begins non-blocking recording to the temp outgoing WAV file.
 */
bool beginRecordingToTemp() {
  if (!sdReady) return false;
  if (recordingActive) return false;

  deleteFileIfExists(TEMP_OUTGOING_WAV);
  deleteFileIfExists(TEMP_OUTGOING_META);

  if (!startI2SMic()) return false;

  recordingFile = SD.open(TEMP_OUTGOING_WAV, FILE_WRITE);
  if (!recordingFile) {
    Serial.println("Failed to open temp outgoing WAV");
    stopI2S();
    return false;
  }

  writeWavHeader(recordingFile, 0);
  recordingDataBytes = 0;
  recordingStartedMs = millis();
  recordHpfState = 0.0f;
  recordLpfState = 0.0f;
  recordPrevIn = 0;

  recordingActive = true;
  currentActionState = ActionState::Recording;
  Serial.println("Recording started");
  return true;
}

/**
 * Records one chunk of microphone data into the temp WAV file.
 */
bool continueRecordingToTemp() {
  if (!recordingActive) return false;

  int32_t i2sBuf[RECORD_CHUNK_SAMPLES];
  int16_t pcmBuf[RECORD_CHUNK_SAMPLES];
  size_t bytesRead = 0;

  esp_err_t res = i2s_read(I2S_PORT, (void*)i2sBuf, sizeof(i2sBuf), &bytesRead, portMAX_DELAY);
  if (res != ESP_OK || bytesRead == 0) {
    Serial.println("I2S read error during recording");
    return false;
  }

  size_t samplesRead = bytesRead / sizeof(int32_t);
  int16_t peak = 0;

  for (size_t i = 0; i < samplesRead; ++i) {
    float scaledSample = ((float)i2sBuf[i] / (float)(1 << RECORD_INPUT_SHIFT));
    scaledSample *= RECORD_INPUT_GAIN;
    if (scaledSample > INT16_MAX) scaledSample = INT16_MAX;
    if (scaledSample < INT16_MIN) scaledSample = INT16_MIN;
    int16_t s = (int16_t)scaledSample;
    int16_t absSample = abs(s);
    if (absSample > peak) peak = absSample;

    //This bit smooths out our gate threshold by squaring the 
    float gate = abs(s) / (float)GATE_THRESHOLD;

    if (gate < 1.0f) {
        gate = gate * gate;  // smooth curve
        s = (int16_t)(s * gate);
    }

    float yHP = HPF_ALPHA * (recordHpfState + (float)s - (float)recordPrevIn);
    recordHpfState = yHP;
    recordPrevIn = s;

    float yLP = recordLpfState + LPF_ALPHA * (yHP - recordLpfState);
    recordLpfState = yLP;

    int32_t out = (int32_t)yLP;
    if (out > INT16_MAX) out = INT16_MAX;
    if (out < INT16_MIN) out = INT16_MIN;

    pcmBuf[i] = (int16_t)out;
  }

  size_t bytesToWrite = samplesRead * sizeof(int16_t);
  recordingFile.write((uint8_t*)pcmBuf, bytesToWrite);
  recordingDataBytes += bytesToWrite;

  unsigned long now = millis();
  if (now - lastMicDebugMs >= 500) {
    lastMicDebugMs = now;
    Serial.print("Mic debug: bytesRead=");
    Serial.print(bytesRead);
    Serial.print(" peak=");
    Serial.println(peak);
  }

  if ((millis() - recordingStartedMs) >= MAX_RECORD_MS) {
    Serial.println("Max recording duration reached.");
    return finishRecordingToTemp();
  }

  return true;
}

/**
 * Finalizes the temp recording and writes metadata.
 */
bool finishRecordingToTemp() {
  if (!recordingActive) return false;

  uint32_t durationMs = millis() - recordingStartedMs;
  writeWavHeader(recordingFile, recordingDataBytes);
  recordingFile.close();
  stopI2S();

  recordingActive = false;
  currentActionState = ActionState::Idle;

  String messageId = buildMessageBaseName(nextMessageSequence);
  String toId;
  String toName;

  if (!buildRecipientSelectionStrings(activeRecipientMask, toId, toName)) {
    const Contact& c = getCurrentRecipient();
    toId = c.deviceId;
    toName = c.displayName;
  }

  bool ok = writeSimpleMetadata(
    TEMP_OUTGOING_META,
    messageId,
    getThisDeviceId(),
    getThisDeviceName(),
    toId,
    toName,
    durationMs,
    "temp_recorded"
  );

  Serial.print("Recording finished: ");
  Serial.print(durationMs);
  Serial.print(" ms, bytes=");
  Serial.println(recordingDataBytes);

  return ok;
}

bool recordFixedDurationToWav(uint16_t seconds, const char* path) {
  if (!sdReady) return false;
  if (!startI2SMic()) return false;

  deleteFileIfExists(path);
  File wav = SD.open(path, FILE_WRITE);
  if (!wav) {
    Serial.println("Failed to open self-test WAV");
    stopI2S();
    return false;
  }

  writeWavHeader(wav, 0);

  const uint32_t totalSamplesTarget = (uint32_t)seconds * SAMPLE_RATE;
  uint32_t samplesWritten = 0;
  int32_t i2sBuf[RECORD_CHUNK_SAMPLES];
  int16_t pcmBuf[RECORD_CHUNK_SAMPLES];
  float hpfState = 0.0f;
  float lpfState = 0.0f;
  int16_t prevIn = 0;
  int16_t sessionPeak = 0;

  while (samplesWritten < totalSamplesTarget) {
    size_t bytesRead = 0;
    esp_err_t res = i2s_read(I2S_PORT, (void*)i2sBuf, sizeof(i2sBuf), &bytesRead, portMAX_DELAY);
    if (res != ESP_OK || bytesRead == 0) {
      Serial.println("Self-test I2S read failed");
      wav.close();
      stopI2S();
      return false;
    }

    size_t samplesRead = bytesRead / sizeof(int32_t);
    int16_t chunkPeak = 0;

    for (size_t i = 0; i < samplesRead; ++i) {
      float scaledSample = ((float)i2sBuf[i] / (float)(1 << RECORD_INPUT_SHIFT));
      scaledSample *= RECORD_INPUT_GAIN;
      if (scaledSample > INT16_MAX) scaledSample = INT16_MAX;
      if (scaledSample < INT16_MIN) scaledSample = INT16_MIN;
      int16_t s = (int16_t)scaledSample;
      int16_t absSample = abs(s);
      if (absSample > chunkPeak) chunkPeak = absSample;
      if (absSample > sessionPeak) sessionPeak = absSample;

      float gate = abs(s) / (float)GATE_THRESHOLD;
      if (gate < 1.0f) {
        gate = gate * gate;
        s = (int16_t)(s * gate);
      }

      float yHP = HPF_ALPHA * (hpfState + (float)s - (float)prevIn);
      hpfState = yHP;
      prevIn = s;

      float yLP = lpfState + LPF_ALPHA * (yHP - lpfState);
      lpfState = yLP;

      int32_t out = (int32_t)yLP;
      if (out > INT16_MAX) out = INT16_MAX;
      if (out < INT16_MIN) out = INT16_MIN;
      pcmBuf[i] = (int16_t)out;
    }

    wav.write((uint8_t*)pcmBuf, samplesRead * sizeof(int16_t));
    samplesWritten += samplesRead;

    Serial.print("Mic self-test chunk peak=");
    Serial.println(chunkPeak);
  }

  uint32_t dataBytes = samplesWritten * sizeof(int16_t);
  writeWavHeader(wav, dataBytes);
  wav.close();
  stopI2S();

  Serial.print("Mic self-test saved to ");
  Serial.print(path);
  Serial.print(", bytes=");
  Serial.print(dataBytes);
  Serial.print(", sessionPeak=");
  Serial.println(sessionPeak);
  return true;
}

/**
 * Cancels an active temp recording and deletes temp files.
 */
bool cancelRecordingToTemp() {
  if (recordingActive) {
    recordingFile.close();
    stopI2S();
  }

  recordingActive = false;
  currentActionState = ActionState::Idle;
  deleteFileIfExists(TEMP_OUTGOING_WAV);
  deleteFileIfExists(TEMP_OUTGOING_META);
  return true;
}

/**
 * Plays a WAV file at the current system volume.
 */
bool playWav(const char* path) {
  return playWavScaled(path, getCurrentVolumeMultiplier());
}

/**
 * Plays a WAV file at the current system volume.
 */
bool playWav(const String& path) {
  return playWavScaled(path.c_str(), getCurrentVolumeMultiplier());
}

/**
 * Plays a WAV file using a specific gain multiplier.
 */
bool playWavScaled(const char* path, float volumeScale) {
  if (!SD.exists(path)) {
    Serial.print("Missing WAV: ");
    Serial.println(path);
    return false;
  }

  File wav = SD.open(path, FILE_READ);
  if (!wav) return false;

  if (!startI2SSpeaker()) {
    wav.close();
    return false;
  }

  currentActionState = ActionState::Playing;
  wav.seek(44);
  setLeds(false, true);

  const size_t SAMPLE_BUF_COUNT = 512;
  int16_t sampleBuf[SAMPLE_BUF_COUNT];

  while (true) {
    int n = wav.read((uint8_t*)sampleBuf, sizeof(sampleBuf));
    if (n <= 0) break;

    size_t samples = n / sizeof(int16_t);
    for (size_t i = 0; i < samples; ++i) {
      sampleBuf[i] = scaleSample(sampleBuf[i], volumeScale);
    }

    size_t written = 0;
    uint8_t* raw = (uint8_t*)sampleBuf;
    size_t bytesTotal = samples * sizeof(int16_t);

    while (written < bytesTotal) {
      size_t out = 0;
      if (i2s_write(I2S_PORT, raw + written, bytesTotal - written, &out, portMAX_DELAY) != ESP_OK) {
        wav.close();
        stopI2S();
        currentActionState = ActionState::Idle;
        updateLedState();
        return false;
      }
      written += out;
    }
  }

  wav.close();
  stopI2S();
  currentActionState = ActionState::Idle;
  updateLedState();
  return true;
}

/**
 * Plays a WAV file using a specific gain multiplier.
 */
bool playWavScaled(const String& path, float volumeScale) {
  return playWavScaled(path.c_str(), volumeScale);
}

/**
 * Applies gain with clipping to a signed PCM sample.
 */
int16_t scaleSample(int16_t sample, float gain) {
  int32_t v = (int32_t)((float)sample * gain);
  if (v > INT16_MAX) v = INT16_MAX;
  if (v < INT16_MIN) v = INT16_MIN;
  return (int16_t)v;
}

/**
 * Plays the current temp outgoing recording if it exists.
 */
bool playCurrentTempRecording() {
  if (!SD.exists(TEMP_OUTGOING_WAV)) {
    Serial.println("No temp recording to preview.");
    playFeedbackToneError();
    return false;
  }

  Serial.println("Previewing temp recording...");
  return playWav(TEMP_OUTGOING_WAV);
}

/**
 * Plays a generated sine tone through the speaker.
 */
void playTone(uint16_t freq, uint16_t ms) {
  playToneWithGain(freq, ms, getCurrentVolumeMultiplier());
}

/**
 * Plays a generated sine tone through the speaker using an explicit gain.
 */
void playToneWithGain(uint16_t freq, uint16_t ms, float gain) {
  if (!startI2SSpeaker()) return;

  const float twoPiF = 2.0f * PI * freq;
  const uint16_t samples = (SAMPLE_RATE * ms) / 1000;

  for (uint16_t i = 0; i < samples; ++i) {
    float t = (float)i / SAMPLE_RATE;
    float s = sinf(twoPiF * t);
    int16_t sample = scaleSample((int16_t)(s * 24000), gain);
    size_t out;
    i2s_write(I2S_PORT, &sample, sizeof(sample), &out, portMAX_DELAY);
  }

  stopI2S();
}

/**
 * Plays a short success tone.
 */
void playFeedbackToneOK() {
  playTone(740, 100);
  delay(40);
  playTone(1040, 120);
}

/**
 * Plays a short error tone.
 */
void playFeedbackToneError() {
  playTone(300, 140);
  delay(30);
  playTone(220, 180);
}

/**
 * Plays a tone that indicates receiving mode.
 */
void playFeedbackToneModeReceive() {
  playTone(520, 100);
}

/**
 * Plays a tone that indicates sending mode.
 */
void playFeedbackToneModeSend() {
  playTone(880, 100);
}

/**
 * Plays a tone that indicates a volume change.
 */
void playFeedbackToneVolume() {
  static const uint16_t VOLUME_SCALE_TONES[5] = {262, 294, 330, 392, 440};
  uint16_t tone = VOLUME_SCALE_TONES[currentVolumeIndex];
  playTone(tone, 160);
}

/**
 * Returns the dedicated notification note for a recipient.
 */
uint16_t getRecipientNotificationTone(const Contact& contact) {
  switch (contact.id) {
    case ContactId::Bonnie: return 494; // scale note 3
    case ContactId::Pete:   return 523; // scale note 4
    case ContactId::Ben:    return 554; // scale note 5
    case ContactId::Sophie: return 587; // scale note 6
    default:                return 523;
  }
}

/**
 * Plays the short fixed beep that marks the start of recording.
 */
void playRecordStartBeep() {
  playToneWithGain(415, 140, 1.0f); // scale note 0
}

/**
 * Plays one short confirmation tone per selected recipient after recording completes.
 */
void playRecipientRoutingBeepSequence(uint8_t recipientMask) {
  bool playedAny = false;
  for (uint8_t i = 0; i < 3; ++i) {
    if ((recipientMask & (1u << i)) == 0) continue;
    const Contact& contact = getButtonRecipient(i);
    playToneWithGain(getRecipientNotificationTone(contact), 115, 1.0f);
    delay(60);
    playedAny = true;
  }

  if (!playedAny) {
    playFeedbackToneError();
  }
}

/**
 * Plays the selected recipients' dedicated send-confirmation notes.
 */
void playMessageSentRecipientSequence(uint8_t recipientMask) {
  bool playedAny = false;
  for (uint8_t i = 0; i < 3; ++i) {
    if ((recipientMask & (1u << i)) == 0) continue;

    const Contact& contact = getButtonRecipient(i);
    playToneWithGain(getRecipientNotificationTone(contact), 145, 1.0f);
    delay(75);
    playedAny = true;
  }

  if (!playedAny) {
    playFeedbackToneError();
  }
}

/**
 * Plays a WAV prompt if present on SD, otherwise returns false.
 */
bool playOptionalPrompt(const char* path, float volumeScale) {
  if (!sdReady || !SD.exists(path)) return false;
  if (volumeScale < 0.0f) {
    return playWav(path);
  }
  return playWavScaled(path, volumeScale);
}

/**
 * Plays the edition prompt for the current device.
 */
bool playEditionPrompt() {
  switch (activeDeviceProfile) {
    case 0:
      return false;
    case 1:
      return playOptionalPrompt(PROMPT_SAPPHIRE_EDITION);
    case 2:
      return playOptionalPrompt(PROMPT_EMERALD_EDITION);
    case 3:
      return playOptionalPrompt(PROMPT_SCARLET_EDITION);
    default:
      return false;
  }
}

/**
 * Plays the startup phrase and current edition.
 */
void playStartupAnnouncement() {
  bool playedInit = playOptionalPrompt(PROMPT_INITIALIZING_FRITZ_PHONE);
  if (playedInit) {
    delay(120);
  }

  bool playedEdition = playEditionPrompt();
  if (!playedInit && !playedEdition) {
    playFeedbackToneOK();
  }
}

/**
 * Plays the "sending to" prompt if present.
 */
void playSendingToPrompt() {
  playOptionalPrompt(PROMPT_SENDING_TO);
}

/**
 * Plays the "message sent to" prompt if present, otherwise falls back to "sending to".
 */
void playMessageSentToPrompt() {
  if (!playOptionalPrompt(PROMPT_MESSAGE_SENT_TO)) {
    playSendingToPrompt();
  }
}

/**
 * Plays the currently selected recipient name if the WAV exists, otherwise prints to Serial and beeps.
 */
void playRecipientPrompt() {
  const Contact& c = getCurrentRecipient();
  unsigned long now = millis();
  bool shouldPlayPrefix = sendToPrefixArmed || ((now - lastRecipientPromptMs) >= SEND_TO_PREFIX_RESET_MS);

  if (shouldPlayPrefix) {
    playSendingToPrompt();
    delay(100);
  }

  if (SD.exists(c.spokenNamePath)) {
    playWavScaled(c.spokenNamePath, 0.85f);
  } else {
    Serial.print("Recipient prompt missing, selected: ");
    Serial.println(c.displayName);
    playFeedbackToneOK();
  }

  sendToPrefixArmed = false;
  lastRecipientPromptMs = now;
}

/**
 * Plays "message from" followed by the sender name if those assets exist.
 */
void playMessageFromPrompt(const char* senderNamePath) {
  bool playedPrefix = playOptionalPrompt(PROMPT_NEW_MESSAGE_FROM);
  if (!playedPrefix) {
    playedPrefix = playOptionalPrompt(PROMPT_MESSAGE_FROM);
  }
  if (playedPrefix) {
    delay(100);
  }

  if (senderNamePath != nullptr && SD.exists(senderNamePath)) {
    playWavScaled(senderNamePath, 0.85f);
  }
}

/**
 * Plays the "no new messages" prompt, with a tone fallback.
 */
void playNoNewMessagesPrompt() {
  if (!playOptionalPrompt(PROMPT_NO_NEW_MESSAGES)) {
    playFeedbackToneError();
  }
}

// ============================================================
// QUEUE / ARCHIVE / MOCK SENDING METHODS
// ============================================================

/**
 * Returns true if unread messages are waiting in the queue folder.
 */
size_t refreshQueueDepth(bool force) {
  unsigned long now = millis();
  bool shouldSanitize = force || lastQueueSanitizeMs == 0 || now - lastQueueSanitizeMs >= QUEUE_SANITIZE_INTERVAL_MS;
  bool shouldRefresh = force || lastQueueStateRefreshMs == 0 || now - lastQueueStateRefreshMs >= QUEUE_STATE_REFRESH_MS;

  if (shouldSanitize) {
    pruneInvalidQueuedMessages(false);
    lastQueueSanitizeMs = now;
  }

  if (shouldRefresh || shouldSanitize) {
    cachedQueueDepth = countWavsInDir(MESSAGE_QUEUE_DIR);
    lastQueueStateRefreshMs = now;
  }

  return cachedQueueDepth;
}

/**
 * Returns true if unread messages are waiting in the queue folder.
 */
bool hasQueuedMessages() {
  return refreshQueueDepth(false) > 0;
}

/**
 * Returns true when a WAV file has a minimally valid RIFF/WAVE header and body length.
 */
bool isValidWavFile(const String& wavPath) {
  if (!SD.exists(wavPath)) {
    return false;
  }

  File wav = SD.open(wavPath.c_str(), FILE_READ);
  if (!wav) return false;

  size_t wavSize = wav.size();
  if (wavSize <= 44) {
    wav.close();
    return false;
  }

  uint8_t header[12];
  size_t read = wav.read(header, sizeof(header));
  wav.close();
  if (read != sizeof(header)) return false;

  bool riffOk = memcmp(header, "RIFF", 4) == 0;
  bool waveOk = memcmp(header + 8, "WAVE", 4) == 0;
  return riffOk && waveOk;
}

/**
 * Returns true when a queued message has both metadata and a minimally valid WAV header/body.
 */
bool isValidQueuedMessage(const String& wavPath, const String& metaPath) {
  if (!SD.exists(metaPath) || !isValidWavFile(wavPath)) {
    return false;
  }

  return true;
}

/**
 * Logs the first 12 bytes of a file for header debugging.
 */
void logFileHeaderBytes(const String& path, const char* label) {
  if (!SD.exists(path)) {
    Serial.print("Header debug missing file: ");
    Serial.println(path);
    return;
  }

  File f = SD.open(path.c_str(), FILE_READ);
  if (!f) {
    Serial.print("Header debug open failed: ");
    Serial.println(path);
    return;
  }

  uint8_t header[12] = {0};
  size_t read = f.read(header, sizeof(header));
  f.close();

  Serial.print("Header debug ");
  Serial.print(label);
  Serial.print(" bytes=");
  for (size_t i = 0; i < read; ++i) {
    if (i > 0) Serial.print(' ');
    if (header[i] < 16) Serial.print('0');
    Serial.print(header[i], HEX);
  }
  Serial.println();
}

/**
 * Deletes corrupt or orphaned queued messages so they cannot block inbox playback.
 */
size_t pruneInvalidQueuedMessages(bool verbose) {
  File dir = SD.open(MESSAGE_QUEUE_DIR);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return 0;
  }

  size_t removed = 0;
  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String fullName = entry.name();
      String shortName = fullName.substring(fullName.lastIndexOf('/') + 1);
      if (shortName.endsWith(".wav")) {
        String baseName = shortName.substring(0, shortName.lastIndexOf('.'));
        String wavPath = String(MESSAGE_QUEUE_DIR) + "/" + shortName;
        String metaPath = String(MESSAGE_QUEUE_DIR) + "/" + baseName + ".txt";

        if (!isValidQueuedMessage(wavPath, metaPath)) {
          if (verbose) {
            Serial.print("Deleting invalid queued message: ");
            Serial.println(wavPath);
          }
          deleteFileIfExists(wavPath.c_str());
          deleteFileIfExists(metaPath.c_str());
          removed++;
        }
      } else if (shortName.endsWith(".txt")) {
        String baseName = shortName.substring(0, shortName.lastIndexOf('.'));
        String wavPath = String(MESSAGE_QUEUE_DIR) + "/" + baseName + ".wav";
        String metaPath = String(MESSAGE_QUEUE_DIR) + "/" + shortName;
        if (!SD.exists(wavPath) || !isValidQueuedMessage(wavPath, metaPath)) {
          if (verbose) {
            Serial.print("Deleting orphan queued metadata: ");
            Serial.println(metaPath);
          }
          deleteFileIfExists(metaPath.c_str());
          if (SD.exists(wavPath)) {
            deleteFileIfExists(wavPath.c_str());
          }
          removed++;
        }
      }
    }
    entry.close();
    entry = dir.openNextFile();
  }

  dir.close();
  return removed;
}

/**
 * Plays the oldest queued message, then archives it.
 */
bool playNextQueuedMessage() {
  size_t removed = pruneInvalidQueuedMessages(true);
  if (removed > 0) {
    refreshNextMessageSequence();
    lastQueueSanitizeMs = millis();
  }
  refreshQueueDepth(true);

  String wavPath;
  String baseName;
  if (!findOldestWavInDir(MESSAGE_QUEUE_DIR, wavPath, baseName)) {
    Serial.println("No queued messages.");
    playNoNewMessagesPrompt();
    return false;
  }

  String metaPath = String(MESSAGE_QUEUE_DIR) + "/" + baseName + ".txt";
  String fromId = readMetadataValue(metaPath.c_str(), "FROM_ID");
  const char* senderNamePath = nullptr;

  if (fromId == "ben_slate") {
    senderNamePath = NAME_BEN;
  } else if (fromId == "pete_sapphire") {
    senderNamePath = NAME_PETER;
  } else if (fromId == "bonnie_emerald") {
    senderNamePath = NAME_BONNIE;
  } else if (fromId == "sophie_scarlet") {
    senderNamePath = NAME_SOPHIE;
  }

  Serial.print("Playing queued message: ");
  Serial.println(wavPath);

  playMessageFromPrompt(senderNamePath);
  bool ok = playWav(wavPath);
  if (!ok) {
    playFeedbackToneError();
    return false;
  }

  lastPlayedWavPath = wavPath;
  lastPlayedMetaPath = metaPath;

  if (!archiveMessage(wavPath, metaPath)) {
    Serial.println("Failed to archive played message.");
    playFeedbackToneError();
    return false;
  }

  String deliveryId = readMetadataValue(lastPlayedMetaPath.c_str(), "DELIVERY_ID");
  if (deliveryId.length() > 0) {
    acknowledgeDelivery(deliveryId, "Played");
  }

  enforceSavedMessageLimit();
  playFeedbackToneOK();
  return true;
}

/**
 * Replays the most recently played message from its last known path.
 */
bool replayLastPlayedMessage() {
  if (lastPlayedWavPath.length() == 0 || !SD.exists(lastPlayedWavPath)) {
    Serial.println("No last played message to replay.");
    playFeedbackToneError();
    return false;
  }

  Serial.print("Replaying: ");
  Serial.println(lastPlayedWavPath);
  return playWav(lastPlayedWavPath);
}

/**
 * Moves a played message from queue into the saved archive.
 */
bool archiveMessage(const String& wavPath, const String& metaPath) {
  String fileName = wavPath.substring(wavPath.lastIndexOf('/') + 1);
  String baseName = fileName.substring(0, fileName.lastIndexOf('.'));

  String dstWav = String(SAVED_MESSAGES_DIR) + "/" + fileName;
  String dstMeta = String(SAVED_MESSAGES_DIR) + "/" + baseName + ".txt";

  bool okWav = moveFile(wavPath.c_str(), dstWav.c_str());
  bool okMeta = true;

  if (SD.exists(metaPath)) {
    okMeta = moveFile(metaPath.c_str(), dstMeta.c_str());
  }

  if (okWav && !okMeta && SD.exists(metaPath)) {
    Serial.println("Archive warning: deleting orphan queue metadata after failed move.");
    deleteFileIfExists(metaPath.c_str());
  }

  if (okWav) {
    lastPlayedWavPath = dstWav;
    lastPlayedMetaPath = okMeta ? dstMeta : "";
  }

  refreshQueueDepth(true);
  return okWav && okMeta;
}

/**
 * Enforces a rolling limit on saved archived messages.
 */
bool enforceSavedMessageLimit() {
  while (countWavsInDir(SAVED_MESSAGES_DIR) > MAX_SAVED_MESSAGES) {
    String oldestWav;
    String oldestBase;
    if (!findOldestWavInDir(SAVED_MESSAGES_DIR, oldestWav, oldestBase)) break;

    String oldestMeta = String(SAVED_MESSAGES_DIR) + "/" + oldestBase + ".txt";
    SD.remove(oldestWav);
    if (SD.exists(oldestMeta)) SD.remove(oldestMeta);

    if (lastPlayedWavPath == oldestWav) {
      lastPlayedWavPath = "";
      lastPlayedMetaPath = "";
    }
  }
  return true;
}

/**
 * Mock-sends the current temp message by moving it into the mock_sent folder.
 */
bool mockSendCurrentTempMessage() {
  if (!SD.exists(TEMP_OUTGOING_WAV) || !SD.exists(TEMP_OUTGOING_META)) {
    Serial.println("Nothing to send.");
    playFeedbackToneError();
    return false;
  }

  String messageId = readMetadataValue(TEMP_OUTGOING_META, "MESSAGE_ID");
  if (messageId.length() == 0) {
    messageId = buildMessageBaseName(nextMessageSequence);
  }

  uint32_t seq = extractSequenceFromName(messageId + ".wav");
  if (seq == 0) seq = nextMessageSequence;

  String dstWav = buildMockSentWavPath(seq);
  String dstMeta = buildMockSentMetaPath(seq);

  bool okWav = moveFile(TEMP_OUTGOING_WAV, dstWav.c_str());
  bool okMeta = moveFile(TEMP_OUTGOING_META, dstMeta.c_str());

  if (okWav && okMeta) {
    Serial.print("Mock sent to ");
    Serial.println(readMetadataValue(dstMeta.c_str(), "TO_NAME"));
    nextMessageSequence = max(nextMessageSequence, seq + 1);
    playFeedbackToneOK();
    return true;
  }

  Serial.println("Mock send failed.");
  playFeedbackToneError();
  return false;
}

// ============================================================
// LED / MODE / STATE METHODS
// ============================================================

/**
 * Initializes button and LED GPIO.
 */
void setupButtonsAndLeds() {
  pinMode(PIN_BTN_B1_GREEN, INPUT_PULLUP);
  pinMode(PIN_BTN_B2_BLACK, INPUT_PULLUP);
  pinMode(PIN_BTN_B3_WHITE, INPUT_PULLUP);
  pinMode(PIN_BTN_B4_RED, INPUT_PULLUP);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  setLeds(false, false);
}

/**
 * Sets the green and red LEDs directly.
 */
void setLeds(bool greenOn, bool redOn) {
  digitalWrite(PIN_LED_GREEN, greenOn ? HIGH : LOW);
  digitalWrite(PIN_LED_RED, redOn ? HIGH : LOW);
}

/**
 * Briefly flashes both LEDs together to signal an upcoming network transfer.
 */
void flashTransferLeds(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < pulses; ++i) {
    setLeds(true, true);
    delay(onMs);
    setLeds(false, false);
    if (i + 1 < pulses) {
      delay(offMs);
    }
  }
}

/**
 * Updates LED outputs based on simple readiness / busy / transfer states.
 */
void updateLedState() {
  if (cloudDownloadActive || cloudUploadActive) {
    unsigned long now = millis();
    if (now - lastBusyBlinkMs >= BUSY_LED_INTERVAL_MS) {
      lastBusyBlinkMs = now;
      busyBlinkState = !busyBlinkState;
    }
    setLeds(busyBlinkState, busyBlinkState);
    return;
  }

  if (recordingActive || currentActionState == ActionState::Sending || currentActionState == ActionState::Playing) {
    setLeds(false, true);
    return;
  }

  if (hasQueuedMessages()) {
    unsigned long now = millis();
    if (now - lastBlinkMs >= LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = now;
      queueBlinkState = !queueBlinkState;
    }
    setLeds(queueBlinkState, false);
    return;
  }

  setLeds(true, false);
}

/**
 * Sets the current device mode and updates indicators.
 */
void setMode(DeviceMode mode) {
  currentMode = mode;
  resetIdleTimer();

  if (mode == DeviceMode::Receiving) {
    Serial.println("Mode -> Receiving");
    playFeedbackToneModeReceive();
    sendToPrefixArmed = true;
  } else {
    Serial.println("Mode -> Sending");
    playFeedbackToneModeSend();
    sendToPrefixArmed = true;
  }

  updateLedState();
}

/**
 * Toggles between receiving and sending modes.
 */
void toggleMode() {
  if (currentMode == DeviceMode::Receiving) {
    setMode(DeviceMode::Sending);
  } else {
    setMode(DeviceMode::Receiving);
  }
}

/**
 * Resets the inactivity timer.
 */
void resetIdleTimer() {
  lastUserActivityMs = millis();
  userActive = true;
  online = false;
}

/**
 * Refreshes user activity state and background-sync eligibility.
 */
void updateInteractionState() {
  if (recordingActive || currentActionState != ActionState::Idle) {
    userActive = true;
    online = false;
    return;
  }

  if (b4PendingSingleClick) {
    userActive = true;
    online = false;
    return;
  }

  if (millis() - lastUserActivityMs < USER_ACTIVE_NETWORK_GRACE_MS) {
    userActive = true;
    online = false;
    return;
  }

  userActive = false;
  online = true;
}

/**
 * Returns to receiving mode after sending mode inactivity.
 */
void updateIdleTimeout() {
  return;
}

/**
 * Returns the current playback volume multiplier.
 */
float getCurrentVolumeMultiplier() {
  return VOLUME_LEVELS[currentVolumeIndex];
}

/**
 * Cycles through five preset playback volume levels.
 */
void cycleVolumeLevel() {
  currentVolumeIndex = (currentVolumeIndex + 1) % 5;
  Serial.print("Volume preset -> ");
  Serial.println(currentVolumeIndex + 1);
  playFeedbackToneVolume();
}

/**
 * Returns true when a temp outgoing draft exists.
 */
bool hasTempDraft() {
  return SD.exists(TEMP_OUTGOING_WAV) && SD.exists(TEMP_OUTGOING_META);
}

/**
 * Deletes the current temp outgoing draft explicitly.
 */
bool discardTempDraft(bool playTone) {
  bool hadDraft = SD.exists(TEMP_OUTGOING_WAV) || SD.exists(TEMP_OUTGOING_META);
  deleteFileIfExists(TEMP_OUTGOING_WAV);
  deleteFileIfExists(TEMP_OUTGOING_META);
  if (playTone && hadDraft) {
    playFeedbackToneOK();
  }
  return hadDraft;
}

/**
 * Starts a direct-send recording using whichever recipient buttons are held right now.
 */
bool beginDirectMessageRecording() {
  if (recordingActive || currentActionState != ActionState::Idle) {
    return false;
  }

  uint8_t recipientMask = getHeldSendButtonMask();
  if (recipientMask == 0) {
    return false;
  }

  activeRecipientMask = recipientMask;
  discardTempDraft(false);
  playRecordStartBeep();
  delay(30);

  if (!beginRecordingToTemp()) {
    activeRecipientMask = 0;
    playFeedbackToneError();
    return false;
  }

  Serial.print("Direct recording started, recipientMask=");
  Serial.println(activeRecipientMask);
  return true;
}

/**
 * Finalizes the direct-send recording and immediately uploads it to the cloud.
 */
bool finalizeDirectMessageRecording() {
  if (!recordingActive || activeRecipientMask == 0) {
    return false;
  }

  if (!finishRecordingToTemp()) {
    activeRecipientMask = 0;
    playFeedbackToneError();
    return false;
  }

  playRecipientRoutingBeepSequence(activeRecipientMask);
  delay(40);

  bool sent = cloudSendCurrentTempMessage();
  activeRecipientMask = 0;
  return sent;
}

// ============================================================
// BUTTON METHODS
// ============================================================

/**
 * Initializes a button state object.
 */
void initButton(Button& btn, int pin) {
  btn.pin = pin;
  btn.stablePressed = false;
  btn.lastRawPressed = false;
  btn.longPressFired = false;
  btn.waitingForDoubleClick = false;
  btn.lastDebounceMs = 0;
  btn.pressedAtMs = 0;
  btn.releasedAtMs = 0;
  btn.lastClickMs = 0;
}

/**
 * Updates one button and returns a high-level button event.
 */
ButtonEvent updateButton(Button& btn) {
  unsigned long now = millis();
  bool rawPressed = (digitalRead(btn.pin) == LOW);

  if (rawPressed != btn.lastRawPressed) {
    btn.lastDebounceMs = now;
    btn.lastRawPressed = rawPressed;
  }

  if ((now - btn.lastDebounceMs) < BUTTON_DEBOUNCE_MS) {
    return ButtonEvent::None;
  }

  if (rawPressed != btn.stablePressed) {
    btn.stablePressed = rawPressed;

    if (btn.stablePressed) {
      btn.pressedAtMs = now;
      btn.longPressFired = false;
      return ButtonEvent::Press;
    } else {
      btn.releasedAtMs = now;

      if (btn.longPressFired) {
        btn.longPressFired = false;
        return ButtonEvent::Release;
      }

      return ButtonEvent::Click;
    }
  }

  if (btn.stablePressed && !btn.longPressFired && (now - btn.pressedAtMs >= BUTTON_LONG_PRESS_MS)) {
    btn.longPressFired = true;
    btn.waitingForDoubleClick = false;
    return ButtonEvent::LongPress;
  }

  return ButtonEvent::None;
}

/**
 * Polls all buttons and routes any events to their handlers.
 */
void pollButtons() {
  ButtonEvent e1 = updateButton(buttonB1);
  ButtonEvent e2 = updateButton(buttonB2);
  ButtonEvent e3 = updateButton(buttonB3);
  ButtonEvent e4 = updateButton(buttonB4);

  if (e1 != ButtonEvent::None) handleButtonEventB1(e1);
  if (e2 != ButtonEvent::None) handleButtonEventB2(e2);
  if (e3 != ButtonEvent::None) handleButtonEventB3(e3);
  if (e4 != ButtonEvent::None) handleButtonEventB4(e4);
}

/**
 * Executes a pending single-click B4 action once the double-click window expires.
 */
void servicePendingB4Click() {
  if (!b4PendingSingleClick) return;
  if (millis() - b4PendingClickMs < BUTTON_DOUBLE_CLICK_MS) return;
  if (recordingActive || currentActionState != ActionState::Idle) return;

  b4PendingSingleClick = false;
  playNextQueuedMessage();
}

/**
 * Handles direct-send behavior for one of the three recipient buttons.
 */
static void handleRecipientButtonEvent(uint8_t buttonIndex, ButtonEvent event) {
  if (currentActionState == ActionState::Playing || currentActionState == ActionState::Sending) {
    return;
  }

  if (event == ButtonEvent::Press) {
    resetIdleTimer();
    return;
  }

  if (event == ButtonEvent::Click) {
    resetIdleTimer();
    playContactName(getButtonRecipient(buttonIndex));
    return;
  }

  if (event == ButtonEvent::LongPress) {
    resetIdleTimer();
    beginDirectMessageRecording();
    return;
  }

  if (event == ButtonEvent::Release) {
    resetIdleTimer();
    if (recordingActive && activeRecipientMask != 0 && (getHeldSendButtonMask() & activeRecipientMask) == 0) {
      finalizeDirectMessageRecording();
    }
  }
}

/**
 * Handles B1 direct-send behavior.
 */
void handleButtonEventB1(ButtonEvent event) {
  handleRecipientButtonEvent(0, event);
}

/**
 * Handles B2 direct-send behavior.
 */
void handleButtonEventB2(ButtonEvent event) {
  handleRecipientButtonEvent(1, event);
}

/**
 * Handles B3 direct-send behavior.
 */
void handleButtonEventB3(ButtonEvent event) {
  handleRecipientButtonEvent(2, event);
}

/**
 * Handles B4 queue playback and replay behavior.
 */
void handleButtonEventB4(ButtonEvent event) {
  if (event != ButtonEvent::Click) return;

  resetIdleTimer();

  if (recordingActive || currentActionState == ActionState::Sending) {
    playFeedbackToneError();
    return;
  }

  if (b4PendingSingleClick && millis() - b4PendingClickMs <= BUTTON_DOUBLE_CLICK_MS) {
    b4PendingSingleClick = false;
    replayLastPlayedMessage();
  } else {
    b4PendingSingleClick = true;
    b4PendingClickMs = millis();
  }
}

// ============================================================
// SETUP / LOOP HELPERS
// ============================================================

void handleSerialDebugCommands() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();

  if (command == "mic") {
    if (recordingActive || currentActionState == ActionState::Playing) {
      Serial.println("Mic self-test ignored: device busy.");
      return;
    }

    if (!sdReady) {
      Serial.println("Mic self-test ignored: SD not ready.");
      return;
    }

    Serial.println("Starting 5s microphone self-test. Speak into the mic now.");
    currentActionState = ActionState::Recording;
    bool ok = recordFixedDurationToWav(5, TEMP_MIC_SELF_TEST_WAV);
    currentActionState = ActionState::Idle;

    if (ok) {
      Serial.println("Mic self-test complete. Type 'playmic' to hear it.");
    } else {
      Serial.println("Mic self-test failed.");
    }
  } else if (command == "playmic") {
    if (!SD.exists(TEMP_MIC_SELF_TEST_WAV)) {
      Serial.println("No mic self-test WAV found.");
      return;
    }

    Serial.println("Playing mic self-test WAV...");
    playWav(TEMP_MIC_SELF_TEST_WAV);
  } else if (command == "micinfo") {
    Serial.print("DirectButtons recordingActive=");
    Serial.print(recordingActive ? "true" : "false");
    Serial.print(" wifiConnected=");
    Serial.print(wifiConnected ? "true" : "false");
    Serial.print(" currentActionState=");
    Serial.print((int)currentActionState);
    Serial.print(" activeRecipientMask=");
    Serial.println(activeRecipientMask);
  }
}

/**
 * Runs background service logic that should happen every loop iteration.
 */
void serviceBackgroundTasks() {
  handleSerialDebugCommands();
  updateInteractionState();
  servicePendingB4Click();

  if (recordingActive) {
    if (!continueRecordingToTemp()) {
      Serial.println("Recording aborted due to microphone read failure.");
      cancelRecordingToTemp();
      playFeedbackToneError();
    }
    updateLedState();
    return;
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (!wifiConnected) {
    connectToWiFiIfNeeded();
  }

  if (wifiConnected) {
    syncClockIfNeeded();
    if (shouldRunBackgroundNetworkTasks()) {
      sendDeviceHeartbeat(false);
      pollCloudInboxIfDue();
    }
  }

  updateIdleTimeout();
  updateLedState();
}

/**
 * Prints remaining implementation notes for the user.
 */
void printFinalTodo() {
  Serial.println();
  Serial.println("TODO BEFORE FINAL PRODUCTION BUILD:");
  Serial.println("1. Confirm required Arduino libraries are installed: ArduinoJson.");
  Serial.println("2. Replace insecure TLS mode with root certificates for production.");
  Serial.println("3. Validate the Azure Function base URL in /fritzphone/system/azure_services.json.");
  Serial.println("4. Validate device heartbeat visibility in the Function backend.");
  Serial.println("5. Add richer retry/backoff and offline send queueing if needed.");
  Serial.println("6. Optionally load more settings from /device_data/settings.txt at boot.");
  Serial.println("Debug commands: mic, playmic, micinfo");
  Serial.println();
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  setupButtonsAndLeds();
  initButton(buttonB1, PIN_BTN_B1_GREEN);
  initButton(buttonB2, PIN_BTN_B2_BLACK);
  initButton(buttonB3, PIN_BTN_B3_WHITE);
  initButton(buttonB4, PIN_BTN_B4_RED);

  sdReady = initSD();
  if (sdReady) {
    loadWifiConfig();
    loadIotConfig();
    loadAzureServicesConfig();
  }

  WiFi.mode(WIFI_MODE_APSTA);
  delay(100);
  resolveDeviceIdentity();

  if (sdReady) {
    ensureFilesystemLayout();
    pruneInvalidQueuedMessages(true);
    refreshQueueDepth(true);
    refreshNextMessageSequence();
  }

  printBootHeader();

  currentRecipientIndex = 0;
  currentTempRecipientId = getCurrentRecipient().deviceId;
  currentTempRecipientName = getCurrentRecipient().displayName;
  currentTempRecipientNameWav = getCurrentRecipient().spokenNamePath;

  if (sdReady) {
    playStartupAnnouncement();
  }

  connectToWiFiIfNeeded();
  if (wifiConnected) {
    syncClockIfNeeded();
  }

  resetIdleTimer();
  updateLedState();
  printFinalTodo();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  pollButtons();

  unsigned long now = millis();
  if (recordingActive || now - lastBackgroundServiceMs >= BACKGROUND_SERVICE_INTERVAL_MS) {
    lastBackgroundServiceMs = now;
    serviceBackgroundTasks();
  }

  delay(2);
}
