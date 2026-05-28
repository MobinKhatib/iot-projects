#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <math.h>

// GPIO34 carries the PIR output and is also used as the EXT0 wake source.
constexpr int PIR_PIN = 34;
constexpr gpio_num_t PIR_WAKE_PIN = GPIO_NUM_34;
// The LDR analog output is read through ADC1 to avoid WiFi/ADC2 conflicts.
constexpr int LDR_PIN = 32;
// Timeout for the ESP-NOW send callback before the transmission is considered failed.
constexpr uint32_t kSendTimeoutMs = 200;
// Bright/dark classification threshold used to set a compact state flag in the packet.
constexpr uint16_t kBrightThresholdLux = 1000;

// Broadcast mode keeps the sender independent from a fixed receiver MAC.
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum PacketFlags : uint8_t {
  FLAG_LIGHT_BRIGHT = 1 << 0,
  FLAG_WAKE_FROM_PIR = 1 << 1,
  FLAG_LOW_TX_POWER = 1 << 2,
};

// Compact 4-byte payload used instead of the original long ASCII message.
struct __attribute__((packed)) SensorPacket {
  uint8_t motion;
  uint16_t luminosity;
  uint8_t stateFlags;
};

// Updated from the ESP-NOW callback so the main flow can wait for completion.
volatile bool sendComplete = false;
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
// Keep the last reported state across deep sleep so repeated identical wake events can be skipped.
RTC_DATA_ATTR bool hasLastReportedState = false;
RTC_DATA_ATTR uint8_t lastReportedMotion = 0;
RTC_DATA_ATTR uint8_t lastReportedLightState = 0;

// Uses the same empirical LDR model as the report to convert ADC samples to lux-like values.
float computeLightLux(int analogValue) {
  if (analogValue <= 0) {
    return 0.0f;
  }

  constexpr float gamma = 0.7f;
  constexpr float rl10 = 50.0f;
  float voltage = analogValue / 4095.0f * 3.3f;

  if (voltage >= 3.299f) {
    voltage = 3.299f;
  }

  float resistance = 2000.0f * voltage / (3.3f - voltage);
  if (resistance <= 0.0f) {
    return 0.0f;
  }

  return powf(rl10 * 1000.0f * powf(10.0f, gamma) / resistance, 1.0f / gamma);
}

// Clamp the floating-point light estimate into the two-byte packet field.
uint16_t encodeLuminosity(float lux) {
  if (!isfinite(lux) || lux <= 0.0f) {
    return 0;
  }

  if (lux >= 65535.0f) {
    return 65535;
  }

  return static_cast<uint16_t>(lroundf(lux));
}

// Human-readable wake causes make the serial trace easier to interpret during testing.
const char* wakeCauseToString(esp_sleep_wakeup_cause_t wakeCause) {
  switch (wakeCause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      return "PIR interrupt wake";
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      return "power-on reset";
    default:
      return "other wake";
  }
}

void onDataSent(const wifi_tx_info_t*, esp_now_send_status_t status) {
  sendComplete = true;
  lastSendStatus = status;
}

// Shut the radio stack down before sleeping so the ESP32 returns to its lowest-power state.
void shutdownRadio() {
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
}

// This design executes one wake-report-sleep pass in setup(), then never uses loop().
void enterDeepSleep() {
  shutdownRadio();

  // EXT0 is level-triggered; arm sleep only after the PIR output drops low,
  // otherwise the ESP32 can wake immediately again while the sensor is latched.
  while (digitalRead(static_cast<int>(PIR_PIN)) == HIGH) {
    delay(10);
  }

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  esp_sleep_enable_ext0_wakeup(PIR_WAKE_PIN, 1);
  Serial.println("Entering deep sleep with EXT0 wake on GPIO34...");
  esp_deep_sleep_start();
}

// Radio setup is deferred until just before transmission to avoid unnecessary active time.
bool initEspNow() {
  WiFi.mode(WIFI_STA);
  // Match the low-power radio configuration discussed in the report.
  WiFi.setTxPower(WIFI_POWER_2dBm);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }

  // Use the asynchronous send callback instead of a fixed post-send delay.
  esp_now_register_send_cb(onDataSent);

  // Register the broadcast peer once per wake cycle before sending the compact packet.
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error adding ESP-NOW peer");
    shutdownRadio();
    return false;
  }

  return true;
}

// Send one packet and block only until the callback confirms completion or times out.
bool sendPacketAndWait(const SensorPacket& packet, unsigned long& durationTxUs) {
  if (!initEspNow()) {
    durationTxUs = 0;
    return false;
  }

  sendComplete = false;
  lastSendStatus = ESP_NOW_SEND_FAIL;

  unsigned long tStartTx = micros();
  esp_err_t sendResult = esp_now_send(
      broadcastAddress,
      reinterpret_cast<const uint8_t*>(&packet),
      sizeof(packet));

  if (sendResult != ESP_OK) {
    Serial.printf("esp_now_send failed: %d\n", static_cast<int>(sendResult));
    durationTxUs = micros() - tStartTx;
    return false;
  }

  uint32_t waitStart = millis();
  // Sleep is entered only after the callback marks the transmission complete.
  while (!sendComplete && (millis() - waitStart) < kSendTimeoutMs) {
    delay(0);
  }
  durationTxUs = micros() - tStartTx;

  if (!sendComplete) {
    Serial.println("ESP-NOW send timed out");
    return false;
  }

  return lastSendStatus == ESP_NOW_SEND_SUCCESS;
}

void setup() {
  Serial.begin(115200);
  // All work happens once here: wake, sense, transmit, then return to deep sleep.
  pinMode(static_cast<int>(PIR_PIN), INPUT);
  pinMode(LDR_PIN, INPUT);
  delay(20);

  esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();

  // Measure the local sensing phase separately for the report timing analysis.
  unsigned long tStartSensing = micros();
  uint8_t motion = digitalRead(static_cast<int>(PIR_PIN)) == HIGH ? 1 : 0;
  int analogValue = analogRead(LDR_PIN);
  float lightLux = computeLightLux(analogValue);
  uint16_t luminosity = encodeLuminosity(lightLux);
  uint8_t lightState = luminosity >= kBrightThresholdLux ? 1 : 0;
  unsigned long durationSensing = micros() - tStartSensing;

  // Flags let the receiver interpret the 4-byte payload without a verbose text format.
  uint8_t stateFlags = FLAG_LOW_TX_POWER;
  if (lightState) {
    stateFlags |= FLAG_LIGHT_BRIGHT;
  }
  if (wakeCause == ESP_SLEEP_WAKEUP_EXT0) {
    stateFlags |= FLAG_WAKE_FROM_PIR;
  }

  unsigned long durationTx = 0;
  bool sendOk = false;
  bool stateChanged =
      !hasLastReportedState ||
      motion != lastReportedMotion ||
      lightState != lastReportedLightState;

  // In this pure PIR-wake design, repeated "motion detected" wakes are suppressed
  // unless the reported motion/light state differs from the last transmitted one.
  if (stateChanged) {
    // Build the compact payload only when a fresh state must be reported.
    SensorPacket packet = {motion, luminosity, stateFlags};
    sendOk = sendPacketAndWait(packet, durationTx);

    if (sendOk) {
      hasLastReportedState = true;
      lastReportedMotion = motion;
      lastReportedLightState = lightState;
    }

    Serial.printf(
        "Wake cause: %s\n",
        wakeCauseToString(wakeCause));
    Serial.printf(
        "Motion=%u, luminosity=%u lux, payload=%u bytes, flags=0x%02X\n",
        motion,
        luminosity,
        static_cast<unsigned>(sizeof(packet)),
        packet.stateFlags);
    Serial.printf("TX power set to 2 dBm\n");
    Serial.printf("Packet send %s\n", sendOk ? "succeeded" : "failed");
  } else {
    Serial.printf("Wake cause: %s\n", wakeCauseToString(wakeCause));
    Serial.printf(
        "Motion=%u, luminosity=%u lux, state unchanged, transmission skipped\n",
        motion,
        luminosity);
  }
  Serial.print("Sensing Time (us): ");
  Serial.println(durationSensing);
  Serial.print("TX Time (us): ");
  Serial.println(durationTx);

  enterDeepSleep();
}

// loop() stays empty because setup() already completes the whole wake cycle.
void loop() {}
