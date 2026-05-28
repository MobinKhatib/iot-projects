#include <WiFi.h>
#include <esp_now.h>

#define PIR_PIN 34 
#define LDR_PIN 32 
// Destination MAC address (Broadcast as per spec)
uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

void setup() {

  Serial.begin(115200);
  // Set PIR sensor pin as input
  pinMode(PIR_PIN, INPUT);
  // Initialize WiFi in Station mode for ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Define and register the peer information
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
}

void loop() {
  //start timing sensing sensor
  unsigned long t_start_sensing = micros();
  // Read digital signal from PIR and analog value from LDR
  int motion = digitalRead(PIR_PIN);
  int analogValue = analogRead(LDR_PIN);

  // Parameters of the LDR model
  const float GAMMA = 0.7;
  const float RL10 = 50;
  // Convert ADC value to voltage (ESP32: 0–4095, 3.3V reference)
  float voltage = analogValue / 4095.0 * 3.3;
  // Calculate LDR resistance using the voltage divider formula (2kΩ resistor)
  float resistance = 2000 * voltage / (3.3 - voltage);
  // Convert resistance to light intensity (lux)
  float light = pow(RL10 * 1000 * pow(10, GAMMA) / resistance, (1 / GAMMA));
  // long duration sensing
  unsigned long duration_sensing = micros() - t_start_sensing;
  // timing sending part
  unsigned long t_start_tx = micros();
  String message;
  // Format the message string based on motion detection
  if(motion == HIGH){
    message = "MOTION_DETECTED-LUMINOSITY:" + String(light);
  }
  else{
    message = "MOTION_NOT_DETECTED-LUMINOSITY:" + String(light);
  }
  // Send the formatted string via ESP-NOW protocol
  esp_now_send(broadcastAddress, (uint8_t*)message.c_str(), message.length());
  //End timing and printing
  unsigned long t_end_tx = micros();
  unsigned long duration_tx = t_end_tx - t_start_tx;

  // Print time needed for enegy estimation
  Serial.print("Sensing Time (us): "); Serial.println(duration_sensing);
  Serial.print("TX Time (us): "); Serial.println(duration_tx);
  Serial.println(message);
  // Deep Sleep implementation to reduce energy consumption
  Serial.println("Going to deep sleep...");
  // X seconds sleep (Example: 10s). You must calculate X using your Person Code 
  esp_sleep_enable_timer_wakeup(4 * 1000000); // 4 seconds
  esp_deep_sleep_start();
}