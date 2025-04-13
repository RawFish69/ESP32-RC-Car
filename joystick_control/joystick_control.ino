/*
Joystick Control for ESP32 with ESP-NOW
Author: RawFish69
*/

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

#define JOYSTICK_X_PIN 2 
#define JOYSTICK_Y_PIN 1 
#define EMERGENCY_STOP_PIN 0 

int joyCenterX = 0;
int joyCenterY = 0;

// ESP-NOW peer configurations
uint8_t DEFAULT_DRIVE_MAC[6] = {0x68, 0x67, 0x25, 0xB1, 0xBC, 0x8C};
uint8_t DRIVE_BOARD_MAC[6];

bool firstSetup = true;

unsigned long lastJoystickCommandTime = 0;
const unsigned long commandInterval = 200; // in ms

float lastSpeed = 0, lastTurn = 0;
String lastDir = "";
String lastDelivery = "Sending...";
static unsigned long lastSendTime = 0;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastDelivery = "Success";
  } else {
    lastDelivery = "Fail";
  }
}

void sendEspNowMessage(const String &msg) {
  unsigned long now = millis();
  if (now - lastSendTime < 20) {
    delay(20 - (now - lastSendTime));
  }
  lastSendTime = millis();
  esp_err_t ret = esp_now_send(DRIVE_BOARD_MAC, (const uint8_t*)msg.c_str(), msg.length() + 1);
  if (ret != ESP_OK) {
    lastDelivery = "SendFail(" + String(ret) + ")";
  }
}

void showSimpleCommand(float spd, const String &dir, float trn) {
  lastSpeed = spd;
  lastDir = dir;
  lastTurn = trn;
  lastDelivery = "Sending...";
}

void initEspNowAndPeer() {
  if (!firstSetup) {
    esp_now_del_peer(DRIVE_BOARD_MAC);
    esp_now_deinit();
  }
  WiFi.mode(WIFI_STA);
  memcpy(DRIVE_BOARD_MAC, DEFAULT_DRIVE_MAC, 6);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNOW init fail");
    delay(2000);
    ESP.restart();
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, DRIVE_BOARD_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer Add FAIL");
  }
  firstSetup = false;
}

// Calibration function: average joystick readings over 3 seconds
void calibrateJoystick() {
  Serial.println("Calibrating joystick... Please keep joystick centered.");
  unsigned long startTime = millis();
  long sumX = 0, sumY = 0;
  int samples = 0;
  while (millis() - startTime < 3000) {
    sumX += analogRead(JOYSTICK_X_PIN);
    sumY += analogRead(JOYSTICK_Y_PIN);
    samples++;
    delay(10);
  }
  joyCenterX = sumX / samples;
  joyCenterY = sumY / samples;
  Serial.print("Calibration done. Center X: ");
  Serial.print(joyCenterX);
  Serial.print(", Center Y: ");
  Serial.println(joyCenterY);
}

// Add helper function to map joystick raw values to -100 to 100 range based on the calibrated center
int mapJoystickValue(int raw, int center) {
  if (raw >= center)
    return map(raw, center, 4095, 0, 100);
  else
    return map(raw, 0, center, -100, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP); 
  calibrateJoystick();
  initEspNowAndPeer();
  Serial.println("Joystick Control Setup complete");
}

void loop() {
  if(digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    showSimpleCommand(0, "stop", 0);
    String query = "mode=default&speed=0&forwardBackward=stop&turnRate=0";
    sendEspNowMessage(query);
    Serial.println("Emergency Stop Activated");
    delay(10);
    return;  
  }
  
  if (millis() - lastJoystickCommandTime >= commandInterval) {
    lastJoystickCommandTime = millis();
    int rawX = analogRead(JOYSTICK_X_PIN);
    int rawY = analogRead(JOYSTICK_Y_PIN);
    int mappedX = mapJoystickValue(rawX, joyCenterX);
    int mappedY = mapJoystickValue(rawY, joyCenterY);
    
    float speed = abs(mappedY);
    String direction = (mappedY >= 0) ? "forward" : "backward";
    float turn = -mappedX;  // Inverted to correct left/right flipping
    showSimpleCommand(speed, direction, turn);
    String query = "mode=default&speed=" + String(speed, 0)
                 + "&forwardBackward=" + direction
                 + "&turnRate=" + String(turn, 0);
    sendEspNowMessage(query);
    
    Serial.print("X: ");
    Serial.print(rawX);
    Serial.print(" (");
    Serial.print(mappedX);
    Serial.print("%), ");
    Serial.print("Y: ");
    Serial.print(rawY);
    Serial.print(" (");
    Serial.print(mappedY);
    Serial.println("%)");
  }
  delay(100);
}