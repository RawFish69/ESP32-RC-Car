#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "rgb.h"

#define LEDC_RESOLUTION_BITS 10
#define LEDC_FREQUENCY 50
#define MOTOR_MIN_PWM 300
#define MOTOR_MAX_PWM 1023

// Pin definitions changed to match c3.ino:
const int motor1In1 = 8;    // IN1 on L298
const int motor1In2 = 7;    // IN2 on L298
const int motor2In1 = 6;    // IN3 on L298
const int motor2In2 = 5;    // IN4 on L298
const int enablePin1 = 10;  // ENA on L298
const int enablePin2 = 4;   // ENB on L298

// Desired PWM and direction
int desiredLeftPWM = 0;
int desiredRightPWM = 0;
int desiredLeftDirection = 1; // 1 = forward, 0 = reverse
int desiredRightDirection = 1; // 1 = forward, 0 = reverse

// Tuning multipliers
volatile float leftTune = 1.0;
volatile float rightTune = 1.0;

bool emergencyStopActive = false;

// Control board mac address
uint8_t CONTROL_BOARD_MAC[] = {0xE4, 0xB3, 0x23, 0xD2, 0xD1, 0x5C};

// 15s no packets => EStop + reboot
static const unsigned long NO_PACKET_TIMEOUT_MS = 15000;

// LED and motion state tracking
volatile int currentSpeed = 0;
volatile float currentTurnRate = 0;
unsigned long lastLedUpdate = 0;

// ESPNOW housekeeping
bool firstPacketReceived = false;
unsigned long lastPacketMillis = 0;

// Forward declarations
void parseEspNowMessage(const String& msg);
void handleEmergencyStop();
void handleTuning(float lt, float rt);
void updateMotorSignals();

// ESPNOW callback for receiving data
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t *data, int data_len) {
  if (!firstPacketReceived) {
    firstPacketReceived = true;
  }
  lastPacketMillis = millis();
  String incoming = String((char*)data);
  Serial.print("[DriveBoard] Received: ");
  Serial.println(incoming);
  parseEspNowMessage(incoming);
}

// Parse incoming ESPNOW messages and route to appropriate handlers
void parseEspNowMessage(const String& msg) {
  if (msg.equalsIgnoreCase("emergencyStop")) {
    handleEmergencyStop();
    return;
  }
  if (msg.startsWith("tune")) {
    int ltIndex = msg.indexOf("leftTune=");
    int rtIndex = msg.indexOf("rightTune=");
    if (ltIndex > 0 && rtIndex > 0) {
      int ampPos = msg.indexOf('&', ltIndex + 9);
      float lt = msg.substring(ltIndex + 9, (ampPos < 0 ? msg.length() : ampPos)).toFloat();
      float rt = msg.substring(rtIndex + 10).toFloat();
      handleTuning(lt, rt);
    }
    return;
  }
  int spIdx = msg.indexOf("speed=");
  int fbIdx = msg.indexOf("forwardBackward=");
  int trIdx = msg.indexOf("turnRate=");
  if (spIdx >= 0 && fbIdx >= 0 && trIdx >= 0) {
    float speedPercent = msg.substring(spIdx + 6, msg.indexOf('&', spIdx + 6)).toFloat();
    int nextAmp = msg.indexOf('&', fbIdx + 16);
    String forwardBackward = msg.substring(fbIdx + 16, (nextAmp < 0 ? msg.length() : nextAmp));
    float turnRatePercent = msg.substring(trIdx + 9).toFloat();

    // Update state tracking for LED
    currentSpeed = (forwardBackward.equalsIgnoreCase("Forward")) ? speedPercent : -speedPercent;
    currentTurnRate = turnRatePercent;

    // c3.ino-like logic:
    int basePWM = (speedPercent > 0) ? map((int)speedPercent, 1, 100, MOTOR_MIN_PWM, MOTOR_MAX_PWM) : 0;
    if (basePWM < MOTOR_MIN_PWM && basePWM != 0) {
      basePWM = MOTOR_MIN_PWM;
    }
    float turnFactor = turnRatePercent / 50.0;
    if (turnFactor > 0) {
      desiredLeftPWM = basePWM;
      desiredRightPWM = basePWM * (1.0 - turnFactor);
    } else {
      desiredLeftPWM = basePWM * (1.0 + turnFactor);
      desiredRightPWM = basePWM;
    }
    desiredLeftPWM  = constrain(desiredLeftPWM,  0, MOTOR_MAX_PWM);
    desiredRightPWM = constrain(desiredRightPWM, 0, MOTOR_MAX_PWM);
    desiredLeftDirection  = (forwardBackward.equalsIgnoreCase("Forward")) ? 1 : 0;
    desiredRightDirection = (forwardBackward.equalsIgnoreCase("Forward")) ? 1 : 0;
  }
}

// Handle emergency stop by disabling all motors
void handleEmergencyStop() {
  emergencyStopActive = true;
  desiredLeftPWM = 0;
  desiredRightPWM = 0;
  currentSpeed = 0;
  currentTurnRate = 0;
  digitalWrite(motor1In1, LOW);
  digitalWrite(motor1In2, LOW);
  digitalWrite(motor2In1, LOW);
  digitalWrite(motor2In2, LOW);
  ledcWrite(enablePin1, 0);
  ledcWrite(enablePin2, 0);
  Serial.println("[DriveBoard] EMERGENCY STOP triggered!");
}

// Update motor tuning parameters
void handleTuning(float lt, float rt) {
  leftTune = constrain(lt, 0.5, 1.5);
  rightTune = constrain(rt, 0.5, 1.5);
  Serial.printf("[DriveBoard] Tuning => Left=%.2f, Right=%.2f\n", leftTune, rightTune);
}

// Update motor signals based on desired states
void updateMotorSignals() {
  if (emergencyStopActive) return;
  digitalWrite(motor1In1, (desiredLeftDirection) ? HIGH : LOW);
  digitalWrite(motor1In2, (desiredLeftDirection) ? LOW : HIGH);
  digitalWrite(motor2In1, (desiredRightDirection) ? HIGH : LOW);
  digitalWrite(motor2In2, (desiredRightDirection) ? LOW : HIGH);
  float finalLeftPWM = desiredLeftPWM * leftTune;
  float finalRightPWM = desiredRightPWM * rightTune;
  int leftOut = (int)constrain(finalLeftPWM, 0, MOTOR_MAX_PWM);
  int rightOut = (int)constrain(finalRightPWM, 0, MOTOR_MAX_PWM);
  ledcWrite(enablePin1, leftOut);
  ledcWrite(enablePin2, rightOut);
}

void setup() {
  Serial.begin(115200);
  pinMode(motor1In1, OUTPUT);
  pinMode(motor1In2, OUTPUT);
  pinMode(motor2In1, OUTPUT);
  pinMode(motor2In2, OUTPUT);
  ledcAttach(enablePin1, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(enablePin2, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  digitalWrite(motor1In1, LOW);
  digitalWrite(motor1In2, LOW);
  digitalWrite(motor2In1, LOW);
  digitalWrite(motor2In2, LOW);
  ledcWrite(enablePin1, 0);
  ledcWrite(enablePin2, 0);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  Serial.print("Drive Board STA MAC: ");
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESPNOW init fail");
    delay(2000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);

  firstPacketReceived = false;
  lastPacketMillis = millis();
  Serial.println("Drive Board ready.");
  rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
}

void loop() {
  unsigned long currentMillis = millis();
  updateMotorSignals();
  
  if (currentMillis - lastLedUpdate > LED_UPDATE_INTERVAL) {
    updateLED();
    lastLedUpdate = currentMillis;
  }
  
  if (firstPacketReceived) {
    if (millis() - lastPacketMillis >= NO_PACKET_TIMEOUT_MS) {
      Serial.println("[DriveBoard] 15s no ESPNOW => EStop + Reboot!");
      handleEmergencyStop();
      ESP.restart();
    }
  }
  delay(20);
}