/*
 * ESP32 Omni-Directional RC Car - Drive Board
 * Version: 1.0.0
 * Author: RawFish69
 * 
 * This firmware controls the motor drivers based on commands received via ESP-NOW.
 * Supports both omni-directional and simple (tank-style) control modes.
 * Features emergency stop and motor tuning capabilities.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define LEDC_RESOLUTION_BITS 10
#define LEDC_FREQUENCY 50
#define MOTOR_MIN_PWM 300
#define MOTOR_MAX_PWM 1023

static const unsigned long REBOOT_TIMEOUT_MS = 5000;

const int enablePin1 = 5;
const int motor1In1 = 6;
const int motor1In2 = 7;
const int enablePin2 = 20;
const int motor2In1 = 21;
const int motor2In2 = 3;
const int enablePin3 = 8;
const int motor3In1 = 9;
const int motor3In2 = 10;
const int enablePin4 = 2;
const int motor4In1 = 1;
const int motor4In2 = 0;

int desiredLeftPWM = 0;
int desiredRightPWM = 0;
int desiredMotor3PWM = 0;
int desiredMotor4PWM = 0;
int desiredLeftDirection = 1;
int desiredRightDirection = 1;
int desiredMotor3Direction = 1;
int desiredMotor4Direction = 1;
volatile float leftTune = 1.0;
volatile float rightTune = 1.0;
bool emergencyStopActive = false;

bool firstPacketReceived = false;
unsigned long lastPacketMillis = 0;

void parseEspNowMessage(const String& msg);
void handleEmergencyStop();
void handleTuning(float lt, float rt);
void setMotorOmni(float xVal, float yVal, float rVal);
void setMotorSimple(float speedPercent, const String& forwardBackward, float turnRatePercent);
void updateMotorSignals();

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t *data, int data_len)
{
  if (!firstPacketReceived) {
    firstPacketReceived = true;
  }
  lastPacketMillis = millis();
  String incoming = String((char*)data);
  Serial.print("[DriveBoard] Received: ");
  Serial.println(incoming);
  parseEspNowMessage(incoming);
}

void parseEspNowMessage(const String& msg)
{
  if (msg.equalsIgnoreCase("emergencyStop")) {
    handleEmergencyStop();
    return;
  }
  if (msg.startsWith("tune")) {
    int ltIndex = msg.indexOf("leftTune=");
    int rtIndex = msg.indexOf("rightTune=");
    if (ltIndex > 0 && rtIndex > 0) {
      int ampPos = msg.indexOf('&', ltIndex+9);
      float lt = msg.substring(ltIndex + 9, (ampPos<0? msg.length() : ampPos)).toFloat();
      float rt = msg.substring(rtIndex + 10).toFloat();
      handleTuning(lt, rt);
    }
    return;
  }
  if (msg.indexOf("mode=omni") >= 0) {
    int xIdx = msg.indexOf("x=");
    int yIdx = msg.indexOf("y=");
    int rIdx = msg.indexOf("rotate=");
    if (xIdx<0 || yIdx<0 || rIdx<0) return;
    float xVal = msg.substring(xIdx+2, msg.indexOf('&', xIdx+2)).toFloat();
    float yVal = msg.substring(yIdx+2, msg.indexOf('&', yIdx+2)).toFloat();
    float rVal = msg.substring(rIdx+7).toFloat();
    setMotorOmni(xVal, yVal, rVal);
    return;
  }
  if (msg.indexOf("mode=simple") >= 0) {
    int spIdx = msg.indexOf("speed=");
    int fbIdx = msg.indexOf("forwardBackward=");
    int trIdx = msg.indexOf("turnRate=");
    if (spIdx<0 || fbIdx<0 || trIdx<0) return;
    float speedVal = msg.substring(spIdx+6, msg.indexOf('&', spIdx+6)).toFloat();
    int nextAmp    = msg.indexOf('&', fbIdx + 16);
    String forwardBack = msg.substring(fbIdx+16, (nextAmp<0? msg.length() : nextAmp));
    float turnVal  = msg.substring(trIdx+9).toFloat();
    setMotorSimple(speedVal, forwardBack, turnVal);
    return;
  }
}

void handleEmergencyStop()
{
  emergencyStopActive = true;
  desiredLeftPWM      = 0;
  desiredRightPWM     = 0;
  desiredMotor3PWM    = 0;
  desiredMotor4PWM    = 0;
  digitalWrite(motor1In1, LOW);
  digitalWrite(motor1In2, LOW);
  digitalWrite(motor2In1, LOW);
  digitalWrite(motor2In2, LOW);
  digitalWrite(motor3In1, LOW);
  digitalWrite(motor3In2, LOW);
  digitalWrite(motor4In1, LOW);
  digitalWrite(motor4In2, LOW);
  ledcWrite(enablePin1, 0);
  ledcWrite(enablePin2, 0);
  ledcWrite(enablePin3, 0);
  ledcWrite(enablePin4, 0);
  Serial.println("[DriveBoard] EMERGENCY STOP triggered!");
}

void handleTuning(float lt, float rt)
{
  leftTune  = constrain(lt, 0.5, 1.5);
  rightTune = constrain(rt, 0.5, 1.5);
  Serial.printf("[DriveBoard] Tuning => Left=%.2f, Right=%.2f\n", leftTune, rightTune);
}

void setMotorOmni(float xVal, float yVal, float rVal)
{
  if (emergencyStopActive) return;
  xVal = constrain(xVal, -100, 100);
  yVal = constrain(yVal, -100, 100);
  rVal = constrain(rVal, -100, 100);
  float frontLeft  = yVal - xVal + rVal;
  float frontRight = yVal + xVal - rVal;
  float backLeft   = yVal + xVal + rVal;
  float backRight  = yVal - xVal - rVal;
  float maxValue = max(
    max(fabs(frontLeft), fabs(frontRight)),
    max(fabs(backLeft),  fabs(backRight))
  );
  if (maxValue > 100.0) {
    float scale = 100.0 / maxValue;
    frontLeft  *= scale;
    frontRight *= scale;
    backLeft   *= scale;
    backRight  *= scale;
  }
  desiredLeftPWM   = map((int)fabs(frontLeft),   0, 100, 0, MOTOR_MAX_PWM);
  desiredRightPWM  = map((int)fabs(frontRight),  0, 100, 0, MOTOR_MAX_PWM);
  desiredMotor3PWM = map((int)fabs(backLeft),    0, 100, 0, MOTOR_MAX_PWM);
  desiredMotor4PWM = map((int)fabs(backRight),   0, 100, 0, MOTOR_MAX_PWM);
  desiredLeftDirection   = (frontLeft  >= 0);
  desiredRightDirection  = (frontRight >= 0);
  desiredMotor3Direction = (backLeft   >= 0);
  desiredMotor4Direction = (backRight  >= 0);
  Serial.printf("[DriveBoard] Omni => x=%.1f y=%.1f r=%.1f\n", xVal, yVal, rVal);
}

void setMotorSimple(float speedPercent, const String& forwardBackward, float turnRatePercent)
{
  if (emergencyStopActive) return;
  float basePWM = map((int)speedPercent, 0, 100, 0, MOTOR_MAX_PWM);
  if (basePWM < MOTOR_MIN_PWM && basePWM != 0) {
    basePWM = MOTOR_MIN_PWM;
  }
  float turnFactor = turnRatePercent / 50.0;
  float leftPWM  = basePWM;
  float rightPWM = basePWM;
  if (turnFactor > 0) {
    rightPWM = basePWM * (1.0 - turnFactor);
  } else if (turnFactor < 0) {
    leftPWM = basePWM  * (1.0 + turnFactor);
  }
  leftPWM  = constrain(leftPWM,  0, MOTOR_MAX_PWM);
  rightPWM = constrain(rightPWM, 0, MOTOR_MAX_PWM);
  int directionBit = (forwardBackward.equalsIgnoreCase("Forward")) ? 1 : 0;
  desiredLeftPWM   = leftPWM;
  desiredRightPWM  = rightPWM;
  desiredMotor3PWM = leftPWM;
  desiredMotor4PWM = rightPWM;
  desiredLeftDirection   = directionBit;
  desiredRightDirection  = directionBit;
  desiredMotor3Direction = directionBit;
  desiredMotor4Direction = directionBit;
  Serial.printf("[DriveBoard] Simple => spd=%.1f dir=%s turn=%.1f\n",
                speedPercent, forwardBackward.c_str(), turnRatePercent);
}

void updateMotorSignals()
{
  if (emergencyStopActive) return;
  digitalWrite(motor1In1, (desiredLeftDirection)   ? HIGH : LOW);
  digitalWrite(motor1In2, (desiredLeftDirection)   ? LOW  : HIGH);
  digitalWrite(motor2In1, (desiredRightDirection)  ? HIGH : LOW);
  digitalWrite(motor2In2, (desiredRightDirection)  ? LOW  : HIGH);
  digitalWrite(motor3In1, (desiredMotor3Direction) ? HIGH : LOW);
  digitalWrite(motor3In2, (desiredMotor3Direction) ? LOW  : HIGH);
  digitalWrite(motor4In1, (desiredMotor4Direction) ? HIGH : LOW);
  digitalWrite(motor4In2, (desiredMotor4Direction) ? LOW  : HIGH);
  float finalLeftPWM   = (float)desiredLeftPWM   * leftTune;
  float finalRightPWM  = (float)desiredRightPWM  * rightTune;
  float finalMotor3PWM = (float)desiredMotor3PWM * leftTune;
  float finalMotor4PWM = (float)desiredMotor4PWM * rightTune;
  int leftOut   = (int)constrain(finalLeftPWM,   0, MOTOR_MAX_PWM);
  int rightOut  = (int)constrain(finalRightPWM,  0, MOTOR_MAX_PWM);
  int motor3Out = (int)constrain(finalMotor3PWM, 0, MOTOR_MAX_PWM);
  int motor4Out = (int)constrain(finalMotor4PWM, 0, MOTOR_MAX_PWM);
  ledcWrite(enablePin1, leftOut);
  ledcWrite(enablePin2, rightOut);
  ledcWrite(enablePin3, motor3Out);
  ledcWrite(enablePin4, motor4Out);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Drive Board (ESP32) with conditional auto-reboot on ESPNOW inactivity");
  pinMode(motor1In1, OUTPUT);
  pinMode(motor1In2, OUTPUT);
  pinMode(motor2In1, OUTPUT);
  pinMode(motor2In2, OUTPUT);
  pinMode(motor3In1, OUTPUT);
  pinMode(motor3In2, OUTPUT);
  pinMode(motor4In1, OUTPUT);
  pinMode(motor4In2, OUTPUT);
  ledcAttach(enablePin1, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(enablePin2, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(enablePin3, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(enablePin4, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  digitalWrite(motor1In1, LOW);
  digitalWrite(motor1In2, LOW);
  digitalWrite(motor2In1, LOW);
  digitalWrite(motor2In2, LOW);
  digitalWrite(motor3In1, LOW);
  digitalWrite(motor3In2, LOW);
  digitalWrite(motor4In1, LOW);
  digitalWrite(motor4In2, LOW);
  ledcWrite(enablePin1, 0);
  ledcWrite(enablePin2, 0);
  ledcWrite(enablePin3, 0);
  ledcWrite(enablePin4, 0);
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
  lastPacketMillis    = millis();
  Serial.printf("Will only auto-reboot if a packet arrives, then 5s passes with no new packets.\n");
  Serial.println("Drive Board ready.");
}

void loop()
{
  updateMotorSignals();
  if (firstPacketReceived) {
    if (millis() - lastPacketMillis >= REBOOT_TIMEOUT_MS) {
      Serial.println("[DriveBoard] No ESPNOW packets for 5s => EStop + Reboot!");
      handleEmergencyStop();
      delay(500);
      ESP.restart();
    }
  }
  delay(20);
}
