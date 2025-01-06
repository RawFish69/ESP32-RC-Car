#include <WiFi.h>
#include <WebServer.h>
#include "web_interface.h"

// PWM configuration
#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY 50
#define MOTOR_MIN_PWM 300 
#define MOTOR_MAX_PWM 1023

// Wi-Fi configuration
const char* ssid = "Web RC Car";
const char* password = "";

// IP configuration
IPAddress local_IP(192, 168, 1, 101);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// Pins for ESP32C3-Mini
const int enablePin1 = 7;  // Left motor PWM
const int enablePin2 = 0;  // Right motor PWM
const int motor1In1 = 8;  // Left motor forward
const int motor1In2 = 9;  // Left motor reverse
const int motor2In1 = 4;   // Right motor forward
const int motor2In2 = 3;   // Right motor reverse

// Global variables
volatile int currentSpeed = 0;         // used for display purposes
volatile float currentTurnRate = 0;    // track turn rate for display

// Server timing
volatile unsigned long serverPrevTime = 0;
const unsigned long serverInterval = 50;

// Motor control variables
int desiredLeftPWM = 0;
int desiredLeftDirection = 1;   // 1=Forward, 0=Reverse
int desiredRightPWM = 0;
int desiredRightDirection = 1;  // 1=Forward, 0=Reverse

// Wheel tuning multipliers (default = no change)
volatile float leftTune = 1.0;
volatile float rightTune = 1.0;

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
    WiFi.persistent(false);
    WiFi.setSleep(false);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ssid, password);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    server.on("/", handleRoot);
    server.on("/setMotor", handleSetMotor);
    server.on("/setWheelTuning", handleSetWheelTuning);
    server.on("/emergencyStop", handleEmergencyStop);
    server.on("/getStatus", handleGetStatus);
    server.begin();
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - serverPrevTime > serverInterval) {
        server.handleClient();
        serverPrevTime = currentMillis;
    }
    updateMotorSignals();
}

// Serve the main page with the joystick
void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

/**
 *  /setMotor?speed=50&forwardBackward=Forward&turnRate=20
 *  speed: 0..100
 *  forwardBackward: "Forward" or "Backward"
 *  turnRate: -50..50
 */
void handleSetMotor() {
    if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {
        float speedPercent = server.arg("speed").toFloat();
        String forwardBackward = server.arg("forwardBackward");
        float turnRatePercent = server.arg("turnRate").toFloat();
        currentSpeed = (forwardBackward == "Forward") ? speedPercent : -speedPercent;
        currentTurnRate = turnRatePercent;
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
        desiredLeftDirection  = (forwardBackward == "Forward") ? 1 : 0;
        desiredRightDirection = (forwardBackward == "Forward") ? 1 : 0;
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

/**
 *  /setWheelTuning?leftTune=1.0&rightTune=1.05
 */
void handleSetWheelTuning() {
    if (server.hasArg("leftTune") && server.hasArg("rightTune")) {
        float lt = server.arg("leftTune").toFloat();
        float rt = server.arg("rightTune").toFloat();
        lt = constrain(lt, 0.5, 1.5);
        rt = constrain(rt, 0.5, 1.5);
        leftTune = lt;
        rightTune = rt;
        server.send(200, "text/plain", "Tuning updated");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

/**
 *  /emergencyStop
 */
void handleEmergencyStop() {
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
    server.send(200, "text/plain", "Emergency Stop Activated");
}

/**
 *  /getStatus
 *  Returns JSON with speed, turnRate, leftPWM, rightPWM, etc.
 */
void handleGetStatus() {
    float leftPWM = (float)desiredLeftPWM * leftTune;
    float rightPWM = (float)desiredRightPWM * rightTune;
    String json = "{";
    json += "\"speed\":"      + String(currentSpeed) + ",";
    json += "\"turn\":"       + String(currentTurnRate) + ",";
    json += "\"leftPWM\":"    + String((int)leftPWM) + ",";
    json += "\"rightPWM\":"   + String((int)rightPWM) + ",";
    json += "\"leftDir\":"    + String(desiredLeftDirection) + ",";
    json += "\"rightDir\":"   + String(desiredRightDirection);
    json += "}";
    server.send(200, "application/json", json);
}

void updateMotorSignals() {
    digitalWrite(motor1In1, desiredLeftDirection ? HIGH : LOW);
    digitalWrite(motor1In2, desiredLeftDirection ? LOW : HIGH);
    digitalWrite(motor2In1, desiredRightDirection ? HIGH : LOW);
    digitalWrite(motor2In2, desiredRightDirection ? LOW : HIGH);
    float finalLeftPWM  = desiredLeftPWM  * leftTune;
    float finalRightPWM = desiredRightPWM * rightTune;
    int leftOut  = (int)constrain(finalLeftPWM,  0, MOTOR_MAX_PWM);
    int rightOut = (int)constrain(finalRightPWM, 0, MOTOR_MAX_PWM);
    ledcWrite(enablePin1, leftOut);
    ledcWrite(enablePin2, rightOut);
}
