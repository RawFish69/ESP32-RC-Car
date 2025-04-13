/*
 * ESP32 RC Car - Control System Documentation
 * Single board approach with integrated web interface
 * The MCU serves as access point and sets up a web server to send steering commands
 * =========================================
 * 
 * Web-to-ESP32 Communication Parameters:
 * ------------------------------------
 * 1. Movement Control (/setMotor):
 *    a) Omni-directional mode:
 *       x=[-100..100]     : Strafe movement (negative=left, positive=right)
 *       y=[-100..100]     : Forward/backward (negative=forward, positive=backward)
 *       rotate=[-100..100]: Rotation (negative=counterclockwise, positive=clockwise)
 *    
 *    b) Simple mode:
 *       speed=[0..100]           : Absolute speed value
 *       forwardBackward=[Forward|Backward]
 *       turnRate=[-50..50]       : Turn rate value*
 * ------------------------------------
 * Board Module: ESP32C3-Mini
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h> 
#include "web_interface.h"
#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY 50
#define MOTOR_MIN_PWM 300 
#define MOTOR_MAX_PWM 1023

const char* ssid = "Web RC Car";
const char* password = "";
IPAddress local_IP(192, 168, 1, 101);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);

// Left Front Motor (Motor 1)
const int enablePin1 = 5;   // PWM pin
const int motor1In1 = 6;    // Direction control 1
const int motor1In2 = 7;    // Direction control 2

// Right Front Motor (Motor 2)
const int enablePin2 = 20;  // PWM pin
const int motor2In1 = 21;   // Direction control 1
const int motor2In2 = 3;    // Direction control 2

// Left Back Motor (Motor 3) 
const int enablePin3 = 8;   // PWM pin
const int motor3In1 = 9;    // Direction control 1
const int motor3In2 = 10;   // Direction control 2

// Right Back Motor (Motor 4)
const int enablePin4 = 2;   // PWM pin
const int motor4In1 = 1;    // Direction control 1
const int motor4In2 = 0;    // Direction control 2

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
volatile unsigned long serverPrevTime = 0;
const unsigned long serverInterval = 50;

void handleRoot();
void handleSetMotor();
void handleSetWheelTuning();
void handleEmergencyStop();
void updateMotorSignals();

void setup() {
    Serial.begin(115200);
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
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ssid, password, 1, 0, 4);
    WiFi.setSleep(false);
    Serial.print("AP SSID: ");
    Serial.println(ssid);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(78);
    server.on("/", handleRoot);
    server.on("/setMotor", handleSetMotor);
    server.on("/setWheelTuning", handleSetWheelTuning);
    server.on("/emergencyStop", handleEmergencyStop);
    server.begin();
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - serverPrevTime >= serverInterval) {
        server.handleClient();
        serverPrevTime = currentMillis;
    }
    updateMotorSignals();
    delay(50); // Small delay to prevent CPU overload
}

void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleSetMotor() {
    if (server.hasArg("x") && server.hasArg("y") && server.hasArg("rotate")) {
        float xVal = server.arg("x").toFloat();
        float yVal = server.arg("y").toFloat();
        float rVal = server.arg("rotate").toFloat();
        float frontLeft = yVal - xVal + rVal;
        float frontRight = yVal + xVal - rVal;
        float backLeft = yVal + xVal + rVal;
        float backRight = yVal - xVal - rVal;
        float maxValue = max(max(fabs(frontLeft), fabs(frontRight)),max(fabs(backLeft), fabs(backRight)));
        if (maxValue > 100.0) {
            float scale = 100.0f / maxValue;
            frontLeft *= scale;
            frontRight *= scale;
            backLeft *= scale;
            backRight *= scale;
        }
        desiredLeftPWM = map((int)fabs(frontLeft), 0, 100, 0, MOTOR_MAX_PWM);
        desiredRightPWM = map((int)fabs(frontRight),0, 100, 0, MOTOR_MAX_PWM);
        desiredMotor3PWM = map((int)fabs(backLeft), 0, 100, 0, MOTOR_MAX_PWM);
        desiredMotor4PWM = map((int)fabs(backRight), 0, 100, 0, MOTOR_MAX_PWM);
        desiredLeftDirection = (frontLeft >= 0);
        desiredRightDirection = (frontRight >= 0);
        desiredMotor3Direction = (backLeft >= 0);
        desiredMotor4Direction = (backRight >= 0);
        server.send(200, "text/plain", "OK");
    }
    else if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {
        float speedPercent = server.arg("speed").toFloat();
        String forwardBackward = server.arg("forwardBackward");
        float turnRatePercent = server.arg("turnRate").toFloat();
        int basePWM = map((int)speedPercent, 0, 100, 0, MOTOR_MAX_PWM);
        if (basePWM < MOTOR_MIN_PWM && basePWM != 0) {
            basePWM = MOTOR_MIN_PWM;
        }
        float turnFactor = turnRatePercent / 50.0;
        int leftPWM = basePWM; 
        int rightPWM = basePWM;
        if (turnFactor > 0) {
            rightPWM = basePWM * (1.0 - turnFactor);
        } else if (turnFactor < 0) {
            leftPWM = basePWM * (1.0 + turnFactor);
        }
        leftPWM = constrain(leftPWM, 0, MOTOR_MAX_PWM);
        rightPWM = constrain(rightPWM, 0, MOTOR_MAX_PWM);
        int directionBit = (forwardBackward == "Forward") ? 1 : 0;
        desiredLeftPWM = leftPWM;
        desiredRightPWM = rightPWM;
        desiredMotor3PWM = leftPWM;
        desiredMotor4PWM = rightPWM;
        desiredLeftDirection = directionBit;
        desiredRightDirection = directionBit;
        desiredMotor3Direction = directionBit;
        desiredMotor4Direction = directionBit;
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

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

void handleEmergencyStop() {
    desiredLeftPWM = 0;
    desiredRightPWM = 0;
    desiredMotor3PWM = 0;
    desiredMotor4PWM = 0;
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
    server.send(200, "text/plain", "Emergency Stop Activated");
}

void updateMotorSignals() {
    digitalWrite(motor1In1, (desiredLeftDirection) ? HIGH : LOW);
    digitalWrite(motor1In2, (desiredLeftDirection) ? LOW : HIGH);
    digitalWrite(motor2In1, (desiredRightDirection) ? HIGH : LOW);
    digitalWrite(motor2In2, (desiredRightDirection) ? LOW : HIGH);
    digitalWrite(motor3In1, (desiredMotor3Direction) ? HIGH : LOW);
    digitalWrite(motor3In2, (desiredMotor3Direction) ? LOW : HIGH);
    digitalWrite(motor4In1, (desiredMotor4Direction) ? HIGH : LOW);
    digitalWrite(motor4In2, (desiredMotor4Direction) ? LOW : HIGH);
    float finalLeftPWM = (float)desiredLeftPWM * leftTune;
    float finalRightPWM = (float)desiredRightPWM * rightTune;
    float finalMotor3PWM = (float)desiredMotor3PWM * leftTune;
    float finalMotor4PWM = (float)desiredMotor4PWM * rightTune;
    int leftOut = (int)constrain(finalLeftPWM, 0, MOTOR_MAX_PWM);
    int rightOut = (int)constrain(finalRightPWM, 0, MOTOR_MAX_PWM);
    int motor3Out = (int)constrain(finalMotor3PWM, 0, MOTOR_MAX_PWM);
    int motor4Out = (int)constrain(finalMotor4PWM, 0, MOTOR_MAX_PWM);
    ledcWrite(enablePin1, leftOut);
    ledcWrite(enablePin2, rightOut);
    ledcWrite(enablePin3, motor3Out);
    ledcWrite(enablePin4, motor4Out);
}

