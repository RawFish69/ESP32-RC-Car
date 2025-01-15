/*
  ESP32 RC Car Control
  Switches between Default (2-wheel) and Omni (4-wheel) modes.
  Author: RawFish69

  Supported features:
    - Switch between Default and Omni modes
    - Control car movement in both modes
    - Emergency stop
    - Tune left and right wheel speeds
    - Monitor status and commands on OLED display
    - Web interface for control
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "web_interface_default.h"
#include "web_interface_omni.h"

#define MODE_SWITCH_PIN 5

// Replace these MAC addresses with the ones you want to use
// These are the MAC addresses of my ESP32 boards
// You can use the mac_reader.ino to get the MAC addresses of your boards
uint8_t DEFAULT_DRIVE_MAC[6] = {0x68, 0x67, 0x25, 0xB1, 0xBC, 0x8C};
uint8_t OMNI_DRIVE_MAC[6] = {0xE4, 0xB3, 0x23, 0xD4, 0xEC, 0x14};
uint8_t DRIVE_BOARD_MAC[6];

const char* ssid_default = "RC Default Mode";
const char* ssid_omni = "RC Omni Mode";
const char* password = "";

IPAddress local_IP(192, 168, 1, 101);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

WebServer server(80);

enum CarMode { MODE_DEFAULT, MODE_OMNI };
CarMode currentMode = MODE_DEFAULT;
CarMode lastMode = MODE_DEFAULT;
bool firstSetup = true;

String g_peerStatus = "N/A";
String g_wifiStatus = "N/A";
String g_serverStatus = "N/A";
String g_finalStatus = "Starting...";
int g_clientCount = 0;

enum CmdType { CMD_OMNI, CMD_SIMPLE, CMD_TUNE, CMD_ESTOP, CMD_NONE };
CmdType lastModeCmd = CMD_NONE;

float lastX = 0, lastY = 0, lastR = 0;
float lastSpeed = 0, lastTurn = 0;
String lastDir = "";
float lastLeft = 1.0, lastRight = 1.0;
String lastDelivery = "Sending...";
static unsigned long lastSendTime = 0;

void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      g_clientCount++;
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      if (g_clientCount > 0) g_clientCount--;
      if (g_clientCount <= 0) {
        g_clientCount = 0;
        g_wifiStatus = "Disconnected";
        showInitScreen();
      }
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      if (g_clientCount > 0) {
        g_wifiStatus = local_IP.toString();
        showInitScreen();
      }
      break;
    default:
      break;
  }
}

String macToString(const uint8_t mac[6]) {
  char buff[20];
  sprintf(buff, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buff);
}

void drawTopLine(const String &msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println(msg);
  display.drawLine(0, 9, SCREEN_WIDTH, 9, SH110X_WHITE);
  display.display();
}

void clearMonitorArea() {
  display.fillRect(0, 10, SCREEN_WIDTH, SCREEN_HEIGHT - 10, SH110X_BLACK);
  display.display();
}

void showInitScreen() {
  clearMonitorArea();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  int yPos = 12;
  display.setCursor(0, yPos);
  display.println("Peer: " + g_peerStatus);
  yPos += 10;
  display.setCursor(0, yPos);
  display.println("WiFi: " + g_wifiStatus);
  yPos += 10;
  display.setCursor(0, yPos);
  display.println("Server: " + g_serverStatus);
  yPos += 10;
  String cMsg = (g_clientCount == 1) ? "Client: 1" : "Client: " + String(g_clientCount);
  display.setCursor(0, yPos);
  display.println(cMsg);
  yPos += 10;
  display.setCursor(0, yPos);
  display.println(g_finalStatus);
  display.display();
}

void clearAndDrawCommand() {
  clearMonitorArea();
  display.setCursor(0, 12);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  int lineY = 12;
  switch(lastModeCmd) {
    case CMD_OMNI: {
      display.println("LastCmd: Omni");
      lineY += 10;
      char buf[32];
      sprintf(buf, "x=%.0f y=%.0f r=%.0f", lastX, lastY, lastR);
      display.setCursor(0, lineY);
      display.println(buf);
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Delivery: " + lastDelivery);
      break;
    }
    case CMD_SIMPLE: {
      display.println("LastCmd: Default");
      lineY += 10;
      char buf[32];
      sprintf(buf, "Sp=%.0f Tr=%.0f", lastSpeed, lastTurn);
      display.setCursor(0, lineY);
      display.println(buf);
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Dir: " + lastDir);
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Delivery: " + lastDelivery);
      break;
    }
    case CMD_TUNE: {
      display.println("LastCmd: Tune");
      lineY += 10;
      char buf[32];
      sprintf(buf, "L=%.2f R=%.2f", lastLeft, lastRight);
      display.setCursor(0, lineY);
      display.println(buf);
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Delivery: " + lastDelivery);
      break;
    }
    case CMD_ESTOP: {
      display.println("LastCmd: E-Stop");
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Delivery: " + lastDelivery);
      break;
    }
    default:
      display.println("No command");
      break;
  }
  display.display();
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
    clearAndDrawCommand();
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastDelivery = "Success";
  } else {
    lastDelivery = "Fail";
  }
  clearAndDrawCommand();
}

void showOmniCommand(float x, float y, float r) {
  lastModeCmd = CMD_OMNI;
  lastX = x;
  lastY = y;
  lastR = r;
  lastDelivery = "Sending...";
  clearAndDrawCommand();
}

void showSimpleCommand(float spd, const String &dir, float trn) {
  lastModeCmd = CMD_SIMPLE;
  lastSpeed = spd;
  lastDir = dir;
  lastTurn = trn;
  lastDelivery = "Sending...";
  clearAndDrawCommand();
}

void showTuneCommand(float lt, float rt) {
  lastModeCmd = CMD_TUNE;
  lastLeft = lt;
  lastRight = rt;
  lastDelivery = "Sending...";
  clearAndDrawCommand();
}

void showEStopCommand() {
  lastModeCmd = CMD_ESTOP;
  lastDelivery = "Sending...";
  clearAndDrawCommand();
}

void handleRoot() {
  if (currentMode == MODE_OMNI) {
    server.send_P(200, "text/html", OMNI_INDEX_HTML);
  } else {
    server.send_P(200, "text/html", DEFAULT_INDEX_HTML);
  }
}

void handleSetMotor() {
  if (currentMode == MODE_OMNI) {
    if (server.hasArg("x") && server.hasArg("y") && server.hasArg("rotate")) {
      float xVal = server.arg("x").toFloat();
      float yVal = server.arg("y").toFloat();
      float rVal = server.arg("rotate").toFloat();
      showOmniCommand(xVal, yVal, rVal);
      String query = "mode=omni&x=" + server.arg("x") + "&y=" + server.arg("y") + "&rotate=" + server.arg("rotate");
      sendEspNowMessage(query);
      server.send(200, "text/plain", "OK (Omni)");
    } else {
      server.send(400, "text/plain", "Bad Omni request");
    }
  } else {
    if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {
      float spd = server.arg("speed").toFloat();
      String dir = server.arg("forwardBackward");
      float trn = server.arg("turnRate").toFloat();
      showSimpleCommand(spd, dir, trn);
      String query = "mode=default&speed=" + server.arg("speed") + "&forwardBackward=" + dir + "&turnRate=" + server.arg("turnRate");
      sendEspNowMessage(query);
      server.send(200, "text/plain", "OK (Default)");
    } else {
      server.send(400, "text/plain", "Bad Default request");
    }
  }
}

void handleSetWheelTuning() {
  if (server.hasArg("leftTune") && server.hasArg("rightTune")) {
    float lt = server.arg("leftTune").toFloat();
    float rt = server.arg("rightTune").toFloat();
    showTuneCommand(lt, rt);
    String query = "tune&leftTune=" + server.arg("leftTune") + "&rightTune=" + server.arg("rightTune");
    sendEspNowMessage(query);
    server.send(200, "text/plain", "Tuning updated");
  } else {
    server.send(400, "text/plain", "Bad Tuning request");
  }
}

void handleEmergencyStop() {
  showEStopCommand();
  sendEspNowMessage("emergencyStop");
  server.send(200, "text/plain", "Emergency Stop Activated");
}

void handleGetStatus() {
  server.send(200, "application/json", "{\"status\":\"OK\"}");
}

void initWiFiAndEspNow() {
  if (!firstSetup) {
    esp_now_del_peer(DRIVE_BOARD_MAC);
    esp_now_deinit();
  }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  if (currentMode == MODE_OMNI) {
    WiFi.softAP(ssid_omni, password);
    memcpy(DRIVE_BOARD_MAC, OMNI_DRIVE_MAC, 6);
    drawTopLine("Omni Mode");
  } else {
    WiFi.softAP(ssid_default, password);
    memcpy(DRIVE_BOARD_MAC, DEFAULT_DRIVE_MAC, 6);
    drawTopLine("Default Mode");
  }
  g_wifiStatus = "AP Ready";
  showInitScreen();
  if (esp_now_init() != ESP_OK) {
    g_finalStatus = "ESPNOW FAIL";
    showInitScreen();
    Serial.println("ESPNOW init fail");
    delay(2000);
    ESP.restart();
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, DRIVE_BOARD_MAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    g_peerStatus = "FAIL";
    showInitScreen();
    Serial.println("Peer Add FAIL");
  } else {
    g_peerStatus = "OK";
  }
  g_finalStatus = (currentMode == MODE_OMNI) ? "Omni Ready" : "Default Ready";
  showInitScreen();
  firstSetup = false;
}

void setup() {
  Serial.begin(115200);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  currentMode = (digitalRead(MODE_SWITCH_PIN) == HIGH) ? MODE_OMNI : MODE_DEFAULT;
  lastMode = currentMode;
  Wire.begin(3, 4);
  display.begin(SCREEN_ADDRESS, true);
  WiFi.onEvent(onWiFiEvent);
  initWiFiAndEspNow();
  server.on("/", handleRoot);
  server.on("/setMotor", handleSetMotor);
  server.on("/setWheelTuning", handleSetWheelTuning);
  server.on("/emergencyStop", handleEmergencyStop);
  server.on("/getStatus", handleGetStatus);
  server.enableCORS(true);
  server.begin();
  g_serverStatus = "Server OK";
  showInitScreen();
  Serial.println("Setup complete");
}

void loop() {
  server.handleClient();
  CarMode newMode = (digitalRead(MODE_SWITCH_PIN) == HIGH) ? MODE_OMNI : MODE_DEFAULT;
  if (newMode != currentMode) {
    currentMode = newMode;
    display.clearDisplay();
    display.display();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    String modeStr = (currentMode == MODE_OMNI) ? "OMNI" : "DEFAULT";
    display.print("SWITCHED to ");
    display.println(modeStr);
    display.drawLine(0, 9, SCREEN_WIDTH, 9, SH110X_WHITE);
    display.setCursor(0, 12);
    String newMac = (currentMode == MODE_OMNI) ? macToString(OMNI_DRIVE_MAC) : macToString(DEFAULT_DRIVE_MAC);
    display.println(newMac);
    for(int i=0; i<=20; i++){
      display.fillRect(0, 54, SCREEN_WIDTH, 8, SH110X_BLACK);
      display.fillRect(0, 54, i * 6, 8, SH110X_WHITE);
      display.display();
      delay(50);
    }
    delay(300);
    initWiFiAndEspNow();
  }
  delay(10);
}
