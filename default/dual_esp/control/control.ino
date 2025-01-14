#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "web_interface.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// 68:67:25:B1:BC:8C
uint8_t DRIVE_BOARD_MAC[] = {0x68, 0x67, 0x25, 0xB1, 0xBC, 0x8C};

WebServer server(80);

const char* ssid = "C3 Web RC";
const char* password = "";

IPAddress local_IP(192, 168, 1, 101);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static unsigned long lastSendTime = 0;

int g_clientCount = 0;

String g_peerStatus = "N/A";
String g_wifiStatus = "N/A";
String g_serverStatus = "N/A";
String g_finalStatus = "Starting...";

enum CmdMode {CMD_SIMPLE, CMD_TUNE, CMD_ESTOP, CMD_NONE};
CmdMode lastMode = CMD_NONE;
float lastSpeed = 0, lastTurn = 0;
String lastDir = "";
float lastLeft = 1.0, lastRight = 1.0;
String lastDelivery = "Sending...";

void sendEspNowMessage(const String &msg) {
  unsigned long now = millis();
  if (now - lastSendTime < 20) {
    delay(20 - (now - lastSendTime));
  }
  lastSendTime = millis();
  esp_err_t ret = esp_now_send(DRIVE_BOARD_MAC, 
                              (const uint8_t*)msg.c_str(),
                              msg.length()+1);
  if (ret != ESP_OK) {
    lastDelivery = "SendFail(" + String(ret) + ")";
    clearAndDrawCommand();
  }
}

// ESPNOW callback
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastDelivery = "Success";
  } else {
    lastDelivery = "Fail";
  }
  clearAndDrawCommand();
}

void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      g_clientCount++;
      updateInitScreen();
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      if (g_clientCount > 0) g_clientCount--;
      updateInitScreen();
      break;
    default:
      break;
  }
}

String macToString(const uint8_t mac[6])
{
  char buff[20];
  sprintf(buff, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buff);
}

void drawTopLine(const String &macStr)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println(macStr);
  display.drawLine(0, 9, SCREEN_WIDTH, 9, SH110X_WHITE);
  display.display();
}

void clearMonitorArea()
{
  display.fillRect(0, 10, SCREEN_WIDTH, SCREEN_HEIGHT - 10, SH110X_BLACK);
}

void showInitScreen()
{
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
  String cMsg;
  if (g_clientCount <= 0)      cMsg = "Client: none";
  else if (g_clientCount == 1) cMsg = "Client: 1 connected";
  else                         cMsg = "Client: " + String(g_clientCount) + " connected";
  display.setCursor(0, yPos);
  display.println(cMsg);
  yPos += 10;
  display.setCursor(0, yPos);
  display.println(g_finalStatus);
  display.display();
}

void updateInitScreen()
{
  showInitScreen();
}

void clearAndDrawCommand()
{
  clearMonitorArea();
  display.setCursor(0, 12);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  int lineY = 12;
  switch(lastMode)
  {
    case CMD_SIMPLE:
    {
      display.setCursor(0, lineY);
      display.println("Mode: Simple");
      lineY += 10;
      char buf[30];
      sprintf(buf, "Speed: %.0f", lastSpeed);
      display.setCursor(0, lineY);
      display.println(buf);
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Direction: " + lastDir);
      lineY += 10;
      sprintf(buf, "Turn: %.0f", lastTurn);
      display.setCursor(0, lineY);
      display.println(buf);
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Delivery: " + lastDelivery);
    }
    break;
    case CMD_TUNE:
    {
      display.setCursor(0, lineY);
      display.println("Mode: Tune");
      lineY += 10;
      char buf[30];
      sprintf(buf, "Left=%.2f, Right=%.2f", lastLeft, lastRight);
      display.setCursor(0, lineY);
      display.println(buf);
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Delivery: " + lastDelivery);
    }
    break;
    case CMD_ESTOP:
    {
      display.setCursor(0, lineY);
      display.println("Mode: E-Stop");
      lineY += 10;
      display.setCursor(0, lineY);
      display.println("Delivery: " + lastDelivery);
    }
    break;
    default:
    {
      display.setCursor(0, lineY);
      display.println("No command");
    }
    break;
  }
  display.display();
}

void showSimpleCommand(float spd, const String &dir, float trn)
{
  lastMode = CMD_SIMPLE;
  lastSpeed = spd;
  lastDir = dir;
  lastTurn = trn;
  lastDelivery = "Sending...";
  clearAndDrawCommand();
}

void showTuneCommand(float lt, float rt)
{
  lastMode = CMD_TUNE;
  lastLeft = lt;
  lastRight = rt;
  lastDelivery = "Sending...";
  clearAndDrawCommand();
}

void showEStopCommand()
{
  lastMode = CMD_ESTOP;
  lastDelivery = "Sending...";
  clearAndDrawCommand();
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleSetMotor() {
  if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {
    float spd = server.arg("speed").toFloat();
    String dir = server.arg("forwardBackward");
    float trn = server.arg("turnRate").toFloat();
    showSimpleCommand(spd, dir, trn);
    String query = "speed=" + server.arg("speed")
                 + "&forwardBackward=" + server.arg("forwardBackward")
                 + "&turnRate=" + server.arg("turnRate");
    sendEspNowMessage(query);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleSetWheelTuning() {
  if (server.hasArg("leftTune") && server.hasArg("rightTune")) {
    float lt = server.arg("leftTune").toFloat();
    float rt = server.arg("rightTune").toFloat();
    showTuneCommand(lt, rt);
    String query = "tune&leftTune=" + server.arg("leftTune")
                 + "&rightTune=" + server.arg("rightTune");
    sendEspNowMessage(query);
    server.send(200, "text/plain", "Tuning updated");
  } else {
    server.send(400, "text/plain", "Bad Request");
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

void setup() {
  Serial.begin(115200);
  String driveMacStr = macToString(DRIVE_BOARD_MAC);
  Wire.begin(3, 4); 
  display.begin(SCREEN_ADDRESS, true);
  drawTopLine(driveMacStr);
  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  g_wifiStatus = "AP Ready";
  updateInitScreen();
  
  server.on("/", handleRoot);
  server.on("/setMotor", handleSetMotor);
  server.on("/setWheelTuning", handleSetWheelTuning);
  server.on("/emergencyStop", handleEmergencyStop);
  server.enableCORS(true);
  server.begin();
  
  g_serverStatus = "Started";
  updateInitScreen();
  
  if (esp_now_init() != ESP_OK) {
    g_finalStatus = "ESPNOW FAIL";
    updateInitScreen();
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
    updateInitScreen();
    Serial.println("Peer Add FAIL");
    return;
  }
  g_peerStatus = "OK";
  g_finalStatus = "Ready...";
  updateInitScreen();
  Serial.println("Control Board ready.");
}

void loop() {
  static unsigned long prev = 0;
  if (millis() - prev >= 50) {
    server.handleClient();
    prev = millis();
  }
}