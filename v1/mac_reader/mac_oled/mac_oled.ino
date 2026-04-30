#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Wire.begin(3, 4);
    display.begin(SCREEN_ADDRESS, true);
    String macAddr = WiFi.macAddress();
    Serial.println("Device STA MAC Address: " + macAddr);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("STA MAC Address:");
    display.setCursor(0, 16);
    display.println(macAddr);
    display.display();
    WiFi.mode(WIFI_AP);
    uint8_t softAPmac[6];
    WiFi.softAPmacAddress(softAPmac);
    Serial.print("Device SoftAP MAC Address: ");
    for(int i = 0; i < 6; i++) {
        Serial.printf("%02X", softAPmac[i]);
        if(i < 5) Serial.print(":");
    }
    Serial.println();
    display.setCursor(0, 32);
    display.println("SoftAP MAC Address:");
    display.setCursor(0, 48);
    for(int i = 0; i < 6; i++) {
        display.printf("%02X", softAPmac[i]);
        if(i < 5) display.print(":");
    }
    display.display();
}

void loop() {
    delay(100);
}
