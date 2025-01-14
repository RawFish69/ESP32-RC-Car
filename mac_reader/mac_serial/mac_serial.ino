#include <WiFi.h>

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    String macAddr = WiFi.macAddress();
    Serial.println("Device STA MAC Address: " + macAddr);
    WiFi.mode(WIFI_AP);
    uint8_t softAPmac[6];
    WiFi.softAPmacAddress(softAPmac);
    Serial.print("Device SoftAP MAC Address: ");
    for(int i = 0; i < 6; i++) {
        Serial.printf("%02X", softAPmac[i]);
        if(i < 5) Serial.print(":");
    }
    Serial.println();
}

void loop() {
    delay(100);
}
