#ifndef RGB_LED_H
#define RGB_LED_H

#include <Arduino.h>

// RGB LED configuration
#define RGB_BUILTIN 2
#define RGB_BRIGHTNESS 255

extern volatile int currentSpeed;
extern unsigned long lastLedUpdate;
const int LED_UPDATE_INTERVAL = 50;

// LED state variables
static int breathBrightness = 0;
static bool breathingUp = true;
static uint8_t lastRed = 0, lastGreen = 0, lastBlue = 0;

// Helper function for smooth color transitions
void smoothTransition(uint8_t startRed, uint8_t startGreen, uint8_t startBlue, 
                     uint8_t endRed, uint8_t endGreen, uint8_t endBlue, 
                     int steps, int delayTime) {
    for (int i = 0; i <= steps; i++) {
        uint8_t red = startRed + ((endRed - startRed) * i) / steps;
        uint8_t green = startGreen + ((endGreen - startGreen) * i) / steps;
        uint8_t blue = startBlue + ((endBlue - startBlue) * i) / steps;
        rgbLedWrite(RGB_BUILTIN, red, green, blue);
        delay(delayTime);
    }
}

void updateLED() {
    static float hue = 0;
    uint8_t targetRed, targetGreen, targetBlue;
    
    if (currentSpeed == 0) {
        float h = hue / 360.0f;
        float s = 1.0f;
        float v = RGB_BRIGHTNESS / 255.0f;
        
        float c = v * s;
        float x = c * (1 - abs(fmod(h * 6, 2) - 1));
        float m = v - c;
        
        if(h < 1.0f/6.0f) {
            targetRed = (c + m) * 255;
            targetGreen = (x + m) * 255;
            targetBlue = m * 255;
        } else if(h < 2.0f/6.0f) {
            targetRed = (x + m) * 255;
            targetGreen = (c + m) * 255;
            targetBlue = m * 255;
        } else if(h < 3.0f/6.0f) {
            targetRed = m * 255;
            targetGreen = (c + m) * 255;
            targetBlue = (x + m) * 255;
        } else if(h < 4.0f/6.0f) {
            targetRed = m * 255;
            targetGreen = (x + m) * 255;
            targetBlue = (c + m) * 255;
        } else if(h < 5.0f/6.0f) {
            targetRed = (x + m) * 255;
            targetGreen = m * 255;
            targetBlue = (c + m) * 255;
        } else {
            targetRed = (c + m) * 255;
            targetGreen = m * 255;
            targetBlue = (x + m) * 255;
        }
        
        smoothTransition(lastRed, lastGreen, lastBlue,
                        targetRed, targetGreen, targetBlue, 30, 1);
        
        lastRed = targetRed;
        lastGreen = targetGreen;
        lastBlue = targetBlue;
        
        hue = fmod(hue + 1, 360);
    } else {
        if (breathingUp) {
            breathBrightness += 5;
            if (breathBrightness >= RGB_BRIGHTNESS) {
                breathBrightness = RGB_BRIGHTNESS;
                breathingUp = false;
            }
        } else {
            breathBrightness -= 5;
            if (breathBrightness <= 10) {
                breathBrightness = 10;
                breathingUp = true;
            }
        }
        
        int scaledBrightness = map(abs(currentSpeed), 0, 100, 10, breathBrightness);
        smoothTransition(lastRed, lastGreen, lastBlue,
                        scaledBrightness, 0, 0, 30, 1);
        
        lastRed = scaledBrightness;
        lastGreen = lastBlue = 0;
    }
}

#endif