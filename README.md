# ESP32-C3 Web-Controlled RC Car

A browser-based RC car project powered by ESP32-C3, featuring real-time control through an intuitive web interface with virtual joystick controls.

## Features

- Browser-based control interface
- Virtual joystick with touch/mouse support
- Wi-Fi AP mode for direct connection
- Real-time motor control
- Individual wheel speed tuning
- Live status monitoring
- Precision turn control
- Emergency stop function

## Quick Start

1. Flash the code to your ESP32-C3
2. Power up the car
3. Connect to Wi-Fi network "Web RC Car"
4. Navigate to `192.168.1.101` in your browser
5. Start driving!

## Hardware Requirements

- ESP32-C3-Mini development board
- Dual H-Bridge motor driver
- 2x DC motors with wheels
- LiPo battery or AA batteries
- Chassis (3D printed, Laser Cut parts, etc)
- Basic electronic components

## Pin Configuration

| Pin | Function |
|-----|----------|
| 7   | Left Motor PWM |
| 0   | Right Motor PWM |
| 8   | Left Motor Forward |
| 9   | Left Motor Reverse |
| 4   | Right Motor Forward |
| 3   | Right Motor Reverse |

## Web Interface

The interface includes:
- Interactive joystick
- Real-time wheel speed indicators
- Quick control buttons
- Fine-tuning controls
- Status monitoring panel

## Images

### CAD Design
![CAD Model](docs/chasis.png)
*3D CAD model of chasis*

### Vehicle Build
![RC Car Build](docs/car.jpg)

### Circuit Wiring
![Circuit Schematic](docs/circuit.jpg)
*Actual wiring*

## Technical Details

### Motor Control
- 10-bit PWM resolution (0-1023)
- 50Hz PWM frequency
- Minimum PWM threshold for reliable start
- Individual wheel speed tuning
- Proportional turn control

### Network Configuration
- AP Mode: `Web RC Car`
- IP: 192.168.1.101
- No password required
- Optimized for low latency

## Performance Tuning

1. **Motor Dead Zone**: Adjust `MOTOR_MIN_PWM` if motors don't start smoothly
2. **Turn Sensitivity**: Modify the turn rate mapping in the web interface
3. **Wheel Balance**: Use the tuning sliders to compensate for motor differences
