# ESP32 RC Car

[![Arduino](https://img.shields.io/badge/Arduino-IDE-00979D.svg?style=for-the-badge&logo=Arduino&logoColor=white)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/ESP32-C3-E7352C.svg?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge)](LICENSE)
> A browser-based RC car project powered by ESP32-C3, featuring real-time control through an intuitive web interface with virtual joystick controls. Available in both single-board and dual-board configurations.

---

## Features

### Core Features
- Browser-based control interface
- Virtual joystick with touch/mouse support
- Wi-Fi AP mode for direct connection
- Real-time motor control
- Precision turn control
- Emergency stop function

### Advanced Features
- Individual wheel speed tuning
- Live status monitoring
- Dual-board long-range option
- ESP-NOW peer-to-peer support

## Quick Start

1. Flash the code to your ESP32-C3
2. Power up the car
3. Connect to Wi-Fi network "Web RC Car"
4. Navigate to `192.168.1.101` in your browser

## Architecture

### Single-Board Version
```
┌─────────────┐     ┌──────────────┐     ┌───────────┐
│  Browser    │ WS  │   ESP32-C3   │ PWM │   Motors  │
│  Interface  │◄───►│  Web Server  │────►│   Driver  │
└─────────────┘     └──────────────┘     └───────────┘
```

### Dual-Board Version
```
┌─────────────┐     ┌──────────────┐     ┌──────────────┐     ┌───────────┐
│  Browser    │ WS  │   Control    │     │    Drive     │ PWM │   Motors  │
│  Interface  │◄───►│    Board     │◄───►│    Board     │────►│   Driver  │
└─────────────┘     └──────────────┘     └──────────────┘     └───────────┘
                          WiFi              ESP-NOW
```

### Component Overview

1. **Web Interface** (`web_interface.cpp`)
    - HTML/CSS layout
    - JavaScript joystick controls
    - WebSocket client
    - Real-time status display
    - UI event handling

2. **Control Board** (`control.ino`)
    - Web server hosting
    - User interface handling
    - ESP-NOW transmitter
    - OLED display updates
    - Connection management

3. **Drive Board** (`drive.ino`)
    - ESP-NOW receiver
    - Motor control logic
    - Failsafe handling
    - Automatic reconnection
    - Status monitoring

### Communication Flow
```
Browser (Web Interface) → WebSocket → Control Board → ESP-NOW → Drive Board → Motors
```

### Data Flow
```
User Input → JSON Command → ESP-NOW Packet → Motor Signal → Physical Movement
```

## Hardware Requirements

- ESP32-C3-Mini development board
- Dual H-Bridge motor driver
- 2x / 4x DC motors with wheels 
- LiPo battery or AA batteries
- Chassis (3D printed, Laser Cut parts, etc)
- Basic electronic components
- Soldering is optional

## Vehicle Builds

<div style="display: flex; justify-content: space-between; margin-bottom: 20px;">
    <div style="flex: 1; margin-right: 10px;">
        <h3>Omni-Drive Version</h3>
        <img src="docs/omni_build.jpg" width="100%" alt="RC Car Omni Drive">
        <em>4-wheel omni-directional drive configuration</em>
    </div>
    <div style="flex: 1; margin-left: 10px;">
        <h3>Standard Version</h3>
        <img src="docs/car.jpg" width="100%" alt="RC Car Default">
        <em>2-wheel differential drive configuration</em>
    </div>
</div>

## Pin Configuration

### Default Version (2-Wheel Drive)
| Pin | Function |
|-----|----------|
| 7   | Left Motor PWM |
| 0   | Right Motor PWM |
| 8   | Left Motor Forward |
| 9   | Left Motor Reverse |
| 4   | Right Motor Forward |
| 3   | Right Motor Reverse |

### Omni Version (4-Wheel Drive)
| Motor          | Enable Pin (PWM) | Direction 1 | Direction 2 |
|----------------|------------------|-------------|-------------|
| Left Front     | 5               | 6           | 7           |
| Right Front    | 20              | 21          | 3           |
| Left Back      | 8               | 9           | 10          |
| Right Back     | 2               | 1           | 0           |

## Dual-Board Version

An alternative version using two ESP32 boards with ESP-NOW communication:

### Features
- Long-range control (hundreds of meters)
- Direct peer-to-peer communication
- No WiFi network required
- Split control and drive functionality
- More reliable communication

### Components
- Control Board: Handles web interface and user input
- Drive Board: Controls motors and receives commands
- ESP-NOW protocol for board-to-board communication

### Benefits
- Separation of concerns
- Extended range capability
- Reduced latency
- More reliable motor control
- Independent WiFi and control systems

The code for this version is in:
- `control.ino`: Web interface and ESP-NOW transmitter
- `drive.ino`: Motor control and ESP-NOW receiver

## Web Interface

The interface includes:
- Interactive joystick
- Real-time wheel speed indicators
- Quick control buttons
- Fine-tuning controls
- Status monitoring panel

## Supporting Documentation

### Design Files
<img src="docs/chasis.png" width="400" alt="CAD Model">
*3D CAD model of chassis*

### Control Display
<img src="docs/control_oled.jpg" width="300" alt="OLED Display">
*OLED status display on control board*

### Wiring Diagrams
<img src="docs/omni_wiring.jpg" width="400" alt="Omni Circuit Schematic">
*Omni version wiring*

<img src="docs/circuit.jpg" width="400" alt="Default Circuit Schematic">
*Default version wiring*

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
