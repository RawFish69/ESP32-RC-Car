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

## Hardware Requirements

- ESP32-C3-Mini development board
- Dual H-Bridge motor driver
- 2x / 4x DC motors with wheels 
- LiPo battery or AA batteries
- Chassis (3D printed, Laser Cut parts, etc)
- Basic electronic components
- Soldering is optional

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
![RC Car Omni Drive](docs/omni_build.jpg)
*Omni version build*

![RC Car Default](docs/car.jpg)
*Default version build*

### Circuit Wiring
![Omni Circuit Schematic](docs/omni_wiring.jpg)
*Omni version wiring*

![Default Circuit Schematic](docs/circuit.jpg)
*Defualt version wiring*

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
