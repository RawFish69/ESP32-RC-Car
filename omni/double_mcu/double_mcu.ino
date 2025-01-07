/*
    Double Microcontroller Approach
    Current idea: 
    - One microcontroller is responsible for the motor control
    - The other microcontroller is responsible for the server and client communication
    - Steering commands are sent to the motor controller via ESP-NOW
    - The motor controller will then control the motors based on the steering commands
*/