#include "web_interface_default.h"

const char DEFAULT_INDEX_HTML[] PROGMEM = R"=====(

<!DOCTYPE html>
<html>
<head>
  <title>Default Mode Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 0;
      padding: 20px;
      background-color: #1a1a1a;
      color: #ffffff;
    }
    .container {
      max-width: 800px;
      margin: 0 auto;
      background: #2d2d2d;
      padding: 25px;
      border-radius: 20px;
      box-shadow: 0 4px 15px rgba(0,0,0,0.2);
    }
    h1 {
      color: #00ee99;
      margin-bottom: 30px;
      font-weight: 600;
    }
    .joystick-container {
      margin: 20px auto;
      width: 300px;
      height: 300px;
      position: relative;
    }
    canvas {
      background: #383838;
      border-radius: 50%;
      touch-action: none;
    }
    .button-group {
      display: flex;
      justify-content: center;
      gap: 10px;
      flex-wrap: wrap;
      margin: 20px 0;
    }
    .button {
      padding: 10px 20px;
      border: none;
      border-radius: 15px;
      color: #1a1a1a;
      font-size: 16px;
      font-weight: 600;
      cursor: pointer;
      min-width: 130px;
      transition: all 0.3s ease;
      font-family: Arial, sans-serif;
    }
    .button:hover {
      transform: translateY(-2px);
    }
    .reverse {
      background: #ff4477;
    }
    .neutral {
      background: #44aaff;
    }
    .forward {
      background: #00ee99;
    }
    .button.stop {
      background: #ff0000;
      font-weight: bold;
      font-size: 18px;
    }
    .slider-container {
      margin: 25px auto;
      padding: 15px;
      background: #383838;
      border-radius: 15px;
      width: 90%;
      max-width: 400px;
    }
    .slider {
      width: 100%;
      margin: 10px 0;
      -webkit-appearance: none;
      height: 20px;
      background: #4d4d4d;
      border-radius: 10px;
      transition: all 0.3s ease;
    }
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 35px;
      height: 35px;
      background: #00ee99;
      border-radius: 50%;
      cursor: pointer;
      box-shadow: 0 0 10px rgba(0,238,153,0.3);
      transition: all 0.3s ease;
    }
    .slider::-webkit-slider-thumb:hover {
      background: #00cc88;
      transform: scale(1.1);
    }
    .tune-label {
      text-align: left;
      margin-top: 10px;
    }

    /* New row around joystick + bars */
    .joystick-row {
      display: flex;
      justify-content: center;
      align-items: center;
      gap: 20px;
      flex-wrap: wrap;
      margin: 20px 0;
    }

    /* Vertical Wheel Bars */
    #leftWheelBar, #rightWheelBar {
      width: 60px;
      height: 220px;
      background: #444;
      border-radius: 8px;
      box-shadow: inset 0 2px 5px rgba(0,0,0,0.5);
      touch-action: none;
    }

    /* Status area */
    #statusArea {
      margin-top: 20px;
      display: none; /* hidden by default, toggled by button */
      background: #222;
      padding: 15px;
      border-radius: 8px;
    }
    #toggleStatusBtn {
      margin: 15px 0 0 0;
      background: #aaaa00;
      color: #1a1a1a;
      font-weight: bold;
    }
  </style>
</head>
<body>
  <div class="container">
    <!-- Joystick + Wheel Bars Row -->
    <div class="joystick-row">
      <canvas id="leftWheelBar" width="60" height="220"></canvas>
      <div class="joystick-container">
        <canvas id="joystick" width="300" height="300"></canvas>
      </div>
      <canvas id="rightWheelBar" width="60" height="220"></canvas>
    </div>

    <!-- Bottom Buttons -->
    <div class="button-group">
      <button class="button reverse" onclick="setReverseMax()">Reverse Max</button>
      <button class="button neutral" onclick="setNeutral()">Neutral</button>
      <button class="button forward" onclick="setForwardMax()">Forward Max</button>
      <button class="button stop" onclick="emergencyStop()">STOP</button>
    </div>

    <!-- Wheel Tuning Sliders -->
    <div class="slider-container">
      <div class="tune-label">Left Wheel Tuning: <span id="leftTuneVal">1.0</span></div>
      <input type="range" id="leftTuneSlider" class="slider" min="0.5" max="1.5" step="0.01" value="1.0">
      
      <div class="tune-label">Right Wheel Tuning: <span id="rightTuneVal">1.0</span></div>
      <input type="range" id="rightTuneSlider" class="slider" min="0.5" max="1.5" step="0.01" value="1.0">
    </div>

    <!-- Toggleable Status Display -->
    <button class="button" id="toggleStatusBtn" onclick="toggleStatus()">Show Status</button>
    <div id="statusArea">
      <h3>Current Status</h3>
      <p>Speed: <span id="speedVal">0</span></p>
      <p>Direction: <span id="directionVal">Forward</span></p>
      <p>Turn Rate: <span id="turnVal">0</span></p>
      <p>Left Wheel PWM: <span id="leftPWMVal">0</span></p>
      <p>Right Wheel PWM: <span id="rightPWMVal">0</span></p>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('joystick');
    const ctx = canvas.getContext('2d');
    let centerX = canvas.width / 2;
    let centerY = canvas.height / 2;
    let knobRadius = 30;
    let outerRadius = 140; // area in which the knob can move

    // Joystick knob position
    let knobX = centerX;
    let knobY = centerY;
    let dragging = false;

    // Keep track of last speed sign
    let lastSpeedValue = 0;
    let lastTurnValue = 0;

    drawJoystick(knobX, knobY);

    function drawJoystick(x, y) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Outer circle
      ctx.beginPath();
      ctx.arc(centerX, centerY, outerRadius, 0, Math.PI * 2);
      ctx.fillStyle = '#383838';
      ctx.fill();

      // Knob
      ctx.beginPath();
      ctx.arc(x, y, knobRadius, 0, Math.PI * 2);
      ctx.fillStyle = '#00ee99';
      ctx.fill();
    }

    // Mouse + Touch events
    canvas.addEventListener('mousedown', startDrag);
    canvas.addEventListener('mousemove', drag);
    canvas.addEventListener('mouseup', endDrag);
    canvas.addEventListener('mouseleave', endDrag);

    canvas.addEventListener('touchstart', startDrag, {passive:false});
    canvas.addEventListener('touchmove', drag, {passive:false});
    canvas.addEventListener('touchend', endDrag, {passive:false});
    canvas.addEventListener('touchcancel', endDrag, {passive:false});

    function getMousePos(evt) {
      let rect = canvas.getBoundingClientRect();
      if (evt.touches) {
        return {
          x: evt.touches[0].clientX - rect.left,
          y: evt.touches[0].clientY - rect.top
        };
      } else {
        return {
          x: evt.clientX - rect.left,
          y: evt.clientY - rect.top
        };
      }
    }

    function startDrag(evt) {
      evt.preventDefault();
      dragging = true;
      moveKnobByEvent(evt);
    }

    function drag(evt) {
      if (!dragging) return;
      evt.preventDefault();
      moveKnobByEvent(evt);
    }

    function endDrag(evt) {
      evt.preventDefault();
      dragging = false;
      // (We do NOT recenter the joystick automatically)
    }

    function moveKnobByEvent(evt) {
      let pos = getMousePos(evt);
      let dx = pos.x - centerX;
      let dy = pos.y - centerY;
      let dist = Math.sqrt(dx*dx + dy*dy);

      if (dist > outerRadius) {
        let angle = Math.atan2(dy, dx);
        pos.x = centerX + outerRadius * Math.cos(angle);
        pos.y = centerY + outerRadius * Math.sin(angle);
      }

      knobX = pos.x;
      knobY = pos.y;
      drawJoystick(knobX, knobY);

      let relX = knobX - centerX; // negative left, positive right
      let relY = knobY - centerY; // negative up, positive down

      // Map X => -50..50 turn, Y => -100..100 speed
      let turnPercent = (relX / outerRadius) * 50;
      let speedPercent = -(relY / outerRadius) * 100; // negative to invert Y

      // Round
      turnPercent = Math.round(turnPercent);
      speedPercent = Math.round(speedPercent);

      // (Optional) Increase minimum speed so motors get enough torque
      if (Math.abs(speedPercent) < 30) {
        speedPercent = 0;
      } else {
        // Remap so that below -30 or above 30 is boosted
        if (speedPercent > 0) {
          // from 30..100 => 50..100
          speedPercent = map(speedPercent, 30, 100, 50, 100);
        } else {
          // from -100..-30 => -100..-50
          speedPercent = map(speedPercent, -100, -30, -100, -50);
        }
        speedPercent = Math.round(speedPercent);
      }

      // Update global last values
      lastSpeedValue = speedPercent;
      lastTurnValue = turnPercent;

      sendJoystickValues(speedPercent, turnPercent);
    }

    function sendJoystickValues(speed, turn) {
      let direction = (speed < 0) ? "Backward" : "Forward";
      let absSpeed = Math.abs(speed);

      fetch(`/setMotor?speed=${absSpeed}&forwardBackward=${direction}&turnRate=${turn}`)
        .catch(err => console.log('Error:', err));

      // Reflect on the side bars
      const [left, right] = getWheelSpeeds(absSpeed, turn, direction);
      drawWheelBars(left, right);
    }

    // Helper to manually set the knob’s position
    // speed: -100..100, turn: -50..50
    function setKnobPosition(speed, turn) {
      let newX = (turn / 50) * outerRadius;   // -1 => -140, 1 => +140
      let newY = -(speed / 100) * outerRadius; // -1 => +140, 1 => -140
      knobX = centerX + newX;
      knobY = centerY + newY;
      drawJoystick(knobX, knobY);
    }

    function emergencyStop() {
      // Reset joystick
      knobX = centerX;
      knobY = centerY;
      drawJoystick(knobX, knobY);

      fetch('/emergencyStop')
        .catch(err => console.error(err));

      // Also clear bars
      drawWheelBars(0, 0);
    }

    function setReverseMax() {
      // speed=100, direction=Backward, turn=0
      setKnobPosition(-100, 0);
      fetch(`/setMotor?speed=100&forwardBackward=Backward&turnRate=0`)
        .catch(err => console.error(err));
      drawWheelBars(-100, -100); 
    }

    function setNeutral() {
      // Keep magnitude from last speed, direction sign
      let direction = (lastSpeedValue < 0) ? "Backward" : "Forward";
      let absSpeed = Math.abs(lastSpeedValue);
      if (absSpeed < 1) absSpeed = 50; // fallback

      setKnobPosition((direction === "Backward") ? -absSpeed : absSpeed, 0);
      fetch(`/setMotor?speed=${absSpeed}&forwardBackward=${direction}&turnRate=0`)
        .catch(err => console.error(err));

      drawWheelBars(absSpeed * (direction === "Backward" ? -1 : 1), 
                    absSpeed * (direction === "Backward" ? -1 : 1));
    }

    function setForwardMax() {
      // speed=100, direction=Forward, turn=0
      setKnobPosition(100, 0);
      fetch(`/setMotor?speed=100&forwardBackward=Forward&turnRate=0`)
        .catch(err => console.error(err));
      drawWheelBars(100, 100);
    }

    const leftTuneSlider  = document.getElementById('leftTuneSlider');
    const rightTuneSlider = document.getElementById('rightTuneSlider');
    const leftTuneVal     = document.getElementById('leftTuneVal');
    const rightTuneVal    = document.getElementById('rightTuneVal');

    leftTuneSlider.oninput = () => {
      leftTuneVal.textContent = leftTuneSlider.value;
      updateWheelTuning();
    };
    rightTuneSlider.oninput = () => {
      rightTuneVal.textContent = rightTuneSlider.value;
      updateWheelTuning();
    };

    function updateWheelTuning() {
      const lt = leftTuneSlider.value;
      const rt = rightTuneSlider.value;
      fetch(`/setWheelTuning?leftTune=${lt}&rightTune=${rt}`)
        .catch(err => console.log('Error:', err));
    }

    function getWheelSpeeds(speed, turn, direction) {
      // speed is 0..100, turn is -50..50
      let left = speed;
      let right = speed;
      if (turn > 0) {
        // turn > 0 => turning right => reduce right
        right = Math.max(speed - turn, 0);
      } else if (turn < 0) {
        // turning left => reduce left
        let t = Math.abs(turn);
        left = Math.max(speed - t, 0);
      }
      if (direction === "Backward") {
        left = -left;
        right = -right;
      }
      return [left, right];
    }

    function drawWheelBars(leftSpeed, rightSpeed) {
      drawSingleBar("leftWheelBar", leftSpeed);
      drawSingleBar("rightWheelBar", rightSpeed);
    }

    function drawSingleBar(canvasId, speedValue) {
      const barCanvas = document.getElementById(canvasId);
      const barCtx = barCanvas.getContext('2d');
      barCtx.clearRect(0, 0, barCanvas.width, barCanvas.height);

      const percentage = Math.min(Math.abs(speedValue), 100) / 100;
      const filled = barCanvas.height * percentage;
      const color = (speedValue >= 0) ? "#1abc9c" : "#e74c3c";

      barCtx.fillStyle = color;
      barCtx.fillRect(0, barCanvas.height - filled, barCanvas.width, filled);
    }

    // Reusable map function
    function map(x, in_min, in_max, out_min, out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    let statusVisible = false;
    const statusArea = document.getElementById("statusArea");
    const toggleBtn = document.getElementById("toggleStatusBtn");

    function toggleStatus() {
      statusVisible = !statusVisible;
      statusArea.style.display = statusVisible ? "block" : "none";
      toggleBtn.textContent = statusVisible ? "Hide Status" : "Show Status";
    }

    // Periodically fetch current status
    setInterval(() => {
      if (!statusVisible) return; // only poll if visible
      fetch("/getStatus")
        .then(response => response.json())
        .then(data => {
          // Update speed/direction
          document.getElementById("speedVal").textContent = Math.abs(data.speed);
          document.getElementById("directionVal").textContent = (data.speed < 0) ? "Backward" : "Forward";
          document.getElementById("turnVal").textContent = data.turn;
          document.getElementById("leftPWMVal").textContent = data.leftPWM;
          document.getElementById("rightPWMVal").textContent = data.rightPWM;
        })
        .catch(err => console.log("Status Error:", err));
    }, 500);  // Fetch status every 500ms

  </script>
</body>
</html>

)=====";
