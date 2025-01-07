#include "web_interface.h"

const char INDEX_HTML[] = R"=====(

<!DOCTYPE html>
<html>
<head>
  <title>ESP32 RC Car</title>
  <meta charset="UTF-8">
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
    .reverse { background: #ff4477; }
    .neutral { background: #44aaff; }
    .forward { background: #00ee99; }
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
    .controls-container {
      display: flex;
      justify-content: center;
      align-items: center;
      gap: 15px;
      margin: 20px auto;
      max-width: 600px; 
    }
    .grid-buttons {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      background: #333;
      padding: 20px;
      border-radius: 15px;
      width: 320px;
      margin: 0;
      box-shadow: 0 4px 8px rgba(0,0,0,0.2);
    }
    .grid-button {
      padding: 15px 10px;
      border: none;
      border-radius: 12px;
      background: linear-gradient(145deg, #3a3a3a, #2d2d2d);
      color: #00ee99;
      font-size: 16px;
      font-weight: bold;
      cursor: pointer;
      transition: all 0.2s ease;
      box-shadow: 3px 3px 5px #252525, -3px -3px 5px #454545;
    }
    .grid-button:hover {
      background: linear-gradient(145deg, #2d2d2d, #3a3a3a);
      transform: translateY(-2px);
      color: #00ff99;
    }
    .grid-button:active {
      box-shadow: inset 2px 2px 5px #252525, inset -2px -2px 5px #454545;
      transform: translateY(0);
    }
    .rotation-button {
      width: 80px;
      height: 80px;
      padding: 0;
      font-size: 32px;
      color: #44aaff;
      display: flex;
      align-items: center;
      justify-content: center;
      border-radius: 50%;
    }
    .joysticks-row {
      display: flex;
      justify-content: space-around;
      flex-wrap: wrap;
      margin: 20px 0;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP32 RC Car</h1>

    <!-- Two Joysticks side-by-side -->
    <div class="joysticks-row">
      <div class="joystick-container">
        <canvas id="joystick" width="300" height="300"></canvas>
      </div>
      <div class="joystick-container">
        <canvas id="joystick2" width="300" height="300"></canvas>
      </div>
    </div>

    <!-- Controls layout -->
    <div class="controls-container">
      <!-- Left rotation -->
      <button class="grid-button rotation-button" onclick="rotateLeft()">↺</button>

      <!-- Main directional grid -->
      <div class="grid-buttons">
        <button class="grid-button" onclick="moveNW()">↖</button>
        <button class="grid-button" onclick="moveUp()">↑</button>
        <button class="grid-button" onclick="moveNE()">↗</button>
        <button class="grid-button" onclick="moveLeft()">←</button>
        <button class="grid-button" onclick="rotateCenter()">⊙</button>
        <button class="grid-button" onclick="moveRight()">→</button>
        <button class="grid-button" onclick="moveSW()">↙</button>
        <button class="grid-button" onclick="moveDown()">↓</button>
        <button class="grid-button" onclick="moveSE()">↘</button>
      </div>

      <!-- Right rotation -->
      <button class="grid-button rotation-button" onclick="rotateRight()">↻</button>
    </div>

    <!-- Forward/Reverse/Stop row -->
    <div class="button-group">
      <button class="button reverse" onclick="setReverseMax()">Reverse</button>
      <button class="button stop" onclick="emergencyStop()">STOP</button>
      <button class="button forward" onclick="setForwardMax()">Forward</button>
    </div>

    <!-- Wheel Tuning Sliders -->
    <div class="slider-container">
      <div class="tune-label">Left Wheel Tuning: <span id="leftTuneVal">1.0</span></div>
      <input type="range" id="leftTuneSlider" class="slider" min="0.5" max="1.5" step="0.01" value="1.0">
      
      <div class="tune-label">Right Wheel Tuning: <span id="rightTuneVal">1.0</span></div>
      <input type="range" id="rightTuneSlider" class="slider" min="0.5" max="1.5" step="0.01" value="1.0">
    </div>
  </div>

  <script>
    /********************************
     *       JOYSTICK #1 (left)     *
     ********************************/
    const canvas = document.getElementById('joystick');
    const ctx = canvas.getContext('2d');
    let centerX = canvas.width / 2;
    let centerY = canvas.height / 2;
    let knobRadius = 30;
    let outerRadius = 140; // area in which the knob can move

    let knobX = centerX;
    let knobY = centerY;
    let dragging = false;
    let activeJoystick = 0; // 0=none, 1=left, 2=right

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

    canvas.addEventListener('mousedown', startDrag);
    canvas.addEventListener('mousemove', drag);
    canvas.addEventListener('mouseup', endDrag);
    canvas.addEventListener('mouseleave', endDrag);

    canvas.addEventListener('touchstart', startDrag, {passive:false});
    canvas.addEventListener('touchmove', drag, {passive:false});
    canvas.addEventListener('touchend', endDrag, {passive:false});
    canvas.addEventListener('touchcancel', endDrag, {passive:false});

    function startDrag(evt) {
      evt.preventDefault();
      if (activeJoystick === 2) return; // If right joystick is active, block
      activeJoystick = 1;
      dragging = true;
      // Reset second joystick
      knob2X = center2X;
      knob2Y = center2Y;
      drawJoystick2(knob2X, knob2Y);
      moveKnobByEvent(evt);
    }
    function drag(evt) {
      if (!dragging || activeJoystick !== 1) return;
      evt.preventDefault();
      moveKnobByEvent(evt);
    }
    function endDrag(evt) {
      evt.preventDefault();
      if (activeJoystick === 1) {
        activeJoystick = 0;
      }
      dragging = false;
      // Snap back
      knobX = centerX;
      knobY = centerY;
      drawJoystick(knobX, knobY);
      // Send 0
      sendJoystickValues(0, 0);
    }

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

      let relX = knobX - centerX;  
      let relY = knobY - centerY;  

      // Convert to speed/turn
      let turnPercent = (relX / outerRadius) * 50;       // -50..50
      let speedPercent = -(relY / outerRadius) * 100;    // up => positive

      turnPercent = Math.round(turnPercent);
      speedPercent = Math.round(speedPercent);

      // Deadzone for speed
      if (Math.abs(speedPercent) < 30) {
        speedPercent = 0;
      } else {
        // map 30..100 => 50..100
        if (speedPercent > 0) {
          speedPercent = mapRange(speedPercent, 30, 100, 50, 100);
        } else {
          speedPercent = mapRange(speedPercent, -100, -30, -100, -50);
        }
        speedPercent = Math.round(speedPercent);
      }

      sendJoystickValues(speedPercent, turnPercent);
    }

    function sendJoystickValues(speed, turn) {
      let direction = (speed >= 0) ? "Forward" : "Backward";
      let absSpeed = Math.abs(speed);
      fetch(`/setMotor?speed=${absSpeed}&forwardBackward=${direction}&turnRate=${turn}`)
        .catch(err => console.log('Error:', err));
    }

    function mapRange(x, inMin, inMax, outMin, outMax) {
      return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    /********************************
     *       JOYSTICK #2 (right)    *
     ********************************/
    const canvas2 = document.getElementById('joystick2');
    const ctx2 = canvas2.getContext('2d');
    let center2X = canvas2.width / 2;
    let center2Y = canvas2.height / 2;
    let knob2Radius = 30;
    let outer2Radius = 140;
    let dragging2 = false;
    let knob2X = center2X;
    let knob2Y = center2Y;

    drawJoystick2(knob2X, knob2Y);

    canvas2.addEventListener('mousedown', startDrag2);
    canvas2.addEventListener('mousemove', drag2);
    canvas2.addEventListener('mouseup', endDrag2);
    canvas2.addEventListener('mouseleave', endDrag2);

    canvas2.addEventListener('touchstart', startDrag2, {passive:false});
    canvas2.addEventListener('touchmove', drag2, {passive:false});
    canvas2.addEventListener('touchend', endDrag2, {passive:false});
    canvas2.addEventListener('touchcancel', endDrag2, {passive:false});

    function drawJoystick2(x, y) {
      ctx2.clearRect(0, 0, canvas2.width, canvas2.height);
      ctx2.beginPath();
      ctx2.arc(center2X, center2Y, outer2Radius, 0, Math.PI * 2);
      ctx2.fillStyle = '#383838';
      ctx2.fill();

      ctx2.beginPath();
      ctx2.arc(x, y, knob2Radius, 0, Math.PI * 2);
      ctx2.fillStyle = '#00ee99';
      ctx2.fill();
    }

    function startDrag2(evt) {
      evt.preventDefault();
      if (activeJoystick === 1) return; // if left joystick in use
      activeJoystick = 2;
      dragging2 = true;
      // Reset first joystick
      knobX = centerX;
      knobY = centerY;
      drawJoystick(knobX, knobY);
      moveKnobByEvent2(evt);
    }
    function drag2(evt) {
      if (!dragging2 || activeJoystick !== 2) return;
      evt.preventDefault();
      moveKnobByEvent2(evt);
    }
    function endDrag2(evt) {
      evt.preventDefault();
      if (activeJoystick === 2) {
        activeJoystick = 0;
      }
      dragging2 = false;
      // Snap back
      knob2X = center2X;
      knob2Y = center2Y;
      drawJoystick2(knob2X, knob2Y);
      // Send 0
      fetch('/setMotor?x=0&y=0&rotate=0').catch(err => console.error(err));
    }

    function getMousePos2(evt) {
      let rect = canvas2.getBoundingClientRect();
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

    function moveKnobByEvent2(evt) {
      let pos = getMousePos2(evt);
      let dx = pos.x - center2X;
      let dy = pos.y - center2Y;
      let dist = Math.sqrt(dx*dx + dy*dy);
      if (dist > outer2Radius) {
        let angle = Math.atan2(dy, dx);
        pos.x = center2X + outer2Radius * Math.cos(angle);
        pos.y = center2Y + outer2Radius * Math.sin(angle);
      }
      knob2X = pos.x;
      knob2Y = pos.y;
      drawJoystick2(knob2X, knob2Y);
      // If you want to interpret these as strafe vs rotate, do it here
      // For now, it just sets some joystick position.
    }

    /********************************
     *      Grid / Button controls   *
     ********************************/
    function moveNW() { 
      fetch('/setMotor?x=-50&y=50&rotate=0').catch(err => console.error(err));
    }
    function moveUp() { 
      fetch('/setMotor?x=0&y=100&rotate=0').catch(err => console.error(err));
    }
    function moveNE() { 
      fetch('/setMotor?x=50&y=50&rotate=0').catch(err => console.error(err));
    }
    function moveLeft() {
      fetch('/setMotor?x=-100&y=0&rotate=0').catch(err => console.error(err));
    }
    function rotateCenter() {
      fetch('/setMotor?x=0&y=0&rotate=0').catch(err => console.error(err));
    }
    function moveRight() {
      fetch('/setMotor?x=100&y=0&rotate=0').catch(err => console.error(err));
    }
    function moveSW() { 
      fetch('/setMotor?x=-50&y=-50&rotate=0').catch(err => console.error(err));
    }
    function moveDown() { 
      fetch('/setMotor?x=0&y=-100&rotate=0').catch(err => console.error(err));
    }
    function moveSE() { 
      fetch('/setMotor?x=50&y=-50&rotate=0').catch(err => console.error(err));
    }
    function rotateLeft() {
      fetch('/setMotor?x=0&y=0&rotate=-70').catch(err => console.error(err));
    }
    function rotateRight() {
      fetch('/setMotor?x=0&y=0&rotate=70').catch(err => console.error(err));
    }

    /********************************
     *   Forward/Reverse/Stop btns   *
     ********************************/
    function setReverseMax() {
      fetch('/setMotor?speed=100&forwardBackward=Backward&turnRate=0')
        .catch(err => console.error(err));
    }
    function emergencyStop() {
      fetch('/emergencyStop').catch(err => console.error(err));
    }
    function setForwardMax() {
      fetch('/setMotor?speed=100&forwardBackward=Forward&turnRate=0')
        .catch(err => console.error(err));
    }

    /********************************
     *     Wheel Tuning Sliders     *
     ********************************/
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
  </script>
</body>
</html>

)=====";
