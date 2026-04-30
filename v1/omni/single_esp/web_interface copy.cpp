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
    <div class="joystick-container">
      <canvas id="joystick" width="300" height="300"></canvas>
    </div>
    <div class="controls-container">
      <button class="grid-button rotation-button" onclick="rotateLeft()">↺</button>
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
      <button class="grid-button rotation-button" onclick="rotateRight()">↻</button>
    </div>
    <div class="button-group">
      <button class="button reverse" onclick="setReverseMax()">Reverse</button>
      <button class="button stop" onclick="emergencyStop()">STOP</button>
      <button class="button forward" onclick="setForwardMax()">Forward</button>
    </div>
    <div class="slider-container">
      <div class="tune-label">Left Wheel Tuning: <span id="leftTuneVal">1.0</span></div>
      <input type="range" id="leftTuneSlider" class="slider" min="0.5" max="1.5" step="0.01" value="1.0">
      <div class="tune-label">Right Wheel Tuning: <span id="rightTuneVal">1.0</span></div>
      <input type="range" id="rightTuneSlider" class="slider" min="0.5" max="1.5" step="0.01" value="1.0">
    </div>
  </div>
  <script>
    const canvas = document.getElementById('joystick');
    const ctx = canvas.getContext('2d');
    let centerX = canvas.width / 2;
    let centerY = canvas.height / 2;
    let knobRadius = 30;
    let outerRadius = 140;
    let knobX = centerX;
    let knobY = centerY;
    let dragging = false;
    drawJoystick(knobX, knobY);
    function drawJoystick(x, y) {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.beginPath();
      ctx.arc(centerX, centerY, outerRadius, 0, Math.PI * 2);
      ctx.fillStyle = '#383838';
      ctx.fill();
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
      knobX = centerX;
      knobY = centerY;
      drawJoystick(knobX, knobY);
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
      let turnPercent = (relX / outerRadius) * 50;
      let speedPercent = -(relY / outerRadius) * 100;
      turnPercent = Math.round(turnPercent);
      speedPercent = Math.round(speedPercent);
      if (Math.abs(speedPercent) < 30) {
        speedPercent = 0;
      } else {
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
