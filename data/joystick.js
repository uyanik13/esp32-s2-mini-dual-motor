let throttleTimeout = null;

class Joystick {
  constructor(canvas, ctx, radius, socket) {  // Add socket as a parameter
    this.canvas = canvas;
    this.ctx = ctx;
    this.radius = radius;
    this.socket = socket;  // Save the socket reference
    this.x_orig = canvas.width / 2;
    this.y_orig = canvas.height / 2;
    this.coord = { x: canvas.width / 2, y: canvas.height / 2 };
    this.paint = false;
    this.throttleDelay = 100;
    this.needsRedraw = true;
    this.animationFrame = null;
    this.init();
  }

  init() {
    this.addEventListeners();
    this.draw();  // Draw the joystick on initialization
  }

  addEventListeners() {
    const events = ["mousedown", "mouseup", "mousemove", "touchstart", "touchend", "touchcancel", "touchmove"];
    events.forEach(event => this.canvas.addEventListener(event, this.handleEvent.bind(this)));
  }

  handleEvent(event) {
    event.preventDefault();
    switch (event.type) {
      case "mousedown":
      case "touchstart":
        this.paint = true;
        this.getPosition(event);
        this.needsRedraw = true;
        this.draw();
        break;
      case "mouseup":
      case "touchend":
      case "touchcancel":
        this.stopDrawing();
        break;
      case "mousemove":
      case "touchmove":
        if (this.paint) {
          this.getPosition(event);
          this.needsRedraw = true;
          this.draw();
        }
        break;
    }
  }

  getPosition(event) {
    const clientX = event.clientX || (event.touches && event.touches[0].clientX) || 0;
    const clientY = event.clientY || (event.touches && event.touches[0].clientY) || 0;
    this.coord.x = clientX - this.canvas.offsetLeft;
    this.coord.y = clientY - this.canvas.offsetTop;
  }

  isWithinCircle() {
    return Math.hypot(this.coord.x - this.x_orig, this.coord.y - this.y_orig) <= this.radius;
  }

  draw() {
    if (!this.needsRedraw) return;

    const { coord, x_orig, y_orig, radius, ctx } = this;
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);  // Clear the canvas
    this.drawBackground();  // Draw the joystick's background

    let x = coord.x;
    let y = coord.y;

    const dx = x - x_orig;
    const dy = y - y_orig;
    const angle = Math.atan2(-dy, dx);  // Flip dy for North-South correction
    const distance = Math.min(Math.hypot(dx, dy), radius);  // Cap the distance to the radius
    const speed = Math.round((100 * distance) / radius);  // Map distance to speed (0-100%)
    const angleDegrees = (angle * 180) / Math.PI;  // Convert radians to degrees
    const correctedAngle = (angleDegrees + 360) % 360;  // Ensure positive angle (0-360)
    const direction = this.getDirection(correctedAngle);  // Get the joystick direction
    console.log(`Angle: ${correctedAngle}, Direction: ${direction}, Speed: ${speed}`);

    // Ensure the joystick stays within bounds of the circle
    if (distance > radius) {
      x = x_orig + radius * Math.cos(angle);
      y = y_orig + radius * Math.sin(angle);
    }

    this.drawJoystick(x, y, direction);  // Draw the joystick
    this.needsRedraw = false;  // Reset redraw flag
  }

  stopDrawing() {
    this.paint = false;
    this.resetJoystick();
    
    // Send reset command to WebSocket
    this.socket.sendJoystickData({
      direction: "C",
      speed: 0,
      angle: 500
    });
  }

  resetJoystick() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    this.drawBackground();
    this.drawJoystick(this.canvas.width / 2, this.canvas.height / 2, "None");
  }

  drawBackground() {
    const { ctx, x_orig, y_orig, radius } = this;
    ctx.fillStyle = "#ECE5E5";
    ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.beginPath();
    ctx.arc(x_orig, y_orig, radius + 20, 0, Math.PI * 2, true);
    ctx.fill();
  }

  drawJoystick(x, y, direction) {
    this.ctx.save();  // Save the current state of the canvas context
    this.ctx.fillStyle = "#F08080";
    this.ctx.shadowBlur = 20;
    this.ctx.shadowColor = this.directionShadowColor(direction);
    this.ctx.beginPath();
    this.ctx.arc(x, y, this.radius, 0, Math.PI * 2, true);
    this.ctx.fill();
    this.ctx.restore();  // Restore the previous state
  }

  directionShadowColor(direction) {
    const colors = {
      N: "#FF0000",
      S: "#00FF00",
      E: "#0000FF",
      W: "#FFFF00",
      NE: "#FF00FF",
      NW: "#00FFFF",
      SE: "#FFAA00",
      SW: "#AAFF00",
      None: "#F08080",
    };
    return colors[direction] || colors.None;
  }

  getDirection(angle) {
    if (angle >= 337.5 || angle < 22.5) return "E";    
    if (angle >= 22.5 && angle < 67.5) return "NE";   
    if (angle >= 67.5 && angle < 112.5) return "N";    
    if (angle >= 112.5 && angle < 157.5) return "NW";  
    if (angle >= 157.5 && angle < 202.5) return "W";    
    if (angle >= 202.5 && angle < 247.5) return "SW";   
    if (angle >= 247.5 && angle < 292.5) return "S";    
    if (angle >= 292.5 && angle < 337.5) return "SE";   
    return "None";
  }

  calculateJoystickData(dx, dy) {
    const distance = Math.hypot(dx, dy);
    const radiusThreshold = 0.1 * this.radius;
    const speed = Math.round((100 * distance) / this.radius);

    if (distance <= radiusThreshold) {
      return { direction: "None", speed: 0 };
    }

    let angle = Math.atan2(-dy, dx) * (180 / Math.PI);  // Flip dy for North-South correction
    angle = (angle + 360) % 360;
    const direction = this.getDirection(angle);
    console.log(`Direction: ${direction}, Speed: ${speed}`, 'angle:', angle);

    return { direction, speed, angle };
  }
}

class WebSocketComponent {
  constructor(url) {
    this.socket = new WebSocket(url);
    this.init();
  }

  init() {
    this.socket.onopen = () => {
      console.log("WebSocket connection established");
    };

    this.socket.onmessage = (event) => {
      console.log("Received data from server:", event.data);
    };

    this.socket.onclose = () => {
      console.log("WebSocket connection closed");
    };

    this.socket.onerror = (error) => {
      console.log("WebSocket error:", error);
    };
  }

  sendJoystickData(joystickData) {
    const message = JSON.stringify({
      direction: joystickData.direction,
      speed: joystickData.speed,
      angle: joystickData.angle
    });

    if (this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(message);
      console.log("Sent joystick data:", message);
    }
  }
}

class CanvasManager {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext("2d");
    this.joystick = null;
    this.socket = null;
  }

  init() {
    this.resize();
    this.socket = new WebSocketComponent(`ws://${window.location.hostname}/ws`); // Create WebSocket instance
    this.joystick = new Joystick(this.canvas, this.ctx, Math.min(this.canvas.width, this.canvas.height) / 4, this.socket);  // Pass WebSocket to Joystick

    window.addEventListener("resize", this.resize.bind(this));
    document.addEventListener("mousemove", this.handleJoystickMovement.bind(this));
    document.addEventListener("touchmove", this.handleJoystickMovement.bind(this));

  }

  destroy() {
    window.removeEventListener("resize", this.resize.bind(this));
    document.removeEventListener("mousemove", this.handleJoystickMovement.bind(this));
    document.removeEventListener("touchmove", this.handleJoystickMovement.bind(this));

    // Close WebSocket if open
    if (this.socket && this.socket.socket.readyState !== WebSocket.CLOSED) {
      this.socket.socket.close();
    }
  }

  resize() {
    this.canvas.width = window.innerWidth;
    this.canvas.height = window.innerHeight;
    if (this.joystick) {
      this.joystick.resetJoystick();
      this.joystick.draw();
    }
  }

  handleJoystickMovement(event) {
    this.joystick.getPosition(event);  // Track joystick movement
    if (!this.joystick.animationFrame) {
      this.joystick.animationFrame = requestAnimationFrame(() => {
        const dx = this.joystick.coord.x - this.joystick.x_orig;
        const dy = this.joystick.coord.y - this.joystick.y_orig;
        const joystickData = this.joystick.calculateJoystickData(dx, dy);
        this.socket.sendJoystickData(joystickData);  // Send data via WebSocket
        this.joystick.draw();  // Redraw joystick position
        this.joystick.animationFrame = null;
      });
    }
  }
}

// Initialize the Canvas and Joystick on page load
window.addEventListener("load", () => {
  const canvasManager = new CanvasManager("canvas");
  canvasManager.init();
});

// Emergency stop
function emergencyStop() {
  fetch('/emergency_stop')
    .then((response) => response.text())
    .then((data) => console.log("Server:", data))
    .catch((error) => console.error("Fetch error:", error));
}


