#!/usr/bin/env python3
import threading
import time
from flask import Flask, render_template_string, jsonify
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Shared state
crosshair_state = {
    "x": 0.0,
    "y": 0.0,
    "max_range": 500,
    "frozen": False,
    "frozen_x": 0.0,
    "frozen_y": 0.0,
    "robot_x": 0.0,  # new dynamic robot position
    "robot_y": 0.0,
    "axis0": 0.0,
    "axis1": 0.0,
}

# === Flask App ===
app = Flask(__name__)

@app.route('/')
def index():
    return render_template_string('''
<!DOCTYPE html>
<html lang="it">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Mirino Obsidian - Joystick Control</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f0f23 100%);
            font-family: 'Consolas', 'Monaco', monospace;
            height: 100vh;
            overflow: hidden;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        
        .container {
            width: 80vw;
            height: 70vh;
            background: rgba(30, 30, 50, 0.8);
            border: 2px solid #4a4a6a;
            border-radius: 15px;
            position: relative;
            box-shadow: 
                0 0 30px rgba(100, 100, 150, 0.3),
                inset 0 0 20px rgba(0, 0, 0, 0.5);
            backdrop-filter: blur(10px);
        }
        
        .alpine-title {
            position: absolute;
            top: 20px;
            left: 50%;
            transform: translateX(-50%);
            color: white;
            font-size: 24px;
            text-shadow: 0 0 10px rgba(255,255,255,0.7);
            z-index: 20;
        }
        
        .crosshair {
            position: absolute;
            width: 40px;
            height: 40px;
            left: 50%;
            top: 50%;
            transform: translate(-50%, -50%);
            transition: all 0.05s ease;
            z-index: 30;
        }
        
        .crosshair::before,
        .crosshair::after {
            content: '';
            position: absolute;
            background: #00ff88;
            box-shadow: 0 0 10px #00ff88;
        }
        
        .crosshair::before {
            width: 40px;
            height: 2px;
            top: 50%;
            left: 0;
            transform: translateY(-50%);
        }
        
        .crosshair::after {
            width: 2px;
            height: 40px;
            left: 50%;
            top: 0;
            transform: translateX(-50%);
        }
        
        .center-dot {
            position: absolute;
            width: 6px;
            height: 6px;
            background: #ff4444;
            border-radius: 50%;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            box-shadow: 0 0 8px #ff4444;
        }
        
        .coordinates {
            position: absolute;
            top: 20px;
            left: 20px;
            color: #00ff88;
            font-size: 16px;
            text-shadow: 0 0 5px #00ff88;
            z-index: 20;
        }
        
        .joystick-info {
            position: absolute;
            top: 60px;
            left: 20px;
            color: #8888aa;
            font-size: 12px;
            z-index: 20;
        }
        
        .controls {
            position: absolute;
            bottom: 20px;
            right: 20px;
            color: #8888aa;
            font-size: 12px;
            text-align: right;
            z-index: 20;
        }
        
        .grid {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            opacity: 0.1;
            background-image: 
                linear-gradient(rgba(100, 100, 150, 0.3) 1px, transparent 1px),
                linear-gradient(90deg, rgba(100, 100, 150, 0.3) 1px, transparent 1px);
            background-size: 30px 30px;
        }
        
        .status-indicator {
            position: absolute;
            top: 20px;
            right: 20px;
            width: 12px;
            height: 12px;
            background: #ff4444;
            border-radius: 50%;
            box-shadow: 0 0 8px #ff4444;
            z-index: 20;
        }
        
        .status-indicator.connected {
            background: #00ff88;
            box-shadow: 0 0 8px #00ff88;
        }
        
        /* Arganelli rossi */
        .winch {
            position: absolute;
            width: 40px;
            height: 25px;
            background: #ff3333;
            border: 2px solid #ff6666;
            border-radius: 4px;
            box-shadow: 0 0 10px rgba(255, 51, 51, 0.5);
            top: 60px;
            z-index: 20;
        }
        
        .winch-left {
            left: 80px;
        }
        
        .winch-right {
            right: 80px;
        }
        
        .winch::before {
            content: '';
            position: absolute;
            width: 6px;
            height: 6px;
            background: #ffaaaa;
            border-radius: 50%;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
        }
        
        /* Robot triangolare viola */
        .robot {
            position: absolute;
            left: 50%;
            top: 50%;
            transform: translate(-50%, -50%);
            width: 0;
            height: 0;
            border-left: 20px solid transparent;
            border-right: 20px solid transparent;
            border-bottom: 35px solid #8833ff;
            filter: drop-shadow(0 0 8px rgba(136, 51, 255, 0.6));
            z-index: 20;
        }
        
        .robot::before {
            content: '';
            position: absolute;
            left: -15px;
            top: 25px;
            width: 0;
            height: 0;
            border-left: 15px solid transparent;
            border-right: 15px solid transparent;
            border-top: 10px solid #aa66ff;
        }
        
        /* Linee tratteggiate migliorate */
        .dashed-line {
            position: absolute;
            border-top: 2px dashed #66aaff;
            opacity: 0.7;
            filter: drop-shadow(0 0 3px rgba(102, 170, 255, 0.3));
            transform-origin: left center;
            z-index: 10;
        }
        
        .line-left {
            /* Posizionata dinamicamente via JavaScript */
        }
        
        .line-right {
            /* Posizionata dinamicamente via JavaScript */
        }
        
        /* Linee tratteggiate verso il mirino */
        .crosshair-line {
            position: absolute;
            border-top: 2px dashed #55ccff;
            opacity: 0.7;
            filter: drop-shadow(0 0 3px rgba(85, 204, 255, 0.5));
            transform-origin: left center;
            z-index: 15;
        }
        
        .crosshair-line-left {
            /* Posizionata dinamicamente via JavaScript */
        }
        
        .crosshair-line-right {
            /* Posizionata dinamicamente via JavaScript */
        }
        
        /* Vettore distanza */
        .distance-vector {
            position: absolute;
            height: 2px;
            background: linear-gradient(90deg, #ffaa00, #ff6600);
            box-shadow: 0 0 6px rgba(255, 170, 0, 0.8);
            transform-origin: left center;
            opacity: 0.9;
            transition: all 0.05s ease;
            z-index: 25;
        }
        
        .distance-vector::after {
            content: '';
            position: absolute;
            right: -8px;
            top: -4px;
            width: 0;
            height: 0;
            border-left: 10px solid #ff6600;
            border-top: 5px solid transparent;
            border-bottom: 5px solid transparent;
            filter: drop-shadow(0 0 4px rgba(255, 102, 0, 0.6));
        }
        
        .distance-label {
            position: absolute;
            color: #ffaa00;
            font-size: 11px;
            font-weight: bold;
            text-shadow: 0 0 4px rgba(255, 170, 0, 0.8);
            white-space: nowrap;
            background: rgba(0, 0, 0, 0.3);
            padding: 2px 6px;
            border-radius: 3px;
            backdrop-filter: blur(5px);
            transition: all 0.05s ease;
            z-index: 25;
        }
    </style>
</head>
<body>
    <div class="container" id="container">
        <div class="alpine-title">alpine robot</div>
        <div class="grid"></div>
        
        <!-- Arganelli rossi -->
        <div class="winch winch-left" title="Arganello Sinistro"></div>
        <div class="winch winch-right" title="Arganello Destro"></div>
        
        <!-- Robot triangolare viola -->
        <div class="robot" title="Robot"></div>
        
        <!-- Linee tratteggiate di collegamento -->
        <div class="dashed-line line-left" id="lineLeft"></div>
        <div class="dashed-line line-right" id="lineRight"></div>
        
        <!-- Nuove linee tratteggiate verso il mirino -->
        <div class="crosshair-line crosshair-line-left" id="crosshairLineLeft"></div>
        <div class="crosshair-line crosshair-line-right" id="crosshairLineRight"></div>
        
        <!-- Vettore distanza -->
        <div class="distance-vector" id="distanceVector"></div>
        <div class="distance-label" id="distanceLabel">ΔX: 0, ΔY: 0</div>
        
        <div class="crosshair" id="crosshair">
            <div class="center-dot"></div>
        </div>
        <div class="coordinates" id="coordinates">X: 0, Y: 0</div>
        <div class="joystick-info" id="joystick-info">
            Axis 0: 0.000<br>
            Axis 1: 0.000
        </div>
        <div class="controls">
            Controllo tramite Joystick ROS2<br>
            Topic: /joy | Assi: 0 (X), 1 (Y)<br>
            🔴 Arganelli | 🔷 Robot | ⊹ Mirino | → Vettore<br>
            ⎯ Linee blu: collegamento arganelli-mirino<br>
            FREEZE: Axis4 = -1
        </div>
        <div class="status-indicator" id="status"></div>
    </div>
    
    <script>
        const crosshair = document.getElementById('crosshair');
        const coordinates = document.getElementById('coordinates');
        const joystickInfo = document.getElementById('joystick-info');
        const statusIndicator = document.getElementById('status');
        const container = document.getElementById('container');
        const lineLeft = document.getElementById('lineLeft');
        const lineRight = document.getElementById('lineRight');
        const crosshairLineLeft = document.getElementById('crosshairLineLeft');
        const crosshairLineRight = document.getElementById('crosshairLineRight');
        const distanceVector = document.getElementById('distanceVector');
        const distanceLabel = document.getElementById('distanceLabel');
        
        let lastUpdate = Date.now();
        let currentState = { x: 0, y: 0 };

        function updateGeometry(crosshairX, crosshairY, robotX, robotY) {
            const containerRect = container.getBoundingClientRect();
            const containerWidth = containerRect.width;
            const containerHeight = containerRect.height;

            // Arrow triangle dimensions (match your CSS exactly!)
            const arrowHeight = 35; // height from CSS (border-bottom: 35px solid)
            const tipOffsetY = arrowHeight / 2; // vertical offset from center to tip (upward)

            // Robot's center position
            const robotCenterX = containerWidth / 2 + robotX;
            const robotCenterY = containerHeight / 2 + robotY;

            // Arrow tip exact position
            const arrowTipX = robotCenterX;
            const arrowTipY = robotCenterY - tipOffsetY;

            // Position the purple arrow (CSS triangle visual)
            const robot = document.querySelector('.robot');
            robot.style.left = robotCenterX + 'px';
            robot.style.top = (robotCenterY - tipOffsetY) + 'px';  // Arrow top-left CSS position adjustment

            // Position the crosshair exactly at the arrow tip by default
            const crosshair = document.getElementById('crosshair');
            crosshair.style.left = (arrowTipX + crosshairX) + 'px';
            crosshair.style.top = (arrowTipY + crosshairY) + 'px';
            crosshair.style.transform = `translate(-50%, -50%)`;

            // Winches position (fixed)
            const winchLeftX = 100;
            const winchLeftY = 72.5;
            const winchRightX = containerWidth - 100;
            const winchRightY = 72.5;

            // Dashed lines from winches directly to arrow tip
            updateDashedLine(lineLeft, winchLeftX, winchLeftY, arrowTipX, arrowTipY);
            updateDashedLine(lineRight, winchRightX, winchRightY, arrowTipX, arrowTipY);

            // Vector from arrow tip to displaced crosshair (useful if joystick moves crosshair)
            const deltaX = crosshairX;
            const deltaY = crosshairY;
            const vectorDistance = Math.sqrt(deltaX**2 + deltaY**2);
            const vectorAngle = Math.atan2(deltaY, deltaX) * 180 / Math.PI;

            if (vectorDistance > 5) {
                distanceVector.style.display = 'block';
                distanceVector.style.left = arrowTipX + 'px';
                distanceVector.style.top = arrowTipY + 'px';
                distanceVector.style.width = vectorDistance + 'px';
                distanceVector.style.transform = `rotate(${vectorAngle}deg)`;

                const labelX = arrowTipX + deltaX * 0.5;
                const labelY = arrowTipY + deltaY * 0.5 - 20;

                distanceLabel.style.display = 'block';
                distanceLabel.style.left = labelX + 'px';
                distanceLabel.style.top = labelY + 'px';
                distanceLabel.textContent = `ΔX: ${Math.round(deltaX)}, ΔY: ${Math.round(deltaY)}`;
            } else {
                distanceVector.style.display = 'none';
                distanceLabel.style.display = 'none';
            }
        }



        // Keep track of robot-relative positions clearly
        async function pollState() {
            while (true) {
                try {
                    const res = await fetch('/state');
                    const data = await res.json();

                    coordinates.textContent = `X: ${Math.round(data.x)}, Y: ${Math.round(data.y)}`;
                    joystickInfo.innerHTML = `Axis 0: ${data.axis0.toFixed(3)}<br>Axis 1: ${data.axis1.toFixed(3)}`;

                    updateGeometry(data.x, data.y, data.robot_x, data.robot_y);

                    const now = Date.now();
                    statusIndicator.classList.toggle('connected', now - lastUpdate < 1000);
                    lastUpdate = now;

                } catch (e) {
                    console.error('Errore connessione:', e);
                    statusIndicator.classList.remove('connected');
                }

                await new Promise(r => setTimeout(r, 50));
            }
        }


        
        function updateDashedLine(line, startX, startY, endX, endY) {
            const dx = endX - startX;
            const dy = endY - startY;
            const distance = Math.sqrt(dx * dx + dy * dy);
            const angle = Math.atan2(dy, dx) * 180 / Math.PI;
            
            line.style.left = startX + 'px';
            line.style.top = startY + 'px';
            line.style.width = distance + 'px';
            line.style.transform = `rotate(${angle}deg)`;
        }
        
        async function pollState() {
            while (true) {
                try {
                    const res = await fetch('/state');
                    const data = await res.json();
                    
                    currentState = data;

                    crosshair.style.transform = 
                        `translate(calc(-50% + ${data.x}px), calc(-50% + ${data.y}px))`;

                    coordinates.textContent = 
                        `X: ${Math.round(data.x)}, Y: ${Math.round(data.y)}`;

                    joystickInfo.innerHTML = 
                        `Axis 0: ${data.axis0.toFixed(3)}<br>Axis 1: ${data.axis1.toFixed(3)}`;

                    updateGeometry(data.x, data.y, data.robot_x, data.robot_y);

                    const now = Date.now();
                    statusIndicator.classList.toggle('connected', now - lastUpdate < 1000);
                    lastUpdate = now;

                } catch (e) {
                    console.error('Errore connessione:', e);
                    statusIndicator.classList.remove('connected');
                }

                await new Promise(r => setTimeout(r, 50));
            }
        }
        
        // Inizializza geometria al caricamento
        window.addEventListener('load', () => {
            setTimeout(() => updateGeometry(0, 0), 100);
        });
        
        // Aggiorna geometria se la finestra viene ridimensionata
        window.addEventListener('resize', () => {
            setTimeout(() => updateGeometry(currentState.x, currentState.y), 100);
        });
        
        pollState();
    </script>
</body>
</html>
    ''')

@app.route('/state')
def state():
    return jsonify(crosshair_state)

def run_flask():
    print("🎯 Avvio GUI Mirino Obsidian con controllo Joystick...")
    print("📍 Server disponibile su: http://localhost:7001")
    print("🎮 Controllo tramite Joystick ROS2 su topic /joy")
    print("   • Axis 0: movimento orizzontale (X)")
    print("   • Axis 1: movimento verticale (Y)")
    print("   • Vettore arancione: distanza dal robot al mirino")
    print("   • Linee blu: collegamento arganelli-mirino")
    print("   • FREEZE: Axis4 = -1")
    print("   • Ctrl+C: chiudi il server")
    app.run(host='0.0.0.0', port=7001, debug=False)

# === ROS 2 Node ===
class JoyListener(Node):
    def __init__(self):
        super().__init__('crosshair_joy_listener')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("🎮 JoyListener avviato - in ascolto su /joy")

    def joy_callback(self, msg):
        if len(msg.axes) >= 6:
            # Axis 4 (freeze/unfreeze)
            if msg.axes[4] == -1:
                if not crosshair_state["frozen"]:
                    crosshair_state["frozen"] = True
                    crosshair_state["frozen_x"] = crosshair_state["x"]
                    crosshair_state["frozen_y"] = crosshair_state["y"]
                    self.get_logger().info("FREEZE ACTIVATED")
            else:
                if crosshair_state["frozen"]:
                    crosshair_state["frozen"] = False
                    self.get_logger().info("FREEZE DEACTIVATED")

            # Axis 5 (update robot position)
            if msg.axes[5] == -1:
                crosshair_state["robot_x"] = crosshair_state["x"]
                crosshair_state["robot_y"] = crosshair_state["y"]
                self.get_logger().info(f"ROBOT POSITION UPDATED to ({crosshair_state['robot_x']}, {crosshair_state['robot_y']})")

        # Update position only when not frozen
        if not crosshair_state["frozen"] and len(msg.axes) >= 2:
            axis0 = -msg.axes[0]
            axis1 = -msg.axes[1]
            max_range = crosshair_state["max_range"]
            crosshair_state["x"] = axis0 * max_range
            crosshair_state["y"] = axis1 * max_range
            crosshair_state["axis0"] = axis0
            crosshair_state["axis1"] = msg.axes[1]

        # Use frozen positions when frozen
        elif crosshair_state["frozen"]:
            crosshair_state["x"] = crosshair_state["frozen_x"]
            crosshair_state["y"] = crosshair_state["frozen_y"]

        # Periodic log
        self.callback_count = getattr(self, 'callback_count', 0) + 1
        if self.callback_count % 20 == 0:
            status = "FROZEN" if crosshair_state["frozen"] else "ACTIVE"
            self.get_logger().info(
                f"[{status}] X={crosshair_state['x']:.1f}, Y={crosshair_state['y']:.1f} | "
                f"Axis0={msg.axes[0]:.3f}, Axis1={msg.axes[1]:.3f}"
            )

def ros_thread():
    rclpy.init()
    node = JoyListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# === Main Entry ===
if __name__ == '__main__':
    # Avvia ROS2 in thread separato
    threading.Thread(target=ros_thread, daemon=True).start()
    
    # Avvia Flask
    run_flask()