import time
import threading
import cv2
import numpy as np
import board
import busio
from flask import Flask, Response, render_template_string, request, jsonify
from adafruit_pca9685 import PCA9685

# OpenCV auf alle 4 CPU-Kerne des Pi 3 B+ optimieren
cv2.setUseOptimized(True)
cv2.setNumThreads(4)

app = Flask(__name__)

# ==========================================
# HARDWARE-INITIALISIERUNG (PCA9685)
# ==========================================
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Standard 50 Hz für RC-Servos & ESCs

state = {
    "ampel_erkannt": False,
    "auto_stopp_ampel": True,
    "status_text": "Bereit (ESC scharf auf 1500 µs)",
    "fps": 0.0,
    "auto_fahrt_aktiv": False,
    "lenkung_kanal": 0,
    "motor_kanal": 1,
    "lenkung_prozent": 0,
    "lenkung_mitte_us": 1500,
    "gas_prozent": 0,          # -100 (Rückwärts) bis +100 (Vorwärts)
    "max_speed": 40,           # Geschwindigkeitslimit in %
    "invertiert": True,        # Standardmäßig aktiv (< 1500 µs = Vorwärts)
    "letzter_heartbeat": time.time()
}
hardware_lock = threading.Lock()
lenkung_timer = None

def set_puls_us(kanal, mikrosekunden):
    """Sendet einen exakten RC-Puls (1000-2000 µs) oder 0 zum Abschalten."""
    if mikrosekunden <= 0:
        pca.channels[kanal].duty_cycle = 0
        return
    duty = int((mikrosekunden / 20000.0) * 65535)
    pca.channels[kanal].duty_cycle = duty

def berechne_lenkung_puls():
    """Berechnet den Lenkpuls (A = Links, D = Rechts)."""
    mitte = max(1100, min(1900, int(state["lenkung_mitte_us"])))
    prozent = max(-100, min(100, int(state["lenkung_prozent"])))
    puls = int(mitte - (prozent / 100.0) * 350)
    return max(1000, min(2000, puls))

def berechne_motor_puls(gas_prozent):
    gas = max(-100, min(100, int(gas_prozent)))
    if gas == 0:
        return 1500
    faktor = -1 if state["invertiert"] else 1
    puls = int(1500 + faktor * (gas / 100.0) * 500)
    return max(1000, min(2000, puls))

def lenkung_ruhezustand():
    """Schaltet das Lenksignal nach kurzer Stellzeit ab, damit das Servo nicht brummt."""
    with hardware_lock:
        if state["lenkung_prozent"] == 0 and state["gas_prozent"] == 0 and not state["auto_fahrt_aktiv"]:
            set_puls_us(state["lenkung_kanal"], 0)

def wende_steuerung_an():
    """Aktualisiert Motor und Lenkung auf dem PCA9685."""
    global lenkung_timer
    with hardware_lock:
        m_kanal = state["motor_kanal"]
        l_kanal = state["lenkung_kanal"]

        # Ampel-Notbremse prüfen
        if state["auto_stopp_ampel"] and state["ampel_erkannt"] and state["gas_prozent"] > 0:
            state["gas_prozent"] = 0
            state["status_text"] = "⚠️ AMPEL ERKANNT -> Vorwärts gesperrt!"

        m_puls = berechne_motor_puls(state["gas_prozent"])
        l_puls = berechne_lenkung_puls()

        set_puls_us(m_kanal, m_puls)
        set_puls_us(l_kanal, l_puls)

        if state["gas_prozent"] != 0 or state["lenkung_prozent"] != 0:
            state["status_text"] = f"Gas: {state['gas_prozent']}% ({m_puls}µs) | Lenk: {state['lenkung_prozent']}% ({l_puls}µs)"

    # Wenn Lenkung & Gas auf 0 stehen, Servo nach 0.5s beruhigen (ESC bleibt auf 1500 µs scharf!)
    if state["lenkung_prozent"] == 0 and state["gas_prozent"] == 0 and not state["auto_fahrt_aktiv"]:
        if lenkung_timer is not None:
            lenkung_timer.cancel()
        lenkung_timer = threading.Timer(0.5, lenkung_ruhezustand)
        lenkung_timer.start()

def not_aus():
    """Sofortiger Stopp aller Bewegungen."""
    global lenkung_timer
    if lenkung_timer is not None:
        lenkung_timer.cancel()
    with hardware_lock:
        state["auto_fahrt_aktiv"] = False
        state["gas_prozent"] = 0
        state["lenkung_prozent"] = 0
        set_puls_us(state["motor_kanal"], 1500)
        set_puls_us(state["lenkung_kanal"], 0)
        state["status_text"] = "🛑 NOT-AUS! Motor Neutral (1500 µs), Lenkung aus."

# Beim Start ESC direkt auf Neutral (1500 µs) scharfschalten
set_puls_us(state["motor_kanal"], 1500)
set_puls_us(state["lenkung_kanal"], 0)

def watchdog_loop():
    """Sicherheit: Stoppt das Auto, falls im WASD-Modus >0.5s kein Signal mehr vom Browser kommt."""
    while True:
        time.sleep(0.1)
        if not state["auto_fahrt_aktiv"] and (state["gas_prozent"] != 0 or state["lenkung_prozent"] != 0):
            if (time.time() - state["letzter_heartbeat"]) > 0.5:
                state["gas_prozent"] = 0
                state["lenkung_prozent"] = 0
                wende_steuerung_an()
                state["status_text"] = "Auto-Stopp (Taste losgelassen / Timeout)"

threading.Thread(target=watchdog_loop, daemon=True).start()

def zeit_fahrt_ablauf(geschwindigkeit, sekunden):
    """Fährt für eine feste Zeitspanne vorwärts (Button-Modus)."""
    with hardware_lock:
        state["auto_fahrt_aktiv"] = True

    start = time.time()
    while (time.time() - start) < sekunden:
        if not state["auto_fahrt_aktiv"]:
            break
        if state["auto_stopp_ampel"] and state["ampel_erkannt"]:
            state["status_text"] = "⚠️ AMPEL -> Automatischer Stopp!"
            break
        state["gas_prozent"] = geschwindigkeit
        wende_steuerung_an()
        time.sleep(0.05)

    state["auto_fahrt_aktiv"] = False
    state["gas_prozent"] = 0
    wende_steuerung_an()
    if "AMPEL" not in state["status_text"]:
        state["status_text"] = "Zeitfahrt beendet (Neutral 1500 µs)"

# ==========================================
# KAMERA & YOLOv4-TINY (MULTI-THREADED)
# ==========================================
KLASSEN = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

class KameraStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.running = True
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.cap.release()

print("[INIT] Lade YOLOv4-tiny...")
net = cv2.dnn.readNetFromDarknet("yolov4-tiny.cfg", "yolov4-tiny.weights")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]

cam = KameraStream(0)
time.sleep(1.0)

def generiere_frames():
    while True:
        start_zeit = time.time()
        erfolg, frame = cam.read()
        if not erfolg or frame is None:
            time.sleep(0.02)
            continue

        height, width, _ = frame.shape
        blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(320, 320), swapRB=True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        boxes, scores, class_ids = [], [], []
        for out in outs:
            confidences = np.max(out[:, 5:], axis=1)
            maske = confidences >= 0.35
            gefiltert = out[maske]
            for det, score, cid in zip(gefiltert, confidences[maske], np.argmax(gefiltert[:, 5:], axis=1)):
                w = int(det[2] * width)
                h = int(det[3] * height)
                x = int((det[0] * width) - w / 2)
                y = int((det[1] * height) - h / 2)
                boxes.append([x, y, w, h])
                scores.append(float(score))
                class_ids.append(int(cid))

        indices = cv2.dnn.NMSBoxes(boxes, scores, 0.35, 0.45)
        ampel_gefunden = False

        if len(indices) > 0:
            for i in indices.flatten():
                name = KLASSEN[class_ids[i]]
                sicherheit = int(scores[i] * 100)
                x, y, w, h = boxes[i]
                if name == "traffic light":
                    ampel_gefunden = True
                farbe = (0, 0, 255) if name == "traffic light" else (0, 255, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), farbe, 2)
                cv2.putText(frame, f"{name} {sicherheit}%", (x, max(y - 5, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, farbe, 2)

        state["ampel_erkannt"] = ampel_gefunden
        dauer_ms = (time.time() - start_zeit) * 1000
        fps = 1000 / dauer_ms if dauer_ms > 0 else 0
        state["fps"] = round(fps, 1)

        cv2.putText(frame, f"FPS: {fps:.1f}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

# ==========================================
# WEB-OBERFLÄCHE (MIT WASD & TOUCH-PAD)
# ==========================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>RC-Car WASD Cockpit</title>
  <style>
    body { background: #121417; color: #e9ecef; font-family: -apple-system, sans-serif; margin: 0; padding: 14px; user-select: none; }
    .container { max-width: 1000px; margin: 0 auto; display: grid; grid-template-columns: 1.1fr 0.9fr; gap: 14px; }
    @media (max-width: 768px) { .container { grid-template-columns: 1fr; } }
    .card { background: #1e2126; border-radius: 12px; padding: 14px; }
    h2 { margin-top: 0; font-size: 1rem; color: #adb5bd; border-bottom: 1px solid #2c3036; padding-bottom: 6px; }
    img { width: 100%; border-radius: 8px; background: #000; display: block; }
    .row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; gap: 8px; }
    input[type=range] { width: 100%; accent-color: #3b82f6; }
    select, input[type=number] { background: #141619; color: #fff; border: 1px solid #374151; border-radius: 6px; padding: 6px; font-size: 0.9rem; }
    .btn { width: 100%; padding: 11px; border: none; border-radius: 8px; font-size: 0.95rem; font-weight: 600; cursor: pointer; margin-bottom: 8px; }
    .btn-small { width: auto; padding: 6px 10px; font-size: 0.85rem; margin-bottom: 0; background: #374151; color: #fff; }
    .btn-go { background: #10b981; color: #fff; }
    .btn-stop { background: #ef4444; color: #fff; font-size: 1.05rem; padding: 14px; }
    .status-box { background: #141619; padding: 10px; border-radius: 8px; font-family: monospace; font-size: 0.85rem; border-left: 4px solid #3b82f6; margin-top: 8px; }
    .badge { padding: 3px 8px; border-radius: 6px; font-size: 0.8rem; font-weight: bold; }
    .badge-ok { background: #065f46; color: #a7f3d0; }
    .badge-warn { background: #991b1b; color: #fecaca; }

    /* WASD Tastatur & Touch-Pad */
    .wasd-grid { display: grid; grid-template-columns: repeat(3, 64px); gap: 8px; justify-content: center; margin: 12px 0; }
    .key { height: 56px; background: #2c3036; border: 2px solid #495057; border-radius: 10px; display: flex; flex-direction: column; align-items: center; justify-content: center; font-weight: bold; font-size: 1.1rem; cursor: pointer; transition: 0.08s; }
    .key small { font-size: 0.65rem; color: #adb5bd; font-weight: normal; }
    .key.active { background: #3b82f6; border-color: #93c5fd; color: #fff; transform: scale(0.95); }
  </style>
</head>
<body>
  <div class="container">
    <!-- Linke Spalte: Kamera & WASD -->
    <div class="card">
      <h2>Live-Kamera &amp; YOLOv4-tiny</h2>
      <img src="/video_feed" alt="Kamera Stream">
      <div style="margin-top: 10px; display: flex; justify-content: space-between; align-items: center;">
        <span>FPS: <strong id="fps_val">0.0</strong></span>
        <span id="ampel_badge" class="badge badge-ok">Keine Ampel</span>
      </div>

      <h2 style="margin-top: 16px;">WASD Echtzeit-Steuerung</h2>
      <div class="wasd-grid">
        <div></div>
        <div class="key" id="key-w" onmousedown="touchKey('w',true)" onmouseup="touchKey('w',false)" ontouchstart="touchKey('w',true)" ontouchend="touchKey('w',false)">W<small>Vor</small></div>
        <div></div>
        <div class="key" id="key-a" onmousedown="touchKey('a',true)" onmouseup="touchKey('a',false)" ontouchstart="touchKey('a',true)" ontouchend="touchKey('a',false)">A<small>Links</small></div>
        <div class="key" id="key-s" onmousedown="touchKey('s',true)" onmouseup="touchKey('s',false)" ontouchstart="touchKey('s',true)" ontouchend="touchKey('s',false)">S<small>Zurück</small></div>
        <div class="key" id="key-d" onmousedown="touchKey('d',true)" onmouseup="touchKey('d',false)" ontouchstart="touchKey('d',true)" ontouchend="touchKey('d',false)">D<small>Rechts</small></div>
      </div>
      <div style="text-align:center; font-size:0.8rem; color:#adb5bd;">Tasten gedrückt halten (Leertaste = Not-Aus)</div>
    </div>

    <!-- Rechte Spalte: Einstellungen & Zeit-Fahrt -->
    <div class="card">
      <h2>Einstellungen &amp; Kalibrierung</h2>
      <div class="row">
        <span>Lenk-Kanal:
          <select id="lenk_ch" onchange="updateSettings()">
            <option value="0" selected>0</option><option value="1">1</option><option value="2">2</option><option value="3">3</option>
          </select>
        </span>
        <span>Motor-Kanal:
          <select id="mot_ch" onchange="updateSettings()">
            <option value="0">0</option><option value="1" selected>1</option><option value="2">2</option><option value="3">3</option>
          </select>
        </span>
      </div>

      <div class="row">
        <span>Lenk-Mitte (µs):</span>
        <div>
          <input type="number" id="mitte_us" min="1100" max="1900" step="10" value="1500" style="width:75px;" onchange="updateSettings()">
          <button class="btn btn-small" onclick="testMitte()">Test</button>
        </div>
      </div>

      <div class="row">
        <span>Max. Geschwindigkeit:</span>
        <span><strong id="speed_out">40</strong> %</span>
      </div>
      <input type="range" id="speed" min="15" max="100" value="40" oninput="document.getElementById('speed_out').innerText=this.value; updateSettings()">

      <div style="margin: 10px 0;">
        <label style="display:block; margin-bottom:6px;"><input type="checkbox" id="invert" checked onchange="updateSettings()"> Vorwärts invertiert (&lt; 1500 µs)</label>
        <label style="display:block;"><input type="checkbox" id="auto_ampel" checked onchange="updateSettings()"> Auto-Stopp bei Ampel</label>
      </div>

      <h2>Zeitgesteuerte Fahrt</h2>
      <div class="row">
        <span>Fahrzeit:</span>
        <span><strong id="time_out">2.0</strong> Sek.</span>
      </div>
      <input type="range" id="duration" min="0.5" max="10.0" step="0.5" value="2.0" oninput="document.getElementById('time_out').innerText=parseFloat(this.value).toFixed(1)">
      <button class="btn btn-go" style="margin-top:8px;" onclick="startZeitFahrt()">▶ Für eingestellte Zeit vorwärts</button>

      <button class="btn btn-stop" onclick="notAus()">🛑 NOT-AUS (Leertaste)</button>
      <div class="status-box" id="status_text">Status: Bereit</div>
    </div>
  </div>

  <script>
    const pressed = { w: false, a: false, s: false, d: false };

    function updateKeyVisuals() {
      ['w', 'a', 's', 'd'].forEach(k => {
        const el = document.getElementById('key-' + k);
        if (pressed[k]) el.classList.add('active');
        else el.classList.remove('active');
      });
    }

    function sendControl() {
      const maxSpeed = parseInt(document.getElementById('speed').value);
      let gas = 0;
      if (pressed.w && !pressed.s) gas = maxSpeed;
      else if (pressed.s && !pressed.w) gas = -maxSpeed;

      let lenk = 0;
      if (pressed.d && !pressed.a) lenk = 100;
      else if (pressed.a && !pressed.d) lenk = -100;

      fetch('/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ gas: gas, lenkung: lenk })
      });
    }

    function touchKey(k, isDown) {
      pressed[k] = isDown;
      updateKeyVisuals();
      sendControl();
    }

    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT') return;
      const k = e.key.toLowerCase();
      if (k === ' ') {
        e.preventDefault();
        notAus();
        return;
      }
      if (k in pressed && !pressed[k]) {
        pressed[k] = true;
        updateKeyVisuals();
        sendControl();
      }
    });

    window.addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      if (k in pressed) {
        pressed[k] = false;
        updateKeyVisuals();
        sendControl();
      }
    });

    // Heartbeat alle 200ms senden, solange eine Taste gedrückt ist
    setInterval(() => {
      if (pressed.w || pressed.a || pressed.s || pressed.d) {
        sendControl();
      }
    }, 200);

    function updateSettings() {
      fetch('/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          lenkung_kanal: parseInt(document.getElementById('lenk_ch').value),
          motor_kanal: parseInt(document.getElementById('mot_ch').value),
          mitte_us: parseInt(document.getElementById('mitte_us').value),
          max_speed: parseInt(document.getElementById('speed').value),
          invertiert: document.getElementById('invert').checked,
          auto_ampel: document.getElementById('auto_ampel').checked
        })
      });
    }

    function testMitte() {
      updateSettings();
      fetch('/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ gas: 0, lenkung: 0 })
      });
    }

    function startZeitFahrt() {
      fetch('/drive_timed', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          speed: parseInt(document.getElementById('speed').value),
          duration: parseFloat(document.getElementById('duration').value)
        })
      });
    }

    function notAus() {
      pressed.w = pressed.a = pressed.s = pressed.d = false;
      updateKeyVisuals();
      fetch('/stop', { method: 'POST' });
    }

    setInterval(() => {
      fetch('/status').then(r => r.json()).then(data => {
        document.getElementById('status_text').innerText = "Status: " + data.status_text;
        document.getElementById('fps_val').innerText = data.fps;
        const badge = document.getElementById('ampel_badge');
        badge.className = data.ampel_erkannt ? "badge badge-warn" : "badge badge-ok";
        badge.innerText = data.ampel_erkannt ? "🚦 AMPEL ERKANNT" : "Keine Ampel";
      });
    }, 300);
  </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(generiere_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    state["letzter_heartbeat"] = time.time()
    if not state["auto_fahrt_aktiv"]:
        state["gas_prozent"] = int(data.get("gas", 0))
        state["lenkung_prozent"] = int(data.get("lenkung", 0))
        wende_steuerung_an()
    return jsonify({"ok": True})

@app.route('/settings', methods=['POST'])
def settings():
    data = request.json
    state["lenkung_kanal"] = int(data.get("lenkung_kanal", 0))
    state["motor_kanal"] = int(data.get("motor_kanal", 1))
    state["lenkung_mitte_us"] = int(data.get("mitte_us", 1500))
    state["max_speed"] = int(data.get("max_speed", 40))
    state["invertiert"] = bool(data.get("invertiert", True))
    state["auto_stopp_ampel"] = bool(data.get("auto_ampel", True))
    return jsonify({"ok": True})

@app.route('/drive_timed', methods=['POST'])
def drive_timed():
    data = request.json
    if not state["auto_fahrt_aktiv"]:
        threading.Thread(
            target=zeit_fahrt_ablauf,
            args=(data.get("speed", 40), data.get("duration", 2.0)),
            daemon=True
        ).start()
    return jsonify({"ok": True})

@app.route('/stop', methods=['POST'])
def stop():
    not_aus()
    return jsonify({"ok": True})

@app.route('/status')
def status():
    return jsonify(state)

if __name__ == '__main__':
    print("[READY] WASD RC-Cockpit läuft! Öffne http://<DEINE-PI-IP>:5000")
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        not_aus()
        cam.stop()
        pca.deinit()
import time
import threading
import cv2
import numpy as np
import board
import busio
from flask import Flask, Response, render_template_string, request, jsonify
from adafruit_pca9685 import PCA9685

# OpenCV auf alle 4 CPU-Kerne des Pi 3 B+ optimieren
cv2.setUseOptimized(True)
cv2.setNumThreads(4)

app = Flask(__name__)

# ==========================================
# HARDWARE-INITIALISIERUNG (PCA9685)
# ==========================================
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Standard 50 Hz für RC-Servos & ESCs

state = {
    "ampel_erkannt": False,
    "auto_stopp_ampel": True,
    "status_text": "Bereit (ESC scharf auf 1500 µs)",
    "fps": 0.0,
    "auto_fahrt_aktiv": False,
    "lenkung_kanal": 0,
    "motor_kanal": 1,
    "lenkung_prozent": 0,
    "lenkung_mitte_us": 1500,
    "gas_prozent": 0,          # -100 (Rückwärts) bis +100 (Vorwärts)
    "max_speed": 40,           # Geschwindigkeitslimit in %
    "invertiert": True,        # Standardmäßig aktiv (< 1500 µs = Vorwärts)
    "letzter_heartbeat": time.time()
}
hardware_lock = threading.Lock()
lenkung_timer = None

def set_puls_us(kanal, mikrosekunden):
    """Sendet einen exakten RC-Puls (1000-2000 µs) oder 0 zum Abschalten."""
    if mikrosekunden <= 0:
        pca.channels[kanal].duty_cycle = 0
        return
    duty = int((mikrosekunden / 20000.0) * 65535)
    pca.channels[kanal].duty_cycle = duty

def berechne_lenkung_puls():
    mitte = max(1100, min(1900, int(state["lenkung_mitte_us"])))
    prozent = max(-100, min(100, int(state["lenkung_prozent"])))
    puls = int(mitte + (prozent / 100.0) * 350)
    return max(1000, min(2000, puls))

def berechne_motor_puls(gas_prozent):
    gas = max(-100, min(100, int(gas_prozent)))
    if gas == 0:
        return 1500
    # Wenn invertiert=True: Vorwärts (+gas) senkt Puls unter 1500 µs
    faktor = -1 if state["invertiert"] else 1
    puls = int(1500 + faktor * (gas / 100.0) * 500)
    return max(1000, min(2000, puls))

def lenkung_ruhezustand():
    """Schaltet das Lenksignal nach kurzer Stellzeit ab, damit das Servo nicht brummt."""
    with hardware_lock:
        if state["lenkung_prozent"] == 0 and state["gas_prozent"] == 0 and not state["auto_fahrt_aktiv"]:
            set_puls_us(state["lenkung_kanal"], 0)

def wende_steuerung_an():
    """Aktualisiert Motor und Lenkung auf dem PCA9685."""
    global lenkung_timer
    with hardware_lock:
        m_kanal = state["motor_kanal"]
        l_kanal = state["lenkung_kanal"]

        # Ampel-Notbremse prüfen
        if state["auto_stopp_ampel"] and state["ampel_erkannt"] and state["gas_prozent"] > 0:
            state["gas_prozent"] = 0
            state["status_text"] = "⚠️ AMPEL ERKANNT -> Vorwärts gesperrt!"

        m_puls = berechne_motor_puls(state["gas_prozent"])
        l_puls = berechne_lenkung_puls()

        set_puls_us(m_kanal, m_puls)
        set_puls_us(l_kanal, l_puls)

        if state["gas_prozent"] != 0 or state["lenkung_prozent"] != 0:
            state["status_text"] = f"Gas: {state['gas_prozent']}% ({m_puls}µs) | Lenk: {state['lenkung_prozent']}% ({l_puls}µs)"

    # Wenn Lenkung & Gas auf 0 stehen, Servo nach 0.5s beruhigen (ESC bleibt auf 1500 µs scharf!)
    if state["lenkung_prozent"] == 0 and state["gas_prozent"] == 0 and not state["auto_fahrt_aktiv"]:
        if lenkung_timer is not None:
            lenkung_timer.cancel()
        lenkung_timer = threading.Timer(0.5, lenkung_ruhezustand)
        lenkung_timer.start()

def not_aus():
    """Sofortiger Stopp aller Bewegungen."""
    global lenkung_timer
    if lenkung_timer is not None:
        lenkung_timer.cancel()
    with hardware_lock:
        state["auto_fahrt_aktiv"] = False
        state["gas_prozent"] = 0
        state["lenkung_prozent"] = 0
        set_puls_us(state["motor_kanal"], 1500)
        set_puls_us(state["lenkung_kanal"], 0)
        state["status_text"] = "🛑 NOT-AUS! Motor Neutral (1500 µs), Lenkung aus."

# Beim Start ESC direkt auf Neutral (1500 µs) scharfschalten
set_puls_us(state["motor_kanal"], 1500)
set_puls_us(state["lenkung_kanal"], 0)

def watchdog_loop():
    """Sicherheit: Stoppt das Auto, falls im WASD-Modus >0.5s kein Signal mehr vom Browser kommt."""
    while True:
        time.sleep(0.1)
        if not state["auto_fahrt_aktiv"] and (state["gas_prozent"] != 0 or state["lenkung_prozent"] != 0):
            if (time.time() - state["letzter_heartbeat"]) > 0.5:
                state["gas_prozent"] = 0
                state["lenkung_prozent"] = 0
                wende_steuerung_an()
                state["status_text"] = "Auto-Stopp (Taste losgelassen / Timeout)"

threading.Thread(target=watchdog_loop, daemon=True).start()

def zeit_fahrt_ablauf(geschwindigkeit, sekunden):
    """Fährt für eine feste Zeitspanne vorwärts (Button-Modus)."""
    with hardware_lock:
        state["auto_fahrt_aktiv"] = True

    start = time.time()
    while (time.time() - start) < sekunden:
        if not state["auto_fahrt_aktiv"]:
            break
        if state["auto_stopp_ampel"] and state["ampel_erkannt"]:
            state["status_text"] = "⚠️ AMPEL -> Automatischer Stopp!"
            break
        state["gas_prozent"] = geschwindigkeit
        wende_steuerung_an()
        time.sleep(0.05)

    state["auto_fahrt_aktiv"] = False
    state["gas_prozent"] = 0
    wende_steuerung_an()
    if "AMPEL" not in state["status_text"]:
        state["status_text"] = "Zeitfahrt beendet (Neutral 1500 µs)"

# ==========================================
# KAMERA & YOLOv4-TINY (MULTI-THREADED)
# ==========================================
KLASSEN = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

class KameraStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.running = True
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.cap.release()

print("[INIT] Lade YOLOv4-tiny...")
net = cv2.dnn.readNetFromDarknet("yolov4-tiny.cfg", "yolov4-tiny.weights")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]

cam = KameraStream(0)
time.sleep(1.0)

def generiere_frames():
    while True:
        start_zeit = time.time()
        erfolg, frame = cam.read()
        if not erfolg or frame is None:
            time.sleep(0.02)
            continue

        height, width, _ = frame.shape
        blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(320, 320), swapRB=True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        boxes, scores, class_ids = [], [], []
        for out in outs:
            confidences = np.max(out[:, 5:], axis=1)
            maske = confidences >= 0.35
            gefiltert = out[maske]
            for det, score, cid in zip(gefiltert, confidences[maske], np.argmax(gefiltert[:, 5:], axis=1)):
                w = int(det[2] * width)
                h = int(det[3] * height)
                x = int((det[0] * width) - w / 2)
                y = int((det[1] * height) - h / 2)
                boxes.append([x, y, w, h])
                scores.append(float(score))
                class_ids.append(int(cid))

        indices = cv2.dnn.NMSBoxes(boxes, scores, 0.35, 0.45)
        ampel_gefunden = False

        if len(indices) > 0:
            for i in indices.flatten():
                name = KLASSEN[class_ids[i]]
                sicherheit = int(scores[i] * 100)
                x, y, w, h = boxes[i]
                if name == "traffic light":
                    ampel_gefunden = True
                farbe = (0, 0, 255) if name == "traffic light" else (0, 255, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), farbe, 2)
                cv2.putText(frame, f"{name} {sicherheit}%", (x, max(y - 5, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, farbe, 2)

        state["ampel_erkannt"] = ampel_gefunden
        dauer_ms = (time.time() - start_zeit) * 1000
        fps = 1000 / dauer_ms if dauer_ms > 0 else 0
        state["fps"] = round(fps, 1)

        cv2.putText(frame, f"FPS: {fps:.1f}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

# ==========================================
# WEB-OBERFLÄCHE (MIT WASD & TOUCH-PAD)
# ==========================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>RC-Car WASD Cockpit</title>
  <style>
    body { background: #121417; color: #e9ecef; font-family: -apple-system, sans-serif; margin: 0; padding: 14px; user-select: none; }
    .container { max-width: 1000px; margin: 0 auto; display: grid; grid-template-columns: 1.1fr 0.9fr; gap: 14px; }
    @media (max-width: 768px) { .container { grid-template-columns: 1fr; } }
    .card { background: #1e2126; border-radius: 12px; padding: 14px; }
    h2 { margin-top: 0; font-size: 1rem; color: #adb5bd; border-bottom: 1px solid #2c3036; padding-bottom: 6px; }
    img { width: 100%; border-radius: 8px; background: #000; display: block; }
    .row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; gap: 8px; }
    input[type=range] { width: 100%; accent-color: #3b82f6; }
    select, input[type=number] { background: #141619; color: #fff; border: 1px solid #374151; border-radius: 6px; padding: 6px; font-size: 0.9rem; }
    .btn { width: 100%; padding: 11px; border: none; border-radius: 8px; font-size: 0.95rem; font-weight: 600; cursor: pointer; margin-bottom: 8px; }
    .btn-small { width: auto; padding: 6px 10px; font-size: 0.85rem; margin-bottom: 0; background: #374151; color: #fff; }
    .btn-go { background: #10b981; color: #fff; }
    .btn-stop { background: #ef4444; color: #fff; font-size: 1.05rem; padding: 14px; }
    .status-box { background: #141619; padding: 10px; border-radius: 8px; font-family: monospace; font-size: 0.85rem; border-left: 4px solid #3b82f6; margin-top: 8px; }
    .badge { padding: 3px 8px; border-radius: 6px; font-size: 0.8rem; font-weight: bold; }
    .badge-ok { background: #065f46; color: #a7f3d0; }
    .badge-warn { background: #991b1b; color: #fecaca; }

    /* WASD Tastatur & Touch-Pad */
    .wasd-grid { display: grid; grid-template-columns: repeat(3, 64px); gap: 8px; justify-content: center; margin: 12px 0; }
    .key { height: 56px; background: #2c3036; border: 2px solid #495057; border-radius: 10px; display: flex; flex-direction: column; align-items: center; justify-content: center; font-weight: bold; font-size: 1.1rem; cursor: pointer; transition: 0.08s; }
    .key small { font-size: 0.65rem; color: #adb5bd; font-weight: normal; }
    .key.active { background: #3b82f6; border-color: #93c5fd; color: #fff; transform: scale(0.95); }
  </style>
</head>
<body>
  <div class="container">
    <!-- Linke Spalte: Kamera & WASD -->
    <div class="card">
      <h2>Live-Kamera &amp; YOLOv4-tiny</h2>
      <img src="/video_feed" alt="Kamera Stream">
      <div style="margin-top: 10px; display: flex; justify-content: space-between; align-items: center;">
        <span>FPS: <strong id="fps_val">0.0</strong></span>
        <span id="ampel_badge" class="badge badge-ok">Keine Ampel</span>
      </div>

      <h2 style="margin-top: 16px;">WASD Echtzeit-Steuerung</h2>
      <div class="wasd-grid">
        <div></div>
        <div class="key" id="key-w" onmousedown="touchKey('w',true)" onmouseup="touchKey('w',false)" ontouchstart="touchKey('w',true)" ontouchend="touchKey('w',false)">W<small>Vor</small></div>
        <div></div>
        <div class="key" id="key-a" onmousedown="touchKey('a',true)" onmouseup="touchKey('a',false)" ontouchstart="touchKey('a',true)" ontouchend="touchKey('a',false)">A<small>Links</small></div>
        <div class="key" id="key-s" onmousedown="touchKey('s',true)" onmouseup="touchKey('s',false)" ontouchstart="touchKey('s',true)" ontouchend="touchKey('s',false)">S<small>Zurück</small></div>
        <div class="key" id="key-d" onmousedown="touchKey('d',true)" onmouseup="touchKey('d',false)" ontouchstart="touchKey('d',true)" ontouchend="touchKey('d',false)">D<small>Rechts</small></div>
      </div>
      <div style="text-align:center; font-size:0.8rem; color:#adb5bd;">Tasten gedrückt halten (Leertaste = Not-Aus)</div>
    </div>

    <!-- Rechte Spalte: Einstellungen & Zeit-Fahrt -->
    <div class="card">
      <h2>Einstellungen &amp; Kalibrierung</h2>
      <div class="row">
        <span>Lenk-Kanal:
          <select id="lenk_ch" onchange="updateSettings()">
            <option value="0" selected>0</option><option value="1">1</option><option value="2">2</option><option value="3">3</option>
          </select>
        </span>
        <span>Motor-Kanal:
          <select id="mot_ch" onchange="updateSettings()">
            <option value="0">0</option><option value="1" selected>1</option><option value="2">2</option><option value="3">3</option>
          </select>
        </span>
      </div>

      <div class="row">
        <span>Lenk-Mitte (µs):</span>
        <div>
          <input type="number" id="mitte_us" min="1100" max="1900" step="10" value="1500" style="width:75px;" onchange="updateSettings()">
          <button class="btn btn-small" onclick="testMitte()">Test</button>
        </div>
      </div>

      <div class="row">
        <span>Max. Geschwindigkeit:</span>
        <span><strong id="speed_out">40</strong> %</span>
      </div>
      <input type="range" id="speed" min="15" max="100" value="40" oninput="document.getElementById('speed_out').innerText=this.value; updateSettings()">

      <div style="margin: 10px 0;">
        <label style="display:block; margin-bottom:6px;"><input type="checkbox" id="invert" checked onchange="updateSettings()"> Vorwärts invertiert (&lt; 1500 µs)</label>
        <label style="display:block;"><input type="checkbox" id="auto_ampel" checked onchange="updateSettings()"> Auto-Stopp bei Ampel</label>
      </div>

      <h2>Zeitgesteuerte Fahrt</h2>
      <div class="row">
        <span>Fahrzeit:</span>
        <span><strong id="time_out">2.0</strong> Sek.</span>
      </div>
      <input type="range" id="duration" min="0.5" max="10.0" step="0.5" value="2.0" oninput="document.getElementById('time_out').innerText=parseFloat(this.value).toFixed(1)">
      <button class="btn btn-go" style="margin-top:8px;" onclick="startZeitFahrt()">▶ Für eingestellte Zeit vorwärts</button>

      <button class="btn btn-stop" onclick="notAus()">🛑 NOT-AUS (Leertaste)</button>
      <div class="status-box" id="status_text">Status: Bereit</div>
    </div>
  </div>

  <script>
    const pressed = { w: false, a: false, s: false, d: false };

    function updateKeyVisuals() {
      ['w', 'a', 's', 'd'].forEach(k => {
        const el = document.getElementById('key-' + k);
        if (pressed[k]) el.classList.add('active');
        else el.classList.remove('active');
      });
    }

    function sendControl() {
      const maxSpeed = parseInt(document.getElementById('speed').value);
      let gas = 0;
      if (pressed.w && !pressed.s) gas = maxSpeed;
      else if (pressed.s && !pressed.w) gas = -maxSpeed;

      let lenk = 0;
      if (pressed.d && !pressed.a) lenk = 100;
      else if (pressed.a && !pressed.d) lenk = -100;

      fetch('/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ gas: gas, lenkung: lenk })
      });
    }

    function touchKey(k, isDown) {
      pressed[k] = isDown;
      updateKeyVisuals();
      sendControl();
    }

    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT') return;
      const k = e.key.toLowerCase();
      if (k === ' ') {
        e.preventDefault();
        notAus();
        return;
      }
      if (k in pressed && !pressed[k]) {
        pressed[k] = true;
        updateKeyVisuals();
        sendControl();
      }
    });

    window.addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      if (k in pressed) {
        pressed[k] = false;
        updateKeyVisuals();
        sendControl();
      }
    });

    // Heartbeat alle 200ms senden, solange eine Taste gedrückt ist
    setInterval(() => {
      if (pressed.w || pressed.a || pressed.s || pressed.d) {
        sendControl();
      }
    }, 200);

    function updateSettings() {
      fetch('/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          lenkung_kanal: parseInt(document.getElementById('lenk_ch').value),
          motor_kanal: parseInt(document.getElementById('mot_ch').value),
          mitte_us: parseInt(document.getElementById('mitte_us').value),
          max_speed: parseInt(document.getElementById('speed').value),
          invertiert: document.getElementById('invert').checked,
          auto_ampel: document.getElementById('auto_ampel').checked
        })
      });
    }

    function testMitte() {
      updateSettings();
      fetch('/control', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ gas: 0, lenkung: 0 })
      });
    }

    function startZeitFahrt() {
      fetch('/drive_timed', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          speed: parseInt(document.getElementById('speed').value),
          duration: parseFloat(document.getElementById('duration').value)
        })
      });
    }

    function notAus() {
      pressed.w = pressed.a = pressed.s = pressed.d = false;
      updateKeyVisuals();
      fetch('/stop', { method: 'POST' });
    }

    setInterval(() => {
      fetch('/status').then(r => r.json()).then(data => {
        document.getElementById('status_text').innerText = "Status: " + data.status_text;
        document.getElementById('fps_val').innerText = data.fps;
        const badge = document.getElementById('ampel_badge');
        badge.className = data.ampel_erkannt ? "badge badge-warn" : "badge badge-ok";
        badge.innerText = data.ampel_erkannt ? "🚦 AMPEL ERKANNT" : "Keine Ampel";
      });
    }, 300);
  </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(generiere_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    state["letzter_heartbeat"] = time.time()
    if not state["auto_fahrt_aktiv"]:
        state["gas_prozent"] = int(data.get("gas", 0))
        state["lenkung_prozent"] = int(data.get("lenkung", 0))
        wende_steuerung_an()
    return jsonify({"ok": True})

@app.route('/settings', methods=['POST'])
def settings():
    data = request.json
    state["lenkung_kanal"] = int(data.get("lenkung_kanal", 0))
    state["motor_kanal"] = int(data.get("motor_kanal", 1))
    state["lenkung_mitte_us"] = int(data.get("mitte_us", 1500))
    state["max_speed"] = int(data.get("max_speed", 40))
    state["invertiert"] = bool(data.get("invertiert", True))
    state["auto_stopp_ampel"] = bool(data.get("auto_ampel", True))
    return jsonify({"ok": True})

@app.route('/drive_timed', methods=['POST'])
def drive_timed():
    data = request.json
    if not state["auto_fahrt_aktiv"]:
        threading.Thread(
            target=zeit_fahrt_ablauf,
            args=(data.get("speed", 40), data.get("duration", 2.0)),
            daemon=True
        ).start()
    return jsonify({"ok": True})

@app.route('/stop', methods=['POST'])
def stop():
    not_aus()
    return jsonify({"ok": True})

@app.route('/status')
def status():
    return jsonify(state)

if __name__ == '__main__':
    print("[READY] WASD RC-Cockpit läuft! Öffne http://<DEINE-PI-IP>:5000")
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        not_aus()
        cam.stop()
        pca.deinit()
import time
import threading
import cv2
import numpy as np
import board
import busio
from flask import Flask, Response, render_template_string, request, jsonify
from adafruit_pca9685 import PCA9685

# OpenCV auf alle 4 Kerne des Pi 3 B+ optimieren
cv2.setUseOptimized(True)
cv2.setNumThreads(4)

app = Flask(__name__)

# ==========================================
# HARDWARE-KONFIGURATION
# ==========================================
LENKUNG_KANAL = 0
MOTOR_KANAL = 1
LENKUNG_MAX_AUSSCHLAG_US = 350  # Maximaler Weg links/rechts von der Mitte (schützt vor Anschlag)
import time
import threading
import cv2
import numpy as np
import board
import busio
from flask import Flask, Response, render_template_string, request, jsonify
from adafruit_pca9685 import PCA9685

cv2.setUseOptimized(True)
cv2.setNumThreads(4)

app = Flask(__name__)

# ==========================================
# HARDWARE-KONFIGURATION
# ==========================================
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Standard 50 Hz für RC-Servos & ESCs

state = {
    "ampel_erkannt": False,
    "auto_stopp_ampel": True,
    "status_text": "Bereit (50 Hz aktiv)",
    "fps": 0.0,
    "motor_aktiv": False,
    "lenkung_kanal": 0,
    "motor_kanal": 1,
    "lenkung_prozent": 0,
    "lenkung_mitte_us": 1500
}
hardware_lock = threading.Lock()
lenkung_timer = None

def set_puls_us(kanal, mikrosekunden):
    """Sendet einen exakten RC-Puls (1000-2000 us) oder 0 zum Abschalten."""
    if mikrosekunden <= 0:
        pca.channels[kanal].duty_cycle = 0
        return
    # 50 Hz = 20.000 us Periodendauer
    duty = int((mikrosekunden / 20000.0) * 65535)
    pca.channels[kanal].duty_cycle = duty

def alle_aus():
    for ch in range(16):
        pca.channels[ch].duty_cycle = 0

alle_aus()

def berechne_lenkung_puls():
    mitte = max(1100, min(1900, int(state["lenkung_mitte_us"])))
    prozent = max(-100, min(100, int(state["lenkung_prozent"])))
    # Maximal +/- 350 us Ausschlag, damit nichts mechanisch anschlägt
    puls = int(mitte + (prozent / 100.0) * 350)
    return max(1000, min(2000, puls))

def lenkung_ruhezustand():
    with hardware_lock:
        set_puls_us(state["lenkung_kanal"], 0)
        if not state["motor_aktiv"]:
            state["status_text"] = f"Lenkung K{state['lenkung_kanal']} im Ruhezustand (0 µs)"

def setze_lenkung_jetzt():
    global lenkung_timer
    puls = berechne_lenkung_puls()
    k = state["lenkung_kanal"]
    with hardware_lock:
        set_puls_us(k, puls)
        if not state["motor_aktiv"]:
            state["status_text"] = f"Lenkung (Kanal {k}): {state['lenkung_prozent']}% -> {puls} µs"

    # Nach 0.6s Signal abschalten, damit das Servo nicht dauerhaft brummt
    if not state["motor_aktiv"]:
        if lenkung_timer is not None:
            lenkung_timer.cancel()
        lenkung_timer = threading.Timer(0.6, lenkung_ruhezustand)
        lenkung_timer.start()

def not_aus():
    global lenkung_timer
    if lenkung_timer is not None:
        lenkung_timer.cancel()
    with hardware_lock:
        state["motor_aktiv"] = False
        set_puls_us(state["motor_kanal"], 1500)
        time.sleep(0.05)
        alle_aus()
        state["status_text"] = "STOPP! Alle 16 Kanäle auf 0."

def fahre_ablauf(geschwindigkeit, sekunden, invertiert):
    global lenkung_timer
    if lenkung_timer is not None:
        lenkung_timer.cancel()

    m_kanal = state["motor_kanal"]
    l_kanal = state["lenkung_kanal"]

    with hardware_lock:
        state["motor_aktiv"] = True
        set_puls_us(l_kanal, berechne_lenkung_puls())

        # 1. ESC Scharfschalten (Neutral 1500 us)
        state["status_text"] = f"Kanal {m_kanal}: ESC Neutral (1500 µs)..."
        set_puls_us(m_kanal, 1500)
        time.sleep(2.0)

        if not state["motor_aktiv"]:
            return

        # 2. Motor-Puls berechnen
        prozent = max(0, min(100, geschwindigkeit))
        if invertiert:
            puls = int(1500 - (prozent / 100.0) * 500)
        else:
            puls = int(1500 + (prozent / 100.0) * 500)

        state["status_text"] = f"Kanal {m_kanal}: Fahre {prozent}% ({puls} µs) für {sekunden}s"
        set_puls_us(m_kanal, puls)

        start = time.time()
        while (time.time() - start) < sekunden:
            if not state["motor_aktiv"]:
                break
            if state["auto_stopp_ampel"] and state["ampel_erkannt"]:
                state["status_text"] = "⚠️ AMPEL -> Automatischer Stopp!"
                break
            set_puls_us(state["lenkung_kanal"], berechne_lenkung_puls())
            time.sleep(0.05)

        # 3. Stopp
        set_puls_us(m_kanal, 1500)
        time.sleep(0.3)
        alle_aus()
        state["motor_aktiv"] = False
        if "AMPEL" not in state["status_text"]:
            state["status_text"] = "Fertig (Alle Kanäle aus)"

# ==========================================
# KAMERA & YOLOv4-TINY
# ==========================================
KLASSEN = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

class KameraStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.running = True
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.cap.release()

print("[INIT] Lade YOLOv4-tiny...")
net = cv2.dnn.readNetFromDarknet("yolov4-tiny.cfg", "yolov4-tiny.weights")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]

cam = KameraStream(0)
time.sleep(1.0)

def generiere_frames():
    while True:
        start_zeit = time.time()
        erfolg, frame = cam.read()
        if not erfolg or frame is None:
            time.sleep(0.02)
            continue

        height, width, _ = frame.shape
        blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(320, 320), swapRB=True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        boxes, scores, class_ids = [], [], []
        for out in outs:
            confidences = np.max(out[:, 5:], axis=1)
            maske = confidences >= 0.35
            gefiltert = out[maske]
            for det, score, cid in zip(gefiltert, confidences[maske], np.argmax(gefiltert[:, 5:], axis=1)):
                w = int(det[2] * width)
                h = int(det[3] * height)
                x = int((det[0] * width) - w / 2)
                y = int((det[1] * height) - h / 2)
                boxes.append([x, y, w, h])
                scores.append(float(score))
                class_ids.append(int(cid))

        indices = cv2.dnn.NMSBoxes(boxes, scores, 0.35, 0.45)
        ampel_gefunden = False

        if len(indices) > 0:
            for i in indices.flatten():
                name = KLASSEN[class_ids[i]]
                sicherheit = int(scores[i] * 100)
                x, y, w, h = boxes[i]
                if name == "traffic light":
                    ampel_gefunden = True
                farbe = (0, 0, 255) if name == "traffic light" else (0, 255, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), farbe, 2)
                cv2.putText(frame, f"{name} {sicherheit}%", (x, max(y - 5, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, farbe, 2)

        state["ampel_erkannt"] = ampel_gefunden
        dauer_ms = (time.time() - start_zeit) * 1000
        fps = 1000 / dauer_ms if dauer_ms > 0 else 0
        state["fps"] = round(fps, 1)

        cv2.putText(frame, f"FPS: {fps:.1f}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

# ==========================================
# WEB-OBERFLÄCHE
# ==========================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>RC-Car Cockpit</title>
  <style>
    body { background: #121417; color: #e9ecef; font-family: -apple-system, sans-serif; margin: 0; padding: 14px; }
    .container { max-width: 960px; margin: 0 auto; display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
    @media (max-width: 768px) { .container { grid-template-columns: 1fr; } }
    .card { background: #1e2126; border-radius: 12px; padding: 14px; }
    h2 { margin-top: 0; font-size: 1rem; color: #adb5bd; border-bottom: 1px solid #2c3036; padding-bottom: 6px; }
    img { width: 100%; border-radius: 8px; background: #000; display: block; }
    .row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; gap: 8px; }
    input[type=range] { width: 100%; accent-color: #3b82f6; }
    select, input[type=number] { background: #141619; color: #fff; border: 1px solid #374151; border-radius: 6px; padding: 6px; font-size: 0.95rem; }
    .btn { width: 100%; padding: 12px; border: none; border-radius: 8px; font-size: 1rem; font-weight: 600; cursor: pointer; margin-bottom: 8px; }
    .btn-small { width: auto; padding: 6px 12px; font-size: 0.85rem; margin-bottom: 0; background: #374151; color: #fff; }
    .btn-go { background: #10b981; color: #fff; }
    .btn-stop { background: #ef4444; color: #fff; font-size: 1.05rem; padding: 14px; }
    .status-box { background: #141619; padding: 10px; border-radius: 8px; font-family: monospace; font-size: 0.85rem; border-left: 4px solid #3b82f6; }
    .badge { padding: 3px 8px; border-radius: 6px; font-size: 0.8rem; font-weight: bold; }
    .badge-ok { background: #065f46; color: #a7f3d0; }
    .badge-warn { background: #991b1b; color: #fecaca; }
  </style>
</head>
<body>
  <div class="container">
    <div class="card">
      <h2>Live-Kamera &amp; YOLOv4-tiny</h2>
      <img src="/video_feed" alt="Kamera Stream">
      <div style="margin-top: 10px; display: flex; justify-content: space-between; align-items: center;">
        <span>FPS: <strong id="fps_val">0.0</strong></span>
        <span id="ampel_badge" class="badge badge-ok">Keine Ampel</span>
      </div>
    </div>

    <div class="card">
      <h2>Kanal-Zuordnung (PCA9685 Pins)</h2>
      <div class="row">
        <span>Lenkung Kanal:
          <select id="lenk_ch" onchange="updateChannels()">
            <option value="0" selected>0</option><option value="1">1</option><option value="2">2</option><option value="3">3</option><option value="4">4</option><option value="5">5</option>
          </select>
        </span>
        <span>Motor Kanal:
          <select id="mot_ch" onchange="updateChannels()">
            <option value="0">0</option><option value="1" selected>1</option><option value="2">2</option><option value="3">3</option><option value="4">4</option><option value="5">5</option>
          </select>
        </span>
      </div>

      <h2>Lenkung &amp; Mitte</h2>
      <div class="row">
        <span>⬅ Links</span>
        <span><strong id="steer_out">0</strong> %</span>
        <span>Rechts ➡</span>
      </div>
      <div class="row">
        <input type="range" id="steer" min="-100" max="100" value="0" oninput="onSteerChange(this.value)">
        <button class="btn btn-small" onclick="resetSteer()">Mitte</button>
      </div>
      <div class="row">
        <span>Mitte (µs):</span>
        <div>
          <input type="number" id="mitte_us" min="1100" max="1900" step="10" value="1500" style="width:80px;" onchange="onSteerChange(document.getElementById('steer').value)">
          <button class="btn btn-small" onclick="resetSteer()">Testen</button>
        </div>
      </div>

      <h2>Antrieb</h2>
      <div class="row">
        <span>Geschwindigkeit:</span>
        <span><strong id="speed_out">40</strong> %</span>
      </div>
      <input type="range" id="speed" min="10" max="100" value="40" oninput="document.getElementById('speed_out').innerText=this.value">

      <div class="row" style="margin-top:8px;">
        <span>Fahrzeit:</span>
        <span><strong id="time_out">2.0</strong> Sek.</span>
      </div>
      <input type="range" id="duration" min="0.5" max="10.0" step="0.5" value="2.0" oninput="document.getElementById('time_out').innerText=parseFloat(this.value).toFixed(1)">

      <div style="margin: 10px 0;">
        <label style="display:block; margin-bottom:6px;"><input type="checkbox" id="invert"> Vorwärts invertiert (&lt; 1500 µs)</label>
        <label style="display:block;"><input type="checkbox" id="auto_ampel" checked onchange="updateSettings()"> Auto-Stopp bei Ampel</label>
      </div>

      <button class="btn btn-go" onclick="startFahren()">▶ Vorwärts fahren</button>
      <button class="btn btn-stop" onclick="notAus()">🛑 NOT-AUS (Alle Kanäle 0)</button>

      <div class="status-box" id="status_text">Status: Bereit</div>
    </div>
  </div>

  <script>
    function updateChannels() {
      fetch('/channels', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          lenkung_kanal: parseInt(document.getElementById('lenk_ch').value),
          motor_kanal: parseInt(document.getElementById('mot_ch').value)
        })
      });
    }
    function onSteerChange(val) {
      document.getElementById('steer_out').innerText = val;
      fetch('/steer', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ prozent: parseInt(val), mitte_us: parseInt(document.getElementById('mitte_us').value) })
      });
    }
    function resetSteer() {
      document.getElementById('steer').value = 0;
      onSteerChange(0);
    }
    function startFahren() {
      fetch('/drive', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          speed: parseInt(document.getElementById('speed').value),
          duration: parseFloat(document.getElementById('duration').value),
          invert: document.getElementById('invert').checked
        })
      });
    }
    function notAus() { fetch('/stop', { method: 'POST' }); }
    function updateSettings() {
      fetch('/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ auto_ampel: document.getElementById('auto_ampel').checked })
      });
    }
    setInterval(() => {
      fetch('/status').then(r => r.json()).then(data => {
        document.getElementById('status_text').innerText = "Status: " + data.status_text;
        document.getElementById('fps_val').innerText = data.fps;
        const badge = document.getElementById('ampel_badge');
        badge.className = data.ampel_erkannt ? "badge badge-warn" : "badge badge-ok";
        badge.innerText = data.ampel_erkannt ? "🚦 AMPEL ERKANNT" : "Keine Ampel";
      });
    }, 300);
  </script>
</body>
</html>
"""

@app.route('/')
def index(): return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed(): return Response(generiere_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/channels', methods=['POST'])
def channels():
    data = request.json
    state["lenkung_kanal"] = int(data.get("lenkung_kanal", 0))
    state["motor_kanal"] = int(data.get("motor_kanal", 1))
    state["status_text"] = f"Kanäle gesetzt: Lenkung={state['lenkung_kanal']}, Motor={state['motor_kanal']}"
    return jsonify({"ok": True})

@app.route('/steer', methods=['POST'])
def steer():
    data = request.json
    state["lenkung_prozent"] = int(data.get("prozent", 0))
    state["lenkung_mitte_us"] = int(data.get("mitte_us", 1500))
    setze_lenkung_jetzt()
    return jsonify({"ok": True})

@app.route('/drive', methods=['POST'])
def drive():
    data = request.json
    if not state["motor_aktiv"]:
        threading.Thread(target=fahre_ablauf, args=(data.get("speed", 40), data.get("duration", 2.0), data.get("invert", False)), daemon=True).start()
    return jsonify({"ok": True})

@app.route('/stop', methods=['POST'])
def stop():
    not_aus()
    return jsonify({"ok": True})

@app.route('/settings', methods=['POST'])
def settings():
    state["auto_stopp_ampel"] = bool(request.json.get("auto_ampel", True))
    return jsonify({"ok": True})

@app.route('/status')
def status(): return jsonify(state)

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        not_aus()
        cam.stop()
        pca.deinit()

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Globaler Zustand für UI & Auto-Stopp
state = {
    "ampel_erkannt": False,
    "auto_stopp_ampel": True,
    "status_text": "Bereit (Hardware im Sleep-Modus)",
    "fps": 0.0,
    "motor_aktiv": False,
    "lenkung_prozent": 0,      # -100 (Links) bis +100 (Rechts)
    "lenkung_mitte_us": 1500   # Mechanische Mittelstellung in µs
}
hardware_lock = threading.Lock()
lenkung_timer = None

def pca_hardware_sleep(schlafen=True):
    """Schaltet alle PWM-Ausgänge des PCA9685 hardwareseitig komplett ab oder ein."""
    while not i2c.try_lock():
        pass
    try:
        wert = 0x10 if schlafen else 0x00
        i2c.writeto(0x40, bytes([0x00, wert]))
    finally:
        i2c.unlock()

def set_puls_us(kanal, mikrosekunden):
    if mikrosekunden <= 0:
        pca.channels[kanal].duty_cycle = 0
        return
    duty = int((mikrosekunden / 20000.0) * 65535)
    pca.channels[kanal].duty_cycle = duty

def berechne_lenkung_puls():
    """Berechnet den exakten Puls aus mechanischer Mitte und gewünschtem Lenkeinschlag (-100..+100%)."""
    mitte = max(1100, min(1900, int(state["lenkung_mitte_us"])))
    prozent = max(-100, min(100, int(state["lenkung_prozent"])))
    puls = int(mitte + (prozent / 100.0) * LENKUNG_MAX_AUSSCHLAG_US)
    return max(1000, min(2000, puls))

def lenkung_ruhezustand():
    """Schaltet das Lenksignal nach kurzer Stellzeit ab, damit das Servo nicht brummt."""
    with hardware_lock:
        set_puls_us(LENKUNG_KANAL, 0)
        if not state["motor_aktiv"]:
            pca_hardware_sleep(True)
            state["status_text"] = "Lenkung im Ruhezustand (Sleep)"

def setze_lenkung_jetzt():
    """Bewegt das Lenkservo kurz zur Zielposition und legt es danach wieder schlafen."""
    global lenkung_timer
    puls = berechne_lenkung_puls()
    with hardware_lock:
        pca_hardware_sleep(False)
        set_puls_us(LENKUNG_KANAL, puls)
        if not state["motor_aktiv"]:
            state["status_text"] = f"Lenkung: {state['lenkung_prozent']}% ({puls} µs)"

    # Falls der Motor gerade NICHT fährt, Servo nach 0.4s wieder stromlos/signallos machen
    if not state["motor_aktiv"]:
        if lenkung_timer is not None:
            lenkung_timer.cancel()
        lenkung_timer = threading.Timer(0.4, lenkung_ruhezustand)
        lenkung_timer.start()

def not_aus_und_sleep():
    """Sofortiger Stopp und Hardware-Sleep für Lenkung & Motor."""
    global lenkung_timer
    if lenkung_timer is not None:
        lenkung_timer.cancel()
    with hardware_lock:
        state["motor_aktiv"] = False
        set_puls_us(MOTOR_KANAL, 1500)
        time.sleep(0.05)
        set_puls_us(MOTOR_KANAL, 0)
        set_puls_us(LENKUNG_KANAL, 0)
        pca_hardware_sleep(True)
        state["status_text"] = "STOPP / Hardware-Sleep aktiv"

# Beim Start sicherheitshalber alles ausschalten
not_aus_und_sleep()

def fahre_ablauf(geschwindigkeit, sekunden, invertiert):
    """Führt den Fahrbefehl inklusive eingestelltem Lenkwinkel aus."""
    global lenkung_timer
    if lenkung_timer is not None:
        lenkung_timer.cancel()

    with hardware_lock:
        state["motor_aktiv"] = True
        pca_hardware_sleep(False)

        # Lenkung auf den im UI eingestellten Winkel bringen
        lenk_puls = berechne_lenkung_puls()
        set_puls_us(LENKUNG_KANAL, lenk_puls)

        # 1. ESC Scharfschalten (Neutral)
        state["status_text"] = f"ESC scharfschalten (Lenkung {lenk_puls} µs)..."
        set_puls_us(MOTOR_KANAL, 1500)
        time.sleep(2.0)

        if not state["motor_aktiv"]:
            return

        # 2. Motor-Puls berechnen
        prozent = max(0, min(100, geschwindigkeit))
        if invertiert:
            puls = int(1500 - (prozent / 100.0) * 500)
        else:
            puls = int(1500 + (prozent / 100.0) * 500)

        state["status_text"] = f"Fahre ({prozent}% | {puls} µs) für {sekunden}s..."
        set_puls_us(MOTOR_KANAL, puls)

        # Während der Fahrt laufend auf Not-Aus, Ampel oder Lenkungsänderungen reagieren
        start = time.time()
        while (time.time() - start) < sekunden:
            if not state["motor_aktiv"]:
                break
            if state["auto_stopp_ampel"] and state["ampel_erkannt"]:
                state["status_text"] = "⚠️ AMPEL ERKANNT -> Automatischer Stopp!"
                break
            # Falls der Nutzer während der Fahrt am Lenkungs-Regler dreht, live aktualisieren:
            set_puls_us(LENKUNG_KANAL, berechne_lenkung_puls())
            time.sleep(0.05)

        # 3. Anhalten & Hardware-Sleep
        set_puls_us(MOTOR_KANAL, 1500)
        time.sleep(0.3)
        set_puls_us(MOTOR_KANAL, 0)
        set_puls_us(LENKUNG_KANAL, 0)
        pca_hardware_sleep(True)
        state["motor_aktiv"] = False
        if "AMPEL" not in state["status_text"]:
            state["status_text"] = "Fertig (Hardware im Sleep-Modus)"

# ==========================================
# KAMERA & YOLOv4-TINY (MULTI-THREADED)
# ==========================================
KLASSEN = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

class KameraStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.running = True
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.cap.release()

print("[INIT] Lade YOLOv4-tiny...")
net = cv2.dnn.readNetFromDarknet("yolov4-tiny.cfg", "yolov4-tiny.weights")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]

cam = KameraStream(0)
time.sleep(1.0)

def generiere_frames():
    while True:
        start_zeit = time.time()
        erfolg, frame = cam.read()
        if not erfolg or frame is None:
            time.sleep(0.02)
            continue

        height, width, _ = frame.shape
        blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(320, 320), swapRB=True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        boxes, scores, class_ids = [], [], []
        for out in outs:
            confidences = np.max(out[:, 5:], axis=1)
            maske = confidences >= 0.35
            gefiltert = out[maske]
            for det, score, cid in zip(gefiltert, confidences[maske], np.argmax(gefiltert[:, 5:], axis=1)):
                w = int(det[2] * width)
                h = int(det[3] * height)
                x = int((det[0] * width) - w / 2)
                y = int((det[1] * height) - h / 2)
                boxes.append([x, y, w, h])
                scores.append(float(score))
                class_ids.append(int(cid))

        indices = cv2.dnn.NMSBoxes(boxes, scores, 0.35, 0.45)
        ampel_gefunden = False

        if len(indices) > 0:
            for i in indices.flatten():
                name = KLASSEN[class_ids[i]]
                sicherheit = int(scores[i] * 100)
                x, y, w, h = boxes[i]
                if name == "traffic light":
                    ampel_gefunden = True
                farbe = (0, 0, 255) if name == "traffic light" else (0, 255, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), farbe, 2)
                cv2.putText(frame, f"{name} {sicherheit}%", (x, max(y - 5, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, farbe, 2)

        state["ampel_erkannt"] = ampel_gefunden
        dauer_ms = (time.time() - start_zeit) * 1000
        fps = 1000 / dauer_ms if dauer_ms > 0 else 0
        state["fps"] = round(fps, 1)

        cv2.putText(frame, f"FPS: {fps:.1f}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

# ==========================================
# WEB-OBERFLÄCHE (HTML / CSS / JS)
# ==========================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>RC-Car Cockpit</title>
  <style>
    body { background: #121417; color: #e9ecef; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; margin: 0; padding: 16px; }
    .container { max-width: 960px; margin: 0 auto; display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
    @media (max-width: 768px) { .container { grid-template-columns: 1fr; } }
    .card { background: #1e2126; border-radius: 12px; padding: 16px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
    h2 { margin-top: 0; font-size: 1.05rem; color: #adb5bd; border-bottom: 1px solid #2c3036; padding-bottom: 8px; }
    img { width: 100%; border-radius: 8px; background: #000; display: block; }
    .control-group { margin-bottom: 14px; }
    label { display: flex; justify-content: space-between; font-size: 0.9rem; margin-bottom: 4px; }
    input[type=range] { width: 100%; accent-color: #3b82f6; }
    input[type=number] { background: #141619; color: #fff; border: 1px solid #374151; border-radius: 6px; padding: 6px 10px; width: 90px; font-size: 0.95rem; }
    .steer-row { display: flex; gap: 8px; align-items: center; }
    .trim-row { display: flex; justify-content: space-between; align-items: center; background: #141619; padding: 8px 12px; border-radius: 8px; margin-bottom: 14px; }
    .checkbox-row { display: flex; align-items: center; gap: 8px; font-size: 0.9rem; margin-bottom: 10px; cursor: pointer; }
    .btn { width: 100%; padding: 12px; border: none; border-radius: 8px; font-size: 1rem; font-weight: 600; cursor: pointer; margin-bottom: 10px; transition: opacity 0.2s; }
    .btn:active { opacity: 0.8; }
    .btn-small { width: auto; padding: 6px 12px; font-size: 0.85rem; margin-bottom: 0; background: #374151; color: #fff; }
    .btn-go { background: #10b981; color: #fff; }
    .btn-stop { background: #ef4444; color: #fff; font-size: 1.1rem; padding: 16px; }
    .status-box { background: #141619; padding: 10px; border-radius: 8px; font-family: monospace; font-size: 0.85rem; margin-top: 8px; border-left: 4px solid #3b82f6; }
    .badge { display: inline-block; padding: 3px 8px; border-radius: 6px; font-size: 0.8rem; font-weight: bold; }
    .badge-ok { background: #065f46; color: #a7f3d0; }
    .badge-warn { background: #991b1b; color: #fecaca; }
  </style>
</head>
<body>
  <div class="container">
    <!-- Linke Spalte: Live Kamera -->
    <div class="card">
      <h2>Live-Kamera &amp; YOLOv4-tiny</h2>
      <img src="/video_feed" alt="Kamera Stream">
      <div style="margin-top: 10px; display: flex; justify-content: space-between; align-items: center;">
        <span>FPS: <strong id="fps_val">0.0</strong></span>
        <span id="ampel_badge" class="badge badge-ok">Keine Ampel</span>
      </div>
    </div>

    <!-- Rechte Spalte: Lenkung & Antrieb -->
    <div class="card">
      <h2>Lenkung &amp; Kalibrierung</h2>

      <div class="control-group">
        <label>
          <span>⬅ Links</span>
          <span>Lenkung: <strong id="steer_out">0</strong> %</span>
          <span>Rechts ➡</span>
        </label>
        <div class="steer-row">
          <input type="range" id="steer" min="-100" max="100" value="0" oninput="onSteerChange(this.value)">
          <button class="btn btn-small" onclick="resetSteer()">Mitte</button>
        </div>
      </div>

      <div class="trim-row">
        <span style="font-size: 0.9rem;">Mechanische Mitte (µs):</span>
        <div style="display: flex; gap: 6px; align-items: center;">
          <input type="number" id="mitte_us" min="1100" max="1900" step="10" value="1500" onchange="onMitteChange(this.value)">
          <button class="btn btn-small" onclick="testMitte()">Testen</button>
        </div>
      </div>

      <h2>Antrieb &amp; Einstellungen</h2>

      <div class="control-group">
        <label>Geschwindigkeit: <span><strong id="speed_out">30</strong> %</span></label>
        <input type="range" id="speed" min="10" max="100" value="30" oninput="document.getElementById('speed_out').innerText=this.value">
      </div>

      <div class="control-group">
        <label>Fahrzeit: <span><strong id="time_out">2.0</strong> Sek.</span></label>
        <input type="range" id="duration" min="0.5" max="10.0" step="0.5" value="2.0" oninput="document.getElementById('time_out').innerText=parseFloat(this.value).toFixed(1)">
      </div>

      <label class="checkbox-row">
        <input type="checkbox" id="invert" checked>
        Vorwärts invertiert (&lt; 1500 µs)
      </label>

      <label class="checkbox-row">
        <input type="checkbox" id="auto_ampel" checked onchange="updateSettings()">
        Automatisch anhalten bei Ampel
      </label>

      <button class="btn btn-go" onclick="startFahren()">▶ Vorwärts fahren</button>
      <button class="btn btn-stop" onclick="notAus()">🛑 NOT-AUS &amp; SLEEP (Alle Kanäle aus)</button>

      <div class="status-box" id="status_text">Status: Bereit</div>
    </div>
  </div>

  <script>
    function onSteerChange(val) {
      document.getElementById('steer_out').innerText = val;
      sendSteer(parseInt(val), parseInt(document.getElementById('mitte_us').value));
    }

    function resetSteer() {
      document.getElementById('steer').value = 0;
      document.getElementById('steer_out').innerText = 0;
      sendSteer(0, parseInt(document.getElementById('mitte_us').value));
    }

    function onMitteChange(val) {
      sendSteer(parseInt(document.getElementById('steer').value), parseInt(val));
    }

    function testMitte() {
      resetSteer();
    }

    function sendSteer(prozent, mitte_us) {
      fetch('/steer', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ prozent: prozent, mitte_us: mitte_us })
      });
    }

    function startFahren() {
      const payload = {
        speed: parseInt(document.getElementById('speed').value),
        duration: parseFloat(document.getElementById('duration').value),
        invert: document.getElementById('invert').checked
      };
      fetch('/drive', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload)
      });
    }

    function notAus() {
      fetch('/stop', { method: 'POST' });
    }

    function updateSettings() {
      fetch('/settings', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ auto_ampel: document.getElementById('auto_ampel').checked })
      });
    }

    setInterval(() => {
      fetch('/status').then(r => r.json()).then(data => {
        document.getElementById('status_text').innerText = "Status: " + data.status_text;
        document.getElementById('fps_val').innerText = data.fps;
        const badge = document.getElementById('ampel_badge');
        if (data.ampel_erkannt) {
          badge.className = "badge badge-warn";
          badge.innerText = "🚦 AMPEL ERKANNT";
        } else {
          badge.className = "badge badge-ok";
          badge.innerText = "Keine Ampel";
        }
      });
    }, 300);
  </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(generiere_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/steer', methods=['POST'])
def steer():
    data = request.json
    state["lenkung_prozent"] = int(data.get("prozent", 0))
    state["lenkung_mitte_us"] = int(data.get("mitte_us", 1500))
    setze_lenkung_jetzt()
    return jsonify({"ok": True})

@app.route('/drive', methods=['POST'])
def drive():
    data = request.json
    if not state["motor_aktiv"]:
        threading.Thread(
            target=fahre_ablauf,
            args=(data.get("speed", 30), data.get("duration", 2.0), data.get("invert", True)),
            daemon=True
        ).start()
    return jsonify({"ok": True})

@app.route('/stop', methods=['POST'])
def stop():
    not_aus_und_sleep()
    return jsonify({"ok": True})

@app.route('/settings', methods=['POST'])
def settings():
    data = request.json
    state["auto_stopp_ampel"] = bool(data.get("auto_ampel", True))
    return jsonify({"ok": True})

@app.route('/status')
def status():
    return jsonify(state)

if __name__ == '__main__':
    print("[READY] RC-Car Cockpit gestartet! Öffne im Browser: http://<DEINE-PI-IP>:5000")
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        not_aus_und_sleep()
        cam.stop()
        pca.deinit()
