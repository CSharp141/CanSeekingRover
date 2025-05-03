#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <vector>
#include <utility>

// —— Wi-Fi AP Configuration ——  
const char* AP_SSID = "ESP32-Rover";
const char* AP_PW   = "roverpw123";
WebServer server(80);

// —— Pins & Pulse Ranges ——  
const int ESC_PIN      = 18;
const int STEER_PIN    = 19;
const int escMin_us    = 1000;
const int escMax_us    = 2000;
const int steerMin_us  = 1300;
const int steerMax_us  = 2000;

// —— FSM Modes ——  
enum class Mode : uint8_t { Neutral, Forward, Seek, Pickup, Return };
volatile Mode currentMode = Mode::Neutral;

// Shared actuator commands (0–100 %)  
volatile int throttlePct = 50;  // 50 = neutral/brake
volatile int steerPct    = 50;  // 50 = straight

// Path memory for Return mode  
static std::vector<std::pair<uint8_t,uint8_t>> path;
static size_t replayIndex = 0;

// Forward-mode timing  
static unsigned long forwardDurationMs = 0;
static unsigned long forwardStartTime  = 0;
static bool forwardActive              = false;

// Seek-mode timing & recovery  
static constexpr unsigned long SEEK_THROTTLE_DURATION_MS = 1000;
static constexpr unsigned long SEEK_RECOVERY_MS          = 500;
static bool  seekThrottleActive       = false;
static unsigned long seekThrottleStart = 0;
static unsigned long lastBurstEnd      = 0;

// Debug: last values parsed from UART0  
volatile int lastSeekRaw   = -1;
volatile int lastSeekAngle = -1;

// Return-stage extra straight reverse  
static bool extraReverseActive      = false;
static unsigned long extraReverseStart = 0;

// Reverse scaling constants  
static constexpr float REVERSE_SCALE    = 3.0f;
static constexpr int   REVERSE_HOLD_PCT = 25;

inline int pctToPulse(int pct, int mn, int mx) {
  pct = constrain(pct, 0, 100);
  return mn + (mx - mn) * pct / 100;
}

// —— Motor & Steering Tasks (50 Hz) ——  
void motorTask(void*) {
  Servo esc;
  esc.attach(ESC_PIN, escMin_us, escMax_us);
  for (;;) {
    esc.writeMicroseconds(pctToPulse(throttlePct, escMin_us, escMax_us));
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void steerTask(void*) {
  Servo steering;
  steering.attach(STEER_PIN, steerMin_us, steerMax_us);
  for (;;) {
    int invertedPct = 100 - steerPct;
    steering.writeMicroseconds(
      pctToPulse(invertedPct, steerMin_us, steerMax_us)
    );
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// —— Control Task ——  
void controlTask(void*) {
  static Mode lastMode = Mode::Neutral;

  for (;;) {
    // on mode transition
    if (currentMode != lastMode) {
      if (currentMode == Mode::Seek) {
        while (Serial.available()) Serial.read();
        seekThrottleActive = false;
        lastSeekRaw   = -1;
        lastSeekAngle = -1;
        lastBurstEnd  = millis();
      }
      lastMode = currentMode;
    }

    switch (currentMode) {

      case Mode::Neutral:
        throttlePct = 50;
        steerPct    = 50;
        break;

      case Mode::Forward:
        if (!forwardActive) {
          forwardStartTime = millis();
          forwardActive    = true;
        }
        if (millis() - forwardStartTime >= forwardDurationMs) {
          currentMode       = Mode::Neutral;
          forwardActive     = false;
          forwardDurationMs = 0;
          break;
        }
        steerPct    = 50;
        throttlePct = 60;
        path.emplace_back(steerPct, throttlePct);
        break;

      case Mode::Seek: {
        // 1) Read & update steerPct
        if (Serial.available()) {
          String pctStr = Serial.readStringUntil(',');
          String angStr = Serial.readStringUntil('\n');
          int pct = pctStr.toInt();
          int ang = angStr.toInt();
          lastSeekRaw   = pct;
          lastSeekAngle = ang;
          if (pct >= 0 && pct <= 100) {
            steerPct = pct;
            // only arm a new burst if recovery time has passed
            if (!seekThrottleActive &&
                (millis() - lastBurstEnd >= SEEK_RECOVERY_MS)) {
              seekThrottleActive = true;
              seekThrottleStart  = millis();
            }
          }
        }

        // 2) Drive throttle or brake
        if (seekThrottleActive) {
          if (millis() - seekThrottleStart < SEEK_THROTTLE_DURATION_MS) {
            throttlePct = 55;
          } else {
            seekThrottleActive = false;
            throttlePct        = 50;           // full stop/brake
            lastBurstEnd       = millis();
          }
        } else {
          throttlePct = 50;                   // hold full brake
        }

        // 3) Record only when moving
        if (throttlePct > 50) {
          path.emplace_back(steerPct, throttlePct);
        }
        break;
      }

      case Mode::Pickup:
        throttlePct = 50;
        steerPct    = 50;
        break;

      case Mode::Return:
        if (replayIndex < path.size()) {
          auto &p = path[path.size() - 1 - replayIndex++];
          steerPct = p.first;
          int delta = p.second - 50;
          int inv   = int(delta * REVERSE_SCALE + 0.5);
          throttlePct = constrain(50 - inv, 0, 100);
        } else {
          if (!extraReverseActive) {
            extraReverseActive = true;
            extraReverseStart  = millis();
          }
          if (millis() - extraReverseStart < 5000) {
            steerPct    = 50;
            throttlePct = REVERSE_HOLD_PCT;
          } else {
            extraReverseActive = false;
            currentMode        = Mode::Neutral;
            path.clear();
            replayIndex        = 0;
          }
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// —— HTML + JS ——  
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
  body{margin:0;padding:1em;font-family:sans-serif;text-align:center;}
  #controls{margin-top:2em;}
  #controls input{width:4em;margin:0 .5em;}
  #controls button{margin:.3em;padding:.6em 1em;font-size:1em;border:none;border-radius:4px;background:#4285F4;color:#fff;}
  #debug{margin-top:1em;font-family:monospace;color:#333;}
</style>
</head>
<body>
  <h3>ESP32-Rover Autonomous Control</h3>
  <div id="controls">
    <label>Forward for <input id="forwardTime" type="number" min="1" value="5"> s</label><br>
    <button onclick="setMode('forward')">Go Forward</button>
    <button onclick="setMode('neutral')">Stop</button>
    <button onclick="setMode('seek')">Can Seek</button>
    <button onclick="setMode('pickup')">Pickup</button>
    <button onclick="setMode('return')">Return</button>
  </div>
  <div id="debug">
    <div>Last percentage: <span id="rawv">–</span>%</div>
    <div>Last angle:      <span id="angv">–</span>°</div>
  </div>
  <script>
    function setMode(m) {
      let url = '/mode?m='+m;
      if (m==='forward') {
        const t = document.getElementById('forwardTime').value||0;
        url += '&t='+t;
      }
      fetch(url);
    }
    // Poll the raw‐UART endpoint every 200 ms
    setInterval(()=>{
      fetch('/seekraw')
        .then(r => r.text())
        .then(t => {
          const [pct, ang] = t.split(',');
          document.getElementById('rawv').textContent = pct;
          document.getElementById('angv').textContent = ang;
        });
    },200);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleSeekRaw() {
  String out = String(lastSeekRaw) + "," + String(lastSeekAngle);
  server.send(200, "text/plain", out);
}

void handleMode() {
  String m = server.arg("m");
  if (m == "forward") {
    if (server.hasArg("t"))
      forwardDurationMs = server.arg("t").toInt() * 1000UL;
    forwardActive = false;
    currentMode   = Mode::Forward;
  }
  else if (m == "neutral") currentMode = Mode::Neutral;
  else if (m == "seek")    currentMode = Mode::Seek;
  else if (m == "pickup")  currentMode = Mode::Pickup;
  else if (m == "return") {
    currentMode        = Mode::Return;
    replayIndex        = 0;
    extraReverseActive = false;
  }
  server.send(200, "text/plain", "OK");
}

void setup(){
  Serial.begin(115200);
  Serial.setTimeout(20);
  WiFi.softAP(AP_SSID, AP_PW);

  // ESC calibration (neutral only)
  {
    Servo escCal;
    escCal.attach(ESC_PIN, escMin_us, escMax_us);
    escCal.writeMicroseconds(pctToPulse(50, escMin_us, escMax_us));
    delay(3000);
    escCal.detach();
  }

  server.on("/",        handleRoot);
  server.on("/seekraw", handleSeekRaw);
  server.on("/mode",    handleMode);
  server.begin();

  xTaskCreate(motorTask,   "motor",   2048, nullptr, 2, nullptr);
  xTaskCreate(steerTask,   "steer",   2048, nullptr, 2, nullptr);
  xTaskCreate(controlTask, "control", 4096, nullptr, 3, nullptr);
}

void loop(){
  server.handleClient();
}


