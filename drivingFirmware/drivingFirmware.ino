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
const int ESC_PIN     = 18;   // ESC throttle signal  
const int STEER_PIN   = 19;   // Steering servo signal  
const int escMin_us   = 1000; // full reverse
const int escMax_us   = 2000; // full forward
const int steerMin_us = 1300; // full left
const int steerMax_us = 2000; // full right

// —— Autonomous State Machine ——  
enum class Mode : uint8_t {
  Neutral,
  Forward,
  Seek,
  Pickup,
  Return
};
volatile Mode currentMode = Mode::Neutral;

// Shared actuator commands (0–100 %)
volatile int throttlePct = 50;  // 50 = neutral
volatile int steerPct    = 50;  // 50 = straight

// Path memory for Return mode
static std::vector<std::pair<uint8_t,uint8_t>> path;
static size_t replayIndex = 0;

// Forward-mode timing
static unsigned long forwardDurationMs = 0;
static unsigned long forwardStartTime  = 0;
static bool forwardActive              = false;

// Return-stage extra straight reverse  
static bool extraReverseActive    = false;
static unsigned long extraReverseStart = 0;

// Tuning constants for reverse scaling  
static constexpr float REVERSE_SCALE   = 1.5f;  // adjust until reverse ≈ forward speed  
static constexpr int   REVERSE_HOLD_PCT = 30;    // throttle % for final 5 s reverse  

inline int pctToPulse(int pct, int mn, int mx) {
  pct = constrain(pct, 0, 100);
  return mn + (mx - mn) * pct / 100;
}

// —— Motor & Steering Tasks (50 Hz) ——  
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
    steering.writeMicroseconds(pctToPulse(steerPct, steerMin_us, steerMax_us));
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// —— Control Task ——  
void controlTask(void*) {
  for (;;) {
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
        if (forwardActive
            && (millis() - forwardStartTime >= forwardDurationMs)) {
          currentMode     = Mode::Neutral;
          forwardActive   = false;
          forwardDurationMs = 0;
          break;
        }
        steerPct    = 50;
        throttlePct = 60;  // adjust speed
        path.emplace_back(steerPct, throttlePct);
        break;

      case Mode::Seek:
        if (Serial.available()) {
          int val = Serial.parseInt();
          if (val >= 0 && val <= 100) {
            steerPct = val;
          }
        }
        throttlePct = 55;  // slow forward
        path.emplace_back(steerPct, throttlePct);
        break;

      case Mode::Pickup:
        throttlePct = 50;
        steerPct    = 50;
        break;

      case Mode::Return:
        if (replayIndex < path.size()) {
          auto &p = path[path.size() - 1 - replayIndex++];
          steerPct    = 100 - p.first;   // mirror steering
          throttlePct = 100 - p.second;  // invert throttle
        }  else {
          // after path replay, do 5 s straight reverse
          if (!extraReverseActive) {
            extraReverseActive  = true;
            extraReverseStart   = millis();
          }
          if (millis() - extraReverseStart < 5000) {
            steerPct    = 50;
            throttlePct = REVERSE_HOLD_PCT;
          } else {
            // sequence complete
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

// —— HTML + JS (mode buttons + forward-time input) ——  
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    body { margin:0; padding:1em; font-family:sans-serif; text-align:center; }
    #controls { margin-top:2em; }
    #controls label { font-size:1em; }
    #controls input { width:4em; margin:0 0.5em; }
    #controls button {
      margin:0.3em; padding:0.6em 1em;
      font-size:1em; border:none; border-radius:4px;
      background:#4285F4; color:#fff;
    }
  </style>
</head>
<body>
  <h3>ESP32-Rover Autonomous Control</h3>
  <div id="controls">
    <label>
      Forward for
      <input id="forwardTime" type="number" min="1" value="5"> s
    </label>
    <br>
    <button onclick="setMode('forward')">Go Forward</button>
    <button onclick="setMode('neutral')">Stop</button>
    <button onclick="setMode('seek')">Can Seek</button>
    <button onclick="setMode('pickup')">Pickup</button>
    <button onclick="setMode('return')">Return</button>
  </div>
  <script>
    function setMode(m) {
      let url = '/mode?m=' + m;
      if (m === 'forward') {
        const t = document.getElementById('forwardTime').value || 0;
        url += '&t=' + t;
      }
      fetch(url);
    }
  </script>
</body>
</html>
)rawliteral";

// —— HTTP handlers ——  
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleMode() {
  String m = server.arg("m");
  if (m == "forward") {
    if (server.hasArg("t")) {
      forwardDurationMs = server.arg("t").toInt() * 1000UL;
    }
    forwardActive = false;
    currentMode   = Mode::Forward;

  } else if (m == "neutral") {
    currentMode = Mode::Neutral;

  } else if (m == "seek") {
    currentMode = Mode::Seek;

  } else if (m == "pickup") {
    currentMode = Mode::Pickup;

  } else if (m == "return") {
    currentMode = Mode::Return;
    replayIndex = 0;
  }
  server.send(200, "text/plain", "OK");
}

void setup() {
  // UART0 for seeker input (and optional debug)
  Serial.begin(115200);

  WiFi.softAP(AP_SSID, AP_PW);
  Serial.printf("AP \"%s\" running at %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());

  // ESC self-calibration at neutral (3 s)
  {
    Servo escCal;
    escCal.attach(ESC_PIN, escMin_us, escMax_us);
    escCal.writeMicroseconds(pctToPulse(50, escMin_us, escMax_us));
    Serial.println("Calibrating ESC — hold neutral for 3 s …");
    delay(3000);
    escCal.detach();
    Serial.println("ESC calibration done.");
  }

  server.on("/",     handleRoot);
  server.on("/mode", handleMode);
  server.begin();

  xTaskCreate(motorTask,   "motor",   2048, nullptr, 2, nullptr);
  xTaskCreate(steerTask,   "steer",   2048, nullptr, 2, nullptr);
  xTaskCreate(controlTask, "control", 4096, nullptr, 3, nullptr);
}

void loop() {
  server.handleClient();
}

