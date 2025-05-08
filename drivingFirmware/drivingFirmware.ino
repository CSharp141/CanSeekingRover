#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
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
const int TRIG_PIN    = 26;
const int ECHO_PIN    = 27;  
const int STEPPER_PIN = 25;

const int escMin_us    = 1000;
const int escMax_us    = 2000;
const int steerMin_us  = 1300;
const int steerMax_us  = 2000;

const char* modeNames[] = {
  "Neutral", "Forward", "Seek", "Pickup", "Retrieve", "Return"
};


//define sound speed in cm/uS
#define SOUND_SPEED 0.034

// cap ultrasonic range to 300 cm → ~17.65 ms round-trip timeout
#define MAX_DIST_CM         300.0f
#define MAX_ECHO_TIMEOUT_US ((unsigned long)(2 * MAX_DIST_CM / SOUND_SPEED))

// For the Ultrasonic sensor (US)
Servo usServo;

long PulseDuration;
float distanceCm;

struct shortestDist{
  float distance;
  int angle;
};

// —— FSM Modes ——  
enum class Mode : uint8_t { Neutral, Forward, Seek, Pickup, Retrieve, Return };
volatile Mode currentMode = Mode::Neutral;

// Shared actuator commands (0–100 %)  
volatile int throttlePct = 50;  // 50 = neutral/brake
volatile int steerPct    = 50;  // 50 = straight

// Path memory for Return mode  
static constexpr int FORWARD_REVERSE_PCT = 35;  // 40%
static constexpr int SEEK_REVERSE_PCT    = 40;

enum class EventType : uint8_t { Forward, SeekPulse };

struct PathEvent {
  uint8_t    steerPct;  // steering % at burst start
  EventType  type;      // Forward vs. SeekPulse
};

static std::vector<PathEvent> pathEvents;

// Forward-mode timing  
static unsigned long forwardDurationMs = 0;
static unsigned long forwardStartTime  = 0;
static bool forwardActive              = false;

// Seek-mode timing & recovery  
static constexpr unsigned long SEEK_THROTTLE_DURATION_MS = 1000;
static constexpr TickType_t SEEK_BURST_TICKS = pdMS_TO_TICKS(1000);
static TimerHandle_t burstTimer = nullptr;
static constexpr unsigned long SEEK_RECOVERY_MS          = 500;
static bool  seekThrottleActive       = false;
static unsigned long seekThrottleStart = 0;
static unsigned long lastBurstEnd      = 0;
static constexpr float PICKUP_DISTANCE_CM = 10.0f;
static constexpr unsigned long ULTRA_ONLY_MS = 2000UL;  // if no UART in this window, go ultrasonic-only
static unsigned long lastSerialTime = 0;
volatile float measuredDistance = 1.0f;

// Debug: last values parsed from UART0 and Ultrasonic sentor
volatile int lastSerialPct     = -1;
volatile int lastSerialAngle   = -1;
volatile float lastUsDist    = -1.0f;
volatile int   lastUsAng     = -1;
volatile int   lastUsPct     = -1;

// Return-stage extra straight reverse  
static bool extraReverseActive      = false;
static unsigned long extraReverseStart = 0;

// Reverse scaling constants  
static constexpr float REVERSE_SCALE    = 1.6f;
static constexpr int   REVERSE_HOLD_PCT = 40;

// Steering constants
static constexpr int STEER_THRESH = 10;
volatile int lastSteerPct = 50;  // initialize to straight

// Crane Pins
const int YIN1 = 5; //pins for up/down movement
const int YIN2 = 22;
const int YIN3 = 21;
const int YIN4 = 23;

const int XIN1 = 12; //pins for horizontal movement
const int XIN2 = 14;
const int XIN3 = 32;
const int XIN4 = 33;

volatile int haveLowered = 0;
const int STEP_DELAY_MS = 1; //step delay for motors
static constexpr int HSTEP_DELAY_MS   = 5;

// how many half-steps = 90° (¼ of 4096)
static constexpr int QTR_SWEEP_STEPS = 1024;
// pause between sweeps
static constexpr int SWEEP_PAUSE_MS   = 500;

// Half-step sequence (8 steps)
const int step_sequence[8][4] = {
    {1, 0, 0, 0}, // A
    {1, 1, 0, 0}, // AB
    {0, 1, 0, 0}, // B
    {0, 1, 1, 0}, // BC
    {0, 0, 1, 0}, // C
    {0, 0, 1, 1}, // CD
    {0, 0, 0, 1}, // D
    {1, 0, 0, 1}  // DA
};

inline int pctToPulse(int pct, int mn, int mx) {
  pct = constrain(pct, 0, 100);
  return mn + (mx - mn) * pct / 100;
}

void onBurstTimeout(TimerHandle_t) {
  throttlePct        = 50;          // full brake
  seekThrottleActive = false;
  lastBurstEnd       = millis();
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
        lastSerialPct   = -1;
        lastSerialAngle = -1;
        lastBurstEnd  = millis();
        lastSerialTime     = millis();
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

        break;

      case Mode::Seek: {
        static constexpr unsigned long UART_WAIT_MS    = 2000;  // how long to wait for CV data
        unsigned long now       = millis();
        unsigned long waitStart = now;
        bool gotSerial          = false;
        int  serialPct = 0, serialAng = 0;

        // 1) Wait up to UART_WAIT_MS for a new packet
        while (millis() - waitStart < UART_WAIT_MS) {
          if (Serial.available()) {
            // read exactly one “pct,ang\n” pair
            String pctStr = Serial.readStringUntil(',');
            String angStr = Serial.readStringUntil('\n');
            serialPct     = pctStr.toInt();
            serialAng     = angStr.toInt();
            gotSerial     = true;
            lastSerialTime = millis();
            break;
          }
          // give other tasks a chance
          vTaskDelay(pdMS_TO_TICKS(5));
        }

        // 2) Always do one ultrasonic sweep next
        shortestDist sd = ultrasonicSpin();
        float USdist = sd.distance;
        int   USang  = sd.angle;

        // map US angle → [0..100]%
        const float HOME = 90.0f, HFOV = 66.0f;
        int USpct = constrain(int(((USang - HOME)/HFOV + 0.5f)*100.0f), 0, 100);

        // save debug
        lastUsDist = USdist;
        lastUsAng  = USang;
        lastUsPct  = USpct;

        // 3) Blend or fallback
        int pct;
        if (gotSerial) {
          int diff = abs(serialPct - USpct);
          if (diff > STEER_THRESH) {
            // whichever is closer to lastSteerPct
            int dS = abs(serialPct - lastSteerPct);
            int dU = abs(USpct     - lastSteerPct);
            pct = (dS <= dU) ? serialPct : USpct;
          } else {
            pct = (serialPct + USpct)/2;
          }
          // save for debugging
          lastSerialPct   = serialPct;
          lastSerialAngle = serialAng;
        } else {
            // camera too slow → pure‐US fallback
            int delta = abs(USpct - lastSteerPct);
            if (delta <= STEER_THRESH) {
                // only accept small jumps
                pct = USpct;
            } else {
                // ignore big jumps, hold previous direction
                pct = lastSteerPct;
                break;
            }
        }
        // 4) pickup check at this new angle
        const int US_MIN_ANGLE = 59, US_MAX_ANGLE = 121;
        int steeringAng = US_MIN_ANGLE + (US_MAX_ANGLE - US_MIN_ANGLE)*pct/100;
        measuredDistance = confirmAngleDistance(steeringAng);

        if (measuredDistance > 0 && measuredDistance < PICKUP_DISTANCE_CM) {
          currentMode = Mode::Pickup;
          break;
        }

        // 5) steering + throttle burst
        steerPct     = pct;
        lastSteerPct = pct;
        if (!seekThrottleActive && (now - lastBurstEnd >= SEEK_RECOVERY_MS)) {
          seekThrottleActive = true;
          throttlePct        = 55;
          xTimerStart(burstTimer, 0);
          pathEvents.push_back({ lastSteerPct, EventType::SeekPulse });
        }

        break;
      }

      case Mode::Pickup:
        throttlePct = 50;
        steerPct    = 50;
        //lower crane fully
        if (haveLowered == 0){
          step_motor(2304, -1, true);
          haveLowered = 1;
        }
        //sweep crane 
        step_motor(QTR_SWEEP_STEPS,     -1, false);
        delay(SWEEP_PAUSE_MS);
        step_motor(2*QTR_SWEEP_STEPS,    1, false);
        delay(SWEEP_PAUSE_MS);
        step_motor(QTR_SWEEP_STEPS,     -1, false);
        break; 

      case Mode::Retrieve:
        throttlePct = 50;
        steerPct = 50;

        step_motor(2304, 1, true);
        currentMode = Mode::Neutral;

        break;

      case Mode::Return:
        if (!pathEvents.empty()) {
          // 1) Replay every event in reverse order
          for (int i = (int)pathEvents.size() - 1; i >= 0; --i) {
            auto &e = pathEvents[i];

            // invert steering
            steerPct = 100 - e.steerPct;

            if (e.type == EventType::Forward) {
              // reverse the single Forward run
              throttlePct = FORWARD_REVERSE_PCT;               // e.g. 35%
              vTaskDelay(pdMS_TO_TICKS(forwardDurationMs));
            } else {
              // reverse one Seek burst
              throttlePct = SEEK_REVERSE_PCT;                  // e.g. 40%
              vTaskDelay(pdMS_TO_TICKS(SEEK_THROTTLE_DURATION_MS));
              throttlePct = 50;                                // coast/brake
              vTaskDelay(pdMS_TO_TICKS(SEEK_RECOVERY_MS));
            }
          }

          // 2) Clear so next cycle we fall into the extraReverseActive block
          pathEvents.clear();
        }
        else {
          // your existing “extraReverseActive” fallback
          if (!extraReverseActive) {
            extraReverseActive = true;
            extraReverseStart  = millis();
          }
          if (millis() - extraReverseStart < 8000) {
            steerPct    = 50;
            throttlePct = REVERSE_HOLD_PCT;
          } else {
            extraReverseActive = false;
            currentMode        = Mode::Neutral;
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
    <label>Forward for <input id="forwardTime" type="number" min="0.1" step="0.1" value="5.0"> s</label><br>
    <button onclick="setMode('forward')">Go Forward</button>
    <button onclick="setMode('neutral')">Stop</button>
    <button onclick="setMode('seek')">Can Seek</button>
    <button onclick="setMode('pickup')">Pickup</button>
    <button onclick="setMode('retrieve')">Retrieve</button>
    <button onclick="setMode('return')">Return</button>
  </div>
  <div id="debug">
    <div>UART %:        <span id="uartPct">–</span>%</div>
    <div>UART angle:    <span id="uartAng">–</span>°</div>
    <div>US %:          <span id="usPct">–</span>%</div>
    <div>US angle:      <span id="usAng">–</span>°</div>
    <div>Steering %:    <span id="steerPct">–</span>%</div>
    <div>Distance to can:<span id="canDistance">–</span>cm</div>
    <div>Current mode:  <span id="curMode">–</span></div>
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
    setInterval(() => {
      fetch('/seekraw')
        .then(r => r.text())
        .then(t => {
          const [uartPct, uartAng, usPct, usAng, steerPct, canDistance, curMode] = t.split(',');
          document.getElementById('uartPct').textContent   = uartPct;
          document.getElementById('uartAng').textContent   = uartAng;
          document.getElementById('usPct').textContent     = usPct;
          document.getElementById('usAng').textContent     = usAng;
          document.getElementById('steerPct').textContent  = steerPct;
          document.getElementById('canDistance').textContent = canDistance;
          document.getElementById('curMode').textContent   = curMode;
        });
    }, 200);
  </script>
</body>
</html>
)rawliteral";


void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleSeekRaw() {
  // send SerialPct, SerialAng, USpct, USang, lastSteerPct, distanceCm, currentMode
  String out = String(lastSerialPct) + "," 
             + String(lastSerialAngle) + "," 
             + String(lastUsPct) + "," 
             + String(lastUsAng) + ","
             + String(lastSteerPct) + ","
             + String(measuredDistance, 1) + ","
             + String(modeNames[static_cast<uint8_t>(currentMode)]);
  server.send(200, "text/plain", out);
}


void handleMode() {
  String m = server.arg("m");
  if (m == "forward") {
    if (server.hasArg("t"))
      forwardDurationMs = (unsigned long)(server.arg("t").toFloat() * 1000.0f);
    forwardActive = false;
    currentMode   = Mode::Forward;
  }
  else if (m == "neutral") currentMode = Mode::Neutral;
  else if (m == "seek")    currentMode = Mode::Seek;
  else if (m == "pickup")  currentMode = Mode::Pickup;
  else if (m == "retrieve") currentMode = Mode::Retrieve;
  else if (m == "return") {
    currentMode        = Mode::Return;
    extraReverseActive = false;
  }
  server.send(200, "text/plain", "OK");
}

float confirmAngleDistance(int angle) {
  usServo.write(angle);
  delay(15);

    // Clears the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  PulseDuration = pulseIn(ECHO_PIN, HIGH, MAX_ECHO_TIMEOUT_US);
  
  // Calculate the distance
  distanceCm = PulseDuration * SOUND_SPEED/2;
  return distanceCm;

}

void measureDistance(int angle, shortestDist &shortestDistance) {

  // Clears the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  PulseDuration = pulseIn(ECHO_PIN, HIGH, MAX_ECHO_TIMEOUT_US);
  
  // Calculate the distance
  distanceCm = PulseDuration * SOUND_SPEED/2;

  if (distanceCm < shortestDistance.distance) {
    shortestDistance.distance = distanceCm;
    shortestDistance.angle = angle;
  }
}

shortestDist ultrasonicSpin() {
  
  shortestDist shortestDistance = {9999.0, -1}; 

  for (int pos = 59; pos <= 121; pos++) {
    usServo.write(pos);
    delay(15);
    measureDistance(pos, shortestDistance);
  }

  for (int pos = 121; pos >= 59; pos--) {
    usServo.write(pos);
    delay(15);
    measureDistance(pos, shortestDistance);
  }

    return shortestDistance;
}

//Set motor pins to step sequence
void set_step(const int step[4], bool motor) {
  if (motor) { // TRUE for UP/DOWN
    digitalWrite(YIN1, step[0]);
    digitalWrite(YIN2, step[1]);
    digitalWrite(YIN3, step[2]);
    digitalWrite(YIN4, step[3]);
  } else { // FALSE for LEFT/RIGHT
    digitalWrite(XIN1, step[0]);
    digitalWrite(XIN2, step[1]);
    digitalWrite(XIN3, step[2]);
    digitalWrite(XIN4, step[3]);
  }
}
void step_motor(int steps, int direction, bool motor) {
  // choose fast vs. slow delay
  int dly = motor ? STEP_DELAY_MS : HSTEP_DELAY_MS;
  for (int i = 0; i < steps; i++) {
    int idx = (direction > 0) ? (i % 8) : (7 - (i % 8));
    set_step(step_sequence[idx], motor);
    delay(dly);
  }
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

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  usServo.attach(STEPPER_PIN /* = 25 */, 500, 2500);
  usServo.write(90);

    // UP/DOWN pins
  pinMode(YIN1, OUTPUT);
  pinMode(YIN2, OUTPUT);
  pinMode(YIN3, OUTPUT);
  pinMode(YIN4, OUTPUT);

  // LEFT/RIGHT pins
  pinMode(XIN1, OUTPUT);
  pinMode(XIN2, OUTPUT);
  pinMode(XIN3, OUTPUT);
  pinMode(XIN4, OUTPUT);

  burstTimer = xTimerCreate(
    "BurstTimer",
    SEEK_BURST_TICKS,
    pdFALSE,      // one‐shot
    nullptr,
    onBurstTimeout
  );

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
  server.handleClient();
}
