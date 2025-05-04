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


//define sound speed in cm/uS
#define SOUND_SPEED 0.034

// For the Ultrasonic sensor (US)
Servo usServo;

long SpinDuration;
float distanceCm;

struct shortestDist{
  float distance;
  int angle;
};

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
static constexpr float PICKUP_DISTANCE_CM = 10.0f;


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
static constexpr float REVERSE_SCALE    = 1.75f;
static constexpr int   REVERSE_HOLD_PCT = 25;

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

const int STEP_DELAY_MS = 1; //step delay for motors

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
          int SerialPct = pctStr.toInt();
          int SerialAng = angStr.toInt();
          
          shortestDist sd = ultrasonicSpin();
          float USdist = sd.distance;
          int USang  = sd.angle;

          if (USdist > 0 && USdist < PICKUP_DISTANCE_CM) {
            currentMode = Mode::Pickup;
            break;  // exit Seek immediately
          }
    
          const float HOME = 90.0f;
          const float HFOV = 62.0f;
          // same conversion as camera code but using sd.angle
          int USpct = constrain(
            int(((USang - HOME)/HFOV + 0.5f) * 100.0f),
            0, 100
          );

          int pct;
          int diff = abs(SerialPct - USpct);

          if (diff > STEER_THRESH) {
            int dSerial = abs(SerialPct - lastSteerPct);
            int dUS     = abs(USpct     - lastSteerPct);
            pct = (dSerial <= dUS) ? SerialPct : USpct;
          }
          else if (diff < STEER_THRESH) {
            pct = (SerialPct + USpct) / 2;
          }
          else {
            pct = lastSteerPct;
          }

          lastSerialPct   = SerialPct;
          lastSerialAngle = SerialAng;
          lastUsDist = USdist;
          lastUsAng = USang;
          lastUsPct = USpct;

          if (pct >= 0 && pct <= 100) {
            steerPct = pct;
            lastSteerPct = pct;
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
        //lower crane fully
        step_motor(512, -1, true);

        //sweep crane 
        for (int i=0;i<2;++i){ 
          step_motor(512, -1, false);
        }
        for (int i=0;i<4;++i){ 
          step_motor(512, 1, false);
        }

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
    <div>UART %:      <span id="uartPct">–</span>%</div>
    <div>UART angle:  <span id="uartAng">–</span>°</div>
    <div>US %:        <span id="usPct">–</span>%</div>
    <div>US angle:    <span id="usAng">–</span>°</div>
    <div>US dist:     <span id="usDist">–</span>cm</div>
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
          const [uartPct, uartAng, usPct, usAng, usDist] = t.split(',');
          document.getElementById('uartPct').textContent = uartPct;
          document.getElementById('uartAng').textContent = uartAng;
          document.getElementById('usPct').textContent   = usPct;
          document.getElementById('usAng').textContent   = usAng;
          document.getElementById('usDist').textContent  = usDist;
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
  // send SerialPct, SerialAng, USpct, USang, USdist
  String out = String(lastSerialPct) + "," 
             + String(lastSerialAngle) + "," 
             + String(lastUsPct) + "," 
             + String(lastUsAng) + "," 
             + String(lastUsDist, 1);    // one decimal place
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
  SpinDuration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance
  distanceCm = SpinDuration * SOUND_SPEED/2;
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
  SpinDuration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance
  distanceCm = SpinDuration * SOUND_SPEED/2;

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
  for (int i = 0; i < steps; i++) {
    int idx = (direction > 0) ? (i % 8) : (7 - (i % 8));
    set_step(step_sequence[idx], motor);
    delay(STEP_DELAY_MS);
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

  //sets for can height
  for (int i=0; i<5; ++i){ //change for height
    step_motor(512, -1, true);
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
