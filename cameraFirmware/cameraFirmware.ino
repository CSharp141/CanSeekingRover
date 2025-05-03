/* 
 * ESP32-CAM + Edge Impulse
 * • Single‐servo pan pointing
 * • 0–100% steering → Serial
 * • Flash LED on detection
 * • Web UI:
 *    /         → last JPEG snapshot with center‐point overlay & stats
 *    /status   → JSON {angle,steering,detected,cx,cy}
 *    /snapshot → the JPEG frame used for last classification
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <csharp141-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// —— Wi-Fi credentials —————————————————————————————————————
#define WIFI_SSID     "ESP32-Rover"
#define WIFI_PASSWORD "roverpw123"

// —— Servo config ————————————————————————————————————————
#define SERVO_PIN        13
#define SERVO_FREQ       50
#define SERVO_MIN_PULSE  500
#define SERVO_MAX_PULSE  2400
#define SERVO_HOME_ANGLE 90

// —— Detection threshold & flash LED ——————————————————————————
#define MIN_CONFIDENCE   0.5f
#define FLASH_GPIO_NUM   4

// —— Camera (AI-Thinker ESP32-CAM) pins —————————————————————————
#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM    32
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM     0
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      21
  #define Y4_GPIO_NUM      19
  #define Y3_GPIO_NUM      18
  #define Y2_GPIO_NUM       5
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22
#else
  #error "Camera model not selected"
#endif

// —— Raw dims & buffers —————————————————————————————————————
#define RAW_W    320
#define RAW_H    240
#define RAW_BPP    3

static camera_config_t camera_config = {
  .pin_pwdn       = PWDN_GPIO_NUM,
  .pin_reset      = RESET_GPIO_NUM,
  .pin_xclk       = XCLK_GPIO_NUM,
  .pin_sscb_sda   = SIOD_GPIO_NUM,
  .pin_sscb_scl   = SIOC_GPIO_NUM,
  .pin_d7         = Y9_GPIO_NUM,
  .pin_d6         = Y8_GPIO_NUM,
  .pin_d5         = Y7_GPIO_NUM,
  .pin_d4         = Y6_GPIO_NUM,
  .pin_d3         = Y5_GPIO_NUM,
  .pin_d2         = Y4_GPIO_NUM,
  .pin_d1         = Y3_GPIO_NUM,
  .pin_d0         = Y2_GPIO_NUM,
  .pin_vsync      = VSYNC_GPIO_NUM,
  .pin_href       = HREF_GPIO_NUM,
  .pin_pclk       = PCLK_GPIO_NUM,
  .xclk_freq_hz   = 20000000,
  .ledc_timer     = LEDC_TIMER_0,
  .ledc_channel   = LEDC_CHANNEL_0,
  .pixel_format   = PIXFORMAT_JPEG,
  .frame_size     = FRAMESIZE_QVGA,
  .jpeg_quality   = 12,
  .fb_count       = 1,
  .fb_location    = CAMERA_FB_IN_PSRAM,
  .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,
};

WebServer server(80);
Servo panServo;

// PSRAM buffers
uint8_t *snapshot_buf      = nullptr; // RGB888 input for Edge Impulse
uint8_t *jpeg_snapshot_buf = nullptr; // raw JPEG for serving
size_t  jpeg_snapshot_len  = 0;

// Live state
volatile int currentAngle    = SERVO_HOME_ANGLE;
volatile int currentSteering = 0;
volatile bool last_detected  = false;
volatile float last_cx       = 0.0f;
volatile float last_cy       = 0.0f;

//—— Initialize camera —————————————————————————————————————
bool initCamera() {
  if (esp_camera_init(&camera_config) != ESP_OK) return false;
  sensor_t *s = esp_camera_sensor_get();
  if (s && s->id.PID == OV3660_PID) {
    s->set_vflip(s,1);
    s->set_brightness(s,1);
    s->set_saturation(s,0);
  }
  return true;
}

//—— EI data provider —————————————————————————————————————
static int getData(size_t offset, size_t length, float *out) {
  size_t pix = offset * 3;
  for (size_t i = 0; i < length; i++) {
    out[i] = (snapshot_buf[pix+2]<<16)
           | (snapshot_buf[pix+1]<<8)
           |  snapshot_buf[pix];
    pix += 3;
  }
  return 0;
}

//—— Capture JPEG + convert to RGB ———————————————————————————
bool captureAndConvert() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  // Store exact JPEG
  jpeg_snapshot_len = fb->len;
  memcpy(jpeg_snapshot_buf, fb->buf, jpeg_snapshot_len);
  // Convert to RGB888
  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(fb);
  if (!ok) return false;
  // Resize input if needed
  if (EI_CLASSIFIER_INPUT_WIDTH != RAW_W ||
      EI_CLASSIFIER_INPUT_HEIGHT != RAW_H) {
    ei::image::processing::crop_and_interpolate_rgb888(
      snapshot_buf, RAW_W, RAW_H,
      snapshot_buf,
      EI_CLASSIFIER_INPUT_WIDTH,
      EI_CLASSIFIER_INPUT_HEIGHT
    );
  }
  return true;
}

//—— HTTP: root page (always show snapshot, single red dot when detected) —————————
void handleRoot() {
  const char* html = R"rawliteral(
<html><head><meta charset="utf-8"><title>ESP32 Steering</title>
<style>
  body {
    margin: 0;
    background: #111;
    color: #eee;
    font-family: sans-serif;
    display: flex;
    flex-direction: column;
    align-items: center;
  }
  #container {
    position: relative;
    width: 320px;
    height: 240px;
    margin-top: 20px;
    border: 2px solid #444;
  }
  #snap {
    width: 100%;
    height: 100%;
    display: block;
  }
  #overlay {
    position: absolute;
    top: 0; left: 0;
    width: 100%;
    height: 100%;
    pointer-events: none;
  }
  #stats {
    margin-top: 10px;
    font-size: 16px;
  }
</style>
</head><body>
  <h1>ESP32 Steering Status</h1>
  <div id="container">
    <img id="snap" src="/snapshot" />
    <canvas id="overlay" width="320" height="240"></canvas>
  </div>
  <div id="stats">
    Angle: <span id="angle">--</span>&deg;
    &nbsp; Steering: <span id="steer">--</span>%
  </div>
<script>
  const img    = document.getElementById('snap'),
        canvas = document.getElementById('overlay'),
        ctx    = canvas.getContext('2d');
  async function upd() {
    // always reload the latest snapshot (cache‐buster)
    img.src = '/snapshot?ts=' + Date.now();
    // fetch status
    let res = await fetch('/status'),
        j   = await res.json();
    document.getElementById('angle').textContent = j.angle;
    document.getElementById('steer').textContent = j.steering;
    // clear previous dot
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // draw one dot if detection
    if (j.detected) {
      ctx.fillStyle = 'red';
      ctx.beginPath();
      ctx.arc(j.cx, j.cy, 5, 0, 2 * Math.PI);
      ctx.fill();
    }
  }
  // poll every 500 ms
  setInterval(upd, 500);
  upd();
</script>
</body></html>
)rawliteral";

  server.send(200, "text/html; charset=utf-8", html);
}


//—— HTTP: serve last JPEG snapshot —————————————————————————
void handleSnapshot() {
  if (!jpeg_snapshot_len) {
    server.send(404);
    return;
  }
  server.sendHeader("Content-Type","image/jpeg");
  server.sendHeader("Content-Length",String(jpeg_snapshot_len));
  server.send(200);
  server.client().write(jpeg_snapshot_buf, jpeg_snapshot_len);
}

//—— HTTP: JSON status (single centroid) —————————————————————
void handleStatus() {
  String js = "{\"angle\":" + String(currentAngle)
            + ",\"steering\":" + String(currentSteering)
            + ",\"detected\":" + (last_detected?"true":"false");
  if (last_detected) {
    js += ",\"cx\":" + String(last_cx,1)
       +  ",\"cy\":" + String(last_cy,1);
  }
  js += "}";
  server.send(200, "application/json", js);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.'); delay(500);
  }
  Serial.println("\nIP = " + WiFi.localIP().toString());

  // Flash LED
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW);

  // Camera
  if (!initCamera()) {
    Serial.println("Camera init failed");
    while (1) delay(1000);
  }

  // PSRAM buffers
  size_t rgb_sz  = RAW_W * RAW_H * RAW_BPP;
  size_t jpeg_sz = RAW_W * RAW_H * RAW_BPP;
  snapshot_buf      = (uint8_t*) ps_malloc(rgb_sz);
  jpeg_snapshot_buf = (uint8_t*) ps_malloc(jpeg_sz);
  if (!snapshot_buf || !jpeg_snapshot_buf) {
    Serial.println("PSRAM alloc failed"); while (1) delay(1000);
  }

  // Servo
  panServo.setPeriodHertz(SERVO_FREQ);
  panServo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  panServo.write(SERVO_HOME_ANGLE);

  // Web routes
  server.on("/",         HTTP_GET, handleRoot);
  server.on("/snapshot", HTTP_GET, handleSnapshot);
  server.on("/status",   HTTP_GET, handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();

  if (ei_sleep(5) != EI_IMPULSE_OK) return;
  if (!captureAndConvert()) return;

  // Inference
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data     = &getData;
  ei_impulse_result_t res;
  if (run_classifier(&signal, &res, false) != EI_IMPULSE_OK) return;

  // find best detection
  float best_conf = 0;
  ei_impulse_result_bounding_box_t best_bb = {0};
  last_detected = false;
  for (uint32_t i = 0; i < res.bounding_boxes_count; i++) {
    auto &b = res.bounding_boxes[i];
    if (b.value >= MIN_CONFIDENCE && b.value > best_conf) {
      best_conf = b.value;
      best_bb   = b;
    }
  }

  if (best_conf >= MIN_CONFIDENCE) {
    // centroid
    last_cx = best_bb.x + best_bb.width/2.0f;
    last_cy = best_bb.y + best_bb.height/2.0f;
    last_detected = true;

    // normalized steering
    int pct = constrain(int((last_cx / float(EI_CLASSIFIER_INPUT_WIDTH)) * 100), 0, 100);
    currentSteering = pct;
    Serial.println(pct);

    // servo angle
    const float HFOV = 62.0f;
    float err = last_cx - (EI_CLASSIFIER_INPUT_WIDTH/2.0f);
    float deg = err * (HFOV / EI_CLASSIFIER_INPUT_WIDTH);
    int ang = constrain(int(SERVO_HOME_ANGLE + deg), 0, 180);
    currentAngle = ang;
    panServo.write(ang);

    // flash LED
    digitalWrite(FLASH_GPIO_NUM, HIGH);
    delay(50);
    digitalWrite(FLASH_GPIO_NUM, LOW);
  }
}
