#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <math.h>

// =========================================================================
// 1. WI-FI CONFIGURATION
// =========================================================================
const char* ssid     = "Infinix HOT 50 Pro+";
const char* password = "doriqueleee";

// =========================================================================
// 2. XIAO ESP32S3 SENSE PINOUT
// =========================================================================
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

const int W = 320, H = 240;
const int ROI = 100;
const int NFRAMES = 10;

// HSV Global variables
float S_MIN = 0.30;
float V_MIN = 0.20;
float hCenter[3] = {0, 120, 240};  // Red, Green, Blue
float hTol[3]    = {20, 30, 30};
bool  done[3]    = {false, false, false};
float whiteS99 = 0, whiteV = 0;
bool  whiteDone = false;
const char* NAMES[3] = {"RED", "GREEN", "BLUE"};

volatile bool isCalibrating = false;
httpd_handle_t camera_httpd = NULL;

// =========================================================================
// 3. IMAGE PROCESSING & HSV FUNCTIONS
// =========================================================================
void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, float &h, float &s, float &v) {
  float rf = r / 255.0, gf = g / 255.0, bf = b / 255.0;
  float mx = max(rf, max(gf, bf)), mn = min(rf, min(gf, bf));
  float d = mx - mn;
  v = mx;
  s = (mx == 0) ? 0 : d / mx;
  if (d == 0) h = 0;
  else if (mx == rf) h = 60 * fmodf((gf - bf) / d, 6);
  else if (mx == gf) h = 60 * ((bf - rf) / d + 2);
  else               h = 60 * ((rf - gf) / d + 4);
  if (h < 0) h += 360;
}

float angDiff(float a, float b) {
  float d = fabsf(a - b);
  return d > 180 ? 360 - d : d;
}

camera_fb_t* grab() {
  camera_fb_t *fb = NULL;
  for (int retry = 0; retry < 10; retry++) {
    fb = esp_camera_fb_get();
    if (fb) return fb;
    delay(30);
  }
  return NULL;
}

inline void pixelHSV(camera_fb_t *fb, int x, int y, float &h, float &s, float &v) {
  int i = (y * W + x) * 2;
  uint16_t p = (fb->buf[i] << 8) | fb->buf[i + 1];
  uint8_t r = ((p >> 11) & 0x1F) << 3;
  uint8_t g = ((p >> 5)  & 0x3F) << 2;
  uint8_t b = (p & 0x1F) << 3;
  rgb2hsv(r, g, b, h, s, v);
}

String calibWhite() {
  isCalibrating = true;
  delay(200);

  int hist[101] = {0};
  long n = 0; double sumV = 0;
  int x0 = (W - ROI) / 2, y0 = (H - ROI) / 2;

  for (int f = 0; f < NFRAMES; f++) {
    camera_fb_t *fb = grab();
    if (!fb) continue;
    for (int y = y0; y < y0 + ROI; y++) {
      for (int x = x0; x < x0 + ROI; x++) {
        float h, s, v; 
        pixelHSV(fb, x, y, h, s, v);
        hist[(int)(s * 100)]++; 
        sumV += v; 
        n++;
      }
    }
    esp_camera_fb_return(fb);
  }

  String msg = "";
  if (n > 0) {
    long acc = 0; int p99 = 0;
    for (int i = 0; i <= 100; i++) { 
      acc += hist[i]; 
      if (acc >= 0.99 * n) { p99 = i; break; } 
    }
    whiteS99 = p99 / 100.0;
    whiteV = sumV / n;
    S_MIN = min(0.60f, whiteS99 + 0.15f);
    whiteDone = true;

    char logBuf[200];
    snprintf(logBuf, sizeof(logBuf), "WHITE: S99=%.2f | Mean V=%.2f -> S_MIN=%.2f\n", whiteS99, whiteV, S_MIN);
    msg += logBuf;

    if (whiteV < 0.5) msg += "⚠️ WARNING: Dark background, add more light!\n";
    if (whiteS99 > 0.25) msg += "⚠️ WARNING: Highly colored background!\n";
    msg += "✅ WHITE calibration completed successfully.";
  } else {
    msg = "❌ FAILURE: White capture failed.";
  }

  isCalibrating = false;
  return msg;
}

String calibColor(int k) {
  isCalibrating = true;
  delay(200);

  double sc = 0, ss = 0, sumS = 0, sumV = 0; long n = 0;
  int x0 = (W - ROI) / 2, y0 = (H - ROI) / 2;

  for (int f = 0; f < NFRAMES; f++) {
    camera_fb_t *fb = grab();
    if (!fb) continue;
    for (int y = y0; y < y0 + ROI; y++) {
      for (int x = x0; x < x0 + ROI; x++) {
        float h, s, v; 
        pixelHSV(fb, x, y, h, s, v);
        if (s > S_MIN && v > V_MIN) {
          float r = h * M_PI / 180.0;
          sc += cosf(r); ss += sinf(r); sumS += s; sumV += v; n++;
        }
      }
    }
    esp_camera_fb_return(fb);
  }

  String msg = "";
  if (n >= 0.03 * ROI * ROI * NFRAMES) {
    float mean = atan2(ss, sc) * 180.0 / M_PI;
    if (mean < 0) mean += 360;
    double R = sqrt(sc * sc + ss * ss) / n;
    double sd = sqrt(-2.0 * log(max(R, 1e-6))) * 180.0 / M_PI;
    hCenter[k] = mean;
    hTol[k] = constrain(2.5 * sd, 12.0, 45.0);
    done[k] = true;

    char logBuf[200];
    snprintf(logBuf, sizeof(logBuf), "%s: H=%.0f° | Tol=+/-%.0f° (Mean S=%.2f, Mean V=%.2f, %ld px)\n✅ %s calibration completed!",
             NAMES[k], mean, hTol[k], sumS / n, sumV / n, n, NAMES[k]);
    msg = logBuf;
  } else {
    msg = "❌ FAILURE: Not enough saturated pixels. Cube missing or poorly centered?";
  }

  isCalibrating = false;
  return msg;
}

String printBlockConfig() {
  String out = "\n// --- C++ CONFIGURATION BLOCK ---\n";
  out += "const float S_MIN = " + String(S_MIN, 2) + ";\n";
  out += "const float V_MIN = " + String(V_MIN, 2) + ";\n";
  out += "const float hCenter[3] = {" + String(hCenter[0], 0) + ", " + String(hCenter[1], 0) + ", " + String(hCenter[2], 0) + "};\n";
  out += "const float hTol[3]    = {" + String(hTol[0], 0) + ", " + String(hTol[1], 0) + ", " + String(hTol[2], 0) + "};\n";
  out += "// -------------------------------\n";
  return out;
}

const char* classify(float h) {
  for (int k = 0; k < 3; k++)
    if (angDiff(h, hCenter[k]) <= hTol[k]) return NAMES[k];
  return "NONE";
}

String detectColorJSON() {
  if (isCalibrating) return "{\"color\":\"CALIBRATION\",\"h\":0,\"s\":0,\"v\":0}";

  camera_fb_t *fb = grab();
  if (!fb) return "{\"color\":\"NONE\",\"h\":0,\"s\":0,\"v\":0}";

  int x0 = (W - ROI) / 2, y0 = (H - ROI) / 2;
  double sc = 0, ss = 0, sumS = 0, sumV = 0; long n = 0;

  for (int y = y0; y < y0 + ROI; y++) {
    for (int x = x0; x < x0 + ROI; x++) {
      float h, s, v; 
      pixelHSV(fb, x, y, h, s, v);
      if (s > S_MIN && v > V_MIN) { 
        float r = h * M_PI / 180.0; 
        sc += cosf(r); ss += sinf(r); sumS += s; sumV += v; n++; 
      }
    }
  }
  esp_camera_fb_return(fb);

  if (n < 0.08 * ROI * ROI) {
    return "{\"color\":\"NONE\",\"h\":0,\"s\":0,\"v\":0}";
  }

  float hm = atan2(ss, sc) * 180.0 / M_PI; 
  if (hm < 0) hm += 360;
  float sm = sumS / n;
  float vm = sumV / n;
  const char* col = classify(hm);

  String json = "{\"color\":\"" + String(col) + "\",";
  json += "\"h\":" + String(hm, 1) + ",";
  json += "\"s\":" + String(sm, 2) + ",";
  json += "\"v\":" + String(vm, 2) + "}";
  return json;
}

// =========================================================================
// 4. MJPEG VIDEO STREAM (LIVE STREAM)
// =========================================================================
esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    if (isCalibrating) {
      delay(100);
      continue;
    }

    fb = esp_camera_fb_get();
    if (!fb) {
      delay(20);
      continue;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!jpeg_converted) {
        delay(20);
        continue;
      }
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (res != ESP_OK) break;
  }
  return res;
}

// =========================================================================
// 5. INTERACTIVE WEB PAGE (HTML/CSS/JS) - COMPACT & OPTIMIZED
// =========================================================================
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Vision System - XIAO ESP32S3</title>
  <style>
    :root {
      --bg: #0b0e14;
      --card: #151922;
      --accent: #00f2fe;
      --text: #e6edf3;
      --border: rgba(255, 255, 255, 0.08);
    }
    body { font-family: 'Segoe UI', system-ui, sans-serif; background: var(--bg); color: var(--text); margin: 0; padding: 10px; text-align: center; }
    h1 { color: var(--accent); margin: 5px 0 15px 0; font-weight: 300; letter-spacing: 1px; font-size: 20px; text-transform: uppercase; }
    
    /* Single line compact grid fitting on screen */
    .grid { display: flex; flex-wrap: nowrap; justify-content: center; gap: 12px; max-width: 1250px; margin: 0 auto; }
    
    .card { background: var(--card); border-radius: 10px; padding: 12px; width: 310px; box-shadow: 0 6px 20px rgba(0,0,0,0.4); border: 1px solid var(--border); display: flex; flex-direction: column; justify-content: space-between; }
    .card-terminal { width: 440px; }
    
    .card h3 { margin: 0 0 8px 0; font-size: 13px; letter-spacing: 0.5px; color: #8b949e; border-bottom: 1px solid var(--border); padding-bottom: 6px; }
    
    .btn { display: block; width: 100%; padding: 9px; margin: 5px 0; border: none; border-radius: 6px; font-weight: 700; cursor: pointer; transition: all 0.2s; font-size: 11px; text-transform: uppercase; letter-spacing: 0.5px; }
    .btn-w { background: #f0f6fc; color: #0d1117; }
    .btn-r { background: #ff4757; color: white; }
    .btn-g { background: #2ed573; color: white; }
    .btn-b { background: #1e90ff; color: white; }
    .btn-export { background: #a55eea; color: white; }
    .btn-test { background: var(--accent); color: #000; font-size: 13px; margin-top: 6px; }
    .btn:hover { transform: translateY(-1px); filter: brightness(1.15); }
    
    #video-container { position: relative; width: 100%; height: 180px; border-radius: 6px; overflow: hidden; border: 1px solid var(--border); background: #000; display: flex; align-items: center; justify-content: center; }
    #video-stream { width: 100%; height: 100%; object-fit: cover; }
    #video-placeholder { display: none; color: #8b949e; font-size: 11px; font-weight: bold; }
    
    #terminal { background: #000; color: #00ff66; font-family: 'Courier New', monospace; font-size: 11px; padding: 10px; border-radius: 6px; text-align: left; height: 260px; overflow-y: auto; border: 1px solid rgba(0,255,102,0.2); white-space: pre-wrap; box-shadow: inset 0 0 10px rgba(0,0,0,0.8); }
  </style>
</head>
<body>
  <h1>ESP32 VISION & CALIBRATION</h1>
  <div class="grid">
    <!-- Block 1: Video -->
    <div class="card">
      <h3>VIDEO STREAM (QVGA)</h3>
      <div id="video-container">
        <img id="video-stream" src="/stream" alt="Video Stream">
        <div id="video-placeholder">🎥 VIDEO PAUSED (TEST IN PROGRESS)</div>
      </div>
      <div style="font-size: 10px; color: #8b949e; margin-top: 6px;">XIAO ESP32S3 Sense</div>
    </div>

    <!-- Block 2: Controls -->
    <div class="card">
      <h3>CALIBRATION CONTROLS</h3>
      <button class="btn btn-w" onclick="calib('w')">1. Calibrate White (Mat)</button>
      <button class="btn btn-r" onclick="calib('r')">2. Calibrate Red Cube</button>
      <button class="btn btn-g" onclick="calib('g')">3. Calibrate Green Cube</button>
      <button class="btn btn-b" onclick="calib('b')">4. Calibrate Blue Cube</button>
      <button class="btn btn-export" onclick="exportConfig()">5. Export C++ Config</button>
      <button id="test-btn" class="btn btn-test" onclick="toggleTest()">LIVE TEST : OFF</button>
    </div>

    <!-- Block 3: Terminal on the right -->
    <div class="card card-terminal">
      <h3>TERMINAL & DETECTION RESULTS</h3>
      <div id="terminal">> System initialized.\n> Ready to calibrate or test...</div>
    </div>
  </div>

  <script>
    let testing = false;
    let timer = null;

    function log(msg) {
      const term = document.getElementById('terminal');
      term.innerText += "\n> " + msg;
      term.scrollTop = term.scrollHeight;
    }

    function setVideoState(active) {
      const img = document.getElementById('video-stream');
      const placeholder = document.getElementById('video-placeholder');
      if (active) {
        img.style.display = "block";
        placeholder.style.display = "none";
        img.src = "/stream?" + new Date().getTime();
      } else {
        img.style.display = "none";
        placeholder.style.display = "block";
        img.src = "";
      }
    }

    function calib(cmd) {
      log("Starting calibration... (Video paused)");
      setVideoState(false);

      setTimeout(() => {
        fetch('/calib?cmd=' + cmd)
          .then(res => res.text())
          .then(msg => { 
            log(msg);
            if (!testing) setVideoState(true);
          })
          .catch(() => { 
            log("❌ HTTP communication error.");
            if (!testing) setVideoState(true);
          });
      }, 300);
    }

    function exportConfig() {
      log("Exporting C++ configuration...");
      const wasTesting = testing;
      if (wasTesting) toggleTest();
      setVideoState(false);

      setTimeout(() => {
        fetch('/export')
          .then(res => res.text())
          .then(data => { 
            log(data);
            if (!wasTesting) setVideoState(true);
          })
          .catch(() => {
            log("❌ Error during export.");
            if (!wasTesting) setVideoState(true);
          });
      }, 200);
    }

    function toggleTest() {
      testing = !testing;
      const btn = document.getElementById('test-btn');
      if (testing) {
        btn.innerText = "LIVE TEST : ON";
        btn.style.background = "#2ed573";
        btn.style.color = "#fff";
        setVideoState(false);
        log("Live Test mode STARTED.");
        timer = setInterval(runDetect, 400);
      } else {
        btn.innerText = "LIVE TEST : OFF";
        btn.style.background = "#00f2fe";
        btn.style.color = "#000";
        clearInterval(timer);
        log("Live Test mode STOPPED.");
        setVideoState(true);
      }
    }

    function runDetect() {
      fetch('/detect')
        .then(res => res.json())
        .then(data => {
          if (data.color === "CALIBRATION") return;
          const txt = data.color === "NONE" ? "WHITE MAT" : data.color + " CUBE";
          log(`[LIVE] ${txt} (H=${data.h}°, S=${data.s}, V=${data.v})`);
        })
        .catch(() => {});
    }
  </script>
</body>
</html>
)rawliteral";

// =========================================================================
// 6. HTTP REQUEST HANDLERS
// =========================================================================
esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

esp_err_t calib_handler(httpd_req_t *req) {
  char buf[32];
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char param[10];
    if (httpd_query_key_value(buf, "cmd", param, sizeof(param)) == ESP_OK) {
      char c = param[0];
      String msg = "";
      if (c == 'w') msg = calibWhite();
      else if (c == 'r') msg = calibColor(0);
      else if (c == 'g') msg = calibColor(1);
      else if (c == 'b') msg = calibColor(2);

      httpd_resp_sendstr(req, msg.c_str());
      return ESP_OK;
    }
  }
  httpd_resp_sendstr(req, "Unknown command");
  return ESP_OK;
}

esp_err_t export_handler(httpd_req_t *req) {
  String out = printBlockConfig();
  httpd_resp_sendstr(req, out.c_str());
  return ESP_OK;
}

esp_err_t detect_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  String json = detectColorJSON();
  httpd_resp_sendstr(req, json.c_str());
  return ESP_OK;
}

void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri  = { .uri = "/",       .method = HTTP_GET, .handler = index_handler,  .user_ctx = NULL };
  httpd_uri_t calib_uri  = { .uri = "/calib",   .method = HTTP_GET, .handler = calib_handler,  .user_ctx = NULL };
  httpd_uri_t export_uri = { .uri = "/export",  .method = HTTP_GET, .handler = export_handler, .user_ctx = NULL };
  httpd_uri_t detect_uri = { .uri = "/detect",  .method = HTTP_GET, .handler = detect_handler, .user_ctx = NULL };
  httpd_uri_t stream_uri = { .uri = "/stream",  .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &calib_uri);
    httpd_register_uri_handler(camera_httpd, &export_uri);
    httpd_register_uri_handler(camera_httpd, &detect_uri);
    httpd_register_uri_handler(camera_httpd, &stream_uri);
  }
}

// =========================================================================
// 7. SETUP & LOOP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM; c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QVGA;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("Camera initialization error");
    while (1);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n------------------------------------------------");
  Serial.println("Wi-Fi connected!");
  Serial.print("Open your browser and type: http://");
  Serial.println(WiFi.localIP());
  Serial.println("------------------------------------------------");

  startServer();
}

void loop() {
  delay(10);
}
