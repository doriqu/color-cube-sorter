#include "esp_camera.h"
#include <math.h>

// =========================================================================
// 1. HSV VALUES: YELLOW, GREEN, BLUE
// =========================================================================
const float S_MIN = 0.16;
const float V_MIN = 0.20;
const float hCenter[3] = {64, 146, 194}; // Hue modified for yellow (~30°) {63, 137, 187};  {12, 13, 31};
const float hTol[3]    = {12, 12, 19};
// =========================================================================

const char NAMES[3][10] = {"YELLOW", "GREEN", "BLUE"};
const char CODES[3]     = {'J', 'V', 'B'};   // 'J' for Yellow (formerly 'R')

// =========================================================================
// 2. PINS & ULTRASONIC SENSOR
// =========================================================================
const uint8_t PIN_TRIG = 1;               // D0 = GPIO1
const uint8_t PIN_ECHO = 4;               // D1 = GPIO2

const int PIN_UART_TX = 43;               // D6 -> GPIO16 of master ESP32
const int PIN_UART_RX = 44;               // D7 (not used for now)

// EMERGENCY STOP push button (to GND, INPUT_PULLUP) -> sends 'A' to master ESP32
const uint8_t PIN_BTN_ARRET = 8;          // D9 = GPIO5 (free: neither camera, nor ultrasonic, nor UART)
const unsigned long DEBOUNCE_STOP_MS = 600;
const char STOP_CODE = 'A';

// XIAO ESP32S3 Sense Camera
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

const int W = 320, H = 240;
const int ROI = 100;
const int FRAMES_TO_DISCARD = 2;          // old frames from camera buffer
const int NFRAMES = 3;                    // frames analyzed per capture
const float MIN_PIXEL_RATIO = 0.08;       // minimum proportion of colored pixels

volatile bool stopRequested = false;
volatile unsigned long lastStopEdge = 0;

bool previousObjectPresent = false;
unsigned long lastTrigger = 0;
const unsigned long CAPTURE_DELAY_MS = 2000; // 2 seconds pause between analyses

// =========================================================================
// 2bis. STOP BUTTON: interrupt (no presses lost during camera analysis)
// =========================================================================
void IRAM_ATTR isrStopButton() {
  unsigned long now = millis();
  if (now - lastStopEdge < DEBOUNCE_STOP_MS) return;
  lastStopEdge = now;
  if (digitalRead(PIN_BTN_ARRET) == LOW) stopRequested = true;
}

// =========================================================================
// 3. IMAGE PROCESSING & ULTRASONIC SENSOR
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
  return d > 180 ? 360.0f - d : d;
}

float measureDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 30000); // Timeout at 30ms (~500 cm max)
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

camera_fb_t* grab() {
  for (int retry = 0; retry < 10; retry++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) return fb;
    delay(30);
  }
  return NULL;
}

// Analyzes the central zone; returns the UART code ('J', 'V', 'B' or 'X')
char analyzeCube() {
  for (int i = 0; i < FRAMES_TO_DISCARD; i++) {
    camera_fb_t *fb = grab();
    if (fb) esp_camera_fb_return(fb);
  }

  int x0 = (W - ROI) / 2, y0 = (H - ROI) / 2;
  double sc = 0, ss = 0, sumS = 0, sumV = 0;
  long n = 0;
  int framesOk = 0;

  for (int f = 0; f < NFRAMES; f++) {
    camera_fb_t *fb = grab();
    if (!fb) continue;
    framesOk++;
    for (int y = y0; y < y0 + ROI; y++) {
      for (int x = x0; x < x0 + ROI; x++) {
        int i = (y * W + x) * 2;
        uint16_t p = (fb->buf[i] << 8) | fb->buf[i + 1];
        uint8_t r = ((p >> 11) & 0x1F) << 3;
        uint8_t g = ((p >> 5)  & 0x3F) << 2;
        uint8_t b = (p & 0x1F) << 3;
        float h, s, v;
        rgb2hsv(r, g, b, h, s, v);
        if (s > S_MIN && v > V_MIN) {
          float rad = h * M_PI / 180.0;
          sc += cosf(rad); ss += sinf(rad);
          sumS += s; sumV += v; n++;
        }
      }
    }
    esp_camera_fb_return(fb);
  }

  if (framesOk == 0) {
    Serial.println("❌ [CAMERA] No image obtained -> X");
    return 'X';
  }

  if (n < MIN_PIXEL_RATIO * ROI * ROI * framesOk) {
    Serial.printf("⚪ [CAMERA] Not enough colored pixels (%ld) : white/gray/empty -> X\n", n);
    return 'X';
  }

  float hm = atan2(ss, sc) * 180.0 / M_PI;
  if (hm < 0) hm += 360;

  for (int k = 0; k < 3; k++) {
    if (angDiff(hm, hCenter[k]) <= hTol[k]) {
      Serial.printf("🎨 [CAMERA] %s | H=%.1f° S=%.2f V=%.2f (%ld px) -> %c\n",
                    NAMES[k], hm, sumS / n, sumV / n, n, CODES[k]);
      return CODES[k];
    }
  }
  Serial.printf("❓ [CAMERA] Unknown color | H=%.1f° S=%.2f V=%.2f (%ld px) -> X\n",
                hm, sumS / n, sumV / n, n);
  return 'X';
}

// =========================================================================
// 4. ULTRASONIC DETECTION (Triggered if < 7cm)
// =========================================================================
bool isDetectionTriggered() {
  float distance = measureDistanceCm();

  if (distance > 0 && distance < 7.0) {
    if (!previousObjectPresent && (millis() - lastTrigger > CAPTURE_DELAY_MS)) {
      previousObjectPresent = true;
      lastTrigger = millis();
      Serial.printf("🎯 [ULTRASONIC] Cube detected at %.1f cm (< 7 cm)!\n", distance);
      return true;
    }
  } else if (distance >= 9.0) { 
    previousObjectPresent = false;
  }
  return false;
}

// =========================================================================
// 5. SETUP & LOOP
// =========================================================================
void setup() {
  Serial.begin(115200);                                        
  Serial1.begin(9600, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);   

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_BTN_ARRET, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_ARRET), isrStopButton, CHANGE);

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

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera initialization error: 0x%x\n", err);
    Serial.printf("PSRAM: %s (%u bytes)\n", psramFound() ? "OK" : "ABSENT", ESP.getPsramSize());
    while (1) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);

  Serial.println("XIAO ready: place a YELLOW, GREEN, or BLUE cube at less than 7 cm.");
}

void loop() {
  if (isDetectionTriggered()) {
    char code = analyzeCube();
    Serial1.write(code);                       
    Serial.printf("📤 [UART] Sent: %c\n", code);
  }

  if (stopRequested) {
    stopRequested = false;
    Serial1.write(STOP_CODE);
    Serial.println("🛑 [BUTTON] Press detected -> 'A' sent (emergency stop)");
  }

  while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r' || c == ' ') continue;   
      c = toupper(c);                                      
      Serial1.write(c);
      Serial.printf("⌨️ [DEBUG] '%c' retransmitted to ESP32\n", c);
  }
}
