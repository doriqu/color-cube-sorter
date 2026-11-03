#include "esp_camera.h"
#include <math.h>

// =========================================================================
// 1. VALEURS HSV : JAUNE, VERT, BLEU
// =========================================================================
const float S_MIN = 0.16;
const float V_MIN = 0.20;
const float hCenter[3] = {64, 146, 194}; // Teinte modifiée pour le jaune (~30°) {63, 137, 187};  {12, 13, 31};
const float hTol[3]    = {12, 12, 19};
// =========================================================================

const char NAMES[3][10] = {"JAUNE", "VERT", "BLEU"};
const char CODES[3]     = {'J', 'V', 'B'};   // 'J' pour Jaune (anciennement 'R')

// =========================================================================
// 2. BROCHES & CAPTEUR ULTRASON
// =========================================================================
const uint8_t PIN_TRIG = 1;               // D0 = GPIO1
const uint8_t PIN_ECHO = 4;               // D1 = GPIO2

const int PIN_UART_TX = 43;               // D6 -> GPIO16 de l'ESP32 maître
const int PIN_UART_RX = 44;               // D7 (non utilisé pour l'instant)

// Bouton poussoir d'ARRÊT D'URGENCE (vers GND, INPUT_PULLUP) -> envoie 'A' à l'ESP32 maître
const uint8_t PIN_BTN_ARRET = 8;          // D9 = GPIO5 (libre : ni caméra, ni ultrason, ni UART)
const unsigned long ANTIREBOND_ARRET_MS = 600;
const char CODE_ARRET = 'A';

// Caméra XIAO ESP32S3 Sense
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
const int FRAMES_A_JETER = 2;             // images anciennes du tampon caméra
const int NFRAMES = 3;                    // images analysées par capture
const float RATIO_MIN_PIXELS = 0.08;      // part minimale de pixels colorés

volatile bool arretDemande = false;
volatile unsigned long dernierFrontArret = 0;

bool objetPresentPrecedent = false;
unsigned long dernierDeclenchement = 0;
const unsigned long DELAI_ENTRE_CAPTURES = 2000; // 2 secondes de pause entre deux analyses

// =========================================================================
// 2bis. BOUTON D'ARRÊT : interruption (aucun appui perdu pendant l'analyse caméra)
// =========================================================================
void IRAM_ATTR isrBoutonArret() {
  unsigned long now = millis();
  if (now - dernierFrontArret < ANTIREBOND_ARRET_MS) return;
  dernierFrontArret = now;
  if (digitalRead(PIN_BTN_ARRET) == LOW) arretDemande = true;
}

// =========================================================================
// 3. TRAITEMENT D'IMAGE & CAPTEUR ULTRASON
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

float mesurerDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duree = pulseIn(PIN_ECHO, HIGH, 30000); // Timeout à 30ms (~500 cm max)
  if (duree == 0) return 999.0;
  return (duree * 0.0343) / 2.0;
}

camera_fb_t* grab() {
  for (int retry = 0; retry < 10; retry++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) return fb;
    delay(30);
  }
  return NULL;
}

// Analyse la zone centrale ; renvoie le code UART ('J','V','B' ou 'X')
char analyserCube() {
  for (int i = 0; i < FRAMES_A_JETER; i++) {
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
    Serial.println("❌ [CAMERA] Aucune image obtenue -> X");
    return 'X';
  }

  if (n < RATIO_MIN_PIXELS * ROI * ROI * framesOk) {
    Serial.printf("⚪ [CAMERA] Pas assez de pixels colorés (%ld) : blanc/gris/vide -> X\n", n);
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
  Serial.printf("❓ [CAMERA] Couleur inconnue | H=%.1f° S=%.2f V=%.2f (%ld px) -> X\n",
                hm, sumS / n, sumV / n, n);
  return 'X';
}

// =========================================================================
// 4. DETECTION PAR ULTRASON (Déclenchement si < 7cm)
// =========================================================================
bool detectionDeclenchee() {
  float distance = mesurerDistanceCm();

  if (distance > 0 && distance < 7.0) {
    if (!objetPresentPrecedent && (millis() - dernierDeclenchement > DELAI_ENTRE_CAPTURES)) {
      objetPresentPrecedent = true;
      dernierDeclenchement = millis();
      Serial.printf("🎯 [ULTRASON] Cube détecté à %.1f cm (< 7 cm) !\n", distance);
      return true;
    }
  } else if (distance >= 9.0) { 
    objetPresentPrecedent = false;
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
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_ARRET), isrBoutonArret, CHANGE);

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
    Serial.printf("Erreur initialisation caméra : 0x%x\n", err);
    Serial.printf("PSRAM : %s (%u octets)\n", psramFound() ? "OK" : "ABSENTE", ESP.getPsramSize());
    while (1) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);

  Serial.println("XIAO prêt : placez un cube JAUNE, VERT ou BLEU à moins de 7 cm.");
}

void loop() {
  if (detectionDeclenchee()) {
    char code = analyserCube();
    Serial1.write(code);                       
    Serial.printf("📤 [UART] Envoyé : %c\n", code);
  }

  if (arretDemande) {
    arretDemande = false;
    Serial1.write(CODE_ARRET);
    Serial.println("🛑 [BOUTON] Appui détecté -> 'A' envoyé (arrêt d'urgence)");
  }

  while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r' || c == ' ') continue;   
      c = toupper(c);                                      
      Serial1.write(c);
      Serial.printf("⌨️ [DEBUG] '%c' retransmis à l'ESP32\n", c);
  }
}