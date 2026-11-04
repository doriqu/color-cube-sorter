// Blynk communication test (ESP32 <-> dashboard)
// Same template, datastreams and pins as the main sorting sketch.
//   Uplink   (ESP32 -> dashboard): V0 text, V1 total, V2/V3/V4 LEDs, V5..V8 counters, V11 chart, V12 terminal, V15 bin
//   Downlink (dashboard -> ESP32): V10 Play/Pause, V13 Reset DB, V15 Bin Selection, V16 code
// Serial monitor (115200): type J / V / B / X to simulate a sorted cube, R to reset, S for status.

// Must come BEFORE the Blynk #includes
#define BLYNK_TEMPLATE_ID   "**********"    //To assign
#define BLYNK_TEMPLATE_NAME "**********"    //To assign
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"     // <-- device authentication token

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASS     = "YOUR_WIFI_PASSWORD";
const char* BLYNK_SERVER  = "ny3.blynk.cloud";          // NY3 region

const char* RESET_CODE    = "EPAC";                     // code to reset the counters
const char  BIN_MENU[3]   = { 'J', 'B', 'V' };          // dropdown order: 0 = YELLOW, 1 = BLUE, 2 = GREEN
const int   MENU_OFFSET   = 0;                          // 1 if your menu returns 1,2,3

const unsigned long AUTO_SEND_MS   = 5000;              // demo data sent every 5 s (0 = disabled)
const unsigned long STATUS_MS      = 5000;              // status line on the serial monitor
const unsigned long WIFI_RETRY_MS  = 10000;

// Test state
uint32_t cntJ = 0, cntV = 0, cntB = 0, cntX = 0;
char lastSorted[16] = "-";
bool resetArmed = false;
bool systemActive = true;                               // Play/Pause state
unsigned long lastAutoSend = 0, lastStatus = 0, lastWifiTry = 0;
unsigned long msgSent = 0, msgReceived = 0;
bool ledToggle = false;

uint32_t totalSorted() { return cntJ + cntV + cntB + cntX; }

const char* colorName(char c) {
  switch (c) {
    case 'J': return "YELLOW";
    case 'V': return "GREEN";
    case 'B': return "BLUE";
    default:  return "UNKNOWN";
  }
}

int menuIndex(char c) {
  for (int i = 0; i < 3; i++) if (BIN_MENU[i] == c) return i + MENU_OFFSET;
  return MENU_OFFSET;
}

// ---- Uplink helpers -----------------------------------------------------
void sendInt(uint8_t pin, int value) {
  if (!Blynk.connected()) return;
  Blynk.virtualWrite(pin, value);
  msgSent++;
}

void sendText(uint8_t pin, const char* text) {
  if (!Blynk.connected()) return;
  Blynk.virtualWrite(pin, text);
  msgSent++;
}

// One line in the Terminal widget (V12): [+HH:MM:SS] message
void termLog(const char* fmt, ...) {
  char body[80];
  va_list args;
  va_start(args, fmt);
  vsnprintf(body, sizeof(body), fmt, args);
  va_end(args);
  unsigned long s = millis() / 1000;
  char line[110];
  snprintf(line, sizeof(line), "[+%02lu:%02lu:%02lu] %s\n", s / 3600, (s / 60) % 60, s % 60, body);
  sendText(V12, line);
}

void sendCounters() {
  sendInt(V1, (int)totalSorted());
  sendInt(V5, (int)cntJ);
  sendInt(V6, (int)cntV);
  sendInt(V7, (int)cntB);
  sendInt(V8, (int)cntX);
}

// Sends everything: this is what the main sketch does after (re)connection
void sendFullState() {
  sendText(V0, lastSorted);
  sendCounters();
  sendInt(V2, 255);                       // Running LED
  sendInt(V3, 0);                         // Paused LED
  sendInt(V4, 0);                         // Alert LED
  sendInt(V10, systemActive ? 1 : 0);
  sendInt(V13, 0);
  sendText(V16, "");
  sendInt(V15, menuIndex('B'));
  termLog("ESP32 connected (test sketch)");
  Serial.println("[BLYNK] Full state sent to the dashboard.");
}

// Simulates a sorted cube (same effect as recordSort() in the main sketch)
void simulateSort(char c) {
  switch (c) {
    case 'J': cntJ++; break;
    case 'V': cntV++; break;
    case 'B': cntB++; break;
    default:  cntX++; break;
  }
  snprintf(lastSorted, sizeof(lastSorted), "%s", colorName(c));
  sendText(V0, lastSorted);
  sendCounters();
  sendInt(V11, (int)totalSorted());       // chart
  sendInt(V15, menuIndex(c));             // bin dropdown follows
  termLog("%s sorted | total: %lu", colorName(c), (unsigned long)totalSorted());
  Serial.printf("[TEST] Simulated sort: %s | total = %lu\n", colorName(c), (unsigned long)totalSorted());
}

void resetCounters() {
  cntJ = cntV = cntB = cntX = 0;
  snprintf(lastSorted, sizeof(lastSorted), "-");
  sendText(V0, lastSorted);
  sendCounters();
  sendInt(V11, 0);
  termLog("Counters reset");
  Serial.println("[TEST] Counters reset to zero.");
}

// ---- Downlink handlers --------------------------------------------------
BLYNK_CONNECTED() {
  Serial.println("[BLYNK] Connected to the cloud.");
  sendFullState();
}

BLYNK_WRITE(V10) {                        // Play / Pause
  msgReceived++;
  systemActive = param.asInt() != 0;
  Serial.printf("[RX V10] Play/Pause = %d -> %s\n", param.asInt(), systemActive ? "PLAY" : "PAUSE");
  sendInt(V2, systemActive ? 255 : 0);
  sendInt(V3, systemActive ? 0 : 255);
  sendInt(V4, systemActive ? 0 : 255);
  termLog(systemActive ? "Resumed from dashboard" : "EMERGENCY STOP (DASHBOARD)");
}

BLYNK_WRITE(V12) {                        // Terminal input: ignored (like the main sketch)
  msgReceived++;
  Serial.printf("[RX V12] Terminal input ignored: \"%s\"\n", param.asStr());
}

BLYNK_WRITE(V13) {                        // Reset DB: arm / cancel
  msgReceived++;
  resetArmed = param.asInt() != 0;
  Serial.printf("[RX V13] Reset DB = %d -> %s\n", param.asInt(), resetArmed ? "ARMED (enter code in V16)" : "cancelled");
}

BLYNK_WRITE(V15) {                        // Bin Selection
  msgReceived++;
  int index = param.asInt() - MENU_OFFSET;
  if (index >= 0 && index < 3) {
    char c = BIN_MENU[index];
    Serial.printf("[RX V15] Bin Selection = %d -> '%c' (%s)\n", param.asInt(), c, colorName(c));
    termLog("Manual bin: %s", colorName(c));
  } else {
    Serial.printf("[RX V15] Bin Selection = %d -> OUT OF RANGE, check MENU_OFFSET\n", param.asInt());
  }
}

BLYNK_WRITE(V16) {                        // Confirmation code
  msgReceived++;
  String code = param.asStr();
  code.trim();
  Serial.printf("[RX V16] Code received: \"%s\"\n", code.c_str());
  if (resetArmed) {
    if (code.equalsIgnoreCase(RESET_CODE)) {
      resetCounters();
    } else {
      Serial.println("[RESET] Wrong code.");
      termLog("Wrong code: reset cancelled");
    }
    resetArmed = false;
    sendInt(V13, 0);                      // Reset switch goes back to OFF
  } else {
    Serial.println("[RESET] Code ignored: Reset DB is not armed (V13 = OFF).");
  }
  sendText(V16, "");                      // clear the input box
}

// ---- Serial commands ----------------------------------------------------
void printStatus() {
  Serial.printf("[STATUS] Wi-Fi: %s (%ddBm) | Blynk: %s | sent: %lu | received: %lu | total sorted: %lu\n",
                WiFi.status() == WL_CONNECTED ? "OK" : "DOWN",
                WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
                Blynk.connected() ? "ONLINE" : "offline",
                msgSent, msgReceived, (unsigned long)totalSorted());
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = toupper((char)Serial.read());
    if (c == 'J' || c == 'V' || c == 'B' || c == 'X') simulateSort(c);
    else if (c == 'R') resetCounters();
    else if (c == 'S') printStatus();
  }
}

// ---- Setup / loop -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Blynk communication test ===");
  Serial.println("Type J / V / B / X = simulate a sorted cube | R = reset | S = status");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] Connecting to '%s'...\n", WIFI_SSID);

  Blynk.config(BLYNK_AUTH_TOKEN, BLYNK_SERVER);   // non-blocking: connection handled in Blynk.run()
}

void loop() {
  static wl_status_t lastWifi = WL_NO_SHIELD;
  wl_status_t now = WiFi.status();
  if (now != lastWifi) {
    lastWifi = now;
    if (now == WL_CONNECTED)
      Serial.printf("[WIFI] Connected | IP = %s | RSSI = %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    else
      Serial.printf("[WIFI] Not connected (status %d)\n", (int)now);
  }

  if (now == WL_CONNECTED) {
    Blynk.run();
  } else if (millis() - lastWifiTry >= WIFI_RETRY_MS) {
    lastWifiTry = millis();
    Serial.println("[WIFI] Retrying...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  handleSerial();

  // Periodic demo data: proves the uplink works without touching anything
  if (AUTO_SEND_MS > 0 && Blynk.connected() && millis() - lastAutoSend >= AUTO_SEND_MS) {
    lastAutoSend = millis();
    ledToggle = !ledToggle;
    sendInt(V2, ledToggle ? 255 : 0);
    sendInt(V3, ledToggle ? 0 : 255);
    Serial.printf("[TX] Demo: V2/V3 LEDs toggled (V2 = %d)\n", ledToggle ? 255 : 0);
  }

  if (millis() - lastStatus >= STATUS_MS) {
    lastStatus = millis();
    printStatus();
  }
}
