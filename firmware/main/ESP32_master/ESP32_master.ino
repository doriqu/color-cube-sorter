// =========================================================================
// 0. BLYNK (must come BEFORE the Blynk #includes)
// =========================================================================
#define BLYNK_TEMPLATE_ID   "TMPL2qTbX3jmd"
#define BLYNK_TEMPLATE_NAME "Tri Industriel"
#define BLYNK_AUTH_TOKEN    "I8FXoE_PgriUNRmdeYW3N6cuVDfQfkgl"      // <-- Device authentication token

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>

// Timestamped log on the serial monitor: [  12345 ms] message
#define LOG(fmt, ...) Serial.printf("[%8lu ms] " fmt "\n", millis(), ##__VA_ARGS__)

// Colour codes (J / V / B / X) are the single-letter protocol used by the XIAO camera:
//   J = Jaune (YELLOW), V = Vert (GREEN), B = Bleu (BLUE), X = unknown.
// They are kept unchanged so the camera firmware does not need to be modified.

// =========================================================================
// 1. PIN ASSIGNMENT
// =========================================================================
// Stepper motor (TB6600): DIR is wired directly to Vcc (single direction)
const uint8_t PIN_PUL = 5;
const uint8_t PIN_ENA = 2;               // GPIO2 = strapping pin
const uint8_t ENA_ACTIVE_LEVEL = LOW;    // level that ENABLES the driver (depends on ENA+/ENA- wiring)

const uint8_t PIN_SERVO = 23;

const uint8_t PIN_SENSOR_START = 33;     // ADC1
const uint8_t PIN_SENSOR_END   = 32;     // ADC1

const uint8_t PIN_LED_GREEN  = 27;       // belt running
const uint8_t PIN_LED_YELLOW = 26;       // belt stopped / paused / error
const uint8_t PIN_LED_RED    = 14;

// LED that blinks to signal an unknown cube (X). Set to PIN_LED_RED to use the red one.
const uint8_t PIN_LED_ALERT = PIN_LED_RED;

const uint8_t PIN_PAUSE = 12;            // toggle switch to GND (INPUT_PULLUP): used ONLY to RESUME
                                         // after an emergency stop (any state change = resume)

const uint8_t LCD_ADDR = 0x27;
const uint8_t PIN_SDA  = 21;
const uint8_t PIN_SCL  = 22;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

const uint8_t PIN_UART_RX = 16;          // <- XIAO TX
const uint8_t PIN_UART_TX = 17;          // not used for now

// =========================================================================
// 2. PARAMETERS
// =========================================================================
// Motor
const float STEPS_PER_REV_MOTOR = 200.0;
const float MICROSTEP           = 1.0;   // TB6600 set to 1 microstep
float rpm                       = 24;    // belt speed (changeable via setRpm)

// Geometry (to be measured). Cubes must be spaced at least END_TO_BIN_DISTANCE_CM apart.
const float ROLLER_PERIMETER_CM    = 10.0;
const float END_TO_BIN_DISTANCE_CM = 5.0;

// Bin servo: 3 colours sorted for now (180° servo)
const int ANGLE_J    = 0;
const int ANGLE_B    = 90;
const int ANGLE_V    = 180;
const int ANGLE_HOME = 90;               // position at startup / after emergency stop

// Analog laser-beam sensors.
// true  = the beam is received at rest (high value ~4095), a cube cutting it makes the value drop (~76)
// false = the value rises when a cube is present
const bool SENSOR_START_ACTIVE_LOW = true;
const bool SENSOR_END_ACTIVE_LOW   = true;
const int DETECTION_THRESHOLD = 2000;
const unsigned long DEBOUNCE_START_MS = 600;
const unsigned long DEBOUNCE_END_MS   = 300;

// Watchdog
const bool WATCHDOG_ENABLED = true;
const unsigned long INACTIVITY_TIMEOUT_MS = 120000;

// Delayed belt start
const unsigned long START_DELAY_MS = 300;

// Toggle switch (resume) + emergency stop
const unsigned long TOGGLE_DEBOUNCE_MS = 50;
const char STOP_CODE = 'A';                           // received from the XIAO (push button) or typed on the keyboard
const unsigned long EMERGENCY_HALF_PERIOD_MS = 250;   // red LED blinking during the stop

// Alert LED
const unsigned long ALERT_HALF_PERIOD_MS = 200;
const int ALERT_BLINK_COUNT = 3;         // when an 'X' is received

// Unknown-cube error: resume as soon as the end sensor is clear.
const int CLEAR_SAMPLES_REQUIRED = 5;

// LCD: periodic full rewrite to erase any stray characters (0 = disabled)
const unsigned long LCD_REFRESH_MS = 5000;

// Debug
const bool KEYBOARD_SIMULATION = true;            // type J/V/B/X in the serial monitor
const bool DEBUG_RAW_UART = true;                 // print every byte received from the camera
const unsigned long HEARTBEAT_MS = 5000;          // periodic status line (0 = disabled)

// ---- Wi-Fi / Blynk ------------------------------------------------------
const char* WIFI_SSID = "Infinix HOT 50 Pro+";
const char* WIFI_PASS = "doriqueleee";
const char* BLYNK_SERVER = "ny3.blynk.cloud";     // NY3 region (seen in the dashboard URL)

const char* RESET_CODE = "EPAC";                  // password to reset the counters to zero

// Time zone for timestamps (Benin = UTC+1, no daylight saving)
const long GMT_OFFSET_SEC = 3600;

// Order of the items in the "Bin Selection" (V15) dropdown on the dashboard.
// Index 0 = first item in the list. ADJUST if your order is different.
const char BIN_MENU[3] = { 'J', 'B', 'V' };       // 0 = YELLOW, 1 = BLUE, 2 = GREEN
const int  MENU_OFFSET = 0;                       // set to 1 if your Blynk menu returns 1,2,3 instead of 0,1,2

// Max send rate to Blynk (the cloud limit is ~10 messages/s)
const unsigned long BLYNK_INTERVAL_MS = 110;

// =========================================================================
// 3. SYSTEM STATE
// =========================================================================
bool beltActive = false;
bool beltStartPending = false;
unsigned long beltStartTime = 0;

bool paused = false;
unsigned long pauseStart = 0;

bool cubeError = false;                  // unknown cube stuck on the end sensor
unsigned long errorStart = 0;
int errorCubePos = 0;                    // position of the faulty cube in the queue
int errorClearReadings = 0;

struct Cube {
  char color;                   // 'J','V','B','X' or '?'
  bool reachedEnd;
  bool binAligned;
  unsigned long entryTime;
  unsigned long dropTime;       // never modified after detection at the end
};

const int MAX_CUBES_ON_BELT = 10;
Cube cubeQueue[MAX_CUBES_ON_BELT];
int queueHead = 0, queueTail = 0, cubesOnBelt = 0;

unsigned long lastActivityTime = 0;
const char* lastActivitySource = "startup";
unsigned long lastHeartbeat = 0;
unsigned long uartByteCount = 0;         // total bytes received from the camera

// Sensors: edge detection (cube arriving) + debounce
struct Sensor {
  uint8_t pin;
  unsigned long debounceMs;
  unsigned long lastMs;
  bool previous;                // cube present at the previous reading
  int value;                    // last analog value read
  bool activeLow;               // true: cube present = low value
};
Sensor sensorStart = { PIN_SENSOR_START, DEBOUNCE_START_MS, 0, false, 0, SENSOR_START_ACTIVE_LOW };
Sensor sensorEnd   = { PIN_SENSOR_END,   DEBOUNCE_END_MS,   0, false, 0, SENSOR_END_ACTIVE_LOW };

// Toggle switch (debounced): we store the LEVEL (true = HIGH, false = LOW)
bool toggleRaw = true, toggleStable = true;
unsigned long toggleLastChange = 0;

// Alert LED (non-blocking blinking)
int alertHalfCyclesLeft = 0;
bool alertState = false;
unsigned long alertLastChange = 0;

// LCD: last message displayed (so it can be rewritten)
char lcdLine1[17] = "";
char lcdLine2[17] = "";
unsigned long lastLcdRefresh = 0;

// Sorting counters (saved in flash memory: they survive a reboot)
Preferences prefs;
uint32_t cntJ = 0, cntV = 0, cntB = 0, cntX = 0;
char lastSorted[16] = "-";
char binColor = 'B';                     // last bin position (ANGLE_HOME = 90° = blue)

// =========================================================================
// 3bis. BLYNK: everything runs in a separate task (core 0)
//       -> a slow Wi-Fi/Cloud NEVER blocks the belt or the sensors.
//       The main loop sends messages through a queue and reads
//       commands via "volatile" flags.
// =========================================================================
struct Msg {
  uint8_t pin;
  uint8_t type;                 // 0 = integer, 1 = text
  int32_t iv;
  char txt[100];
};
QueueHandle_t blynkQueue = nullptr;

// Commands received from the dashboard (written by the Blynk task, read by loop)
volatile int8_t cmdPlayPause = 0;        // 0 = nothing, 1 = PLAY (resume), 2 = PAUSE
volatile int   cmdBinIndex   = -1;       // -1 = nothing, otherwise dropdown index
volatile bool  resetArmed    = false;    // "Reset DB" switch (V13)
volatile bool  codeReceived  = false;    // a code was entered in V16
char           codeText[24]  = "";
volatile bool  resyncRequested = false;  // Blynk (re)connection: resend the full state
volatile bool  blynkOnline   = false;
bool forceState = true;                  // force resending the LEDs / Play-Pause

void pushInt(uint8_t pin, int32_t v) {
  if (!blynkQueue) return;
  Msg m; m.pin = pin; m.type = 0; m.iv = v; m.txt[0] = 0;
  xQueueSend(blynkQueue, &m, 0);         // 0 = never blocks (message lost if queue is full)
}

void pushTxt(uint8_t pin, const char* s) {
  if (!blynkQueue) return;
  Msg m; m.pin = pin; m.type = 1; m.iv = 0;
  snprintf(m.txt, sizeof(m.txt), "%s", s);
  xQueueSend(blynkQueue, &m, 0);
}

// Real time (NTP) if available, otherwise time since boot
void timestamp(char* buf, size_t n) {
  struct tm t;
  if (getLocalTime(&t, 0)) {
    strftime(buf, n, "%H:%M:%S", &t);
  } else {
    unsigned long s = millis() / 1000;
    snprintf(buf, n, "+%02lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
  }
}

// One line in the "History" Terminal widget (V12): [HH:MM:SS] message
void termLog(const char* fmt, ...) {
  char ts[16];
  timestamp(ts, sizeof(ts));
  char body[80];
  va_list args;
  va_start(args, fmt);
  vsnprintf(body, sizeof(body), fmt, args);
  va_end(args);
  char line[100];
  snprintf(line, sizeof(line), "[%s] %s\n", ts, body);
  pushTxt(V12, line);
}

// --- Handlers: called by the Blynk task when the dashboard changes ------
BLYNK_CONNECTED() {
  resyncRequested = true;
}

BLYNK_WRITE(V10) {                       // Play / Pause
  cmdPlayPause = param.asInt() ? 1 : 2;
}

BLYNK_WRITE(V12) {                       // Terminal: input is IGNORED
}

BLYNK_WRITE(V13) {                       // Reset DB: "arm" (ON) or cancel (OFF)
  resetArmed = param.asInt() != 0;
}

BLYNK_WRITE(V15) {                       // Bin Selection
  cmdBinIndex = param.asInt() - MENU_OFFSET;
}

BLYNK_WRITE(V16) {                       // Confirmation code (Enter)
  snprintf(codeText, sizeof(codeText), "%s", param.asStr());
  codeReceived = true;                   // written last
}

void blynkTask(void*) {
  Blynk.config(BLYNK_AUTH_TOKEN, BLYNK_SERVER);
  unsigned long lastSend = 0;
  Msg m;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      blynkOnline = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    Blynk.run();                         // may block during a reconnection: irrelevant here
    blynkOnline = Blynk.connected();

    if (blynkOnline) {
      if (millis() - lastSend >= BLYNK_INTERVAL_MS && xQueueReceive(blynkQueue, &m, 0) == pdTRUE) {
        if (m.type == 0) Blynk.virtualWrite(m.pin, (int)m.iv);
        else             Blynk.virtualWrite(m.pin, m.txt);
        lastSend = millis();
      }
    } else {
      xQueueReset(blynkQueue);           // offline: discard, the full state will be resent on reconnection
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =========================================================================
// 4. TIME: always a fresh read of millis()
// =========================================================================
unsigned long elapsedSince(unsigned long t) { return millis() - t; }

void signalActivity(const char* source) {
  lastActivityTime = millis();
  lastActivitySource = source;
}

// =========================================================================
// 5. UTILITIES
// =========================================================================
unsigned long computeDropDelayMs() {
  if (rpm <= 0) return 2000;
  float speedCmPerSec = (rpm / 60.0) * ROLLER_PERIMETER_CM;
  return (unsigned long)((END_TO_BIN_DISTANCE_CM / speedCmPerSec) * 1000.0);
}

float stepFrequencyHz() { return rpm * STEPS_PER_REV_MOTOR * MICROSTEP / 60.0; }

void startMotor() {
  ledcWriteTone(PIN_PUL, (uint32_t)stepFrequencyHz());
  LOG("⚙️ [MOTOR] ON (%.0f Hz, %.1f rpm)", stepFrequencyHz(), rpm);
}
void stopMotor() {
  ledcWriteTone(PIN_PUL, 0);
  LOG("⚙️ [MOTOR] OFF");
}

// Change the speed (avoid during a sort in progress)
void setRpm(float newRpm) {
  rpm = newRpm;
  if (beltActive && !paused && !cubeError) startMotor();
}

void servoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  float pulseUs = 500.0 + (angle / 180.0) * 1900.0;         // 500..2400 µs
  uint32_t duty = (uint32_t)(pulseUs / 20000.0 * 16383.0);  // 14 bits @ 50 Hz
  ledcWrite(PIN_SERVO, duty);
}

const char* colorName(char c) {
  switch (c) {
    case 'J': return "YELLOW";
    case 'V': return "GREEN";
    case 'B': return "BLUE";
    default:  return "UNKNOWN";
  }
}

// Non-sortable colour: X, '?' or any other value
bool isUnknown(char c) { return c != 'J' && c != 'V' && c != 'B'; }

// Servo angle for a colour (-1 if not sortable)
int angleFor(char c) {
  switch (c) {
    case 'J': return ANGLE_J;
    case 'V': return ANGLE_V;
    case 'B': return ANGLE_B;
    default:  return -1;
  }
}

// Blynk dropdown index for a colour
int menuIndex(char c) {
  for (int i = 0; i < 3; i++) if (BIN_MENU[i] == c) return i + MENU_OFFSET;
  return MENU_OFFSET;
}

// --- LCD ---------------------------------------------------------------
void writeLcd() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(lcdLine1);
  lcd.setCursor(0, 1); lcd.print(lcdLine2);
}

void showLcd(const char* line1, const char* line2 = "") {
  snprintf(lcdLine1, sizeof(lcdLine1), "%s", line1);
  snprintf(lcdLine2, sizeof(lcdLine2), "%s", line2);
  writeLcd();
}

// Re-initialises the screen and rewrites the last message: erases stray characters
void refreshLcd() {
  if (LCD_REFRESH_MS == 0 || elapsedSince(lastLcdRefresh) < LCD_REFRESH_MS) return;
  lcd.init();
  lcd.backlight();
  writeLcd();
  lastLcdRefresh = millis();
}

// Positions the bin, updates the LCD AND the dashboard dropdown (V15)
void orientBin(char color) {
  int angle = angleFor(color);
  if (angle < 0) {
    LOG("⚠️ [BIN] Colour '%c' is not sortable: bin not oriented.", color);
    return;
  }
  servoAngle(angle);
  binColor = color;
  pushInt(V15, menuIndex(color));
  showLcd("BIN:", colorName(color));
  LOG("🔀 [BIN] Oriented for '%c' (%s) → servo at %d°", color, colorName(color), angle);
}

void resetQueue() { cubesOnBelt = 0; queueHead = 0; queueTail = 0; }

// Removes the cube at position pos (0 = head) by shifting the following ones
void removeCubeAt(int pos) {
  if (pos < 0 || pos >= cubesOnBelt) return;
  for (int k = pos; k < cubesOnBelt - 1; k++) {
    cubeQueue[(queueHead + k) % MAX_CUBES_ON_BELT] = cubeQueue[(queueHead + k + 1) % MAX_CUBES_ON_BELT];
  }
  cubesOnBelt--;
  queueTail = (queueTail - 1 + MAX_CUBES_ON_BELT) % MAX_CUBES_ON_BELT;
}

// =========================================================================
// 5bis. COUNTERS + DASHBOARD
// =========================================================================
uint32_t totalSorted() { return cntJ + cntV + cntB + cntX; }

void saveCounters() {
  prefs.putUInt("J", cntJ);
  prefs.putUInt("V", cntV);
  prefs.putUInt("B", cntB);
  prefs.putUInt("X", cntX);
}

// Sends all counters to the dashboard
void pushCounters() {
  pushInt(V1, (int32_t)totalSorted());
  pushInt(V5, (int32_t)cntJ);
  pushInt(V6, (int32_t)cntV);
  pushInt(V7, (int32_t)cntB);
  pushInt(V8, (int32_t)cntX);
}

// A cube has just been sorted (or removed if unknown)
void recordSort(char color) {
  switch (color) {
    case 'J': cntJ++; break;
    case 'V': cntV++; break;
    case 'B': cntB++; break;
    default:  cntX++; break;
  }
  saveCounters();
  snprintf(lastSorted, sizeof(lastSorted), "%s", colorName(color));

  pushTxt(V0, lastSorted);
  pushCounters();
  pushInt(V11, (int32_t)totalSorted());  // chart: the total over time
  termLog("%s sorted | total: %lu", colorName(color), (unsigned long)totalSorted());
}

void resetCounters() {
  cntJ = cntV = cntB = cntX = 0;
  saveCounters();
  snprintf(lastSorted, sizeof(lastSorted), "-");
  pushTxt(V0, lastSorted);
  pushCounters();
  pushInt(V11, 0);
  LOG("🗑️ [RESET] Counters reset to zero from the dashboard.");
  termLog("Counters reset");
}

// After (re)connection: resend everything so the dashboard reflects the real system state
void sendFullState() {
  pushTxt(V0, lastSorted);
  pushCounters();
  pushInt(V13, 0);
  pushTxt(V16, "");
  pushInt(V15, menuIndex(binColor));
  forceState = true;
  termLog("ESP32 connected");
  LOG("☁️ [BLYNK] Connected: full state resent to the dashboard.");
}

// Virtual Run/Pause/Alert LEDs + Play/Pause button: sent only when they change
void syncBlynkState() {
  static int8_t last[4] = { -1, -1, -1, -1 };
  if (forceState) {
    for (auto &d : last) d = -1;
    forceState = false;
  }
  bool running = beltActive && !paused && !cubeError;
  int8_t v[4] = {
    (int8_t)(running ? 1 : 0),                     // V2 Running (= green LED)
    (int8_t)(!running ? 1 : 0),                    // V3 Paused  (= yellow LED)
    (int8_t)((paused || cubeError) ? 1 : 0),       // V4 Alert   (= red LED)
    (int8_t)(paused ? 0 : 1)                       // V10 Play/Pause: ON = system active
  };
  const uint8_t pins[4] = { V2, V3, V4, V10 };
  for (int i = 0; i < 4; i++) {
    if (v[i] != last[i]) {
      last[i] = v[i];
      pushInt(pins[i], (i < 3) ? (v[i] ? 255 : 0) : v[i]);
    }
  }
}

// =========================================================================
// 6. SENSORS
// =========================================================================
// Is a cube in front of the sensor? (based on the last value read)
bool cubePresent(const Sensor &s) {
  return s.activeLow ? (s.value < DETECTION_THRESHOLD) : (s.value > DETECTION_THRESHOLD);
}

// Returns true only once, when a cube ARRIVES in front of the sensor
bool risingEdge(Sensor &s) {
  s.value = analogRead(s.pin);
  bool present = cubePresent(s);
  bool edge = present && !s.previous && elapsedSince(s.lastMs) > s.debounceMs;
  if (edge) s.lastMs = millis();
  s.previous = present;
  return edge;
}

// Avoids a false detection at startup: start from the real state of the sensors
void initSensors() {
  sensorStart.value = analogRead(sensorStart.pin);
  sensorEnd.value   = analogRead(sensorEnd.pin);
  sensorStart.previous = cubePresent(sensorStart);
  sensorEnd.previous   = cubePresent(sensorEnd);
  LOG("📍 [SENSORS] At startup: START = %d (%s) | END = %d (%s)",
      sensorStart.value, cubePresent(sensorStart) ? "CUBE" : "clear",
      sensorEnd.value,   cubePresent(sensorEnd)   ? "CUBE" : "clear");
}

// =========================================================================
// 7. LEDs
// =========================================================================
void triggerAlertX() {
  alertHalfCyclesLeft = ALERT_BLINK_COUNT * 2;
  alertState = false;
  alertLastChange = millis() - ALERT_HALF_PERIOD_MS;  // first change is immediate
}

// Blinking phase: 3 blinks after an 'X' is received; continuous during an error
void updateAlert() {
  bool blinking = cubeError || alertHalfCyclesLeft > 0;
  if (!blinking) { alertState = false; return; }
  if (elapsedSince(alertLastChange) >= ALERT_HALF_PERIOD_MS) {
    alertState = !alertState;
    alertLastChange = millis();
    if (!cubeError && alertHalfCyclesLeft > 0) alertHalfCyclesLeft--;
  }
}

void updateLeds() {
  updateAlert();
  bool running = beltActive && !paused && !cubeError;
  // During the emergency stop, only the red LED blinks (the yellow one stays on)
  bool blinking = !paused && (cubeError || alertHalfCyclesLeft > 0 || alertState);
  bool redEmergency = paused && ((millis() / EMERGENCY_HALF_PERIOD_MS) % 2 == 0);

  digitalWrite(PIN_LED_GREEN, running);
  if (PIN_LED_ALERT == PIN_LED_YELLOW) {
    digitalWrite(PIN_LED_YELLOW, blinking ? alertState : !running);
    digitalWrite(PIN_LED_RED, redEmergency);
  } else {
    digitalWrite(PIN_LED_YELLOW, !running);
    digitalWrite(PIN_LED_RED, redEmergency || (blinking && alertState));
  }
  syncBlynkState();
}

// =========================================================================
// 8. EMERGENCY STOP / RESUME
// =========================================================================
// Triggered by 'A' (XIAO push button) or by the dashboard Play/Pause button.
void enterPause(const char* source) {
  if (paused) return;
  paused = true;
  pauseStart = millis();
  stopMotor();
  showLcd("EMERGENCY STOP", "Flip switch");
  LOG("⏸️ [EMERGENCY] Stop requested (%s): system paused. Flip the switch (or press Play) to resume.", source);
  termLog("EMERGENCY STOP (%s)", source);
}

void exitPause() {
  unsigned long duration = elapsedSince(pauseStart);

  // Freeze all timers: shift them by the duration of the pause
  for (int i = 0; i < cubesOnBelt; i++) {
    int idx = (queueHead + i) % MAX_CUBES_ON_BELT;
    cubeQueue[idx].entryTime += duration;
    if (cubeQueue[idx].reachedEnd) cubeQueue[idx].dropTime += duration;
  }
  beltStartTime += duration;
  errorStart += duration;
  errorClearReadings = 0;

  paused = false;
  if (beltActive && !cubeError) startMotor();
  signalActivity("resume");
  showLcd(cubeError ? "CUBE ERROR" : "RESUMED", cubeError ? "Remove the cube" : "");
  LOG("▶️ [PAUSE] Resumed after %lu ms.", duration);
  termLog("Resumed after %lu ms", duration);
}

// Continuously tracks the (debounced) state of the toggle switch.
// If it CHANGES state during an emergency stop, the system resumes.
void handleToggleSwitch() {
  bool reading = digitalRead(PIN_PAUSE);
  if (reading != toggleRaw) {
    toggleRaw = reading;
    toggleLastChange = millis();
  }
  if (elapsedSince(toggleLastChange) >= TOGGLE_DEBOUNCE_MS && toggleStable != toggleRaw) {
    toggleStable = toggleRaw;
    LOG("🔘 [TOGGLE] Switch changed to %s.", toggleStable ? "HIGH" : "LOW");
    if (paused) exitPause();
  }
}

// =========================================================================
// 8bis. COMMANDS COMING FROM THE BLYNK DASHBOARD
// =========================================================================
void handleBlynkCommands() {
  // (Re)connection: resend the full state
  if (resyncRequested) {
    resyncRequested = false;
    sendFullState();
  }

  // Play / Pause button (V10)
  int8_t pp = cmdPlayPause;
  if (pp) {
    cmdPlayPause = 0;
    if (pp == 2) enterPause("DASHBOARD");
    else if (paused) exitPause();
    else forceState = true;               // already running: just set the button back to ON
  }

  // Bin Selection dropdown (V15) -> the servo follows
  int bi = cmdBinIndex;
  if (bi >= 0) {
    cmdBinIndex = -1;
    if (bi < 3 && !paused) {
      char c = BIN_MENU[bi];
      servoAngle(angleFor(c));
      binColor = c;
      showLcd("BIN (manual):", colorName(c));
      LOG("🎛️ [BIN] Dashboard selection: '%c' (%s) → servo at %d°", c, colorName(c), angleFor(c));
      termLog("Manual bin: %s", colorName(c));
    } else {
      pushInt(V15, menuIndex(binColor));  // refused (paused): put the dropdown back on the real position
    }
  }

  // Confirmation code (V16): only has an effect if "Reset DB" (V13) is ON
  if (codeReceived) {
    codeReceived = false;
    char code[24];
    snprintf(code, sizeof(code), "%s", codeText);
    // strip spaces/newlines around the text
    char *p = code;
    while (*p == ' ') p++;
    for (int n = strlen(p); n > 0 && (p[n - 1] == ' ' || p[n - 1] == '\r' || p[n - 1] == '\n'); n--) p[n - 1] = 0;

    if (resetArmed) {
      if (strcasecmp(p, RESET_CODE) == 0) {
        resetCounters();
      } else {
        LOG("⛔ [RESET] Wrong code.");
        termLog("Wrong code: reset cancelled");
      }
      resetArmed = false;
      pushInt(V13, 0);                    // the Reset button automatically goes back to OFF
    }
    pushTxt(V16, "");                     // clear the input box in all cases
  }
}

// =========================================================================
// 9. SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_ENA, ENA_ACTIVE_LEVEL);

  // Hardware PWM: channels set by hand (0 and 1 share the same timer on ESP32)
  ledcAttachChannel(PIN_PUL, 1000, 8, 0);
  stopMotor();
  ledcAttachChannel(PIN_SERVO, 50, 14, 2);
  servoAngle(ANGLE_HOME);

  pinMode(PIN_SENSOR_START, INPUT);
  pinMode(PIN_SENSOR_END, INPUT);
  pinMode(PIN_PAUSE, INPUT_PULLUP);
  delay(5);
  toggleRaw = toggleStable = digitalRead(PIN_PAUSE);   // start from the real switch position
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_RED, LOW);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);        // I2C at 100 kHz: more noise tolerant
  Wire.setTimeOut(100);         // prevents an I2C hang from freezing the program
  lcd.init();
  lcd.backlight();
  showLcd("System ready", "Waiting...");
  lastLcdRefresh = millis();

  initSensors();

  // Saved counters
  prefs.begin("tri", false);
  cntJ = prefs.getUInt("J", 0);
  cntV = prefs.getUInt("V", 0);
  cntB = prefs.getUInt("B", 0);
  cntX = prefs.getUInt("X", 0);
  LOG("💾 [COUNTERS] J=%lu V=%lu B=%lu X=%lu (total %lu)",
      (unsigned long)cntJ, (unsigned long)cntV, (unsigned long)cntB, (unsigned long)cntX, (unsigned long)totalSorted());

  // Wi-Fi (non-blocking) + NTP time + Blynk task on core 0
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTime(GMT_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");
  blynkQueue = xQueueCreate(48, sizeof(Msg));
  xTaskCreatePinnedToCore(blynkTask, "blynk", 8192, nullptr, 1, nullptr, 0);

  signalActivity("startup");
  updateLeds();
  LOG("✔️ [SYSTEM] Ready | %.1f rpm | %.0f Hz | drop delay %lu ms | watchdog %s (%lu s)",
      rpm, stepFrequencyHz(), computeDropDelayMs(),
      WATCHDOG_ENABLED ? "ON" : "OFF", INACTIVITY_TIMEOUT_MS / 1000);
  LOG("✔️ [SYSTEM] Servo: J=%d° B=%d° V=%d° | sensor threshold = %d", ANGLE_J, ANGLE_B, ANGLE_V, DETECTION_THRESHOLD);
  LOG("✔️ [SYSTEM] Camera UART: RX=GPIO%d at 9600 baud, 8N1.", PIN_UART_RX);
  LOG("✔️ [SYSTEM] Wi-Fi '%s' connecting (non-blocking)...", WIFI_SSID);
  if (KEYBOARD_SIMULATION) LOG("⌨️ [SYSTEM] Camera simulation: type J, V, B or X in the serial monitor.");
}

// =========================================================================
// 10. LOGIC
// =========================================================================
void addNewCube() {
  if (cubesOnBelt >= MAX_CUBES_ON_BELT) {
    LOG("⚠️ [START] Queue full, cube ignored.");
    return;
  }
  cubeQueue[queueTail] = { '?', false, false, millis(), 0 };
  queueTail = (queueTail + 1) % MAX_CUBES_ON_BELT;
  cubesOnBelt++;
  LOG("📥 [START] New cube added to the queue (%d on the belt)", cubesOnBelt);

  char line2[17];
  snprintf(line2, sizeof(line2), "On belt: %d", cubesOnBelt);
  showLcd("New cube", line2);

  if (!beltActive && !beltStartPending) {
    beltStartPending = true;
    beltStartTime = millis();
    LOG("⏱️ [BELT] Starting in %lu ms.", START_DELAY_MS);
  }
}

// A single function for the colour, whether it comes from the XIAO or the keyboard
void receiveColor(char code, const char* source) {
  code = toupper(code);
  if (code != 'J' && code != 'V' && code != 'B' && code != 'X') return; // ignore \r \n etc.
  signalActivity(source);

  for (int i = 0; i < cubesOnBelt; i++) {
    int idx = (queueHead + i) % MAX_CUBES_ON_BELT;
    if (cubeQueue[idx].color == '?') {
      cubeQueue[idx].color = code;
      LOG("📷 [%s] Colour '%c' (%s) received → assigned to the cube at position %d", source, code, colorName(code), i);
      if (code == 'X') {
        LOG("🟡 [CAMERA] Unknown colour: the cube will block the belt at the end of its run.");
        triggerAlertX();
      }
      return;
    }
  }
  LOG("📷 [%s] Colour '%c' (%s) received but no cube is waiting: ignored.", source, code, colorName(code));
}

// Reads a byte from the camera UART and prints it as-is (debug)
int readCameraByte(bool accepted) {
  int byteRead = Serial1.read();
  uartByteCount++;
  if (DEBUG_RAW_UART) {
    if (byteRead >= 32 && byteRead < 127)
      LOG("🔌 [UART RX] byte 0x%02X '%c'%s", byteRead, (char)byteRead, accepted ? "" : " (ignored: emergency stop)");
    else
      LOG("🔌 [UART RX] byte 0x%02X (non-printable)%s", byteRead, accepted ? "" : " (ignored: emergency stop)");
  }
  return byteRead;
}

// Dispatches a received byte: 'A' = emergency stop, otherwise colour (J/V/B/X)
void receiveByte(char c, const char* source) {
  if (toupper(c) == STOP_CODE) { enterPause(source); return; }
  receiveColor(c, source);
}

void handleCameraUart() {
  while (Serial1.available() > 0) receiveByte((char)readCameraByte(true), "UART");
  if (KEYBOARD_SIMULATION) {
    while (Serial.available() > 0) receiveByte((char)Serial.read(), "KEYBOARD");
  }
}

// Unknown cube at the end of the belt: the motor stops immediately, right on the end sensor
void triggerCubeError(int pos) {
  cubeError = true;
  errorCubePos = pos;
  errorStart = millis();
  errorClearReadings = 0;
  stopMotor();
  showLcd("CUBE ERROR", "Remove the cube");
  LOG("🚨 [ERROR] Unknown-colour cube on the end sensor: motor stopped, remove it by hand.");
  termLog("ERROR: unknown cube, remove it");
}

// After the head cube is sorted / removed: prepare the next one
void advanceToNextCube(unsigned long dropDelay) {
  if (cubesOnBelt == 0) {
    beltActive = false;
    stopMotor();
    LOG("🛑 [BELT] Queue empty. Normal motor stop.");
    return;
  }
  Cube &next = cubeQueue[queueHead];
  if (next.reachedEnd && !next.binAligned) {
    next.binAligned = true;
    orientBin(next.color);
    unsigned long elapsed = elapsedSince(next.dropTime);
    if (elapsed >= dropDelay)
      LOG("⚠️ [ALERT] Cubes too close together: the next one already dropped before the bin was oriented!");
    else
      LOG("ℹ️ [BIN] Oriented with %lu ms to spare before the next cube drops.", dropDelay - elapsed);
  }
}

void handleEndOfBelt() {
  for (int i = 0; i < cubesOnBelt; i++) {
    int idx = (queueHead + i) % MAX_CUBES_ON_BELT;
    if (!cubeQueue[idx].reachedEnd) {
      cubeQueue[idx].reachedEnd = true;
      cubeQueue[idx].dropTime = millis();
      LOG("🏁 [END] Cube '%c' (%s, position %d) reaches the end of the belt.",
          cubeQueue[idx].color, colorName(cubeQueue[idx].color), i);

      if (isUnknown(cubeQueue[idx].color)) {
        // Immediate stop, regardless of position in the queue
        if (i > 0) LOG("⚠️ [ALERT] A previous cube is still dropping: spacing between cubes is < %.0f cm.", END_TO_BIN_DISTANCE_CM);
        cubeQueue[idx].binAligned = true;
        triggerCubeError(i);
      } else if (i == 0) {
        cubeQueue[idx].binAligned = true;
        orientBin(cubeQueue[idx].color);
      } else {
        LOG("⏳ [BIN] Orientation postponed: previous cube still dropping.");
      }
      return;
    }
  }
  LOG("ℹ️ [END] Detection at the end of the belt but no cube waiting: ignored.");
}

// Waits for the stuck cube to be removed by hand: resume as soon as the end sensor is clear
void handleCubeError() {
  if (cubePresent(sensorEnd)) errorClearReadings = 0;
  else errorClearReadings++;
  if (errorClearReadings < CLEAR_SAMPLES_REQUIRED) return;

  unsigned long duration = elapsedSince(errorStart);
  LOG("✅ [ERROR] Cube removed (end sensor clear, value = %d, after %lu ms stopped). Resuming.", sensorEnd.value, duration);
  cubeError = false;

  recordSort('X');                        // counted in "Count UNKNOWN"

  // the faulty cube leaves the queue; the other cubes must not have "advanced" during the stop
  removeCubeAt(errorCubePos);
  for (int i = 0; i < cubesOnBelt; i++) {
    int idx = (queueHead + i) % MAX_CUBES_ON_BELT;
    cubeQueue[idx].entryTime += duration;
    if (cubeQueue[idx].reachedEnd) cubeQueue[idx].dropTime += duration;
  }

  signalActivity("error resolved");
  showLcd("Cube removed", "Resuming");

  advanceToNextCube(computeDropDelayMs());
  if (cubesOnBelt > 0) startMotor();
}

void printHeartbeat() {
  if (HEARTBEAT_MS == 0 || elapsedSince(lastHeartbeat) < HEARTBEAT_MS) return;
  lastHeartbeat = millis();
  const char* state = paused ? "PAUSED" : cubeError ? "ERROR" : beltActive ? "RUNNING" : "STOPPED";
  LOG("💓 [STATE] %s | cubes: %d | START=%d (%s) END=%d (%s) | UART: %lu bytes received | %.1f rpm | Blynk: %s",
      state, cubesOnBelt,
      sensorStart.value, cubePresent(sensorStart) ? "CUBE" : "clear",
      sensorEnd.value,   cubePresent(sensorEnd)   ? "CUBE" : "clear",
      uartByteCount, rpm, blynkOnline ? "ONLINE" : "offline");
}

// =========================================================================
// 11. LOOP
// =========================================================================
void loop() {
  handleToggleSwitch();
  handleBlynkCommands();
  printHeartbeat();
  refreshLcd();

  // --- PAUSED: everything is ignored (flush buffers and keep the edges up to date)
  if (paused) {
    while (Serial1.available() > 0) readCameraByte(false);
    if (KEYBOARD_SIMULATION) while (Serial.available() > 0) Serial.read();
    risingEdge(sensorStart);
    risingEdge(sensorEnd);
    updateLeds();
    return;
  }

  // 1. Single read of the sensors; any detection is a sign of life
  bool detectedStart = risingEdge(sensorStart);
  bool detectedEnd   = risingEdge(sensorEnd);
  if (detectedStart) {
    LOG("📍 [START SENSOR] Cube detected (value = %d)", sensorStart.value);
    signalActivity("START sensor");
    addNewCube();
  }
  if (detectedEnd) {
    LOG("📍 [END SENSOR] Cube detected (value = %d)", sensorEnd.value);
    signalActivity("END sensor");
  }

  // 2. Colour (XIAO UART or keyboard) or emergency stop 'A'
  handleCameraUart();

  // 'A' was just received: cut everything right away (no delayed start, etc.)
  if (paused) {
    updateLeds();
    return;
  }

  // 2bis. Unknown-cube error: belt stopped, waiting for manual removal
  if (cubeError) {
    handleCubeError();
    updateLeds();
    return;
  }

  // 3. Inactivity watchdog
  if (WATCHDOG_ENABLED && beltActive && elapsedSince(lastActivityTime) >= INACTIVITY_TIMEOUT_MS) {
    LOG("🚨 [EMERGENCY] %lu ms without activity (last: %s). Stop and reset!",
        elapsedSince(lastActivityTime), lastActivitySource);
    termLog("Watchdog: inactivity, resetting");
    beltActive = false;
    beltStartPending = false;
    stopMotor();
    servoAngle(ANGLE_HOME);
    binColor = 'B';
    pushInt(V15, menuIndex(binColor));
    resetQueue();
    char msg[17];
    snprintf(msg, sizeof(msg), "Idle %lus", INACTIVITY_TIMEOUT_MS / 1000);
    showLcd("EMERGENCY", msg);
    signalActivity("reset");
  }

  // 4. Delayed belt start
  if (beltStartPending && elapsedSince(beltStartTime) >= START_DELAY_MS) {
    beltActive = true;
    beltStartPending = false;
    startMotor();
    signalActivity("belt start");
    LOG("🚀 [BELT] Starting the motor.");
  }

  // 5. End-of-belt sensor
  if (detectedEnd) handleEndOfBelt();

  // 6. End of the head cube's drop (not during an error)
  if (!cubeError && cubesOnBelt > 0) {
    Cube &head = cubeQueue[queueHead];
    unsigned long dropDelay = computeDropDelayMs();

    if (head.reachedEnd && elapsedSince(head.dropTime) >= dropDelay) {
      signalActivity("cube sorted");
      LOG("✅ [SORT] ITEM SORTED | colour '%c' (%s) | delay: %lu ms | total transit: %lu ms",
          head.color, colorName(head.color), dropDelay, elapsedSince(head.entryTime));
      showLcd("ITEM SORTED", colorName(head.color));

      recordSort(head.color);             // counters + Last Sorted + History + Chart

      queueHead = (queueHead + 1) % MAX_CUBES_ON_BELT;
      cubesOnBelt--;

      advanceToNextCube(dropDelay);
    }
  }

  // 7. LEDs
  updateLeds();
}
