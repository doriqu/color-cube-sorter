// =========================================================================
// 0. BLYNK (doit être AVANT les #include Blynk)
// =========================================================================
#define BLYNK_TEMPLATE_ID   "TMPL2qTbX3jmd"
#define BLYNK_TEMPLATE_NAME "Tri Industriel"
#define BLYNK_AUTH_TOKEN    "I8FXoE_PgriUNRmdeYW3N6cuVDfQfkgl"      // <-- Jeton d'authentification du device

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>

// Log horodaté dans le moniteur série : [  12345 ms] message
#define LOG(fmt, ...) Serial.printf("[%8lu ms] " fmt "\n", millis(), ##__VA_ARGS__)

// =========================================================================
// 1. BROCHAGE
// =========================================================================
// Moteur pas-à-pas (TB6600) : DIR est câblé directement sur Vcc (un seul sens)
const uint8_t PIN_PUL = 5;
const uint8_t PIN_ENA = 2;               // GPIO2 = broche de strapping
const uint8_t ENA_ACTIF = LOW;           // niveau qui ACTIVE le driver (dépend du câblage ENA+/ENA-)

const uint8_t PIN_SERVO = 23;

const uint8_t PIN_CAPTEUR_DEBUT = 33;    // ADC1
const uint8_t PIN_CAPTEUR_FIN   = 32;    // ADC1

const uint8_t PIN_LED_VERTE = 27;        // tapis en marche
const uint8_t PIN_LED_JAUNE = 26;        // tapis à l'arrêt / pause / erreur
const uint8_t PIN_LED_ROUGE = 14;

// LED qui clignote pour signaler un cube inconnu (X). Mettre PIN_LED_ROUGE pour utiliser la rouge.
const uint8_t PIN_LED_ALERTE = PIN_LED_ROUGE;

const uint8_t PIN_PAUSE = 12;            // interrupteur à bascule vers GND (INPUT_PULLUP) : sert UNIQUEMENT à REPRENDRE
                                         // après un arrêt d'urgence (tout changement d'état = reprise)

const uint8_t LCD_ADDR = 0x27;
const uint8_t PIN_SDA  = 21;
const uint8_t PIN_SCL  = 22;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

const uint8_t PIN_UART_RX = 16;          // <- TX du XIAO
const uint8_t PIN_UART_TX = 17;          // non utilisé pour l'instant

// =========================================================================
// 2. PARAMÈTRES
// =========================================================================
// Moteur
const float STEPS_PER_REV_MOTOR = 200.0;
const float MICROSTEP           = 1.0;   // TB6600 réglé sur 1 micropas
float rpm                       = 24;    // vitesse du tapis (modifiable via setRpm)

// Géométrie (à mesurer). Les cubes doivent être espacés d'au moins DISTANCE_FIN_BAC_CM.
const float PERIMETRE_ROULEAU_CM = 10.0;
const float DISTANCE_FIN_BAC_CM  = 5.0;

// Servo du bac : 3 couleurs triées pour l'instant (servo 180°)
const int ANGLE_J = 0;
const int ANGLE_B = 90;
const int ANGLE_V = 180;
const int ANGLE_JEPOS = 90;              // position au démarrage / après urgence

// Capteurs analogiques à faisceau laser.
// true  = le faisceau est reçu au repos (valeur haute ~4095), un cube qui le coupe fait chuter la valeur (~76)
// false = la valeur monte quand un cube est présent
const bool CAPTEUR_DEBUT_ACTIF_BAS = true;
const bool CAPTEUR_FIN_ACTIF_BAS   = true;
const int SEUIL_DETECTION = 2000;
const unsigned long ANTIREBOND_DEBUT_MS = 600;
const unsigned long ANTIREBOND_FIN_MS   = 300;

// Watchdog
const bool WATCHDOG_ACTIF = true;
const unsigned long TIMEOUT_INACTIVITE_MS = 120000;

// Démarrage différé du tapis
const unsigned long DELAI_DEMARRAGE_MS = 300;

// Interrupteur à bascule (reprise) + arrêt d'urgence
const unsigned long DEBOUNCE_PAUSE_MS = 50;
const char CODE_ARRET = 'A';                       // reçu du XIAO (bouton poussoir) ou tapé au clavier
const unsigned long URGENCE_DEMI_PERIODE_MS = 250; // clignotement de la LED rouge pendant l'arrêt

// LED d'alerte
const unsigned long ALERTE_DEMI_PERIODE_MS = 200;
const int ALERTE_NB_CLIGNOTEMENTS = 3;   // à la réception d'un 'X'

// Erreur cube inconnu : reprise dès que le capteur de fin est libre.
const int ECHANTILLONS_LIBRE = 5;

// LCD : réécriture complète périodique pour effacer d'éventuels caractères parasites (0 = désactivé)
const unsigned long LCD_REFRESH_MS = 5000;

// Debug
const bool SIMULATION_CLAVIER = true;             // taper J/V/B/X dans le moniteur série
const bool DEBUG_UART_BRUT = true;                // affiche chaque octet reçu de la caméra
const unsigned long HEARTBEAT_MS = 5000;          // ligne d'état périodique (0 = désactivé)

// ---- Wi-Fi / Blynk ------------------------------------------------------
const char* WIFI_SSID = "Infinix HOT 50 Pro+";
const char* WIFI_PASS = "doriqueleee";
const char* BLYNK_SERVEUR = "ny3.blynk.cloud";    // région NY3 (vue dans l'URL du dashboard)

const char* CODE_RESET = "EPAC";                  // mot de passe pour remettre les compteurs à zéro

// Fuseau horaire pour l'horodatage (Bénin = UTC+1, pas d'heure d'été)
const long GMT_OFFSET_SEC = 3600;

// Ordre des éléments dans la liste déroulante "Bac Sélection" (V15) du dashboard.
// Index 0 = premier élément de la liste. ADAPTE si ton ordre est différent.
const char MENU_BAC[3] = { 'J', 'B', 'V' };       // 0 = YELLOW, 1 = BLUE, 2 = GREEN
const int  MENU_DECALAGE = 0;                     // mettre 1 si ton menu Blynk renvoie 1,2,3 au lieu de 0,1,2

// Cadence max d'envoi vers Blynk (la limite du cloud est ~10 messages/s)
const unsigned long BLYNK_INTERVALLE_MS = 110;

// =========================================================================
// 3. ÉTAT DU SYSTÈME
// =========================================================================
bool tapisActif = false;
bool attenteDemarrageTapis = false;
unsigned long tempsDemarrageTapis = 0;

bool enPause = false;
unsigned long debutPause = 0;

bool erreurCube = false;                 // cube inconnu bloqué sur le capteur de fin
unsigned long debutErreur = 0;
int posCubeErreur = 0;                   // position du cube fautif dans la file
int erreurLecturesLibres = 0;

struct Cube {
  char couleur;                 // 'J','V','B','X' ou '?'
  bool arriveFin;
  bool binOriente;
  unsigned long chronoEntree;
  unsigned long chronoChute;    // jamais modifié après la détection en fin
};

const int MAX_CUBES_SUR_TAPIS = 10;
Cube fileCubes[MAX_CUBES_SUR_TAPIS];
int debutFile = 0, finFile = 0, nbCubesSurTapis = 0;

unsigned long dernierTempsActivite = 0;
const char* derniereSourceActivite = "demarrage";
unsigned long dernierHeartbeat = 0;
unsigned long nbOctetsUart = 0;          // total d'octets reçus de la caméra

// Capteurs : détection par front (cube qui arrive) + antirebond
struct Capteur {
  uint8_t pin;
  unsigned long antirebondMs;
  unsigned long dernierMs;
  bool precedent;               // cube présent à la lecture précédente
  int valeur;                   // dernière valeur analogique lue
  bool actifBas;                // true : cube présent = valeur basse
};
Capteur capDebut = { PIN_CAPTEUR_DEBUT, ANTIREBOND_DEBUT_MS, 0, false, 0, CAPTEUR_DEBUT_ACTIF_BAS };
Capteur capFin   = { PIN_CAPTEUR_FIN,   ANTIREBOND_FIN_MS,   0, false, 0, CAPTEUR_FIN_ACTIF_BAS };

// Interrupteur à bascule (anti-rebond) : on mémorise le NIVEAU (true = HIGH, false = LOW)
bool basculeBrute = true, basculeStable = true;
unsigned long basculeDernierChangement = 0;

// LED d'alerte (clignotement non bloquant)
int alerteDemiCyclesRestants = 0;
bool alerteEtat = false;
unsigned long alerteDernierChangement = 0;

// LCD : dernier message affiché (pour pouvoir le réécrire)
char lcdLigne1[17] = "";
char lcdLigne2[17] = "";
unsigned long dernierRafraichissementLcd = 0;

// Compteurs de tri (sauvegardés en mémoire flash : survivent à un redémarrage)
Preferences prefs;
uint32_t cntJ = 0, cntV = 0, cntB = 0, cntX = 0;
char dernierTri[16] = "-";
char bacCouleur = 'B';                   // dernier bac orienté (ANGLE_JEPOS = 90° = bleu)

// =========================================================================
// 3bis. BLYNK : tout tourne dans une tâche séparée (cœur 0)
//       -> un Wi-Fi/Cloud lent ne bloque JAMAIS le tapis ni les capteurs.
//       La boucle principale envoie des messages via une file (queue) et lit
//       des commandes via des drapeaux "volatile".
// =========================================================================
struct Msg {
  uint8_t pin;
  uint8_t type;                 // 0 = entier, 1 = texte
  int32_t iv;
  char txt[100];
};
QueueHandle_t fileBlynk = nullptr;

// Commandes reçues du dashboard (écrites par la tâche Blynk, lues par loop)
volatile int8_t cmdPlayPause = 0;        // 0 = rien, 1 = PLAY (reprise), 2 = PAUSE
volatile int   cmdBacIndex   = -1;       // -1 = rien, sinon index de la liste déroulante
volatile bool  resetArme     = false;    // interrupteur "Reset DB" (V13)
volatile bool  codeRecu      = false;    // un code a été saisi dans V16
char           codeTexte[24] = "";
volatile bool  resyncDemande = false;    // (re)connexion Blynk : renvoyer tout l'état
volatile bool  blynkEnLigne  = false;
bool forcerEtat = true;                  // forcer le renvoi des LED/Play-Pause

void pushInt(uint8_t pin, int32_t v) {
  if (!fileBlynk) return;
  Msg m; m.pin = pin; m.type = 0; m.iv = v; m.txt[0] = 0;
  xQueueSend(fileBlynk, &m, 0);          // 0 = ne bloque jamais (message perdu si file pleine)
}

void pushTxt(uint8_t pin, const char* s) {
  if (!fileBlynk) return;
  Msg m; m.pin = pin; m.type = 1; m.iv = 0;
  snprintf(m.txt, sizeof(m.txt), "%s", s);
  xQueueSend(fileBlynk, &m, 0);
}

// Heure réelle (NTP) si disponible, sinon temps depuis le démarrage
void horodatage(char* buf, size_t n) {
  struct tm t;
  if (getLocalTime(&t, 0)) {
    strftime(buf, n, "%H:%M:%S", &t);
  } else {
    unsigned long s = millis() / 1000;
    snprintf(buf, n, "+%02lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
  }
}

// Une ligne dans le widget Terminal "Historique" (V12) : [HH:MM:SS] message
void termLog(const char* fmt, ...) {
  char ts[16];
  horodatage(ts, sizeof(ts));
  char corps[80];
  va_list args;
  va_start(args, fmt);
  vsnprintf(corps, sizeof(corps), fmt, args);
  va_end(args);
  char ligne[100];
  snprintf(ligne, sizeof(ligne), "[%s] %s\n", ts, corps);
  pushTxt(V12, ligne);
}

// --- Handlers : appelés par la tâche Blynk quand le dashboard change ----
BLYNK_CONNECTED() {
  resyncDemande = true;
}

BLYNK_WRITE(V10) {                       // Play / Pause
  cmdPlayPause = param.asInt() ? 1 : 2;
}

BLYNK_WRITE(V12) {                       // Terminal : l'entrée est IGNORÉE
}

BLYNK_WRITE(V13) {                       // Reset DB : "armer" (ON) ou annuler (OFF)
  resetArme = param.asInt() != 0;
}

BLYNK_WRITE(V15) {                       // Bac Sélection
  cmdBacIndex = param.asInt() - MENU_DECALAGE;
}

BLYNK_WRITE(V16) {                       // Code de confirmation (Entrée)
  snprintf(codeTexte, sizeof(codeTexte), "%s", param.asStr());
  codeRecu = true;                       // écrit en dernier
}

void blynkTask(void*) {
  Blynk.config(BLYNK_AUTH_TOKEN, BLYNK_SERVEUR);
  unsigned long dernierEnvoi = 0;
  Msg m;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      blynkEnLigne = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    Blynk.run();                         // peut bloquer pendant une reconnexion : sans importance ici
    blynkEnLigne = Blynk.connected();

    if (blynkEnLigne) {
      if (millis() - dernierEnvoi >= BLYNK_INTERVALLE_MS && xQueueReceive(fileBlynk, &m, 0) == pdTRUE) {
        if (m.type == 0) Blynk.virtualWrite(m.pin, (int)m.iv);
        else             Blynk.virtualWrite(m.pin, m.txt);
        dernierEnvoi = millis();
      }
    } else {
      xQueueReset(fileBlynk);            // hors ligne : on jette, l'état complet sera renvoyé à la reconnexion
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =========================================================================
// 4. TEMPS : toujours une lecture fraîche de millis()
// =========================================================================
unsigned long ecouleDepuis(unsigned long t) { return millis() - t; }

void signalerActivite(const char* source) {
  dernierTempsActivite = millis();
  derniereSourceActivite = source;
}

// =========================================================================
// 5. UTILITAIRES
// =========================================================================
unsigned long calculerDelaiChuteMs() {
  if (rpm <= 0) return 2000;
  float vitesseCmSec = (rpm / 60.0) * PERIMETRE_ROULEAU_CM;
  return (unsigned long)((DISTANCE_FIN_BAC_CM / vitesseCmSec) * 1000.0);
}

float frequencePasHz() { return rpm * STEPS_PER_REV_MOTOR * MICROSTEP / 60.0; }

void demarrerMoteur() {
  ledcWriteTone(PIN_PUL, (uint32_t)frequencePasHz());
  LOG("⚙️ [MOTEUR] ON (%.0f Hz, %.1f tr/min)", frequencePasHz(), rpm);
}
void arreterMoteur() {
  ledcWriteTone(PIN_PUL, 0);
  LOG("⚙️ [MOTEUR] OFF");
}

// Change la vitesse (éviter pendant un tri en cours)
void setRpm(float nouveauRpm) {
  rpm = nouveauRpm;
  if (tapisActif && !enPause && !erreurCube) demarrerMoteur();
}

void servoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  float pulseUs = 500.0 + (angle / 180.0) * 1900.0;         // 500..2400 µs
  uint32_t duty = (uint32_t)(pulseUs / 20000.0 * 16383.0);  // 14 bits @ 50 Hz
  ledcWrite(PIN_SERVO, duty);
}

const char* nomCouleur(char c) {
  switch (c) {
    case 'J': return "JAUNE";
    case 'V': return "VERT";
    case 'B': return "BLEU";
    default:  return "INCONNU";
  }
}

// Couleur non triable : X, '?' ou toute autre valeur
bool estInconnue(char c) { return c != 'J' && c != 'V' && c != 'B'; }

// Angle du servo pour une couleur (-1 si non triable)
int angleDe(char c) {
  switch (c) {
    case 'J': return ANGLE_J;
    case 'V': return ANGLE_V;
    case 'B': return ANGLE_B;
    default:  return -1;
  }
}

// Index de la liste déroulante Blynk pour une couleur
int indexMenu(char c) {
  for (int i = 0; i < 3; i++) if (MENU_BAC[i] == c) return i + MENU_DECALAGE;
  return MENU_DECALAGE;
}

// --- LCD ---------------------------------------------------------------
void ecrireLcd() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(lcdLigne1);
  lcd.setCursor(0, 1); lcd.print(lcdLigne2);
}

void afficherLcd(const char* ligne1, const char* ligne2 = "") {
  snprintf(lcdLigne1, sizeof(lcdLigne1), "%s", ligne1);
  snprintf(lcdLigne2, sizeof(lcdLigne2), "%s", ligne2);
  ecrireLcd();
}

// Réinitialise l'écran et réécrit le dernier message : efface les caractères parasites
void rafraichirLcd() {
  if (LCD_REFRESH_MS == 0 || ecouleDepuis(dernierRafraichissementLcd) < LCD_REFRESH_MS) return;
  lcd.init();
  lcd.backlight();
  ecrireLcd();
  dernierRafraichissementLcd = millis();
}

// Positionne le bac, met à jour le LCD ET la liste déroulante du dashboard (V15)
void orienterBac(char couleur) {
  int angle = angleDe(couleur);
  if (angle < 0) {
    LOG("⚠️ [BAC] Couleur '%c' non triable : bac non orienté.", couleur);
    return;
  }
  servoAngle(angle);
  bacCouleur = couleur;
  pushInt(V15, indexMenu(couleur));
  afficherLcd("BAC :", nomCouleur(couleur));
  LOG("🔀 [BAC] Orienté pour '%c' (%s) → servo à %d°", couleur, nomCouleur(couleur), angle);
}

void reinitialiserFile() { nbCubesSurTapis = 0; debutFile = 0; finFile = 0; }

// Retire le cube à la position pos (0 = tête) en décalant les suivants
void retirerCubeALaPosition(int pos) {
  if (pos < 0 || pos >= nbCubesSurTapis) return;
  for (int k = pos; k < nbCubesSurTapis - 1; k++) {
    fileCubes[(debutFile + k) % MAX_CUBES_SUR_TAPIS] = fileCubes[(debutFile + k + 1) % MAX_CUBES_SUR_TAPIS];
  }
  nbCubesSurTapis--;
  finFile = (finFile - 1 + MAX_CUBES_SUR_TAPIS) % MAX_CUBES_SUR_TAPIS;
}

// =========================================================================
// 5bis. COMPTEURS + DASHBOARD
// =========================================================================
uint32_t totalTri() { return cntJ + cntV + cntB + cntX; }

void sauverCompteurs() {
  prefs.putUInt("J", cntJ);
  prefs.putUInt("V", cntV);
  prefs.putUInt("B", cntB);
  prefs.putUInt("X", cntX);
}

// Envoie tous les compteurs au dashboard
void pousserCompteurs() {
  pushInt(V1, (int32_t)totalTri());
  pushInt(V5, (int32_t)cntJ);
  pushInt(V6, (int32_t)cntV);
  pushInt(V7, (int32_t)cntB);
  pushInt(V8, (int32_t)cntX);
}

// Un cube vient d'être trié (ou retiré s'il est inconnu)
void enregistrerTri(char couleur) {
  switch (couleur) {
    case 'J': cntJ++; break;
    case 'V': cntV++; break;
    case 'B': cntB++; break;
    default:  cntX++; break;
  }
  sauverCompteurs();
  snprintf(dernierTri, sizeof(dernierTri), "%s", nomCouleur(couleur));

  pushTxt(V0, dernierTri);
  pousserCompteurs();
  pushInt(V11, (int32_t)totalTri());     // graphique : le total au fil du temps
  termLog("%s trie | total : %lu", nomCouleur(couleur), (unsigned long)totalTri());
}

void remettreCompteursAZero() {
  cntJ = cntV = cntB = cntX = 0;
  sauverCompteurs();
  snprintf(dernierTri, sizeof(dernierTri), "-");
  pushTxt(V0, dernierTri);
  pousserCompteurs();
  pushInt(V11, 0);
  LOG("🗑️ [RESET] Compteurs remis à zéro depuis le dashboard.");
  termLog("Compteurs remis a zero");
}

// Après (re)connexion : on renvoie tout, le dashboard reflète la réalité du système
void envoyerEtatComplet() {
  pushTxt(V0, dernierTri);
  pousserCompteurs();
  pushInt(V13, 0);
  pushTxt(V16, "");
  pushInt(V15, indexMenu(bacCouleur));
  forcerEtat = true;
  termLog("ESP32 connecte");
  LOG("☁️ [BLYNK] Connecté : état complet renvoyé au dashboard.");
}

// LED virtuelles Marche/Pause/Alerte + bouton Play/Pause : envoyées seulement si elles changent
void synchroniserEtatBlynk() {
  static int8_t derniers[4] = { -1, -1, -1, -1 };
  if (forcerEtat) {
    for (auto &d : derniers) d = -1;
    forcerEtat = false;
  }
  bool tourne = tapisActif && !enPause && !erreurCube;
  int8_t v[4] = {
    (int8_t)(tourne ? 1 : 0),                      // V2 Marche  (= LED verte)
    (int8_t)(!tourne ? 1 : 0),                     // V3 Pause   (= LED jaune)
    (int8_t)((enPause || erreurCube) ? 1 : 0),     // V4 Alerte  (= LED rouge)
    (int8_t)(enPause ? 0 : 1)                      // V10 Play/Pause : ON = système actif
  };
  const uint8_t pins[4] = { V2, V3, V4, V10 };
  for (int i = 0; i < 4; i++) {
    if (v[i] != derniers[i]) {
      derniers[i] = v[i];
      pushInt(pins[i], (i < 3) ? (v[i] ? 255 : 0) : v[i]);
    }
  }
}

// =========================================================================
// 6. CAPTEURS
// =========================================================================
// Un cube est-il devant le capteur ? (d'après la dernière valeur lue)
bool cubePresent(const Capteur &c) {
  return c.actifBas ? (c.valeur < SEUIL_DETECTION) : (c.valeur > SEUIL_DETECTION);
}

// Retourne true une seule fois quand un cube ARRIVE devant le capteur
bool frontMontant(Capteur &c) {
  c.valeur = analogRead(c.pin);
  bool present = cubePresent(c);
  bool front = present && !c.precedent && ecouleDepuis(c.dernierMs) > c.antirebondMs;
  if (front) c.dernierMs = millis();
  c.precedent = present;
  return front;
}

// Évite une fausse détection au démarrage : on part de l'état réel des capteurs
void initialiserCapteurs() {
  capDebut.valeur = analogRead(capDebut.pin);
  capFin.valeur   = analogRead(capFin.pin);
  capDebut.precedent = cubePresent(capDebut);
  capFin.precedent   = cubePresent(capFin);
  LOG("📍 [CAPTEURS] Au démarrage : DEBUT = %d (%s) | FIN = %d (%s)",
      capDebut.valeur, cubePresent(capDebut) ? "CUBE" : "libre",
      capFin.valeur,   cubePresent(capFin)   ? "CUBE" : "libre");
}

// =========================================================================
// 7. LED
// =========================================================================
void declencherAlerteX() {
  alerteDemiCyclesRestants = ALERTE_NB_CLIGNOTEMENTS * 2;
  alerteEtat = false;
  alerteDernierChangement = millis() - ALERTE_DEMI_PERIODE_MS;  // premier changement immédiat
}

// Phase de clignotement : 3 clignotements après un 'X' reçu ; continu pendant une erreur
void majAlerte() {
  bool clignote = erreurCube || alerteDemiCyclesRestants > 0;
  if (!clignote) { alerteEtat = false; return; }
  if (ecouleDepuis(alerteDernierChangement) >= ALERTE_DEMI_PERIODE_MS) {
    alerteEtat = !alerteEtat;
    alerteDernierChangement = millis();
    if (!erreurCube && alerteDemiCyclesRestants > 0) alerteDemiCyclesRestants--;
  }
}

void majLeds() {
  majAlerte();
  bool tourne = tapisActif && !enPause && !erreurCube;
  // Pendant l'arrêt d'urgence, seule la LED rouge clignote (la jaune reste fixe)
  bool clignote = !enPause && (erreurCube || alerteDemiCyclesRestants > 0 || alerteEtat);
  bool rougeUrgence = enPause && ((millis() / URGENCE_DEMI_PERIODE_MS) % 2 == 0);

  digitalWrite(PIN_LED_VERTE, tourne);
  if (PIN_LED_ALERTE == PIN_LED_JAUNE) {
    digitalWrite(PIN_LED_JAUNE, clignote ? alerteEtat : !tourne);
    digitalWrite(PIN_LED_ROUGE, rougeUrgence);
  } else {
    digitalWrite(PIN_LED_JAUNE, !tourne);
    digitalWrite(PIN_LED_ROUGE, rougeUrgence || (clignote && alerteEtat));
  }
  synchroniserEtatBlynk();
}

// =========================================================================
// 8. ARRÊT D'URGENCE / REPRISE
// =========================================================================
// Déclenché par 'A' (bouton poussoir du XIAO) ou par le bouton Play/Pause du dashboard.
void entrerPause(const char* source) {
  if (enPause) return;
  enPause = true;
  debutPause = millis();
  arreterMoteur();
  afficherLcd("ARRET URGENCE", "Basculer inter.");
  LOG("⏸️ [URGENCE] Arrêt demandé (%s) : système en pause. Bascule l'interrupteur (ou Play) pour reprendre.", source);
  termLog("ARRET D'URGENCE (%s)", source);
}

void sortirPause() {
  unsigned long duree = ecouleDepuis(debutPause);

  // On gèle tous les chronos : on les décale de la durée de la pause
  for (int i = 0; i < nbCubesSurTapis; i++) {
    int idx = (debutFile + i) % MAX_CUBES_SUR_TAPIS;
    fileCubes[idx].chronoEntree += duree;
    if (fileCubes[idx].arriveFin) fileCubes[idx].chronoChute += duree;
  }
  tempsDemarrageTapis += duree;
  debutErreur += duree;
  erreurLecturesLibres = 0;

  enPause = false;
  if (tapisActif && !erreurCube) demarrerMoteur();
  signalerActivite("reprise");
  afficherLcd(erreurCube ? "ERREUR CUBE" : "REPRISE", erreurCube ? "Retirer le cube" : "");
  LOG("▶️ [PAUSE] Reprise après %lu ms.", duree);
  termLog("Reprise apres %lu ms", duree);
}

// Suit en permanence l'état (anti-rebond) de l'interrupteur à bascule.
// S'il CHANGE d'état pendant un arrêt d'urgence, le système reprend.
void gererInterrupteurBascule() {
  bool lecture = digitalRead(PIN_PAUSE);
  if (lecture != basculeBrute) {
    basculeBrute = lecture;
    basculeDernierChangement = millis();
  }
  if (ecouleDepuis(basculeDernierChangement) >= DEBOUNCE_PAUSE_MS && basculeStable != basculeBrute) {
    basculeStable = basculeBrute;
    LOG("🔘 [BASCULE] Interrupteur passé à %s.", basculeStable ? "HIGH" : "LOW");
    if (enPause) sortirPause();
  }
}

// =========================================================================
// 8bis. COMMANDES VENANT DU DASHBOARD BLYNK
// =========================================================================
void traiterCommandesBlynk() {
  // (Re)connexion : renvoyer tout l'état
  if (resyncDemande) {
    resyncDemande = false;
    envoyerEtatComplet();
  }

  // Bouton Play / Pause (V10)
  int8_t pp = cmdPlayPause;
  if (pp) {
    cmdPlayPause = 0;
    if (pp == 2) entrerPause("DASHBOARD");
    else if (enPause) sortirPause();
    else forcerEtat = true;               // déjà en marche : on remet juste le bouton à ON
  }

  // Liste déroulante Bac Sélection (V15) -> le servo suit
  int bi = cmdBacIndex;
  if (bi >= 0) {
    cmdBacIndex = -1;
    if (bi < 3 && !enPause) {
      char c = MENU_BAC[bi];
      servoAngle(angleDe(c));
      bacCouleur = c;
      afficherLcd("BAC (manuel) :", nomCouleur(c));
      LOG("🎛️ [BAC] Sélection dashboard : '%c' (%s) → servo à %d°", c, nomCouleur(c), angleDe(c));
      termLog("Bac manuel : %s", nomCouleur(c));
    } else {
      pushInt(V15, indexMenu(bacCouleur));  // refus (pause) : on remet la liste sur la vraie position
    }
  }

  // Code de confirmation (V16) : n'a d'effet que si "Reset DB" (V13) est sur ON
  if (codeRecu) {
    codeRecu = false;
    char code[24];
    snprintf(code, sizeof(code), "%s", codeTexte);
    // retire espaces/retours à la ligne autour du texte
    char *p = code;
    while (*p == ' ') p++;
    for (int n = strlen(p); n > 0 && (p[n - 1] == ' ' || p[n - 1] == '\r' || p[n - 1] == '\n'); n--) p[n - 1] = 0;

    if (resetArme) {
      if (strcasecmp(p, CODE_RESET) == 0) {
        remettreCompteursAZero();
      } else {
        LOG("⛔ [RESET] Code incorrect.");
        termLog("Code incorrect : reset annule");
      }
      resetArme = false;
      pushInt(V13, 0);                    // le bouton Reset repasse automatiquement à OFF
    }
    pushTxt(V16, "");                     // on vide la case de saisie dans tous les cas
  }
}

// =========================================================================
// 9. SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_ENA, ENA_ACTIF);

  // PWM matériel : canaux fixés à la main (0 et 1 partagent le même timer sur ESP32)
  ledcAttachChannel(PIN_PUL, 1000, 8, 0);
  arreterMoteur();
  ledcAttachChannel(PIN_SERVO, 50, 14, 2);
  servoAngle(ANGLE_JEPOS);

  pinMode(PIN_CAPTEUR_DEBUT, INPUT);
  pinMode(PIN_CAPTEUR_FIN, INPUT);
  pinMode(PIN_PAUSE, INPUT_PULLUP);
  delay(5);
  basculeBrute = basculeStable = digitalRead(PIN_PAUSE);   // on part de la position réelle de l'interrupteur
  pinMode(PIN_LED_VERTE, OUTPUT);
  pinMode(PIN_LED_JAUNE, OUTPUT);
  pinMode(PIN_LED_ROUGE, OUTPUT);
  digitalWrite(PIN_LED_ROUGE, LOW);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);        // I2C à 100 kHz : plus tolérant au bruit
  Wire.setTimeOut(100);         // évite qu'un blocage I2C fige le programme
  lcd.init();
  lcd.backlight();
  afficherLcd("Systeme pret", "En attente...");
  dernierRafraichissementLcd = millis();

  initialiserCapteurs();

  // Compteurs sauvegardés
  prefs.begin("tri", false);
  cntJ = prefs.getUInt("J", 0);
  cntV = prefs.getUInt("V", 0);
  cntB = prefs.getUInt("B", 0);
  cntX = prefs.getUInt("X", 0);
  LOG("💾 [COMPTEURS] J=%lu V=%lu B=%lu X=%lu (total %lu)",
      (unsigned long)cntJ, (unsigned long)cntV, (unsigned long)cntB, (unsigned long)cntX, (unsigned long)totalTri());

  // Wi-Fi (non bloquant) + heure NTP + tâche Blynk sur le cœur 0
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  configTime(GMT_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");
  fileBlynk = xQueueCreate(48, sizeof(Msg));
  xTaskCreatePinnedToCore(blynkTask, "blynk", 8192, nullptr, 1, nullptr, 0);

  signalerActivite("demarrage");
  majLeds();
  LOG("✔️ [SYSTEME] Prêt | %.1f tr/min | %.0f Hz | délai chute %lu ms | watchdog %s (%lu s)",
      rpm, frequencePasHz(), calculerDelaiChuteMs(),
      WATCHDOG_ACTIF ? "ON" : "OFF", TIMEOUT_INACTIVITE_MS / 1000);
  LOG("✔️ [SYSTEME] Servo : J=%d° B=%d° V=%d° | seuil capteurs = %d", ANGLE_J, ANGLE_B, ANGLE_V, SEUIL_DETECTION);
  LOG("✔️ [SYSTEME] UART caméra : RX=GPIO%d à 9600 bauds, 8N1.", PIN_UART_RX);
  LOG("✔️ [SYSTEME] Wi-Fi '%s' en cours de connexion (non bloquant)...", WIFI_SSID);
  if (SIMULATION_CLAVIER) LOG("⌨️ [SYSTEME] Simulation caméra : tape J, V, B ou X dans le moniteur série.");
}

// =========================================================================
// 10. LOGIQUE
// =========================================================================
void ajouterNouveauCube() {
  if (nbCubesSurTapis >= MAX_CUBES_SUR_TAPIS) {
    LOG("⚠️ [DEBUT] File pleine, cube ignoré.");
    return;
  }
  fileCubes[finFile] = { '?', false, false, millis(), 0 };
  finFile = (finFile + 1) % MAX_CUBES_SUR_TAPIS;
  nbCubesSurTapis++;
  LOG("📥 [DEBUT] Nouveau cube ajouté à la file (%d sur le tapis)", nbCubesSurTapis);

  char ligne2[17];
  snprintf(ligne2, sizeof(ligne2), "Sur tapis : %d", nbCubesSurTapis);
  afficherLcd("Nouveau cube", ligne2);

  if (!tapisActif && !attenteDemarrageTapis) {
    attenteDemarrageTapis = true;
    tempsDemarrageTapis = millis();
    LOG("⏱️ [TAPIS] Démarrage dans %lu ms.", DELAI_DEMARRAGE_MS);
  }
}

// Une seule fonction pour la couleur, qu'elle vienne du XIAO ou du clavier
void recevoirCouleur(char code, const char* source) {
  code = toupper(code);
  if (code != 'J' && code != 'V' && code != 'B' && code != 'X') return; // ignore \r \n etc.
  signalerActivite(source);

  for (int i = 0; i < nbCubesSurTapis; i++) {
    int idx = (debutFile + i) % MAX_CUBES_SUR_TAPIS;
    if (fileCubes[idx].couleur == '?') {
      fileCubes[idx].couleur = code;
      LOG("📷 [%s] Couleur '%c' (%s) reçue → attribuée au cube en position %d", source, code, nomCouleur(code), i);
      if (code == 'X') {
        LOG("🟡 [CAMERA] Couleur inconnue : le cube bloquera le tapis en fin de course.");
        declencherAlerteX();
      }
      return;
    }
  }
  LOG("📷 [%s] Couleur '%c' (%s) reçue mais aucun cube en attente : ignorée.", source, code, nomCouleur(code));
}

// Lit un octet de l'UART caméra et l'affiche tel quel (debug)
int lireOctetCamera(bool pris_en_compte) {
  int octet = Serial1.read();
  nbOctetsUart++;
  if (DEBUG_UART_BRUT) {
    if (octet >= 32 && octet < 127)
      LOG("🔌 [UART RX] octet 0x%02X '%c'%s", octet, (char)octet, pris_en_compte ? "" : " (ignoré : arrêt d'urgence)");
    else
      LOG("🔌 [UART RX] octet 0x%02X (non imprimable)%s", octet, pris_en_compte ? "" : " (ignoré : arrêt d'urgence)");
  }
  return octet;
}

// Aiguillage d'un octet reçu : 'A' = arrêt d'urgence, sinon couleur (J/V/B/X)
void recevoirOctet(char c, const char* source) {
  if (toupper(c) == CODE_ARRET) { entrerPause(source); return; }
  recevoirCouleur(c, source);
}

void traiteruartCamera() {
  while (Serial1.available() > 0) recevoirOctet((char)lireOctetCamera(true), "UART");
  if (SIMULATION_CLAVIER) {
    while (Serial.available() > 0) recevoirOctet((char)Serial.read(), "CLAVIER");
  }
}

// Cube inconnu en bout de tapis : le moteur s'arrête immédiatement, pile sur le capteur de fin
void declencherErreurCube(int pos) {
  erreurCube = true;
  posCubeErreur = pos;
  debutErreur = millis();
  erreurLecturesLibres = 0;
  arreterMoteur();
  afficherLcd("ERREUR CUBE", "Retirer le cube");
  LOG("🚨 [ERREUR] Cube de couleur inconnue sur le capteur de fin : moteur arrêté, retire-le à la main.");
  termLog("ERREUR : cube inconnu, retirer");
}

// Après le tri / retrait du cube de tête : prépare le suivant
void passerAuCubeSuivant(unsigned long delaiChute) {
  if (nbCubesSurTapis == 0) {
    tapisActif = false;
    arreterMoteur();
    LOG("🛑 [TAPIS] File vide. Arrêt normal du moteur.");
    return;
  }
  Cube &suivant = fileCubes[debutFile];
  if (suivant.arriveFin && !suivant.binOriente) {
    suivant.binOriente = true;
    orienterBac(suivant.couleur);
    unsigned long ecoule = ecouleDepuis(suivant.chronoChute);
    if (ecoule >= delaiChute)
      LOG("⚠️ [ALERTE] Cubes trop rapprochés : le suivant est déjà tombé avant l'orientation du bac !");
    else
      LOG("ℹ️ [BAC] Orienté avec %lu ms de marge avant la chute du suivant.", delaiChute - ecoule);
  }
}

void traiterFinDeTapis() {
  for (int i = 0; i < nbCubesSurTapis; i++) {
    int idx = (debutFile + i) % MAX_CUBES_SUR_TAPIS;
    if (!fileCubes[idx].arriveFin) {
      fileCubes[idx].arriveFin = true;
      fileCubes[idx].chronoChute = millis();
      LOG("🏁 [FIN] Cube '%c' (%s, position %d) arrive au bout du tapis.",
          fileCubes[idx].couleur, nomCouleur(fileCubes[idx].couleur), i);

      if (estInconnue(fileCubes[idx].couleur)) {
        // Arrêt immédiat, quelle que soit la position dans la file
        if (i > 0) LOG("⚠️ [ALERTE] Un cube précédent est encore en chute : l'espacement entre cubes est < %.0f cm.", DISTANCE_FIN_BAC_CM);
        fileCubes[idx].binOriente = true;
        declencherErreurCube(i);
      } else if (i == 0) {
        fileCubes[idx].binOriente = true;
        orienterBac(fileCubes[idx].couleur);
      } else {
        LOG("⏳ [BAC] Orientation reportée : cube précédent encore en chute.");
      }
      return;
    }
  }
  LOG("ℹ️ [FIN] Détection en fin de tapis mais aucun cube en attente : ignorée.");
}

// Attend que le cube bloqué soit retiré à la main : reprise dès que le capteur de fin est libre
void gererErreurCube() {
  if (cubePresent(capFin)) erreurLecturesLibres = 0;
  else erreurLecturesLibres++;
  if (erreurLecturesLibres < ECHANTILLONS_LIBRE) return;

  unsigned long duree = ecouleDepuis(debutErreur);
  LOG("✅ [ERREUR] Cube retiré (capteur de fin libre, valeur = %d, après %lu ms d'arrêt). Reprise.", capFin.valeur, duree);
  erreurCube = false;

  enregistrerTri('X');                    // compté dans "Count UNKNOWN"

  // le cube fautif quitte la file ; les autres cubes ne doivent pas avoir "avancé" pendant l'arrêt
  retirerCubeALaPosition(posCubeErreur);
  for (int i = 0; i < nbCubesSurTapis; i++) {
    int idx = (debutFile + i) % MAX_CUBES_SUR_TAPIS;
    fileCubes[idx].chronoEntree += duree;
    if (fileCubes[idx].arriveFin) fileCubes[idx].chronoChute += duree;
  }

  signalerActivite("erreur resolue");
  afficherLcd("Cube retire", "Reprise");

  passerAuCubeSuivant(calculerDelaiChuteMs());
  if (nbCubesSurTapis > 0) demarrerMoteur();
}

void afficherHeartbeat() {
  if (HEARTBEAT_MS == 0 || ecouleDepuis(dernierHeartbeat) < HEARTBEAT_MS) return;
  dernierHeartbeat = millis();
  const char* etat = enPause ? "PAUSE" : erreurCube ? "ERREUR" : tapisActif ? "MARCHE" : "ARRET";
  LOG("💓 [ETAT] %s | cubes : %d | DEBUT=%d (%s) FIN=%d (%s) | UART : %lu octets reçus | %.1f tr/min | Blynk : %s",
      etat, nbCubesSurTapis,
      capDebut.valeur, cubePresent(capDebut) ? "CUBE" : "libre",
      capFin.valeur,   cubePresent(capFin)   ? "CUBE" : "libre",
      nbOctetsUart, rpm, blynkEnLigne ? "EN LIGNE" : "hors ligne");
}

// =========================================================================
// 11. LOOP
// =========================================================================
void loop() {
  gererInterrupteurBascule();
  traiterCommandesBlynk();
  afficherHeartbeat();
  rafraichirLcd();

  // --- EN PAUSE : tout est ignoré (on vide les buffers et on garde les fronts à jour)
  if (enPause) {
    while (Serial1.available() > 0) lireOctetCamera(false);
    if (SIMULATION_CLAVIER) while (Serial.available() > 0) Serial.read();
    frontMontant(capDebut);
    frontMontant(capFin);
    majLeds();
    return;
  }

  // 1. Lecture unique des capteurs ; toute détection est un signe de vie
  bool detectionDebut = frontMontant(capDebut);
  bool detectionFin   = frontMontant(capFin);
  if (detectionDebut) {
    LOG("📍 [CAPTEUR DEBUT] Cube détecté (valeur = %d)", capDebut.valeur);
    signalerActivite("capteur DEBUT");
    ajouterNouveauCube();
  }
  if (detectionFin) {
    LOG("📍 [CAPTEUR FIN] Cube détecté (valeur = %d)", capFin.valeur);
    signalerActivite("capteur FIN");
  }

  // 2. Couleur (UART du XIAO ou clavier) ou arrêt d'urgence 'A'
  traiteruartCamera();

  // 'A' vient d'être reçu : on coupe tout de suite (pas de démarrage différé, etc.)
  if (enPause) {
    majLeds();
    return;
  }

  // 2bis. Erreur cube inconnu : tapis arrêté, on attend le retrait manuel
  if (erreurCube) {
    gererErreurCube();
    majLeds();
    return;
  }

  // 3. Watchdog d'inactivité
  if (WATCHDOG_ACTIF && tapisActif && ecouleDepuis(dernierTempsActivite) >= TIMEOUT_INACTIVITE_MS) {
    LOG("🚨 [URGENCE] %lu ms sans activité (dernière : %s). Arrêt et réinitialisation !",
        ecouleDepuis(dernierTempsActivite), derniereSourceActivite);
    termLog("Watchdog : inactivite, reinitialisation");
    tapisActif = false;
    attenteDemarrageTapis = false;
    arreterMoteur();
    servoAngle(ANGLE_JEPOS);
    bacCouleur = 'B';
    pushInt(V15, indexMenu(bacCouleur));
    reinitialiserFile();
    char msg[17];
    snprintf(msg, sizeof(msg), "Inactivite %lus", TIMEOUT_INACTIVITE_MS / 1000);
    afficherLcd("URGENCE", msg);
    signalerActivite("reinitialisation");
  }

  // 4. Démarrage différé du tapis
  if (attenteDemarrageTapis && ecouleDepuis(tempsDemarrageTapis) >= DELAI_DEMARRAGE_MS) {
    tapisActif = true;
    attenteDemarrageTapis = false;
    demarrerMoteur();
    signalerActivite("demarrage tapis");
    LOG("🚀 [TAPIS] Démarrage du moteur.");
  }

  // 5. Capteur de fin de tapis
  if (detectionFin) traiterFinDeTapis();

  // 6. Fin de chute du cube de tête (pas pendant une erreur)
  if (!erreurCube && nbCubesSurTapis > 0) {
    Cube &tete = fileCubes[debutFile];
    unsigned long delaiChute = calculerDelaiChuteMs();

    if (tete.arriveFin && ecouleDepuis(tete.chronoChute) >= delaiChute) {
      signalerActivite("cube trie");
      LOG("✅ [TRI] OBJET TRIÉ | couleur '%c' (%s) | délai : %lu ms | transit total : %lu ms",
          tete.couleur, nomCouleur(tete.couleur), delaiChute, ecouleDepuis(tete.chronoEntree));
      afficherLcd("OBJET TRIE", nomCouleur(tete.couleur));

      enregistrerTri(tete.couleur);       // compteurs + Dernier Tri + Historique + Graphique

      debutFile = (debutFile + 1) % MAX_CUBES_SUR_TAPIS;
      nbCubesSurTapis--;

      passerAuCubeSuivant(delaiChute);
    }
  }

  // 7. LED
  majLeds();
}
