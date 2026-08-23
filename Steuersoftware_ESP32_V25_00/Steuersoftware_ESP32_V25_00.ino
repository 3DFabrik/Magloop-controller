/*
 ------------------------------------------------------------
 Projekt: Variablenkondensator-Steuerung V25.00
 ------------------------------------------------------------

 FUNKTIONSBESCHREIBUNG:
 Steuersoftware für einen Schrittmotor, der einen variablen Kondensator (VC)
 über einen ESP32 WROOM antreibt. Die Steuerung erfolgt über ein 4x3 Tastenfeld
 und einem 128x64 OLED-Display (SSD1306) sowie über eine Web-Schnittstelle.
 - Frequenzbasierte Positionierung mittels Kalibrierungstabelle (EEPROM).
 - Homing: schnell zum Endschalter, langsam ein Stück darüber, langsam
   hoch bis der Schalter öffnet = Position 0.
 - Backlash-Kompensation bei Richtungswechsel (Wegfahren vom Nullpunkt).
 - Web-Schnittstelle zur Steuerung und voll-manuellen Kalibrierung über WLAN.
 - Kalibrierungspunkte werden in 50kHz Schritten innerhalb der Ham-Bänder
   und in 500kHz Schritten dazwischen generiert.

 PINOUT ESP32 WROOM:
 ------------------------------------------------------------
 TASTENFELD (KEYPAD 4x3):
 - R1: GPIO 25
 - R2: GPIO 26
 - R3: GPIO 33
 - R4: GPIO 32
 - C1: GPIO 13
 - C2: GPIO 12
 - C3: GPIO 14

 MOTOR (Stepper Driver):
 - A (IN1/Vio. Pin): GPIO 17
 - B (IN2/Vio. Pin): GPIO 5
 - C (IN3/Vio. Pin): GPIO 18
 - D (IN4/Vio. Pin): GPIO 19

 OLED DISPLAY (I2C):
 - SDA: GPIO 21
 - SCL: GPIO 22
 ------------------------------------------------------------
*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BluetoothSerial.h>
#include <EEPROM.h>
#include "esp_coexist.h"
#include "esp_wifi.h"
#include <Fonts/FreeSansBold12pt7b.h>
#include <Keypad.h>
#include <WebServer.h>
#include <WebSocketsServer.h> // WebSocket library - https://github.com/Links2004/arduinoWebSockets
#include <WiFi.h>
#include <Wire.h>
#include <algorithm>
#include <ctype.h>
#include <math.h>
#include <set> // Required for the new calibration logic
#include <vector>


// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Serial and I2C configuration
#define SERIAL_BAUDRATE 115200
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Endstop configuration
#define ENDSTOP_PIN 4 // GPIO4 für Endstop
volatile bool endstopTriggered = false;
void IRAM_ATTR endstopISR() {
  endstopTriggered = true; // nur Flag setzen
}

// Keypad configuration
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}};
byte rowPins[ROWS] = {25, 26, 33, 32};
byte colPins[COLS] = {13, 12, 14};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Motor configuration
const int motorPins[] = {17, 5, 18, 19};
// Step sequence
int stepSequence[8][4] = {{0, 1, 1, 0}, {1, 1, 1, 0}, {1, 1, 0, 0},
                          {1, 1, 0, 1}, {1, 0, 0, 1}, {1, 0, 1, 1},
                          {0, 0, 1, 1}, {0, 1, 1, 1}};

// Motor state variables
const long gesamtSchritte = 11640;
// Must be at least the real gear lash, otherwise a reversal is swallowed whole
// and the capacitor never moves. Overshooting it cancels out exactly, so this
// is deliberately generous.
const int BACKLASH_STEPS = 60;
const int HOMING_BACKOFF_STEPS = 200;
const int HOMING_CLEAR_EXTRA = 20;
const int HOMING_CLEAR_MAX_STEPS = 200;
const int HOMING_SEEK_MAX_STEPS = 200;
const uint8_t HOMING_PRESS_DEBOUNCE = 8;
const uint8_t HOMING_OPEN_DEBOUNCE = 3;
const uint8_t HOMING_ZERO_DEBOUNCE = 3;
const long RTEST_HIGH_POS = 4000;
const long RTEST_LOW_POS = 400;
const int RTEST_DEFAULT_CYCLES = 5;
const int RTEST_MAX_CYCLES = 20;
const unsigned long RTEST_MOVE_TIMEOUT_MS = 90000;

// Motor speed constants
const int SPEED_FAST = 2;   // Fast speed for initial homing
const int SPEED_NORMAL = 3; // Normal speed for regular movement
const int SPEED_SLOW = 16;  // Slow speed for precise homing
const int SPEED_CAL = 32;   // Slow cal fine-search / dip approach

// Started from standstill at full rate the rotor lags the field and a short
// move ends before it catches up. Ease into the target rate instead.
const int RAMP_START_MS = 16;
const int RAMP_STEPS = 40;
// Hold the last phase briefly so the rotor pulls in before the field goes
// away, otherwise it drops back to the nearest detent and the step is lost.
const int MOTOR_SETTLE_MS = 80;

// Backlash compensation state variables
volatile bool motorJobActive = false;
volatile long motorFinalGoal = -1;
volatile int motorLegSpeed = SPEED_NORMAL;
volatile long motorRampLeft = 0;
// True after a completed move that finished approaching from below (UP).
volatile bool settledFromBelow = false;

// Simplified Motor State Machine
enum MotorState {
  MOTOR_IDLE,
  MOTOR_HOMING,
  MOTOR_HOMING_AT_ENDSTOP,
  MOTOR_MOVING
};

enum HomingPhase {
  HOMING_APPROACH,  // Down until switch closes
  HOMING_CLEAR,     // Up until open, then HOMING_CLEAR_EXTRA
  HOMING_SLOW_SEEK  // Slow down until close = position 0
};

enum RepeatTestState {
  RTEST_IDLE,
  RTEST_WAIT_REF_HOME,
  RTEST_GO_HIGH,
  RTEST_WAIT_HIGH,
  RTEST_GO_LOW,
  RTEST_WAIT_LOW,
  RTEST_WAIT_FINAL_HOME
};

enum Direction {
  DIR_UP,  // Away from endstop (higher position / higher frequency)
  DIR_DOWN // Towards endstop (lower position / lower frequency)
};

enum MotorRunKind {
  MOTOR_RUN_HOME,
  MOTOR_RUN_RELATIVE,
  MOTOR_RUN_ABSOLUTE
};

// Motor state variables
volatile MotorState currentMotorState = MOTOR_IDLE;
volatile HomingPhase currentHomingPhase = HOMING_APPROACH;
volatile int currentStep = 0;
volatile bool motorAktiv = false;
volatile long aktuellePosition = 0;
volatile long targetPosition = 0;
volatile long stepsRemaining = 0;
volatile int currentSpeed = SPEED_NORMAL;
volatile Direction currentDirection = DIR_DOWN;
volatile bool endstopHit = false;
volatile bool homingSucceeded = false;
volatile bool homingContactLatched = false;
volatile bool homingClearOpenSeen = false;
volatile bool lastHomingErrorValid = false;
volatile long lastHomingError = 0;
RepeatTestState rtestState = RTEST_IDLE;
int rtestCycles = 0;
int rtestIndex = 0;
bool rtestSavedTracking = false;
unsigned long rtestWaitUntil = 0;
char serialLineBuf[48];
uint8_t serialLineLen = 0;

// EEPROM configuration
#define EEPROM_SIZE 4096
#define ADRESSE_LETZTE_POSITION 0
#define ADRESSE_PUNKTE_ZAEHLER (ADRESSE_LETZTE_POSITION + sizeof(long))
#define ADRESSE_ERSTER_PUNKT 100
#define MAX_SPEICHERPUNKTE 100
#define ADRESSE_MOTOR_RUNNING_FLAG 1000
#define ADRESSE_BT_SETTINGS 1010
#define ADRESSE_BT_PEER 1020
#define BT_SETTINGS_MAGIC 0xB3
#define BT_SETTINGS_MAGIC_V2 0xB2
#define BT_SETTINGS_MAGIC_V1 0xB1
#define BT_PEER_MAGIC 0xC7
// V2 records carry the RFCOMM channel that actually delivered CI-V.
#define BT_PEER_MAGIC_V2 0xC8
#define BT_DEADBAND_MIN_KHZ 1
#define BT_DEADBAND_MAX_KHZ 1
#define BT_DEADBAND_DEFAULT_KHZ 1

#define CIV_PREAMBLE 0xFE
#define CIV_END 0xFD
#define CIV_CTRL_ADDR 0xE0
#define CIV_IC705_ADDR 0xA4
#define CIV_CMD_FREQ_TRANSCEIVE 0x00
#define CIV_CMD_READ_FREQ 0x03
#define CIV_CMD_READ_MODE 0x04
#define CIV_CMD_SET_FREQ 0x05
#define CIV_CMD_SET_MODE 0x06
#define CIV_CMD_PTT 0x1C
#define CIV_SUB_TUNER 0x01
#define CIV_SUB_RF_POWER 0x0A
#define CIV_CMD_LEVEL 0x14
#define CIV_CMD_METER 0x15
#define CIV_SUB_SWR 0x12
#define CIV_SUB_PO 0x11
#define CIV_MODE_FM 0x05
// RTTY keys a steady unmodulated carrier with the mic out of the path. CW
// would only arm TX without a key, and FM lets room noise modulate the sweep.
#define CIV_MODE_RTTY 0x04
#define BT_SPP_NAME "Magloop Tuner"
#define AUTOCAL_MAX_POINTS 40
#define AUTOCAL_SWR_GOOD 80
#define AUTOCAL_SWR_HIGH 48
#define AUTOCAL_SWR_HYST 15
#define AUTOCAL_SWR_FAR 5.5f
#define AUTOCAL_REWIND_STEPS 100
#define AUTOCAL_FINE_STEPS 280
#define AUTOCAL_SPEED_NEAR_MS 64
#define AUTOCAL_COARSE_MARGIN 80
#define AUTOCAL_FINE_MARGIN 40
#define AUTOCAL_UNDER_MIN_STEPS 40
// The SWR bridge gets vague near zero output, so measure with some power.
// BCD 0064 of 0000..0255 is roughly a quarter of the set maximum.
#define AUTOCAL_TX_PWR_HI 0x00
#define AUTOCAL_TX_PWR_LO 0x64
// Final grid for a retune: stop, wait for a fresh reading, step on. Immune to
// the CI-V round trip that a moving measurement cannot escape.
#define AUTOCAL_SCAN_POINTS 13
#define AUTOCAL_SCAN_GAP 3
#define AUTOCAL_SCAN_SPAN ((AUTOCAL_SCAN_POINTS - 1) * AUTOCAL_SCAN_GAP)
#define AUTOCAL_SCAN_SETTLE_MS 150
// The swept minimum always lands above the real one, so the grid starts well
// below it. If the best sample still falls on an edge, the grid moves there.
#define AUTOCAL_SCAN_BIAS 30
#define AUTOCAL_SCAN_PASSES 3
#define AUTOCAL_TX_SETTLE_MS 600
#define AUTOCAL_TX_METER_MS 800
#define AUTOCAL_MAX_TX_MS 45000
#define AUTOCAL_IGNORE_START 50
#define AUTOCAL_MIN_DIP_RUN 20
#define AUTOCAL_CONFIRM_MS 600
#define AUTOCAL_CONFIRM_SAMPLE_MS 500
BluetoothSerial SerialBT;

// Calibration table
struct Speicherpunkt {
  long position;
  float frequenz;
};
std::vector<Speicherpunkt> kalibrierTabelle;

// Ham Band definition for calibration
struct Band {
  long start;
  long end;
};
const std::vector<Band> hamBands = {
    {1800, 2000},   // 160m
    {3500, 3800},   // 80m
    {5351, 5367},   // 60m (approx)
    {7000, 7300},   // 40m
    {10100, 10150}, // 30m
    {14000, 14350}, // 20m
    {18068, 18168}, // 17m
    {21000, 21450}, // 15m
    {24890, 24990}, // 12m
    {28000, 29700}  // 10m
};

// UI state variables
volatile bool cursorVisible = false;
unsigned long lastCursorToggle = 0;
const unsigned long cursorInterval = 500;

// Menu page enumeration
enum MenuPage {
  PAGE_MENU = 0,
  PAGE_MANUAL = 1,
  PAGE_QRG_TARGET = 2,
  PAGE_QRG_SAVE = 3,
  PAGE_INIT = 4,
  PAGE_INIT_CONFIRM = 5,
  PAGE_WEB_STATUS = 6,
  PAGE_CAL_GRAPH = 7,
  PAGE_RESULT = 8,
  MAX_PAGE_COUNT = 9
};
MenuPage currentPage = PAGE_MENU;

// UI state variables
String eingabePuffer = "";
String statusMeldung = "System Start...";
String resultTitle = "ERGEBNIS";
String resultL1 = "";
String resultL2 = "";
String resultL3 = "";
String resultL4 = "";
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 250;
const unsigned long displayIntervalManualMove = 80;
const unsigned long displayIntervalAutoCal = 50;
unsigned long statusMeldungEnde = 0;

// Web server configuration
WebServer server(80);
WebSocketsServer webSocket =
    WebSocketsServer(81); // WebSocket server on port 81
const char *ssid = "AntennaTuner";
const char *password = "12345678";

enum ValidateResult {
  VALIDATE_OK = 0,
  VALIDATE_CLEANED,
  VALIDATE_CRITICAL,
  VALIDATE_RESET
};

// Web calibration state
struct WebCalibrationState {
  bool active = false;
  std::vector<Speicherpunkt> backup;
};
WebCalibrationState webCalibration;

struct __attribute__((packed)) BtSettings {
  uint8_t magic;
  uint8_t trackingEnabled;
  uint8_t deadbandKhz;
  uint8_t btAutoStart;
};
BtSettings btSettings = {BT_SETTINGS_MAGIC, 0, 5, 0};

enum BleCommand {
  BLE_CMD_NONE = 0,
  BLE_CMD_START,
  BLE_CMD_STOP
};
enum BleLinkState {
  BLE_LINK_IDLE = 0,
  BLE_LINK_READY,
  BLE_LINK_ERROR
};

enum AutoCalState {
  AUTOCAL_IDLE = 0,
  AUTOCAL_CONFIRM,
  AUTOCAL_PREP,
  AUTOCAL_WAIT_CIV,
  AUTOCAL_SET_FREQ,
  AUTOCAL_MOVE_EST,
  AUTOCAL_WAIT_MOVE,
  AUTOCAL_TX_ON,
  AUTOCAL_WAIT_TX,
  AUTOCAL_SWEEP,
  AUTOCAL_REWIND,
  AUTOCAL_WAIT_REWIND,
  AUTOCAL_FINE_TX,
  AUTOCAL_WAIT_FINE_TX,
  AUTOCAL_FINE,
  AUTOCAL_UNDER_MIN,
  AUTOCAL_SCAN_STEP,
  AUTOCAL_SCAN_WAIT,
  AUTOCAL_GOTO_MIN,
  AUTOCAL_APPROACH_MIN,
  AUTOCAL_CONFIRM_SWR,
  AUTOCAL_SAVE,
  AUTOCAL_NEXT,
  AUTOCAL_RESTORE
};

enum AutoCalCiv {
  AUTOCAL_CIV_NONE = 0,
  AUTOCAL_CIV_PREP,
  AUTOCAL_CIV_SET_FREQ,
  AUTOCAL_CIV_PTT_ON,
  AUTOCAL_CIV_PTT_OFF,
  AUTOCAL_CIV_RESTORE
};

struct AutoCalBand {
  const char *name;
  long startKhz;
  long endKhz;
};

volatile BleCommand bleCommand = BLE_CMD_NONE;
volatile BleLinkState bleLinkState = BLE_LINK_IDLE;
volatile bool blePairGranted = false;
volatile bool bleReady = false;
volatile bool bleTxActive = false;
volatile uint32_t bleRigFreqHz = 0;
char bleErrorMessage[48] = {0};
volatile bool bleInitialized = false;
unsigned long bleReadyAt = 0;
unsigned long btAutoStartAt = 0;
unsigned long btReconnectAt = 0;
unsigned long bleCivWatchAt = 0;
uint8_t blePeerMac[6] = {0};
bool blePeerValid = false;
bool bleGotCiv = false;
bool bleOutboundEnabled = true;
int bleSppChannelTry = 0;
int bleOutboundFails = 0;
const uint8_t bleSppChannels[] = {1, 2, 3, 4, 5, 0};
// Probing the rig from the serial console: dump what arrives, send anything,
// and poll the tuner. Sending is queued because only bleTask may touch SPP.
volatile bool civMonitor = false;
volatile bool civTunerWatch = false;
unsigned long civTunerPollAt = 0;
uint8_t civTxPending[8];
volatile uint8_t civTxPendingLen = 0;
uint8_t bleTunerState = 0xFF; // 0xFF = noch nichts gesehen
// The 705 accepts SPP on channels that carry no CI-V, so probing costs a
// connect plus the CI-V watchdog each time. Remember the one that worked.
uint8_t blePeerChannel = 0; // 0 = unknown, fall back to probing
bool blePeerChannelTried = false;
uint8_t bleActiveChannel = 0;
bool wifiApActive = false;
uint8_t civRxBuf[256];
size_t civRxLen = 0;
float lastTrackedKhz = -1;

const AutoCalBand autoCalBands[] = {
    {"40m", 7000, 7200},   {"30m", 10100, 10150}, {"20m", 14000, 14350},
    {"17m", 18068, 18168}, {"15m", 21000, 21450}, {"12m", 24890, 24990},
    {"10m", 28000, 29700}};

// Retune = the fine search of an auto-cal applied to the rig's current
// frequency only. It corrects drift and never touches the table.
volatile bool autoCalRetune = false;
volatile AutoCalState autoCalState = AUTOCAL_IDLE;
volatile AutoCalCiv autoCalCivReq = AUTOCAL_CIV_NONE;
volatile bool autoCalCivDone = false;
volatile bool autoCalPollSwr = false;
volatile bool autoCalEndstopHit = false;
volatile uint16_t bleSwrRaw = 0;
volatile bool bleSwrValid = false;
// Position that bleSwrRaw actually belongs to. The reading is up to a poll
// interval plus a Bluetooth round trip old, so the live position would pin it
// far past where it was taken.
volatile long bleSwrReqPos = 0;
volatile long bleSwrPos = 0;
long autoCalScanBestPos = 0;
uint16_t autoCalScanBestSwr = 255;
int autoCalScanLeft = 0;
int autoCalScanPass = 0;
long autoCalScanStart = 0;
unsigned long autoCalScanSettleAt = 0;
// Runtime-adjustable via SETTLE, so the dwell time can be compared against the
// confirmation step without reflashing.
unsigned long autoCalScanSettleMs = AUTOCAL_SCAN_SETTLE_MS;
volatile uint16_t blePoRaw = 0;
volatile uint8_t bleSavedMode = CIV_MODE_FM;
volatile uint8_t bleSavedFilter = 0x01;
volatile uint8_t bleSavedPowerHi = 0x00;
volatile uint8_t bleSavedPowerLo = 0x80;
volatile uint32_t autoCalTargetHz = 0;
volatile uint32_t autoCalSavedFreqHz = 0;
volatile uint8_t autoCalSavedMode = CIV_MODE_FM;
volatile uint8_t autoCalSavedFilter = 0x01;
volatile uint8_t autoCalSavedPowerHi = 0x00;
volatile uint8_t autoCalSavedPowerLo = 0x80;
bool autoCalBlockMenu = false;
bool autoCalSavedTracking = false;
int autoCalBandIndex = -1;
float autoCalFreqs[AUTOCAL_MAX_POINTS];
int autoCalCount = 0;
int autoCalIndex = 0;
int autoCalSkipped = 0;
// Dips measured in this run, in the order they were found (ascending freq).
float autoCalPtFreq[AUTOCAL_MAX_POINTS];
long autoCalPtPos[AUTOCAL_MAX_POINTS];
int autoCalPtCount = 0;
uint16_t autoCalMinSwr = 255;
long autoCalMinPos = 0;
unsigned long autoCalWaitUntil = 0;
unsigned long autoCalTxStarted = 0;
AutoCalState autoCalAfterCiv = AUTOCAL_IDLE;
AutoCalState autoCalAfterMove = AUTOCAL_IDLE;
bool autoCalSawHighSwr = false;
bool autoCalTxLive = false;
bool autoCalConfirmSample = false;
long autoCalSweepStartPos = 0;

// Function prototypes
void updateDisplay();
void drawPositionBar();
void processKeypad(char taste);
void saveCurrentPosition();
void loadKalibrierTabelle();
void sortKalibrierTabelle();
float getPositionFromFrequency(float frequenz);
float getFrequencyFromPosition(long position);
ValidateResult saveSpeicherpunkt(long pos, float freq);
void resetKalibrierung();
void persistKalibrierTabelle();
void ensureCalibrationSession();
void restoreCalibrationBackup();
void commitCalibrationSession();
void setStatusMessage(String msg, unsigned int durationMs);
void jumpToPage(MenuPage targetPage);
void showResultWindow(const String &title, const String &l1,
                      const String &l2 = "", const String &l3 = "",
                      const String &l4 = "");
void drawCalTableGraph();
ValidateResult validateAndCleanKalibrierTabelle();
void dumpKalibrierTabelle();

// WebSocket function prototypes
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length);
void broadcastStatusUpdate();

// Web server function prototypes
void setupWiFi();
void restoreWifiAp();
void wifiStopForBt();
void btStopRadio();
void setupWebServer();
void handleRoot();
void handleStatus();
void handleMove();
void handleCalibration();
void handleCalibrationControl();
void handleCalibrationSave();
void handleCalibrationSaveAndExit();
void handleCalibrationAbort();
void handleCalibrationPoint();
void handleCalibrationData();
void handleReset();
void loadBtSettings();
void saveBtSettings();
void loadBtPeer();
void saveBtPeer();
void bleRememberPeer(const uint8_t *mac);
bool bleSelectPeerMac(uint8_t *outMac);
bool bleConnectPeer();
void bleDropDeadLink(const char *reason);
void bleSppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void toggleTracking();
void bleTask(void *pvParameters);
bool btEnsureInit();
void btConfirmPin(uint32_t pin);
void processRigTracking();
void processAutoCal();
void autoCalAbort(const char *msg, bool restoreRadio = true);
String bleStateText();
String formatSwr(uint16_t raw);
float swrFromRaw(uint16_t raw);
int autoCalSpeedForSwr(uint16_t raw);
bool autoCalPastDip();
void autoCalApplySweepSpeed();
String getHtmlHeader();
String getHtmlFooter();
String getStateText(MotorState state);

// Motor control: one door. Callers use startHoming / startMove / moveToPosition.
bool motorRun(MotorRunKind kind, long value, int speed);
void motorArmLeg(long dest, int speed);
void motorFinishJob();
void startHoming();
void startMove(long steps, int speed);
void motorCancelToStop();
void motorSettleAndStop();
void stopMotor();
void noteHomingContact(long believedPos, const char *why);
void processSerialCommands();
void handleSerialCommand(const String &line);
void processRepeatTest();
void rtestStart(int cycles);
void rtestAbort(const char *msg);
void setMotorRunningFlag(bool running);
bool isMotorActive();
bool isHomingState();
bool endstopPressed();
long getCurrentPosition();
String motorStateToString(MotorState state);
String directionToString(Direction dir);
String homingPhaseToString(HomingPhase phase);

// Application logic functions
void moveToFrequency(float frequency);
void moveToPosition(long position);
void moveToPosition(long position, int speed);

// Motor task function
void motorTask(void *pvParameters);

// Helper function to get string representation of motor state
String motorStateToString(MotorState state) {
  switch (state) {
  case MOTOR_IDLE:
    return "IDLE";
  case MOTOR_HOMING:
    return "HOMING";
  case MOTOR_HOMING_AT_ENDSTOP:
    return "HOMING_AT_ENDSTOP";
  case MOTOR_MOVING:
    return "MOVING";
  default:
    return "UNKNOWN";
  }
}

// Helper function to get string representation of direction
String directionToString(Direction dir) {
  switch (dir) {
  case DIR_UP:
    return "UP";
  case DIR_DOWN:
    return "DOWN";
  default:
    return "UNKNOWN";
  }
}

// Helper function to get string representation of homing phase
String homingPhaseToString(HomingPhase phase) {
  switch (phase) {
  case HOMING_APPROACH:
    return "APPROACH";
  case HOMING_CLEAR:
    return "CLEAR";
  case HOMING_SLOW_SEEK:
    return "SLOW_SEEK";
  default:
    return "UNKNOWN";
  }
}

bool isHomingState() {
  return currentMotorState == MOTOR_HOMING ||
         currentMotorState == MOTOR_HOMING_AT_ENDSTOP;
}

bool endstopPressed() { return digitalRead(ENDSTOP_PIN) == LOW; }

void noteHomingContact(long believedPos, const char *why) {
  if (homingContactLatched) {
    return;
  }
  homingContactLatched = true;
  Serial.println("[HOME] Endschalter zu bei angenommener Pos " +
                 String(believedPos) + " (" + String(why) + ")");
}

void noteHomingZero(long believedPos) {
  lastHomingError = believedPos;
  lastHomingErrorValid = true;
  Serial.println("[HOME] Null=Schliessen bei angenommener Pos " +
                 String(believedPos));
}

void enterHomingClear(const char *why) {
  noteHomingContact(aktuellePosition, why);
  endstopHit = true;
  homingClearOpenSeen = false;
  currentHomingPhase = HOMING_CLEAR;
  currentDirection = DIR_UP;
  currentSpeed = SPEED_SLOW;
  motorRampLeft = RAMP_STEPS;
  stepsRemaining = HOMING_CLEAR_MAX_STEPS;
  Serial.println(F("[HOME] Frei: hoch bis offen, dann +20"));
}

void enterHomingSlowSeek() {
  currentHomingPhase = HOMING_SLOW_SEEK;
  currentDirection = DIR_DOWN;
  currentSpeed = SPEED_SLOW;
  motorRampLeft = RAMP_STEPS;
  stepsRemaining = HOMING_SEEK_MAX_STEPS;
  Serial.println(F("[HOME] Langsam auf Schliessen = 0"));
}

void setMotorRunningFlag(bool running) {
  EEPROM.put(ADRESSE_MOTOR_RUNNING_FLAG, running);
  EEPROM.commit();
}

void motorArmLeg(long dest, int speed) {
  dest = constrain(dest, 0L, gesamtSchritte);
  long steps = dest - aktuellePosition;
  if (steps == 0) {
    return;
  }
  currentDirection = (steps > 0) ? DIR_UP : DIR_DOWN;
  targetPosition = dest;
  stepsRemaining = labs(steps);
  currentSpeed = speed;
  // A leg that continues an already running move needs no ramp.
  if (!motorAktiv) {
    motorRampLeft = RAMP_STEPS;
  }
  currentMotorState = MOTOR_MOVING;
  motorAktiv = true;
  endstopHit = false;
  homingSucceeded = false;
  Serial.println("[MOTOR] Leg to P=" + String(dest) + " steps=" + String(steps) +
                 " speed=" + String(speed));
}

void motorFinishJob() {
  Direction dirAtStop = currentDirection;
  bool homed = homingSucceeded;
  motorAktiv = false;
  stepsRemaining = 0;
  motorRampLeft = 0;
  motorFinalGoal = -1;
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], LOW);
  }

  if (homed) {
    homingSucceeded = false;
    settledFromBelow = false;
    aktuellePosition = 0;
    Serial.println("[HOME] Fertig. Null=Schliessen  P=" +
                   String(aktuellePosition) + "  Fehler=" +
                   (lastHomingErrorValid ? String(lastHomingError) : String("-")));
    if (lastHomingErrorValid) {
      setStatusMessage("HOMING P=" + String(aktuellePosition), 2000);
      if (rtestState == RTEST_IDLE && autoCalState <= AUTOCAL_CONFIRM) {
        showResultWindow("HOMING",
                         "P=" + String(aktuellePosition),
                         "Fehler " + String(lastHomingError),
                         "Null = Schliessen", "* Menue");
      }
    } else {
      setStatusMessage("HOMING P=" + String(aktuellePosition), 2000);
      if (rtestState == RTEST_IDLE && autoCalState <= AUTOCAL_CONFIRM) {
        showResultWindow("HOMING", "P=" + String(aktuellePosition),
                         "Null = Schliessen", "* Menue", "");
      }
    }
  } else {
    aktuellePosition = constrain(aktuellePosition, 0L, gesamtSchritte);
    settledFromBelow = (dirAtStop == DIR_UP);
  }

  currentMotorState = MOTOR_IDLE;
  currentHomingPhase = HOMING_APPROACH;
  saveCurrentPosition();
  setMotorRunningFlag(false);
  motorJobActive = false;
  Serial.println("[MOTOR] Ankunft P=" + String(aktuellePosition));
}

bool motorRun(MotorRunKind kind, long value, int speed) {
  if (motorJobActive || motorAktiv) {
    Serial.println("[MOTOR] Busy, reject");
    return false;
  }

  long stored = 0;
  EEPROM.get(ADRESSE_LETZTE_POSITION, stored);
  if (stored >= 0 && stored <= gesamtSchritte) {
    long mismatch = labs(stored - aktuellePosition);
    if (kind != MOTOR_RUN_HOME && mismatch > 100) {
      Serial.println("[MOTOR] Pos mismatch EEPROM=" + String(stored) +
                     " RAM=" + String(aktuellePosition) + " -> Home");
      kind = MOTOR_RUN_HOME;
      value = 0;
    } else if (mismatch != 0) {
      Serial.println("[MOTOR] Pos EEPROM " + String(stored) + " RAM " +
                     String(aktuellePosition));
      aktuellePosition = stored;
    }
  }

  if (kind == MOTOR_RUN_HOME) {
    motorJobActive = true;
    motorFinalGoal = -1;
    setMotorRunningFlag(true);
    homingContactLatched = false;
    homingClearOpenSeen = false;
    lastHomingErrorValid = false;
    lastHomingError = 0;
    settledFromBelow = false;
    homingSucceeded = false;
    endstopTriggered = false;
    endstopHit = false;
    if (endstopPressed()) {
      Serial.println("[MOTOR] Homing at endstop");
      currentMotorState = MOTOR_HOMING_AT_ENDSTOP;
      enterHomingClear("schon aktiv");
    } else {
      Serial.println("[MOTOR] Homing approach");
      currentMotorState = MOTOR_HOMING;
      currentHomingPhase = HOMING_APPROACH;
      currentSpeed = SPEED_FAST;
      currentDirection = DIR_DOWN;
      motorRampLeft = RAMP_STEPS;
      stepsRemaining = gesamtSchritte + HOMING_BACKOFF_STEPS;
    }
    // Release the motor task only once phase, direction, speed and step
    // count are in place. It runs on the other core and would otherwise
    // start on the previous move's settings.
    motorAktiv = true;
    return true;
  }

  if (speed < 1) {
    speed = SPEED_NORMAL;
  }

  long target;
  if (kind == MOTOR_RUN_RELATIVE) {
    if (value == 0) {
      return false;
    }
    target = constrain(aktuellePosition + value, 0L, gesamtSchritte);
    if (target == aktuellePosition) {
      return false;
    }
  } else {
    target = constrain(value, 0L, gesamtSchritte);
    if (target == aktuellePosition && settledFromBelow) {
      Serial.println("[MOTOR] Already at P=" + String(target));
      return true;
    }
  }
  motorJobActive = true;
  motorFinalGoal = target;
  motorLegSpeed = speed;
  setMotorRunningFlag(true);
  Serial.println("[MOTOR] GoTo P=" + String(target) + " from " +
                 String(aktuellePosition));
  if (settledFromBelow && target > aktuellePosition) {
    motorArmLeg(target, speed);
  } else {
    long pre = target - (long)BACKLASH_STEPS;
    if (pre < 0) {
      pre = 0;
    }
    if (aktuellePosition != pre) {
      motorArmLeg(pre, speed);
    } else {
      motorArmLeg(target, speed);
    }
  }
  if (!motorAktiv) {
    motorFinishJob();
  }
  return true;
}

void startHoming() { motorRun(MOTOR_RUN_HOME, 0, SPEED_FAST); }

void startMove(long steps, int speed) {
  motorRun(MOTOR_RUN_RELATIVE, steps, speed);
}

// Drop the rest of the planned move and coast to a stop at the current
// position. Without clearing the goal the motor task would re-arm the
// remaining leg.
void motorCancelToStop() {
  if (currentMotorState != MOTOR_MOVING) {
    return;
  }
  motorFinalGoal = -1;
  stepsRemaining = 0;
}

void stopMotor() {
  if (!motorAktiv && !motorJobActive) {
    currentMotorState = MOTOR_IDLE;
    currentHomingPhase = HOMING_APPROACH;
    return;
  }
  motorFinishJob();
}

bool isMotorActive() { return motorJobActive || motorAktiv; }

long getCurrentPosition() { return aktuellePosition; }

// Keeps the coils on the last commanded phase for a moment so the rotor can
// pull in. Only safe from the motor task, it blocks.
void motorSettleAndStop() {
  vTaskDelay(pdMS_TO_TICKS(MOTOR_SETTLE_MS));
  stopMotor();
}

// Motor task implementation
void motorTask(void *pvParameters) {
  unsigned long lastStepTime = 0;
  uint8_t endstopLowCount = 0;
  uint8_t endstopOpenCount = 0;

  for (;;) {
    if (motorAktiv) {
      unsigned long interval = (unsigned long)currentSpeed;
      if (motorRampLeft > 0) {
        long slower = (long)(RAMP_START_MS - currentSpeed) * motorRampLeft /
                      (long)RAMP_STEPS;
        if (slower > 0) {
          interval += (unsigned long)slower;
        }
      }
      if (millis() - lastStepTime >= interval) {
        lastStepTime = millis();

        bool pinLow = endstopPressed();
        endstopTriggered = false;
        if (pinLow) {
          if (endstopLowCount < 20) {
            endstopLowCount++;
          }
          endstopOpenCount = 0;
        } else {
          endstopLowCount = 0;
          if (endstopOpenCount < 20) {
            endstopOpenCount++;
          }
        }

        bool pressed = pinLow;
        if (currentMotorState == MOTOR_MOVING) {
          uint8_t need = 3;
          if (aktuellePosition > (HOMING_BACKOFF_STEPS + 80)) {
            need = 8;
          }
          pressed = endstopLowCount >= need;
        } else if (isHomingState() && currentHomingPhase == HOMING_APPROACH) {
          if (pinLow) {
            currentSpeed = SPEED_SLOW;
          }
          pressed = endstopLowCount >= HOMING_PRESS_DEBOUNCE;
        } else if (isHomingState() && currentHomingPhase == HOMING_SLOW_SEEK) {
          pressed = endstopLowCount >= HOMING_ZERO_DEBOUNCE;
        }

        if (pressed && currentMotorState == MOTOR_MOVING &&
            currentDirection == DIR_DOWN) {
          Serial.println(F("[MOTOR] Endstop during move, re-homing"));
          homingContactLatched = false;
          noteHomingContact(aktuellePosition, "waehrend Fahrt");
          endstopLowCount = 0;
          endstopOpenCount = 0;
          if (autoCalState > AUTOCAL_CONFIRM) {
            autoCalEndstopHit = true;
          }
          stopMotor();
          startHoming();
          continue;
        }

        if (pressed && isHomingState() &&
            currentHomingPhase == HOMING_APPROACH) {
          enterHomingClear("Anfahrt");
          continue;
        }

        if (pressed && isHomingState() &&
            currentHomingPhase == HOMING_SLOW_SEEK) {
          noteHomingZero(aktuellePosition);
          aktuellePosition = 0;
          Serial.println(F("[HOME] Null = Schliessen"));
          homingSucceeded = true;
          motorSettleAndStop();
          continue;
        }

        currentStep = (currentDirection == DIR_DOWN)
                          ? (currentStep + 1) % 8
                          : (currentStep - 1 + 8) % 8;

        for (int i = 0; i < 4; i++) {
          digitalWrite(motorPins[i], stepSequence[currentStep][i]);
        }

        aktuellePosition += (currentDirection == DIR_DOWN) ? -1 : 1;
        if (isHomingState()) {
          aktuellePosition = constrain(aktuellePosition, -HOMING_BACKOFF_STEPS,
                                       gesamtSchritte + HOMING_BACKOFF_STEPS);
        } else {
          aktuellePosition = constrain(aktuellePosition, 0L, gesamtSchritte);
        }

        if (stepsRemaining > 0) {
          stepsRemaining = stepsRemaining - 1;
        }
        if (motorRampLeft > 0) {
          motorRampLeft = motorRampLeft - 1;
        }

        if (isHomingState() && currentHomingPhase == HOMING_CLEAR) {
          if (!homingClearOpenSeen) {
            if (endstopOpenCount >= HOMING_OPEN_DEBOUNCE) {
              homingClearOpenSeen = true;
              stepsRemaining = HOMING_CLEAR_EXTRA;
              Serial.println(F("[HOME] Schalter offen, +20"));
            }
          } else if (stepsRemaining <= 0) {
            enterHomingSlowSeek();
            continue;
          }
        }

        if (stepsRemaining <= 0) {
          if (currentMotorState == MOTOR_MOVING) {
            if (motorFinalGoal >= 0 && aktuellePosition != motorFinalGoal) {
              Serial.println("[MOTOR] Continue to P=" + String(motorFinalGoal));
              motorArmLeg(motorFinalGoal, motorLegSpeed);
              if (motorAktiv && stepsRemaining > 0) {
                continue;
              }
            }
            motorSettleAndStop();
          } else if (isHomingState()) {
            if (currentHomingPhase == HOMING_APPROACH) {
              Serial.println(
                  F("[MOTOR] Homing approach finished without endstop"));
              setStatusMessage("Homing fehlgeschlagen", 3000);
              if (rtestState == RTEST_IDLE && autoCalState <= AUTOCAL_CONFIRM) {
                showResultWindow("HOMING", "Fehlgeschlagen",
                                 "Endschalter nicht", "gefunden", "* Menue");
              }
              stopMotor();
            } else if (currentHomingPhase == HOMING_CLEAR) {
              Serial.println(F("[HOME] Frei: Schalter bleibt zu"));
              setStatusMessage("Homing: Endstop aktiv", 3000);
              if (rtestState == RTEST_IDLE && autoCalState <= AUTOCAL_CONFIRM) {
                showResultWindow("HOMING", "Endstop bleibt zu",
                                 "P=" + String(aktuellePosition), "",
                                 "* Menue");
              }
              stopMotor();
            } else if (currentHomingPhase == HOMING_SLOW_SEEK) {
              Serial.println(F("[HOME] Fein ohne Schliessen, neue Anfahrt"));
              homingContactLatched = false;
              homingClearOpenSeen = false;
              currentHomingPhase = HOMING_APPROACH;
              currentDirection = DIR_DOWN;
              currentSpeed = SPEED_FAST;
              stepsRemaining = gesamtSchritte + HOMING_BACKOFF_STEPS;
            }
          }
        }
      }
    } else {
      endstopLowCount = 0;
      endstopOpenCount = 0;
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorPins[i], LOW);
      }
    }

    vTaskDelay(1);
  }
}

// Application logic implementation
void moveToFrequency(float frequency) {
  Serial.println("[APP] Moving to frequency: " + String(frequency) + " kHz");

  long targetPosition = (long)round(getPositionFromFrequency(frequency));
  Serial.println("[APP] Target position: " + String(targetPosition));

  moveToPosition(targetPosition);
}

void moveToPosition(long position) { moveToPosition(position, SPEED_NORMAL); }

void moveToPosition(long position, int speed) {
  motorRun(MOTOR_RUN_ABSOLUTE, position, speed);
}

void saveCurrentPosition() {
  long positionToSave = constrain(aktuellePosition, 0L, gesamtSchritte);
  EEPROM.put(ADRESSE_LETZTE_POSITION, positionToSave);
  EEPROM.commit();
  long verify = 0x7fffffffL;
  EEPROM.get(ADRESSE_LETZTE_POSITION, verify);
  if (verify != positionToSave) {
    Serial.println("[SYSTEM] Position EEPROM verify fail, retry");
    EEPROM.put(ADRESSE_LETZTE_POSITION, positionToSave);
    EEPROM.commit();
  } else {
    Serial.println("[SYSTEM] Saving position: " + String(positionToSave));
  }
}

void setStatusMessage(String msg, unsigned int durationMs) {
  Serial.println("[SYSTEM] Status: " + msg);
  statusMeldung = msg;
  statusMeldungEnde = millis() + durationMs;
}

void jumpToPage(MenuPage targetPage) {
  if (currentPage == targetPage) {
    return;
  }
  Serial.println("[UI] Jumping to page: " + String(targetPage));
  currentPage = targetPage;
  eingabePuffer = "";
}

void showResultWindow(const String &title, const String &l1, const String &l2,
                      const String &l3, const String &l4) {
  resultTitle = title;
  resultL1 = l1;
  resultL2 = l2;
  resultL3 = l3;
  resultL4 = l4;
  jumpToPage(PAGE_RESULT);
  lastDisplayUpdate = 0;
}

void persistKalibrierTabelle() {
  EEPROM.put(ADRESSE_PUNKTE_ZAEHLER, (int)kalibrierTabelle.size());
  for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
    EEPROM.put(ADRESSE_ERSTER_PUNKT + i * sizeof(Speicherpunkt),
               kalibrierTabelle[i]);
  }
  EEPROM.commit();
}

void ensureCalibrationSession() {
  if (!webCalibration.active) {
    webCalibration.backup = kalibrierTabelle;
    webCalibration.active = true;
    Serial.println("[CAL] Session started, backup " +
                   String(webCalibration.backup.size()) + " points");
  }
}

void restoreCalibrationBackup() {
  kalibrierTabelle = webCalibration.backup;
  persistKalibrierTabelle();
  webCalibration.active = false;
  webCalibration.backup.clear();
  Serial.println("[CAL] Session aborted, table restored");
}

void commitCalibrationSession() {
  persistKalibrierTabelle();
  webCalibration.active = false;
  webCalibration.backup.clear();
    Serial.println("[CAL] Session committed, " +
                   String((int)kalibrierTabelle.size()) + " points");
}

void setupWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(ssid, password, 1, 0, 4);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
  IPAddress IP = WiFi.softAPIP();
  wifiApActive = true;
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void restoreWifiAp() {
  WiFi.mode(WIFI_OFF);
  delay(150);
  setupWiFi();
}

void wifiStopForBt() {
  if (!wifiApActive && WiFi.getMode() == WIFI_OFF) {
    Serial.println("[WIFI] Already off, skip stop");
    return;
  }
  Serial.println("[WIFI] Off for Bluetooth pairing");
  WiFi.softAPdisconnect(true);
  delay(50);
  WiFi.mode(WIFI_OFF);
  delay(50);
  esp_wifi_stop();
  delay(250);
  wifiApActive = false;
}

void btStopRadio() {
  if (autoCalState != AUTOCAL_IDLE) {
    if (SerialBT.hasClient()) {
      uint8_t pttOff[] = {CIV_CMD_PTT, 0x00, 0x00};
      bleSendCiv(pttOff, 3);
      delay(80);
    }
    autoCalAbort("Kal. abgebrochen", false);
  }
  if (bleInitialized) {
    if (SerialBT.hasClient()) {
      SerialBT.disconnect();
      delay(150);
    }
    SerialBT.end();
    delay(200);
  }
  bleInitialized = false;
  bleReady = false;
  bleTxActive = false;
  bleRigFreqHz = 0;
  bleSwrValid = false;
  lastTrackedKhz = -1;
  civRxLen = 0;
  bleLinkState = BLE_LINK_IDLE;
  restoreWifiAp();
  Serial.println("[BT] Stack off, WiFi AP restored");
}

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;

  case WStype_CONNECTED: {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0],
                  ip[1], ip[2], ip[3], payload);

    // Send current status immediately when client connects
    broadcastStatusUpdate();

    // Send calibration data
    String calibData = "{\"type\":\"calibration\",\"points\":[";
    for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
      if (i > 0)
        calibData += ",";
      calibData += "{\"position\":" + String(kalibrierTabelle[i].position) +
                   ",\"frequency\":" + String(kalibrierTabelle[i].frequenz, 2) +
                   "}";
    }
    calibData += "]}";
    webSocket.sendTXT(num, calibData);
    break;
  }

  case WStype_TEXT: {
    Serial.printf("[%u] get Text: %s\n", num, payload);

    // Handle calibration requests
    String message = String((char *)payload);
    if (message.startsWith("{\"calibrate\":")) {
      // Parse frequency from JSON
      int freqStart = message.indexOf(":") + 1;
      int freqEnd = message.indexOf("}");
      String freqStr = message.substring(freqStart, freqEnd);
      float frequency = freqStr.toFloat();

      if (frequency > 0) {
        ensureCalibrationSession();
        ValidateResult result = saveSpeicherpunkt(aktuellePosition, frequency);

        String calibData = "{\"type\":\"calibration\",\"points\":[";
        for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
          if (i > 0)
            calibData += ",";
          calibData +=
              "{\"position\":" + String(kalibrierTabelle[i].position) +
              ",\"frequency\":" + String(kalibrierTabelle[i].frequenz, 2) + "}";
        }
        calibData += "]}";
        webSocket.broadcastTXT(calibData);
        broadcastStatusUpdate();

        bool success =
            (result != VALIDATE_CRITICAL && result != VALIDATE_RESET);
        String resultJson = "{\"type\":\"calibrate_result\",\"success\":";
        resultJson += success ? "true" : "false";
        resultJson += ",\"message\":\"";
        resultJson += statusMeldung;
        resultJson += "\"}";
        webSocket.sendTXT(num, resultJson);
      }
    }
    break;
  }

  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
    break;
  }
}

String getStateText(MotorState state) {
  switch (state) {
  case MOTOR_IDLE:
    return "Idle";
  case MOTOR_MOVING:
    return "Moving";
  case MOTOR_HOMING:
    switch (currentHomingPhase) {
    case HOMING_APPROACH:
      return "Homing: Approach";
    case HOMING_CLEAR:
      return "Homing: Clear";
    case HOMING_SLOW_SEEK:
      return "Homing: Seek=0";
    }
    return "Homing";
  case MOTOR_HOMING_AT_ENDSTOP:
    switch (currentHomingPhase) {
    case HOMING_APPROACH:
      return "Homing@Endstop: Approach";
    case HOMING_CLEAR:
      return "Homing@Endstop: Clear";
    case HOMING_SLOW_SEEK:
      return "Homing@Endstop: Seek=0";
    }
    return "Homing@Endstop";
  default:
    return "Unknown";
  }
}

void broadcastStatusUpdate() {
  String status =
      "{\"type\":\"status\",\"position\":" + String(aktuellePosition) +
      ",\"frequency\":" +
      String(getFrequencyFromPosition(aktuellePosition), 2) + ",\"state\":\"" +
      getStateText(currentMotorState) + "\",\"btLink\":\"" + bleStateText() +
      "\",\"btFrequency\":" +
      String(bleRigFreqHz > 0 ? bleRigFreqHz / 1000.0f : 0, 2) + ",\"btTx\":" +
      String(bleTxActive ? "true" : "false") + ",\"btReady\":" +
      String(bleReady ? "true" : "false") + ",\"tracking\":" +
      String(btSettings.trackingEnabled ? "true" : "false") + "}";
  webSocket.broadcastTXT(status);
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/move", HTTP_POST, handleMove);
  server.on("/calibration", HTTP_GET, handleCalibration);
  server.on("/calibration/data", HTTP_GET, handleCalibrationData);
  server.on("/calibration/start", HTTP_POST, []() {
    Serial.println("[WEB] Calibration start requested");
    if (!isMotorActive()) {
      ensureCalibrationSession();
      startHoming();
      setStatusMessage("Calibration: Homing...", 2000);
      Serial.println("[WEB] Calibration homing started");
    } else {
      Serial.println("[WEB] Calibration start rejected - motor busy");
    }
    server.sendHeader("Location", "/calibration");
    server.send(302, "text/plain", "Redirecting...");
  });
  server.on("/calibration/control", HTTP_POST, handleCalibrationControl);
  server.on("/calibration/save", HTTP_POST, handleCalibrationSave);
  server.on("/calibration/save_and_exit", HTTP_POST,
            handleCalibrationSaveAndExit);
  server.on("/calibration/abort", HTTP_POST, handleCalibrationAbort);
  server.on("/calibration/point", HTTP_POST, handleCalibrationPoint);
  server.on("/reset", HTTP_POST, handleReset);

  server.begin();
  Serial.println("HTTP server started");

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started on port 81");
}

String getHtmlHeader() {
  return R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Antenna Tuner Control</title>
  <style>
    :root {
      --bg-color: #121212;
      --text-color: #ffffff;
      --card-bg: #1e1e1e;
      --border-color: #444;
      --button-bg: #2e7d32;
      --button-danger-bg: #c62828;
      --progress-bg: #333;
    }
    
    body { 
      font-family: Arial, sans-serif; 
      margin: 20px; 
      background-color: var(--bg-color);
      color: var(--text-color);
      transition: background-color 0.3s, color 0.3s;
    }
    
    .container { max-width: 800px; margin: 0 auto; }
    
    .card { 
      border: 1px solid var(--border-color); 
      border-radius: 5px; 
      padding: 15px; 
      margin-bottom: 20px;
      background-color: var(--card-bg);
    }
    
    .button { 
      background-color: var(--button-bg); 
      border: none; 
      color: white; 
      padding: 10px 15px; 
      text-align: center; 
      text-decoration: none; 
      display: inline-block; 
      font-size: 16px; 
      margin: 4px 2px; 
      cursor: pointer; 
      border-radius: 4px;
    }
    
    .button-danger { background-color: var(--button-danger-bg); }
    
    .input { 
      padding: 8px; 
      margin: 5px 0; 
      border: 1px solid var(--border-color); 
      border-radius: 4px; 
      width: 100%; 
      background-color: var(--bg-color);
      color: var(--text-color);
      box-sizing: border-box;
    }
    
    .status { 
      background-color: var(--card-bg); 
      padding: 10px; 
      border-radius: 4px; 
      margin: 10px 0; 
    }
    
    .calibration-step { 
      background-color: var(--card-bg); 
      padding: 15px; 
      border-radius: 4px; 
      margin: 10px 0; 
    }
    
    .progress { 
      width: 100%; 
      background-color: var(--progress-bg); 
      border-radius: 4px; 
    }
    
    .progress-bar { 
      height: 20px; 
      background-color: var(--button-bg); 
      border-radius: 4px; 
    }
    
    .control-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 15px;
    }
    
    .control-group {
      text-align: center;
    }
    
    .control-label {
      font-weight: bold;
      margin-bottom: 5px;
    }

    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 20px;
    }
    
    th, td {
      border: 1px solid var(--border-color);
      padding: 8px;
      text-align: center;
    }
    
    th {
      background-color: var(--button-bg);
      color: white;
    }
    
    .connection-status {
      position: fixed;
      top: 10px;
      right: 10px;
      padding: 5px 10px;
      border-radius: 4px;
      font-size: 12px;
    }
    
    .connected {
      background-color: #2e7d32;
    }
    
    .disconnected {
      background-color: #c62828;
    }

    .nav {
      margin: 0 0 20px 0;
    }

    .nav a {
      color: var(--text-color);
      margin-right: 16px;
      text-decoration: none;
    }

    .nav a:hover {
      text-decoration: underline;
    }
  </style>
</head>
<body>
  <div class="container">
    <div style="display: flex; justify-content: space-between; align-items: center;">
      <h1>Antenna Tuner Control</h1>
      <div id="connectionStatus" class="connection-status disconnected">Connecting...</div>
    </div>
    <div class="nav">
      <a href="/">Home</a>
      <a href="/calibration">Calibration</a>
    </div>
)";
}

String getHtmlFooter() {
  return R"(
  </div>
  <script>
    // WebSocket connection
    let socket;
    let selectedFreq = '';

    function initWebSocket() {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const wsUrl = protocol + '//' + window.location.hostname + ':81';
      socket = new WebSocket(wsUrl);
      
      socket.onopen = function(event) {
        console.log('WebSocket connected');
        document.getElementById('connectionStatus').textContent = 'Web OK';
        document.getElementById('connectionStatus').className = 'connection-status connected';
      };
      
      socket.onmessage = function(event) {
        const data = JSON.parse(event.data);
        console.log('Received:', data);
        
        if (data.type === 'status') {
          // Update position and frequency on main page
          const posElement = document.getElementById('position');
          const freqElement = document.getElementById('currentFrequency');
          const stateElement = document.getElementById('state');
          if (posElement) posElement.textContent = data.position;
          if (freqElement) freqElement.textContent = data.frequency.toFixed(2);
          if (stateElement) stateElement.textContent = data.state;
          
          const calibPosElement = document.getElementById('currentPos');
          const calibFreqElement = document.getElementById('currentFreqCalib');
          if (calibPosElement) calibPosElement.textContent = data.position;
          if (calibFreqElement) calibFreqElement.textContent = data.frequency.toFixed(2);
        } else if (data.type === 'calibration') {
          // Update calibration table
          const table = document.getElementById('calibrationTable');
          if (table) {
            table.innerHTML = '';
            data.points.forEach(point => {
              const row = table.insertRow();
              row.insertCell(0).textContent = point.position;
              row.insertCell(1).textContent = point.frequency.toFixed(2);
            });
          }
        } else if (data.type === 'calibrate_result') {
          const feedback = document.getElementById('calibrationFeedback');
          if (feedback) {
            feedback.textContent = data.message || (data.success ? 'Point saved' : 'Calibration failed');
            feedback.style.backgroundColor = data.success ? '#2E7D32' : '#8B0000';
            feedback.style.display = 'block';
            setTimeout(() => { feedback.style.display = 'none'; }, data.success ? 3000 : 6000);
          }
        }
      };
      
      socket.onclose = function(event) {
        console.log('WebSocket disconnected, attempting to reconnect...');
        document.getElementById('connectionStatus').textContent = 'Web down';
        document.getElementById('connectionStatus').className = 'connection-status disconnected';
        setTimeout(initWebSocket, 2000);
      };
      
      socket.onerror = function(error) {
        console.error('WebSocket error:', error);
        document.getElementById('connectionStatus').textContent = 'Web error';
        document.getElementById('connectionStatus').className = 'connection-status disconnected';
      };
    }
    
    // Initialize WebSocket when page loads
    window.addEventListener('load', function() {
      initWebSocket();
      
      // Handle move form submission
      const moveForm = document.getElementById('moveForm');
      if (moveForm) {
        moveForm.addEventListener('submit', function(event) {
          event.preventDefault();
          
          const frequencyInput = document.getElementById('targetFrequency');
          console.log('Frequency input element:', frequencyInput);
          
          if (!frequencyInput) {
            console.error('Could not find frequency input element');
            showMoveFeedback('Internal error: input not found', false);
            return;
          }
          
          const frequencyStr = frequencyInput.value.trim();
          console.log('Frequency string:', frequencyStr);
          
          // Check if empty
          if (frequencyStr === '') {
            showMoveFeedback('Please enter a frequency', false);
            return;
          }
          
          // Parse to number
          const frequency = parseFloat(frequencyStr);
          console.log('Parsed frequency:', frequency);
          
          // Check if valid number and positive
          if (isNaN(frequency) || frequency <= 0) {
            showMoveFeedback('Please enter a valid frequency', false);
            return;
          }
          
          // Send request using URL-encoded form data
          const body = 'frequency=' + encodeURIComponent(frequency);
          console.log('Sending request with body:', body);
          
          fetch('/move', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/x-www-form-urlencoded'
            },
            body: body
          })
          .then(response => {
            console.log('Response status:', response.status);
            return response.json();
          })
          .then(data => {
            console.log('Response data:', data);
            if (data.status === 'ok') {
              showMoveFeedback(data.message, true);
              frequencyInput.value = '';
            } else {
              showMoveFeedback(data.message, false);
            }
          })
          .catch(error => {
            console.error('Error:', error);
            showMoveFeedback('Error communicating with server', false);
          });
        });
      }
      
      // Handle Move to button
      const moveToBtn = document.getElementById('moveToBtn');
      if (moveToBtn) {
        moveToBtn.addEventListener('click', function() {
          const freqSelect = document.getElementById('freqSelect');
          if (freqSelect) {
            const frequency = parseFloat(freqSelect.value);
            if (!isNaN(frequency) && frequency > 0) {
              fetch('/move', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: 'frequency=' + frequency
              })
              .then(response => response.json())
              .then(data => {
                if (data.status === 'ok') {
                  showCalibrationFeedback(data.message, true);
                } else {
                  showCalibrationFeedback(data.message, false);
                }
              })
              .catch(error => {
                console.error('Error:', error);
                showCalibrationFeedback('Error communicating with server', false);
              });
            }
          }
        });
      }
      
      // Save dropdown selection
      const freqSelect = document.getElementById('freqSelect');
      if (freqSelect) {
        freqSelect.addEventListener('change', function() {
          selectedFreq = this.value;
        });
      }
      
      // Calibrate button handler
      const calibrateBtn = document.getElementById('calibrateBtn');
      if (calibrateBtn) {
        calibrateBtn.addEventListener('click', function() {
          const freq = document.getElementById('freqSelect').value;
          if (freq && socket && socket.readyState === WebSocket.OPEN) {
            socket.send('{"calibrate":' + freq + '}');
          }
        });
      }
    });
    
    function showMoveFeedback(message, success) {
      const feedback = document.getElementById('moveFeedback');
      if (feedback) {
        feedback.textContent = message;
        feedback.style.backgroundColor = success ? '#2E7D32' : '#8B0000';
        feedback.style.display = 'block';
        setTimeout(() => { feedback.style.display = 'none'; }, 3000);
      }
    }
    
    function showCalibrationFeedback(message, success) {
      const feedback = document.getElementById('calibrationFeedback');
      if (feedback) {
        feedback.textContent = message;
        feedback.style.backgroundColor = success ? '#2E7D32' : '#8B0000';
        feedback.style.display = 'block';
        setTimeout(() => { feedback.style.display = 'none'; }, 3000);
      }
    }
    
    // Function to send movement commands without page reload
    function moveMotor(steps) {
      console.log("Sending move command: " + steps);
      fetch('/calibration/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'steps=' + steps
      })
      .then(response => response.json())
      .then(data => {
        console.log('Move response:', data);
        // Position will update via WebSocket
      })
      .catch(error => console.error('Error moving motor:', error));
    }
    
    // Function to stop the motor
    function stopMotor() {
      console.log("Sending stop command");
      fetch('/calibration/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'stop=true'
      })
      .then(response => response.json())
      .then(data => {
        console.log('Stop response:', data);
        // Position will update via WebSocket
      })
      .catch(error => console.error('Error stopping motor:', error));
    }
    
    // Function to home the motor
    function homeMotor() {
      console.log("Sending home command");
      fetch('/calibration/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: 'home=true'
      })
      .then(response => response.json())
      .then(data => {
        console.log('Home response:', data);
        // Position will update via WebSocket
      })
      .catch(error => console.error('Error homing motor:', error));
    }
  </script>
</body>
</html>
)";
}

void handleRoot() {
  String html = getHtmlHeader();

  html += "<div class=\"card\">";
  html += "<h2>Quick Control</h2>";
  html += "<div class=\"status\">";
  html += "<p>Position: <span id=\"position\">" + String(aktuellePosition) +
          "</span> steps</p>";
  html += "<p>Frequency: <span id=\"currentFrequency\">" +
          String(getFrequencyFromPosition(aktuellePosition), 2) +
          "</span> kHz</p>";
  html += "<p>State: <span id=\"state\">" + getStateText(currentMotorState) +
          "</span></p>";
  html += "</div>";
  html += "<form id=\"moveForm\">";
  html += "<label for=\"targetFrequency\">Target Frequency (kHz):</label>";
  html += "<input type=\"number\" id=\"targetFrequency\" name=\"frequency\" "
          "class=\"input\" step=\"0.01\">";
  html += "<button type=\"submit\" class=\"button\">Move to Frequency</button>";
  html += "</form>";
  html += "<div id=\"moveFeedback\" style=\"margin-top: 10px; padding: 10px; "
          "background-color: #333; border-radius: 4px; display: none;\"></div>";
  html += "</div>";

  html += "<div class=\"card\">";
  html += "<h2>Calibration</h2>";
  html += "<p>Start a non-destructive calibration process. Your existing table "
          "will be preserved.</p>";
  html += "<form action=\"/calibration/start\" method=\"post\">";
  html += "<button type=\"submit\" class=\"button\">Start Calibration "
          "Process</button>";
  html += "</form>";
  html += "</div>";

  html += getHtmlFooter();
  server.send(200, "text/html", html);
}

void handleStatus() {
  String json = "{";
  json += "\"position\":" + String(aktuellePosition) + ",";
  json +=
      "\"frequency\":" + String(getFrequencyFromPosition(aktuellePosition), 2) +
      ",";
  json += "\"state\":\"" + getStateText(currentMotorState) + "\",";
  json += "\"calibrationPoints\":" + String(kalibrierTabelle.size()) + ",";
  json += "\"btLink\":\"" + bleStateText() + "\",";
  json += "\"btFrequency\":" +
          String(bleRigFreqHz > 0 ? bleRigFreqHz / 1000.0f : 0, 2) + ",";
  json += "\"btTx\":" + String(bleTxActive ? "true" : "false") + ",";
  json += "\"btReady\":" + String(bleReady ? "true" : "false") + ",";
  json += "\"btMac\":\"";
  json += bleInitialized ? SerialBT.getBtAddressString() : String("--");
  json += "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleMove() {
  Serial.println("[WEB] Move request received");
  String json = "{\"status\":\"error\", \"message\":\"Invalid request\"}";

  if (server.hasArg("frequency")) {
    float frequency = server.arg("frequency").toFloat();
    Serial.println("[WEB] Move to frequency: " + String(frequency) + " kHz");

    if (frequency > 0) {
      if (!isMotorActive()) {
        moveToFrequency(frequency);
        json = "{\"status\":\"ok\", \"message\":\"Moving to " +
               String(frequency) + " kHz\"}";
      } else {
        json = "{\"status\":\"error\", \"message\":\"Motor busy\"}";
        Serial.println("[WEB] Move rejected - motor busy");
      }
    } else {
      json = "{\"status\":\"error\", \"message\":\"Invalid frequency\"}";
      Serial.println("[WEB] Move rejected - invalid frequency");
    }
  }

  server.send(200, "application/json", json);
}

void handleCalibration() {
  String html = getHtmlHeader();

  html += "<div class=\"card\">";
  html += "<h2>Calibration</h2>";
  html += "<p>Current position: <span id=\"currentPos\">" +
          String(aktuellePosition) + "</span> steps (<span id=\"currentFreqCalib\">" +
          String(getFrequencyFromPosition(aktuellePosition), 2) + "</span> kHz)</p>";
  html += "<p>Select a frequency and click 'Calibrate' to save current "
          "position.</p>";
  html +=
      "<div id=\"calibrationFeedback\" style=\"margin: 10px 0; padding: 10px; "
      "background-color: #333; border-radius: 4px; display: none;\"></div>";

  // Frequency selection with Calibrate and Move to buttons
  html += "<div style=\"margin: 15px 0; display: flex; align-items: center; "
          "gap: 10px;\">";
  html += "<button id=\"calibrateBtn\" class=\"button\">Calibrate</button>";
  html += "<label for=\"freqSelect\">Select Frequency:</label>";
  html += "<select id=\"freqSelect\" class=\"input\" style=\"width: auto;\">";

  // Generate the list of all frequencies for the dropdown
  std::set<float> allFreqs;
  for (const auto &band : hamBands) {
    for (long freq = band.start; freq <= band.end; freq += 50) {
      allFreqs.insert((float)freq);
    }
  }
  long startFreq = 500;
  for (long freq = startFreq; freq <= 30000; freq += 500) {
    bool inBand = false;
    for (const auto &band : hamBands) {
      if (freq >= band.start && freq <= band.end) {
        inBand = true;
        break;
      }
    }
    if (!inBand) {
      allFreqs.insert((float)freq);
    }
  }

  std::vector<float> sortedFreqs(allFreqs.begin(), allFreqs.end());
  std::sort(sortedFreqs.begin(), sortedFreqs.end());

  for (size_t i = 0; i < sortedFreqs.size(); ++i) {
    float targetFreq = sortedFreqs[i];
    html += "<option value=\"" + String(targetFreq, 2) + "\">" +
            String(targetFreq, 2) + " kHz</option>";
  }

  html += "</select>";
  html += "<button id=\"moveToBtn\" class=\"button\" style=\"background-color: "
          "#2196F3;\">Move to</button>";
  html += "</div>";

  // Manual movement controls in a single row
  html += "<div style=\"margin: 20px 0;\">";
  html += "<div style=\"display: flex; justify-content: center; align-items: "
          "center; gap: 5px;\">";

  // Movement buttons
  html +=
      "<button class=\"button\" onclick=\"moveMotor(-1000)\">-1000</button>";
  html += "<button class=\"button\" onclick=\"moveMotor(-100)\">-100</button>";
  html += "<button class=\"button\" onclick=\"moveMotor(-10)\">-10</button>";
  html += "<button class=\"button\" onclick=\"moveMotor(-1)\">-1</button>";

  // Home button above Stop
  html += "<div style=\"display: flex; flex-direction: column; align-items: "
          "center;\">";
  html += "<button class=\"button\" style=\"background-color: #ff9800; "
          "margin-bottom: 5px;\" onclick=\"homeMotor()\">Home</button>";
  html += "<button class=\"button button-danger\" "
          "onclick=\"stopMotor()\">STOP</button>";
  html += "</div>";

  // Positive movement buttons
  html += "<button class=\"button\" onclick=\"moveMotor(1)\">1</button>";
  html += "<button class=\"button\" onclick=\"moveMotor(10)\">10</button>";
  html += "<button class=\"button\" onclick=\"moveMotor(100)\">100</button>";
  html += "<button class=\"button\" onclick=\"moveMotor(1000)\">1000</button>";

  html += "</div>";
  html += "</div>";

  // Simple table for calibration data
  html += "<div style=\"margin-top: 20px;\">";
  html += "<h3>Calibration Points</h3>";
  html += "<table>";
  html += "<tr><th>Position</th><th>Frequency (kHz)</th></tr>";
  html += "<tbody id=\"calibrationTable\">";

  // Add existing calibration points
  for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
    html += "<tr>";
    html += "<td>" + String(kalibrierTabelle[i].position) + "</td>";
    html += "<td>" + String(kalibrierTabelle[i].frequenz, 2) + "</td>";
    html += "</tr>";
  }

  html += "</tbody>";
  html += "</table>";
  html += "</div>";

  html += "<hr><div style=\"margin-top: 20px;\">";
  html += "<form action=\"/calibration/save_and_exit\" method=\"post\" "
          "style=\"display:inline;\">";
  html += "<button type=\"submit\" class=\"button\">Save & Exit</button>";
  html += "</form>";
  html += "<form action=\"/calibration/abort\" method=\"post\" "
          "style=\"display:inline; margin-left: 10px;\" onsubmit=\"return "
          "confirm('Discard calibration changes and restore the previous table?')\">";
  html += "<button type=\"submit\" class=\"button button-danger\">Abort "
          "Calibration</button>";
  html += "</form>";
  html += "<form action=\"/reset\" method=\"post\" "
          "style=\"display:inline; margin-left: 10px;\" onsubmit=\"return "
          "confirm('Reset calibration table to the two default points?')\">";
  html += "<button type=\"submit\" class=\"button button-danger\">Reset "
          "Calibration</button>";
  html += "</form>";
  html += "</div>";

  html += "</div>";

  html += getHtmlFooter();
  server.send(200, "text/html", html);
}

void handleCalibrationControl() {
  Serial.println("[WEB] Calibration control request received");

  if (!server.hasArg("steps") && !server.hasArg("stop") &&
      !server.hasArg("home")) {
    server.send(400, "application/json",
                "{\"status\":\"error\", \"message\":\"Bad Request\"}");
    return;
  }

  // Handle stop command
  if (server.hasArg("stop") && server.arg("stop") == "true") {
    Serial.println("[WEB] Stop command received");
    if (isMotorActive()) {
      stopMotor();
      server.send(200, "application/json",
                  "{\"status\":\"ok\", \"message\":\"Motor stopped\"}");
    } else {
      server.send(400, "application/json",
                  "{\"status\":\"error\", \"message\":\"Motor not active\"}");
    }
    return;
  }

  // Handle home command
  if (server.hasArg("home") && server.arg("home") == "true") {
    Serial.println("[WEB] Home command received");
    if (!isMotorActive()) {
      startHoming();
      server.send(200, "application/json",
                  "{\"status\":\"ok\", \"message\":\"Homing started\"}");
      Serial.println("[WEB] Homing started");
    } else {
      server.send(400, "application/json",
                  "{\"status\":\"error\", \"message\":\"Motor busy\"}");
      Serial.println("[WEB] Homing rejected - motor busy");
    }
    return;
  }

  // Handle movement command
  if (server.hasArg("steps")) {
    long steps = server.arg("steps").toInt();
    if (steps != 0) {
      Serial.println("[WEB] Move command received: " + String(steps) +
                     " steps");

      // Allow movement regardless of current state, as long as motor is not
      // active
      if (!isMotorActive()) {
        startMove(steps, SPEED_NORMAL);

        server.send(200, "application/json",
                    "{\"status\":\"ok\", \"message\":\"Moving\"}");
      } else {
        server.send(
            400, "application/json",
            "{\"status\":\"error\", \"message\":\"Motor already active\"}");
        Serial.println("[WEB] Move rejected - motor already active");
      }
      return;
    }
  }

  server.send(
      400, "application/json",
      "{\"status\":\"error\", \"message\":\"No valid command specified\"}");
}

void handleCalibrationPoint() {
  Serial.println("[WEB] Calibration point save request received");

  if (!server.hasArg("freq")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  float targetFreq = server.arg("freq").toFloat();
  Serial.println("[WEB] Saving calibration point at frequency: " +
                 String(targetFreq) + " kHz");

  // Don't move the motor, just save the current position to the selected
  // frequency
  if (!isMotorActive()) {
    ensureCalibrationSession();
    ValidateResult result = saveSpeicherpunkt(aktuellePosition, targetFreq);
    bool success = (result != VALIDATE_CRITICAL && result != VALIDATE_RESET);
    server.send(success ? 200 : 400, "text/plain", statusMeldung);
  } else {
    server.send(400, "text/plain", "Motor is busy");
    Serial.println("[WEB] Calibration point save rejected - motor busy");
  }
}

void handleCalibrationSave() {
  Serial.println("[WEB] Calibration save request received");

  if (!webCalibration.active) {
    server.send(400, "text/plain", "Calibration not active");
    return;
  }

  server.sendHeader("Location", "/calibration");
  server.send(302, "text/plain", "Redirecting...");
}

void handleCalibrationSaveAndExit() {
  Serial.println("[WEB] Calibration save and exit request received");

  if (webCalibration.active) {
    commitCalibrationSession();
  }
  saveCurrentPosition();
  setStatusMessage("Kalibrierung gespeichert", 2000);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirecting...");
}

void handleCalibrationAbort() {
  Serial.println("[WEB] Calibration abort request received");

  if (webCalibration.active) {
    restoreCalibrationBackup();
    setStatusMessage("Kalibrierung verworfen", 2000);
  }
  stopMotor();
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirecting...");
}

void handleReset() {
  Serial.println("[WEB] Reset request received");

  resetKalibrierung();

  String html = getHtmlHeader();
  html += "<div class=\"card\"><h2>Reset Complete</h2><p>Calibration table has "
          "been reset.</p>";
  html += "<p><a href=\"/\">Back to main page</a></p></div>";
  html += getHtmlFooter();
  server.send(200, "text/html", html);
}

void handleCalibrationData() {
  String json = "{";
  json += "\"calibrationPoints\":[";

  // Add existing calibration points to JSON array
  for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
    if (i > 0)
      json += ",";
    json += "{\"position\":" + String(kalibrierTabelle[i].position) +
            ",\"frequency\":" + String(kalibrierTabelle[i].frequenz, 2) + "}";
  }

  json += "]}";
  server.send(200, "application/json", json);
}

void loadBtSettings() {
  uint8_t magic = EEPROM.read(ADRESSE_BT_SETTINGS);
  uint8_t tracking = EEPROM.read(ADRESSE_BT_SETTINGS + 1);
  uint8_t deadband = EEPROM.read(ADRESSE_BT_SETTINGS + 2);
  uint8_t autoStart = EEPROM.read(ADRESSE_BT_SETTINGS + 3);

  if (magic == BT_SETTINGS_MAGIC || magic == BT_SETTINGS_MAGIC_V2) {
    btSettings.magic = BT_SETTINGS_MAGIC;
    btSettings.trackingEnabled = tracking ? 1 : 0;
    btSettings.btAutoStart = autoStart ? 1 : 0;
    btSettings.deadbandKhz = deadband;
    bool dirty = (magic != BT_SETTINGS_MAGIC);
    if (btSettings.deadbandKhz < BT_DEADBAND_MIN_KHZ ||
        btSettings.deadbandKhz > BT_DEADBAND_MAX_KHZ) {
      btSettings.deadbandKhz = BT_DEADBAND_DEFAULT_KHZ;
      dirty = true;
    }
    if (dirty) {
      saveBtSettings();
    }
    Serial.println("[BT] Settings loaded: tracking=" +
                   String(btSettings.trackingEnabled ? "on" : "off") +
                   " deadband=" + String(btSettings.deadbandKhz) +
                   " kHz autoStart=" +
                   String(btSettings.btAutoStart ? "on" : "off"));
    return;
  }

  if (magic == BT_SETTINGS_MAGIC_V1) {
    btSettings.magic = BT_SETTINGS_MAGIC;
    btSettings.trackingEnabled = tracking ? 1 : 0;
    btSettings.deadbandKhz = deadband;
    btSettings.btAutoStart = 1;
    if (btSettings.deadbandKhz < BT_DEADBAND_MIN_KHZ ||
        btSettings.deadbandKhz > BT_DEADBAND_MAX_KHZ) {
      btSettings.deadbandKhz = BT_DEADBAND_DEFAULT_KHZ;
    }
    saveBtSettings();
    Serial.println("[BT] Migrated V1 settings, tracking preserved");
    return;
  }

  btSettings.magic = BT_SETTINGS_MAGIC;
  btSettings.trackingEnabled = 0;
  btSettings.deadbandKhz = BT_DEADBAND_DEFAULT_KHZ;
  btSettings.btAutoStart = 0;
  saveBtSettings();
  Serial.println("[BT] Settings reset to defaults (tracking off, BT off)");
}

void saveBtSettings() {
  btSettings.magic = BT_SETTINGS_MAGIC;
  btSettings.trackingEnabled = btSettings.trackingEnabled ? 1 : 0;
  btSettings.btAutoStart = btSettings.btAutoStart ? 1 : 0;
  if (btSettings.deadbandKhz < BT_DEADBAND_MIN_KHZ) {
    btSettings.deadbandKhz = BT_DEADBAND_MIN_KHZ;
  } else if (btSettings.deadbandKhz > BT_DEADBAND_MAX_KHZ) {
    btSettings.deadbandKhz = BT_DEADBAND_MAX_KHZ;
  }
  EEPROM.write(ADRESSE_BT_SETTINGS, btSettings.magic);
  EEPROM.write(ADRESSE_BT_SETTINGS + 1, btSettings.trackingEnabled);
  EEPROM.write(ADRESSE_BT_SETTINGS + 2, btSettings.deadbandKhz);
  EEPROM.write(ADRESSE_BT_SETTINGS + 3, btSettings.btAutoStart);
  EEPROM.commit();
}

void toggleTracking() {
  btSettings.trackingEnabled = btSettings.trackingEnabled ? 0 : 1;
  saveBtSettings();
  if (btSettings.trackingEnabled) {
    lastTrackedKhz = -1;
    setStatusMessage("Tracking AN", 2000);
  } else {
    setStatusMessage("Tracking AUS", 2000);
  }
  Serial.println("[BT] Tracking " +
                 String(btSettings.trackingEnabled ? "on" : "off"));
}

uint32_t civBcdToHz(const uint8_t *bcd) {
  uint32_t hz = 0;
  hz += (bcd[0] & 0x0F) * 1UL;
  hz += ((bcd[0] >> 4) & 0x0F) * 10UL;
  hz += (bcd[1] & 0x0F) * 100UL;
  hz += ((bcd[1] >> 4) & 0x0F) * 1000UL;
  hz += (bcd[2] & 0x0F) * 10000UL;
  hz += ((bcd[2] >> 4) & 0x0F) * 100000UL;
  hz += (bcd[3] & 0x0F) * 1000000UL;
  hz += ((bcd[3] >> 4) & 0x0F) * 10000000UL;
  hz += (bcd[4] & 0x0F) * 100000000UL;
  hz += ((bcd[4] >> 4) & 0x0F) * 1000000000UL;
  return hz;
}

void civHzToBcd(uint32_t hz, uint8_t *bcd) {
  bcd[0] = (uint8_t)((hz / 1UL) % 10) | (uint8_t)(((hz / 10UL) % 10) << 4);
  bcd[1] = (uint8_t)((hz / 100UL) % 10) | (uint8_t)(((hz / 1000UL) % 10) << 4);
  bcd[2] =
      (uint8_t)((hz / 10000UL) % 10) | (uint8_t)(((hz / 100000UL) % 10) << 4);
  bcd[3] = (uint8_t)((hz / 1000000UL) % 10) |
           (uint8_t)(((hz / 10000000UL) % 10) << 4);
  bcd[4] = (uint8_t)((hz / 100000000UL) % 10) |
           (uint8_t)(((hz / 1000000000UL) % 10) << 4);
}

uint16_t civBcdWord(uint8_t hi, uint8_t lo) {
  return (uint16_t)((hi & 0x0F) * 100 + ((lo >> 4) & 0x0F) * 10 + (lo & 0x0F));
}

float swrFromRaw(uint16_t raw) {
  float swr;
  if (raw <= 48) {
    swr = 1.0f + (0.5f * raw) / 48.0f;
  } else if (raw <= 80) {
    swr = 1.5f + (0.5f * (raw - 48)) / 32.0f;
  } else if (raw <= 120) {
    swr = 2.0f + (1.0f * (raw - 80)) / 40.0f;
  } else {
    swr = 3.0f + (3.0f * (raw - 120)) / 120.0f;
    if (swr > 9.9f) {
      swr = 9.9f;
    }
  }
  return swr;
}

String formatSwr(uint16_t raw) { return String(swrFromRaw(raw), 1); }

int autoCalSpeedForSwr(uint16_t raw) {
  float swr = swrFromRaw(raw);
  if (swr < 1.0f) {
    swr = 1.0f;
  }
  if (swr > 6.0f) {
    swr = 6.0f;
  }
  float t = (swr - 1.0f) / 5.0f;
  int spd = (int)round((float)AUTOCAL_SPEED_NEAR_MS -
                       t * (float)(AUTOCAL_SPEED_NEAR_MS - SPEED_NORMAL));
  if (spd < SPEED_NORMAL) {
    spd = SPEED_NORMAL;
  }
  if (spd > AUTOCAL_SPEED_NEAR_MS) {
    spd = AUTOCAL_SPEED_NEAR_MS;
  }
  return spd;
}

bool autoCalPastDip() {
  if (!autoCalSawHighSwr || autoCalMinSwr > AUTOCAL_SWR_GOOD) {
    return false;
  }
  if ((autoCalMinPos - autoCalSweepStartPos) < 8) {
    return false;
  }
  if (!bleSwrValid) {
    return false;
  }
  return swrFromRaw(bleSwrRaw) >= AUTOCAL_SWR_FAR;
}

void autoCalApplySweepSpeed() {
  if (!isMotorActive() || !bleSwrValid) {
    return;
  }
  int spd = autoCalSpeedForSwr(bleSwrRaw);
  // The fine pass already sits on the dip, so it must never run at travel
  // speed: the reading it steers by is older than the ground it would cover.
  if (autoCalState == AUTOCAL_FINE && spd < SPEED_CAL) {
    spd = SPEED_CAL;
  }
  currentSpeed = spd;
}

void civParseBuffer(const uint8_t *data, size_t length) {
  for (size_t i = 0; i + 4 < length; ++i) {
    if (data[i] == CIV_PREAMBLE && data[i + 1] == 0xF1 && data[i + 2] == 0x00) {
      uint8_t cmd = data[i + 3];
      if (cmd == 0x64) {
        blePairGranted = true;
      }
    }

    if (data[i] != CIV_PREAMBLE || data[i + 1] != CIV_PREAMBLE) {
      continue;
    }
    size_t end = i + 2;
    while (end < length && data[end] != CIV_END) {
      end++;
    }
    if (end >= length || (end - i) < 5) {
      continue;
    }
    size_t p = i + 2;
    while (p + 2 <= end && data[p] == CIV_PREAMBLE) {
      p++;
    }
    if (p + 2 > end) {
      continue;
    }
    bleGotCiv = true;
    if (civMonitor) {
      String hex = "[CIV] RX";
      for (size_t k = i; k <= end; ++k) {
        hex += (data[k] < 0x10) ? " 0" : " ";
        hex += String(data[k], HEX);
      }
      hex.toUpperCase();
      Serial.println(hex);
    }
    uint8_t cmd = data[p + 2];
    if ((cmd == CIV_CMD_READ_FREQ || cmd == CIV_CMD_FREQ_TRANSCEIVE) &&
        (end >= p + 8)) {
      uint32_t hz = civBcdToHz(&data[p + 3]);
      if (hz >= 100000UL && hz <= 470000000UL) {
        bleRigFreqHz = hz;
      }
    } else if (cmd == CIV_CMD_PTT && end >= p + 5 && data[p + 3] == 0x00) {
      bleTxActive = (data[p + 4] == 0x01);
    } else if (cmd == CIV_CMD_PTT && end >= p + 5 &&
               data[p + 3] == CIV_SUB_TUNER) {
      if (data[p + 4] != bleTunerState) {
        bleTunerState = data[p + 4];
        Serial.println("[CIV] Tuner-Status " + String(bleTunerState) +
                       " (0=AUS 1=EIN 2=Tune)");
      }
    } else if (cmd == CIV_CMD_READ_MODE && end >= p + 4) {
      bleSavedMode = data[p + 3];
      if (end >= p + 5 && data[p + 4] != CIV_END) {
        bleSavedFilter = data[p + 4];
      }
    } else if (cmd == CIV_CMD_LEVEL && end >= p + 6 &&
               data[p + 3] == CIV_SUB_RF_POWER) {
      bleSavedPowerHi = data[p + 4];
      bleSavedPowerLo = data[p + 5];
    } else if (cmd == CIV_CMD_METER && end >= p + 6 &&
               data[p + 3] == CIV_SUB_SWR) {
      bleSwrRaw = civBcdWord(data[p + 4], data[p + 5]);
      // The rig sampled somewhere between our request and this reply.
      bleSwrPos = (bleSwrReqPos + aktuellePosition) / 2;
      bleSwrValid = true;
    } else if (cmd == CIV_CMD_METER && end >= p + 6 &&
               data[p + 3] == CIV_SUB_PO) {
      blePoRaw = civBcdWord(data[p + 4], data[p + 5]);
    }
  }
}

void civFeedBytes(const uint8_t *data, size_t length) {
  if (!data || length == 0) {
    return;
  }
  if (civRxLen + length > sizeof(civRxBuf)) {
    civRxLen = 0;
  }
  memcpy(civRxBuf + civRxLen, data, length);
  civRxLen += length;

  size_t start = 0;
  while (start + 5 <= civRxLen) {
    if (!(civRxBuf[start] == CIV_PREAMBLE &&
          (civRxBuf[start + 1] == CIV_PREAMBLE ||
           civRxBuf[start + 1] == 0xF1))) {
      start++;
      continue;
    }
    size_t end = start + 2;
    while (end < civRxLen && civRxBuf[end] != CIV_END) {
      end++;
    }
    if (end >= civRxLen) {
      break;
    }
    civParseBuffer(civRxBuf + start, end - start + 1);
    start = end + 1;
  }
  if (start > 0 && start <= civRxLen) {
    memmove(civRxBuf, civRxBuf + start, civRxLen - start);
    civRxLen -= start;
  }
}

bool bleWriteRaw(const uint8_t *data, size_t len) {
  if (!SerialBT.hasClient() || !data || len == 0) {
    return false;
  }
  return SerialBT.write(data, len) == len;
}

bool bleSendCiv(const uint8_t *payload, size_t payloadLen) {
  uint8_t frame[16];
  size_t n = 0;
  frame[n++] = CIV_PREAMBLE;
  frame[n++] = CIV_PREAMBLE;
  frame[n++] = CIV_IC705_ADDR;
  frame[n++] = CIV_CTRL_ADDR;
  for (size_t i = 0; i < payloadLen && n < sizeof(frame) - 1; ++i) {
    frame[n++] = payload[i];
  }
  frame[n++] = CIV_END;
  return bleWriteRaw(frame, n);
}

void loadBtPeer() {
  uint8_t magic = EEPROM.read(ADRESSE_BT_PEER);
  if (magic != BT_PEER_MAGIC && magic != BT_PEER_MAGIC_V2) {
    blePeerValid = false;
    return;
  }
  for (int i = 0; i < 6; i++) {
    blePeerMac[i] = EEPROM.read(ADRESSE_BT_PEER + 1 + i);
  }
  blePeerValid = (blePeerMac[0] | blePeerMac[1] | blePeerMac[2] | blePeerMac[3] |
                  blePeerMac[4] | blePeerMac[5]) != 0;
  // A V1 record predates the channel byte, so keep probing until one works.
  blePeerChannel = 0;
  if (magic == BT_PEER_MAGIC_V2) {
    uint8_t ch = EEPROM.read(ADRESSE_BT_PEER + 7);
    if (ch >= 1 && ch <= 30) {
      blePeerChannel = ch;
    }
  }
  if (blePeerValid && blePeerChannel > 0) {
    Serial.println("[BT] Peer-Kanal aus EEPROM: " + String(blePeerChannel));
  }
}

void saveBtPeer() {
  EEPROM.write(ADRESSE_BT_PEER, BT_PEER_MAGIC_V2);
  for (int i = 0; i < 6; i++) {
    EEPROM.write(ADRESSE_BT_PEER + 1 + i, blePeerMac[i]);
  }
  EEPROM.write(ADRESSE_BT_PEER + 7, blePeerChannel);
  EEPROM.commit();
}

void bleRememberPeer(const uint8_t *mac) {
  if (!mac) {
    return;
  }
  bool same = blePeerValid;
  if (same) {
    for (int i = 0; i < 6; i++) {
      if (blePeerMac[i] != mac[i]) {
        same = false;
        break;
      }
    }
  }
  memcpy(blePeerMac, mac, 6);
  blePeerValid = true;
  if (!same) {
    // Different radio, so the stored channel says nothing about this one.
    blePeerChannel = 0;
    blePeerChannelTried = false;
    saveBtPeer();
    Serial.printf("[BT] Peer gespeichert %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
}

// Called once CI-V actually arrives, which is the only proof the channel is
// the right one.
void bleConfirmChannel() {
  blePeerChannelTried = false;
  if (bleActiveChannel == 0 || bleActiveChannel == blePeerChannel) {
    return;
  }
  blePeerChannel = bleActiveChannel;
  if (blePeerValid) {
    saveBtPeer();
  }
  Serial.println("[BT] CI-V-Kanal gemerkt: " + String(blePeerChannel));
}

bool bleSelectPeerMac(uint8_t *outMac) {
  if (blePeerValid) {
    memcpy(outMac, blePeerMac, 6);
    return true;
  }
  int n = SerialBT.getNumberOfBondedDevices();
  if (n <= 0) {
    return false;
  }
  if (n > 8) {
    n = 8;
  }
  esp_bd_addr_t list[8];
  if (SerialBT.getBondedDevices(n, list) <= 0) {
    return false;
  }
  int pick = 0;
  for (int i = 0; i < n; i++) {
    if (list[i][0] == 0x00 && list[i][1] == 0x90 && list[i][2] == 0xC7) {
      pick = i;
      break;
    }
  }
  memcpy(outMac, list[pick], 6);
  bleRememberPeer(outMac);
  return true;
}

void bleSppCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (!param) {
    return;
  }
  if (event == ESP_SPP_SRV_OPEN_EVT && param->srv_open.status == ESP_SPP_SUCCESS) {
    bleRememberPeer(param->srv_open.rem_bda);
  } else if (event == ESP_SPP_OPEN_EVT && param->open.status == ESP_SPP_SUCCESS) {
    bleRememberPeer(param->open.rem_bda);
  }
}

void btConfirmPin(uint32_t pin) {
  (void)pin;
  SerialBT.confirmReply(true);
}

bool btEnsureInit() {
  if (bleInitialized) {
    return true;
  }
  Serial.printf("[BT] Heap before SPP: %u\n", (unsigned)ESP.getFreeHeap());
  wifiStopForBt();
  loadBtPeer();
  SerialBT.setPin("0000", 4);
  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(btConfirmPin);
  SerialBT.register_callback(bleSppCallback);
  if (!SerialBT.begin(BT_SPP_NAME, true, true)) {
    strncpy(bleErrorMessage, "BT SPP init failed", sizeof(bleErrorMessage) - 1);
    bleLinkState = BLE_LINK_ERROR;
    Serial.println("[BT] SerialBT.begin failed");
    restoreWifiAp();
    return false;
  }
  bleInitialized = true;
  bleLinkState = BLE_LINK_IDLE;
  if (!btSettings.btAutoStart) {
    btSettings.btAutoStart = true;
    saveBtSettings();
    Serial.println("[BT] Auto-start saved to EEPROM");
  }
  SerialBT.setTimeout(20);
  Serial.print("[BT] SPP name ");
  Serial.print(BT_SPP_NAME);
  Serial.print(" MAC ");
  Serial.println(SerialBT.getBtAddressString());
  btReconnectAt = millis();
  return true;
}

void bleDropDeadLink(const char *reason) {
  Serial.print("[BT] ");
  Serial.println(reason);
  if (SerialBT.hasClient()) {
    SerialBT.disconnect();
  }
  bleGotCiv = false;
  bleRigFreqHz = 0;
  bleOutboundFails++;
  bleSppChannelTry++;
  int channelCount = (int)(sizeof(bleSppChannels) / sizeof(bleSppChannels[0]));
  if (bleSppChannelTry >= channelCount || bleOutboundFails >= 6) {
    bleOutboundEnabled = false;
    Serial.println("[BT] Auto-connect pausiert, warte auf 705");
  }
  btReconnectAt = millis() + 4000;
}

bool bleConnectPeer() {
  uint8_t mac[6];
  if (!bleSelectPeerMac(mac)) {
    return false;
  }
  int channelCount = (int)(sizeof(bleSppChannels) / sizeof(bleSppChannels[0]));
  bool usingRemembered = (blePeerChannel > 0 && !blePeerChannelTried);
  uint8_t channel;
  if (usingRemembered) {
    blePeerChannelTried = true;
    channel = blePeerChannel;
  } else {
    if (bleSppChannelTry < 0 || bleSppChannelTry >= channelCount) {
      bleSppChannelTry = 0;
    }
    channel = bleSppChannels[bleSppChannelTry];
  }
  bleActiveChannel = channel;
  Serial.printf("[BT] Auto-connect %02X:%02X:%02X:%02X:%02X:%02X ch %u%s\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                (unsigned)channel, usingRemembered ? " (gemerkt)" : "");
  bool ok = SerialBT.connect(mac, channel, ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER);
  if (ok) {
    Serial.println("[BT] Auto-connect OK, warte auf CI-V");
  } else {
    Serial.println("[BT] Auto-connect miss");
    if (usingRemembered) {
      // A refused connect says nothing about the channel, only that the rig is
      // not reachable yet. Keep the known-good one instead of rescanning.
      blePeerChannelTried = false;
    } else {
      bleSppChannelTry++;
      if (bleSppChannelTry >= channelCount) {
        bleOutboundEnabled = false;
        Serial.println("[BT] Auto-connect pausiert, warte auf 705");
      }
    }
  }
  return ok;
}

void blePumpRx() {
  if (!SerialBT.hasClient()) {
    return;
  }
  uint8_t buf[64];
  int n = SerialBT.available();
  if (n <= 0) {
    return;
  }
  if (n > (int)sizeof(buf)) {
    n = sizeof(buf);
  }
  n = SerialBT.readBytes(buf, n);
  if (n > 0) {
    if (!bleGotCiv) {
      Serial.print("[BT] RX");
      int show = n > 24 ? 24 : n;
      for (int i = 0; i < show; i++) {
        Serial.printf(" %02X", buf[i]);
      }
      Serial.println();
    }
    civFeedBytes(buf, (size_t)n);
  }
}

void bleRunAutoCalCiv() {
  AutoCalCiv req = autoCalCivReq;
  if (req == AUTOCAL_CIV_NONE) {
    return;
  }
  autoCalCivReq = AUTOCAL_CIV_NONE;
  autoCalCivDone = false;

  if (req == AUTOCAL_CIV_PREP) {
    auto snapshotWait = [](unsigned long ms) {
      unsigned long t = millis();
      while (millis() - t < ms) {
        blePumpRx();
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    };
    uint8_t freqRd[] = {CIV_CMD_READ_FREQ};
    bleSendCiv(freqRd, 1);
    snapshotWait(250);
    uint8_t modeRd[] = {CIV_CMD_READ_MODE};
    bleSendCiv(modeRd, 1);
    snapshotWait(250);
    uint8_t pwrRd[] = {CIV_CMD_LEVEL, CIV_SUB_RF_POWER};
    bleSendCiv(pwrRd, 2);
    snapshotWait(250);
    autoCalSavedFreqHz = bleRigFreqHz;
    autoCalSavedMode = bleSavedMode;
    autoCalSavedFilter = bleSavedFilter;
    autoCalSavedPowerHi = bleSavedPowerHi;
    autoCalSavedPowerLo = bleSavedPowerLo;
    Serial.println("[CAL] Radio merken: " + String(autoCalSavedFreqHz) +
                   " Hz  Mode " + String(autoCalSavedMode, HEX) + " FIL " +
                   String(autoCalSavedFilter, HEX) + " PWR " +
                   String(autoCalSavedPowerHi, HEX) + " " +
                   String(autoCalSavedPowerLo, HEX));
    uint8_t txMode[] = {CIV_CMD_SET_MODE, CIV_MODE_RTTY};
    bleSendCiv(txMode, 2);
    vTaskDelay(40 / portTICK_PERIOD_MS);
    uint8_t ptune[] = {CIV_CMD_LEVEL, CIV_SUB_RF_POWER, AUTOCAL_TX_PWR_HI,
                       AUTOCAL_TX_PWR_LO};
    bleSendCiv(ptune, 4);
  } else if (req == AUTOCAL_CIV_SET_FREQ) {
    uint8_t payload[6];
    payload[0] = CIV_CMD_SET_FREQ;
    civHzToBcd(autoCalTargetHz, &payload[1]);
    bleSendCiv(payload, 6);
  } else if (req == AUTOCAL_CIV_PTT_ON) {
    uint8_t cmd[] = {CIV_CMD_PTT, 0x00, 0x01};
    bleSendCiv(cmd, 3);
    bleSwrValid = false;
  } else if (req == AUTOCAL_CIV_PTT_OFF) {
    uint8_t cmd[] = {CIV_CMD_PTT, 0x00, 0x00};
    bleSendCiv(cmd, 3);
    autoCalPollSwr = false;
  } else if (req == AUTOCAL_CIV_RESTORE) {
    uint8_t pttOff[] = {CIV_CMD_PTT, 0x00, 0x00};
    bleSendCiv(pttOff, 3);
    vTaskDelay(80 / portTICK_PERIOD_MS);
    if (autoCalSavedFreqHz >= 100000UL) {
      uint8_t payload[6];
      payload[0] = CIV_CMD_SET_FREQ;
      civHzToBcd(autoCalSavedFreqHz, &payload[1]);
      bleSendCiv(payload, 6);
      vTaskDelay(80 / portTICK_PERIOD_MS);
    }
    if (autoCalSavedFilter >= 1 && autoCalSavedFilter <= 3) {
      uint8_t mode[] = {CIV_CMD_SET_MODE, autoCalSavedMode, autoCalSavedFilter};
      bleSendCiv(mode, 3);
    } else {
      uint8_t mode[] = {CIV_CMD_SET_MODE, autoCalSavedMode};
      bleSendCiv(mode, 2);
    }
    vTaskDelay(80 / portTICK_PERIOD_MS);
    uint8_t pwr[] = {CIV_CMD_LEVEL, CIV_SUB_RF_POWER, autoCalSavedPowerHi,
                     autoCalSavedPowerLo};
    bleSendCiv(pwr, 4);
    Serial.println("[CAL] Radio restore: " + String(autoCalSavedFreqHz) +
                   " Hz  Mode " + String(autoCalSavedMode, HEX) + " PWR " +
                   String(autoCalSavedPowerHi, HEX) + " " +
                   String(autoCalSavedPowerLo, HEX));
    autoCalPollSwr = false;
  }
  autoCalCivDone = true;
}

void bleTask(void *pvParameters) {
  (void)pvParameters;
  unsigned long lastPoll = 0;
  unsigned long lastPttPoll = 0;
  unsigned long lastSwrPoll = 0;
  for (;;) {
    if (bleCommand == BLE_CMD_START) {
      bleCommand = BLE_CMD_NONE;
      if (btEnsureInit()) {
        setStatusMessage("BT an", 2500);
        jumpToPage(PAGE_WEB_STATUS);
      }
    } else if (bleCommand == BLE_CMD_STOP) {
      bleCommand = BLE_CMD_NONE;
      btStopRadio();
      setStatusMessage("WLAN an", 2500);
    }

    bool linked = bleInitialized && SerialBT.hasClient();
    if (linked && !bleReady) {
      bleReady = true;
      bleReadyAt = millis();
      bleCivWatchAt = millis();
      bleGotCiv = false;
      bleRigFreqHz = 0;
      lastPoll = 0;
      bleLinkState = BLE_LINK_READY;
      Serial.println("[BT] IC-705 SPP connected");
      jumpToPage(PAGE_WEB_STATUS);
      setStatusMessage("IC-705 verbunden", 2000);
    } else if (!linked && bleReady) {
      bleReady = false;
      bleTxActive = false;
      bleRigFreqHz = 0;
      bleSwrValid = false;
      civRxLen = 0;
      bleGotCiv = false;
      bleLinkState = BLE_LINK_IDLE;
      btReconnectAt = millis() + 4000;
      Serial.println("[BT] IC-705 SPP disconnected");
    }

    if (bleInitialized && !linked && bleOutboundEnabled &&
        autoCalState <= AUTOCAL_CONFIRM && btReconnectAt > 0 &&
        millis() >= btReconnectAt) {
      btReconnectAt = millis() + 12000;
#ifdef ESP_IDF_VERSION_MAJOR
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
#endif
      bleConnectPeer();
    }

    if (linked) {
      blePumpRx();
      bleRunAutoCalCiv();
      blePumpRx();
      bool calBusy = autoCalState > AUTOCAL_CONFIRM;
      if (!bleGotCiv && !calBusy && bleCivWatchAt > 0 &&
          millis() - bleCivWatchAt >= 2500) {
        bleDropDeadLink("Kein CI-V auf diesem Kanal");
      }
      if (autoCalPollSwr && millis() - lastSwrPoll >= 40) {
        lastSwrPoll = millis();
        bleSwrReqPos = aktuellePosition;
        uint8_t swrCmd[] = {CIV_CMD_METER, CIV_SUB_SWR};
        bleSendCiv(swrCmd, 2);
        uint8_t poCmd[] = {CIV_CMD_METER, CIV_SUB_PO};
        bleSendCiv(poCmd, 2);
      }
      if (calBusy && autoCalState == AUTOCAL_WAIT_CIV &&
          autoCalAfterCiv == AUTOCAL_MOVE_EST &&
          millis() - lastPoll >= 200) {
        lastPoll = millis();
        uint8_t freqCmd[] = {CIV_CMD_READ_FREQ};
        bleSendCiv(freqCmd, 1);
      }
      unsigned long freqInterval = (bleRigFreqHz == 0) ? 400 : 2000;
      if (!calBusy && bleReadyAt > 0 && millis() - bleReadyAt >= 150 &&
          millis() - lastPoll >= freqInterval) {
        lastPoll = millis();
        uint8_t freqCmd[] = {CIV_CMD_READ_FREQ};
        bleSendCiv(freqCmd, 1);
      }
      if (!calBusy && bleReadyAt > 0 && millis() - lastPttPoll >= 4000) {
        lastPttPoll = millis();
        uint8_t pttCmd[] = {CIV_CMD_PTT, 0x00};
        bleSendCiv(pttCmd, 2);
      }
      if (civTxPendingLen > 0) {
        bleSendCiv((const uint8_t *)civTxPending, civTxPendingLen);
        civTxPendingLen = 0;
      }
      if (civTunerWatch && !calBusy && millis() >= civTunerPollAt) {
        civTunerPollAt = millis() + 300;
        uint8_t tunerCmd[] = {CIV_CMD_PTT, CIV_SUB_TUNER};
        bleSendCiv(tunerCmd, 2);
      }
      if (bleGotCiv) {
        bleConfirmChannel();
        if (bleOutboundFails > 0) {
          bleOutboundFails = 0;
          bleSppChannelTry = 0;
          bleOutboundEnabled = true;
        }
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

String bleStateText() {
  if (bleReady) {
    return "Connected";
  }
  if (bleLinkState == BLE_LINK_ERROR) {
    return bleErrorMessage[0] ? String(bleErrorMessage) : "Error";
  }
  if (bleInitialized) {
    return "Waiting for IC-705";
  }
  return "Bluetooth off";
}

void processRigTracking() {
  if (rtestState != RTEST_IDLE) {
    return;
  }
  if (autoCalState > AUTOCAL_CONFIRM) {
    return;
  }
  if (!btSettings.trackingEnabled || bleLinkState != BLE_LINK_READY ||
      bleTxActive || !bleReady || isMotorActive() ||
      currentPage == PAGE_MANUAL) {
    return;
  }
  if (bleRigFreqHz < 100000UL) {
    return;
  }
  float rigKhz = bleRigFreqHz / 1000.0f;
  long targetPos = (long)round(getPositionFromFrequency(rigKhz));
  long err = targetPos - aktuellePosition;
  if (labs(err) <= 2 && settledFromBelow) {
    lastTrackedKhz = rigKhz;
    return;
  }
  float antKhz = getFrequencyFromPosition(aktuellePosition);
  if (fabs(rigKhz - antKhz) < (float)btSettings.deadbandKhz && labs(err) <= 2) {
    lastTrackedKhz = rigKhz;
    return;
  }
  lastTrackedKhz = rigKhz;
  Serial.println("[BT] Tracking to " + String(rigKhz, 2) + " kHz -> P=" +
                 String(targetPos) + " (Tabelle " +
                 String((int)kalibrierTabelle.size()) + " Pkt)");
  moveToPosition(targetPos);
}

void rtestAbort(const char *msg) {
  if (rtestState == RTEST_IDLE) {
    return;
  }
  rtestState = RTEST_IDLE;
  btSettings.trackingEnabled = rtestSavedTracking ? 1 : 0;
  lastTrackedKhz = -1;
  Serial.println(String("[TEST] Abbruch: ") + msg);
  showResultWindow("POS-TEST", "Abgebrochen", msg, "", "* Menue");
  setStatusMessage(msg, 2500);
}

void rtestStart(int cycles) {
  if (rtestState != RTEST_IDLE) {
    Serial.println("[TEST] Laeuft bereits");
    return;
  }
  if (isMotorActive()) {
    Serial.println("[TEST] Motor busy");
    return;
  }
  if (autoCalState != AUTOCAL_IDLE) {
    Serial.println("[TEST] Auto-Cal laeuft");
    return;
  }
  if (cycles < 1) {
    cycles = RTEST_DEFAULT_CYCLES;
  }
  if (cycles > RTEST_MAX_CYCLES) {
    cycles = RTEST_MAX_CYCLES;
  }
  rtestCycles = cycles;
  rtestIndex = 0;
  rtestSavedTracking = btSettings.trackingEnabled;
  btSettings.trackingEnabled = 0;
  lastTrackedKhz = -1;
  Serial.println("[TEST] Start: erst Home, dann " + String(rtestCycles) +
                 "x " + String(RTEST_HIGH_POS) + " <-> " + String(RTEST_LOW_POS) +
                 ", dann wieder Home");
  setStatusMessage("Test: Referenz-Home", 2000);
  showResultWindow("POS-TEST", "Referenz-Home", "Bitte warten", "", "* Abbruch");
  startHoming();
  rtestWaitUntil = millis() + RTEST_MOVE_TIMEOUT_MS;
  rtestState = RTEST_WAIT_REF_HOME;
}

void processRepeatTest() {
  if (rtestState == RTEST_IDLE) {
    return;
  }

  unsigned long now = millis();
  bool idle = !isMotorActive();

  switch (rtestState) {
  case RTEST_WAIT_REF_HOME:
    if (!idle) {
      if (now >= rtestWaitUntil) {
        stopMotor();
        rtestAbort("Timeout Referenz-Home");
      }
      break;
    }
    if (!lastHomingErrorValid) {
      rtestAbort("Referenz-Home ohne Endschalter");
      break;
    }
    Serial.println("[TEST] Referenz-Home Fehler=" + String(lastHomingError) +
                   " (alte Positionsannahme)");
    showResultWindow("POS-TEST", "Referenz OK",
                     "Startfehler " + String(lastHomingError),
                     "Fahre " + String(RTEST_HIGH_POS), "* Abbruch");
    rtestState = RTEST_GO_HIGH;
    break;

  case RTEST_GO_HIGH:
    Serial.println("[TEST] Zyklus " + String(rtestIndex + 1) + "/" +
                   String(rtestCycles) + " -> " + String(RTEST_HIGH_POS));
    setStatusMessage("Test " + String(rtestIndex + 1) + "/" +
                         String(rtestCycles) + " hoch",
                     1500);
    showResultWindow("POS-TEST",
                     "Zyklus " + String(rtestIndex + 1) + "/" +
                         String(rtestCycles),
                     "Ziel " + String(RTEST_HIGH_POS), "nach oben",
                     "* Abbruch");
    moveToPosition(RTEST_HIGH_POS);
    rtestWaitUntil = now + RTEST_MOVE_TIMEOUT_MS;
    rtestState = RTEST_WAIT_HIGH;
    break;

  case RTEST_WAIT_HIGH:
    if (!idle) {
      if (now >= rtestWaitUntil) {
        stopMotor();
        rtestAbort("Timeout hoch");
      }
      break;
    }
    Serial.println("[TEST] Oben Pos=" + String(aktuellePosition));
    showResultWindow("POS-TEST",
                     "Zyklus " + String(rtestIndex + 1) + "/" +
                         String(rtestCycles),
                     "Oben P=" + String(aktuellePosition), "fahre runter",
                     "* Abbruch");
    rtestState = RTEST_GO_LOW;
    break;

  case RTEST_GO_LOW:
    Serial.println("[TEST] Zyklus " + String(rtestIndex + 1) + "/" +
                   String(rtestCycles) + " -> " + String(RTEST_LOW_POS));
    setStatusMessage("Test " + String(rtestIndex + 1) + "/" +
                         String(rtestCycles) + " runter",
                     1500);
    showResultWindow("POS-TEST",
                     "Zyklus " + String(rtestIndex + 1) + "/" +
                         String(rtestCycles),
                     "Ziel " + String(RTEST_LOW_POS), "nach unten",
                     "* Abbruch");
    moveToPosition(RTEST_LOW_POS);
    rtestWaitUntil = now + RTEST_MOVE_TIMEOUT_MS;
    rtestState = RTEST_WAIT_LOW;
    break;

  case RTEST_WAIT_LOW:
    if (!idle) {
      if (now >= rtestWaitUntil) {
        stopMotor();
        rtestAbort("Timeout runter");
      }
      break;
    }
    Serial.println("[TEST] Unten Pos=" + String(aktuellePosition));
    rtestIndex++;
    if (rtestIndex < rtestCycles) {
      rtestState = RTEST_GO_HIGH;
    } else {
      Serial.println("[TEST] Abschluss-Home...");
      setStatusMessage("Test: Abschluss-Home", 2000);
      showResultWindow("POS-TEST", "Abschluss-Home", "Bitte warten", "",
                       "* Abbruch");
      startHoming();
      rtestWaitUntil = now + RTEST_MOVE_TIMEOUT_MS;
      rtestState = RTEST_WAIT_FINAL_HOME;
    }
    break;

  case RTEST_WAIT_FINAL_HOME:
    if (!idle) {
      if (now >= rtestWaitUntil) {
        stopMotor();
        rtestAbort("Timeout Abschluss-Home");
      }
      break;
    }
    {
      long err = lastHomingErrorValid ? lastHomingError : 99999;
      Serial.println("[TEST] Fertig. Zyklen=" + String(rtestCycles) +
                     " Hub=" + String(RTEST_HIGH_POS) + " Fehler=" +
                     String(err) + " Schritte (am Schliessen, Soll ~0)");
      Serial.println("[TEST] Nach Null=Schliessen sollte der Fehler nahe 0 liegen.");
      setStatusMessage("Test-Fehler: " + String(err), 5000);
      showResultWindow("POS-TEST", "Fertig",
                       String(rtestCycles) + " x Hub " + String(RTEST_HIGH_POS),
                       "Fehler " + String(err) + " (Schliessen)",
                       "Soll ~0  * Menue");
    }
    rtestState = RTEST_IDLE;
    btSettings.trackingEnabled = rtestSavedTracking ? 1 : 0;
    lastTrackedKhz = -1;
    break;

  default:
    rtestState = RTEST_IDLE;
    break;
  }
}

void handleSerialCommand(const String &line) {
  String s = line;
  s.trim();
  if (s.length() == 0) {
    return;
  }
  String u = s;
  u.toUpperCase();
  if (u == "HELP" || u == "?") {
    Serial.println("[TEST] HOME | POS | STEP n [ms] | TRACK ON|OFF | TEST [n] | STOP");
    Serial.println("[TEST] TEST fährt n mal 4000<->400, dann Home.");
    Serial.println("[TEST] STEP 500 8 = 500 Schritte hoch mit 8 ms/Schritt.");
    Serial.println("[CIV] CIVMON ON|OFF = alle CI-V-Frames vom Rig mitschreiben.");
    Serial.println("[CIV] TUNERWATCH ON|OFF = Tuner-Status alle 300 ms abfragen.");
    Serial.println("[CIV] CIVTX 1C 01 = beliebiges CI-V-Kommando senden.");
    Serial.println("[CAL] SETTLE n = Wartezeit je Rasterpunkt in ms.");
    return;
  }
  if (u.startsWith("SETTLE")) {
    String args = u.substring(6);
    args.trim();
    if (args.length() > 0) {
      long ms = args.toInt();
      if (ms < 0 || ms > 5000) {
        Serial.println("[CAL] SETTLE 0..5000");
        return;
      }
      autoCalScanSettleMs = (unsigned long)ms;
    }
    Serial.println("[CAL] Rasterwartezeit " + String(autoCalScanSettleMs) +
                   " ms");
    return;
  }
  if (u == "CIVMON ON" || u == "CIVMON OFF") {
    civMonitor = u.endsWith("ON");
    Serial.println("[CIV] Monitor " + String(civMonitor ? "AN" : "AUS"));
    return;
  }
  if (u == "TUNERWATCH ON" || u == "TUNERWATCH OFF") {
    civTunerWatch = u.endsWith("ON");
    bleTunerState = 0xFF;
    civTunerPollAt = millis();
    Serial.println("[CIV] Tuner-Poll " + String(civTunerWatch ? "AN" : "AUS"));
    return;
  }
  if (u.startsWith("CIVTX ")) {
    String args = u.substring(6);
    args.trim();
    uint8_t payload[sizeof(civTxPending)];
    size_t n = 0;
    while (args.length() > 0 && n < sizeof(payload)) {
      int sp = args.indexOf(' ');
      String tok = (sp < 0) ? args : args.substring(0, sp);
      args = (sp < 0) ? String("") : args.substring(sp + 1);
      tok.trim();
      args.trim();
      if (tok.length() > 0) {
        payload[n++] = (uint8_t)strtol(tok.c_str(), nullptr, 16);
      }
    }
    if (n == 0) {
      Serial.println("[CIV] Beispiel: CIVTX 1C 01");
      return;
    }
    if (!bleReady) {
      Serial.println("[CIV] Kein Rig verbunden");
      return;
    }
    for (size_t i = 0; i < n; ++i) {
      civTxPending[i] = payload[i];
    }
    civTxPendingLen = (uint8_t)n;
    Serial.println("[CIV] TX eingereiht (" + String((int)n) + " Byte)");
    return;
  }
  if (u == "TRACK ON" || u == "TRACK OFF") {
    btSettings.trackingEnabled = u.endsWith("ON") ? 1 : 0;
    lastTrackedKhz = -1;
    saveBtSettings();
    Serial.println("[TEST] Tracking " +
                   String(btSettings.trackingEnabled ? "AN" : "AUS"));
    return;
  }
  if (u == "POS") {
    Serial.print("[TEST] Pos=");
    Serial.print(aktuellePosition);
    Serial.print(" Endstop=");
    Serial.print(endstopPressed() ? "AN" : "AUS");
    Serial.print(" LastHomeErr=");
    if (lastHomingErrorValid) {
      Serial.println(lastHomingError);
    } else {
      Serial.println("-");
    }
    return;
  }
  if (u == "HOME") {
    if (isMotorActive()) {
      Serial.println("[TEST] Motor busy");
      return;
    }
    startHoming();
    return;
  }
  if (u == "STOP") {
    if (isMotorActive()) {
      stopMotor();
    }
    rtestAbort("STOP");
    return;
  }
  // One relative move at a chosen speed: same number of steps with a
  // different number of stops (or a different rate) tells lost steps apart
  // from position drift at every stop.
  if (u.startsWith("STEP ")) {
    if (isMotorActive()) {
      Serial.println("[TEST] Motor busy");
      return;
    }
    String args = s.substring(5);
    args.trim();
    int sep = args.indexOf(' ');
    long steps = (sep > 0) ? args.substring(0, sep).toInt() : args.toInt();
    int ms = 0;
    if (sep > 0) {
      ms = args.substring(sep + 1).toInt();
    }
    if (steps == 0) {
      Serial.println("[TEST] STEP n [ms], n != 0");
      return;
    }
    if (ms < 1) {
      ms = SPEED_NORMAL;
    }
    Serial.println("[TEST] STEP " + String(steps) + " @ " + String(ms) +
                   " ms von P=" + String(aktuellePosition));
    startMove(steps, ms);
    return;
  }
  if (u == "TEST" || u.startsWith("TEST ")) {
    int n = RTEST_DEFAULT_CYCLES;
    if (u.length() > 5) {
      int parsed = u.substring(5).toInt();
      if (parsed > 0) {
        n = parsed;
      }
    }
    rtestStart(n);
    return;
  }
  Serial.println("[TEST] Unbekannt. HELP fuer Kommandos.");
}

void processSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      serialLineBuf[serialLineLen] = 0;
      if (serialLineLen > 0) {
        handleSerialCommand(String(serialLineBuf));
      }
      serialLineLen = 0;
    } else if (serialLineLen < sizeof(serialLineBuf) - 1) {
      serialLineBuf[serialLineLen++] = c;
    } else {
      serialLineLen = 0;
    }
  }
}

int autoCalFindBand(uint32_t hz) {
  if (hz < 100000UL) {
    return -1;
  }
  long khz = (long)((hz + 500UL) / 1000UL);
  for (size_t i = 0; i < sizeof(autoCalBands) / sizeof(autoCalBands[0]); ++i) {
    if (khz >= autoCalBands[i].startKhz && khz <= autoCalBands[i].endKhz) {
      return (int)i;
    }
  }
  return -1;
}

// Ascending, so every point is a short step up from the one before.
void autoCalBuildPoints(long startKhz, long endKhz) {
  autoCalCount = 0;
  autoCalFreqs[autoCalCount++] = (float)startKhz;
  long firstGrid = ((startKhz / 50L) + 1L) * 50L;
  for (long f = firstGrid; f < endKhz && autoCalCount < AUTOCAL_MAX_POINTS - 1;
       f += 50) {
    autoCalFreqs[autoCalCount++] = (float)f;
  }
  if (endKhz != startKhz && autoCalCount < AUTOCAL_MAX_POINTS) {
    autoCalFreqs[autoCalCount++] = (float)endKhz;
  }
}

void autoCalStripBand(long startKhz, long endKhz) {
  std::vector<Speicherpunkt> kept;
  for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
    long f = (long)round(kalibrierTabelle[i].frequenz);
    if (f < startKhz || f > endKhz) {
      kept.push_back(kalibrierTabelle[i]);
    }
  }
  kalibrierTabelle = kept;
}

void autoCalQueueCiv(int req, int next) {
  autoCalCivDone = false;
  autoCalAfterCiv = (AutoCalState)next;
  unsigned long waitMs = 1500;
  if (req == AUTOCAL_CIV_SET_FREQ || req == AUTOCAL_CIV_PTT_ON ||
      req == AUTOCAL_CIV_PREP || req == AUTOCAL_CIV_RESTORE) {
    waitMs = 2500;
  }
  autoCalWaitUntil = millis() + waitMs;
  autoCalState = AUTOCAL_WAIT_CIV;
  autoCalCivReq = (AutoCalCiv)req;
}

void autoCalAbort(const char *msg, bool restoreRadio) {
  Serial.println(String("[CAL] Abort: ") + msg);
  bool wasRetune = autoCalRetune;
  autoCalRetune = false;
  const char *bandName =
      wasRetune ? "Nachstimmen"
                : ((autoCalBandIndex >= 0) ? autoCalBands[autoCalBandIndex].name
                                           : "Auto-Kal");
  autoCalPollSwr = false;
  autoCalCivReq = AUTOCAL_CIV_NONE;
  if (isMotorActive() && !isHomingState()) {
    stopMotor();
  }
  if (webCalibration.active) {
    restoreCalibrationBackup();
  }
  btSettings.trackingEnabled = autoCalSavedTracking ? 1 : 0;
  autoCalState = AUTOCAL_IDLE;
  autoCalBandIndex = -1;
  autoCalBlockMenu = true;
  if (restoreRadio && bleInitialized && SerialBT.hasClient()) {
    autoCalCivDone = false;
    autoCalCivReq = AUTOCAL_CIV_RESTORE;
  }
  setStatusMessage(msg, 2500);
  showResultWindow(wasRetune ? "NACHSTIMMEN" : "AUTO-KAL", String(bandName),
                   "Abbruch", String(msg), "* Menue");
}

void autoCalBegin() {
  if (autoCalBandIndex < 0 || !bleReady) {
    autoCalState = AUTOCAL_IDLE;
    setStatusMessage("Kein IC-705", 2000);
    return;
  }
  autoCalSavedTracking = btSettings.trackingEnabled;
  btSettings.trackingEnabled = 0;
  lastTrackedKhz = -1;
  autoCalSavedFreqHz = bleRigFreqHz;
  autoCalSavedMode = bleSavedMode;
  autoCalSavedFilter = bleSavedFilter;
  autoCalSavedPowerHi = bleSavedPowerHi;
  autoCalSavedPowerLo = bleSavedPowerLo;
  ensureCalibrationSession();
  autoCalStripBand(autoCalBands[autoCalBandIndex].startKhz,
                   autoCalBands[autoCalBandIndex].endKhz);
  autoCalBuildPoints(autoCalBands[autoCalBandIndex].startKhz,
                     autoCalBands[autoCalBandIndex].endKhz);
  autoCalIndex = 0;
  autoCalSkipped = 0;
  autoCalPtCount = 0;
  autoCalPollSwr = false;
  autoCalEndstopHit = false;
  Serial.println("[CAL] Auto-cal " + String(autoCalBands[autoCalBandIndex].name) +
                 " " + String(autoCalCount) + " points");
  autoCalQueueCiv(AUTOCAL_CIV_PREP, AUTOCAL_SET_FREQ);
}

void autoCalBeginRetune() {
  if (!bleReady || bleRigFreqHz < 100000UL) {
    setStatusMessage("Kein IC-705", 2000);
    return;
  }
  if (kalibrierTabelle.empty()) {
    setStatusMessage("Keine Tabelle", 2000);
    return;
  }
  autoCalRetune = true;
  autoCalSavedTracking = btSettings.trackingEnabled;
  btSettings.trackingEnabled = 0;
  lastTrackedKhz = -1;
  autoCalSavedFreqHz = bleRigFreqHz;
  autoCalSavedMode = bleSavedMode;
  autoCalSavedFilter = bleSavedFilter;
  autoCalSavedPowerHi = bleSavedPowerHi;
  autoCalSavedPowerLo = bleSavedPowerLo;
  autoCalBandIndex = autoCalFindBand(bleRigFreqHz);
  autoCalFreqs[0] = (float)bleRigFreqHz / 1000.0f;
  autoCalCount = 1;
  autoCalIndex = 0;
  autoCalSkipped = 0;
  autoCalPtCount = 0;
  autoCalPollSwr = false;
  autoCalEndstopHit = false;
  Serial.println("[CAL] Nachstimmen bei " + String(autoCalFreqs[0], 2) + " kHz");
  setStatusMessage("Nachstimmen...", 2000);
  autoCalQueueCiv(AUTOCAL_CIV_PREP, AUTOCAL_SET_FREQ);
}

// Two dips from this run give a local slope that beats the stored table. A
// retune has no dips of its own but trusts the table for its single point.
bool autoCalEstimateIsFine() { return autoCalPtCount >= 2 || autoCalRetune; }

long autoCalEstimatePos(float freqKhz) {
  long est;
  if (autoCalPtCount >= 2) {
    int a = 0;
    int b = 1;
    float distA = fabsf(autoCalPtFreq[0] - freqKhz);
    float distB = fabsf(autoCalPtFreq[1] - freqKhz);
    if (distB < distA) {
      a = 1;
      b = 0;
      float t = distA;
      distA = distB;
      distB = t;
    }
    for (int i = 2; i < autoCalPtCount; ++i) {
      float d = fabsf(autoCalPtFreq[i] - freqKhz);
      if (d < distA) {
        b = a;
        distB = distA;
        a = i;
        distA = d;
      } else if (d < distB) {
        b = i;
        distB = d;
      }
    }
    float df = autoCalPtFreq[b] - autoCalPtFreq[a];
    if (fabsf(df) >= 0.5f) {
      float slope = (float)(autoCalPtPos[b] - autoCalPtPos[a]) / df;
      est = autoCalPtPos[a] +
            (long)round(slope * (freqKhz - autoCalPtFreq[a]));
    } else {
      est = autoCalPtPos[a];
    }
  } else if (autoCalPtCount == 1) {
    // One dip: keep the table slope, shift it onto the measured point.
    long tableHere = (long)round(getPositionFromFrequency(freqKhz));
    long tableAtPt = (long)round(getPositionFromFrequency(autoCalPtFreq[0]));
    est = tableHere + (autoCalPtPos[0] - tableAtPt);
  } else {
    est = (long)round(getPositionFromFrequency(freqKhz));
  }
  return constrain(est, 0L, gesamtSchritte);
}

void autoCalSkipOrFail(const char *reason) {
  Serial.println(String("[CAL] Point skip: ") + reason);
  autoCalPollSwr = false;
  if (isMotorActive()) {
    stopMotor();
  }
  // Without a single dip the estimate never improves, so give up after the
  // second failure instead of grinding through the whole band.
  if (autoCalPtCount == 0 && autoCalIndex >= 1) {
    autoCalAbort("Kein Dip gefunden");
    return;
  }
  autoCalSkipped++;
  autoCalQueueCiv(AUTOCAL_CIV_PTT_OFF, AUTOCAL_NEXT);
}

void autoCalEnterConfirm() {
  if (isMotorActive()) {
    return;
  }
  autoCalConfirmSample = false;
  autoCalPollSwr = true;
  bleSwrValid = false;
  autoCalWaitUntil = millis() + AUTOCAL_CONFIRM_MS;
  autoCalState = AUTOCAL_CONFIRM_SWR;
  Serial.println("[CAL] Confirm SWR at " + String(aktuellePosition) +
                 " (min was " + String(autoCalMinPos) + ")");
}

void processAutoCal() {
  if (autoCalState <= AUTOCAL_CONFIRM) {
    return;
  }
  if (autoCalEndstopHit) {
    if (isMotorActive()) {
      return;
    }
    autoCalEndstopHit = false;
    autoCalAbort("Endschalter");
    return;
  }
  if (!bleReady) {
    autoCalAbort("IC-705 weg");
    return;
  }

  unsigned long now = millis();
  float freqKhz =
      (autoCalIndex >= 0 && autoCalIndex < autoCalCount) ? autoCalFreqs[autoCalIndex] : 0;

  switch (autoCalState) {
  case AUTOCAL_WAIT_CIV:
    if (autoCalCivDone) {
      if (autoCalAfterCiv == AUTOCAL_MOVE_EST && autoCalTargetHz > 0) {
        long diff = labs((long)bleRigFreqHz - (long)autoCalTargetHz);
        if (diff > 2500L && now < autoCalWaitUntil + 800) {
          break;
        }
        if (diff > 2500L) {
          Serial.println("[CAL] Freq verify miss, have " +
                         String(bleRigFreqHz) + " want " +
                         String(autoCalTargetHz));
        }
      }
      autoCalCivDone = false;
      if (autoCalAfterCiv == AUTOCAL_WAIT_TX ||
          autoCalAfterCiv == AUTOCAL_WAIT_FINE_TX) {
        autoCalTxStarted = now;
        autoCalTxLive = false;
        bleSwrValid = false;
        blePoRaw = 0;
        autoCalSawHighSwr = false;
        autoCalMinSwr = 255;
      }
      if (autoCalAfterCiv == AUTOCAL_IDLE) {
        btSettings.trackingEnabled = autoCalSavedTracking ? 1 : 0;
        lastTrackedKhz = -1;
      }
      autoCalState = autoCalAfterCiv;
    } else if (now >= autoCalWaitUntil) {
      autoCalAbort("CI-V Timeout");
    }
    break;

  case AUTOCAL_SET_FREQ:
    autoCalTargetHz = (uint32_t)round(freqKhz * 1000.0f);
    bleSwrValid = false;
    autoCalQueueCiv(AUTOCAL_CIV_SET_FREQ, AUTOCAL_MOVE_EST);
    break;

  case AUTOCAL_MOVE_EST: {
    long est = autoCalEstimatePos(freqKhz);
    long margin = autoCalEstimateIsFine() ? AUTOCAL_FINE_MARGIN
                                          : AUTOCAL_COARSE_MARGIN;
    long target = est - margin;
    if (target < 0) {
      target = 0;
    }
    if (target > gesamtSchritte) {
      target = gesamtSchritte;
    }
    long delta = target - aktuellePosition;
    if (labs(delta) < 8) {
      autoCalState = AUTOCAL_TX_ON;
    } else {
      autoCalAfterMove = AUTOCAL_TX_ON;
      autoCalWaitUntil = now + 90000;
      // Nothing is measured on the way to the start point, so travel there.
      moveToPosition(target, SPEED_NORMAL);
      autoCalState = AUTOCAL_WAIT_MOVE;
    }
  } break;

  case AUTOCAL_WAIT_MOVE:
    if (!isMotorActive()) {
      autoCalState = autoCalAfterMove;
    } else if (now >= autoCalWaitUntil) {
      stopMotor();
      autoCalSkipOrFail("Move timeout");
    }
    break;

  case AUTOCAL_TX_ON:
    autoCalMinSwr = 255;
    autoCalMinPos = aktuellePosition;
    bleSwrValid = false;
    blePoRaw = 0;
    autoCalSawHighSwr = false;
    autoCalTxLive = false;
    autoCalPollSwr = true;
    autoCalQueueCiv(AUTOCAL_CIV_PTT_ON, AUTOCAL_WAIT_TX);
    break;

  case AUTOCAL_WAIT_TX:
  case AUTOCAL_WAIT_FINE_TX: {
    bool txOk = bleTxActive || blePoRaw > 8;
    if (!txOk && now < autoCalTxStarted + 1500) {
      break;
    }
    if (!txOk) {
      autoCalAbort("Kein TX vom 705");
      break;
    }
    if (!autoCalTxLive) {
      autoCalTxLive = true;
      autoCalTxStarted = now;
      bleSwrValid = false;
      break;
    }
    if (now < autoCalTxStarted + AUTOCAL_TX_SETTLE_MS) {
      break;
    }
    if ((!bleSwrValid || bleSwrRaw == 0) &&
        now < autoCalTxStarted + AUTOCAL_TX_SETTLE_MS + AUTOCAL_TX_METER_MS) {
      break;
    }
    autoCalSweepStartPos = aktuellePosition;
    autoCalMinSwr = 255;
    autoCalMinPos = aktuellePosition;
    autoCalSawHighSwr = false;
    long room = gesamtSchritte - aktuellePosition;
    bool fineOnly =
        (autoCalState == AUTOCAL_WAIT_FINE_TX) || autoCalEstimateIsFine();
    long steps = fineOnly ? min((long)AUTOCAL_FINE_STEPS, room) : room;
    if (steps < 20) {
      autoCalSkipOrFail("Kein Weg nach oben");
      break;
    }
    startMove(steps, SPEED_NORMAL);
    autoCalState = fineOnly ? AUTOCAL_FINE : AUTOCAL_SWEEP;
  } break;

  case AUTOCAL_SWEEP:
  case AUTOCAL_FINE: {
    if (now - autoCalTxStarted > AUTOCAL_MAX_TX_MS) {
      autoCalSkipOrFail("TX Timeout");
      break;
    }
    autoCalApplySweepSpeed();
    long fromStart = aktuellePosition - autoCalSweepStartPos;
    bool swrLive = bleSwrValid && (bleSwrRaw > 0 || autoCalSawHighSwr);
    if (swrLive) {
      if (!autoCalSawHighSwr) {
        if (bleSwrRaw >= AUTOCAL_SWR_HIGH || fromStart >= AUTOCAL_IGNORE_START) {
          autoCalSawHighSwr = true;
          autoCalMinSwr = bleSwrRaw;
          autoCalMinPos = bleSwrPos;
        }
      } else {
        if (bleSwrRaw < autoCalMinSwr) {
          autoCalMinSwr = bleSwrRaw;
          autoCalMinPos = bleSwrPos;
        } else if (autoCalPastDip()) {
          Serial.println("[CAL] Ueber Dip, SWR " + formatSwr(bleSwrRaw) +
                         " min " + formatSwr(autoCalMinSwr) + " @P=" +
                         String(autoCalMinPos));
          motorCancelToStop();
          autoCalState =
              (autoCalState == AUTOCAL_SWEEP) ? AUTOCAL_REWIND : AUTOCAL_UNDER_MIN;
          break;
        }
      }
    }
    if (!isMotorActive()) {
      if (autoCalSawHighSwr && autoCalMinSwr <= AUTOCAL_SWR_GOOD &&
          (autoCalMinPos - autoCalSweepStartPos) >= 8) {
        autoCalState =
            (autoCalState == AUTOCAL_SWEEP) ? AUTOCAL_REWIND : AUTOCAL_UNDER_MIN;
      } else {
        autoCalSkipOrFail("Kein Dip");
      }
    }
  } break;

  case AUTOCAL_REWIND:
    autoCalPollSwr = false;
    autoCalQueueCiv(AUTOCAL_CIV_PTT_OFF, AUTOCAL_WAIT_REWIND);
    break;

  case AUTOCAL_WAIT_REWIND:
    if (!autoCalCivDone && autoCalCivReq != AUTOCAL_CIV_NONE) {
      break;
    }
    if (isMotorActive()) {
      break;
    }
    {
      long dest = autoCalMinPos - AUTOCAL_REWIND_STEPS;
      if (dest < 0) {
        dest = 0;
      }
      if (dest < autoCalSweepStartPos) {
        dest = autoCalSweepStartPos;
      }
      long back = aktuellePosition - dest;
      if (back < 8) {
        autoCalState = AUTOCAL_FINE_TX;
      } else {
        Serial.println("[CAL] Unter Dip nach P=" + String(dest));
        autoCalAfterMove = AUTOCAL_FINE_TX;
        autoCalWaitUntil = now + 30000;
        moveToPosition(dest, SPEED_NORMAL);
        autoCalState = AUTOCAL_WAIT_MOVE;
      }
    }
    break;

  case AUTOCAL_FINE_TX:
    autoCalMinSwr = 255;
    autoCalMinPos = aktuellePosition;
    autoCalSawHighSwr = false;
    autoCalTxLive = false;
    bleSwrValid = false;
    autoCalPollSwr = true;
    autoCalTxStarted = now;
    autoCalQueueCiv(AUTOCAL_CIV_PTT_ON, AUTOCAL_WAIT_FINE_TX);
    break;

  case AUTOCAL_UNDER_MIN: {
    if (isMotorActive()) {
      break;
    }
    // A retune does not trust the swept minimum. It drops to the lower edge of
    // a small grid and measures it standing still.
    bool scan = autoCalRetune;
    long dest = autoCalMinPos - (scan ? (long)AUTOCAL_SCAN_BIAS
                                      : (long)AUTOCAL_UNDER_MIN_STEPS);
    if (dest < 0) {
      dest = 0;
    }
    AutoCalState next = scan ? AUTOCAL_SCAN_STEP : AUTOCAL_GOTO_MIN;
    if (scan) {
      autoCalScanLeft = AUTOCAL_SCAN_POINTS;
      autoCalScanBestSwr = 255;
      autoCalScanBestPos = autoCalMinPos;
      autoCalScanStart = dest;
      autoCalScanPass = 1;
    }
    if (scan ? (aktuellePosition == dest) : (aktuellePosition <= dest)) {
      autoCalState = next;
      break;
    }
    Serial.println("[CAL] Unter Min nach P=" + String(dest));
    autoCalAfterMove = next;
    autoCalWaitUntil = now + 30000;
    moveToPosition(dest, SPEED_NORMAL);
    autoCalState = AUTOCAL_WAIT_MOVE;
  } break;

  case AUTOCAL_SCAN_STEP: {
    if (isMotorActive()) {
      break;
    }
    if (now - autoCalTxStarted > AUTOCAL_MAX_TX_MS) {
      autoCalSkipOrFail("TX Timeout");
      break;
    }
    if (autoCalScanLeft <= 0) {
      if (autoCalScanBestSwr == 255) {
        autoCalSkipOrFail("Kein Messwert im Raster");
        break;
      }
      long scanEnd = autoCalScanStart + (long)AUTOCAL_SCAN_SPAN;
      bool onEdge = (autoCalScanBestPos <= autoCalScanStart + 1) ||
                    (autoCalScanBestPos >= scanEnd - 1);
      if (onEdge && autoCalScanPass < AUTOCAL_SCAN_PASSES) {
        autoCalScanPass++;
        long start = autoCalScanBestPos - (long)(AUTOCAL_SCAN_SPAN / 2);
        if (start < 0) {
          start = 0;
        }
        autoCalScanStart = start;
        autoCalScanLeft = AUTOCAL_SCAN_POINTS;
        Serial.println("[CAL] Raster am Rand, neu ab P=" + String(start));
        autoCalAfterMove = AUTOCAL_SCAN_STEP;
        autoCalWaitUntil = now + 30000;
        moveToPosition(start, SPEED_NORMAL);
        autoCalState = AUTOCAL_WAIT_MOVE;
        break;
      }
      autoCalMinPos = autoCalScanBestPos;
      autoCalMinSwr = autoCalScanBestSwr;
      Serial.println("[CAL] Raster best P=" + String(autoCalMinPos) + " SWR " +
                     formatSwr(autoCalMinSwr));
      autoCalState = AUTOCAL_GOTO_MIN;
      break;
    }
    autoCalScanSettleAt = now + autoCalScanSettleMs;
    autoCalState = AUTOCAL_SCAN_WAIT;
  } break;

  case AUTOCAL_SCAN_WAIT: {
    if (now - autoCalTxStarted > AUTOCAL_MAX_TX_MS) {
      autoCalSkipOrFail("TX Timeout");
      break;
    }
    if (now < autoCalScanSettleAt) {
      // Discard everything sampled during the dwell, so the value we keep was
      // taken after it elapsed.
      bleSwrValid = false;
      break;
    }
    if (!bleSwrValid) {
      if (now > autoCalScanSettleAt + 2000) {
        autoCalSkipOrFail("Kein SWR im Raster");
      }
      break;
    }
    if (bleSwrRaw < autoCalScanBestSwr) {
      autoCalScanBestSwr = bleSwrRaw;
      autoCalScanBestPos = aktuellePosition;
    }
    Serial.println("[CAL] Raster P=" + String(aktuellePosition) + " SWR " +
                   formatSwr(bleSwrRaw) + " raw " + String(bleSwrRaw));
    autoCalScanLeft--;
    if (autoCalScanLeft > 0) {
      // Always upward, so the backlash taken up on the way in still holds.
      startMove(AUTOCAL_SCAN_GAP, SPEED_CAL);
    }
    autoCalState = AUTOCAL_SCAN_STEP;
  } break;

  case AUTOCAL_GOTO_MIN: {
    if (isMotorActive()) {
      break;
    }
    Serial.println("[CAL] Von unten auf min P=" + String(autoCalMinPos));
    autoCalAfterMove = AUTOCAL_APPROACH_MIN;
    autoCalWaitUntil = now + 90000;
    moveToPosition(autoCalMinPos, SPEED_CAL);
    autoCalState = AUTOCAL_WAIT_MOVE;
  } break;

  case AUTOCAL_APPROACH_MIN: {
    if (isMotorActive()) {
      break;
    }
    autoCalMinPos = aktuellePosition;
    autoCalEnterConfirm();
  } break;

  case AUTOCAL_CONFIRM_SWR: {
    if (now - autoCalTxStarted > AUTOCAL_MAX_TX_MS) {
      autoCalSkipOrFail("TX Timeout");
      break;
    }
    if (!autoCalConfirmSample) {
      if (now < autoCalWaitUntil) {
        break;
      }
      autoCalConfirmSample = true;
      autoCalMinSwr = 255;
      autoCalWaitUntil = now + AUTOCAL_CONFIRM_SAMPLE_MS;
      bleSwrValid = false;
      break;
    }
    if (bleSwrValid) {
      if (bleSwrRaw < autoCalMinSwr) {
        autoCalMinSwr = bleSwrRaw;
      }
    }
    if (now < autoCalWaitUntil) {
      break;
    }
    Serial.println("[CAL] Confirmed SWR raw " + String(autoCalMinSwr) +
                   " at minP=" + String(autoCalMinPos) + " stopP=" +
                   String(aktuellePosition));
    if (autoCalMinSwr > AUTOCAL_SWR_GOOD) {
      autoCalSkipOrFail("SWR Bestätigung fehlgeschlagen");
      break;
    }
    autoCalState = AUTOCAL_SAVE;
  } break;

  case AUTOCAL_SAVE:
    autoCalPollSwr = false;
    if (isMotorActive()) {
      break;
    }
    if (autoCalMinSwr > AUTOCAL_SWR_GOOD) {
      autoCalSkipOrFail("SWR zu hoch");
      break;
    }
    autoCalQueueCiv(AUTOCAL_CIV_PTT_OFF, AUTOCAL_NEXT);
    autoCalMinPos = aktuellePosition;
    // The measured dip has to land in the table even for a retune. Tracking
    // steers by the table alone, so a result kept outside it would be undone
    // the moment tracking resumes.
    saveSpeicherpunkt(autoCalMinPos, freqKhz);
    if (autoCalRetune) {
      Serial.println("[CAL] Nachgestimmt " + String(freqKhz, 2) + " kHz @ " +
                     String(autoCalMinPos) + " SWR raw " +
                     String(autoCalMinSwr));
      break;
    }
    if (autoCalPtCount < AUTOCAL_MAX_POINTS) {
      autoCalPtFreq[autoCalPtCount] = freqKhz;
      autoCalPtPos[autoCalPtCount] = autoCalMinPos;
      autoCalPtCount++;
    }
    Serial.println("[CAL] Saved " + String(freqKhz, 2) + " kHz @ " +
                   String(autoCalMinPos) + " SWR raw " +
                   String(autoCalMinSwr));
    break;

  case AUTOCAL_NEXT:
    autoCalIndex++;
    if (autoCalIndex >= autoCalCount) {
      autoCalState = AUTOCAL_RESTORE;
    } else {
      autoCalState = AUTOCAL_SET_FREQ;
    }
    break;

  case AUTOCAL_RESTORE:
    autoCalPollSwr = false;
    if (webCalibration.active) {
      commitCalibrationSession();
    }
    lastTrackedKhz = -1;
    autoCalQueueCiv(AUTOCAL_CIV_RESTORE, AUTOCAL_IDLE);
    if (autoCalRetune) {
      autoCalRetune = false;
      if (autoCalSkipped > 0) {
        setStatusMessage("Kein Dip", 3000);
        showResultWindow("NACHSTIMMEN", String(autoCalFreqs[0], 2) + " kHz",
                         "Kein Dip gefunden", "Position unveraendert",
                         "* Menue");
      } else {
        setStatusMessage("Nachgestimmt SWR " + formatSwr(autoCalMinSwr), 3000);
        showResultWindow("NACHSTIMMEN", String(autoCalFreqs[0], 2) + " kHz",
                         "Pos " + String(autoCalMinPos),
                         "SWR " + formatSwr(autoCalMinSwr), "* Menue");
      }
      autoCalBandIndex = -1;
      break;
    }
    {
      const char *bandName =
          (autoCalBandIndex >= 0) ? autoCalBands[autoCalBandIndex].name : "";
      int savedPts = autoCalCount - autoCalSkipped;
      if (savedPts < 0) {
        savedPts = 0;
      }
      String msg = String(bandName) + " fertig";
      if (autoCalSkipped > 0) {
        msg += " skip " + String(autoCalSkipped);
      }
      setStatusMessage(msg, 3000);
      showResultWindow("AUTO-KAL", String(bandName) + " fertig",
                       "Punkte " + String(savedPts) + "/" +
                           String(autoCalCount),
                       autoCalSkipped > 0
                           ? ("Skip " + String(autoCalSkipped))
                           : "Alle OK",
                       "* Menue");
    }
    autoCalBandIndex = -1;
    break;

  default:
    break;
  }
}

void setup() {
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_PIN), endstopISR, FALLING);
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("\n--- ESP32 WROOM System Start (V25.00) ---");
  Serial.println("[TEST] USB-Kommandos: HELP  HOME  POS  TEST [n]  STOP");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  display.setRotation(0);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("INIT OK");
  display.display();
  delay(1000);

  for (int i = 0; i < 4; i++)
    pinMode(motorPins[i], OUTPUT);
  for (int i = 0; i < 4; i++)
    digitalWrite(motorPins[i], LOW);

  EEPROM.begin(EEPROM_SIZE);
  loadBtSettings();

  xTaskCreatePinnedToCore(motorTask, "MotorControl", 4096, NULL, 2, NULL, 0);

  aktuellePosition = 0;
  int anzahlPunkte = 0;
  EEPROM.get(ADRESSE_PUNKTE_ZAEHLER, anzahlPunkte);

  if (anzahlPunkte < 2) {
    resetKalibrierung();
    setStatusMessage("INIT: Freq. ges. H-Strt.", 2000);
  } else {
    loadKalibrierTabelle();
    setStatusMessage(
        "Tabelle geladen (" + String(kalibrierTabelle.size()) + " Pkt)", 2000);
  }

  long lastKnownPosition = 0;
  bool wasRunning = false;

  EEPROM.get(ADRESSE_MOTOR_RUNNING_FLAG, wasRunning);
  EEPROM.get(ADRESSE_LETZTE_POSITION, lastKnownPosition);

  if (wasRunning) {
    Serial.println(
        "[SYSTEM] System was running during last shutdown, forcing homing");
    startHoming();
    setStatusMessage("Fehler! Not-Homing!", 2000);
  } else if (lastKnownPosition >= 0 && lastKnownPosition <= gesamtSchritte) {
    Serial.println("[SYSTEM] Restoring last known position: " +
                   String(lastKnownPosition));
    aktuellePosition = lastKnownPosition;
  } else {
    Serial.println("[SYSTEM] Invalid last known position, starting homing");
    startHoming();
    setStatusMessage("Homing startet...", 2000);
  }

  setupWiFi();
  setupWebServer();
  xTaskCreatePinnedToCore(bleTask, "BT", 8192, NULL, 1, NULL, 0);
  if (btSettings.btAutoStart) {
    // Just enough for the AP to finish coming up before btEnsureInit tears it
    // down again; the motor check below handles the homing case.
    btAutoStartAt = millis() + 800;
    Serial.println("[BT] Auto-start armed");
    setStatusMessage("BT startet...", 2000);
  } else {
    setStatusMessage("WLAN: " + String(ssid), 2000);
  }

  currentPage = PAGE_MENU;
  updateDisplay();
}

void loop() {
  unsigned long jetzt = millis();
  char taste = keypad.getKey();

  if (btAutoStartAt > 0 && jetzt >= btAutoStartAt &&
      (!isMotorActive() || jetzt >= btAutoStartAt + 15000)) {
    btAutoStartAt = 0;
    Serial.println("[BT] Auto-start now");
    bleCommand = BLE_CMD_START;
  }

  // Always process server requests
  server.handleClient();
  webSocket.loop();
  processRigTracking();
  processAutoCal();
  processSerialCommands();
  processRepeatTest();

  // Always process keypad input
  if (taste) {
    Serial.println("[KEYPAD] Key pressed: " + String(taste));
    processKeypad(taste);
  }

  // Send status on change (state immediately, position throttled while moving)
  // plus a 5s heartbeat so the UI stays alive when nothing happens.
  if (webSocket.connectedClients() > 0) {
    static long lastBroadcastPosition = -1;
    static MotorState lastBroadcastState = MOTOR_IDLE;
    static HomingPhase lastBroadcastPhase = HOMING_APPROACH;
    static unsigned long lastBroadcastTime = 0;
    static uint32_t lastBroadcastRigHz = 0;
    static BleLinkState lastBroadcastBle = BLE_LINK_IDLE;
    const unsigned long wsMinPosIntervalMs = 100;
    const unsigned long wsHeartbeatMs = 5000;

    bool stateChanged = (currentMotorState != lastBroadcastState) ||
                        (currentHomingPhase != lastBroadcastPhase);
    bool posChanged = (aktuellePosition != lastBroadcastPosition);
    bool posDue =
        posChanged && (jetzt - lastBroadcastTime >= wsMinPosIntervalMs);
    bool bleChanged = (bleLinkState != lastBroadcastBle) ||
                      (bleRigFreqHz != lastBroadcastRigHz);
    bool heartbeat = (jetzt - lastBroadcastTime >= wsHeartbeatMs);

    if (stateChanged || posDue || bleChanged || heartbeat) {
      broadcastStatusUpdate();
      lastBroadcastPosition = aktuellePosition;
      lastBroadcastState = currentMotorState;
      lastBroadcastPhase = currentHomingPhase;
      lastBroadcastRigHz = bleRigFreqHz;
      lastBroadcastBle = bleLinkState;
      lastBroadcastTime = jetzt;
    }
  }

  if (jetzt - lastCursorToggle >= cursorInterval) {
    lastCursorToggle = jetzt;
    cursorVisible = !cursorVisible;
  }

  if (taste == '*') {
    if (currentPage == PAGE_RESULT) {
      // processKeypad: Abbruch bleibt auf Ergebnis, sonst Menue
    } else if (rtestState != RTEST_IDLE) {
      rtestAbort("Stop");
    } else if (autoCalBlockMenu) {
      autoCalBlockMenu = false;
    } else if (isMotorActive()) {
      Serial.println("[KEYPAD] Stop command received");
      stopMotor();
    } else {
      jumpToPage(PAGE_MENU);
    }
  }

  if (statusMeldungEnde > 0 && jetzt >= statusMeldungEnde) {
    statusMeldungEnde = 0;
    statusMeldung = "";
    if (currentPage == PAGE_INIT_CONFIRM) {
      jumpToPage(PAGE_MENU);
    }
  }

  unsigned long dispInterval = displayInterval;
  if (autoCalState > AUTOCAL_CONFIRM && currentPage == PAGE_WEB_STATUS) {
    dispInterval = displayIntervalAutoCal;
  } else if (isMotorActive()) {
    dispInterval = displayIntervalManualMove;
  }
  if (jetzt - lastDisplayUpdate >= dispInterval) {
    lastDisplayUpdate = jetzt;
    updateDisplay();
  }
}

void processKeypad(char taste) {
  if (!taste) {
    return;
  }

  // Handle '*' key (global stop/menu)
  if (taste == '*') {
    Serial.println("[KEYPAD] Stop/menu key pressed");

    if (rtestState != RTEST_IDLE) {
      rtestAbort("Stop");
      return;
    }

    if (autoCalState != AUTOCAL_IDLE) {
      if (autoCalState == AUTOCAL_CONFIRM) {
        autoCalState = AUTOCAL_IDLE;
        autoCalBandIndex = -1;
        autoCalBlockMenu = true;
        setStatusMessage("Kal. abgebrochen", 1500);
      } else {
        autoCalAbort("Kal. abgebrochen");
      }
      return;
    }

    if (currentPage == PAGE_RESULT) {
      jumpToPage(PAGE_MENU);
      return;
    }

    if (isMotorActive()) {
      stopMotor();
    } else {
      jumpToPage(PAGE_MENU);
    }
    return;
  }

  // Handle '#' key (stop or execute)
  if (taste == '#') {
    Serial.println("[KEYPAD] Execute/stop key pressed");

    if (autoCalState == AUTOCAL_CONFIRM) {
      autoCalBegin();
      return;
    }
    if (autoCalState > AUTOCAL_CONFIRM) {
      autoCalAbort("Kal. abgebrochen");
      return;
    }

    // First check if we need to stop a running operation
    if (isMotorActive()) {
      Serial.println("[KEYPAD] Stopping current operation");
      stopMotor();
      return;
    }

    // Then check for execute operations
    if (currentPage == PAGE_QRG_TARGET && eingabePuffer.length() > 0 &&
        !isMotorActive()) {
      Serial.println("[KEYPAD] Executing target move");
      float frequency = eingabePuffer.toFloat();
      moveToFrequency(frequency);
      eingabePuffer = "";
      return;
    } else if (currentPage == PAGE_QRG_SAVE && eingabePuffer.length() > 0 &&
               !isMotorActive()) {
      Serial.println("[KEYPAD] Executing save operation");
      float freqToSave = eingabePuffer.toFloat();
      saveSpeicherpunkt(aktuellePosition, freqToSave);
      eingabePuffer = "";
      setStatusMessage("Frequency saved", 2000);
      return;
    } else if (currentPage == PAGE_INIT) {
      if (!isMotorActive()) {
        Serial.println("[KEYPAD] Going to init confirm");
        jumpToPage(PAGE_INIT_CONFIRM);
        return;
      }
    } else if (currentPage == PAGE_INIT_CONFIRM) {
      Serial.println("[KEYPAD] Checking reset code");
      if (eingabePuffer == "1234") {
        Serial.println("[KEYPAD] Reset code correct, performing reset");
        resetKalibrierung();
        setStatusMessage("RESET ERFOLGREICH!", 3000);
      } else {
        Serial.println("[KEYPAD] Reset code incorrect");
        setStatusMessage("Falscher Code!", 2000);
        eingabePuffer = "";
      }
      return;
    }
    return;
  }

  if (taste >= '0' && taste <= '9') {
    if (currentPage == PAGE_RESULT) {
      return;
    }
    // Handle menu navigation
    if (currentPage == PAGE_MENU) {
      Serial.println("[KEYPAD] Menu navigation: " + String(taste));

      if (taste == '1') {
        jumpToPage(PAGE_QRG_TARGET);
      } else if (taste == '2') {
        jumpToPage(PAGE_MANUAL);
      } else if (taste == '3') {
        jumpToPage(PAGE_QRG_SAVE);
      } else if (taste == '4') {
        jumpToPage(PAGE_INIT);
      } else if (taste == '5') {
        jumpToPage(PAGE_WEB_STATUS);
      } else if (taste == '6') {
        jumpToPage(PAGE_CAL_GRAPH);
      }
      return;
    }

    if (currentPage == PAGE_WEB_STATUS && taste == '0') {
      if (autoCalState != AUTOCAL_IDLE) {
        return;
      }
      toggleTracking();
      return;
    }
    if (currentPage == PAGE_WEB_STATUS && taste == '1') {
      if (autoCalState != AUTOCAL_IDLE) {
        return;
      }
      if (bleInitialized) {
        Serial.println("[KEYPAD] Toggle radio: Bluetooth -> WLAN");
        bleCommand = BLE_CMD_STOP;
      } else {
        Serial.println("[KEYPAD] Toggle radio: WLAN -> Bluetooth");
        bleCommand = BLE_CMD_START;
      }
      return;
    }
    if (currentPage == PAGE_WEB_STATUS && taste == '2') {
      if (autoCalState > AUTOCAL_CONFIRM) {
        return;
      }
      if (!bleReady) {
        setStatusMessage("Kein IC-705", 2000);
        return;
      }
      int band = autoCalFindBand(bleRigFreqHz);
      if (band < 0) {
        setStatusMessage("Band 40-10m", 2000);
        return;
      }
      autoCalBandIndex = band;
      autoCalState = AUTOCAL_CONFIRM;
      setStatusMessage(String(autoCalBands[band].name) + " kalibrieren?", 2000);
      return;
    }
    if (currentPage == PAGE_WEB_STATUS && taste == '3') {
      if (autoCalState != AUTOCAL_IDLE || isMotorActive()) {
        return;
      }
      Serial.println("[KEYPAD] Nachstimmen");
      autoCalBeginRetune();
      return;
    }

    // Manual movement buttons
    // In processKeypad function, replace the manual move section with this:

    // Manual movement buttons
    if (currentPage == PAGE_MANUAL) {
      if (taste == '1' || taste == '4' || taste == '7' || taste == '3' ||
          taste == '6' || taste == '9') {
        long schritte;
        if (taste == '1' || taste == '3') {
          schritte = 1;
        } else if (taste == '4' || taste == '6') {
          schritte = 10;
        } else if (taste == '7' || taste == '9') {
          schritte = 100;
        } else {
          return;
        }

        bool upTaste =
            (taste == '3' || taste == '6' ||
             taste == '9'); // Renamed to upTaste since 3,6,9 are up buttons
        long steps = upTaste ? schritte : -schritte;
        Serial.println("[KEYPAD] Manual move: " + String(schritte) + " steps " +
                       (upTaste ? "up" : "down") + " -> pos " +
                       String(aktuellePosition + steps));

        // Allow manual movement only if motor is not active
        if (!isMotorActive()) {
          startMove(steps, SPEED_NORMAL);
        }
        return; // Important: return after handling manual movement
      } else if (taste == '0') {
        // Only allow homing when in manual mode
        Serial.println("[KEYPAD] Homing requested");

        if (!isMotorActive()) {
          startHoming();
        }
        return; // Important: return after handling homing
      }
    }

    // Handle page-specific input
    switch (currentPage) {
    case PAGE_QRG_TARGET:
    case PAGE_QRG_SAVE:
      if (eingabePuffer.length() < 5) {
        eingabePuffer += taste;
        Serial.println("[KEYPAD] Input buffer: " + eingabePuffer);
      }
      break;

    case PAGE_INIT_CONFIRM:
      if (eingabePuffer.length() < 4) {
        eingabePuffer += taste;
        Serial.println("[KEYPAD] Reset code: " + eingabePuffer);
      }
      break;
    case PAGE_INIT:
      if (taste == '0' && !isMotorActive()) {
        Serial.println("[KEYPAD] Export table requested");
        dumpKalibrierTabelle();
        return;
      }
      if (taste == '1' && !isMotorActive()) {
        rtestStart(RTEST_DEFAULT_CYCLES);
        return;
      }
      break;
    }
  }
}

void loadKalibrierTabelle() {
  Serial.println("[SYSTEM] Loading calibration table");
  kalibrierTabelle.clear();
  int anzahlPunkte = 0;
  EEPROM.get(ADRESSE_PUNKTE_ZAEHLER, anzahlPunkte);
  if (anzahlPunkte < 2)
    return;
  Speicherpunkt tempPunkt;
  for (int i = 0; i < anzahlPunkte; ++i) {
    EEPROM.get(ADRESSE_ERSTER_PUNKT + i * sizeof(Speicherpunkt), tempPunkt);
    kalibrierTabelle.push_back(tempPunkt);
  }
  sortKalibrierTabelle();
  ValidateResult loaded = validateAndCleanKalibrierTabelle();
  if (loaded == VALIDATE_CLEANED) {
    persistKalibrierTabelle();
  } else if (loaded == VALIDATE_RESET) {
    resetKalibrierung();
  }
  Serial.println("[SYSTEM] Loaded " + String(kalibrierTabelle.size()) +
                 " calibration points");
}

ValidateResult validateAndCleanKalibrierTabelle() {
  Serial.println("[SYSTEM] Validating calibration table");
  if (kalibrierTabelle.size() < 2)
    return VALIDATE_OK;

  const float minFrequenz = kalibrierTabelle.front().frequenz;
  const float maxFrequenz = kalibrierTabelle.back().frequenz;

  std::vector<Speicherpunkt> tempTabelle;
  size_t initialSize = kalibrierTabelle.size();
  int fehlerZaehler = 0;

  tempTabelle.push_back(kalibrierTabelle[0]);

  for (size_t i = 1; i < initialSize; ++i) {
    const Speicherpunkt &currentPoint = kalibrierTabelle[i];
    const Speicherpunkt &lastValidPoint = tempTabelle.back();

    bool punktFehlerhaft = false;

    if (currentPoint.frequenz < minFrequenz ||
        currentPoint.frequenz > maxFrequenz) {
      Serial.print("FEHLER: Pos ");
      Serial.print(currentPoint.position);
      Serial.println(" -> Grenzwertverletzung (ausserhalb P0/P_Ende).");
      punktFehlerhaft = true;
    } else if (currentPoint.frequenz < lastValidPoint.frequenz) {
      Serial.print("FEHLER: Pos ");
      Serial.print(currentPoint.position);
      Serial.println(" -> Frequenzabfall (zu klein).");
      punktFehlerhaft = true;
    } else if (i + 1 < initialSize) {
      const Speicherpunkt &nextPoint = kalibrierTabelle[i + 1];
      if (nextPoint.frequenz < currentPoint.frequenz) {
        Serial.print("FEHLER: Pos ");
        Serial.print(currentPoint.position);
        Serial.println(" -> Ausreisser nach oben (bricht Monotonie zur Folge).");
        punktFehlerhaft = true;
      }
    }

    if (punktFehlerhaft) {
      fehlerZaehler++;

      if (fehlerZaehler > 1) {
        Serial.println("ABBRUCH: Mehr als ein Fehler in der Tabelle gefunden.");
        return VALIDATE_CRITICAL;
      }
    } else {
      tempTabelle.push_back(currentPoint);
    }
  }

  if (fehlerZaehler > 0 || tempTabelle.size() < initialSize) {
    if (tempTabelle.size() < 2) {
      return VALIDATE_RESET;
    }
    kalibrierTabelle = tempTabelle;
    return VALIDATE_CLEANED;
  }
  return VALIDATE_OK;
}

void resetKalibrierung() {
  Serial.println("[SYSTEM] Resetting calibration table");
  webCalibration.active = false;
  webCalibration.backup.clear();
  kalibrierTabelle.clear();
  kalibrierTabelle.push_back({0, 5150.0f});
  kalibrierTabelle.push_back({gesamtSchritte, 24641.0f});
  persistKalibrierTabelle();
}

void sortKalibrierTabelle() {
  std::sort(kalibrierTabelle.begin(), kalibrierTabelle.end(),
            [](const Speicherpunkt &a, const Speicherpunkt &b) {
              return a.position < b.position;
            });
}

ValidateResult saveSpeicherpunkt(long pos, float freq) {
  Serial.println("[SYSTEM] Saving calibration point: Pos=" + String(pos) +
                 " Freq=" + String(freq, 2) + "kHz");
  std::vector<Speicherpunkt> before = kalibrierTabelle;
  Speicherpunkt neuerPunkt = {pos, freq};
  bool punktGefunden = false;
  bool saveNeeded = false;

  for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
    if (kalibrierTabelle[i].position == pos) {
      kalibrierTabelle[i] = neuerPunkt;
      punktGefunden = true;
      saveNeeded = true;
      break;
    }
  }

  if (!punktGefunden) {
    for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
      if (abs(kalibrierTabelle[i].frequenz - freq) < 0.001f) {
        kalibrierTabelle[i] = neuerPunkt;
        punktGefunden = true;
        saveNeeded = true;
        break;
      }
    }
  }

  if (!punktGefunden) {
    if (kalibrierTabelle.size() < MAX_SPEICHERPUNKTE) {
      kalibrierTabelle.push_back(neuerPunkt);
      saveNeeded = true;
    } else {
      setStatusMessage("Fehler: Max. Pkt erreicht!", 2000);
      return VALIDATE_CRITICAL;
    }
  }

  if (!saveNeeded) {
    return VALIDATE_OK;
  }

  sortKalibrierTabelle();
  ValidateResult result = validateAndCleanKalibrierTabelle();

  if (result == VALIDATE_CRITICAL) {
    kalibrierTabelle = before;
    setStatusMessage("KRITISCH! Punkt verworfen", 4000);
    return result;
  }

  if (result == VALIDATE_RESET) {
    resetKalibrierung();
    setStatusMessage("FEHLER: Tabelle zu kurz. Reset auf 2 Punkte.", 4000);
    return result;
  }

  if (!webCalibration.active) {
    persistKalibrierTabelle();
  }

  if (result == VALIDATE_CLEANED) {
    setStatusMessage(
        "Tab. korrigiert: " + String(kalibrierTabelle.size()) + " Pkt", 3000);
  } else {
    setStatusMessage("Table aktualisiert", 2000);
  }
  return result;
}

float getPositionFromFrequency(float frequenz) {
  if (kalibrierTabelle.size() < 2)
    return 0;

  const Speicherpunkt *lo = nullptr;
  const Speicherpunkt *hi = nullptr;
  for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
    const Speicherpunkt &p = kalibrierTabelle[i];
    if (p.frequenz <= frequenz &&
        (!lo || p.frequenz > lo->frequenz)) {
      lo = &p;
    }
    if (p.frequenz >= frequenz &&
        (!hi || p.frequenz < hi->frequenz)) {
      hi = &p;
    }
  }
  if (lo && hi) {
    if (fabs(hi->frequenz - lo->frequenz) < 0.001f) {
      return (float)lo->position;
    }
    return (float)lo->position +
           (float)(hi->position - lo->position) *
               (frequenz - lo->frequenz) / (hi->frequenz - lo->frequenz);
  }
  if (lo) {
    return (float)lo->position;
  }
  if (hi) {
    return (float)hi->position;
  }
  return (float)aktuellePosition;
}

float getFrequencyFromPosition(long position) {
  if (kalibrierTabelle.size() < 2)
    return 0.0f;
  if (position <= kalibrierTabelle.front().position)
    return kalibrierTabelle.front().frequenz;
  if (position >= kalibrierTabelle.back().position)
    return kalibrierTabelle.back().frequenz;
  for (size_t i = 0; i < kalibrierTabelle.size() - 1; ++i) {
    const Speicherpunkt &p1 = kalibrierTabelle[i];
    const Speicherpunkt &p2 = kalibrierTabelle[i + 1];
    if (position >= p1.position && position <= p2.position) {
      float freq = p1.frequenz + (p2.frequenz - p1.frequenz) *
                                     (position - p1.position) /
                                     (p2.position - p1.position);
      return freq;
    }
  }
  return 0.0f;
}

void dumpKalibrierTabelle() {
  Serial.println("\n--- START DATEN-EXPORT Kalibrierungstabelle ---");
  Serial.println("Index; Position (Schritte); Frequenz (kHz)");

  loadKalibrierTabelle();

  if (kalibrierTabelle.size() < 2) {
    Serial.println("FEHLER: Kalibrierungstabelle ist leer oder enthaelt zu "
                   "wenige Punkte.");
    setStatusMessage("Export: Tabelle leer!", 2000);
  } else {
    for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
      Speicherpunkt p = kalibrierTabelle[i];
      Serial.print(i);
      Serial.print("; ");
      Serial.print(p.position);
      Serial.print("; ");
      Serial.println(p.frequenz, 4);
    }
    setStatusMessage("Export gesendet.", 3000);
  }
  Serial.println("--- ENDE DATEN-EXPORT ---");
}

void drawCalTableGraph() {
  display.setFont();
  display.setTextSize(1);

  if (kalibrierTabelle.size() < 2) {
    display.setCursor(0, 28);
    display.print("Keine Tabelle");
    display.setCursor(0, 44);
    display.print("* Menue");
    return;
  }

  float fMin = kalibrierTabelle.front().frequenz;
  float fMax = kalibrierTabelle.front().frequenz;
  long pMin = kalibrierTabelle.front().position;
  long pMax = kalibrierTabelle.front().position;
  for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
    const Speicherpunkt &p = kalibrierTabelle[i];
    if (p.frequenz < fMin) {
      fMin = p.frequenz;
    }
    if (p.frequenz > fMax) {
      fMax = p.frequenz;
    }
    if (p.position < pMin) {
      pMin = p.position;
    }
    if (p.position > pMax) {
      pMax = p.position;
    }
  }

  const int x0 = 32;
  const int y0 = 12;
  const int x1 = 127;
  const int y1 = 52;

  display.drawFastVLine(x0, y0, y1 - y0, SSD1306_WHITE);
  display.drawFastHLine(x0, y1, x1 - x0, SSD1306_WHITE);

  display.setCursor(0, y0);
  display.print(pMax);
  display.setCursor(0, y1 - 8);
  display.print(pMin);
  display.setCursor(x0 + 2, 56);
  display.print((long)round(fMin));
  String fMaxStr = String((long)round(fMax));
  display.setCursor(SCREEN_WIDTH - (int)fMaxStr.length() * 6, 56);
  display.print(fMaxStr);

  auto mapX = [&](float freq) -> int {
    if (fMax <= fMin) {
      return (x0 + x1) / 2;
    }
    int x = x0 + (int)round((freq - fMin) * (float)(x1 - x0) / (fMax - fMin));
    if (x < x0) {
      x = x0;
    }
    if (x > x1) {
      x = x1;
    }
    return x;
  };
  auto mapY = [&](long pos) -> int {
    if (pMax <= pMin) {
      return (y0 + y1) / 2;
    }
    int y = y1 - (int)(((pos - pMin) * (long)(y1 - y0)) / (pMax - pMin));
    if (y < y0) {
      y = y0;
    }
    if (y > y1) {
      y = y1;
    }
    return y;
  };

  int lastX = -1;
  int lastY = -1;
  for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
    const Speicherpunkt &p = kalibrierTabelle[i];
    int x = mapX(p.frequenz);
    int y = mapY(p.position);
    display.drawPixel(x, y, SSD1306_WHITE);
    if (lastX >= 0) {
      display.drawLine(lastX, lastY, x, y, SSD1306_WHITE);
    }
    lastX = x;
    lastY = y;
  }

  int cx = mapX(getFrequencyFromPosition(aktuellePosition));
  int cy = mapY(aktuellePosition);
  display.drawCircle(cx, cy, 2, SSD1306_WHITE);
}

void drawPositionBar() {
  const int barY = 55;
  const int barHeight = 8;
  const int barWidth = 115;

  display.drawRect(0, barY, barWidth, barHeight, SSD1306_WHITE);

  long constrainedPosition = constrain(aktuellePosition, 0L, gesamtSchritte);
  int fillWidth = map(constrainedPosition, 0, gesamtSchritte, 0, barWidth - 2);

  display.fillRect(1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(barWidth + 2, barY);
}

// Arrows drift through a fixed field in the travel direction. The drift rate
// follows the step pause, so a slow cal approach also reads as slow.
String motorRunMarquee() {
  const int width = 5;
  const int period = 3;

  int stepMs = (int)currentSpeed * 5;
  stepMs = constrain(stepMs, 50, 320);

  bool up = (currentDirection == DIR_UP);
  int phase = (int)((millis() / (unsigned long)stepMs) % period);

  char buf[width + 1];
  for (int i = 0; i < width; ++i) {
    int slot = (up ? (i - phase) : (i + phase)) % period;
    if (slot < 0) {
      slot += period;
    }
    buf[i] = (slot == 0) ? (up ? '>' : '<') : ' ';
  }
  buf[width] = '\0';
  return String(buf);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setFont();

  String titleStr;
  display.setCursor(0, 0);
  switch (currentPage) {
  case PAGE_MENU:
    titleStr = "HAUPTMENUE";
    break;
  case PAGE_MANUAL:
    titleStr = "MANUELL";
    break;
  case PAGE_QRG_TARGET:
    titleStr = "ZIEL";
    break;
  case PAGE_QRG_SAVE:
    titleStr = "SPEICHERN";
    break;
  case PAGE_INIT:
    titleStr = "INIT";
    break;
  case PAGE_INIT_CONFIRM:
    titleStr = "BESTAETIGUNG";
    break;
  case PAGE_WEB_STATUS:
    if (autoCalState > AUTOCAL_CONFIRM) {
      titleStr = autoCalRetune ? "Tune" : "Auto-Kal";
    } else {
      titleStr =
          bleReady ? "IC-705" : (bleInitialized ? "Bluetooth" : "WLAN");
    }
    break;
  case PAGE_CAL_GRAPH:
    titleStr = "TABELLE";
    break;
  case PAGE_RESULT:
    titleStr = resultTitle.length() > 0 ? resultTitle : "ERGEBNIS";
    break;
  default:
    titleStr = "MENU";
    break;
  }
  display.print(titleStr);

  String stateStr;
  bool statusInHeader = false;
  if (!isMotorActive() && statusMeldungEnde > millis() &&
      statusMeldung.length() > 0 && currentPage != PAGE_MANUAL &&
      currentPage != PAGE_RESULT) {
    stateStr = statusMeldung;
    statusInHeader = true;
  } else {
    switch (currentMotorState) {
    case MOTOR_IDLE:
      if (autoCalState > AUTOCAL_CONFIRM) {
        stateStr = autoCalPollSwr ? "TX" : "Kal";
        statusInHeader = autoCalPollSwr;
      } else {
        stateStr = (currentPage == PAGE_MANUAL) ? "0>HOME" : "IDLE";
      }
      break;
    case MOTOR_MOVING:
      stateStr = motorRunMarquee();
      break;
    case MOTOR_HOMING:
      switch (currentHomingPhase) {
      case HOMING_APPROACH:
        stateStr = "HOME-A";
        break;
      case HOMING_CLEAR:
        stateStr = "HOME-F";
        break;
      case HOMING_SLOW_SEEK:
        stateStr = "HOME-0";
        break;
      }
      break;
    case MOTOR_HOMING_AT_ENDSTOP:
      switch (currentHomingPhase) {
      case HOMING_APPROACH:
        stateStr = "HOME@-A";
        break;
      case HOMING_CLEAR:
        stateStr = "HOME@-F";
        break;
      case HOMING_SLOW_SEEK:
        stateStr = "HOME@-0";
        break;
      }
      break;
    default:
      stateStr = "??";
      break;
    }
  }

  const int charW = 6;
  int maxChars = (SCREEN_WIDTH / charW) - (int)titleStr.length() - 1;
  if (maxChars < 4) {
    maxChars = 4;
  }
  if ((int)stateStr.length() > maxChars) {
    stateStr = stateStr.substring(0, maxChars);
  }
  int stateX = SCREEN_WIDTH - (int)stateStr.length() * charW;
  display.setCursor(stateX, 0);
  if (motorAktiv || statusInHeader) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  }
  display.print(stateStr);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);

  display.setFont(&FreeSansBold12pt7b);

  switch (currentPage) {
  case PAGE_MENU:
    display.setFont();
    display.setTextSize(1);

    display.setCursor(0, 12);
    display.print("1 > Automatik");
    display.setCursor(0, 20);
    display.print("2 > Manuell");
    display.setCursor(0, 28);
    display.print("3 > QRG speichern");
    display.setCursor(0, 36);
    display.print("4 > Reset/Export");
    display.setCursor(0, 44);
    display.print("5 > IC-705 / WLAN");
    display.setCursor(0, 52);
    display.print("6 > Tabelle");
    break;

  case PAGE_MANUAL: {
    display.setCursor(0, 30);
    display.print("P:");
    display.print(aktuellePosition);

    display.setCursor(0, 51);
    display.print("F:");

    float currentFreq = getFrequencyFromPosition(aktuellePosition);
    if (currentFreq > 1000) {
      display.print((long)round(currentFreq));
      display.print("k");
    } else if (currentFreq > 0) {
      display.print((long)round(currentFreq * 1000));
      display.print("Hz");
    } else {
      display.print("0");
    }
  } break;

  case PAGE_QRG_TARGET: {
    display.setFont();
    display.setTextSize(1);
    display.setCursor(0, 13);
    display.print("F: ");
    display.print((long)round(getFrequencyFromPosition(aktuellePosition)));
    display.print("kHz (P");
    display.print(aktuellePosition);
    display.print(")");

    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(0, 38);
    display.print("ZIEL");

    int cursorX = display.getCursorX();
    display.print(eingabePuffer);
    cursorX = display.getCursorX();

    if (cursorVisible && !isMotorActive() && eingabePuffer.length() < 5) {
      display.setCursor(cursorX, 38);
      display.print("|");
    }

    display.setFont();
    display.setTextSize(1);
    display.setCursor(0, 44);
    if (eingabePuffer.length() > 0) {
      display.print("Est. Pos: ");
      float estPos = getPositionFromFrequency(eingabePuffer.toFloat());
      display.print((long)round(estPos));
    }
  } break;

  case PAGE_QRG_SAVE: {
    display.setFont();
    display.setTextSize(1);
    display.setCursor(0, 13);
    display.print("F: ");
    display.print((long)round(getFrequencyFromPosition(aktuellePosition)));
    display.print("kHz (P");
    display.print(aktuellePosition);
    display.print(")");

    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(0, 38);
    display.print("F:");

    int cursorX = display.getCursorX();
    display.print(eingabePuffer);
    cursorX = display.getCursorX();

    if (cursorVisible && !isMotorActive() && eingabePuffer.length() < 5) {
      display.setCursor(cursorX, 38);
      display.print("|");
    }

    display.setFont();
    display.setTextSize(1);
    display.setCursor(0, 44);
    display.print("Punkte: ");
    display.print(kalibrierTabelle.size());
    display.print(" / ");
    display.print(MAX_SPEICHERPUNKTE);
  } break;

  case PAGE_INIT:
    display.setFont();
    display.setTextSize(1);

    display.setCursor(0, 13);
    display.print("KALIBRIERUNG RESET");
    display.setCursor(0, 23);
    display.print("Akt. Pkt.: ");
    display.print(kalibrierTabelle.size());

    display.drawFastHLine(0, 33, SCREEN_WIDTH, SSD1306_WHITE);

    display.setCursor(0, 35);
    display.print("0>Export  1>Pos-Test");
    display.setCursor(0, 45);
    if (lastHomingErrorValid) {
      display.print("Err ");
      display.print(lastHomingError);
      display.print("  # Reset");
    } else {
      display.print("# > Reset bestaetigen");
    }
    break;

  case PAGE_INIT_CONFIRM: {
    display.setFont();
    display.setTextSize(1);

    display.setCursor(0, 13);
    display.print("Bestaetigen mit 1234#");

    display.drawFastHLine(0, 23, SCREEN_WIDTH, SSD1306_WHITE);

    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(0, 45);

    int cursorX = display.getCursorX();
    display.print(eingabePuffer);
    cursorX = display.getCursorX();

    if (cursorVisible && eingabePuffer.length() < 4) {
      display.setCursor(cursorX, 45);
      display.print("|");
    }
  } break;

  case PAGE_WEB_STATUS:
    display.setFont();
    display.setTextSize(1);
    if (autoCalState == AUTOCAL_CONFIRM && autoCalBandIndex >= 0) {
      display.setCursor(0, 13);
      display.print(autoCalBands[autoCalBandIndex].name);
      display.print(" kalibrieren?");
      display.setCursor(0, 25);
      display.print(String(autoCalBands[autoCalBandIndex].startKhz));
      display.print("-");
      display.print(String(autoCalBands[autoCalBandIndex].endKhz));
      display.print(" kHz");
      display.setCursor(0, 43);
      display.print("# Start");
      display.setCursor(0, 55);
      display.print("* Abbruch");
    } else if (autoCalState > AUTOCAL_CONFIRM &&
               (autoCalBandIndex >= 0 || autoCalRetune)) {
      display.setCursor(0, 13);
      if (autoCalRetune) {
        display.print("Nachstimmen");
      } else {
        display.print(autoCalBands[autoCalBandIndex].name);
        display.print("  ");
        display.print(autoCalIndex + 1);
        display.print("/");
        display.print(autoCalCount);
      }
      display.setCursor(0, 25);
      if (autoCalIndex >= 0 && autoCalIndex < autoCalCount) {
        display.print("QRG ");
        display.print(String(autoCalFreqs[autoCalIndex], 0));
        display.print(" kHz");
      }
      display.setCursor(0, 37);
      display.print("SWR ");
      if (bleSwrValid && autoCalPollSwr) {
        display.print(formatSwr(bleSwrRaw));
      } else if (autoCalMinSwr < 255) {
        display.print(formatSwr(autoCalMinSwr));
      } else {
        display.print("--");
      }
      {
        String posStr = String(aktuellePosition);
        int posX = SCREEN_WIDTH - (int)posStr.length() * 6;
        if (posX < 72) {
          posX = 72;
        }
        display.setCursor(posX, 37);
        display.print(posStr);
      }
      display.setCursor(0, 55);
      display.print("* Abbruch");
    } else if (bleReady) {
      display.setCursor(0, 13);
      display.print(bleTxActive ? "PTT TX  " : "PTT RX  ");
      display.print(btSettings.trackingEnabled ? "Trk AN" : "Trk AUS");
      display.setCursor(0, 25);
      if (bleRigFreqHz > 0) {
        display.print("QRG ");
        display.print(String(bleRigFreqHz / 1000.0f, 2));
        display.print(" kHz");
      } else {
        display.print("QRG warten...");
      }
      display.setCursor(0, 37);
      display.print("0 Trk   1 WLAN/BT");
      display.setCursor(0, 47);
      display.print("2 Kal   3 Nachstimmen");
      display.setCursor(0, 57);
      display.print("* Menue");
    } else if (bleInitialized) {
      display.setCursor(0, 13);
      display.print("Warten auf IC-705");
      display.setCursor(0, 23);
      display.print(btSettings.trackingEnabled ? "Tracking AN" : "Tracking AUS");
      display.setCursor(0, 37);
      display.print("0 Tracking ein/aus");
      display.setCursor(0, 47);
      display.print("1 WLAN/BT  * Menue");
    } else {
      display.setCursor(0, 13);
      display.print("WLAN: " + String(ssid));
      display.setCursor(0, 23);
      display.print(WiFi.softAPIP().toString());
      display.setCursor(0, 33);
      display.print(btSettings.trackingEnabled ? "Tracking AN" : "Tracking AUS");
      display.setCursor(0, 43);
      display.print("0 Tracking ein/aus");
      display.setCursor(0, 53);
      display.print("1 WLAN/BT  * Menue");
    }
    break;

  case PAGE_CAL_GRAPH:
    drawCalTableGraph();
    break;

  case PAGE_RESULT:
    display.setFont();
    display.setTextSize(1);
    display.setCursor(0, 14);
    display.print(resultL1);
    display.setCursor(0, 24);
    display.print(resultL2);
    display.setCursor(0, 34);
    display.print(resultL3);
    display.setCursor(0, 44);
    display.print(resultL4.length() > 0 ? resultL4 : "* Menue");
    break;
  }

  display.setFont();
  display.setTextSize(1);

  // These pages draw their own footer. A motor-active fillRect at y=54
  // would clip the top pixels of "* Abbruch" / menu text.
  bool pageOwnsFooter = (currentPage == PAGE_WEB_STATUS ||
                         currentPage == PAGE_RESULT ||
                         currentPage == PAGE_MENU ||
                         currentPage == PAGE_CAL_GRAPH);

  if (!pageOwnsFooter) {
    if (isMotorActive()) {
      display.fillRect(0, 54, SCREEN_WIDTH, 10, SSD1306_BLACK);
    }
    if (currentMotorState == MOTOR_HOMING ||
        currentMotorState == MOTOR_HOMING_AT_ENDSTOP) {
      display.setCursor(0, 55);
      display.print("HOMING... (# Abbr)");
    } else {
      bool drawBar = (currentPage != PAGE_INIT &&
                      currentPage != PAGE_INIT_CONFIRM);

      if (drawBar) {
        drawPositionBar();
      }

      if (currentPage == PAGE_INIT || currentPage == PAGE_INIT_CONFIRM) {
        display.setCursor(0, 55);
        display.print("Zurueck: * Taste");
      } else if (drawBar) {
        display.setCursor(120, 55);
        display.print("*");
      }
    }
  }
  display.display();
}