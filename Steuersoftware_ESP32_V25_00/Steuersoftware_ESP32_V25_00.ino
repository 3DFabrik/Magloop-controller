/*
 ------------------------------------------------------------
 Projekt: Variablenkondensator-Steuerung V25.00
 ------------------------------------------------------------

 FUNKTIONSBESCHREIBUNG:
 Steuersoftware für einen Schrittmotor, der einen variablen Kondensator (VC)
 über einen ESP32 WROOM antreibt. Die Steuerung erfolgt über ein 4x3 Tastenfeld
 und einem 128x64 OLED-Display (SSD1306) sowie über eine Web-Schnittstelle.
 - Frequenzbasierte Positionierung mittels Kalibrierungstabelle (EEPROM).
 - Zwei-Geschwindigkeits-Homing für bessere Genauigkeit.
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
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Keypad.h>
#include <WebServer.h>
#include <WebSocketsServer.h> // WebSocket library - https://github.com/Links2004/arduinoWebSockets
#include <WiFi.h>
#include <Wire.h>
#include <algorithm>
#include <ctype.h>
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
const int BACKLASH_STEPS = 50;
const int HOMING_BACKOFF_STEPS = 200;
// Backlash compensation state variables
volatile bool backlashCompensationPending = false;
volatile long backlashTargetPosition = 0;

// Motor speed constants
const int SPEED_FAST = 2;   // Fast speed for initial homing
const int SPEED_NORMAL = 3; // Normal speed for regular movement
const int SPEED_SLOW = 8;   // Slow speed for precise homing

// Simplified Motor State Machine
enum MotorState {
  MOTOR_IDLE,
  MOTOR_HOMING,
  MOTOR_HOMING_AT_ENDSTOP,
  MOTOR_MOVING
};

enum HomingPhase {
  HOMING_APPROACH,      // Direct approach to endstop
  HOMING_BACK_OFF,      // Move back from endstop after hit
  HOMING_SLOW_APPROACH, // Slow approach for precision
  HOMING_FINAL_RELEASE  // Move up until endstop is released
};

enum Direction {
  DIR_UP,  // Away from endstop (lower frequency)
  DIR_DOWN // Towards endstop (higher frequency)
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

// EEPROM configuration
#define EEPROM_SIZE 4096
#define ADRESSE_LETZTE_POSITION 0
#define ADRESSE_PUNKTE_ZAEHLER (ADRESSE_LETZTE_POSITION + sizeof(long))
#define ADRESSE_ERSTER_PUNKT 100
#define MAX_SPEICHERPUNKTE 100
#define ADRESSE_MOTOR_RUNNING_FLAG 1000

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
  MAX_PAGE_COUNT = 7
};
MenuPage currentPage = PAGE_MENU;

// UI state variables
String eingabePuffer = "";
String statusMeldung = "System Start...";
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 250;
unsigned long statusMeldungEnde = 0;

// Web server configuration
WebServer server(80);
WebSocketsServer webSocket =
    WebSocketsServer(81); // WebSocket server on port 81
const char *ssid = "AntennaTuner";
const char *password = "12345678";

// Web calibration state
struct WebCalibrationState {
  bool active = false;
  int currentStep = 0;
  float targetFrequency = 0;
  float minFrequency = 0;
  float maxFrequency = 0;
  std::vector<float> calibrationFrequencies;
};
WebCalibrationState webCalibration;

// Function prototypes
void updateDisplay();
void drawPositionBar();
void processKeypad(char taste);
void saveCurrentPosition();
void loadKalibrierTabelle();
void sortKalibrierTabelle();
float getPositionFromFrequency(float frequenz);
float getFrequencyFromPosition(long position);
void saveSpeicherpunkt(long pos, float freq);
void resetKalibrierung();
void setStatusMessage(String msg, unsigned int durationMs);
void jumpToPage(MenuPage targetPage);
void validateAndCleanKalibrierTabelle();
void dumpKalibrierTabelle();

// WebSocket function prototypes
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length);
void broadcastStatusUpdate();

// Web server function prototypes
void setupWiFi();
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
String getHtmlHeader();
String getHtmlFooter();
String getStateText(MotorState state);

// Motor control interface functions
void startHoming();
void startHomingAtEndstop();
void startMove(long steps, int speed);
void stopMotor();
bool isMotorActive();
long getCurrentPosition();
String motorStateToString(MotorState state);
String directionToString(Direction dir);
String homingPhaseToString(HomingPhase phase);

// Application logic functions
void moveToFrequency(float frequency);
void moveToPosition(long position);
void performBacklashCompensation(long targetPosition);

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
  case HOMING_BACK_OFF:
    return "BACK_OFF";
  case HOMING_SLOW_APPROACH:
    return "SLOW_APPROACH";
  case HOMING_FINAL_RELEASE:
    return "FINAL_RELEASE";
  default:
    return "UNKNOWN";
  }
}

// Motor control interface implementation
void startHoming() {
  if (isMotorActive()) {
    Serial.println("[MOTOR] Cannot start homing - motor already active");
    return;
  }

  // Check if endstop is already triggered
  if (digitalRead(ENDSTOP_PIN) == LOW) {
    Serial.println(
        "[MOTOR] Endstop already triggered, starting homing at endstop");
    startHomingAtEndstop();
    return;
  }

  Serial.println("[MOTOR] Starting homing sequence");
  currentMotorState = MOTOR_HOMING;
  currentHomingPhase = HOMING_APPROACH;
  currentSpeed = SPEED_FAST;
  currentDirection = DIR_DOWN; // Move towards endstop (decreasing position)

  // Use the full range of steps for homing, not just 200
  targetPosition = -HOMING_BACKOFF_STEPS; // Move past endstop
  stepsRemaining = gesamtSchritte;        // Use full range as safety limit
  motorAktiv = true;
  endstopHit = false;

  Serial.println(
      "[MOTOR] Homing phase: " + homingPhaseToString(currentHomingPhase) +
      " Direction: " + directionToString(currentDirection) + " Speed: " +
      String(currentSpeed) + " Max Steps: " + String(stepsRemaining) +
      " Current Position: " + String(aktuellePosition));
}

void startHomingAtEndstop() {
  if (isMotorActive()) {
    Serial.println(
        "[MOTOR] Cannot start homing at endstop - motor already active");
    return;
  }

  Serial.println("[MOTOR] Starting homing at endstop sequence");
  currentMotorState = MOTOR_HOMING_AT_ENDSTOP;
  currentHomingPhase =
      HOMING_BACK_OFF; // Start with backoff since we're already at endstop
  currentSpeed = SPEED_NORMAL;
  currentDirection = DIR_UP;             // Move away from endstop
  targetPosition = HOMING_BACKOFF_STEPS; // Move back from endstop
  stepsRemaining = abs(targetPosition - aktuellePosition);
  motorAktiv = true;
  endstopHit = false;

  Serial.println("[MOTOR] Homing at endstop phase: " +
                 homingPhaseToString(currentHomingPhase) +
                 " Direction: " + directionToString(currentDirection) +
                 " Speed: " + String(currentSpeed) +
                 " Steps: " + String(stepsRemaining));
}

void startMove(long steps, int speed) {
  if (isMotorActive()) {
    Serial.println("[MOTOR] Cannot start move - motor already active");
    return;
  }

  if (steps == 0) {
    Serial.println("[MOTOR] No steps to move");
    return;
  }

  currentDirection =
      (steps > 0)
          ? DIR_UP
          : DIR_DOWN; // FIXED: Positive steps = UP, Negative steps = DOWN
  targetPosition = aktuellePosition + steps;

  // Check if target position is within bounds
  if (targetPosition < 0) {
    Serial.println("[MOTOR] Target position below 0, adjusting to 0");
    targetPosition = 0;
  } else if (targetPosition > gesamtSchritte) {
    Serial.println("[MOTOR] Target position above max, adjusting to max");
    targetPosition = gesamtSchritte;
  }

  stepsRemaining = abs(targetPosition - aktuellePosition);
  currentSpeed = speed;
  currentMotorState = MOTOR_MOVING;
  motorAktiv = true;
  endstopHit = false;

  Serial.println(
      "[MOTOR] Starting move: " + String(steps) + " steps at speed " +
      String(speed) + " Direction: " + directionToString(currentDirection) +
      " From: " + String(aktuellePosition) + " To: " + String(targetPosition));
}

void stopMotor() {
  if (!motorAktiv) {
    Serial.println("[MOTOR] Motor already stopped");
    return;
  }

  Serial.println("[MOTOR] Stopping motor. State: " +
                 motorStateToString(currentMotorState) +
                 " Position: " + String(aktuellePosition));

  motorAktiv = false;
  stepsRemaining = 0;

  // Turn off all motor pins
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], LOW);
  }

  // Check if we need to complete backlash compensation
  if (backlashCompensationPending) {
    Serial.println("[MOTOR] Completing backlash compensation");
    backlashCompensationPending = false;
    long steps = backlashTargetPosition - aktuellePosition;
    startMove(steps, SPEED_NORMAL);
    return;
  }

  // If we were homing and hit the endstop, set position to 0
  if ((currentMotorState == MOTOR_HOMING ||
       currentMotorState == MOTOR_HOMING_AT_ENDSTOP) &&
      endstopHit) {
    aktuellePosition = 0;
    Serial.println("[MOTOR] Homing completed, position set to 0");
  }

  // Constrain position to valid range
  aktuellePosition = constrain(aktuellePosition, 0L, gesamtSchritte);

  // Return to idle state
  currentMotorState = MOTOR_IDLE;

  // Save current position
  saveCurrentPosition();

  Serial.println("[MOTOR] Motor stopped. Final position: " +
                 String(aktuellePosition));
}

bool isMotorActive() { return motorAktiv; }

long getCurrentPosition() { return aktuellePosition; }

// Motor task implementation
void motorTask(void *pvParameters) {
  unsigned long lastStepTime = 0;

  for (;;) {
    if (motorAktiv) {
      // Check if it's time for the next step
      if (millis() - lastStepTime >= currentSpeed) {
        lastStepTime = millis();

        // Check for endstop during movement
        if (endstopTriggered) {
          Serial.println("[MOTOR] Endstop triggered during movement");
          endstopTriggered = false;
          endstopHit = true;

          // Handle endstop based on current state
          if (currentMotorState == MOTOR_MOVING) {
            Serial.println("[MOTOR] Endstop hit during normal move, starting "
                           "homing at endstop");
            stopMotor();
            startHomingAtEndstop();
            continue;
          } else if (currentMotorState == MOTOR_HOMING &&
                     currentHomingPhase == HOMING_APPROACH) {
            Serial.println(
                "[MOTOR] Endstop hit during homing approach, starting backoff");
            currentHomingPhase = HOMING_BACK_OFF;
            currentDirection = DIR_UP; // Move away from endstop
            targetPosition = HOMING_BACKOFF_STEPS;
            stepsRemaining = abs(targetPosition - aktuellePosition);
            currentSpeed = SPEED_NORMAL;
            continue;
          } else if (currentMotorState == MOTOR_HOMING_AT_ENDSTOP &&
                     currentHomingPhase == HOMING_SLOW_APPROACH) {
            Serial.println("[MOTOR] Endstop hit during slow approach, starting "
                           "final release");
            currentHomingPhase = HOMING_FINAL_RELEASE;
            currentDirection = DIR_UP; // Move away from endstop
            targetPosition = HOMING_BACKOFF_STEPS;
            stepsRemaining = abs(targetPosition - aktuellePosition);
            currentSpeed = SPEED_SLOW;
            continue;
          }
        }

        // Take a step
        currentStep = (currentDirection == DIR_DOWN)
                          ? (currentStep + 1) % 8
                          : (currentStep - 1 + 8) % 8;

        for (int i = 0; i < 4; i++) {
          digitalWrite(motorPins[i], stepSequence[currentStep][i]);
        }

        // Update position - DIR_DOWN should decrease position
        aktuellePosition += (currentDirection == DIR_DOWN) ? -1 : 1;

        // Allow position to go negative during homing
        if (currentMotorState == MOTOR_HOMING ||
            currentMotorState == MOTOR_HOMING_AT_ENDSTOP) {
          aktuellePosition = constrain(aktuellePosition, -HOMING_BACKOFF_STEPS,
                                       gesamtSchritte + HOMING_BACKOFF_STEPS);
        } else {
          aktuellePosition = constrain(aktuellePosition, 0L, gesamtSchritte);
        }

        // Decrease remaining steps
        if (stepsRemaining > 0) {
          stepsRemaining--;
        }

        // Check if movement is complete
        if (stepsRemaining <= 0) {
          Serial.println("[MOTOR] Movement completed");

          // Handle state-specific completion logic
          if (currentMotorState == MOTOR_MOVING) {
            Serial.println("[MOTOR] Normal move completed");
            stopMotor();
          } else if (currentMotorState == MOTOR_HOMING ||
                     currentMotorState == MOTOR_HOMING_AT_ENDSTOP) {
            // Handle homing phase transitions
            if (currentHomingPhase == HOMING_APPROACH) {
              Serial.println(
                  "[MOTOR] Homing approach completed without hitting endstop");
              stopMotor();
            } else if (currentHomingPhase == HOMING_BACK_OFF) {
              Serial.println(
                  "[MOTOR] Homing backoff completed, starting slow approach");
              currentHomingPhase = HOMING_SLOW_APPROACH;
              currentDirection = DIR_DOWN; // Move towards endstop
              targetPosition = -HOMING_BACKOFF_STEPS;
              stepsRemaining = abs(targetPosition - aktuellePosition);
              currentSpeed = SPEED_SLOW;
            } else if (currentHomingPhase == HOMING_SLOW_APPROACH) {
              Serial.println("[MOTOR] Homing slow approach completed without "
                             "hitting endstop");
              stopMotor();
            } else if (currentHomingPhase == HOMING_FINAL_RELEASE) {
              // Check if endstop is released
              if (digitalRead(ENDSTOP_PIN) == HIGH) {
                Serial.println("[MOTOR] Endstop released, homing complete");
                stopMotor();
              } else {
                // Endstop still triggered, continue moving up
                Serial.println(
                    "[MOTOR] Endstop still triggered, continuing release");
                targetPosition = aktuellePosition + 50;
                stepsRemaining = 50;
              }
            }
          }
        }
      }
    } else {
      // Motor is not active, ensure all pins are low
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

void moveToPosition(long position) {
  Serial.println("[APP] Moving to position: " + String(position));

  // Check if we need backlash compensation
  if (position < getCurrentPosition()) {
    // Moving down (to lower position number) - need backlash compensation
    Serial.println("[APP] Applying backlash compensation");
    performBacklashCompensation(position);
  } else {
    // Moving up (to higher position number) - no backlash compensation needed
    long steps = position - getCurrentPosition();
    startMove(steps, SPEED_NORMAL);
  }
}

void performBacklashCompensation(long targetPosition) {
  Serial.println("[APP] Performing backlash compensation to position: " +
                 String(targetPosition));

  // Check if we have enough room for backlash compensation
  if (targetPosition >= BACKLASH_STEPS) {
    // Move past the target position (overshoot by 50 steps)
    long tempTarget = targetPosition - BACKLASH_STEPS;
    long steps = tempTarget - getCurrentPosition();
    startMove(steps, SPEED_NORMAL);

    // Set a flag to indicate we need to move back after this move completes
    backlashCompensationPending = true;
    backlashTargetPosition = targetPosition;
  } else {
    // Not enough room for backlash compensation, move directly to target
    Serial.println(
        "[APP] Not enough room for backlash compensation, moving directly");
    long steps = targetPosition - getCurrentPosition();
    startMove(steps, SPEED_NORMAL);
  }
}

void saveCurrentPosition() {
  long positionToSave = constrain(aktuellePosition, 0L, gesamtSchritte);
  Serial.println("[SYSTEM] Saving position: " + String(positionToSave));
  EEPROM.put(ADRESSE_LETZTE_POSITION, positionToSave);
  EEPROM.commit();
}

void setStatusMessage(String msg, unsigned int durationMs) {
  Serial.println("[SYSTEM] Status: " + msg);
  statusMeldung = msg;
  statusMeldungEnde = millis() + durationMs;
}

void jumpToPage(MenuPage targetPage) {
  Serial.println("[UI] Jumping to page: " + String(targetPage));
  currentPage = targetPage;
  eingabePuffer = "";
}

void setupWiFi() {
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
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
    String status =
        "{\"type\":\"status\",\"position\":" + String(aktuellePosition) +
        ",\"frequency\":" +
        String(getFrequencyFromPosition(aktuellePosition), 2) +
        ",\"state\":\"" + getStateText(currentMotorState) + "\"}";
    webSocket.sendTXT(num, status);

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
        saveSpeicherpunkt(aktuellePosition, frequency);

        // Broadcast updated calibration data to all clients
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

        // Send success response
        webSocket.sendTXT(num,
                          "{\"type\":\"calibrate_result\",\"success\":true}");
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
    case HOMING_BACK_OFF:
      return "Homing: Back Off";
    case HOMING_SLOW_APPROACH:
      return "Homing: Slow Approach";
    case HOMING_FINAL_RELEASE:
      return "Homing: Final Release";
    }
    return "Homing";
  case MOTOR_HOMING_AT_ENDSTOP:
    switch (currentHomingPhase) {
    case HOMING_APPROACH:
      return "Homing@Endstop: Approach";
    case HOMING_BACK_OFF:
      return "Homing@Endstop: Back Off";
    case HOMING_SLOW_APPROACH:
      return "Homing@Endstop: Slow Approach";
    case HOMING_FINAL_RELEASE:
      return "Homing@Endstop: Final Release";
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
      getStateText(currentMotorState) + "\"}";
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
      webCalibration.active = true;
      webCalibration.currentStep = 0;
      webCalibration.calibrationFrequencies.clear();
      // IMPORTANT: Do NOT clear the existing table anymore

      // Start with homing
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
  </style>
</head>
<body>
  <div class="container">
    <div style="display: flex; justify-content: space-between; align-items: center;">
      <h1>Antenna Tuner Control</h1>
      <div id="connectionStatus" class="connection-status disconnected">Connecting...</div>
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
        document.getElementById('connectionStatus').textContent = 'Connected';
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
          if (freqElement) freqElement.textContent = data.frequency.toFixed(2) + ' kHz';
          if (stateElement) stateElement.textContent = data.state;
          
          // Update position and frequency on calibration page
          const calibPosElement = document.getElementById('currentPos');
          if (calibPosElement) {
            calibPosElement.textContent = data.position + ' steps (' + data.frequency.toFixed(2) + ' kHz)';
          }
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
            if (data.success) {
              feedback.textContent = 'Calibration successful!';
              feedback.style.backgroundColor = '#2E7D32';
            } else {
              feedback.textContent = 'Calibration failed!';
              feedback.style.backgroundColor = '#8B0000';
            }
            feedback.style.display = 'block';
            setTimeout(() => { feedback.style.display = 'none'; }, 3000);
          }
        }
      };
      
      socket.onclose = function(event) {
        console.log('WebSocket disconnected, attempting to reconnect...');
        document.getElementById('connectionStatus').textContent = 'Disconnected';
        document.getElementById('connectionStatus').className = 'connection-status disconnected';
        setTimeout(initWebSocket, 2000);
      };
      
      socket.onerror = function(error) {
        console.error('WebSocket error:', error);
        document.getElementById('connectionStatus').textContent = 'Error';
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
  html += "<h2>Manual Calibration</h2>";
  html += "<p>Start a non-destructive calibration process. Your existing table "
          "will be preserved.</p>";
  html += "<form action=\"/calibration/start\" method=\"post\">";
  html += "<button type=\"submit\" class=\"button\">Start Calibration "
          "Process</button>";
  html += "</form>";
  html += "</div>";

  html += "<div class=\"card\">";
  html += "<h2>System</h2>";
  html += "<form action=\"/reset\" method=\"post\" onsubmit=\"return "
          "confirm('Are you sure you want to reset the calibration table?')\">";
  html += "<button type=\"submit\" class=\"button button-danger\">Reset "
          "Calibration</button>";
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
  json += "\"calibrationPoints\":" + String(kalibrierTabelle.size());
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
  html += "<h2>Manual Calibration</h2>";
  html += "<p>Current position: <span id=\"currentPos\">" +
          String(aktuellePosition) + "</span> steps (" +
          String(getFrequencyFromPosition(aktuellePosition), 2) + " kHz)</p>";
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
          "style=\"display:inline; margin-left: 10px;\">";
  html += "<button type=\"submit\" class=\"button button-danger\">Abort "
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
        // Calculate target position
        long targetPos = aktuellePosition + steps;

        // Use moveToPosition to ensure backlash compensation is applied
        moveToPosition(targetPos);

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
    saveSpeicherpunkt(aktuellePosition, targetFreq);

    // Run validation immediately
    validateAndCleanKalibrierTabelle();

    // Check for validation errors and report them
    if (statusMeldung.indexOf("KRITISCH") != -1) {
      setStatusMessage("CALIBRATION ERROR! Check table.", 10000);
    } else {
      setStatusMessage("Point saved and validated.", 3000);
    }

    // Send success response
    server.send(200, "text/plain", "Calibration successful");
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

  webCalibration.active = false;
  saveCurrentPosition();
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirecting...");
}

void handleCalibrationAbort() {
  Serial.println("[WEB] Calibration abort request received");

  webCalibration.active = false;
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

void setup() {
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_PIN), endstopISR, FALLING);
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("\n--- ESP32 WROOM System Start (V25.00) ---");
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

  xTaskCreatePinnedToCore(motorTask, "MotorControl", 2048, NULL, 2, NULL, 0);

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
    stopMotor();
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
  setStatusMessage("WLAN: " + String(ssid), 2000);

  currentPage = PAGE_MENU;
  updateDisplay();
}

void loop() {
  unsigned long jetzt = millis();
  static long lastBroadcastPosition = -1;
  static unsigned long lastBroadcastTime = 0;
  char taste = keypad.getKey();

  // Always process server requests
  server.handleClient();
  webSocket.loop();

  // Always process keypad input
  if (taste) {
    Serial.println("[KEYPAD] Key pressed: " + String(taste));
    processKeypad(taste);
  }

  // Broadcast position updates when position changes or every second
  if (aktuellePosition != lastBroadcastPosition ||
      (jetzt - lastBroadcastTime > 1000)) {
    broadcastStatusUpdate();
    lastBroadcastPosition = aktuellePosition;
    lastBroadcastTime = jetzt;
  }

  if (jetzt - lastCursorToggle >= cursorInterval) {
    lastCursorToggle = jetzt;
    cursorVisible = !cursorVisible;
  }

  if (taste == '*') {
    if (isMotorActive()) {
      Serial.println("[KEYPAD] Stop command received");
      stopMotor();
    } else {
      if (currentPage == PAGE_INIT_CONFIRM) {
        jumpToPage(PAGE_MENU);
      } else {
        jumpToPage(PAGE_MENU);
      }
    }
  }

  if (statusMeldungEnde > 0 && jetzt >= statusMeldungEnde) {
    statusMeldungEnde = 0;
    statusMeldung = "";
    if (currentPage == PAGE_INIT_CONFIRM) {
      jumpToPage(PAGE_MENU);
    }
  }

  if (jetzt - lastDisplayUpdate >= displayInterval) {
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

    if (isMotorActive()) {
      stopMotor();
    } else {
      if (currentPage == PAGE_INIT_CONFIRM) {
        jumpToPage(PAGE_MENU);
      } else {
        jumpToPage(PAGE_MENU);
      }
    }
    return;
  }

  // Handle '#' key (stop or execute)
  if (taste == '#') {
    Serial.println("[KEYPAD] Execute/stop key pressed");

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
      }
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
                       (upTaste ? "up" : "down"));

        // Allow manual movement only if motor is not active
        if (!isMotorActive()) {
          // Calculate target position
          long targetPos = aktuellePosition + steps;

          // Use moveToPosition to ensure backlash compensation is applied
          moveToPosition(targetPos);
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
  validateAndCleanKalibrierTabelle();
  Serial.println("[SYSTEM] Loaded " + String(kalibrierTabelle.size()) +
                 " calibration points");
}

void validateAndCleanKalibrierTabelle() {
  Serial.println("[SYSTEM] Validating calibration table");
  if (kalibrierTabelle.size() < 2)
    return;

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
      Serial.println(" -> Grenzwertverletzung (außerhalb P0/P_Ende).");
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
        Serial.println(" -> Ausreißer nach oben (bricht Monotonie zur Folge).");
        punktFehlerhaft = true;
      }
    }

    if (punktFehlerhaft) {
      fehlerZaehler++;

      if (fehlerZaehler > 1) {
        setStatusMessage("KRITISCH! Mehrere Fehler. Tabelle ungültig.", 4000);
        Serial.println("ABBRUCH: Mehr als ein Fehler in der Tabelle gefunden.");
        return;
      }
    } else {
      tempTabelle.push_back(currentPoint);
    }
  }

  if (fehlerZaehler > 0 || tempTabelle.size() < initialSize) {
    if (tempTabelle.size() < 2) {
      resetKalibrierung();
      setStatusMessage("FEHLER: Tabelle zu kurz. Reset auf 2 Punkte.", 4000);
    } else {
      kalibrierTabelle = tempTabelle;

      EEPROM.put(ADRESSE_PUNKTE_ZAEHLER, (int)kalibrierTabelle.size());
      for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
        EEPROM.put(ADRESSE_ERSTER_PUNKT + i * sizeof(Speicherpunkt),
                   kalibrierTabelle[i]);
      }
      EEPROM.commit();

      setStatusMessage(
          "Tab. korrigiert: " + String(kalibrierTabelle.size()) + " Pkt", 3000);
    }
  }
}

void resetKalibrierung() {
  Serial.println("[SYSTEM] Resetting calibration table");
  kalibrierTabelle.clear();
  saveSpeicherpunkt(0, 5150.0f);               // Start bei 0 Schritten
  saveSpeicherpunkt(gesamtSchritte, 24641.0f); // Ende bei 11640 Schritten
  EEPROM.commit();
}

void sortKalibrierTabelle() {
  std::sort(kalibrierTabelle.begin(), kalibrierTabelle.end(),
            [](const Speicherpunkt &a, const Speicherpunkt &b) {
              return a.position < b.position;
            });
}

void saveSpeicherpunkt(long pos, float freq) {
  Serial.println("[SYSTEM] Saving calibration point: Pos=" + String(pos) +
                 " Freq=" + String(freq, 2) + "kHz");
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
      return;
    }
  }

  if (saveNeeded) {
    sortKalibrierTabelle();
    EEPROM.put(ADRESSE_PUNKTE_ZAEHLER, (int)kalibrierTabelle.size());
    for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
      EEPROM.put(ADRESSE_ERSTER_PUNKT + i * sizeof(Speicherpunkt),
                 kalibrierTabelle[i]);
    }
    EEPROM.commit();

    validateAndCleanKalibrierTabelle();
    setStatusMessage("Table aktualisiert", 2000);
  }
}

float getPositionFromFrequency(float frequenz) {
  if (kalibrierTabelle.size() < 2)
    return 0;
  if (frequenz <= kalibrierTabelle.front().frequenz)
    return (float)kalibrierTabelle.front().position;
  if (frequenz >= kalibrierTabelle.back().frequenz)
    return (float)kalibrierTabelle.back().position;
  for (size_t i = 0; i < kalibrierTabelle.size() - 1; ++i) {
    const Speicherpunkt &p1 = kalibrierTabelle[i];
    const Speicherpunkt &p2 = kalibrierTabelle[i + 1];
    if (frequenz >= p1.frequenz && frequenz <= p2.frequenz) {
      float pos = p1.position + (p2.position - p1.position) *
                                    (frequenz - p1.frequenz) /
                                    (p2.frequenz - p1.frequenz);
      return pos;
    }
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

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setFont();

  display.setCursor(0, 0);
  switch (currentPage) {
  case PAGE_MENU:
    display.print("HAUPTMENUE");
    break;
  case PAGE_MANUAL:
    display.print("MANUELL");
    break;
  case PAGE_QRG_TARGET:
    display.print("ZIEL");
    break;
  case PAGE_QRG_SAVE:
    display.print("SPEICHERN");
    break;
  case PAGE_INIT:
    display.print("INIT");
    break;
  case PAGE_INIT_CONFIRM:
    display.print("BESTAETIGUNG");
    break;
  case PAGE_WEB_STATUS:
    display.print("WEB STATUS");
    break;
  default:
    display.print("MENU");
    break;
  }

  display.setCursor(100, 0);
  String stateStr;
  switch (currentMotorState) {
  case MOTOR_IDLE:
    stateStr = "IDLE";
    break;
  case MOTOR_MOVING:
    stateStr = "RUN";
    break;
  case MOTOR_HOMING:
    switch (currentHomingPhase) {
    case HOMING_APPROACH:
      stateStr = "HOME-A";
      break;
    case HOMING_BACK_OFF:
      stateStr = "HOME-B";
      break;
    case HOMING_SLOW_APPROACH:
      stateStr = "HOME-S";
      break;
    case HOMING_FINAL_RELEASE:
      stateStr = "HOME-R";
      break;
    }
    break;
  case MOTOR_HOMING_AT_ENDSTOP:
    switch (currentHomingPhase) {
    case HOMING_APPROACH:
      stateStr = "HOME@-A";
      break;
    case HOMING_BACK_OFF:
      stateStr = "HOME@-B";
      break;
    case HOMING_SLOW_APPROACH:
      stateStr = "HOME@-S";
      break;
    case HOMING_FINAL_RELEASE:
      stateStr = "HOME@-R";
      break;
    }
    break;
  default:
    stateStr = "??";
    break;
  }
  if (motorAktiv) {
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

    display.setCursor(0, 13);
    display.print("1 > Automatik");
    display.setCursor(0, 23);
    display.print("2 > Manuell");
    display.setCursor(0, 33);
    display.print("3 > QRG speichern");
    display.setCursor(0, 43);
    display.print("4 > Reset/Export");
    display.setCursor(0, 53);
    display.print("5 > Web Status");
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
    display.print("0 > Tabelle exportieren");

    display.setCursor(0, 45);
    display.print("# > Reset bestaetigen");
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

    display.setCursor(0, 13);
    display.print("WLAN: " + String(ssid));
    display.setCursor(0, 23);
    display.print("IP: ");
    String ip = WiFi.softAPIP().toString();
    if (ip.length() > 15) {
      display.setCursor(0, 31);
      display.print(ip.substring(0, 15));
      display.setCursor(0, 39);
      display.print(ip.substring(15));
    } else {
      display.print(ip);
    }
    display.setCursor(0, 47);
    display.print("Pass: " + String(password));
    break;
  }

  display.setFont();
  display.setTextSize(1);

  if (statusMeldungEnde > millis() || isMotorActive()) {
    display.fillRect(0, 54, SCREEN_WIDTH, 10, SSD1306_BLACK);
  }
  if (statusMeldungEnde > millis() && statusMeldung.length() > 0) {
    display.setCursor(0, 55);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.print(statusMeldung);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  } else if (currentMotorState == MOTOR_HOMING ||
             currentMotorState == MOTOR_HOMING_AT_ENDSTOP) {
    display.setCursor(0, 55);
    display.print("HOMING... (# Abbr)");

  } else {
    bool drawBar =
        (currentPage != PAGE_MENU && currentPage != PAGE_INIT &&
         currentPage != PAGE_INIT_CONFIRM && currentPage != PAGE_WEB_STATUS);

    if (drawBar) {
      drawPositionBar();
    }

    if (currentPage == PAGE_MENU) {
      display.setCursor(0, 55);
      display.print("Waehle Seite 1-5");
    } else if (currentPage == PAGE_INIT || currentPage == PAGE_INIT_CONFIRM ||
               currentPage == PAGE_WEB_STATUS) {
      display.setCursor(0, 55);
      display.print("Zurueck: * Taste");
    } else if (drawBar) {
      display.setCursor(120, 55);
      display.print("*");
    }
  }
  display.display();
}