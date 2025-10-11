
/*
 ------------------------------------------------------------
 Projekt: Variablenkondensator-Steuerung V24.2
 ------------------------------------------------------------
 
 FUNKTIONSBESCHREIBUNG:
 Steuersoftware für einen Schrittmotor, der einen variablen Kondensator (VC)
 über einen ESP32 WROOM antreibt. Die Steuerung erfolgt über ein 4x3 Tastenfeld
 und einem 128x64 OLED-Display (SSD1306).
 - Frequenzbasierte Positionierung mittels Kalibrierungstabelle (EEPROM).
 - Homing (Referenzfahrt) zur Bestimmung des Nullpunktes.
 - Backlash-Kompensation bei Richtungswechsel (Wegfahren vom Nullpunkt).
 
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

#include <Keypad.h>
#include <EEPROM.h>
#include <ctype.h>
#include <vector> 
#include <algorithm>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold12pt7b.h> 
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define SERIAL_BAUDRATE 115200
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = { { '1', '2', '3' }, { '4', '5', '6' }, { '7', '8', '9' }, { '*', '0', '#' } };
byte rowPins[ROWS] = {25, 26, 33, 32}; 
byte colPins[COLS] = {13, 12, 14};      
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
const int motorPins[] = {17, 5, 18, 19}; 
int stepSequence[8][4] = {
    {0, 1, 1, 1}, {0, 0, 1, 1}, {1, 0, 1, 1}, {1, 0, 0, 1}, 
    {1, 1, 0, 1}, {1, 1, 0, 0}, {1, 1, 1, 0}, {0, 1, 1, 0}  
}; 
volatile int currentStep = 0;
volatile bool motorAktiv = false;
const long gesamtSchritte = 11640;
const int BACKLASH_STEPS = 50; 
volatile long aktuellePosition = 0;
volatile long zielPosition = 0; 
volatile bool zielErreicht = false; 
volatile int manualGeschwindigkeit = 0;
volatile bool manualRichtungRechts = true;
volatile bool stoppenErzwungen = false;
volatile bool homingInitialisiert = false; 
volatile long stepsToDrive = 0; 
volatile bool manualDriving = false; 
volatile bool lastRichtungRechts = false; 
volatile long finalTargetPosition = 0; 
volatile bool isBacklashCorrectionNeeded = false; 
volatile bool backlashSequenceRunning = false; 
#define EEPROM_SIZE 4096 
#define ADRESSE_LETZTE_POSITION 0
#define ADRESSE_PUNKTE_ZAEHLER (ADRESSE_LETZTE_POSITION + sizeof(long)) 
#define ADRESSE_ERSTER_PUNKT 100 
#define MAX_SPEICHERPUNKTE 100 
#define ADRESSE_MOTOR_RUNNING_FLAG 1000 
struct Speicherpunkt {
    long position;      
    float frequenz;     
};
std::vector<Speicherpunkt> kalibrierTabelle; 
volatile bool cursorVisible = false;
unsigned long lastCursorToggle = 0;
const unsigned long cursorInterval = 500; 
enum MotorState { STATE_IDLE,
                  STATE_MANUAL_MOVE, 
                  STATE_TARGET_MOVE,
                  STATE_SAVING,
                  STATE_HOMING,
                  STATE_BACKLASH_CORRECTION 
                }; 
MotorState currentMotorState = STATE_IDLE; 
enum MenuPage { PAGE_MENU = 0,             
                PAGE_MANUAL = 1,           
                PAGE_QRG_TARGET = 2,       
                PAGE_QRG_SAVE = 3,         
                PAGE_INIT = 4,             
                PAGE_INIT_CONFIRM = 5,     
                MAX_PAGE_COUNT = 6 };       
MenuPage currentPage = PAGE_MENU;           
String eingabePuffer = "";
String statusMeldung = "System Start..."; 
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 250;
unsigned long statusMeldungEnde = 0; 
void updateDisplay();
void drawPositionBar();
void processKeypad(char taste);
void motorStop();
void saveCurrentPosition(); 
void loadKalibrierTabelle();
void sortKalibrierTabelle();
float getPositionFromFrequency(float frequenz);
float getFrequencyFromPosition(long position); 
void saveSpeicherpunkt(long pos, float freq);
void resetKalibrierung(); 
void startBacklashMove(long newTargetPos, long schritte); 
void setStatusMessage(String msg, unsigned int durationMs); 
void jumpToPage(MenuPage targetPage); 

void motorTask(void* pvParameters) {
  unsigned long lastStepTime = 0;
  for (;;) {
    if (motorAktiv) {
      
      int interval;
      if (manualGeschwindigkeit == 1) {
          interval = 5;  
      } else if (manualGeschwindigkeit == 2) {
          interval = 5;  
      } else { 
          interval = 4;  
      }
      if (millis() - lastStepTime >= interval) {
        lastStepTime = millis();
        bool richtungRechts = manualRichtungRechts;
        if (manualDriving && stepsToDrive <= 0) {
            motorAktiv = false; 
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
        }
        
        if (currentMotorState != STATE_HOMING && currentMotorState != STATE_BACKLASH_CORRECTION) {
            if ((richtungRechts && aktuellePosition <= 0) || 
                (!richtungRechts && aktuellePosition >= gesamtSchritte)) 
            {
                motorAktiv = false; 
                aktuellePosition = constrain(aktuellePosition, 0L, gesamtSchritte); 
                vTaskDelay(1 / portTICK_PERIOD_MS);
                continue;
            }
        }
        
        currentStep = richtungRechts ? (currentStep - 1 + 8) % 8 : (currentStep + 1) % 8;
        for (int i = 0; i < 4; i++) {
          digitalWrite(motorPins[i], stepSequence[currentStep][i]);
        }
        
        aktuellePosition += richtungRechts ? -1 : 1; 
        
        if (currentMotorState == STATE_HOMING || currentMotorState == STATE_BACKLASH_CORRECTION) {
            aktuellePosition = constrain(aktuellePosition, 0L - BACKLASH_STEPS, gesamtSchritte + BACKLASH_STEPS); 
        } else {
            aktuellePosition = constrain(aktuellePosition, 0L, gesamtSchritte);
        }
        
        if (manualDriving && stepsToDrive > 0) {
            stepsToDrive--;
        }
      }
    } else {
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorPins[i], LOW); 
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
void motorStop() {
  motorAktiv = false;
  stoppenErzwungen = false;
  stepsToDrive = 0;
  manualDriving = false;
  
  if (currentMotorState != STATE_HOMING && currentMotorState != STATE_BACKLASH_CORRECTION) {
      aktuellePosition = constrain(aktuellePosition, 0L, gesamtSchritte);
  }
  
  bool running = false;
  EEPROM.put(ADRESSE_MOTOR_RUNNING_FLAG, running);
  EEPROM.commit();
}
void saveCurrentPosition() {
    long positionToSave = constrain(aktuellePosition, 0L, gesamtSchritte);
    EEPROM.put(ADRESSE_LETZTE_POSITION, positionToSave);
    EEPROM.commit();
}
void startBacklashMove(long newTargetPos, long schritte) {
    zielPosition = newTargetPos;
    stepsToDrive = schritte;
    
    bool newDirectionRight = (zielPosition < aktuellePosition); 
    manualRichtungRechts = newDirectionRight;
    lastRichtungRechts = newDirectionRight; 
    manualGeschwindigkeit = 3; 
    manualDriving = (schritte > 0); 
    motorAktiv = true;
    
    if (currentMotorState != STATE_BACKLASH_CORRECTION) { 
      bool running = true;
      EEPROM.put(ADRESSE_MOTOR_RUNNING_FLAG, running); 
      EEPROM.commit();
    }
}
void setStatusMessage(String msg, unsigned int durationMs) {
    statusMeldung = msg;
    statusMeldungEnde = millis() + durationMs;
}
void jumpToPage(MenuPage targetPage) {
    currentPage = targetPage;
    eingabePuffer = "";
    stoppenErzwungen = false;
}
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("\n--- ESP32 WROOM System Start (V24.0) ---");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }
  
  display.setRotation(2); 
  
  display.display(); 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(0,0);
  display.println("INIT OK");
  display.display();
  delay(1000); 
  for (int i = 0; i < 4; i++) pinMode(motorPins[i], OUTPUT);
  
  for (int i = 0; i < 4; i++) digitalWrite(motorPins[i], LOW);
  
  EEPROM.begin(EEPROM_SIZE); 
  
  xTaskCreatePinnedToCore(
    motorTask,        
    "MotorControl",   
    2048,             
    NULL,             
    2,                
    NULL,             
    0                 
  );
  aktuellePosition = 0; 
  homingInitialisiert = false; 
  int anzahlPunkte = 0;
  EEPROM.get(ADRESSE_PUNKTE_ZAEHLER, anzahlPunkte);
  
  if (anzahlPunkte < 2) { 
    resetKalibrierung(); 
    setStatusMessage("INIT: Freq. ges. H-Strt.", 2000); 
  } else {
    loadKalibrierTabelle(); 
    setStatusMessage("Tabelle geladen (" + String(kalibrierTabelle.size()) + " Pkt)", 2000); 
  }
  long lastKnownPosition = 0;
  bool wasRunning = false;
  
  EEPROM.get(ADRESSE_MOTOR_RUNNING_FLAG, wasRunning); 
  EEPROM.get(ADRESSE_LETZTE_POSITION, lastKnownPosition); 
  
  if (wasRunning) { 
      motorStop(); 
      aktuellePosition = gesamtSchritte + 50; 
      currentMotorState = STATE_HOMING; 
      setStatusMessage("Fehler! Not-Homing!", 2000); 
  }
  else if (lastKnownPosition >= 0 && lastKnownPosition <= gesamtSchritte) { 
      aktuellePosition = lastKnownPosition;
      currentMotorState = STATE_IDLE; 
  } else {
      aktuellePosition = gesamtSchritte + 50; 
      currentMotorState = STATE_HOMING; 
      setStatusMessage("Homing startet...", 2000); 
  }
  
  currentPage = PAGE_MENU; 
  updateDisplay(); 
}

void loop() {
  unsigned long jetzt = millis();
  char taste = keypad.getKey();
  if (jetzt - lastCursorToggle >= cursorInterval) {
    lastCursorToggle = jetzt;
    cursorVisible = !cursorVisible;
  }
  if (taste == '*') {
    if (motorAktiv) {
      motorStop();
      stoppenErzwungen = true;
      currentMotorState = STATE_IDLE;
      saveCurrentPosition(); 
    } else {
      if (currentPage == PAGE_INIT_CONFIRM) {
          jumpToPage(PAGE_INIT); 
      } else {
          jumpToPage(PAGE_MENU); 
      }
    }
  }

  if (taste == '#') {
     if (currentMotorState == STATE_HOMING || currentMotorState == STATE_TARGET_MOVE || currentMotorState == STATE_MANUAL_MOVE || currentMotorState == STATE_BACKLASH_CORRECTION) {
        
        // Neu: Prüfen, ob der abgebrochene Zustand Homing war
        bool wasHoming = (currentMotorState == STATE_HOMING); 
        
        motorStop();
        stoppenErzwungen = true;
        
        if (wasHoming) {
           // FIX V24.3: P=0 wird erzwungen und homingInitialisiert zurückgesetzt.
           aktuellePosition = 0; 
           homingInitialisiert = false; // <<< Wichtig: Homing-Flag zurücksetzen
           setStatusMessage("STOP (P=0 gesetzt)", 2000);
        } else {
           // Optionale Meldung für andere Abbrüche
           setStatusMessage("Bewegung gestoppt", 1000);
        }
        
        currentMotorState = STATE_IDLE;
        saveCurrentPosition(); 
        
    } else {
      processKeypad(taste);
    }
  } else {
    // Dieser Teil bleibt unverändert, um andere Tasten zu verarbeiten
    processKeypad(taste);
  }
  if (statusMeldungEnde > 0 && jetzt >= statusMeldungEnde) {
      statusMeldungEnde = 0;
      statusMeldung = ""; 
      if (currentPage == PAGE_INIT_CONFIRM) {
          jumpToPage(PAGE_MENU); 
      }
  }

  switch (currentMotorState) {
    case STATE_IDLE:
      if (stepsToDrive > 0) {
          currentMotorState = STATE_MANUAL_MOVE;
          manualDriving = true;
          motorAktiv = true;
          bool running = true;
          EEPROM.put(ADRESSE_MOTOR_RUNNING_FLAG, running); 
          EEPROM.commit();
      }
      break;
    case STATE_MANUAL_MOVE:
      if (!motorAktiv && manualDriving) {
          
          if (backlashSequenceRunning) {
              aktuellePosition = finalTargetPosition; 
              backlashSequenceRunning = false;
          }
          
          motorStop(); 
          currentMotorState = STATE_IDLE;
          manualDriving = false;
          saveCurrentPosition(); 
      } else if (!manualDriving) {
          motorStop();
          currentMotorState = STATE_IDLE;
      }
      break;
    case STATE_TARGET_MOVE:
        if (!motorAktiv && !zielErreicht) {
            
            float zielFloat = eingabePuffer.toFloat();
            eingabePuffer = ""; 
            
            finalTargetPosition = (long)round(getPositionFromFrequency(zielFloat)); 
            
            if (aktuellePosition == finalTargetPosition) {
                zielErreicht = true;
                break; 
            }
            
            bool zielRichtungRechts = (finalTargetPosition < aktuellePosition); 
            
            if (!zielRichtungRechts) {
                isBacklashCorrectionNeeded = false;
                backlashSequenceRunning = false; 
                currentMotorState = STATE_MANUAL_MOVE; 
                startBacklashMove(finalTargetPosition, abs(finalTargetPosition - aktuellePosition));
            } else {
                isBacklashCorrectionNeeded = true;
                long tempTarget = finalTargetPosition - BACKLASH_STEPS;
                
                currentMotorState = STATE_BACKLASH_CORRECTION; 
                startBacklashMove(tempTarget, abs(tempTarget - aktuellePosition));
            }
        }
        if (currentMotorState == STATE_MANUAL_MOVE && !isBacklashCorrectionNeeded) {
            bool zielUeberfahren = (manualRichtungRechts && aktuellePosition <= zielPosition) || 
                                   (!manualRichtungRechts && aktuellePosition >= zielPosition);
            if (zielUeberfahren) {
                motorStop();
                aktuellePosition = finalTargetPosition;
                zielErreicht = true;
                currentMotorState = STATE_IDLE;
                saveCurrentPosition(); 
            }
        } else if (zielErreicht) {
            zielErreicht = false;
            currentMotorState = STATE_IDLE;
        }
        break;
        
    case STATE_BACKLASH_CORRECTION:
        if (!motorAktiv) { 
            
            if (isBacklashCorrectionNeeded) {
                
                isBacklashCorrectionNeeded = false; 
                
                if (finalTargetPosition <= BACKLASH_STEPS) {
                    aktuellePosition = 0; 
                    
                } else {
                    currentMotorState = STATE_MANUAL_MOVE;
                    long stepsBack = abs(finalTargetPosition - aktuellePosition); 
                    startBacklashMove(finalTargetPosition, stepsBack);
                    backlashSequenceRunning = true; 
                }
            } 
            
            if (currentMotorState == STATE_BACKLASH_CORRECTION) { 
                motorStop(); 
                currentMotorState = STATE_IDLE;
                backlashSequenceRunning = false;
                saveCurrentPosition(); 
            }
        }
        break;
        
    case STATE_SAVING:
      {
        float freqToSave = eingabePuffer.toFloat();
        saveSpeicherpunkt(aktuellePosition, freqToSave);
        eingabePuffer = ""; 
        currentMotorState = STATE_IDLE;
        saveCurrentPosition(); 
        break;
      }
      
    case STATE_HOMING: 
        if (!homingInitialisiert) {
            manualRichtungRechts = true; 
            manualGeschwindigkeit = 1; 
            motorAktiv = true;
            homingInitialisiert = true;
            lastRichtungRechts = true; 
            stepsToDrive = 0; 
            manualDriving = false; 
        }
        if (aktuellePosition <= (0L - BACKLASH_STEPS)) { 
              motorStop();
              aktuellePosition = 0; 
              saveCurrentPosition(); 
              
              homingInitialisiert = false;
              currentMotorState = STATE_IDLE;
              setStatusMessage("HOMING abgeschlossen (P=0)", 2000); 
        }
        break;
    default:
      break;
  }
  if (jetzt - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = jetzt;
    updateDisplay();
  }
  delay(1);
}

void processKeypad(char taste) {
    if (!taste || taste == '*' || taste == '#') {
        
        if (taste == '#') {
            if (currentPage == PAGE_QRG_TARGET && eingabePuffer.length() > 0 && currentMotorState == STATE_IDLE) {
                currentMotorState = STATE_TARGET_MOVE; 
                return; 
            } else if (currentPage == PAGE_QRG_SAVE && eingabePuffer.length() > 0 && currentMotorState == STATE_IDLE) {
                currentMotorState = STATE_SAVING;
                return;
            } else if (currentPage == PAGE_INIT) { 
                if (currentMotorState == STATE_IDLE) {
                    jumpToPage(PAGE_INIT_CONFIRM);
                    return;
                }
            } else if (currentPage == PAGE_INIT_CONFIRM) {
                if (eingabePuffer == "1234") {
                    resetKalibrierung(); 
                    setStatusMessage("RESET ERFOLGREICH!", 3000); 
                } else {
                    setStatusMessage("Falscher Code!", 2000); 
                    eingabePuffer = "";
                }
                return;
            }
        }
        return;
    }
    if (taste >= '0' && taste <= '9') {
        
        if (currentPage == PAGE_MENU) {
            if (taste == '1') {
                jumpToPage(PAGE_QRG_TARGET); 
            } else if (taste == '2') {
                jumpToPage(PAGE_MANUAL); 
            } else if (taste == '3') {
                jumpToPage(PAGE_QRG_SAVE); 
            } else if (taste == '4') { // Menüpunkt 5 wurde entfernt, 4 ist jetzt der letzte Punkt
                jumpToPage(PAGE_INIT); 
            }
            return;
        }
        
        switch (currentPage) {
            case PAGE_MANUAL:
                if (taste == '1' || taste == '4' || taste == '7' || taste == '3' || taste == '6' || taste == '9') {
                    
                    long schritte;
                    int geschw;
                    if (taste == '1' || taste == '3') {
                        schritte = 1; 
                        geschw = 1; 
                    } else if (taste == '4' || taste == '6') {
                        schritte = 10; 
                        geschw = 2; 
                    } else if (taste == '7' || taste == '9') {
                        schritte = 100; 
                        geschw = 3; 
                    } else {
                        return;
                    }
                    bool rechtsTaste = (taste == '3' || taste == '6' || taste == '9');
                    bool neueRichtungRechts = !rechtsTaste; 
                    if (currentMotorState == STATE_IDLE) {
                        finalTargetPosition = aktuellePosition + (rechtsTaste ? schritte : -schritte);
                        
                        if (!neueRichtungRechts) {
                            isBacklashCorrectionNeeded = false;
                            backlashSequenceRunning = false; 
                            currentMotorState = STATE_MANUAL_MOVE;
                            startBacklashMove(finalTargetPosition, schritte);
                        } else {
                            isBacklashCorrectionNeeded = true;
                            long tempTarget = finalTargetPosition - BACKLASH_STEPS;
                            
                            currentMotorState = STATE_BACKLASH_CORRECTION;
                            startBacklashMove(tempTarget, abs(tempTarget - aktuellePosition));
                        }
                    } else {
                    }
                } 
                else if (taste == '0') {
                    if (!motorAktiv) {
                        aktuellePosition = gesamtSchritte + 50; 
                        homingInitialisiert = false; 
                        currentMotorState = STATE_HOMING;
                    } else {
                    }
                }
                break;
            case PAGE_QRG_TARGET:
            case PAGE_QRG_SAVE:
                if (eingabePuffer.length() < 5) { 
                    eingabePuffer += taste;
                }
                break;
                
            case PAGE_INIT_CONFIRM:
                if (eingabePuffer.length() < 4) { 
                    eingabePuffer += taste;
                }
                break;
            case PAGE_INIT: // NEU: '0' für Export
                if (taste == '0' && currentMotorState == STATE_IDLE) { 
                    dumpKalibrierTabelle();
                    return;
                }
                break;
            case PAGE_MENU: 
                break;
        }
    }
}

void loadKalibrierTabelle() {
    kalibrierTabelle.clear();
    int anzahlPunkte = 0;
    EEPROM.get(ADRESSE_PUNKTE_ZAEHLER, anzahlPunkte);
    if (anzahlPunkte < 2) return; 
    Speicherpunkt tempPunkt;
    for (int i = 0; i < anzahlPunkte; ++i) {
        EEPROM.get(ADRESSE_ERSTER_PUNKT + i * sizeof(Speicherpunkt), tempPunkt);
        kalibrierTabelle.push_back(tempPunkt);
    }
    sortKalibrierTabelle();
}
void resetKalibrierung() {
    kalibrierTabelle.clear();
    saveSpeicherpunkt(0, 5150.0f);                   // Start bei 0 Schritten
    saveSpeicherpunkt(gesamtSchritte, 24641.0f);     // Ende bei 11640 Schritten
    EEPROM.commit();
}
void sortKalibrierTabelle() {
    std::sort(kalibrierTabelle.begin(), kalibrierTabelle.end(), [](const Speicherpunkt& a, const Speicherpunkt& b) {
        return a.position < b.position;
    });
}


void saveSpeicherpunkt(long pos, float freq) {
    Speicherpunkt neuerPunkt = {pos, freq};
    bool punktGefunden = false;

    // 1. Nach existierender Position suchen und aktualisieren
    for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
        if (kalibrierTabelle[i].position == pos) {
            kalibrierTabelle[i] = neuerPunkt;
            punktGefunden = true;
            goto save_and_exit; 
        }
    }

    // 2. Nach existierender FREQUENZ suchen und aktualisieren/ersetzen
    // (Verwendung einer Toleranz für Float-Vergleiche: 0.001 kHz = 1 Hz)
    for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
        if (abs(kalibrierTabelle[i].frequenz - freq) < 0.001f) {
            // Frequenz gefunden: Wir aktualisieren diesen Punkt mit der neuen Position
            kalibrierTabelle[i] = neuerPunkt; 
            punktGefunden = true;
            goto save_and_exit; 
        }
    }
    
    // 3. Wenn weder Position noch Frequenz gefunden wurden: Neuen Punkt hinzufügen
    if (!punktGefunden) {
        if (kalibrierTabelle.size() < MAX_SPEICHERPUNKTE) {
            kalibrierTabelle.push_back(neuerPunkt);
        } else {
            setStatusMessage("Fehler: Max. Pkt erreicht!", 2000);
            return;
        }
    }

save_and_exit:
    // Sortieren, speichern und Status setzen
    sortKalibrierTabelle();
    EEPROM.put(ADRESSE_PUNKTE_ZAEHLER, (int)kalibrierTabelle.size());
    for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
        EEPROM.put(ADRESSE_ERSTER_PUNKT + i * sizeof(Speicherpunkt), kalibrierTabelle[i]);
    }
    EEPROM.commit();
    setStatusMessage("Table aktualisiert", 2000);
}

float getPositionFromFrequency(float frequenz) {
    if (kalibrierTabelle.size() < 2) return 0;
    if (frequenz <= kalibrierTabelle.front().frequenz) return (float)kalibrierTabelle.front().position;
    if (frequenz >= kalibrierTabelle.back().frequenz) return (float)kalibrierTabelle.back().position;
    for (size_t i = 0; i < kalibrierTabelle.size() - 1; ++i) {
        const Speicherpunkt& p1 = kalibrierTabelle[i];
        const Speicherpunkt& p2 = kalibrierTabelle[i+1];
        if (frequenz >= p1.frequenz && frequenz <= p2.frequenz) {
            float pos = p1.position + (p2.position - p1.position) * (frequenz - p1.frequenz) / (p2.frequenz - p1.frequenz);
            return pos;
        }
    }
    return (float)aktuellePosition;
}
float getFrequencyFromPosition(long position) {
    if (kalibrierTabelle.size() < 2) return 0.0f;
    if (position <= kalibrierTabelle.front().position) return kalibrierTabelle.front().frequenz;
    if (position >= kalibrierTabelle.back().position) return kalibrierTabelle.back().frequenz;
    for (size_t i = 0; i < kalibrierTabelle.size() - 1; ++i) {
        const Speicherpunkt& p1 = kalibrierTabelle[i];
        const Speicherpunkt& p2 = kalibrierTabelle[i+1];
        if (position >= p1.position && position <= p2.position) {
            float freq = p1.frequenz + (p2.frequenz - p1.frequenz) * (position - p1.position) / (p2.position - p1.position);
            return freq;
        }
    }
    return 0.0f; 
}

void dumpKalibrierTabelle() {
    Serial.println("\n--- START DATEN-EXPORT Kalibrierungstabelle ---");
    Serial.println("Index; Position (Schritte); Frequenz (kHz)");
    
    // Sicherstellen, dass die interne Tabelle mit dem EEPROM synchron ist
    loadKalibrierTabelle(); 
    
    if (kalibrierTabelle.size() < 2) {
        Serial.println("FEHLER: Kalibrierungstabelle ist leer oder enthaelt zu wenige Punkte.");
        setStatusMessage("Export: Tabelle leer!", 2000);
    } else {
        for (size_t i = 0; i < kalibrierTabelle.size(); ++i) {
            Speicherpunkt p = kalibrierTabelle[i];
            Serial.print(i);
            Serial.print("; ");
            Serial.print(p.position);
            Serial.print("; ");
            Serial.println(p.frequenz, 4); // Frequenz mit 4 Dezimalstellen ausgeben
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
        case PAGE_MENU: display.print("HAUPTMENUE"); break; 
        case PAGE_MANUAL: display.print("MANUELL"); break; 
        case PAGE_QRG_TARGET: display.print("ZIEL"); break;
        case PAGE_QRG_SAVE: display.print("SPEICHERN"); break;
        case PAGE_INIT: display.print("INIT"); break;
        case PAGE_INIT_CONFIRM: display.print("BESTAETIGUNG"); break; 
        default: display.print("MENU"); break;
    }
    
    display.setCursor(100, 0);
    String stateStr;
    switch (currentMotorState) {
        case STATE_IDLE: stateStr = "IDLE"; break;
        case STATE_MANUAL_MOVE: stateStr = "RUN"; break;
        case STATE_TARGET_MOVE: stateStr = "TARGET"; break;
        case STATE_SAVING: stateStr = "SAVE"; break;
        case STATE_HOMING: stateStr = "HOME"; break;
        case STATE_BACKLASH_CORRECTION: stateStr = "CORR"; break; 
        default: stateStr = "??"; break;
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
            
            display.setCursor(0, 13); display.print("1 > Atomatik");
            display.setCursor(0, 23); display.print("2 > Manuell");
            display.setCursor(0, 33); display.print("3 > QRG speichern");
            display.setCursor(0, 43); display.print("4 > Reset/Export");
            break;
        
        case PAGE_MANUAL:
            {
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
            }
            break;
            
        case PAGE_QRG_TARGET:
            {
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
                
                if (cursorVisible && currentMotorState == STATE_IDLE && eingabePuffer.length() < 5) {
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
            }
            break;
            
        case PAGE_QRG_SAVE:
            {
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
                
                if (cursorVisible && currentMotorState == STATE_IDLE && eingabePuffer.length() < 5) {
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
            }
            break;
            
        case PAGE_INIT:
            display.setFont(); 
            display.setTextSize(1);
            
            // Reset-Bereich (oben)
            display.setCursor(0, 13);
            display.print("KALIBRIERUNG RESET");
            display.setCursor(0, 23); 
            display.print("Akt. Pkt.: ");
            display.print(kalibrierTabelle.size());
            
            display.drawFastHLine(0, 33, SCREEN_WIDTH, SSD1306_WHITE); 
            
            // Export-Bereich (Mitte)
            display.setCursor(0, 35); 
            display.print("0 > Tabelle exportieren"); // NEUE ANZEIGE
            
            // Reset Bestätigung (unten)
            display.setCursor(0, 45); 
            display.print("# > Reset bestaetigen"); 
            break;
            
        case PAGE_INIT_CONFIRM:
            { 
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
            }
            break;
    }
    
    display.setFont(); 
    display.setTextSize(1);
    
    if (statusMeldungEnde > millis() || currentMotorState == STATE_HOMING || stoppenErzwungen) {
        display.fillRect(0, 54, SCREEN_WIDTH, 10, SSD1306_BLACK); 
    }
    if (statusMeldungEnde > millis() && statusMeldung.length() > 0) {
        display.setCursor(0, 55);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(statusMeldung);
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        
    } else if (currentMotorState == STATE_HOMING) {
        display.setCursor(0, 55);
        display.print("HOMING... (# Abbr)"); 
        
    } else if (stoppenErzwungen) {
        display.setCursor(0, 55);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print("STOPP erz. (* Menu)"); 
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        
    } else {
        
        bool drawBar = (currentPage != PAGE_MENU && currentPage != PAGE_INIT && currentPage != PAGE_INIT_CONFIRM);
        
        if (drawBar) {
            drawPositionBar();
        }
        
        if (currentPage == PAGE_MENU) {
            display.setCursor(0, 55);
            display.print("Waehle Seite 1-4"); 
        } else if (currentPage == PAGE_INIT || currentPage == PAGE_INIT_CONFIRM) {
            display.setCursor(0, 55);
            display.print("Zurueck: * Taste");
        } else if (drawBar) {
            display.setCursor(120, 55); 
            display.print("*");
        }
    }
    display.display();
}

