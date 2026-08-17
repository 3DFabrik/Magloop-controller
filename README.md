# Magloop-Controller V25.00

Steuersoftware für einen **variablen Kondensator** (Magnetloop) mit **ESP32 WROOM**, **4x3-Keypad**, **128x64 OLED (SSD1306)** und **Endstop**.

Bedienung über das Tastenfeld und das OLED-Menü. Frequenz-/Schritt-Kalibrierung über WLAN-Webinterface. Frequenz-Tracking vom **Icom IC-705** über **Bluetooth Classic (SPP / CI-V)**.

Firmware: [`Steuersoftware_ESP32_V25_00/Steuersoftware_ESP32_V25_00.ino`](Steuersoftware_ESP32_V25_00/Steuersoftware_ESP32_V25_00.ino)

---

## Funktionen

- **Frequenzbasierte Positionierung** aus der Kalibrierungstabelle (lineare Interpolation)
- **Homing über Endstop** mit Backoff, langsamer Annäherung und Release (kein fester Absolutwert)
- **Backlash-Kompensation** beim Richtungswechsel
- **EEPROM**: letzte Position, Kalibrierpunkte, Motor-Running-Flag, Tracking und Bluetooth-Auto-Start
- **OLED + Keypad**: Menü, Frequenz, Position, IC-705-Status
- **Webinterface** nur für Setup/Kalibrierung (nicht für den Funkbetrieb)
- **IC-705-Tracking** über Bluetooth SPP, sobald Tracking eingeschaltet ist und das Funkgerät nicht sendet

WLAN und Bluetooth teilen sich das Funkmodul des ESP32. Es ist immer nur **eines von beiden** aktiv.

---

## Hardware (ESP32 WROOM)

| Komponente | Anschluss | GPIO |
|---|---|---|
| Keypad 4x3 | R1 / R2 / R3 / R4 | 25 / 26 / 33 / 32 |
| | C1 / C2 / C3 | 13 / 12 / 14 |
| Stepper (IN1–IN4) | A / B / C / D | 17 / 5 / 18 / 19 |
| OLED I²C | SDA / SCL | 21 / 22 |
| Endstop | Eingang, Pull-up | 4 |

---

## Kompilieren / Flashen

Board: `esp32:esp32:esp32`  
Partition: **`min_spiffs`** (sonst wird der Sketch zu groß)

```text
arduino-cli compile --fqbn "esp32:esp32:esp32:PartitionScheme=min_spiffs" -u -p COMx Steuersoftware_ESP32_V25_00/Steuersoftware_ESP32_V25_00.ino
```

Serial Monitor: **115200 Baud**.

---

## Alltag: Bluetooth / IC-705

Nach dem ersten erfolgreichen Bluetooth-Start merkt sich der Tuner das im EEPROM und kommt beim nächsten Einschalten direkt mit Bluetooth hoch (WLAN bleibt aus).

### Pairing am IC-705

1. Am Tuner Menü **5**, dann **1** (WLAN → Bluetooth)
2. Am IC-705: **MENU → SET → Bluetooth Set → Pairing/Connect → Device Search → Search Data Device**
3. Gerät **Magloop Tuner** wählen, PIN **0000**

Nicht gleichzeitig die Website und das IC-705 am Funkmodul nutzen.

### Tracking

Auf Seite **5**: Taste **0** schaltet Tracking ein/aus. Die Einstellung bleibt im EEPROM.  
Der Motor folgt der IC-705-QRG nur bei aktivem Tracking, verbundenem Funkgerät, RX (kein PTT) und stillstehendem Motor. Deadband standardmäßig **5 kHz**.

---

## Setup: WLAN / Webinterface

Für die Kalibrierung Bluetooth aus und WLAN an:

1. Menü **5**, Taste **1** (Bluetooth → WLAN)
2. WLAN **AntennaTuner**, Passwort **12345678**
3. Browser: **http://192.168.4.1**

Die Website hat **Home** und **Calibration** (Frequenz/Schritte). Bluetooth wird dort nicht mehr bedient.

`#` auf Seite 5 schaltet **nicht** mehr zwischen WLAN und Bluetooth. Dafür ist Taste **1**.

Nach dem Setup einfach neu starten oder erneut **1** drücken — Auto-Start bleibt gespeichert.

---

## Menü (OLED)

| Taste | Funktion |
|---|---|
| **1** | Automatik (Ziel-Frequenz) |
| **2** | Manuell |
| **3** | QRG speichern |
| **4** | Reset / Export |
| **5** | IC-705 / WLAN |

### Seite 5 (IC-705 / WLAN)

| Taste | Funktion |
|---|---|
| **0** | Tracking ein/aus (EEPROM) |
| **1** | Umschalten **WLAN ↔ Bluetooth** |
| **\*** | Zurück ins Hauptmenü |

Oben rechts erscheint kurz der Status (`BT an`, `WLAN an`, `Tracking AN`/`AUS`) anstelle von IDLE.

---

## Tasten allgemein

| Taste | Funktion |
|---|---|
| `*` | Abbrechen / Hauptmenü |
| `#` | Bestätigen / Motor stoppen |
| `0–9` | Zifferneingabe bzw. Menüwahl |
| `1 / 4 / 7` | Manuell eine Richtung (1 / 10 / 100 Schritte) |
| `3 / 6 / 9` | Manuell andere Richtung (1 / 10 / 100 Schritte) |
| `0` (manuell) | Homing |

---

## EEPROM

| Adresse | Zweck |
|---|---|
| `0` | Letzte Position |
| `4` | Anzahl Kalibrierpunkte |
| `100` | Kalibrierungstabelle |
| `1000` | Motor-Running-Flag (Not-Homing nach Stromausfall) |
| `1010` | BT-Settings: Tracking, Deadband, Auto-Start |

EEPROM-Größe: **4 KB**.

---

## Kalibrierung

Zwei Wege:

1. **Webinterface** (empfohlen): Homing, Motor per Buttons, Punkte speichern, Session committen oder verwerfen. Die bestehende Tabelle wird erst überschrieben, wenn gespeichert wird.
2. **Keypad Menü 3**: aktuelle Position mit Frequenz speichern.

Beim Reset (Menü 4, Code `1234`) werden zwei Standardpunkte gesetzt:

- `0 Schritte → 5150 kHz`
- `11640 Schritte → 24641 kHz`

Zwischenwerte werden linear interpoliert.

---

## Datenexport

Serial Monitor **115200 Baud**, Menü **Reset/Export**, Taste **0**:

```
--- START DATEN-EXPORT Kalibrierungstabelle ---
Index; Position (Schritte); Frequenz (kHz)
0; 0; 5150.0000
1; 11640; 24641.0000
--- ENDE DATEN-EXPORT ---
```

---

## Hinweise

- WLAN und Bluetooth nicht gleichzeitig — das ESP32-WROOM hat ein Funkmodul.
- Homing über den Endstop an GPIO 4.
- Kalibrierung per Export sichern.
- Nach Stromausfall während einer Fahrt startet ein Not-Homing.

---

## Version

**Projekt:** Magloop-Controller / Variablenkondensator-Steuerung  
**Version:** V25.00  
**Plattform:** ESP32 WROOM  
**Sprache:** C++ / Arduino  
**Lizenz:** Privatprojekt / Non-Commercial Use
