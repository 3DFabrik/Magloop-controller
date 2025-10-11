# Variablenkondensator-Steuerung V24.2

Steuersoftware für einen **variablen Kondensator (VC)** mit einem **ESP32 WROOM**, einem **4x3 Keypad** und einem **128x64 OLED-Display (SSD1306)**.  
Die Steuerung erfolgt über eine benutzerfreundliche Menüführung und speichert Kalibrierdaten im EEPROM.  

---

## 🔧 Funktionen

- **Frequenzbasierte Positionierung**  
  Berechnet die Motorposition aus einer gespeicherten Kalibrierungstabelle.
  
- **Homing-Funktion (Referenzfahrt)**  
  Bestimmt den mechanischen Nullpunkt des Kondensators.

- **Backlash-Kompensation**  
  Korrigiert mechanisches Spiel beim Richtungswechsel.

- **EEPROM-Datenspeicherung**  
  Speichert letzte Position und Kalibrierpunkte dauerhaft.

- **OLED-Display**  
  Zeigt Betriebszustände, Menü, Frequenz und Motorposition an.

- **Bedienung über 4x3-Tastenfeld**  
  Navigation, Eingabe von Frequenzen und manuelle Motorsteuerung.

---

## 🧠 Hardware-Anschluss (ESP32 WROOM)

| Komponente | Beschreibung | GPIO |
|-------------|---------------|------|
| **Keypad 4x3** | R1 | 25 |
| | R2 | 26 |
| | R3 | 33 |
| | R4 | 32 |
| | C1 | 13 |
| | C2 | 12 |
| | C3 | 14 |
| **Stepper Motor (Driver IN1–IN4)** | A | 17 |
| | B | 5 |
| | C | 18 |
| | D | 19 |
| **OLED Display (I²C)** | SDA | 21 |
| | SCL | 22 |

---

## 🧩 Software-Komponenten

Verwendete Bibliotheken:
```cpp
#include <Keypad.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold12pt7b.h>
```

---

## 📋 Menüstruktur

| Menüpunkt | Funktion |
|------------|-----------|
| **1 – Automatik (Ziel-Frequenz)** | Gibt Ziel-Frequenz ein, Motor fährt automatisch auf Position. |
| **2 – Manuell** | Bewegung in festen Schrittgrößen (1 / 10 / 100 Schritte). |
| **3 – QRG speichern** | Speichert aktuelle Position mit zugehöriger Frequenz. |
| **4 – Reset / Export** | Löscht Kalibrierung oder exportiert Tabelle über Serial Monitor. |

---

## ⚙️ Steuerung

| Taste | Funktion |
|--------|-----------|
| `*` | Abbrechen / Zurück ins Hauptmenü |
| `#` | Bestätigen / Starten |
| `0–9` | Zifferneingabe im jeweiligen Menü |
| `1,4,7` | Manuell nach links (langsam → schnell) |
| `3,6,9` | Manuell nach rechts (langsam → schnell) |
| `0` (im manuellen Modus) | Startet Homing-Fahrt |

---

## 💾 EEPROM-Adressen

| Adresse | Zweck |
|----------|--------|
| `0` | Letzte Position (long) |
| `+4` | Anzahl gespeicherter Punkte |
| `100` | Beginn der Kalibrierungstabelle |
| `1000` | Motor-Running-Flag |

---

## 🔍 Kalibrierung

Die Kalibrierung erfolgt durch Speichern von Frequenz-Position-Paaren.  
Bei einem Reset werden zwei Standardpunkte gesetzt:
- `0 Schritte → 5150 kHz`
- `11640 Schritte → 24641 kHz`

Weitere Punkte können über das Menü hinzugefügt werden.  
Zwischenwerte werden linear interpoliert.

---

## 🧾 Datenexport

Über den **Serial Monitor** (115200 Baud) kann die Kalibrierungstabelle exportiert werden.  
Im Menü „Reset/Export“ die Taste `0` drücken:

Beispielausgabe:
```
--- START DATEN-EXPORT Kalibrierungstabelle ---
Index; Position (Schritte); Frequenz (kHz)
0; 0; 5150.0000
1; 11640; 24641.0000
--- ENDE DATEN-EXPORT ---
```

---

## ⚠️ Hinweise

- Vor Nutzung **Homing durchführen**, um Nullpunkt zu definieren.  
- Kalibrierung regelmäßig sichern (Export).  
- EEPROM auf mindestens **4 KB** konfigurieren.

---

## 🧑‍💻 Autor / Versionierung

**Projekt:** Variablenkondensator-Steuerung  
**Version:** V24.2  
**Plattform:** ESP32 WROOM  
**Sprache:** C++ / Arduino-Framework  
**Lizenz:** Privatprojekt / Non-Commercial Use  
