# SRF08 Ultraschall-Bibliothek für Teensy 4.x

Eine vollständig nicht-blockierende Arduino-Bibliothek für den **Devantech SRF08 Ultraschall-Entfernungsmesser** mit mehrstufiger Filterpipeline, optimiert für den Einsatz im **RoboCup Junior Soccer**.

---

## Inhaltsverzeichnis

1. [Überblick](#überblick)
2. [Hardware-Voraussetzungen](#hardware-voraussetzungen)
3. [Installation](#installation)
4. [Verdrahtung](#verdrahtung)
5. [I²C-Adressen programmieren](#i²c-adressen-programmieren)
6. [Schnellstart](#schnellstart)
7. [API-Referenz](#api-referenz)
8. [Konfiguration](#konfiguration)
   - [Range-Register](#range-register)
   - [Gain-Register](#gain-register)
   - [Filter-Parameter](#filter-parameter)
9. [Filterpipeline — Funktionsweise](#filterpipeline--funktionsweise)
10. [State Machine](#state-machine)
11. [Mehrere Sensoren (SRF08Manager)](#mehrere-sensoren-srf08manager)
12. [Performance & Timing](#performance--timing)
13. [RoboCup-Empfehlungen](#robocup-empfehlungen)
14. [Fehlerbehebung](#fehlerbehebung)
15. [Technischer Hintergrund](#technischer-hintergrund)

---

## Überblick

Der Devantech SRF08 ist ein I²C-basierter Ultraschall-Entfernungsmesser mit einem Messbereich von 3 cm bis 6 m. Die Werkseinstellung (65 ms Messzeit, maximaler Gain) ist für Hochgeschwindigkeits-Robotik ungeeignet. Diese Bibliothek löst folgende Probleme:

| Problem | Lösung |
|---|---|
| Messrate nur ~14 Hz (65 ms Default) | Konfigurierbares Range-Register → bis zu ~70 Hz bei 2 m |
| Blockierendes `delay()` beim Warten | Vollständige State Machine mit `elapsedMillis` |
| Ghost-Echos bei reflektiven Wänden | Sprungfilter + Median-Filter |
| Messrauschen | EMA-Glättungsfilter |
| Crosstalk bei mehreren Sensoren | SRF08Manager: sequenzielles Feuern |
| I²C Adressverwaltung | `changeI2CAddress()` direkt in der API |

### Filterpipeline auf einen Blick

```
Rohwert → [Bereichscheck] → [Sprungfilter] → [Median N=5] → [EMA α=0.30] → Ausgabe
```

---

## Hardware-Voraussetzungen

- **Mikrocontroller:** Teensy 4.0 oder 4.1 (andere AVR/ARM-Boards möglich, `elapsedMillis` erforderlich)
- **Sensor:** Devantech SRF08 (ein oder mehrere)
- **Spannung:** 5V Versorgung für den SRF08 (Teensy-Logik: 3.3V — I²C-Pegel beachten!)
- **Pull-up-Widerstände:** 1,8 kΩ von SDA und SCL nach +5V (einmalig pro Bus, nicht pro Sensor)

> ⚠️ **Spannungsebenen:** Der Teensy 4.0 arbeitet mit 3.3V-Logik, der SRF08 mit 5V. Da der SRF08 I²C-Slave ist (Open-Drain), funktioniert dies in der Praxis mit 1,8-kΩ-Pull-ups nach 5V in den meisten Fällen. Für eine saubere Lösung empfiehlt sich ein I²C-Levelshifter (z.B. PCA9306).

---

## Installation

### Arduino IDE

1. Dieses Repository als `.zip` herunterladen
2. In der Arduino IDE: **Sketch → Bibliothek einbinden → .ZIP-Bibliothek hinzufügen**
3. Die Datei `SRF08.zip` auswählen
4. In eigenen Sketches einbinden: `#include <SRF08.h>`

### PlatformIO

Bibliothek in den `lib/`-Ordner des Projekts kopieren:

```
mein_roboter/
├── lib/
│   └── SRF08/
│       ├── SRF08.h
│       ├── SRF08.cpp
│       └── library.properties
├── src/
│   └── main.cpp
└── platformio.ini
```

`platformio.ini` für Teensy 4.0:

```ini
[env:teensy40]
platform  = teensy
board     = teensy40
framework = arduino
lib_deps  =
    Wire
```

---

## Verdrahtung

### Einzelner Sensor

```
Teensy 4.0          SRF08
──────────          ──────────────────────
Pin 19 (SCL) ───┬── SCL
Pin 18 (SDA) ───┼── SDA
3.3V / 5V    ───┼── (Pull-ups, siehe unten)
GND          ───┼── GND (0V)
5V (extern)  ───┘── VCC (+5V)
                     DNC → NICHT verbinden!

Pull-up-Widerstände (einmalig):
  SCL ──[1k8]── +5V
  SDA ──[1k8]── +5V
```

> **DNC (Do Not Connect):** Dieser Pin ist die MCLR-Leitung des internen PIC-Prozessors. Er hat einen internen Pull-up und darf keinesfalls mit Spannung oder GND verbunden werden.

### Mehrere Sensoren (Parallelschaltung am I²C-Bus)

```
Teensy 4.0          SRF08 #1 (0xE0)    SRF08 #2 (0xE2)
──────────          ───────────────    ───────────────
Pin 19 (SCL) ──┬── SCL ─────────────── SCL
Pin 18 (SDA) ──┼── SDA ─────────────── SDA
               │   VCC (5V)            VCC (5V)
               │   GND                 GND
               │
               ├── [1k8] ── +5V   (SCL Pull-up, einmalig!)
               └── [1k8] ── +5V   (SDA Pull-up, einmalig!)
```

Alle SRF08-Sensoren teilen sich denselben I²C-Bus. Jeder Sensor **muss eine einzigartige Adresse** haben (siehe nächsten Abschnitt).

---

## I²C-Adressen programmieren

Der SRF08 wird mit der Adresse `0xE0` ausgeliefert. Für mehrere Sensoren am selben Bus muss jeder Sensor eine eigene Adresse erhalten. Verfügbare Adressen (8-bit): `0xE0, 0xE2, 0xE4, 0xE6, 0xE8, 0xEA, 0xEC, 0xEE, 0xF0, 0xF2, 0xF4, 0xF6, 0xF8, 0xFA, 0xFC, 0xFE`

### Adresse ändern — Schritt für Schritt

> ⚠️ **Wichtig:** Während der Adressänderung darf **nur ein einziger Sensor** am I²C-Bus angeschlossen sein. Kollisionen erzeugen unvorhersehbares Verhalten.

**1. Nur den Sensor mit Adresse `0xE0` anschließen.**

**2. Folgenden Code in `setup()` einfügen, hochladen, einmal ausführen:**

```cpp
#include <Wire.h>
#include "SRF08.h"

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);  // Niedrige Taktrate für Adressänderung empfohlen

    SRF08Sensor sensor(SRF08_ADDR(0xE0));
    if (!sensor.begin(Wire)) {
        Serial.println("Sensor nicht gefunden!");
        return;
    }

    // Adresse auf 0xE2 ändern
    if (sensor.changeI2CAddress(0xE2)) {
        Serial.println("Erfolg! Sensor jetzt auf 0xE2.");
        Serial.println("Bitte '0xE2' auf den Sensor schreiben.");
    } else {
        Serial.println("Fehler bei der Adressänderung!");
    }
}

void loop() {}
```

**3. Code nach einmaligem Ausführen wieder entfernen.**

**4. Sensor beschriften** (z.B. mit Edding), zweiten Sensor anschließen.

### Adresse auslesen (falls vergessen)

Der SRF08 blinkt seine Adresse beim Einschalten über die rote LED aus:
- **1 langer Blitz** = Präambel
- **N kurze Blitze** = Adressnummer (0 = `0xE0`, 1 = `0xE2`, 2 = `0xE4`, …)

---

## Schnellstart

### Minimales Beispiel — Einzelner Sensor

```cpp
#include <Wire.h>
#include "SRF08.h"

// Sensor mit 2m Reichweite, mittlerem Gain
SRF08Sensor sensor(SRF08_ADDR(0xE0), SRF08_RANGE_2M, SRF08_GAIN_MID);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // 400 kHz Fast Mode

    if (!sensor.begin()) {
        Serial.println("Sensor nicht gefunden!");
        while (true) {}
    }
    sensor.startRanging();  // Erste Messung starten
}

void loop() {
    // update() muss so oft wie möglich aufgerufen werden — kein delay() in loop()!
    if (sensor.update()) {
        Serial.print("Distanz: ");
        Serial.print(sensor.getDistance());
        Serial.println(" cm");

        sensor.startRanging();  // Nächste Messung sofort starten
    }

    // Hier: andere Aufgaben (Motoren, IR-Sensor, etc.) — nicht-blockierend!
}
```

### Zwei Sensoren mit SRF08Manager

```cpp
#include <Wire.h>
#include "SRF08.h"

SRF08Sensor sensorVorne (SRF08_ADDR(0xE0), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor sensorHinten(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Manager sonar;

void setup() {
    Wire.begin();
    Wire.setClock(400000);

    sonar.addSensor(&sensorVorne);
    sonar.addSensor(&sensorHinten);
    sonar.begin();  // Initialisiert alle Sensoren, startet erste Messung
}

void loop() {
    sonar.update();  // State Machine antreiben

    if (sonar.isNewData(0)) {
        Serial.print("Vorne: ");
        Serial.print(sonar.getDistance(0));
        Serial.println(" cm");
    }

    if (sonar.isNewData(1)) {
        Serial.print("Hinten: ");
        Serial.print(sonar.getDistance(1));
        Serial.println(" cm");
    }
}
```

---

## API-Referenz

### `SRF08Sensor`

#### Konstruktor

```cpp
SRF08Sensor(uint8_t addr7bit,
            uint8_t rangeReg = SRF08_RANGE_2M,
            uint8_t gainReg  = SRF08_GAIN_MID);
```

`addr7bit` kann als 7-bit oder 8-bit SRF08-Adresse angegeben werden:

```cpp
SRF08Sensor s1(SRF08_ADDR(0xE0));  // 8-bit Hardware-Adresse via Makro
SRF08Sensor s2(0xE0);              // 8-bit direkt (wird intern auf 7-bit normalisiert)
SRF08Sensor s3(0x70);              // 7-bit direkt (z.B. aus I²C-Scanner)
```

#### Setup-Methoden

| Methode | Beschreibung |
|---|---|
| `bool begin(TwoWire& wire = Wire)` | Sensor initialisieren. Gibt `false` zurück wenn nicht erreichbar. |
| `void setRange(uint8_t rangeReg)` | Range-Register direkt setzen (0–255) |
| `void setRangeMeters(float meters)` | Reichweite in Metern setzen (berechnet Register automatisch) |
| `void setGain(uint8_t gainReg)` | Gain-Register setzen (0–31) |

#### Messbetrieb

| Methode | Beschreibung |
|---|---|
| `void startRanging()` | Ping starten (bei Verwendung von SRF08Manager nicht manuell aufrufen) |
| `bool update()` | State Machine antreiben. Gibt `true` zurück wenn neue Daten vorliegen (einmalig pro Messung) |

#### Datenzugriff

| Methode | Rückgabe | Beschreibung |
|---|---|---|
| `getDistance()` | `uint16_t` | Gefilterter Messwert in cm |
| `getRaw()` | `uint16_t` | Letzter ungefilteter Rohwert in cm |
| `getLightLevel()` | `uint8_t` | Lichtsensor (0 = dunkel, 255 = hell) |
| `getCycleTimeMs()` | `uint32_t` | Tatsächliche Messzeit der letzten Messung in ms |
| `getState()` | `SRF08State` | Aktueller Zustand: `UNINITIALIZED`, `IDLE`, `RANGING` |
| `getAddress()` | `uint8_t` | Aktuelle 7-bit I²C-Adresse |
| `hasValidData()` | `bool` | `true` wenn mindestens eine gültige Messung vorliegt |

#### Filter-Konfiguration

| Methode | Beschreibung |
|---|---|
| `void resetFilters()` | Alle Filterzustände zurücksetzen (nach Reinitialisierung sinnvoll) |
| `void setEMAAlpha(float alpha)` | EMA-Faktor setzen (0.0 = sehr träge, 1.0 = kein Filter) |
| `void setJumpThreshold(uint16_t cm)` | Maximale plausible Änderung pro Zyklus in cm |

#### Sonstiges

| Methode | Beschreibung |
|---|---|
| `bool changeI2CAddress(uint8_t newAddr8bit)` | I²C-Adresse dauerhaft ändern. Nur mit einem Sensor am Bus! |

---

### `SRF08Manager`

#### Methoden

| Methode | Beschreibung |
|---|---|
| `bool addSensor(SRF08Sensor* sensor)` | Sensor registrieren (max. 8). Gibt `false` zurück wenn voll. |
| `void begin(TwoWire& wire = Wire)` | Alle Sensoren initialisieren, erste Messung starten |
| `void update()` | **Muss in `loop()` aufgerufen werden.** Steuert alle Sensoren. |
| `uint16_t getDistance(uint8_t idx)` | Gefilterter Wert von Sensor `idx` in cm |
| `uint16_t getRaw(uint8_t idx)` | Rohwert von Sensor `idx` in cm |
| `uint8_t getLightLevel(uint8_t idx)` | Lichtsensor von Sensor `idx` |
| `bool isNewData(uint8_t idx)` | `true` wenn neue Daten seit letztem Aufruf. **Löscht Flag beim Lesen.** |
| `uint8_t getSensorCount()` | Anzahl der registrierten Sensoren |
| `SRF08Sensor* getSensor(uint8_t idx)` | Direkter Zeiger auf Sensor-Objekt |

---

### Konstanten und Makros

#### Adress-Makro

```cpp
#define SRF08_ADDR(hw8bit)   // Wandelt 8-bit Hardware-Adresse in 7-bit Wire-Adresse um
```

#### Range-Register Presets

| Konstante | Wert | Reichweite | Messzeit (ca.) |
|---|---|---|---|
| `SRF08_RANGE_1M` | 24 | ~1075 mm | ~8 ms |
| `SRF08_RANGE_2M` | 46 | ~2021 mm | ~14 ms |
| `SRF08_RANGE_3M` | 69 | ~3010 mm | ~20 ms |
| `SRF08_RANGE_4M` | 92 | ~3999 mm | ~25 ms |
| `SRF08_RANGE_6M` | 139 | ~6020 mm | ~37 ms |

#### Gain-Register Presets

| Konstante | Wert | Max. Gain | Empfohlener Einsatz |
|---|---|---|---|
| `SRF08_GAIN_MIN` | 0 | 94 | Sehr kurze Zyklen, Nahbereich |
| `SRF08_GAIN_LOW` | 10 | 133 | Stark reflektive Umgebungen |
| `SRF08_GAIN_MID` | 18 | 199 | **Empfehlung: RoboCup-Spielfeld** |
| `SRF08_GAIN_HIGH` | 25 | 352 | Normale Räume, gelegentliche Reflexionen |
| `SRF08_GAIN_MAX` | 31 | 1025 | Werkseinstellung, 65 ms Zykluszeit |

---

## Konfiguration

### Range-Register

Das Range-Register begrenzt die Zeit, die der Sensor auf Echos wartet. Die Formel laut Datenblatt:

```
range_mm = (Range-Register × 43 mm) + 43 mm
```

Beispiele für eigene Werte:

```cpp
// Manuelle Berechnung für 2,5 m:
// reg = (2500 / 43) - 1 = 57
sensor.setRange(57);

// Oder direkt in Metern (Bibliothek berechnet automatisch):
sensor.setRangeMeters(2.5f);
```

> **Regel:** Range-Register so klein wie möglich setzen, um kurze Messzeiten zu erreichen — aber nie kleiner als die tatsächlich benötigte Reichweite!

### Gain-Register

Das Gain-Register bestimmt die maximale Verstärkung des Analogteils. Der Gain steigt während einer Messung automatisch an (startet bei 94, wächst alle ~70 µs). Ein niedrigerer Maximalwert sorgt dafür, dass schwache, späte Echos (Ghost-Echos von weiter entfernten Objekten oder dem vorigen Ping) ignoriert werden.

```
Gain startet bei: 94 (immer)
Gain wächst bis:  Wert in Register 1
Maximum erreicht nach: ~390 mm Reichweite
```

**Wichtige Regel:** Je kürzer das Ranging-Window (kleines Range-Register), desto niedriger sollte der Gain sein, um Ghost-Echos des vorherigen Pings zu unterdrücken.

```cpp
// Beispiel: aggressiver 1m-Betrieb
sensor.setRange(SRF08_RANGE_1M);  // ~8 ms
sensor.setGain(SRF08_GAIN_LOW);   // Gain 133 — alte Echos werden ignoriert

// Beispiel: konservative 3m-Messung
sensor.setRange(SRF08_RANGE_3M);  // ~20 ms
sensor.setGain(SRF08_GAIN_MID);   // Gain 199
```

### Filter-Parameter

Alle Filter-Parameter können zur Laufzeit angepasst werden:

```cpp
// Compile-Zeit-Defaults (in SRF08.h):
static constexpr uint8_t  SRF08_MEDIAN_WINDOW = 5;     // Median-Fenstergröße
static constexpr float    SRF08_EMA_ALPHA_DEF = 0.30f; // EMA-Faktor
static constexpr uint16_t SRF08_JUMP_DEF      = 40;    // Sprungfilter [cm]

// Laufzeit-Anpassung:
sensor.setEMAAlpha(0.15f);       // Träger (mehr Glättung, mehr Latenz)
sensor.setEMAAlpha(0.50f);       // Reaktiver (weniger Glättung, schneller)
sensor.setJumpThreshold(25);     // Engere Spielfeldgrenzen → kleinerer Wert
sensor.setJumpThreshold(80);     // Schnell bewegende Objekte → größerer Wert
```

**EMA-Alpha Richtwerte:**

| α-Wert | Verhalten | Empfehlung |
|---|---|---|
| 0.10 – 0.20 | Sehr träge, starke Glättung | Statische Hindernisse |
| 0.25 – 0.35 | Ausgewogen | **RoboCup-Standardeinstellung** |
| 0.40 – 0.60 | Reaktiv, wenig Glättung | Schnell bewegende Objekte |
| 1.00 | Filter deaktiviert | Nur Median aktiv |

---

## Filterpipeline — Funktionsweise

Jede Rohmessung durchläuft vier Stufen in dieser Reihenfolge:

```
       Rohwert (cm)
           │
    ┌──────▼──────┐
    │ 1. Bereichs-│   Wert == 0 oder außerhalb [3, 600] cm?
    │    check    │──► ja: letzten validen Wert halten (kein Echo = kein Objekt)
    └──────┬──────┘
           │ gültiger Wert
    ┌──────▼──────┐
    │ 2. Sprung-  │   |raw - lastValid| > Schwellwert (default 40 cm)?
    │    filter   │──► ja → Median befragen:
    └──────┬──────┘        Median bestätigt Sprung? → EMA-Reset, Wert übernehmen
           │ plausibel     Median bestätigt nicht?  → Ausgabe einfrieren (Ghost-Echo!)
    ┌──────▼──────┐
    │ 3. Median   │   Sliding Window über N=5 Messungen
    │    N=5      │   Insertion-Sort → mittlerer Wert
    └──────┬──────┘   Unterdrückt: einzelne Ausreißer, Ghost-Echos
           │
    ┌──────▼──────┐
    │ 4. EMA      │   Exponential Moving Average: α=0.30
    │    α=0.30   │   ema_neu = 0.30 × median + 0.70 × ema_alt
    └──────┬──────┘   Unterdrückt: Messrauschen, kleine Schwankungen
           │
        Ausgabe (cm)
```

### Warum Median statt Mittelwert?

```
Rohwerte:   [182, 184, 183,  12, 185]   ← Ghost-Echo bei 12 cm
Mittelwert: (182+184+183+12+185)/5 = 149 cm  ← FALSCH
Median:     [12, 182, 183, 184, 185] → 183 cm  ← KORREKT
```

Der Median ist unempfindlich gegenüber einzelnen Ausreißern (Ghost-Echos), da er den mittleren — nicht den durchschnittlichen — Wert zurückgibt.

### Sprungfilter — Zusammenspiel mit Median

Der Sprungfilter allein reicht nicht aus, da echte schnelle Bewegungen (z.B. ein Roboter der schnell auf den Sensor zukommt) abgelehnt würden. Deshalb arbeitet er mit dem Median zusammen:

```
Situation A: Einzelner Ghost-Echo
  Messung 1: 180 cm  │
  Messung 2: 180 cm  │ Sprung erkannt (180→20: Δ=160 > 40)
  Messung 3:  20 cm  ◄─ Median noch bei ~180 → bestätigt nicht → eingefroren ✓
  Messung 4: 180 cm  │

Situation B: Echter Annäherung
  Messung 1: 180 cm  │
  Messung 2: 150 cm  │ Sprung erkannt (180→40: Δ=140 > 40)
  Messung 3: 100 cm  │ Median verschiebt sich → bestätigt Bewegung
  Messung 4:  40 cm  ◄─ EMA-Reset, neuer Wert wird übernommen ✓
```

---

## State Machine

Jeder `SRF08Sensor` durchläuft intern folgende Zustände:

```
                  ┌─────────────────┐
  begin() OK      │  UNINITIALIZED  │  begin() fehlgeschlagen
     ┌────────────┤                 ├────────────┐
     ▼            └─────────────────┘            │ (bleibt hier)
┌─────────┐                                      │
│  IDLE   │◄─────────────────────────────────────┘
└────┬────┘
     │ startRanging()
     │ (Ping gesendet, _timer = 0)
     ▼
┌──────────┐
│ RANGING  │  update() prüft:
│          │  Phase 1: _timer < rangingWindow → return false
│          │  Phase 2: _timer >= rangingWindow → _isBusy()?
└────┬─────┘           ja → return false (weiter warten)
     │                 nein → Daten lesen, Filter anwenden
     │ update() gibt true zurück
     ▼
  IDLE (wieder)  →  startRanging() für nächste Messung
```

Der **SRF08Manager** orchestriert mehrere Sensoren auf diesem Prinzip:

```
Sensor 0: startRanging() → RANGING ──► update()=true → IDLE
                                          │
                                          └──► Sensor 1: startRanging() → RANGING
                                                         update()=true → IDLE
                                                           │
                                                           └──► Sensor 0: startRanging() → ...
```

---

## Mehrere Sensoren (SRF08Manager)

### Warum sequenziell und nicht parallel?

Würden alle Sensoren gleichzeitig feuern, kann jeder Sensor die Echos der anderen empfangen:

```
Sensor A ─── ping ──► [Objekt] ──► echo ──► Sensor A  ✓
                   └──────────────────────► Sensor B  ✗ (Falschmessung!)
```

Der `SRF08Manager` feuert immer nur einen Sensor gleichzeitig. Erst wenn dessen Messung abgeschlossen ist, wird der nächste gestartet.

### Timing bei mehreren Sensoren

```
Zeit →    0ms    14ms    28ms    42ms
          │       │       │       │
Sensor 0: ████████│       ████████│
Sensor 1:         ████████│       ████████
```

Bei 2 Sensoren mit `SRF08_RANGE_2M` (~14 ms pro Messung):
- Jeder Sensor liefert alle **28 ms** neue Daten (~36 Hz)
- Kein Crosstalk, kein Rauschen durch Fremdeckos

Bei 4 Sensoren: 4 × 14 ms = 56 ms pro Vollzyklus (~18 Hz pro Sensor).

### Broadcast-Modus (ohne Manager, für Spezialanwendungen)

Das Datenblatt beschreibt eine Broadcast-Adresse `0x00`, die alle Sensoren gleichzeitig startet. Die Ergebnisse müssen dann individuell von jeder Adresse gelesen werden. **Nicht empfohlen** außer in sehr kontrollierten Aufbauten, da Crosstalk möglich ist:

```cpp
// NICHT empfohlen — nur für Spezialfälle:
Wire.beginTransmission(0x00);   // Broadcast an alle Sensoren
Wire.write(0x00);               // Command Register
Wire.write(0x51);               // Ranging in cm
Wire.endTransmission();
delay(65);                      // Volle 65ms warten (!)
// Danach individuell lesen...
```

---

## Performance & Timing

### Messzeiten nach Range-Register

| Konfiguration | Range-Reg | Messzeit | Max. Rate (1 Sensor) |
|---|---|---|---|
| `SRF08_RANGE_1M` | 24 | ~8 ms | ~120 Hz |
| `SRF08_RANGE_2M` | 46 | ~14 ms | ~70 Hz |
| `SRF08_RANGE_3M` | 69 | ~20 ms | ~50 Hz |
| `SRF08_RANGE_6M` (Default) | 139 | ~37 ms | ~27 Hz |
| Werkseinstellung | 255 | 65 ms | ~15 Hz |

### Filterlatenz

Der Median-Filter mit N=5 Samples führt zu einer Latenz von bis zu 4 Messung (wenn der Buffer noch nicht voll ist, ist die Latenz geringer). Bei `SRF08_RANGE_2M` und N=5:

```
Maximale Filterlatenz: 4 × 14 ms = ~56 ms
```

Das ist für Hindernisvermeidung bei typischen RoboCup-Geschwindigkeiten (~1 m/s) völlig ausreichend — der Roboter bewegt sich in 56 ms maximal **5,6 cm**.

### I²C-Overhead bei 400 kHz

Eine einzelne I²C-Transaktion (Schreiben eines Bytes) dauert bei 400 kHz ca. 40–80 µs. Das Polling in Phase 2 des `update()` erzeugt typischerweise 1–3 Polling-Aufrufe pro Messung, also < 250 µs zusätzlicher Overhead — vernachlässigbar.

---

## RoboCup-Empfehlungen

### Spielfeldbedingungen

Das RoboCup Junior Soccer Spielfeld (ca. 122 × 183 cm) hat folgende Eigenschaften, die die Ultraschall-Messung beeinflussen:

- **Harte Kunststoffwände:** Starke Reflexionen, potenzielle Ghost-Echos
- **Glatte Oberflächen:** Gerichtete Reflexion, wenig Dämpfung
- **Andere Roboter:** Bewegte Objekte, gegenseitige Interferenz bei mehreren Robotern
- **Orange Ball:** Kleines, unregelmäßiges Objekt — schlechte Reflektion für Ultraschall (IR/Kamera für Ballerkennung bevorzugen!)

### Empfohlene Konfiguration

```cpp
// Für typisches RoboCup-Spielfeld (max. 2m relevante Distanz):
SRF08Sensor sensor(SRF08_ADDR(0xE0),
                   SRF08_RANGE_2M,    // ~2m Reichweite, ~14ms Zykluszeit
                   SRF08_GAIN_MID);   // Gain 199 — guter Kompromiss

// Filter-Tuning für Spielfeldbedingungen:
sensor.setEMAAlpha(0.25f);       // Etwas träger wegen reflektiver Wände
sensor.setJumpThreshold(50);     // ~50cm Sprungtoleranz (Roboter schnell)
```

### Sensor-Positionierung

Für einen typischen RoboCup-Roboter empfiehlt sich:

```
        [Vorne]
    SRF08 (0xE0)        Hauptrichtung Hindernisvermeidung
         │
  ───────┼───────       Roboter-Footprint
         │
    SRF08 (0xE2)        Rückwärtsbewegung / Torhüter-Positionierung
        [Hinten]
```

Seitliche Sensoren sind für 2vs2 optional, können aber bei 4 Sensoren mit ~14ms/Messung noch ~18Hz pro Sensor erreichen.

### Beispiel: Hindernisvermeidung im Spielcode

```cpp
// In loop() — vollständig nicht-blockierend:
void handleSensors() {
    sonar.update();  // State Machine antreiben

    if (sonar.isNewData(0)) {  // Vorne
        uint16_t d = sonar.getDistance(0);
        if (d > 0 && d < 25) {
            // Hindernis in 25cm → Ausweichmanöver einleiten
            initiateEvasion();
        }
    }
}
```

---

## Fehlerbehebung

### Sensor wird nicht gefunden (`begin()` gibt `false` zurück)

1. **Spannungsversorgung prüfen:** SRF08 benötigt 5V, nicht 3.3V
2. **Pull-up-Widerstände prüfen:** 1,8 kΩ an SDA und SCL nach +5V (nicht nach 3.3V!)
3. **DNC-Pin prüfen:** Muss offen bleiben
4. **Adresse prüfen:** Ist der richtige `SRF08_ADDR(...)` angegeben?
5. **I²C-Scanner ausführen:** Standard-I²C-Scanner-Sketch zur Diagnose
6. **Richtigen Bus prüfen:** Wenn die Sensoren an `Wire1` hängen, müssen Scanner **und** Bibliothek mit `Wire1` laufen (`Wire1.begin();`, `sensor.begin(Wire1)` bzw. `sonar.begin(Wire1)`).

```cpp
// I²C-Scanner (in setup() ausführen):
for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
        Serial.print("Gefunden: 0x");
        Serial.println(addr << 1, HEX);  // Ausgabe als 8-bit Adresse
    }
}
```

### Messwerte springen stark

1. **Gain reduzieren:** `sensor.setGain(SRF08_GAIN_LOW)` oder niedriger
2. **Range-Register auf tatsächliche Distanz begrenzen:** Kein unnötig großes Fenster
3. **Reflexive Objekte identifizieren:** Metallene oder glatte Flächen im Messbereich?
4. **Sprungfilter verschärfen:** `sensor.setJumpThreshold(30)` (kleiner = strenger)

### Messwerte reagieren zu träge auf Änderungen

1. **EMA-Alpha erhöhen:** `sensor.setEMAAlpha(0.45f)`
2. **Sprungschwelle erhöhen:** `sensor.setJumpThreshold(60)` (erlaubt größere Sprünge)
3. **Median-Fenstergröße verkleinern:** `SRF08_MEDIAN_WINDOW` in `SRF08.h` auf 3 setzen und neu kompilieren

### Alle Messungen geben 0 zurück

- Der SRF08 liefert `0` wenn **kein Echo** empfangen wurde (Objekt außerhalb Reichweite, zu schräge Reflexion, oder der Wert 0 bedeutet tatsächlich kein Objekt).
- Die Bibliothek gibt bei `0` den letzten validen Wert zurück (`_lastValid`).
- Prüfen ob `hasValidData()` `true` ist — wenn nicht, hat noch keine gültige Messung stattgefunden.

### Zwei Sensoren stören sich gegenseitig

- Sicherstellen dass **SRF08Manager** verwendet wird (sequenzielles Feuern)
- `SRF08_GAIN_MID` oder niedriger verwenden
- Sensoren physikalisch so ausrichten dass ihre Keulen (±45°) sich nicht überschneiden

---

## Technischer Hintergrund

### Wie der SRF08 intern funktioniert

Der SRF08 sendet 8 Zyklen eines 40-kHz-Ultraschallpulses (der "Ping") und wartet dann auf ein zurückkommendes Echo. Die Empfangs-Verstärkung beginnt bei einem sehr niedrigen Wert (Gain 94) um das direkte Übersprechen zwischen Sender und Empfänger zu unterdrücken, und steigt dann alle ~70 µs an, bis das Maximum (eingestellt durch Register 1) erreicht ist. Auf diese Weise können nahe Objekte auch bei niedrigem Gain erkannt werden, während weiter entfernte Objekte von der steigenden Verstärkung profitieren.

Der interne Timer (durch das Range-Register konfigurierbar) legt fest, wann die Messung endet. Nach Ablauf des Timers sind die Ergebnisse in den Registern 2–35 verfügbar (bis zu 17 Echos).

### Warum 0xFF während der Messung?

Der SRF08 *antwortet nicht* auf I²C-Anfragen während er misst. Da die I²C-Datenleitungen (SDA, SCL) über Pull-up-Widerstände auf High gezogen sind, liest ein Byte-Leseversuch den Wert `0xFF` (alle Bits = 1 = High). Die Bibliothek nutzt dies als "Busy"-Indikator in Phase 2 der State Machine.

### Formel: Range-Register → Messzeit

```
range_mm  = (Register × 43 mm) + 43 mm
time_ms   = (2 × range_mm) / 343.0 m/s + ~2 ms Overhead

Beispiel Register=46:
  range_mm = (46 × 43) + 43 = 2021 mm
  time_ms  = (2 × 2021) / 343 + 2 = 11.78 + 2 ≈ 13.8 ms
```

Der Faktor 2 kommt daher, dass der Schall den Weg **hin und zurück** zurücklegt.

### EMA — Mathematik

```
EMA_neu = α × Messwert + (1-α) × EMA_alt

Zeitkonstante (Anzahl der Messungen bis 63% des Einflusses neuer Werte):
  τ ≈ 1/α

Bei α = 0.30:  τ ≈ 3.3 Messungen
Bei α = 0.20:  τ ≈ 5.0 Messungen
```

---

## Lizenz

Dieses Projekt steht unter der MIT-Lizenz. Freie Nutzung für private, educational und kommerzielle Projekte.

---

## Danksagung

Basierend auf dem offiziellen SRF08-Datenblatt von Devantech Ltd. (Gerry Coe, robot-electronics.co.uk). Entwickelt für den Einsatz im RoboCup Junior Soccer 2vs2.
