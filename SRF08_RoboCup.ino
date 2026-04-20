/**
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  SRF08_RoboCup.ino  —  Beispiel für Teensy 4.0 / RoboCup Soccer ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  Demonstriert:                                                   ║
 * ║   • Zwei Sensoren vorne/hinten am Roboter                       ║
 * ║   • Nicht-blockierender Betrieb mit SRF08Manager                ║
 * ║   • Angepasste Filter-Parameter für Spielfeld-Umgebung          ║
 * ║   • Serielles Debug-Logging                                      ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * Verdrahtung:
 *   SRF08  →  Teensy 4.0
 *   VCC    →  5V
 *   GND    →  GND
 *   SDA    →  Pin 18 (I2C0 SDA)  + 1k8 Pull-up nach 5V
 *   SCL    →  Pin 19 (I2C0 SCL)  + 1k8 Pull-up nach 5V
 *   DNC    →  nicht verbinden!
 *
 * Adressen:
 *   Sensor VORNE: 0xE0 (Werkseinstellung)
 *   Sensor HINTEN: 0xE2 (muss vorher umprogrammiert werden!)
 *
 * I²C Adressen umprogrammieren:
 *   → Nur einen Sensor anschliessen und changeI2CAddress() nutzen
 *   → Siehe Funktion reprogramAddress() unten
 */

#include <Wire.h>
#include "SRF08.h"

// ── Sensor-Definitionen ───────────────────────────────────────────────────────
//
//   Spielfeld RoboCup Junior Soccer (ca. 122×183cm):
//   Relevante Messdistanz: max. 2m, typisch < 1.5m
//   → SRF08_RANGE_2M:  ~14ms Zykluszeit pro Sensor
//   → Gain MID (199):  gut für harte Plastikwände, kein übermäßiges Gain
//
SRF08Sensor sensorVorne (SRF08_ADDR(0xE0), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor sensorHinten(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID);

SRF08Manager sonar;

// ── Timing ────────────────────────────────────────────────────────────────────
elapsedMillis debugTimer;
static constexpr uint32_t DEBUG_INTERVAL_MS = 100; // Serielle Ausgabe alle 100ms

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}  // Warte auf Serial (USB)

    Serial.println(F("=== SRF08 RoboCup Demo ==="));

    // I²C initialisieren — 400kHz (Fast Mode) für Teensy 4.0
    Wire.begin();
    Wire.setClock(400000);

    // Filter-Parameter anpassen (optional — Defaults sind bereits gut)
    // Bei sehr reflektiven Spielfeldwänden: niedrigerer EMA-Faktor (träger)
    sensorVorne.setEMAAlpha(0.25f);
    sensorHinten.setEMAAlpha(0.25f);

    // Sprungfilter: Max. 50cm Änderung pro Zyklus
    // (Roboter bewegt sich max. ~1.5 m/s × 0.03s ≈ 4.5cm/Zyklus → 50cm sehr konservativ)
    sensorVorne.setJumpThreshold(50);
    sensorHinten.setJumpThreshold(50);

    // Sensoren zum Manager hinzufügen
    sonar.addSensor(&sensorVorne);
    sonar.addSensor(&sensorHinten);

    // Initialisieren — begin() testet Erreichbarkeit
    sonar.begin(Wire);

    // Status prüfen
    Serial.print(F("Sensor VORNE  (0xE0): "));
    Serial.println(sensorVorne.getState()  != SRF08State::UNINITIALIZED ? F("OK") : F("FEHLER!"));
    Serial.print(F("Sensor HINTEN (0xE2): "));
    Serial.println(sensorHinten.getState() != SRF08State::UNINITIALIZED ? F("OK") : F("FEHLER!"));

    Serial.println(F("Starte Messung...\n"));
}

// ── Haupt-Loop ────────────────────────────────────────────────────────────────
void loop() {
    // ── WICHTIG: sonar.update() muss so oft wie möglich aufgerufen werden ──
    // Kein delay() in loop() verwenden — das blockiert die State Machine!
    sonar.update();

    // ── Serielle Debug-Ausgabe (zeitgesteuert, nicht blockierend) ────────────
    if (debugTimer >= DEBUG_INTERVAL_MS) {
        debugTimer = 0;
        printSensorData();
    }

    // ── Hier: Motorsteuerung, IR-Sensor, usw. ────────────────────────────────
    // Alle nicht-blockierend implementieren!
    //
    // Beispiel: Hindernisvermeidung
    handleObstacleAvoidance();
}

// ── Sensor-Daten ausgeben ─────────────────────────────────────────────────────
void printSensorData() {
    // Gefilterter Wert (empfohlen für Steuerung)
    uint16_t frontDist = sonar.getDistance(0);
    uint16_t backDist  = sonar.getDistance(1);

    // Rohwert (für Debugging / Filtertuning)
    uint16_t frontRaw  = sonar.getRaw(0);
    uint16_t backRaw   = sonar.getRaw(1);

    // Lichtsensor (Bonus: könnte für Balllinien-Erkennung nützlich sein)
    uint8_t frontLight = sonar.getLightLevel(0);

    Serial.print(F("VORNE:  "));
    Serial.print(frontDist); Serial.print(F("cm"));
    Serial.print(F(" (raw="));  Serial.print(frontRaw); Serial.print(F(")"));
    Serial.print(F("  Licht=")); Serial.print(frontLight);
    Serial.print(F("  Zyklus=")); Serial.print(sensorVorne.getCycleTimeMs()); Serial.print(F("ms"));

    Serial.print(F("   |   HINTEN: "));
    Serial.print(backDist); Serial.print(F("cm"));
    Serial.print(F(" (raw="));  Serial.print(backRaw); Serial.println(F(")"));
}

// ── Beispiel: Hindernisvermeidung ─────────────────────────────────────────────
void handleObstacleAvoidance() {
    static constexpr uint16_t OBSTACLE_THRESHOLD_CM = 25; // 25cm Stoppdistanz

    uint16_t frontDist = sonar.getDistance(0);

    // Nur reagieren wenn neue Daten vorliegen (verhindert Überreaktionen)
    if (sonar.isNewData(0)) {
        if (frontDist > 0 && frontDist < OBSTACLE_THRESHOLD_CM) {
            // Hindernis erkannt → Motorsteuerung aufrufen
            // stopMotors();  // eigene Funktion
            Serial.print(F("[WARNUNG] Hindernis vorne: "));
            Serial.print(frontDist);
            Serial.println(F("cm !"));
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  HILFSPROGRAMM: I²C Adresse eines Sensors ändern
// ═════════════════════════════════════════════════════════════════════════════
/**
 * Diese Funktion in setup() aufrufen wenn ein neuer Sensor auf 0xE2 gesetzt
 * werden muss. NUR mit EINEM Sensor am Bus!
 *
 * Ablauf:
 *   1. Zweiten Sensor NICHT anschliessen
 *   2. reprogramAddress() in setup() einkommentieren
 *   3. Hochladen → Sensor wird auf 0xE2 gesetzt
 *   4. Funktion wieder auskommentieren, zweiten Sensor anschliessen
 */
void reprogramAddress() {
    Wire.begin();
    Wire.setClock(100000); // Langsamere Clock für Adressänderung empfohlen

    // Temporärer Sensor-Objekt bei Werksadresse 0xE0
    SRF08Sensor tempSensor(SRF08_ADDR(0xE0));
    if (!tempSensor.begin(Wire)) {
        Serial.println(F("Sensor nicht gefunden!"));
        return;
    }

    Serial.println(F("Ändere Adresse 0xE0 → 0xE2 ..."));
    if (tempSensor.changeI2CAddress(0xE2)) {
        Serial.println(F("Adresse erfolgreich geändert!"));
        Serial.println(F("Sensor bitte mit '0xE2' beschriften."));
    } else {
        Serial.println(F("Fehler beim Ändern der Adresse!"));
    }
}
