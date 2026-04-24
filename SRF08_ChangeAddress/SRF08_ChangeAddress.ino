/**
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  SRF08_ChangeAddress.ino  —  I²C Adresse eines SRF08 ändern        ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  Ändert die I²C Adresse eines einzelnen SRF08 Ultraschall-Sensors  ║
 * ║  von OLD_ADDRESS auf NEW_ADDRESS über Wire1.                        ║
 * ║                                                                      ║
 * ║  WICHTIG: Nur EINEN Sensor am Bus anschliessen!                     ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  Konfiguration (unten anpassen):                                    ║
 * ║    OLD_ADDRESS  —  aktuelle 8-bit Adresse des Sensors (z.B. 0xE0)  ║
 * ║    NEW_ADDRESS  —  gewünschte neue 8-bit Adresse      (z.B. 0xE2)  ║
 * ║                                                                      ║
 * ║  Gültige Adressen: 0xE0, 0xE2, 0xE4, ... 0xFE (gerade, ≥ 0xE0)   ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  Verdrahtung (Teensy 4.x, Wire1):                                   ║
 * ║    SRF08 SDA  →  Pin 17 (I2C1 SDA)  + 1k8 Pull-up nach 5V         ║
 * ║    SRF08 SCL  →  Pin 16 (I2C1 SCL)  + 1k8 Pull-up nach 5V         ║
 * ║    SRF08 VCC  →  5V                                                 ║
 * ║    SRF08 GND  →  GND                                                ║
 * ║    SRF08 DNC  →  nicht verbinden!                                   ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║  Ablauf:                                                             ║
 * ║    1. Alte und neue Adresse unten eintragen                         ║
 * ║    2. NUR den umzuprogrammierenden Sensor anschliessen              ║
 * ║    3. Sketch hochladen                                               ║
 * ║    4. Seriellen Monitor öffnen (115200 Baud) und Ergebnis prüfen    ║
 * ║    5. Sensor mit neuer Adresse beschriften                          ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 */

#include <Wire.h>
#include "SRF08.h"

// ── Adress-Konfiguration ──────────────────────────────────────────────────────
//
//   8-bit Adressen (SRF08-Aufdruckadresse) angeben.
//   Gültige Werte: 0xE0, 0xE2, 0xE4, 0xE6, ... 0xFE
//   Werkseinstellung: 0xE0
//
#define OLD_ADDRESS  0xE0   ///< Aktuelle Adresse des Sensors
#define NEW_ADDRESS  0xE2   ///< Gewünschte neue Adresse

// ─────────────────────────────────────────────────────────────────────────────

static bool addressChangeSuccessful = false;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println(F("=== SRF08 Adressaenderung ==="));
    Serial.print(F("Bus:          Wire1\n"));
    Serial.print(F("Alte Adresse: 0x"));
    Serial.println(OLD_ADDRESS, HEX);
    Serial.print(F("Neue Adresse: 0x"));
    Serial.println(NEW_ADDRESS, HEX);
    Serial.println();

    // ── Eingabevalidierung ────────────────────────────────────────────────
    if (OLD_ADDRESS == NEW_ADDRESS) {
        Serial.println(F("[FEHLER] Alte und neue Adresse sind identisch!"));
        return;
    }
    if (OLD_ADDRESS < 0xE0 || OLD_ADDRESS > 0xFE || (OLD_ADDRESS & 0x01)) {
        Serial.println(F("[FEHLER] OLD_ADDRESS ungueltig! Gueltig: 0xE0, 0xE2, ... 0xFE"));
        return;
    }
    if (NEW_ADDRESS < 0xE0 || NEW_ADDRESS > 0xFE || (NEW_ADDRESS & 0x01)) {
        Serial.println(F("[FEHLER] NEW_ADDRESS ungueltig! Gueltig: 0xE0, 0xE2, ... 0xFE"));
        return;
    }

    // ── Wire1 initialisieren ──────────────────────────────────────────────
    Wire1.begin();
    Wire1.setClock(100000); // 100 kHz — langsamere Clock für Adressänderung

    // ── Sensor unter alter Adresse suchen ────────────────────────────────
    Serial.print(F("Suche Sensor an 0x"));
    Serial.print(OLD_ADDRESS, HEX);
    Serial.print(F(" (7-bit: 0x"));
    Serial.print(OLD_ADDRESS >> 1, HEX);
    Serial.println(F(") ..."));

    SRF08Sensor sensor(SRF08_ADDR(OLD_ADDRESS));
    if (!sensor.begin(Wire1)) {
        Serial.println(F("[FEHLER] Sensor nicht gefunden!"));
        Serial.println(F("         Verdrahtung pruefen, nur EINEN Sensor anschliessen."));
        return;
    }
    Serial.println(F("[OK]    Sensor gefunden."));

    // ── Adresse ändern ────────────────────────────────────────────────────
    Serial.print(F("Aendere Adresse 0x"));
    Serial.print(OLD_ADDRESS, HEX);
    Serial.print(F(" -> 0x"));
    Serial.print(NEW_ADDRESS, HEX);
    Serial.println(F(" ..."));

    if (sensor.changeI2CAddress(NEW_ADDRESS)) {
        addressChangeSuccessful = true;
        Serial.println(F("[OK]    Adresse erfolgreich geaendert!"));
        Serial.print(F("        Sensor bitte mit '0x"));
        Serial.print(NEW_ADDRESS, HEX);
        Serial.println(F("' beschriften."));
    } else {
        Serial.println(F("[FEHLER] Adressaenderung fehlgeschlagen!"));
        Serial.println(F("         Sensor antwortet nicht unter neuer Adresse."));
    }
}

void loop() {
    // Nichts zu tun — Adressänderung ist ein einmaliger Vorgang in setup()
    static bool printed = false;
    if (!printed) {
        printed = true;
        Serial.println();
        if (addressChangeSuccessful) {
            Serial.println(F("Fertig. Sketch kann jetzt durch Normalbetrieb ersetzt werden."));
        } else {
            Serial.println(F("Vorgang abgeschlossen (mit Fehler). Bitte Konfiguration pruefen."));
        }
    }
    delay(5000);
}
