#pragma once

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  SRF08.h  —  Devantech SRF08 Ultrasonic Sensor Library für Teensy 4.x  ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  Features:                                                               ║
 * ║   • Vollständig non-blocking (State Machine + elapsedMillis)            ║
 * ║   • Mehrstufige Filterpipeline:                                         ║
 * ║       1. Bereichsvalidierung                                             ║
 * ║       2. Sprungfilter (Jump-Guard)                                       ║
 * ║       3. Median-Filter (Ghost-Echo Unterdrückung)                       ║
 * ║       4. EMA-Filter   (Glättung)                                        ║
 * ║   • SRF08Manager: Paarweises Feuern mehrerer Sensoren (2 gleichzeitig)   ║
 * ║   • Konfigurierbare Range & Gain Register (hohe Abfragerate < 20ms)     ║
 * ║   • I²C Address Change Support                                          ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Beispiel-Verwendung:
 *   SRF08Sensor front(SRF08_ADDR(0xE0), SRF08_RANGE_2M, SRF08_GAIN_MID);
 *   SRF08Manager manager;
 *   manager.addSensor(&front);
 *   manager.begin();          // in setup()
 *   manager.update();         // in loop()
 *   int cm = front.getDistance();
 *
 * Wichtig: Wire.begin() und Wire.setClock(400000) vor manager.begin() aufrufen!
 */

#include <Arduino.h>
#include <Wire.h>

// ── I²C Adress-Makro ─────────────────────────────────────────────────────────
// Wire.h erwartet 7-bit Adressen. SRF08 liefert 8-bit (0xE0..0xFE).
// Einfach das Makro auf die Aufdruckadresse anwenden:
//   SRF08_ADDR(0xE0) → 0x70
//   SRF08_ADDR(0xE2) → 0x71  usw.
#define SRF08_ADDR(hw8bit)   (static_cast<uint8_t>((hw8bit) >> 1))

// ── Register ─────────────────────────────────────────────────────────────────
static constexpr uint8_t SRF08_REG_CMD      = 0x00; ///< W: Befehl     | R: SW-Revision
static constexpr uint8_t SRF08_REG_GAIN     = 0x01; ///< W: Gain       | R: Lichtsensor
static constexpr uint8_t SRF08_REG_RANGE    = 0x02; ///< W: Range      | R: Echo1 High
static constexpr uint8_t SRF08_REG_ECHO_H   = 0x02; ///< Echo 1 High Byte (Read)
static constexpr uint8_t SRF08_REG_ECHO_L   = 0x03; ///< Echo 1 Low  Byte (Read)

// ── Befehle ───────────────────────────────────────────────────────────────────
static constexpr uint8_t SRF08_CMD_RANGE_CM = 0x51; ///< Messung starten, Ergebnis in cm
static constexpr uint8_t SRF08_CMD_RANGE_IN = 0x50; ///< Messung starten, Ergebnis in Zoll
static constexpr uint8_t SRF08_CMD_RANGE_US = 0x52; ///< Messung starten, Ergebnis in µs

// Indikator: Sensor misst noch (SDA hochgezogen → 0xFF)
static constexpr uint8_t SRF08_BUSY         = 0xFF;

// ── Range-Register Presets ────────────────────────────────────────────────────
// Berechnung: range_mm = (reg * 43) + 43
// Wartezeit (ca.): time_ms = (2 * range_mm / 343) + 2
static constexpr uint8_t SRF08_RANGE_1M  =  24; ///< ~1075 mm → ~8  ms Wartezeit
static constexpr uint8_t SRF08_RANGE_2M  =  46; ///< ~2021 mm → ~14 ms Wartezeit
static constexpr uint8_t SRF08_RANGE_3M  =  69; ///< ~3010 mm → ~20 ms Wartezeit
static constexpr uint8_t SRF08_RANGE_4M  =  92; ///< ~3999 mm → ~25 ms Wartezeit
static constexpr uint8_t SRF08_RANGE_6M  = 139; ///< ~6020 mm → ~37 ms Wartezeit

// ── Gain-Register Presets ─────────────────────────────────────────────────────
static constexpr uint8_t SRF08_GAIN_MIN  =  0;  ///< Gain  94  – sehr kurze Zyklen
static constexpr uint8_t SRF08_GAIN_LOW  = 10;  ///< Gain 133  – reflektive Umgebung
static constexpr uint8_t SRF08_GAIN_MID  = 18;  ///< Gain 199  – guter Kompromiss (default)
static constexpr uint8_t SRF08_GAIN_HIGH = 25;  ///< Gain 352  – normale Umgebung
static constexpr uint8_t SRF08_GAIN_MAX  = 31;  ///< Gain 1025 – Werkseinstellung

// ── Filter-Parameter (anpassbar zur Compile-Zeit) ─────────────────────────────
static constexpr uint8_t  SRF08_MEDIAN_WINDOW  = 5;     ///< Median-Fenstergröße (ungerade!)
static constexpr float    SRF08_EMA_ALPHA_DEF  = 0.30f; ///< EMA-Faktor (0=träge, 1=aus)
static constexpr uint16_t SRF08_JUMP_DEF       = 40;    ///< Max. plausible Änderung [cm]
static constexpr uint16_t SRF08_MIN_DIST_CM    = 3;     ///< Minimale Messdistanz
static constexpr uint16_t SRF08_MAX_DIST_CM    = 600;   ///< Maximale Messdistanz

// ── Timeout ───────────────────────────────────────────────────────────────────
static constexpr uint32_t SRF08_POLL_TIMEOUT_MS = 20;   ///< Max. Polling-Fenster nach Ranging-Window

// ═════════════════════════════════════════════════════════════════════════════
//  MedianFilter  —  Sliding-Window Median über N Samples
// ═════════════════════════════════════════════════════════════════════════════
struct MedianFilter {
    uint16_t buf[SRF08_MEDIAN_WINDOW] = {};
    uint8_t  idx  = 0;
    bool     full = false;

    void     reset();
    void     push(uint16_t value);
    uint16_t compute() const; ///< sortiert intern auf Stack-Kopie → kein Seiteneffekt
};

// ═════════════════════════════════════════════════════════════════════════════
//  SRF08State  —  Interner Sensor-Zustand
// ═════════════════════════════════════════════════════════════════════════════
enum class SRF08State : uint8_t {
    UNINITIALIZED, ///< begin() wurde nicht aufgerufen
    IDLE,          ///< Bereit für neue Messung
    RANGING,       ///< Ping gesendet, warte auf Echo
};

// ═════════════════════════════════════════════════════════════════════════════
//  SRF08Sensor  —  Einzelner Sensor
// ═════════════════════════════════════════════════════════════════════════════
class SRF08Sensor {
public:
    /**
     * @param addr7bit  7-bit I²C Adresse → SRF08_ADDR(0xE0)
     * @param rangeReg  Range-Register (z.B. SRF08_RANGE_2M)
     * @param gainReg   Gain-Register  (z.B. SRF08_GAIN_MID)
     */
    SRF08Sensor(uint8_t addr7bit,
                uint8_t rangeReg = SRF08_RANGE_2M,
                uint8_t gainReg  = SRF08_GAIN_MID);

    // ── Setup ──────────────────────────────────────────────────────────────
    /** Initialisiert Sensor, schreibt Range & Gain. Gibt false zurück wenn nicht erreichbar. */
    bool begin(TwoWire& wire = Wire);

    /** Range-Register direkt setzen (0–255). */
    void setRange(uint8_t rangeReg);

    /** Range in Metern angeben (Convenience-Wrapper). */
    void setRangeMeters(float meters);

    /** Gain-Register setzen (0–31). */
    void setGain(uint8_t gainReg);

    // ── Non-blocking Betrieb ───────────────────────────────────────────────
    /** Ping starten. Sollte nicht direkt aufgerufen werden wenn SRF08Manager genutzt wird. */
    void startRanging();

    /**
     * Muss in loop() aufgerufen werden.
     * @return true wenn neue gefilterte Messung verfügbar (einmalig).
     */
    bool update();

    // ── Datenzugriff ───────────────────────────────────────────────────────
    uint16_t   getDistance()    const { return _filtered;  } ///< Gefilterter Wert [cm]
    uint16_t   getRaw()         const { return _raw;       } ///< Letzter Rohwert   [cm]
    uint8_t    getLightLevel()  const { return _light;     } ///< Lichtsensor [0–255]
    uint32_t   getCycleTimeMs() const { return _cycleTime; } ///< Letzte Messzeit   [ms]
    SRF08State getState()       const { return _state;     }
    uint8_t    getAddress()     const { return _addr;      }
    bool       hasValidData()   const { return _hasData;   }

    // ── Filter-Konfiguration ───────────────────────────────────────────────
    void resetFilters();
    void setEMAAlpha(float alpha)         { _emaAlpha = constrain(alpha, 0.0f, 1.0f); }
    void setJumpThreshold(uint16_t cm)    { _jumpThreshold = cm; }

    // ── I²C Adresse ändern ─────────────────────────────────────────────────
    /**
     * Ändert die I²C Adresse des Sensors.
     * NUR aufrufen wenn GENAU EIN Sensor am Bus ist!
     * @param newAddr8bit  Neue 8-bit Adresse (0xE0, 0xE2, 0xE4 … 0xFE)
     * @return true bei Erfolg
     */
    bool changeI2CAddress(uint8_t newAddr8bit);

private:
    uint8_t     _addr;
    uint8_t     _rangeReg;
    uint8_t     _gainReg;
    TwoWire*    _wire;
    SRF08State  _state;

    elapsedMillis _timer;        ///< Läuft seit startRanging()
    uint32_t      _rangingWindow; ///< Berechnete Wartezeit [ms]
    uint32_t      _cycleTime;

    // ── Filterzustand ──────────────────────────────────────────────────────
    MedianFilter _median;
    float        _ema;
    bool         _emaInit;
    float        _emaAlpha;
    uint16_t     _jumpThreshold;
    uint16_t     _lastValid;

    // ── Ausgabe ────────────────────────────────────────────────────────────
    uint16_t _raw;
    uint16_t _filtered;
    uint8_t  _light;
    bool     _hasData;

    // ── Interne Helfer ─────────────────────────────────────────────────────
    void     _writeReg(uint8_t reg, uint8_t val);
    uint8_t  _readByte(uint8_t reg);
    uint16_t _readWord(uint8_t reg);
    uint32_t _calcRangingWindow() const;
    bool     _isBusy();                    ///< Pollt Register 0 auf 0xFF
    uint16_t _applyFilters(uint16_t raw);
};

// ═════════════════════════════════════════════════════════════════════════════
//  SRF08Manager  —  Mehrere Sensoren paarweise (2 gleichzeitig) betreiben
// ═════════════════════════════════════════════════════════════════════════════
/**
 * Verwaltet bis zu MAX_SENSORS SRF08-Sensoren.
 * Sensoren werden in Paaren gefeuert (2 gleichzeitig).
 * Bei 4 Sensoren: Paar 0+2 und Paar 1+3.
 *
 * Gesamtlatenz (2 Sensoren, 2m-Range):  1 × 14ms = ~14ms pro Volldurchlauf
 * Jeder Sensor liefert individuell Daten sobald seine Messung abgeschlossen ist.
 */
class SRF08Manager {
public:
    static constexpr uint8_t MAX_SENSORS = 8;

    SRF08Manager();

    /** Sensor hinzufügen. Gibt false zurück wenn MAX_SENSORS erreicht. */
    bool addSensor(SRF08Sensor* sensor);

    /** Alle Sensoren initialisieren und erste Messung starten. */
    void begin(TwoWire& wire = Wire);

    /**
     * Muss in loop() aufgerufen werden — so oft wie möglich.
     * Treibt die State Machine aller Sensoren an.
     */
    void update();

    // ── Datenzugriff ───────────────────────────────────────────────────────
    uint16_t getDistance(uint8_t idx)   const;  ///< Gefilterter Wert [cm]
    uint16_t getRaw(uint8_t idx)        const;  ///< Rohwert [cm]
    uint8_t  getLightLevel(uint8_t idx) const;  ///< Lichtsensor [0–255]

    /**
     * Gibt true zurück wenn seit dem letzten Aufruf neue Daten vorliegen.
     * Flag wird beim Lesen automatisch zurückgesetzt.
     */
    bool     isNewData(uint8_t idx);

    uint8_t  getSensorCount()           const { return _count; }
    SRF08Sensor* getSensor(uint8_t idx) const;

private:
    static constexpr uint8_t INVALID_INDEX = 0xFF;

    SRF08Sensor* _sensors[MAX_SENSORS];
    uint8_t      _count;
    uint8_t      _current;       ///< Aktueller Paar-Start / Paar-Index
    uint8_t      _activeA;       ///< Sensor-Index im aktiven Paar
    uint8_t      _activeB;       ///< Sensor-Index im aktiven Paar (oder INVALID_INDEX)
    bool         _newData[MAX_SENSORS];

    void _selectActivePair();
    void _startActivePair();
    bool _activePairFinished() const;
    void _advancePair();
};
