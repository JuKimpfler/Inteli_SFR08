/**
 * SRF08.cpp  —  Implementierung der SRF08 Ultraschall-Bibliothek
 * Siehe SRF08.h für Dokumentation.
 */

#include "SRF08.h"
#include <string.h>

static uint8_t normalizeSRF08Address(uint8_t addr7bitOr8bit) {
    return (addr7bitOr8bit > 0x7F)
        ? static_cast<uint8_t>(addr7bitOr8bit >> 1)
        : addr7bitOr8bit;
}

// ═════════════════════════════════════════════════════════════════════════════
//  MedianFilter
// ═════════════════════════════════════════════════════════════════════════════

void MedianFilter::reset() {
    memset(buf, 0, sizeof(buf));
    idx  = 0;
    full = false;
}

void MedianFilter::push(uint16_t value) {
    buf[idx] = value;
    idx = (idx + 1) % SRF08_MEDIAN_WINDOW;
    if (idx == 0) full = true;
}

uint16_t MedianFilter::compute() const {
    uint8_t n = full ? SRF08_MEDIAN_WINDOW : idx;
    if (n == 0) return 0;
    if (n == 1) return buf[0];

    // Stack-Kopie → Original-Buffer bleibt unverändert
    uint16_t tmp[SRF08_MEDIAN_WINDOW];
    memcpy(tmp, buf, n * sizeof(uint16_t));

    // Insertion-Sort: optimal für kleine, feste N (≤ 7)
    for (uint8_t i = 1; i < n; i++) {
        uint16_t key = tmp[i];
        int8_t   j   = static_cast<int8_t>(i) - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    return tmp[n / 2];
}

// ═════════════════════════════════════════════════════════════════════════════
//  SRF08Sensor
// ═════════════════════════════════════════════════════════════════════════════

SRF08Sensor::SRF08Sensor(uint8_t addr7bitOr8bit, uint8_t rangeReg, uint8_t gainReg)
    : _addr(normalizeSRF08Address(addr7bitOr8bit))
    , _rangeReg(rangeReg)
    , _gainReg(gainReg)
    , _wire(nullptr)
    , _state(SRF08State::UNINITIALIZED)
    , _rangingWindow(0)
    , _cycleTime(0)
    , _ema(0.0f)
    , _emaInit(false)
    , _emaAlpha(SRF08_EMA_ALPHA_DEF)
    , _jumpThreshold(SRF08_JUMP_DEF)
    , _lastValid(0)
    , _raw(0)
    , _filtered(0)
    , _light(0)
    , _hasData(false)
{
    _rangingWindow = _calcRangingWindow();
}

// ── Setup ─────────────────────────────────────────────────────────────────────

bool SRF08Sensor::begin(TwoWire& wire) {
    _wire = &wire;

    // Startup-Settling: SRF08 braucht nach Power-On etwas Zeit bevor er I²C-Anfragen beantwortet
    delay(15);

    // Sensor-Erreichbarkeit prüfen — bis zu 3 Versuche mit je 10 ms Pause
    bool found = false;
    for (uint8_t attempt = 0; attempt < 3 && !found; attempt++) {
        _wire->beginTransmission(_addr);
        if (_wire->endTransmission() == 0) {
            found = true;
        } else {
            delay(10);
        }
    }
    if (!found) return false;

    // Gain setzen BEVOR Range, da Range-Register = Echo-Register beim Lesen
    _writeReg(SRF08_REG_GAIN,  _gainReg);
    delayMicroseconds(300);
    _writeReg(SRF08_REG_RANGE, _rangeReg);
    delayMicroseconds(300);

    _rangingWindow = _calcRangingWindow();
    _state         = SRF08State::IDLE;
    return true;
}

void SRF08Sensor::setRange(uint8_t rangeReg) {
    _rangeReg      = rangeReg;
    _rangingWindow = _calcRangingWindow();
    if (_state != SRF08State::UNINITIALIZED)
        _writeReg(SRF08_REG_RANGE, _rangeReg);
}

void SRF08Sensor::setRangeMeters(float meters) {
    // range_mm = (reg × 43) + 43  →  reg = (range_mm / 43) - 1
    uint32_t mm  = static_cast<uint32_t>(meters * 1000.0f);
    int16_t  reg = static_cast<int16_t>(mm / 43) - 1;
    setRange(static_cast<uint8_t>(constrain(reg, 0, 255)));
}

void SRF08Sensor::setGain(uint8_t gainReg) {
    _gainReg = static_cast<uint8_t>(constrain(gainReg, 0, 31));
    if (_state != SRF08State::UNINITIALIZED)
        _writeReg(SRF08_REG_GAIN, _gainReg);
}

void SRF08Sensor::resetFilters() {
    _median.reset();
    _ema       = 0.0f;
    _emaInit   = false;
    _lastValid = 0;
    _hasData   = false;
}

// ── Non-blocking Betrieb ──────────────────────────────────────────────────────

void SRF08Sensor::startRanging() {
    if (_state == SRF08State::UNINITIALIZED) return;
    _writeReg(SRF08_REG_CMD, SRF08_CMD_RANGE_CM);
    _timer = 0;
    _state = SRF08State::RANGING;
}

bool SRF08Sensor::update() {
    if (_state != SRF08State::RANGING) return false;

    // ── Phase 1: Timer-basiertes Warten ───────────────────────────────────
    // Kein I²C-Overhead während der Hauptmesszeit
    if (_timer < _rangingWindow) return false;

    // ── Phase 2: Polling-Fenster (bis zu SRF08_POLL_TIMEOUT_MS) ──────────
    // Sensor antwortet mit 0xFF solange er misst (SDA pulled high = NAK)
    if (_timer < (_rangingWindow + SRF08_POLL_TIMEOUT_MS)) {
        if (_isBusy()) return false;
    }
    // Nach Timeout: Messung lesen auch wenn Polling nicht sauber abschloss
    // (verhindert dass ein I²C-Fehler den Sensor dauerhaft blockiert)

    // ── Phase 3: Daten lesen ──────────────────────────────────────────────
    // Reihenfolge wichtig: Lichtsensor VOR Echo lesen (beide fertig nach Ranging)
    _light   = _readByte(SRF08_REG_GAIN);   // Reg 1: nach Ranging = Lichtsensor
    _raw     = _readWord(SRF08_REG_ECHO_H); // Reg 2+3: Echo 1 High+Low
    _cycleTime = static_cast<uint32_t>(_timer);

    _filtered = _applyFilters(_raw);
    _hasData  = true;
    _state    = SRF08State::IDLE;
    return true;  // Neue Daten verfügbar
}

// ── I²C Adresse ändern ───────────────────────────────────────────────────────

bool SRF08Sensor::changeI2CAddress(uint8_t newAddr7or8bit) {
    if (_wire == nullptr) return false;

    uint8_t newAddr8bit = (newAddr7or8bit <= 0x7F)
        ? static_cast<uint8_t>(newAddr7or8bit << 1)
        : newAddr7or8bit;

    // Nur gerade Adressen im Bereich 0xE0–0xFE zulässig
    if (newAddr8bit < 0xE0 || newAddr8bit > 0xFE || (newAddr8bit & 0x01))
        return false;

    // Sequenz laut Datenblatt: A0, AA, A5, <neue Adresse>
    // Alle 4 Schreibvorgänge müssen auf Register 0 erfolgen, ohne Unterbrechung
    _writeReg(SRF08_REG_CMD, 0xA0); delayMicroseconds(100);
    _writeReg(SRF08_REG_CMD, 0xAA); delayMicroseconds(100);
    _writeReg(SRF08_REG_CMD, 0xA5); delayMicroseconds(100);
    _writeReg(SRF08_REG_CMD, newAddr8bit);
    delay(100); // Sensor braucht Zeit um Adresse zu speichern

    // Interne Adresse aktualisieren (7-bit für Wire)
    _addr = newAddr8bit >> 1;

    // Verify: Sensor unter neuer Adresse erreichbar?
    _wire->beginTransmission(_addr);
    return (_wire->endTransmission() == 0);
}

// ── Private Helfer ────────────────────────────────────────────────────────────

void SRF08Sensor::_writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

uint8_t SRF08Sensor::_readByte(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);  // Repeated Start (kein STOP-Bit)
    _wire->requestFrom(_addr, static_cast<uint8_t>(1));
    return _wire->available() ? _wire->read() : SRF08_BUSY;
}

uint16_t SRF08Sensor::_readWord(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, static_cast<uint8_t>(2));
    if (_wire->available() >= 2) {
        uint16_t hi = _wire->read();
        uint16_t lo = _wire->read();
        return (hi << 8) | lo;
    }
    return 0;
}

uint32_t SRF08Sensor::_calcRangingWindow() const {
    // Aus Datenblatt: range_mm = (rangeReg × 43) + 43
    // Laufzeit:       t = 2 × range_mm / 343 m/s + 2ms Overhead
    uint32_t range_mm = (static_cast<uint32_t>(_rangeReg) * 43UL) + 43UL;
    return (2UL * range_mm / 343UL) + 2UL;
}

bool SRF08Sensor::_isBusy() {
    // Liest Register 0: 0xFF = noch am Messen, anderer Wert = fertig
    // Bei NAK durch Teensy Wire: _readByte gibt 0xFF zurück → korrekt
    return (_readByte(SRF08_REG_CMD) == SRF08_BUSY);
}

// ── Filterpipeline ────────────────────────────────────────────────────────────
/**
 * Filterpipeline (in Reihenfolge):
 *
 *  raw → [1. Bereichscheck] → [2. Erstinitialisierung]
 *       → [3. Sprungfilter] → [4. Median] → [5. EMA] → _lastValid
 *
 * Der Sprungfilter arbeitet mit dem Median zusammen:
 *  - Einzelner Ausreißer (Ghost-Echo): Sprung erkannt, Median bestätigt NICHT
 *    → Wert in Median aufgenommen, Ausgabe eingefroren
 *  - Echte schnelle Bewegung: Mehrere aufeinanderfolgende große Werte
 *    → Median verschiebt sich → EMA wird hart zurückgesetzt
 */
uint16_t SRF08Sensor::_applyFilters(uint16_t raw) {

    // ── 1. Bereichsvalidierung ────────────────────────────────────────────
    // Wert 0 = kein Echo, außerhalb Bereich = ungültig
    if (raw == 0 || raw < SRF08_MIN_DIST_CM || raw > SRF08_MAX_DIST_CM) {
        // Kein gültiges Echo → letzten validen Wert halten
        // (Median nicht befüllen, da kein Objekt != Objekt sehr weit weg)
        return _lastValid;
    }

    // ── 2. Erste gültige Messung: alle Filter kaltstart-initialisieren ────
    if (!_emaInit || _lastValid == 0) {
        _median.push(raw);
        _ema       = static_cast<float>(raw);
        _emaInit   = true;
        _lastValid = raw;
        return _lastValid;
    }

    // ── 3. Sprungfilter (Jump Guard) ──────────────────────────────────────
    // Prüft ob die Änderung physikalisch plausibel ist
    uint16_t delta = (raw > _lastValid) ? (raw - _lastValid) : (_lastValid - raw);

    if (delta > _jumpThreshold) {
        // Großer Sprung erkannt — könnte Ghost-Echo oder echte Bewegung sein
        // Zuerst in Median aufnehmen, dann prüfen ob Median auch bestätigt
        _median.push(raw);
        uint16_t medianNow = _median.compute();
        uint16_t medDelta  = (medianNow > _lastValid)
                             ? (medianNow - _lastValid)
                             : (_lastValid - medianNow);

        if (medDelta > _jumpThreshold) {
            // Mehrere aufeinanderfolgende Sprung-Werte → echte Objektbewegung
            // EMA hart zurücksetzen um schnell nachzuführen
            _ema       = static_cast<float>(raw);
            _lastValid = raw;
        }
        // else: Ghost-Echo — Median noch nicht mitgezogen → Ausgabe einfrieren
        return _lastValid;
    }

    // ── 4. Median-Filter ──────────────────────────────────────────────────
    // Unterdrückt einzelne Ausreißer (Ghost-Echos, Rauschen)
    _median.push(raw);
    uint16_t med = _median.compute();

    // ── 5. EMA (Exponential Moving Average) ───────────────────────────────
    // Glättet Messrauschen; alpha = 0.30 → ca. 60% Einfluss letzter 3 Werte
    _ema = _emaAlpha * static_cast<float>(med)
         + (1.0f - _emaAlpha) * _ema;

    _lastValid = static_cast<uint16_t>(_ema + 0.5f); // kaufmännisch runden
    return _lastValid;
}

// ═════════════════════════════════════════════════════════════════════════════
//  SRF08Manager
// ═════════════════════════════════════════════════════════════════════════════

SRF08Manager::SRF08Manager()
    : _count(0)
    , _current(0)
{
    memset(_sensors, 0, sizeof(_sensors));
    memset(_newData, 0, sizeof(_newData));
}

bool SRF08Manager::addSensor(SRF08Sensor* sensor) {
    if (_count >= MAX_SENSORS || sensor == nullptr) return false;
    _sensors[_count++] = sensor;
    return true;
}

void SRF08Manager::begin(TwoWire& wire) {
    int8_t firstInitialized = -1;

    for (uint8_t i = 0; i < _count; i++) {
        bool ok = _sensors[i]->begin(wire);
        if (ok && firstInitialized < 0) {
            firstInitialized = static_cast<int8_t>(i);
        }
        delay(15); // Kurz warten zwischen Sensor-Inits
    }

    // Erste Messung nur mit erfolgreich initialisiertem Sensor starten
    if (firstInitialized >= 0) {
        _current = static_cast<uint8_t>(firstInitialized);
        _sensors[_current]->startRanging();
    }
}

void SRF08Manager::update() {
    if (_count == 0) return;

    SRF08Sensor* cur = _sensors[_current];
    if (cur == nullptr || cur->getState() == SRF08State::UNINITIALIZED) {
        int8_t next = _findNextInitializedIndex(static_cast<uint8_t>(_current + 1));
        if (next < 0) return; // Kein Sensor erfolgreich initialisiert
        _current = static_cast<uint8_t>(next);
        _sensors[_current]->startRanging();
        return;
    }

    // update() gibt true zurück wenn neue Daten bereit sind
    if (cur->update()) {
        _newData[_current] = true;

        // Nächsten erfolgreich initialisierten Sensor starten (sequenziell → kein Crosstalk)
        int8_t next = _findNextInitializedIndex(static_cast<uint8_t>(_current + 1));
        if (next < 0) return;
        _current = static_cast<uint8_t>(next);
        _sensors[_current]->startRanging();
    }
}

uint16_t SRF08Manager::getDistance(uint8_t idx) const {
    if (idx >= _count) return 0;
    return _sensors[idx]->getDistance();
}

uint16_t SRF08Manager::getRaw(uint8_t idx) const {
    if (idx >= _count) return 0;
    return _sensors[idx]->getRaw();
}

uint8_t SRF08Manager::getLightLevel(uint8_t idx) const {
    if (idx >= _count) return 0;
    return _sensors[idx]->getLightLevel();
}

bool SRF08Manager::isNewData(uint8_t idx) {
    if (idx >= _count) return false;
    bool nd        = _newData[idx];
    _newData[idx]  = false;
    return nd;
}

SRF08Sensor* SRF08Manager::getSensor(uint8_t idx) const {
    if (idx >= _count) return nullptr;
    return _sensors[idx];
}

int8_t SRF08Manager::_findNextInitializedIndex(uint8_t startIdx) const {
    if (_count == 0) return -1;

    for (uint8_t step = 0; step < _count; step++) {
        uint8_t idx = static_cast<uint8_t>((startIdx + step) % _count);
        SRF08Sensor* s = _sensors[idx];
        if (s != nullptr && s->getState() != SRF08State::UNINITIALIZED) {
            return static_cast<int8_t>(idx);
        }
    }
    return -1;
}
