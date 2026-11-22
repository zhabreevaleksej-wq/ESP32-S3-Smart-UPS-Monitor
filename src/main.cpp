
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Wire.h>
#include <INA226.h>
#include <LittleFS.h>
#include <math.h>
#include <esp_system.h>
#include <time.h>

// =====================================================
// ESP32-S3 SMART UPS MONITOR
// PlatformIO / Arduino
// ADC calibration + Wi-Fi reconnect + OTA
// System Diagnostics
// =====================================================

static const char* const FIRMWARE_VERSION = "v3.9-dev6";
static const char* const BUILD_DATE = __DATE__;
static const char* const BUILD_TIME = __TIME__;

// Wi-Fi and OTA credentials
#include "secrets.h"

// Hardware pins
constexpr uint8_t SDA_PIN = 16;
constexpr uint8_t SCL_PIN = 15;

// Adapter-presence divider: YX850 DC+ -> 100 kOhm -> GPIO10 -> 20 kOhm -> OUT-.
// This measures adapter INPUT presence, not the actual YX850 relay position.
constexpr uint8_t ADAPTER_SENSE_PIN = 10;
constexpr float ADAPTER_DIVIDER_RATIO = (100.0f + 20.0f) / 20.0f; // 6:1
constexpr float ADAPTER_ON_THRESHOLD_V = 1.20f;  // ADC pin volts
constexpr float ADAPTER_OFF_THRESHOLD_V = 0.65f; // ADC pin volts
constexpr uint8_t SOURCE_CONFIRM_SAMPLES = 2;


constexpr uint8_t TAP_PINS[6] = {
  4, 5, 6, 7, 8, 9
};

// Calibrated voltage-divider coefficients
constexpr float TAP_RATIO[6] = {
  8.0279f,  // B1 / GPIO4
  8.0337f,  // B2 / GPIO5
  7.9621f,  // B3 / GPIO6
  7.8000f,  // B4 / GPIO7 - not calibrated
  7.8000f,  // B5 / GPIO8 - not calibrated
  7.8000f   // B6 / GPIO9 - not calibrated
};

constexpr float CELL_MIN_DETECT = 2.5f;
constexpr float CELL_MAX_DETECT = 4.30f;

constexpr uint32_t SAMPLE_INTERVAL_MS = 3000;
constexpr uint32_t WIFI_RETRY_MS = 15000;
// Event history limits
constexpr size_t MAX_EVENT_RECORDS = 200;
// Persistent power-source state
constexpr const char* POWER_STATE_FILE = "/power_state.txt";

// Experimental v3.9: 5-minute measurement history.
// Keep at most 7 days of 5-minute records; compact down to 6 days
// only after the limit has been exceeded (avoid rewriting the file
// on every five-minute append).
constexpr uint32_t MEASUREMENT_LOG_INTERVAL_MS = 5UL * 60UL * 1000UL;
constexpr size_t MAX_MEASUREMENT_RECORDS = 7UL * 24UL * 12UL; // 2016
constexpr size_t COMPACT_KEEP_RECORDS = 6UL * 24UL * 12UL;   // 1728
constexpr const char* MEASUREMENT_LOG_FILE = "/measurements.csv";
constexpr const char* MEASUREMENT_TEMP_FILE = "/measurements.tmp";
constexpr const char* MEASUREMENT_CSV_HEADER =
    "timestamp,source,cells,battery_v,bus_v,current_ma,power_w,"
    "cell1_v,cell2_v,cell3_v,cell4_v,cell5_v,cell6_v\n";

// Energy counters: accumulated measured Wh, persisted approximately every 5 minutes.
constexpr const char* ENERGY_FILE = "/energy.txt";
constexpr const char* ENERGY_TEMP_FILE = "/energy.tmp";
constexpr uint32_t ENERGY_SAVE_INTERVAL_MS = 5UL * 60UL * 1000UL;
double adapterEnergyWh = 0.0;
double batteryEnergyWh = 0.0;
uint32_t lastEnergySampleMs = 0;
uint32_t lastEnergySaveMs = 0;
bool lastEnergyValid = false;
float lastEnergyPowerW = 0.0f;

// Outage statistics (v3.9-dev6): confirmed adapter absence, not relay position.
// Times are measured while the ESP32 is running; downtime during resets is unknown.
constexpr const char* OUTAGE_FILE = "/outages.txt";
constexpr const char* OUTAGE_TEMP_FILE = "/outages.tmp";
constexpr uint32_t OUTAGE_SAVE_INTERVAL_MS = 5UL * 60UL * 1000UL;
uint32_t outageCount = 0;
uint64_t outageTotalMs = 0;
uint64_t outageLastMs = 0;
uint64_t outageLongestMs = 0;
uint64_t outageActiveMs = 0;
uint32_t outageTickMs = 0;
uint32_t outageSaveMs = 0;
bool outageTickInitialized = false;

// Objects
WebServer server(80);
INA226 ina226(0x40);

// Measurement state
bool inaOK = false;
bool littlefsMounted = false;

float taps[6] = {};
float cells[6] = {};

int cellCount = 0;
bool batteryValid = false;

float batteryV = 0.0f;
float busV = 0.0f;
float currentMA = 0.0f;
float powerMW = 0.0f;



// Power-source indication is inferred from adapter voltage only.
enum class AdapterState : uint8_t { Unknown, Present, Absent };
AdapterState lastEnergySource = AdapterState::Unknown;

// Measurement history accumulator (new feature; event log untouched).
size_t measurementRecordCount = 0;
uint32_t measurementWindowStart = 0;
uint16_t measurementWindowSamples = 0;
AdapterState measurementSource = AdapterState::Unknown;
int measurementCellCount = 0;
double measurementBatteryV = 0.0;
double measurementBusV = 0.0;
double measurementCurrentMA = 0.0;
double measurementPowerMW = 0.0;
double measurementCells[6] = {};


AdapterState adapterState = AdapterState::Unknown;
AdapterState pendingAdapterState = AdapterState::Unknown;
AdapterState savedAdapterState = AdapterState::Unknown;

uint8_t pendingAdapterSamples = 0;
float adapterSenseV = 0.0f;
float adapterInputV = 0.0f; // Approximate: divider resistor/ADC tolerance
uint32_t pendingEventId = 0;
uint32_t pendingEventMillis = 0;
bool pendingEventTime = false;


// Timers
uint32_t lastSample = 0;
uint32_t lastWifiAttempt = 0;

// Network state
bool wifiWasConnected = false;
bool otaStarted = false;

// v3.9-dev6: completed outage journal, independent of v3.9-dev4 totals.
// CSV epoch timestamps are UTC; the browser formats them for Europe/Kyiv.
constexpr const char* OUTAGE_LOG_FILE = "/outages.csv";
constexpr const char* OUTAGE_LOG_TEMP = "/outages_csv.tmp";
constexpr const char* OUTAGE_PENDING_FILE = "/outage_pending.txt";
constexpr const char* OUTAGE_PENDING_TEMP = "/outage_pending.tmp";
constexpr const char* OUTAGE_CSV_HEADER = "start_epoch,end_epoch,measured_seconds,status\n";
constexpr size_t MAX_OUTAGE_LOG_RECORDS = 200;
uint32_t outageStartEpoch = 0;
uint32_t outageStartMillis = 0;
bool outageStartKnown = false;
bool outageInterrupted = false;
bool outagePendingExists = false;
size_t outageLogCount = 0;

// This metadata is written at loss and read at reboot. Duration comes from
// the existing v3.9-dev4 measured counter; no time is invented during resets.
bool savePendingOutage() {
  if (!littlefsMounted) return false;
  File f = LittleFS.open(OUTAGE_PENDING_TEMP, "w");
  if (!f) return false;
  String payload = String(outageStartEpoch) + "\n" +
      String(outageInterrupted ? 1 : 0) + "\n";
  bool ok = f.print(payload) == payload.length();
  if (ok) { f.flush(); ok = !f.getWriteError(); }
  f.close();
  if (!ok) { LittleFS.remove(OUTAGE_PENDING_TEMP); return false; }
  if (!LittleFS.rename(OUTAGE_PENDING_TEMP, OUTAGE_PENDING_FILE)) return false;
  outagePendingExists = true;
  return true;
}

void loadPendingOutage() {
  if (!littlefsMounted || !LittleFS.exists(OUTAGE_PENDING_FILE)) return;
  File f = LittleFS.open(OUTAGE_PENDING_FILE, "r");
  if (!f) return;
  String start = f.readStringUntil('\n');
  String interrupted = f.readStringUntil('\n');
  f.close();
  start.trim(); interrupted.trim();
  outageStartEpoch = (uint32_t)strtoul(start.c_str(), nullptr, 10);
  outageStartKnown = outageStartEpoch >= 1735689600UL;
  outageInterrupted = true; // Any reboot during an open episode makes it partial.
  outagePendingExists = true;
}

void initializeOutageJournal() {
  outageLogCount = 0;
  if (!littlefsMounted) return;
  File f = LittleFS.open(OUTAGE_LOG_FILE, "r");
  if (!f) return;
  while (f.available()) {
    String row = f.readStringUntil('\n');
    row.trim();
    if (row.length() && !row.startsWith("start_epoch,")) ++outageLogCount;
  }
  f.close();
}

bool compactOutageJournal() {
  if (outageLogCount <= MAX_OUTAGE_LOG_RECORDS) return true;
  File src = LittleFS.open(OUTAGE_LOG_FILE, "r");
  if (!src) return false;
  LittleFS.remove(OUTAGE_LOG_TEMP);
  File dst = LittleFS.open(OUTAGE_LOG_TEMP, "w");
  if (!dst) { src.close(); return false; }
  const size_t skip = outageLogCount - MAX_OUTAGE_LOG_RECORDS;
  size_t seen = 0;
  bool ok = dst.print(OUTAGE_CSV_HEADER) == strlen(OUTAGE_CSV_HEADER);
  while (ok && src.available()) {
    String row = src.readStringUntil('\n');
    row.trim();
    if (!row.length() || row.startsWith("start_epoch,")) continue;
    if (seen++ < skip) continue;
    if (dst.println(row) != row.length() + 2) ok = false;
  }
  if (ok) { dst.flush(); ok = !dst.getWriteError(); }
  dst.close(); src.close();
  if (!ok) { LittleFS.remove(OUTAGE_LOG_TEMP); return false; }
  if (!LittleFS.rename(OUTAGE_LOG_TEMP, OUTAGE_LOG_FILE)) return false;
  outageLogCount = MAX_OUTAGE_LOG_RECORDS;
  return true;
}

bool appendCompletedOutage(uint32_t endEpoch) {
  if (!littlefsMounted) return false;
  // Keep a missing start/end as 0 rather than presenting an invented date.
  const String status = outageInterrupted ? "partial_reset" :
      (!outageStartKnown || endEpoch == 0 ? "time_unknown" : "complete");
  const String row = String(outageStartKnown ? outageStartEpoch : 0) + "," +
      String(endEpoch) + "," + String((unsigned long)(outageLastMs / 1000ULL)) + "," + status + "\n";
  File f = LittleFS.open(OUTAGE_LOG_FILE, "a");
  if (!f) return false;
  bool ok = true;
  if (f.size() == 0) ok = f.print(OUTAGE_CSV_HEADER) == strlen(OUTAGE_CSV_HEADER);
  if (ok) ok = f.print(row) == row.length();
  if (ok) { f.flush(); ok = !f.getWriteError(); }
  f.close();
  if (!ok) return false;
  ++outageLogCount;
  if (!compactOutageJournal()) Serial.println("OUTAGE JOURNAL: Compaction deferred");
  return true;
}

// File stores completed and ongoing measured durations separately. A reboot
// during an outage resumes the counter without inventing the offline interval.
bool saveOutageStats() {
  if (!littlefsMounted) return false;
  File file = LittleFS.open(OUTAGE_TEMP_FILE, "w");
  if (!file) return false;
  char payload[160];
  snprintf(payload, sizeof(payload), "1\n%lu\n%llu\n%llu\n%llu\n%llu\n",
      (unsigned long)outageCount, (unsigned long long)outageTotalMs,
      (unsigned long long)outageLastMs, (unsigned long long)outageLongestMs,
      (unsigned long long)outageActiveMs);
  bool ok = file.print(payload) == strlen(payload);
  if (ok) { file.flush(); ok = !file.getWriteError(); }
  file.close();
  if (!ok) { LittleFS.remove(OUTAGE_TEMP_FILE); return false; }
  if (!LittleFS.rename(OUTAGE_TEMP_FILE, OUTAGE_FILE)) return false;
  outageSaveMs = millis();
  return true;
}

void loadOutageStats() {
  if (!littlefsMounted) return;
  File file = LittleFS.open(OUTAGE_FILE, "r");
  if (!file) return;
  String fields[6];
  for (int i = 0; i < 6; ++i) { fields[i] = file.readStringUntil('\n'); fields[i].trim(); }
  file.close();
  if (fields[0] != "1") return;
  for (int i = 1; i < 6; ++i) {
    if (!fields[i].length()) return;
    for (size_t j = 0; j < fields[i].length(); ++j)
      if (fields[i][j] < '0' || fields[i][j] > '9') return;
  }
  outageCount = strtoul(fields[1].c_str(), nullptr, 10);
  outageTotalMs = strtoull(fields[2].c_str(), nullptr, 10);
  outageLastMs = strtoull(fields[3].c_str(), nullptr, 10);
  outageLongestMs = strtoull(fields[4].c_str(), nullptr, 10);
  outageActiveMs = strtoull(fields[5].c_str(), nullptr, 10);
  Serial.println("OUTAGES: Previous statistics loaded");
}

void accumulateOutageTime() {
  const uint32_t now = millis();
  if (!outageTickInitialized) { outageTickInitialized = true; outageTickMs = now; return; }
  const uint32_t dt = (uint32_t)(now - outageTickMs);
  outageTickMs = now;
  if (adapterState == AdapterState::Absent && dt <= 15000UL) {
    outageActiveMs += dt;
    outageTotalMs += dt;
  }
  if ((uint32_t)(now - outageSaveMs) >= OUTAGE_SAVE_INTERVAL_MS) {
    if (!saveOutageStats()) Serial.println("OUTAGES: Save deferred");
  }
}

// =====================================================
// ADC
// =====================================================

float readTap(uint8_t channel) {
  const uint8_t pin = TAP_PINS[channel];

  // Discard first reading after switching ADC channel
  analogReadMilliVolts(pin);
  delay(2);

  uint32_t sum = 0;

  for (int j = 0; j < 24; ++j) {
    sum += analogReadMilliVolts(pin);
    delay(1);
  }

  const float adcVoltage =
      (sum / 24.0f) / 1000.0f;

  return adcVoltage * TAP_RATIO[channel];
}

// =====================================================
// ADAPTER INPUT SENSING (GPIO10 / ADC1)
// =====================================================

bool isTimeValid();
bool appendEvent(const char* eventType);
bool savePowerState(AdapterState state);

void sampleAdapterInput() {
  // Discard the first ADC conversion after changing channels.
  analogReadMilliVolts(ADAPTER_SENSE_PIN);
  delay(2);

  uint32_t sumMV = 0;
  for (int j = 0; j < 24; ++j) {
    sumMV += analogReadMilliVolts(ADAPTER_SENSE_PIN);
    delay(1);
  }

  adapterSenseV = (sumMV / 24.0f) / 1000.0f;
  adapterInputV = adapterSenseV * ADAPTER_DIVIDER_RATIO;

  // Hysteresis: ignore readings between the two thresholds.
  AdapterState observed = AdapterState::Unknown;
  if (adapterSenseV >= ADAPTER_ON_THRESHOLD_V) {
    observed = AdapterState::Present;
  } else if (adapterSenseV <= ADAPTER_OFF_THRESHOLD_V) {
    observed = AdapterState::Absent;
  }

  if (observed == AdapterState::Unknown || observed == adapterState) {
    pendingAdapterState = AdapterState::Unknown;
    pendingAdapterSamples = 0;
    return;
  }

  if (observed != pendingAdapterState) {
    pendingAdapterState = observed;
    pendingAdapterSamples = 1;
  } else if (pendingAdapterSamples < SOURCE_CONFIRM_SAMPLES) {
    ++pendingAdapterSamples;
  }

  if (pendingAdapterSamples >= SOURCE_CONFIRM_SAMPLES) {
    const AdapterState previousState = adapterState;

    adapterState = observed;
    pendingAdapterState = AdapterState::Unknown;
    pendingAdapterSamples = 0;

    if (previousState == AdapterState::Unknown) {
      // First confirmed measurement after boot: compare with LittleFS.
      Serial.println(adapterState == AdapterState::Present
        ? "EVENT: Initial state - adapter present"
        : "EVENT: Initial state - adapter absent");

      if (savedAdapterState == AdapterState::Present &&
          adapterState == AdapterState::Absent) {
        Serial.println("EVENT: Adapter input lost while restarting (inferred)");
        appendEvent("adapter_lost");
      } else if (savedAdapterState == AdapterState::Absent &&
                 adapterState == AdapterState::Present) {
        Serial.println("EVENT: Adapter input restored while restarting (inferred)");
        appendEvent("adapter_restored");
      }
    } else if (adapterState == AdapterState::Present) {
      Serial.println("EVENT: Adapter input restored");
      appendEvent("adapter_restored");
    } else {
      Serial.println("EVENT: Adapter input lost");
      appendEvent("adapter_lost");
    }

    outageTickMs = millis();
    outageTickInitialized = true;

    // Count only confirmed adapter-loss episodes. If the saved state was
    // already absent at boot, resume the existing episode without counting twice.
    if (adapterState == AdapterState::Absent) {
      if (previousState != AdapterState::Unknown || savedAdapterState != AdapterState::Absent) {
        ++outageCount;
        outageActiveMs = 0;
      }
      if (previousState != AdapterState::Unknown || savedAdapterState != AdapterState::Absent) {
        outageStartEpoch = isTimeValid() ? (uint32_t)time(nullptr) : 0;
        outageStartKnown = outageStartEpoch != 0;
        outageStartMillis = millis();
        outageInterrupted = previousState == AdapterState::Unknown;
        if (!savePendingOutage()) Serial.println("OUTAGE JOURNAL: Pending start save failed");
      } else if (!outagePendingExists) {
        // Existing v3.9-dev4 outage upgraded while already on battery.
        outageStartEpoch = 0;
        outageStartKnown = false;
        outageInterrupted = true;
        savePendingOutage();
      }
      if (!saveOutageStats()) Serial.println("OUTAGES: Save after loss failed");
    } else if (adapterState == AdapterState::Present &&
               (previousState == AdapterState::Absent ||
                (previousState == AdapterState::Unknown && savedAdapterState == AdapterState::Absent))) {
      outageLastMs = outageActiveMs;
      if (outageLastMs > outageLongestMs) outageLongestMs = outageLastMs;
      outageActiveMs = 0;
      // If NTP became available during the same boot, reconstruct the start
      // from monotonic millis only when no reset interrupted the episode.
      const uint32_t endEpoch = isTimeValid() ? (uint32_t)time(nullptr) : 0;
      if (!outageStartKnown && !outageInterrupted && endEpoch) {
        outageStartEpoch = endEpoch - (uint32_t)((millis() - outageStartMillis) / 1000UL);
        outageStartKnown = true;
      }
      if (appendCompletedOutage(endEpoch)) {
        LittleFS.remove(OUTAGE_PENDING_FILE);
        outagePendingExists = false;
      } else Serial.println("OUTAGE JOURNAL: Append failed; pending retained");
      if (!saveOutageStats()) Serial.println("OUTAGES: Save after restore failed");
    }

    // Persist only confirmed changes, not every ADC sample.
    // Update the in-memory saved state only after a successful write.
    if (savedAdapterState != adapterState && savePowerState(adapterState)) {
      savedAdapterState = adapterState;
      Serial.println("POWER STATE: Confirmed state saved");
    }
  }
}

// =====================================================
// MEASUREMENTS
// =====================================================

void sampleSensors() {
  for (int i = 0; i < 6; ++i) {
    taps[i] = readTap(i);
    cells[i] = 0.0f;
  }

  cellCount = 0;
  batteryValid = false;
  batteryV = 0.0f;

  float previous = 0.0f;

  for (int i = 0; i < 6; ++i) {
    const float v = taps[i] - previous;

    if (!isfinite(v) ||
        v < CELL_MIN_DETECT ||
        v > CELL_MAX_DETECT) {
      break;
    }

    cells[i] = v;
    ++cellCount;
    previous = taps[i];
  }

  if (cellCount > 0) {
    batteryV = taps[cellCount - 1];
    batteryValid = true;

    for (int i = cellCount; i < 6; ++i) {
      if (taps[i] > 0.5f) {
        batteryValid = false;
      }
    }
  }

  sampleAdapterInput();

  if (inaOK) {
    busV = ina226.getBusVoltage();
    currentMA = ina226.getCurrent_mA();
    powerMW = ina226.getPower_mW();
  }
}

// =====================================================
// TIME VALIDATION (NTP)
// =====================================================

bool isTimeValid() {
  time_t now = time(nullptr);

  // Reject timestamps before 2025-01-01 UTC.
  return now >= 1735689600;
}

// =====================================================
// EVENT LOG (LittleFS)
// =====================================================

// Keep the most recent MAX_EVENT_RECORDS entries.


// Read last confirmed power-source state from LittleFS
AdapterState loadPowerState() {
  if (!littlefsMounted) {
    return AdapterState::Unknown;
  }

  File file = LittleFS.open(POWER_STATE_FILE, "r");
  if (!file) {
    return AdapterState::Unknown;
  }

  String state = file.readStringUntil('\n');
  file.close();
  state.trim();

  if (state == "adapter") {
    return AdapterState::Present;
  }

  if (state == "battery") {
    return AdapterState::Absent;
  }

  return AdapterState::Unknown;
}

// Save last confirmed power-source state to LittleFS
bool savePowerState(AdapterState state) {
  if (!littlefsMounted || state == AdapterState::Unknown) {
    return false;
  }

  File file = LittleFS.open(POWER_STATE_FILE, "w");
  if (!file) {
    Serial.println("POWER STATE: Failed to open file");
    return false;
  }

  const char* value =
      state == AdapterState::Present ? "adapter\n" : "battery\n";

  size_t written = file.print(value);
  file.close();

  if (written != strlen(value)) {
    Serial.println("POWER STATE: Write failed");
    return false;
  }

  return true;
}


bool trimEventLog() {
  if (!littlefsMounted || !LittleFS.exists("/events.csv")) {
    return false;
  }

  File source = LittleFS.open("/events.csv", "r");
  if (!source) return false;

  size_t lines = 0;
  while (source.available()) {
    if (source.read() == '\n') ++lines;
  }

  if (lines <= MAX_EVENT_RECORDS) {
    source.close();
    return true;
  }

  const size_t skipLines = lines - MAX_EVENT_RECORDS;

if (!source.seek(0)) {
  Serial.println("EVENT LOG: Failed to rewind source");
  source.close();
  return false;
}

if (LittleFS.exists("/events.tmp")) {
  if (!LittleFS.remove("/events.tmp")) {
    Serial.println("EVENT LOG: Failed to remove old temp file");
    source.close();
    return false;
  }
}

  File temp = LittleFS.open("/events.tmp", "w");
  if (!temp) {
    source.close();
    return false;
  }

  size_t currentLine = 0;
  bool copyOK = true;

  while (source.available()) {
    String line = source.readStringUntil('\n');

    if (currentLine++ < skipLines) continue;

    if (temp.print(line + "\n") != line.length() + 1) {
  copyOK = false;
  break;
}
  }

    if (copyOK) {
    temp.flush();
    if (temp.getWriteError()) {
      Serial.println("EVENT LOG: Flush failed");
      copyOK = false;
    }
  }

  temp.close();
  source.close();

  if (!copyOK) {
    LittleFS.remove("/events.tmp");
    return false;
  }

  // Keep the original file until the temporary copy is ready.
  if (!LittleFS.rename("/events.tmp", "/events.csv")) {
    Serial.println("EVENT LOG: Trim rename failed");
    return false;
  }

  return true;
}

bool appendEvent(const char* eventType) {
  if (!littlefsMounted || !eventType) {
    return false;
  }

  File file = LittleFS.open("/events.csv", "a");

  if (!file) {
    Serial.println("EVENT LOG: Failed to open file");
    return false;
  }

  time_t now = time(nullptr);

  // Zero means that a reliable timestamp is unavailable.
  if (!isTimeValid()) {
    now = 0;
  }

  String record = String((uint32_t)now);
  record += ",";
  record += eventType;
  if (now == 0 && !pendingEventTime) {
    pendingEventId = esp_random();
    if (!pendingEventId) pendingEventId = 1;
    pendingEventMillis = millis();
    pendingEventTime = true;
    record += ",boot:";
    record += String(pendingEventId);
  }
  record += "\n";

  const size_t written = file.print(record);
  file.close();

  if (written != record.length()) {
    Serial.println("EVENT LOG: Write failed");
    pendingEventTime = false;
    return false;
  }

  Serial.print("EVENT LOG: ");
Serial.print(record);

if (!trimEventLog()) {
  Serial.println("EVENT LOG: Trim failed");
  return false;
}

return true;
}

// After NTP sync, resolve only the pending event created in THIS boot.
// The calculated time is approximate (the power loss precedes reboot).
void resolvePendingEventTime() {
  if (!pendingEventTime || !isTimeValid() || !littlefsMounted) return;
  File src = LittleFS.open("/events.csv", "r");
  if (!src) return;
  LittleFS.remove("/events_time.tmp");
  File dst = LittleFS.open("/events_time.tmp", "w");
  if (!dst) { src.close(); return; }
  const String marker = String(",boot:") + String(pendingEventId);
  const uint32_t elapsed = (uint32_t)(millis() - pendingEventMillis) / 1000UL;
  const uint32_t approxEpoch = (uint32_t)time(nullptr) - elapsed;
  bool found = false;
  bool ok = true;
  while (src.available()) {
    String row = src.readStringUntil('\n');
    row.trim();
    if (!row.length()) continue;
    if (row.endsWith(marker)) {
      int comma = row.indexOf(',');
      if (comma > 0) {
        row = String(approxEpoch) + row.substring(comma, row.length() - marker.length()) + ",approx";
        found = true;
      }
    }
    if (dst.println(row) != row.length() + 2) { ok = false; break; }
  }
  dst.flush();
  if (dst.getWriteError()) ok = false;
  dst.close();
  src.close();
  if (ok && found && LittleFS.rename("/events_time.tmp", "/events.csv")) {
    pendingEventTime = false;
    Serial.println("EVENT LOG: Approximate boot event timestamp resolved");
  } else {
    LittleFS.remove("/events_time.tmp");
  }
}

// =====================================================
// MEASUREMENT HISTORY (LittleFS, experimental v3.9)
// =====================================================

void resetMeasurementWindow() {
  measurementWindowStart = 0;
  measurementWindowSamples = 0;
  measurementSource = AdapterState::Unknown;
  measurementCellCount = 0;
  measurementBatteryV = 0;
  measurementBusV = 0;
  measurementCurrentMA = 0;
  measurementPowerMW = 0;
  for (double &value : measurementCells) value = 0;
}

// Count existing CSV rows once during boot, without loading the file in RAM.
void initializeMeasurementLog() {
  measurementRecordCount = 0;
  if (!littlefsMounted || !LittleFS.exists(MEASUREMENT_LOG_FILE)) return;

  File input = LittleFS.open(MEASUREMENT_LOG_FILE, "r");
  if (!input) {
    Serial.println("MEASUREMENTS: Cannot open existing log");
    return;
  }
  while (input.available()) {
    String line = input.readStringUntil('\n');
    line.trim();
    if (line.length() && !line.startsWith("timestamp,")) {
      ++measurementRecordCount;
    }
  }
  input.close();
  Serial.printf("MEASUREMENTS: %u existing records\n",
                (unsigned)measurementRecordCount);
}

// Compact only after more than 7 days of rows have accumulated.
// Keep the original file until a complete temporary copy is ready.
bool compactMeasurementLog() {
  if (!littlefsMounted || measurementRecordCount <= MAX_MEASUREMENT_RECORDS) {
    return true;
  }
  File src = LittleFS.open(MEASUREMENT_LOG_FILE, "r");
  if (!src) return false;
  if (LittleFS.exists(MEASUREMENT_TEMP_FILE) &&
      !LittleFS.remove(MEASUREMENT_TEMP_FILE)) {
    src.close();
    return false;
  }
  File dst = LittleFS.open(MEASUREMENT_TEMP_FILE, "w");
  if (!dst) {
    src.close();
    return false;
  }

  const size_t skip = measurementRecordCount - COMPACT_KEEP_RECORDS;
  size_t encountered = 0;
  bool ok = dst.print(MEASUREMENT_CSV_HEADER) == strlen(MEASUREMENT_CSV_HEADER);
  while (ok && src.available()) {
    String line = src.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("timestamp,")) continue;
    if (encountered++ < skip) continue;
    if (dst.print(line + "\n") != line.length() + 1) ok = false;
  }
  if (ok) {
    dst.flush();
    if (dst.getWriteError()) ok = false;
  }
  dst.close();
  src.close();

  if (!ok) {
    LittleFS.remove(MEASUREMENT_TEMP_FILE);
    Serial.println("MEASUREMENTS: Failed to compact, original preserved");
    return false;
  }
  if (!LittleFS.rename(MEASUREMENT_TEMP_FILE, MEASUREMENT_LOG_FILE)) {
    Serial.println("MEASUREMENTS: Failed to replace history file");
    return false;
  }
  measurementRecordCount = COMPACT_KEEP_RECORDS;
  Serial.println("MEASUREMENTS: Compacted to 6 days of data");
  return true;
}

bool appendMeasurementWindow() {
  if (!littlefsMounted || !measurementWindowSamples || !isTimeValid()) {
    return false;
  }
  const double count = measurementWindowSamples;
  String record;
  record.reserve(176);
  record += String((uint32_t)time(nullptr));
  record += ',';
  record += (measurementSource == AdapterState::Present) ? "adapter" : "battery";
  record += ',';
  record += String(measurementCellCount);
  record += ',';
  record += String(measurementBatteryV / count, 3);
  record += ',';
  record += String(measurementBusV / count, 3);
  record += ',';
  record += String(measurementCurrentMA / count, 2);
  record += ',';
  record += String(measurementPowerMW / (1000.0 * count), 3);
  for (int i = 0; i < 6; ++i) {
    record += ',';
    record += i < measurementCellCount
        ? String(measurementCells[i] / count, 3) : String("0.000");
  }
  record += '\n';

  File file = LittleFS.open(MEASUREMENT_LOG_FILE, "a");
  if (!file) {
    Serial.println("MEASUREMENTS: Cannot open CSV for append");
    return false;
  }
  bool ok = true;
  if (file.size() == 0) {
    ok = file.print(MEASUREMENT_CSV_HEADER) == strlen(MEASUREMENT_CSV_HEADER);
  }
  if (ok) ok = file.print(record) == record.length();
  if (ok) {
    file.flush();
    ok = !file.getWriteError();
  }
  file.close();

  if (!ok) {
    Serial.println("MEASUREMENTS: Failed to write CSV row");
    return false;
  }
  ++measurementRecordCount;
  Serial.printf("MEASUREMENTS: CSV row stored (%u total)\n",
                (unsigned)measurementRecordCount);
  if (!compactMeasurementLog()) {
    Serial.println("MEASUREMENTS: Compaction deferred");
  }
  return true;
}

// Called once after each 3-second sensor sample. Averages samples across
// a 5-minute window, rather than writing continuously to flash.
void accumulateMeasurement() {
  const bool valid = inaOK && batteryValid && cellCount > 0 &&
      adapterState != AdapterState::Unknown &&
      isfinite(batteryV) && isfinite(busV) &&
      isfinite(currentMA) && isfinite(powerMW);
  if (!valid) {
    resetMeasurementWindow();
    return;
  }
  for (int i = 0; i < cellCount; ++i) {
    if (!isfinite(cells[i])) {
      resetMeasurementWindow();
      return;
    }
  }
  if (measurementWindowSamples &&
      (measurementSource != adapterState || measurementCellCount != cellCount)) {
    // Do not average together samples from different input-source states.
    resetMeasurementWindow();
  }
  if (!measurementWindowSamples) {
    measurementWindowStart = millis();
    measurementSource = adapterState;
    measurementCellCount = cellCount;
  }
  measurementBatteryV += batteryV;
  measurementBusV += busV;
  measurementCurrentMA += currentMA;
  measurementPowerMW += powerMW;
  for (int i = 0; i < cellCount; ++i) measurementCells[i] += cells[i];
  ++measurementWindowSamples;

  if ((uint32_t)(millis() - measurementWindowStart) >=
      MEASUREMENT_LOG_INTERVAL_MS) {
    if (!appendMeasurementWindow()) {
      Serial.println("MEASUREMENTS: Window skipped (time/FS/write unavailable)");
    }
    resetMeasurementWindow();
  }
}

// =====================================================
// ENERGY ACCOUNTING (v3.9-dev6)
// =====================================================
void loadEnergyCounters() {
  if (!littlefsMounted) return;
  File file = LittleFS.open(ENERGY_FILE, "r");
  if (!file) return;
  String a = file.readStringUntil('\n');
  String b = file.readStringUntil('\n');
  file.close();
  double av = a.toDouble(), bv = b.toDouble();
  if (isfinite(av) && isfinite(bv) && av >= 0 && bv >= 0) {
    adapterEnergyWh = av;
    batteryEnergyWh = bv;
    Serial.println("ENERGY: Previous totals loaded");
  }
}

bool saveEnergyCounters() {
  if (!littlefsMounted) return false;
  File file = LittleFS.open(ENERGY_TEMP_FILE, "w");
  if (!file) return false;
  String payload = String(adapterEnergyWh, 6) + "\n" +
                   String(batteryEnergyWh, 6) + "\n";
  bool ok = file.print(payload) == payload.length();
  if (ok) {
    file.flush();
    ok = !file.getWriteError();
  }
  file.close();
  if (!ok) { LittleFS.remove(ENERGY_TEMP_FILE); return false; }
  if (!LittleFS.rename(ENERGY_TEMP_FILE, ENERGY_FILE)) return false;
  lastEnergySaveMs = millis();
  return true;
}

void accumulateEnergy() {
  const uint32_t now = millis();
  const bool valid = inaOK && isfinite(powerMW) && powerMW >= 0 &&
      adapterState != AdapterState::Unknown;
  if (lastEnergyValid && valid && adapterState == lastEnergySource) {
    const uint32_t dt = (uint32_t)(now - lastEnergySampleMs);
    // Avoid counting outages, stalls or changes of detected source.
    if (dt > 0 && dt <= 15000UL) {
      const double wh = (double)lastEnergyPowerW * ((double)dt / 3600000.0);
      if (adapterState == AdapterState::Present) adapterEnergyWh += wh;
      else batteryEnergyWh += wh;
    }
  }
  lastEnergySampleMs = now;
  lastEnergyValid = valid;
  lastEnergySource = adapterState;
  lastEnergyPowerW = valid ? powerMW / 1000.0f : 0.0f;
  if ((uint32_t)(now - lastEnergySaveMs) >= ENERGY_SAVE_INTERVAL_MS) {
    if (!saveEnergyCounters()) Serial.println("ENERGY: Save deferred");
  }
}

// =====================================================
// JSON API
// =====================================================

const char* getResetReason();

void sendData() {
  String json;
  json.reserve(768);

  json += "{";

  json += "\"inaReady\":";
  json += (inaOK ? "true" : "false");

  json += ",\"batteryValid\":";
  json += (batteryValid ? "true" : "false");

  json += ",\"detectedCells\":";
  json += String(cellCount);

  json += ",\"batteryVoltage\":";
  json += String(batteryV, 3);

  json += ",\"busVoltage\":";
  json += String(busV, 3);

  json += ",\"currentMA\":";
  json += String(currentMA, 2);

  json += ",\"powerMW\":";
  json += String(powerMW, 2);

  json += ",\"adapterSenseVoltage\":";
  json += String(adapterSenseV, 3);

  json += ",\"adapterInputVoltage\":";
  json += String(adapterInputV, 2);

  json += ",\"powerSource\":\"";
  json += adapterState == AdapterState::Present ? "adapter" :
          adapterState == AdapterState::Absent ? "battery" : "unknown";
  json += "\"";

  json += ",\"wifiConnected\":";
  json += (
    WiFi.status() == WL_CONNECTED ? "true" : "false"
  );

  json += ",\"wifiRSSI\":";
  json += String(
    WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0
  );

   json += ",\"uptimeSeconds\":";
  json += String(millis() / 1000UL);

  // System diagnostics
  json += ",\"resetReason\":\"";
  json += getResetReason();
  json += "\"";

    // Free heap memory
  json += ",\"freeHeap\":";
  json += String(ESP.getFreeHeap());

      // LittleFS diagnostics
  json += ",\"littlefs_mounted\":";
  json += (littlefsMounted ? "true" : "false");

  json += ",\"littlefs_total\":";
  json += String(littlefsMounted ? LittleFS.totalBytes() : 0);

  json += ",\"littlefs_used\":";
  json += String(littlefsMounted ? LittleFS.usedBytes() : 0);

  json += ",\"adapterEnergyWh\":";
  json += String(adapterEnergyWh, 4);
  json += ",\"batteryEnergyWh\":";
  json += String(batteryEnergyWh, 4);

  json += ",\"outageCount\":" + String(outageCount);
  json += ",\"outageTotalSeconds\":" + String((double)outageTotalMs / 1000.0, 1);
  json += ",\"outageLastSeconds\":" + String((double)outageLastMs / 1000.0, 1);
  json += ",\"outageLongestSeconds\":" + String((double)outageLongestMs / 1000.0, 1);
  json += ",\"outageActiveSeconds\":" + String((double)outageActiveMs / 1000.0, 1);

  // Diagnostics for the experimental history logger.
  json += ",\"measurementRecords\":";
  json += String(measurementRecordCount);

// NTP diagnostics
json += ",\"ntpSynced\":";
json += (isTimeValid() ? "true" : "false");

json += ",\"unixTime\":";
json += String((uint32_t)time(nullptr));

  json += ",\"otaReady\":";
  json += (otaStarted ? "true" : "false");

  // Firmware information
  json += ",\"firmwareVersion\":\"";
  json += FIRMWARE_VERSION;
  json += "\"";

  json += ",\"buildDate\":\"";
  json += BUILD_DATE;
  json += "\"";

  json += ",\"buildTime\":\"";
  json += BUILD_TIME;
  json += "\"";

  json += ",\"taps\":[";
  for (int i = 0; i < 6; ++i) {
    if (i) json += ",";
    json += String(taps[i], 3);
  }
  json += "]";

  json += ",\"cells\":[";
  for (int i = 0; i < 6; ++i) {
    if (i) json += ",";
    json += String(cells[i], 3);
  }
  json += "]";

  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(
    200,
    "application/json; charset=utf-8",
    json
  );
}

// =====================================================
// WEB PAGE
// =====================================================

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="uk"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-S3 UPS Monitor v3.9-dev6</title>
<style>
:root{color-scheme:dark;font-family:system-ui,Arial,sans-serif}*{box-sizing:border-box}
body{margin:0;background:#0b1220;color:#e8f0ff;font-size:14px}
.wrap{max-width:1240px;margin:auto;padding:12px 16px 22px}
header{text-align:center;padding:12px 10px 8px}h1{font-size:21px;margin:0}header p{margin:3px 0;color:#9bb3d6;font-size:12px}
.tabs{display:flex;gap:5px;background:#14223b;border:1px solid #304765;border-radius:12px;padding:4px;margin:8px 0 12px}
.tab{flex:1;background:transparent;border:0;border-radius:9px;color:#b8c9e0;padding:10px;cursor:pointer;font-size:14px;font-weight:650}
.tab.active{background:#2c4768;color:#fff}.panel{display:none}.panel.active{display:block}
.card{background:#182940;border:1px solid #304765;border-radius:11px;padding:12px;min-width:0}
.label{font-size:12px;color:#a6bfdf}.value{font-size:21px;font-weight:750;margin-top:3px;overflow-wrap:anywhere}
.grid{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:9px}
.source{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:9px}
.source strong{font-size:19px}.source small{color:#a6bfdf;display:block;margin-top:3px}
.ok{color:#4ade80}.warn{color:#fbbf24}.danger{color:#fb7185}.muted{color:#9bb3d6;font-size:12px}
h2{font-size:15px;margin:13px 0 7px}.cellgrid{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:8px}
.cellitem{background:#182940;border:1px solid #304765;border-radius:9px;padding:9px;min-width:0}
.cellitem strong{display:block;font-size:16px;margin-top:3px}.track{height:5px;background:#33455d;border-radius:8px;margin-top:7px;overflow:hidden}
.fill{height:100%;background:#35c98c}.fill.warnfill{background:#fbbf24}
.charts{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:9px}.chart{display:block;width:100%;height:132px}
.foot{font-size:11px;color:#93a9c8;margin:10px 0}.notice{padding:8px 11px;margin-bottom:9px;font-size:12px}
.tablewrap{max-height:65vh;overflow:auto;border:1px solid #304765;border-radius:10px}
table{width:100%;border-collapse:collapse}th,td{text-align:left;padding:11px 12px;border-bottom:1px solid #304765}th{color:#a6bfdf;background:#14223b;position:sticky;top:0}td:first-child{white-space:nowrap}
.analyticsgrid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:9px}
.historycontrols{display:flex;gap:9px;align-items:center;flex-wrap:wrap;margin:10px 0}.historycontrols button{background:#203957;color:#e8f0ff;border:1px solid #486587;border-radius:8px;padding:8px 12px;cursor:pointer}.historycontrols button.selected{background:#34577d;border-color:#80c7ff}.historychart{height:205px;width:100%;display:block}.historycharts{display:grid;grid-template-columns:1fr;gap:10px}.systemgrid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:9px}
@media(max-width:900px){.grid{grid-template-columns:repeat(3,minmax(0,1fr))}.cellgrid{grid-template-columns:repeat(3,minmax(0,1fr))}.systemgrid{grid-template-columns:repeat(2,minmax(0,1fr))}.analyticsgrid{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media(max-width:600px){.wrap{padding:8px}.grid{grid-template-columns:repeat(2,minmax(0,1fr))}.charts{grid-template-columns:1fr}.chart{height:105px}.source strong{font-size:16px}.cellitem strong{font-size:14px}.systemgrid{grid-template-columns:repeat(2,minmax(0,1fr))}.analyticsgrid{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style></head><body><div class="wrap"><header><h1>ESP32-S3 UPS Monitor</h1><p>Моніторинг резервного живлення · <span id="headerVersion">v3.9-dev6</span></p></header>
<nav class="tabs" aria-label="Вкладки"><button class="tab active" data-tab="main">Головна</button><button class="tab" data-tab="events">Журнал</button><button class="tab" data-tab="analytics">Аналітика</button><button class="tab" data-tab="system">Система</button></nav>
<section id="panel-main" class="panel active">
<div class="card source"><div><div class="label">Поточне джерело живлення (за входом адаптера)</div><strong id="powerSource">Визначення...</strong><small id="lastSwitch">Останнє перемикання: —</small></div><div id="adapterVoltage" class="muted">DC+: —</div></div>
<div id="status" class="card notice muted">Очікування вимірювань...</div>
<div class="grid">
<div class="card"><div class="label">Акумулятор</div><div class="value" id="battery">—</div></div>
<div class="card"><div class="label">Напруга шини</div><div class="value" id="bus">—</div></div>
<div class="card"><div class="label">Струм</div><div class="value" id="current">—</div></div>
<div class="card"><div class="label">Потужність</div><div class="value" id="power">—</div></div>
<div class="card"><div class="label">Заряд (оцінка)</div><div class="value" id="soc">—</div></div>
<div class="card"><div class="label">Розбаланс</div><div class="value" id="imbalance">—</div></div>
</div>
<h2>Напруга комірок <span id="count" class="muted"></span></h2><div class="cellgrid" id="cells"></div>
<h2>Балансувальні канали B1–B6</h2><div class="cellgrid" id="taps"></div>
<h2>Графіки (останні 60 вимірювань)</h2><div class="charts">
<div class="card"><div class="label">Напруга шини, В</div><canvas class="chart" id="chartV"></canvas></div>
<div class="card"><div class="label">Струм, мА</div><canvas class="chart" id="chartI"></canvas></div>
<div class="card"><div class="label">Потужність, Вт</div><canvas class="chart" id="chartP"></canvas></div></div>
<p class="foot">Графіки поточних вимірювань зберігаються лише в браузері та починаються заново після оновлення сторінки.</p>
<p class="foot">Без BMS вебмоніторинг не запобігає перерозряду. Джерело живлення визначається за входом DC+, а не положенням реле YX850.</p>
</section>
<section id="panel-events" class="panel"><h2>Журнал подій живлення</h2><p class="muted">≈ — приблизний час відключення, визначений після запуску ESP32 та синхронізації NTP.</p><div class="tablewrap"><table><thead><tr><th>Дата та час</th><th>Подія</th></tr></thead><tbody id="events"><tr><td colspan="2">Завантаження...</td></tr></tbody></table></div>
<h2>Завершені відключення</h2>
<details id="outagesDetails" class="card" style="margin:12px 0 14px">
 <summary style="cursor:pointer;font-weight:700">Журнал завершених відключень <span id="outagesCountLabel" class="muted">(завантаження...)</span></summary>
 <p class="muted">Тільки завершені події. «Неповний час» означає перезапуск ESP32 під час відключення; показано лише підтверджений час роботи. Часові позначки без NTP не вигадуються.</p>
 <div class="historycontrols"><button type="button" id="reloadOutages">Оновити журнал</button><a href="/api/outages.csv" download="outages.csv" style="color:#80c7ff">Завантажити CSV</a></div>
 <p id="outagesStatus" class="muted">Журнал ще не завантажено.</p>
 <div class="tablewrap"><table><thead><tr><th>Початок</th><th>Відновлення</th><th>Підтверджена тривалість</th><th>Примітка</th></tr></thead><tbody id="outagesRows"><tr><td colspan="4">Немає даних</td></tr></tbody></table></div>
 <div class="historycontrols"><button type="button" id="moreOutages" hidden>Показати ще 10</button></div>
</details>
</section>
<section id="panel-analytics" class="panel">
<h2>Аналітика енергоспоживання</h2>
<p class="muted">Аналітичні оцінки розраховуються в браузері за наявними 5-хвилинними усередненими записами LittleFS. Відсутні інтервали не враховуються як нульове споживання.</p>
<h2>Накопичена енергія (лічильники ESP32)</h2>
<div class="analyticsgrid" style="margin-bottom:12px">
<div class="card"><div class="label">Енергія від адаптера</div><div class="value" id="adapterWh">—</div></div>
<div class="card"><div class="label">Енергія від акумулятора</div><div class="value" id="batteryWh">—</div></div>
</div>
<p class="muted">Лічильники ведуться з v3.9-dev3 та зберігаються приблизно кожні 5 хвилин. Останні хвилини можуть втратитися при раптовому знеструмленні.</p>
<h2>Оцінка споживання за останні періоди</h2>
<div class="analyticsgrid" style="margin-bottom:10px">
<div class="card"><div class="label">Останні 24 години · за записами</div><div class="value" id="estimate24h">—</div></div>
<div class="card"><div class="label">Останні 7 днів · за записами</div><div class="value" id="estimate7d">—</div></div>
</div>
<h2>Аналіз за вибраний період</h2>
<p class="muted">Оберіть період для статистики та історичних графіків.</p>
<div class="historycontrols"><button type="button" data-range="1" class="selected">1 година</button><button type="button" data-range="24">24 години</button><button type="button" data-range="168">7 днів</button><button type="button" id="reloadHistory">Оновити</button></div>
<div class="analyticsgrid" style="margin-bottom:10px">
<div class="card"><div class="label">Споживання · оцінка</div><div class="value" id="periodWh">—</div></div>
<div class="card"><div class="label">Середня потужність за записами</div><div class="value" id="periodAvgPower">—</div></div>
<div class="card"><div class="label">Максимум 5-хв. середньої потужності</div><div class="value" id="periodPeakPower">—</div></div>
<div class="card"><div class="label">Покриття записами · оцінка</div><div class="value" id="periodCoverage">—</div></div>
</div>
<div class="analyticsgrid" style="margin-bottom:10px">
<div class="card"><div class="label">Від адаптера · оцінка</div><div class="value" id="periodAdapterWh">—</div></div>
<div class="card"><div class="label">Від акумулятора · оцінка</div><div class="value" id="periodBatteryWh">—</div></div>
</div>
<p id="analyticsNote" class="muted">Немає даних для розрахунку.</p>
<h2>Історичні графіки (LittleFS)</h2>
<p id="historyStatus" class="muted" aria-live="polite">Відкрийте вкладку, щоб завантажити історію.</p>
<div class="historycharts"><div class="card"><div class="label">Напруга акумулятора, В</div><canvas id="historicBattery" class="historychart"></canvas></div><div class="card"><div class="label">Струм, мА</div><canvas id="historicCurrent" class="historychart"></canvas></div><div class="card"><div class="label">Потужність, Вт</div><canvas id="historicPower" class="historychart"></canvas></div></div>
<p class="foot">Проміжки без записів не означають нульове споживання. Після зміни періоду показуються лише наявні вимірювання.</p>
<h2>Статистика відключень адаптера</h2>
<div class="systemgrid" style="margin-bottom:12px">
<div class="card"><div class="label">Кількість відключень</div><div class="value" id="outageCount">—</div></div>
<div class="card"><div class="label">Останнє завершене відключення</div><div class="value" id="outageLast">—</div></div>
<div class="card"><div class="label">Загальний час без адаптера</div><div class="value" id="outageTotal">—</div></div>
<div class="card"><div class="label">Найдовше завершене відключення</div><div class="value" id="outageLongest">—</div></div>
</div><p class="muted">Тривалість враховується лише під час роботи ESP32; час перезавантаження невідомий. Статистика починається з v3.9-dev4. Активне відключення враховується в загальному часі, але до завершення не входить до показників останнього та найдовшого.</p>
</section>
<section id="panel-system" class="panel"><h2>Діагностика системи</h2><div class="systemgrid">
<div class="card"><div class="label">Версія прошивки</div><div class="value" id="firmwareVersion">—</div></div>
<div class="card"><div class="label">INA226</div><div class="value" id="inaState">—</div></div>
<div class="card"><div class="label">Wi-Fi / RSSI</div><div class="value" id="wifi">—</div></div>
<div class="card"><div class="label">Час роботи</div><div class="value" id="uptime">—</div></div>
<div class="card"><div class="label">Причина перезапуску</div><div class="value" id="resetReason">—</div></div>
<div class="card"><div class="label">Вільна RAM</div><div class="value" id="freeHeap">—</div></div>
<div class="card"><div class="label">LittleFS</div><div class="value" id="littlefs">—</div></div>
<div class="card"><div class="label">Історія вимірювань</div><div class="value" id="historyRecords">—</div></div>
<div class="card"><div class="label">NTP</div><div class="value" id="ntp">—</div></div>
<div class="card"><div class="label">OTA</div><div class="value" id="ota">—</div></div>
<div class="card"><div class="label">Дата компіляції</div><div class="value" id="buildDateTime" style="font-size:15px">—</div></div>
</div><p class="foot"><a href="/api/measurements" style="color:#80c7ff">Завантажити історію вимірювань (CSV)</a></p><h2>Попередження</h2><div id="alerts" class="card muted">Очікування даних...</div></section>
</div><script>
const $=id=>document.getElementById(id);
let activeTab='main';let latestData=null;let events=[];let historicRows=[];let historicRangeHours=1;let historicLoaded=false;
const history={v:[],i:[],p:[]};
function setTab(tab){activeTab=tab;document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('active',b.dataset.tab===tab));document.querySelectorAll('.panel').forEach(p=>p.classList.toggle('active',p.id==='panel-'+tab));if(tab==='events'){updateEvents();loadOutages();}if(tab==='main')drawCharts();if(tab==='analytics'){if(!historicLoaded)loadHistoricData();else drawHistoricCharts();}}
document.querySelectorAll('.tab').forEach(b=>b.addEventListener('click',()=>setTab(b.dataset.tab)));
function add(k,v){if(!Number.isFinite(v))return;history[k].push(v);if(history[k].length>60)history[k].shift();}
function line(canvas,values){if(!canvas.clientWidth||!canvas.clientHeight)return;const dpr=window.devicePixelRatio||1,w=canvas.clientWidth,h=canvas.clientHeight;canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);const c=canvas.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);c.strokeStyle='#38516f';c.lineWidth=1;for(let j=1;j<4;j++){let y=h*j/4;c.beginPath();c.moveTo(8,y);c.lineTo(w-8,y);c.stroke();}if(!values.length)return;let lo=Math.min(...values),hi=Math.max(...values);if(hi-lo<0.1){lo-=0.05;hi+=0.05;}c.strokeStyle='#55b5ff';c.lineWidth=2;c.beginPath();values.forEach((v,j)=>{const x=9+j*(w-18)/Math.max(1,values.length-1),y=h-10-(v-lo)/(hi-lo)*(h-20);if(!j)c.moveTo(x,y);else c.lineTo(x,y);});c.stroke();c.fillStyle='#aec4df';c.font='12px system-ui';c.fillText(hi.toFixed(2),10,13);c.fillText(lo.toFixed(2),10,h-3);}
function drawCharts(){line($('chartV'),history.v);line($('chartI'),history.i);line($('chartP'),history.p);}
// History rendering uses the existing read-only CSV endpoint; no new flash writes.
function historyDate(sec){return new Date(sec*1000).toLocaleString('uk-UA',{timeZone:'Europe/Kyiv',day:'2-digit',month:'2-digit',hour:'2-digit',minute:'2-digit'});}
async function loadHistoricData(){
 const status=$('historyStatus');status.textContent='Завантаження історії...';
 try{
  const response=await fetch('/api/measurements',{cache:'no-store'});
  if(!response.ok)throw Error('HTTP '+response.status);
  const csv=await response.text();const rows=[];
  for(const line of csv.split(/\r?\n/)){
   if(!line||line.startsWith('timestamp,'))continue;
   const c=line.trim().split(',');if(c.length<7)continue;
   const t=Number(c[0]),battery=Number(c[3]),current=Number(c[5]),power=Number(c[6]);
   if(Number.isFinite(t)&&t>0&&[battery,current,power].every(Number.isFinite))rows.push({t,battery,current,power,source:c[1]});
  }
  historicRows=rows.sort((a,b)=>a.t-b.t);historicLoaded=true;drawHistoricCharts();
 }catch(err){status.textContent='Не вдалося завантажити історію: '+err.message;console.error(err);}
}
function historicLine(canvas,rows,key,unit){
 const w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;
 const dpr=window.devicePixelRatio||1;canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
 const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);
 const left=58,right=12,top=20,bottom=33,pw=w-left-right,ph=h-top-bottom;
 ctx.font='11px system-ui';ctx.strokeStyle='#38516f';ctx.fillStyle='#aec4df';ctx.lineWidth=1;
 if(!rows.length){ctx.fillText('Немає даних за вибраний період',left,top+25);return;}
 let min=Math.min(...rows.map(r=>r[key])),max=Math.max(...rows.map(r=>r[key]));
 const margin=Math.max((max-min)*0.12,key==='battery'?0.025:key==='power'?0.05:1);min-=margin;max+=margin;
 const start=Math.min(...rows.map(r=>r.t)),end=Math.max(...rows.map(r=>r.t));
 for(let j=0;j<=3;j++){const y=top+ph*j/3;ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(w-right,y);ctx.stroke();ctx.fillText((max-(max-min)*j/3).toFixed(key==='current'?0:2),4,y+4);}
 const x=t=>left+(end===start?pw/2:(t-start)/(end-start)*pw),y=v=>top+(max-v)/(max-min)*ph;
 ctx.strokeStyle='#55b5ff';ctx.lineWidth=2;ctx.beginPath();
 rows.forEach((r,i)=>{if(i===0)ctx.moveTo(x(r.t),y(r[key]));else ctx.lineTo(x(r.t),y(r[key]));});ctx.stroke();
 if(rows.length===1){ctx.fillStyle='#55b5ff';ctx.beginPath();ctx.arc(x(rows[0].t),y(rows[0][key]),3,0,Math.PI*2);ctx.fill();}
 ctx.fillStyle='#aec4df';ctx.textAlign='left';ctx.fillText(historyDate(start),left,h-7);ctx.textAlign='right';ctx.fillText(historyDate(end),w-right,h-7);ctx.textAlign='left';
}
// Each stored row is approximately one averaged five-minute window.
// We never interpret a missing interval as zero or extrapolate across outages.
function summarizePeriod(hours){
 const now=Math.floor(Date.now()/1000),cutoff=now-hours*3600;
 const rows=historicRows.filter(r=>r.t>=cutoff&&r.t<=now+120);
 const good=rows.filter(r=>Number.isFinite(r.power)&&r.power>=0&&(r.source==='adapter'||r.source==='battery'));
 const intervalHours=5/60;
 const wh=good.reduce((sum,r)=>sum+r.power*intervalHours,0);
 const adapter=good.filter(r=>r.source==='adapter');
 const battery=good.filter(r=>r.source==='battery');
 return {rows,good,wh,adapterWh:adapter.reduce((sum,r)=>sum+r.power*intervalHours,0),batteryWh:battery.reduce((sum,r)=>sum+r.power*intervalHours,0),avg:good.length?good.reduce((sum,r)=>sum+r.power,0)/good.length:null,max:good.length?Math.max(...good.map(r=>r.power)):null,minutes:good.length*5,hours};
}
function displayEstimate(value,unit){return value===null?'—':value.toFixed(3)+' '+unit;}
function renderAnalyticsMetrics(){
 const selected=summarizePeriod(historicRangeHours),last24=summarizePeriod(24),last7d=summarizePeriod(168);
 $('estimate24h').textContent=displayEstimate(last24.good.length?last24.wh:null,'Wh');
 $('estimate7d').textContent=displayEstimate(last7d.good.length?last7d.wh:null,'Wh');
 $('periodWh').textContent=displayEstimate(selected.good.length?selected.wh:null,'Wh');
 $('periodAdapterWh').textContent=displayEstimate(selected.good.some(r=>r.source==='adapter')?selected.adapterWh:null,'Wh');
 $('periodBatteryWh').textContent=displayEstimate(selected.good.some(r=>r.source==='battery')?selected.batteryWh:null,'Wh');
 $('periodAvgPower').textContent=selected.avg===null?'—':selected.avg.toFixed(2)+' Вт';
 $('periodPeakPower').textContent=selected.max===null?'—':selected.max.toFixed(2)+' Вт';
 $('periodCoverage').textContent=selected.good.length?Math.min(100,selected.minutes/(selected.hours*60)*100).toFixed(0)+' %':'—';
 $('analyticsNote').textContent=selected.good.length?
  'Оцінка за '+selected.good.length+' збереженими інтервалами (≈ '+selected.minutes+' хв). Кожен запис вважається 5-хвилинним. Перерви та час перезапусків не оцінюються; максимальна потужність — найбільша середня за інтервал, не миттєвий пік.':
  'За вибраний період немає придатних записів для оцінки споживання.';
}
function drawHistoricCharts(){
 const m=summarizePeriod(historicRangeHours),rows=m.rows;
 $('historyStatus').textContent=rows.length+' записів за '+(historicRangeHours===1?'1 годину':historicRangeHours===24?'24 години':'7 днів')+' (усього збережено: '+historicRows.length+').';
 renderAnalyticsMetrics();
 historicLine($('historicBattery'),rows,'battery','В');historicLine($('historicCurrent'),rows,'current','мА');historicLine($('historicPower'),rows,'power','Вт');
}
document.querySelectorAll('[data-range]').forEach(button=>button.addEventListener('click',()=>{historicRangeHours=Number(button.dataset.range);document.querySelectorAll('[data-range]').forEach(b=>b.classList.toggle('selected',b===button));if(historicLoaded)drawHistoricCharts();else loadHistoricData();}));
$('reloadHistory').addEventListener('click',loadHistoricData);

window.addEventListener('resize',()=>{if(activeTab==='main')drawCharts();if(activeTab==='analytics'&&historicLoaded)drawHistoricCharts();});
function durationText(sec){if(!Number.isFinite(sec))return '—';const n=Math.floor(Math.max(0,sec));return Math.floor(n/3600)+' год '+Math.floor(n%3600/60)+' хв '+n%60+' с';}
function approxSOC(v){const p=[[3,0],[3.3,5],[3.5,10],[3.6,20],[3.7,35],[3.8,50],[3.9,65],[4,80],[4.1,92],[4.2,100]];if(v<=3)return 0;if(v>=4.2)return 100;for(let j=1;j<p.length;j++)if(v<=p[j][0]){let a=p[j-1],b=p[j];return Math.round(a[1]+(v-a[0])*(b[1]-a[1])/(b[0]-a[0]));}return 100;}
function formatDate(sec){if(!(sec>0))return null;return new Date(sec*1000).toLocaleString('uk-UA',{timeZone:'Europe/Kyiv',day:'2-digit',month:'2-digit',year:'numeric',hour:'2-digit',minute:'2-digit'});}
function eventLabel(type){return type==='adapter_lost'?'Адаптер відключено — акумулятор':type==='adapter_restored'?'Адаптер підключено — мережа':type;}
function renderLastSwitch(){const last=events[0];$('lastSwitch').textContent=last&&last.time>0?'Останнє перемикання: '+(last.approx?'≈ ':'')+formatDate(last.time):'Останнє перемикання: —';}
async function updateEvents(){try{const r=await fetch('/api/events',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);const csv=await r.text();events=csv.split('\n').map(x=>x.trim()).filter(Boolean).map(x=>{const [time,type,flag]=x.split(',');return {time:Number(time),type,approx:flag==='approx'};}).reverse();renderLastSwitch();if(activeTab!=='events')return;const box=$('events');box.replaceChildren();if(!events.length){const tr=document.createElement('tr');const td=document.createElement('td');td.colSpan=2;td.textContent='Подій поки немає.';tr.appendChild(td);box.appendChild(tr);return;}events.forEach(e=>{const tr=document.createElement('tr'),td1=document.createElement('td'),td2=document.createElement('td');td1.textContent=e.time>0?(e.approx?'≈ ':'')+formatDate(e.time):'Немає збереженого часу';td2.textContent=eventLabel(e.type);td2.className=e.type==='adapter_lost'?'warn':e.type==='adapter_restored'?'ok':'';tr.append(td1,td2);box.appendChild(tr);});}catch(e){console.error(e);if(activeTab==='events')$('events').textContent='Не вдалося завантажити журнал.';}}
let outageEntries=[];let outageVisible=10;
function renderOutages(){
 const body=$('outagesRows'),status=$('outagesStatus');body.replaceChildren();
 $('outagesCountLabel').textContent='('+outageEntries.length+' записів)';
 status.textContent='Показано '+Math.min(outageVisible,outageEntries.length)+' із '+outageEntries.length+' завершених записів (максимум 200).';
 if(!outageEntries.length){const tr=document.createElement('tr'),td=document.createElement('td');td.colSpan=4;td.textContent='Завершених відключень у журналі поки немає.';tr.appendChild(td);body.appendChild(tr);}
 outageEntries.slice(0,outageVisible).forEach(line=>{
  const [a,b,d,flag]=line.split(','),tr=document.createElement('tr');
  const cells=[formatDate(Number(a))||'Час невідомий',formatDate(Number(b))||'Час невідомий',durationText(Number(d)),flag==='partial_reset'?'Неповний час (перезапуск)':flag==='time_unknown'?'Час NTP недоступний':'Завершено'];
  cells.forEach(value=>{const td=document.createElement('td');td.textContent=value;tr.appendChild(td);});body.appendChild(tr);
 });
 $('moreOutages').hidden=outageVisible>=outageEntries.length;
}
async function loadOutages(){
 const status=$('outagesStatus');
 try{
  const r=await fetch('/api/outages.csv',{cache:'no-store'});
  if(!r.ok)throw Error('HTTP '+r.status);
  outageEntries=(await r.text()).split(/\r?\n/).filter(x=>x.trim()&&!x.startsWith('start_epoch,')).reverse();
  outageVisible=10;renderOutages();
 }catch(e){status.textContent='Помилка завантаження журналу: '+e.message;}
}
$('reloadOutages').addEventListener('click',loadOutages);
$('moreOutages').addEventListener('click',()=>{outageVisible+=10;renderOutages();});
function tile(label,value,warning){const el=document.createElement('div');el.className='cellitem';const a=document.createElement('div');a.className='label';a.textContent=label;const b=document.createElement('strong');b.textContent=value;if(warning)b.className='warn';el.append(a,b);return el;}
async function update(){try{const r=await fetch('/api/data',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);const d=await r.json();latestData=d;$('powerSource').textContent=d.powerSource==='adapter'?'Адаптер 12 В':d.powerSource==='battery'?'Акумулятор':'Визначення...';$('powerSource').className=d.powerSource==='adapter'?'ok':d.powerSource==='battery'?'warn':'';$('adapterVoltage').textContent='Вхід DC+: '+(typeof d.adapterInputVoltage==='number'?'≈ '+d.adapterInputVoltage.toFixed(2)+' В':'—');$('status').textContent=d.batteryValid?'Батарея: конфігурацію визначено':'Конфігурацію батареї не підтверджено';$('status').className='card notice '+(d.batteryValid?'ok':'warn');$('battery').textContent=d.batteryValid?d.batteryVoltage.toFixed(2)+' В':'—';$('bus').textContent=d.inaReady?d.busVoltage.toFixed(2)+' В':'—';$('current').textContent=d.inaReady?d.currentMA.toFixed(1)+' мА':'—';$('power').textContent=d.inaReady?(d.powerMW/1000).toFixed(2)+' Вт':'—';$('soc').textContent=d.batteryValid?approxSOC(d.batteryVoltage/d.detectedCells)+' %':'—';$('count').textContent=d.batteryValid?'· '+d.detectedCells+'S':'';
const cellBox=$('cells');cellBox.replaceChildren();const vs=d.batteryValid?d.cells.slice(0,d.detectedCells):[];const imbalance=vs.length?Math.max(...vs)-Math.min(...vs):0;$('imbalance').textContent=vs.length?(imbalance*1000).toFixed(0)+' мВ':'—';if(vs.length){vs.forEach((v,i)=>{const el=tile('Комірка '+(i+1),v.toFixed(3)+' В',v<3.3||v>4.2);const track=document.createElement('div');track.className='track';const fill=document.createElement('div');fill.className='fill'+(v<3.3||v>4.2?' warnfill':'');fill.style.width=Math.max(0,Math.min(100,v/4.2*100))+'%';track.appendChild(fill);el.appendChild(track);cellBox.appendChild(el);});}else cellBox.textContent='Немає підтверджених даних про комірки.';
const tapBox=$('taps');tapBox.replaceChildren();d.taps.forEach((v,i)=>tapBox.appendChild(tile('B'+(i+1),v.toFixed(3)+' В')));
const warnings=[];if(vs.some(v=>v<3.3))warnings.push('Низька напруга однієї або кількох комірок.');if(vs.some(v=>v>4.2))warnings.push('Напруга комірки перевищує 4,20 В.');if(imbalance>0.10)warnings.push('Різниця між комірками понад 100 мВ.');$('alerts').textContent=!d.batteryValid?'Перевір балансувальні канали.':warnings.length?warnings.join(' '):'Критичних відхилень за заданими порогами не виявлено.';
$('outageCount').textContent=typeof d.outageCount==='number'?d.outageCount:'—';$('outageLast').textContent=durationText(d.outageLastSeconds);$('outageTotal').textContent=durationText(d.outageTotalSeconds);$('outageLongest').textContent=durationText(d.outageLongestSeconds);$('adapterWh').textContent=typeof d.adapterEnergyWh==='number'?d.adapterEnergyWh.toFixed(3)+' Wh':'—';$('batteryWh').textContent=typeof d.batteryEnergyWh==='number'?d.batteryEnergyWh.toFixed(3)+' Wh':'—';$('firmwareVersion').textContent=d.firmwareVersion||'—';$('headerVersion').textContent=d.firmwareVersion||'v3.9-dev6';$('inaState').textContent=d.inaReady?'OK':'Помилка';$('inaState').className='value '+(d.inaReady?'ok':'warn');$('wifi').textContent=d.wifiConnected?d.wifiRSSI+' dBm':'Відключено';$('uptime').textContent=Math.floor(d.uptimeSeconds/3600)+' год '+Math.floor(d.uptimeSeconds%3600/60)+' хв';$('resetReason').textContent=d.resetReason||'—';$('freeHeap').textContent=typeof d.freeHeap==='number'?(d.freeHeap/1024).toFixed(1)+' КБ':'—';$('littlefs').textContent=d.littlefs_mounted?((d.littlefs_used/1024).toFixed(0)+' / '+(d.littlefs_total/1024).toFixed(0)+' КБ'):'Помилка';$('historyRecords').textContent=typeof d.measurementRecords==='number'?d.measurementRecords+' / 2016':'—';$('ntp').textContent=d.ntpSynced?'Синхронізовано':'Очікування';$('ota').textContent=d.otaReady?'Готово':'Очікування';$('buildDateTime').textContent=d.buildDate&&d.buildTime?d.buildDate+' '+d.buildTime:'—';if(d.inaReady){add('v',d.busVoltage);add('i',d.currentMA);add('p',d.powerMW/1000);if(activeTab==='main')drawCharts();}
}catch(e){$('status').textContent='Втрачено зв’язок з ESP32';$('status').className='card notice danger';console.error(e);}}
update();updateEvents();setInterval(update,3000);setInterval(updateEvents,10000);
</script></body></html>
)HTML";

// =====================================================
// WI-FI MANAGEMENT
// =====================================================

void maintainWiFi() {
  const bool connected =
      WiFi.status() == WL_CONNECTED;

  if (connected) {
    if (!wifiWasConnected) {
      Serial.println("Wi-Fi connected/restored!");

      Serial.print("ESP32 IP: ");
      Serial.println(WiFi.localIP());

      Serial.print("ESP32 MAC: ");
      Serial.println(WiFi.macAddress());

      Serial.print("Wi-Fi RSSI: ");
      Serial.println(WiFi.RSSI());
    }

    wifiWasConnected = true;

    if (!otaStarted) {
      ArduinoOTA.begin();
      otaStarted = true;
      Serial.println("OTA initialized");
    }

    return;
  }

  if (wifiWasConnected) {
    Serial.println("Wi-Fi connection lost!");
    wifiWasConnected = false;
  }

  if (millis() - lastWifiAttempt >= WIFI_RETRY_MS) {
    lastWifiAttempt = millis();

    Serial.println("Wi-Fi reconnect attempt...");

    WiFi.reconnect();
  }
}

// =====================================================
// SETUP
// =====================================================


// ======================================================
// SYSTEM DIAGNOSTICS — RESET REASON
// ======================================================

const char* getResetReason() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:
            return "Power on";

        case ESP_RST_EXT:
            return "External reset";

        case ESP_RST_SW:
            return "Software reset";

        case ESP_RST_PANIC:
            return "System panic";

        case ESP_RST_INT_WDT:
            return "Interrupt watchdog";

        case ESP_RST_TASK_WDT:
            return "Task watchdog";

        case ESP_RST_WDT:
            return "Watchdog reset";

        case ESP_RST_DEEPSLEEP:
            return "Deep sleep wakeup";

        case ESP_RST_BROWNOUT:
            return "Brownout";

        case ESP_RST_SDIO:
            return "SDIO reset";

        default:
            return "Unknown";
    }
}

void setup() {
  Serial.begin(115200);
  delay(1200);

    // LittleFS: mount existing filesystem without formatting
  littlefsMounted = LittleFS.begin(false, "/littlefs", 10, "spiffs");

  
if (littlefsMounted) {
  Serial.println("LittleFS mounted successfully");
  Serial.printf("LittleFS total: %u bytes\n",
                (unsigned)LittleFS.totalBytes());
  Serial.printf("LittleFS used: %u bytes\n",
                (unsigned)LittleFS.usedBytes());
} else {
  Serial.println("LittleFS mount failed - formatting disabled");
}

// Count persistent measurement-history records at boot.
initializeMeasurementLog();
  loadEnergyCounters();
  loadOutageStats();
  initializeOutageJournal();
  loadPendingOutage();
  outageSaveMs = millis();
  lastEnergySaveMs = millis();

// Restore last confirmed power-source state
savedAdapterState = loadPowerState();

Serial.print("POWER STATE: Previous state = ");

switch (savedAdapterState) {
  case AdapterState::Present:
    Serial.println("adapter");
    break;

  case AdapterState::Absent:
    Serial.println("battery");
    break;

  default:
    Serial.println("unknown");
    break;
}

  Serial.println();
  Serial.println("==============================");
  Serial.print("ESP32-S3 UPS Monitor ");
  Serial.println(FIRMWARE_VERSION);

  Serial.print("Build: ");
  Serial.print(BUILD_DATE);
  Serial.print(" ");
  Serial.println(BUILD_TIME);

  Serial.println("==============================");

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  for (uint8_t p : TAP_PINS) {
    pinMode(p, INPUT);
  }
  pinMode(ADAPTER_SENSE_PIN, INPUT);

   // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Optional INA226
  inaOK = ina226.begin();

  if (inaOK) {
    int calibrationResult =
        ina226.setMaxCurrentShunt(3.0, 0.01, false);

    if (calibrationResult == 0) {
      Serial.println("INA226 OK");
    } else {
      inaOK = false;
      Serial.println("INA226 CALIBRATION ERROR");
    }
  } else {
    Serial.println("INA226 NOT FOUND");
  }

  sampleSensors();
  lastSample = millis();

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttempt = millis();

  Serial.println("Wi-Fi connection started");

// NTP time synchronization (Ukraine)
configTzTime(
  "EET-2EEST,M3.5.0/3,M10.5.0/4",
  "pool.ntp.org",
  "time.google.com"
);

Serial.println("NTP synchronization configured");

  // OTA
  ArduinoOTA.setHostname("esp32-ups");
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA update started");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA update finished");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error: %u\n", error);
  });

  // Web server
  server.on("/", HTTP_GET, []() {
    server.send_P(
      200,
      "text/html; charset=utf-8",
      PAGE
    );
  });

  server.on("/api/data", HTTP_GET, sendData);

  // Download experiment's stored 5-minute measurements as CSV.
  server.on("/api/measurements", HTTP_GET, []() {
    if (!littlefsMounted) {
      server.send(503, "text/plain; charset=utf-8", "LittleFS unavailable");
      return;
    }
    File file = LittleFS.open(MEASUREMENT_LOG_FILE, "r");
    if (!file) {
      server.send(200, "text/csv; charset=utf-8", MEASUREMENT_CSV_HEADER);
      return;
    }
    server.sendHeader("Content-Disposition",
                      "attachment; filename=\"measurements.csv\"");
    server.streamFile(file, "text/csv; charset=utf-8");
    file.close();
  });

  server.on("/api/outages.csv", HTTP_GET, []() {
    if (!littlefsMounted) {
      server.send(503, "text/plain; charset=utf-8", "LittleFS unavailable");
      return;
    }
    File f = LittleFS.open(OUTAGE_LOG_FILE, "r");
    server.sendHeader("Cache-Control", "no-store");
    if (!f) {
      server.send(200, "text/csv; charset=utf-8", OUTAGE_CSV_HEADER);
      return;
    }
    server.streamFile(f, "text/csv; charset=utf-8");
    f.close();
  });

  server.on("/api/events", HTTP_GET, []() {
  if (!littlefsMounted) {
    server.send(503, "text/plain; charset=utf-8",
                "LittleFS unavailable");
    return;
  }

  File file = LittleFS.open("/events.csv", "r");

  if (!file) {
    server.send(200, "text/plain; charset=utf-8", "");
    return;
  }

  server.streamFile(file, "text/plain; charset=utf-8");
  file.close();
});

  
  server.begin();

  Serial.println("Web server started");
  Serial.println("Setup completed");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  maintainWiFi();

  if (WiFi.status() == WL_CONNECTED && otaStarted) {
    ArduinoOTA.handle();
  }

  server.handleClient();
  resolvePendingEventTime();

  if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = millis();
    sampleSensors();
    accumulateMeasurement();
    accumulateEnergy();
    accumulateOutageTime();
  }

  delay(5);
}

// =====================================================
// END
// =====================================================
