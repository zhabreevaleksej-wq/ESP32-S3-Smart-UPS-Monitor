
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
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

static const char* const FIRMWARE_VERSION = "v3.9";
static const char* const BUILD_DATE = __DATE__;
static const char* const BUILD_TIME = __TIME__;

// Wi-Fi and OTA credentials
#include "secrets.h"

// Telegram tg3: українські команди, автоматичні повідомлення GPV1.2 і GPIO10.
// Token and chat ID are stored only in ignored include/secrets.h.
#ifndef TELEGRAM_BOT_TOKEN
#error "Define TELEGRAM_BOT_TOKEN in include/secrets.h"
#endif
#ifndef TELEGRAM_CHAT_ID
#error "Define TELEGRAM_CHAT_ID in include/secrets.h"
#endif

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
// Valid only when the loss was actually detected in this boot. A reset makes
// millis() from the prior boot unusable, even if a pending journal is present.
bool outageStartMillisValid = false;
bool outageStartKnown = false;
// This is the time of detection, NOT necessarily when power actually failed.
bool outageStartApprox = false;
bool outageInterrupted = false;
bool outagePendingExists = false;
size_t outageLogCount = 0;

// This metadata is written at loss and read at reboot. Duration comes from
// the existing v3.9-dev4 measured counter; no time is invented during resets.
bool savePendingOutage() {
  if (!littlefsMounted) return false;
  File f = LittleFS.open(OUTAGE_PENDING_TEMP, "w");
  if (!f) return false;
  // Third line is optional for older v3.9-dev files (backward compatible).
  String payload = String(outageStartEpoch) + "\n" +
      String(outageInterrupted ? 1 : 0) + "\n" +
      String(outageStartApprox ? 1 : 0) + "\n";
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
  String approximate = f.readStringUntil('\n');
  f.close();
  start.trim(); interrupted.trim(); approximate.trim();
  outageStartEpoch = (uint32_t)strtoul(start.c_str(), nullptr, 10);
  outageStartKnown = outageStartEpoch >= 1735689600UL;
  outageStartApprox = approximate == "1";
  outageStartMillisValid = false; // Old millis() is not valid after reboot.
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
  // Existing four-column CSV remains compatible. Only add more precise flags.
  const String status = outageInterrupted ?
      (outageStartKnown && outageStartApprox ? "partial_reset_approx" : "partial_reset") :
      (!outageStartKnown || endEpoch == 0 ? "time_unknown" :
       (outageStartApprox ? "start_approx" : "complete"));
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
        outageStartMillisValid = true;
        outageInterrupted = previousState == AdapterState::Unknown;
        // If ESP32 booted after a power-path interruption, this is only
        // the time of first observation after reboot, not the outage onset.
        outageStartApprox = outageInterrupted;
        if (!savePendingOutage()) Serial.println("OUTAGE JOURNAL: Pending start save failed");
      } else if (!outagePendingExists) {
        // Existing v3.9-dev4 outage upgraded while already on battery.
        outageStartEpoch = 0;
        outageStartKnown = false;
        outageStartMillisValid = false;
        outageStartApprox = false;
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
      // Reconstruct the time of FIRST OBSERVATION after this boot if NTP
      // synchronized late. Never use millis() across a reboot.
      const uint32_t endEpoch = isTimeValid() ? (uint32_t)time(nullptr) : 0;
      if (!outageStartKnown && outageStartMillisValid && endEpoch) {
        const uint32_t elapsed = (uint32_t)(millis() - outageStartMillis) / 1000UL;
        if (endEpoch > elapsed) {
          outageStartEpoch = endEpoch - elapsed;
          outageStartKnown = true;
          outageStartApprox = true;
        }
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

// Persist the approximate first-observation time as soon as NTP syncs,
// even if power has not returned yet. Keeps the best evidence across a
// subsequent restart, without filling any unobserved downtime interval.
void resolvePendingOutageStartTime() {
  if (adapterState != AdapterState::Absent || !outagePendingExists ||
      outageStartKnown || !outageStartMillisValid || !isTimeValid()) return;
  const uint32_t nowEpoch = (uint32_t)time(nullptr);
  const uint32_t elapsed = (uint32_t)(millis() - outageStartMillis) / 1000UL;
  if (nowEpoch <= elapsed) return;
  outageStartEpoch = nowEpoch - elapsed;
  outageStartKnown = true;
  outageStartApprox = true;
  if (!savePendingOutage()) {
    outageStartEpoch = 0;
    outageStartKnown = false;
    outageStartApprox = false;
    Serial.println("OUTAGE JOURNAL: Approximate start save deferred");
  } else {
    Serial.println("OUTAGE JOURNAL: Approximate detection time saved");
  }
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
<title>ESP32-S3 UPS Monitor v3.9</title>
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
</style></head><body><div class="wrap"><header><h1>ESP32-S3 UPS Monitor</h1><p>Моніторинг резервного живлення · <span id="headerVersion">v3.9</span></p></header>
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
 <p class="muted">Тільки завершені події. ≈ — приблизний час першого виявлення відсутності адаптера, відновлений після NTP або за журналом подій. Це не обов’язково момент фактичного зникнення електрики. «Неповний час» означає перезапуск ESP32; тривалість враховує лише підтверджений час роботи.</p>
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
function formatOutageDate(sec){if(!(sec>0))return null;return new Date(sec*1000).toLocaleString('uk-UA',{timeZone:'Europe/Kyiv',day:'2-digit',month:'2-digit',year:'numeric',hour:'2-digit',minute:'2-digit',second:'2-digit'});}
function eventLabel(type){return type==='adapter_lost'?'Адаптер відключено — акумулятор':type==='adapter_restored'?'Адаптер підключено — мережа':type;}
function renderLastSwitch(){const last=events[0];$('lastSwitch').textContent=last&&last.time>0?'Останнє перемикання: '+(last.approx?'≈ ':'')+formatDate(last.time):'Останнє перемикання: —';}
async function updateEvents(){try{const r=await fetch('/api/events',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);const csv=await r.text();events=csv.split('\n').map(x=>x.trim()).filter(Boolean).map(x=>{const [time,type,flag]=x.split(',');return {time:Number(time),type,approx:flag==='approx'};}).reverse();renderLastSwitch();if(activeTab==='events'&&outageEntries.length)renderOutages();if(activeTab!=='events')return;const box=$('events');box.replaceChildren();if(!events.length){const tr=document.createElement('tr');const td=document.createElement('td');td.colSpan=2;td.textContent='Подій поки немає.';tr.appendChild(td);box.appendChild(tr);return;}events.forEach(e=>{const tr=document.createElement('tr'),td1=document.createElement('td'),td2=document.createElement('td');td1.textContent=e.time>0?(e.approx?'≈ ':'')+formatDate(e.time):'Немає збереженого часу';td2.textContent=eventLabel(e.type);td2.className=e.type==='adapter_lost'?'warn':e.type==='adapter_restored'?'ok':'';tr.append(td1,td2);box.appendChild(tr);});}catch(e){console.error(e);if(activeTab==='events')$('events').textContent='Не вдалося завантажити журнал.';}}
let outageEntries=[];let outageVisible=10;
// Older firmware saved a zero start_epoch after a reboot. Display a start
// ONLY when a single matching restore event is immediately preceded by an
// approximately timed loss event. CSV is never rewritten or fabricated.
function approximateLegacyStart(endEpoch){
 if(!(endEpoch>0))return null;
 // Reject ambiguous restores rather than guessing which outage this row means.
 const restoredIndexes=[];
 for(let i=0;i<events.length;i++){
  if(events[i].type==='adapter_restored'&&Math.abs(events[i].time-endEpoch)<=30)restoredIndexes.push(i);
 }
 if(restoredIndexes.length!==1)return null;
 const i=restoredIndexes[0];
 if(i+1>=events.length)return null;
 const lost=events[i+1]; // events is newest-first
 if(lost.type!=='adapter_lost'||!lost.approx||!(lost.time>0)||lost.time>endEpoch||endEpoch-lost.time>48*3600)return null;
 return lost.time;
}
function renderOutages(){
 const body=$('outagesRows'),status=$('outagesStatus');body.replaceChildren();
 $('outagesCountLabel').textContent='('+outageEntries.length+' записів)';
 status.textContent='Показано '+Math.min(outageVisible,outageEntries.length)+' із '+outageEntries.length+' завершених записів (максимум 200).';
 if(!outageEntries.length){const tr=document.createElement('tr'),td=document.createElement('td');td.colSpan=4;td.textContent='Завершених відключень у журналі поки немає.';tr.appendChild(td);body.appendChild(tr);}
 outageEntries.slice(0,outageVisible).forEach(line=>{
  const [a,b,d,flag]=line.split(','),tr=document.createElement('tr');
  const end=Number(b);let start=Number(a);
  let estimated=flag==='partial_reset_approx'||flag==='start_approx';
  let fromEvents=false;
  if(!(start>0)){
   const recovered=approximateLegacyStart(end);
   if(recovered!==null){start=recovered;estimated=true;fromEvents=true;}
  }
  let note=flag==='partial_reset'||flag==='partial_reset_approx'?'Неповний час (перезапуск)':flag==='time_unknown'?'Час NTP недоступний':'Завершено';
  if(fromEvents)note+='; початок ≈ за журналом подій';
  else if(estimated)note+='; початок приблизний';
  const cells=[start>0?(estimated?'≈ ':'')+formatOutageDate(start):'Час невідомий',formatOutageDate(end)||'Час невідомий',durationText(Number(d)),note];
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
$('outageCount').textContent=typeof d.outageCount==='number'?d.outageCount:'—';$('outageLast').textContent=durationText(d.outageLastSeconds);$('outageTotal').textContent=durationText(d.outageTotalSeconds);$('outageLongest').textContent=durationText(d.outageLongestSeconds);$('adapterWh').textContent=typeof d.adapterEnergyWh==='number'?d.adapterEnergyWh.toFixed(3)+' Wh':'—';$('batteryWh').textContent=typeof d.batteryEnergyWh==='number'?d.batteryEnergyWh.toFixed(3)+' Wh':'—';$('firmwareVersion').textContent=d.firmwareVersion||'—';$('headerVersion').textContent=d.firmwareVersion||'v3.9';$('inaState').textContent=d.inaReady?'OK':'Помилка';$('inaState').className='value '+(d.inaReady?'ok':'warn');$('wifi').textContent=d.wifiConnected?d.wifiRSSI+' dBm':'Відключено';$('uptime').textContent=Math.floor(d.uptimeSeconds/3600)+' год '+Math.floor(d.uptimeSeconds%3600/60)+' хв';$('resetReason').textContent=d.resetReason||'—';$('freeHeap').textContent=typeof d.freeHeap==='number'?(d.freeHeap/1024).toFixed(1)+' КБ':'—';$('littlefs').textContent=d.littlefs_mounted?((d.littlefs_used/1024).toFixed(0)+' / '+(d.littlefs_total/1024).toFixed(0)+' КБ'):'Помилка';$('historyRecords').textContent=typeof d.measurementRecords==='number'?d.measurementRecords+' / 2016':'—';$('ntp').textContent=d.ntpSynced?'Синхронізовано':'Очікування';$('ota').textContent=d.otaReady?'Готово':'Очікування';$('buildDateTime').textContent=d.buildDate&&d.buildTime?d.buildDate+' '+d.buildTime:'—';if(d.inaReady){add('v',d.busVoltage);add('i',d.currentMA);add('p',d.powerMW/1000);if(activeTab==='main')drawCharts();}
}catch(e){$('status').textContent='Втрачено зв’язок з ESP32';$('status').className='card notice danger';console.error(e);}}
update();updateEvents();setInterval(update,3000);setInterval(updateEvents,10000);
</script></body></html>
)HTML";

// =====================================================
// TELEGRAM TEST (v3.9-dev7-tg1)
// Boot test sent once after Wi-Fi + NTP; Ukrainian content.
// Boot connectivity notification; automatic alerts run in Telegram worker.
// =====================================================

// Telegram API certificate trust roots (previously tested on hardware).
// Verify TLS; DO NOT replace with setInsecure().
static const char TELEGRAM_ROOT_CA[] = R"PEM(
-----BEGIN CERTIFICATE-----
MIIDxTCCAq2gAwIBAgIBADANBgkqhkiG9w0BAQsFADCBgzELMAkGA1UEBhMCVVMx
EDAOBgNVBAgTB0FyaXpvbmExEzARBgNVBAcTClNjb3R0c2RhbGUxGjAYBgNVBAoT
EUdvRGFkZHkuY29tLCBJbmMuMTEwLwYDVQQDEyhHbyBEYWRkeSBSb290IENlcnRp
ZmljYXRlIEF1dGhvcml0eSAtIEcyMB4XDTA5MDkwMTAwMDAwMFoXDTM3MTIzMTIz
NTk1OVowgYMxCzAJBgNVBAYTAlVTMRAwDgYDVQQIEwdBcml6b25hMRMwEQYDVQQH
EwpTY290dHNkYWxlMRowGAYDVQQKExFHb0RhZGR5LmNvbSwgSW5jLjExMC8GA1UE
AxMoR28gRGFkZHkgUm9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkgLSBHMjCCASIw
DQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAL9xYgjx+lk09xvJGKP3gElY6SKD
E6bFIEMBO4Tx5oVJnyfq9oQbTqC023CYxzIBsQU+B07u9PpPL1kwIuerGVZr4oAH
/PMWdYA5UXvl+TW2dE6pjYIT5LY/qQOD+qK+ihVqf94Lw7YZFAXK6sOoBJQ7Rnwy
DfMAZiLIjWltNowRGLfTshxgtDj6AozO091GB94KPutdfMh8+7ArU6SSYmlRJQVh
GkSBjCypQ5Yj36w6gZoOKcUcqeldHraenjAKOc7xiID7S13MMuyFYkMlNAJWJwGR
tDtwKj9useiciAF9n9T521NtYJ2/LOdYq7hfRvzOxBsDPAnrSTFcaUaz4EcCAwEA
AaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAQYwHQYDVR0OBBYE
FDqahQcQZyi27/a9BUFuIMGU2g/eMA0GCSqGSIb3DQEBCwUAA4IBAQCZ21151fmX
WWcDYfF+OwYxdS2hII5PZYe096acvNjpL9DbWu7PdIxztDhC2gV7+AJ1uP2lsdeu
9tfeE8tTEH6KRtGX+rcuKxGrkLAngPnon1rpN5+r5N9ss4UXnT3ZJE95kTXWXwTr
gIOrmgIttRD02JDHBHNA7XIloKmf7J6raBKZV8aPEjoJpL1E/QYVN8Gb5DKj7Tjo
2GTzLH4U/ALqn83/B2gX2yKQOC16jdFU8WnjXzPKej17CuPKf1855eJ1usV2GDPO
LPAvTK33sefOT6jEm0pUBsV/fdUID+Ic/n4XuKxe9tQWskMJDE32p2u0mYRlynqI
4uJEvlz36hz1
-----END CERTIFICATE-----
)PEM";

bool telegramTestSent = false;
uint32_t telegramLastAttemptMs = 0;
uint8_t telegramAttempts = 0;
constexpr uint8_t TELEGRAM_MAX_TEST_ATTEMPTS = 3;
constexpr uint32_t TELEGRAM_TEST_INITIAL_DELAY_MS = 15000UL;
constexpr uint32_t TELEGRAM_TEST_RETRY_MS = 60000UL;

String telegramFormEncode(const String& value) {
  static const char* hexDigits = "0123456789ABCDEF";
  String encoded;
  encoded.reserve(value.length() * 3);
  for (size_t i = 0; i < value.length(); ++i) {
    const uint8_t c = (uint8_t)value[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' ||
        c == '.' || c == '~') {
      encoded += (char)c;
    } else if (c == ' ') {
      encoded += '+';
    } else {
      encoded += '%';
      encoded += hexDigits[c >> 4];
      encoded += hexDigits[c & 15];
    }
  }
  return encoded;
}

bool telegramSendMessage(const String& message) {
  if (WiFi.status() != WL_CONNECTED || !isTimeValid()) return false;

  WiFiClientSecure tls;
  tls.setCACert(TELEGRAM_ROOT_CA);
  HTTPClient http;
  http.setTimeout(4500); // Bounded request; production will use a separate worker.

  // Never print this URL: it contains the bot token.
  const String url = String("https://api.telegram.org/bot") +
                     TELEGRAM_BOT_TOKEN + "/sendMessage";
  if (!http.begin(tls, url)) {
    Serial.println("TELEGRAM: HTTPS begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  const String body = String("chat_id=") + telegramFormEncode(TELEGRAM_CHAT_ID) +
                      "&text=" + telegramFormEncode(message);
  const int httpStatus = http.POST(body);
  http.end();

  if (httpStatus == 200) {
    Serial.println("TELEGRAM: sendMessage HTTP 200");
    return true;
  }
  // Status only: do not leak token or full request URL to serial output.
  Serial.printf("TELEGRAM: send failed (HTTP=%d)\n", httpStatus);
  return false;
}

void telegramTestTick() {
  if (telegramTestSent || telegramAttempts >= TELEGRAM_MAX_TEST_ATTEMPTS)
    return;
  if (millis() < TELEGRAM_TEST_INITIAL_DELAY_MS ||
      WiFi.status() != WL_CONNECTED || !isTimeValid())
    return;
  if (telegramAttempts > 0 &&
      (uint32_t)(millis() - telegramLastAttemptMs) < TELEGRAM_TEST_RETRY_MS)
    return;

  telegramLastAttemptMs = millis();
  ++telegramAttempts;
  const String text = String("✅ ESP32 Smart UPS Monitor\n") +
      "Система підключена до Wi-Fi та Telegram.\n" +
      "🔧 Прошивка: " + String(FIRMWARE_VERSION) +
      "\nДоступні команди: /start, /status, /battery, /schedule";
  telegramTestSent = telegramSendMessage(text);
}


// =====================================================
// TELEGRAM v3.9-dev7-tg2
// Ukrainian commands; restrict access to TELEGRAM_CHAT_ID.
// Networking runs in a separate FreeRTOS task (not in loop()).
// Schedule fetched on demand and periodically for automatic notifications.
// =====================================================

// TLS trust roots for raw.githubusercontent.com; refreshed as needed.
// Never use setInsecure(). Runtime certificate changes may require an update.
static const char GITHUB_ROOT_CAS[] = R"CERTS(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIICPzCCAcWgAwIBAgIQBVVWvPJepDU1w6QP1atFcjAKBggqhkjOPQQDAzBhMQsw
CQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cu
ZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBHMzAe
Fw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVTMRUw
EwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5jb20x
IDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEczMHYwEAYHKoZIzj0CAQYF
K4EEACIDYgAE3afZu4q4C/sLfyHS8L6+c/MzXRq8NOrexpu80JX28MzQC7phW1FG
fp4tn+6OYwwX7Adw9c+ELkCDnOg/QW07rdOkFFk2eJ0DQ+4QE2xy3q6Ip6FrtUPO
Z9wj/wMco+I+o0IwQDAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAd
BgNVHQ4EFgQUs9tIpPmhxdiuNkHMEWNpYim8S8YwCgYIKoZIzj0EAwMDaAAwZQIx
AK288mw/EkrRLTnDCgmXc/SINoyIJ7vmiI1Qhadj+Z4y3maTD/HMsQmP3Wyr+mt/
oAIwOWZbwmSNuJ5Q3KjVSaLtx9zRSX8XAbjIho9OjIgrqJqpisXRAL34VOKa5Vt8
sycX
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIFkDCCA3igAwIBAgIQBZsbV56OITLiOQe9p3d1XDANBgkqhkiG9w0BAQwFADBi
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSEwHwYDVQQDExhEaWdpQ2VydCBUcnVzdGVkIFJvb3Qg
RzQwHhcNMTMwODAxMTIwMDAwWhcNMzgwMTE1MTIwMDAwWjBiMQswCQYDVQQGEwJV
UzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNlcnQu
Y29tMSEwHwYDVQQDExhEaWdpQ2VydCBUcnVzdGVkIFJvb3QgRzQwggIiMA0GCSqG
SIb3DQEBAQUAA4ICDwAwggIKAoICAQC/5pBzaN675F1KPDAiMGkz7MKnJS7JIT3y
ithZwuEppz1Yq3aaza57G4QNxDAf8xukOBbrVsaXbR2rsnnyyhHS5F/WBTxSD1If
xp4VpX6+n6lXFllVcq9ok3DCsrp1mWpzMpTREEQQLt+C8weE5nQ7bXHiLQwb7iDV
ySAdYyktzuxeTsiT+CFhmzTrBcZe7FsavOvJz82sNEBfsXpm7nfISKhmV1efVFiO
DCu3T6cw2Vbuyntd463JT17lNecxy9qTXtyOj4DatpGYQJB5w3jHtrHEtWoYOAMQ
jdjUN6QuBX2I9YI+EJFwq1WCQTLX2wRzKm6RAXwhTNS8rhsDdV14Ztk6MUSaM0C/
CNdaSaTC5qmgZ92kJ7yhTzm1EVgX9yRcRo9k98FpiHaYdj1ZXUJ2h4mXaXpI8OCi
EhtmmnTK3kse5w5jrubU75KSOp493ADkRSWJtppEGSt+wJS00mFt6zPZxd9LBADM
fRyVw4/3IbKyEbe7f/LVjHAsQWCqsWMYRJUadmJ+9oCw++hkpjPRiQfhvbfmQ6QY
uKZ3AeEPlAwhHbJUKSWJbOUOUlFHdL4mrLZBdd56rF+NP8m800ERElvlEFDrMcXK
chYiCd98THU/Y+whX8QgUWtvsauGi0/C1kVfnSD8oR7FwI+isX4KJpn15GkvmB0t
9dmpsh3lGwIDAQABo0IwQDAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIB
hjAdBgNVHQ4EFgQU7NfjgtJxXWRM3y5nP+e6mK4cD08wDQYJKoZIhvcNAQEMBQAD
ggIBALth2X2pbL4XxJEbw6GiAI3jZGgPVs93rnD5/ZpKmbnJeFwMDF/k5hQpVgs2
SV1EY+CtnJYYZhsjDT156W1r1lT40jzBQ0CuHVD1UvyQO7uYmWlrx8GnqGikJ9yd
+SeuMIW59mdNOj6PWTkiU0TryF0Dyu1Qen1iIQqAyHNm0aAFYF/opbSnr6j3bTWc
fFqK1qI4mfN4i/RN0iAL3gTujJtHgXINwBQy7zBZLq7gcfJW5GqXb5JQbZaNaHqa
sjYUegbyJLkJEVDXCLG4iXqEI2FCKeWjzaIgQdfRnGTZ6iahixTXTBmyUEFxPT9N
cCOGDErcgdLMMpSEDQgJlxxPwO5rIHQw0uA5NBCFIRUBCOhVMt5xSdkoF1BN5r5N
0XWs0Mr7QbhDparTwwVETyw2m+L64kW4I1NsBm9nVX9GtUw/bihaeSbSpKhil9Ie
4u1Ki7wb/UdKDd9nZn6yW0HQO+T0O/QEY+nvwlQAUaCKKsnOeMzV6ocEGLPOr0mI
r/OSmbaz5mEP0oUA51Aa5BuVnRmhuZyxm7EAHu/QD09CbMkKvO5D+jpxpchNJqU1
/YldvIViHTLSoCtU7ZpXwdv6EM8Zt4tKG48BtieVU+i2iW1bvGjUI+iLUaJW+fCm
gKDWHrO8Dw9TdSmq6hN35N6MgSGtBxBHEa2HPQfRdbzP82Z+
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIDXzCCAkegAwIBAgILBAAAAAABIVhTCKIwDQYJKoZIhvcNAQELBQAwTDEgMB4G
A1UECxMXR2xvYmFsU2lnbiBSb290IENBIC0gUjMxEzARBgNVBAoTCkdsb2JhbFNp
Z24xEzARBgNVBAMTCkdsb2JhbFNpZ24wHhcNMDkwMzE4MTAwMDAwWhcNMjkwMzE4
MTAwMDAwWjBMMSAwHgYDVQQLExdHbG9iYWxTaWduIFJvb3QgQ0EgLSBSMzETMBEG
A1UEChMKR2xvYmFsU2lnbjETMBEGA1UEAxMKR2xvYmFsU2lnbjCCASIwDQYJKoZI
hvcNAQEBBQADggEPADCCAQoCggEBAMwldpB5BngiFvXAg7aEyiie/QV2EcWtiHL8
RgJDx7KKnQRfJMsuS+FggkbhUqsMgUdwbN1k0ev1LKMPgj0MK66X17YUhhB5uzsT
gHeMCOFJ0mpiLx9e+pZo34knlTifBtc+ycsmWQ1z3rDI6SYOgxXG71uL0gRgykmm
KPZpO/bLyCiR5Z2KYVc3rHQU3HTgOu5yLy6c+9C7v/U9AOEGM+iCK65TpjoWc4zd
QQ4gOsC0p6Hpsk+QLjJg6VfLuQSSaGjlOCZgdbKfd/+RFO+uIEn8rUAVSNECMWEZ
XriX7613t2Saer9fwRPvm2L7DWzgVGkWqQPabumDk3F2xmmFghcCAwEAAaNCMEAw
DgYDVR0PAQH/BAQDAgEGMA8GA1UdEwEB/wQFMAMBAf8wHQYDVR0OBBYEFI/wS3+o
LkUkrk1Q+mOai97i3Ru8MA0GCSqGSIb3DQEBCwUAA4IBAQBLQNvAUKr+yAzv95ZU
RUm7lgAJQayzE4aGKAczymvmdLm6AC2upArT9fHxD4q/c2dKg8dEe3jgr25sbwMp
jjM5RcOO5LlXbKr8EpbsU8Yt5CRsuZRj+9xTaGdWPoO4zzUhw8lo/s7awlOqzJCK
6fBdRoyV3XpYKBovHd7NADdBj+1EbddTKJd+82cEHhXXipa0095MJ6RMG3NzdvQX
mcIfeg7jLQitChws/zyrVQ4PkX4268NXSb7hLi18YIvDQVETI53O9zJrlAGomecs
Mx86OyXShkDOOyyGeMlhLxS67ttVb9+E7gUJTb0o2HLO02JQZR7rkpeDMdmztcpH
WD9f
-----END CERTIFICATE-----
)CERTS";

constexpr uint32_t TG_POLL_INTERVAL_MS = 8000UL;
constexpr size_t TG_MAX_UPDATES_BYTES = 12000;
constexpr size_t TG_MAX_SCHEDULE_BYTES = 110000;
constexpr const char* OUTAGE_JSON_URL =
    "https://raw.githubusercontent.com/yaroslav2901/OE_OUTAGE_DATA/main/data/Poltavaoblenergo.json";
constexpr const char* OUTAGE_GROUP = "GPV1.2";

Preferences telegramPrefs;
bool telegramPrefsReady = false;
uint32_t telegramNextUpdate = 0;
bool telegramPollBootstrapped = false;
uint32_t telegramLastPollMs = 0;

String formatTelegramDuration(uint64_t ms) {
  const uint64_t minutes = ms / 60000ULL;
  return String((unsigned long)(minutes / 60ULL)) + " год " +
         String((unsigned long)(minutes % 60ULL)) + " хв";
}

String telegramStartText() {
  return String("🤖 Вітаю! Це ESP32 Smart UPS Monitor.\n\n") +
      "Зараз доступні команди:\n"
      "/status — стан UPS\n"
      "/battery — стан акумулятора та комірок\n"
      "/schedule — планові відключення, черга 1.2\n"
      "/help — список команд\n\n"
      "Дані про реальну наявність адаптера вимірює ESP32. "
      "Графік відключень отримується з окремого стороннього джерела.";
}

String telegramStatusText() {
  String result = "📊 СТАН UPS\n";
  result += "🔌 Вхід адаптера: ";
  result += adapterState == AdapterState::Present ? "напруга є\n" :
            adapterState == AdapterState::Absent ? "напруги немає\n" : "невідомо\n";
  result += "🔋 Акумулятор: ";
  if (batteryValid && cellCount > 0) {
    result += String(batteryV, 2) + " В (" + String(cellCount) + "S)\n";
  } else result += "показники не підтверджено\n";
  result += "⚡ Вхід DC+: ≈ " + String(adapterInputV, 2) + " В\n";
  if (inaOK) {
    result += "📏 Шина: " + String(busV, 2) + " В\n";
    result += "🔄 Струм: " + String(currentMA, 1) + " мА\n";
    result += "💡 Потужність: " + String(powerMW / 1000.0f, 2) + " Вт\n";
  } else result += "⚠️ INA226: дані недоступні\n";
  result += "📶 Wi-Fi: " + String(WiFi.RSSI()) + " dBm\n";
  result += "🕒 Безперервна робота: " +
            formatTelegramDuration((uint64_t)millis()) + "\n";
  result += "🔧 Прошивка: " + String(FIRMWARE_VERSION) + "\n";
  result += "ℹ️ GPIO10 фіксує напругу адаптера, а не стан реле YX850.";
  return result;
}

String telegramBatteryText() {
  String result = "🔋 АКУМУЛЯТОР\n";
  if (!batteryValid || cellCount <= 0) {
    return result + "⚠️ Немає достовірних показників балансувальних каналів. "
                    "Перевір підключення акумулятора.";
  }
  result += "Збірка: " + String(cellCount) + "S\n";
  result += "Напруга: " + String(batteryV, 3) + " В\n";
  float minV = cells[0], maxV = cells[0];
  for (int i = 0; i < cellCount && i < 6; ++i) {
    result += "Комірка " + String(i + 1) + ": " +
              String(cells[i], 3) + " В\n";
    if (cells[i] < minV) minV = cells[i];
    if (cells[i] > maxV) maxV = cells[i];
  }
  result += "⚖️ Розбаланс: " + String((maxV - minV) * 1000.0f, 0) + " мВ\n";
  if (minV < 3.30f) result += "⚠️ Низька напруга однієї з комірок!\n";
  if (maxV > 4.20f) result += "⚠️ Напруга комірки перевищує 4,20 В!\n";
  if ((maxV - minV) > 0.10f) result += "⚠️ Розбаланс понад 100 мВ!\n";
  result += "ℹ️ Це вимірювання напруги, не точний залишок ємності (Ah).";
  return result;
}

// One half-hour is 0 = scheduled power, 1 = planned outage,
// 2 = possible outage. Unknown values invalidate the day.
int8_t decodeScheduleHour(const String& value, uint8_t& first, uint8_t& second) {
  if (value == "yes") { first = 0; second = 0; }
  else if (value == "no") { first = 1; second = 1; }
  else if (value == "first") { first = 1; second = 0; }
  else if (value == "second") { first = 0; second = 1; }
  else if (value == "mfirst") { first = 2; second = 0; }
  else if (value == "msecond") { first = 0; second = 2; }
  else return -1;
  return 0;
}

String formatHalfHour(int index) {
  const int minute = index * 30;
  const int hours = minute / 60;
  const int mins = minute % 60;
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%02d:%02d", hours, mins);
  return String(buffer);
}

String telegramScheduleText() {
  if (WiFi.status() != WL_CONNECTED || !isTimeValid()) {
    return "⚠️ Графік поки недоступний: немає Wi-Fi або синхронізації часу NTP.";
  }
  WiFiClientSecure tls;
  tls.setCACert(GITHUB_ROOT_CAS);
  HTTPClient http;
  http.setTimeout(4500);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(tls, OUTAGE_JSON_URL))
    return "⚠️ Не вдалося встановити захищене з’єднання з джерелом графіка.";
  http.addHeader("Accept", "application/json");
  const int status = http.GET();
  const int size = http.getSize();
  if (status != 200 || size > (int)TG_MAX_SCHEDULE_BYTES) {
    http.end();
    Serial.printf("TELEGRAM: schedule HTTP=%d bytes=%d\n", status, size);
    return "⚠️ Графік зараз недоступний. Спробуй пізніше.";
  }
  const String payload = http.getString();
  http.end();
  if (payload.length() < 50 || payload.length() > TG_MAX_SCHEDULE_BYTES)
    return "⚠️ Джерело повернуло некоректний обсяг даних графіка.";

  JsonDocument json;
  if (deserializeJson(json, payload))
    return "⚠️ Не вдалося прочитати JSON графіка.";
  const uint32_t todayEpoch = json["fact"]["today"] | 0UL;
  if (!todayEpoch) return "⚠️ Джерело не вказало дату графіка.";

  const time_t nowEpoch = time(nullptr);
  const time_t dateEpoch = (time_t)todayEpoch;
  tm nowLocal = {}, dayLocal = {};
  localtime_r(&nowEpoch, &nowLocal);
  localtime_r(&dateEpoch, &dayLocal);
  if (nowLocal.tm_year != dayLocal.tm_year || nowLocal.tm_yday != dayLocal.tm_yday) {
    return "⚠️ Свіжого графіка на сьогодні немає. Застарілі дані не показуємо.";
  }
  const String dayKey = String(todayEpoch);
  JsonVariantConst group = json["fact"]["data"][dayKey][OUTAGE_GROUP];
  if (group.isNull()) return "⚠️ Не знайдено графіка для черги 1.2 на сьогодні.";
  uint8_t halfHours[48] = {};
  for (int h = 1; h <= 24; ++h) {
    const String hourKey = String(h);
    const char* type = group[hourKey] | "";
    if (decodeScheduleHour(String(type), halfHours[(h-1)*2], halfHours[(h-1)*2+1]) < 0)
      return "⚠️ Невідомий формат графіка. Дані не інтерпретовано.";
  }
  char dateText[48];
  snprintf(dateText, sizeof(dateText), "%02d.%02d.%04d", dayLocal.tm_mday,
           dayLocal.tm_mon + 1, dayLocal.tm_year + 1900);
  String result = String("📅 ГРАФІК НА ") + dateText + "\n";
  result += "📍 Полтавська область, черга 1.2\n\n";
  bool any = false;
  for (uint8_t kind = 1; kind <= 2; ++kind) {
    String section;
    for (int i = 0; i < 48;) {
      if (halfHours[i] != kind) { ++i; continue; }
      const int start = i;
      while (i < 48 && halfHours[i] == kind) ++i;
      section += "• " + formatHalfHour(start) + "–" + formatHalfHour(i) + "\n";
    }
    if (section.length()) {
      any = true;
      result += kind == 1 ? "🔴 Планові відключення:\n" :
                            "🟡 Можливі відключення:\n";
      result += section + "\n";
    }
  }
  if (!any) result += "✅ На сьогодні відключень у графіку немає.\n\n";
  const char* updated = json["fact"]["update"] | "";
  if (*updated) result += String("Оновлено у джерелі: ") + updated + "\n";
  result += "ℹ️ Графік орієнтовний. Фактичну наявність напруги визначає ESP32.\n";
  result += "Джерело: OE_OUTAGE_DATA (неофіційне).";
  return result;
}

void telegramHandleCommand(const String& body) {
  String command = body;
  command.trim();
  const int space = command.indexOf(' ');
  if (space >= 0) command = command.substring(0, space);
  const int at = command.indexOf('@');
  if (at >= 0) command = command.substring(0, at);
  command.toLowerCase();
  String answer;
  if (command == "/start" || command == "/help") answer = telegramStartText();
  else if (command == "/status") answer = telegramStatusText();
  else if (command == "/battery") answer = telegramBatteryText();
  else if (command == "/schedule") answer = telegramScheduleText();
  else answer = "Не розпізнав команду. Введи /help для списку доступних команд.";
  telegramSendMessage(answer);
}

void telegramPollTick() {
  if (WiFi.status() != WL_CONNECTED || !isTimeValid()) return;
  if ((uint32_t)(millis() - telegramLastPollMs) < TG_POLL_INTERVAL_MS) return;
  telegramLastPollMs = millis();
  WiFiClientSecure tls;
  tls.setCACert(TELEGRAM_ROOT_CA);
  HTTPClient http;
  http.setTimeout(4000);
  // Initial bootstrap discards old commands left from tg1 or PC tests.
  // Subsequent updates use an NVS-persisted offset to avoid replying twice.
  String url = String("https://api.telegram.org/bot") +
               TELEGRAM_BOT_TOKEN + "/getUpdates?timeout=0&limit=4&offset=";
  if (!telegramPollBootstrapped && telegramNextUpdate == 0) url += "-1";
  else url += String((unsigned long)telegramNextUpdate);
  if (!http.begin(tls, url)) return;
  const int code = http.GET();
  const int bytes = http.getSize();
  if (code != 200 || bytes > (int)TG_MAX_UPDATES_BYTES) {
    Serial.printf("TELEGRAM: getUpdates HTTP=%d bytes=%d\n", code, bytes);
    http.end();
    return;
  }
  const String payload = http.getString();
  http.end();
  if (payload.length() > TG_MAX_UPDATES_BYTES) return;
  JsonDocument json;
  if (deserializeJson(json, payload) || !(json["ok"] | false)) return;
  JsonArrayConst updates = json["result"].as<JsonArrayConst>();
  for (JsonVariantConst update : updates) {
    const uint32_t updateId = update["update_id"] | 0UL;
    if (updateId + 1UL <= telegramNextUpdate) continue;
    const uint32_t nextId = updateId + 1UL;
    telegramNextUpdate = nextId;
    if (telegramPrefsReady) telegramPrefs.putUInt("offset", nextId);
    if (!telegramPollBootstrapped) continue; // old messages at first boot
    JsonVariantConst message = update["message"];
    if (message.isNull()) continue;
    const int64_t chat = message["chat"]["id"] | (int64_t)0;
    char chatBuffer[32];
    snprintf(chatBuffer, sizeof(chatBuffer), "%lld", (long long)chat);
    if (String(chatBuffer) != String(TELEGRAM_CHAT_ID)) continue;
    const char* command = message["text"] | "";
    if (command[0]) telegramHandleCommand(String(command));
  }
  telegramPollBootstrapped = true;
}


// =====================================================
// AUTOMATIC TELEGRAM ALERTS — v3.9-dev7-tg3
// Runs exclusively inside telegramWorker on core 0.
//  * GPIO10: actual presence/absence of adapter INPUT voltage, not relay position.
//  * Schedule: UNOFFICIAL GPV1.2 JSON; must match the current local date.
//  * NVS queue: survives Wi-Fi interruption and ESP32 resets (max 8 notices).
//  * Delivery: at least once, so a reset just after Telegram accepts a notice
//    can rarely result in a duplicate. NVS holds no bot token.
// =====================================================

constexpr uint32_t TG_AUTO_SCHEDULE_INTERVAL_MS = 15UL * 60UL * 1000UL;
constexpr uint32_t TG_AUTO_SCHEDULE_RETRY_MS = 90UL * 1000UL;
constexpr uint32_t TG_AUTO_QUEUE_SEND_INTERVAL_MS = 5000UL;
constexpr size_t TG_AUTO_MAX_QUEUE = 8;
String tgQueued[TG_AUTO_MAX_QUEUE];
uint32_t tgQueueExpires[TG_AUTO_MAX_QUEUE] = {};
size_t tgQueueCount = 0;
uint32_t tgLastSendMs = 0;
uint32_t tgLastScheduleAttemptMs = 0;
bool tgScheduleAttempted = false;
bool tgScheduleLastFetchOk = false;
uint32_t tgScheduleFetchMs = 0;
uint32_t tgCurrentScheduleDay = 0;
uint8_t tgCurrentScheduleHalfHours[48] = {};
uint64_t tgWarned60 = 0;
uint64_t tgWarned15 = 0;
uint32_t tgWarnedDay = 0;
uint8_t tgKnownAdapter = 0;  // 0 unknown, 1 DC+ present, 2 DC+ absent
bool tgWasWifiConnected = false;
uint32_t tgWifiDownSince = 0;
// Random identifier for this boot. A monotonic timestamp cannot be
// reconstructed across resets unless the clock was already synchronized.
uint32_t tgBootNonce = 0;

bool tgPersistQueue() {
  if (!telegramPrefsReady) return false;
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (size_t i = 0; i < tgQueueCount; ++i) {
    JsonObject item = arr.add<JsonObject>();
    item["msg"] = tgQueued[i];
    item["expires"] = tgQueueExpires[i];
  }
  String payload;
  serializeJson(doc, payload);
  return telegramPrefs.putString("pending", payload) == payload.length();
}

void tgLoadQueue() {
  if (!telegramPrefsReady) return;
  String raw = telegramPrefs.getString("pending", "");
  if (!raw.length()) return;
  JsonDocument doc;
  if (deserializeJson(doc, raw)) {
    Serial.println("TELEGRAM: pending queue data invalid");
    return;
  }
  JsonArrayConst arr = doc.as<JsonArrayConst>();
  for (JsonVariantConst item : arr) {
    if (tgQueueCount >= TG_AUTO_MAX_QUEUE) break;
    const char* msg = item["msg"] | "";
    const uint32_t expires = item["expires"] | 0UL;
    if (msg[0] && strlen(msg) < 650) {
      tgQueued[tgQueueCount] = msg;
      tgQueueExpires[tgQueueCount] = expires;
      ++tgQueueCount;
    }
  }
  Serial.printf("TELEGRAM: restored pending messages: %u\n", (unsigned)tgQueueCount);
}

void tgRemoveOldestQueueMessage() {
  if (!tgQueueCount) return;
  for (size_t i = 1; i < tgQueueCount; ++i) {
    tgQueued[i-1] = tgQueued[i];
    tgQueueExpires[i-1] = tgQueueExpires[i];
  }
  --tgQueueCount;
  tgQueued[tgQueueCount] = String();
  tgQueueExpires[tgQueueCount] = 0;
}

void tgPruneExpiredQueueMessages() {
  if (!isTimeValid()) return;
  const uint32_t now = (uint32_t)time(nullptr);
  bool dirty = false;
  for (size_t i = 0; i < tgQueueCount;) {
    if (tgQueueExpires[i] && now >= tgQueueExpires[i]) {
      for (size_t j = i + 1; j < tgQueueCount; ++j) {
        tgQueued[j-1] = tgQueued[j];
        tgQueueExpires[j-1] = tgQueueExpires[j];
      }
      --tgQueueCount;
      tgQueued[tgQueueCount] = String();
      tgQueueExpires[tgQueueCount] = 0;
      dirty = true;
    } else ++i;
  }
  if (dirty && !tgPersistQueue())
    Serial.println("TELEGRAM: unable to persist expired alert cleanup");
}

// Only schedule reminders expire; real GPIO10 events stay queued until sent.
bool tgQueueMessage(const String& message, uint32_t expires = 0) {
  tgPruneExpiredQueueMessages();
  if (!message.length() || message.length() >= 650 ||
      tgQueueCount >= TG_AUTO_MAX_QUEUE) {
    Serial.println("TELEGRAM: alert queue is full or invalid");
    return false;
  }
  tgQueued[tgQueueCount] = message;
  tgQueueExpires[tgQueueCount] = expires;
  ++tgQueueCount;
  if (tgPersistQueue()) return true;
  --tgQueueCount;
  tgQueued[tgQueueCount] = String();
  tgQueueExpires[tgQueueCount] = 0;
  Serial.println("TELEGRAM: could not persist pending message");
  return false;
}

String tgFormatLocalEpoch(time_t epoch) {
  struct tm local = {};
  localtime_r(&epoch, &local);
  char text[32];
  if (!strftime(text, sizeof(text), "%d.%m.%Y %H:%M:%S", &local))
    return "час недоступний (форматування)";
  return String(text);
}

String tgLocalTimeStamp() {
  if (!isTimeValid()) return "час недоступний (NTP)";
  return tgFormatLocalEpoch(time(nullptr));
}

// While NTP is unavailable, save both the boot identifier and millis() in
// the persistent message. The time can be reconstructed when NTP syncs in
// this SAME boot; the original loss may have happened before the reboot.
String tgDeferredAlertTime() {
  if (isTimeValid()) return tgLocalTimeStamp();
  return String("[[UPS_TIME:") + String(tgBootNonce) + ":" +
         String(millis()) + "]]";
}

// Render before delivery, not when queued: Telegram HTTPS is attempted only
// after time sync. If another reboot occurred meanwhile, do not invent a date.
String tgResolveQueuedAlertTime(const String& message) {
  const String tag = "[[UPS_TIME:";
  const int begin = message.indexOf(tag);
  if (begin < 0) return message;
  const int end = message.indexOf("]]", begin + tag.length());
  if (end < 0) return message;
  const String encoded = message.substring(begin + tag.length(), end);
  const int colon = encoded.indexOf(':');
  if (colon < 1 || colon == (int)encoded.length() - 1) return message;
  const uint32_t boot = (uint32_t)strtoul(encoded.substring(0, colon).c_str(), nullptr, 10);
  const uint32_t atMs = (uint32_t)strtoul(encoded.substring(colon + 1).c_str(), nullptr, 10);
  String replacement;
  const uint32_t nowMs = millis();
  if (boot == tgBootNonce && atMs <= nowMs && isTimeValid()) {
    const time_t nowEpoch = time(nullptr);
    const time_t detected = nowEpoch - (time_t)((nowMs - atMs) / 1000UL);
    const time_t started = nowEpoch - (time_t)(nowMs / 1000UL);
    replacement = String("≈ ") + tgFormatLocalEpoch(detected) +
                  " (після запуску ESP32)\n" +
                  "🚀 Запуск ESP32: ≈ " + tgFormatLocalEpoch(started);
  } else {
    const uint32_t seconds = atMs / 1000UL;
    replacement = String("через ") + String(seconds / 3600UL) + " год " +
                  String((seconds / 60UL) % 60UL) + " хв " +
                  String(seconds % 60UL) + " с після запуску ESP32" +
                  " (після ще одного перезапуску календарний час невідомий)";
  }
  String rendered = message;
  rendered.replace(message.substring(begin, end + 2), replacement);
  return rendered;
}

// Detect confirmed GPIO10 state changes even if network is offline.
// The LAST observed state is persisted only AFTER the alert has been queued.
void tgAdapterAlertTick() {
  const AdapterState seen = adapterState;
  if (seen == AdapterState::Unknown) return;
  const uint8_t state = seen == AdapterState::Present ? 1 : 2;
  if (tgKnownAdapter == 0) {
    // New installation: baseline the state without inventing an outage event.
    if (telegramPrefsReady && telegramPrefs.putUChar("lastSrc", state) == 1)
      tgKnownAdapter = state;
    return;
  }
  if (state == tgKnownAdapter) return;
  String message;
  if (state == 2) {
    message = String("⚡ ЗНИКЛА НАПРУГА АДАПТЕРА\n") +
      "🕒 Виявлено: " + tgDeferredAlertTime() + "\n" +
      "🔋 Акумулятор: " + (batteryValid ? String(batteryV, 2) + " В" : String("невідомо")) + "\n" +
      "💡 Навантаження: " + (inaOK ? String(powerMW / 1000.0f, 2) + " Вт" : String("невідомо")) + "\n" +
      "ℹ️ Перехід визначено за GPIO10. Під час перемикання ESP32 могла перезавантажитися.";
  } else {
    message = String("✅ НАПРУГУ АДАПТЕРА ВІДНОВЛЕНО\n") +
      "🕒 Виявлено: " + tgDeferredAlertTime() + "\n" +
      "⏱ Підтверджена тривалість: " + formatTelegramDuration(outageLastMs) + "\n" +
      "ℹ️ Час під час перезапусків ESP32 не враховано.";
  }
  if (tgQueueMessage(message)) {
    // If NVS save fails we can requeue on a later pass; no silent loss.
    if (telegramPrefsReady && telegramPrefs.putUChar("lastSrc", state) == 1)
      tgKnownAdapter = state;
  }
}

// Observe Wi-Fi outages separately from GPIO10 power events.
void tgWifiRecoveryTick() {
  const bool online = WiFi.status() == WL_CONNECTED;
  if (!online) {
    if (tgWasWifiConnected) tgWifiDownSince = millis();
    tgWasWifiConnected = false;
    return;
  }
  if (!tgWasWifiConnected && tgWifiDownSince != 0 &&
      (uint32_t)(millis() - tgWifiDownSince) >= 60000UL) {
    tgQueueMessage(String("📶 Wi-Fi знову підключений\n🕒 ") + tgLocalTimeStamp() +
                   "\n📨 Накопичені повідомлення буде доставлено після відновлення доступу до Telegram.");
  }
  tgWasWifiConnected = true;
  tgWifiDownSince = 0;
}

struct TgScheduleSnapshot {
  uint32_t day = 0;
  uint8_t halfHours[48] = {};
  String update;
};

// Dedicated periodic fetch; /schedule keeps its already-tested request path.
bool tgFetchSchedule(TgScheduleSnapshot& out) {
  if (WiFi.status() != WL_CONNECTED || !isTimeValid()) return false;
  WiFiClientSecure tls;
  tls.setCACert(GITHUB_ROOT_CAS);
  HTTPClient http;
  http.setTimeout(4500);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(tls, OUTAGE_JSON_URL)) return false;
  http.addHeader("Accept", "application/json");
  const int status = http.GET();
  const int size = http.getSize();
  if (status != 200 || size > (int)TG_MAX_SCHEDULE_BYTES) {
    http.end();
    return false;
  }
  const String payload = http.getString();
  http.end();
  if (payload.length() < 50 || payload.length() > TG_MAX_SCHEDULE_BYTES) return false;
  JsonDocument json;
  if (deserializeJson(json, payload)) return false;
  const uint32_t day = json["fact"]["today"] | 0UL;
  if (day == 0) return false;
  const time_t nowEpoch = time(nullptr);
  const time_t dayEpoch = (time_t)day;
  struct tm nowLocal = {}, scheduleLocal = {};
  localtime_r(&nowEpoch, &nowLocal);
  localtime_r(&dayEpoch, &scheduleLocal);
  if (nowLocal.tm_year != scheduleLocal.tm_year ||
      nowLocal.tm_yday != scheduleLocal.tm_yday) return false;
  const String dayKey = String(day);
  JsonVariantConst group = json["fact"]["data"][dayKey][OUTAGE_GROUP];
  if (group.isNull()) return false;
  out.day = day;
  for (int hour = 1; hour <= 24; ++hour) {
    const char* raw = group[String(hour)] | "";
    if (decodeScheduleHour(String(raw), out.halfHours[(hour-1)*2],
                           out.halfHours[(hour-1)*2+1]) < 0) return false;
  }
  out.update = String(json["fact"]["update"] | "");
  return true;
}

String tgScheduleSignature(const TgScheduleSnapshot& schedule) {
  String out;
  out.reserve(48);
  for (int i = 0; i < 48; ++i)
    out += char('0' + schedule.halfHours[i]);
  return out;
}

String tgScheduleSections(const uint8_t* values) {
  String out;
  for (uint8_t kind = 1; kind <= 2; ++kind) {
    String lines;
    for (int i = 0; i < 48;) {
      if (values[i] != kind) { ++i; continue; }
      const int begin = i;
      while (i < 48 && values[i] == kind) ++i;
      lines += String("• ") + formatHalfHour(begin) + "–" + formatHalfHour(i) + "\n";
    }
    if (lines.length()) {
      out += (kind == 1 ? "🔴 Планові:\n" : "🟡 Можливі:\n");
      out += lines;
    }
  }
  return out.length() ? out : String("✅ Відключень не заплановано.\n");
}

// Parse once every 15 minutes. A new day is a baseline, not a changed schedule.
void tgScheduleRefreshTick() {
  if (WiFi.status() != WL_CONNECTED || !isTimeValid()) return;
  const uint32_t now = millis();
  const uint32_t interval = tgScheduleLastFetchOk ?
      TG_AUTO_SCHEDULE_INTERVAL_MS : TG_AUTO_SCHEDULE_RETRY_MS;
  if (tgScheduleAttempted && (uint32_t)(now - tgLastScheduleAttemptMs) < interval)
    return;
  tgScheduleAttempted = true;
  tgLastScheduleAttemptMs = now;
  TgScheduleSnapshot schedule;
  if (!tgFetchSchedule(schedule)) {
    tgScheduleLastFetchOk = false;
    Serial.println("TELEGRAM: scheduled feed unavailable/stale");
    return;
  }
  tgScheduleLastFetchOk = true;
  tgScheduleFetchMs = now;
  tgCurrentScheduleDay = schedule.day;
  memcpy(tgCurrentScheduleHalfHours, schedule.halfHours, sizeof(tgCurrentScheduleHalfHours));

  if (!telegramPrefsReady) return;
  const String newSig = tgScheduleSignature(schedule);
  const uint32_t prevDay = telegramPrefs.getUInt("schedDay", 0);
  const String oldSig = telegramPrefs.getString("schedSig", "");
  if (prevDay == schedule.day && oldSig.length() == 48 && oldSig != newSig) {
    const String msg = String("🔄 ЗМІНЕНО ГРАФІК ВІДКЛЮЧЕНЬ\n") +
      "📍 Полтавська область, черга 1.2\n" +
      "🕒 Дані джерела: " + schedule.update + "\n" +
      tgScheduleSections(schedule.halfHours) +
      "ℹ️ Джерело неофіційне. /schedule — актуальний графік.";
    if (!tgQueueMessage(msg, schedule.day + 86400UL)) return;  // Retry changed schedule later.
  }
  if (prevDay != schedule.day) {
    tgWarnedDay = schedule.day;
    tgWarned60 = 0;
    tgWarned15 = 0;
    telegramPrefs.putULong64("warn60", 0);
    telegramPrefs.putULong64("warn15", 0);
    telegramPrefs.putUInt("warnDay", schedule.day);
  }
  // Persist the baseline/signature to suppress duplicates after a reset.
  telegramPrefs.putUInt("schedDay", schedule.day);
  telegramPrefs.putString("schedSig", newSig);
}

void tgScheduleWarningTick() {
  if (!tgScheduleLastFetchOk || !tgCurrentScheduleDay ||
      (uint32_t)(millis() - tgScheduleFetchMs) > TG_AUTO_SCHEDULE_INTERVAL_MS + 60000UL)
    return;  // Do not warn using stale or unverified data.
  if (tgWarnedDay != tgCurrentScheduleDay) return;
  const int64_t nowEpoch = (int64_t)time(nullptr);
  for (uint8_t i = 0; i < 48; ++i) {
    const uint8_t kind = tgCurrentScheduleHalfHours[i];
    if (kind == 0 || (i > 0 && tgCurrentScheduleHalfHours[i-1] == kind)) continue;
    const int64_t remaining = (int64_t)tgCurrentScheduleDay + (int64_t)i * 1800LL - nowEpoch;
    if (remaining <= 0 || remaining > 3600LL) continue;
    const bool near = remaining <= 900LL;
    const uint64_t bit = (1ULL << i);
    uint64_t& sent = near ? tgWarned15 : tgWarned60;
    if (sent & bit) continue;
    const String text = String(kind == 1 ? "⏰ НАБЛИЖАЄТЬСЯ ПЛАНОВЕ ВІДКЛЮЧЕННЯ\n" :
                                        "⏰ МОЖЛИВЕ ВІДКЛЮЧЕННЯ\n") +
      "📍 Черга 1.2\n" +
      "🕒 Початок: " + formatHalfHour(i) + "\n" +
      "⏳ Залишилося: " + String((unsigned long)((remaining + 59LL) / 60LL)) + " хв\n" +
      "🔌 Адаптер: " + (adapterState == AdapterState::Present ? "напруга присутня\n" :
                            adapterState == AdapterState::Absent ? "напруги немає\n" : "невідомо\n") +
      "ℹ️ Прогноз неофіційний, фактичний стан вимірює GPIO10.";
    if (tgQueueMessage(text, tgCurrentScheduleDay + (uint32_t)i * 1800UL)) {
      sent |= bit;
      if (telegramPrefsReady) telegramPrefs.putULong64(near ? "warn15" : "warn60", sent);
    }
  }
}

void tgDeliverQueueTick() {
  tgPruneExpiredQueueMessages();
  if (!tgQueueCount || WiFi.status() != WL_CONNECTED || !isTimeValid()) return;
  if (tgLastSendMs && (uint32_t)(millis() - tgLastSendMs) < TG_AUTO_QUEUE_SEND_INTERVAL_MS)
    return;
  tgLastSendMs = millis();
  if (!telegramSendMessage(tgResolveQueuedAlertTime(tgQueued[0]))) return;
  tgRemoveOldestQueueMessage();
  if (!tgPersistQueue()) Serial.println("TELEGRAM: pending queue persist after delivery failed");
}

void tgAutoSetup() {
  tgBootNonce = esp_random();
  if (!tgBootNonce) tgBootNonce = 1;
  tgLoadQueue();
  if (telegramPrefsReady) {
    tgKnownAdapter = telegramPrefs.getUChar("lastSrc", 0);
    tgWarnedDay = telegramPrefs.getUInt("warnDay", 0);
    tgWarned60 = telegramPrefs.getULong64("warn60", 0ULL);
    tgWarned15 = telegramPrefs.getULong64("warn15", 0ULL);
  }
  tgWasWifiConnected = WiFi.status() == WL_CONNECTED;
  Serial.println("TELEGRAM: automated alerts ready");
}

void tgAutoTick() {
  tgAdapterAlertTick();
  tgWifiRecoveryTick();
  tgScheduleRefreshTick();
  tgScheduleWarningTick();
  tgDeliverQueueTick();
}

void telegramWorker(void* unused) {
  telegramPrefsReady = telegramPrefs.begin("ups-tg", false);
  if (telegramPrefsReady) telegramNextUpdate = telegramPrefs.getUInt("offset", 0);
  telegramPollBootstrapped = telegramNextUpdate > 0;
  tgAutoSetup();
  Serial.println("TELEGRAM: worker started");
  while (true) {
    // No test-message spam on reboots: only meaningful automatic notices.
    tgAutoTick();
    telegramPollTick();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

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
  xTaskCreatePinnedToCore(telegramWorker, "ups-tg", 16384, nullptr, 1, nullptr, 0);
  Serial.println("Setup completed");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  maintainWiFi();
  // Telegram commands and HTTPS run in a separate FreeRTOS task.

  if (WiFi.status() == WL_CONNECTED && otaStarted) {
    ArduinoOTA.handle();
  }

  server.handleClient();
  resolvePendingEventTime();
  resolvePendingOutageStartTime();

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
