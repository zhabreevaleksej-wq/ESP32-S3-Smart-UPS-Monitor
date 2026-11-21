
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Wire.h>
#include <INA226.h>
#include <LittleFS.h>
#include <math.h>
#include <esp_system.h>

// =====================================================
// ESP32-S3 SMART UPS MONITOR
// PlatformIO / Arduino
// ADC calibration + Wi-Fi reconnect + OTA
// System Diagnostics
// =====================================================

static const char* const FIRMWARE_VERSION = "v3.7";
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
AdapterState adapterState = AdapterState::Unknown;
AdapterState pendingAdapterState = AdapterState::Unknown;
uint8_t pendingAdapterSamples = 0;
float adapterSenseV = 0.0f;
float adapterInputV = 0.0f; // Approximate: divider resistor/ADC tolerance


// Timers
uint32_t lastSample = 0;
uint32_t lastWifiAttempt = 0;

// Network state
bool wifiWasConnected = false;
bool otaStarted = false;

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
    adapterState = observed;
    pendingAdapterState = AdapterState::Unknown;
    pendingAdapterSamples = 0;
    Serial.println(adapterState == AdapterState::Present
      ? "Adapter input: PRESENT" : "Adapter input: ABSENT");
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
<!doctype html>
<html lang="uk">
<head>
<meta charset="utf-8">
<meta name="viewport"
      content="width=device-width,initial-scale=1">

<title>ESP32-S3 UPS Monitor</title>

<style>
:root {
  color-scheme: dark;
  font-family: system-ui, Arial, sans-serif;
}

* {
  box-sizing: border-box;
}

body {
  margin: 0;
  background: #0b1220;
  color: #e8f0ff;
}

header {
  background: #14223b;
  border-bottom: 1px solid #30425e;
  padding: 21px;
  text-align: center;
}

h1 {
  font-size: 23px;
  margin: 0;
}

header p {
  color: #9bb3d6;
  margin: 6px 0 0;
}

.wrap {
  max-width: 1100px;
  margin: auto;
  padding: 18px;
}

.status {
  text-align: center;
  padding: 13px;
  font-weight: 700;
}

.ok { color: #4ade80; }
.warn { color: #fbbf24; }
.danger { color: #fb7185; }

.grid {
  display: grid;
  grid-template-columns:
    repeat(auto-fit, minmax(190px, 1fr));
  gap: 12px;
}

.card {
  background: #182940;
  border: 1px solid #304765;
  border-radius: 13px;
  padding: 17px;
}

.label {
  font-size: 13px;
  color: #a6bfdf;
}

.value {
  font-size: 26px;
  font-weight: 750;
  margin-top: 9px;
}

h2 {
  font-size: 18px;
  margin: 26px 0 12px;
}

.cellrow {
  padding: 12px 0;
  border-bottom: 1px solid #354962;
}

.cellrow:last-child {
  border: 0;
}

.celltop {
  display: flex;
  justify-content: space-between;
  gap: 12px;
}

.track {
  height: 9px;
  border-radius: 8px;
  background: #33455d;
  margin-top: 9px;
  overflow: hidden;
}

.fill {
  height: 100%;
  background: #35c98c;
  border-radius: 8px;
}

.fill.warnfill {
  background: #fbbf24;
}

.muted {
  color: #9bb3d6;
  font-size: 13px;
}

.charts {
  display: grid;
  grid-template-columns:
    repeat(auto-fit, minmax(290px, 1fr));
  gap: 12px;
}

.chart {
  width: 100%;
  height: 155px;
  display: block;
}

.foot {
  font-size: 12px;
  color: #93a9c8;
  margin: 22px 0;
}
</style>
</head>

<body>

<header>
  <h1>ESP32-S3 UPS Monitor</h1>
  <p>Моніторинг резервного живлення · <span id="headerVersion">—</span></p>
</header>

<main class="wrap">

<div id="status" class="status warn">
  Очікування вимірювань...
</div>

<div class="grid">

  <div class="card">
    <div class="label">Джерело живлення</div>
    <div id="powerSource" class="value">Визначення...</div>
    <div id="adapterVoltage" class="muted">Вхід DC+: —</div>
    <div class="muted">За наявністю напруги адаптера, не за положенням реле YX850.</div>
  </div>

  <div class="card">
    <div class="label">Напруга акумулятора</div>
    <div id="battery" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Струм навантаження</div>
    <div id="current" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Потужність</div>
    <div id="power" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Кількість комірок</div>
    <div id="count" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Орієнтовний заряд</div>
    <div id="soc" class="value">—</div>
    <div class="muted">
      Оцінка за напругою, не точний SoC
    </div>
  </div>

  <div class="card">
    <div class="label">Шина INA226</div>
    <div id="bus" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Розбаланс комірок</div>
    <div id="imbalance" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Wi-Fi сигнал</div>
    <div id="wifi" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Час роботи ESP32</div>
    <div id="uptime" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Причина перезапуску</div>
    <div id="resetReason" class="value">—</div>
  </div>

    <div class="card">
    <div class="label">Вільна RAM</div>
    <div id="freeHeap" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Версія прошивки</div>
    <div id="firmwareVersion" class="value">—</div>
  </div>

  <div class="card">
    <div class="label">Дата компіляції</div>
    <div id="buildDateTime" class="value">—</div>
  </div>

</div>


<h2>Попередження</h2>
<div id="alerts" class="card muted">
  Очікування даних...
</div>

<h2>Напруга комірок</h2>
<div id="cells" class="card">—</div>

<h2>Балансувальні канали B1–B6</h2>
<div id="taps" class="card">—</div>

<h2>Графіки (останні 60 вимірювань)</h2>

<div class="charts">

  <div class="card">
    <div class="label">Напруга шини, В</div>
    <canvas class="chart" id="chartV"></canvas>
  </div>

  <div class="card">
    <div class="label">Струм, мА</div>
    <canvas class="chart" id="chartI"></canvas>
  </div>

  <div class="card">
    <div class="label">Потужність, Вт</div>
    <canvas class="chart" id="chartP"></canvas>
  </div>

</div>

<p class="foot">
  Графіки зберігаються лише в браузері.
  Без BMS і апаратного захисту вебмоніторинг
  не запобігає перерозряду акумулятора.
</p>

</main>

<script>
const $ = id => document.getElementById(id);

const history = {
  v: [],
  i: [],
  p: []
};

function line(canvas, values) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;

  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);

  const c = canvas.getContext('2d');

  c.scale(dpr, dpr);
  c.clearRect(0, 0, w, h);

  c.strokeStyle = '#38516f';
  c.lineWidth = 1;

  for (let j = 1; j < 4; j++) {
    let y = h * j / 4;
    c.beginPath();
    c.moveTo(8, y);
    c.lineTo(w - 8, y);
    c.stroke();
  }

  if (!values.length) return;

  let lo = Math.min(...values);
  let hi = Math.max(...values);

  if (hi - lo < 0.1) {
    lo -= 0.05;
    hi += 0.05;
  }

  c.strokeStyle = '#55b5ff';
  c.lineWidth = 2;
  c.beginPath();

  values.forEach((v, j) => {
    let x = 9 + j * (w - 18) /
      Math.max(1, values.length - 1);

    let y = h - 10 -
      (v - lo) / (hi - lo) * (h - 20);

    if (!j) c.moveTo(x, y);
    else c.lineTo(x, y);
  });

  c.stroke();

  c.fillStyle = '#aec4df';
  c.font = '12px system-ui';

  c.fillText(hi.toFixed(2), 10, 13);
  c.fillText(lo.toFixed(2), 10, h - 3);
}

function add(k, v) {
  if (!Number.isFinite(v)) return;

  history[k].push(v);

  if (history[k].length > 60) {
    history[k].shift();
  }
}

function approxSOC(v) {
  const points = [
    [3.0, 0],
    [3.3, 5],
    [3.5, 10],
    [3.6, 20],
    [3.7, 35],
    [3.8, 50],
    [3.9, 65],
    [4.0, 80],
    [4.1, 92],
    [4.2, 100]
  ];

  if (v <= 3) return 0;
  if (v >= 4.2) return 100;

  for (let j = 1; j < points.length; j++) {
    if (v <= points[j][0]) {
      let a = points[j - 1];
      let b = points[j];

      return Math.round(
        a[1] +
        (v - a[0]) *
        (b[1] - a[1]) /
        (b[0] - a[0])
      );
    }
  }

  return 100;
}

async function update() {
  try {
    const r = await fetch('/api/data', {
      cache: 'no-store'
    });

    if (!r.ok) {
      throw Error('HTTP ' + r.status);
    }

    const d = await r.json();

    const sourceLabel = d.powerSource === 'adapter'
      ? 'Адаптер 12 В'
      : d.powerSource === 'battery'
        ? 'Акумулятор'
        : 'Визначення...';
    $('powerSource').textContent = sourceLabel;
    $('adapterVoltage').textContent =
      typeof d.adapterInputVoltage === 'number'
        ? 'Вхід DC+: ≈ ' + d.adapterInputVoltage.toFixed(2) + ' В'
        : 'Вхід DC+: —';

    $('status').textContent =
      d.batteryValid
        ? 'Батарея: попередньо визначена'
        : 'Конфігурацію батареї не підтверджено';

    $('status').className =
      'status ' + (d.batteryValid ? 'ok' : 'warn');

    $('battery').textContent =
      d.batteryValid
        ? d.batteryVoltage.toFixed(2) + ' В'
        : '—';

    $('current').textContent =
      d.inaReady
        ? d.currentMA.toFixed(1) + ' мА'
        : '—';

    $('power').textContent =
      d.inaReady
        ? (d.powerMW / 1000).toFixed(2) + ' Вт'
        : '—';

    $('bus').textContent =
      d.inaReady
        ? d.busVoltage.toFixed(2) + ' В'
        : '—';

    $('count').textContent =
      d.batteryValid
        ? d.detectedCells + 'S'
        : 'Невідомо';

    $('soc').textContent =
      d.batteryValid
        ? approxSOC(
            d.batteryVoltage / d.detectedCells
          ) + ' %'
        : '—';

    $('wifi').textContent =
      d.wifiConnected
        ? d.wifiRSSI + ' dBm'
        : '—';

    const hours = Math.floor(
      d.uptimeSeconds / 3600
    );

    const minutes = Math.floor(
      (d.uptimeSeconds % 3600) / 60
    );

     $('uptime').textContent =
      hours + ' год ' + minutes + ' хв';

    $('resetReason').textContent =
      d.resetReason || '—';

          // Free heap memory
    $('freeHeap').textContent =
      typeof d.freeHeap === 'number'
        ? (d.freeHeap / 1024).toFixed(1) + ' КБ'
        : '—';

    // Firmware version
    $('firmwareVersion').textContent =
      d.firmwareVersion || '—';

    $('headerVersion').textContent =
      d.firmwareVersion || '—';

    // Build date and time
    $('buildDateTime').textContent =
      d.buildDate && d.buildTime
        ? d.buildDate + ' ' + d.buildTime
        : '—';

    const cellBox = $('cells');
    cellBox.replaceChildren();

    const alertBox = $('alerts');
    alertBox.replaceChildren();

    if (d.batteryValid) {
      const vs = d.cells.slice(0, d.detectedCells);

      const imbalance =
        Math.max(...vs) - Math.min(...vs);

      $('imbalance').textContent =
        (imbalance * 1000).toFixed(0) + ' мВ';

      vs.forEach((v, i) => {
        const row = document.createElement('div');
        row.className = 'cellrow';

        const top = document.createElement('div');
        top.className = 'celltop';

        const name = document.createElement('span');
        name.textContent = 'Комірка ' + (i + 1);

        const value = document.createElement('strong');
        value.textContent = v.toFixed(3) + ' В';

        top.append(name, value);
        row.appendChild(top);

        const track = document.createElement('div');
        track.className = 'track';

        const fill = document.createElement('div');
        fill.className =
          'fill' +
          (v < 3.3 || v > 4.2 ? ' warnfill' : '');

        fill.style.width =
          Math.max(0, Math.min(100, v / 4.2 * 100)) + '%';

        track.appendChild(fill);
        row.appendChild(track);
        cellBox.appendChild(row);
      });

      const warnings = [];

      if (vs.some(v => v < 3.3)) {
        warnings.push(
          'Низька напруга однієї або кількох комірок.'
        );
      }

      if (vs.some(v => v > 4.2)) {
        warnings.push(
          'Напруга комірки перевищує 4,20 В.'
        );
      }

      if (imbalance > 0.10) {
        warnings.push(
          'Різниця між комірками понад 100 мВ.'
        );
      }

      alertBox.textContent =
        warnings.length
          ? warnings.join(' ')
          : 'Критичних відхилень за заданими порогами не виявлено.';

    } else {
      $('imbalance').textContent = '—';

      cellBox.textContent =
        'Немає підтверджених даних про комірки.';

      alertBox.textContent =
        'Перевір балансувальні канали.';
    }

    const tapBox = $('taps');
    tapBox.replaceChildren();

    d.taps.forEach((v, i) => {
      const row = document.createElement('div');
      row.className = 'cellrow';

      row.textContent =
        'B' + (i + 1) +
        ': ' + v.toFixed(3) + ' В';

      tapBox.appendChild(row);
    });

    if (d.inaReady) {
      add('v', d.busVoltage);
      add('i', d.currentMA);
      add('p', d.powerMW / 1000);

      line($('chartV'), history.v);
      line($('chartI'), history.i);
      line($('chartP'), history.p);
    }

  } catch (e) {
    $('status').textContent =
      'Втрачено зв’язок з ESP32';

    $('status').className = 'status danger';

    console.error(e);
  }
}

update();
setInterval(update, 3000);
</script>
</body>
</html>
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

  if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = millis();
    sampleSensors();
  }

  delay(5);
}

// =====================================================
// END
// =====================================================
