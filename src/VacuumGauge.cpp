//
// VacuumGauge.cpp
//
// Vacuum pressure gauge for Waveshare ESP32-S3-Touch-AMOLED-1.64
// (CO5300 QSPI AMOLED, 280x456).
//
// Sensor:   Posifa I2C @ 0x50, SDA=GPIO1, SCL=GPIO2
// Display:  Arduino_GFX + Arduino_Canvas (offscreen framebuffer flushed per frame)
// Storage:  Preferences (NVS), namespace "vacgauge"
// WiFi:     Captive-portal AP on first boot; STA + HTTP REST API thereafter
// Commands: USB Serial at 115200 via GAACE_Core commandProcessor
//

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Arduino_GFX_Library.h>

#include <commandProcessor.h>
#include <debug.h>
#include <RingBuffer.h>
#include <charAllocate.h>

#include "VacuumGauge.h"
#include "build_info.h"

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------

// QSPI display (verified against GFX library Arduino_GFX_dev_device.h for this board)
static const int8_t kQSPI_CS    = 9;
static const int8_t kQSPI_SCLK  = 10;
static const int8_t kQSPI_SDIO0 = 11;
static const int8_t kQSPI_SDIO1 = 12;
static const int8_t kQSPI_SDIO2 = 13;
static const int8_t kQSPI_SDIO3 = 14;
static const int8_t kQSPI_RST   = 21;

static const int16_t kScreenW = 456;
static const int16_t kScreenH = 280;

// I2C sensor
static const int  kI2C_SDA = 1;
static const int  kI2C_SCL = 2;
static const long kI2C_Hz  = 100000;

// Boot button (active LOW)
static const uint8_t       kBootPin         = 0;
static const unsigned long kBootLongPressMs = 3000;

// ---------------------------------------------------------------------------
// Configuration constants
// ---------------------------------------------------------------------------

static const int   kSensorBytes      = 6;
static const int   kSensorAddr       = 0x50;

static const float kTorrThreshold    = 10.0f;
static const float kTrendKnee        = 50000.0f;
static const float kMicronsPerCount  = 45.7f;
static const float kMinPressure      = 0.0f;

static const int   kMaxOffsetTorr    = 700;
static const int   kMaxOffsetmTorr   = 10000;

#define CAL_MAX_POINTS 32
static const int kCalSnapCounts = 100;
static const int kCalAvgSamples = 8;

static const unsigned long kReadIntervalMs = 500;
static const unsigned long kSaveIntervalMs = 10000;

const char *Version = "Vacuum Gauge, version " FIRMWARE_VERSION "_" BUILD_TIMESTAMP;

// Captive portal AP
static const char      *kApSSID = "VacuumGauge-Setup";
static const IPAddress  kApIP(192, 168, 4, 1);

// RGB565 display colors (avoid relying on Arduino_GFX macro exports)
static const uint16_t C_BLACK  = 0x0000u;
static const uint16_t C_WHITE  = 0xFFFFu;
static const uint16_t C_YELLOW = 0xFFE0u;
static const uint16_t C_CYAN   = 0x07FFu;
static const uint16_t C_GREEN  = 0x07E0u;
static const uint16_t C_LGRAY  = 0x7BEFu;

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

typedef struct
{
    int   raw;
    float torr;
} CalPoint;

typedef struct
{
    int16_t  Size;
    char     Name[20];
    int8_t   Rev;
    int      TWIadd;
    double   calPress;
    int      rawData;
    int      rawTemp;
    int      offsetTorr;
    int      offsetmTorr;
    int      useCalTable;
    char     ssid[33];
    char     pass[65];
    int      calCount;
    CalPoint calTable[CAL_MAX_POINTS];
} Data;

Data data =
{
    sizeof(Data), "Press", 1,
    kSensorAddr,
    0, 0, 0,
    0, 0,
    0,
    "", "",
    0
};

Data lastSaved;

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------

enum AppState { STATE_PORTAL, STATE_CONNECTING, STATE_RUNNING };
static AppState appState = STATE_PORTAL;

// ---------------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------------

static Arduino_DataBus *bus =
    new Arduino_ESP32QSPI(kQSPI_CS, kQSPI_SCLK, kQSPI_SDIO0,
                          kQSPI_SDIO1, kQSPI_SDIO2, kQSPI_SDIO3);
// Constructor: (bus, rst, rotation, w, h, col_offset1, row_offset1, col_offset2, row_offset2)
// Offsets from GFX library board reference — required to map the physical pixel array correctly.
// CO5300 has no hardware transpose — keep it in portrait (its only native mode).
// Software landscape is done via canvas rotation=1, which remaps draw calls into
// the portrait framebuffer so flush() sends correctly-ordered pixels to the panel.
static Arduino_GFX    *display = new Arduino_CO5300(
    bus, kQSPI_RST, 0, 280, 456, 20, 0, 180, 24);
static Arduino_Canvas *canvas  = new Arduino_Canvas(280, 456, display, 0, 0, 1);

static AsyncWebServer server(80);
static DNSServer      dnsServer;

commandProcessor cp;
debug            dbg(&cp);

static SemaphoreHandle_t dataMutex;   // guards calPress, rawData, rawTemp
static SemaphoreHandle_t wireMutex;   // guards Wire I2C bus

static unsigned long bootPressStart  = 0;
static bool          bootWasPressed  = false;
static unsigned long lastDisplayMs   = 0;
static unsigned long lastSaveMs      = 0;
static unsigned long connectStartMs  = 0;
static volatile bool restartPending  = false;

// Body accumulator for POST /save (file-scope so lambdas can reference it)
static char   s_saveBody[512];
static size_t s_saveBodyLen = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

static int  sampleRawAveraged(int samples);
static bool addCalPoint(int raw, float torr);
static void loadDefaultCalTable(void);
static bool settingsChanged(void);
static void setupServer(void);
extern Command cmds[]; 

// ---------------------------------------------------------------------------
// Persistence (NVS via Preferences)
// ---------------------------------------------------------------------------

bool saveData()
{
    Preferences p;
    if (!p.begin("vacgauge", false)) return false;
    p.putInt("toff",    data.offsetTorr);
    p.putInt("mtoff",   data.offsetmTorr);
    p.putInt("calmode", data.useCalTable);
    p.putString("name", data.Name);
    p.putString("ssid", data.ssid);
    p.putString("pass", data.pass);
    p.putInt("calcnt",  data.calCount);
    for (int i = 0; i < data.calCount; i++)
    {
        char k[6];
        snprintf(k, sizeof(k), "cr%d", i);
        p.putInt(k, data.calTable[i].raw);
        snprintf(k, sizeof(k), "ct%d", i);
        p.putFloat(k, data.calTable[i].torr);
    }
    p.end();
    return true;
}

bool loadData()
{
    Preferences p;
    if (!p.begin("vacgauge", true)) return false;
    if (!p.isKey("toff")) { p.end(); return false; }

    data.offsetTorr  = p.getInt("toff",    0);
    data.offsetmTorr = p.getInt("mtoff",   0);
    data.useCalTable = p.getInt("calmode", 0);

    String nm = p.getString("name", "Press");
    strncpy(data.Name, nm.c_str(), sizeof(data.Name) - 1);
    data.Name[sizeof(data.Name) - 1] = '\0';

    String ss = p.getString("ssid", "");
    strncpy(data.ssid, ss.c_str(), sizeof(data.ssid) - 1);
    data.ssid[sizeof(data.ssid) - 1] = '\0';

    String pw = p.getString("pass", "");
    strncpy(data.pass, pw.c_str(), sizeof(data.pass) - 1);
    data.pass[sizeof(data.pass) - 1] = '\0';

    data.calCount = p.getInt("calcnt", 0);
    if (data.calCount > CAL_MAX_POINTS) data.calCount = CAL_MAX_POINTS;
    for (int i = 0; i < data.calCount; i++)
    {
        char k[6];
        snprintf(k, sizeof(k), "cr%d", i);
        data.calTable[i].raw  = p.getInt(k, 0);
        snprintf(k, sizeof(k), "ct%d", i);
        data.calTable[i].torr = p.getFloat(k, 0.0f);
    }
    p.end();
    return true;
}

static void clearCredentials()
{
    Preferences p;
    if (p.begin("vacgauge", false))
    {
        p.remove("ssid");
        p.remove("pass");
        p.end();
    }
    data.ssid[0] = '\0';
    data.pass[0] = '\0';
}

// ---------------------------------------------------------------------------
// Calibration table
// ---------------------------------------------------------------------------

static const CalPoint defaultCalTable[] =
{
    {18836, 760.00f}, {21012, 28.90f}, {21916, 18.70f},
    {24386,   8.57f}, {25868,  5.94f}, {27390,  4.14f},
    {28354,   3.39f}, {29279,  2.77f}, {30015,  2.38f},
    {30556,   2.08f}, {30777,  1.97f}, {31484,  1.70f},
};
static const int defaultCalCount = sizeof(defaultCalTable) / sizeof(defaultCalTable[0]);

static void loadDefaultCalTable()
{
    data.calCount = defaultCalCount;
    for (int i = 0; i < defaultCalCount; i++) data.calTable[i] = defaultCalTable[i];
}

static void sortCalTable()
{
    for (int i = 1; i < data.calCount; i++)
    {
        CalPoint key = data.calTable[i];
        int j = i - 1;
        while (j >= 0 && data.calTable[j].raw > key.raw)
        {
            data.calTable[j + 1] = data.calTable[j];
            j--;
        }
        data.calTable[j + 1] = key;
    }
}

static bool addCalPoint(int raw, float torr)
{
    for (int i = 0; i < data.calCount; i++)
    {
        if (abs(data.calTable[i].raw - raw) <= kCalSnapCounts)
        {
            data.calTable[i].raw  = raw;
            data.calTable[i].torr = torr;
            sortCalTable();
            return true;
        }
    }
    if (data.calCount >= CAL_MAX_POINTS) return false;
    data.calTable[data.calCount].raw  = raw;
    data.calTable[data.calCount].torr = torr;
    data.calCount++;
    sortCalTable();
    return true;
}

static float pwlPressure(int raw)
{
    if (data.calCount <= 0) return 0.0f;
    if (data.calCount == 1) return data.calTable[0].torr;

    int i = 1;
    while (i < data.calCount - 1 && raw >= data.calTable[i].raw) i++;

    float r0 = (float)data.calTable[i - 1].raw,  r1 = (float)data.calTable[i].raw;
    float p0 = data.calTable[i - 1].torr,         p1 = data.calTable[i].torr;
    if (r1 == r0) return p0;
    return p0 + (p1 - p0) * ((float)(raw - r0) / (r1 - r0));
}

// ---------------------------------------------------------------------------
// Sensor reading
// ---------------------------------------------------------------------------

static bool readSensorWord(int &word, int &temp)
{
    Wire.requestFrom(data.TWIadd, kSensorBytes);
    delay(10);
    word = 0; temp = 0;
    int i = 0;
    while (Wire.available() > 0)
    {
        uint8_t c = Wire.read();
        if (i == 1) word |= c << 8;
        if (i == 2) word |= c;
        if (i == 4) temp |= c << 8;
        if (i == 5) temp |= c;
        i++;
    }
    return i == kSensorBytes;
}

static int sampleRawAveraged(int samples)
{
    if (samples < 1) samples = 1;
    long sum = 0;
    int  raw, tmp;
    xSemaphoreTake(wireMutex, portMAX_DELAY);
    for (int i = 0; i < samples; i++)
    {
        Wire.beginTransmission(data.TWIadd);
        Wire.write(0xD0);
        Wire.endTransmission(true);
        readSensorWord(raw, tmp);
        sum += raw;
        delay(10);
    }
    xSemaphoreGive(wireMutex);
    return (int)(sum / samples);
}

static void sensorTask(void *param)
{
    for (;;)
    {
        int press = 0, rawP = 0, rawT = 0, scratch = 0;

        xSemaphoreTake(wireMutex, portMAX_DELAY);

        Wire.beginTransmission(data.TWIadd);
        Wire.endTransmission(true);
        readSensorWord(press, scratch);

        Wire.beginTransmission(data.TWIadd);
        Wire.write(0xD0);
        Wire.endTransmission(true);
        readSensorWord(rawP, rawT);

        xSemaphoreGive(wireMutex);

        // Compute pressure using local copies of cal params (no mutex needed for
        // calTable/offsets — only written from the main task at infrequent intervals)
        float fval;
        if (data.useCalTable)
        {
            fval = pwlPressure(rawP);
        }
        else
        {
            fval = (float)press;
            if (fval > kTrendKnee)
                fval = kTrendKnee + (fval - kTrendKnee) * kMicronsPerCount;
            fval /= 1000.0f;
        }

        if (fval > kTorrThreshold) fval += (float)data.offsetTorr;
        else                       fval += (float)data.offsetmTorr / 1000.0f;

        if (fval < kMinPressure) fval = kMinPressure;

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        data.rawData  = rawP;
        data.rawTemp  = rawT;
        data.calPress = (double)fval;
        xSemaphoreGive(dataMutex);

        vTaskDelay(pdMS_TO_TICKS(kReadIntervalMs));
    }
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

static void centerText(const char *str, int16_t y, uint8_t sz)
{
    canvas->setTextSize(sz);
    int16_t  x1, y1;
    uint16_t w, h;
    canvas->getTextBounds(str, 0, 0, &x1, &y1, &w, &h);
    canvas->setCursor((kScreenW - (int16_t)w) / 2 - x1, y);
    canvas->print(str);
}

static void updateDisplay()
{
    canvas->fillScreen(C_BLACK);
    canvas->setTextWrap(false);

    switch (appState)
    {
    case STATE_PORTAL:
        canvas->setTextColor(C_YELLOW);
        centerText("VacuumGauge", kScreenH / 2 - 80, 3);
        centerText("Setup", kScreenH / 2 - 40, 3);
        canvas->setTextColor(C_WHITE);
        centerText("Connect to WiFi:", kScreenH / 2 + 10, 2);
        canvas->setTextColor(C_CYAN);
        centerText(kApSSID, kScreenH / 2 + 40, 2);
        canvas->setTextColor(C_WHITE);
        centerText("then browse to", kScreenH / 2 + 72, 1);
        centerText("192.168.4.1", kScreenH / 2 + 90, 2);
        break;

    case STATE_CONNECTING:
        canvas->setTextColor(C_YELLOW);
        centerText("Connecting...", kScreenH / 2 - 30, 3);
        canvas->setTextColor(C_WHITE);
        centerText(data.ssid, kScreenH / 2 + 20, 2);
        break;

    case STATE_RUNNING:
    {
        // Device name — top left
        canvas->setTextColor(C_GREEN);
        canvas->setTextSize(2);
        canvas->setCursor(4, 4);
        canvas->print(data.Name);

        // Snapshot live readings
        float p;
        int   raw;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        p   = (float)data.calPress;
        raw = data.rawData;
        xSemaphoreGive(dataMutex);

        // Pressure number and unit
        char        pStr[16];
        const char *unit;
        if (p < 1.0f)
        {
            snprintf(pStr, sizeof(pStr), "%.0f", p * 1000.0f);
            unit = "mTorr";
        }
        else if (p < 100.0f)
        {
            snprintf(pStr, sizeof(pStr), "%.1f", p);
            unit = "Torr";
        }
        else
        {
            snprintf(pStr, sizeof(pStr), "%.0f", p);
            unit = "Torr";
        }

        canvas->setTextColor(C_WHITE);
        centerText(pStr, kScreenH / 2 - 56, 6);   // 48px tall at size 6

        canvas->setTextColor(C_CYAN);
        centerText(unit, kScreenH / 2 + 16, 4);    // 32px tall at size 4

        // IP + cal mode — bottom
        canvas->setTextColor(C_LGRAY);
        canvas->setTextSize(2);

        String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "---";
        String ipLine = "IP: " + ip;
        String calLine = String("Cal: ") + (data.useCalTable ? "Table" : "Factory");

        canvas->setCursor(4, kScreenH - 40);
        canvas->print(ipLine);
        canvas->setCursor(4, kScreenH - 20);
        canvas->print(calLine);
        break;
    }
    }

    canvas->flush();
}

// ---------------------------------------------------------------------------
// WiFi helpers
// ---------------------------------------------------------------------------

static void startAP()
{
    dnsServer.stop();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(kApIP, kApIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(kApSSID);
    dnsServer.start(53, "*", kApIP);
    WiFi.scanNetworks(true);  // async scan; results available via GET /scan
    appState = STATE_PORTAL;
}

static void startSTA()
{
    dnsServer.stop();
    WiFi.mode(WIFI_STA);
    WiFi.begin(data.ssid, data.pass);
    connectStartMs = millis();
    appState = STATE_CONNECTING;
}

// ---------------------------------------------------------------------------
// Captive portal HTML
// ---------------------------------------------------------------------------

static const char kSetupHtml[] = R"rawhtml(<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VacuumGauge Setup</title>
<style>
body{font-family:sans-serif;max-width:440px;margin:0 auto;padding:20px 16px 48px;background:#111;color:#eee}
h1{color:#4af;margin-bottom:4px}
.step{color:#aaa;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin:22px 0 8px}
select{width:100%;padding:12px;box-sizing:border-box;border-radius:6px;
  border:1px solid #444;background:#222;color:#eee;font-size:16px}
input[type=password]{width:100%;padding:12px;box-sizing:border-box;border-radius:6px;
  border:1px solid #444;background:#222;color:#eee;font-size:16px;margin-top:2px}
.net{display:flex;align-items:center;padding:12px 14px;margin-bottom:6px;border-radius:8px;
  background:#1c1c1c;border:2px solid transparent;cursor:pointer;gap:10px}
.net:hover{border-color:#334}
.net.sel{border-color:#4af;background:#0d1d2d}
.ssid-txt{flex:1;font-size:15px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.bars{font-size:13px;color:#4af;min-width:28px}
.lock{font-size:13px;opacity:.55}
#scan-msg{color:#888;font-size:13px;padding:6px 0}
#pass-wrap{display:none;margin-top:2px}
button{display:block;width:100%;padding:14px;margin-top:26px;background:#4af;color:#000;
  border:none;border-radius:8px;font-size:17px;font-weight:bold;cursor:pointer}
button:hover{background:#6cf}
#msg{text-align:center;margin-top:14px;min-height:18px;font-size:14px;color:#4f4}
</style>
</head>
<body>
<h1>VacuumGauge Setup</h1>

<div class="step">Step 1 — Gauge identity</div>
<select id="gname">
  <option value="Gauge A">Gauge A</option>
  <option value="Gauge B">Gauge B</option>
  <option value="Gauge C">Gauge C</option>
  <option value="Gauge D">Gauge D</option>
  <option value="Gauge E">Gauge E</option>
  <option value="Gauge F">Gauge F</option>
  <option value="Gauge G">Gauge G</option>
  <option value="Gauge H">Gauge H</option>
</select>

<div class="step">Step 2 — Select WiFi network</div>
<div id="scan-msg">&#8635; Scanning for networks&hellip;</div>
<div id="netlist"></div>
<input type="hidden" id="ssid">

<div id="pass-wrap">
  <div class="step">Step 3 — WiFi password</div>
  <input id="pass" type="password" placeholder="Leave blank if open network">
</div>

<button onclick="doSave()">Save &amp; Connect</button>
<p id="msg"></p>

<script>
function bars(r){
  if(r>=-55)return'&#9608;&#9608;&#9608;&#9608;';
  if(r>=-65)return'&#9608;&#9608;&#9608;&thinsp;';
  if(r>=-75)return'&#9608;&#9608;&thinsp;&thinsp;';
  return'&#9608;&thinsp;&thinsp;&thinsp;';
}
function esc(s){var d=document.createElement('div');d.appendChild(document.createTextNode(s));return d.innerHTML;}
var chosen='';
function pick(el,ssid){
  document.querySelectorAll('.net').forEach(function(n){n.classList.remove('sel');});
  el.classList.add('sel');
  chosen=ssid;
  document.getElementById('ssid').value=ssid;
  document.getElementById('pass-wrap').style.display='block';
  document.getElementById('pass').focus();
}
function render(nets){
  var seen={},list=[];
  nets.forEach(function(n){if(!seen[n.ssid]||n.rssi>seen[n.ssid].rssi)seen[n.ssid]=n;});
  for(var k in seen)list.push(seen[k]);
  list.sort(function(a,b){return b.rssi-a.rssi;});
  var d=document.getElementById('netlist');
  d.innerHTML='';
  if(!list.length){
    d.innerHTML='<div id="scan-msg">No networks found. <a href="#" onclick="doScan(true);return false">Retry</a></div>';
    return;
  }
  list.forEach(function(n){
    var el=document.createElement('div');
    el.className='net';
    el.innerHTML='<span class="bars">'+bars(n.rssi)+'</span>'+
      '<span class="ssid-txt">'+esc(n.ssid)+'</span>'+
      (n.enc?'<span class="lock">&#128274;</span>':'');
    el.onclick=function(){pick(el,n.ssid);};
    d.appendChild(el);
  });
}
function poll(){
  fetch('/scan').then(function(r){return r.json();}).then(function(j){
    if(j.status==='scanning'){setTimeout(poll,2000);}
    else{document.getElementById('scan-msg').style.display='none';render(j);}
  }).catch(function(){setTimeout(poll,3000);});
}
function doScan(refresh){
  document.getElementById('scan-msg').innerHTML='&#8635; Scanning&hellip;';
  document.getElementById('scan-msg').style.display='';
  document.getElementById('netlist').innerHTML='';
  if(refresh){fetch('/scan?refresh=1').then(function(){setTimeout(poll,2500);});}
  else{poll();}
}
doScan(false);
function doSave(){
  if(!chosen){
    document.getElementById('msg').style.color='#fa4';
    document.getElementById('msg').innerText='Please select a WiFi network first.';
    return;
  }
  var payload={ssid:chosen,pass:document.getElementById('pass').value,
               name:document.getElementById('gname').value};
  document.getElementById('msg').style.color='#4f4';
  document.getElementById('msg').innerText='Saving…';
  fetch('/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)})
  .then(function(r){return r.text();})
  .then(function(){document.getElementById('msg').innerText='Saved! Device restarting…';})
  .catch(function(){
    document.getElementById('msg').style.color='#f84';
    document.getElementById('msg').innerText='Error — please try again.';
  });
}
</script>
</body>
</html>)rawhtml";

// ---------------------------------------------------------------------------
// HTTP server
// ---------------------------------------------------------------------------

static void setupServer()
{
    // Serve setup page (AP mode)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        request->send(200, "text/html", kSetupHtml);
    });

    // WiFi scan results for the portal network picker
    server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        bool doRefresh = request->hasParam("refresh");
        int16_t n = WiFi.scanComplete();

        if (doRefresh || n == WIFI_SCAN_FAILED)
        {
            if (n != WIFI_SCAN_RUNNING) WiFi.scanNetworks(true);
            request->send(202, "application/json", "{\"status\":\"scanning\"}");
            return;
        }
        if (n == WIFI_SCAN_RUNNING)
        {
            request->send(202, "application/json", "{\"status\":\"scanning\"}");
            return;
        }

        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (int16_t i = 0; i < n && i < 20; i++)
        {
            JsonObject obj = arr.add<JsonObject>();
            obj["ssid"] = WiFi.SSID(i);
            obj["rssi"] = WiFi.RSSI(i);
            obj["enc"]  = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        }
        WiFi.scanDelete();
        String body;
        serializeJson(doc, body);
        request->send(200, "application/json", body);
    });

    // Receive credentials via POST /save
    server.on("/save", HTTP_POST,
        // Response handler — runs after body handler has finished
        [](AsyncWebServerRequest *request)
        {
            Preferences p;
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, s_saveBody, s_saveBodyLen);
            if (err || !p.begin("vacgauge", false))
            {
                request->send(400, "text/plain", "Bad request");
                return;
            }
            p.putString("ssid", doc["ssid"] | "");
            p.putString("pass", doc["pass"] | "");
            const char *nm = doc["name"] | "Press";
            p.putString("name", (nm && strlen(nm) > 0) ? nm : "Press");
            // Preserve existing cal/offset settings
            p.putInt("toff",    data.offsetTorr);
            p.putInt("mtoff",   data.offsetmTorr);
            p.putInt("calmode", data.useCalTable);
            p.putInt("calcnt",  data.calCount);
            p.end();
            request->send(200, "text/plain", "Saved. Restarting...");
            restartPending = true;
        },
        nullptr,  // no file upload
        // Body accumulator
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
           size_t index, size_t total)
        {
            if (index == 0) s_saveBodyLen = 0;
            size_t space = sizeof(s_saveBody) - s_saveBodyLen - 1;
            size_t copy  = (len < space) ? len : space;
            memcpy(s_saveBody + s_saveBodyLen, data, copy);
            s_saveBodyLen += copy;
            s_saveBody[s_saveBodyLen] = '\0';
        }
    );

    // REST: GET /pressure
    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        float p;
        int   raw;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        p   = (float)data.calPress;
        raw = data.rawData;
        xSemaphoreGive(dataMutex);

        float       displayVal;
        const char *displayUnit;
        if (p < 1.0f) { displayVal = p * 1000.0f; displayUnit = "mTorr"; }
        else          { displayVal = p;             displayUnit = "Torr";  }

        JsonDocument doc;
        doc["device"]        = data.Name;
        doc["pressure"]      = p;
        doc["unit"]          = "Torr";
        doc["display_value"] = displayVal;
        doc["display_unit"]  = displayUnit;
        doc["raw"]           = (uint32_t)raw;
        doc["cal_mode"]      = data.useCalTable;
        doc["timestamp"]     = (uint32_t)(millis() / 1000);

        String body;
        serializeJson(doc, body);
        AsyncWebServerResponse *r = request->beginResponse(200, "application/json", body);
        r->addHeader("Access-Control-Allow-Origin", "*");
        request->send(r);
    });

    // REST: GET /status
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        JsonDocument doc;
        doc["device"]     = data.Name;
        doc["ip"]         = WiFi.localIP().toString();
        doc["rssi"]       = WiFi.RSSI();
        doc["cal_mode"]   = data.useCalTable;
        doc["cal_points"] = data.calCount;

        String body;
        serializeJson(doc, body);
        AsyncWebServerResponse *r = request->beginResponse(200, "application/json", body);
        r->addHeader("Access-Control-Allow-Origin", "*");
        request->send(r);
    });

    // Captive portal redirect for all unknown paths
    server.onNotFound([](AsyncWebServerRequest *request)
    {
        if (appState == STATE_PORTAL)
            request->redirect("http://192.168.4.1/");
        else
            request->send(404, "text/plain", "Not found");
    });

    server.begin();
}

// ---------------------------------------------------------------------------
// Serial command handlers
// ---------------------------------------------------------------------------

void saveSettings()
{
    if (!cp.checkExpectedArgs(0)) return;
    if (saveData()) { lastSaved = data; cp.sendACK(); }
    else              cp.sendNAK();
}

void loadSettings()
{
    if (!cp.checkExpectedArgs(0)) return;
    if (loadData()) { lastSaved = data; cp.sendACK(); }
    else              cp.sendNAK();
}

void wifiSsid()
{
    if (cp.getNumArgs() == 0) { cp.sendACK(false); cp.println(data.ssid); return; }
    if (cp.getNumArgs() != 1) { cp.sendNAK(); return; }
    char *v;
    if (!cp.getValue(&v)) { cp.sendNAK(); return; }
    strncpy(data.ssid, v, sizeof(data.ssid) - 1);
    data.ssid[sizeof(data.ssid) - 1] = '\0';
    cp.sendACK();
}

void wifiPass()
{
    if (cp.getNumArgs() == 0) { cp.sendACK(false); cp.println(data.pass); return; }
    if (cp.getNumArgs() != 1) { cp.sendNAK(); return; }
    char *v;
    if (!cp.getValue(&v)) { cp.sendNAK(); return; }
    strncpy(data.pass, v, sizeof(data.pass) - 1);
    data.pass[sizeof(data.pass) - 1] = '\0';
    cp.sendACK();
}

void wifiConn()
{
    if (!cp.checkExpectedArgs(0)) return;
    saveData();
    lastSaved = data;
    startSTA();
    cp.sendACK();
}

void wifiStatus()
{
    if (!cp.checkExpectedArgs(0)) return;
    cp.sendACK(false);
    cp.print("SSID: ");
    cp.println(data.ssid);
    cp.print("State: ");
    if (appState == STATE_RUNNING)    cp.println("Connected");
    else if (appState == STATE_CONNECTING) cp.println("Connecting");
    else                              cp.println("Portal");
    cp.print("IP: ");
    cp.println(WiFi.localIP().toString().c_str());
}

void calAddPoint()
{
    float torr;
    if (cp.getNumArgs() != 1) { cp.sendNAK(); return; }
    if (!cp.getValue(&torr))  { cp.sendNAK(); return; }

    int raw = sampleRawAveraged(kCalAvgSamples);
    if (!addCalPoint(raw, torr)) { cp.sendNAK(); return; }

    saveData();
    lastSaved = data;
    cp.sendACK(false);
    cp.print("Cal point raw=");
    cp.print(raw);
    cp.print(" torr=");
    cp.println(torr);
}

void calClear()
{
    if (!cp.checkExpectedArgs(0)) return;
    data.calCount = 0;
    saveData();
    lastSaved = data;
    cp.sendACK();
}

void calDefaults()
{
    if (!cp.checkExpectedArgs(0)) return;
    loadDefaultCalTable();
    saveData();
    lastSaved = data;
    cp.sendACK();
}

void getRaw()
{
    if (!cp.checkExpectedArgs(0)) return;
    cp.println(sampleRawAveraged(kCalAvgSamples));
}

void calDump()
{
    if (!cp.checkExpectedArgs(0)) return;
    cp.sendACK(false);
    cp.print("Cal table, ");
    cp.print(data.calCount);
    cp.println(" points:");
    for (int i = 0; i < data.calCount; i++)
    {
        cp.print("  ");
        cp.print(data.calTable[i].raw);
        cp.print("  ");
        cp.println(data.calTable[i].torr);
    }
}

void Debug()
{
    cp.print("Raw data: ");
    cp.println(data.rawData);
}

void cmdNop()
{
    // MIPS sends NOP as a keepalive — just ACK it
    cp.sendACK();
}

void cmdGetName()
{
    if (cp.getNumArgs() == 0)
    {
        cp.sendACK(false);
        cp.println(data.Name);
        return;
    }
    if (cp.getNumArgs() != 1) { cp.sendNAK(); return; }
    char *v;
    if (!cp.getValue(&v)) { cp.sendNAK(); return; }
    strncpy(data.Name, v, sizeof(data.Name) - 1);
    data.Name[sizeof(data.Name) - 1] = '\0';
    cp.sendACK();
}

void cmdGetCmds()
{
    if (!cp.checkExpectedArgs(0)) return;
    cp.sendACK(false);
    for (int i = 0; cmds[i].cmd != NULL; i++)
    {
        cp.print(cmds[i].cmd);
        cp.print(",");
        cp.println(cmds[i].help);
    }
}
// ---------------------------------------------------------------------------
// Command table
// ---------------------------------------------------------------------------

Command cmds[] =
{
    {"NOP",      CMDfunction,  0, (void *)cmdNop,             NULL, "No operation"},
    {"GVER",     CMDstr,       0, (void *)Version,            NULL, "Firmware version"},
    {"GNAME",    CMDfunction, -1, (void *)cmdGetName,         NULL, "Get/set device name"},
    {"SNAME",    CMDfunction,  1, (void *)cmdGetName,         NULL, "Set device name"},
    {"GCMDS",    CMDfunction,  0, (void *)cmdGetCmds,         NULL, "List all commands"},
    {"?NAME",    CMDstr,      -1, (void *)&data.Name,         NULL, "Device name (legacy)"},
    {"LOAD",     CMDfunction,  0, (void *)loadSettings,       NULL, "Load saved parameters from NVS"},
    {"SAVE",     CMDfunction,  0, (void *)saveSettings,       NULL, "Save parameters to NVS"},
    {"GPRES",    CMDdouble,    0, (void *)&data.calPress,     NULL, "Return pressure in Torr"},
    {"GRPRES",   CMDint,       0, (void *)&data.rawData,      NULL, "Return last raw pressure count"},
    {"GRAW",     CMDfunction,  0, (void *)getRaw,             NULL, "Return a fresh averaged raw reading"},
    {"GRTEMP",   CMDint,       0, (void *)&data.rawTemp,      NULL, "Return raw temperature count"},
    {"?TOFFSET", CMDint,      -1, (void *)&data.offsetTorr,   NULL, "Set/return Torr offset"},
    {"?MTOFFSET",CMDint,      -1, (void *)&data.offsetmTorr,  NULL, "Set/return milli-Torr offset"},
    {"?CALMODE", CMDint,      -1, (void *)&data.useCalTable,  NULL, "Calibration mode: 0=factory formula, 1=PWL table"},
    {"CALPRES",  CMDfunction,  1, (void *)calAddPoint,        NULL, "CALPRES,<torr>: capture cal point at current raw"},
    {"CALCLEAR", CMDfunction,  0, (void *)calClear,           NULL, "Clear the calibration table"},
    {"CALDEF",   CMDfunction,  0, (void *)calDefaults,        NULL, "Restore factory default cal table"},
    {"CALDUMP",  CMDfunction,  0, (void *)calDump,            NULL, "List the calibration table"},
    {"SSID",     CMDfunction, -1, (void *)wifiSsid,           NULL, "Get/set WiFi SSID"},
    {"PASS",     CMDfunction, -1, (void *)wifiPass,           NULL, "Get/set WiFi password"},
    {"WIFICONN", CMDfunction,  0, (void *)wifiConn,           NULL, "Save WiFi settings and connect in STA mode"},
    {"WIFI",     CMDfunction,  0, (void *)wifiStatus,         NULL, "Report WiFi status"},
    {NULL}
};
static CommandList cmdList = {cmds, NULL};

// ---------------------------------------------------------------------------
// Change detection
// ---------------------------------------------------------------------------

static bool settingsChanged()
{
    return data.offsetTorr  != lastSaved.offsetTorr  ||
           data.offsetmTorr != lastSaved.offsetmTorr ||
           data.useCalTable != lastSaved.useCalTable  ||
           data.TWIadd      != lastSaved.TWIadd       ||
           data.Rev         != lastSaved.Rev          ||
           strncmp(data.Name, lastSaved.Name, sizeof(data.Name)) != 0 ||
           strncmp(data.ssid, lastSaved.ssid, sizeof(data.ssid)) != 0 ||
           strncmp(data.pass, lastSaved.pass, sizeof(data.pass)) != 0;
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup()
{
    delay(100);
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    dataMutex = xSemaphoreCreateMutex();
    wireMutex = xSemaphoreCreateMutex();

    Wire.begin(kI2C_SDA, kI2C_SCL, kI2C_Hz);
    pinMode(kBootPin, INPUT_PULLUP);

    if (!canvas->begin())
        Serial.println("Display init failed — check QSPI wiring");
    canvas->fillScreen(C_BLACK);
    canvas->flush();

    cp.registerStream(&Serial);
    cp.registerCommands(&cmdList);
    cp.registerCommands(dbg.debugCommands());
    dbg.registerDebugFunction(Debug);

    if (!loadData())
    {
        loadDefaultCalTable();
        saveData();
    }
    lastSaved = data;

    if (strlen(data.ssid) == 0)
        startAP();
    else
        startSTA();

    setupServer();

    xTaskCreatePinnedToCore(sensorTask, "sensor", 4096, nullptr, 1, nullptr, 1);
}

void loop()
{
    cp.processStreams();
    cp.processCommands();

    // DNS captive portal (AP mode only)
    if (appState == STATE_PORTAL)
        dnsServer.processNextRequest();

    // WiFi connection state machine
    if (appState == STATE_CONNECTING)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            appState = STATE_RUNNING;
            Serial.print("Connected, IP: ");
            Serial.println(WiFi.localIP());
        }
        else if (millis() - connectStartMs > 30000)
        {
            Serial.println("WiFi timeout — falling back to portal");
            startAP();
        }
    }

    // Boot button long-press: clear WiFi credentials and reboot to portal
    bool bootDown = (digitalRead(kBootPin) == LOW);
    if (bootDown && !bootWasPressed)
    {
        bootWasPressed = true;
        bootPressStart = millis();
    }
    if (!bootDown)
        bootWasPressed = false;
    if (bootWasPressed && (millis() - bootPressStart >= kBootLongPressMs))
    {
        clearCredentials();
        delay(200);
        ESP.restart();
    }

    // Display refresh
    unsigned long now = millis();
    if (now - lastDisplayMs >= 500)
    {
        lastDisplayMs = now;
        updateDisplay();
    }

    // Persist changed settings
    if (now - lastSaveMs >= kSaveIntervalMs)
    {
        lastSaveMs = now;
        if (settingsChanged())
        {
            lastSaved = data;
            saveData();
        }
    }

    // Deferred restart (triggered by POST /save)
    if (restartPending)
    {
        delay(1500);
        ESP.restart();
    }
}
