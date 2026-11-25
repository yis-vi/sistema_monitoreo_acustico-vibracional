#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <SD.h>
#include <WebServer.h>
#include <math.h>

#define USAR_SH1106 true
#if USAR_SH1106
  #include <Adafruit_SH110X.h>
#else
  #include <Adafruit_SSD1306.h>
#endif

#define PIN_SD_CS       5
#define PIN_SD_SCK     18
#define PIN_SD_MOSI    23
#define PIN_SD_MISO    19
#define OLED_SDA       21
#define OLED_SCL       22

#define MPU_ADDR       0x68
#define OLED_WIDTH     128
#define OLED_HEIGHT     64
#define OLED_ADDR      0x3C
#if USAR_SH1106
  Adafruit_SH1106G oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
  #define OLED_BEGIN() oled.begin(OLED_ADDR, true)
  #define OLED_WHITE   SH110X_WHITE
  #define OLED_BLACK   SH110X_BLACK
#else
  Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
  #define OLED_BEGIN() oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)
  #define OLED_WHITE   SSD1306_WHITE
  #define OLED_BLACK   SSD1306_BLACK
#endif

const char* AP_SSID = "Sentina";
const char* AP_PASS = "sentina123";

uint8_t CABINA_MAC[] = {0x00, 0x70, 0x07, 0x1C, 0x83, 0xC0};

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint16_t noise_level;
    uint8_t  peak_event;
    uint8_t  tipo;
} EspNowPacket;

#define SAMPLE_WINDOW_MS   50
#define PEAK_THRESHOLD   2000
#define OLED_GRAPH_MAX   4095

#define INTERVAL_SENSOR   50
#define INTERVAL_OLED    100
#define INTERVAL_ESPNOW  100
#define INTERVAL_CSV     500

#define GRAPH_POINTS  124
uint16_t graphBuf[GRAPH_POINTS];
uint8_t  graphIdx = 0;

#define WEB_HISTORY  200
uint16_t webBuf[WEB_HISTORY];
uint8_t  webIdx = 0;

volatile uint16_t latestNoise      = 0;
volatile uint32_t totalPeaks       = 0;
volatile bool     sdAvailable      = false;
volatile bool     newPktFromCabina = false;
EspNowPacket      pktFromCabina;

uint16_t peakHold   = 0;
uint32_t peakHoldMs = 0;

bool  wasAboveThreshold = false;
float latestAx    = 0.0f;
float latestAy    = 0.0f;
float latestAz    = 1.0f;
float latestAmag  = 1.0f;
float latestGx    = 0.0f;
float latestGy    = 0.0f;
float latestGz    = 0.0f;
float latestTemp  = 0.0f;
float latestPitch = 0.0f;
float latestRoll  = 0.0f;

uint32_t lastSensorMs = 0;
uint32_t lastOledMs   = 0;
uint32_t lastEspnowMs = 0;
uint32_t lastCsvMs    = 0;

WebServer server(80);

void initWiFiAP();
void initEspNow();
void initOled();
void drawSplashScreen();
void initMpu();
void initSD();
void initWebServer();
bool readMpuFull(uint32_t& rawMag);
void taskSensor();
void taskOled();
void taskEspNow();
void taskSD();
void handleCabinaPacket();
void writeCSV(const EspNowPacket& pkt, const char* source);
void onDataSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status);
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);
String buildHtmlPage();

void setup() {
    Serial.begin(115200);
    Wire.begin(OLED_SDA, OLED_SCL);

    initMpu();
    initOled();
    drawSplashScreen();

    initSD();
    initWiFiAP();
    Serial.print("[Sentina] MAC STA: ");
    Serial.println(WiFi.macAddress());
    initEspNow();
    initWebServer();

    Serial.println("[Sentina] Listo.");
}

void loop() {
    uint32_t now = millis();

    if (now - lastSensorMs >= INTERVAL_SENSOR) {
        lastSensorMs = now;
        taskSensor();
    }
    if (now - lastOledMs >= INTERVAL_OLED) {
        lastOledMs = now;
        taskOled();
    }
    if (now - lastEspnowMs >= INTERVAL_ESPNOW) {
        lastEspnowMs = now;
        taskEspNow();
    }
    if (now - lastCsvMs >= INTERVAL_CSV) {
        lastCsvMs = now;
        taskSD();
    }

    if (newPktFromCabina) {
        newPktFromCabina = false;
        handleCabinaPacket();
    }

    server.handleClient();
}

void initWiFiAP() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS, 1);
    Serial.print("[WiFi] AP IP: ");
    Serial.println(WiFi.softAPIP());
}

void initEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Error al inicializar.");
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, CABINA_MAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Error al agregar peer Cabina.");
    }
}

void initMpu() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) {
        Serial.println("[MPU6050] No encontrado.");
        return;
    }

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x02);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x18);
    Wire.endTransmission(true);

    Serial.println("[MPU6050] Inicializado en modo impacto casco (+-16g).");
}

void initOled() {
    if (!OLED_BEGIN()) {
        Serial.println("[OLED] No encontrada — continuando sin pantalla.");
    }
    oled.clearDisplay();
    oled.display();
}

void drawSplashScreen() {
    oled.clearDisplay();
    oled.drawRoundRect(2, 2, 124, 60, 5, OLED_WHITE);
    oled.drawFastHLine(10, 28, 108, OLED_WHITE);

    oled.setTextColor(OLED_WHITE);
    oled.setTextSize(2);
    oled.setCursor(22, 9);
    oled.print("SENTINA");

    oled.setTextSize(1);
    oled.setCursor(16, 34);
    oled.print("SENSOR IMU +-16g");

    oled.drawRoundRect(16, 47, 96, 8, 2, OLED_WHITE);
    oled.fillRect(18, 49, 92, 4, OLED_WHITE);
    oled.display();
    delay(600);
}

void initSD() {
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    if (!SD.begin(PIN_SD_CS)) {
        Serial.println("[SD] Tarjeta no encontrada o error — continuando sin SD.");
        sdAvailable = false;
        return;
    }
    sdAvailable = true;
    Serial.println("[SD] Tarjeta inicializada.");

    if (!SD.exists("/registro.csv")) {
        File f = SD.open("/registro.csv", FILE_WRITE);
        if (f) {
            f.println("timestamp_ms,fuente,noise_level,peak_event,tipo");
            f.close();
        }
    }
}

void initWebServer() {
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", buildHtmlPage());
    });

    server.on("/data", HTTP_GET, []() {
        String json;
        json.reserve(1400);
        char header[360];
        snprintf(
            header,
            sizeof(header),
            "{\"current\":%u,\"threshold\":%u,\"peaks\":%lu,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"amag\":%.2f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"pitch\":%.1f,\"roll\":%.1f,\"temp\":%.1f,\"sd\":%s,\"uptime\":%lu,\"history\":[",
            latestNoise,
            (uint16_t)PEAK_THRESHOLD,
            (unsigned long)totalPeaks,
            latestAx,
            latestAy,
            latestAz,
            latestAmag,
            latestGx,
            latestGy,
            latestGz,
            latestPitch,
            latestRoll,
            latestTemp,
            sdAvailable ? "true" : "false",
            (unsigned long)(millis() / 1000)
        );
        json += header;
        for (int i = 0; i < WEB_HISTORY; i++) {
            int idx = (webIdx + i) % WEB_HISTORY;
            if (i > 0) json += ",";
            json += String(webBuf[idx]);
        }
        json += "]}";
        server.send(200, "application/json", json);
    });

    server.on("/reset", HTTP_POST, []() {
        totalPeaks = 0;
        server.send(200, "application/json", "{\"ok\":true}");
    });

    server.begin();
    Serial.println("[Web] Servidor iniciado.");
}

bool readMpuFull(uint32_t& rawMag) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true);
    if (Wire.available() < 14) return false;

    int16_t axRaw = (Wire.read() << 8) | Wire.read();
    int16_t ayRaw = (Wire.read() << 8) | Wire.read();
    int16_t azRaw = (Wire.read() << 8) | Wire.read();
    int16_t tRaw  = (Wire.read() << 8) | Wire.read();
    int16_t gxRaw = (Wire.read() << 8) | Wire.read();
    int16_t gyRaw = (Wire.read() << 8) | Wire.read();
    int16_t gzRaw = (Wire.read() << 8) | Wire.read();

    float mag = sqrtf((float)axRaw * axRaw + (float)ayRaw * ayRaw + (float)azRaw * azRaw);
    rawMag = (uint32_t)mag;

    latestAx    = axRaw / 2048.0f;
    latestAy    = ayRaw / 2048.0f;
    latestAz    = azRaw / 2048.0f;
    latestAmag  = mag / 2048.0f;
    latestTemp  = (tRaw / 340.0f) + 36.53f;
    latestGx    = gxRaw / 131.0f;
    latestGy    = gyRaw / 131.0f;
    latestGz    = gzRaw / 131.0f;
    latestPitch = atan2f(-latestAx, sqrtf(latestAy * latestAy + latestAz * latestAz)) * 57.29578f;
    latestRoll  = atan2f(latestAy, latestAz) * 57.29578f;

    return true;
}

void taskSensor() {
    uint32_t start = millis();
    uint32_t vMax = 0;
    uint32_t vMin = 0xFFFFFFFF;
    bool hasSample = false;

    while (millis() - start < SAMPLE_WINDOW_MS) {
        uint32_t v = 0;
        if (readMpuFull(v) && v > 0) {
            if (v > vMax) vMax = v;
            if (v < vMin) vMin = v;
            hasSample = true;
        }
    }

    uint32_t rawPeak = (hasSample && vMax >= vMin) ? (vMax - vMin) : 0;
    uint16_t peakToPeak = constrain(rawPeak / 12, 0, 4095);
    latestNoise = peakToPeak;

    if (peakToPeak >= PEAK_THRESHOLD) {
        if (!wasAboveThreshold) {
            totalPeaks++;
            wasAboveThreshold = true;
        }
    } else {
        wasAboveThreshold = false;
    }

    graphBuf[graphIdx] = peakToPeak;
    graphIdx = (graphIdx + 1) % GRAPH_POINTS;

    webBuf[webIdx] = peakToPeak;
    webIdx = (webIdx + 1) % WEB_HISTORY;
}

void taskOled() {
    uint32_t now = millis();

    if (latestNoise >= peakHold) {
        peakHold = latestNoise;
        peakHoldMs = now;
    } else if (now - peakHoldMs > 800 && peakHold > 0) {
        peakHold = (peakHold > 65) ? (peakHold - 65) : 0;
    }

    oled.clearDisplay();

    oled.fillRoundRect(0, 0, 48, 12, 2, OLED_WHITE);
    oled.setTextSize(1);
    oled.setTextColor(OLED_BLACK);
    oled.setCursor(3, 2);
    oled.print("SENTINA");

    if (sdAvailable) {
        oled.setTextColor(OLED_WHITE);
        oled.setCursor(52, 2);
        oled.print("SD");
    }

    if (latestNoise >= PEAK_THRESHOLD) {
        if ((now / 150) % 2 == 0) {
            oled.fillRoundRect(74, 0, 54, 12, 2, OLED_WHITE);
            oled.setTextColor(OLED_BLACK);
            oled.setCursor(77, 2);
            oled.print("! PICO !");
        } else {
            oled.drawRoundRect(74, 0, 54, 12, 2, OLED_WHITE);
            oled.setTextColor(OLED_WHITE);
            oled.setCursor(77, 2);
            oled.print("! PICO !");
        }
    } else {
        oled.setTextColor(OLED_WHITE);
        oled.setCursor(74, 2);
        oled.print("UMB:");
        oled.print(PEAK_THRESHOLD);
    }

    char valStr[8];
    snprintf(valStr, sizeof(valStr), "%4u", latestNoise);
    oled.setTextColor(OLED_WHITE);
    oled.setTextSize(2);
    oled.setCursor(0, 17);
    oled.print(valStr);

    oled.drawRoundRect(56, 16, 72, 9, 2, OLED_WHITE);
    int fillW = map(constrain(latestNoise, 0, OLED_GRAPH_MAX), 0, OLED_GRAPH_MAX, 0, 68);
    if (fillW > 0) {
        oled.fillRect(58, 18, fillW, 5, OLED_WHITE);
    }
    int holdX = 58 + map(constrain(peakHold, 0, OLED_GRAPH_MAX), 0, OLED_GRAPH_MAX, 0, 67);
    oled.drawFastVLine(holdX, 15, 11, OLED_WHITE);
    int threshTickX = 58 + map(constrain(PEAK_THRESHOLD, 0, OLED_GRAPH_MAX), 0, OLED_GRAPH_MAX, 0, 67);
    oled.drawPixel(threshTickX, 14, OLED_WHITE);
    oled.drawPixel(threshTickX, 26, OLED_WHITE);

    oled.setTextSize(1);
    oled.setCursor(56, 27);
    oled.print("MAX:");
    oled.print(peakHold);

    const int plotX = 2;
    const int plotW = GRAPH_POINTS;
    const int plotTop = 37;
    const int plotBottom = 62;
    const int plotH = plotBottom - plotTop;

    int threshVal = constrain(PEAK_THRESHOLD, 0, OLED_GRAPH_MAX);
    int threshY = plotBottom - map(threshVal, 0, OLED_GRAPH_MAX, 0, plotH);

    oled.drawFastVLine(0, plotTop, plotH + 1, OLED_WHITE);
    oled.drawFastVLine(127, plotTop, plotH + 1, OLED_WHITE);
    oled.drawPixel(1, plotTop, OLED_WHITE);
    oled.drawPixel(1, threshY, OLED_WHITE);
    oled.drawPixel(1, plotBottom, OLED_WHITE);
    oled.drawPixel(126, plotTop, OLED_WHITE);
    oled.drawPixel(126, threshY, OLED_WHITE);
    oled.drawPixel(126, plotBottom, OLED_WHITE);

    for (int x = plotX; x < plotX + plotW; x += 4) {
        oled.drawPixel(x, threshY, OLED_WHITE);
    }

    int prevX = plotX;
    int prevY = plotBottom;
    for (int i = 0; i < GRAPH_POINTS; i++) {
        int idx = (graphIdx + i) % GRAPH_POINTS;
        uint16_t val = constrain(graphBuf[idx], 0, OLED_GRAPH_MAX);
        int x = plotX + i;
        int y = plotBottom - map(val, 0, OLED_GRAPH_MAX, 0, plotH);

        if ((x % 2) == 0) {
            for (int fy = y + 2; fy <= plotBottom; fy += 3) {
                oled.drawPixel(x, fy, OLED_WHITE);
            }
        }

        if (i > 0) {
            oled.drawLine(prevX, prevY, x, y, OLED_WHITE);
        } else {
            oled.drawPixel(x, y, OLED_WHITE);
        }

        if (val >= PEAK_THRESHOLD) {
            oled.fillCircle(x, y, 1, OLED_WHITE);
        }

        prevX = x;
        prevY = y;
    }

    oled.display();
}

void taskEspNow() {
    EspNowPacket pkt;
    pkt.timestamp_ms = millis();
    pkt.noise_level  = latestNoise;
    pkt.peak_event   = (latestNoise >= PEAK_THRESHOLD) ? 1 : 0;
    pkt.tipo         = 0;

    esp_now_send(CABINA_MAC, (uint8_t*)&pkt, sizeof(pkt));
}

void taskSD() {
    if (!sdAvailable) return;
    EspNowPacket pkt;
    pkt.timestamp_ms = millis();
    pkt.noise_level  = latestNoise;
    pkt.peak_event   = (latestNoise >= PEAK_THRESHOLD) ? 1 : 0;
    pkt.tipo         = 0;
    writeCSV(pkt, "SENTINA");
}

void handleCabinaPacket() {
    writeCSV(pktFromCabina, "CABINA");
    Serial.print("[ESP-NOW] Evento de Cabina guardado. Nivel=");
    Serial.println(pktFromCabina.noise_level);
}

void writeCSV(const EspNowPacket& pkt, const char* source) {
    if (!sdAvailable) return;
    File f = SD.open("/registro.csv", FILE_APPEND);
    if (!f) {
        Serial.println("[SD] No se pudo abrir registro.csv");
        return;
    }
    f.print(pkt.timestamp_ms);  f.print(",");
    f.print(source);            f.print(",");
    f.print(pkt.noise_level);   f.print(",");
    f.print(pkt.peak_event);    f.print(",");
    f.println(pkt.tipo);
    f.close();
}

void onDataSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[ESP-NOW] Envío fallido.");
    }
}

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len == sizeof(EspNowPacket)) {
        memcpy(&pktFromCabina, data, sizeof(EspNowPacket));
        newPktFromCabina = true;
    }
}

String buildHtmlPage() {
    return R"rawhtml(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Sentina — Monitor Estructural de Casco</title>
  <style>
    :root {
      --bg: #070b14;
      --surface: rgba(15, 23, 42, 0.82);
      --surface-alt: rgba(30, 41, 59, 0.55);
      --border: rgba(148, 163, 184, 0.14);
      --text: #f1f5f9;
      --muted: #94a3b8;
      --cyan: #22d3ee;
      --blue: #38bdf8;
      --emerald: #10b981;
      --amber: #f59e0b;
      --rose: #f43f5e;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      background: radial-gradient(circle at 15% 10%, #0f1e36 0%, var(--bg) 55%);
      color: var(--text);
      min-height: 100vh;
      padding: 20px;
    }
    .container { max-width: 1220px; margin: 0 auto; }
    header {
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      align-items: center;
      gap: 14px;
      margin-bottom: 22px;
      padding: 16px 22px;
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 16px;
      backdrop-filter: blur(12px);
    }
    .brand { display: flex; align-items: center; gap: 12px; }
    .logo-box {
      width: 40px; height: 40px;
      border-radius: 10px;
      background: linear-gradient(135deg, var(--cyan), #2563eb);
      display: grid; place-items: center;
      font-weight: 800; color: #04101d; font-size: 1.1rem;
      box-shadow: 0 0 20px rgba(34, 211, 238, 0.35);
    }
    .brand h1 { font-size: 1.15rem; font-weight: 700; letter-spacing: 0.3px; }
    .brand p { font-size: 0.8rem; color: var(--muted); }
    .badges { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; }
    .badge {
      display: inline-flex; align-items: center; gap: 7px;
      padding: 6px 12px; border-radius: 999px;
      font-size: 0.78rem; font-weight: 600;
      background: var(--surface-alt); border: 1px solid var(--border);
    }
    .dot { width: 8px; height: 8px; border-radius: 50%; background: var(--emerald); box-shadow: 0 0 8px var(--emerald); }
    .dot.off { background: var(--rose); box-shadow: 0 0 8px var(--rose); }
    .kpi-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 16px;
      margin-bottom: 20px;
    }
    .card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 18px 20px;
      backdrop-filter: blur(12px);
      transition: border-color 0.25s ease, box-shadow 0.25s ease;
    }
    .card.alert {
      border-color: rgba(244, 63, 94, 0.65);
      box-shadow: 0 0 24px rgba(244, 63, 94, 0.22);
    }
    .kpi-label {
      font-size: 0.78rem; text-transform: uppercase; letter-spacing: 0.8px;
      color: var(--muted); margin-bottom: 8px; display: flex; justify-content: space-between;
    }
    .kpi-value { font-size: 2rem; font-weight: 800; font-variant-numeric: tabular-nums; line-height: 1.1; }
    .kpi-unit { font-size: 0.9rem; font-weight: 500; color: var(--muted); margin-left: 4px; }
    .kpi-sub { font-size: 0.8rem; color: var(--muted); margin-top: 8px; display: flex; justify-content: space-between; align-items: center; }
    .status-pill {
      padding: 2px 9px; border-radius: 6px; font-size: 0.72rem; font-weight: 700;
      background: rgba(16, 185, 129, 0.16); color: var(--emerald);
    }
    .status-pill.warn { background: rgba(244, 63, 94, 0.2); color: var(--rose); }
    .chart-card { margin-bottom: 20px; }
    .chart-header {
      display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center;
      gap: 14px; margin-bottom: 14px;
    }
    .chart-title h2 { font-size: 1rem; font-weight: 700; }
    .chart-title span { font-size: 0.8rem; color: var(--muted); }
    .controls { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; }
    .ctrl-group {
      display: flex; align-items: center; gap: 6px;
      background: var(--surface-alt); padding: 5px 10px;
      border-radius: 10px; border: 1px solid var(--border);
    }
    .ctrl-group label { font-size: 0.75rem; color: var(--muted); font-weight: 600; }
    select, button {
      background: #0f172a; color: var(--text);
      border: 1px solid var(--border); border-radius: 7px;
      padding: 5px 10px; font-size: 0.78rem; font-weight: 600;
      cursor: pointer; transition: all 0.2s;
    }
    select:hover, button:hover { border-color: var(--cyan); color: var(--cyan); }
    button.active { background: rgba(34, 211, 238, 0.18); border-color: var(--cyan); color: var(--cyan); }
    .canvas-wrap {
      position: relative; width: 100%; height: 320px;
      background: rgba(4, 8, 18, 0.75);
      border: 1px solid var(--border);
      border-radius: 12px; overflow: hidden;
      transition: height 0.25s ease;
    }
    canvas { display: block; width: 100%; height: 100%; }
    .sensors-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(310px, 1fr));
      gap: 16px;
    }
    .section-title {
      font-size: 0.9rem; font-weight: 700; margin-bottom: 14px;
      display: flex; justify-content: space-between; align-items: center;
    }
    .section-title span { font-size: 0.75rem; color: var(--muted); font-weight: 500; }
    .axis-row { margin-bottom: 12px; }
    .axis-row:last-child { margin-bottom: 0; }
    .axis-meta { display: flex; justify-content: space-between; font-size: 0.82rem; margin-bottom: 5px; font-variant-numeric: tabular-nums; }
    .axis-name { font-weight: 600; color: var(--muted); }
    .axis-val { font-weight: 700; color: var(--text); }
    .bar-track {
      height: 8px; background: rgba(148, 163, 184, 0.12);
      border-radius: 999px; position: relative; overflow: hidden;
    }
    .bar-center { position: absolute; left: 50%; top: 0; bottom: 0; width: 1px; background: rgba(255,255,255,0.25); }
    .bar-fill {
      position: absolute; top: 0; bottom: 0;
      background: linear-gradient(90deg, var(--blue), var(--cyan));
      border-radius: 999px; transition: all 0.15s ease;
    }
    .gyro-fill { background: linear-gradient(90deg, #818cf8, #c084fc); }
    .tilt-box {
      display: flex; align-items: center; justify-content: space-around;
      gap: 16px; padding-top: 6px;
    }
    .horizon {
      width: 112px; height: 112px; border-radius: 50%;
      border: 2px solid var(--cyan);
      background: linear-gradient(180deg, rgba(14, 165, 233, 0.22) 50%, rgba(16, 185, 129, 0.22) 50%);
      position: relative; overflow: hidden;
      box-shadow: inset 0 0 18px rgba(0,0,0,0.65);
    }
    .horizon-line {
      position: absolute; left: -20%; right: -20%; top: 50%; height: 2px;
      background: var(--cyan); box-shadow: 0 0 8px var(--cyan);
      transition: transform 0.15s ease;
    }
    .horizon-cross {
      position: absolute; inset: 0; display: grid; place-items: center;
      font-size: 0.7rem; color: rgba(255,255,255,0.5); pointer-events: none;
    }
    .tilt-stats { display: flex; flex-direction: column; gap: 10px; flex: 1; }
    .tilt-item {
      background: var(--surface-alt); border: 1px solid var(--border);
      padding: 8px 12px; border-radius: 10px;
      display: flex; justify-content: space-between; align-items: center;
    }
    .tilt-item span { font-size: 0.8rem; color: var(--muted); }
    .tilt-item strong { font-size: 1rem; font-variant-numeric: tabular-nums; color: var(--cyan); }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <div class="brand">
        <div class="logo-box">S</div>
        <div>
          <h1>Sentina — Monitor Estructural de Casco</h1>
          <p>Detección de impactos severos, ruido estructural y dinámica inercial (±16g)</p>
        </div>
      </div>
      <div class="badges">
        <div class="badge"><span id="connDot" class="dot"></span><span id="connText">Conectado</span></div>
        <div class="badge" id="sdBadge">SD: Verificando...</div>
        <div class="badge">Activo: <span id="uptime">0s</span></div>
      </div>
    </header>

    <section class="kpi-grid">
      <div class="card" id="vibCard">
        <div class="kpi-label"><span>Impacto / Ruido Casco</span><span id="vibState" class="status-pill">NORMAL</span></div>
        <div class="kpi-value" id="nivel">0<span class="kpi-unit">pts</span></div>
        <div class="kpi-sub"><span>Umbral de alerta: <strong id="umbralText">2000</strong></span><span id="maxWin">Máx: 0</span></div>
      </div>

      <div class="card">
        <div class="kpi-label"><span>Fuerza G Resultante</span><span>Rango ±16g</span></div>
        <div class="kpi-value"><span id="amag">1.00</span><span class="kpi-unit">g</span></div>
        <div class="kpi-sub"><span>Reposo nominal: 1.00 g</span><span id="gDelta">Δ 0.00 g</span></div>
      </div>

      <div class="card">
        <div class="kpi-label"><span>Picos Severos</span><span>Evento &ge; 2000</span></div>
        <div class="kpi-value" id="peaks">0</div>
        <div class="kpi-sub"><span>Cruces de umbral</span><button onclick="resetPeaks()">Reiniciar</button></div>
      </div>

      <div class="card">
        <div class="kpi-label"><span>Temperatura IMU</span><span>Filtro 94 Hz</span></div>
        <div class="kpi-value"><span id="temp">0.0</span><span class="kpi-unit">°C</span></div>
        <div class="kpi-sub"><span>MPU-6050 I2C (0x68)</span><span>±16g</span></div>
      </div>
    </section>

    <section class="card chart-card">
      <div class="chart-header">
        <div class="chart-title">
          <h2>Oscilograma de Impactos en el Casco</h2>
          <span>Calibrado para filtrar roces leves y destacar picos de alta energía</span>
        </div>
        <div class="controls">
          <div class="ctrl-group">
            <label for="yScaleSelect">Escala Y (Techo):</label>
            <select id="yScaleSelect" onchange="drawChart()">
              <option value="auto">Auto-Ajuste</option>
              <option value="1000">1000 (Zoom 4x)</option>
              <option value="2500">2500 (Detalle Umbral)</option>
              <option value="4095" selected>4095 (Casco Completo)</option>
            </select>
          </div>
          <div class="ctrl-group">
            <label>Altura:</label>
            <button id="h220" onclick="setHeight(220, this)">220px</button>
            <button id="h320" class="active" onclick="setHeight(320, this)">320px</button>
            <button id="h440" onclick="setHeight(440, this)">440px</button>
          </div>
          <button id="pauseBtn" onclick="togglePause()">Pausar</button>
        </div>
      </div>
      <div class="canvas-wrap" id="canvasWrap" style="height:320px;">
        <canvas id="g"></canvas>
      </div>
    </section>

    <section class="sensors-grid">
      <div class="card">
        <div class="section-title">Acelerómetro de Impacto <span>Rango ±16g</span></div>
        <div class="axis-row">
          <div class="axis-meta"><span class="axis-name">Eje X (Babor / Estribor)</span><span class="axis-val" id="axVal">0.00 g</span></div>
          <div class="bar-track"><div class="bar-center"></div><div class="bar-fill" id="axBar"></div></div>
        </div>
        <div class="axis-row">
          <div class="axis-meta"><span class="axis-name">Eje Y (Proa / Popa)</span><span class="axis-val" id="ayVal">0.00 g</span></div>
          <div class="bar-track"><div class="bar-center"></div><div class="bar-fill" id="ayBar"></div></div>
        </div>
        <div class="axis-row">
          <div class="axis-meta"><span class="axis-name">Eje Z (Vertical / Quilla)</span><span class="axis-val" id="azVal">1.00 g</span></div>
          <div class="bar-track"><div class="bar-center"></div><div class="bar-fill" id="azBar"></div></div>
        </div>
      </div>

      <div class="card">
        <div class="section-title">Giroscopio (Velocidad Angular) <span>Rango ±250 °/s</span></div>
        <div class="axis-row">
          <div class="axis-meta"><span class="axis-name">Giro X (Balanceo)</span><span class="axis-val" id="gxVal">0.0 °/s</span></div>
          <div class="bar-track"><div class="bar-center"></div><div class="bar-fill gyro-fill" id="gxBar"></div></div>
        </div>
        <div class="axis-row">
          <div class="axis-meta"><span class="axis-name">Giro Y (Cabeceo)</span><span class="axis-val" id="gyVal">0.0 °/s</span></div>
          <div class="bar-track"><div class="bar-center"></div><div class="bar-fill gyro-fill" id="gyBar"></div></div>
        </div>
        <div class="axis-row">
          <div class="axis-meta"><span class="axis-name">Giro Z (Guiñada)</span><span class="axis-val" id="gzVal">0.0 °/s</span></div>
          <div class="bar-track"><div class="bar-center"></div><div class="bar-fill gyro-fill" id="gzBar"></div></div>
        </div>
      </div>

      <div class="card">
        <div class="section-title">Inclinación de Embarcación <span>Cálculo Inercial</span></div>
        <div class="tilt-box">
          <div class="horizon">
            <div class="horizon-line" id="horizonLine"></div>
            <div class="horizon-cross">+</div>
          </div>
          <div class="tilt-stats">
            <div class="tilt-item"><span>Cabeceo (Pitch)</span><strong id="pitchVal">0.0°</strong></div>
            <div class="tilt-item"><span>Escora (Roll)</span><strong id="rollVal">0.0°</strong></div>
          </div>
        </div>
      </div>
    </section>
  </div>

  <script>
    const canvas = document.getElementById('g');
    const ctx = canvas.getContext('2d');
    let historyData = [];
    let thresholdVal = 2000;
    let paused = false;

    function setHeight(px, btn) {
      document.getElementById('canvasWrap').style.height = px + 'px';
      document.querySelectorAll('.ctrl-group button').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      setTimeout(drawChart, 260);
    }

    function togglePause() {
      paused = !paused;
      const btn = document.getElementById('pauseBtn');
      btn.textContent = paused ? 'Reanudar' : 'Pausar';
      btn.classList.toggle('active', paused);
    }

    async function resetPeaks() {
      try {
        await fetch('/reset', { method: 'POST' });
        document.getElementById('peaks').textContent = '0';
      } catch (e) {}
    }

    function setBiBar(id, val, maxRange) {
      const el = document.getElementById(id);
      const clamped = Math.max(-maxRange, Math.min(maxRange, val));
      const pct = (Math.abs(clamped) / maxRange) * 50;
      if (clamped >= 0) {
        el.style.left = '50%';
        el.style.width = pct + '%';
      } else {
        el.style.left = (50 - pct) + '%';
        el.style.width = pct + '%';
      }
    }

    function drawChart() {
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

      const W = rect.width;
      const H = rect.height;
      const padL = 44, padR = 14, padT = 16, padB = 22;
      const plotW = W - padL - padR;
      const plotH = H - padT - padB;

      ctx.clearRect(0, 0, W, H);
      if (!historyData.length) return;

      const mode = document.getElementById('yScaleSelect').value;
      const maxData = Math.max(...historyData, 100);
      let yMax = mode === 'auto' ? Math.max(400, Math.ceil(maxData * 1.25)) : parseInt(mode, 10);

      ctx.strokeStyle = 'rgba(148, 163, 184, 0.12)';
      ctx.fillStyle = '#94a3b8';
      ctx.font = '11px sans-serif';
      ctx.lineWidth = 1;

      const gridLines = 4;
      for (let i = 0; i <= gridLines; i++) {
        const y = padT + (plotH / gridLines) * i;
        const val = Math.round(yMax * (1 - i / gridLines));
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(W - padR, y);
        ctx.stroke();
        ctx.fillText(val, 6, y + 4);
      }

      if (thresholdVal <= yMax) {
        const yThresh = padT + plotH - (thresholdVal / yMax) * plotH;
        ctx.save();
        ctx.strokeStyle = '#f43f5e';
        ctx.setLineDash([6, 4]);
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(padL, yThresh);
        ctx.lineTo(W - padR, yThresh);
        ctx.stroke();
        ctx.fillStyle = '#f43f5e';
        ctx.fillText('Umbral ' + thresholdVal, W - padR - 74, yThresh - 5);
        ctx.restore();
      }

      const step = plotW / Math.max(1, historyData.length - 1);
      const grad = ctx.createLinearGradient(0, padT, 0, padT + plotH);
      grad.addColorStop(0, 'rgba(34, 211, 238, 0.34)');
      grad.addColorStop(1, 'rgba(34, 211, 238, 0.0)');

      ctx.beginPath();
      historyData.forEach((v, i) => {
        const clamped = Math.min(v, yMax);
        const x = padL + i * step;
        const y = padT + plotH - (clamped / yMax) * plotH;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });

      ctx.save();
      ctx.lineTo(padL + (historyData.length - 1) * step, padT + plotH);
      ctx.lineTo(padL, padT + plotH);
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();
      ctx.restore();

      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 2;
      ctx.stroke();

      historyData.forEach((v, i) => {
        if (v >= thresholdVal) {
          const clamped = Math.min(v, yMax);
          const x = padL + i * step;
          const y = padT + plotH - (clamped / yMax) * plotH;
          ctx.fillStyle = '#f43f5e';
          ctx.beginPath();
          ctx.arc(x, y, 3.5, 0, Math.PI * 2);
          ctx.fill();
        }
      });
    }

    async function fetchData() {
      if (paused) return;
      try {
        const r = await fetch('/data');
        const d = await r.json();

        document.getElementById('connDot').classList.remove('off');
        document.getElementById('connText').textContent = 'En Línea';
        document.getElementById('sdBadge').textContent = d.sd ? 'SD: Activa' : 'SD: Sin tarjeta';
        document.getElementById('uptime').textContent = d.uptime + 's';

        thresholdVal = d.threshold;
        historyData = d.history || [];

        document.getElementById('nivel').innerHTML = d.current + '<span class="kpi-unit">pts</span>';
        document.getElementById('umbralText').textContent = d.threshold;
        document.getElementById('maxWin').textContent = 'Máx: ' + Math.max(...historyData, 0);
        document.getElementById('peaks').textContent = d.peaks;
        document.getElementById('temp').textContent = d.temp.toFixed(1);

        document.getElementById('amag').textContent = d.amag.toFixed(2);
        const deltaG = Math.abs(d.amag - 1.0);
        document.getElementById('gDelta').textContent = 'Δ ' + deltaG.toFixed(2) + ' g';

        const vibCard = document.getElementById('vibCard');
        const vibState = document.getElementById('vibState');
        if (d.current >= d.threshold) {
          vibCard.classList.add('alert');
          vibState.classList.add('warn');
          vibState.textContent = '¡PICO ALTO!';
        } else {
          vibCard.classList.remove('alert');
          vibState.classList.remove('warn');
          vibState.textContent = 'NORMAL';
        }

        document.getElementById('axVal').textContent = d.ax.toFixed(2) + ' g';
        document.getElementById('ayVal').textContent = d.ay.toFixed(2) + ' g';
        document.getElementById('azVal').textContent = d.az.toFixed(2) + ' g';
        setBiBar('axBar', d.ax, 16.0);
        setBiBar('ayBar', d.ay, 16.0);
        setBiBar('azBar', d.az, 16.0);

        document.getElementById('gxVal').textContent = d.gx.toFixed(1) + ' °/s';
        document.getElementById('gyVal').textContent = d.gy.toFixed(1) + ' °/s';
        document.getElementById('gzVal').textContent = d.gz.toFixed(1) + ' °/s';
        setBiBar('gxBar', d.gx, 250.0);
        setBiBar('gyBar', d.gy, 250.0);
        setBiBar('gzBar', d.gz, 250.0);

        document.getElementById('pitchVal').textContent = d.pitch.toFixed(1) + '°';
        document.getElementById('rollVal').textContent = d.roll.toFixed(1) + '°';
        const horizon = document.getElementById('horizonLine');
        const pitchOffset = Math.max(-35, Math.min(35, d.pitch));
        horizon.style.transform = `translateY(${pitchOffset}px) rotate(${-d.roll}deg)`;

        drawChart();
      } catch (e) {
        document.getElementById('connDot').classList.add('off');
        document.getElementById('connText').textContent = 'Sin conexión';
      }
    }

    window.addEventListener('resize', drawChart);
    setInterval(fetchData, 200);
    fetchData();
  </script>
</body>
</html>
)rawhtml";
}