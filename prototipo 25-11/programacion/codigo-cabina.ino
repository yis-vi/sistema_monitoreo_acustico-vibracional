#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
// Controlador de la pantalla: true = SH1106 (el que funcionó), false = SSD1306
#define USAR_SH1106 true
#if USAR_SH1106
  #include <Adafruit_SH110X.h>
#else
  #include <Adafruit_SSD1306.h>
#endif

#define OLED_SDA       21
#define OLED_SCL       22
#define PIN_BUZZER     26
// Buzzer pasivo (módulo MH-FMD): se hace sonar con tone()
#define BUZZER_FREQ    2000   // Hz
#define BUZZER_OFF     HIGH   // reposo: nivel alto
#define PIN_LED        25

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

// MAC de la SENTINA (38:3E:51:D4:72:48)
uint8_t SENTINA_MAC[] = {0x38, 0x3E, 0x51, 0xD4, 0x72, 0x48};

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint16_t noise_level;
    uint8_t  peak_event;
    uint8_t  tipo;
} EspNowPacket;

#define PEAK_THRESHOLD    2000
#define OLED_GRAPH_MAX    4095
#define ALERT_DURATION_MS 1000

#define GRAPH_POINTS  OLED_WIDTH
uint16_t graphBuf[GRAPH_POINTS];
uint8_t  graphIdx = 0;

volatile uint16_t latestNoise       = 0;
volatile bool     newPktFromSentina = false;
EspNowPacket      pktFromSentina;

uint32_t alertStartMs = 0;
uint32_t lastPktMs    = 0;
bool     alertActive  = false;

uint32_t lastOledMs   = 0;
#define  INTERVAL_OLED  100

void initOled();
void initEspNow();
void taskOled();
void taskAlert();
void processPktFromSentina();
void sendEventToCentral(const EspNowPacket& pkt);
void onDataSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status);
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);

void setup() {
    Serial.begin(115200);

    pinMode(PIN_LED,    OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED,    LOW);
    digitalWrite(PIN_BUZZER, BUZZER_OFF);

    Wire.begin(OLED_SDA, OLED_SCL);
    initOled();

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(OLED_WHITE);
    oled.setCursor(0, 0);
    oled.println("Iniciando Cabina...");
    oled.display();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    initEspNow();

    Serial.println("[Cabina] Listo.");
    Serial.print("[Cabina] MAC: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
    uint32_t now = millis();

    if (newPktFromSentina) {
        newPktFromSentina = false;
        processPktFromSentina();
    }

    if (now - lastOledMs >= INTERVAL_OLED) {
        lastOledMs = now;
        taskOled();
    }

    taskAlert();
}

void initOled() {
    if (!OLED_BEGIN()) {
        Serial.println("[OLED] No encontrada — continuando sin pantalla.");
    }
    oled.clearDisplay();
    oled.display();
}

void initEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Error al inicializar.");
        return;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, SENTINA_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Error al agregar peer Sentina.");
    }
}

void processPktFromSentina() {
    if (pktFromSentina.tipo != 0) return;

    lastPktMs = millis();

    latestNoise = pktFromSentina.noise_level;

    graphBuf[graphIdx] = latestNoise;
    graphIdx = (graphIdx + 1) % GRAPH_POINTS;

    if (latestNoise >= PEAK_THRESHOLD && !alertActive) {
        alertActive  = true;
        alertStartMs = millis();
        digitalWrite(PIN_LED,    HIGH);
        tone(PIN_BUZZER, BUZZER_FREQ);
        Serial.print("[Cabina] ¡PICO DETECTADO! Nivel=");
        Serial.println(latestNoise);

        EspNowPacket evt;
        evt.timestamp_ms = pktFromSentina.timestamp_ms;
        evt.noise_level  = latestNoise;
        evt.peak_event   = 1;
        evt.tipo         = 1;
        sendEventToCentral(evt);
    } else if (alertActive && latestNoise >= PEAK_THRESHOLD) {
        alertStartMs = millis();   // sigue en alerta mientras la vibración continúe alta
    }
}

void taskAlert() {
    if (alertActive && (millis() - alertStartMs >= ALERT_DURATION_MS)) {
        alertActive = false;
        digitalWrite(PIN_LED,    LOW);
        noTone(PIN_BUZZER);
        digitalWrite(PIN_BUZZER, BUZZER_OFF);
    }
}

void textoCentrado(const char* t, uint8_t tam, int y) {
    oled.setTextSize(tam);
    oled.setCursor((OLED_WIDTH - (int)strlen(t) * 6 * tam) / 2, y);
    oled.print(t);
}

void taskOled() {
    uint32_t now = millis();
    uint16_t nivel = latestNoise;
    bool conectado = (lastPktMs != 0) && (now - lastPktMs < 1000);
    const int escala = PEAK_THRESHOLD + PEAK_THRESHOLD / 4;   // el umbral queda al 80 % de la escala

    oled.clearDisplay();

    // ---------- PANTALLA DE ALERTA (parpadea) ----------
    if (alertActive) {
        bool inv = ((now / 250) % 2) == 0;
        if (inv) oled.fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, OLED_WHITE);
        oled.setTextColor(inv ? OLED_BLACK : OLED_WHITE);

        textoCentrado("RUIDO", 2, 3);
        textoCentrado("CRITICO", 2, 21);
        textoCentrado("REDUZCA VELOCIDAD", 1, 43);

        char buf[20];
        snprintf(buf, sizeof(buf), "Vibr: %u", nivel);
        textoCentrado(buf, 1, 54);

        oled.display();
        return;
    }

    // ---------- PANTALLA NORMAL ----------
    oled.setTextColor(OLED_WHITE);

    // Cabecera
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("VIBRACION");
    oled.setCursor(conectado ? 116 : 74, 0);
    oled.print(conectado ? "OK" : "SIN SENAL");
    oled.drawFastHLine(0, 9, OLED_WIDTH, OLED_WHITE);

    // Valor grande
    oled.setTextSize(3);
    oled.setCursor(0, 13);
    oled.print(nivel);

    // Panel derecho: umbral y porcentaje del umbral
    int pct = (int)((uint32_t)nivel * 100 / PEAK_THRESHOLD);
    if (pct > 999) pct = 999;
    oled.setTextSize(1);
    oled.setCursor(84, 13);
    oled.print("UMBRAL");
    oled.setCursor(84, 23);
    oled.print(PEAK_THRESHOLD);
    oled.setCursor(84, 33);
    oled.print(pct);
    oled.print("%");

    // Barra de nivel con marca del umbral
    const int barY = 41, barH = 9;
    oled.drawRect(0, barY, OLED_WIDTH, barH, OLED_WHITE);
    int lleno = map(constrain((int)nivel, 0, escala), 0, escala, 0, OLED_WIDTH - 4);
    oled.fillRect(2, barY + 2, lleno, barH - 4, OLED_WHITE);
    int marca = 2 + map(PEAK_THRESHOLD, 0, escala, 0, OLED_WIDTH - 4);
    oled.drawFastVLine(marca, barY - 2, barH + 4, OLED_WHITE);

    // Historial (últimos ~13 s) con línea punteada del umbral
    const int gH = 10;
    int umbralY = 64 - map(PEAK_THRESHOLD, 0, escala, 0, gH);
    for (int x = 0; x < OLED_WIDTH; x += 4) {
        oled.drawPixel(x, umbralY, OLED_WHITE);
    }
    for (int x = 0; x < GRAPH_POINTS; x++) {
        int idx = (graphIdx + x) % GRAPH_POINTS;
        int h = map(constrain((int)graphBuf[idx], 0, escala), 0, escala, 0, gH);
        if (h > 0) oled.drawFastVLine(x, 64 - h, h, OLED_WHITE);
    }

    oled.display();
}

void sendEventToCentral(const EspNowPacket& pkt) {
    esp_err_t result = esp_now_send(SENTINA_MAC, (uint8_t*)&pkt, sizeof(pkt));
    if (result != ESP_OK) {
        Serial.println("[ESP-NOW] Error al enviar evento a Sentina.");
    }
}

void onDataSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[ESP-NOW] Envío fallido.");
    }
}

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len == sizeof(EspNowPacket)) {
        memcpy(&pktFromSentina, data, sizeof(EspNowPacket));
        newPktFromSentina = true;
    }
}
