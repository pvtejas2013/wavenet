#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include <WiFi.h>
#include <esp_now.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include <esp_idf_version.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_err.h"
#include <math.h>

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// =====================================================
// -------------------- USER CONFIG --------------------
// =====================================================

//Wave1
// ---------- ESP-NOW peer (set each board to the OTHER board MAC) ----------
// uint8_t PEER_MAC[6] = { 0x00,0x4B,0x12,0xBE,0x39,0x10 };

//Wave2
// ---------- ESP-NOW peer (set each board to the OTHER board MAC) ----------
uint8_t PEER_MAC[6] = { 0x00,0x4B,0x12,0xBE,0x25,0x00 };

// ---------- PTT + Audio pins ----------
#define PIN_PTT       12
#define I2S_BCLK      26
#define I2S_LRCLK     25
#define I2S_DOUT      27   // change to 22 if your MAX98357 DIN is on GPIO22

// ---------- Audio settings ----------
#define SAMPLE_RATE   11025
#define FRAME_SAMPLES 220

// ADC (MAX4466 on GPIO36)
#define ADC_CHANNEL   ADC1_CHANNEL_0
#define ADC_ATTEN     ADC_ATTEN_DB_11

// TX auto-DC + gain
#define DC_ALPHA_SHIFT   6
#define DIGITAL_GAIN_Q   16

// RX playback gain
#define RX_PLAYBACK_GAIN 2

// Debug logging
#define LOG_EVERY_N   8
#define TEST_TONE_ON_BOOT 1

// ---------- Beacon target ----------
static const char* TARGET_MAC = "dd:34:02:0c:04:62";

// Distance model defaults
static const int   TX_POWER_AT_1M = -59;
static const float N_ENV = 2.7f;

// Timing
const uint32_t BEACON_OUT_MS  = 6000;   // out-of-reach threshold
const uint32_t VOICE_HOLD_MS  = 1200;   // disable beacon briefly after voice activity
const uint32_t SCAN_PERIOD_MS = 1200;   // run a 1s scan every 1.2s in idle

// =====================================================
// -------------------- OLED (SH1107) ------------------
// =====================================================
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// Beacon name
String beaconName = "UNKNOWN";
volatile bool beaconNameKnown = false;

// RSSI smoothing
volatile float rssi_ema = -100.0f;
const float alpha = 0.25f;

// Last distance + last seen time
volatile float    lastDist   = 0.0f;
volatile uint32_t lastSeenMs = 0;

// “Seen in this scan window” flag (key for stability)
volatile bool seenThisScan = false;

// Status string set ONLY in loop()
String beaconStatus = "----";

// Mode
enum Mode { MODE_BEACON, MODE_TALK, MODE_LISTEN };
volatile Mode g_mode = MODE_BEACON;

// Voice activity
volatile uint32_t lastVoiceRxMs = 0;
uint32_t lastVoiceTxMs = 0;

// BLE
BLEScan* pBLEScan = nullptr;
bool bleScanning = false;

// =====================================================
// -------------------- Helpers ------------------------
// =====================================================
static inline int clamp16(int x){ if (x>32767) return 32767; if (x<-32768) return -32768; return x; }
static inline int16_t sat16(int32_t x){ if(x>32767) return 32767; if(x<-32768) return -32768; return (int16_t)x; }

float estimateDistanceMeters(float rssi, int txPowerAt1m, float n) {
  return powf(10.0f, ((float)txPowerAt1m - rssi) / (10.0f * n));
}

// =====================================================
// -------------------- OLED Draw ----------------------
// =====================================================
void drawOLED() {
  static bool blink = false;
  static uint32_t lastBlink = 0;

  uint32_t now = millis();
  if (now - lastBlink > 400) {
    lastBlink = now;
    blink = !blink;
  }

  // Invert flashing only for OUT (but still show text always)
  bool invert = (g_mode == MODE_BEACON && beaconStatus == "OUT" && blink);
  display.invertDisplay(invert);

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println(beaconNameKnown ? beaconName : "Beacon");

  display.setCursor(0, 16);

  if (g_mode == MODE_TALK) {
    display.println("VOICE: TALKING");
    display.println("Beacon OFF");
  }
  else if (g_mode == MODE_LISTEN) {
    display.println("VOICE: LISTEN");
    display.println("Beacon OFF");
  }
  else {
    // Always show status text (no more blank second line)
    if (beaconStatus == "NEAR") display.println("STATUS: NEAR");
    else if (beaconStatus == "MID") display.println("STATUS: MID");
    else if (beaconStatus == "FAR") display.println("STATUS: FAR");
    else if (beaconStatus == "OUT") display.println("OUT OF REACH");
    else display.println("STATUS: ----");

    // Optional: show rough distance on 3rd line
    float d = (float)lastDist;
    display.setCursor(0, 32);
    display.print("d=");
    display.print(d, 2);
    display.println("m");
  }

  display.display();
}

// =====================================================
// -------------------- Beacon Callback ----------------
// =====================================================
class MyCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (g_mode != MODE_BEACON) return;

    String mac = dev.getAddress().toString().c_str();
    mac.toLowerCase();
    if (mac != String(TARGET_MAC)) return;

    // Mark seen for this scan
    seenThisScan = true;

    // Name
    if (dev.haveName()) {
      String newName = dev.getName();
      if (newName.length() > 0 && newName != beaconName) {
        beaconName = newName;
        beaconNameKnown = true;
        Serial.print("Beacon name updated: ");
        Serial.println(beaconName);
      }
    }

    int rssi = dev.getRSSI();

    float ema = rssi_ema;
    if (ema < -99.0f) ema = (float)rssi;
    ema = alpha * (float)rssi + (1.0f - alpha) * ema;
    rssi_ema = ema;

    float dist = estimateDistanceMeters(ema, TX_POWER_AT_1M, N_ENV);
    lastDist = dist;

    lastSeenMs = (uint32_t)millis();

    Serial.printf("Beacon %s (%s) RSSI=%d sm=%.1f est=%.2fm\n",
                  TARGET_MAC, beaconName.c_str(), rssi, ema, dist);
  }
};

// BLE scan helpers
void stopBeaconScan() {
  if (!pBLEScan) return;
  if (bleScanning) {
    pBLEScan->stop();
    pBLEScan->clearResults();
    bleScanning = false;
  }
}

void runBeaconScanChunk_1s() {
  if (!pBLEScan) return;
  if (g_mode != MODE_BEACON) return;

  // Reset "seen this scan" flag BEFORE scanning
  seenThisScan = false;

  bleScanning = true;
  pBLEScan->start(1, false);
  pBLEScan->clearResults();
  bleScanning = false;
}

// =====================================================
// ---------------- ADPCM tables -----------------------
// =====================================================
static const int indexTable[16] = {
  -1,-1,-1,-1, 2,4,6,8, -1,-1,-1,-1, 2,4,6,8
};
static const int stepSizeTable[89] = {
  7,8,9,10,11,12,13,14,16,17, 19,21,23,25,28,31,34,37,41,45,
  50,55,60,66,73,80,88,97,107,118, 130,143,157,173,190,209,230,253,279,307,
  337,371,408,449,494,544,598,658,724,796, 876,963,1060,1166,1282,1411,1552,1707,1878,2066,
  2272,2499,2749,3024,3327,3660,4026,4428,4871,5358, 5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
  15289,16818,18500,20350,22385,24623,27086,29794,32767
};

typedef struct __attribute__((packed)) {
  uint16_t seq;
  uint8_t  pred_lo;
  uint8_t  pred_hi;
  uint8_t  index;
  uint8_t  payload[FRAME_SAMPLES/2];
} VoicePkt;

volatile uint16_t g_seq = 0;
static int16_t pcm_in[FRAME_SAMPLES];
static int16_t pcm_out[FRAME_SAMPLES];

static int32_t dc_est = 2048;
static int16_t hpf_prev_x = 0;
static int32_t hpf_prev_y = 0;

static inline int16_t hpf_dc_block(int16_t x){
  int32_t dx = (int32_t)x - (int32_t)hpf_prev_x;
  int32_t y  = dx + ((hpf_prev_y * 32640) >> 15);
  hpf_prev_x = x;
  hpf_prev_y = y;
  return sat16(y);
}

void adpcm_encode_block(int16_t *pcm, uint8_t *out, int &pred, int &index){
  int step = stepSizeTable[index];
  bool high = true;
  uint8_t byte = 0;
  int outPos = 0;

  for (int i=0;i<FRAME_SAMPLES;i++){
    int diff = pcm[i] - pred;
    int sign = (diff < 0) ? 8 : 0; if (diff < 0) diff = -diff;
    int delta = 0; int t = step;
    if (diff >= t){ delta|=4; diff-=t; } t >>= 1;
    if (diff >= t){ delta|=2; diff-=t; } t >>= 1;
    if (diff >= t){ delta|=1; }

    int diffq = step >> 3;
    if (delta & 4) diffq += step;
    if (delta & 2) diffq += step >> 1;
    if (delta & 1) diffq += step >> 2;

    pred += (sign ? -diffq : diffq);
    pred = clamp16(pred);

    index += indexTable[delta | sign];
    if (index < 0) index = 0; else if (index > 88) index = 88;
    step = stepSizeTable[index];

    uint8_t nib = (delta | sign) & 0x0F;
    if (high){ byte = nib << 4; high = false; }
    else { byte |= nib; out[outPos++] = byte; high = true; byte = 0; }
  }
}

void adpcm_decode_block(const uint8_t *in, int16_t *pcm, int &pred, int &index){
  int step = stepSizeTable[index];
  for (int i=0;i<FRAME_SAMPLES/2;i++){
    uint8_t b = in[i];
    for (int k=0;k<2;k++){
      uint8_t code = (k==0)? ((b>>4)&0x0F) : (b&0x0F);
      int sign = code & 8;
      int delta = code & 7;

      int diffq = step >> 3;
      if (delta & 4) diffq += step;
      if (delta & 2) diffq += step >> 1;
      if (delta & 1) diffq += step >> 2;

      pred += (sign ? -diffq : diffq);
      pred = clamp16(pred);

      index += indexTable[code];
      if (index < 0) index = 0; else if (index > 88) index = 88;
      step = stepSizeTable[index];

      *pcm++ = (int16_t)pred;
    }
  }
}

// I2S
void setup_i2s_tx(){
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = FRAME_SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num   = I2S_BCLK,
    .ws_io_num    = I2S_LRCLK,
    .data_out_num = I2S_DOUT,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };
  ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pins));
  ESP_ERROR_CHECK(i2s_set_clk(I2S_NUM_1, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO));
}

// Test tone
void play_test_tone_ms(uint32_t ms){
  const float f = 1000.0f;
  const float w = 2.0f * 3.14159265f * f / (float)SAMPLE_RATE;
  uint32_t total = (uint32_t)((ms/1000.0f) * SAMPLE_RATE);
  static int16_t stereo[FRAME_SAMPLES*2];
  for (uint32_t n=0; n<total; ){
    uint32_t chunk = min((uint32_t)FRAME_SAMPLES, total - n);
    for (uint32_t i=0;i<chunk;i++){
      float s = sinf(w * (float)(n+i));
      int16_t v = (int16_t)(s * 12000);
      stereo[2*i+0] = v;
      stereo[2*i+1] = v;
    }
    size_t wr=0; i2s_write(I2S_NUM_1, (const char*)stereo, chunk*4, &wr, portMAX_DELAY);
    n += chunk;
  }
}

// ESP-NOW callbacks
static int enc_pred = 0, enc_index = 0;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
static void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(VoicePkt)) return;
  const VoicePkt* pkt = (const VoicePkt*)incomingData;

  lastVoiceRxMs = millis();

  int p   = (int)((pkt->pred_hi<<8) | pkt->pred_lo);
  int idx = pkt->index;

  adpcm_decode_block(pkt->payload, pcm_out, p, idx);

  static int16_t stereo[FRAME_SAMPLES * 2];
  for (int i=0, j=0; i<FRAME_SAMPLES; i++, j+=2){
    int16_t s = hpf_dc_block(pcm_out[i]);
    int16_t g = sat16((int32_t)s * RX_PLAYBACK_GAIN);
    stereo[j+0] = g; stereo[j+1] = g;
  }

  size_t wr=0; i2s_write(I2S_NUM_1, (const char*)stereo, FRAME_SAMPLES*4, &wr, 0);
}
static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) { (void)tx_info; (void)status; }
#else
static void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(VoicePkt)) return;
  const VoicePkt* pkt = (const VoicePkt*)incomingData;

  lastVoiceRxMs = millis();

  int p   = (int)((pkt->pred_hi<<8) | pkt->pred_lo);
  int idx = pkt->index;

  adpcm_decode_block(pkt->payload, pcm_out, p, idx);

  static int16_t stereo[FRAME_SAMPLES * 2];
  for (int i=0, j=0; i<FRAME_SAMPLES; i++, j+=2){
    int16_t s = hpf_dc_block(pcm_out[i]);
    int16_t g = sat16((int32_t)s * RX_PLAYBACK_GAIN);
    stereo[j+0] = g; stereo[j+1] = g;
  }

  size_t wr=0; i2s_write(I2S_NUM_1, (const char*)stereo, FRAME_SAMPLES*4, &wr, 0);
}
static void onDataSent(const uint8_t *mac, esp_now_send_status_t status) { (void)mac; (void)status; }
#endif

void setup_espnow(){
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK){
    Serial.println("ESP-NOW init failed!");
    while(1) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  esp_err_t e = esp_now_add_peer(&peer);
  if (e != ESP_OK) Serial.printf("Peer add failed (err=%d)\n", (int)e);
  else Serial.printf("Peer added: %02X:%02X:%02X:%02X:%02X:%02X\n",
                     PEER_MAC[0],PEER_MAC[1],PEER_MAC[2],PEER_MAC[3],PEER_MAC[4],PEER_MAC[5]);
}

void setup_adc(){
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
}

void print_macs(){
  uint8_t mac[6];
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
#else
  esp_read_mac(mac, ESP_IF_WIFI_STA);
#endif
  Serial.printf("STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.print("WiFi.macAddress(): "); Serial.println(WiFi.macAddress());
}

// Sample frame
static inline void sample_frame_adc11k(int16_t *dst){
  const uint32_t TICK_US = 1000000UL / SAMPLE_RATE;
  uint32_t t = micros();

  for (int i=0;i<FRAME_SAMPLES;i++){
    uint32_t next_t = t + TICK_US;
    while ((int32_t)(micros() - next_t) < 0) {}

    int raw12 = adc1_get_raw(ADC_CHANNEL);
    if (raw12 < 0) raw12 = 0; if (raw12 > 4095) raw12 = 4095;

    dc_est += ((int32_t)raw12 - dc_est) >> DC_ALPHA_SHIFT;

    int32_t centered  = (int32_t)raw12 - dc_est;
    int32_t amplified = centered * DIGITAL_GAIN_Q;
    dst[i] = sat16(amplified);

    t = next_t;
  }
}

// Send frame
static inline void send_frame_over_now(int16_t *pcm){
  VoicePkt pkt;
  pkt.seq = g_seq++;

  int headerPred = enc_pred;
  int headerIdx  = enc_index;

  adpcm_encode_block(pcm, pkt.payload, enc_pred, enc_index);

  pkt.pred_lo = (uint8_t)(headerPred & 0xFF);
  pkt.pred_hi = (uint8_t)((headerPred >> 8) & 0xFF);
  pkt.index   = (uint8_t)headerIdx;

  esp_now_send(PEER_MAC, (const uint8_t*)&pkt, sizeof(pkt));
}

// =====================================================
// -------------------- Setup --------------------------
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_PTT, INPUT_PULLUP);

  // OLED
  Wire.begin();
  if (!display.begin(0x3C, true)) {
    Serial.println("SH1107 OLED not found at 0x3C");
    while (1) delay(10);
  }
  display.setRotation(1);

  // Audio + ESP-NOW
  print_macs();
  setup_adc();
  setup_i2s_tx();
  setup_espnow();

#if TEST_TONE_ON_BOOT
  if (digitalRead(PIN_PTT) == LOW) {
    Serial.println("PTT held at boot -> 1 kHz test tone 1s");
    play_test_tone_ms(1000);
  }
#endif

  // BLE
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(160);
  pBLEScan->setWindow(120);

  beaconStatus = "----";
  drawOLED();

  Serial.println("Merged Walkie + Beacon started.");
  Serial.println("Hold PTT to TALK. Release to LISTEN. Beacon runs only when idle.");
}

// =====================================================
// -------------------- Main Loop ----------------------
// =====================================================
void loop() {
  uint32_t now = millis();

  bool localTalking = (digitalRead(PIN_PTT) == LOW);
  bool recentRx = (now - (uint32_t)lastVoiceRxMs) < VOICE_HOLD_MS;
  bool recentTx = (now - lastVoiceTxMs) < VOICE_HOLD_MS;

  if (localTalking) g_mode = MODE_TALK;
  else if (recentRx || recentTx) g_mode = MODE_LISTEN;
  else g_mode = MODE_BEACON;

  if (g_mode != MODE_BEACON) stopBeaconScan();

  // TALK
  if (g_mode == MODE_TALK) {
    sample_frame_adc11k(pcm_in);
    send_frame_over_now(pcm_in);
    lastVoiceTxMs = now;
  }

  // BEACON
  static uint32_t lastScanMs = 0;
  if (g_mode == MODE_BEACON) {
    if (now - lastScanMs >= SCAN_PERIOD_MS) {
      lastScanMs = now;
      runBeaconScanChunk_1s(); // blocks 1s
    }

    // Decide status AFTER scan using snapshots
    uint32_t seen = (uint32_t)lastSeenMs;
    float dist = (float)lastDist;
    bool seenNow = (bool)seenThisScan;

    if (seen == 0) {
      beaconStatus = "----";
    } else if (!seenNow && (now - seen) > BEACON_OUT_MS) {
      beaconStatus = "OUT";
    } else {
      if (dist < 1.5f) beaconStatus = "NEAR";
      else if (dist < 5.0f) beaconStatus = "MID";
      else beaconStatus = "FAR";
    }
  }

  // OLED update
  static uint32_t lastDraw = 0;
  if (now - lastDraw > 250) {
    lastDraw = now;
    drawOLED();
  }

  delay(1);
}
