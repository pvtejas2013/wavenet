Below is a ready-to-paste **README.md** for your WaveNet project (ESP32 Walkie-Talkie + BLE Beacon tracker). Edit the names/MACs/pin notes as needed.

````markdown
# WaveNet (ESP32) — Walkie-Talkie + BLE Beacon Tracker (OLED)

WaveNet is a two-ESP32 communication project that combines:
- **Push-To-Talk voice** between two ESP32 boards using **ESP-NOW**
- **BLE beacon proximity tracking** (RSSI → estimated distance)
- **OLED status display** (SH1107 128×64) showing voice mode + beacon status

When you **hold the PTT button**, the device transmits voice.
When you **release PTT**, it listens for incoming voice.
When **idle**, it scans for a BLE beacon and shows **NEAR / MID / FAR / OUT** on the OLED.

---

## Features

### 🎙️ Voice Walkie-Talkie (ESP-NOW)
- Audio captured from an analog mic (ex: **MAX4466**) using ESP32 **ADC**
- Encoded using **IMA ADPCM (4-bit)** to reduce bandwidth
- Sent over **ESP-NOW** to a paired ESP32 (by MAC address)
- Decoded and played using **I2S DAC amp** (ex: **MAX98357**)

### 📡 BLE Beacon Tracking
- Scans for one target beacon MAC address (configurable)
- Smooths RSSI using an **EMA filter**
- Converts RSSI to rough distance (log-distance path loss model)
- Classifies distance into:
  - **NEAR** ( < 1.5 m )
  - **MID**  ( < 5.0 m )
  - **FAR**  ( ≥ 5.0 m )
  - **OUT** if not seen for a time threshold

### 🖥️ OLED Display (SH1107)
- Shows beacon name (if available)
- Shows mode:
  - `VOICE: TALKING`
  - `VOICE: LISTEN`
  - Beacon status during idle
- **OUT OF REACH** flashes (invert display) while still showing text

---

## Hardware Required

Per device (2 devices total recommended):
- **ESP32 dev board** (WiFi + BLE)
- **Analog mic module** (MAX4466 recommended)
- **I2S speaker amplifier** (MAX98357 I2S amp recommended)
- **Speaker** (4–8Ω small speaker)
- **PTT push button**
- **SH1107 OLED 128×64** (I2C, address usually `0x3C`)
- Jumper wires / breadboard or PCB

---

## Wiring

### PTT Button
- One side → **GPIO 12** (`PIN_PTT`)
- Other side → **GND**
- Code uses `INPUT_PULLUP`, so **pressed = LOW**

### Microphone (MAX4466 → ESP32 ADC)
- `VCC` → 3.3V
- `GND` → GND
- `OUT` → **GPIO 36** (ADC1_CH0)

### I2S Amp (MAX98357)
- `VIN` → 3.3V or 5V (depends on your module; many work at 3.3V)
- `GND` → GND
- `BCLK` → **GPIO 26**
- `LRC`  → **GPIO 25**
- `DIN`  → **GPIO 27** *(changeable; see config note below)*
- Speaker → Amp `+ / -`

### OLED (SH1107 I2C)
- `VCC` → 3.3V
- `GND` → GND
- `SDA` → ESP32 SDA (default varies by board)
- `SCL` → ESP32 SCL (default varies by board)

> If your ESP32 board uses non-default I2C pins, set them in `Wire.begin(SDA, SCL)`.

---

## Software / Libraries

This sketch uses Arduino framework with these libraries:
- `WiFi.h` + `esp_now.h`
- `BLEDevice.h` (ESP32 BLE)
- `Adafruit_GFX`
- `Adafruit_SH110X` (for SH1107 OLED)
- ESP-IDF drivers included via Arduino:
  - `driver/i2s.h`
  - `driver/adc.h`

Install via Arduino Library Manager:
- **Adafruit GFX Library**
- **Adafruit SH110X**

Board package:
- **ESP32 by Espressif Systems**

---

## Configuration (Edit These)

At the top of the code:

### 1) ESP-NOW Peer MAC
Each board must be configured to send to the **other board’s STA MAC**.

```cpp
uint8_t PEER_MAC[6] = { 0x00,0x4B,0x12,0xBE,0x25,0x00 };
````

To find your MAC, open Serial Monitor on each board:

* Look for:

  * `STA MAC: xx:xx:xx:xx:xx:xx`

Then set Board A’s `PEER_MAC` = Board B’s MAC, and vice versa.

### 2) BLE Beacon Target MAC

Set the BLE beacon MAC string:

```cpp
static const char* TARGET_MAC = "dd:34:02:0c:04:62";
```

### 3) I2S DIN Pin (if needed)

If your MAX98357 `DIN` is wired to GPIO22 instead of 27:

```cpp
#define I2S_DOUT 22
```

---

## How It Works (Modes)

WaveNet has 3 modes:

1. **MODE_TALK**

   * PTT pressed (GPIO12 LOW)
   * Samples mic audio → ADPCM encode → ESP-NOW send
   * Beacon scanning stops

2. **MODE_LISTEN**

   * Not talking, but voice was recently sent/received
   * Beacon scanning stops briefly to avoid glitches

3. **MODE_BEACON** (Idle)

   * Runs BLE scan chunks (1 second scan every ~1.2 seconds)
   * Updates status + distance on OLED

---

## OLED Status Meanings

During Beacon mode:

* `STATUS: NEAR` — estimated distance < 1.5 m
* `STATUS: MID`  — estimated distance < 5.0 m
* `STATUS: FAR`  — estimated distance ≥ 5.0 m
* `OUT OF REACH` — beacon not detected within timeout window
* `STATUS: ----` — not enough data yet / never seen

The OLED also prints:

* `d= X.XX m` (rough estimate)

---

## Test Tone (Quick Speaker Check)

If `TEST_TONE_ON_BOOT` is enabled:

* **Hold PTT while powering/resetting**
* It plays a **1 kHz tone for 1 second**
* Confirms I2S amp + speaker wiring works

```cpp
#define TEST_TONE_ON_BOOT 1
```

---

## Tuning Tips

### Distance estimate not accurate?

RSSI distance is always approximate. You can adjust:

```cpp
static const int   TX_POWER_AT_1M = -59;
static const float N_ENV = 2.7f;
```

* `TX_POWER_AT_1M`: calibrate for your beacon (try -55 to -70)
* `N_ENV`: environment factor (2.0 open space, 2.7–4.0 indoors)

### Voice too quiet / too loud?

* Transmit digital gain (mic side):

  ```cpp
  #define DIGITAL_GAIN_Q 16
  ```
* Playback gain (speaker side):

  ```cpp
  #define RX_PLAYBACK_GAIN 2
  ```

---

## Troubleshooting

### OLED not found

* Ensure address is correct (common: `0x3C`)
* Check SDA/SCL wiring and your board’s I2C pins
* Confirm 3.3V power

### ESP-NOW not receiving

* Make sure both boards:

  * Are in `WIFI_STA` mode (code does this)
  * Use correct **peer MAC** (the other board)
* Keep both boards close for initial testing

### Beacon always OUT

* Confirm `TARGET_MAC` exactly matches the beacon MAC (lower/upper case doesn’t matter; code converts to lower)
* Beacon may randomize addresses (some devices do). Use a real beacon or one with fixed MAC.

### Voice works but beacon freezes

* Beacon scanning is intentionally disabled during TALK/LISTEN modes.
* After VOICE activity it waits `VOICE_HOLD_MS` before scanning again.

---

## Future Improvements (Ideas)

* Add **multi-peer broadcast** (one talker → multiple listeners)
* Add **encryption** for ESP-NOW
* Replace ADC mic with true I2S mic (better quality)
* Show battery voltage on OLED
* Add “channel” selection / pairing UI

---

## License

Choose a license if you want to share publicly (MIT is common). Otherwise remove this section.

---

## Credits

Built with ESP32 Arduino + Adafruit display libraries. Uses IMA ADPCM for low-bandwidth voice over ESP-NOW.

```

If you want, paste your repo folder structure (or tell me if you use Arduino IDE vs PlatformIO), and I’ll tailor the README to include **exact wiring table**, **build steps**, and a **pinout diagram** matching your specific ESP32 board (Feather V2, DevKit, etc.).
```
