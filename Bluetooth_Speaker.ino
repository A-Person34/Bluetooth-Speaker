#include "ESP_I2S.h"
#include "BluetoothA2DPSink.h"
#include "BluetoothSerial.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

// =====================================================
//                     PINS
// =====================================================

const uint8_t I2S_SCK = 26;
const uint8_t I2S_WS = 25;
const uint8_t I2S_SDOUT = 17;

const uint8_t PIN_VOL_UP = 27;
const uint8_t PIN_VOL_DOWN = 33;
const uint8_t PIN_MEDIA_BTN = 32;

const uint8_t PIN_BATTERY = 34;

// =====================================================
//                   GLOBALS
// =====================================================

I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);
BluetoothSerial SerialBT;

int volume = 90;
const int VOLUME_STEP = 8;

unsigned long lastVolUpChange = 0;
unsigned long lastVolDownChange = 0;
const unsigned long DEBOUNCE_MS = 40;
const unsigned long HOLD_REPEAT_MS = 150;

bool mediaBtnPrev = false;
unsigned long mediaBtnLastChange = 0;
unsigned long mediaBtnLastRelease = 0;
int mediaPressCount = 0;
const unsigned long MULTI_PRESS_TIMEOUT = 350;

float batteryVoltage = 0.0f;
const float BATT_ALPHA = 0.12f;
const float ADC_MULT = 4.3f;

bool isPlaying = true;

// =====================================================
//              BLUETOOTH RESET
// =====================================================

void clearBluetoothBonds() {
  Serial.println("Clearing Bluetooth bonds...");
  SerialBT.begin("BT_Reset");
  SerialBT.deleteAllBondedDevices();
  SerialBT.end();
  delay(500);
}

// =====================================================
//                UTILITY
// =====================================================

void togglePlayPause() {
  if (isPlaying) a2dp_sink.pause();
  else a2dp_sink.play();
  isPlaying = !isPlaying;
}

float adcToVoltage(int raw) {
  return ((raw / 4095.0f) * 3.3f) * ADC_MULT;
}

void setVolumeClamped(int v) {
  volume = constrain(v, 0, 127);
  a2dp_sink.set_volume(volume);
}

// =====================================================
//                 MEDIA ACTIONS
// =====================================================

void handleMediaButtonAction(int count) {
  if (count == 1) togglePlayPause();
  else if (count == 2) a2dp_sink.next();
  else if (count == 3) a2dp_sink.previous();
}

// =====================================================
//                        SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_VOL_UP, INPUT_PULLUP);
  pinMode(PIN_VOL_DOWN, INPUT_PULLUP);
  pinMode(PIN_MEDIA_BTN, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);

  // I2S
  i2s.setPins(I2S_SCK, I2S_WS, I2S_SDOUT);
  i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);

  // Factory Reset
  unsigned long bootStart = millis();
  bool resetRequested = false;

  while (millis() - bootStart < 5000) {
    if (digitalRead(PIN_MEDIA_BTN) == LOW) {
      resetRequested = true;
      break;
    }
  }

  if (resetRequested) {
    clearBluetoothBonds();
  }

  // Windows 11 Fix
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  esp_bt_dev_set_device_name("T-Money Speaker");

  a2dp_sink.set_auto_reconnect(false);

  esp_bt_gap_set_scan_mode(
    ESP_BT_CONNECTABLE,
    ESP_BT_GENERAL_DISCOVERABLE
  );

  // Start A2DP
  a2dp_sink.start("T-Money_Speaker");
  setVolumeClamped(volume);

  Serial.println("READY FOR WINDOWS 11");
}

// =====================================================
//                        LOOP
// =====================================================

void loop() {
  unsigned long now = millis();

  bool pressed = (digitalRead(PIN_MEDIA_BTN) == LOW);

  if (pressed != mediaBtnPrev && (now - mediaBtnLastChange) > DEBOUNCE_MS) {
    mediaBtnLastChange = now;
    mediaBtnPrev = pressed;
    if (!pressed) {
      mediaPressCount++;
      mediaBtnLastRelease = now;
    }
  }

  if (mediaPressCount > 0 && (now - mediaBtnLastRelease) > MULTI_PRESS_TIMEOUT) {
    handleMediaButtonAction(mediaPressCount);
    mediaPressCount = 0;
  }

  static bool volUpPrev = false;
  bool volUpNow = (digitalRead(PIN_VOL_UP) == LOW);

  if (volUpNow && !volUpPrev) {
    lastVolUpChange = now;
    setVolumeClamped(volume + VOLUME_STEP);
  }
  if (volUpNow && volUpPrev && (now - lastVolUpChange > HOLD_REPEAT_MS)) {
    setVolumeClamped(volume + 2);
    lastVolUpChange = now;
  }
  volUpPrev = volUpNow;

  static bool volDownPrev = false;
  bool volDownNow = (digitalRead(PIN_VOL_DOWN) == LOW);

  if (volDownNow && !volDownPrev) {
    lastVolDownChange = now;
    setVolumeClamped(volume - VOLUME_STEP);
  }
  if (volDownNow && volDownPrev && (now - lastVolDownChange > HOLD_REPEAT_MS)) {
    setVolumeClamped(volume - 2);
    lastVolDownChange = now;
  }
  volDownPrev = volDownNow;

  delay(8);
}
