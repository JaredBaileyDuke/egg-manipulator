#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <WiFi.h>
#include <esp_now.h>

#define SENSOR_COUNT 12
#define WINDOW_SIZE   5      // rolling median window
#define LED_PIN       2      // on‑board LED

// VL53L0X instances
Adafruit_VL53L0X sensors[SENSOR_COUNT];

// XSHUT pins for each sensor
const uint8_t xshutPins[SENSOR_COUNT] = {
   13, 32, 14, 27, 26, 25,  // LHTA, LHTB, LHI, LHM, LHR, LHP
    4, 23, 19, 18, 15, 33   // RHTA, RHTB, RHI, RHM, RHR, RHP
};

// I²C addresses for each sensor
const uint8_t sensorAddresses[SENSOR_COUNT] = {
  0x30,0x31,0x32,0x33,
  0x34,0x35,0x36,0x37,
  0x38,0x39,0x3A,0x3B
};

// Finger labels
const char* fingerNames[SENSOR_COUNT] = {
  "LHTA","LHTB","LHI","LHM","LHR","LHP",
  "RHTA","RHTB","RHI","RHM","RHR","RHP"
};

// Valid distance windows per finger
const uint16_t minRange[SENSOR_COUNT] = {
   89,  87,  86,  84,  95,  93,
   86,  99,  89,  88,  85, 107
};
const uint16_t maxRange[SENSOR_COUNT] = {
   99,  97,  96,  94, 105, 103,
   96, 109,  99,  98,  95, 117
};

bool sensorReady[SENSOR_COUNT];
uint16_t buffer[SENSOR_COUNT][WINDOW_SIZE];
uint8_t  bufIndex = 0;

// LED blink
unsigned long blinkInterval = 1000;
unsigned long lastBlink     = 0;
bool          ledState      = LOW;

// ESP‑NOW peer (robot) MAC address
uint8_t peerMAC[6] = { 0xCC, 0xDB, 0xA7, 0x92, 0xCE, 0x20 };

// Called when a packet is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Send to %02X:%02X:%02X:%02X:%02X:%02X %s\n",
    mac_addr[0],mac_addr[1],mac_addr[2],
    mac_addr[3],mac_addr[4],mac_addr[5],
    status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// Compute median of WINDOW_SIZE values
uint16_t medianOf(uint16_t *arr) {
  uint16_t tmp[WINDOW_SIZE];
  memcpy(tmp, arr, WINDOW_SIZE * sizeof(uint16_t));
  for (int i = 1; i < WINDOW_SIZE; i++) {
    uint16_t key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j+1] = tmp[j];
      j--;
    }
    tmp[j+1] = key;
  }
  return tmp[WINDOW_SIZE/2];
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);         // SDA = 21, SCL = 22
  Wire.setClock(100000);      // 100 kHz I²C
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(100);

  // Shutdown all sensors & init buffers
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
    sensorReady[i] = false;
    for (int w = 0; w < WINDOW_SIZE; w++) {
      buffer[i][w] = 0xFFFF;  // sentinel = out of range
    }
  }
  delay(100);

  // Initialize each VL53L0X
  for (int i = 0; i < SENSOR_COUNT; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(100);
    if (!sensors[i].begin(0x29, &Wire)) {
      Serial.printf("❌ Sensor %d boot fail\n", i);
      continue;
    }
    sensors[i].setAddress(sensorAddresses[i]);
    if (!sensors[i].begin(sensorAddresses[i], &Wire)) {
      Serial.printf("❌ Sensor %d addr set fail\n", i);
    } else {
      sensorReady[i] = true;
      Serial.printf("✅ Sensor %d @0x%X\n", i, sensorAddresses[i]);
    }
  }

  // Determine LED blink interval
  bool allOk = true;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (!sensorReady[i]) { allOk = false; break; }
  }
  blinkInterval = allOk ? 1000UL : 5000UL;
  Serial.printf("Setup complete – LED blink %lums\n\n", blinkInterval);

  // === ESP‑NOW Setup ===
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    while (1) { delay(1000); }
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add ESP-NOW peer");
    while (1) { delay(1000); }
  }
}

void loop() {
  VL53L0X_RangingMeasurementData_t m;

  // 1) Take one sample per sensor
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorReady[i]) {
      sensors[i].rangingTest(&m, false);
      buffer[i][bufIndex] = (m.RangeStatus != 4)
        ? m.RangeMilliMeter
        : 0xFFFF;
    }
  }

  // 2) Compute median, print distances & flags, build state bytes
  uint8_t leftState  = 0;
  uint8_t rightState = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(fingerNames[i]);
    Serial.print(" - ");

    if (!sensorReady[i]) {
      Serial.print("❌ Not init");
    } else {
      uint16_t med = medianOf(buffer[i]);
      if (med == 0xFFFF) {
        Serial.print("⚠️ Out of range (0)");
      } else {
        bool inRange = (med >= minRange[i] && med <= maxRange[i]);
        Serial.printf("%dmm (%d)", med, inRange ? 1 : 0);
        if (i < 6) {
          if (inRange) leftState  |= (1 << i);
        } else {
          if (inRange) rightState |= (1 << (i - 6));
        }
      }
    }
    Serial.print(i < SENSOR_COUNT - 1 ? ", " : "\n");
  }

  // 3) Print state bytes
  Serial.printf("LeftState: 0x%02X, RightState: 0x%02X\n\n",
                leftState, rightState);

  // 4) Send state bytes via ESP‑NOW
  uint8_t packet[2] = { leftState, rightState };
  esp_err_t res = esp_now_send(peerMAC, packet, sizeof(packet));
  if (res != ESP_OK) {
    Serial.printf("esp_now_send err: %d\n", res);
  }

  // advance circular buffer index
  bufIndex = (bufIndex + 1) % WINDOW_SIZE;

  // 5) LED blink (non‑blocking)
  unsigned long now = millis();
  if (now - lastBlink >= blinkInterval) {
    lastBlink = now;
    ledState   = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  // wait until next cycle (~360 ms)
  delay(360);
}
