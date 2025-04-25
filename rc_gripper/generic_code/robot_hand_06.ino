#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <esp_now.h>

//—— CONFIGURATION ——————————————————————————————————————————————————
#define LED_PIN                2     // on‑board LED

// STEP SIZES (configurable)
#define STEP_INCREMENT_SLOW    3     // for servos 0 & 3
#define STEP_INCREMENT_FAST    20    // for servos 1,2,4,5

#define SMOOTH_STEP            1     // degrees per smoothing tick
#define SMOOTH_DELAY           20    // ms between smoothing steps
#define SERVO_MIN             150    // PCA9685 pulse for 0°
#define SERVO_MAX             600    // PCA9685 pulse for 180°
#define SERVO_FREQ             60    // Hz

const uint8_t senderMAC[6] = { 0x30,0xC6,0xF7,0x1E,0x79,0xE8 };
#define SERVO_COUNT            6

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//—— GLOBALS ———————————————————————————————————————————————————————
// actual and target positions
int servoPos[SERVO_COUNT]    = { 165, 160, 160,   15, 160, 160 };
int servoTarget[SERVO_COUNT] = { 165, 160, 160,   15, 160, 160 };

// per‑servo allowed ranges
const int servoMinDeg[SERVO_COUNT] = { 135,   0,   0,   0,   0,   0 };
const int servoMaxDeg[SERVO_COUNT] = { 180, 180, 180,  45, 180, 180 };

// per‑servo step increments
const int stepIncrement[SERVO_COUNT] = {
  STEP_INCREMENT_SLOW,    // servo 0
  STEP_INCREMENT_FAST,    // servo 1
  STEP_INCREMENT_FAST,    // servo 2
  STEP_INCREMENT_SLOW,    // servo 3
  STEP_INCREMENT_FAST,    // servo 4
  STEP_INCREMENT_FAST     // servo 5
};

// map degrees → PWM pulse, respecting per‑servo range
uint16_t degToPulse(int i, int deg) {
  deg = constrain(deg, servoMinDeg[i], servoMaxDeg[i]);
  return map(deg,
             servoMinDeg[i], servoMaxDeg[i],
             SERVO_MIN, SERVO_MAX);
}

// ESP‑NOW receive: update targets by ±stepIncrement[i]
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (memcmp(info->src_addr, senderMAC, 6) != 0) return;

  uint8_t L = data[0], R = data[1];

  bool LF_TA = L & (1<<0), LF_TB = L & (1<<1),
       LF_I  = L & (1<<2), LF_M  = L & (1<<3),
       LF_R  = L & (1<<4), LF_P  = L & (1<<5);

  bool RF_TA = R & (1<<0), RF_TB = R & (1<<1),
       RF_I  = R & (1<<2), RF_M  = R & (1<<3),
       RF_R  = R & (1<<4), RF_P  = R & (1<<5);

  // Left servos (0–2):
  if (LF_TA ^ LF_TB) servoTarget[0] += (LF_TB ?  stepIncrement[0] : -stepIncrement[0]);
  if (LF_I  ^ LF_M ) servoTarget[1] += (LF_M  ?  stepIncrement[1] : -stepIncrement[1]);
  if (LF_R  ^ LF_P ) servoTarget[2] += (LF_P  ?  stepIncrement[2] : -stepIncrement[2]);

  // Right servos (3–5):
  if (RF_TA ^ RF_TB) servoTarget[3] += (RF_TA ?  stepIncrement[3] : -stepIncrement[3]);

  // servo 4: index finger → decrease, middle → increase
  if (RF_I ^ RF_M)   servoTarget[4] += (RF_M  ?  stepIncrement[4] : -stepIncrement[4]);

  // servo 5: ring finger → decrease, pinky → increase
  if (RF_R ^ RF_P)   servoTarget[5] += (RF_P  ?  stepIncrement[5] : -stepIncrement[5]);

  // clamp targets into per‑servo ranges
  for (int i = 0; i < SERVO_COUNT; i++) {
    servoTarget[i] = constrain(servoTarget[i], servoMinDeg[i], servoMaxDeg[i]);
  }

  // debug output
  String out = "L=0x" + String(L, HEX) + " R=0x" + String(R, HEX);
  for (int i = 0; i < SERVO_COUNT; i++) {
    out += ", S" + String(i) + ":" + String(servoTarget[i]);
  }
  Serial.println(out);

  // flash LED
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // Move each servo to its initial home position
  for (int i = 0; i < SERVO_COUNT; i++) {
    pwm.setPWM(i, 0, degToPulse(i, servoPos[i]));
    delay(100);
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) { delay(1000); }
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  bool anyMoving = false;
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (servoPos[i] < servoTarget[i]) {
      servoPos[i] += SMOOTH_STEP;
      anyMoving = true;
    } else if (servoPos[i] > servoTarget[i]) {
      servoPos[i] -= SMOOTH_STEP;
      anyMoving = true;
    }
    pwm.setPWM(i, 0, degToPulse(i, servoPos[i]));
  }
  if (anyMoving) delay(SMOOTH_DELAY);
}
